#pragma once

#include "executor.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace bgspprc {

namespace detail {

/// Simple thread pool with split submit/wait semantics.
/// Workers sleep on a condition variable until tasks are submitted.
/// The pool is process-lifetime — worker threads are detached.
class ThreadPool {
public:
    explicit ThreadPool(int n_workers) : n_workers_(n_workers) {
        workers_.reserve(n_workers);
        for (int i = 0; i < n_workers; ++i) {
            workers_.emplace_back([this] { worker_loop(); });
            workers_.back().detach();
        }
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    int n_workers() const { return n_workers_; }

    /// A batch of submitted tasks. Caller can wait for completion.
    struct Batch {
        std::atomic<int> remaining{0};
        std::exception_ptr exception;
        std::mutex exception_mutex;
    };

    /// Submit tasks to the pool. Returns immediately.
    /// The Batch must outlive all submitted tasks (caller owns it).
    void submit(Batch& batch, std::function<void()>* tasks, int n_tasks) {
        if (n_tasks <= 0) {
            return;
        }
        batch.remaining.fetch_add(n_tasks, std::memory_order_relaxed);
        {
            std::lock_guard lock(mutex_);
            for (int i = 0; i < n_tasks; ++i) {
                queue_.push_back({&tasks[i], &batch});
            }
        }
        if (n_tasks >= n_workers_) {
            cv_work_.notify_all();
        } else {
            for (int i = 0; i < n_tasks; ++i) {
                cv_work_.notify_one();
            }
        }
    }

    /// Block until all tasks in the batch have completed.
    /// Rethrows the first exception from any task.
    static void wait(Batch& batch) {
        int val;
        while ((val = batch.remaining.load(std::memory_order_acquire)) != 0) {
            batch.remaining.wait(val, std::memory_order_acquire);
        }
        if (batch.exception) {
            std::rethrow_exception(batch.exception);
        }
    }

private:
    struct Task {
        std::function<void()>* fn;
        Batch* batch;
    };

    void worker_loop() {
        for (;;) {
            Task task;
            {
                std::unique_lock lock(mutex_);
                cv_work_.wait(lock, [this] { return !queue_.empty(); });
                task = queue_.front();
                queue_.pop_front();
            }
            try {
                (*task.fn)();
            } catch (...) {
                std::lock_guard lock(task.batch->exception_mutex);
                if (!task.batch->exception) {
                    task.batch->exception = std::current_exception();
                }
            }
            if (task.batch->remaining.fetch_sub(1, std::memory_order_release) == 1) {
                task.batch->remaining.notify_all();
            }
        }
    }

    int n_workers_;
    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable cv_work_;
    std::deque<Task> queue_;
};

inline ThreadPool& global_thread_pool() {
    // Intentionally leaked: the pool must outlive all threads. A static local
    // would be destroyed at exit while detached workers still reference it.
    static auto* pool = new ThreadPool([] {
        int hw = static_cast<int>(std::thread::hardware_concurrency());
        return std::max(1, hw) - 1;  // -1: caller thread participates
    }());
    return *pool;
}

}  // namespace detail

/// Minimum range size for parallel_for to dispatch to the thread pool.
/// Below this, the overhead of task submission exceeds the parallelism benefit.
inline constexpr int kParallelForMinChunk = 512;

/// Thread-backed executor using a static thread pool.
/// parallel_for distributes work across pool threads + the calling thread.
/// parallel_invoke runs two tasks concurrently (one on pool, one on caller).
///
/// Exception safety: if a pool task throws, the exception is captured and
/// rethrown on the calling thread after all tasks complete.
struct StdThreadExecutor {
    void parallel_for(int begin, int end, auto&& f) const {
        int n = end - begin;
        if (n <= 0) {
            return;
        }

        if (n < kParallelForMinChunk) {
            for (int i = begin; i < end; ++i) {
                f(i);
            }
            return;
        }

        parallel_for_chunked(begin, end, [&f](int c_begin, int c_end, int) {
            for (int i = c_begin; i < c_end; ++i) {
                f(i);
            }
        });
    }

    /// Divides [begin, end) into chunks (one per thread) and calls
    /// f(chunk_begin, chunk_end, chunk_idx) on each. chunk_idx is unique per
    /// chunk in [0, n_threads()), enabling lock-free per-thread accumulation.
    /// Unlike parallel_for, there is no element-count threshold: callers pass
    /// chunk-level work, so per-chunk cost is assumed high.
    void parallel_for_chunked(int begin, int end, auto&& f) const {
        int n = end - begin;
        if (n <= 0) {
            return;
        }

        auto& pool = detail::global_thread_pool();
        int total_threads = pool.n_workers() + 1;  // +1 for calling thread

        if (total_threads <= 1 || n == 1) {
            f(begin, end, 0);
            return;
        }

        // Stack-allocated task array. Cap chunk count at kMaxPoolTasks + 1 so
        // that every computed chunk is actually dispatched (otherwise the tail
        // chunks would be silently dropped on very high core counts).
        static constexpr int kMaxPoolTasks = 128;
        int n_chunks = std::min({total_threads, n, kMaxPoolTasks + 1});
        int chunk_size = (n + n_chunks - 1) / n_chunks;

        // Build tasks for pool threads (chunks 1..n_chunks-1).
        // Pack (c_begin, c_end, chunk_idx) into a side struct so the lambda
        // only captures two pointers (&f + &ctx) = 16 bytes, fitting in
        // libstdc++'s std::function SBO and avoiding a per-task heap alloc.
        struct ChunkCtx {
            int c_begin;
            int c_end;
            int chunk_idx;
        };
        int pool_tasks = n_chunks - 1;
        std::function<void()> tasks[kMaxPoolTasks];
        ChunkCtx ctxs[kMaxPoolTasks];
        for (int c = 0; c < pool_tasks; ++c) {
            ctxs[c] = {begin + (c + 1) * chunk_size, std::min(begin + (c + 2) * chunk_size, end),
                       c + 1};
            auto* ctx = &ctxs[c];
            tasks[c] = [&f, ctx]() { f(ctx->c_begin, ctx->c_end, ctx->chunk_idx); };
        }

        // Submit to pool (non-blocking).
        detail::ThreadPool::Batch batch;
        pool.submit(batch, tasks, pool_tasks);

        // Run chunk 0 on the calling thread while pool works.
        // Exception safety: if f throws, wait for pool before unwinding so
        // workers don't access the stack-allocated tasks/batch after destruction.
        int c0_end = std::min(begin + chunk_size, end);
        try {
            f(begin, c0_end, 0);
        } catch (...) {
            detail::ThreadPool::wait(batch);
            throw;
        }
        detail::ThreadPool::wait(batch);
    }

    void parallel_invoke(auto&& f, auto&& g) const {
        auto& pool = detail::global_thread_pool();
        if (pool.n_workers() == 0) {
            f();
            g();
            return;
        }

        // Run f on pool, g on caller.
        std::function<void()> task = [&f]() { f(); };
        detail::ThreadPool::Batch batch;
        pool.submit(batch, &task, 1);
        try {
            g();
        } catch (...) {
            detail::ThreadPool::wait(batch);
            throw;
        }
        detail::ThreadPool::wait(batch);
    }

    int n_threads() const { return detail::global_thread_pool().n_workers() + 1; }
};

static_assert(Executor<StdThreadExecutor>, "StdThreadExecutor must satisfy the Executor concept");

}  // namespace bgspprc
