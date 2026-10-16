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

    [[nodiscard]] int n_workers() const { return n_workers_; }

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
            std::scoped_lock lock(mutex_);
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

    /// Block until all tasks in the batch have completed, running queued
    /// tasks (of any batch) while waiting. Helping is what makes nested
    /// dispatch safe: bidirectional labeling runs parallel_for inside a
    /// parallel_invoke task, and a worker that blocked on its sub-batch could
    /// otherwise leave that sub-batch queued with no free worker to take it.
    /// The waiter sleeps on cv_work_, so it wakes both for new work and for
    /// its batch completing. Rethrows the first exception from any task.
    void wait(Batch& batch) {
        std::unique_lock lock(mutex_);
        for (;;) {
            // Under mutex_: run() signals completion only after taking it, so
            // a zero seen here cannot be missed by the cv wait below.
            if (batch.remaining.load(std::memory_order_acquire) == 0) {
                break;
            }
            if (queue_.empty()) {
                cv_work_.wait(lock);
                continue;
            }
            Task task = queue_.front();
            queue_.pop_front();
            lock.unlock();
            run(task);
            lock.lock();
        }
        lock.unlock();
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
            run(task);
        }
    }

    void run(Task task) {
        try {
            (*task.fn)();
        } catch (...) {
            std::scoped_lock lock(task.batch->exception_mutex);
            if (!task.batch->exception) {
                task.batch->exception = std::current_exception();
            }
        }
        // The batch may be gone once remaining hits 0 (its waiter returns), so
        // completion is signalled through the pool's own mutex and cv only.
        if (task.batch->remaining.fetch_sub(1, std::memory_order_release) == 1) {
            {
                std::scoped_lock lock(mutex_);
            }
            cv_work_.notify_all();
        }
    }

    int n_workers_;
    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable cv_work_;
    std::deque<Task> queue_;
};

struct ChunkLayout {
    int chunk_size;
    int n_chunks;
};

/// Splits n > 0 items into at most max_chunks > 0 chunks of chunk_size, the
/// last one possibly shorter. n_chunks is recomputed from the chunk size so
/// every chunk is non-empty: n=5 over 4 is 3 chunks of 2, not a 4th at [6,5).
constexpr ChunkLayout chunk_layout(int n, int max_chunks) {
    int chunks = std::min(max_chunks, n);
    int chunk_size = (n + chunks - 1) / chunks;
    return {.chunk_size = chunk_size, .n_chunks = (n + chunk_size - 1) / chunk_size};
}

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
        auto [chunk_size, n_chunks] =
            detail::chunk_layout(n, std::min(total_threads, kMaxPoolTasks + 1));

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
            pool.wait(batch);
            throw;
        }
        pool.wait(batch);
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
            pool.wait(batch);
            throw;
        }
        pool.wait(batch);
    }

    [[nodiscard]] int n_threads() const { return detail::global_thread_pool().n_workers() + 1; }
};

static_assert(Executor<StdThreadExecutor>, "StdThreadExecutor must satisfy the Executor concept");

}  // namespace bgspprc
