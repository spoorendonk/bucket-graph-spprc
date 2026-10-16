#pragma once

// Executor shape contract. Normative text: the 2026-09-08 comment on
// flowty/pull-rcspp#23; decision flowty/flowty#58.
//
// The concept below is bgspprc's `Executor`, verbatim. It is duplicated here
// rather than depended on: C++ concepts are structural, so a type of this
// shape satisfies bgspprc's concept and ours alike, and the BCP core can hand
// the same executor to bgspprc's Solver and to this repo's pricer.
// DO NOT change a signature here alone -- it must match
// bgspprc/include/bgspprc/executor.h and every other copy.
//
// bgspprc's own `StdThreadExecutor` reaches for a leaked, process-lifetime
// global pool with detached workers. That cannot be handed a pool by
// reference, cannot be sized per solve, and cannot be shut down. So this
// header adds `ThreadPoolExecutor`: an 8-byte non-owning handle to a
// caller-owned `ThreadPool`. It satisfies the same concept, so it drops into
// bgspprc's `Solver(pv, pack, opts, Exec executor = {})` -- which takes the
// executor by value and stores it `[[no_unique_address]]` -- with no change to
// bgspprc. Worth upstreaming.
//
// `ThreadPool` keeps the fork-join structure and memory-ordering notes of
// mcfcg's `thread_pool` (MIT), with workers joined in the destructor rather
// than detached. A dispatch made while one is running (nested or concurrent)
// runs inline, and a task's exception is rethrown on the dispatching thread.

#include <algorithm>
#include <atomic>
#include <concepts>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace bgspprc {

/// Pluggable execution strategy. Structural: no base class.
///
/// - parallel_for(begin, end, f): applies f(i) to [begin, end)
/// - parallel_for_chunked(begin, end, f): f(chunk_begin, chunk_end, chunk_idx)
///   receives bulk ranges and a chunk index in [0, n_threads()). chunk_idx is
///   what lets callers index per-chunk storage lock-free -- it is a chunk
///   index, not a thread id, so it stays unique even if one worker takes two
///   chunks.
/// - parallel_invoke(f, g): runs both callables, possibly concurrently. Neither
///   may wait on the other: SequentialExecutor, the default, runs f then g, and
///   so does a ThreadPool already dispatching.
/// - n_threads(): upper bound on chunk_idx
template <typename E>
concept Executor = requires(E e, int n) {
    e.parallel_for(0, n, [](int) {});
    e.parallel_for_chunked(0, n, [](int, int, int) {});
    e.parallel_invoke([] {}, [] {});
    { e.n_threads() } -> std::convertible_to<int>;
};

/// Sequential executor. The default everywhere.
struct SequentialExecutor {
    void parallel_for(int begin, int end, auto&& f) const {
        for (int i = begin; i < end; ++i) {
            f(i);
        }
    }

    void parallel_for_chunked(int begin, int end, auto&& f) const {
        if (begin < end) {
            f(begin, end, 0);
        }
    }

    void parallel_invoke(auto&& f, auto&& g) const {
        f();
        g();
    }

    [[nodiscard]] int n_threads() const { return 1; }
};

static_assert(Executor<SequentialExecutor>);

/// Minimum range size for parallel_for to dispatch to the pool. Below this the
/// submission overhead exceeds the parallelism benefit. Matches bgspprc.
inline constexpr int kParallelForMinChunk = 512;

/// Fork-join pool, owned by the caller -- typically the BCP core, one per
/// solve. Workers are created once and joined on destruction; the calling
/// thread participates as chunk 0 so a core is not left idle.
class ThreadPool {
public:
    /// n_threads counts the calling thread, so n_threads-1 workers are spawned.
    explicit ThreadPool(int n_threads) : n_threads_(std::max(n_threads, 1)) {
        workers_.reserve(static_cast<std::size_t>(n_threads_ - 1));
        try {
            for (int ti = 1; ti < n_threads_; ++ti) {
                workers_.emplace_back([this] { worker_loop(); });
            }
        } catch (...) {
            // A thread that failed to start: join the ones that did, or their
            // std::thread destructors terminate the process.
            stop_and_join();
            throw;
        }
    }

    ~ThreadPool() { stop_and_join(); }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    [[nodiscard]] int n_threads() const noexcept { return n_threads_; }

    /// Runs task(i) for i in [0, n_tasks), distributed across the pool and the
    /// calling thread. Blocks until all complete. Task indices are handed out
    /// from a shared counter, so a slow chunk does not stall the others.
    ///
    /// One dispatch at a time per pool. A call made while the pool is already
    /// dispatching -- nested, from inside a task, or concurrent, from another
    /// thread -- runs inline on its calling thread, in index order. bgspprc's
    /// bidirectional solve does exactly this: `parallel_invoke(bw, fw)`, and
    /// each side's `compact_labels` calls `parallel_for` on the same executor.
    /// So a nested `parallel_invoke(f, g)` runs f, then g: f must not wait on g.
    ///
    /// If a task throws, no further indices are handed out, the tasks already
    /// running finish, and the first exception is rethrown here.
    template <typename F>
    void dispatch(int n_tasks, F&& task) {
        if (n_tasks <= 0) {
            return;
        }
        if (n_threads_ <= 1 || n_tasks == 1 || busy_.exchange(true, std::memory_order_acquire)) {
            for (int i = 0; i < n_tasks; ++i) {
                task(i);
            }
            return;
        }
        struct Release {
            std::atomic<bool>& busy;
            ~Release() { busy.store(false, std::memory_order_release); }
        } release{busy_};

        // Relaxed is safe: this store is sequenced-before the mutex lock below,
        // and the release/acquire chain on mutex_ synchronizes it with workers
        // waking from cv_work_. Do not move it after notify_all() without
        // restoring an explicit release ordering.
        next_task_.store(0, std::memory_order_relaxed);

        {
            std::scoped_lock lock(mutex_);
            task_fn_ = [&task](int i) { task(i); };
            task_count_ = n_tasks;
            done_count_ = 0;
            error_ = nullptr;
            ++generation_;
        }
        cv_work_.notify_all();

        run_tasks();

        std::exception_ptr error;
        {
            std::unique_lock lock(mutex_);
            cv_done_.wait(lock, [this] { return done_count_ >= n_threads_ - 1; });
            task_fn_ = nullptr;
            error = std::exchange(error_, nullptr);
        }
        if (error) {
            std::rethrow_exception(error);
        }
    }

    /// Every chunk is non-empty: n_chunks is recomputed from the chunk size,
    /// so a short range never yields a chunk that starts past `end`.
    template <typename F>
    void parallel_for_chunked(int begin, int end, F&& f) {
        const int n = end - begin;
        if (n <= 0) {
            return;
        }
        const int max_chunks = std::min(n_threads_, n);
        const int chunk_size = (n + max_chunks - 1) / max_chunks;
        const int n_chunks = (n + chunk_size - 1) / chunk_size;
        dispatch(n_chunks, [&](int c) {
            const int c_begin = begin + c * chunk_size;
            const int c_end = std::min(c_begin + chunk_size, end);
            f(c_begin, c_end, c);
        });
    }

    template <typename F>
    void parallel_for(int begin, int end, F&& f) {
        const int n = end - begin;
        if (n <= 0) {
            return;
        }
        if (n < kParallelForMinChunk || n_threads_ <= 1) {
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

    template <typename F, typename G>
    void parallel_invoke(F&& f, G&& g) {
        if (n_threads_ <= 1) {
            f();
            g();
            return;
        }
        dispatch(2, [&](int i) {
            if (i == 0) {
                f();
            } else {
                g();
            }
        });
    }

private:
    void stop_and_join() {
        {
            std::scoped_lock lock(mutex_);
            stop_ = true;
            ++generation_;
        }
        cv_work_.notify_all();
        for (auto& worker : workers_) {
            worker.join();
        }
    }

    void worker_loop() {
        std::uint64_t seen_gen = 0;
        for (;;) {
            {
                std::unique_lock lock(mutex_);
                cv_work_.wait(lock, [&] { return generation_ > seen_gen || stop_; });
                if (stop_) {
                    return;
                }
                seen_gen = generation_;
            }

            run_tasks();

            {
                std::scoped_lock lock(mutex_);
                ++done_count_;
            }
            cv_done_.notify_one();
        }
    }

    void run_tasks() {
        const int total = task_count_;
        for (;;) {
            const int idx = next_task_.fetch_add(1, std::memory_order_relaxed);
            if (idx >= total) {
                return;
            }
            try {
                task_fn_(idx);
            } catch (...) {
                {
                    std::scoped_lock lock(mutex_);
                    if (!error_) {
                        error_ = std::current_exception();
                    }
                }
                // Hand out no more indices; the ones already taken finish.
                next_task_.store(total, std::memory_order_relaxed);
                return;
            }
        }
    }

    int n_threads_;
    std::vector<std::thread> workers_;

    std::mutex mutex_;
    std::condition_variable cv_work_;
    std::condition_variable cv_done_;
    std::uint64_t generation_ = 0;
    int done_count_ = 0;
    bool stop_ = false;
    std::exception_ptr error_;  // first exception of the current dispatch

    std::function<void(int)> task_fn_;
    int task_count_ = 0;
    // Set while a dispatch owns the workers; a second dispatch runs inline.
    std::atomic<bool> busy_{false};
    // Own cache line: hammered by fetch_add on every task. Sharing with
    // task_fn_/task_count_ would bounce those reads on every increment.
    alignas(64) std::atomic<int> next_task_{0};
};

/// Non-owning handle to a caller-owned ThreadPool. This is what gets passed
/// to a pricer: the pool lives in the core, the executor is 8 bytes.
class ThreadPoolExecutor {
public:
    explicit ThreadPoolExecutor(ThreadPool& pool) noexcept : pool_(&pool) {}

    void parallel_for(int begin, int end, auto&& f) const { pool_->parallel_for(begin, end, f); }

    void parallel_for_chunked(int begin, int end, auto&& f) const {
        pool_->parallel_for_chunked(begin, end, f);
    }

    void parallel_invoke(auto&& f, auto&& g) const { pool_->parallel_invoke(f, g); }

    [[nodiscard]] int n_threads() const noexcept { return pool_->n_threads(); }

private:
    ThreadPool* pool_;  // not owned
};

static_assert(Executor<ThreadPoolExecutor>);

// NO free functions taking an Executor as their first argument may live in
// this namespace. bgspprc defines `parallel_sort(Exec&, It, It, Cmp)` and calls
// it unqualified on its own executor_. If we define the same name here, ADL on
// the executor argument finds both overloads and every such call inside
// bgspprc becomes ambiguous -- it fails to compile only when the two are
// actually used together. The contract is the concept and the executor types;
// utilities built on top of it stay in exactly one namespace.

}  // namespace bgspprc
