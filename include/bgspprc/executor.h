#pragma once

#include <concepts>
#include <thread>
#include <type_traits>
#include <utility>

namespace bgspprc {

/// Concept for pluggable execution strategies.
///
/// An Executor provides two primitives:
/// - parallel_for(begin, end, f): data-parallel, applies f to [begin, end)
/// - parallel_invoke(f, g): task-parallel, runs two callables concurrently
///
/// SequentialExecutor is the default, running everything sequentially.
/// Users can inject a thread-pool-backed executor for actual parallelism.
template <typename E>
concept Executor = requires(E e, int n) {
    // parallel_for: must accept (int, int, callable(int))
    e.parallel_for(0, n, [](int) {});
    // parallel_invoke: must accept two callables
    e.parallel_invoke([] {}, [] {});
};

/// Sequential (single-threaded) executor. Default for BucketGraph/Solver.
struct SequentialExecutor {
    void parallel_for(int begin, int end, auto&& f) const {
        for (int i = begin; i < end; ++i)
            f(i);
    }

    void parallel_invoke(auto&& f, auto&& g) const {
        f();
        g();
    }
};

static_assert(Executor<SequentialExecutor>, "SequentialExecutor must satisfy the Executor concept");

/// Thread-backed executor using std::thread.
/// parallel_invoke runs two tasks on separate threads.
/// parallel_for runs sequentially (data-parallel threading comes in later issues).
struct StdThreadExecutor {
    void parallel_for(int begin, int end, auto&& f) const {
        for (int i = begin; i < end; ++i)
            f(i);
    }

    void parallel_invoke(auto&& f, auto&& g) const {
        std::thread t(std::forward<decltype(f)>(f));
        g();
        t.join();
    }
};

static_assert(Executor<StdThreadExecutor>, "StdThreadExecutor must satisfy the Executor concept");

}  // namespace bgspprc
