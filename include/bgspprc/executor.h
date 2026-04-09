#pragma once

#include <algorithm>
#include <concepts>
#include <iterator>

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
    // n_threads: number of threads available for parallel_for
    { e.n_threads() } -> std::convertible_to<int>;
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

    int n_threads() const { return 1; }
};

static_assert(Executor<SequentialExecutor>, "SequentialExecutor must satisfy the Executor concept");

/// Parallel sort: partition into chunks, sort each via parallel_for, then merge.
/// Falls back to std::sort for sizes below kParallelSortThreshold.
inline constexpr int kParallelSortThreshold = 4096;

template <Executor Exec, std::random_access_iterator It, typename Cmp>
void parallel_sort(Exec& exec, It first, It last, Cmp cmp) {
    auto n = std::distance(first, last);
    if (n <= kParallelSortThreshold) {
        std::sort(first, last, cmp);
        return;
    }

    // Choose chunk count: use enough chunks to exploit parallelism, but not so
    // many that merge overhead dominates. 8 chunks is a reasonable default.
    constexpr int max_chunks = 8;
    int n_chunks = std::min(max_chunks, static_cast<int>(n / 512));
    if (n_chunks < 2)
        n_chunks = 2;

    auto chunk_size = n / n_chunks;

    // Sort each chunk in parallel.
    exec.parallel_for(0, n_chunks, [&](int c) {
        auto chunk_begin = first + c * chunk_size;
        auto chunk_end = (c == n_chunks - 1) ? last : chunk_begin + chunk_size;
        std::sort(chunk_begin, chunk_end, cmp);
    });

    // Iterative pairwise merge of sorted chunks.
    // Each round merges adjacent pairs, doubling the sorted run length.
    // Independent pairs within a round are merged in parallel.
    for (int width = 1; width < n_chunks; width *= 2) {
        int n_pairs = 0;
        for (int i = 0; i + width < n_chunks; i += 2 * width)
            ++n_pairs;
        exec.parallel_for(0, n_pairs, [&](int p) {
            int i = p * 2 * width;
            auto lo = first + i * chunk_size;
            auto mid_idx = i + width;
            auto mid = (mid_idx >= n_chunks) ? last : first + mid_idx * chunk_size;
            auto hi_idx = i + 2 * width;
            auto hi = (hi_idx >= n_chunks) ? last : first + hi_idx * chunk_size;
            std::inplace_merge(lo, mid, hi, cmp);
        });
    }
}

}  // namespace bgspprc
