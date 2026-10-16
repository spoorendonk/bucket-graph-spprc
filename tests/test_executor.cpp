#include <algorithm>
#include <atomic>
#include <bgspprc/executor_thread.h>
#include <doctest/doctest.h>
#include <functional>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

using namespace bgspprc;

TEST_CASE("SequentialExecutor satisfies Executor concept") {
    static_assert(Executor<SequentialExecutor>);
}

TEST_CASE("SequentialExecutor parallel_for iterates range") {
    SequentialExecutor exec;
    std::vector<int> results;
    exec.parallel_for(0, 5, [&](int i) { results.push_back(i); });
    CHECK(results == std::vector<int>{0, 1, 2, 3, 4});
}

TEST_CASE("SequentialExecutor parallel_for empty range") {
    SequentialExecutor exec;
    int count = 0;
    exec.parallel_for(3, 3, [&](int) { ++count; });
    CHECK(count == 0);
}

TEST_CASE("SequentialExecutor parallel_invoke runs both tasks") {
    SequentialExecutor exec;
    int a = 0;
    int b = 0;
    exec.parallel_invoke([&]() { a = 1; }, [&]() { b = 2; });
    CHECK(a == 1);
    CHECK(b == 2);
}

TEST_CASE("SequentialExecutor parallel_invoke runs in order") {
    SequentialExecutor exec;
    std::vector<int> order;
    exec.parallel_invoke([&]() { order.push_back(1); }, [&]() { order.push_back(2); });
    CHECK(order == std::vector<int>{1, 2});
}

// ── StdThreadExecutor ──

TEST_CASE("StdThreadExecutor satisfies Executor concept") {
    static_assert(Executor<StdThreadExecutor>);
}

TEST_CASE("StdThreadExecutor parallel_for iterates range") {
    StdThreadExecutor exec;
    std::vector<int> results;
    exec.parallel_for(0, 5, [&](int i) { results.push_back(i); });
    CHECK(results == std::vector<int>{0, 1, 2, 3, 4});
}

TEST_CASE("StdThreadExecutor parallel_for empty range") {
    StdThreadExecutor exec;
    int count = 0;
    exec.parallel_for(3, 3, [&](int) { ++count; });
    CHECK(count == 0);
}

TEST_CASE("StdThreadExecutor parallel_invoke runs both tasks") {
    StdThreadExecutor exec;
    std::atomic<int> a{0};
    std::atomic<int> b{0};
    exec.parallel_invoke([&]() { a.store(1); }, [&]() { b.store(2); });
    CHECK(a.load() == 1);
    CHECK(b.load() == 2);
}

TEST_CASE("StdThreadExecutor parallel_for distributes work across threads") {
    StdThreadExecutor exec;
    constexpr int n = 10000;
    std::vector<std::atomic<int>> flags(n);
    for (auto& f : flags) {
        f.store(0, std::memory_order_relaxed);
    }

    exec.parallel_for(0, n, [&](int i) { flags[i].store(1, std::memory_order_relaxed); });

    for (int i = 0; i < n; ++i) {
        CHECK(flags[i].load() == 1);
    }
}

TEST_CASE("StdThreadExecutor parallel_for atomic sum") {
    StdThreadExecutor exec;
    constexpr int n = 100000;
    std::atomic<int64_t> sum{0};
    exec.parallel_for(0, n, [&](int i) { sum.fetch_add(i, std::memory_order_relaxed); });
    CHECK(sum.load() == static_cast<int64_t>(n) * (n - 1) / 2);
}

TEST_CASE("StdThreadExecutor parallel_for small range runs sequentially") {
    StdThreadExecutor exec;
    std::vector<int> results;
    exec.parallel_for(0, 5, [&](int i) { results.push_back(i); });
    // Small range: sequential fallback, order preserved
    CHECK(results == std::vector<int>{0, 1, 2, 3, 4});
}

TEST_CASE("StdThreadExecutor n_threads returns positive") {
    StdThreadExecutor exec;
    CHECK(exec.n_threads() >= 1);
}

TEST_CASE("SequentialExecutor n_threads returns 1") {
    SequentialExecutor exec;
    CHECK(exec.n_threads() == 1);
}

TEST_CASE("SequentialExecutor parallel_for_chunked emits single chunk") {
    SequentialExecutor exec;
    int n_calls = 0;
    int c_begin_out = -1;
    int c_end_out = -1;
    int c_idx_out = -1;
    exec.parallel_for_chunked(5, 15, [&](int cb, int ce, int ci) {
        ++n_calls;
        c_begin_out = cb;
        c_end_out = ce;
        c_idx_out = ci;
    });
    CHECK(n_calls == 1);
    CHECK(c_begin_out == 5);
    CHECK(c_end_out == 15);
    CHECK(c_idx_out == 0);
}

TEST_CASE("StdThreadExecutor parallel_for_chunked covers full range with unique chunk indices") {
    StdThreadExecutor exec;
    constexpr int n = 10000;
    int n_threads = exec.n_threads();
    std::vector<std::atomic<int>> seen(n);
    for (auto& s : seen) {
        s.store(0, std::memory_order_relaxed);
    }
    std::vector<std::atomic<int>> chunk_counts(n_threads);
    for (auto& c : chunk_counts) {
        c.store(0, std::memory_order_relaxed);
    }

    exec.parallel_for_chunked(0, n, [&](int c_begin, int c_end, int chunk_idx) {
        CHECK(chunk_idx >= 0);
        CHECK(chunk_idx < n_threads);
        chunk_counts[chunk_idx].fetch_add(1, std::memory_order_relaxed);
        for (int i = c_begin; i < c_end; ++i) {
            seen[i].fetch_add(1, std::memory_order_relaxed);
        }
    });

    // Every element covered exactly once
    for (int i = 0; i < n; ++i) {
        CHECK(seen[i].load() == 1);
    }
    // Each chunk_idx called at most once (unused slots stay 0)
    int used_chunks = 0;
    for (int c = 0; c < n_threads; ++c) {
        int cnt = chunk_counts[c].load();
        CHECK(cnt <= 1);
        used_chunks += cnt;
    }
    // At least one chunk must have fired (total chunks = used_chunks)
    CHECK(used_chunks >= 1);
    // For n=10000 > n_threads, effective chunk count equals n_threads
    CHECK(used_chunks == std::min(n_threads, n));
}

TEST_CASE("StdThreadExecutor parallel_for_chunked empty range") {
    StdThreadExecutor exec;
    int n_calls = 0;
    exec.parallel_for_chunked(5, 5, [&](int, int, int) { ++n_calls; });
    CHECK(n_calls == 0);
}

TEST_CASE("StdThreadExecutor parallel_for_chunked single-element range") {
    StdThreadExecutor exec;
    int n_calls = 0;
    int c_begin_out = -1;
    int c_end_out = -1;
    int c_idx_out = -1;
    exec.parallel_for_chunked(7, 8, [&](int cb, int ce, int ci) {
        ++n_calls;
        c_begin_out = cb;
        c_end_out = ce;
        c_idx_out = ci;
    });
    CHECK(n_calls == 1);
    CHECK(c_begin_out == 7);
    CHECK(c_end_out == 8);
    CHECK(c_idx_out == 0);
}

TEST_CASE("chunk_layout tiles short ranges with non-empty chunks") {
    // Independent of this machine's core count: the old ceil sizing left a
    // chunk past the end from three threads up (n=5 over 4 gave [6,5)).
    static_assert(detail::chunk_layout(5, 4).chunk_size == 2);
    static_assert(detail::chunk_layout(5, 4).n_chunks == 3);
    for (int max_chunks = 1; max_chunks <= 8; ++max_chunks) {
        for (int n = 1; n <= 40; ++n) {
            CAPTURE(max_chunks);
            CAPTURE(n);
            auto [chunk_size, n_chunks] = detail::chunk_layout(n, max_chunks);
            CHECK(n_chunks >= 1);
            CHECK(n_chunks <= std::min(max_chunks, n));
            CHECK((n_chunks - 1) * chunk_size < n);  // last chunk non-empty
            CHECK(n_chunks * chunk_size >= n);       // range covered
        }
    }
}

TEST_CASE("StdThreadExecutor parallel_for_chunked short ranges give non-empty chunks") {
    StdThreadExecutor exec;
    const int n_threads = exec.n_threads();
    // Lengths 0 and 1, shorter than the thread count, and just above it, where
    // ceil-sized chunks used to leave trailing chunks starting past `end`.
    for (int n = 0; n <= 3 * n_threads + 1; ++n) {
        CAPTURE(n);
        std::vector<std::atomic<int>> seen(n);
        std::vector<std::atomic<int>> chunk_counts(n_threads);
        std::atomic<int> empty_chunks{0};
        exec.parallel_for_chunked(10, 10 + n, [&](int c_begin, int c_end, int chunk_idx) {
            if (c_begin >= c_end || chunk_idx < 0 || chunk_idx >= n_threads) {
                empty_chunks.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            chunk_counts[chunk_idx].fetch_add(1, std::memory_order_relaxed);
            for (int i = c_begin; i < c_end; ++i) {
                seen[i - 10].fetch_add(1, std::memory_order_relaxed);
            }
        });
        CHECK(empty_chunks.load() == 0);
        for (int i = 0; i < n; ++i) {
            CHECK(seen[i].load() == 1);
        }
        for (int c = 0; c < n_threads; ++c) {
            CHECK(chunk_counts[c].load() <= 1);
        }
    }
}

TEST_CASE("StdThreadExecutor parallel_for_chunked rethrows a chunk's exception") {
    StdThreadExecutor exec;
    const int n = 4 * exec.n_threads();
    // Throw from the last chunk, which runs on a pool worker whenever there
    // is more than one thread, and from chunk 0, which runs on the caller.
    for (int throw_at : {n - 1, 0}) {
        CAPTURE(throw_at);
        CHECK_THROWS_AS(exec.parallel_for_chunked(0, n,
                                                  [&](int c_begin, int c_end, int) {
                                                      if (c_begin <= throw_at && throw_at < c_end) {
                                                          throw std::runtime_error("chunk");
                                                      }
                                                  }),
                        std::runtime_error);
    }
}

TEST_CASE("StdThreadExecutor parallel_invoke rethrows from either task") {
    StdThreadExecutor exec;
    CHECK_THROWS_AS(exec.parallel_invoke([] { throw std::runtime_error("f"); }, [] {}),
                    std::runtime_error);
    CHECK_THROWS_AS(exec.parallel_invoke([] {}, [] { throw std::runtime_error("g"); }),
                    std::runtime_error);
}

TEST_CASE("StdThreadExecutor nested parallel_for inside parallel_invoke") {
    // The shape of bidirectional labeling: each side of parallel_invoke runs
    // a parallel_for large enough to dispatch to the pool.
    StdThreadExecutor exec;
    constexpr int n = 4 * kParallelForMinChunk;
    std::vector<std::atomic<int>> fw(n);
    std::vector<std::atomic<int>> bw(n);
    for (int round = 0; round < 50; ++round) {
        exec.parallel_invoke([&] { exec.parallel_for(0, n, [&](int i) { bw[i].fetch_add(1); }); },
                             [&] { exec.parallel_for(0, n, [&](int i) { fw[i].fetch_add(1); }); });
    }
    for (int i = 0; i < n; ++i) {
        CHECK(fw[i].load() == 50);
        CHECK(bw[i].load() == 50);
    }
}

TEST_CASE("ThreadPool with one worker completes nested batches") {
    // One worker, as on a two-core machine: the worker runs the outer task,
    // which waits on a sub-batch that only a helping wait can drain. Leaked
    // like the global pool, since its detached worker references it forever.
    auto& pool = *new detail::ThreadPool(1);
    auto nested = [&pool](std::atomic<int>& hits) {
        std::function<void()> inner = [&hits] { hits.fetch_add(1); };
        detail::ThreadPool::Batch batch;
        pool.submit(batch, &inner, 1);
        hits.fetch_add(1);
        pool.wait(batch);
    };
    std::atomic<int> outer_hits{0};
    std::atomic<int> caller_hits{0};
    std::function<void()> outer = [&] { nested(outer_hits); };
    detail::ThreadPool::Batch batch;
    pool.submit(batch, &outer, 1);
    nested(caller_hits);
    pool.wait(batch);
    CHECK(outer_hits.load() == 2);
    CHECK(caller_hits.load() == 2);
}

TEST_CASE("SequentialExecutor parallel_for_chunked empty range") {
    SequentialExecutor exec;
    int n_calls = 0;
    exec.parallel_for_chunked(5, 5, [&](int, int, int) { ++n_calls; });
    CHECK(n_calls == 0);
}

// ── parallel_sort ──

TEST_CASE("parallel_sort with SequentialExecutor - small input falls back to std::sort") {
    SequentialExecutor exec;
    std::vector<int> data = {5, 3, 8, 1, 9, 2, 7, 4, 6};
    parallel_sort(exec, data.begin(), data.end(), std::less<>{});
    CHECK(std::ranges::is_sorted(data));
    CHECK(data == std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9});
}

TEST_CASE("parallel_sort with SequentialExecutor - empty input") {
    SequentialExecutor exec;
    std::vector<int> data;
    parallel_sort(exec, data.begin(), data.end(), std::less<>{});
    CHECK(data.empty());
}

TEST_CASE("parallel_sort with SequentialExecutor - single element") {
    SequentialExecutor exec;
    std::vector<int> data = {42};
    parallel_sort(exec, data.begin(), data.end(), std::less<>{});
    CHECK(data == std::vector<int>{42});
}

TEST_CASE("parallel_sort with SequentialExecutor - large input above threshold") {
    SequentialExecutor exec;
    constexpr int n = 10000;
    std::vector<int> data(n);
    std::ranges::iota(data, 0);
    std::mt19937 rng(42);
    std::shuffle(data.begin(), data.end(), rng);

    parallel_sort(exec, data.begin(), data.end(), std::less<>{});
    CHECK(std::ranges::is_sorted(data));
    CHECK(data.front() == 0);
    CHECK(data.back() == n - 1);
}

TEST_CASE("parallel_sort with SequentialExecutor - custom comparator (descending)") {
    SequentialExecutor exec;
    constexpr int n = 5000;
    std::vector<int> data(n);
    std::ranges::iota(data, 0);
    std::mt19937 rng(123);
    std::shuffle(data.begin(), data.end(), rng);

    parallel_sort(exec, data.begin(), data.end(), std::greater<>{});
    CHECK(std::ranges::is_sorted(data, std::greater<>{}));
    CHECK(data.front() == n - 1);
    CHECK(data.back() == 0);
}

TEST_CASE("parallel_sort with StdThreadExecutor - large input above threshold") {
    StdThreadExecutor exec;
    constexpr int n = 10000;
    std::vector<int> data(n);
    std::ranges::iota(data, 0);
    std::mt19937 rng(99);
    std::shuffle(data.begin(), data.end(), rng);

    parallel_sort(exec, data.begin(), data.end(), std::less<>{});
    CHECK(std::ranges::is_sorted(data));
    CHECK(data.front() == 0);
    CHECK(data.back() == n - 1);
}

TEST_CASE("parallel_sort with StdThreadExecutor - stress 100k elements") {
    StdThreadExecutor exec;
    constexpr int n = 100000;
    std::vector<int> data(n);
    std::ranges::iota(data, 0);
    std::mt19937 rng(777);
    std::shuffle(data.begin(), data.end(), rng);

    parallel_sort(exec, data.begin(), data.end(), std::less<>{});
    CHECK(std::ranges::is_sorted(data));
    CHECK(data.front() == 0);
    CHECK(data.back() == n - 1);
}
