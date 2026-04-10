#include <algorithm>
#include <atomic>
#include <bgspprc/executor_thread.h>
#include <doctest/doctest.h>
#include <numeric>
#include <random>
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
    for (auto& f : flags)
        f.store(0, std::memory_order_relaxed);

    exec.parallel_for(0, n, [&](int i) { flags[i].store(1, std::memory_order_relaxed); });

    for (int i = 0; i < n; ++i)
        CHECK(flags[i].load() == 1);
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
    for (auto& s : seen)
        s.store(0, std::memory_order_relaxed);
    std::vector<std::atomic<int>> chunk_counts(n_threads);
    for (auto& c : chunk_counts)
        c.store(0, std::memory_order_relaxed);

    exec.parallel_for_chunked(0, n, [&](int c_begin, int c_end, int chunk_idx) {
        CHECK(chunk_idx >= 0);
        CHECK(chunk_idx < n_threads);
        chunk_counts[chunk_idx].fetch_add(1, std::memory_order_relaxed);
        for (int i = c_begin; i < c_end; ++i)
            seen[i].fetch_add(1, std::memory_order_relaxed);
    });

    // Every element covered exactly once
    for (int i = 0; i < n; ++i)
        CHECK(seen[i].load() == 1);
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
    CHECK(std::is_sorted(data.begin(), data.end()));
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
    std::iota(data.begin(), data.end(), 0);
    std::mt19937 rng(42);
    std::shuffle(data.begin(), data.end(), rng);

    parallel_sort(exec, data.begin(), data.end(), std::less<>{});
    CHECK(std::is_sorted(data.begin(), data.end()));
    CHECK(data.front() == 0);
    CHECK(data.back() == n - 1);
}

TEST_CASE("parallel_sort with SequentialExecutor - custom comparator (descending)") {
    SequentialExecutor exec;
    constexpr int n = 5000;
    std::vector<int> data(n);
    std::iota(data.begin(), data.end(), 0);
    std::mt19937 rng(123);
    std::shuffle(data.begin(), data.end(), rng);

    parallel_sort(exec, data.begin(), data.end(), std::greater<>{});
    CHECK(std::is_sorted(data.begin(), data.end(), std::greater<>{}));
    CHECK(data.front() == n - 1);
    CHECK(data.back() == 0);
}

TEST_CASE("parallel_sort with StdThreadExecutor - large input above threshold") {
    StdThreadExecutor exec;
    constexpr int n = 10000;
    std::vector<int> data(n);
    std::iota(data.begin(), data.end(), 0);
    std::mt19937 rng(99);
    std::shuffle(data.begin(), data.end(), rng);

    parallel_sort(exec, data.begin(), data.end(), std::less<>{});
    CHECK(std::is_sorted(data.begin(), data.end()));
    CHECK(data.front() == 0);
    CHECK(data.back() == n - 1);
}

TEST_CASE("parallel_sort with StdThreadExecutor - stress 100k elements") {
    StdThreadExecutor exec;
    constexpr int n = 100000;
    std::vector<int> data(n);
    std::iota(data.begin(), data.end(), 0);
    std::mt19937 rng(777);
    std::shuffle(data.begin(), data.end(), rng);

    parallel_sort(exec, data.begin(), data.end(), std::less<>{});
    CHECK(std::is_sorted(data.begin(), data.end()));
    CHECK(data.front() == 0);
    CHECK(data.back() == n - 1);
}
