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

TEST_CASE("StdThreadExecutor parallel_invoke runs concurrently") {
    StdThreadExecutor exec;
    std::atomic<int> a{0};
    std::atomic<int> b{0};
    exec.parallel_invoke([&]() { a.store(1); }, [&]() { b.store(2); });
    CHECK(a.load() == 1);
    CHECK(b.load() == 2);
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
