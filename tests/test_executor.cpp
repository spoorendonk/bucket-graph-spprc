#include <atomic>
#include <bgspprc/executor_thread.h>
#include <doctest/doctest.h>
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
