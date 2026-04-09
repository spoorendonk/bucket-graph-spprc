#pragma once

#include "executor.h"

#include <thread>
#include <utility>

namespace bgspprc {

/// Thread-backed executor using std::thread.
/// parallel_invoke runs two tasks on separate threads.
/// parallel_for runs sequentially (data-parallel threading comes in later issues).
///
/// Exception safety: if the second callable throws, the background thread is
/// joined before the exception propagates. If the background thread throws,
/// std::terminate is called (std::thread does not propagate exceptions).
struct StdThreadExecutor {
    void parallel_for(int begin, int end, auto&& f) const {
        for (int i = begin; i < end; ++i)
            f(i);
    }

    void parallel_invoke(auto&& f, auto&& g) const {
        std::thread t(std::forward<decltype(f)>(f));
        try {
            g();
        } catch (...) {
            t.join();
            throw;
        }
        t.join();
    }
};

static_assert(Executor<StdThreadExecutor>, "StdThreadExecutor must satisfy the Executor concept");

}  // namespace bgspprc
