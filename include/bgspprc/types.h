#pragma once

#include <cstdint>
#include <limits>

namespace bgspprc {

enum class Direction : uint8_t { Forward, Backward };

enum class Symmetry : uint8_t { Asymmetric, Symmetric };

enum class Stage : uint8_t {
    Heuristic1,  // light dominance, fast
    Heuristic2,  // cost + resource dominance, ignore ng/R1C
    Fixing,      // arc/bucket fixing with gap
    Exact,       // full exact labeling
    Enumerate    // find all paths within gap
};

enum class Status : uint8_t {
    Optimal,
    Infeasible,
    TimeLimit,
    IterationLimit
};

inline constexpr double INF = std::numeric_limits<double>::infinity();
inline constexpr double NEG_INF = -std::numeric_limits<double>::infinity();
inline constexpr double EPS = 1e-9;

}  // namespace bgspprc
