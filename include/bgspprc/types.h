#pragma once

#include <cstdint>
#include <limits>

// Prefetch hint for read-only data with low temporal locality (L2 cache).
// Used in dominance and concatenation loops to hide pointer-chase latency.
#if defined(__GNUC__) || defined(__clang__)
#define BGSPPRC_PREFETCH_R(addr) __builtin_prefetch(addr, 0, 1)
#else
#define BGSPPRC_PREFETCH_R(addr) ((void)0)
#endif

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

enum class Status : uint8_t { Optimal, Infeasible, TimeLimit, IterationLimit };

inline constexpr double INF = std::numeric_limits<double>::infinity();
inline constexpr double NEG_INF = -std::numeric_limits<double>::infinity();
inline constexpr double EPS = 1e-9;

}  // namespace bgspprc
