#pragma once

#include <algorithm>
#include <utility>

#include "../types.h"

namespace bgspprc {

/// Standard disposable resource (time windows, capacity, etc.).
///
/// State = double (accumulated consumption q).
/// Extension: q' = max(q + d_a, lb[v']), infeasible if q' > ub[v'].
/// Domination cost: INF when L1's state exceeds L2's (forward: s1 > s2,
/// backward: s1 < s2), otherwise 0 (Meta-Solver §4.2.1).
/// Concatenation cost: INF when fw/bw states are incompatible at join vertex.
///
/// This resource is typically used as one of the 1-2 "main resources" that
/// define the bucket grid. Main resource extension is handled directly in the
/// bucket graph labeling loop (not via the resource pack) for performance.
///
/// When used as a non-main resource in the pack, it provides additional
/// resource constraints (e.g., a third resource beyond the 2 main ones).
struct StandardResource {
  using State = double;

  /// Standard resource symmetry depends on arc consumptions and bounds
  /// (Meta-Solver 2026 §4.2.1). Caller should verify conditions externally;
  /// conservatively returns false.
  bool symmetric() const { return false; }

  const double* consumption;  // per-arc consumption d_a, size = n_arcs
  const double* lb;           // per-vertex lower bound, size = n_vertices
  const double* ub;           // per-vertex upper bound, size = n_vertices
  int source;
  int sink;
  int n_arcs;

  StandardResource(const double* consumption_, const double* lb_,
                   const double* ub_, int source_, int sink_, int n_arcs_)
      : consumption(consumption_),
        lb(lb_),
        ub(ub_),
        source(source_),
        sink(sink_),
        n_arcs(n_arcs_) {}

  State init_state(Direction dir) const {
    if (dir == Direction::Forward) return lb[source];
    return ub[sink];  // backward starts from sink upper bound (Meta-Solver §4.2.1)
  }

  /// Extend along arc. Returns (new_state, cost_delta).
  /// cost_delta = INF means infeasible.
  std::pair<State, double> extend_along_arc(Direction dir, State q,
                                            int arc_id) const {
    // For a standard resource used as a non-main resource,
    // we do the same extension as main resources.
    double d = consumption[arc_id];
    if (dir == Direction::Forward) {
      return {q + d, 0.0};
    } else {
      return {q - d, 0.0};
    }
  }

  /// Extend to vertex: clamp to vertex bounds and check feasibility.
  /// Forward: q = max(q, lb[v]), infeasible if q > ub[v].
  /// Backward: q = min(q, ub[v]), infeasible if q < lb[v].
  std::pair<State, double> extend_to_vertex(Direction dir, State q,
                                            int vertex) const {
    if (dir == Direction::Forward) {
      double q_new = std::max(q, lb[vertex]);
      if (q_new > ub[vertex]) return {q_new, INF};
      return {q_new, 0.0};
    } else {
      double q_new = std::min(q, ub[vertex]);
      if (q_new < lb[vertex]) return {q_new, INF};
      return {q_new, 0.0};
    }
  }

  double domination_cost(Direction dir, int /*vertex*/, State s1,
                         State s2) const {
    if (dir == Direction::Forward) {
      if (s1 > s2 + EPS) return INF;
    } else {
      if (s1 < s2 - EPS) return INF;
    }
    return 0.0;
  }

  /// Lower bound on domination_cost: always 0.
  double min_domination_cost() const { return 0.0; }

  double concatenation_cost(Symmetry sym, int vertex, State s_fw,
                            State s_bw) const {
    if (sym == Symmetry::Symmetric) {
      if (s_fw + s_bw > ub[vertex] + EPS) return INF;
    } else {
      if (s_fw > s_bw + EPS) return INF;
    }
    return 0.0;
  }
};

}  // namespace bgspprc
