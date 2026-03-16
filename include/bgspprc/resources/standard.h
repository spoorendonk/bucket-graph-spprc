#pragma once

#include <algorithm>
#include <span>
#include <utility>

#include "../types.h"

namespace bgspprc {

/// Standard disposable resource (time windows, capacity, etc.).
///
/// State = double (accumulated consumption q).
/// Extension: q' = max(q + d_a, lb[v']), infeasible if q' > ub[v'].
/// Domination cost: 0 (pure feasibility resource — handled by main resource
/// comparison). Concatenation cost: 0 (feasibility checked via main resource
/// bounds).
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
    return lb[sink];  // backward starts from sink lower bound
  }

  /// Extend along arc. Returns (new_state, cost_delta).
  /// cost_delta = INF means infeasible.
  std::pair<State, double> extend_along_arc(Direction dir, State q,
                                            int arc_id) const {
    // For a standard resource used as a non-main resource,
    // we do the same extension as main resources.
    double d = consumption[arc_id];
    if (dir == Direction::Forward) {
      // Not available from arc arrays here, so we assume arc_to is
      // handled by the caller. For non-main resources, we track state
      // but vertex bounds are checked by the caller during extend.
      double q_new = q + d;
      // We can't check vertex bounds without knowing the target vertex.
      // This is returned to the caller who checks bounds.
      return {q_new, 0.0};
    } else {
      double q_new = q + d;
      return {q_new, 0.0};
    }
  }

  /// Extend to vertex: no-op for standard resource.
  std::pair<State, double> extend_to_vertex(Direction /*dir*/, State q,
                                            int /*vertex*/) const {
    return {q, 0.0};
  }

  double domination_cost(Direction /*dir*/, int /*vertex*/, State /*s1*/,
                         State /*s2*/) const {
    return 0.0;  // pure feasibility resource
  }

  /// Lower bound on domination_cost: always 0.
  double min_domination_cost() const { return 0.0; }

  double concatenation_cost(Symmetry /*sym*/, int /*vertex*/, State /*s_fw*/,
                            State /*s_bw*/) const {
    return 0.0;
  }
};

}  // namespace bgspprc
