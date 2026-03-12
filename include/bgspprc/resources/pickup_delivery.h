#pragma once

#include <algorithm>
#include <utility>

#include "../types.h"

namespace bgspprc {

/// Simultaneous pickup-and-delivery resource R^spd (Meta-Solver 2026 §6).
///
/// Forward state: (P = pickup collected, D = remaining delivery capacity).
/// Backward state: (P_bar = remaining pickup capacity, D_bar = delivery
/// served).
///
/// All changes happen at vertices (extend_to_vertex), not arcs.
/// Domination is component-wise (INF or 0), concatenation checks compatibility.
struct PickupDeliveryResource {
  struct State {
    double P;  // forward: pickup collected; backward: remaining pickup capacity
    double
        D;  // forward: remaining delivery capacity; backward: delivery served
  };

  bool symmetric() const { return false; }

  const double* pickup;    // per-vertex pickup demand, size = n_vertices
  const double* delivery;  // per-vertex delivery demand, size = n_vertices
  double Q;                // vehicle capacity
  int n_vertices;

  PickupDeliveryResource(const double* pickup_, const double* delivery_,
                         double Q_, int n_vertices_)
      : pickup(pickup_), delivery(delivery_), Q(Q_), n_vertices(n_vertices_) {}

  State init_state(Direction dir) const {
    if (dir == Direction::Forward) return {0.0, Q};
    return {Q, 0.0};
  }

  /// No-op: all changes happen at vertices.
  std::pair<State, double> extend_along_arc(Direction /*dir*/, State s,
                                            int /*arc_id*/) const {
    return {s, 0.0};
  }

  /// Forward: P' = P + p_v, D' = min(D - d_v, Q - P'),
  ///          infeasible if P + p_v > Q or d_v > D.
  /// Backward: P_bar' = min(P_bar - p_v, Q - (D_bar + d_v)),
  ///           D_bar' = D_bar + d_v,
  ///           infeasible if D_bar + d_v > Q or p_v > P_bar.
  std::pair<State, double> extend_to_vertex(Direction dir, State s,
                                            int vertex) const {
    double p = pickup[vertex];
    double d = delivery[vertex];

    if (dir == Direction::Forward) {
      double P_new = s.P + p;
      if (P_new > Q + EPS) return {{}, INF};
      if (d > s.D + EPS) return {{}, INF};
      double D_new = std::min(s.D - d, Q - P_new);
      return {{P_new, D_new}, 0.0};
    } else {
      double D_new = s.D + d;
      if (D_new > Q + EPS) return {{}, INF};
      if (p > s.P + EPS) return {{}, INF};
      double P_new = std::min(s.P - p, Q - D_new);
      return {{P_new, D_new}, 0.0};
    }
  }

  /// Forward: INF if P1 > P2 or D1 < D2, else 0.
  /// Backward: INF if D_bar1 > D_bar2 or P_bar1 < P_bar2, else 0.
  double domination_cost(Direction dir, int /*vertex*/, State s1,
                         State s2) const {
    if (dir == Direction::Forward) {
      if (s1.P > s2.P + EPS) return INF;
      if (s1.D < s2.D - EPS) return INF;
      return 0.0;
    } else {
      if (s1.D > s2.D + EPS) return INF;
      if (s1.P < s2.P - EPS) return INF;
      return 0.0;
    }
  }

  /// Concatenation: INF if P_fw > P_bar_bw or D_bar_bw > D_fw, else 0.
  double concatenation_cost(Symmetry /*sym*/, int /*vertex*/, State s_fw,
                            State s_bw) const {
    if (s_fw.P > s_bw.P + EPS) return INF;
    if (s_bw.D > s_fw.D + EPS) return INF;
    return 0.0;
  }
};

}  // namespace bgspprc
