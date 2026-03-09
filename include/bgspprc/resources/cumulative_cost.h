#pragma once

#include "../types.h"

#include <algorithm>
#include <utility>

namespace bgspprc {

/// Cumulative cost resource R^cc for CCVRP (Meta-Solver 2026 §5).
///
/// Minimizes weighted average arrival time. Forward state tracks
/// (cumulative_cost S, arrival_time T); backward tracks
/// (cumulative_cost S, cumulative_weight W).
///
/// Key property: extend_along_arc returns non-zero cost deltas,
/// exercising ResourcePack cost accumulation.
struct CumulativeCostResource {
    struct State {
        double S;       // cumulative cost
        double T_or_W;  // forward: arrival time T; backward: cumulative weight W
    };

    bool symmetric() const { return false; }

    const double* travel_time;  // per-arc travel time l_a, size = n_arcs
    const double* weight;       // per-arc weight w_a, size = n_arcs
    double W_max;               // maximum total weight (for domination bound)
    double T_max;               // maximum total time (for domination bound)
    int n_arcs;

    CumulativeCostResource(const double* travel_time_, const double* weight_,
                           double W_max_, double T_max_, int n_arcs_)
        : travel_time(travel_time_)
        , weight(weight_)
        , W_max(W_max_)
        , T_max(T_max_)
        , n_arcs(n_arcs_) {}

    State init_state(Direction /*dir*/) const {
        return {0.0, 0.0};
    }

    /// Forward: S' = S + w_a * (T + l_a),  T' = T + l_a,  cost_delta = w_a * (T + l_a)
    /// Backward: S' = S + (W + w_a) * l_a,  W' = W + w_a,  cost_delta = (W + w_a) * l_a
    std::pair<State, double> extend_along_arc(Direction dir, State s, int arc_id) const {
        double l = travel_time[arc_id];
        double w = weight[arc_id];

        if (dir == Direction::Forward) {
            double delta = w * (s.T_or_W + l);
            return {{s.S + delta, s.T_or_W + l}, delta};
        } else {
            double delta = (s.T_or_W + w) * l;
            return {{s.S + delta, s.T_or_W + w}, delta};
        }
    }

    /// No vertex-level changes.
    std::pair<State, double> extend_to_vertex(Direction /*dir*/, State s,
                                               int /*vertex*/) const {
        return {s, 0.0};
    }

    /// Forward: (S1 - S2) + max(0, T1 - T2) * W_max
    /// Backward: (S1 - S2) + max(0, W1 - W2) * T_max
    double domination_cost(Direction dir, int /*vertex*/,
                           State s1, State s2) const {
        double cost_gap = s1.S - s2.S;
        if (dir == Direction::Forward) {
            return cost_gap + std::max(0.0, s1.T_or_W - s2.T_or_W) * W_max;
        } else {
            return cost_gap + std::max(0.0, s1.T_or_W - s2.T_or_W) * T_max;
        }
    }

    /// Cross-product: T_fw * W_bw.
    double concatenation_cost(Symmetry /*sym*/, int /*vertex*/,
                              State s_fw, State s_bw) const {
        return s_fw.T_or_W * s_bw.T_or_W;
    }

    double arc_concatenation_cost(Symmetry /*sym*/, int /*arc_id*/,
                                  State /*s_fw*/, State /*s_bw*/) const {
        return 0.0;
    }
};

}  // namespace bgspprc
