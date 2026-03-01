#pragma once

#include "../types.h"

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

namespace bgspprc {

/// Ng-path resource for elementarity relaxation.
///
/// State = bitset of "forbidden" vertices (vertices in the ng-neighborhood
/// that have been visited and cannot be immediately revisited).
///
/// For small ng-sets (|NG(v)| ~ 8-12), a uint64_t bitset suffices for
/// up to 64 vertices. For larger problems, use a vector<uint64_t>.
///
/// Here we use a fixed-size bitset for up to 64 vertices.
struct NgPathResource {
    using State = uint64_t;  // bitset of visited ng-neighbors

    int n_vertices;
    const int* arc_to;  // per-arc target vertex, size = n_arcs

    // ng_sets[v] = bitmask of vertices in NG(v)
    // If vertex w ∈ NG(v), then bit w is set in ng_sets[v].
    const uint64_t* ng_sets;

    NgPathResource(int n_vertices_, const int* arc_to_,
                   const uint64_t* ng_sets_)
        : n_vertices(n_vertices_)
        , arc_to(arc_to_)
        , ng_sets(ng_sets_) {}

    State init_state(Direction /*dir*/) const {
        return 0ULL;  // no vertices visited
    }

    /// Extend: when arriving at vertex v via arc a:
    ///   - If v is in the current forbidden set, extension is infeasible
    ///   - Otherwise, new forbidden set = (old ∩ NG(v)) | {v}
    std::pair<State, double> extend(Direction /*dir*/, State s, int arc_id) const {
        int v = arc_to[arc_id];

        // Check if v is forbidden
        uint64_t v_bit = 1ULL << v;
        if (s & v_bit) {
            return {0ULL, INF};  // infeasible: revisiting forbidden vertex
        }

        // New state: keep only vertices in NG(v), then add v
        State new_s = (s & ng_sets[v]) | v_bit;
        return {new_s, 0.0};  // no cost contribution
    }

    /// Domination: L1 dominates L2 only if L1's forbidden set is a subset of L2's.
    /// If L1 has MORE forbidden vertices, it's more constrained → cannot dominate.
    /// Cost penalty for extra forbidden vertices in L1: INF (cannot dominate).
    double domination_cost(Direction /*dir*/, int /*vertex*/,
                           State s1, State s2) const {
        // s1 must be subset of s2 for domination (L1 less constrained)
        if (s1 & ~s2) return INF;  // L1 has forbidden vertices L2 doesn't → no domination
        return 0.0;
    }

    double concatenation_cost(Symmetry /*sym*/, int /*vertex*/,
                              State /*s_fw*/, State /*s_bw*/) const {
        return 0.0;  // ng-path concatenation feasibility is complex; simplified here
    }
};

}  // namespace bgspprc
