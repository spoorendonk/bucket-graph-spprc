#pragma once

#include "../types.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace bgspprc {

/// Ng-path resource for elementarity relaxation using LOCAL bit positions.
///
/// Each vertex v assigns local bit positions 0..k-1 to its k ng-neighbors.
/// State is a uint64_t with up to k bits used.
///
/// Source marking: when extending arc i→j forward, we mark the SOURCE i
/// (set i's bit in j's ordering) rather than the destination j.
/// This allows same-vertex concatenation to work correctly and eliminates
/// the need for separate across-arc concatenation logic.
///
/// When extending arc i→j forward, we TRANSFORM the bit vector: remap common
/// neighbors from i's local positions to j's local positions, then set i's
/// bit in j's ordering (source marking).
/// When extending arc i→j backward (j→i), we remap from j's to i's positions,
/// then set j's bit in i's ordering.
/// Both transforms are precomputed per arc for O(k) extension.
struct NgPathResource {
    using State = uint64_t;

    /// Per-arc precomputed transform data for both directions.
    struct ArcInfo {
        // Forward: extending from→to
        int8_t fw_check_bit;   // bit pos of to in from's ng-set (-1 if not neighbor)
        uint64_t fw_mark_mask; // bit for source 'from' in to's ordering (0 if from ∉ N(to))
        std::array<std::pair<int8_t, int8_t>, 64> fw_shift_pairs;
        int fw_n_pairs;

        // Backward: extending to→from (traversing arc in reverse)
        int8_t bw_check_bit;   // bit pos of from in to's ng-set (-1 if not neighbor)
        uint64_t bw_mark_mask; // bit for source 'to' in from's ordering (0 if to ∉ N(from))
        std::array<std::pair<int8_t, int8_t>, 64> bw_shift_pairs;
        int bw_n_pairs;
    };

    int n_vertices_;
    int n_arcs_;

    // bit_map[v * n_vertices + w] = local bit position of w in v's ng-set, or -1
    std::vector<int8_t> bit_map;  // flat [n_vertices × n_vertices]

    // Per-arc precomputed transforms
    std::vector<ArcInfo> arc_info;  // [n_arcs]

    // neighbors_[v] = list of vertex IDs in v's ng-neighborhood
    std::vector<std::vector<int>> neighbors_;

    NgPathResource() : n_vertices_(0), n_arcs_(0) {}

    NgPathResource(int n_vertices, int n_arcs,
                   const int* arc_from, const int* arc_to,
                   const std::vector<std::vector<int>>& neighbors)
        : n_vertices_(n_vertices)
        , n_arcs_(n_arcs)
        , neighbors_(neighbors)
    {
        // Build bit_map: for each vertex v, assign local positions to its neighbors
        bit_map.assign(static_cast<size_t>(n_vertices) * n_vertices, -1);

        // Ensure neighbors_ is sized correctly (may be empty for no-ng instances)
        if (neighbors_.size() < static_cast<size_t>(n_vertices))
            neighbors_.resize(n_vertices);

        for (int v = 0; v < n_vertices; ++v) {
            int8_t pos = 0;
            for (int w : neighbors_[v]) {
                if (w >= 0 && w < n_vertices && pos < 64) {
                    bit_map[static_cast<size_t>(v) * n_vertices + w] = pos++;
                }
            }
        }

        // Precompute arc_info
        arc_info.resize(n_arcs);
        for (int a = 0; a < n_arcs; ++a) {
            int from = arc_from[a];
            int to = arc_to[a];
            auto& ai = arc_info[a];

            // === Forward: extending from→to ===
            // check: is 'to' in 'from's ng-set?
            ai.fw_check_bit = bit_map[static_cast<size_t>(from) * n_vertices + to];
            // mark: source 'from' in 'to's ordering (if from ∈ N(to))
            int8_t from_in_to = bit_map[static_cast<size_t>(to) * n_vertices + from];
            ai.fw_mark_mask = (from_in_to >= 0) ? (1ULL << from_in_to) : 0ULL;

            // shift: remap from 'from's ordering to 'to's ordering
            ai.fw_n_pairs = 0;
            for (int w : neighbors_[from]) {
                if (w < 0 || w >= n_vertices) continue;
                int8_t from_pos = bit_map[static_cast<size_t>(from) * n_vertices + w];
                int8_t to_pos = bit_map[static_cast<size_t>(to) * n_vertices + w];
                if (from_pos >= 0 && to_pos >= 0 && ai.fw_n_pairs < 64) {
                    ai.fw_shift_pairs[ai.fw_n_pairs++] = {from_pos, to_pos};
                }
            }

            // === Backward: extending to→from (reverse traversal) ===
            // check: is 'from' in 'to's ng-set?
            ai.bw_check_bit = bit_map[static_cast<size_t>(to) * n_vertices + from];
            // mark: source 'to' in 'from's ordering (if to ∈ N(from))
            int8_t to_in_from = bit_map[static_cast<size_t>(from) * n_vertices + to];
            ai.bw_mark_mask = (to_in_from >= 0) ? (1ULL << to_in_from) : 0ULL;

            // shift: remap from 'to's ordering to 'from's ordering
            ai.bw_n_pairs = 0;
            for (int w : neighbors_[to]) {
                if (w < 0 || w >= n_vertices) continue;
                int8_t to_pos = bit_map[static_cast<size_t>(to) * n_vertices + w];
                int8_t from_pos = bit_map[static_cast<size_t>(from) * n_vertices + w];
                if (to_pos >= 0 && from_pos >= 0 && ai.bw_n_pairs < 64) {
                    ai.bw_shift_pairs[ai.bw_n_pairs++] = {to_pos, from_pos};
                }
            }
        }
    }

    /// Initialize state: no vertices visited yet (source marking).
    State init_state(Direction /*dir*/) const {
        return 0ULL;
    }

    /// Extend: transform bit vector from source ordering to target ordering.
    /// Source marking: marks the source vertex's bit in the target's ordering.
    std::pair<State, double> extend(Direction dir, State s, int arc_id) const {
        auto& ai = arc_info[arc_id];

        if (dir == Direction::Forward) {
            // Forward: arc from→to, label at from, extending to to
            // Check: is 'to' forbidden? (to ∈ N(from) and to's bit set in from's state)
            if (ai.fw_check_bit >= 0 && (s & (1ULL << ai.fw_check_bit)))
                return {0ULL, INF};

            uint64_t new_s = 0;
            for (int i = 0; i < ai.fw_n_pairs; ++i) {
                auto [fp, tp] = ai.fw_shift_pairs[i];
                if (s & (1ULL << fp))
                    new_s |= (1ULL << tp);
            }
            new_s |= ai.fw_mark_mask;  // mark source 'from' in to's ordering
            return {new_s, 0.0};
        } else {
            // Backward: arc from→to traversed as to→from, label at to, extending to from
            if (ai.bw_check_bit >= 0 && (s & (1ULL << ai.bw_check_bit)))
                return {0ULL, INF};

            uint64_t new_s = 0;
            for (int i = 0; i < ai.bw_n_pairs; ++i) {
                auto [fp, tp] = ai.bw_shift_pairs[i];
                if (s & (1ULL << fp))
                    new_s |= (1ULL << tp);
            }
            new_s |= ai.bw_mark_mask;  // mark source 'to' in from's ordering
            return {new_s, 0.0};
        }
    }

    /// Domination: L1 dominates L2 only if L1's forbidden set is subset of L2's.
    /// Same vertex → same local mapping, so direct bitwise comparison works.
    double domination_cost(Direction /*dir*/, int /*vertex*/,
                           State s1, State s2) const {
        if (s1 & ~s2) return INF;
        return 0.0;
    }

    /// Same-vertex concatenation: forward and backward labels at same vertex have same
    /// local bit mapping. If any bit is set in both, a vertex was visited
    /// in both directions → cycle in the concatenated path.
    double concatenation_cost(Symmetry /*sym*/, int /*vertex*/,
                              State s_fw, State s_bw) const {
        if (s_fw & s_bw) return INF;
        return 0.0;
    }
};

}  // namespace bgspprc
