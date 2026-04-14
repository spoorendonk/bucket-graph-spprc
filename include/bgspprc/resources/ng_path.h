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
/// Each vertex v assigns local bit positions 0..k-1 to its k ng-neighbors,
/// plus bit k = **self bit** representing v itself. State is a uint32_t
/// (max 31 neighbors, covers ng=8/16/24).
///
/// Destination marking (Meta-Solver 2026 §4.2.2):
///   init_state()       → 0 (BucketGraph calls extend_to_vertex to mark
///   source/sink) extend_along_arc(fw, s, a) → extendAlongArc: check forbidden,
///   remap bits, do NOT mark extend_to_vertex(fw, s, v) → extendToVertex: set
///   v's self bit
///
/// When extending arc i→j forward:
///   1. Check: is j forbidden? (j ∈ N(i) and j's bit set in i's state)
///   2. Remap: transform bits from i's ordering to j's ordering via shift_pairs
///      (includes self-bit-to-neighbor mapping as an extra shift pair)
///   3. Do NOT mark j — that's done by extend_to_vertex.
///
/// Concatenation correctness: across-arc (i,j) extends fw@i through arc (no
/// self bit set for j), then checks concatenation_cost with bw@j (which has
/// j's self bit set). Since fw doesn't have j's self bit, j isn't
/// double-counted.
struct NgPathResource {
    using State = uint32_t;

    /// Ng-path resource is always symmetric (Meta-Solver 2026 §4.2.2).
    bool symmetric() const { return true; }

    /// Per-arc precomputed transform data for both directions.
    struct ArcInfo {
        // Forward: extending from→to
        int8_t fw_check_bit;  // bit pos of to in from's ng-set (-1 if not neighbor)
        std::array<std::pair<int8_t, int8_t>, 32> fw_shift_pairs;
        int fw_n_pairs;

        // Backward: extending to→from (traversing arc in reverse)
        int8_t bw_check_bit;  // bit pos of from in to's ng-set (-1 if not neighbor)
        std::array<std::pair<int8_t, int8_t>, 32> bw_shift_pairs;
        int bw_n_pairs;
    };

    int n_vertices_;
    int n_arcs_;

    // bit_map[v * n_vertices + w] = local bit position of w in v's ng-set, or -1
    std::vector<int8_t> bit_map;  // flat [n_vertices × n_vertices]

    // self_bit_[v] = bit position of v's self bit = number of ng-neighbors
    std::vector<int8_t> self_bit_;  // [n_vertices]

    // Per-arc precomputed transforms
    std::vector<ArcInfo> arc_info;  // [n_arcs]

    // neighbors_[v] = list of vertex IDs in v's ng-neighborhood
    std::vector<std::vector<int>> neighbors_;

    NgPathResource() : n_vertices_(0), n_arcs_(0) {}

    NgPathResource(int n_vertices, int n_arcs, const int* arc_from, const int* arc_to,
                   const std::vector<std::vector<int>>& neighbors)
        : n_vertices_(n_vertices), n_arcs_(n_arcs), neighbors_(neighbors) {
        // Build bit_map: for each vertex v, assign local positions to its neighbors
        bit_map.assign(static_cast<size_t>(n_vertices) * n_vertices, -1);

        // Ensure neighbors_ is sized correctly (may be empty for no-ng instances)
        if (neighbors_.size() < static_cast<size_t>(n_vertices)) {
            neighbors_.resize(n_vertices);
        }

        self_bit_.resize(n_vertices);
        for (int v = 0; v < n_vertices; ++v) {
            int8_t pos = 0;
            for (int w : neighbors_[v]) {
                if (w >= 0 && w < n_vertices && pos < 31) {
                    bit_map[static_cast<size_t>(v) * n_vertices + w] = pos++;
                }
            }
            self_bit_[v] = pos;  // next available bit = number of ng-neighbors
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

            // shift: remap from 'from's ordering to 'to's ordering
            ai.fw_n_pairs = 0;
            for (int w : neighbors_[from]) {
                if (w < 0 || w >= n_vertices) {
                    continue;
                }
                int8_t from_pos = bit_map[static_cast<size_t>(from) * n_vertices + w];
                int8_t to_pos = bit_map[static_cast<size_t>(to) * n_vertices + w];
                if (from_pos >= 0 && to_pos >= 0 && ai.fw_n_pairs < 32) {
                    ai.fw_shift_pairs[ai.fw_n_pairs++] = {from_pos, to_pos};
                }
            }

            // Self-bit shift pair: from's self bit → from's position in to's ordering
            // This maps "I was at from" to "from is visited" in to's bit layout
            int8_t from_pos_in_to = bit_map[static_cast<size_t>(to) * n_vertices + from];
            if (from_pos_in_to >= 0 && ai.fw_n_pairs < 32) {
                ai.fw_shift_pairs[ai.fw_n_pairs++] = {self_bit_[from], from_pos_in_to};
            }

            // === Backward: extending to→from (reverse traversal) ===
            // check: is 'from' in 'to's ng-set?
            ai.bw_check_bit = bit_map[static_cast<size_t>(to) * n_vertices + from];

            // shift: remap from 'to's ordering to 'from's ordering
            ai.bw_n_pairs = 0;
            for (int w : neighbors_[to]) {
                if (w < 0 || w >= n_vertices) {
                    continue;
                }
                int8_t to_pos = bit_map[static_cast<size_t>(to) * n_vertices + w];
                int8_t from_pos = bit_map[static_cast<size_t>(from) * n_vertices + w];
                if (to_pos >= 0 && from_pos >= 0 && ai.bw_n_pairs < 32) {
                    ai.bw_shift_pairs[ai.bw_n_pairs++] = {to_pos, from_pos};
                }
            }

            // Self-bit shift pair: to's self bit → to's position in from's ordering
            int8_t to_pos_in_from = bit_map[static_cast<size_t>(from) * n_vertices + to];
            if (to_pos_in_from >= 0 && ai.bw_n_pairs < 32) {
                ai.bw_shift_pairs[ai.bw_n_pairs++] = {self_bit_[to], to_pos_in_from};
            }
        }
    }

    /// Initialize state: empty (BucketGraph calls extend_to_vertex to mark
    /// source/sink).
    State init_state(Direction /*dir*/) const { return 0U; }

    /// extendAlongArc: transform bit vector from source ordering to target
    /// ordering. Does NOT mark the destination vertex — that's done by
    /// extend_to_vertex.
    std::pair<State, double> extend_along_arc(Direction dir, State s, int arc_id) const {
        auto& ai = arc_info[arc_id];

        if (dir == Direction::Forward) {
            // Forward: arc from→to, label at from, extending to to
            // Check: is 'to' forbidden? (to ∈ N(from) and to's bit set in from's
            // state)
            if (ai.fw_check_bit >= 0 && (s & (uint32_t{1} << ai.fw_check_bit))) {
                return {0U, INF};
            }

            uint32_t new_s = 0;
            for (int i = 0; i < ai.fw_n_pairs; ++i) {
                auto [fp, tp] = ai.fw_shift_pairs[i];
                if (s & (uint32_t{1} << fp)) {
                    new_s |= (uint32_t{1} << tp);
                }
            }
            return {new_s, 0.0};
        } else {
            // Backward: arc from→to traversed as to→from, label at to, extending to
            // from
            if (ai.bw_check_bit >= 0 && (s & (uint32_t{1} << ai.bw_check_bit))) {
                return {0U, INF};
            }

            uint32_t new_s = 0;
            for (int i = 0; i < ai.bw_n_pairs; ++i) {
                auto [fp, tp] = ai.bw_shift_pairs[i];
                if (s & (uint32_t{1} << fp)) {
                    new_s |= (uint32_t{1} << tp);
                }
            }
            return {new_s, 0.0};
        }
    }

    /// extendToVertex: set the destination vertex's self bit, and also
    /// mark vertex in its own ng-neighborhood (if v ∈ N(v)) so that
    /// self-loop arcs are correctly detected by extend_along_arc's check.
    std::pair<State, double> extend_to_vertex(Direction /*dir*/, State s, int vertex) const {
        s |= (uint32_t{1} << self_bit_[vertex]);
        int8_t self_as_neighbor = bit_map[static_cast<size_t>(vertex) * n_vertices_ + vertex];
        if (self_as_neighbor >= 0) {
            s |= (uint32_t{1} << self_as_neighbor);
        }
        return {s, 0.0};
    }

    /// Domination: L1 dominates L2 only if L1's visited set is subset of L2's.
    /// Same vertex → same local mapping (including self bit, which is always set
    /// in both), so direct bitwise comparison works.
    double domination_cost(Direction /*dir*/, int /*vertex*/, State s1, State s2) const {
        if (s1 & ~s2) {
            return INF;
        }
        return 0.0;
    }

    /// Lower bound on domination_cost: 0 (subset check returns 0 or INF).
    double min_domination_cost() const { return 0.0; }

    /// Concatenation: across-arc (i,j) produces fw state at j via
    /// extend_along_arc only (no self bit for j), while bw state at j has j's
    /// self bit set. Any common bit means a vertex visited in both directions →
    /// cycle.
    double concatenation_cost(Symmetry /*sym*/, int /*vertex*/, State s_fw, State s_bw) const {
        if (s_fw & s_bw) {
            return INF;
        }
        return 0.0;
    }

    /// Strip self bit from state for SoA storage (all labels in a bucket share
    /// the same vertex, so the self bit is always 1 — redundant for dominance).
    uint32_t compress_state(State s, int vertex) const {
        return s & ~(uint32_t{1} << self_bit_[vertex]);
    }
};

}  // namespace bgspprc
