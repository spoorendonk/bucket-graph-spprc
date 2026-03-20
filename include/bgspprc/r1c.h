#pragma once

#include <bit>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "types.h"

namespace bgspprc {

/// Rank-1 Cut definition (caller provides).
struct R1Cut {
  std::vector<int> base_set;        // C ⊆ V \ {source, sink}
  std::vector<double> multipliers;  // p_v for v ∈ C (typically 1/2 for 3-SRC)
  std::vector<std::pair<int, int>> memory_arcs;  // AM ⊆ A (arc-memory set)
  double dual_value = 0.0;  // σ_ℓ (dual variable, β = -σ_ℓ > 0)
};

/// lm-R1C resource conforming to the Resource concept.
///
/// Specialized for 3-Subset Row Cuts (|C|=3, p_v=1/2). State per cut: 1 bit,
/// packed into a single uint64_t (max 64 active cuts).
///
/// extend_along_arc: memory reset (keep bits for cuts where arc ∈ AM)
/// extend_to_vertex: toggle bits for cuts where vertex ∈ C, cost from overflow
struct R1CResource {
  using State = uint64_t;

  /// Set the active cuts. Rebuilds all lookup masks.
  /// Throws if >64 cuts (single uint64_t state).
  void set_cuts(std::span<const R1Cut> cuts, int n_vertices, int n_arcs) {
    if (cuts.size() > 64)
      throw std::invalid_argument(
          "R1CResource supports at most 64 cuts (uint64_t state limit)");
    for (const auto& cut : cuts) {
      if (cut.base_set.size() != cut.multipliers.size())
        throw std::invalid_argument(
            "R1CResource: base_set and multipliers must have equal size");
      for (double p : cut.multipliers) {
        if (std::abs(p - 0.5) > 1e-9)
          throw std::invalid_argument(
              "R1CResource only supports 3-SRC (p=1/2) multipliers");
      }
    }
    n_vertices_ = n_vertices;
    n_arcs_ = n_arcs;
    cuts_.assign(cuts.begin(), cuts.end());
    n_active_ = static_cast<int>(cuts_.size());

    build_masks();
    precompute_betas();
  }

  int n_active() const { return n_active_; }

  // ── Resource concept interface ──

  bool symmetric() const { return true; }

  State init_state(Direction /*dir*/) const { return 0ULL; }

  /// Memory reset along arc: keep bits for cuts where arc ∈ AM.
  std::pair<State, double> extend_along_arc(Direction /*dir*/, State state,
                                            int arc_id) const {
    if (n_active_ == 0) return {state, 0.0};
    return {state & arc_keep_mask_[arc_id], 0.0};
  }

  /// Toggle bits for cuts where vertex ∈ C. Overflow (1→0) applies cost -β.
  std::pair<State, double> extend_to_vertex(Direction /*dir*/, State state,
                                            int vertex) const {
    if (n_active_ == 0) return {state, 0.0};

    uint64_t toggle = vertex_toggle_mask_[vertex];
    uint64_t overflow = state & toggle;
    state ^= toggle;

    double cost = 0.0;
    if (overflow) cost = compute_overflow_cost(overflow);
    return {state, cost};
  }

  /// Extra cost L1 pays vs L2: penalty for cuts where s2 has credit but s1
  /// doesn't.
  double domination_cost(Direction /*dir*/, int /*vertex*/, State s1,
                         State s2) const {
    if (n_active_ == 0) return 0.0;
    uint64_t disadvantage = s2 & ~s1;
    if (disadvantage) return compute_overflow_cost(disadvantage);
    return 0.0;
  }

  /// Lower bound on domination_cost: sum of all negative beta contributions.
  /// domination_cost sums -beta[c] for cuts where s2 has credit but s1 doesn't,
  /// so worst case is all cuts contributing their negative beta.
  double min_domination_cost() const {
    double total = 0.0;
    for (int c = 0; c < n_active_; ++c) {
      double contrib = -betas_[c];
      if (contrib < 0.0) total += contrib;
    }
    return total;
  }

  /// Cost when joining fw/bw labels: if both have credit, overflow.
  double concatenation_cost(Symmetry /*sym*/, int /*vertex*/, State s_fw,
                            State s_bw) const {
    if (n_active_ == 0) return 0.0;
    uint64_t both = s_fw & s_bw;
    if (both) return compute_overflow_cost(both);
    return 0.0;
  }

 private:
  void build_masks() {
    arc_keep_mask_.assign(n_arcs_, 0ULL);
    vertex_toggle_mask_.assign(n_vertices_, 0ULL);

    for (int c = 0; c < n_active_; ++c) {
      uint64_t bit = 1ULL << c;

      for (auto [from, to] : cuts_[c].memory_arcs) {
        int arc_id = from;  // convention: first = arc_id
        if (arc_id >= 0 && arc_id < n_arcs_) {
          arc_keep_mask_[arc_id] |= bit;
        }
      }

      for (int v : cuts_[c].base_set) {
        if (v >= 0 && v < n_vertices_) {
          vertex_toggle_mask_[v] |= bit;
        }
      }
    }
  }

  void precompute_betas() {
    betas_.resize(n_active_);
    for (int c = 0; c < n_active_; ++c) {
      betas_[c] = -cuts_[c].dual_value;
    }
  }

  double compute_overflow_cost(uint64_t mask) const {
    double cost = 0.0;
    while (mask) {
      int bit = std::countr_zero(mask);
      if (bit < n_active_) {
        cost -= betas_[bit];
      }
      mask &= mask - 1;
    }
    return cost;
  }

  int n_vertices_ = 0;
  int n_arcs_ = 0;
  int n_active_ = 0;

  std::vector<R1Cut> cuts_;
  std::vector<double> betas_;
  std::vector<uint64_t> arc_keep_mask_;       // [arc_id]
  std::vector<uint64_t> vertex_toggle_mask_;  // [vertex]
};

/// Backward-compatible alias.
using R1CManager = R1CResource;

}  // namespace bgspprc
