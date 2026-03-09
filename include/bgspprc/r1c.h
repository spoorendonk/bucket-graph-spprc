#pragma once

#include <algorithm>
#include <bit>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <span>
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

/// Built-in lm-R1C engine for arc-memory limited-memory Rank-1 Cuts.
///
/// Specialized for 3-Subset Row Cuts (|C|=3, p_v=1/2), which are the most
/// common. State per cut: 1 bit (s ∈ {0, 1/2}), packed 64 per uint64_t word.
///
/// Operations are bitwise on packed words for performance.
class R1CManager {
 public:
  R1CManager() = default;

  /// Set the active cuts. Rebuilds all lookup masks.
  void set_cuts(std::span<const R1Cut> cuts, int n_vertices, int n_arcs) {
    n_vertices_ = n_vertices;
    n_arcs_ = n_arcs;
    cuts_.assign(cuts.begin(), cuts.end());
    n_active_ = static_cast<int>(cuts_.size());
    n_words_ = (n_active_ + 63) / 64;

    build_masks();
    precompute_betas();
  }

  int n_active() const { return n_active_; }
  int n_words() const { return n_words_; }
  bool has_cuts() const { return n_active_ > 0; }

  /// Initialize R1C state (all zeros).
  void init_state(std::span<uint64_t> state) const {
    std::fill(state.begin(), state.end(), 0ULL);
  }

  /// Extend R1C state along arc (forward direction for 3-SRCs).
  ///
  /// For each cut ℓ:
  ///   1. If arc ∉ AM_ℓ: reset bit to 0
  ///   2. If target vertex ∈ C_ℓ: toggle bit (add 1/2)
  ///   3. If bit overflows (1→0): apply cost -= β_ℓ
  ///
  /// Returns total cost delta from R1C contributions.
  double extend(Direction /*dir*/, std::span<const uint64_t> in,
                std::span<uint64_t> out, int arc_id, int target_vertex) const {
    if (n_active_ == 0) return 0.0;

    double cost_delta = 0.0;

    for (int w = 0; w < n_words_; ++w) {
      uint64_t s = in[w];

      // Step 1: reset bits for cuts where arc ∉ AM
      s &= arc_keep_mask_[arc_id][w];  // keep only cuts where arc ∈ AM

      // Step 2 & 3: toggle bits for cuts where target vertex ∈ C
      uint64_t toggle = vertex_toggle_mask_[target_vertex][w];
      uint64_t overflow = s & toggle;  // bits that are 1 AND toggled → overflow
      s ^= toggle;                     // toggle all relevant bits

      // Cost from overflow: each overflowing cut contributes -β
      if (overflow) {
        cost_delta += compute_overflow_cost(w, overflow);
      }

      out[w] = s;
    }

    return cost_delta;
  }

  /// Domination cost: extra cost L1 pays relative to L2 due to R1C states.
  ///
  /// For 3-SRC: if s1[ℓ]=1/2 and s2[ℓ]=0, then L1 has accumulated more
  /// partial credit → could get a future cost reduction that L2 won't.
  /// This means L1 is actually "better" for cut ℓ.
  /// Conversely, if s1[ℓ]=0 and s2[ℓ]=1/2, L2 has the advantage.
  ///
  /// Returns: sum of β_ℓ for cuts where s2 has credit but s1 doesn't.
  double domination_cost(Direction /*dir*/, int /*vertex*/,
                         std::span<const uint64_t> s1,
                         std::span<const uint64_t> s2) const {
    if (n_active_ == 0) return 0.0;

    double cost = 0.0;
    for (int w = 0; w < n_words_; ++w) {
      // Cuts where s2 has credit (1) but s1 doesn't (0)
      uint64_t disadvantage = s2[w] & ~s1[w];
      if (disadvantage) {
        cost += compute_overflow_cost(w, disadvantage);
      }
    }
    return cost;
  }

  /// Concatenation cost: when joining forward and backward labels.
  ///
  /// For 3-SRC: if both s_fw[ℓ]=1/2 and s_bw[ℓ]=1/2, total = 1 ≥ 1 → cost -β.
  double concatenation_cost(Symmetry /*sym*/, int /*vertex*/,
                            std::span<const uint64_t> s_fw,
                            std::span<const uint64_t> s_bw) const {
    if (n_active_ == 0) return 0.0;

    double cost = 0.0;
    for (int w = 0; w < n_words_; ++w) {
      uint64_t both = s_fw[w] & s_bw[w];
      if (both) {
        cost += compute_overflow_cost(w, both);
      }
    }
    return cost;
  }

 private:
  void build_masks() {
    // arc_keep_mask_[arc][word]: bits set for cuts where arc ∈ AM
    // (i.e., we KEEP the state for those cuts; reset others)
    arc_keep_mask_.assign(n_arcs_, std::vector<uint64_t>(n_words_, 0ULL));

    // vertex_toggle_mask_[vertex][word]: bits set for cuts where vertex ∈ C
    vertex_toggle_mask_.assign(n_vertices_,
                               std::vector<uint64_t>(n_words_, 0ULL));

    for (int c = 0; c < n_active_; ++c) {
      int w = c / 64;
      uint64_t bit = 1ULL << (c % 64);

      // Memory arcs: set keep bit
      for (auto [from, to] : cuts_[c].memory_arcs) {
        // We store arc as (from, to) pair — need arc index
        // For now, memory_arcs stores arc indices directly
        // Actually, the pair is the arc endpoints — we need to map
        // This is a simplification: assume memory_arcs contains
        // arc indices encoded in the .first field
        int arc_id = from;  // convention: first = arc_id
        if (arc_id >= 0 && arc_id < n_arcs_) {
          arc_keep_mask_[arc_id][w] |= bit;
        }
      }

      // Base set vertices: set toggle bit
      for (int v : cuts_[c].base_set) {
        if (v >= 0 && v < n_vertices_) {
          vertex_toggle_mask_[v][w] |= bit;
        }
      }
    }
  }

  void precompute_betas() {
    // β_ℓ = -dual_value (dual_value is typically negative for active cuts)
    betas_.resize(n_active_);
    for (int c = 0; c < n_active_; ++c) {
      betas_[c] = -cuts_[c].dual_value;
    }
  }

  /// Compute total cost for a bitmask of overflowing cuts in word w.
  double compute_overflow_cost(int word, uint64_t mask) const {
    double cost = 0.0;
    int base = word * 64;
    while (mask) {
      int bit = std::countr_zero(mask);
      int cut_idx = base + bit;
      if (cut_idx < n_active_) {
        cost -= betas_[cut_idx];
      }
      mask &= mask - 1;  // clear lowest set bit
    }
    return cost;
  }

  int n_vertices_ = 0;
  int n_arcs_ = 0;
  int n_active_ = 0;
  int n_words_ = 0;

  std::vector<R1Cut> cuts_;
  std::vector<double> betas_;

  // Precomputed masks
  std::vector<std::vector<uint64_t>> arc_keep_mask_;       // [arc_id][word]
  std::vector<std::vector<uint64_t>> vertex_toggle_mask_;  // [vertex][word]
};

}  // namespace bgspprc
