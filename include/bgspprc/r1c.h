#pragma once

#include "executor.h"
#include "types.h"

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bgspprc {

/// Rank-1 Cut definition (caller provides).
struct R1Cut {
    std::vector<int> base_set;                     // C ⊆ V \ {source, sink}
    std::vector<double> multipliers;               // p_v for v ∈ C (typically 1/2 for 3-SRC)
    std::vector<std::pair<int, int>> memory_arcs;  // AM ⊆ A (arc-memory set)
    double dual_value = 0.0;  // σ_ℓ (dual variable, β = -σ_ℓ ≥ 0; clamped at 0)
};

/// Derived R1C data: the active cuts and their lookup masks. Owns its
/// arrays; borrowed by `R1CView`. Rebuilt by `set_cuts` whenever the cuts or
/// their duals change.
class R1CData {
public:
    /// Set the active cuts. Rebuilds all lookup masks.
    /// Throws if >64 cuts (single uint64_t state).
    /// The executor is used to parallelize mask building over cuts.
    template <Executor Exec = SequentialExecutor>
    void set_cuts(std::span<const R1Cut> cuts, int n_vertices, int n_arcs, Exec&& exec = {}) {
        if (cuts.size() > 64) {
            throw std::invalid_argument(
                "R1CResource supports at most 64 cuts (uint64_t state limit)");
        }
        for (const auto& cut : cuts) {
            if (cut.base_set.size() != cut.multipliers.size()) {
                throw std::invalid_argument(
                    "R1CResource: base_set and multipliers must have equal size");
            }
            for (double p : cut.multipliers) {
                if (std::abs(p - 0.5) > 1e-9) {
                    throw std::invalid_argument(
                        "R1CResource only supports 3-SRC (p=1/2) multipliers");
                }
            }
        }
        n_vertices_ = n_vertices;
        n_arcs_ = n_arcs;
        cuts_.assign(cuts.begin(), cuts.end());
        n_active_ = static_cast<int>(cuts_.size());

        build_masks(std::forward<Exec>(exec));
        precompute_betas();
    }

    [[nodiscard]] int n_active() const { return n_active_; }

private:
    friend class R1CView;

    /// Build arc_keep_mask_ and vertex_toggle_mask_ from cuts.
    /// Parallelizes the outer cut loop via the executor. Each cut writes a
    /// unique bit (1ULL << c), so concurrent |= targets disjoint bit lanes.
    /// std::atomic_ref ensures the read-modify-write is race-free.
    template <Executor Exec = SequentialExecutor>
    void build_masks(Exec&& exec = {}) {
        arc_keep_mask_.assign(n_arcs_, 0ULL);
        vertex_toggle_mask_.assign(n_vertices_, 0ULL);

        exec.parallel_for(0, n_active_, [&](int c) {
            uint64_t bit = 1ULL << c;

            for (auto [from, to] : cuts_[c].memory_arcs) {
                int arc_id = from;  // convention: first = arc_id
                if (arc_id >= 0 && arc_id < n_arcs_) {
                    std::atomic_ref(arc_keep_mask_[arc_id])
                        .fetch_or(bit, std::memory_order_relaxed);
                }
            }

            for (int v : cuts_[c].base_set) {
                if (v >= 0 && v < n_vertices_) {
                    std::atomic_ref(vertex_toggle_mask_[v])
                        .fetch_or(bit, std::memory_order_relaxed);
                }
            }
        });
    }

    void precompute_betas() {
        betas_.resize(n_active_);
        for (int c = 0; c < n_active_; ++c) {
            // β = -σ_ℓ ≥ 0 for a valid R1C dual (σ_ℓ ≤ 0). Clamp at 0 so a
            // malformed / LP-noisy positive dual cannot make β negative — a
            // negative β would turn the overflow penalty into a credit and break
            // the min_domination_cost() == 0 lower bound (→ unsound over-pruning).
            betas_[c] = std::max(0.0, -cuts_[c].dual_value);
        }
    }

    int n_vertices_ = 0;
    int n_arcs_ = 0;
    int n_active_ = 0;

    std::vector<R1Cut> cuts_;
    std::vector<double> betas_;
    std::vector<uint64_t> arc_keep_mask_;       // [arc_id]
    std::vector<uint64_t> vertex_toggle_mask_;  // [vertex]
};

/// lm-R1C resource conforming to the Resource concept, borrowing caller-owned
/// `R1CData` (flowty/flowty#58), 8 bytes. The data must outlive the view.
/// BucketGraph finds it in a pack by the `is_r1c_resource` tag.
///
/// Specialized for 3-Subset Row Cuts (|C|=3, p_v=1/2). State per cut: 1 bit,
/// packed into a single uint64_t (max 64 active cuts).
///
/// extend_along_arc: memory reset (keep bits for cuts where arc ∈ AM)
/// extend_to_vertex: toggle bits for cuts where vertex ∈ C, cost from overflow
class R1CView {
public:
    static constexpr bool is_r1c_resource = true;
    using State = uint64_t;

    // No default: a view must borrow something. Without cuts, borrow an
    // R1CData that has none (n_active() == 0), like a default R1CResource.
    R1CView() = delete;
    explicit R1CView(const R1CData& data) noexcept : data_(&data) {}
    R1CView(const R1CData&&) = delete;

    [[nodiscard]] int n_active() const { return data_->n_active(); }

    // ── Resource concept interface ──

    [[nodiscard]] bool symmetric() const { return true; }

    [[nodiscard]] State init_state(Direction /*dir*/) const { return 0ULL; }

    /// Memory reset along arc: keep bits for cuts where arc ∈ AM.
    [[nodiscard]] std::pair<State, double> extend_along_arc(Direction /*dir*/, State state,
                                                            int arc_id) const {
        if (data_->n_active_ == 0) {
            return {state, 0.0};
        }
        return {state & data_->arc_keep_mask_[arc_id], 0.0};
    }

    /// Toggle bits for cuts where vertex ∈ C. Overflow (1→0) applies cost +β (penalty).
    [[nodiscard]] std::pair<State, double> extend_to_vertex(Direction /*dir*/, State state,
                                                            int vertex) const {
        if (data_->n_active_ == 0) {
            return {state, 0.0};
        }

        uint64_t toggle = data_->vertex_toggle_mask_[vertex];
        uint64_t overflow = state & toggle;
        state ^= toggle;

        double cost = 0.0;
        if (overflow) {
            cost = compute_overflow_cost(overflow);
        }
        return {state, cost};
    }

    /// Extra cost L1 must overcome to dominate L2 — a non-negative PENALTY.
    /// Holding a cut bit is a disadvantage (you are one base-set visit from paying
    /// the +β overflow penalty). For each cut where the dominator L1 holds a bit
    /// the dominated L2 lacks (s1 & ~s2), L1 pays one extra +β over an odd
    /// completion that L2 escapes, so L1 must be +β cheaper to dominate for every
    /// completion: c(L1) + Σ_{s1 & ~s2} β ≤ c(L2).
    [[nodiscard]] double domination_cost(Direction /*dir*/, int /*vertex*/, State s1,
                                         State s2) const {
        if (data_->n_active_ == 0) {
            return 0.0;
        }
        uint64_t disadvantage = s1 & ~s2;
        if (disadvantage) {
            return compute_overflow_cost(disadvantage);
        }
        return 0.0;
    }

    /// Lower bound on domination_cost: 0. The dominance penalty is a sum of +β
    /// (β ≥ 0), so it is never negative.
    [[nodiscard]] double min_domination_cost() const { return 0.0; }

    /// Cost when joining fw/bw labels: if both sides hold the bit, the join
    /// completes that coefficient and is charged the +β overflow penalty.
    [[nodiscard]] double concatenation_cost(Symmetry /*sym*/, int /*vertex*/, State s_fw,
                                            State s_bw) const {
        if (data_->n_active_ == 0) {
            return 0.0;
        }
        uint64_t both = s_fw & s_bw;
        if (both) {
            return compute_overflow_cost(both);
        }
        return 0.0;
    }

private:
    /// Sum of +β over the set bits of `mask` — the reduced-cost penalty for the
    /// coefficients those bits represent (β = -σ ≥ 0, so each overflow raises cost).
    [[nodiscard]] double compute_overflow_cost(uint64_t mask) const {
        double cost = 0.0;
        while (mask) {
            int bit = std::countr_zero(mask);
            if (bit < data_->n_active_) {
                cost += data_->betas_[bit];
            }
            mask &= mask - 1;
        }
        return cost;
    }

    const R1CData* data_ = nullptr;  // not owned
};

/// Owning R1C resource: `R1CData` plus the resource methods, forwarded to a
/// view over itself.
struct R1CResource : R1CData {
    static constexpr bool is_r1c_resource = true;
    using State = R1CView::State;

    /// The borrowing view. Deleted on an rvalue, so a view of a temporary is a
    /// compile error rather than a dangle.
    [[nodiscard]] R1CView view() const& noexcept { return R1CView(*this); }
    [[nodiscard]] R1CView view() const&& = delete;

    [[nodiscard]] bool symmetric() const { return view().symmetric(); }
    [[nodiscard]] State init_state(Direction dir) const { return view().init_state(dir); }
    [[nodiscard]] std::pair<State, double> extend_along_arc(Direction dir, State state,
                                                            int arc_id) const {
        return view().extend_along_arc(dir, state, arc_id);
    }
    [[nodiscard]] std::pair<State, double> extend_to_vertex(Direction dir, State state,
                                                            int vertex) const {
        return view().extend_to_vertex(dir, state, vertex);
    }
    [[nodiscard]] double domination_cost(Direction dir, int vertex, State s1, State s2) const {
        return view().domination_cost(dir, vertex, s1, s2);
    }
    [[nodiscard]] double min_domination_cost() const { return view().min_domination_cost(); }
    [[nodiscard]] double concatenation_cost(Symmetry sym, int vertex, State s_fw,
                                            State s_bw) const {
        return view().concatenation_cost(sym, vertex, s_fw, s_bw);
    }
};

/// Backward-compatible alias.
using R1CManager = R1CResource;

}  // namespace bgspprc
