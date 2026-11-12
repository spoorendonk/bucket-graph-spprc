#pragma once

#include "executor.h"
#include "problem_view.h"
#include "types.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace bgspprc {

/// Rank-1 Cut definition (caller provides).
///
/// A cut with limited arc memory (lm-R1C, Pecin et al. 2017): a path's
/// coefficient is ⌊Σ_v p_v·a_v⌋, summed over the visits a_v to the base set
/// while the path stays on memory arcs; leaving the memory resets the sum.
/// Every multiplier is a fraction p_v = k_v/d in (0, 1) over a common
/// denominator d ∈ [2, 8] — the subset-row family SR12, SR32, SR52 (d = 2),
/// SR43, SR53 (3), SR54 (4), SR55, SR65 (5), SR76 (6), SR87 (7), SR98 (8).
/// `set_cuts` takes d as the smallest denominator the multipliers share:
/// ⌊Σ k_v a_v / d⌋ is the same for any common one, and the smallest takes
/// the fewest bits. A vertex listed twice adds both multipliers, as in
/// pull-rcspp's cut form.
struct R1Cut {
    std::vector<int> base_set;                     // C ⊆ V \ {source, sink}
    std::vector<double> multipliers;               // p_v for v ∈ C (1/2 for a 3-SRC)
    std::vector<std::pair<int, int>> memory_arcs;  // AM ⊆ A (arc-memory set); first = arc id
    double dual_value = 0.0;  // σ_ℓ (dual variable, β = -σ_ℓ ≥ 0; clamped at 0)
};

/// Rank-1 cut with vertex memory, pull-rcspp's cut form field for field: a
/// path's coefficient is ⌊Σ_v k_v·a_v / d⌋ over its visits a_v to the
/// coefficient vertices while it stays inside the memory. With
/// M' = memory ∪ coefficient vertices, arc (i, j) keeps the running sum iff
/// both i and j are in M'; any other arc resets it. Each numerator lies in
/// [1, d - 1], and a vertex listed twice sums its numerators. The denominator
/// is taken as given, not reduced to the smallest one the numerators share.
struct R1VertexCut {
    std::vector<std::pair<int, int>> coefficients;  // (vertex, numerator k_v)
    int denominator = 2;                            // d ∈ [2, 8]
    std::vector<int> memory;                        // memory vertices
    double dual = 0.0;                              // σ_ℓ (β = -σ_ℓ ≥ 0; clamped at 0)
};

/// The largest denominator of a cut's multipliers.
inline constexpr int kR1CMaxDenominator = 8;

/// Why `set_cuts` refused its cuts.
enum class R1CErrc : uint8_t {
    InvalidCut,       // a malformed cut: bad sizes, multiplier, numerator,
                      // denominator, vertex or arc, or a NaN value
    TooManyCounters,  // the fields need more bits than the state words hold
};

/// What `set_cuts` throws. Derives from `std::invalid_argument`, so callers
/// catching that keep working. `cut` is the index of the offending cut and
/// `vertex` the offending vertex, -1 where none applies.
struct R1CError : std::invalid_argument {
    R1CError(R1CErrc errc, const char* what, int cut_index, int vertex_id = -1)
        : std::invalid_argument(what), code(errc), cut(cut_index), vertex(vertex_id) {}

    R1CErrc code;
    int cut = -1;
    int vertex = -1;
};

/// The packed state of `Words` 64-bit words: a plain `uint64_t` for one word,
/// so that with only 1-bit cuts, cut c is bit c.
template <std::size_t Words>
using R1CState = std::conditional_t<Words == 1, uint64_t, std::array<uint64_t, Words>>;

template <std::size_t Words>
class BasicR1CView;

/// Derived R1C data: the active cuts, their packed layout and the lookup
/// masks. Owns its arrays; borrowed by `BasicR1CView`. Rebuilt by `set_cuts`
/// whenever the cuts or their duals change.
///
/// Layout: each cut c holds its running sum mod d_c in a field of
/// w_c = ⌈log2 d_c⌉ bits (1 for d = 2, 2 for d ∈ {3, 4}, 3 for d ∈ [5, 8]),
/// placed in cut order in the `Words` 64-bit words of the state; a field never
/// straddles two words. A cut that does not fit makes `set_cuts` throw, as
/// does a malformed one; on a throw the previous cuts stay.
template <std::size_t Words>
class BasicR1CData {
    static_assert(Words >= 1, "at least one state word");

public:
    static constexpr std::size_t kWords = Words;

    /// Where cut c's field lives in the state.
    struct Field {
        int word = 0;   // [0, Words)
        int shift = 0;  // lowest bit of the field in its word
        int width = 0;  // bits: 1, 2 or 3
        int denominator = 2;
    };

    /// Set the active cuts (arc memory). Rebuilds the layout and all lookup
    /// tables. Throws `R1CError` (InvalidCut) if a cut is malformed (sizes
    /// differ, a multiplier outside (0, 1) or not k/d for some d ≤ 8, a vertex
    /// or arc out of range, a NaN dual), or (TooManyCounters) if the cuts need
    /// more than `64·Words` bits. The executor is used to parallelize the
    /// arc-memory masks over cuts.
    template <Executor Exec = SequentialExecutor>
    void set_cuts(std::span<const R1Cut> cuts, int n_vertices, int n_arcs, Exec&& exec = {}) {
        BasicR1CData next;
        next.n_vertices_ = n_vertices;
        next.n_arcs_ = n_arcs;
        std::vector<NormCut> norm(cuts.size());
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            norm[c] = normalize(cuts[c], static_cast<int>(c), n_vertices, n_arcs);
        }
        next.build(norm, nullptr, nullptr, std::forward<Exec>(exec));
        *this = std::move(next);  // in place: borrowing views stay valid
    }

    /// Set the active cuts (vertex memory, pull-rcspp's form). The view gives
    /// the arc endpoints the memory is read on, and must be the one the
    /// resource prices. Throws as the `R1Cut` overload, InvalidCut also for a
    /// denominator outside [2, 8], a numerator outside [1, d - 1] or a memory
    /// vertex out of range.
    template <Executor Exec = SequentialExecutor>
    void set_cuts(std::span<const R1VertexCut> cuts, const ProblemView& pv, Exec&& exec = {}) {
        BasicR1CData next;
        next.n_vertices_ = pv.n_vertices;
        next.n_arcs_ = pv.n_arcs;
        std::vector<NormCut> norm(cuts.size());
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            norm[c] = normalize(cuts[c], static_cast<int>(c), pv.n_vertices);
        }
        next.build(norm, pv.arc_from, pv.arc_to, std::forward<Exec>(exec));
        *this = std::move(next);  // in place: borrowing views stay valid
    }

    [[nodiscard]] int n_active() const { return n_active_; }

    /// Cut c's field in the packed state.
    [[nodiscard]] Field field(int cut) const { return fields_[static_cast<std::size_t>(cut)]; }

    /// Cut c's β = max(0, -σ).
    [[nodiscard]] double beta(int cut) const { return betas_[static_cast<std::size_t>(cut)]; }

private:
    friend class BasicR1CView<Words>;

    static constexpr int kBits = 64;

    // The one form both cut types are built from: merged numerators, the
    // denominator, β, and the memory as either arc ids or a vertex set.
    struct NormCut {
        std::vector<std::pair<int, int>> numerators;  // (vertex, k_v), sorted, merged
        int denominator = 2;
        double beta = 0.0;
        std::vector<int> memory_arcs;  // arc memory: the arcs that keep the sum
        std::vector<char> in_memory;   // vertex memory: [vertex] ∈ M'; empty for arc memory
    };

    // A cut a vertex adds to off the 1-bit fast path: its field and the
    // vertex's numerator k_v (summed when listed twice, so it may reach d).
    struct Increment {
        int word = 0;
        int shift = 0;
        uint64_t mask = 0;  // (1 << width) - 1, unshifted
        uint64_t numerator = 0;
        uint64_t denominator = 0;
        double beta = 0.0;
    };

    // The smallest d ∈ [2, 8] with every p·d integral, or 0 if there is none.
    static int common_denominator(std::span<const double> multipliers) {
        for (int d = 2; d <= kR1CMaxDenominator; ++d) {
            const bool all = std::ranges::all_of(multipliers, [d](double p) {
                const double k = p * d;
                return std::abs(k - std::round(k)) <= 1e-9;
            });
            if (all) {
                return d;
            }
        }
        return 0;
    }

    static int field_width(int denominator) {
        return std::bit_width(static_cast<unsigned>(denominator - 1));
    }

    // β = -σ_ℓ ≥ 0 for a valid R1C dual (σ_ℓ ≤ 0). Clamp at 0 so a malformed /
    // LP-noisy positive dual cannot make β negative — a negative β would turn
    // the overflow penalty into a credit and break the min_domination_cost()
    // == 0 lower bound (→ unsound over-pruning).
    static double beta_of(double dual, int c) {
        if (std::isnan(dual)) {
            throw R1CError(R1CErrc::InvalidCut, "R1CResource: a cut's dual is NaN", c);
        }
        return std::max(0.0, -dual);
    }

    // Sort and merge (vertex, k) pairs: a vertex listed twice sums its
    // numerators, as pull-rcspp does.
    static void merge_numerators(std::vector<std::pair<int, int>>& numerators) {
        std::ranges::sort(numerators);
        std::size_t out = 0;
        for (std::size_t i = 0; i < numerators.size();) {
            const int v = numerators[i].first;
            int k = 0;
            for (; i < numerators.size() && numerators[i].first == v; ++i) {
                k += numerators[i].second;
            }
            numerators[out++] = {v, k};
        }
        numerators.resize(out);
    }

    static NormCut normalize(const R1Cut& cut, int c, int n_vertices, int n_arcs) {
        if (cut.base_set.size() != cut.multipliers.size()) {
            throw R1CError(R1CErrc::InvalidCut,
                           "R1CResource: base_set and multipliers must have equal size", c);
        }
        for (std::size_t i = 0; i < cut.multipliers.size(); ++i) {
            const double p = cut.multipliers[i];
            if (std::isnan(p) || p <= 1e-9 || p >= 1.0 - 1e-9) {
                throw R1CError(R1CErrc::InvalidCut, "R1CResource: multipliers must lie in (0, 1)",
                               c, cut.base_set[i]);
            }
        }
        NormCut out;
        out.beta = beta_of(cut.dual_value, c);
        out.denominator = cut.multipliers.empty() ? 2 : common_denominator(cut.multipliers);
        if (out.denominator == 0) {
            throw R1CError(R1CErrc::InvalidCut,
                           "R1CResource: multipliers must be k/d over one denominator d in [2, 8]",
                           c);
        }
        out.numerators.reserve(cut.base_set.size());
        for (std::size_t i = 0; i < cut.base_set.size(); ++i) {
            const int v = cut.base_set[i];
            if (v < 0 || v >= n_vertices) {
                throw R1CError(R1CErrc::InvalidCut,
                               "R1CResource: a base-set vertex is out of range", c, v);
            }
            out.numerators.emplace_back(
                v, static_cast<int>(std::lround(cut.multipliers[i] * out.denominator)));
        }
        merge_numerators(out.numerators);
        out.memory_arcs.reserve(cut.memory_arcs.size());
        for (const auto& arc : cut.memory_arcs) {
            if (arc.first < 0 || arc.first >= n_arcs) {
                throw R1CError(R1CErrc::InvalidCut, "R1CResource: a memory arc is out of range", c);
            }
            out.memory_arcs.push_back(arc.first);
        }
        return out;
    }

    static NormCut normalize(const R1VertexCut& cut, int c, int n_vertices) {
        const int d = cut.denominator;
        if (d < 2 || d > kR1CMaxDenominator) {
            throw R1CError(R1CErrc::InvalidCut, "R1CResource: a denominator must lie in [2, 8]", c);
        }
        NormCut out;
        out.beta = beta_of(cut.dual, c);
        out.denominator = d;
        out.in_memory.assign(static_cast<std::size_t>(n_vertices), 0);
        auto in_range = [&](int v) {
            if (v < 0 || v >= n_vertices) {
                throw R1CError(R1CErrc::InvalidCut, "R1CResource: a cut's vertex is out of range",
                               c, v);
            }
            return static_cast<std::size_t>(v);
        };
        out.numerators.reserve(cut.coefficients.size());
        for (const auto& [v, k] : cut.coefficients) {
            out.in_memory[in_range(v)] = 1;
            if (k < 1 || k >= d) {
                throw R1CError(R1CErrc::InvalidCut,
                               "R1CResource: a numerator must lie in [1, denominator - 1]", c, v);
            }
            out.numerators.emplace_back(v, k);
        }
        merge_numerators(out.numerators);
        for (int v : cut.memory) {
            out.in_memory[in_range(v)] = 1;
        }
        return out;
    }

    // `arc_from`/`arc_to` are the endpoints vertex memories are read on; null
    // when every cut has arc memory.
    template <Executor Exec>
    void build(std::span<const NormCut> cuts, const int* arc_from, const int* arc_to, Exec&& exec) {
        n_active_ = static_cast<int>(cuts.size());
        fields_.resize(cuts.size());
        betas_.resize(cuts.size());
        one_mask_.fill(0);
        low2_mask_.fill(0);
        low3_mask_.fill(0);
        beta_at_.fill(0.0);
        den_at_.fill(0);

        int word = 0;
        int shift = 0;
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            const int d = cuts[c].denominator;
            const int width = field_width(d);
            if (shift + width > kBits) {
                ++word;
                shift = 0;
            }
            if (std::cmp_greater_equal(word, Words)) {
                throw R1CError(
                    R1CErrc::TooManyCounters,
                    "R1CResource: the cuts need more state bits than the resource's words hold",
                    static_cast<int>(c));
            }
            fields_[c] = {.word = word, .shift = shift, .width = width, .denominator = d};
            betas_[c] = cuts[c].beta;
            const uint64_t low = 1ULL << shift;
            const auto w = static_cast<std::size_t>(word);
            if (width == 1) {
                one_mask_[w] |= low;
            } else if (width == 2) {
                low2_mask_[w] |= low;
            } else {
                low3_mask_[w] |= low;
            }
            beta_at_[(w * kBits) + static_cast<std::size_t>(shift)] = betas_[c];
            den_at_[(w * kBits) + static_cast<std::size_t>(shift)] = static_cast<uint8_t>(d);
            shift += width;
        }

        build_vertex_tables(cuts);
        build_arc_masks(cuts, arc_from, arc_to, std::forward<Exec>(exec));
    }

    // Per vertex: the toggle mask of the 1-bit fast path (d = 2, numerator 1)
    // and the increments of every other (cut, vertex) pair.
    void build_vertex_tables(std::span<const NormCut> cuts) {
        const auto n_v = static_cast<std::size_t>(n_vertices_);
        toggle_mask_.assign(n_v * Words, 0ULL);
        std::vector<std::vector<Increment>> incs(n_v);
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            const Field f = fields_[c];
            for (const auto& [v, k] : cuts[c].numerators) {
                const auto vi = static_cast<std::size_t>(v);
                if (f.width == 1 && k == 1) {
                    toggle_mask_[(vi * Words) + static_cast<std::size_t>(f.word)] |= 1ULL
                                                                                     << f.shift;
                } else {
                    incs[vi].push_back({.word = f.word,
                                        .shift = f.shift,
                                        .mask = (1ULL << f.width) - 1,
                                        .numerator = static_cast<uint64_t>(k),
                                        .denominator = static_cast<uint64_t>(f.denominator),
                                        .beta = betas_[c]});
                }
            }
        }
        inc_begin_.assign(1, 0);
        increments_.clear();
        for (const auto& row : incs) {
            increments_.insert(increments_.end(), row.begin(), row.end());
            inc_begin_.push_back(static_cast<uint32_t>(increments_.size()));
        }
    }

    /// Build arc_keep_mask_ from the cuts' memories. Parallelizes the cut
    /// loop via the executor. Each cut owns its field's bits, so concurrent
    /// |= targets disjoint bits; std::atomic_ref keeps the read-modify-write
    /// race-free.
    template <Executor Exec>
    void build_arc_masks(std::span<const NormCut> cuts, const int* arc_from, const int* arc_to,
                         Exec&& exec) {
        arc_keep_mask_.assign(static_cast<std::size_t>(n_arcs_) * Words, 0ULL);
        exec.parallel_for(0, n_active_, [&](int c) {
            const NormCut& cut = cuts[static_cast<std::size_t>(c)];
            const Field f = fields_[static_cast<std::size_t>(c)];
            const uint64_t bits = ((1ULL << f.width) - 1) << f.shift;
            auto keep = [&](int a) {
                auto& m = arc_keep_mask_[(static_cast<std::size_t>(a) * Words) +
                                         static_cast<std::size_t>(f.word)];
                std::atomic_ref(m).fetch_or(bits, std::memory_order_relaxed);
            };
            if (cut.in_memory.empty()) {
                for (int a : cut.memory_arcs) {
                    keep(a);
                }
                return;
            }
            for (int a = 0; a < n_arcs_; ++a) {
                const auto i = static_cast<std::size_t>(arc_from[a]);
                const auto j = static_cast<std::size_t>(arc_to[a]);
                if (cut.in_memory[i] != 0 && cut.in_memory[j] != 0) {
                    keep(a);
                }
            }
        });
    }

    int n_vertices_ = 0;
    int n_arcs_ = 0;
    int n_active_ = 0;

    std::vector<Field> fields_;                    // [cut]
    std::vector<double> betas_;                    // [cut]
    std::array<uint64_t, Words> one_mask_{};       // bits of the 1-bit fields
    std::array<uint64_t, Words> low2_mask_{};      // lowest bit of each 2-bit field
    std::array<uint64_t, Words> low3_mask_{};      // lowest bit of each 3-bit field
    std::array<double, Words * kBits> beta_at_{};  // β of the field whose lowest bit this is
    std::array<uint8_t, Words * kBits> den_at_{};  // d of the field whose lowest bit this is
    std::vector<uint64_t> arc_keep_mask_;          // [arc * Words + word]
    std::vector<uint64_t> toggle_mask_;            // [vertex * Words + word]
    std::vector<uint32_t> inc_begin_;              // [vertex], n_vertices + 1 entries
    std::vector<Increment> increments_;            // a vertex's increments
};

/// lm-R1C resource conforming to the Resource concept, borrowing caller-owned
/// `BasicR1CData` (flowty/flowty#58), 8 bytes. The data must outlive the view.
/// It carries the `is_r1c_resource` tag, by which `detail::R1CResourceIndex`
/// finds it in a pack.
///
/// State: per cut, its running sum mod d in its field (see `BasicR1CData`).
/// extend_along_arc: memory reset (keep the fields of cuts where arc ∈ AM)
/// extend_to_vertex: add the vertex's numerators; a sum reaching d pays +β
/// per multiple of d and keeps the remainder. On 1-bit cuts with numerator 1
/// (the 3-SRC) that is a toggle, and runs word-parallel.
template <std::size_t Words>
class BasicR1CView {
public:
    static constexpr bool is_r1c_resource = true;
    using State = R1CState<Words>;
    using Data = BasicR1CData<Words>;

    // No default: a view must borrow something. Without cuts, borrow an
    // R1CData that has none (n_active() == 0), like a default R1CResource.
    BasicR1CView() = delete;
    explicit BasicR1CView(const Data& data) noexcept : data_(&data) {}
    BasicR1CView(const Data&&) = delete;

    [[nodiscard]] int n_active() const { return data_->n_active(); }

    /// Word `i` of a state.
    [[nodiscard]] static uint64_t word(const State& s, std::size_t i) {
        if constexpr (Words == 1) {
            static_cast<void>(i);
            return s;
        } else {
            return s[i];
        }
    }

    /// For each field of `left`'s layout of width 2 (with `low` its lowest
    /// bits), the lowest bit set where `left`'s field exceeds `right`'s.
    /// Bit-parallel over the word: with (a b) the left field and (c d) the
    /// right one, (a b) > (c d) ⟺ a¬c ∨ (a = c) b¬d.
    [[nodiscard]] static constexpr uint64_t exceeds2(uint64_t left, uint64_t right, uint64_t low) {
        const uint64_t a = (left >> 1) & low;
        const uint64_t b = left & low;
        const uint64_t not_c = ~(right >> 1) & low;
        const uint64_t not_d = ~right & low;
        return (b & not_c & not_d) | (a & b & not_d) | (a & not_c);
    }

    /// As `exceeds2` for 3-bit fields: (a b c) > (d e f).
    [[nodiscard]] static constexpr uint64_t exceeds3(uint64_t left, uint64_t right, uint64_t low) {
        const uint64_t a = (left >> 2) & low;
        const uint64_t b = (left >> 1) & low;
        const uint64_t c = left & low;
        const uint64_t not_d = ~(right >> 2) & low;
        const uint64_t not_e = ~(right >> 1) & low;
        const uint64_t not_f = ~right & low;
        return (c & not_d & not_e & not_f) | (b & c & not_d & not_f) | (a & c & not_e & not_f) |
               (a & b & c & not_f) | (b & not_d & not_e) | (a & b & not_e) | (a & not_d);
    }

    // ── Resource concept interface ──

    [[nodiscard]] bool symmetric() const { return true; }

    [[nodiscard]] State init_state(Direction /*dir*/) const { return State{}; }

    /// Memory reset along arc: keep the fields of cuts where arc ∈ AM.
    [[nodiscard]] std::pair<State, double> extend_along_arc(Direction /*dir*/, State state,
                                                            int arc_id) const {
        if (data_->n_active_ == 0) {
            return {state, 0.0};
        }
        const uint64_t* keep = &data_->arc_keep_mask_[static_cast<std::size_t>(arc_id) * Words];
        for (std::size_t w = 0; w < Words; ++w) {
            word_ref(state, w) &= keep[w];
        }
        return {state, 0.0};
    }

    /// Add the vertex's numerators. A 1-bit field with numerator 1 toggles,
    /// and pays +β when it was set; any other adds k_v and pays +β per
    /// multiple of d reached.
    [[nodiscard]] std::pair<State, double> extend_to_vertex(Direction /*dir*/, State state,
                                                            int vertex) const {
        if (data_->n_active_ == 0) {
            return {state, 0.0};
        }
        const auto v = static_cast<std::size_t>(vertex);
        double cost = 0.0;
        const uint64_t* toggle = &data_->toggle_mask_[v * Words];
        for (std::size_t w = 0; w < Words; ++w) {
            if (toggle[w] != 0) {
                uint64_t& s = word_ref(state, w);
                const uint64_t overflow = s & toggle[w];
                s ^= toggle[w];
                if (overflow != 0) {
                    cost += beta_sum(w, overflow);
                }
            }
        }
        for (uint32_t i = data_->inc_begin_[v]; i < data_->inc_begin_[v + 1]; ++i) {
            const auto& inc = data_->increments_[i];
            uint64_t& s = word_ref(state, static_cast<std::size_t>(inc.word));
            const uint64_t sum = ((s >> inc.shift) & inc.mask) + inc.numerator;
            const uint64_t laps = sum / inc.denominator;
            if (laps != 0) {
                cost += static_cast<double>(laps) * inc.beta;
            }
            s = (s & ~(inc.mask << inc.shift)) | ((sum % inc.denominator) << inc.shift);
        }
        return {state, cost};
    }

    /// Extra cost L1 must overcome to dominate L2 — a non-negative PENALTY.
    /// For each cut where the dominator L1's sum exceeds the dominated L2's,
    /// some completion lifts L1 to d one visit before L2 (both sums lie in
    /// [0, d), so never more than once more), so L1 pays at most one extra
    /// +β: c(L1) + Σ_{s1 > s2} β ≤ c(L2) dominates for every completion.
    [[nodiscard]] double domination_cost(Direction /*dir*/, int /*vertex*/, State s1,
                                         State s2) const {
        if (data_->n_active_ == 0) {
            return 0.0;
        }
        double cost = 0.0;
        for (std::size_t w = 0; w < Words; ++w) {
            const uint64_t a = word(s1, w);
            const uint64_t b = word(s2, w);
            uint64_t worse = a & ~b & data_->one_mask_[w];
            if (data_->low2_mask_[w] != 0) {
                worse |= exceeds2(a, b, data_->low2_mask_[w]);
            }
            if (data_->low3_mask_[w] != 0) {
                worse |= exceeds3(a, b, data_->low3_mask_[w]);
            }
            if (worse != 0) {
                cost += beta_sum(w, worse);
            }
        }
        return cost;
    }

    /// Lower bound on domination_cost: 0. The dominance penalty is a sum of +β
    /// (β ≥ 0), so it is never negative.
    [[nodiscard]] double min_domination_cost() const { return 0.0; }

    /// Cost when joining fw/bw labels: a cut whose two sums reach d together
    /// gains one more coefficient on the joined path (both lie in [0, d), so
    /// at most one) and is charged +β. On 1-bit fields: both bits set.
    [[nodiscard]] double concatenation_cost(Symmetry /*sym*/, int /*vertex*/, State s_fw,
                                            State s_bw) const {
        if (data_->n_active_ == 0) {
            return 0.0;
        }
        double cost = 0.0;
        for (std::size_t w = 0; w < Words; ++w) {
            const uint64_t f = word(s_fw, w);
            const uint64_t b = word(s_bw, w);
            const uint64_t both = f & b & data_->one_mask_[w];
            if (both != 0) {
                cost += beta_sum(w, both);
            }
            const uint64_t low2 = data_->low2_mask_[w];
            const uint64_t low3 = data_->low3_mask_[w];
            if ((low2 | low3) == 0) {
                continue;
            }
            // Fields nonzero on both sides; only they can reach d together.
            uint64_t cand =
                (((f | (f >> 1)) & low2) & ((b | (b >> 1)) & low2)) |
                (((f | (f >> 1) | (f >> 2)) & low3) & ((b | (b >> 1) | (b >> 2)) & low3));
            while (cand != 0) {
                const int bit = std::countr_zero(cand);
                const uint64_t mask = ((low2 >> bit) & 1ULL) != 0 ? 0x3ULL : 0x7ULL;
                const uint64_t sum = ((f >> bit) & mask) + ((b >> bit) & mask);
                const std::size_t at = (w * Data::kBits) + static_cast<std::size_t>(bit);
                if (sum >= data_->den_at_[at]) {
                    cost += data_->beta_at_[at];
                }
                cand &= cand - 1;
            }
        }
        return cost;
    }

private:
    [[nodiscard]] static uint64_t& word_ref(State& s, std::size_t i) {
        if constexpr (Words == 1) {
            static_cast<void>(i);
            return s;
        } else {
            return s[i];
        }
    }

    /// Sum of +β over the fields whose lowest bits are set in `mask` (word w)
    /// — the reduced-cost penalty for the coefficients they stand for.
    [[nodiscard]] double beta_sum(std::size_t w, uint64_t mask) const {
        double cost = 0.0;
        while (mask != 0) {
            const int bit = std::countr_zero(mask);
            cost += data_->beta_at_[(w * Data::kBits) + static_cast<std::size_t>(bit)];
            mask &= mask - 1;
        }
        return cost;
    }

    const Data* data_ = nullptr;  // not owned
};

/// Owning R1C resource: `BasicR1CData` plus the resource methods, forwarded
/// to a view over itself.
template <std::size_t Words>
struct BasicR1CResource : BasicR1CData<Words> {
    static constexpr bool is_r1c_resource = true;
    using View = BasicR1CView<Words>;
    using State = View::State;

    /// The borrowing view. Deleted on an rvalue, so a view of a temporary is a
    /// compile error rather than a dangle.
    [[nodiscard]] View view() const& noexcept { return View(*this); }
    [[nodiscard]] View view() const&& = delete;

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

/// One state word: 64 one-bit cuts (3-SRCs), 32 two-bit or 21 three-bit ones.
using R1CData = BasicR1CData<1>;
using R1CView = BasicR1CView<1>;
using R1CResource = BasicR1CResource<1>;

/// Backward-compatible alias.
using R1CManager = R1CResource;

}  // namespace bgspprc
