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
/// straddles two words. On the global layout (`set_cuts(cuts, n_vertices,
/// n_arcs)`) every cut has its field at every vertex. On the per-vertex
/// layout (the `ProblemView` overloads) a cut has a field only at the
/// vertices where its sum can be nonzero, and each vertex packs its own cuts
/// from bit 0, so a state is read in the layout of the vertex it is at; an
/// arc moves each kept field from its place at one end to its place at the
/// other. A cut that does not fit makes `set_cuts` throw, as does a malformed
/// one; on a throw the previous cuts stay.
template <std::size_t Words>
class BasicR1CData {
    static_assert(Words >= 1, "at least one state word");
    static_assert(Words <= 256, "a Move stores word indices in a byte");

public:
    static constexpr std::size_t kWords = Words;

    /// Where a cut's field lives in the state at a vertex. `word` is -1 where
    /// the cut has no slot.
    struct Field {
        int word = 0;   // [0, Words), or -1: no slot
        int shift = 0;  // lowest bit of the field in its word
        int width = 0;  // bits: 1, 2 or 3
        int denominator = 2;
    };

    /// Set the active cuts (arc memory) on the global layout: every cut has
    /// its field at every vertex, so they share `64·Words` bits. Rebuilds the
    /// layout and all lookup tables. Throws `R1CError` (InvalidCut) if a cut
    /// is malformed (sizes differ, a multiplier outside (0, 1) or not k/d for
    /// some d ≤ 8, a vertex or arc out of range, a NaN dual), or
    /// (TooManyCounters) if the cuts need more than `64·Words` bits. The
    /// executor is used to parallelize the arc-memory masks over cuts.
    template <Executor Exec = SequentialExecutor>
    void set_cuts(std::span<const R1Cut> cuts, int n_vertices, int n_arcs, Exec&& exec = {}) {
        BasicR1CData next;
        next.n_vertices_ = n_vertices;
        next.n_arcs_ = n_arcs;
        std::vector<NormCut> norm(cuts.size());
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            norm[c] = normalize(cuts[c], static_cast<int>(c), n_vertices, n_arcs);
        }
        next.build_global(norm, std::forward<Exec>(exec));
        *this = std::move(next);  // in place: borrowing views stay valid
    }

    /// Set the active cuts (arc memory) on the per-vertex layout: a cut has a
    /// field only at its base-set vertices and the ends of its memory arcs,
    /// packed per vertex in cut order, so `64·Words` bits bound the cuts
    /// sharing a vertex, not all of them. The view gives the arc endpoints and
    /// must be the one the resource prices. Throws as the global overload,
    /// TooManyCounters naming the cut and the vertex where it found no room.
    template <Executor Exec = SequentialExecutor>
    void set_cuts(std::span<const R1Cut> cuts, const ProblemView& pv, Exec&& exec = {}) {
        BasicR1CData next;
        next.n_vertices_ = pv.n_vertices;
        next.n_arcs_ = pv.n_arcs;
        std::vector<NormCut> norm(cuts.size());
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            norm[c] = normalize(cuts[c], static_cast<int>(c), pv.n_vertices, pv.n_arcs);
            add_arc_ends(norm[c], pv);
        }
        next.build_per_vertex(norm, pv, /*vertex_memory=*/false, std::forward<Exec>(exec));
        *this = std::move(next);
    }

    /// Set the active cuts (vertex memory, pull-rcspp's form) on the
    /// per-vertex layout: a cut has a field at the vertices of M' (its memory
    /// and coefficient vertices). The view gives the arc endpoints the memory
    /// is read on, and must be the one the resource prices. Throws as the
    /// `R1Cut` overloads, InvalidCut also for a denominator outside [2, 8], a
    /// numerator outside [1, d - 1] or a memory vertex out of range.
    template <Executor Exec = SequentialExecutor>
    void set_cuts(std::span<const R1VertexCut> cuts, const ProblemView& pv, Exec&& exec = {}) {
        BasicR1CData next;
        next.n_vertices_ = pv.n_vertices;
        next.n_arcs_ = pv.n_arcs;
        std::vector<NormCut> norm(cuts.size());
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            norm[c] = normalize(cuts[c], static_cast<int>(c), pv.n_vertices);
        }
        next.build_per_vertex(norm, pv, /*vertex_memory=*/true, std::forward<Exec>(exec));
        *this = std::move(next);
    }

    [[nodiscard]] int n_active() const { return n_active_; }

    /// True for the per-vertex layout (the `ProblemView` overloads).
    [[nodiscard]] bool per_vertex() const { return per_vertex_; }

    /// Cut c's field on the global layout. On the per-vertex layout it has
    /// `word == -1` and only the width and denominator: use `field(cut, v)`.
    [[nodiscard]] Field field(int cut) const { return fields_[static_cast<std::size_t>(cut)]; }

    /// Cut c's field at vertex v (`word == -1` where it has no slot).
    [[nodiscard]] Field field(int cut, int vertex) const {
        if (!per_vertex()) {
            return field(cut);
        }
        const auto v = static_cast<std::size_t>(vertex);
        const auto first = vslots_.begin() + vslot_begin_[v];
        const auto last = vslots_.begin() + vslot_begin_[v + 1];
        const auto it = std::ranges::lower_bound(first, last, cut, {}, &Slot::cut);
        Field f = fields_[static_cast<std::size_t>(cut)];
        if (it != last && it->cut == cut) {
            f.word = it->word;
            f.shift = it->shift;
        }
        return f;
    }

    /// Cut c's β = max(0, -σ).
    [[nodiscard]] double beta(int cut) const { return betas_[static_cast<std::size_t>(cut)]; }

private:
    friend class BasicR1CView<Words>;

    static constexpr int kBits = 64;

    // The one form all cut types are built from: merged numerators, the
    // denominator, β, the memory arcs (arc form) or vertex memory, and M',
    // the vertices a per-vertex layout gives the cut a slot at.
    struct NormCut {
        std::vector<std::pair<int, int>> numerators;  // (vertex, k_v), sorted, merged
        int denominator = 2;
        double beta = 0.0;
        std::vector<int> memory_arcs;    // arc memory: the arcs that keep the sum
        std::vector<int> slot_vertices;  // M', sorted, unique (per-vertex layouts)
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

    // A cut's slot at a vertex (per-vertex layout), in cut order.
    struct Slot {
        int cut = 0;
        int word = 0;
        int shift = 0;
    };

    // A kept field that sits at different places at an arc's two ends.
    struct Move {
        uint8_t tail_word = 0;
        uint8_t tail_shift = 0;
        uint8_t head_word = 0;
        uint8_t head_shift = 0;
        uint64_t mask = 0;  // (1 << width) - 1, unshifted
    };

    // One table row: the global layout's only row, or a vertex's on the
    // per-vertex layout, which lays out that vertex's fields.
    struct Row {
        std::array<uint64_t, Words> one{};         // bits of the 1-bit fields
        std::array<uint64_t, Words> low2{};        // lowest bit of each 2-bit field
        std::array<uint64_t, Words> low3{};        // lowest bit of each 3-bit field
        std::array<double, Words * kBits> beta{};  // [word * 64 + bit] β of the field
        std::array<uint8_t, Words * kBits> den{};  // d of the field whose lowest bit this is
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

    static void sort_unique(std::vector<int>& v) {
        std::ranges::sort(v);
        const auto [first, last] = std::ranges::unique(v);
        v.erase(first, last);
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
        sort_unique(out.memory_arcs);  // a duplicate would only repeat a move
        return out;
    }

    // M' of an arc-form cut: its base set and the ends of its memory arcs.
    static void add_arc_ends(NormCut& cut, const ProblemView& pv) {
        cut.slot_vertices.clear();
        for (const auto& [v, k] : cut.numerators) {
            cut.slot_vertices.push_back(v);
        }
        for (int a : cut.memory_arcs) {
            cut.slot_vertices.push_back(pv.arc_from[a]);
            cut.slot_vertices.push_back(pv.arc_to[a]);
        }
        sort_unique(cut.slot_vertices);
    }

    static NormCut normalize(const R1VertexCut& cut, int c, int n_vertices) {
        const int d = cut.denominator;
        if (d < 2 || d > kR1CMaxDenominator) {
            throw R1CError(R1CErrc::InvalidCut, "R1CResource: a denominator must lie in [2, 8]", c);
        }
        NormCut out;
        out.beta = beta_of(cut.dual, c);
        out.denominator = d;
        auto in_range = [&](int v) {
            if (v < 0 || v >= n_vertices) {
                throw R1CError(R1CErrc::InvalidCut, "R1CResource: a cut's vertex is out of range",
                               c, v);
            }
            return v;
        };
        out.numerators.reserve(cut.coefficients.size());
        out.slot_vertices.reserve(cut.coefficients.size() + cut.memory.size());
        for (const auto& [v, k] : cut.coefficients) {
            out.slot_vertices.push_back(in_range(v));
            if (k < 1 || k >= d) {
                throw R1CError(R1CErrc::InvalidCut,
                               "R1CResource: a numerator must lie in [1, denominator - 1]", c, v);
            }
            out.numerators.emplace_back(v, k);
        }
        merge_numerators(out.numerators);
        for (int v : cut.memory) {
            out.slot_vertices.push_back(in_range(v));
        }
        sort_unique(out.slot_vertices);
        return out;
    }

    // Cut-level tables shared by both layouts.
    void init_cuts(std::span<const NormCut> cuts, std::size_t vertex_rows) {
        n_active_ = static_cast<int>(cuts.size());
        fields_.resize(cuts.size());
        betas_.resize(cuts.size());
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            const int d = cuts[c].denominator;
            fields_[c] = {.word = -1, .shift = 0, .width = field_width(d), .denominator = d};
            betas_[c] = cuts[c].beta;
        }
        global_row_ = Row{};
        vertex_rows_.assign(vertex_rows, Row{});
    }

    // Record cut c's field at (word, shift) in table row `row`.
    void place(Row& row, std::size_t c, int word, int shift) {
        const Field f = fields_[c];
        const auto w = static_cast<std::size_t>(word);
        const uint64_t low = 1ULL << shift;
        if (f.width == 1) {
            row.one[w] |= low;
        } else if (f.width == 2) {
            row.low2[w] |= low;
        } else {
            row.low3[w] |= low;
        }
        const std::size_t at = (w * kBits) + static_cast<std::size_t>(shift);
        row.beta[at] = betas_[c];
        row.den[at] = static_cast<uint8_t>(f.denominator);
    }

    // The table row of vertex v: the inline global row, read at a fixed
    // offset as before #146, or v's own on the per-vertex layout.
    [[nodiscard]] const Row& row(int v) const {
        return !per_vertex_ ? global_row_ : vertex_rows_[static_cast<std::size_t>(v)];
    }

    // Advance a (word, shift) cursor past a field of `width` bits; false when
    // the words are full. A field never straddles two words.
    static bool next_slot(int& word, int& shift, int width, int& at_word, int& at_shift) {
        if (shift + width > kBits) {
            ++word;
            shift = 0;
        }
        if (std::cmp_greater_equal(word, Words)) {
            return false;
        }
        at_word = word;
        at_shift = shift;
        shift += width;
        return true;
    }

    template <Executor Exec>
    void build_global(std::span<const NormCut> cuts, Exec&& exec) {
        per_vertex_ = false;
        init_cuts(cuts, 0);
        int word = 0;
        int shift = 0;
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            Field& f = fields_[c];
            if (!next_slot(word, shift, f.width, f.word, f.shift)) {
                throw R1CError(
                    R1CErrc::TooManyCounters,
                    "R1CResource: the cuts need more state bits than the resource's words hold",
                    static_cast<int>(c));
            }
            place(global_row_, c, f.word, f.shift);
        }
        build_vertex_tables(cuts, [&](std::size_t c, int /*v*/) {
            const Field f = fields_[c];
            return std::pair{f.word, f.shift};
        });

        // Each cut owns its field's bits, so concurrent |= targets disjoint
        // bits; std::atomic_ref keeps the read-modify-write race-free.
        keep_.assign(static_cast<std::size_t>(n_arcs_), {});
        move_begin_.clear();
        moves_.clear();
        exec.parallel_for(0, n_active_, [&](int c) {
            const Field f = fields_[static_cast<std::size_t>(c)];
            const uint64_t bits = ((1ULL << f.width) - 1) << f.shift;
            for (int a : cuts[static_cast<std::size_t>(c)].memory_arcs) {
                auto& m = keep_[static_cast<std::size_t>(a)][static_cast<std::size_t>(f.word)];
                std::atomic_ref(m).fetch_or(bits, std::memory_order_relaxed);
            }
        });
    }

    template <Executor Exec>
    void build_per_vertex(std::span<const NormCut> cuts, const ProblemView& pv, bool vertex_memory,
                          Exec&& exec) {
        per_vertex_ = true;
        const auto n_v = static_cast<std::size_t>(n_vertices_);
        init_cuts(cuts, n_v);

        // Slots: each vertex packs the cuts of whose M' it is in, in cut order.
        std::vector<int> word(n_v, 0);
        std::vector<int> shift(n_v, 0);
        std::vector<std::vector<Slot>> slots(n_v);
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            const int width = fields_[c].width;
            for (int v : cuts[c].slot_vertices) {
                const auto vi = static_cast<std::size_t>(v);
                Slot s{.cut = static_cast<int>(c)};
                if (!next_slot(word[vi], shift[vi], width, s.word, s.shift)) {
                    throw R1CError(R1CErrc::TooManyCounters,
                                   "R1CResource: the cuts at a vertex need more state bits than "
                                   "the resource's words hold",
                                   static_cast<int>(c), v);
                }
                slots[vi].push_back(s);
                place(vertex_rows_[vi], c, s.word, s.shift);
            }
        }
        vslot_begin_.assign(1, 0);
        vslots_.clear();
        for (const auto& row : slots) {
            vslots_.insert(vslots_.end(), row.begin(), row.end());
            vslot_begin_.push_back(static_cast<uint32_t>(vslots_.size()));
        }
        build_vertex_tables(cuts, [&](std::size_t c, int v) {
            const Field f = field(static_cast<int>(c), v);
            return std::pair{f.word, f.shift};
        });
        build_arc_tables(cuts, pv, vertex_memory, std::forward<Exec>(exec));
    }

    // Per vertex: the toggle mask of the 1-bit fast path (d = 2, numerator 1)
    // and the increments of every other (cut, vertex) pair, at `slot_of(c, v)`.
    template <typename SlotOf>
    void build_vertex_tables(std::span<const NormCut> cuts, SlotOf&& slot_of) {
        const auto n_v = static_cast<std::size_t>(n_vertices_);
        toggle_mask_.assign(n_v * Words, 0ULL);
        std::vector<std::vector<Increment>> incs(n_v);
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            const Field f = fields_[c];
            for (const auto& [v, k] : cuts[c].numerators) {
                const auto vi = static_cast<std::size_t>(v);
                const auto [word, shift] = slot_of(c, v);
                if (f.width == 1 && k == 1) {
                    toggle_mask_[(vi * Words) + static_cast<std::size_t>(word)] |= 1ULL << shift;
                } else {
                    incs[vi].push_back({.word = word,
                                        .shift = shift,
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

    // The slots of vertex v, in cut order.
    [[nodiscard]] std::span<const Slot> slots_at(int v) const {
        const auto vi = static_cast<std::size_t>(v);
        return {vslots_.data() + vslot_begin_[vi], vslot_begin_[vi + 1] - vslot_begin_[vi]};
    }

    // Per arc (i, j), the cuts that keep their sum on it: a kept field at the
    // same place at i and j goes in the keep mask, any other in a move. An
    // arc-form cut keeps on its memory arcs; a vertex-form one on the arcs
    // with both ends in M', i.e. with a slot at both. Two passes over the
    // arcs on the executor (count, then fill); each arc writes its own words.
    template <Executor Exec>
    void build_arc_tables(std::span<const NormCut> cuts, const ProblemView& pv, bool vertex_memory,
                          Exec&& exec) {
        const auto n_a = static_cast<std::size_t>(n_arcs_);
        // Arc-form memories, bucketed by arc in cut order.
        std::vector<uint32_t> mem_begin(n_a + 1, 0);
        std::vector<int> mem_cuts;
        for (const NormCut& cut : cuts) {
            for (int a : cut.memory_arcs) {
                ++mem_begin[static_cast<std::size_t>(a) + 1];
            }
        }
        for (std::size_t a = 0; a < n_a; ++a) {
            mem_begin[a + 1] += mem_begin[a];
        }
        mem_cuts.resize(mem_begin[n_a]);
        {
            std::vector<uint32_t> fill(mem_begin.begin(), mem_begin.end() - 1);
            for (std::size_t c = 0; c < cuts.size(); ++c) {
                for (int a : cuts[c].memory_arcs) {
                    mem_cuts[fill[static_cast<std::size_t>(a)]++] = static_cast<int>(c);
                }
            }
        }

        // Visit each kept cut of arc a with its slots at the two ends.
        auto for_each_kept = [&](int a, auto&& visit) {
            const auto ia = static_cast<std::size_t>(a);
            const std::span<const Slot> at_i = slots_at(pv.arc_from[a]);
            const std::span<const Slot> at_j = slots_at(pv.arc_to[a]);
            auto find = [](std::span<const Slot> s, int c) -> const Slot& {
                return *std::ranges::lower_bound(s, c, {}, &Slot::cut);
            };
            for (uint32_t m = mem_begin[ia]; m < mem_begin[ia + 1]; ++m) {
                const int c = mem_cuts[m];
                visit(c, find(at_i, c), find(at_j, c));
            }
            // Vertex-form cuts: the cuts with a slot at both ends.
            std::size_t p = 0;
            std::size_t q = 0;
            while (vertex_memory && p < at_i.size() && q < at_j.size()) {
                if (at_i[p].cut < at_j[q].cut) {
                    ++p;
                } else if (at_j[q].cut < at_i[p].cut) {
                    ++q;
                } else {
                    visit(at_i[p].cut, at_i[p], at_j[q]);
                    ++p;
                    ++q;
                }
            }
        };

        keep_.assign(n_a, {});
        move_begin_.assign(n_a + 1, 0);
        exec.parallel_for(0, n_arcs_, [&](int a) {
            uint32_t count = 0;
            auto& keep = keep_[static_cast<std::size_t>(a)];
            for_each_kept(a, [&](int c, const Slot& si, const Slot& sj) {
                if (si.word == sj.word && si.shift == sj.shift) {
                    const uint64_t mask = (1ULL << fields_[static_cast<std::size_t>(c)].width) - 1;
                    keep[si.word] |= mask << si.shift;
                } else {
                    ++count;
                }
            });
            move_begin_[static_cast<std::size_t>(a) + 1] = count;  // a count until the scan
        });
        for (std::size_t a = 0; a < n_a; ++a) {
            move_begin_[a + 1] += move_begin_[a];
        }
        moves_.assign(move_begin_[n_a], Move{});
        exec.parallel_for(0, n_arcs_, [&](int a) {
            uint32_t out = move_begin_[static_cast<std::size_t>(a)];
            for_each_kept(a, [&](int c, const Slot& si, const Slot& sj) {
                if (si.word != sj.word || si.shift != sj.shift) {
                    moves_[out++] = {
                        .tail_word = static_cast<uint8_t>(si.word),
                        .tail_shift = static_cast<uint8_t>(si.shift),
                        .head_word = static_cast<uint8_t>(sj.word),
                        .head_shift = static_cast<uint8_t>(sj.shift),
                        .mask = (1ULL << fields_[static_cast<std::size_t>(c)].width) - 1};
                }
            });
        });
    }

    int n_vertices_ = 0;
    int n_arcs_ = 0;
    int n_active_ = 0;
    bool per_vertex_ = false;  // a table row per vertex, or the inline global one

    std::vector<Field> fields_;                      // [cut]; word -1 on the per-vertex layout
    std::vector<double> betas_;                      // [cut]
    Row global_row_{};                               // the global layout's tables, inline
    std::vector<Row> vertex_rows_;                   // [vertex], per-vertex layout only
    std::vector<std::array<uint64_t, Words>> keep_;  // [arc] the fields kept in place
    std::vector<uint32_t> move_begin_;   // [arc], n_arcs + 1 entries, per-vertex layout only
    std::vector<Move> moves_;            // an arc's fields kept at another place
    std::vector<uint64_t> toggle_mask_;  // [vertex * Words + word]
    std::vector<uint32_t> inc_begin_;    // [vertex], n_vertices + 1 entries
    std::vector<Increment> increments_;  // a vertex's increments
    std::vector<uint32_t> vslot_begin_;  // [vertex], per-vertex layout only
    std::vector<Slot> vslots_;           // a vertex's slots, in cut order
};

/// lm-R1C resource conforming to the Resource concept, borrowing caller-owned
/// `BasicR1CData` (flowty/flowty#58), 8 bytes. The data must outlive the view.
/// It carries the `is_r1c_resource` tag, by which `detail::R1CResourceIndex`
/// finds it in a pack.
///
/// State: per cut, its running sum mod d in its field (see `BasicR1CData`).
/// extend_along_arc: memory reset (keep the fields of cuts that remember the
///   arc: arc ∈ AM, or both its ends in M' for vertex memory).
/// extend_to_vertex: add the vertex's numerators; a sum reaching d pays +β
///   per multiple of d and keeps the remainder. On 1-bit cuts with numerator
///   1 (the 3-SRC) that is a toggle, and runs word-parallel.
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

    /// Memory reset along arc: keep the fields of cuts that remember the arc
    /// (arc ∈ AM, or both ends in M').
    [[nodiscard]] std::pair<State, double> extend_along_arc(Direction dir, State state,
                                                            int arc_id) const {
        if (data_->n_active_ == 0) {
            return {state, 0.0};
        }
        const auto a = static_cast<std::size_t>(arc_id);
        const auto& keep = data_->keep_[a];
        const uint32_t move_begin = data_->per_vertex_ ? data_->move_begin_[a] : 0;
        const uint32_t move_end = data_->per_vertex_ ? data_->move_begin_[a + 1] : 0;
        if (move_begin == move_end) {
            for (std::size_t w = 0; w < Words; ++w) {
                word_ref(state, w) &= keep[w];
            }
            return {state, 0.0};
        }
        const State in = state;
        for (std::size_t w = 0; w < Words; ++w) {
            word_ref(state, w) &= keep[w];
        }
        // Fields kept at another place at the target end (per-vertex layout).
        const bool fw = dir == Direction::Forward;
        for (uint32_t m = move_begin; m < move_end; ++m) {
            const auto& mv = data_->moves_[m];
            const auto [from_w, from_s, to_w, to_s] =
                fw ? std::array{mv.tail_word, mv.tail_shift, mv.head_word, mv.head_shift}
                   : std::array{mv.head_word, mv.head_shift, mv.tail_word, mv.tail_shift};
            const uint64_t field = (word(in, from_w) >> from_s) & mv.mask;
            word_ref(state, to_w) |= field << to_s;
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
                    cost += beta_sum(data_->row(vertex), w, overflow);
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
    [[nodiscard]] double domination_cost(Direction /*dir*/, int vertex, State s1, State s2) const {
        if (data_->n_active_ == 0) {
            return 0.0;
        }
        const auto& t = data_->row(vertex);
        double cost = 0.0;
        for (std::size_t w = 0; w < Words; ++w) {
            const uint64_t a = word(s1, w);
            const uint64_t b = word(s2, w);
            uint64_t worse = a & ~b & t.one[w];
            if (t.low2[w] != 0) {
                worse |= exceeds2(a, b, t.low2[w]);
            }
            if (t.low3[w] != 0) {
                worse |= exceeds3(a, b, t.low3[w]);
            }
            if (worse != 0) {
                cost += beta_sum(t, w, worse);
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
    [[nodiscard]] double concatenation_cost(Symmetry /*sym*/, int vertex, State s_fw,
                                            State s_bw) const {
        if (data_->n_active_ == 0) {
            return 0.0;
        }
        const auto& t = data_->row(vertex);
        double cost = 0.0;
        for (std::size_t w = 0; w < Words; ++w) {
            const uint64_t f = word(s_fw, w);
            const uint64_t b = word(s_bw, w);
            const uint64_t both = f & b & t.one[w];
            if (both != 0) {
                cost += beta_sum(t, w, both);
            }
            const uint64_t low2 = t.low2[w];
            const uint64_t low3 = t.low3[w];
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
                if (sum >= t.den[at]) {
                    cost += t.beta[at];
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

    /// Sum of +β over the fields whose lowest bits are set in `mask` (word w
    /// of table row t) — the reduced-cost penalty for the coefficients they
    /// stand for.
    [[nodiscard]] static double beta_sum(const Data::Row& t, std::size_t w, uint64_t mask) {
        const double* beta_at = &t.beta[w * Data::kBits];
        double cost = 0.0;
        while (mask != 0) {
            const int bit = std::countr_zero(mask);
            cost += beta_at[bit];
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
