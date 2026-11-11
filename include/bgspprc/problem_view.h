#pragma once

// Problem-data shape contract. Normative text: the 2026-09-08 comment on
// flowty/pull-rcspp#23; decision flowty/flowty#58.
//
// This file is one copy of a shared header: flowty, bucket-graph-spprc
// (bgspprc), pull-rcspp, single-resource-spp, time-indexed-spp and spp-graph
// each carry it. `ProblemView` (originally bgspprc's struct), `AdjacencyView`
// and the owning CSR adjacency match in every copy, field for field and
// accessor for accessor, modulo namespace and, in bgspprc, the owning
// adjacency's name (below). The copies are duplicated rather than depended on:
// the BCP core fills one layout and hands it to every pricer in the family. DO
// NOT rename or reorder a field in one copy alone -- a change is made in all of
// them together.
//
// `ProblemView` is a *non-owning view*: every array stays owned by the core.
// That is what makes pass-by-reference work, and it matters more than it looks
// -- arc costs are reduced costs and change every CG iteration, so the core
// rewrites its own cost array in place and the pricers see it with no copy and
// no rebuild.
//
// The owning adjacency is CSR (four flat vectors: offsets and arcs per
// direction) rather than `vector<vector<int>>`: four allocations instead of
// 2*n_vertices, contiguous traversal in the hot loop. It is named `Adjacency`
// in every copy but bgspprc's, which names it `CsrAdjacency` and keeps its
// original `vector<vector<int>>` `Adjacency` beside it.
//
// `AdjacencyView` is the non-owning form of that CSR: topology only, fixed for
// the lifetime of a solve. Costs, resource consumptions and vertex bounds stay
// in `ProblemView`, which the core rewrites every CG iteration. Standalone, a
// pricer builds the owning adjacency and passes its view; core-fed, the view
// points at the core's CSR and nothing is built twice. There is deliberately no
// `Graph` concept: `outgoing`/`incoming` always return contiguous spans.

#include <cassert>
#include <cstdint>
#include <numeric>
#include <span>
#include <vector>

namespace bgspprc {

/// Non-owning view of the problem data. All arrays are caller-owned.
struct ProblemView {
    int n_vertices = 0;  // includes source(0) and sink(n_vertices-1)
    int source = 0;
    int sink = 0;

    int n_arcs = 0;

    // Arc topology: parallel arrays of size n_arcs.
    // Arc ids are the caller's insertion order, so caller-side arc data stays
    // index-aligned without a permutation table.
    const int* arc_from = nullptr;
    const int* arc_to = nullptr;
    const double* arc_base_cost = nullptr;

    // Per-arc resource consumptions: n_resources pointers, each to n_arcs
    int n_resources = 0;
    const double* const* arc_resource = nullptr;  // [resource][arc]

    // Per-vertex resource bounds: n_resources pointers, each to n_vertices
    const double* const* vertex_lb = nullptr;  // [resource][vertex]
    const double* const* vertex_ub = nullptr;  // [resource][vertex]

    // Resources [0, n_main_resources) define the bucket grid, which takes at
    // most two: a larger count is read as 2. BucketGraph never reads the
    // resources past its grid ones; to enforce them, put them in the pack
    // (SideResources, resources/side.h, enforces them from this view).
    // BucketGraph throws std::invalid_argument for a view without a main
    // resource, with n_resources below the grid's count, or with a null
    // arc_resource/vertex_lb/vertex_ub.
    int n_main_resources = 1;

    // Per-resource non-disposability: array of n_resources bools.
    // true = non-disposable (equality required in dominance).
    // nullptr means all resources are disposable (default).
    const bool* resource_nondisposable = nullptr;

    // A prebuilt adjacency is not part of the view: pass an AdjacencyView to
    // the BucketGraph or Solver constructor after the ProblemView.
};

/// Helper to build adjacency lists from arc arrays.
struct Adjacency {
    std::vector<std::vector<int>> outgoing;  // [vertex] -> arc indices
    std::vector<std::vector<int>> incoming;  // [vertex] -> arc indices

    void build(const ProblemView& pv) {
        outgoing.assign(pv.n_vertices, {});
        incoming.assign(pv.n_vertices, {});
        for (int a = 0; a < pv.n_arcs; ++a) {
            outgoing[pv.arc_from[a]].push_back(a);
            incoming[pv.arc_to[a]].push_back(a);
        }
    }
};

/// Non-owning CSR topology, ~40 bytes, passed by value. Arrays are owned by
/// whoever built them (the owning adjacency, or the core).
/// `out_begin`/`in_begin` have n_vertices+1 entries; `out_arcs`/`in_arcs` have
/// n_arcs.
struct AdjacencyView {
    const int* out_begin = nullptr;
    const int* out_arcs = nullptr;
    const int* in_begin = nullptr;
    const int* in_arcs = nullptr;
    int n_vertices = 0;
    int n_arcs = 0;

    [[nodiscard]] std::span<const int> outgoing(int v) const noexcept {
        assert(v >= 0 && v < n_vertices);
        const auto b = static_cast<std::size_t>(out_begin[v]);
        const auto e = static_cast<std::size_t>(out_begin[v + 1]);
        return {out_arcs + b, e - b};
    }

    [[nodiscard]] std::span<const int> incoming(int v) const noexcept {
        assert(v >= 0 && v < n_vertices);
        const auto b = static_cast<std::size_t>(in_begin[v]);
        const auto e = static_cast<std::size_t>(in_begin[v + 1]);
        return {in_arcs + b, e - b};
    }
};

/// CSR adjacency built from a ProblemView's arc arrays: the owning form of
/// `AdjacencyView`. Same accessors as the shared contract's owning adjacency
/// (flowty/flowty#58), renamed so bgspprc's `Adjacency` above stays as it is.
///
/// Invariant: every arc appears exactly once in outgoing(arc_from[a]) and
/// exactly once in incoming(arc_to[a]).
class CsrAdjacency {
public:
    CsrAdjacency() = default;

    void build(const ProblemView& pv) {
        const int n_v = pv.n_vertices;
        const int n_a = pv.n_arcs;

        out_begin_.assign(static_cast<std::size_t>(n_v) + 1, 0);
        in_begin_.assign(static_cast<std::size_t>(n_v) + 1, 0);
        out_arcs_.resize(static_cast<std::size_t>(n_a));
        in_arcs_.resize(static_cast<std::size_t>(n_a));

        std::vector<int> out_count(static_cast<std::size_t>(n_v), 0);
        std::vector<int> in_count(static_cast<std::size_t>(n_v), 0);
        for (int a = 0; a < n_a; ++a) {
            assert(pv.arc_from[a] >= 0 && pv.arc_from[a] < n_v);
            assert(pv.arc_to[a] >= 0 && pv.arc_to[a] < n_v);
            ++out_count[static_cast<std::size_t>(pv.arc_from[a])];
            ++in_count[static_cast<std::size_t>(pv.arc_to[a])];
        }
        std::exclusive_scan(out_count.begin(), out_count.end(), out_begin_.begin(), 0);
        std::exclusive_scan(in_count.begin(), in_count.end(), in_begin_.begin(), 0);
        out_begin_[static_cast<std::size_t>(n_v)] = n_a;
        in_begin_[static_cast<std::size_t>(n_v)] = n_a;

        auto out_fill = out_begin_;
        auto in_fill = in_begin_;
        for (int a = 0; a < n_a; ++a) {
            out_arcs_[static_cast<std::size_t>(
                out_fill[static_cast<std::size_t>(pv.arc_from[a])]++)] = a;
            in_arcs_[static_cast<std::size_t>(in_fill[static_cast<std::size_t>(pv.arc_to[a])]++)] =
                a;
        }
    }

    [[nodiscard]] std::span<const int> outgoing(int v) const noexcept {
        assert(v >= 0 && v + 1 < static_cast<int>(out_begin_.size()));
        const auto b = static_cast<std::size_t>(out_begin_[static_cast<std::size_t>(v)]);
        const auto e = static_cast<std::size_t>(out_begin_[static_cast<std::size_t>(v) + 1]);
        return {out_arcs_.data() + b, e - b};
    }

    [[nodiscard]] std::span<const int> incoming(int v) const noexcept {
        assert(v >= 0 && v + 1 < static_cast<int>(in_begin_.size()));
        const auto b = static_cast<std::size_t>(in_begin_[static_cast<std::size_t>(v)]);
        const auto e = static_cast<std::size_t>(in_begin_[static_cast<std::size_t>(v) + 1]);
        return {in_arcs_.data() + b, e - b};
    }

    [[nodiscard]] int n_vertices() const noexcept {
        return out_begin_.empty() ? 0 : static_cast<int>(out_begin_.size()) - 1;
    }
    [[nodiscard]] int n_arcs() const noexcept { return static_cast<int>(out_arcs_.size()); }

    /// The non-owning view. Deleted on an rvalue, so binding a view to a
    /// temporary `CsrAdjacency` is a compile error rather than a dangle.
    [[nodiscard]] AdjacencyView view() const& noexcept {
        return {.out_begin = out_begin_.data(),
                .out_arcs = out_arcs_.data(),
                .in_begin = in_begin_.data(),
                .in_arcs = in_arcs_.data(),
                .n_vertices = n_vertices(),
                .n_arcs = n_arcs()};
    }
    [[nodiscard]] AdjacencyView view() const&& = delete;

    // Implicit on purpose: an owner is usable wherever its view is expected.
    operator AdjacencyView() const& noexcept { return view(); }
    operator AdjacencyView() const&& = delete;

private:
    std::vector<int> out_begin_;  // size n_vertices+1
    std::vector<int> in_begin_;   // size n_vertices+1
    std::vector<int> out_arcs_;
    std::vector<int> in_arcs_;
};

}  // namespace bgspprc
