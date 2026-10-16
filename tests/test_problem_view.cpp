#include <algorithm>
#include <bgspprc/problem_view.h>
#include <doctest/doctest.h>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using bgspprc::AdjacencyView;
using bgspprc::CsrAdjacency;
using bgspprc::ProblemView;

struct Diamond {
    // 0 -> 1 -> 3, 0 -> 2 -> 3
    std::vector<int> from{0, 0, 1, 2};
    std::vector<int> to{1, 2, 3, 3};
    std::vector<double> cost{1.0, 2.0, 3.0, 4.0};

    [[nodiscard]] ProblemView view() const {
        ProblemView pv;
        pv.n_vertices = 4;
        pv.source = 0;
        pv.sink = 3;
        pv.n_arcs = static_cast<int>(from.size());
        pv.arc_from = from.data();
        pv.arc_to = to.data();
        pv.arc_base_cost = cost.data();
        return pv;
    }
};

TEST_CASE("ProblemView: defaults match bgspprc") {
    ProblemView pv;
    CHECK(pv.n_vertices == 0);
    CHECK(pv.n_arcs == 0);
    CHECK(pv.n_resources == 0);
    CHECK(pv.n_main_resources == 1);
    CHECK(pv.resource_nondisposable == nullptr);
}

TEST_CASE("ProblemView: is non owning so core can rewrite costs") {
    // The core rewrites reduced costs in place every CG iteration; the view
    // must see the new values without a rebuild.
    Diamond d;
    auto pv = d.view();
    CHECK(pv.arc_base_cost[0] == doctest::Approx(1.0));
    d.cost[0] = -7.5;
    CHECK(pv.arc_base_cost[0] == doctest::Approx(-7.5));
}

TEST_CASE("CsrAdjacency: outgoing and incoming are complete") {
    Diamond d;
    auto pv = d.view();
    CsrAdjacency adj;
    adj.build(pv);

    CHECK(adj.n_vertices() == 4);
    CHECK(adj.n_arcs() == 4);

    auto out0 = adj.outgoing(0);
    REQUIRE(out0.size() == 2u);
    std::vector<int> t0{pv.arc_to[out0[0]], pv.arc_to[out0[1]]};
    std::ranges::sort(t0);
    CHECK(t0 == (std::vector<int>{1, 2}));

    auto in3 = adj.incoming(3);
    REQUIRE(in3.size() == 2u);
    std::vector<int> s3{pv.arc_from[in3[0]], pv.arc_from[in3[1]]};
    std::ranges::sort(s3);
    CHECK(s3 == (std::vector<int>{1, 2}));

    CHECK(adj.incoming(0).empty());
    CHECK(adj.outgoing(3).empty());
}

TEST_CASE("CsrAdjacency: every arc appears exactly once each way") {
    Diamond d;
    auto pv = d.view();
    CsrAdjacency adj;
    adj.build(pv);

    std::vector<int> out_seen(static_cast<std::size_t>(pv.n_arcs), 0);
    std::vector<int> in_seen(static_cast<std::size_t>(pv.n_arcs), 0);
    for (int v = 0; v < pv.n_vertices; ++v) {
        for (int a : adj.outgoing(v)) {
            ++out_seen[static_cast<std::size_t>(a)];
            CHECK(pv.arc_from[a] == v);
        }
        for (int a : adj.incoming(v)) {
            ++in_seen[static_cast<std::size_t>(a)];
            CHECK(pv.arc_to[a] == v);
        }
    }
    for (int a = 0; a < pv.n_arcs; ++a) {
        CHECK(out_seen[static_cast<std::size_t>(a)] == 1);
        CHECK(in_seen[static_cast<std::size_t>(a)] == 1);
    }
}

TEST_CASE("CsrAdjacency: arc ids follow insertion order") {
    // Caller-side arc data is indexed by arc id and must stay aligned.
    Diamond d;
    auto pv = d.view();
    CsrAdjacency adj;
    adj.build(pv);
    auto out0 = adj.outgoing(0);
    REQUIRE(out0.size() == 2u);
    CHECK(out0[0] == 0);
    CHECK(out0[1] == 1);
    CHECK(pv.arc_base_cost[out0[0]] == doctest::Approx(1.0));
    CHECK(pv.arc_base_cost[out0[1]] == doctest::Approx(2.0));
}

TEST_CASE("CsrAdjacency: isolated vertices and rebuild") {
    std::vector<int> from{0};
    std::vector<int> to{1};
    ProblemView pv;
    pv.n_vertices = 5;
    pv.n_arcs = 1;
    pv.arc_from = from.data();
    pv.arc_to = to.data();

    CsrAdjacency adj;
    adj.build(pv);
    adj.build(pv);  // rebuild must be idempotent

    CHECK(adj.n_vertices() == 5);
    for (int v = 2; v < 5; ++v) {
        CHECK(adj.outgoing(v).empty());
        CHECK(adj.incoming(v).empty());
    }
}

TEST_CASE("CsrAdjacency: empty graph") {
    ProblemView pv;
    pv.n_vertices = 3;
    pv.n_arcs = 0;
    CsrAdjacency adj;
    adj.build(pv);
    CHECK(adj.n_arcs() == 0);
    for (int v = 0; v < 3; ++v) {
        CHECK(adj.outgoing(v).empty());
    }
}

// A view must not bind to a temporary owner: that would dangle.
static_assert(!std::is_convertible_v<CsrAdjacency&&, AdjacencyView>);
template <typename A>
concept HasView = requires(A&& a) { std::forward<A>(a).view(); };
static_assert(!HasView<CsrAdjacency>);
static_assert(HasView<const CsrAdjacency&>);
static_assert(std::is_convertible_v<const CsrAdjacency&, AdjacencyView>);
static_assert(std::is_trivially_copyable_v<AdjacencyView>);
static_assert(sizeof(AdjacencyView) <= 40);
static_assert(
    std::is_same_v<decltype(std::declval<AdjacencyView>().outgoing(0)), std::span<const int>>);

TEST_CASE("AdjacencyView: matches its owner") {
    Diamond d;
    auto pv = d.view();
    CsrAdjacency adj;
    adj.build(pv);
    const AdjacencyView view = adj;

    CHECK(view.n_vertices == adj.n_vertices());
    CHECK(view.n_arcs == adj.n_arcs());
    for (int v = 0; v < pv.n_vertices; ++v) {
        auto ov = view.outgoing(v);
        auto oa = adj.outgoing(v);
        CHECK(ov.data() == oa.data());  // borrowed, not copied
        CHECK(std::ranges::equal(ov, oa));
        CHECK(std::ranges::equal(view.incoming(v), adj.incoming(v)));
    }
}

TEST_CASE("AdjacencyView: borrows caller owned csr") {
    // Core-fed: the view points at arrays someone else built; no CsrAdjacency.
    // 0 -> 1 (arc 0), 0 -> 2 (arc 1), 1 -> 2 (arc 2)
    const int out_begin[] = {0, 2, 3, 3};
    const int out_arcs[] = {0, 1, 2};
    const int in_begin[] = {0, 0, 1, 3};
    const int in_arcs[] = {0, 1, 2};
    const AdjacencyView view{.out_begin = out_begin,
                             .out_arcs = out_arcs,
                             .in_begin = in_begin,
                             .in_arcs = in_arcs,
                             .n_vertices = 3,
                             .n_arcs = 3};

    CHECK(std::ranges::equal(view.outgoing(0), std::vector<int>{0, 1}));
    CHECK(std::ranges::equal(view.outgoing(1), std::vector<int>{2}));
    CHECK(view.outgoing(2).empty());
    CHECK(view.incoming(0).empty());
    CHECK(std::ranges::equal(view.incoming(2), std::vector<int>{1, 2}));
}

TEST_CASE("AdjacencyView: of empty graph") {
    ProblemView pv;
    pv.n_vertices = 2;
    CsrAdjacency adj;
    adj.build(pv);
    const AdjacencyView view = adj.view();
    CHECK(view.n_arcs == 0);
    CHECK(view.outgoing(0).empty());
    CHECK(view.incoming(1).empty());
}

}  // namespace
