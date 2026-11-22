// Bucket grid sizing (#161): counts are checked in their wider types before
// any conversion to int or allocation. Also built with the float-cast and
// signed-overflow sanitizers (test_grid_ubsan), where the compiler has them.
#include <array>
#include <bgspprc/bucket_graph.h>
#include <doctest/doctest.h>
#include <limits>
#include <stdexcept>

using namespace bgspprc;

TEST_CASE("Grid axis count is checked in double before conversion (#161)") {
    constexpr int kMax = std::numeric_limits<int>::max();
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(detail::grid_axis_count(10.0, 3.0) == 4);
    CHECK(detail::grid_axis_count(0.0, 1.0) == 1);
    CHECK(detail::grid_axis_count(-5.0, 1.0) == 1);
    CHECK(detail::grid_axis_count(static_cast<double>(kMax), 1.0) == kMax);
    CHECK_FALSE(detail::grid_axis_count(1.0, inf));  // lb + 0 · inf is NaN
    CHECK_FALSE(detail::grid_axis_count(-inf, 1.0));
    CHECK_FALSE(detail::grid_axis_count(static_cast<double>(kMax) + 0.5, 1.0));
    CHECK_FALSE(detail::grid_axis_count(static_cast<double>(kMax) + 1.0, 1.0));
    CHECK_FALSE(detail::grid_axis_count(1.0, 1e-12));
    CHECK_FALSE(detail::grid_axis_count(inf, 1.0));
    CHECK_FALSE(detail::grid_axis_count(nan, 1.0));
    CHECK_FALSE(detail::grid_axis_count(1.0, 0.0));
    CHECK_FALSE(detail::grid_axis_count(1.0, -1.0));
    CHECK_FALSE(detail::grid_axis_count(1.0, nan));
}

TEST_CASE("Grid vertex and total bucket counts are checked in int64 (#161)") {
    constexpr int kMax = std::numeric_limits<int>::max();
    CHECK(detail::grid_add_vertex(0, {kMax, 1}) == kMax);
    CHECK(detail::grid_add_vertex(kMax - 1, {1, 1}) == kMax);
    CHECK_FALSE(detail::grid_add_vertex(1, {kMax, 1}));
    CHECK_FALSE(detail::grid_add_vertex(kMax, {1, 1}));
    CHECK(detail::grid_add_vertex(0, {46341, 46340}) == 46341 * 46340);  // just under 2^31
    CHECK_FALSE(detail::grid_add_vertex(0, {46341, 46341}));
    CHECK_FALSE(detail::grid_add_vertex(0, {kMax, kMax}));
    CHECK_FALSE(detail::grid_add_vertex(kMax, {kMax, kMax}));
}

namespace {
// Two vertices, window [0, 1] on the one main resource.
struct UnitWindowGraph {
    int from[1] = {0};
    int to[1] = {1};
    double cost[1] = {-1.0};
    double time_d[1] = {0.5};
    double tw_lb[2] = {0.0, 0.0};
    double tw_ub[2] = {1.0, 1.0};
    const double* arc_res[1] = {time_d};
    const double* v_lb[1] = {tw_lb};
    const double* v_ub[1] = {tw_ub};
    ProblemView pv;

    UnitWindowGraph() {
        pv.n_vertices = 2;
        pv.source = 0;
        pv.sink = 1;
        pv.n_arcs = 1;
        pv.arc_from = from;
        pv.arc_to = to;
        pv.arc_base_cost = cost;
        pv.n_resources = 1;
        pv.arc_resource = arc_res;
        pv.vertex_lb = v_lb;
        pv.vertex_ub = v_ub;
        pv.n_main_resources = 1;
    }
};
}  // namespace

namespace {
// 0 → 1 → 2 on windows [0, 1], consuming 0.25 then 0.5: the label at vertex 1
// (q = 0.25) reaches the sink only from the bucket it belongs in, since
// 0.75 + 0.5 > 1.
struct UnitWindowChain {
    int from[2] = {0, 1};
    int to[2] = {1, 2};
    double cost[2] = {-1.0, -1.0};
    double time_d[2] = {0.25, 0.5};
    double tw_lb[3] = {0.0, 0.0, 0.0};
    double tw_ub[3] = {1.0, 1.0, 1.0};
    const double* arc_res[1] = {time_d};
    const double* v_lb[1] = {tw_lb};
    const double* v_ub[1] = {tw_ub};
    ProblemView pv;

    UnitWindowChain() {
        pv.n_vertices = 3;
        pv.source = 0;
        pv.sink = 2;
        pv.n_arcs = 2;
        pv.arc_from = from;
        pv.arc_to = to;
        pv.arc_base_cost = cost;
        pv.n_resources = 1;
        pv.arc_resource = arc_res;
        pv.vertex_lb = v_lb;
        pv.vertex_ub = v_ub;
        pv.n_main_resources = 1;
    }
};
}  // namespace

TEST_CASE("A step too fine for its window is refused, the grid kept (#161)") {
    UnitWindowChain g;
    BucketGraph<EmptyPack> bg(g.pv, EmptyPack{}, {.bucket_steps = {0.25, 0.0}, .theta = 1e9});
    bg.build();
    REQUIRE(bg.n_buckets() == 12);

    // 1 / 1e-12 buckets per vertex do not fit an int: refused before the cast.
    bg.set_vertex_bucket_steps({{1e-12, 0.0}, {1e-12, 0.0}, {1e-12, 0.0}});
    CHECK_THROWS_AS(bg.build(), std::invalid_argument);
    CHECK(bg.n_buckets() == 12);  // the old grid is untouched

    // Three vertices of 2^30 buckets each fit an axis but not the total.
    bg.set_vertex_bucket_steps({{0x1p-30, 0.0}, {0x1p-30, 0.0}, {0x1p-30, 0.0}});
    CHECK_THROWS_AS(bg.build(), std::invalid_argument);
    CHECK(bg.n_buckets() == 12);

    // The kept grid still places labels by its own steps, not the refused
    // ones: by 2^-30 the vertex-1 label would land in [0.75, 1] and miss
    // the sink.
    auto paths = bg.solve();
    REQUIRE(paths.size() == 1);
    CHECK(paths[0].reduced_cost == doctest::Approx(-2.0));

    // One step pair per vertex, or no build can index them.
    CHECK_THROWS_AS(bg.set_vertex_bucket_steps({{0.25, 0.0}}), std::invalid_argument);

    // The issue's input: an explicit uniform step of 1e-12 on [0, 1].
    BucketGraph<EmptyPack> fine(g.pv, EmptyPack{}, {.bucket_steps = {1e-12, 0.0}});
    CHECK_THROWS_AS(fine.build(), std::invalid_argument);
}

TEST_CASE("Boundary q values keep their buckets after the #161 checks") {
    UnitWindowGraph g;
    BucketGraph<EmptyPack> bg(g.pv, EmptyPack{}, {.bucket_steps = {0.25, 0.0}, .theta = 1e9});
    bg.build();
    REQUIRE(bg.n_buckets() == 8);
    // Each vertex's four buckets tile [0, 1] in order, the last ending at ub.
    for (int v = 0; v < 2; ++v) {
        for (int k = 0; k < 4; ++k) {
            const Bucket& b = bg.bucket((v * 4) + k);
            CHECK(b.vertex == v);
            CHECK(b.lb[0] == doctest::Approx(0.25 * k));
            CHECK(b.ub[0] == doctest::Approx(0.25 * (k + 1)));
        }
    }
    // The arc 0 → 1 consumes 0.5, so source bucket k arrives at q = 0.25k + 0.5:
    // on an edge (0.5, 0.75), at the window's ub (1.0), or past it (no arc).
    const std::array<int, 4> expected = {4 + 2, 4 + 3, 4 + 3, -1};
    for (int k = 0; k < 4; ++k) {
        const auto& arcs = bg.bucket(k).bucket_arcs;
        if (expected[k] < 0) {
            CHECK(arcs.empty());
            continue;
        }
        REQUIRE(arcs.size() == 1);
        CHECK(arcs[0].to_bucket == expected[k]);
    }
    auto paths = bg.solve();
    REQUIRE(paths.size() == 1);
    CHECK(paths[0].reduced_cost == doctest::Approx(-1.0));
}
