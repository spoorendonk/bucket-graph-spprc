#include "instance_io.h"

#include <bgspprc/resource.h>
#include <bgspprc/resources/ng_path.h>
#include <bgspprc/solver.h>
#include <doctest/doctest.h>
#include <string>

using namespace bgspprc;

#ifndef FIXTURE_DIR
#error "FIXTURE_DIR must be defined"
#endif

static std::string fixture(const char* name) {
    return std::string(FIXTURE_DIR) + "/" + name;
}

// ── Parser tests ──

TEST_CASE("load_sppcc: parse tiny.sppcc") {
    auto inst = io::load_sppcc(fixture("tiny.sppcc"));

    // 4 original nodes + 1 sink copy = 5 vertices
    CHECK(inst.n_vertices == 5);
    CHECK(inst.source == 0);
    CHECK(inst.sink == 4);
    CHECK(inst.n_main_resources == 1);

    // Arcs: source(0)→{1,2,3}, 1→{2,3}, 2→{1,3}, 3→{1,2} = 3+2+2+2 = 9 customer arcs
    // Plus return arcs: 1→4, 2→4, 3→4 = 3 sink arcs
    // Total = 12
    CHECK(inst.arc_from.size() == 12);

    // Check a specific arc: 0→1 cost = dist[0][1] + node_weight[1] + depot_dual
    // dist[0][1]=10, node_weight[1]=0, depot_dual=0 → cost=10
    bool found_0_1 = false;
    for (size_t a = 0; a < inst.arc_from.size(); ++a) {
        if (inst.arc_from[a] == 0 && inst.arc_to[a] == 1) {
            CHECK(inst.arc_cost[a] == doctest::Approx(10.0));
            CHECK(inst.arc_demand[a] == doctest::Approx(10.0));
            found_0_1 = true;
            break;
        }
    }
    CHECK(found_0_1);
}

TEST_CASE("load_roberti_vrp: parse tiny.vrp") {
    auto inst = io::load_roberti_vrp(fixture("tiny.vrp"));

    // 4 original nodes + 1 sink copy = 5 vertices
    CHECK(inst.n_vertices == 5);
    CHECK(inst.source == 0);  // depot=1 (1-indexed) → 0 (0-indexed)
    CHECK(inst.sink == 4);
    CHECK(inst.n_main_resources == 1);

    // Arcs: source(0)→{1,2,3}, 1→{2,3}, 2→{1,3}, 3→{1,2} = 9 customer arcs
    // Plus return arcs: 1→4, 2→4, 3→4 = 3 sink arcs
    // Total = 12
    CHECK(inst.arc_from.size() == 12);

    // Check arc 0→1: dist((0,0),(3,0)) - profit[1] = 3.0 - 0 = 3.0
    bool found_0_1 = false;
    for (size_t a = 0; a < inst.arc_from.size(); ++a) {
        if (inst.arc_from[a] == 0 && inst.arc_to[a] == 1) {
            CHECK(inst.arc_cost[a] == doctest::Approx(3.0));
            CHECK(inst.arc_demand[a] == doctest::Approx(10.0));
            found_0_1 = true;
            break;
        }
    }
    CHECK(found_0_1);

    // Check arc 0→2: dist((0,0),(0,4)) - profit[2] = 4.0 - 0 = 4.0
    bool found_0_2 = false;
    for (size_t a = 0; a < inst.arc_from.size(); ++a) {
        if (inst.arc_from[a] == 0 && inst.arc_to[a] == 2) {
            CHECK(inst.arc_cost[a] == doctest::Approx(4.0));
            found_0_2 = true;
            break;
        }
    }
    CHECK(found_0_2);
}

TEST_CASE("load_rcspp_graph: parse tiny.graph") {
    auto inst = io::load_rcspp_graph(fixture("tiny.graph"));

    CHECK(inst.n_vertices == 5);
    CHECK(inst.source == 0);
    CHECK(inst.sink == 4);
    CHECK(inst.n_main_resources == 2);

    // 7 arcs
    CHECK(inst.arc_from.size() == 7);

    // Time data populated
    CHECK(inst.arc_time.size() == 7);

    // Check arc 0→1: cost=10, time=2
    bool found = false;
    for (size_t a = 0; a < inst.arc_from.size(); ++a) {
        if (inst.arc_from[a] == 0 && inst.arc_to[a] == 1) {
            CHECK(inst.arc_cost[a] == doctest::Approx(10.0));
            CHECK(inst.arc_time[a] == doctest::Approx(2.0));
            found = true;
            break;
        }
    }
    CHECK(found);

    // ng-neighbors parsed from file
    CHECK(inst.ng_neighbors.size() == 5);
    // Vertex 0's neighbors: {0, 1, 2} (self included)
    CHECK(inst.ng_neighbors[0].size() == 3);
}

TEST_CASE("compute_ng_neighbors: on sppcc instance with k=2") {
    auto inst = io::load_sppcc(fixture("tiny.sppcc"));
    io::compute_ng_neighbors(inst, 2);

    // Each vertex should have itself + 1 neighbor = 2 entries
    CHECK(inst.ng_neighbors.size() == 5);
    for (int v = 0; v < 4; ++v) {
        CHECK(inst.ng_neighbors[v].size() == 2);
        CHECK(inst.ng_neighbors[v][0] == v);  // self is first
    }
    // Sink copies source's neighbors
    CHECK(inst.ng_neighbors[4] == inst.ng_neighbors[0]);
}

// ── Solve-through-parser tests ──

TEST_CASE("Solve through parser: tiny.sppcc") {
    auto inst = io::load_sppcc(fixture("tiny.sppcc"));
    auto pv = inst.problem_view();

    Solver<EmptyPack> solver(pv, EmptyPack{}, {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    // 0→2→4(sink): dist[0][2] + dist[2][0] = 5 + 5 = 10
    CHECK(paths[0].reduced_cost == doctest::Approx(10.0));
}

TEST_CASE("Solve through parser: tiny.vrp") {
    auto inst = io::load_roberti_vrp(fixture("tiny.vrp"));
    auto pv = inst.problem_view();

    Solver<EmptyPack> solver(pv, EmptyPack{}, {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    // 0→1→4(sink): dist((0,0),(3,0)) + dist((3,0),(0,0)) = 3 + 3 = 6
    CHECK(paths[0].reduced_cost == doctest::Approx(6.0));
}

TEST_CASE("Solve through parser: tiny.graph (optimal cost=9)") {
    auto inst = io::load_rcspp_graph(fixture("tiny.graph"));
    auto pv = inst.problem_view();

    Solver<EmptyPack> solver(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    CHECK(paths[0].reduced_cost == doctest::Approx(9.0));
    CHECK(paths[0].vertices.size() == 4);
    CHECK(paths[0].vertices[0] == 0);
    CHECK(paths[0].vertices[1] == 2);
    CHECK(paths[0].vertices[2] == 3);
    CHECK(paths[0].vertices[3] == 4);
}

TEST_CASE("Solve through parser: tiny.graph with ng-path") {
    auto inst = io::load_rcspp_graph(fixture("tiny.graph"));
    auto pv = inst.problem_view();

    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
    using NgPack = ResourcePack<NgPathResource>;

    Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
                          {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    CHECK(paths[0].reduced_cost == doctest::Approx(9.0));
}
