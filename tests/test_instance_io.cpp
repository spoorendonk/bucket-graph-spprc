#include "instance_io.h"
#include "sppg_io.h"

#include <bgspprc/resource.h>
#include <bgspprc/resources/ng_path.h>
#include <bgspprc/resources/side.h>
#include <bgspprc/solver.h>
#include <bit>
#include <cmath>
#include <cstdint>
#include <doctest/doctest.h>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

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
    // Source (depot, vertex 0) must not appear as an ng-candidate of any
    // customer (it's the start/end, never an intermediate-revisit target).
    for (int v = 1; v < 4; ++v) {
        for (int w : inst.ng_neighbors[v]) {
            CHECK(w != inst.source);
        }
    }
    // Sink is terminal — its ng-set is {sink} only.
    REQUIRE(inst.ng_neighbors[4].size() == 1);
    CHECK(inst.ng_neighbors[4][0] == 4);
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

TEST_CASE("Solve through parser: tiny.sppcc with computed ng-path (k=2)") {
    // End-to-end test: load .sppcc → compute_ng_neighbors (skip-source policy
    // + sink ng-set = {sink}) → NgPathResource → solve. Verifies the new
    // ng-set construction policy doesn't break solver correctness.
    auto inst = io::load_sppcc(fixture("tiny.sppcc"));
    io::compute_ng_neighbors(inst, /*k=*/2);
    auto pv = inst.problem_view();

    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, inst.ng_neighbors);
    using NgPack = ResourcePack<NgPathResource>;

    Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
                          {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    // 0→2→4(sink): dist[0][2] + dist[2][0] = 5 + 5 = 10 (same as no-ng case)
    CHECK(paths[0].reduced_cost == doctest::Approx(10.0));
}

// ── sppg ──

static std::expected<io::Instance, std::string> read_sppg_text(const std::string& text) {
    std::istringstream in(text);
    return io::read_sppg(in);
}

TEST_CASE("read_sppg: the format's two-arc example") {
    auto inst = read_sppg_text(
        "c two-arc example\n"
        "p sppg 1 3 2 2 1\n"
        "t 0 2\n"
        "k d n\n"
        "w 0 0 0 0 10\n"
        "w 2 0 100 0 10\n"
        "a 0 1 1.5 3 1\n"
        "a 1 2 -0.25 4 2");  // no final newline
    REQUIRE(inst.has_value());
    auto pv = inst->problem_view();
    CHECK(pv.n_vertices == 3);
    CHECK(pv.n_arcs == 2);
    CHECK(pv.n_resources == 2);
    CHECK(pv.n_main_resources == 1);
    CHECK(pv.source == 0);
    CHECK(pv.sink == 2);
    REQUIRE(pv.resource_nondisposable != nullptr);
    CHECK_FALSE(pv.resource_nondisposable[0]);
    CHECK(pv.resource_nondisposable[1]);
    CHECK(pv.arc_base_cost[1] == -0.25);
    CHECK(pv.arc_resource[1][1] == 2.0);
    CHECK(pv.vertex_ub[0][2] == 100.0);
    // Vertex 1 has no w line: [-inf, +inf].
    CHECK(pv.vertex_lb[0][1] == -std::numeric_limits<double>::infinity());
    CHECK(pv.vertex_ub[1][1] == std::numeric_limits<double>::infinity());
}

TEST_CASE("read_sppg: CRLF, tabs, domain and layers read") {
    auto inst = read_sppg_text(
        "p sppg 1 2 1 1 1\r\n"
        "domain time_indexed\r\n"
        "layers\r\n"
        "t\t0 1\r\n"
        "\r\n"
        "l 0 -1\r\n"
        "a 0 1 inf -inf\r\n"
        "l 1 3\r\n");
    REQUIRE(inst.has_value());
    CHECK(inst->arc_cost[0] == std::numeric_limits<double>::infinity());
    CHECK(inst->res_nondisposable == nullptr);
}

TEST_CASE("read_sppg: errors carry their line") {
    // {text, error}
    const std::pair<const char*, const char*> cases[] = {
        {"", "line 0: no p line"},
        {"t 0 1\n", "line 1: expected the p line first"},
        {"p sppg 2 1 0 0 0\n", "line 1: unsupported sppg version 2"},
        {"p sppg 1 -1 0 0 0\n", "line 1: counts must be non-negative integers"},
        {"p sppg 1 2 0 0 0\n", "line 0: no t line"},
        {"p sppg 1 2 0 0 0\nt 0 2\n", "line 2: source or sink out of range"},
        {"p sppg 1 2 1 0 0\nt 0 1\na 0 1 1\nk\n", "line 4: header line k after the body"},
        {"p sppg 1 2 1 1 0\nt 0 1\na 0 1 nan 1\n", "line 3: bad number"},
        {"p sppg 1 2 1 1 0\nt 0 1\na 0 1 +1 1\n", "line 3: bad number"},
        {"p sppg 1 2 1 1 0\nt 0 1\na 0 1 1\n",
         "line 3: expected a <from> <to> <cost> and a "
         "consumption per resource"},
        {"p sppg 1 2 2 0 0\nt 0 1\na 0 1 1\n", "line 0: fewer a lines than n_arcs"},
        {"p sppg 1 2 0 0 0\nt 0 1\na 0 1 1\n", "line 3: more a lines than n_arcs"},
        {"p sppg 1 2 0 1 0\nt 0 1\nw 1 0 1\nw 1 0 1\n", "line 4: second w line for one vertex"},
        {"p sppg 1 2 0 1 0\nt 0 1\nk x\n", "line 3: kind must be d or n"},
        {"p sppg 1 2 0 0 0\nt 0 1\nl 0 0\n", "line 3: l line without a layers line"},
        {"p sppg 1 2 0 0 0\nlayers\nt 0 1\nl 0 0\n",
         "line 0: layers given but not one l line per vertex"},
        {"p sppg 1 2 0 0 0\nt 0 1\nn 0 1\n", "line 3: unknown tag n"},
        {"p sppg 1 2 0 0 0\nw 0\n", "line 2: missing t line before the body"},
    };
    for (const auto& [text, error] : cases) {
        CAPTURE(text);
        auto inst = read_sppg_text(text);
        REQUIRE_FALSE(inst.has_value());
        CHECK(inst.error() == error);
    }
}

TEST_CASE("write_sppg then read_sppg gives back the same bits") {
    const double inf = std::numeric_limits<double>::infinity();
    io::Instance inst;
    inst.general_resources = true;
    inst.n_vertices = 3;
    inst.source = 0;
    inst.sink = 2;
    inst.n_main_resources = 1;
    inst.arc_from = {0, 1};
    inst.arc_to = {1, 2};
    inst.arc_cost = {0.1, -0.0};
    inst.res_arc = {{1.0 / 3.0, 1e-300}, {-0.0, 2.5}};
    inst.res_lb = {{0, -inf, 0}, {0, -inf, -1}};
    inst.res_ub = {{10, inf, 1e17}, {4, inf, inf}};
    inst.res_nondisposable = std::make_unique<bool[]>(2);
    inst.res_nondisposable[1] = true;

    std::ostringstream out;
    io::write_sppg(out, inst.problem_view());
    CHECK(out.str().find("w 1 ") == std::string::npos);  // unbounded vertex: no w line
    auto back = read_sppg_text(out.str());
    REQUIRE(back.has_value());

    auto bits = [](double x) { return std::bit_cast<std::uint64_t>(x); };
    auto a = inst.problem_view();
    auto b = back->problem_view();
    REQUIRE(b.n_arcs == a.n_arcs);
    REQUIRE(b.n_resources == a.n_resources);
    CHECK(b.n_main_resources == a.n_main_resources);
    CHECK(b.resource_nondisposable[1]);
    CHECK_FALSE(b.resource_nondisposable[0]);
    for (int e = 0; e < a.n_arcs; ++e) {
        CHECK(bits(b.arc_base_cost[e]) == bits(a.arc_base_cost[e]));
        for (int r = 0; r < a.n_resources; ++r) {
            CHECK(bits(b.arc_resource[r][e]) == bits(a.arc_resource[r][e]));
        }
    }
    for (int v = 0; v < a.n_vertices; ++v) {
        for (int r = 0; r < a.n_resources; ++r) {
            CHECK(bits(b.vertex_lb[r][v]) == bits(a.vertex_lb[r][v]));
            CHECK(bits(b.vertex_ub[r][v]) == bits(a.vertex_ub[r][v]));
        }
    }
}

TEST_CASE("Solve through parser: tiny.sppg (optimal cost=9)") {
    auto inst = io::load_sppg(fixture("tiny.sppg"));
    REQUIRE(inst.has_value());
    auto pv = inst->problem_view();

    Solver<EmptyPack> solver(pv, EmptyPack{}, {.theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    CHECK(paths[0].reduced_cost == doctest::Approx(9.0));
    CHECK(paths[0].vertices == std::vector<int>{0, 2, 3, 4});
}

TEST_CASE("Solve through parser: tiny.graph written as sppg solves the same") {
    auto graph = io::load_rcspp_graph(fixture("tiny.graph"));
    std::ostringstream out;
    io::write_sppg(out, graph.problem_view());
    auto inst = read_sppg_text(out.str());
    REQUIRE(inst.has_value());
    auto pv = inst->problem_view();
    CHECK(pv.n_main_resources == 2);

    Solver<EmptyPack> solver(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    CHECK(paths[0].reduced_cost == doctest::Approx(9.0));
}

TEST_CASE("Solve through parser: tiny_side.sppg enforces its non-main resource") {
    auto inst = io::load_sppg(fixture("tiny_side.sppg"));
    REQUIRE(inst.has_value());
    auto pv = inst->problem_view();
    using SidePack = ResourcePack<SideResources>;

    for (bool bidir : {false, true}) {
        CAPTURE(bidir);
        Solver<SidePack> solver(pv, make_resource_pack(SideResources(pv, 1, 1)),
                                {.bidirectional = bidir, .theta = 1e9});
        solver.build();
        solver.set_stage(Stage::Exact);
        auto paths = solver.solve();

        // 0->2->3->4 (cost 9) spends 6 of a budget of 4; 0->2->4 is next.
        REQUIRE(!paths.empty());
        CHECK(paths[0].reduced_cost == doctest::Approx(10.0));
        CHECK(paths[0].vertices == std::vector<int>{0, 2, 4});
    }
}
