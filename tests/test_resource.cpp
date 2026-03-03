#include "doctest.h"
#include <bgspprc/resource.h>
#include <bgspprc/resources/standard.h>
#include <bgspprc/resources/ng_path.h>
#include <bgspprc/solver.h>

using namespace bgspprc;

// ── StandardResource ──

TEST_CASE("StandardResource init_state forward") {
    //  source=0, sink=2, 3 vertices
    double consumption[] = {1.0, 2.0, 3.0};
    double lb[] = {0.0, 5.0, 10.0};
    double ub[] = {100.0, 50.0, 30.0};

    StandardResource res(consumption, lb, ub, 0, 2, 3);

    CHECK(res.init_state(Direction::Forward) == 0.0);    // lb[source]
    CHECK(res.init_state(Direction::Backward) == 10.0);  // lb[sink]
}

TEST_CASE("StandardResource extend") {
    double consumption[] = {5.0, 3.0};
    double lb[] = {0.0, 0.0, 0.0};
    double ub[] = {100.0, 100.0, 100.0};

    StandardResource res(consumption, lb, ub, 0, 2, 2);

    auto [s1, c1] = res.extend(Direction::Forward, 10.0, 0);
    CHECK(s1 == 15.0);
    CHECK(c1 == 0.0);

    auto [s2, c2] = res.extend(Direction::Forward, 10.0, 1);
    CHECK(s2 == 13.0);
    CHECK(c2 == 0.0);
}

// ── ResourcePack ──

TEST_CASE("EmptyPack operations") {
    EmptyPack pack;
    auto states = pack.init_states(Direction::Forward);
    static_assert(std::tuple_size_v<decltype(states)> == 0);

    auto [new_states, cost] = pack.extend(Direction::Forward, states, 0);
    CHECK(cost == 0.0);
}

TEST_CASE("ResourcePack with StandardResource") {
    double consumption[] = {2.0, 3.0};
    double lb[] = {0.0, 0.0, 0.0};
    double ub[] = {100.0, 100.0, 100.0};

    auto pack = make_resource_pack(
        StandardResource(consumption, lb, ub, 0, 2, 2));

    auto states = pack.init_states(Direction::Forward);
    CHECK(std::get<0>(states) == 0.0);

    auto [new_states, cost] = pack.extend(Direction::Forward, states, 0);
    CHECK(std::get<0>(new_states) == 2.0);
    CHECK(cost == 0.0);
}

// ════════════════════════════════════════════════════════════════
// NgPathResource — local bit mapping tests
// ════════════════════════════════════════════════════════════════

// Helper: build a small graph with ng-neighborhoods for testing.
//
// Graph: 5 vertices (0=source, 4=sink), complete-ish arcs.
//
//   0 → 1  (arc 0)     1 → 2  (arc 2)     2 → 3  (arc 5)
//   0 → 2  (arc 1)     1 → 3  (arc 3)     2 → 4  (arc 6)
//                       1 → 4  (arc 4)     3 → 4  (arc 7)
//                       2 → 1  (arc 8)     3 → 1  (arc 9)
//
// ng-neighbors (size 3): each vertex knows itself + 2 nearest.
//
static NgPathResource make_ng5() {
    static int from[] = {0, 0, 1, 1, 1, 2, 2, 3, 2, 3};
    static int to[]   = {1, 2, 2, 3, 4, 3, 4, 4, 1, 1};
    // 10 arcs
    std::vector<std::vector<int>> neighbors = {
        {0, 1, 2},  // v0: {0, 1, 2}
        {1, 0, 2},  // v1: {1, 0, 2}
        {2, 1, 3},  // v2: {2, 1, 3}
        {3, 2, 1},  // v3: {3, 2, 1}
        {4, 3, 2},  // v4: {4, 3, 2}
    };
    return NgPathResource(5, 10, from, to, neighbors);
}

TEST_CASE("NgPathResource: bit_map assigns local positions") {
    auto ng = make_ng5();

    // v0's neighbors are {0,1,2} → bit positions 0,1,2
    CHECK(ng.bit_map[0 * 5 + 0] == 0);  // 0 in v0's set at pos 0
    CHECK(ng.bit_map[0 * 5 + 1] == 1);  // 1 in v0's set at pos 1
    CHECK(ng.bit_map[0 * 5 + 2] == 2);  // 2 in v0's set at pos 2
    CHECK(ng.bit_map[0 * 5 + 3] == -1); // 3 not in v0's set
    CHECK(ng.bit_map[0 * 5 + 4] == -1); // 4 not in v0's set

    // v2's neighbors are {2,1,3} → bit positions 0,1,2
    CHECK(ng.bit_map[2 * 5 + 2] == 0);  // 2 at pos 0
    CHECK(ng.bit_map[2 * 5 + 1] == 1);  // 1 at pos 1
    CHECK(ng.bit_map[2 * 5 + 3] == 2);  // 3 at pos 2
    CHECK(ng.bit_map[2 * 5 + 0] == -1); // 0 not in v2's set
}

TEST_CASE("NgPathResource: init_state is zero") {
    auto ng = make_ng5();
    CHECK(ng.init_state(Direction::Forward) == 0ULL);
    CHECK(ng.init_state(Direction::Backward) == 0ULL);
}

TEST_CASE("NgPathResource: forward extend marks source in target ng-set") {
    auto ng = make_ng5();

    // Extend arc 0: 0→1. Label at v0, moving to v1.
    // v1's ng-set = {1,0,2}. v0 is at bit_map[1*5+0] = 1.
    // So mark_mask should set bit 1.
    auto [s1, c1] = ng.extend(Direction::Forward, 0ULL, 0);
    CHECK(c1 == 0.0);
    // bit for v0 in v1's local mapping
    int8_t v0_in_v1 = ng.bit_map[1 * 5 + 0];
    CHECK(v0_in_v1 >= 0);
    CHECK((s1 & (1ULL << v0_in_v1)) != 0);  // v0 marked as visited
}

TEST_CASE("NgPathResource: forward extend transforms bits across vertices") {
    auto ng = make_ng5();

    // Start at v0, extend to v1 (arc 0), then to v2 (arc 2: 1→2)
    auto [s1, c1] = ng.extend(Direction::Forward, 0ULL, 0);
    REQUIRE(c1 == 0.0);

    // At v1, state has v0 marked. Now extend to v2.
    // v0 is in v1's ng-set at some position, but v0 is NOT in v2's ng-set.
    // So the v0 bit should be dropped when we arrive at v2.
    auto [s2, c2] = ng.extend(Direction::Forward, s1, 2);
    CHECK(c2 == 0.0);

    // At v2, ng-set = {2,1,3}. v0 is not in it, so no bit for v0.
    // v1 should be marked (source of arc 2 is v1, and v1 is in v2's ng-set).
    int8_t v1_in_v2 = ng.bit_map[2 * 5 + 1];
    CHECK(v1_in_v2 >= 0);
    CHECK((s2 & (1ULL << v1_in_v2)) != 0);  // v1 marked

    // v0 is NOT in v2's ng-set, so no bit should be set for v0.
    int8_t v0_in_v2 = ng.bit_map[2 * 5 + 0];
    CHECK(v0_in_v2 == -1);  // v0 not in v2's neighbors at all
}

TEST_CASE("NgPathResource: forward revisit detection") {
    auto ng = make_ng5();

    // Path: 0→1 (arc 0), 1→2 (arc 2), 2→1 (arc 8) should be infeasible.
    // At v1: v0 is marked.
    auto [s1, c1] = ng.extend(Direction::Forward, 0ULL, 0);
    REQUIRE(c1 == 0.0);

    // At v2: v1 is marked.
    auto [s2, c2] = ng.extend(Direction::Forward, s1, 2);
    REQUIRE(c2 == 0.0);

    // Try 2→1 (arc 8). v1 is in v2's ng-set, and v1 was visited (marked).
    // check_bit for arc 8 = bit_map[from=2][to=1] = v1 in v2's set.
    auto [s3, c3] = ng.extend(Direction::Forward, s2, 8);
    CHECK(c3 == INF);  // infeasible: revisiting v1
}

TEST_CASE("NgPathResource: visit outside ng-set is allowed") {
    auto ng = make_ng5();

    // v0's ng-set = {0,1,2}. v3 is NOT in v0's ng-set.
    // So visiting v3 after v0 doesn't set any bit at v0,
    // and revisiting v3 via a vertex whose ng-set doesn't include v3
    // should not be blocked.

    // Path: 0→1 (arc 0), 1→3 (arc 3), 3→1 (arc 9)
    // v1's ng-set = {1,0,2}. v3 not in v1's set → check_bit = -1 → no block.
    auto [s1, c1] = ng.extend(Direction::Forward, 0ULL, 0);
    REQUIRE(c1 == 0.0);

    // Arc 3: 1→3. v3 is in v1's set? bit_map[1*5+3]
    int8_t v3_in_v1 = ng.bit_map[1 * 5 + 3];
    CHECK(v3_in_v1 == -1);  // v3 NOT in v1's ng-set

    auto [s2, c2] = ng.extend(Direction::Forward, s1, 3);
    CHECK(c2 == 0.0);  // feasible

    // Arc 9: 3→1. At v3, v1 is in v3's ng-set.
    // v1 should be marked in state from extending arc 3 (mark v1 in v3's set).
    int8_t v1_in_v3 = ng.bit_map[3 * 5 + 1];
    CHECK(v1_in_v3 >= 0);
    CHECK((s2 & (1ULL << v1_in_v3)) != 0);  // v1 marked at v3

    // So extending arc 9 (3→1) should be infeasible because v1 is marked.
    auto [s3, c3] = ng.extend(Direction::Forward, s2, 9);
    CHECK(c3 == INF);  // infeasible: v1 forbidden at v3
}

TEST_CASE("NgPathResource: backward extend") {
    auto ng = make_ng5();

    // Arc 7: 3→4 (from=3, to=4). Backward: label at v4, extending to v3.
    // check: is v3 (=from) in v4's ng-set? bit_map[4*5+3]
    int8_t v3_in_v4 = ng.bit_map[4 * 5 + 3];
    CHECK(v3_in_v4 >= 0);

    auto [s1, c1] = ng.extend(Direction::Backward, 0ULL, 7);
    CHECK(c1 == 0.0);

    // At v3: v4 should be marked (mark source=to=4 in from=3's ng-set)
    // But v4 is NOT in v3's ng-set: bit_map[3*5+4] == -1
    int8_t v4_in_v3 = ng.bit_map[3 * 5 + 4];
    CHECK(v4_in_v3 == -1);  // so mark_mask is 0, no bit set for v4

    // v2 is in v3's ng-set at bit_map[3*5+2]
    // Extend arc 5: 2→3. Backward: label at v3, extending to v2.
    auto [s2, c2] = ng.extend(Direction::Backward, s1, 5);
    CHECK(c2 == 0.0);

    // At v2: v3 should be marked (source=to=3 in from=2's ng-set)
    int8_t v3_in_v2 = ng.bit_map[2 * 5 + 3];
    CHECK(v3_in_v2 >= 0);
    CHECK((s2 & (1ULL << v3_in_v2)) != 0);
}

TEST_CASE("NgPathResource: backward revisit detection") {
    auto ng = make_ng5();

    // Backward path: start at v4, go backward.
    // Arc 7: from=3, to=4. Backward: label at v4, extend to v3.
    auto [s1, c1] = ng.extend(Direction::Backward, 0ULL, 7);
    REQUIRE(c1 == 0.0);

    // Arc 3: from=1, to=3. Backward: label at v3, extend to v1.
    auto [s2, c2] = ng.extend(Direction::Backward, s1, 3);
    REQUIRE(c2 == 0.0);

    // At v1, v3 should be marked (v3=to of arc 3, marked in from=1's ng-set).
    // v3 is in v1's ng-set? bit_map[1*5+3] = -1.
    // v3 is NOT in v1's ng-set, so no bit for v3 at v1.
    int8_t v3_in_v1 = ng.bit_map[1 * 5 + 3];
    CHECK(v3_in_v1 == -1);

    // But v4 was the starting vertex, and v4 is also not in v1's ng-set.
    // So at v1, the state may just have bits for neighbors of v1 that were
    // remapped through the chain.

    // Now test a real cycle block:
    // Arc 2: from=1, to=2. Backward: label at v2, extend to v1.
    // Arc 8: from=2, to=1. Backward: label at v1, extend to v2.
    // Path: 4←3←1←2←1 would revisit v1.

    // Start fresh: backward on arc 6 (from=2, to=4): at v4, go to v2.
    auto [sb1, cb1] = ng.extend(Direction::Backward, 0ULL, 6);
    REQUIRE(cb1 == 0.0);

    // Now at v2. Backward on arc 2 (from=1, to=2): at v2, go to v1.
    auto [sb2, cb2] = ng.extend(Direction::Backward, sb1, 2);
    REQUIRE(cb2 == 0.0);

    // Now at v1. v2 should be marked in v1's state (v2=to of arc 2, marked
    // in from=1's ng-set). v2 is in v1's ng-set at bit_map[1*5+2].
    int8_t v2_in_v1 = ng.bit_map[1 * 5 + 2];
    CHECK(v2_in_v1 >= 0);
    CHECK((sb2 & (1ULL << v2_in_v1)) != 0);

    // Try backward on arc 8 (from=2, to=1): at v1, go to v2 (revisit!).
    // bw_check_bit = bit_map[to=1][from=2] = bit_map[1*5+2] = v2 in v1's set.
    // v2 is marked → should be infeasible.
    auto [sb3, cb3] = ng.extend(Direction::Backward, sb2, 8);
    CHECK(cb3 == INF);  // infeasible: revisiting v2
}

// ── Domination ──

TEST_CASE("NgPathResource: domination subset check") {
    auto ng = make_ng5();

    // At same vertex, states use same local bit positions.
    uint64_t fewer = 0b010;   // one bit set
    uint64_t more  = 0b110;   // two bits set (superset)
    uint64_t disjoint = 0b001;

    // fewer ⊆ more → can dominate
    CHECK(ng.domination_cost(Direction::Forward, 1, fewer, more) == 0.0);
    // more ⊄ fewer → cannot dominate
    CHECK(ng.domination_cost(Direction::Forward, 1, more, fewer) == INF);
    // disjoint ⊄ more → cannot dominate
    CHECK(ng.domination_cost(Direction::Forward, 1, disjoint, more) == INF);
    // empty set ⊆ anything → can dominate
    CHECK(ng.domination_cost(Direction::Forward, 1, 0ULL, more) == 0.0);
    // same set → can dominate (equal is subset)
    CHECK(ng.domination_cost(Direction::Forward, 1, more, more) == 0.0);

    // Direction doesn't matter for domination
    CHECK(ng.domination_cost(Direction::Backward, 1, fewer, more) == 0.0);
    CHECK(ng.domination_cost(Direction::Backward, 1, more, fewer) == INF);
}

// ── Concatenation ──

TEST_CASE("NgPathResource: concatenation rejects shared visited vertices") {
    auto ng = make_ng5();

    // Forward and backward labels at same vertex: same local mapping.
    // If any bit set in both → cycle → INF.
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0b010, 0b100) == 0.0);
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0b010, 0b010) == INF);
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0b111, 0b001) == INF);
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0ULL, 0b111) == 0.0);
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0ULL, 0ULL) == 0.0);
}

// ── Empty ng-neighbors degrades to no enforcement ──

TEST_CASE("NgPathResource: empty neighbors = no elementarity") {
    int from[] = {0, 1, 2, 1};
    int to[]   = {1, 2, 1, 0};
    std::vector<std::vector<int>> neighbors(3);  // all empty

    NgPathResource ng(3, 4, from, to, neighbors);

    auto s0 = ng.init_state(Direction::Forward);
    CHECK(s0 == 0ULL);

    // Extend 0→1→2→1 — no cycle block because no ng-neighbors
    auto [s1, c1] = ng.extend(Direction::Forward, s0, 0);
    CHECK(c1 == 0.0);
    CHECK(s1 == 0ULL);  // no bits set

    auto [s2, c2] = ng.extend(Direction::Forward, s1, 1);
    CHECK(c2 == 0.0);
    CHECK(s2 == 0ULL);

    // Revisit v1 — allowed
    auto [s3, c3] = ng.extend(Direction::Forward, s2, 2);
    CHECK(c3 == 0.0);
    CHECK(s3 == 0ULL);
}

// ── ResourcePack with NgPathResource ──

TEST_CASE("ResourcePack<NgPathResource> composes correctly") {
    auto ng = make_ng5();
    auto pack = make_resource_pack(std::move(ng));

    auto states = pack.init_states(Direction::Forward);
    CHECK(std::get<0>(states) == 0ULL);

    // Extend arc 0 (0→1)
    auto [s1, c1] = pack.extend(Direction::Forward, states, 0);
    CHECK(c1 == 0.0);
    CHECK(std::get<0>(s1) != 0ULL);  // some bits set

    // Domination: zero state dominates any non-zero state
    CHECK(pack.domination_cost(Direction::Forward, 1, states, s1) == 0.0);
    CHECK(pack.domination_cost(Direction::Forward, 1, s1, states) == INF);
}

// ════════════════════════════════════════════════════════════════
// End-to-end solver tests with NgPathResource
// ════════════════════════════════════════════════════════════════

// Graph with a cycle that has negative cost:
//
//   0 --(-5)--> 1 --(-5)--> 2 --(-5)--> 1  (cycle!)  --> 3
//   0 --( 1)--> 3  (direct)
//
// Without ng-path: solver can exploit the 1→2→1 cycle for very negative cost.
// With ng-path (all vertices in each other's ng-set): cycle is blocked.

static ProblemView make_cycle_pv(int* from, int* to, double* cost,
                                  double* time_d, double* tw_lb, double* tw_ub,
                                  const double** arc_res, const double** v_lb,
                                  const double** v_ub) {
    // 4 vertices: 0=source, 3=sink
    // Arcs: 0→1, 0→3, 1→2, 2→1, 1→3, 2→3
    //  idx:  0    1    2    3    4    5
    from[0] = 0; to[0] = 1; cost[0] = -5.0; time_d[0] = 1.0;
    from[1] = 0; to[1] = 3; cost[1] =  1.0; time_d[1] = 1.0;
    from[2] = 1; to[2] = 2; cost[2] = -5.0; time_d[2] = 1.0;
    from[3] = 2; to[3] = 1; cost[3] = -5.0; time_d[3] = 1.0;
    from[4] = 1; to[4] = 3; cost[4] =  1.0; time_d[4] = 1.0;
    from[5] = 2; to[5] = 3; cost[5] =  1.0; time_d[5] = 1.0;

    tw_lb[0] = 0; tw_lb[1] = 0; tw_lb[2] = 0; tw_lb[3] = 0;
    tw_ub[0] = 100; tw_ub[1] = 100; tw_ub[2] = 100; tw_ub[3] = 100;

    arc_res[0] = time_d;
    v_lb[0] = tw_lb;
    v_ub[0] = tw_ub;

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 6;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;
    return pv;
}

using NgPack = ResourcePack<NgPathResource>;

TEST_CASE("Solver with NgPathResource: cycle blocked (mono)") {
    int from[6], to[6]; double cost[6], time_d[6], tw_lb[4], tw_ub[4];
    const double* arc_res[1]; const double* v_lb[1]; const double* v_ub[1];
    auto pv = make_cycle_pv(from, to, cost, time_d, tw_lb, tw_ub,
                            arc_res, v_lb, v_ub);

    // ng-neighbors: everyone is a neighbor of everyone (full elementarity for 4 verts)
    std::vector<std::vector<int>> neighbors = {
        {0, 1, 2, 3}, {1, 0, 2, 3}, {2, 0, 1, 3}, {3, 0, 1, 2}
    };
    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, neighbors);
    Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
        {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    // Best elementary path: 0→1→2→3 cost = -5 + -5 + 1 = -9
    // Or: 0→1→3 cost = -5 + 1 = -4
    // The 1→2→1 cycle is blocked, so no exploiting it.
    CHECK(paths[0].reduced_cost == doctest::Approx(-9.0));

    // Verify no vertex repeats in best path
    auto& verts = paths[0].vertices;
    for (size_t i = 0; i < verts.size(); ++i)
        for (size_t j = i + 1; j < verts.size(); ++j)
            CHECK(verts[i] != verts[j]);
}

TEST_CASE("Solver with NgPathResource: cycle blocked (bidir)") {
    int from[6], to[6]; double cost[6], time_d[6], tw_lb[4], tw_ub[4];
    const double* arc_res[1]; const double* v_lb[1]; const double* v_ub[1];
    auto pv = make_cycle_pv(from, to, cost, time_d, tw_lb, tw_ub,
                            arc_res, v_lb, v_ub);

    std::vector<std::vector<int>> neighbors = {
        {0, 1, 2, 3}, {1, 0, 2, 3}, {2, 0, 1, 3}, {3, 0, 1, 2}
    };
    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, neighbors);
    Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
        {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    CHECK(paths[0].reduced_cost == doctest::Approx(-9.0));

    auto& verts = paths[0].vertices;
    for (size_t i = 0; i < verts.size(); ++i)
        for (size_t j = i + 1; j < verts.size(); ++j)
            CHECK(verts[i] != verts[j]);
}

TEST_CASE("Solver with NgPathResource: mono == bidir cost") {
    int from[6], to[6]; double cost[6], time_d[6], tw_lb[4], tw_ub[4];
    const double* arc_res[1]; const double* v_lb[1]; const double* v_ub[1];
    auto pv = make_cycle_pv(from, to, cost, time_d, tw_lb, tw_ub,
                            arc_res, v_lb, v_ub);

    std::vector<std::vector<int>> neighbors = {
        {0, 1, 2, 3}, {1, 0, 2, 3}, {2, 0, 1, 3}, {3, 0, 1, 2}
    };

    NgPathResource ng_m(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, neighbors);
    Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_m)),
        {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
    mono.build();
    mono.set_stage(Stage::Exact);
    auto mono_paths = mono.solve();

    NgPathResource ng_b(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, neighbors);
    Solver<NgPack> bidir(pv, make_resource_pack(std::move(ng_b)),
        {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
    bidir.build();
    bidir.set_stage(Stage::Exact);
    auto bidir_paths = bidir.solve();

    REQUIRE(!mono_paths.empty());
    REQUIRE(!bidir_paths.empty());
    CHECK(mono_paths[0].reduced_cost ==
          doctest::Approx(bidir_paths[0].reduced_cost).epsilon(0.01));
}

TEST_CASE("Solver without ng: cycle exploited for lower cost") {
    int from[6], to[6]; double cost[6], time_d[6], tw_lb[4], tw_ub[4];
    const double* arc_res[1]; const double* v_lb[1]; const double* v_ub[1];
    auto pv = make_cycle_pv(from, to, cost, time_d, tw_lb, tw_ub,
                            arc_res, v_lb, v_ub);

    // Without ng-path, the solver can use the 1→2→1 cycle.
    Solver<EmptyPack> solver(pv, EmptyPack{},
        {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    // With cycles: 0→1→2→1→2→1→...→3, each 1→2→1 adds -10 cost.
    // Must be more negative than -9 (the elementary optimum).
    CHECK(paths[0].reduced_cost < -9.0 - 1e-6);
}

TEST_CASE("Solver with partial ng-set: only nearby vertices blocked") {
    // Same graph, but ng-set only includes self — no cycle blocking.
    int from[6], to[6]; double cost[6], time_d[6], tw_lb[4], tw_ub[4];
    const double* arc_res[1]; const double* v_lb[1]; const double* v_ub[1];
    auto pv = make_cycle_pv(from, to, cost, time_d, tw_lb, tw_ub,
                            arc_res, v_lb, v_ub);

    // ng-neighbors: only self. No neighbors means no cycle detection.
    std::vector<std::vector<int>> neighbors = {
        {0}, {1}, {2}, {3}
    };
    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, neighbors);
    Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
        {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();

    REQUIRE(!paths.empty());
    // Self-only ng-set won't block 1→2→1 because the check_bit for
    // "is target in source's ng-set" will be -1 for non-self vertices.
    // So cycles are still exploited.
    CHECK(paths[0].reduced_cost < -9.0 - 1e-6);
}
