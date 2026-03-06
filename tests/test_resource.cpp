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

    // StandardResource is conservatively non-symmetric
    CHECK(pack.symmetric() == false);
}

// ════════════════════════════════════════════════════════════════
// NgPathResource — local bit mapping tests (SOURCE MARKING)
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

TEST_CASE("NgPathResource: init_state returns zero (source marking)") {
    auto ng = make_ng5();

    // Source marking: init state has no bits set
    CHECK(ng.init_state(Direction::Forward) == 0ULL);
    CHECK(ng.init_state(Direction::Backward) == 0ULL);
}

TEST_CASE("NgPathResource: forward extend marks SOURCE in target ordering") {
    auto ng = make_ng5();

    auto s0 = ng.init_state(Direction::Forward);

    // Extend arc 0: 0→1. Source = v0.
    // v0 in v1's ng-set at bit_map[1*5+0] = 1. Mark mask = 1<<1 = 2.
    auto [s1, c1] = ng.extend(Direction::Forward, s0, 0);
    CHECK(c1 == 0.0);

    // v0 should be marked in v1's ordering
    int8_t v0_in_v1 = ng.bit_map[1 * 5 + 0];
    CHECK(v0_in_v1 >= 0);
    CHECK((s1 & (1ULL << v0_in_v1)) != 0);

    // No other bits should be set (init was 0, no neighbor remaps from empty)
    CHECK(s1 == (1ULL << v0_in_v1));
}

TEST_CASE("NgPathResource: forward extend transforms bits across vertices") {
    auto ng = make_ng5();

    auto s0 = ng.init_state(Direction::Forward);

    // Extend 0→1 (arc 0), then 1→2 (arc 2)
    auto [s1, c1] = ng.extend(Direction::Forward, s0, 0);
    REQUIRE(c1 == 0.0);

    auto [s2, c2] = ng.extend(Direction::Forward, s1, 2);
    CHECK(c2 == 0.0);

    // At v2: source v1 should be marked (v1 ∈ N(v2) at bit_map[2*5+1]=1)
    int8_t v1_in_v2 = ng.bit_map[2 * 5 + 1];
    CHECK(v1_in_v2 >= 0);
    CHECK((s2 & (1ULL << v1_in_v2)) != 0);  // v1 marked (source of arc 1→2)

    // v0 was marked at v1 (bit_map[1*5+0]=1). When remapping from v1 to v2:
    // v0 ∈ N(v1) at pos 1. Is v0 ∈ N(v2)? bit_map[2*5+0] = -1. No remap.
    // So v0 is NOT tracked at v2 (correct — v0 ∉ N(v2))
    int8_t v0_in_v2 = ng.bit_map[2 * 5 + 0];
    CHECK(v0_in_v2 == -1);
}

TEST_CASE("NgPathResource: forward revisit detection") {
    auto ng = make_ng5();

    auto s0 = ng.init_state(Direction::Forward);

    // Path: 0→1 (arc 0), 1→2 (arc 2), 2→1 (arc 8) should be infeasible.
    auto [s1, c1] = ng.extend(Direction::Forward, s0, 0);
    REQUIRE(c1 == 0.0);

    auto [s2, c2] = ng.extend(Direction::Forward, s1, 2);
    REQUIRE(c2 == 0.0);

    // Try 2→1 (arc 8). v1 is in v2's ng-set. v1 was marked (source marking
    // of arc 1→2 set v1's bit in v2's ordering). So check detects v1 visited.
    auto [s3, c3] = ng.extend(Direction::Forward, s2, 8);
    CHECK(c3 == INF);  // infeasible: revisiting v1
}

TEST_CASE("NgPathResource: visit outside ng-set is allowed") {
    auto ng = make_ng5();

    auto s0 = ng.init_state(Direction::Forward);

    // Path: 0→1 (arc 0), 1→3 (arc 3), 3→1 (arc 9)
    auto [s1, c1] = ng.extend(Direction::Forward, s0, 0);
    REQUIRE(c1 == 0.0);

    // Arc 3: 1→3. v3 not in v1's ng-set → check_bit = -1 → no block.
    int8_t v3_in_v1 = ng.bit_map[1 * 5 + 3];
    CHECK(v3_in_v1 == -1);  // v3 NOT in v1's ng-set

    auto [s2, c2] = ng.extend(Direction::Forward, s1, 3);
    CHECK(c2 == 0.0);  // feasible

    // At v3: v1 should be marked (source of arc 1→3, v1 ∈ N(v3))
    int8_t v1_in_v3 = ng.bit_map[3 * 5 + 1];
    CHECK(v1_in_v3 >= 0);
    CHECK((s2 & (1ULL << v1_in_v3)) != 0);  // v1 marked at v3

    // Arc 9: 3→1. v1 is in v3's ng-set and v1 was visited → infeasible.
    auto [s3, c3] = ng.extend(Direction::Forward, s2, 9);
    CHECK(c3 == INF);  // infeasible: v1 forbidden at v3
}

TEST_CASE("NgPathResource: backward extend") {
    auto ng = make_ng5();

    auto s0 = ng.init_state(Direction::Backward);

    // Arc 7: 3→4 (from=3, to=4). Backward: label at v4, extending to v3.
    // Source = v4. Mark v4 in v3's ordering.
    auto [s1, c1] = ng.extend(Direction::Backward, s0, 7);
    CHECK(c1 == 0.0);

    // v4 should be marked in v3's ordering (if v4 ∈ N(v3))
    // v3's neighbors are {3,2,1}. v4 ∉ N(v3) → mark_mask = 0
    // So no bits set from v4 marking alone (v4 not tracked at v3)

    // Extend arc 5: 2→3. Backward: label at v3, extending to v2.
    // Source = v3. Mark v3 in v2's ordering.
    auto [s2, c2] = ng.extend(Direction::Backward, s1, 5);
    CHECK(c2 == 0.0);

    // At v2: v3 should be marked (v3 ∈ N(v2) at bit_map[2*5+3]=2)
    int8_t v3_in_v2 = ng.bit_map[2 * 5 + 3];
    CHECK(v3_in_v2 >= 0);
    CHECK((s2 & (1ULL << v3_in_v2)) != 0);
}

TEST_CASE("NgPathResource: backward revisit detection") {
    auto ng = make_ng5();

    auto s0 = ng.init_state(Direction::Backward);

    // Start fresh: backward on arc 6 (from=2, to=4): at v4, go to v2.
    auto [sb1, cb1] = ng.extend(Direction::Backward, s0, 6);
    REQUIRE(cb1 == 0.0);

    // Now at v2. Backward on arc 2 (from=1, to=2): at v2, go to v1.
    auto [sb2, cb2] = ng.extend(Direction::Backward, sb1, 2);
    REQUIRE(cb2 == 0.0);

    // At v1: v2 should be marked (source of backward arc 2 traversal, v2 ∈ N(v1))
    int8_t v2_in_v1 = ng.bit_map[1 * 5 + 2];
    CHECK(v2_in_v1 >= 0);
    CHECK((sb2 & (1ULL << v2_in_v1)) != 0);

    // Try backward on arc 8 (from=2, to=1): at v1, go to v2 (revisit!).
    // bw_check_bit for arc 8 = bit_map[to=1][from=2] = v2 in v1's set.
    // v2 is marked → should be infeasible.
    auto [sb3, cb3] = ng.extend(Direction::Backward, sb2, 8);
    CHECK(cb3 == INF);  // infeasible: revisiting v2
}

// ── Domination ──

TEST_CASE("NgPathResource: domination subset check") {
    auto ng = make_ng5();

    // At same vertex, states use same local bit positions.
    uint64_t fewer = 0b0010;   // one neighbor bit set
    uint64_t more  = 0b0110;   // two neighbor bits set (superset)
    uint64_t disjoint = 0b0001;

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

TEST_CASE("NgPathResource: domination after multi-hop extend") {
    auto ng = make_ng5();

    auto s0 = ng.init_state(Direction::Forward);

    // Path A: 0→1→2 (two hops, visits 0 and 1)
    auto [sA1, cA1] = ng.extend(Direction::Forward, s0, 0);  // 0→1
    REQUIRE(cA1 == 0.0);
    auto [sA2, cA2] = ng.extend(Direction::Forward, sA1, 2); // 1→2
    REQUIRE(cA2 == 0.0);

    // Path B: 0→2 (one hop, visits only 0)
    auto [sB1, cB1] = ng.extend(Direction::Forward, s0, 1);  // 0→2
    REQUIRE(cB1 == 0.0);

    // At v2: B has fewer visited vertices → B dominates A
    CHECK(ng.domination_cost(Direction::Forward, 2, sB1, sA2) == 0.0);
    // A does NOT dominate B (A has superset of visited vertices)
    CHECK(ng.domination_cost(Direction::Forward, 2, sA2, sB1) == INF);
}

TEST_CASE("NgPathResource: domination transitivity") {
    auto ng = make_ng5();

    uint64_t s0 = 0b000;   // empty
    uint64_t s1 = 0b010;   // {bit 1}
    uint64_t s2 = 0b110;   // {bit 1, bit 2}

    // s0 ⊆ s1 ⊆ s2 → chain holds
    CHECK(ng.domination_cost(Direction::Forward, 1, s0, s1) == 0.0);
    CHECK(ng.domination_cost(Direction::Forward, 1, s1, s2) == 0.0);
    CHECK(ng.domination_cost(Direction::Forward, 1, s0, s2) == 0.0);

    // Reverse: none holds
    CHECK(ng.domination_cost(Direction::Forward, 1, s2, s1) == INF);
    CHECK(ng.domination_cost(Direction::Forward, 1, s1, s0) == INF);
}

TEST_CASE("NgPathResource: domination with disjoint bits") {
    auto ng = make_ng5();

    uint64_t sa = 0b001;
    uint64_t sb = 0b010;

    // Neither dominates the other
    CHECK(ng.domination_cost(Direction::Forward, 1, sa, sb) == INF);
    CHECK(ng.domination_cost(Direction::Forward, 1, sb, sa) == INF);
}

TEST_CASE("NgPathResource: domination after extend preserves subset") {
    auto ng = make_ng5();

    // Two labels at v1 with s1 ⊆ s2. After extending both through same arc,
    // the subset relationship should hold at the target vertex.
    uint64_t s1_at_v1 = 0ULL;  // no visited vertices
    // v0 in v1's ordering: bit_map[1*5+0]=1 → bit 1 set
    uint64_t s2_at_v1 = 1ULL << ng.bit_map[1 * 5 + 0];  // v0 visited

    CHECK(ng.domination_cost(Direction::Forward, 1, s1_at_v1, s2_at_v1) == 0.0);

    // Extend both through arc 2 (1→2)
    auto [ext1, c1] = ng.extend(Direction::Forward, s1_at_v1, 2);
    REQUIRE(c1 == 0.0);
    auto [ext2, c2] = ng.extend(Direction::Forward, s2_at_v1, 2);
    REQUIRE(c2 == 0.0);

    // ext1 ⊆ ext2 should still hold at v2
    CHECK(ng.domination_cost(Direction::Forward, 2, ext1, ext2) == 0.0);
}

// ── Same-vertex Concatenation ──

TEST_CASE("NgPathResource: concatenation rejects shared visited vertices") {
    auto ng = make_ng5();

    // With source marking, init state is 0, so same-vertex concatenation
    // CAN succeed (no self-bit overlap).
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0b0010, 0b0100) == 0.0);
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0b0010, 0b0010) == INF);
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0b0111, 0b0001) == INF);
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0ULL, 0b1111) == 0.0);
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0ULL, 0ULL) == 0.0);
}

// ── Across-arc Concatenation via extend + concatenation_cost ──

TEST_CASE("NgPathResource: across-arc via extend + concat detects overlap") {
    auto ng = make_ng5();

    // Arc 2: 1→2. Test across-arc concatenation of fw@v1 + bw@v2.

    // fw state at v1 with no vertices visited (fresh from source)
    uint64_t fw_empty = 0ULL;

    // bw state at v2 with no vertices visited
    uint64_t bw_empty = 0ULL;

    // Extend fw through arc 2 (1→2): marks v1 (source) in v2's ordering
    auto [ext, ec] = ng.extend(Direction::Forward, fw_empty, 2);
    REQUIRE(ec == 0.0);

    // No overlap with empty bw → feasible
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, ext, bw_empty) == 0.0);

    // Now make bw have v1 visited in v2's ordering: bit_map[2*5+1]=1
    int8_t v1_in_v2 = ng.bit_map[2 * 5 + 1];
    REQUIRE(v1_in_v2 >= 0);
    uint64_t bw_with_v1 = 1ULL << v1_in_v2;

    // v1 in extended fw and v1 in bw → overlap
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, ext, bw_with_v1) == INF);
}

TEST_CASE("NgPathResource: across-arc extend detects j visited in fw") {
    auto ng = make_ng5();

    // Arc 2: 1→2. If v2 was already visited in fw path, extend should fail.

    // fw state at v1 with v2 marked (v2 ∈ N(v1) at bit_map[1*5+2]=2)
    int8_t v2_in_v1 = ng.bit_map[1 * 5 + 2];
    REQUIRE(v2_in_v1 >= 0);
    uint64_t fw_with_v2 = 1ULL << v2_in_v1;

    // Extend through arc 2 (1→2): check_bit for v2 in v1's set → v2 is set → INF
    auto [ext, ec] = ng.extend(Direction::Forward, fw_with_v2, 2);
    CHECK(ec == INF);
}

TEST_CASE("NgPathResource: across-arc allows disjoint paths") {
    auto ng = make_ng5();

    // Arc 5: 2→3. fw@v2, bw@v3.
    // fw: visited v0, v1, v2 (v0 not in v2's ng-set, so only v1 tracked)
    auto s0 = ng.init_state(Direction::Forward);
    auto [s1, c1] = ng.extend(Direction::Forward, s0, 0);  // 0→1
    REQUIRE(c1 == 0.0);
    auto [s2, c2] = ng.extend(Direction::Forward, s1, 2);  // 1→2
    REQUIRE(c2 == 0.0);

    // bw: visited v4, v3
    auto sb0 = ng.init_state(Direction::Backward);
    auto [sb1, cb1] = ng.extend(Direction::Backward, sb0, 7);  // arc 7: 3→4, bw: v4→v3
    REQUIRE(cb1 == 0.0);

    // Extend fw through arc 5 (2→3): marks v2 (source) in v3's ordering
    auto [ext_fw, ext_cost] = ng.extend(Direction::Forward, s2, 5);
    REQUIRE(ext_cost == 0.0);

    // Check concatenation at v3: extended fw vs bw → paths are 0→1→2→3 and 4→3
    // No vertex visited in both → feasible
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 3, ext_fw, sb1) == 0.0);
}

TEST_CASE("NgPathResource: splice with shared intermediate vertex") {
    auto ng = make_ng5();

    // Across-arc splice on arc 2 (1→2): fw@v1, bw@v2.
    // Both paths visit v1 — should be detected as infeasible.

    // bw at v2 with v1 marked in v2's ordering
    int8_t v1_in_v2 = ng.bit_map[2 * 5 + 1];
    REQUIRE(v1_in_v2 >= 0);
    uint64_t bw_at_v2 = 1ULL << v1_in_v2;

    // Extend fw (empty state) through arc 2 (1→2): marks v1 in v2's ordering
    auto [ext_fw, ext_cost] = ng.extend(Direction::Forward, 0ULL, 2);
    REQUIRE(ext_cost == 0.0);

    // ext_fw has v1 marked, bw has v1 marked → overlap → infeasible
    CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, ext_fw, bw_at_v2) == INF);
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
    // mark mask = 0 (v0 not in v1's empty ng-set), no remaps
    CHECK(s1 == 0ULL);

    auto [s2, c2] = ng.extend(Direction::Forward, s1, 1);
    CHECK(c2 == 0.0);
    CHECK(s2 == 0ULL);

    // Revisit v1 — allowed (no check bits since empty ng-sets)
    auto [s3, c3] = ng.extend(Direction::Forward, s2, 2);
    CHECK(c3 == 0.0);
    CHECK(s3 == 0ULL);
}

// ── ResourcePack with NgPathResource ──

TEST_CASE("ResourcePack<NgPathResource> composes correctly") {
    auto ng = make_ng5();
    auto pack = make_resource_pack(std::move(ng));

    auto states = pack.init_states(Direction::Forward);
    // Source marking: init state is 0
    CHECK(std::get<0>(states) == 0ULL);

    // Extend arc 0 (0→1): marks v0 in v1's ordering
    auto [s1, c1] = pack.extend(Direction::Forward, states, 0);
    CHECK(c1 == 0.0);
    CHECK(std::get<0>(s1) != 0ULL);  // some bits set (v0 marked)

    // Domination: init state (no bits) dominates extended state (has bits)
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

// ════════════════════════════════════════════════════════════════
// End-to-end bidir splice test: across-arc finds path same-vertex misses
// ════════════════════════════════════════════════════════════════

TEST_CASE("Bidir across-arc splice finds optimal path") {
    // Graph: 6 vertices, source=0, sink=5
    // Design: fw reaches v1 (q=25<50) but NOT v3 (q=55>50 stops fw).
    //         bw reaches v3 (q=75>50) but NOT v1 (q=45<50 stops bw).
    //         Same-vertex concat misses 0→1→3→5 because no vertex has both.
    //         Across-arc on arc 1→3 finds it.
    //
    // Arcs:       cost     time
    // 0→1 (0)    -10       25
    // 0→2 (1)     -1        1
    // 1→3 (2)    -10       30    ← crosses midpoint
    // 2→3 (3)     -1        1
    // 2→4 (4)     -1        1
    // 3→5 (5)     -1       25
    // 4→5 (6)     -1        1

    const int N = 7;
    int from[N] = {0, 0, 1, 2, 2, 3, 4};
    int to[N]   = {1, 2, 3, 3, 4, 5, 5};
    double cost[N] = {-10, -1, -10, -1, -1, -1, -1};
    double time_d[N] = {25, 1, 30, 1, 1, 25, 1};
    double tw_lb[6] = {0, 0, 0, 0, 0, 0};
    double tw_ub[6] = {100, 100, 100, 100, 100, 100};

    const double* arc_res[1] = {time_d};
    const double* v_lb[1] = {tw_lb};
    const double* v_ub[1] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 6;
    pv.source = 0;
    pv.sink = 5;
    pv.n_arcs = N;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // ng-neighbors: full elementarity
    std::vector<std::vector<int>> neighbors = {
        {0,1,2,3,4,5}, {1,0,2,3,4,5}, {2,0,1,3,4,5},
        {3,0,1,2,4,5}, {4,0,1,2,3,5}, {5,0,1,2,3,4}
    };

    // Mono: should find 0→1→3→5 (cost = -10 + -10 + -1 = -21)
    NgPathResource ng_m(pv.n_vertices, pv.n_arcs, from, to, neighbors);
    Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_m)),
        {.bucket_steps = {10.0, 1.0}, .tolerance = 1e9});
    mono.build();
    mono.set_stage(Stage::Exact);
    auto mono_paths = mono.solve();
    REQUIRE(!mono_paths.empty());
    double mono_best = mono_paths[0].reduced_cost;
    CHECK(mono_best == doctest::Approx(-21.0));

    // Bidir: should also find -21 via across-arc splice
    NgPathResource ng_b(pv.n_vertices, pv.n_arcs, from, to, neighbors);
    Solver<NgPack> bidir(pv, make_resource_pack(std::move(ng_b)),
        {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .tolerance = 1e9});
    bidir.build();
    bidir.set_stage(Stage::Exact);
    auto bidir_paths = bidir.solve();
    REQUIRE(!bidir_paths.empty());
    double bidir_best = bidir_paths[0].reduced_cost;
    CHECK(bidir_best <= mono_best + 1e-6);
}
