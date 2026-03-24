#include <bgspprc/resource.h>
#include <bgspprc/resources/cumulative_cost.h>
#include <bgspprc/resources/ng_path.h>
#include <bgspprc/resources/pickup_delivery.h>
#include <bgspprc/resources/standard.h>
#include <bgspprc/solver.h>

#include <doctest/doctest.h>

using namespace bgspprc;

// ── StandardResource ──

TEST_CASE("StandardResource init_state forward") {
  //  source=0, sink=2, 3 vertices
  double consumption[] = {1.0, 2.0, 3.0};
  double lb[] = {0.0, 5.0, 10.0};
  double ub[] = {100.0, 50.0, 30.0};

  StandardResource res(consumption, lb, ub, 0, 2, 3);

  CHECK(res.init_state(Direction::Forward) == 0.0);    // lb[source]
  CHECK(res.init_state(Direction::Backward) == 30.0);  // ub[sink]
}

TEST_CASE("StandardResource extend") {
  double consumption[] = {5.0, 3.0};
  double lb[] = {0.0, 0.0, 0.0};
  double ub[] = {100.0, 100.0, 100.0};

  StandardResource res(consumption, lb, ub, 0, 2, 2);

  auto [s1, c1] = res.extend_along_arc(Direction::Forward, 10.0, 0);
  CHECK(s1 == 15.0);
  CHECK(c1 == 0.0);

  auto [s2, c2] = res.extend_along_arc(Direction::Forward, 10.0, 1);
  CHECK(s2 == 13.0);
  CHECK(c2 == 0.0);
}

TEST_CASE("StandardResource extend backward") {
  double consumption[] = {5.0, 3.0};
  double lb[] = {0.0, 0.0, 0.0};
  double ub[] = {100.0, 100.0, 100.0};

  StandardResource res(consumption, lb, ub, 0, 2, 2);

  // Backward extension subtracts consumption
  auto [s1, c1] = res.extend_along_arc(Direction::Backward, 50.0, 0);
  CHECK(s1 == 45.0);
  CHECK(c1 == 0.0);

  auto [s2, c2] = res.extend_along_arc(Direction::Backward, 50.0, 1);
  CHECK(s2 == 47.0);
  CHECK(c2 == 0.0);
}

TEST_CASE("StandardResource extend_to_vertex forward") {
  double consumption[] = {5.0};
  double lb[] = {0.0, 10.0};
  double ub[] = {100.0, 50.0};

  StandardResource res(consumption, lb, ub, 0, 1, 1);

  // Within bounds, no clamping needed
  auto [s1, c1] = res.extend_to_vertex(Direction::Forward, 25.0, 1);
  CHECK(s1 == 25.0);
  CHECK(c1 == 0.0);

  // Below lb → clamp up
  auto [s2, c2] = res.extend_to_vertex(Direction::Forward, 5.0, 1);
  CHECK(s2 == 10.0);
  CHECK(c2 == 0.0);

  // Above ub → infeasible
  auto [s3, c3] = res.extend_to_vertex(Direction::Forward, 60.0, 1);
  CHECK(c3 >= INF);
}

TEST_CASE("StandardResource extend_to_vertex backward") {
  double consumption[] = {5.0};
  double lb[] = {0.0, 10.0};
  double ub[] = {100.0, 50.0};

  StandardResource res(consumption, lb, ub, 0, 1, 1);

  // Within bounds, no clamping needed
  auto [s1, c1] = res.extend_to_vertex(Direction::Backward, 25.0, 1);
  CHECK(s1 == 25.0);
  CHECK(c1 == 0.0);

  // Above ub → clamp down
  auto [s2, c2] = res.extend_to_vertex(Direction::Backward, 60.0, 1);
  CHECK(s2 == 50.0);
  CHECK(c2 == 0.0);

  // Below lb → infeasible
  auto [s3, c3] = res.extend_to_vertex(Direction::Backward, 5.0, 1);
  CHECK(c3 >= INF);
}

// ── ResourcePack ──

TEST_CASE("EmptyPack operations") {
  EmptyPack pack;
  auto states = pack.init_states(Direction::Forward);
  static_assert(std::tuple_size_v<decltype(states)> == 0);

  auto [new_states, cost] =
      pack.extend_along_arc(Direction::Forward, states, 0);
  CHECK(cost == 0.0);

  auto [vtx_states, vtx_cost] =
      pack.extend_to_vertex(Direction::Forward, states, 0);
  CHECK(vtx_cost == 0.0);
}

TEST_CASE("ResourcePack with StandardResource") {
  double consumption[] = {2.0, 3.0};
  double lb[] = {0.0, 0.0, 0.0};
  double ub[] = {100.0, 100.0, 100.0};

  auto pack =
      make_resource_pack(StandardResource(consumption, lb, ub, 0, 2, 2));

  auto states = pack.init_states(Direction::Forward);
  CHECK(std::get<0>(states) == 0.0);

  auto [new_states, cost] =
      pack.extend_along_arc(Direction::Forward, states, 0);
  CHECK(std::get<0>(new_states) == 2.0);
  CHECK(cost == 0.0);

  auto [vtx_states, vtx_cost] =
      pack.extend_to_vertex(Direction::Forward, new_states, 1);
  CHECK(std::get<0>(vtx_states) == 2.0);  // within bounds, no clamp
  CHECK(vtx_cost == 0.0);

  // StandardResource is conservatively non-symmetric
  CHECK(pack.symmetric() == false);
}

// ════════════════════════════════════════════════════════════════
// NgPathResource — local bit mapping tests (DESTINATION MARKING)
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
  static int to[] = {1, 2, 2, 3, 4, 3, 4, 4, 1, 1};
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
  CHECK(ng.bit_map[0 * 5 + 0] == 0);   // 0 in v0's set at pos 0
  CHECK(ng.bit_map[0 * 5 + 1] == 1);   // 1 in v0's set at pos 1
  CHECK(ng.bit_map[0 * 5 + 2] == 2);   // 2 in v0's set at pos 2
  CHECK(ng.bit_map[0 * 5 + 3] == -1);  // 3 not in v0's set
  CHECK(ng.bit_map[0 * 5 + 4] == -1);  // 4 not in v0's set

  // v2's neighbors are {2,1,3} → bit positions 0,1,2
  CHECK(ng.bit_map[2 * 5 + 2] == 0);   // 2 at pos 0
  CHECK(ng.bit_map[2 * 5 + 1] == 1);   // 1 at pos 1
  CHECK(ng.bit_map[2 * 5 + 3] == 2);   // 3 at pos 2
  CHECK(ng.bit_map[2 * 5 + 0] == -1);  // 0 not in v2's set
}

TEST_CASE("NgPathResource: self_bit is at position k (number of neighbors)") {
  auto ng = make_ng5();

  // Each vertex has 3 ng-neighbors → self_bit = 3
  for (int v = 0; v < 5; ++v) CHECK(ng.self_bit_[v] == 3);
}

TEST_CASE("NgPathResource: init_state returns zero") {
  auto ng = make_ng5();

  CHECK(ng.init_state(Direction::Forward) == 0ULL);
  CHECK(ng.init_state(Direction::Backward) == 0ULL);
}

TEST_CASE("NgPathResource: extend_to_vertex sets self bit") {
  auto ng = make_ng5();

  auto [s, c] = ng.extend_to_vertex(Direction::Forward, 0ULL, 2);
  CHECK(c == 0.0);
  // v2's self_bit_ = 3, and v2 ∈ N(v2) at position 0 → both bits set
  CHECK(s == ((1ULL << 3) | (1ULL << 0)));

  // Idempotent
  auto [s2, c2] = ng.extend_to_vertex(Direction::Forward, s, 2);
  CHECK(s2 == s);
}

TEST_CASE(
    "NgPathResource: forward extend remaps via shift_pairs including "
    "self-bit") {
  auto ng = make_ng5();

  // Start at v0 with self bit set (as BucketGraph would do)
  auto [s0, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);
  // s0 = 1<<3 (v0's self bit)

  // Extend arc 0: 0→1. The self-bit shift pair maps
  // (self_bit_[0]=3, bit_map[1*5+0]=1) → bit 1 at v1.
  // This replaces the old mark_mask.
  auto [s1, c1] = ng.extend_along_arc(Direction::Forward, s0, 0);
  CHECK(c1 == 0.0);

  // v0 should be mapped to v1's ordering
  int8_t v0_in_v1 = ng.bit_map[1 * 5 + 0];
  CHECK(v0_in_v1 >= 0);
  CHECK((s1 & (1ULL << v0_in_v1)) != 0);

  // Self bit for v0 should NOT be present (it was remapped)
  // But v1's self bit should NOT be set either (extend doesn't mark
  // destination)
  CHECK((s1 & (1ULL << ng.self_bit_[1])) == 0);
}

TEST_CASE("NgPathResource: extend + extend_to_vertex compose correctly") {
  auto ng = make_ng5();

  // Simulate what BucketGraph does: init → extend_to_vertex(source) → extend →
  // extend_to_vertex(target)
  auto [s0, _] =
      ng.extend_to_vertex(Direction::Forward, 0ULL, 0);  // mark source v0
  auto [s1_arc, c1] =
      ng.extend_along_arc(Direction::Forward, s0, 0);  // arc 0→1
  REQUIRE(c1 == 0.0);
  auto [s1, __] =
      ng.extend_to_vertex(Direction::Forward, s1_arc, 1);  // mark v1

  // v0 should be mapped in v1's ordering
  int8_t v0_in_v1 = ng.bit_map[1 * 5 + 0];
  CHECK(v0_in_v1 >= 0);
  CHECK((s1 & (1ULL << v0_in_v1)) != 0);

  // v1's self bit should be set
  CHECK((s1 & (1ULL << ng.self_bit_[1])) != 0);
}

TEST_CASE("NgPathResource: forward extend transforms bits across vertices") {
  auto ng = make_ng5();

  auto [s0, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);

  // Extend 0→1 (arc 0), mark v1, then 1→2 (arc 2)
  auto [s1_arc, c1] = ng.extend_along_arc(Direction::Forward, s0, 0);
  REQUIRE(c1 == 0.0);
  auto [s1, __] = ng.extend_to_vertex(Direction::Forward, s1_arc, 1);

  auto [s2_arc, c2] = ng.extend_along_arc(Direction::Forward, s1, 2);
  CHECK(c2 == 0.0);

  // At v2 (after arc, before extend_to_vertex): v1 should be mapped
  // v1 ∈ N(v2) at bit_map[2*5+1]=1
  int8_t v1_in_v2 = ng.bit_map[2 * 5 + 1];
  CHECK(v1_in_v2 >= 0);
  CHECK((s2_arc & (1ULL << v1_in_v2)) != 0);  // v1 mapped

  // v0 was at v1 (bit_map[1*5+0]=1). Is v0 ∈ N(v2)? bit_map[2*5+0] = -1. No
  // remap.
  int8_t v0_in_v2 = ng.bit_map[2 * 5 + 0];
  CHECK(v0_in_v2 == -1);
}

TEST_CASE("NgPathResource: forward revisit detection") {
  auto ng = make_ng5();

  auto [s0, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);

  // Path: 0→1 (arc 0), 1→2 (arc 2), 2→1 (arc 8) should be infeasible.
  auto [s1_arc, c1] = ng.extend_along_arc(Direction::Forward, s0, 0);
  REQUIRE(c1 == 0.0);
  auto [s1, __] = ng.extend_to_vertex(Direction::Forward, s1_arc, 1);

  auto [s2_arc, c2] = ng.extend_along_arc(Direction::Forward, s1, 2);
  REQUIRE(c2 == 0.0);
  auto [s2, ___] = ng.extend_to_vertex(Direction::Forward, s2_arc, 2);

  // Try 2→1 (arc 8). v1 is in v2's ng-set. v1 was visited (its bit is set
  // in v2's ordering from the remap). The self-bit shift pair for arc 1→2
  // mapped self_bit_[1] → v1's position in v2. So check detects v1 visited.
  auto [s3, c3] = ng.extend_along_arc(Direction::Forward, s2, 8);
  CHECK(c3 == INF);  // infeasible: revisiting v1
}

TEST_CASE("NgPathResource: visit outside ng-set is allowed") {
  auto ng = make_ng5();

  auto [s0, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);

  // Path: 0→1 (arc 0), 1→3 (arc 3), 3→1 (arc 9)
  auto [s1_arc, c1] = ng.extend_along_arc(Direction::Forward, s0, 0);
  REQUIRE(c1 == 0.0);
  auto [s1, __] = ng.extend_to_vertex(Direction::Forward, s1_arc, 1);

  // Arc 3: 1→3. v3 not in v1's ng-set → check_bit = -1 → no block.
  int8_t v3_in_v1 = ng.bit_map[1 * 5 + 3];
  CHECK(v3_in_v1 == -1);  // v3 NOT in v1's ng-set

  auto [s2_arc, c2] = ng.extend_along_arc(Direction::Forward, s1, 3);
  CHECK(c2 == 0.0);  // feasible
  auto [s2, ___] = ng.extend_to_vertex(Direction::Forward, s2_arc, 3);

  // At v3: v1 should be mapped (self-bit shift pair: self_bit_[1] →
  // bit_map[3*5+1])
  int8_t v1_in_v3 = ng.bit_map[3 * 5 + 1];
  CHECK(v1_in_v3 >= 0);
  CHECK((s2 & (1ULL << v1_in_v3)) != 0);  // v1 mapped at v3

  // Arc 9: 3→1. v1 is in v3's ng-set and v1 was visited → infeasible.
  auto [s3, c3] = ng.extend_along_arc(Direction::Forward, s2, 9);
  CHECK(c3 == INF);  // infeasible: v1 forbidden at v3
}

TEST_CASE("NgPathResource: backward extend") {
  auto ng = make_ng5();

  auto [s0, _] =
      ng.extend_to_vertex(Direction::Backward, 0ULL, 4);  // mark sink v4

  // Arc 7: 3→4 (from=3, to=4). Backward: label at v4, extending to v3.
  auto [s1_arc, c1] = ng.extend_along_arc(Direction::Backward, s0, 7);
  CHECK(c1 == 0.0);
  auto [s1, __] =
      ng.extend_to_vertex(Direction::Backward, s1_arc, 3);  // mark v3

  // v4's self bit was set. Self-bit shift pair for arc 7 backward:
  // (self_bit_[4]=3, bit_map[3*5+4]). v4 ∉ N(v3) → no shift pair added.
  // So v4 is NOT tracked at v3 (v4 not in v3's ng-set).

  // Extend arc 5: 2→3. Backward: label at v3, extending to v2.
  auto [s2_arc, c2] = ng.extend_along_arc(Direction::Backward, s1, 5);
  CHECK(c2 == 0.0);

  // At v2: v3 should be mapped (self-bit shift pair: self_bit_[3] →
  // bit_map[2*5+3])
  int8_t v3_in_v2 = ng.bit_map[2 * 5 + 3];
  CHECK(v3_in_v2 >= 0);
  CHECK((s2_arc & (1ULL << v3_in_v2)) != 0);
}

TEST_CASE("NgPathResource: backward revisit detection") {
  auto ng = make_ng5();

  auto [s0, _] = ng.extend_to_vertex(Direction::Backward, 0ULL, 4);  // mark v4

  // Start fresh: backward on arc 6 (from=2, to=4): at v4, go to v2.
  auto [sb1_arc, cb1] = ng.extend_along_arc(Direction::Backward, s0, 6);
  REQUIRE(cb1 == 0.0);
  auto [sb1, __] =
      ng.extend_to_vertex(Direction::Backward, sb1_arc, 2);  // mark v2

  // Now at v2. Backward on arc 2 (from=1, to=2): at v2, go to v1.
  auto [sb2_arc, cb2] = ng.extend_along_arc(Direction::Backward, sb1, 2);
  REQUIRE(cb2 == 0.0);
  auto [sb2, ___] =
      ng.extend_to_vertex(Direction::Backward, sb2_arc, 1);  // mark v1

  // At v1: v2 should be mapped (self-bit shift pair: self_bit_[2] →
  // bit_map[1*5+2])
  int8_t v2_in_v1 = ng.bit_map[1 * 5 + 2];
  CHECK(v2_in_v1 >= 0);
  CHECK((sb2 & (1ULL << v2_in_v1)) != 0);

  // Try backward on arc 8 (from=2, to=1): at v1, go to v2 (revisit!).
  // bw_check_bit for arc 8 = bit_map[to=1][from=2] = v2 in v1's set.
  // v2 is mapped → should be infeasible.
  auto [sb3, cb3] = ng.extend_along_arc(Direction::Backward, sb2, 8);
  CHECK(cb3 == INF);  // infeasible: revisiting v2
}

// ── Domination ──

TEST_CASE("NgPathResource: domination subset check") {
  auto ng = make_ng5();

  // At same vertex, states use same local bit positions.
  uint64_t fewer = 0b0010;  // one neighbor bit set
  uint64_t more = 0b0110;   // two neighbor bits set (superset)
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

  // Path A: 0→1→2 (two hops, visits 0 and 1)
  auto [sA0, _a0] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);
  auto [sA1_arc, cA1] = ng.extend_along_arc(Direction::Forward, sA0, 0);  // 0→1
  REQUIRE(cA1 == 0.0);
  auto [sA1, _a1] = ng.extend_to_vertex(Direction::Forward, sA1_arc, 1);
  auto [sA2_arc, cA2] = ng.extend_along_arc(Direction::Forward, sA1, 2);  // 1→2
  REQUIRE(cA2 == 0.0);
  auto [sA2, _a2] = ng.extend_to_vertex(Direction::Forward, sA2_arc, 2);

  // Path B: 0→2 (one hop, visits only 0)
  auto [sB0, _b0] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);
  auto [sB1_arc, cB1] = ng.extend_along_arc(Direction::Forward, sB0, 1);  // 0→2
  REQUIRE(cB1 == 0.0);
  auto [sB1, _b1] = ng.extend_to_vertex(Direction::Forward, sB1_arc, 2);

  // At v2: both have self bit set. B has fewer other visited vertices → B
  // dominates A
  CHECK(ng.domination_cost(Direction::Forward, 2, sB1, sA2) == 0.0);
  // A does NOT dominate B (A has superset of visited vertices)
  CHECK(ng.domination_cost(Direction::Forward, 2, sA2, sB1) == INF);
}

TEST_CASE("NgPathResource: domination transitivity") {
  auto ng = make_ng5();

  uint64_t s0 = 0b000;  // empty
  uint64_t s1 = 0b010;  // {bit 1}
  uint64_t s2 = 0b110;  // {bit 1, bit 2}

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
  // Both have self bit for v1 set (as they're at v1)
  uint64_t self1 = 1ULL << ng.self_bit_[1];
  uint64_t s1_at_v1 = self1;  // just self bit
  // v0 in v1's ordering: bit_map[1*5+0]=1 → bit 1 set
  uint64_t s2_at_v1 =
      self1 | (1ULL << ng.bit_map[1 * 5 + 0]);  // self + v0 visited

  CHECK(ng.domination_cost(Direction::Forward, 1, s1_at_v1, s2_at_v1) == 0.0);

  // Extend both through arc 2 (1→2)
  auto [ext1, c1] = ng.extend_along_arc(Direction::Forward, s1_at_v1, 2);
  REQUIRE(c1 == 0.0);
  auto [ext2, c2] = ng.extend_along_arc(Direction::Forward, s2_at_v1, 2);
  REQUIRE(c2 == 0.0);

  // ext1 ⊆ ext2 should still hold at v2
  CHECK(ng.domination_cost(Direction::Forward, 2, ext1, ext2) == 0.0);
}

// ── Concatenation ──

TEST_CASE("NgPathResource: concatenation rejects shared visited vertices") {
  auto ng = make_ng5();

  // Disjoint bits → ok
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0b0010, 0b0100) == 0.0);
  // Overlapping → cycle
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0b0010, 0b0010) == INF);
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0b0111, 0b0001) == INF);
  // Empty fw → ok
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0ULL, 0b1111) == 0.0);
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, 0ULL, 0ULL) == 0.0);
}

// ── Across-arc Concatenation via extend + concatenation_cost ──

TEST_CASE("NgPathResource: across-arc via extend + concat detects overlap") {
  auto ng = make_ng5();

  // Arc 2: 1→2. Test across-arc concatenation of fw@v1 + bw@v2.

  // fw state at v1 with self bit set (fresh from source, extended to v1)
  auto [fw_at_v1, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 1);

  // bw state at v2 with self bit set
  auto [bw_at_v2, __] = ng.extend_to_vertex(Direction::Backward, 0ULL, 2);

  // Extend fw through arc 2 (1→2): remaps v1's self bit → v1's pos in v2's
  // ordering Does NOT set v2's self bit (that's extend_to_vertex's job, not
  // done for concatenation)
  auto [ext, ec] = ng.extend_along_arc(Direction::Forward, fw_at_v1, 2);
  REQUIRE(ec == 0.0);

  // ext has v1 mapped in v2's ordering, but NOT v2's self bit.
  // bw has v2's self bit set. No overlap → feasible.
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, ext, bw_at_v2) == 0.0);

  // Now make bw have v1 visited in v2's ordering too
  int8_t v1_in_v2 = ng.bit_map[2 * 5 + 1];
  REQUIRE(v1_in_v2 >= 0);
  uint64_t bw_with_v1 = bw_at_v2 | (1ULL << v1_in_v2);

  // v1 in extended fw and v1 in bw → overlap → cycle
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, ext, bw_with_v1) == INF);
}

TEST_CASE("NgPathResource: across-arc extend detects j visited in fw") {
  auto ng = make_ng5();

  // Arc 2: 1→2. If v2 was already visited in fw path, extend should fail.

  // fw state at v1 with v2 marked (v2 ∈ N(v1) at bit_map[1*5+2]=2)
  int8_t v2_in_v1 = ng.bit_map[1 * 5 + 2];
  REQUIRE(v2_in_v1 >= 0);
  uint64_t fw_with_v2 = (1ULL << ng.self_bit_[1]) | (1ULL << v2_in_v1);

  // Extend through arc 2 (1→2): check_bit for v2 in v1's set → v2 is set → INF
  auto [ext, ec] = ng.extend_along_arc(Direction::Forward, fw_with_v2, 2);
  CHECK(ec == INF);
}

TEST_CASE("NgPathResource: across-arc allows disjoint paths") {
  auto ng = make_ng5();

  // Arc 5: 2→3. fw@v2, bw@v3.
  // fw: visited v0, v1, v2
  auto [s0, _0] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);
  auto [s1_arc, c1] = ng.extend_along_arc(Direction::Forward, s0, 0);  // 0→1
  REQUIRE(c1 == 0.0);
  auto [s1, _1] = ng.extend_to_vertex(Direction::Forward, s1_arc, 1);
  auto [s2_arc, c2] = ng.extend_along_arc(Direction::Forward, s1, 2);  // 1→2
  REQUIRE(c2 == 0.0);
  auto [s2, _2] = ng.extend_to_vertex(Direction::Forward, s2_arc, 2);

  // bw: visited v4, v3
  auto [sb0, _b0] = ng.extend_to_vertex(Direction::Backward, 0ULL, 4);
  auto [sb1_arc, cb1] = ng.extend_along_arc(Direction::Backward, sb0,
                                            7);  // arc 7: 3→4, bw: v4→v3
  REQUIRE(cb1 == 0.0);
  auto [sb1, _b1] = ng.extend_to_vertex(Direction::Backward, sb1_arc, 3);

  // Extend fw through arc 5 (2→3): maps v2's self bit to v3's ordering
  auto [ext_fw, ext_cost] = ng.extend_along_arc(Direction::Forward, s2, 5);
  REQUIRE(ext_cost == 0.0);

  // Check concatenation at v3: extended fw vs bw → paths are 0→1→2→3 and 4→3
  // ext_fw does NOT have v3's self bit. bw has v3's self bit.
  // Other vertices: v2 mapped in v3's ordering by fw. v4 in bw but v4 ∉ N(v3)
  // so not tracked. No vertex visited in both → feasible
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 3, ext_fw, sb1) == 0.0);
}

TEST_CASE("NgPathResource: splice with shared intermediate vertex") {
  auto ng = make_ng5();

  // Across-arc splice on arc 2 (1→2): fw@v1, bw@v2.
  // Both paths visit v1 — should be detected as infeasible.

  // fw at v1 with self bit set
  auto [fw_at_v1, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 1);

  // bw at v2 with v1 marked in v2's ordering (v1 visited in bw path)
  auto [bw_base, __] = ng.extend_to_vertex(Direction::Backward, 0ULL, 2);
  int8_t v1_in_v2 = ng.bit_map[2 * 5 + 1];
  REQUIRE(v1_in_v2 >= 0);
  uint64_t bw_at_v2 = bw_base | (1ULL << v1_in_v2);

  // Extend fw through arc 2 (1→2): maps v1's self bit → v1's pos in v2's
  // ordering
  auto [ext_fw, ext_cost] =
      ng.extend_along_arc(Direction::Forward, fw_at_v1, 2);
  REQUIRE(ext_cost == 0.0);

  // ext_fw has v1 mapped, bw has v1 marked → overlap → infeasible
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, ext_fw, bw_at_v2) ==
        INF);
}

// ── Empty ng-neighbors degrades to no enforcement ──

TEST_CASE("NgPathResource: empty neighbors = no elementarity") {
  int from[] = {0, 1, 2, 1};
  int to[] = {1, 2, 1, 0};
  std::vector<std::vector<int>> neighbors(3);  // all empty

  NgPathResource ng(3, 4, from, to, neighbors);

  auto s0 = ng.init_state(Direction::Forward);
  CHECK(s0 == 0ULL);

  // self_bit_ for each vertex is 0 (no neighbors)
  CHECK(ng.self_bit_[0] == 0);

  // extend_to_vertex sets bit 0 (self bit)
  auto [sv, _] = ng.extend_to_vertex(Direction::Forward, s0, 0);
  CHECK(sv == 1ULL);

  // Extend 0→1→2→1 — no cycle block because no ng-neighbors
  auto [s1_arc, c1] = ng.extend_along_arc(Direction::Forward, sv, 0);
  CHECK(c1 == 0.0);
  auto [s1, __] = ng.extend_to_vertex(Direction::Forward, s1_arc, 1);

  auto [s2_arc, c2] = ng.extend_along_arc(Direction::Forward, s1, 1);
  CHECK(c2 == 0.0);
  auto [s2, ___] = ng.extend_to_vertex(Direction::Forward, s2_arc, 2);

  // Revisit v1 — allowed (no check bits since empty ng-sets)
  auto [s3_arc, c3] = ng.extend_along_arc(Direction::Forward, s2, 2);
  CHECK(c3 == 0.0);
}

// ── ResourcePack with NgPathResource ──

TEST_CASE("ResourcePack<NgPathResource> composes correctly") {
  auto ng = make_ng5();
  auto pack = make_resource_pack(std::move(ng));

  auto states = pack.init_states(Direction::Forward);
  CHECK(std::get<0>(states) == 0ULL);

  // extend_to_vertex to mark source
  auto [vtx_states, vtx_cost] =
      pack.extend_to_vertex(Direction::Forward, states, 0);
  CHECK(vtx_cost == 0.0);
  CHECK(std::get<0>(vtx_states) != 0ULL);  // self bit set

  // Extend arc 0 (0→1): remaps v0's self bit
  auto [s1, c1] = pack.extend_along_arc(Direction::Forward, vtx_states, 0);
  CHECK(c1 == 0.0);
  CHECK(std::get<0>(s1) != 0ULL);  // v0 mapped

  // Domination: init state (no bits) dominates extended state (has bits)
  CHECK(pack.domination_cost(Direction::Forward, 1, states, s1) == 0.0);
  CHECK(pack.domination_cost(Direction::Forward, 1, s1, states) == INF);
}

// ════════════════════════════════════════════════════════════════
// Concatenation invariant: extend_along_arc does NOT set destination self bit
// ════════════════════════════════════════════════════════════════

TEST_CASE("NgPathResource: extend_along_arc omits destination self bit") {
  auto ng = make_ng5();

  // Label at v1 with self bit set
  auto [s1, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 1);
  CHECK((s1 & (1ULL << ng.self_bit_[1])) != 0);

  // Extend arc 2: 1→2. After extend_along_arc, state is at v2's ordering
  // but v2's self bit must NOT be set.
  auto [s2_arc, c] = ng.extend_along_arc(Direction::Forward, s1, 2);
  REQUIRE(c == 0.0);
  CHECK((s2_arc & (1ULL << ng.self_bit_[2])) == 0);  // no self bit for v2

  // After extend_to_vertex, it IS set.
  auto [s2, __] = ng.extend_to_vertex(Direction::Forward, s2_arc, 2);
  CHECK((s2 & (1ULL << ng.self_bit_[2])) != 0);
}

TEST_CASE(
    "NgPathResource: concatenation correctness — no double-count at splice "
    "vertex") {
  auto ng = make_ng5();

  // Forward label at v2 with visited {v0, v1, v2}
  auto [fw0, _0] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);
  auto [fw1_arc, c1] = ng.extend_along_arc(Direction::Forward, fw0, 0);  // 0→1
  REQUIRE(c1 == 0.0);
  auto [fw1, _1] = ng.extend_to_vertex(Direction::Forward, fw1_arc, 1);
  auto [fw2_arc, c2] = ng.extend_along_arc(Direction::Forward, fw1, 2);  // 1→2
  REQUIRE(c2 == 0.0);
  auto [fw2, _2] = ng.extend_to_vertex(Direction::Forward, fw2_arc, 2);

  // Backward label at v3 with visited {v4, v3}
  auto [bw0, _b0] = ng.extend_to_vertex(Direction::Backward, 0ULL, 4);
  auto [bw1_arc, cb1] =
      ng.extend_along_arc(Direction::Backward, bw0, 7);  // 3→4 bw
  REQUIRE(cb1 == 0.0);
  auto [bw1, _b1] = ng.extend_to_vertex(Direction::Backward, bw1_arc, 3);

  // Across-arc splice on arc 5 (2→3):
  // extend_along_arc fw@v2 through arc → state at v3's ordering, NO v3 self bit
  auto [ext, ec] = ng.extend_along_arc(Direction::Forward, fw2, 5);
  REQUIRE(ec == 0.0);
  CHECK((ext & (1ULL << ng.self_bit_[3])) == 0);  // no self bit for v3

  // bw@v3 HAS v3's self bit. No overlap at v3 → feasible concatenation.
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 3, ext, bw1) == 0.0);
}

// ════════════════════════════════════════════════════════════════
// Self-bit loss when from ∉ N(to)
// ════════════════════════════════════════════════════════════════

TEST_CASE("NgPathResource: self-bit lost when vertex not in target ng-set") {
  auto ng = make_ng5();
  // v4's ng-neighbors are {4, 3, 2}. v0 ∉ N(v4), v1 ∉ N(v4).
  // v0's ng-neighbors are {0, 1, 2}. v4 ∉ N(v0).

  // Build a custom graph to test: vertex with no relationship to target.
  // Use make_ng5: v0's neighbors = {0,1,2}. v4's neighbors = {4,3,2}.
  // Arc 4: 1→4.
  // v1's self_bit_ = 3. Is v1 ∈ N(v4)? bit_map[4*5+1] should be -1.
  CHECK(ng.bit_map[4 * 5 + 1] == -1);  // v1 ∉ N(v4)

  // Label at v1 with self bit set
  auto [s1, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 1);
  CHECK((s1 & (1ULL << ng.self_bit_[1])) != 0);

  // Extend arc 4 (1→4). v1 ∉ N(v4), so no self-bit shift pair was added.
  // v1's self bit should be LOST at v4.
  auto [s4_arc, c] = ng.extend_along_arc(Direction::Forward, s1, 4);
  REQUIRE(c == 0.0);

  // v1 is not tracked at v4 (not in v4's ng-set)
  CHECK(ng.bit_map[4 * 5 + 1] == -1);
  // State should have no trace of v1 (self bit was not remapped)
  // Only bits that could be set are for vertices in both N(v1) and N(v4).
  // Common: v2 ∈ N(v1) at pos 2, v2 ∈ N(v4) at pos 2. But v2 wasn't visited.
  // So state should be 0.
  CHECK(s4_arc == 0ULL);
}

// ════════════════════════════════════════════════════════════════
// Multi-hop backward composition
// ════════════════════════════════════════════════════════════════

TEST_CASE("NgPathResource: multi-hop backward tracks all visited vertices") {
  auto ng = make_ng5();

  // Backward path: v4 → v3 → v2 → v1 (arcs 7, 5, 8 in reverse)
  auto [s0, _0] = ng.extend_to_vertex(Direction::Backward, 0ULL, 4);

  // Arc 7: 3→4. Backward: v4→v3.
  auto [s1_arc, c1] = ng.extend_along_arc(Direction::Backward, s0, 7);
  REQUIRE(c1 == 0.0);
  auto [s1, _1] = ng.extend_to_vertex(Direction::Backward, s1_arc, 3);

  // v4 ∉ N(v3) = {3,2,1}, so v4's self bit lost. But v3's self bit set.
  CHECK((s1 & (1ULL << ng.self_bit_[3])) != 0);

  // Arc 5: 2→3. Backward: v3→v2.
  auto [s2_arc, c2] = ng.extend_along_arc(Direction::Backward, s1, 5);
  REQUIRE(c2 == 0.0);
  auto [s2, _2] = ng.extend_to_vertex(Direction::Backward, s2_arc, 2);

  // At v2: v3 should be tracked (v3 ∈ N(v2)={2,1,3} at pos 2)
  int8_t v3_in_v2 = ng.bit_map[2 * 5 + 3];
  CHECK(v3_in_v2 >= 0);
  CHECK((s2 & (1ULL << v3_in_v2)) != 0);
  // v2's self bit set
  CHECK((s2 & (1ULL << ng.self_bit_[2])) != 0);

  // Arc 8: 2→1. Backward: v2→v1. But wait — bw_check_bit for arc 8 is
  // bit_map[to=1][from=2] = bit of v2 in v1's ng-set.
  // v2 ∈ N(v2) is at v2's ordering. But we need v2 at the CURRENT label's
  // vertex (v2). bw for arc 8: label is at to=1? No — backward on arc 8
  // (from=2, to=1) means label at to=1, extending to from=2. That's v1→v2, not
  // v2→v1. For v2→v1 backward we need an arc where from=1, to=2 traversed
  // backward. That's arc 2 (1→2), traversed backward = v2→v1.

  auto [s3_arc, c3] = ng.extend_along_arc(Direction::Backward, s2, 2);
  REQUIRE(c3 == 0.0);
  auto [s3, _3] = ng.extend_to_vertex(Direction::Backward, s3_arc, 1);

  // At v1: v2 should be tracked (v2 ∈ N(v1)={1,0,2} at pos 2)
  int8_t v2_in_v1 = ng.bit_map[1 * 5 + 2];
  CHECK(v2_in_v1 >= 0);
  CHECK((s3 & (1ULL << v2_in_v1)) != 0);

  // v3 was at v2 (pos 2). Is v3 ∈ N(v1)? bit_map[1*5+3] = -1. Not tracked.
  CHECK(ng.bit_map[1 * 5 + 3] == -1);

  // v1's self bit set
  CHECK((s3 & (1ULL << ng.self_bit_[1])) != 0);

  // Now try to revisit v2: backward on arc 8 (from=2, to=1) = v1→v2.
  // bw_check_bit = bit_map[1*5+2] = v2 in v1's ng-set. v2 is marked →
  // infeasible.
  auto [s4, c4] = ng.extend_along_arc(Direction::Backward, s3, 8);
  CHECK(c4 == INF);
}

// ════════════════════════════════════════════════════════════════
// Mixed ResourcePack: StandardResource + NgPathResource
// ════════════════════════════════════════════════════════════════

TEST_CASE("Mixed ResourcePack extend_to_vertex composes correctly") {
  double consumption[] = {2.0, 3.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  double lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double ub[] = {100.0, 100.0, 100.0, 100.0, 100.0};

  auto ng = make_ng5();
  auto pack = make_resource_pack(
      StandardResource(consumption, lb, ub, 0, 4, 10), std::move(ng));

  auto states = pack.init_states(Direction::Forward);
  CHECK(std::get<0>(states) == 0.0);   // StandardResource: lb[source]
  CHECK(std::get<1>(states) == 0ULL);  // NgPath: empty

  // extend_to_vertex: StandardResource is no-op, NgPath sets self bit
  auto [vtx_states, vtx_cost] =
      pack.extend_to_vertex(Direction::Forward, states, 0);
  CHECK(vtx_cost == 0.0);
  CHECK(std::get<0>(vtx_states) == 0.0);   // unchanged
  CHECK(std::get<1>(vtx_states) != 0ULL);  // self bit set

  // extend_along_arc: StandardResource adds consumption, NgPath remaps
  auto [arc_states, arc_cost] =
      pack.extend_along_arc(Direction::Forward, vtx_states, 0);
  CHECK(arc_cost == 0.0);
  CHECK(std::get<0>(arc_states) == 2.0);   // q += consumption[0]
  CHECK(std::get<1>(arc_states) != 0ULL);  // v0 remapped to v1's ordering

  // extend_to_vertex at v1
  auto [v1_states, v1_cost] =
      pack.extend_to_vertex(Direction::Forward, arc_states, 1);
  CHECK(v1_cost == 0.0);
  CHECK(std::get<0>(v1_states) == 2.0);  // still unchanged
  // NgPath: v1's self bit now set
  auto ng_state = std::get<1>(v1_states);
  NgPathResource ng2 = make_ng5();
  CHECK((ng_state & (1ULL << ng2.self_bit_[1])) != 0);
}

// ════════════════════════════════════════════════════════════════
// NgPathResource — Edge-case tests for (arc type, neighborhood config, state)
// ════════════════════════════════════════════════════════════════

TEST_CASE("NgPathResource: extend_to_vertex v not in own ng-set sets only self "
          "bit") {
  // 3 vertices, neighbors[1] = {0, 2} — v1 NOT in its own ng-set
  int from[] = {0, 1};
  int to[] = {1, 2};
  std::vector<std::vector<int>> neighbors = {{0}, {0, 2}, {2}};
  NgPathResource ng(3, 2, from, to, neighbors);

  CHECK(ng.bit_map[1 * 3 + 0] == 0);   // v0 in N(v1) at pos 0
  CHECK(ng.bit_map[1 * 3 + 2] == 1);   // v2 in N(v1) at pos 1
  CHECK(ng.bit_map[1 * 3 + 1] == -1);  // v1 NOT in N(v1)
  CHECK(ng.self_bit_[1] == 2);         // 2 neighbors → self bit at pos 2

  auto [s, c] = ng.extend_to_vertex(Direction::Forward, 0ULL, 1);
  CHECK(c == 0.0);
  // Only self bit set — no neighbor-self bit since v1 ∉ N(v1)
  CHECK(s == (1ULL << 2));
}

TEST_CASE(
    "NgPathResource: self-loop blocked when v is in own ng-set") {
  // 3 vertices, neighbors[1] = {0, 1, 2}, self-loop arc 1→1
  int from[] = {0, 1, 1};
  int to[] = {1, 1, 2};
  std::vector<std::vector<int>> neighbors = {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}};
  NgPathResource ng(3, 3, from, to, neighbors);

  CHECK(ng.bit_map[1 * 3 + 1] == 1);  // v1 in N(v1) at pos 1
  CHECK(ng.self_bit_[1] == 3);        // 3 neighbors → self bit at 3

  // extend_to_vertex sets self bit + neighbor-self bit
  auto [s, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 1);
  CHECK(s == ((1ULL << 3) | (1ULL << 1)));  // self bit + neighbor-self bit

  // Self-loop arc (index 1): fw_check_bit = bit_map[1*3+1] = 1, which is set
  auto [s2, c] = ng.extend_along_arc(Direction::Forward, s, 1);
  CHECK(c == INF);
}

TEST_CASE(
    "NgPathResource: self-loop allowed when v not in own ng-set") {
  // 3 vertices, neighbors[1] = {0, 2} (v1 NOT in own ng-set), self-loop 1→1
  int from[] = {0, 1, 1};
  int to[] = {1, 1, 2};
  std::vector<std::vector<int>> neighbors = {{0}, {0, 2}, {2}};
  NgPathResource ng(3, 3, from, to, neighbors);

  CHECK(ng.bit_map[1 * 3 + 1] == -1);  // v1 NOT in N(v1)
  CHECK(ng.self_bit_[1] == 2);

  auto [s, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 1);
  CHECK(s == (1ULL << 2));  // only self bit

  // Self-loop arc (index 1): fw_check_bit = -1 → no check → pass
  auto [s2, c] = ng.extend_along_arc(Direction::Forward, s, 1);
  CHECK(c == 0.0);
}

TEST_CASE(
    "NgPathResource: backward self-loop blocked when v is in own ng-set") {
  // Same graph as "self-loop blocked" test but backward direction
  int from[] = {0, 1, 1};
  int to[] = {1, 1, 2};
  std::vector<std::vector<int>> neighbors = {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}};
  NgPathResource ng(3, 3, from, to, neighbors);

  auto [s, _] = ng.extend_to_vertex(Direction::Backward, 0ULL, 1);
  CHECK(s == ((1ULL << 3) | (1ULL << 1)));

  // bw_check_bit for self-loop arc 1→1 = bit_map[1*3+1] = 1, which is set
  auto [s2, c] = ng.extend_along_arc(Direction::Backward, s, 1);
  CHECK(c == INF);
}

TEST_CASE("NgPathResource: to in N(from) but bit unset passes") {
  auto ng = make_ng5();

  // v1 ∈ N(v0) at pos 1. State at v0 after extend_to_vertex(0) only.
  // v1's bit (pos 1) is NOT set → extend should pass.
  auto [s0, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);

  // Verify v1 is indeed a neighbor of v0
  CHECK(ng.bit_map[0 * 5 + 1] >= 0);

  // Verify v1's bit is not set
  int8_t v1_pos = ng.bit_map[0 * 5 + 1];
  CHECK((s0 & (1ULL << v1_pos)) == 0);

  // Extend arc 0 (0→1): v1 is neighbor but not visited → pass
  auto [s1, c] = ng.extend_along_arc(Direction::Forward, s0, 0);
  CHECK(c == 0.0);
}

TEST_CASE("NgPathResource: backward extend, to not in N(from) — self-bit "
          "lost") {
  auto ng = make_ng5();

  // Arc 4 (from=1, to=4). Backward: label at v4, extending to v1.
  // bw_check_bit = bit_map[4*5+1] → v1 ∉ N(v4) → -1 → no check
  CHECK(ng.bit_map[4 * 5 + 1] == -1);

  auto [s0, _] = ng.extend_to_vertex(Direction::Backward, 0ULL, 4);

  // v4's self bit and neighbor-self bit should be set
  CHECK((s0 & (1ULL << ng.self_bit_[4])) != 0);

  auto [s1, c] = ng.extend_along_arc(Direction::Backward, s0, 4);
  CHECK(c == 0.0);

  // v4 ∉ N(v1), so v4's self-bit shift pair wasn't added → state should be 0
  // (only shared neighbors could survive, but v4's self bit is lost)
  CHECK(s1 == 0ULL);
}

TEST_CASE("NgPathResource: asymmetric neighborhoods — from not in N(to) but "
          "to in N(from)") {
  // 3 vertices. neighbors[0] = {1} (just v1), neighbors[1] = {} (empty)
  int from[] = {0, 1};
  int to[] = {1, 2};
  std::vector<std::vector<int>> neighbors = {{1}, {}, {}};
  NgPathResource ng(3, 2, from, to, neighbors);

  // v1 ∈ N(v0) at pos 0
  CHECK(ng.bit_map[0 * 3 + 1] == 0);
  // v0 ∉ N(v1) (N(v1) is empty)
  CHECK(ng.bit_map[1 * 3 + 0] == -1);
  CHECK(ng.self_bit_[0] == 1);  // 1 neighbor → self bit at 1

  auto [s0, _] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);
  // self_bit_[0]=1 set. v0∈N(v0)? bit_map[0*3+0]=-1 → no neighbor-self.
  CHECK(s0 == (1ULL << 1));

  // Extend arc 0 (0→1): check v1 at pos 0 → NOT set → pass
  auto [s1, c] = ng.extend_along_arc(Direction::Forward, s0, 0);
  CHECK(c == 0.0);

  // No bits survive: v0's self-bit not remapped (v0∉N(v1)), no shared neighbors
  CHECK(s1 == 0ULL);
}

TEST_CASE(
    "NgPathResource: self-loop remap preserves other neighbor bits") {
  // 3 vertices, all fully connected neighborhoods, arcs: 0→1, 1→1 (self-loop)
  int from[] = {0, 1};
  int to[] = {1, 1};
  std::vector<std::vector<int>> neighbors = {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}};
  NgPathResource ng(3, 2, from, to, neighbors);

  // bit_map[v*3+w] = w for all v (symmetric, same ordering)
  CHECK(ng.bit_map[1 * 3 + 0] == 0);
  CHECK(ng.bit_map[1 * 3 + 1] == 1);
  CHECK(ng.bit_map[1 * 3 + 2] == 2);
  CHECK(ng.self_bit_[1] == 3);

  // Path: extend_to_vertex(0) → arc 0→1 → extend_to_vertex(1)
  auto [s0, _0] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);
  auto [s1_arc, c1] = ng.extend_along_arc(Direction::Forward, s0, 0);
  REQUIRE(c1 == 0.0);
  auto [s1, _1] = ng.extend_to_vertex(Direction::Forward, s1_arc, 1);

  // v0 mapped at bit 0, v1 self bit at 3, v1 neighbor-self at 1
  CHECK((s1 & (1ULL << 0)) != 0);  // v0
  CHECK((s1 & (1ULL << 3)) != 0);  // v1 self
  CHECK((s1 & (1ULL << 1)) != 0);  // v1 neighbor-self

  // Self-loop should be blocked (v1 neighbor-self bit at pos 1 is set)
  auto [s2, c2] = ng.extend_along_arc(Direction::Forward, s1, 1);
  CHECK(c2 == INF);

  // Now test: artificially clear the v1-as-neighbor bit (pos 1) but keep v0's
  // bit. This simulates a state where we're at v1 but haven't marked v1 as
  // neighbor-visited.
  uint64_t artificial = (1ULL << 0) | (1ULL << 3);  // v0 at pos 0, self bit 3
  auto [s3, c3] = ng.extend_along_arc(Direction::Forward, artificial, 1);
  CHECK(c3 == 0.0);  // check bit 1 not set → pass

  // After remap through self-loop (identity for neighbor bits, self-bit→pos 1):
  // bit 0 (v0) → pair (0,0) → bit 0 survives
  // bit 3 (self) → pair (3,1) → bit 1
  CHECK((s3 & (1ULL << 0)) != 0);  // v0 survives
  CHECK((s3 & (1ULL << 1)) != 0);  // self remapped to neighbor-self pos
}

TEST_CASE("NgPathResource: domination with v not in own ng-set") {
  // neighbors[1] = {0, 2} (v1 NOT in N(v1)). self_bit_[1] = 2.
  int from[] = {0};
  int to[] = {1};
  std::vector<std::vector<int>> neighbors = {{0}, {0, 2}, {2}};
  NgPathResource ng(3, 1, from, to, neighbors);

  CHECK(ng.self_bit_[1] == 2);
  CHECK(ng.bit_map[1 * 3 + 1] == -1);  // v1 NOT in N(v1)

  // Two labels at v1, both with self-bit set
  uint64_t self_only = (1ULL << 2);                    // just self bit
  uint64_t self_plus_v0 = (1ULL << 2) | (1ULL << 0);  // self + v0 visited

  // Fewer bits dominates more bits
  CHECK(ng.domination_cost(Direction::Forward, 1, self_only, self_plus_v0) ==
        0.0);
  // More bits cannot dominate fewer
  CHECK(ng.domination_cost(Direction::Forward, 1, self_plus_v0, self_only) ==
        INF);
  // Equal → dominates
  CHECK(ng.domination_cost(Direction::Forward, 1, self_only, self_only) == 0.0);
}

TEST_CASE("NgPathResource: concatenation trivially passes when fw state is "
          "empty after remap") {
  // N(0)={0,1}, N(1)={1}, N(2)={2}. Arcs: 0→1, 1→2.
  // After extending 0→1→2, fw state becomes 0 (no shared neighbors).
  int from[] = {0, 1};
  int to[] = {1, 2};
  std::vector<std::vector<int>> neighbors = {{0, 1}, {1}, {2}};
  NgPathResource ng(3, 2, from, to, neighbors);

  CHECK(ng.self_bit_[0] == 2);  // 2 neighbors
  CHECK(ng.self_bit_[1] == 1);  // 1 neighbor
  CHECK(ng.self_bit_[2] == 1);  // 1 neighbor

  // fw path: source v0, extend to v1, extend to v2
  auto [s0, _0] = ng.extend_to_vertex(Direction::Forward, 0ULL, 0);
  auto [s1_arc, c1] = ng.extend_along_arc(Direction::Forward, s0, 0);
  REQUIRE(c1 == 0.0);
  auto [s1, _1] = ng.extend_to_vertex(Direction::Forward, s1_arc, 1);

  // Extend arc 1 (1→2): v1∉N(v2), v2∉N(v1) → no check, no shared neighbors
  auto [s2_arc, c2] = ng.extend_along_arc(Direction::Forward, s1, 1);
  CHECK(c2 == 0.0);
  CHECK(s2_arc == 0ULL);  // all bits lost — no overlap possible

  // bw label at v2 with self bit set
  auto [bw, _b] = ng.extend_to_vertex(Direction::Backward, 0ULL, 2);
  CHECK(bw != 0ULL);

  // Concatenation: 0 & bw = 0 → always feasible
  CHECK(ng.concatenation_cost(Symmetry::Asymmetric, 2, s2_arc, bw) == 0.0);
}

// ════════════════════════════════════════════════════════════════
// Solver-level tests with NgPathResource
// ════════════════════════════════════════════════════════════════

// Helper: 4-vertex graph with a negative-cost cycle.
//
//   0 --(-5)--> 1 --(-5)--> 2 --(-5)--> 1  (cycle!)  --> 3
//   0 --( 1)--> 3  (direct)
//
// Without ng-path: solver exploits the 1→2→1 cycle.
// With full ng-path: cycle is blocked, best elementary = -9.

static ProblemView make_cycle_pv(int* from, int* to, double* cost,
                                 double* time_d, double* tw_lb, double* tw_ub,
                                 const double** arc_res, const double** v_lb,
                                 const double** v_ub) {
  from[0] = 0;
  to[0] = 1;
  cost[0] = -5.0;
  time_d[0] = 1.0;
  from[1] = 0;
  to[1] = 3;
  cost[1] = 1.0;
  time_d[1] = 1.0;
  from[2] = 1;
  to[2] = 2;
  cost[2] = -5.0;
  time_d[2] = 1.0;
  from[3] = 2;
  to[3] = 1;
  cost[3] = -5.0;
  time_d[3] = 1.0;
  from[4] = 1;
  to[4] = 3;
  cost[4] = 1.0;
  time_d[4] = 1.0;
  from[5] = 2;
  to[5] = 3;
  cost[5] = 1.0;
  time_d[5] = 1.0;

  tw_lb[0] = 0;
  tw_lb[1] = 0;
  tw_lb[2] = 0;
  tw_lb[3] = 0;
  tw_ub[0] = 100;
  tw_ub[1] = 100;
  tw_ub[2] = 100;
  tw_ub[3] = 100;

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

static auto make_full_neighbors(int n) {
  std::vector<std::vector<int>> neighbors(n);
  for (int v = 0; v < n; ++v) {
    neighbors[v].resize(n);
    // Put self first, then others
    neighbors[v][0] = v;
    for (int w = 0, pos = 1; w < n; ++w)
      if (w != v) neighbors[v][pos++] = w;
  }
  return neighbors;
}

// Helper: verify all paths are elementary (no repeated vertices).
static void check_elementary(const auto& paths) {
  for (auto& p : paths) {
    auto& v = p.vertices;
    for (size_t i = 0; i < v.size(); ++i)
      for (size_t j = i + 1; j < v.size(); ++j) CHECK(v[i] != v[j]);
  }
}

TEST_CASE("Solver: ng-path blocks cycles, mono and bidir agree") {
  int from[6], to[6];
  double cost[6], time_d[6], tw_lb[4], tw_ub[4];
  const double* arc_res[1];
  const double* v_lb[1];
  const double* v_ub[1];
  auto pv =
      make_cycle_pv(from, to, cost, time_d, tw_lb, tw_ub, arc_res, v_lb, v_ub);

  auto neighbors = make_full_neighbors(4);

  // Mono
  NgPathResource ng_m(4, 6, from, to, neighbors);
  Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_m)),
                      {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
  mono.build();
  mono.set_stage(Stage::Exact);
  auto mono_paths = mono.solve();

  REQUIRE(!mono_paths.empty());
  CHECK(mono_paths[0].reduced_cost == doctest::Approx(-9.0));
  check_elementary(mono_paths);

  // Bidir — must agree
  NgPathResource ng_b(4, 6, from, to, neighbors);
  Solver<NgPack> bidir(
      pv, make_resource_pack(std::move(ng_b)),
      {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .theta = 1e9});
  bidir.build();
  bidir.set_stage(Stage::Exact);
  auto bidir_paths = bidir.solve();

  REQUIRE(!bidir_paths.empty());
  CHECK(bidir_paths[0].reduced_cost == doctest::Approx(-9.0));
  check_elementary(bidir_paths);
}

TEST_CASE("Solver: without ng-path, cycles are exploited") {
  int from[6], to[6];
  double cost[6], time_d[6], tw_lb[4], tw_ub[4];
  const double* arc_res[1];
  const double* v_lb[1];
  const double* v_ub[1];
  auto pv =
      make_cycle_pv(from, to, cost, time_d, tw_lb, tw_ub, arc_res, v_lb, v_ub);

  Solver<EmptyPack> solver(pv, EmptyPack{},
                           {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
  solver.build();
  solver.set_stage(Stage::Exact);
  auto paths = solver.solve();

  REQUIRE(!paths.empty());
  CHECK(paths[0].reduced_cost < -9.0 - 1e-6);
}

TEST_CASE("Solver: bidir across-arc splice finds optimal elementary path") {
  // 6 vertices, source=0, sink=5.
  // Optimal elementary: 0→1→3→5 (cost=-21), crosses bidir midpoint.
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
  int to[N] = {1, 2, 3, 3, 4, 5, 5};
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

  auto neighbors = make_full_neighbors(6);

  // Mono
  NgPathResource ng_m(6, N, from, to, neighbors);
  Solver<NgPack> mono(pv, make_resource_pack(std::move(ng_m)),
                      {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
  mono.build();
  mono.set_stage(Stage::Exact);
  auto mono_paths = mono.solve();
  REQUIRE(!mono_paths.empty());
  CHECK(mono_paths[0].reduced_cost == doctest::Approx(-21.0));
  check_elementary(mono_paths);

  // Bidir must find same cost via across-arc splice
  NgPathResource ng_b(6, N, from, to, neighbors);
  Solver<NgPack> bidir(
      pv, make_resource_pack(std::move(ng_b)),
      {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .theta = 1e9});
  bidir.build();
  bidir.set_stage(Stage::Exact);
  auto bidir_paths = bidir.solve();
  REQUIRE(!bidir_paths.empty());
  CHECK(bidir_paths[0].reduced_cost <= mono_paths[0].reduced_cost + 1e-6);
  check_elementary(bidir_paths);
}

TEST_CASE("Solver: bidir with cycle + concatenation") {
  // 5 vertices, source=0, sink=4.
  // Negative cycle 1→2→1 plus the optimal path must cross bidir midpoint.
  //
  // Arcs:           cost    time
  // 0→1 (0)         -10      20
  // 0→3 (1)          -1       1
  // 1→2 (2)         -10      15    ← crosses midpoint
  // 2→1 (3)         -10       1    (cycle back)
  // 1→4 (4)           1      30
  // 2→4 (5)           1      30
  // 3→4 (6)          -1       1
  //
  // Without ng: cycle 1→2→1→2→... exploited.
  // With ng: best elementary = 0→1→2→4 (cost = -10 + -10 + 1 = -19)
  // The 1→2 arc crosses midpoint → bidir needs across-arc splice.

  const int N = 7;
  int from[N] = {0, 0, 1, 2, 1, 2, 3};
  int to[N] = {1, 3, 2, 1, 4, 4, 4};
  double cost[N] = {-10, -1, -10, -10, 1, 1, -1};
  double time_d[N] = {20, 1, 15, 1, 30, 30, 1};
  double tw_lb[5] = {0, 0, 0, 0, 0};
  double tw_ub[5] = {100, 100, 100, 100, 100};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 5;
  pv.source = 0;
  pv.sink = 4;
  pv.n_arcs = N;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  auto neighbors = make_full_neighbors(5);

  // Without ng: cycle exploited
  {
    Solver<EmptyPack> solver(pv, EmptyPack{},
                             {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();
    REQUIRE(!paths.empty());
    CHECK(paths[0].reduced_cost < -19.0 - 1e-6);
  }

  // Mono with ng: elementary optimum
  double mono_best;
  {
    NgPathResource ng(5, N, from, to, neighbors);
    Solver<NgPack> solver(pv, make_resource_pack(std::move(ng)),
                          {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();
    REQUIRE(!paths.empty());
    mono_best = paths[0].reduced_cost;
    CHECK(mono_best == doctest::Approx(-19.0));
    check_elementary(paths);
  }

  // Bidir with ng: must agree
  {
    NgPathResource ng(5, N, from, to, neighbors);
    Solver<NgPack> solver(
        pv, make_resource_pack(std::move(ng)),
        {.bucket_steps = {10.0, 1.0}, .bidirectional = true, .theta = 1e9});
    solver.build();
    solver.set_stage(Stage::Exact);
    auto paths = solver.solve();
    REQUIRE(!paths.empty());
    CHECK(paths[0].reduced_cost <= mono_best + 1e-6);
    check_elementary(paths);
  }
}

// ════════════════════════════════════════════════════════════════
// CumulativeCostResource (R^cc) — Meta-Solver 2026 §5
// ════════════════════════════════════════════════════════════════

TEST_CASE("CumulativeCostResource: init_state") {
  double travel[] = {1.0};
  double wt[] = {2.0};
  CumulativeCostResource res(travel, wt, 10.0, 20.0, 1);

  auto sf = res.init_state(Direction::Forward);
  CHECK(sf.S == 0.0);
  CHECK(sf.T_or_W == 0.0);

  auto sb = res.init_state(Direction::Backward);
  CHECK(sb.S == 0.0);
  CHECK(sb.T_or_W == 0.0);
}

TEST_CASE("CumulativeCostResource: extend_along_arc forward") {
  // Arc 0: travel_time=3, weight=2
  // Arc 1: travel_time=5, weight=1
  double travel[] = {3.0, 5.0};
  double wt[] = {2.0, 1.0};
  CumulativeCostResource res(travel, wt, 10.0, 20.0, 2);

  // Start: S=0, T=0. Extend arc 0: delta = w*(T+l) = 2*(0+3) = 6
  auto [s1, c1] = res.extend_along_arc(Direction::Forward, {0.0, 0.0}, 0);
  CHECK(s1.S == doctest::Approx(6.0));
  CHECK(s1.T_or_W == doctest::Approx(3.0));
  CHECK(c1 == doctest::Approx(6.0));

  // Continue: S=6, T=3. Extend arc 1: delta = 1*(3+5) = 8
  auto [s2, c2] = res.extend_along_arc(Direction::Forward, s1, 1);
  CHECK(s2.S == doctest::Approx(14.0));
  CHECK(s2.T_or_W == doctest::Approx(8.0));
  CHECK(c2 == doctest::Approx(8.0));
}

TEST_CASE("CumulativeCostResource: extend_along_arc backward") {
  // Arc 0: travel_time=3, weight=2
  double travel[] = {3.0};
  double wt[] = {2.0};
  CumulativeCostResource res(travel, wt, 10.0, 20.0, 1);

  // Start: S=0, W=0. Extend arc 0: delta = (W+w)*l = (0+2)*3 = 6
  auto [s1, c1] = res.extend_along_arc(Direction::Backward, {0.0, 0.0}, 0);
  CHECK(s1.S == doctest::Approx(6.0));
  CHECK(s1.T_or_W == doctest::Approx(2.0));  // W' = 0 + 2
  CHECK(c1 == doctest::Approx(6.0));
}

TEST_CASE("CumulativeCostResource: extend_to_vertex is no-op") {
  double travel[] = {1.0};
  double wt[] = {2.0};
  CumulativeCostResource res(travel, wt, 10.0, 20.0, 1);

  auto [s, c] = res.extend_to_vertex(Direction::Forward, {5.0, 3.0}, 1);
  CHECK(s.S == 5.0);
  CHECK(s.T_or_W == 3.0);
  CHECK(c == 0.0);
}

TEST_CASE("CumulativeCostResource: domination_cost forward") {
  double travel[] = {1.0};
  double wt[] = {2.0};
  CumulativeCostResource res(travel, wt, 10.0, 20.0, 1);

  // Forward: (S1-S2) + max(0, T1-T2) * W_max
  // s1={S=5, T=3}, s2={S=2, T=1}: (5-2) + max(0,3-1)*10 = 3 + 20 = 23
  CHECK(res.domination_cost(Direction::Forward, 0, {5.0, 3.0}, {2.0, 1.0}) ==
        doctest::Approx(23.0));

  // When T1 < T2: max(0, T1-T2)*W_max = 0
  CHECK(res.domination_cost(Direction::Forward, 0, {5.0, 1.0}, {2.0, 3.0}) ==
        doctest::Approx(3.0));

  // Equal states → 0
  CHECK(res.domination_cost(Direction::Forward, 0, {5.0, 3.0}, {5.0, 3.0}) ==
        doctest::Approx(0.0));
}

TEST_CASE("CumulativeCostResource: domination_cost backward") {
  double travel[] = {1.0};
  double wt[] = {2.0};
  CumulativeCostResource res(travel, wt, 10.0, 20.0, 1);

  // Backward: (S1-S2) + max(0, W1-W2) * T_max
  // s1={S=5, W=3}, s2={S=2, W=1}: (5-2) + max(0,3-1)*20 = 3 + 40 = 43
  CHECK(res.domination_cost(Direction::Backward, 0, {5.0, 3.0}, {2.0, 1.0}) ==
        doctest::Approx(43.0));
}

TEST_CASE("CumulativeCostResource: concatenation_cost") {
  double travel[] = {1.0};
  double wt[] = {2.0};
  CumulativeCostResource res(travel, wt, 10.0, 20.0, 1);

  // T_fw * W_bw
  CHECK(res.concatenation_cost(Symmetry::Asymmetric, 0, {0.0, 4.0},
                               {0.0, 3.0}) == doctest::Approx(12.0));
  CHECK(res.concatenation_cost(Symmetry::Asymmetric, 0, {0.0, 0.0},
                               {0.0, 5.0}) == doctest::Approx(0.0));
}

TEST_CASE("ResourcePack<CumulativeCostResource> extend + domination") {
  double travel[] = {3.0, 5.0};
  double wt[] = {2.0, 1.0};
  auto pack =
      make_resource_pack(CumulativeCostResource(travel, wt, 10.0, 20.0, 2));

  auto states = pack.init_states(Direction::Forward);
  auto& s0 = std::get<0>(states);
  CHECK(s0.S == 0.0);
  CHECK(s0.T_or_W == 0.0);

  auto [s1, c1] = pack.extend_along_arc(Direction::Forward, states, 0);
  CHECK(c1 == doctest::Approx(6.0));
  CHECK(std::get<0>(s1).S == doctest::Approx(6.0));

  // Domination: init (0,0) vs extended (6,3) → (0-6) + max(0,0-3)*10 = -6
  CHECK(pack.domination_cost(Direction::Forward, 0, states, s1) ==
        doctest::Approx(-6.0));
  // Extended vs init → (6-0) + max(0,3-0)*10 = 6 + 30 = 36
  CHECK(pack.domination_cost(Direction::Forward, 0, s1, states) ==
        doctest::Approx(36.0));
}

// ════════════════════════════════════════════════════════════════
// PickupDeliveryResource (R^spd) — Meta-Solver 2026 §6
// ════════════════════════════════════════════════════════════════

TEST_CASE("PickupDeliveryResource: init_state") {
  double pk[] = {0.0, 1.0, 2.0};
  double dl[] = {0.0, 3.0, 1.0};
  PickupDeliveryResource res(pk, dl, 10.0, 3);

  auto sf = res.init_state(Direction::Forward);
  CHECK(sf.P == 0.0);
  CHECK(sf.D == 10.0);

  auto sb = res.init_state(Direction::Backward);
  CHECK(sb.P == 10.0);
  CHECK(sb.D == 0.0);
}

TEST_CASE("PickupDeliveryResource: extend_to_vertex forward feasible") {
  double pk[] = {0.0, 3.0, 2.0};
  double dl[] = {0.0, 1.0, 4.0};
  PickupDeliveryResource res(pk, dl, 10.0, 3);

  // Start: P=0, D=10. Visit v1: p=3, d=1.
  // P' = 0+3 = 3, D' = min(10-1, 10-3) = min(9, 7) = 7
  auto [s1, c1] = res.extend_to_vertex(Direction::Forward, {0.0, 10.0}, 1);
  CHECK(c1 == 0.0);
  CHECK(s1.P == doctest::Approx(3.0));
  CHECK(s1.D == doctest::Approx(7.0));
}

TEST_CASE(
    "PickupDeliveryResource: extend_to_vertex forward infeasible pickup") {
  double pk[] = {0.0, 8.0};
  double dl[] = {0.0, 0.0};
  PickupDeliveryResource res(pk, dl, 10.0, 2);

  // P=5, D=5. Visit v1: p=8. P+p = 13 > Q=10 → infeasible
  auto [s, c] = res.extend_to_vertex(Direction::Forward, {5.0, 5.0}, 1);
  CHECK(c >= INF);
}

TEST_CASE(
    "PickupDeliveryResource: extend_to_vertex forward infeasible delivery") {
  double pk[] = {0.0, 0.0};
  double dl[] = {0.0, 7.0};
  PickupDeliveryResource res(pk, dl, 10.0, 2);

  // P=0, D=5. Visit v1: d=7. d > D → infeasible
  auto [s, c] = res.extend_to_vertex(Direction::Forward, {0.0, 5.0}, 1);
  CHECK(c >= INF);
}

TEST_CASE("PickupDeliveryResource: extend_to_vertex backward feasible") {
  double pk[] = {0.0, 2.0, 3.0};
  double dl[] = {0.0, 1.0, 4.0};
  PickupDeliveryResource res(pk, dl, 10.0, 3);

  // Start bw: P_bar=10, D_bar=0. Visit v2: p=3, d=4.
  // D_bar' = 0+4 = 4, P_bar' = min(10-3, 10-4) = min(7, 6) = 6
  auto [s1, c1] = res.extend_to_vertex(Direction::Backward, {10.0, 0.0}, 2);
  CHECK(c1 == 0.0);
  CHECK(s1.P == doctest::Approx(6.0));
  CHECK(s1.D == doctest::Approx(4.0));
}

TEST_CASE("PickupDeliveryResource: extend_to_vertex backward infeasible") {
  double pk[] = {0.0, 8.0};
  double dl[] = {0.0, 0.0};
  PickupDeliveryResource res(pk, dl, 10.0, 2);

  // P_bar=5, D_bar=3. Visit v1: p=8. p > P_bar → infeasible
  auto [s, c] = res.extend_to_vertex(Direction::Backward, {5.0, 3.0}, 1);
  CHECK(c >= INF);
}

TEST_CASE("PickupDeliveryResource: extend_along_arc is no-op") {
  double pk[] = {0.0, 1.0};
  double dl[] = {0.0, 2.0};
  PickupDeliveryResource res(pk, dl, 10.0, 2);

  auto [s, c] = res.extend_along_arc(Direction::Forward, {3.0, 7.0}, 0);
  CHECK(s.P == 3.0);
  CHECK(s.D == 7.0);
  CHECK(c == 0.0);
}

TEST_CASE("PickupDeliveryResource: domination_cost forward") {
  double pk[] = {0.0};
  double dl[] = {0.0};
  PickupDeliveryResource res(pk, dl, 10.0, 1);

  // P1 <= P2 and D1 >= D2 → 0 (s1 dominates)
  CHECK(res.domination_cost(Direction::Forward, 0, {2.0, 8.0}, {3.0, 7.0}) ==
        0.0);
  // P1 > P2 → INF
  CHECK(res.domination_cost(Direction::Forward, 0, {4.0, 8.0}, {3.0, 7.0}) ==
        INF);
  // D1 < D2 → INF
  CHECK(res.domination_cost(Direction::Forward, 0, {2.0, 6.0}, {3.0, 7.0}) ==
        INF);
  // Equal → 0
  CHECK(res.domination_cost(Direction::Forward, 0, {3.0, 7.0}, {3.0, 7.0}) ==
        0.0);
}

TEST_CASE("PickupDeliveryResource: domination_cost backward") {
  double pk[] = {0.0};
  double dl[] = {0.0};
  PickupDeliveryResource res(pk, dl, 10.0, 1);

  // D_bar1 <= D_bar2 and P_bar1 >= P_bar2 → 0
  CHECK(res.domination_cost(Direction::Backward, 0, {8.0, 2.0}, {7.0, 3.0}) ==
        0.0);
  // D_bar1 > D_bar2 → INF
  CHECK(res.domination_cost(Direction::Backward, 0, {8.0, 4.0}, {7.0, 3.0}) ==
        INF);
  // P_bar1 < P_bar2 → INF
  CHECK(res.domination_cost(Direction::Backward, 0, {6.0, 2.0}, {7.0, 3.0}) ==
        INF);
}

TEST_CASE("PickupDeliveryResource: concatenation_cost") {
  double pk[] = {0.0};
  double dl[] = {0.0};
  PickupDeliveryResource res(pk, dl, 10.0, 1);

  // Compatible: P_fw <= P_bar_bw and D_bar_bw <= D_fw
  CHECK(res.concatenation_cost(Symmetry::Asymmetric, 0, {3.0, 7.0},
                               {5.0, 2.0}) == 0.0);
  // P_fw > P_bar_bw → INF
  CHECK(res.concatenation_cost(Symmetry::Asymmetric, 0, {6.0, 7.0},
                               {5.0, 2.0}) == INF);
  // D_bar_bw > D_fw → INF
  CHECK(res.concatenation_cost(Symmetry::Asymmetric, 0, {3.0, 4.0},
                               {5.0, 5.0}) == INF);
}

TEST_CASE("ResourcePack<PickupDeliveryResource> through solver-like sequence") {
  double pk[] = {0.0, 3.0, 2.0, 0.0};
  double dl[] = {0.0, 1.0, 4.0, 0.0};
  auto pack = make_resource_pack(PickupDeliveryResource(pk, dl, 10.0, 4));

  auto states = pack.init_states(Direction::Forward);
  auto& s0 = std::get<0>(states);
  CHECK(s0.P == 0.0);
  CHECK(s0.D == 10.0);

  // extend_to_vertex at source (v0: p=0, d=0)
  auto [vs0, vc0] = pack.extend_to_vertex(Direction::Forward, states, 0);
  CHECK(vc0 == 0.0);

  // extend_along_arc (no-op for PD)
  auto [as1, ac1] = pack.extend_along_arc(Direction::Forward, vs0, 0);
  CHECK(ac1 == 0.0);

  // extend_to_vertex at v1: p=3, d=1
  auto [vs1, vc1] = pack.extend_to_vertex(Direction::Forward, as1, 1);
  CHECK(vc1 == 0.0);
  CHECK(std::get<0>(vs1).P == doctest::Approx(3.0));
  CHECK(std::get<0>(vs1).D == doctest::Approx(7.0));

  // Domination: fresh state dominates consumed state
  CHECK(pack.domination_cost(Direction::Forward, 1, states, vs1) == 0.0);
  CHECK(pack.domination_cost(Direction::Forward, 1, vs1, states) == INF);
}

// ════════════════════════════════════════════════════════════════
// Solver-level: R^cc changes optimal path via cumulative cost
// ════════════════════════════════════════════════════════════════

TEST_CASE("Solver: CumulativeCostResource changes optimal path") {
  // 4 vertices: source=0, sink=3
  // Path A: 0→1→3  base_cost = -1 + -1 = -2, but heavy first arc
  // Path B: 0→2→3  base_cost = -1 + -1 = -2, but light first arc
  //
  // Without R^cc: both paths cost -2, either is optimal.
  // With R^cc: Path A has higher cumulative cost because heavy weight
  // arrives early, so Path B should be preferred.
  //
  // Arcs:       cost   time  weight
  // 0→1 (0)     -1      5     10    (heavy)
  // 0→2 (1)     -1      5      1    (light)
  // 1→3 (2)     -1      5      1
  // 2→3 (3)     -1      5      1

  const int NA = 4;
  int from[NA] = {0, 0, 1, 2};
  int to[NA] = {1, 2, 3, 3};
  double cost[NA] = {-1.0, -1.0, -1.0, -1.0};
  double time_d[NA] = {5.0, 5.0, 5.0, 5.0};
  double tw_lb[4] = {0, 0, 0, 0};
  double tw_ub[4] = {100, 100, 100, 100};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 4;
  pv.source = 0;
  pv.sink = 3;
  pv.n_arcs = NA;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  double travel[NA] = {5.0, 5.0, 5.0, 5.0};
  double wt[NA] = {10.0, 1.0, 1.0, 1.0};

  using CcPack = ResourcePack<CumulativeCostResource>;
  CumulativeCostResource cc(travel, wt, 12.0, 100.0, NA);
  Solver<CcPack> solver(pv, make_resource_pack(std::move(cc)),
                        {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
  solver.build();
  solver.set_stage(Stage::Exact);
  auto paths = solver.solve();

  REQUIRE(!paths.empty());
  // Path A cumulative: arc0 delta=10*(0+5)=50, arc2 delta=1*(5+5)=10 → total=60
  //   total cost = -2 + 60 = 58
  // Path B cumulative: arc1 delta=1*(0+5)=5,  arc3 delta=1*(5+5)=10 → total=15
  //   total cost = -2 + 15 = 13
  // Best path should be B with total cost 13
  CHECK(paths[0].reduced_cost == doctest::Approx(13.0));
  // Verify it's path B: 0→2→3
  REQUIRE(paths[0].vertices.size() == 3);
  CHECK(paths[0].vertices[0] == 0);
  CHECK(paths[0].vertices[1] == 2);
  CHECK(paths[0].vertices[2] == 3);
}

// ════════════════════════════════════════════════════════════════
// Solver-level: R^spd blocks paths that violate capacity
// ════════════════════════════════════════════════════════════════

TEST_CASE("Solver: PickupDeliveryResource blocks capacity-violating paths") {
  // 4 vertices: source=0, sink=3, Q=5
  // Path A: 0→1→3  cheaper but v1 has pickup=6 > Q=5 → infeasible
  // Path B: 0→2→3  feasible (v2 has pickup=2, delivery=1)
  //
  // Arcs:       cost   time
  // 0→1 (0)     -10     1
  // 0→2 (1)     -1      1
  // 1→3 (2)     -10     1
  // 2→3 (3)     -1      1

  const int NA = 4;
  int from[NA] = {0, 0, 1, 2};
  int to[NA] = {1, 2, 3, 3};
  double cost[NA] = {-10.0, -1.0, -10.0, -1.0};
  double time_d[NA] = {1.0, 1.0, 1.0, 1.0};
  double tw_lb[4] = {0, 0, 0, 0};
  double tw_ub[4] = {100, 100, 100, 100};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 4;
  pv.source = 0;
  pv.sink = 3;
  pv.n_arcs = NA;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  double pk[4] = {0.0, 6.0, 2.0, 0.0};  // v1 pickup=6 exceeds Q=5
  double dl[4] = {0.0, 0.0, 1.0, 0.0};

  using PdPack = ResourcePack<PickupDeliveryResource>;
  PickupDeliveryResource pd(pk, dl, 5.0, 4);
  Solver<PdPack> solver(pv, make_resource_pack(std::move(pd)),
                        {.bucket_steps = {10.0, 1.0}, .theta = 1e9});
  solver.build();
  solver.set_stage(Stage::Exact);
  auto paths = solver.solve();

  REQUIRE(!paths.empty());
  // Path A (0→1→3, cost=-20) is blocked by PD. Path B (0→2→3, cost=-2) is best.
  CHECK(paths[0].reduced_cost == doctest::Approx(-2.0));
  REQUIRE(paths[0].vertices.size() == 3);
  CHECK(paths[0].vertices[0] == 0);
  CHECK(paths[0].vertices[1] == 2);
  CHECK(paths[0].vertices[2] == 3);
}
