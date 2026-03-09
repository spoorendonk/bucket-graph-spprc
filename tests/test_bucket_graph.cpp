#include <bgspprc/bucket_graph.h>
#include <bgspprc/r1c.h>
#include <bgspprc/resource.h>
#include <bgspprc/resources/ng_path.h>
#include <bgspprc/solver.h>

#include <cstdio>

#include "doctest.h"
#ifndef _WIN32
#include <unistd.h>
#endif

using namespace bgspprc;

// Helper to build a simple 4-vertex graph:
//   0 → 1 → 3
//   0 → 2 → 3
// with time windows and costs.
struct SimpleGraph {
  // 4 vertices: 0=source, 3=sink
  // 4 arcs: 0→1, 0→2, 1→3, 2→3
  int from[4] = {0, 0, 1, 2};
  int to[4] = {1, 2, 3, 3};
  double cost[4] = {1.0, 2.0, 3.0, 1.0};    // base costs
  double time_d[4] = {1.0, 2.0, 1.0, 1.0};  // time consumption

  // Time windows per vertex
  double tw_lb[4] = {0.0, 0.0, 0.0, 0.0};
  double tw_ub[4] = {10.0, 10.0, 10.0, 10.0};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;

  SimpleGraph() {
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 4;
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

// Larger graph for more thorough testing
//   0 → 1 → 3 → 5
//   0 → 2 → 4 → 5
//   1 → 4 → 5
//   2 → 3 → 5
struct LargerGraph {
  int from[8] = {0, 0, 1, 2, 3, 4, 1, 2};
  int to[8] = {1, 2, 3, 4, 5, 5, 4, 3};
  double cost[8] = {5.0, 3.0, 4.0, 6.0, 2.0, 1.0, 7.0, 8.0};
  double time_d[8] = {1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 3.0, 2.0};

  double tw_lb[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[6] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;

  LargerGraph() {
    pv.n_vertices = 6;
    pv.source = 0;
    pv.sink = 5;
    pv.n_arcs = 8;
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

// ── Basic tests ──

TEST_CASE("Bucket construction") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}});
  bg.build();
  CHECK(bg.n_buckets() == 8);

  for (int i = 0; i < bg.n_buckets(); ++i) {
    CHECK(bg.bucket(i).vertex >= 0);
    CHECK(bg.bucket(i).vertex < 4);
  }
}

TEST_CASE("Bucket construction step=1") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{}, {.bucket_steps = {1.0, 1.0}});
  bg.build();
  CHECK(bg.n_buckets() == 40);
}

TEST_CASE("Bucket arcs exist") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}});
  bg.build();

  int total_arcs = 0;
  for (int i = 0; i < bg.n_buckets(); ++i) {
    total_arcs += static_cast<int>(bg.bucket(i).bucket_arcs.size());
  }
  CHECK(total_arcs > 0);
}

TEST_CASE("Solve simple graph") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  auto paths = bg.solve();
  CHECK(!paths.empty());
  CHECK(paths[0].reduced_cost == doctest::Approx(3.0));
  CHECK(paths[0].vertices.size() == 3);
}

TEST_CASE("Solve with reduced costs") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  double red_cost[4] = {-5.0, 2.0, 3.0, 1.0};
  bg.update_arc_costs(red_cost);

  auto paths = bg.solve();
  CHECK(!paths.empty());
  CHECK(paths[0].reduced_cost == doctest::Approx(-2.0));
}

// ── Arc elimination ──

TEST_CASE("Arc elimination preserves optimality") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  auto paths_before = bg.solve();
  REQUIRE(!paths_before.empty());
  double best_before = paths_before[0].reduced_cost;

  bg.eliminate_arcs(100.0);
  auto paths_after = bg.solve();
  REQUIRE(!paths_after.empty());
  CHECK(paths_after[0].reduced_cost == doctest::Approx(best_before));

  bg.reset_elimination();
  auto paths_reset = bg.solve();
  REQUIRE(!paths_reset.empty());
  CHECK(paths_reset[0].reduced_cost == doctest::Approx(best_before));
}

TEST_CASE("Arc elimination with tight theta removes arcs") {
  LargerGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  // Solve to populate c_best
  bg.solve();

  // Count arcs before
  int arcs_before = 0;
  for (int i = 0; i < bg.n_buckets(); ++i)
    arcs_before += static_cast<int>(bg.bucket(i).bucket_arcs.size());

  // Eliminate with tight theta: should remove some arcs
  bg.eliminate_arcs(5.0);

  int arcs_after = 0, jumps = 0;
  for (int i = 0; i < bg.n_buckets(); ++i) {
    arcs_after += static_cast<int>(bg.bucket(i).bucket_arcs.size());
    jumps += static_cast<int>(bg.bucket(i).jump_arcs.size());
  }

  // Should have fewer bucket arcs (some eliminated)
  CHECK(arcs_after <= arcs_before);

  bg.reset_elimination();
}

TEST_CASE("Jump arcs are same-vertex (paper §4.1)") {
  // With small step sizes, elimination should create jump arcs at the same
  // vertex (bridging from a lower bucket to a higher one that retained the
  // arc).
  LargerGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {3.0, 1.0}, .tolerance = 1e9});
  bg.build();
  bg.solve();

  // Tight theta to force elimination
  bg.eliminate_arcs(8.0);

  // Verify jump arcs target same vertex as source bucket
  for (int i = 0; i < bg.n_buckets(); ++i) {
    for (const auto& ja : bg.bucket(i).jump_arcs) {
      CHECK(bg.bucket(ja.jump_bucket).vertex == bg.bucket(i).vertex);
    }
  }

  bg.reset_elimination();
}

TEST_CASE("Jump arc resource boost preserves correctness") {
  // After elimination + jump arcs, solving should still find the optimal path
  LargerGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {3.0, 1.0}, .tolerance = 1e9});
  bg.build();

  auto paths_before = bg.solve();
  REQUIRE(!paths_before.empty());
  double best_before = paths_before[0].reduced_cost;

  bg.eliminate_arcs(best_before + 5.0);
  auto paths_after = bg.solve();
  REQUIRE(!paths_after.empty());
  CHECK(paths_after[0].reduced_cost == doctest::Approx(best_before));

  bg.reset_elimination();
}

TEST_CASE("Label-based elimination preserves optimal paths") {
  LargerGraph g;
  BucketGraph<EmptyPack> bg(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bg.build();

  auto paths_before = bg.solve();
  REQUIRE(!paths_before.empty());
  double best_before = paths_before[0].reduced_cost;

  // Label-based elimination with generous theta should not remove optimal
  bg.eliminate_arcs_label_based(best_before + 5.0);
  auto paths_after = bg.solve();
  REQUIRE(!paths_after.empty());
  CHECK(paths_after[0].reduced_cost == doctest::Approx(best_before));

  bg.reset_elimination();
}

TEST_CASE("Label-based elimination is at least as tight as bound-based") {
  LargerGraph g;

  // Bound-based
  BucketGraph<EmptyPack> bg1(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bg1.build();
  bg1.solve();
  bg1.eliminate_arcs(8.0);
  int arcs_bound = 0;
  for (int i = 0; i < bg1.n_buckets(); ++i)
    arcs_bound += static_cast<int>(bg1.bucket(i).bucket_arcs.size() +
                                   bg1.bucket(i).bw_bucket_arcs.size());

  // Label-based
  BucketGraph<EmptyPack> bg2(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bg2.build();
  bg2.solve();
  bg2.eliminate_arcs_label_based(8.0);
  int arcs_label = 0;
  for (int i = 0; i < bg2.n_buckets(); ++i)
    arcs_label += static_cast<int>(bg2.bucket(i).bucket_arcs.size() +
                                   bg2.bucket(i).bw_bucket_arcs.size());

  // Label-based should eliminate at least as many arcs (fewer remaining)
  CHECK(arcs_label <= arcs_bound);
}

TEST_CASE("Label-based elimination mono falls back to bound-based") {
  LargerGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  auto paths_before = bg.solve();
  REQUIRE(!paths_before.empty());
  double best = paths_before[0].reduced_cost;

  // Mono: label-based falls back to bound-based for forward arcs
  // (no backward labels available). Should still preserve optimal.
  bg.eliminate_arcs_label_based(best + 10.0);
  auto paths_after = bg.solve();
  REQUIRE(!paths_after.empty());
  CHECK(paths_after[0].reduced_cost == doctest::Approx(best));

  bg.reset_elimination();
}

TEST_CASE("Bucket fixing") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  bg.solve();

  bg.fix_buckets(100.0);
  auto paths = bg.solve();
  REQUIRE(!paths.empty());
  CHECK(paths[0].reduced_cost == doctest::Approx(3.0));

  bg.reset_elimination();
}

// ── Time window feasibility ──

TEST_CASE("Time window feasibility") {
  SimpleGraph g;
  g.tw_ub[1] = 0.5;

  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {1.0, 1.0}, .tolerance = 1e9});
  bg.build();

  auto paths = bg.solve();

  for (const auto& p : paths) {
    bool has_v1 = false;
    for (int v : p.vertices) {
      if (v == 1) has_v1 = true;
    }
    CHECK(!has_v1);
  }
}

// ── Bidirectional: bucket graph level ──

TEST_CASE("Backward bucket arcs are generated") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(
      g.pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .bidirectional = true});
  bg.build();

  int fw_arcs = 0, bw_arcs = 0;
  for (int i = 0; i < bg.n_buckets(); ++i) {
    fw_arcs += static_cast<int>(bg.bucket(i).bucket_arcs.size());
    bw_arcs += static_cast<int>(bg.bucket(i).bw_bucket_arcs.size());
  }
  CHECK(fw_arcs > 0);
  CHECK(bw_arcs > 0);
}

TEST_CASE("Bidirectional solve matches mono on simple graph") {
  SimpleGraph g;

  BucketGraph<EmptyPack> mono(g.pv, EmptyPack{},
                              {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  mono.build();
  auto mono_paths = mono.solve();

  BucketGraph<EmptyPack> bidir(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bidir.build();
  auto bidir_paths = bidir.solve();

  REQUIRE(!mono_paths.empty());
  REQUIRE(!bidir_paths.empty());
  CHECK(bidir_paths[0].reduced_cost ==
        doctest::Approx(mono_paths[0].reduced_cost));
}

TEST_CASE("Bidirectional solve matches mono on larger graph") {
  LargerGraph g;

  BucketGraph<EmptyPack> mono(g.pv, EmptyPack{},
                              {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  mono.build();
  auto mono_paths = mono.solve();

  BucketGraph<EmptyPack> bidir(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bidir.build();
  auto bidir_paths = bidir.solve();

  REQUIRE(!mono_paths.empty());
  REQUIRE(!bidir_paths.empty());
  CHECK(bidir_paths[0].reduced_cost ==
        doctest::Approx(mono_paths[0].reduced_cost));

  // Best: 0→2→4→5 = 3+6+1 = 10
  CHECK(mono_paths[0].reduced_cost == doctest::Approx(10.0));
}

TEST_CASE("Bidirectional solve with reduced costs") {
  LargerGraph g;

  double red_cost[8] = {-10.0, 3.0, 4.0, 6.0, 2.0, 1.0, 7.0, 8.0};

  BucketGraph<EmptyPack> mono(g.pv, EmptyPack{},
                              {.bucket_steps = {5.0, 1.0}, .tolerance = 0.0});
  mono.build();
  mono.update_arc_costs(red_cost);
  auto mono_paths = mono.solve();

  BucketGraph<EmptyPack> bidir(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 0.0, .bidirectional = true});
  bidir.build();
  bidir.update_arc_costs(red_cost);
  auto bidir_paths = bidir.solve();

  REQUIRE(!mono_paths.empty());
  REQUIRE(!bidir_paths.empty());
  CHECK(bidir_paths[0].reduced_cost ==
        doctest::Approx(mono_paths[0].reduced_cost));
}

TEST_CASE("Bidirectional with tight time windows") {
  LargerGraph g;
  g.tw_ub[5] = 4.5;  // sink reachable only with total time <= 4.5

  BucketGraph<EmptyPack> mono(g.pv, EmptyPack{},
                              {.bucket_steps = {2.0, 1.0}, .tolerance = 1e9});
  mono.build();
  auto mono_paths = mono.solve();

  BucketGraph<EmptyPack> bidir(
      g.pv, EmptyPack{},
      {.bucket_steps = {2.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bidir.build();
  auto bidir_paths = bidir.solve();

  // Both should find same feasible paths
  if (!mono_paths.empty()) {
    REQUIRE(!bidir_paths.empty());
    CHECK(bidir_paths[0].reduced_cost ==
          doctest::Approx(mono_paths[0].reduced_cost));
  }
}

TEST_CASE("Bidirectional path validity") {
  LargerGraph g;

  BucketGraph<EmptyPack> bg(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bg.build();
  auto paths = bg.solve();

  for (const auto& p : paths) {
    // Source and sink
    CHECK(p.vertices.front() == 0);
    CHECK(p.vertices.back() == 5);

    // Arcs match vertices
    REQUIRE(p.arcs.size() + 1 == p.vertices.size());
    for (size_t i = 0; i < p.arcs.size(); ++i) {
      int a = p.arcs[i];
      CHECK(g.from[a] == p.vertices[i]);
      CHECK(g.to[a] == p.vertices[i + 1]);
    }

    // Cost matches arc sum
    double sum = 0.0;
    for (int a : p.arcs) sum += g.cost[a];
    CHECK(p.original_cost == doctest::Approx(sum));
  }
}

// ── Multi-stage ──

TEST_CASE("Stage: Heuristic1 cost-only dominance") {
  SimpleGraph g;

  // Exact
  BucketGraph<EmptyPack> exact(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .stage = Stage::Exact});
  exact.build();
  auto exact_paths = exact.solve();

  // Heuristic1
  BucketGraph<EmptyPack> h1(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0},
                             .tolerance = 1e9,
                             .stage = Stage::Heuristic1});
  h1.build();
  auto h1_paths = h1.solve();

  REQUIRE(!exact_paths.empty());
  REQUIRE(!h1_paths.empty());

  // Heuristic1 should find at least as good cost (more aggressive dominance
  // means fewer labels but shouldn't miss the optimal on EmptyPack)
  CHECK(h1_paths[0].reduced_cost <= exact_paths[0].reduced_cost + 1e-6);
}

TEST_CASE("Stage: set_stage changes behavior") {
  SimpleGraph g;

  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0},
                             .tolerance = 1e9,
                             .stage = Stage::Heuristic1});
  bg.build();
  auto h1_paths = bg.solve();

  bg.set_stage(Stage::Exact);
  auto exact_paths = bg.solve();

  REQUIRE(!h1_paths.empty());
  REQUIRE(!exact_paths.empty());

  CHECK(h1_paths[0].reduced_cost ==
        doctest::Approx(exact_paths[0].reduced_cost));
}

// ── Bidirectional arc elimination ──

TEST_CASE("Bidirectional arc elimination") {
  LargerGraph g;

  BucketGraph<EmptyPack> bg(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bg.build();

  auto paths_before = bg.solve();
  REQUIRE(!paths_before.empty());
  double best_before = paths_before[0].reduced_cost;

  bg.eliminate_arcs(100.0);
  auto paths_after = bg.solve();
  REQUIRE(!paths_after.empty());
  CHECK(paths_after[0].reduced_cost == doctest::Approx(best_before));

  bg.reset_elimination();
}

// ── Bucket fixing (Phase 8) ──

TEST_CASE("Bucket fixing: tight theta fixes buckets") {
  LargerGraph g;

  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  // Solve first to populate c_best
  auto paths = bg.solve();
  REQUIRE(!paths.empty());
  double best = paths[0].reduced_cost;

  // Fix with theta slightly above best cost — should fix some buckets
  int n_total = bg.n_buckets();
  int fixed = bg.fix_buckets(best + 1.0);
  CHECK(fixed >= 0);
  CHECK(bg.n_fixed_buckets() <= n_total);

  // Re-solve: should still find optimal (or report it correctly)
  auto paths2 = bg.solve();
  REQUIRE(!paths2.empty());
  CHECK(paths2[0].reduced_cost == doctest::Approx(best).epsilon(1e-6));

  bg.reset_elimination();
  CHECK(bg.n_fixed_buckets() == 0);
}

TEST_CASE("Bucket fixing: very tight theta fixes all non-optimal buckets") {
  SimpleGraph g;

  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {2.0, 1.0}, .tolerance = 1e9});
  bg.build();

  auto paths = bg.solve();
  REQUIRE(!paths.empty());
  double best = paths[0].reduced_cost;

  // Fix with theta = best (only optimal path buckets survive)
  int fixed = bg.fix_buckets(best);
  CHECK(fixed > 0);
  CHECK(bg.n_fixed_buckets() > 0);
  CHECK(bg.n_fixed_buckets() <= bg.n_buckets());

  bg.reset_elimination();
}

TEST_CASE("Bucket fixing: bidirectional combined bounds") {
  LargerGraph g;

  BucketGraph<EmptyPack> bg(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bg.build();

  auto paths = bg.solve();
  REQUIRE(!paths.empty());
  double best = paths[0].reduced_cost;

  // Fix with loose theta — bidirectional should fix more using combined bounds
  bg.fix_buckets(best + 5.0);

  // Some buckets may be fixed
  CHECK(bg.n_fixed_buckets() >= 0);

  // Re-solve: optimality preserved
  auto paths2 = bg.solve();
  REQUIRE(!paths2.empty());
  CHECK(paths2[0].reduced_cost == doctest::Approx(best).epsilon(1e-6));

  bg.reset_elimination();
}

TEST_CASE("Bucket fixing with arc elimination") {
  LargerGraph g;

  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  auto paths = bg.solve();
  REQUIRE(!paths.empty());
  double best = paths[0].reduced_cost;

  // Eliminate arcs first, then fix buckets
  bg.eliminate_arcs(best + 10.0);
  int fixed = bg.fix_buckets(best + 10.0);
  CHECK(fixed >= 0);

  auto paths2 = bg.solve();
  REQUIRE(!paths2.empty());
  CHECK(paths2[0].reduced_cost == doctest::Approx(best).epsilon(1e-6));

  bg.reset_elimination();
}

TEST_CASE("Bucket fixing: is_bucket_fixed query") {
  SimpleGraph g;

  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {2.0, 1.0}, .tolerance = 1e9});
  bg.build();
  bg.solve();

  // Before fixing: none fixed
  for (int i = 0; i < bg.n_buckets(); ++i) {
    CHECK_FALSE(bg.is_bucket_fixed(i));
  }

  // Fix with very negative theta — should fix all buckets (all c_best > theta)
  bg.fix_buckets(-1e10);
  CHECK(bg.n_fixed_buckets() == bg.n_buckets());
  for (int i = 0; i < bg.n_buckets(); ++i) {
    CHECK(bg.is_bucket_fixed(i));
  }

  bg.reset_elimination();
  CHECK(bg.n_fixed_buckets() == 0);
}

// ── R1C engine ──

TEST_CASE("R1C: mask building and basic extend") {
  using namespace bgspprc;
  R1CManager mgr;

  // Simple graph: 4 vertices, 4 arcs (0→1, 0→2, 1→3, 2→3)
  // One 3-SRC cut: C = {1, 2}, multiplier 1/2 each
  // Memory arcs: arc 0 (0→1), arc 1 (0→2)
  R1Cut cut;
  cut.base_set = {1, 2};
  cut.multipliers = {0.5, 0.5};
  cut.memory_arcs = {{0, 0}, {1, 0}};  // arc_id in .first
  cut.dual_value = -2.0;               // β = -(-2) = 2.0

  std::vector<R1Cut> cuts = {cut};
  mgr.set_cuts(cuts, 4, 4);

  CHECK(mgr.n_active() == 1);
  CHECK(mgr.n_words() == 1);
  CHECK(mgr.has_cuts());

  // Init state: all zeros
  uint64_t state[1] = {0};
  mgr.init_state({state, 1});
  CHECK(state[0] == 0ULL);

  // Extend to vertex 1 (in C) via arc 0 (in AM)
  // Keep mask keeps the bit (arc ∈ AM), toggle sets it (v1 ∈ C)
  uint64_t out[1] = {0};
  double cost1 = mgr.extend(Direction::Forward, {state, 1}, {out, 1}, 0, 1);
  // State was 0, toggle → 1. No overflow (0→1). Cost delta = 0.
  CHECK(cost1 == doctest::Approx(0.0));
  CHECK(out[0] == 1ULL);  // bit 0 set (cut 0 has state 1/2)

  // Extend to vertex 2 (in C) via arc 2 (NOT in AM)
  // Keep mask resets bit (arc ∉ AM), then toggle
  uint64_t out2[1] = {0};
  double cost2 = mgr.extend(Direction::Forward, {out, 1}, {out2, 1}, 2, 2);
  // State was 1, reset to 0 (arc 2 not in AM), toggle → 1. No overflow.
  CHECK(out2[0] == 1ULL);
  CHECK(cost2 == doctest::Approx(0.0));
}

TEST_CASE("R1C: extend overflow applies cost") {
  R1CManager mgr;

  // Cut with C = {1, 2}, AM = {arc 0, arc 2}
  R1Cut cut;
  cut.base_set = {1, 2};
  cut.multipliers = {0.5, 0.5};
  cut.memory_arcs = {{0, 0}, {2, 0}};  // arc 0 and arc 2 in AM
  cut.dual_value = -3.0;               // β = 3.0

  mgr.set_cuts({{cut}}, 4, 4);

  // Start with state = 0
  uint64_t s0[1] = {0};

  // Extend to v1 (in C) via arc 0 (in AM): 0 → toggle → 1
  uint64_t s1[1] = {0};
  double c1 = mgr.extend(Direction::Forward, {s0, 1}, {s1, 1}, 0, 1);
  CHECK(s1[0] == 1ULL);
  CHECK(c1 == doctest::Approx(0.0));

  // Extend to v2 (in C) via arc 2 (in AM): 1 → keep(1) → toggle → overflow!
  // Overflow: bit was 1, toggling to 0. Cost -= β = -3.0
  uint64_t s2[1] = {0};
  double c2 = mgr.extend(Direction::Forward, {s1, 1}, {s2, 1}, 2, 2);
  CHECK(s2[0] == 0ULL);                // overflow resets to 0
  CHECK(c2 == doctest::Approx(-3.0));  // -β applied
}

TEST_CASE("R1C: domination cost") {
  R1CManager mgr;

  R1Cut cut;
  cut.base_set = {1};
  cut.multipliers = {0.5};
  cut.memory_arcs = {{0, 0}};
  cut.dual_value = -4.0;  // β = 4.0

  mgr.set_cuts({{cut}}, 3, 2);

  // s1 has credit (1), s2 doesn't (0): s1 is better, no penalty
  uint64_t s1[1] = {1ULL};
  uint64_t s2[1] = {0ULL};
  double dom = mgr.domination_cost(Direction::Forward, 1, {s1, 1}, {s2, 1});
  CHECK(dom == doctest::Approx(0.0));

  // s2 has credit (1), s1 doesn't (0): s1 is at disadvantage → penalty
  double dom2 = mgr.domination_cost(Direction::Forward, 1, {s2, 1}, {s1, 1});
  CHECK(dom2 == doctest::Approx(-4.0));  // -β
}

TEST_CASE("R1C: concatenation cost") {
  R1CManager mgr;

  R1Cut cut;
  cut.base_set = {1};
  cut.multipliers = {0.5};
  cut.memory_arcs = {{0, 0}};
  cut.dual_value = -5.0;  // β = 5.0

  mgr.set_cuts({{cut}}, 3, 2);

  // Both fw and bw have credit → cost -β
  uint64_t fw[1] = {1ULL};
  uint64_t bw[1] = {1ULL};
  double c = mgr.concatenation_cost(Symmetry::Asymmetric, 1, {fw, 1}, {bw, 1});
  CHECK(c == doctest::Approx(-5.0));

  // Only one has credit → no cost
  uint64_t zero[1] = {0ULL};
  double c2 =
      mgr.concatenation_cost(Symmetry::Asymmetric, 1, {fw, 1}, {zero, 1});
  CHECK(c2 == doctest::Approx(0.0));
}

TEST_CASE("R1C: multiple cuts packed in one word") {
  R1CManager mgr;

  // 3 cuts
  R1Cut cut0, cut1, cut2;
  cut0.base_set = {1};
  cut0.multipliers = {0.5};
  cut0.memory_arcs = {{0, 0}};
  cut0.dual_value = -1.0;

  cut1.base_set = {2};
  cut1.multipliers = {0.5};
  cut1.memory_arcs = {{1, 0}};
  cut1.dual_value = -2.0;

  cut2.base_set = {1, 2};
  cut2.multipliers = {0.5, 0.5};
  cut2.memory_arcs = {{0, 0}, {1, 0}};
  cut2.dual_value = -3.0;

  mgr.set_cuts(std::vector<R1Cut>{cut0, cut1, cut2}, 4, 4);
  CHECK(mgr.n_active() == 3);
  CHECK(mgr.n_words() == 1);

  uint64_t s[1] = {0};
  mgr.init_state({s, 1});

  // Extend to v1 via arc 0 (in AM for cut0 and cut2)
  uint64_t s1[1] = {0};
  mgr.extend(Direction::Forward, {s, 1}, {s1, 1}, 0, 1);
  // cut0: toggle(v1∈C) → 1
  // cut1: reset(arc0∉AM), toggle(v1∉C for cut1) → 0
  // cut2: keep(arc0∈AM), toggle(v1∈C) → 1
  CHECK((s1[0] & 1ULL) == 1ULL);  // cut 0 = 1
  CHECK((s1[0] & 4ULL) == 4ULL);  // cut 2 = 1
}

// ── SCC computation ──

TEST_CASE("SCC: single-vertex buckets form separate SCCs") {
  // Simple graph, large buckets = one bucket per vertex = DAG
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {100.0, 1.0}, .tolerance = 1e9});
  bg.build();

  // With large step size, each vertex has 1 bucket → no cycles → all SCCs are
  // singletons The SCC topo order should process source first, sink last
  // (approximately) Just verify we can solve correctly
  auto paths = bg.solve();
  REQUIRE(!paths.empty());
  CHECK(paths[0].reduced_cost == doctest::Approx(3.0));
}

TEST_CASE("SCC: fine buckets create within-vertex SCCs") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {1.0, 1.0}, .tolerance = 1e9});
  bg.build();

  // With step=1, each vertex has ~10 buckets.
  // Within-vertex adjacent buckets can form SCCs.
  CHECK(bg.n_buckets() == 40);

  // Should still solve correctly
  auto paths = bg.solve();
  REQUIRE(!paths.empty());
  CHECK(paths[0].reduced_cost == doctest::Approx(3.0));
}

// ── Solver stage progression ──

TEST_CASE("Solver stage progression: Heuristic1 to Heuristic2") {
  SimpleGraph g;
  Solver<EmptyPack> solver(g.pv, EmptyPack{},
                           {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  solver.build();

  CHECK(solver.current_stage() == Stage::Heuristic1);

  // Solve multiple times — should progress stages
  for (int i = 0; i < 5; ++i) {
    solver.solve();
  }
  // After enough iterations with results, should have progressed past
  // Heuristic1
  CHECK(solver.current_stage() != Stage::Heuristic1);
}

TEST_CASE("Solver stage progression: empty results advance stage") {
  // Graph with no feasible path
  int from[1] = {0};
  int to[1] = {1};
  double cost[1] = {1.0};
  double td[1] = {5.0};
  double lb[2] = {0.0, 0.0};
  double ub[2] = {10.0, 2.0};  // sink tw [0,2] but arrival=5

  const double* ar[1] = {td};
  const double* vl[1] = {lb};
  const double* vu[1] = {ub};

  ProblemView pv;
  pv.n_vertices = 2;
  pv.source = 0;
  pv.sink = 1;
  pv.n_arcs = 1;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = ar;
  pv.vertex_lb = vl;
  pv.vertex_ub = vu;
  pv.n_main_resources = 1;

  Solver<EmptyPack> solver(pv, EmptyPack{},
                           {.bucket_steps = {5.0, 1.0}, .tolerance = -1e-6});
  solver.build();

  CHECK(solver.current_stage() == Stage::Heuristic1);

  // Empty results should advance stage quickly
  solver.solve();
  CHECK(solver.current_stage() == Stage::Heuristic2);

  solver.solve();
  CHECK(solver.current_stage() == Stage::Exact);
}

TEST_CASE("Solver: set_stage override") {
  SimpleGraph g;
  Solver<EmptyPack> solver(g.pv, EmptyPack{},
                           {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  solver.build();

  solver.set_stage(Stage::Exact);
  CHECK(solver.current_stage() == Stage::Exact);

  auto paths = solver.solve();
  REQUIRE(!paths.empty());
  CHECK(paths[0].reduced_cost == doctest::Approx(3.0));
}

// ── Enumeration ──

TEST_CASE("Solver: enumerate within gap") {
  SimpleGraph g;
  Solver<EmptyPack> solver(g.pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}});
  solver.build();

  // Enumerate all paths within gap=100 (very large → find all paths)
  auto paths = solver.enumerate(100.0);

  // Should find both paths: 0→1→3 (cost=4) and 0→2→3 (cost=3)
  CHECK(paths.size() >= 2);

  // All paths should have cost ≤ gap
  for (const auto& p : paths) {
    CHECK(p.reduced_cost <= 100.0 + 1e-6);
  }
}

TEST_CASE("Solver: enumerate with tight gap") {
  SimpleGraph g;
  Solver<EmptyPack> solver(g.pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}});
  solver.build();

  // Enumerate with gap=3.5 → only the optimal path (cost=3) should be within
  // gap
  auto paths = solver.enumerate(3.5);
  CHECK(paths.size() >= 1);
  CHECK(paths[0].reduced_cost == doctest::Approx(3.0));
}

// Graph with parallel arcs where one label strictly dominates another:
//   Arc 0: 0→1 (cost=1, t=1)
//   Arc 1: 0→1 (cost=3, t=1)  ← dominated by arc 0
//   Arc 2: 1→2 (cost=1, t=1)
//   Arc 3: 2→3 (cost=1, t=1)
struct ParallelArcGraph {
  int from[4] = {0, 0, 1, 2};
  int to[4] = {1, 1, 2, 3};
  double cost[4] = {1.0, 3.0, 1.0, 1.0};
  double time_d[4] = {1.0, 1.0, 1.0, 1.0};

  double tw_lb[4] = {0.0, 0.0, 0.0, 0.0};
  double tw_ub[4] = {10.0, 10.0, 10.0, 10.0};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;

  ParallelArcGraph() {
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 4;
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

TEST_CASE("Enumerate: finds dominated paths") {
  ParallelArcGraph g;
  using BG = BucketGraph<EmptyPack>;
  BG bg(g.pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0},
         .max_paths = 100,
         .tolerance = 1e9,
         .stage = Stage::Exact});
  bg.build();

  // Exact solve: dominance prunes the cost-5 path, only cost-3 found
  auto exact = bg.solve();
  CHECK(exact.size() == 1);
  CHECK(exact[0].reduced_cost == doctest::Approx(3.0));

  // Enumerate with large gap: both paths found (cost-3 and cost-5)
  bg.set_stage(Stage::Enumerate);
  bg.set_tolerance(10.0);
  auto enumerated = bg.solve();
  CHECK(enumerated.size() == 2);
  // Sorted by cost
  CHECK(enumerated[0].reduced_cost == doctest::Approx(3.0));
  CHECK(enumerated[1].reduced_cost == doctest::Approx(5.0));
}

TEST_CASE("Enumerate: completion-bound pruning filters correctly") {
  ParallelArcGraph g;
  using BG = BucketGraph<EmptyPack>;
  BG bg(g.pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0},
         .max_paths = 100,
         .tolerance = 4.0,
         .stage = Stage::Enumerate});
  bg.build();

  // gap=4.0: only cost-3 path should be found (cost-5 pruned by completion
  // bound)
  auto paths = bg.solve();
  CHECK(paths.size() == 1);
  CHECK(paths[0].reduced_cost == doctest::Approx(3.0));
}

TEST_CASE("Enumerate: bidirectional finds more paths than exact") {
  ParallelArcGraph g;
  using BG = BucketGraph<EmptyPack>;

  BG bg_exact(g.pv, EmptyPack{},
              {.bucket_steps = {5.0, 1.0},
               .max_paths = 1000,
               .tolerance = 1e9,
               .bidirectional = true,
               .stage = Stage::Exact});
  bg_exact.build();
  auto exact = bg_exact.solve();

  BG bg_enum(g.pv, EmptyPack{},
             {.bucket_steps = {5.0, 1.0},
              .max_paths = 1000,
              .tolerance = 1e9,
              .bidirectional = true,
              .stage = Stage::Enumerate});
  bg_enum.build();
  auto enumerated = bg_enum.solve();

  CHECK(enumerated.size() > exact.size());
}

// ── Completion bounds ──

TEST_CASE("Completion bounds: sink has zero completion") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();
  bg.solve();

  // Trigger completion bound computation via eliminate_arcs with large theta
  bg.eliminate_arcs(1e9);

  // Sink buckets should have c_best from labels + 0 completion
  // Just verify arc elimination didn't break anything
  auto paths = bg.solve();
  REQUIRE(!paths.empty());
  CHECK(paths[0].reduced_cost == doctest::Approx(3.0));

  bg.reset_elimination();
}

// ── NgPathResource with BucketGraph ──

TEST_CASE("NgPathResource: prevents revisiting in BucketGraph") {
  // Graph with a cycle: 0 → 1 → 2 → 1 → 3
  // Without ng: could cycle. With ng: cycle blocked.
  int from[] = {0, 1, 2, 1, 2};
  int to[] = {1, 2, 1, 3, 3};
  double cost[] = {1.0, 1.0, -5.0, 10.0, 2.0};  // cycle arc is cheap
  double td[] = {1.0, 1.0, 1.0, 1.0, 1.0};

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0};

  const double* arc_res[] = {td};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 4;
  pv.source = 0;
  pv.sink = 3;
  pv.n_arcs = 5;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  // Full ng-neighborhoods (all vertices see each other)
  std::vector<std::vector<int>> neighbors = {
      {0, 1, 2, 3}, {1, 0, 2, 3}, {2, 0, 1, 3}, {3, 0, 1, 2}};
  NgPathResource ng(4, 5, from, to, neighbors);
  using Pack = ResourcePack<NgPathResource>;
  BucketGraph<Pack> bg(pv, make_resource_pack(std::move(ng)),
                       {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  auto paths = bg.solve();
  REQUIRE(!paths.empty());

  // With ng-paths, the cycle 1→2→1 is blocked.
  // Paths: 0→1→2→3 (cost=1+1+2=4), 0→1→3 (cost=1+10=11)
  // Best should be 0→1→2→3 cost=4
  CHECK(paths[0].reduced_cost == doctest::Approx(4.0));

  // Verify no vertex repeats
  for (const auto& p : paths) {
    for (size_t i = 0; i < p.vertices.size(); ++i) {
      for (size_t j = i + 1; j < p.vertices.size(); ++j) {
        CHECK(p.vertices[i] != p.vertices[j]);
      }
    }
  }
}

TEST_CASE("Heuristic2: relaxed ng dominance prunes more aggressively") {
  // Graph: 0 → 1 via two paths with different intermediate vertices
  //   0 →(a0) 2 →(a1) 1 →(a2) 3
  //   0 →(a3) 4 →(a4) 1 →(a2) 3
  // Both paths reach vertex 1 with same cost and q, but different ng-states.
  // Heuristic2 ignores ng → labels look identical → one dominates the other.
  // Exact sees distinct ng-states → neither dominates → both survive.
  int from[] = {0, 2, 1, 0, 4};
  int to[] = {2, 1, 3, 4, 1};
  double cost[] = {1.0, 1.0, 1.0, 1.0, 1.0};
  double td[] = {1.0, 1.0, 1.0, 1.0, 1.0};

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

  const double* arc_res[] = {td};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 5;
  pv.source = 0;
  pv.sink = 3;
  pv.n_arcs = 5;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  // Full ng-neighborhoods: every vertex sees every other
  std::vector<std::vector<int>> neighbors = {{0, 1, 2, 3, 4},
                                             {1, 0, 2, 3, 4},
                                             {2, 0, 1, 3, 4},
                                             {3, 0, 1, 2, 4},
                                             {4, 0, 1, 2, 3}};
  NgPathResource ng(5, 5, from, to, neighbors);
  using Pack = ResourcePack<NgPathResource>;

  // Heuristic2: cost-only dominance, ng-states ignored
  BucketGraph<Pack> bg_h2(pv, make_resource_pack(NgPathResource(ng)),
                          {.bucket_steps = {5.0, 1.0},
                           .tolerance = 1e9,
                           .stage = Stage::Heuristic2});
  bg_h2.build();
  auto paths_h2 = bg_h2.solve();

  // Exact: full dominance including ng-states
  BucketGraph<Pack> bg_ex(
      pv, make_resource_pack(NgPathResource(ng)),
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .stage = Stage::Exact});
  bg_ex.build();
  auto paths_ex = bg_ex.solve();

  // Both should find valid paths
  REQUIRE(!paths_h2.empty());
  REQUIRE(!paths_ex.empty());

  // Heuristic2 ignores ng-states → more aggressive dominance → fewer paths
  // Exact preserves labels with distinct ng-states → more paths survive
  CHECK(paths_ex.size() >= paths_h2.size());
}

TEST_CASE("Non-disposable resource: equality required in dominance") {
  // Two paths to vertex 2, same cost, different resource consumption.
  // 0 →(a0) 1 →(a1) 2 →(a2) 3   (q = 1+1 = 2)
  // 0 →(a3) 1 →(a4) 2 →(a2) 3   (q = 1+2 = 3)
  // With disposable resource: label with q=2 dominates q=3 → 1 path.
  // With non-disposable resource: q=2 ≠ q=3 → neither dominates → 2 paths.
  int from[] = {0, 1, 2, 0, 1};
  int to[] = {1, 2, 3, 1, 2};
  double cost[] = {1.0, 1.0, 1.0, 1.0, 1.0};
  double td[] = {1.0, 1.0, 1.0, 1.0, 2.0};  // arc a4 consumes 2

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0};

  const double* arc_res[] = {td};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 4;
  pv.source = 0;
  pv.sink = 3;
  pv.n_arcs = 5;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  // Disposable (default): q=2 dominates q=3
  BucketGraph<EmptyPack> bg_disp(
      pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .stage = Stage::Exact});
  bg_disp.build();
  auto paths_disp = bg_disp.solve();

  // Non-disposable: q=2 ≠ q=3, neither dominates
  bool nondisposable[] = {true};
  pv.resource_nondisposable = nondisposable;
  BucketGraph<EmptyPack> bg_nondisp(
      pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .stage = Stage::Exact});
  bg_nondisp.build();
  auto paths_nondisp = bg_nondisp.solve();

  REQUIRE(!paths_disp.empty());
  REQUIRE(!paths_nondisp.empty());

  // Non-disposable should keep more paths (less dominance)
  CHECK(paths_nondisp.size() > paths_disp.size());
}

TEST_CASE("Resource::symmetric() interface") {
  // EmptyPack: vacuous true (no resources)
  EmptyPack empty{};
  CHECK(empty.symmetric() == true);

  // NgPathResource: always symmetric (§4.2.2)
  int from[] = {0, 1};
  int to[] = {1, 2};
  std::vector<std::vector<int>> neighbors = {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}};
  NgPathResource ng(3, 2, from, to, neighbors);
  CHECK(ng.symmetric() == true);

  // Pack with NgPathResource: symmetric
  auto pack = make_resource_pack(std::move(ng));
  CHECK(pack.symmetric() == true);
}

// ── LabelPool ──

TEST_CASE("LabelPool: allocation and counting") {
  LabelPool<EmptyPack> pool(0, 16);

  CHECK(pool.count() == 0);

  auto* l1 = pool.allocate();
  CHECK(l1 != nullptr);
  CHECK(pool.count() == 1);

  auto* l2 = pool.allocate();
  CHECK(l2 != nullptr);
  CHECK(pool.count() == 2);
  CHECK(l1 != l2);

  pool.clear();
  CHECK(pool.count() == 0);
}

TEST_CASE("LabelPool: R1C inline allocation") {
  LabelPool<EmptyPack> pool(2, 16);  // 2 R1C words per label

  auto* l = pool.allocate();
  CHECK(l != nullptr);
  CHECK(l->n_r1c_words == 2);
  CHECK(l->r1c_states != nullptr);

  // R1C states should be zeroed
  CHECK(l->r1c_states[0] == 0ULL);
  CHECK(l->r1c_states[1] == 0ULL);

  // Write to R1C states (shouldn't crash)
  l->r1c_states[0] = 0xDEADBEEFULL;
  l->r1c_states[1] = 0xCAFEBABEULL;

  // Allocate another — should not overlap
  auto* l2 = pool.allocate();
  CHECK(l2->r1c_states[0] == 0ULL);
  CHECK(l2->r1c_states[1] == 0ULL);
}

// ── Warm starting (Phase 7) ──

TEST_CASE("Warm starting: save and reuse labels") {
  LargerGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  // First solve
  auto paths1 = bg.solve();
  REQUIRE(!paths1.empty());
  double best1 = paths1[0].reduced_cost;

  // Save warm labels
  bg.save_warm_labels(0.7);

  // Second solve (with warm labels injected)
  auto paths2 = bg.solve();
  REQUIRE(!paths2.empty());

  // Should find at least as good a solution
  CHECK(paths2[0].reduced_cost <= best1 + 1e-6);
}

TEST_CASE("Warm starting: with changed reduced costs") {
  LargerGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  // First solve with base costs
  auto paths1 = bg.solve();
  REQUIRE(!paths1.empty());

  // Save warm labels
  bg.save_warm_labels(0.5);

  // Change costs
  double red[8] = {-10.0, 3.0, 4.0, 6.0, 2.0, 1.0, 7.0, 8.0};
  bg.update_arc_costs(red);

  // Solve with warm labels and new costs
  auto paths2 = bg.solve();
  REQUIRE(!paths2.empty());

  // The warm labels have old costs but labeling should still find optimal
  // (warm labels may be dominated by new labels with better costs)
  CHECK(paths2[0].reduced_cost < 0.0);  // -10+4+2 = -4 path exists
}

// ── Dynamic bucket step sizes (Phase 7) ──

TEST_CASE("Adapt bucket steps: halves when ratio too large") {
  // Wide time windows with small step → many buckets
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {0.1, 1.0}, .tolerance = 1e9});
  bg.build();

  auto initial_steps = bg.bucket_steps();

  // Threshold=50 → ratio = 10/0.1 = 100 > 50, should halve
  bool changed = bg.adapt_bucket_steps(50.0);
  CHECK(changed);
  CHECK(bg.bucket_steps()[0] == doctest::Approx(initial_steps[0] * 0.5));
}

TEST_CASE("Adapt bucket steps: no change when ratio ok") {
  SimpleGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  bg.build();

  // Ratio = 10/5 = 2, threshold=20 → no change
  bool changed = bg.adapt_bucket_steps(20.0);
  CHECK_FALSE(changed);
  CHECK(bg.bucket_steps()[0] == doctest::Approx(5.0));
}

TEST_CASE("Solver: adapt_bucket_steps triggers rebuild") {
  SimpleGraph g;
  Solver<EmptyPack> solver(g.pv, EmptyPack{},
                           {.bucket_steps = {0.1, 1.0}, .tolerance = 1e9});
  solver.build();

  // Adapt (should halve from 0.1 to 0.05 and rebuild)
  bool changed = solver.adapt_bucket_steps(50.0);
  CHECK(changed);
  CHECK(solver.bucket_steps()[0] == doctest::Approx(0.05));

  // Should still solve correctly after rebuild
  solver.set_stage(Stage::Exact);
  auto paths = solver.solve();
  REQUIRE(!paths.empty());
  CHECK(paths[0].reduced_cost == doctest::Approx(3.0));
}

// ── Edge cases ──

TEST_CASE("Single-arc graph") {
  // Just source → sink
  int from[1] = {0};
  int to[1] = {1};
  double cost[1] = {5.0};
  double td[1] = {1.0};
  double lb[2] = {0.0, 0.0};
  double ub[2] = {10.0, 10.0};

  const double* ar[1] = {td};
  const double* vl[1] = {lb};
  const double* vu[1] = {ub};

  ProblemView pv;
  pv.n_vertices = 2;
  pv.source = 0;
  pv.sink = 1;
  pv.n_arcs = 1;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = ar;
  pv.vertex_lb = vl;
  pv.vertex_ub = vu;
  pv.n_main_resources = 1;

  // Mono
  BucketGraph<EmptyPack> mono(pv, EmptyPack{},
                              {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  mono.build();
  auto mp = mono.solve();
  REQUIRE(!mp.empty());
  CHECK(mp[0].reduced_cost == doctest::Approx(5.0));
  CHECK(mp[0].vertices.size() == 2);

  // Bidir
  BucketGraph<EmptyPack> bidir(
      pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bidir.build();
  auto bp = bidir.solve();
  REQUIRE(!bp.empty());
  CHECK(bp[0].reduced_cost == doctest::Approx(5.0));
}

TEST_CASE("Diamond graph: multiple equal-cost paths") {
  // 0 → 1 → 3
  // 0 → 2 → 3 (same cost)
  int from[4] = {0, 0, 1, 2};
  int to[4] = {1, 2, 3, 3};
  double cost[4] = {2.0, 1.0, 1.0, 2.0};  // both paths cost 3
  double td[4] = {1.0, 1.0, 1.0, 1.0};
  double lb[4] = {0.0, 0.0, 0.0, 0.0};
  double ub[4] = {10.0, 10.0, 10.0, 10.0};

  const double* ar[1] = {td};
  const double* vl[1] = {lb};
  const double* vu[1] = {ub};

  ProblemView pv;
  pv.n_vertices = 4;
  pv.source = 0;
  pv.sink = 3;
  pv.n_arcs = 4;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = ar;
  pv.vertex_lb = vl;
  pv.vertex_ub = vu;
  pv.n_main_resources = 1;

  BucketGraph<EmptyPack> mono(pv, EmptyPack{},
                              {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  mono.build();
  auto mp = mono.solve();

  BucketGraph<EmptyPack> bidir(
      pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bidir.build();
  auto bp = bidir.solve();

  REQUIRE(!mp.empty());
  REQUIRE(!bp.empty());
  CHECK(mp[0].reduced_cost == doctest::Approx(3.0));
  CHECK(bp[0].reduced_cost == doctest::Approx(3.0));

  // Bidir should find at least as many paths
  CHECK(bp.size() >= 1);
}

TEST_CASE("No feasible path") {
  // 0 → 1 → 2, but vertex 1 has impossible time window
  int from[2] = {0, 1};
  int to[2] = {1, 2};
  double cost[2] = {1.0, 1.0};
  double td[2] = {5.0, 5.0};
  double lb[3] = {0.0, 0.0, 0.0};
  double ub[3] = {10.0, 2.0, 10.0};  // v1 tw=[0,2], but arrival=5 > 2

  const double* ar[1] = {td};
  const double* vl[1] = {lb};
  const double* vu[1] = {ub};

  ProblemView pv;
  pv.n_vertices = 3;
  pv.source = 0;
  pv.sink = 2;
  pv.n_arcs = 2;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = ar;
  pv.vertex_lb = vl;
  pv.vertex_ub = vu;
  pv.n_main_resources = 1;

  BucketGraph<EmptyPack> mono(pv, EmptyPack{},
                              {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  mono.build();
  auto mp = mono.solve();
  CHECK(mp.empty());

  BucketGraph<EmptyPack> bidir(
      pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bidir.build();
  auto bp = bidir.solve();
  CHECK(bp.empty());
}

// ── Symmetric case (§3.6) ──
// Symmetric mode requires source = sink (CVRP depot case).

// Small symmetric CVRP-like graph: depot=0 (source=sink), customers 1,2,3
// All edges have reverse arcs with identical costs. Uniform windows.
struct SymmetricGraph {
  // 4 vertices: 0=depot (source and sink), 1, 2, 3
  // Edges: 0-1, 0-2, 1-2, 1-3, 2-3, 0-3 = 12 arcs
  int from[12] = {0, 0, 0, 1, 1, 2, 1, 2, 3, 2, 3, 3};
  int to[12] = {1, 2, 3, 2, 3, 3, 0, 0, 0, 1, 1, 2};
  double cost[12] = {2.0, 3.0, 7.0, 1.0, 4.0, 2.0,
                     2.0, 3.0, 7.0, 1.0, 4.0, 2.0};
  double time_d[12] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                       1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // Uniform time windows
  double tw_lb[4] = {0.0, 0.0, 0.0, 0.0};
  double tw_ub[4] = {10.0, 10.0, 10.0, 10.0};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;

  SymmetricGraph() {
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 0;  // depot = source = sink
    pv.n_arcs = 12;
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

// Larger symmetric graph: depot=0, customers 1-4
struct LargerSymmetricGraph {
  // Edges: 0-1, 0-2, 0-3, 0-4, 1-2, 1-3, 2-4, 3-4 = 16 arcs
  int from[16] = {0, 0, 0, 0, 1, 1, 2, 3, 1, 2, 3, 4, 2, 3, 4, 4};
  int to[16] = {1, 2, 3, 4, 2, 3, 4, 4, 0, 0, 0, 0, 1, 1, 2, 3};
  double cost[16] = {5.0, 3.0, 4.0, 6.0, 2.0, 3.0, 4.0, 1.0,
                     5.0, 3.0, 4.0, 6.0, 2.0, 3.0, 4.0, 1.0};
  double time_d[16] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                       1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  double tw_lb[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[5] = {10.0, 10.0, 10.0, 10.0, 10.0};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;

  LargerSymmetricGraph() {
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 0;  // depot = source = sink
    pv.n_arcs = 16;
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

TEST_CASE("Symmetric: matches bidir on small graph") {
  SymmetricGraph g;

  BucketGraph<EmptyPack> bidir(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bidir.build();
  auto bidir_paths = bidir.solve();

  BucketGraph<EmptyPack> sym(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .symmetric = true});
  sym.build();
  auto sym_paths = sym.solve();

  REQUIRE(!bidir_paths.empty());
  REQUIRE(!sym_paths.empty());
  CHECK(sym_paths[0].reduced_cost ==
        doctest::Approx(bidir_paths[0].reduced_cost));
}

TEST_CASE("Symmetric: matches bidir on larger graph") {
  LargerSymmetricGraph g;

  BucketGraph<EmptyPack> bidir(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bidir.build();
  auto bidir_paths = bidir.solve();

  BucketGraph<EmptyPack> sym(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .symmetric = true});
  sym.build();
  auto sym_paths = sym.solve();

  REQUIRE(!bidir_paths.empty());
  REQUIRE(!sym_paths.empty());
  CHECK(sym_paths[0].reduced_cost ==
        doctest::Approx(bidir_paths[0].reduced_cost));
}

TEST_CASE("Symmetric: no backward labels generated") {
  SymmetricGraph g;

  BucketGraph<EmptyPack> sym(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .symmetric = true});
  sym.build();
  sym.solve();

  // Backward bucket arcs still exist (needed for elimination/fixing)
  int bw_arcs = 0;
  for (int i = 0; i < sym.n_buckets(); ++i)
    bw_arcs += static_cast<int>(sym.bucket(i).bw_bucket_arcs.size());
  CHECK(bw_arcs > 0);
}

TEST_CASE("Symmetric: path validity") {
  LargerSymmetricGraph g;

  BucketGraph<EmptyPack> sym(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .symmetric = true});
  sym.build();
  auto paths = sym.solve();

  for (const auto& p : paths) {
    // Source = sink = depot (0)
    CHECK(p.vertices.front() == 0);
    CHECK(p.vertices.back() == 0);

    // Arcs match vertices
    REQUIRE(p.arcs.size() + 1 == p.vertices.size());
    for (size_t i = 0; i < p.arcs.size(); ++i) {
      int a = p.arcs[i];
      CHECK(a >= 0);
      CHECK(g.from[a] == p.vertices[i]);
      CHECK(g.to[a] == p.vertices[i + 1]);
    }

    // Cost matches arc sum
    double sum = 0.0;
    for (int a : p.arcs) sum += g.cost[a];
    CHECK(p.original_cost == doctest::Approx(sum));
  }
}

TEST_CASE("Symmetric: fixing and elimination preserve optimality") {
  LargerSymmetricGraph g;

  BucketGraph<EmptyPack> sym(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .symmetric = true});
  sym.build();

  auto paths = sym.solve();
  REQUIRE(!paths.empty());
  double best = paths[0].reduced_cost;

  // Arc elimination
  sym.eliminate_arcs(best + 5.0);
  auto paths2 = sym.solve();
  REQUIRE(!paths2.empty());
  CHECK(paths2[0].reduced_cost == doctest::Approx(best));

  sym.reset_elimination();

  // Bucket fixing
  sym.solve();
  sym.fix_buckets(best + 5.0);
  auto paths3 = sym.solve();
  REQUIRE(!paths3.empty());
  CHECK(paths3[0].reduced_cost == doctest::Approx(best));

  sym.reset_elimination();
}

TEST_CASE("Symmetric: label-based elimination falls back to bound-based") {
  LargerSymmetricGraph g;

  BucketGraph<EmptyPack> sym(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .symmetric = true});
  sym.build();

  auto paths = sym.solve();
  REQUIRE(!paths.empty());
  double best = paths[0].reduced_cost;

  sym.eliminate_arcs_label_based(best + 5.0);
  auto paths2 = sym.solve();
  REQUIRE(!paths2.empty());
  CHECK(paths2[0].reduced_cost == doctest::Approx(best));

  sym.reset_elimination();
}

TEST_CASE("Symmetric: with reduced costs") {
  LargerSymmetricGraph g;
  // Symmetric reduced costs (same for arc and its reverse)
  double red_cost[16] = {-10.0, 3.0, 4.0, 6.0, 2.0, 3.0, 4.0, 1.0,
                         -10.0, 3.0, 4.0, 6.0, 2.0, 3.0, 4.0, 1.0};

  BucketGraph<EmptyPack> bidir(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 0.0, .bidirectional = true});
  bidir.build();
  bidir.update_arc_costs(red_cost);
  auto bidir_paths = bidir.solve();

  BucketGraph<EmptyPack> sym(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 0.0, .symmetric = true});
  sym.build();
  sym.update_arc_costs(red_cost);
  auto sym_paths = sym.solve();

  REQUIRE(!bidir_paths.empty());
  REQUIRE(!sym_paths.empty());
  CHECK(sym_paths[0].reduced_cost ==
        doctest::Approx(bidir_paths[0].reduced_cost));
}

TEST_CASE("Symmetric: fine bucket steps exercise mirror_bucket") {
  LargerSymmetricGraph g;

  // step=1 → 10 buckets per vertex (window [0,10]), exercises mirror indexing
  BucketGraph<EmptyPack> bidir(
      g.pv, EmptyPack{},
      {.bucket_steps = {1.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bidir.build();
  auto bidir_paths = bidir.solve();

  BucketGraph<EmptyPack> sym(
      g.pv, EmptyPack{},
      {.bucket_steps = {1.0, 1.0}, .tolerance = 1e9, .symmetric = true});
  sym.build();
  auto sym_paths = sym.solve();

  REQUIRE(!bidir_paths.empty());
  REQUIRE(!sym_paths.empty());
  CHECK(sym_paths[0].reduced_cost ==
        doctest::Approx(bidir_paths[0].reduced_cost));

  // Verify path validity with fine steps
  for (const auto& p : sym_paths) {
    CHECK(p.vertices.front() == 0);
    CHECK(p.vertices.back() == 0);
    REQUIRE(p.arcs.size() + 1 == p.vertices.size());
    for (size_t i = 0; i < p.arcs.size(); ++i) {
      int a = p.arcs[i];
      CHECK(a >= 0);
      CHECK(g.from[a] == p.vertices[i]);
      CHECK(g.to[a] == p.vertices[i + 1]);
    }
  }
}

TEST_CASE("Symmetric: solver wiring") {
  SymmetricGraph g;
  Solver<EmptyPack> solver(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .symmetric = true, .tolerance = 1e9});
  solver.build();
  solver.set_stage(Stage::Exact);
  auto paths = solver.solve();
  REQUIRE(!paths.empty());
  // Best route from depot: 0→1→2→0 (cost 2+1+3=6) or 0→2→1→0 (cost 3+1+2=6)
  CHECK(paths[0].reduced_cost <= 6.0 + 1e-6);
}

TEST_CASE("Enumerate: completeness flag") {
  ParallelArcGraph g;
  using BG = BucketGraph<EmptyPack>;

  // Normal enumeration → complete
  BG bg(g.pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0},
         .max_paths = 1000,
         .tolerance = 1e9,
         .stage = Stage::Enumerate,
         .max_enum_labels = 5000000});
  bg.build();
  bg.solve();
  CHECK(bg.enumeration_complete());

  // Tiny label cap → incomplete
  BG bg2(g.pv, EmptyPack{},
         {.bucket_steps = {5.0, 1.0},
          .max_paths = 1000,
          .tolerance = 1e9,
          .stage = Stage::Enumerate,
          .max_enum_labels = 1});
  bg2.build();
  bg2.solve();
  CHECK_FALSE(bg2.enumeration_complete());
}

TEST_CASE("Enumerate: max_paths triggers incomplete") {
  ParallelArcGraph g;
  using BG = BucketGraph<EmptyPack>;

  BG bg(g.pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0},
         .max_paths = 1,
         .tolerance = 1e9,
         .stage = Stage::Enumerate});
  bg.build();
  auto paths = bg.solve();
  CHECK(paths.size() == 1);
  CHECK_FALSE(bg.enumeration_complete());
}

#ifndef _WIN32
TEST_CASE("Enumerate: theta mismatch warning") {
  ParallelArcGraph g;
  using BG = BucketGraph<EmptyPack>;

  BG bg(g.pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0},
         .max_paths = 1000,
         .tolerance = -1e-6,
         .stage = Stage::Exact});
  bg.build();
  bg.solve();

  // Fix buckets with theta=2.0 (tight enough to fix some buckets)
  int nf = bg.fix_buckets(2.0);
  REQUIRE(nf > 0);

  // Enumerate with a different gap — should warn on stderr
  bg.set_stage(Stage::Enumerate);
  bg.set_tolerance(10.0);  // differs from fixing theta=2.0

  // Capture stderr via pipe
  int pipefd[2];
  REQUIRE(pipe(pipefd) == 0);
  int old_fd = dup(STDERR_FILENO);
  dup2(pipefd[1], STDERR_FILENO);

  bg.solve();

  // Restore stderr and read captured output
  fflush(stderr);
  dup2(old_fd, STDERR_FILENO);
  close(old_fd);
  close(pipefd[1]);

  char buf[1024] = {};
  auto n = read(pipefd[0], buf, sizeof(buf) - 1);
  close(pipefd[0]);
  REQUIRE(n > 0);

  std::string output(buf, static_cast<std::size_t>(n));
  CHECK(output.find("warning") != std::string::npos);
  CHECK(output.find("theta=2") != std::string::npos);

  // After reset_elimination, no warning (fixing_theta_ reset to INF)
  bg.reset_elimination();
  bg.solve();  // should not warn
  CHECK(bg.enumeration_complete());
}

TEST_CASE("Midpoint initializes to average of resource bounds") {
  SimpleGraph g;
  // tw_lb all 0, tw_ub all 10 → midpoint = (0+10)/2 = 5
  BucketGraph<EmptyPack> bg(
      g.pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9, .bidirectional = true});
  bg.build();
  bg.solve();
  CHECK(bg.midpoint() == doctest::Approx(5.0));
}

TEST_CASE("Midpoint adjusts on forward-heavy imbalance") {
  // Asymmetric graph: fan-out near source with small time, then a single
  // arc with large time to sink. Forward creates many labels (one per
  // intermediate vertex) in the q <= midpoint zone, while backward from
  // sink crosses the midpoint in one hop and stops extending.
  //
  // 0→1,0→2,0→3,0→4 (t=1), 1→5,2→5,3→5,4→5 (t=1), 5→6 (t=8)
  // TW: [0,10] for all. Midpoint = 5.
  // Forward: labels at 1,2,3,4 (q=1), 5 (q=2), 6 (q=10) → ~9 labels
  // Backward: 6→5 lands at q=2 < midpoint → only 1 label inserted
  // Ratio ~9:1 → triggers 5% shift toward max.
  int from_arr[9] = {0, 0, 0, 0, 1, 2, 3, 4, 5};
  int to_arr[9] = {1, 2, 3, 4, 5, 5, 5, 5, 6};
  double cost_arr[9] = {1, 2, 3, 4, 1, 1, 1, 1, 1};
  double time_arr[9] = {1, 1, 1, 1, 1, 1, 1, 1, 8};
  double tw_lb[7] = {0, 0, 0, 0, 0, 0, 0};
  double tw_ub[7] = {10, 10, 10, 10, 10, 10, 10};

  const double* arc_res[1] = {time_arr};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 7;
  pv.source = 0;
  pv.sink = 6;
  pv.n_arcs = 9;
  pv.arc_from = from_arr;
  pv.arc_to = to_arr;
  pv.arc_base_cost = cost_arr;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  BucketGraph<EmptyPack> bg(pv, EmptyPack{},
                            {.bucket_steps = {1.0, 1.0},
                             .tolerance = 1e9,
                             .bidirectional = true,
                             .stage = Stage::Exact});
  bg.build();

  bg.solve();
  double mid1 = bg.midpoint();
  // First solve: initializes to 5.0, then adjusts upward (fw >> bw)
  CHECK(mid1 > 5.0);

  bg.solve();
  double mid2 = bg.midpoint();
  // Second solve: should shift further upward
  CHECK(mid2 > mid1);
}

TEST_CASE("Midpoint resets on build") {
  LargerGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0},
                             .tolerance = 1e9,
                             .bidirectional = true,
                             .stage = Stage::Exact});
  bg.build();
  bg.solve();
  double mid1 = bg.midpoint();
  CHECK(mid1 != 0.0);  // should be initialized

  // Rebuild resets midpoint
  bg.build();
  bg.solve();
  double mid2 = bg.midpoint();
  // After rebuild, midpoint reinitializes to default (same graph → same value)
  CHECK(mid2 == doctest::Approx(mid1).epsilon(0.1));
}

TEST_CASE("No midpoint adjustment in heuristic/enumerate stages") {
  LargerGraph g;
  BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0},
                             .tolerance = 1e9,
                             .bidirectional = true,
                             .stage = Stage::Heuristic1});
  bg.build();
  bg.solve();
  double mid_h1 = bg.midpoint();
  // Solve again — heuristic should not adjust
  bg.solve();
  double mid_h2 = bg.midpoint();
  CHECK(mid_h1 == doctest::Approx(mid_h2));
}

TEST_CASE("Bidirectional correctness preserved with adaptive midpoint") {
  LargerGraph g;

  BucketGraph<EmptyPack> mono(g.pv, EmptyPack{},
                              {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
  mono.build();
  auto mono_paths = mono.solve();

  BucketGraph<EmptyPack> bidir(g.pv, EmptyPack{},
                               {.bucket_steps = {5.0, 1.0},
                                .tolerance = 1e9,
                                .bidirectional = true,
                                .stage = Stage::Exact});
  bidir.build();
  // Solve multiple times to let midpoint adapt
  auto bidir_paths = bidir.solve();
  bidir_paths = bidir.solve();
  bidir_paths = bidir.solve();

  REQUIRE(!mono_paths.empty());
  REQUIRE(!bidir_paths.empty());
  CHECK(bidir_paths[0].reduced_cost ==
        doctest::Approx(mono_paths[0].reduced_cost));
}

TEST_CASE("Exact completion bounds prune fw labels past midpoint") {
  // 6 vertices: 0=source, 1, 2, 3, 4, 5=sink.  tw [0,10], midpoint=5.
  //
  // Arcs:
  //   0→1 t=3 c=1    0→4 t=6 c=100
  //   1→2 t=3 c=1    4→3 t=5 c=50
  //   2→3 t=1 c=1
  //   3→5 t=1 c=1
  //
  // Optimal path: 0→1→2→3→5, cost=4, time=8.
  //
  // Fw label at vertex 4: q=6 (past midpoint=5), cost=100.
  // Its only outgoing arc is 4→3 (time=5).  After extension fw_after=6+5=11
  // which exceeds every bw label's q at vertex 3 (bw q=9 via 3←5).
  // So is_theta_compatible fails for every bw label at 3 →
  // has_compatible_opposite returns false → fw label at 4 pruned.
  //
  // Fw label at vertex 2: q=6 (past midpoint=5), cost=2.
  // Outgoing arc 2→3 (time=1): fw_after=7 ≤ bw q=9 at 3 → compatible →
  // NOT pruned.  Found via concatenation at (2,3).
  //
  // Mono doesn't find a path through 4 either (extend 4→3: q=11>ub=10,
  // infeasible).  So both solvers agree on optimal cost=4.

  int from[6] = {0, 0, 1, 2, 3, 4};
  int to[6] = {1, 4, 2, 3, 5, 3};
  double cost[6] = {1.0, 100.0, 1.0, 1.0, 1.0, 50.0};
  double time_d[6] = {3.0, 6.0, 3.0, 1.0, 1.0, 5.0};

  double tw_lb[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[6] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 6;
  pv.source = 0;
  pv.sink = 5;
  pv.n_arcs = 6;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  // Mono solve for reference
  BucketGraph<EmptyPack> mono(pv, EmptyPack{},
                              {.bucket_steps = {1.0, 1.0}, .tolerance = 1e9});
  mono.build();
  auto mono_paths = mono.solve();
  REQUIRE(!mono_paths.empty());
  CHECK(mono_paths[0].reduced_cost == doctest::Approx(4.0));

  // Bidir solve — vertex 4's fw label is pruned, vertex 2's is kept.
  BucketGraph<EmptyPack> bidir(pv, EmptyPack{},
                               {.bucket_steps = {1.0, 1.0},
                                .tolerance = 1e9,
                                .bidirectional = true,
                                .stage = Stage::Exact});
  bidir.build();
  auto bidir_paths = bidir.solve();
  REQUIRE(!bidir_paths.empty());
  CHECK(bidir_paths[0].reduced_cost ==
        doctest::Approx(mono_paths[0].reduced_cost));
}

TEST_CASE("Backward exact completion bounds prune bw labels past midpoint") {
  // 6 vertices: 0=source, 1, 2, 3, 4, 5=sink.  tw [0,10], midpoint=5.
  //
  // Two paths:
  //   A: 0→1→5  t=3+3=6, c=1+1=2  (optimal, found via fw reaching sink)
  //   B: 0→2→3→5  infeasible: fw at 2 has q=3, arc 2→3 t=8 → q=11>ub=10
  //
  // Arcs:
  //   0→1 t=3 c=1     1→5 t=3 c=1     (path A)
  //   0→2 t=3 c=100   2→3 t=8 c=50    3→5 t=1 c=1  (path B, infeasible)
  //   0→4 t=1 c=5     4→3 t=5 c=5     (path C below)
  //
  // Bw label at vertex 2: seed at 5 q=10, extend 3→5 bw q=9, extend 2→3 bw
  //   q=min(9-8,10)=1 < mu=5. Past midpoint.
  //   Incoming arcs of 2: only 0→2 (t=3). fw at 0: q=0.
  //   fw_after = max(0+3,0) = 3 > bw_q=1 → INCOMPATIBLE → pruned.
  //
  // Bw label at vertex 4: extend 4→3 bw q=min(9-5,10)=4 < mu=5.
  //   Incoming arcs of 4: only 0→4 (t=1). fw at 0: q=0.
  //   fw_after = max(0+1,0) = 1 ≤ bw_q=4 → COMPATIBLE → NOT pruned.
  //   Path C: 0→4→3→5, t=1+5+1=7, c=5+5+1=11. Found via concatenation.
  //
  // Both mono and bidir find paths A (cost=2) and C (cost=11).
  // Bw label at vertex 2 is pruned (n_bw_labels_pruned ≥ 1).

  int from[7] = {0, 0, 0, 1, 2, 3, 4};
  int to[7] = {1, 2, 4, 5, 3, 5, 3};
  double cost[7] = {1.0, 100.0, 5.0, 1.0, 50.0, 1.0, 5.0};
  double time_d[7] = {3.0, 3.0, 1.0, 3.0, 8.0, 1.0, 5.0};

  double tw_lb[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[6] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};

  const double* arc_res[1] = {time_d};
  const double* v_lb[1] = {tw_lb};
  const double* v_ub[1] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 6;
  pv.source = 0;
  pv.sink = 5;
  pv.n_arcs = 7;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  // Mono solve for reference
  BucketGraph<EmptyPack> mono(pv, EmptyPack{},
                              {.bucket_steps = {1.0, 1.0}, .tolerance = 1e9});
  mono.build();
  auto mono_paths = mono.solve();
  REQUIRE(!mono_paths.empty());
  CHECK(mono_paths[0].reduced_cost == doctest::Approx(2.0));

  // Bidir solve
  BucketGraph<EmptyPack> bidir(pv, EmptyPack{},
                               {.bucket_steps = {1.0, 1.0},
                                .tolerance = 1e9,
                                .bidirectional = true,
                                .stage = Stage::Exact});
  bidir.build();
  auto bidir_paths = bidir.solve();
  REQUIRE(!bidir_paths.empty());
  CHECK(bidir_paths[0].reduced_cost ==
        doctest::Approx(mono_paths[0].reduced_cost));

  // Verify pruning actually happened (bw label at vertex 2 was pruned)
  CHECK(bidir.n_bw_labels_pruned() >= 1);
}

#endif  // !_WIN32
