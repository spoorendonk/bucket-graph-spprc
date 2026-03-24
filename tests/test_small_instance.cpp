#include <bgspprc/resource.h>
#include <bgspprc/solver.h>

#include <doctest/doctest.h>

using namespace bgspprc;

// End-to-end test on a small known instance.
//
// Graph: 5 vertices (0=source, 4=sink)
//
//   0 --a0(c=10,t=2)--> 1 --a2(c=5,t=3)--> 3 --a5(c=2,t=1)--> 4
//   0 --a1(c=3,t=4)---> 2 --a3(c=4,t=2)--> 3 --a5(c=2,t=1)--> 4
//   1 --a4(c=8,t=5)---> 4
//   2 --a6(c=7,t=3)---> 4
//
// Paths:
//   0→1→3→4: cost = 10+5+2 = 17, time = 2+3+1 = 6
//   0→2→3→4: cost = 3+4+2 = 9,   time = 4+2+1 = 7
//   0→1→4:   cost = 10+8 = 18,    time = 2+5 = 7
//   0→2→4:   cost = 3+7 = 10,     time = 4+3 = 7
//
// Optimal (min cost): 0→2→3→4, cost=9

TEST_CASE("End-to-end: 5-vertex graph optimal path") {
  int from[] = {0, 0, 1, 2, 1, 3, 2};
  int to[] = {1, 2, 3, 3, 4, 4, 4};
  double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
  double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

  const double* arc_res[] = {time_d};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 5;
  pv.source = 0;
  pv.sink = 4;
  pv.n_arcs = 7;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  Solver<EmptyPack> solver(pv, EmptyPack{},
                           {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
  solver.build();

  auto paths = solver.solve();
  REQUIRE(!paths.empty());

  // Best path cost = 9 (0→2→3→4)
  CHECK(paths[0].reduced_cost == doctest::Approx(9.0));
  CHECK(paths[0].original_cost == doctest::Approx(9.0));

  CHECK(paths[0].vertices.size() == 4);
  CHECK(paths[0].vertices[0] == 0);
  CHECK(paths[0].vertices[1] == 2);
  CHECK(paths[0].vertices[2] == 3);
  CHECK(paths[0].vertices[3] == 4);
}

TEST_CASE("End-to-end: with reduced costs (pricing)") {
  int from[] = {0, 0, 1, 2, 1, 3, 2};
  int to[] = {1, 2, 3, 3, 4, 4, 4};
  double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
  double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

  const double* arc_res[] = {time_d};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 5;
  pv.source = 0;
  pv.sink = 4;
  pv.n_arcs = 7;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  Solver<EmptyPack> solver(pv, EmptyPack{},
                           {.bucket_steps = {5.0, 1.0}, .theta = 0.0});
  solver.build();

  // Reduced costs: make 0→1→3→4 have negative reduced cost
  double red[] = {-15.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
  solver.update_arc_costs(red);

  auto paths = solver.solve();
  REQUIRE(!paths.empty());

  // 0→1→3→4: -15+5+2 = -8 (negative reduced cost)
  CHECK(paths[0].reduced_cost == doctest::Approx(-8.0));
}

TEST_CASE("End-to-end: tight time windows") {
  int from[] = {0, 0, 1, 2, 1, 3, 2};
  int to[] = {1, 2, 3, 3, 4, 4, 4};
  double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
  double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 6.5};  // sink tw: [0, 6.5]

  const double* arc_res[] = {time_d};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 5;
  pv.source = 0;
  pv.sink = 4;
  pv.n_arcs = 7;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  Solver<EmptyPack> solver(pv, EmptyPack{},
                           {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
  solver.set_stage(Stage::Exact);
  solver.build();

  auto paths = solver.solve();
  REQUIRE(!paths.empty());

  // With sink time window [0, 6.5]:
  //   0→1→3→4: time=6 ≤ 6.5 → feasible, cost=17
  //   0→2→3→4: time=7 > 6.5 → INFEASIBLE
  //   0→1→4:   time=7 > 6.5 → INFEASIBLE
  //   0→2→4:   time=7 > 6.5 → INFEASIBLE
  // Only feasible path: 0→1→3→4, cost=17
  CHECK(paths[0].reduced_cost == doctest::Approx(17.0));
  CHECK(paths[0].vertices[1] == 1);
  CHECK(paths[0].vertices[2] == 3);
}

// ── Bi-directional tests ──

TEST_CASE("Bidirectional: same optimal as mono-directional") {
  int from[] = {0, 0, 1, 2, 1, 3, 2};
  int to[] = {1, 2, 3, 3, 4, 4, 4};
  double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
  double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

  const double* arc_res[] = {time_d};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 5;
  pv.source = 0;
  pv.sink = 4;
  pv.n_arcs = 7;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  // Mono-directional
  Solver<EmptyPack> mono(pv, EmptyPack{},
                         {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
  mono.build();
  auto mono_paths = mono.solve();

  // Bi-directional
  Solver<EmptyPack> bidir(
      pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .bidirectional = true, .theta = 1e9});
  bidir.build();
  auto bidir_paths = bidir.solve();

  REQUIRE(!mono_paths.empty());
  REQUIRE(!bidir_paths.empty());

  // Same best cost
  CHECK(bidir_paths[0].reduced_cost ==
        doctest::Approx(mono_paths[0].reduced_cost));

  // Best path should be 0→2→3→4, cost=9
  CHECK(bidir_paths[0].reduced_cost == doctest::Approx(9.0));
  CHECK(bidir_paths[0].vertices.front() == 0);
  CHECK(bidir_paths[0].vertices.back() == 4);
}

TEST_CASE("Bidirectional: with reduced costs") {
  int from[] = {0, 0, 1, 2, 1, 3, 2};
  int to[] = {1, 2, 3, 3, 4, 4, 4};
  double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
  double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

  const double* arc_res[] = {time_d};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 5;
  pv.source = 0;
  pv.sink = 4;
  pv.n_arcs = 7;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  Solver<EmptyPack> bidir(
      pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .bidirectional = true, .theta = 0.0});
  bidir.build();

  double red[] = {-15.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
  bidir.update_arc_costs(red);

  auto paths = bidir.solve();
  REQUIRE(!paths.empty());

  // 0→1→3→4: -15+5+2 = -8
  CHECK(paths[0].reduced_cost == doctest::Approx(-8.0));
  CHECK(paths[0].vertices.front() == 0);
  CHECK(paths[0].vertices.back() == 4);
}

TEST_CASE("Bidirectional: tight time windows") {
  int from[] = {0, 0, 1, 2, 1, 3, 2};
  int to[] = {1, 2, 3, 3, 4, 4, 4};
  double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
  double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 6.5};

  const double* arc_res[] = {time_d};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  ProblemView pv;
  pv.n_vertices = 5;
  pv.source = 0;
  pv.sink = 4;
  pv.n_arcs = 7;
  pv.arc_from = from;
  pv.arc_to = to;
  pv.arc_base_cost = cost;
  pv.n_resources = 1;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 1;

  // Mono
  Solver<EmptyPack> mono(pv, EmptyPack{},
                         {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
  mono.set_stage(Stage::Exact);
  mono.build();
  auto mono_paths = mono.solve();

  // Bidir
  Solver<EmptyPack> bidir(
      pv, EmptyPack{},
      {.bucket_steps = {5.0, 1.0}, .bidirectional = true, .theta = 1e9});
  bidir.set_stage(Stage::Exact);
  bidir.build();
  auto bidir_paths = bidir.solve();

  REQUIRE(!mono_paths.empty());
  REQUIRE(!bidir_paths.empty());

  // Same best: 0→1→3→4, cost=17 (only feasible path with sink tw [0, 6.5])
  CHECK(bidir_paths[0].reduced_cost ==
        doctest::Approx(mono_paths[0].reduced_cost));
  CHECK(bidir_paths[0].reduced_cost == doctest::Approx(17.0));
}
