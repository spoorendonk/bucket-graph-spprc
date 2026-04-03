// Basic SPPRC example: solve a 5-vertex shortest path problem with time windows.
//
// Graph: 5 vertices (0=source, 4=sink), 7 arcs
//
//   0 --a0(c=10,t=2)--> 1 --a2(c=5,t=3)--> 3 --a5(c=2,t=1)--> 4
//   0 --a1(c=3,t=4)---> 2 --a3(c=4,t=2)--> 3 --a5(c=2,t=1)--> 4
//                        1 --a4(c=8,t=5)--> 4
//                        2 --a6(c=7,t=3)--> 4
//
// All time windows [0, 20]. Optimal path: 0→2→3→4, cost=9.

#include <bgspprc/solver.h>

#include <cstdio>

int main() {
  using namespace bgspprc;

  // Arc topology
  int from[] = {0, 0, 1, 2, 1, 3, 2};
  int to[] = {1, 2, 3, 3, 4, 4, 4};
  double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};

  // Resource: time consumption per arc
  double time_consumption[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

  // Time windows per vertex: [lb, ub]
  double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

  // ProblemView uses arrays of pointers (one per resource dimension)
  const double* arc_res[] = {time_consumption};
  const double* v_lb[] = {tw_lb};
  const double* v_ub[] = {tw_ub};

  // Build the problem view (non-owning — caller owns all arrays)
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

  // Create solver with no extra resources (EmptyPack).
  // theta = 1e9 returns all feasible paths (not just negative reduced cost).
  Solver<EmptyPack> solver(pv, EmptyPack{},
                           {.bucket_steps = {5.0, 1.0}, .bidirectional = false, .theta = 1e9});
  solver.set_stage(Stage::Exact);
  solver.build();

  auto paths = solver.solve();

  std::printf("Found %zu paths\n", paths.size());
  for (size_t i = 0; i < paths.size(); ++i) {
    std::printf("  Path %zu: cost=%.2f  vertices:", i, paths[i].reduced_cost);
    for (size_t j = 0; j < paths[i].vertices.size(); ++j) {
      std::printf("%s%d", j == 0 ? " " : " -> ", paths[i].vertices[j]);
    }
    std::printf("\n");
  }

  return paths.empty() ? 1 : 0;
}
