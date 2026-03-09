/// Example: CVRP pricing subproblem using bucket-graph-spprc.
///
/// Demonstrates how a column generation framework would use this library
/// to solve the resource-constrained shortest path pricing problem.

#include <bgspprc/bgspprc.h>

#include <cstdio>
#include <vector>

using namespace bgspprc;

int main() {
  // Small CVRP instance: 4 customers + depot (= source and sink)
  // Vertices: 0=depot(source), 1-4=customers, 5=depot(sink)
  constexpr int N = 6;
  constexpr int source = 0;
  constexpr int sink = 5;

  // Build complete graph (source→customers, customers→customers,
  // customers→sink)
  std::vector<int> from, to;
  std::vector<double> base_cost;
  std::vector<double> time_consumption;
  std::vector<double> demand_consumption;

  // Distance matrix (symmetric)
  double dist[N][N] = {
      {0, 10, 15, 20, 25, 0}, {10, 0, 8, 12, 18, 10}, {15, 8, 0, 10, 14, 15},
      {20, 12, 10, 0, 9, 20}, {25, 18, 14, 9, 0, 25}, {0, 0, 0, 0, 0, 0},
  };

  // Customer demands
  double demands[] = {0, 3, 5, 4, 6, 0};

  // Service times
  double service[] = {0, 2, 3, 2, 4, 0};

  // Build arcs (no direct source→sink: must visit ≥1 customer)
  for (int i = 0; i < N - 1; ++i) {  // from: 0..4
    for (int j = 1; j < N; ++j) {    // to: 1..5
      if (i == j) continue;
      if (i == sink || j == source) continue;
      if (i == source && j == sink) continue;  // no empty route
      from.push_back(i);
      to.push_back(j);
      base_cost.push_back(dist[i][j]);
      time_consumption.push_back(dist[i][j] + service[j]);
      demand_consumption.push_back(demands[j]);
    }
  }

  int n_arcs = static_cast<int>(from.size());

  // Resource bounds
  double tw_lb[N] = {0, 0, 0, 0, 0, 0};
  double tw_ub[N] = {100, 100, 100, 100, 100, 100};
  double cap_lb[N] = {0, 0, 0, 0, 0, 0};
  double cap_ub[N] = {15, 15, 15, 15, 15, 15};  // vehicle capacity = 15

  const double* arc_res[] = {time_consumption.data(),
                             demand_consumption.data()};
  const double* v_lb[] = {tw_lb, cap_lb};
  const double* v_ub[] = {tw_ub, cap_ub};

  ProblemView pv;
  pv.n_vertices = N;
  pv.source = source;
  pv.sink = sink;
  pv.n_arcs = n_arcs;
  pv.arc_from = from.data();
  pv.arc_to = to.data();
  pv.arc_base_cost = base_cost.data();
  pv.n_resources = 2;
  pv.arc_resource = arc_res;
  pv.vertex_lb = v_lb;
  pv.vertex_ub = v_ub;
  pv.n_main_resources = 2;

  // Solve with empty resource pack (main resources handled by bucket graph)
  Solver<EmptyPack> solver(
      pv, EmptyPack{},
      {.bucket_steps = {20.0, 5.0}, .tolerance = 1e9});  // accept all paths
  solver.build();

  std::printf("Bucket graph built.\n");

  // First solve (no duals — find shortest paths by original cost)
  auto paths = solver.solve();
  std::printf("Found %zu paths.\n", paths.size());

  for (std::size_t i = 0; i < std::min(paths.size(), std::size_t{5}); ++i) {
    std::printf("  Path %zu: cost=%.1f, vertices=", i, paths[i].reduced_cost);
    for (int v : paths[i].vertices) std::printf("%d ", v);
    std::printf("\n");
  }

  // Simulate CG iteration: apply dual values
  // dual[v] reduces cost of arcs entering v
  double duals[] = {0, 8, 10, 7, 12, 0};

  std::vector<double> reduced(n_arcs);
  for (int a = 0; a < n_arcs; ++a) {
    reduced[a] = base_cost[a] - duals[to[a]];
  }

  solver.update_arc_costs(reduced);
  paths = solver.solve();

  std::printf("\nAfter applying duals:\n");
  std::printf("Found %zu paths with negative reduced cost.\n", paths.size());
  for (std::size_t i = 0; i < std::min(paths.size(), std::size_t{5}); ++i) {
    std::printf("  Path %zu: rc=%.1f, orig_cost=%.1f, vertices=", i,
                paths[i].reduced_cost, paths[i].original_cost);
    for (int v : paths[i].vertices) std::printf("%d ", v);
    std::printf("\n");
  }

  return 0;
}
