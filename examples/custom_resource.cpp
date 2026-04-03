// Custom resource example: add a vehicle capacity constraint to SPPRC.
//
// Demonstrates the Resource concept (7 functions) and ResourcePack composition.
//
// Same 5-vertex graph as basic_spprc.cpp, with arc demands and a capacity of 6.
// The cheapest path (0→2→3→4, cost=9) requires demand 5+3+1=9 > 6, so the
// capacity constraint forces the solver to find a different optimal path.

#include <bgspprc/resource.h>
#include <bgspprc/solver.h>

#include <cstdio>
#include <utility>

// A capacity resource: tracks accumulated demand consumed along the path.
// Forward: demand grows from 0. Infeasible if demand exceeds capacity.
// Backward: remaining capacity shrinks from Q. Infeasible if < 0.
struct CapacityResource {
  using State = double;

  const double* demand;  // per-arc demand, size = n_arcs
  double capacity;       // maximum total demand (Q)

  bool symmetric() const { return false; }

  State init_state(bgspprc::Direction dir) const {
    if (dir == bgspprc::Direction::Forward) return 0.0;  // no demand consumed
    return capacity;  // backward: full capacity remaining
  }

  std::pair<State, double> extend_along_arc(bgspprc::Direction dir, State s,
                                            int arc_id) const {
    if (dir == bgspprc::Direction::Forward)
      return {s + demand[arc_id], 0.0};
    else
      return {s - demand[arc_id], 0.0};
  }

  std::pair<State, double> extend_to_vertex(bgspprc::Direction dir, State s,
                                            int /*vertex*/) const {
    if (dir == bgspprc::Direction::Forward) {
      if (s > capacity + bgspprc::EPS) return {s, bgspprc::INF};
    } else {
      if (s < -bgspprc::EPS) return {s, bgspprc::INF};
    }
    return {s, 0.0};
  }

  // Forward: lower accumulated demand dominates (more room left).
  // Backward: higher remaining capacity dominates.
  double domination_cost(bgspprc::Direction dir, int /*vertex*/, State s1,
                         State s2) const {
    if (dir == bgspprc::Direction::Forward) {
      if (s1 > s2 + bgspprc::EPS) return bgspprc::INF;
    } else {
      if (s1 < s2 - bgspprc::EPS) return bgspprc::INF;
    }
    return 0.0;
  }

  double min_domination_cost() const { return 0.0; }

  // Forward consumed + backward consumed must not exceed capacity.
  double concatenation_cost(bgspprc::Symmetry /*sym*/, int /*vertex*/,
                            State s_fw, State s_bw) const {
    double bw_consumed = capacity - s_bw;
    if (s_fw + bw_consumed > capacity + bgspprc::EPS) return bgspprc::INF;
    return 0.0;
  }
};

// Verify at compile time that CapacityResource satisfies the Resource concept.
static_assert(bgspprc::Resource<CapacityResource>);

int main() {
  using namespace bgspprc;

  // Arc topology (same graph as basic_spprc.cpp)
  int from[] = {0, 0, 1, 2, 1, 3, 2};
  int to[] = {1, 2, 3, 3, 4, 4, 4};
  double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
  double time_consumption[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

  double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

  const double* arc_res[] = {time_consumption};
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

  // --- Solve without capacity constraint ---
  {
    Solver<EmptyPack> solver(pv, EmptyPack{},
                             {.bucket_steps = {5.0, 1.0}, .bidirectional = false, .theta = 1e9});
    solver.set_stage(Stage::Exact);
    solver.build();
    auto paths = solver.solve();

    std::printf("Without capacity constraint:\n");
    std::printf("  Optimal: cost=%.2f  vertices:", paths[0].reduced_cost);
    for (size_t j = 0; j < paths[0].vertices.size(); ++j)
      std::printf("%s%d", j == 0 ? " " : " -> ", paths[0].vertices[j]);
    std::printf("\n\n");
  }

  // --- Solve with capacity constraint (Q=6) ---
  {
    double demand[] = {1.0, 5.0, 2.0, 3.0, 1.0, 1.0, 2.0};
    CapacityResource cap_res{.demand = demand, .capacity = 6.0};

    // Bundle the custom resource into a ResourcePack
    using Pack = ResourcePack<CapacityResource>;
    Solver<Pack> solver(pv, Pack{cap_res},
                        {.bucket_steps = {5.0, 1.0}, .bidirectional = false, .theta = 1e9});
    solver.set_stage(Stage::Exact);
    solver.build();
    auto paths = solver.solve();

    std::printf("With capacity constraint (Q=6):\n");
    std::printf("  Found %zu feasible paths\n", paths.size());
    for (size_t i = 0; i < paths.size(); ++i) {
      std::printf("  Path %zu: cost=%.2f  vertices:", i,
                  paths[i].reduced_cost);
      for (size_t j = 0; j < paths[i].vertices.size(); ++j)
        std::printf("%s%d", j == 0 ? " " : " -> ", paths[i].vertices[j]);
      std::printf("\n");
    }
  }

  return 0;
}
