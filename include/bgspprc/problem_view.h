#pragma once

#include <cassert>
#include <cstdint>
#include <span>
#include <vector>

namespace bgspprc {

/// Non-owning view of the problem data. All arrays are caller-owned.
struct ProblemView {
  int n_vertices = 0;  // includes source(0) and sink(n_vertices-1)
  int source = 0;
  int sink = 0;

  int n_arcs = 0;

  // Arc topology: parallel arrays of size n_arcs
  const int* arc_from = nullptr;
  const int* arc_to = nullptr;
  const double* arc_base_cost = nullptr;

  // Per-arc resource consumptions: n_resources pointers, each to array of
  // n_arcs
  int n_resources = 0;
  const double* const* arc_resource = nullptr;  // [resource][arc]

  // Per-vertex resource bounds: n_resources pointers, each to array of
  // n_vertices
  const double* const* vertex_lb = nullptr;  // [resource][vertex]
  const double* const* vertex_ub = nullptr;  // [resource][vertex]

  int n_main_resources = 1;  // 1 or 2 (defines bucket grid dimensions)

  // Per-resource non-disposability: array of n_resources bools.
  // true = non-disposable (equality required in dominance).
  // nullptr means all resources are disposable (default).
  const bool* resource_nondisposable = nullptr;

  // Adjacency (caller-built or we build internally)
  // outgoing_arcs[v] = span of arc indices leaving v
  // incoming_arcs[v] = span of arc indices entering v
  // These are optional — bucket_graph builds its own if null.
};

/// Helper to build adjacency lists from arc arrays.
struct Adjacency {
  std::vector<std::vector<int>> outgoing;  // [vertex] -> arc indices
  std::vector<std::vector<int>> incoming;  // [vertex] -> arc indices

  void build(const ProblemView& pv) {
    outgoing.assign(pv.n_vertices, {});
    incoming.assign(pv.n_vertices, {});
    for (int a = 0; a < pv.n_arcs; ++a) {
      outgoing[pv.arc_from[a]].push_back(a);
      incoming[pv.arc_to[a]].push_back(a);
    }
  }
};

}  // namespace bgspprc
