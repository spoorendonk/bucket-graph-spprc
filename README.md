# bucket-graph-spprc

[![CI](https://github.com/spoorendonk/bucket-graph-spprc/actions/workflows/ci.yml/badge.svg)](https://github.com/spoorendonk/bucket-graph-spprc/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)

Header-only C++23 template library implementing the bucket graph labeling algorithm for the Shortest Path Problem with Resource Constraints (SPPRC) and vehicle routing variants. Based on [Sadykov, Uchoa & Pessoa (2021)](#references) with extensions from the [Meta-Solver (2026)](#references).

## Key Features

- **Generic Resource concept** — 6-function compile-time interface (Meta-Solver §4.1)
- **5 built-in resources** — Standard (time/capacity), NgPath, R1C (rank-1 cuts), CumulativeCost, PickupDelivery
- **Mono and bidirectional labeling** with across-arc concatenation
- **Multi-stage heuristics** — Heuristic1 → Heuristic2 → Exact → Enumerate
- **Bucket fixing and arc elimination** with completion-bound pruning
- **SoA label storage** with SIMD-accelerated dominance checks
- **2D bucketing** for problems with two main resources

## Quick Start

Add as a CMake dependency:

```cmake
include(FetchContent)
FetchContent_Declare(
    bgspprc
    GIT_REPOSITORY https://github.com/spoorendonk/bucket-graph-spprc.git
    GIT_TAG main)
FetchContent_MakeAvailable(bgspprc)

target_link_libraries(your_target PRIVATE bgspprc)
```

Minimal usage:

```cpp
#include <bgspprc/solver.h>

using namespace bgspprc;

// Set up problem data (caller owns all arrays)
ProblemView pv;
pv.n_vertices = N;
pv.source = 0;
pv.sink = N - 1;
pv.n_arcs = M;
pv.arc_from = from.data();
pv.arc_to = to.data();
pv.arc_base_cost = cost.data();
pv.n_resources = 1;
pv.arc_resource = &arc_resource_ptr;
pv.vertex_lb = &vertex_lb_ptr;
pv.vertex_ub = &vertex_ub_ptr;

// Create solver with no extra resources
Solver<EmptyPack> solver(pv, EmptyPack{},
    {.bucket_steps = {10.0, 1.0}, .bidirectional = true});
solver.set_stage(Stage::Exact);
solver.build();

auto paths = solver.solve();
for (auto& p : paths)
    printf("cost=%.2f  vertices=%zu\n", p.reduced_cost, p.vertices.size());
```

## Build from Source

Requires GCC 14+ (C++23) and CMake 3.14+.

```bash
cmake -B build -DCMAKE_CXX_COMPILER=g++-14
cmake --build build
```

## Running Tests

```bash
# Run all tests (~195 unit tests)
ctest --test-dir build

# Run a specific test by name
./build/test_runner --test-case="Bucket construction"

# Run tests matching a pattern
./build/test_runner "*NgPath*"

# List all test cases
./build/test_runner --list-test-cases
```

## CLI Usage

```
Usage: bgspprc-solve [OPTIONS] <path>...

Arguments:
  <path>    Instance file or directory (recurse, detect type by extension)

Options:
  --mono          Use mono solver (default: bidir)
  --stage STAGE   heuristic1|heuristic2|exact (default: exact)
  --ng K          ng-neighborhood size (default: 0/off for sppcc/vrp;
                  from file or 8 for graph; 0 disables)
  --steps S1,S2   Bucket step sizes (default: per-type)
  --max-paths N   Number of paths to return (0=all, 1=best; default: 1)
  --theta T       Pricing threshold θ (default: -1e-6 for CG)
  --auto-steps    Compute per-vertex auto-computed steps
```

Supported formats: `.sppcc` (SPPCC), `.vrp` (Roberti VRPTW), `.graph` (Solomon RCSPP).

## Benchmarks

```bash
# Fetch benchmark instances (once)
benchmarks/fetch_instances.sh

# Run benchmarks
benchmarks/run_benchmarks.sh
```

See [`benchmarks/README.md`](benchmarks/README.md) for details on instance sets and expected results.

## Custom Resources

Implement the `Resource` concept to define custom resource types:

```cpp
struct MyResource {
    using State = double;

    bool symmetric() const;
    State init_state(Direction dir) const;
    std::pair<State, double> extend_along_arc(Direction dir, State s, int arc) const;
    std::pair<State, double> extend_to_vertex(Direction dir, State s, int vertex) const;
    double domination_cost(Direction dir, int vertex, State s1, State s2) const;
    double concatenation_cost(Symmetry sym, int vertex, State s_fw, State s_bw) const;
    double min_domination_cost() const;
};

// Bundle into a resource pack
using MyPack = ResourcePack<StandardResource, MyResource>;
Solver<MyPack> solver(pv, MyPack{std_res, my_res}, opts);
```

See [`include/bgspprc/resource.h`](include/bgspprc/resource.h) for the full concept definition and [`include/bgspprc/resources/`](include/bgspprc/resources/) for built-in implementations.

## References

1. **Petersen, Spoorendonk (2025)** — *A parallel pull labelling algorithm for the resource constrained shortest path problem*. arXiv:2511.01397. <https://arxiv.org/abs/2511.01397>

2. **Sadykov, Uchoa, Pessoa (2021)** — *A bucket graph-based labeling algorithm with application to vehicle routing*. Transportation Science, 55(1):4-28. DOI: [10.1287/trsc.2020.0985](https://doi.org/10.1287/trsc.2020.0985)

3. **Sadykov, Froger, Uchoa, Pessoa, Bulhoes, de Araujo (2026)** — *Bucket graph meta-solver for the resource constrained shortest path problem* (Meta-Solver). HAL: hal-05486295. <https://inria.hal.science/hal-05486295v2>

4. **Pessoa, Sadykov, Uchoa, Vanderbeck (2020)** — *A generic exact solver for vehicle routing and related problems* (VRPSolver). Mathematical Programming, 183:483-523. DOI: [10.1007/s10107-020-01523-z](https://doi.org/10.1007/s10107-020-01523-z)

## Related Projects

- [**Baldes**](https://github.com/lseman/baldes) — Bucket graph labeling for CVRP (BG2021 implementation in C++)
- [**Pathwyse**](https://github.com/pathwyse/pathwyse) — Standard labeling + DSSR for RCSPP
- [**Flowty**](https://flowty.ai) — Commercial solver for vehicle routing and scheduling

## License

[MIT](LICENSE)
