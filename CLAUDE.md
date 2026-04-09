# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

@.devkit/standards/cpp.md

## Project Overrides

The devkit `cpp.md` is a generic template. This project diverges in:
- **Test framework**: doctest (not GoogleTest). Fetched via FetchContent.
- **Test naming**: `test_<module>.cpp` (not `<module>_test.cpp`).
- **C++ standard**: C++20 (`cxx_std_20`). See [#84](https://github.com/spoorendonk/bucket-graph-spprc/issues/84) for C++23 bump tracking.

# Project: bucket-graph-spprc

Header-only C++20 template library implementing the bucket graph labeling algorithm for SPPRC (Shortest Path Problem with Resource Constraints) and vehicle routing variants. Based on Sadykov/Uchoa/Pessoa 2021, with extensions from Meta-Solver 2026.

## Build & Test

```bash
# Build
cmake -B build -DCMAKE_CXX_COMPILER=g++-14
cmake --build build

# Run all tests (~195 unit tests)
ctest --test-dir build

# Run a specific test by name (doctest)
./build/test_runner --test-case="Bucket construction"

# Run tests matching a pattern
./build/test_runner "*NgPath*"

# List all test cases
./build/test_runner --list-test-cases

# Fetch benchmark instances (once)
benchmarks/fetch_instances.sh

# Run benchmarks
benchmarks/run_benchmarks.sh
```

Options `BGSPPRC_BUILD_CLI` and `BGSPPRC_BUILD_TESTS` default ON at top-level, OFF as submodule.

## Architecture

### Core Types (include/bgspprc/)

- **`Solver<Pack>`** (`solver.h`) — Top-level entry point. Wraps `BucketGraph` with multi-stage control (Heuristic1 → Heuristic2 → Exact → Enumerate). Owns `ProblemView`, `Pack`, and options (bucket_steps, bidirectional, theta).
- **`BucketGraph<Pack>`** (`bucket_graph.h`) — Core labeling engine. Manages the bucket grid, label pools, SCC topology, c_best/completion bounds, bucket fixing, and arc elimination. Runs mono or bidirectional labeling.
- **`Label<Pack>`** (`label.h`) — Per-label state: vertex, bucket, cost, main resource values (q[0], q[1]), parent chain, and a compile-time tuple of all resource states. `BucketLabelPool` provides cache-line-aligned per-bucket arena allocation.
- **`Bucket`** (`bucket.h`) — Grid cell: vertex, SCC, resource interval [lb,ub), c_best, bucket arcs, and jump arcs.
- **`ProblemView`** (`problem_view.h`) — Non-owning view of problem data (vertices, arcs, costs, resource bounds). Caller retains ownership.

### Resource System

Resources are defined by the `Resource` concept (`resource.h`) with 7 functions: `symmetric`, `init_state`, `extend_along_arc`, `extend_to_vertex`, `domination_cost`, `concatenation_cost`, `min_domination_cost`. `ResourcePack<Rs...>` bundles resources at compile time and dispatches all operations via index_sequence.

Built-in resources:
- **`StandardResource`** (`resources/standard.h`) — Single double state (time windows, capacity).
- **`NgPathResource`** (`resources/ng_path.h`) — uint32_t state, local k+1 bit remapping with destination marking per Meta-Solver §4.2.2.
- **`R1CResource`** (`r1c.h`) — uint64_t state for up to 64 rank-1 cuts. Conforms to Resource concept.
- **`CumulativeCostResource`** (`resources/cumulative_cost.h`) — struct{S, T_or_W} for CCVRP.
- **`PickupDeliveryResource`** (`resources/pickup_delivery.h`) — struct{P, D} for PDVRP.

### Key Algorithms

- **Bucket arcs / Jump arcs**: Bucket arcs connect buckets across vertices; jump arcs connect same-vertex buckets with resource boost.
- **c_best**: Min forward label cost at bucket, used for dominance pruning. `min_domination_cost()` handles resources with negative domination costs (R1C).
- **fw_completion / bw_completion**: Cost-to-go DP lower bounds computed in reverse topo order on bucket arc graph. Used for arc elimination and bucket fixing.
- **Bucket fixing** (BG2021 §4.2): A bucket b is fixed when all paths through it are non-improving. Mono: `c_best(b) + fw_completion(b) > θ`. Bidir: additionally requires `bw_c_best(b) + bw_completion(b) > θ` AND `c_best(b) + bw_c_best(b) > θ` (all three must hold). θ = UB − LB + min{0, z} per eq (16).
- **2D bucketing**: `n_main_resources=2` activates 2D grid with `nb[0]*nb[1]` buckets per vertex.
- **A+ ξ-doubling**: Auto-triggers on CG convergence (avg ratio > 500 AND arcs/vertex < 10000) — halves bucket steps and rebuilds.
- **Concatenation**: Across-arc only (Meta-Solver style). For arc (i,j): extend fw@i through arc, check concatenation_cost at j with bw@j.

### Tests (tests/)

| File | Coverage |
|------|----------|
| `test_resource.cpp` | All resource types: extend, domination, concatenation |
| `test_bucket_graph.cpp` | Bucket construction, arcs, labeling basics |
| `test_small_instance.cpp` | End-to-end solver on 5-10 vertex instances |
| `test_stress.cpp` | Larger instances, 2D buckets, bidir, ng-paths, elimination, fixing, enumeration |
| `test_benchmarks.cpp` | Benchmark instance tests |
| `test_instance_io.cpp` | Instance I/O parsing |

## Reference Papers

See [References](README.md#references) for full citations. Key papers:
- BucketGraph 2021 (Sadykov/Uchoa/Pessoa) — core algorithm
- Meta-Solver 2026 (Sadykov et al.) — resource interface, ng-path destination marking, across-arc concatenation
- VRPSolver 2020 (Pessoa et al.) — broader context

## Known Pitfalls

- `extend_label` must compute actual bucket from label's q values, not use bucket arc's `to_bucket`.
- Bucket fixing must check `fixed_.test(actual_bi)` (label's actual landing bucket), not `fixed_.test(ba.to_bucket)`.
- Bidir fixing requires AND logic — fix only if ALL path types (fw, bw, concatenation) exceed theta.
- Dominance cost early-exit (`existing->cost > L->cost + EPS`) is unsafe when resource domination_cost can be negative. Use `c_best + min_dom_cost_ > L->cost + EPS`.
- Solver starts at `Stage::Heuristic1` by default. Benchmarks should call `set_stage(Stage::Exact)` for deterministic results.
- F-class Roberti instances with large capacity windows are inherently slow with step=10 bidir — filter them from benchmarks.
