# bucket-graph-spprc

[![CI](https://github.com/spoorendonk/bucket-graph-spprc/actions/workflows/ci.yml/badge.svg)](https://github.com/spoorendonk/bucket-graph-spprc/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)
[![arXiv](https://img.shields.io/badge/arXiv-2606.30847-b31b1b.svg)](https://arxiv.org/abs/2606.30847)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.20819208.svg)](https://doi.org/10.5281/zenodo.20819208)

Header-only C++23 template library implementing the bucket graph labeling algorithm for the Shortest Path Problem with Resource Constraints (SPPRC) and vehicle routing variants. Based on [Sadykov, Uchoa & Pessoa (2021)](https://doi.org/10.1287/trsc.2020.0985) with extensions from [Sadykov et al. (2026)](https://inria.hal.science/hal-05486295v2).

## Key Features

- **Generic Resource concept** — 7-function compile-time interface
- **5 built-in resources** — Standard (time/capacity), NgPath, R1C (rank-1 cuts), CumulativeCost, PickupDelivery
- **Mono and bidirectional labeling** with across-arc concatenation
- **Multi-stage heuristics** — Heuristic1 → Heuristic2 → Exact → Enumerate
- **Bucket fixing and arc elimination** with completion-bound pruning
- **SoA label storage** with SIMD-accelerated dominance checks
- **2D bucketing** for problems with two main resources

## Quick Start

Requires GCC 14+ and CMake 3.25+.

```bash
cmake -B build -DCMAKE_CXX_COMPILER=g++-14
cmake --build build

# One-time: fetch benchmark instances
benchmarks/fetch_instances.sh

# Solve an SPPRC instance
./build/bgspprc-solve --stats --timing benchmarks/instances/spprclib/A-n54-k7-149.sppcc
```

Sample output:

```
bgspprc 0.1.0 (v0.1.0-0-g2eb4a85)
bgspprc: executor=StdThread  bidir=parallel
A-n54-k7-149                    sppcc  n=55    arcs=2862    theta=-1e-06  cost=-56718.000  paths=1    26.5ms
  0 34 41 25 47 25 47 25 41 34 54
  n_buckets=550  n_labels_created=113210  n_dominance_checks=128977  n_non_dominated=12059
  n_dominance_checks_fw=53407  n_dominance_checks_bw=75570  n_non_dominated_fw=4854  n_non_dominated_bw=7205
  n_fixed_buckets=0  n_eliminated_arcs=0  label_state_bytes=1
  timing:  fw=5.570ms  bw=17.709ms  completion=1.009ms  concat=2.562ms  paths=1.167ms  sum=28.018ms
```

Supported instance formats: `.sppcc` (SPPCC), `.vrp` (Roberti VRPTW), `.graph` (Solomon RCSPP).
Full flag reference in [CLI Reference](#cli-reference) below.

## Benchmarks

```bash
# Fetch benchmark instances (once)
benchmarks/fetch_instances.sh

# Run benchmarks
benchmarks/run_benchmarks.sh
```

### Results at a glance

Headline numbers from the committed CSVs, 120 s timeout per instance. Full
tables, methodology, and reproducer one-liners in
[`benchmarks/README.md`](benchmarks/README.md#results).

`sgm (s)` = shifted geometric mean with **shift = 1 s**;
`mean (s)` = arithmetic mean; `solved` = #instances finished within the
120 s timeout (TL substitutes as 120 s in both).

**Pathwyse comparison (sppcc + vrp)** — bgspprc `para_bidir_vec` vs patched
Pathwyse, both in pure-ng mode.

| set      | ng | solver   | sgm (s) | mean (s) | solved |
|----------|---:|----------|--------:|---------:|-------:|
| spprclib |  8 | bgspprc  |   0.917 |    7.238 |  44/45 |
| spprclib |  8 | pathwyse |   1.522 |   12.133 |  42/45 |
| spprclib | 16 | bgspprc  |   2.048 |   15.801 |  40/45 |
| spprclib | 16 | pathwyse |   4.284 |   24.290 |  38/45 |
| spprclib | 24 | bgspprc  |   5.838 |   26.725 |  38/45 |
| spprclib | 24 | pathwyse |  10.590 |   42.512 |  31/45 |
| roberti  |  8 | bgspprc  |   0.551 |    0.911 |  31/31 |
| roberti  |  8 | pathwyse |   2.330 |    9.999 |  30/31 |
| roberti  | 16 | bgspprc  |   3.176 |   15.003 |  28/31 |
| roberti  | 16 | pathwyse |   8.796 |   28.152 |  28/31 |
| roberti  | 24 | bgspprc  |  14.813 |   44.346 |  23/31 |
| roberti  | 24 | pathwyse |  26.824 |   62.019 |  19/31 |

bgspprc is **1.3×–2.4× faster** than Pathwyse by sgm across the six
(set, ng) cells (ratio = `(pathwyse_sgm + 1) / (bgspprc_sgm + 1)`,
shift = 1 s, TL → 120 s on both sides). Both solvers reach the same
optimal reduced cost on `.sppcc`/`.vrp` modulo cost-scale rounding
(verified per-row via `compute_means.py pathwyse --rows`). Pathwyse needs
patches against upstream `d53c01b` — see
[`benchmarks/patches/`](benchmarks/patches/) (auto-applied by
`run_pathwyse.sh`).

**Paper comparison (rcspp)** — bgspprc `para_bidir_vec` vs Petersen & Spoorendonk
2025 (arXiv:2511.01397) `all_s` column.

| ng | solver   | sgm (s) | mean (s) | solved |
|---:|----------|--------:|---------:|-------:|
|  8 | bgspprc  |   1.884 |    5.640 |  56/56 |
|  8 | paper    |   0.203 |    0.406 |  56/56 |
| 16 | bgspprc  |   2.200 |    8.783 |  56/56 |
| 16 | paper    |   0.526 |    3.010 |  56/56 |
| 24 | bgspprc  |   2.588 |   15.433 |  52/56 |
| 24 | paper    |   0.873 |    9.123 |  53/56 |

bgspprc is **1.9×–2.4× slower** than the paper on rcspp at sgm — same
formula `(bg_sgm + 1) / (paper_sgm + 1)`, both sides share the 120 s
budget. Gap narrows with ng: 2.40× at ng=8 → 2.10× at ng=16 → 1.92× at
ng=24, suggesting bgspprc's overhead is fixed-per-instance rather than
proportional to search-tree size.

## Use as a Library

Add via CMake FetchContent:

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

// Create solver with no extra resources. Bucket steps default to the
// BG2021 §6.3 rule (horizon / ξ, ξ = 25 buckets per vertex, doubled when
// dominance gets expensive); pass .bucket_steps = {10.0, 1.0} to fix them.
Solver<EmptyPack> solver(pv, EmptyPack{});
solver.set_stage(Stage::Exact);
solver.build();

auto paths = solver.solve();
for (auto& p : paths)
    printf("cost=%.2f  vertices=%zu\n", p.reduced_cost, p.vertices.size());
```

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

See [`examples/custom_resource.cpp`](examples/custom_resource.cpp) for a complete working example, [`include/bgspprc/resource.h`](include/bgspprc/resource.h) for the full concept definition, and [`include/bgspprc/resources/`](include/bgspprc/resources/) for built-in implementations.

## Examples

The [`examples/`](examples/) directory contains standalone programs, built by default at top level:

- **`basic_spprc.cpp`** — Solve a 5-vertex SPPRC with time windows
- **`custom_resource.cpp`** — Implement a custom capacity resource using the `Resource` concept

```bash
./build/examples/example_basic_spprc
./build/examples/example_custom_resource
```

## CLI Reference

```
Usage: bgspprc-solve [OPTIONS] <path>...

Arguments:
  <path>    Instance file or directory (recurse, detect type by extension)

Options:
  --version       Print version and build git hash, then exit
  --mono          Use mono solver (default: bidir)
  --stage STAGE   heuristic1|heuristic2|exact (default: exact)
  --ng K          ng-neighborhood size (default: 0/off for sppcc/vrp;
                  from file or 8 for graph; 0 disables)
  --steps S1,S2   Bucket step sizes (default: per-type)
  --xi X          Derive steps from ξ = X buckets per vertex (BG2021 §6.3)
  --max-paths N   Number of paths to return (0=all, 1=best; default: 1)
  --theta T       Pricing threshold θ (default: -1e-6 for CG)
  --auto-steps    Use per-vertex auto-computed steps
  --stats         Print solve statistics after each instance
  --csv           Machine-readable CSV output
  --timing        Print phase timing breakdown
  --no-parallel   Use sequential executor (default: parallel)
  --no-parallel-bidir  Sequential fw/bw labeling
```

## Running Tests

```bash
# Run all tests (~195 unit tests)
ctest --test-dir build

# Run a specific test by name
./build/test_runner --test-case="Bucket construction"

# Run tests matching a pattern
./build/test_runner "*NgPath*"
```

## Citing

Cite the paper for the result and the archived release for the code.
[`CITATION.cff`](CITATION.cff) carries both in machine-readable form — GitHub's
"Cite this repository" button reads it.

The paper is **arXiv:2606.30847**, <https://arxiv.org/abs/2606.30847>.

```bibtex
@misc{spoorendonk2026bgspprc,
  title        = {bucket-graph-spprc: an extensible C++ library for the shortest
                  path problem with resource constraints},
  author       = {Spoorendonk, Simon},
  year         = {2026},
  eprint       = {2606.30847},
  archivePrefix= {arXiv},
  primaryClass = {math.OC},
  url          = {https://arxiv.org/abs/2606.30847}
}
```

The archived code is on Zenodo. **v0.1.0 is [10.5281/zenodo.20819209](https://doi.org/10.5281/zenodo.20819209)**; the
concept DOI [10.5281/zenodo.20819208](https://doi.org/10.5281/zenodo.20819208) always resolves to the latest version.
Cite the version DOI when reproducibility matters — it pins one tree and one set
of benchmark results:

```bibtex
@software{spoorendonk2026bgspprcsoftware,
  title     = {bucket-graph-spprc: a header-only C++23 bucket graph labeling
               library for the SPPRC},
  author    = {Spoorendonk, Simon},
  year      = {2026},
  version   = {0.1.0},
  doi       = {10.5281/zenodo.20819209},
  url       = {https://doi.org/10.5281/zenodo.20819209}
}
```

Cite a tag or a DOI, never `main`: the benchmark tables move with the code.

## References

1. **Sadykov, Uchoa, Pessoa (2021)** — *A bucket graph-based labeling algorithm with application to vehicle routing*. Transportation Science, 55(1):4-28. DOI: [10.1287/trsc.2020.0985](https://doi.org/10.1287/trsc.2020.0985)

2. **Sadykov, Froger, Uchoa, Pessoa, Bulhoes, de Araujo (2026)** — *Bucket graph meta-solver for the resource constrained shortest path problem* (Meta-Solver). HAL: hal-05486295. <https://inria.hal.science/hal-05486295v2>

3. **Petersen, Spoorendonk (2025)** — *A parallel pull labelling algorithm for the resource constrained shortest path problem*. arXiv:2511.01397. <https://arxiv.org/abs/2511.01397>

4. **Pessoa, Sadykov, Uchoa, Vanderbeck (2020)** — *A generic exact solver for vehicle routing and related problems* (VRPSolver). Mathematical Programming, 183:483-523. DOI: [10.1007/s10107-020-01523-z](https://doi.org/10.1007/s10107-020-01523-z)

5. **Salani, Basso, Giuffrida (2024)** — *PathWyse: a flexible, open-source library for the resource constrained shortest path problem*. Optimization Methods and Software, 1–23. DOI: [10.1080/10556788.2023.2296978](https://doi.org/10.1080/10556788.2023.2296978)

6. **Seman, Munari, Bulhões, Camponogara (2024)** — *BALDES: A Branch-Cut-and-Price Bucket Graph Labeling Algorithm for Vehicle Routing*. GitHub: <https://github.com/lseman/baldes>

7. **Spoorendonk (2026)** — *bucket-graph-spprc: an extensible C++ library for the shortest path problem with resource constraints*. arXiv:2606.30847. <https://arxiv.org/abs/2606.30847>

## Related Projects

- [**BALDES**](https://github.com/lseman/baldes) — Branch-Cut-and-Price bucket graph labeling for CVRP/VRPTW in C++ (Seman et al. 2024, ref [6]); BG2021 with R1C cuts and HGS-VRPTW heuristics
- [**PathWyse**](https://github.com/pathwyse/pathwyse) — Standard labeling + DSSR for RCSPP (Salani, Basso, Giuffrida 2024, ref [5])

## License

[MIT](LICENSE)
