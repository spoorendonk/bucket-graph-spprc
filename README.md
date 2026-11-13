# bucket-graph-spprc

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)
[![arXiv](https://img.shields.io/badge/arXiv-2606.30847-b31b1b.svg)](https://arxiv.org/abs/2606.30847)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.20819208.svg)](https://doi.org/10.5281/zenodo.20819208)

Header-only C++23 template library implementing the bucket graph labeling algorithm for the Shortest Path Problem with Resource Constraints (SPPRC) and vehicle routing variants. Based on [Sadykov, Uchoa & Pessoa (2021)](https://doi.org/10.1287/trsc.2020.0985) with extensions from [Sadykov et al. (2026)](https://inria.hal.science/hal-05486295v2).

## Key Features

- **Generic Resource concept** — 7-function compile-time interface
- **6 built-in resources** — Standard (time/capacity), Side (a view's non-main resources), NgPath, R1C (rank-1 cuts), CumulativeCost, PickupDelivery
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
bgspprc 0.2.0 (v0.2.0-0-g32a8c1c)
bgspprc: executor=StdThread  bidir=parallel
A-n54-k7-149                    sppcc  n=55    arcs=2862    theta=-1e-06  cost=-56718.000  paths=1    26.5ms
  0 34 41 25 47 25 47 25 41 34 54
  n_buckets=550  n_labels_created=113210  n_dominance_checks=128977  n_non_dominated=12059
  n_dominance_checks_fw=53407  n_dominance_checks_bw=75570  n_non_dominated_fw=4854  n_non_dominated_bw=7205
  n_fixed_buckets=0  n_eliminated_arcs=0  label_state_bytes=1
  timing:  fw=5.570ms  bw=17.709ms  completion=1.009ms  concat=2.562ms  paths=1.167ms  sum=28.018ms
```

Supported instance formats: `.sppcc` (SPPCC), `.vrp` (Roberti VRPTW), `.graph` (Solomon RCSPP),
`.sppg` (spp-graph's general format, shared by the flowty pricers). An `.sppg` file carries no
ng sets (`--ng K` builds them from the K cheapest outgoing arcs), its main windows must be finite,
and resources past the first two main ones are enforced as ordinary resources (up to 8).
`--write-sppg DIR` converts any supported file to `.sppg` instead of solving it.
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

The bucket grid takes its one or two main resources (`n_main_resources`, at
most 2) from the view and never reads the rest. Resources past them are the
pack's to enforce: `SideResources` (`resources/side.h`) enforces view resources
`[first, first + n)` as standard resources, e.g.
`SideResources(pv, n_grid, pv.n_resources - n_grid)` with
`n_grid = std::min(pv.n_main_resources, 2)`; it throws `std::invalid_argument`
for more than `kMaxSideResources` of them.
A view without a main resource makes the constructor throw
`std::invalid_argument`.

`solver.cost_of(arcs)` prices a given source-to-sink walk (arc ids in order)
as labelling would, under the current reduced costs and resource states: an
`optional` of its reduced and original cost, empty when the walk is
infeasible. Use it to reprice a column the solver did not generate.

### Executors

`Solver` and `BucketGraph` take an executor as their second template argument
(default `SequentialExecutor`). The two threaded ones differ on a nested
dispatch, which the parallel bidirectional solve makes: it runs
`parallel_invoke(bw, fw)`, and each side's label compaction calls
`parallel_for` on the same executor.

- `ThreadPoolExecutor` (`executor.h`, over a caller-owned `ThreadPool`) runs
  one dispatch at a time per pool; a nested one runs inline on its calling
  thread. So in a parallel bidir solve, compaction uses at most 2 threads
  however large the pool.
- `StdThreadExecutor` (`executor_thread.h`, a process-wide pool) lets a
  waiting thread run queued tasks, so a nested `parallel_for` fans out across
  the whole pool.

Results are identical; only speed differs, so `StdThreadExecutor` timings
overstate what a `ThreadPoolExecutor` of the same size gives a bidir solve.

### Sharing a pool, an adjacency and ng/R1C data

A column-generation core that prices several s-t pricing problems over the same
graph can build the expensive parts once and lend them to every solver (the
shared pricer contract, flowty/flowty#58). Each piece is a small non-owning
handle over something the caller owns, so **the caller keeps the owner alive,
and unchanged, for as long as any solver borrowing it lives**.

```cpp
#include <bgspprc/bgspprc.h>

using namespace bgspprc;

// Owned by the core, built once.
ThreadPool pool(8);                       // 8 threads, the calling one included
CsrAdjacency adj;
adj.build(pv);                            // CSR of pv's arcs
const NgPathData ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, ng_neighbors);
R1CData r1c;
r1c.set_cuts(cuts, pv.n_vertices, pv.n_arcs);  // again whenever cuts or duals change

// Borrowed by each solver: all three handles are 8 bytes or a few pointers.
using Pack = ResourcePack<NgPathView, R1CView>;
Solver<Pack, ThreadPoolExecutor> solver(
    pv, adj.view(), Pack(NgPathView(ng), R1CView(r1c)), {}, ThreadPoolExecutor(pool));
```

- **Pool.** `ThreadPoolExecutor` is a handle to a caller-owned `ThreadPool`
  (`executor.h`); copies of it share the pool. The pool runs one dispatch at a
  time, so solvers that solve concurrently on one pool do not oversubscribe it:
  a dispatch that finds the pool busy runs inline on its calling thread.
- **Adjacency.** Pass an `AdjacencyView` after the `ProblemView` and the solver
  builds no CSR of its own (`owns_adjacency()` is false). It must describe the
  view's arcs (a vertex or arc count that differs throws
  `std::invalid_argument`) and may list a vertex's arcs in any order. A
  `CsrAdjacency` builds one; `view()` on a temporary does not compile. Without
  the argument the solver builds and owns its own.
- **ng and R1C data.** `NgPathData` and `R1CData` own the derived tables;
  `NgPathView` and `R1CView` are the resources that borrow them, and neither
  binds to a temporary. `NgPathResource` and `R1CResource` are the owning
  forms, for a solver that needs no sharing. Update `R1CData` (`set_cuts`)
  between solves, never while a solver borrowing it is solving; a view with no
  cuts borrows an `R1CData` that has none.
- **Rank-1 cut denominators.** An `R1Cut`'s multipliers are k/d in (0, 1)
  over one denominator d ∈ [2, 8], so the subset-row family from SR12 to
  SR98 fits, not only the 3/2 cut. Each cut takes ⌈log2 d⌉ state bits (1 for
  d = 2, 2 for d ≤ 4, 3 above), so one word holds 64 3/2 cuts or 21 cuts with
  d ≥ 5. `BasicR1CData<Words>` and its view and resource hold `Words` words;
  `set_cuts` throws when the cuts need more.
- **Rank-1 cuts with vertex memory.** `set_cuts` also takes pull-rcspp's
  cut form, `R1VertexCut` (`(vertex, numerator)` coefficients over one
  denominator, a vertex memory and a dual), with the `ProblemView` whose arcs
  it reads: with M' the memory and the coefficient vertices, arc (i, j)
  keeps the running sum iff i and j are both in M'. The denominator is taken
  as given, not reduced. Both forms throw `R1CError` (a
  `std::invalid_argument`) with a code, `InvalidCut` or `TooManyCounters`,
  and the offending cut and vertex.
- **Per-vertex layout.** `set_cuts(cuts, n_vertices, n_arcs)` lays the cuts
  out globally: every cut has its bits at every vertex, so all of them share
  `64·Words` bits. The `ProblemView` overloads, for `R1Cut` and
  `R1VertexCut` alike, lay them out per vertex: a cut has its bits only at
  the vertices of M' (for an `R1Cut`, its base set and the ends of its memory
  arcs), each vertex packs its own cuts, and `64·Words` bounds the cuts
  sharing a vertex, not all of them. Local cuts, as separation produces
  them, then fit many times over. `TooManyCounters` names the vertex that
  overflowed; `field(cut, vertex)` gives a cut's place at a vertex.
- **Checking the room first.** `BasicR1CData<Words>::fit(cuts, view)`, for
  either cut form, tells a separator which cuts the per-vertex layout has
  room for without catching a throw. It packs as `set_cuts` does, greedily
  in cut order, and places a cut only if it fits at every vertex of its M'.
  `R1CFit::unplaced` lists the cuts that did not fit and `bit_fails[v]`
  counts those that found no room at vertex v (a vertex may refuse a wide
  field and still take a narrower one). `set_cuts` accepts the set iff
  `unplaced` is empty, and accepts the placed cuts alone in the same slots.

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
  <path>    Instance file or directory (recurse, detect type by extension:
            .sppcc, .vrp, .graph, .sppg)

Options:
  --version       Print version and build git hash, then exit
  --mono          Use mono solver (default: bidir)
  --stage STAGE   heuristic1|heuristic2|exact (default: exact)
  --ng K          ng-neighborhood size (default: 0/off for sppcc/vrp/sppg;
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
  --write-sppg DIR  Write each instance to DIR/<stem>.sppg instead of solving
```

## Running Tests

```bash
# Run all tests (~195 unit tests)
ctest --test-dir build

# Run a specific test by name
./build/test_runner --test-case="Bucket construction"

# Run tests matching a pattern
./build/test_runner -tc="*NgPath*"
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

The archived code is on Zenodo. The concept DOI
[10.5281/zenodo.20819208](https://doi.org/10.5281/zenodo.20819208) always resolves to the latest version;
v0.1.0 is [10.5281/zenodo.20819209](https://doi.org/10.5281/zenodo.20819209) and v0.2.0 is
[10.5281/zenodo.24137027](https://doi.org/10.5281/zenodo.24137027). Cite a version DOI when
reproducibility matters — it pins one tree and one set of benchmark results:

```bibtex
@software{spoorendonk2026bgspprcsoftware,
  title     = {bucket-graph-spprc: a header-only C++23 bucket graph labeling
               library for the SPPRC},
  author    = {Spoorendonk, Simon},
  year      = {2026},
  version   = {0.2.0},
  doi       = {10.5281/zenodo.24137027},
  url       = {https://doi.org/10.5281/zenodo.24137027}
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
