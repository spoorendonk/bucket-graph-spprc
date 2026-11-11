# Changelog

## 0.2.0 (2026-11-11)

bucket-graph-spprc 0.2.0 carries the shared pricer contract (flowty/flowty#58) and a round of correctness fixes since 0.1.0.

### Breaking changes

- **Default bucket steps come from ξ** (BG2021 §6.3, #106): a step left at 0 is range/ξ^(1/n_main), ξ = 25 by default. Explicit steps are kept.
  - The new `xi` field follows `bucket_steps` in `BucketGraph::Options` and `Solver::Options`, so every later member moves. Positional aggregate initialisation from 0.1.0 still compiles but fills the wrong fields: use designated initialisers.
  - `BucketGraph::halve_bucket_steps()` is gone. `double_xi()` replaces it, and it divides 2D steps by √2, not 2.
- **One s-t path, outside symmetric mode**: a path ends at its first arrival at the sink and leaves the source once (#124, #125). `source == sink` is the empty path there (#110). Symmetric mode needs `source == sink`, and the constructor throws otherwise (#126). Its depot tours are single (#123).
- **A view the grid cannot read is refused** (#119). `BucketGraph` throws `std::invalid_argument` in three cases:
  - the view has no main resource;
  - `n_resources` is smaller than the number of main resources;
  - `arc_resource`, `vertex_lb` or `vertex_ub` is null.

  A view's non-main resources are enforced by `SideResources`.
- **The rank-1 cut sign is corrected**: R1C domination, extension and concatenation now charge a non-negative penalty, and `precompute_betas` clamps β ≥ 0. Pricing with active rank-1 cuts gives different results from 0.1.0.
- **`Stage::Fixing` is removed** from the `Stage` enum. It was unused, but the stages after it are renumbered: `Exact` goes from 3 to 2 and `Enumerate` from 4 to 3. A stage stored or sent as an integer must be mapped again.
- **The `.graph` reader throws on malformed lines** where it used to skip them, and it keeps `:` in names (#127).
- **`parallel_sort` is no longer reachable from `executor.h` alone**: include `parallel_sort.h`. `ThreadPool` and `ThreadPoolExecutor` are new names in the `bgspprc` namespace.
- The CLI, tests and examples need `<print>` (libstdc++ ≥ 14, libc++ ≥ 17), and configure checks for it (#131). The header-only library does not.

### Pricer contract (flowty/flowty#58)

- `executor.h`: the `Executor` concept, `SequentialExecutor`, and `ThreadPoolExecutor` over a caller-owned `ThreadPool`.
- `AdjacencyView` / `CsrAdjacency`. `BucketGraph` and `Solver` can borrow a caller's CSR (#133).
- Borrowing split: `NgPathData`/`R1CData` own the tables, and the 8-byte `NgPathView`/`R1CView` borrow them.
- The README shows how to share one pool, one adjacency and one set of ng/R1C data across solvers (#138).
- A nested dispatch fans out on `StdThreadExecutor` but runs inline on `ThreadPoolExecutor`, so parallel bidir's compaction there uses at most 2 threads (#136). A `Solver` on a `ThreadPoolExecutor` is tested end to end (#135).
- The contract headers carry the shared reference comments, which read correctly in every copy (#134, #137).

### Features

- `cost_of(arcs)` reprices a walk by the same extension that labelling uses (#105).
- `label_cap_hit()` reports when an SCC label cap cut the solve (#96).
- CLI: reads and writes `.sppg` instance files (#107).

### Correctness

- Main resources are compared exactly in dominance (#114).
- Completion bounds stay lower bounds through jump arcs and negative cycles. A negative cycle is bounded by its laps, not by `NEG_INF` (#115, #121).
- Elimination and fixing stay safe at a gap above the solve's θ (#116). Truncated solves are reported and skipped (#115).
- Bidirectional fixes:
  - A path whose first arc crosses the midpoint is found (#108), and the mono-optimal path is found at the default θ (#112).
  - Each path is returned once (#109).
  - Enumerate buckets are sorted for the join (#117).
  - Elimination stays sound when the midpoint moves (#104), and label-based elimination decides only arcs that cross the midpoint (#111).
- Warm-started solves return paths that start at the source (#113).
- `StdThreadExecutor` no longer starts a chunk past the end of a short range, and a waiting thread runs queued tasks (#99).
- Fixing and elimination no longer read out of range after a solve with an infeasible source or sink (#118).

### Performance

- Jump arcs are built per arc in linear time (#128).
- Completion Bellman-Ford stops at a negative cycle (#129).
- `dominates()` inlines again (#130).

### Tooling

- Local CI is the gate: clang-format and clang-tidy 22.1.8 as ctest lint tests, with pre-push running the build and test fences. There are no GitHub Actions.
- ruff, mypy `--strict` and shellcheck run as lint tests too, at versions pinned in `requirements-dev.txt` (#140). Configure holds clang-format and clang-tidy to the full 22.1.8 pin, as the hooks do (#139).

Full list: https://github.com/spoorendonk/bucket-graph-spprc/compare/v0.1.0...v0.2.0

## 0.1.0 (2026-06-23)

First release, archived on Zenodo (10.5281/zenodo.20819209).
