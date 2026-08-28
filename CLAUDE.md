# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# Project: bucket-graph-spprc

Header-only C++23 template library implementing the bucket graph labeling algorithm for SPPRC (Shortest Path Problem with Resource Constraints) and vehicle routing variants. Based on Sadykov/Uchoa/Pessoa 2021, with extensions from Meta-Solver 2026.

## Build & Test

Needs a C++23 compiler: GCC >= 14 or Clang >= 18. If the default `g++` is older
than that, CMake picks up an installed `g++-15`/`g++-14` automatically — stock
Ubuntu 24.04 defaults to GCC 13 but packages 14, and the commands below are run
verbatim by `pre-push`, so they have to work there unattended. Setting `CXX` or
passing `-DCMAKE_CXX_COMPILER=` always overrides. Anything still too old fails
with an explicit message rather than a wall of template errors.

```clean
rm -rf build
```

```build
cmake -B build && cmake --build build -j"$(nproc)"
```

```test
ctest --test-dir build --output-on-failure -j"$(nproc)"
```

The three fenced blocks above are not just documentation — `.githooks/pre-push`
extracts and runs them verbatim. Renaming a fence, or moving this section so it
is no longer followed by a `## ` heading, blocks the push with an explicit
error. Keep them runnable.

Other useful commands:

```bash
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

- **`Solver<Pack, Exec>`** (`solver.h`) — Top-level entry point. Wraps `BucketGraph` with multi-stage control (Heuristic1 → Heuristic2 → Exact → Enumerate). Owns `ProblemView`, `Pack`, and options (bucket_steps, bidirectional, theta). `Exec` defaults to `SequentialExecutor`.
- **`BucketGraph<Pack, Exec>`** (`bucket_graph.h`) — Core labeling engine. Manages the bucket grid, label pools, SCC topology, c_best/completion bounds, bucket fixing, and arc elimination. Runs mono or bidirectional labeling. `Exec` controls parallel dispatch (see `executor.h`).
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
| `test_benchmarks.cpp` | Solver vs benchmark instances, checks optimal values |
| `test_instance_io.cpp` | Parser tests for `.sppcc` file format |

## C++

- Target C++23. Use modern features (`std::expected`, concepts, ranges, `constexpr`).
- Style: Google-based, enforced by `.clang-format` and `.clang-tidy`.
- Use `#pragma once` for include guards.
- Minimize includes in headers. Forward-declare where possible.

Naming, as enforced by `.clang-tidy`:

- `lower_case` functions, `CamelCase` classes and structs, `lower_case` namespaces.
- `lower_case` variables, with a trailing `_` on private members.
- Short capitals are allowed where they mirror the reference papers — `L`/`L0` for labels, `P`/`D` for pickup and delivery loads, `N`/`W`/`A` for set sizes and SIMD widths. Prefer the paper's symbol over a paraphrase; the point is that the code can be checked against the paper.
- Constants are deliberately not case-checked. `types.h` uses `UPPER_CASE` for the numeric limits (`INF`, `EPS`, `NEG_INF`) and internal tuning knobs use k-prefixed `CamelCase`. Follow whichever the surrounding file already uses.

## CMake

- `set(CMAKE_EXPORT_COMPILE_COMMANDS ON)` for clang-tidy — top-level only, so a consumer embedding us doesn't get its compile db clobbered.
- Use FetchContent for dependencies (doctest is fetched this way).
- The root `CMakeLists.txt` covers the library, CLI and tests; `examples/` has its own. Sources under `cli/` and `tests/` are listed from the root file.

## Testing (doctest)

- Test files: `test_<module>.cpp` in `tests/`.
- Name test cases descriptively: `TEST_CASE("solver returns optimal for feasible input")`.
- Every test target is built twice — once normally and once with `BGSPPRC_DISABLE_SIMD` — so a SIMD path that diverges from its scalar fallback fails the suite. Keep both registered when adding a target.

## Python

The Python here is benchmark tooling under `benchmarks/`, not a packaged
project — there is no `pyproject.toml` and no venv by default.

- Full type annotations on all functions; the pre-push hook runs `mypy --strict`.
- Built-in generics (`list[int]`, `dict[str, Any]`) and `|` unions.
- `ruff` for format and lint; `test_<module>.py` for tests, run with pytest.
- `ruff`, `mypy` and `pytest` are optional here. When one is missing the hooks say so out loud and continue — they never skip in silence. Install them (or drop them in `.venv/bin/`, which the hooks prefer over `PATH`) to get the full gate.

## Tooling and Gates

Git hooks live in `.githooks/` and are tracked. CMake points `core.hooksPath`
at them at configure time, since that setting is per-checkout and cannot be
committed — so **a fresh clone has no gates until you configure the build
once**. `-DBGSPPRC_INSTALL_GIT_HOOKS=OFF` opts out.

- **pre-commit** — formats staged files (clang-format, shfmt, ruff), applies the safe clang-tidy fixes, re-stages, then runs the suite. Blocks on test failure.
- **commit-msg** — Conventional Commits, subject ≤72 chars. `fixup!`/`squash!` pass through so `git rebase --autosquash` works.
- **pre-push** — clean build + full suite from the fences above, then shellcheck, ruff complexity and mypy as warnings. Deletion-only pushes and docs-only pushes skip the build.

The clang-tidy warning step in `pre-push` is **temporarily disabled**
(`CLANG_TIDY_WARNINGS_ENABLED=0`): it reports 130-170 findings per translation
unit on untouched code, so `All checks passed.` was unreachable on any C++
push. Tracked in [#95](https://github.com/spoorendonk/bucket-graph-spprc/issues/95).
The `clang-tidy --fix` subset in `pre-commit` is unaffected, and build+test
still blocks.

clang-format and clang-tidy must be **version 18 or newer**; the hooks check and
skip loudly below that, rather than reformatting the tree against everyone else.
18 and 21 were verified to format this tree byte-identically.

There are deliberately **no Claude Code hooks**. The rules they used to enforce
are now conventions, stated here:

- **Branch from main, never from another feature branch.** `git checkout main && git pull` first.
- Prefer the built-in `Grep` tool over shelling out to `grep`/`rg`.

Editing a hook to get around a failure is not a fix — the hooks are in-repo now,
so change one deliberately, in its own commit, or fix the root cause. Do not use
`--no-verify` unless explicitly asked.

## Working Agreements

**Communication.** Be terse. No preamble, no filler.

**Code navigation.** Prefer narrow queries over full-file reads: LSP for symbol
questions (`goToDefinition`, `hover`, `documentSymbol`), then `Grep` with a small
`head_limit` to locate lines, then `Read` with `offset`/`limit` for a slice.
Full-file `Read` is fine for files under ~200 lines or when structure matters.
`.clangd` points at `build/compile_commands.json`, so LSP needs a configured build.

**Workflow.** plan (non-trivial) → implement → test → review → push to main.
Trunk-based, linear history: commit directly to main and push when the gates
pass. Feature branches are optional for larger changes; keep them short-lived
and rebase or squash — no merge commits on main. After a push, close any gh
issue the work resolved (`gh issue close <num> -c "..."`) and delete the branch.

**Commit messages.** Conventional Commits, enforced by the hook. `type: description`
or `type(scope): description`; types `feat`, `fix`, `refactor`, `test`, `docs`,
`style`, `perf`, `chore`, `build`, `ci`. Focus on **why**, not what.

**Issue tracking.** GitHub Issues via the `gh` CLI; default to HTTPS remotes.
Read with `gh issue view <num> --json title,body,labels,state,comments`. Issues
get picked up cold in fresh sessions, so write them self-contained: problem,
motivation, acceptance criteria, repro. No local paths or machine-specific
locations, and prefer stable external links. Don't open an issue just to defer
something small — fix it inline or leave it.

**Plan adherence.** Follow the agreed plan. If you think it should change, stop
and discuss rather than silently diverging — same outside a written plan: if the
current approach isn't working, say so. Implement everything specified; no TODO
placeholders or stub implementations unless asked.

**Reference correctness.** When implementing from papers or reference code, match
the algorithm exactly. No early exits, iteration limits, size caps or
"optimization" shortcuts that change behavior, and no simplifying away edge
cases. Introduce heuristic approximations only when asked. When in doubt, be
faithful to the reference and let tests verify.

**Complexity.** When a complexity warning fires, don't extract methods
mechanically. Ask what the independent responsibilities are and split along those
boundaries. If the function is genuinely complex because the domain is, say so in
a comment.

**Performance matters.** Most of this code is solver hot paths — profile before
micro-optimizing, but don't trade real throughput for tidiness. Look hard at
allocations, copies, and asymptotics in labeling and dominance loops.

**Keep this file honest.** When Claude gets something wrong, fix CLAUDE.md in the
same commit. Don't invent APIs — verify a function or flag exists before using
it. Don't suppress a type error or a clang-tidy finding; fix the cause.

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
- Benchmark tables come solely from `benchmarks/compute_means.py`, which reads the source CSVs (`bgspprc.csv`, `pathwyse.csv`, `pull_algo_runtimes.csv`) and joins them in-memory — there are no committed `comparison_*.csv` files. It uses shift=1 s and TL→120 s for every table; `--rows` dumps the per-row joins (the former comparison CSVs) for spot-checking. Don't reintroduce a materialized join CSV: it goes stale against `bgspprc.csv` (the README numbers used to drift this way).
- `bgspprc.csv` encodes rcspp as `set ∈ {ng8, ng16, ng24}` (one per ng dir). Canonicalize to a single `rcspp` bucket before grouping by set (`compute_means.py` does this via `SET_DISPLAY`). The Pathwyse table excludes rcspp rows (`INCLUDED_SETS = {spprclib, roberti}`) because the ng metric differs between bgspprc and Pathwyse on `.graph` instances; rcspp is compared against the paper (`pull_algo_runtimes.csv`) instead.
