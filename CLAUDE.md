<!-- wcp: CLAUDE.md base=e6cb8efde227 type=cpp -->
<!-- wcp-declined: 996da8f2cf7d Communication Style -->
<!-- wcp-declined: 0c56c7b46579 Code Navigation -->
<!-- wcp-declined: aa64ca55c171 Development Workflow -->
<!-- wcp-declined: b95d9e68df99 Git Workflow -->
<!-- wcp-declined: 058a3f5e0313 Issue Tracking -->
<!-- wcp-declined: 2b4541d688e6 Commit Messages -->
<!-- wcp-declined: a6e5ec6d33d8 CLAUDE.md Discipline -->
<!-- wcp-declined: 6e3ad83d1cb5 Complexity -->
<!-- wcp-declined: 1a2483f6e234 Plan Adherence -->
<!-- wcp-declined: e1a2b49b4c80 Reference Correctness -->
<!-- wcp-declined: 027569c8a6fc Common Mistakes -->
<!-- wcp-declined: e9feadcddcb5 Testing (GoogleTest) -->
<!-- wcp-declined: 913ba5bd4d6f LSP -->
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
extracts and runs them verbatim, in a fresh worktree of the pushed commit.
Without a `build` fence the push runs **no local CI at all, silently**, so do
not rename or drop one. Keep them runnable.

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
- **`AdjacencyView`** / **`CsrAdjacency`** (`problem_view.h`) — Non-owning CSR topology and its owning builder, from the shared pricer contract (flowty/flowty#58). `Adjacency` (`vector<vector<int>>`) is bgspprc's own and stays as is.
- **Executors** — `executor.h` is the shared contract text (flowty/flowty#58), kept byte-identical to the reference modulo namespace: the `Executor` concept, `SequentialExecutor`, and `ThreadPoolExecutor`, an 8-byte handle over a caller-owned `ThreadPool`. Don't edit it alone; change the reference and every copy. `StdThreadExecutor` (`executor_thread.h`) uses a process-wide pool. `parallel_sort` lives in `parallel_sort.h`.

### Resource System

Resources are defined by the `Resource` concept (`resource.h`) with 7 functions: `symmetric`, `init_state`, `extend_along_arc`, `extend_to_vertex`, `domination_cost`, `concatenation_cost`, `min_domination_cost`. `ResourcePack<Rs...>` bundles resources at compile time and dispatches all operations via index_sequence.

Built-in resources:
- **`StandardResource`** (`resources/standard.h`) — Single double state (time windows, capacity).
- **`NgPathResource`** (`resources/ng_path.h`) — uint32_t state, local k+1 bit remapping with destination marking per Meta-Solver §4.2.2.
- **`R1CResource`** (`r1c.h`) — uint64_t state for up to 64 rank-1 cuts. Conforms to Resource concept.
- **Borrowing split**: `NgPathData`/`R1CData` own the derived tables; `NgPathView`/`R1CView` are 8-byte resources borrowing them, so a caller can build the data once and share it across solvers. `NgPathResource`/`R1CResource` derive from the data and forward to `view()`. BucketGraph finds ng by duck-typing `compress_state` and R1C by the `is_r1c_resource` tag, so both views keep those.
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

- Full type annotations on all functions; check with `mypy --strict`.
- Built-in generics (`list[int]`, `dict[str, Any]`) and `|` unions.
- `ruff` for format and lint; `test_<module>.py` for tests, run with pytest.
- No hook runs `ruff`, `mypy` or `pytest`; run them yourself on Python changes.

## Tooling and Gates

Git hooks live in `.githooks/` and are tracked; they came from
[wcp](https://github.com/spoorendonk/wcp) (`wcp init`), see the sections below.
`core.hooksPath` is per-checkout and cannot be committed, so **a fresh clone has
no gates until it is set**. Here either `wcp init` or configuring the build
once sets it: CMake points it at `.githooks` whenever it is anything else
(`-DBGSPPRC_INSTALL_GIT_HOOKS=OFF` opts out). The wcp sections below name
only `wcp init`; the CMake route gates a clone just as well.

- **pre-commit** — `clang-format --dry-run -Werror` over every tracked C++ file. It checks, it does not reformat: run `clang-format -i` and commit again. No tests at commit.
- **commit-msg** — Conventional Commits, subject ≤72 chars. `fixup!`/`squash!`/`amend!` pass through so `git rebase --autosquash` works, and `type!:` marks a breaking change (both local edits to wcp's hook).
- **pre-push** — refuses merge commits on the trunk and a HEAD without a review receipt (`## Self-Review`), then runs the clean/build/test fences above in a fresh worktree of the pushed commit. Docs-only pushes skip the build.

The old hooks also auto-formatted and re-staged (with shfmt and ruff too),
applied safe clang-tidy fixes, ran the suite at commit, and ran clang-tidy,
shellcheck, ruff complexity and mypy as push warnings. wcp's pre-push lints
only through ctest tests named `clang_tidy:<path>`, which this repo does not
register yet, so **run clang-tidy yourself on C++ changes**. The tree is
clang-tidy clean; keep it that way.

On a shared machine, pin the local CI to the job budget: `git config
hooks.localCiCpus 0-3` makes the fences' `-j"$(nproc)"` see four CPUs.

Disabling a check is a decision, not a way out of a finding. `.clang-tidy`
records a reason per disabled check, and there are only two acceptable ones:
the check fights a convention this codebase holds on purpose, or it is a defect
in clang-tidy itself (two currently are — see the notes above `Checks:`).

Use clang-format and clang-tidy **18 or newer**; 18 and 21 were verified to
format this tree byte-identically. Nothing checks this: pre-commit runs
whatever clang-format is first on PATH (or in `.venv/bin`), so an older one
refuses commits of correctly formatted files.

There are deliberately **no Claude Code hooks**. The rules they used to enforce
are now conventions, stated here:

- **Branch from main, never from another feature branch.** `git checkout main && git pull` first.
- Prefer the built-in `Grep` tool over shelling out to `grep`/`rg`.

## Hooks Are Files In This Repository

The git hooks live in `.githooks/` as ordinary shell scripts, committed like any other file. `wcp init` copies them there and points `core.hooksPath` at them; wcp installs no Claude Code hooks at all. The gate is those files and nothing else, so a clone without wcp installed is still gated — including the review receipt, which is a line of plain git the Self-Review section documents rather than anything wcp runs.

- **They are yours to read, and yours to edit.** Each hook carries a `# wcp:` line recording the digest of the bytes wcp wrote. That is what lets `wcp doctor` tell an edit you made from a version wcp has moved past: an untouched hook is refreshed silently, an edited one is left alone, and one that moved on both sides is reported rather than overwritten. Edit them when the project needs something different — and say so upstream if the default was wrong for everyone.
- **A clone is not gated until someone runs `wcp init` in it.** git clones neither a hooks directory nor local config, so the files arriving with the clone is not the same as git running them. `wcp doctor` reports an unset `core.hooksPath` as an error for exactly this reason.
- **They call no wcp.** A hook that shelled out to a missing tool would exit 127, which git reads as a refusal — so a machine without wcp could not commit, and could not commit the fix either. What the hooks do call is the project's own toolchain, and there they fail open and say so on stderr.
- **Never use `git push --no-verify` or `git commit --no-verify`** unless explicitly asked. A failing hook is a signal — fix the root cause, or change the hook deliberately and commit that.

## This Document Carries A Provenance Line

The comment at the top of this file is wcp's record of the bytes it wrote, in the same scheme the hooks use. `wcp doctor` reads it to tell an edit you made from a version wcp has moved past, and it is where this project's type is recorded — there is no `.wcp/` directory and no config file beside it.

**The prose is yours.** wcp writes this document once and never rewrites it unasked; edit it freely. When wcp adds or changes a section you have not edited, `wcp doctor` names it and `wcp init --interactive` offers it, one section at a time; answering `n` declines that version for good. Leave the top line in place, and the tooling stays able to tell your changes from its own.

## Running wcp

wcp is a command-line tool installed once per machine, outside any repository. The hooks never call it, so a machine without it is still gated.

```sh
uv tool install git+https://github.com/spoorendonk/wcp   # once per machine; @vX.Y.Z for a release
uv tool upgrade wcp                                       # to update an install without a tag
uv tool install --force git+https://github.com/spoorendonk/wcp@vX.Y.Z   # to move a tagged one
wcp --help                                                # the commands; `wcp <command> --help` for each one
```

- **Run `wcp doctor`** after pulling, after updating wcp, and before editing the hooks or this file. It changes nothing, and every finding names the command that fixes it. It exits 1 only when the gate does not work, e.g. `core.hooksPath` is unset, which is how every fresh clone starts until someone runs `wcp init` in it.
- **Files wcp wrote carry a `wcp:` provenance line**: `# wcp:` in each hook, and the comment at the top of this file. Edit the files when the project needs to, but leave that line in place. It is how `wcp doctor` tells your edits apart from a newer wcp version.
- **If `wcp` is not on PATH**, say so rather than installing it unasked. Installing it is a machine-wide change.

## Working Through Issues

A session's goal is often a list of issues (`wcp goal`). Work through them in the order given, one at a time:

1. **Issue check.** Read the issue and what it links. If it is clear and implementable, say so in one line and go on; this should be fast. If not, write a plan as a comment on the issue (plans go on issues, never in git) and work from it. If a later issue's plan changes an earlier one, re-plan then.
2. **Implement.**
3. **Gate and commit.** Run the project's gate (`## Build & Test`) and commit: a review reads commits, never uncommitted changes (`## Self-Review`).
4. **Review by one cold reviewer**: a fresh teammate or subagent running `/code-review` on the commits not yet pushed, never several reviewers in parallel. On findings, fix, gate, commit, and review again with a fresh reviewer. **At most two rounds**; the fix after the second is confirmed, not reviewed again (`## Self-Review`). Anything real left after that, or found out of scope, becomes its own issue. The bar for filing is a concrete failure scenario or a missing acceptance criterion; nits are fixed within the round or dropped.
5. **Record, push, close.** Record the review on a clean tree, push it with the commit (`## Self-Review`), and close the issue.

Without agent teams, implement in the session itself and review with a fresh subagent; the caps and the bar are the same.

**A hygiene goal** (`/goal Hygiene round …`, set by `wcp hygiene`) is a task, not a list of issues: work it from its note. Its findings are edits to existing issues or new issues labelled `tech-debt`; nothing is fixed inside the round, one cold reviewer checks the findings (at most two rounds), and the task ends with the summary comment its note specifies, whose first line is the marker wcp looks for.

## Self-Review

**Code you produced gets reviewed before it lands on main.** Run `/code-review`
on your own changes and act on what it finds; the review is not a formality you
report having done. The pre-push hook is the backstop, not the review.

**Then record the review, and push it with the commit**, or `pre-push` will
refuse the push:

```sh
git update-ref "refs/reviews/$(git rev-parse HEAD)" HEAD
git push --atomic origin HEAD "refs/reviews/$(git rev-parse HEAD)"
```

The receipt is a ref named by the reviewed commit, `refs/reviews/<sha>`, and it
travels with the push, so a CI step can check every commit that reaches the
trunk — including one pushed with `--no-verify`, or from a clone whose hooks
never ran. wcp's README has the step to paste into the project's CI.
`--atomic` lands the branch and its receipt together or not at all.

Plain git, on purpose. No wcp, no harness, nothing installed — so a reviewer on
any machine can write it, and a clone that has never heard of wcp is still able
to record a review and push. Every worktree of a clone sees the same receipts,
and each names one commit, so a review counts for that commit wherever it is
checked out and for nothing else.

**What it means, and what it does not.** It records that a review happened at
this commit — not what was reviewed. It carries exactly the trust of whoever ran
it, and it is a line anybody can write by hand; what it buys is that nobody can
push work no one has looked at *without noticing they are doing it*.

A later commit has no receipt, and the next push is refused again. That is
deliberate: a new commit is work nobody reviewed, including the commit that
fixes what the review found.

**The fix after the last round.** Reviews stop after two rounds, so the commit
that fixes the second round's findings gets no third review. That round's
reviewer reads the fix commit alone (`git show <sha>`) against its own
findings, and the receipt is recorded for the fix once it confirms they are
addressed. It looks for nothing new. What it does not confirm is taken out of
the fix, which makes a new commit: the same reviewer confirms that one before
its receipt is recorded, and what was taken out becomes its own issue.

**Record it only when the tree is clean.** With nothing new committed, a review
falls back to reading uncommitted changes — so a record written then names a
commit that does not contain what was read, and the push it clears is of work
nobody reviewed. Commit first, then review, then record. Untracked scratch files
do not matter; staged and unstaged changes to tracked files do.

## Working Agreements

**Communication.** Be terse. No preamble, no filler.

**Code navigation.** Prefer narrow queries over full-file reads: LSP for symbol
questions (`goToDefinition`, `hover`, `documentSymbol`), then `Grep` with a small
`head_limit` to locate lines, then `Read` with `offset`/`limit` for a slice.
Full-file `Read` is fine for files under ~200 lines or when structure matters.
`.clangd` points at `build/compile_commands.json`, so LSP needs a configured build.

**Workflow.** plan (non-trivial) → implement → test → review → push to main.
Trunk-based, linear history: commit directly to main, record the review
(`## Self-Review`) and push when the gates pass. Feature branches are optional for larger changes; keep them short-lived
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
