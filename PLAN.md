# Implementation plan — gaps vs. papers

## Actionable (SPPRC-layer)

### 1. ~~Heuristic2: relax ng/R1C in dominance~~ ✓ DONE
**Paper**: VRPSolver 2020 §5.1.3
**Status**: Done. `dominates()` now skips ng/R1C for both Heuristic1 and
Heuristic2 stages. Test added.

### 2. Exact completion bounds during elimination
**Paper**: BucketGraph 2021 §5
**Status**: Not implemented. For labels extended past q*, exhaustively check if
a θ-compatible opposite label exists before keeping them. Tighter elimination,
beyond current bound-based + label-based.
**Effort**: Medium — requires per-label compatibility scan during elimination.

### 3. ~~Non-disposable resource dominance~~ ✓ DONE
**Paper**: VRPSolver 2020 §5.1.2
**Status**: Done. `ProblemView::resource_nondisposable` flag; main resource
dominance uses equality when set. Test added.

### 4. ~~`symmetric()` interface function~~ ✓ DONE
**Paper**: Meta-Solver 2026 §4.1
**Status**: Done. Per-resource `symmetric()` method added to Resource concept.
NgPathResource=true, StandardResource=false. BucketGraph asserts on mismatch.

## Revisit

### 5. R^cc (cumulative cost) and R^spd (pickup-delivery) resources
**Paper**: Meta-Solver 2026 §5–6
**Status**: Not implemented. Application-specific resources that users would
define via the ResourcePack interface. R^cc is novel (cumulative CVRP), R^spd
handles simultaneous pickup and delivery.
**Why revisit**: Validates the ResourcePack interface design — implementing these
as user-defined resources would confirm the interface is general enough (or
expose gaps).

### 6. `extendAlongArc` / `extendToVertex` split
**Paper**: Meta-Solver 2026 §4.1 (functions 3–4)
**Status**: Deliberately unified into single `extend()`, which is equivalent.
**Why revisit**: The split enables arc-ending labels which are the natural unit
for across-arc concatenation. Currently concatenation extends a forward label
through the across-arc and then checks compatibility — the split would let
forward labels stop at the arc midpoint naturally. May also matter for resources
where arc-ending vs vertex-ending state differs (e.g. R^spd).

## Out of scope (BCP-layer)

These are above the SPPRC solver layer:

- RCC separation (VRPSolver §3.6)
- Branching strategies (VRPSolver §4)
- Dual price smoothing / stabilization (VRPSolver §5.2)
- Safe dual bounds (VRPSolver §5.3)
- Primal heuristics — restricted master, diving with LDS (VRPSolver §5.5)
- Dynamic ng-set augmentation (VRPSolver §5.2)
- Rollback procedure — removing cuts when pricing is slow (VRPSolver §5.2)
