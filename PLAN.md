# Implementation plan — gaps vs. papers

## Actionable (SPPRC-layer)

### 1. ~~Heuristic2: relax ng/R1C in dominance~~ ✓ DONE
**Paper**: VRPSolver 2020 §5.1.3
**Status**: Done. `dominates()` now skips ng/R1C for both Heuristic1 and
Heuristic2 stages. Test added.

### 2. ~~Exact completion bounds during elimination~~ ✓ DONE
**Paper**: BucketGraph 2021 §5
**Status**: Done. `has_compatible_opposite` prunes fw labels past q* at creation
if no θ-compatible bw label exists. Bw runs first so bw labels are available.
Disabled during enumeration.

### 3. ~~Non-disposable resource dominance~~ ✓ DONE
**Paper**: VRPSolver 2020 §5.1.2
**Status**: Done. `ProblemView::resource_nondisposable` flag; main resource
dominance uses equality when set. Test added.

### 4. ~~`symmetric()` interface function~~ ✓ DONE
**Paper**: Meta-Solver 2026 §4.1
**Status**: Done. Per-resource `symmetric()` method added to Resource concept.
NgPathResource=true, StandardResource=false. BucketGraph asserts on mismatch.

## Revisit

### 5. ~~R^cc (cumulative cost) and R^spd (pickup-delivery) resources~~ ✓ DONE
**Paper**: Meta-Solver 2026 §5–6
**Status**: Done. `CumulativeCostResource` (R^cc) and `PickupDeliveryResource`
(R^spd) implemented as ResourcePack-compatible resources. No interface gaps
found — the existing concept handles struct states, non-zero cost deltas,
vertex-only feasibility, non-trivial domination/concatenation costs. Tests added.

### 6. ~~`extendAlongArc` / `extendToVertex` split~~ ✓ DONE
**Paper**: Meta-Solver 2026 §4.1 (functions 3–4)
**Status**: Done. Resource concept has separate `extend_along_arc` and
`extend_to_vertex` methods. BucketGraph calls both in sequence. NgPathResource
uses the split for destination marking; R^spd uses it for vertex-only changes.

## Out of scope (BCP-layer)

These are above the SPPRC solver layer:

- RCC separation (VRPSolver §3.6)
- Branching strategies (VRPSolver §4)
- Dual price smoothing / stabilization (VRPSolver §5.2)
- Safe dual bounds (VRPSolver §5.3)
- Primal heuristics — restricted master, diving with LDS (VRPSolver §5.5)
- Dynamic ng-set augmentation (VRPSolver §5.2)
- Rollback procedure — removing cuts when pricing is slow (VRPSolver §5.2)
