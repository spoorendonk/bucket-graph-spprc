#!/usr/bin/env bash
# run_ablation.sh — Master script for ablation studies.
#
# Runs all or selected ablation experiments and saves CSV results to
# benchmarks/ablation/results/.
#
# Usage:
#   ./benchmarks/run_ablation.sh [--timeout S] [--experiments N,N,...] [PATH...]
#
# Arguments:
#   --timeout S         Per-instance timeout in seconds (default: 120).
#   --experiments N,... Comma-separated list of experiment numbers to run
#                       (default: all, 1-10). Example: --experiments 1,3,5
#   PATH                Instance file or directory. Defaults to standard
#                       benchmark instances.
#
# Experiments:
#    1  Bucket step sensitivity (steps 5/10/15/20/30/50)
#    2  Bidirectional crossover point (mono vs bidir)
#    3  Jump arc effectiveness (with/without jump arcs)
#    4  Label state scaling (ng-path sizes 0/4/8/12/16/24)
#    5  Stage progression (heuristic1/heuristic2/exact)
#    6  Dominance tradeoffs (ng-path + mono/bidir cross)
#    7  Completion bound effectiveness (theta values)
#    8  Instance structure vs configuration (C/R/RC classes)
#    9  SIMD impact (with/without -march=native)
#   10  A+ refinement predictability (initial step sizes)
#
# Output:
#   benchmarks/ablation/results/NN_name.csv per experiment.
#
# Prerequisites:
#   cmake -B build -DCMAKE_CXX_COMPILER=g++-14 && cmake --build build
#   benchmarks/fetch_instances.sh  (for default instances)
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
ABLDIR="$SCRIPTDIR/ablation"
RESDIR="$ABLDIR/results"

TIMEOUT=120
EXPERIMENTS=""
PATHS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout)     TIMEOUT="$2"; shift 2 ;;
    --experiments) EXPERIMENTS="$2"; shift 2 ;;
    -h|--help)     sed -n '2,/^$/s/^# \?//p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)             PATHS+=("$1"); shift ;;
  esac
done

mkdir -p "$RESDIR"

# Build experiment list
ALL_EXPERIMENTS=(1 2 3 4 5 6 7 8 9 10)
if [[ -n "$EXPERIMENTS" ]]; then
  IFS=',' read -ra RUN_EXPERIMENTS <<< "$EXPERIMENTS"
else
  RUN_EXPERIMENTS=("${ALL_EXPERIMENTS[@]}")
fi

EXPERIMENT_SCRIPTS=(
  [1]="01_bucket_steps.sh"
  [2]="02_bidirectional.sh"
  [3]="03_jump_arcs.sh"
  [4]="04_label_state.sh"
  [5]="05_stage_progression.sh"
  [6]="06_dominance.sh"
  [7]="07_completion_bounds.sh"
  [8]="08_instance_structure.sh"
  [9]="09_simd.sh"
  [10]="10_refinement.sh"
)

EXPERIMENT_NAMES=(
  [1]="bucket_steps"
  [2]="bidirectional"
  [3]="jump_arcs"
  [4]="label_state"
  [5]="stage_progression"
  [6]="dominance"
  [7]="completion_bounds"
  [8]="instance_structure"
  [9]="simd"
  [10]="refinement"
)

TOTAL=${#RUN_EXPERIMENTS[@]}
IDX=0
PASSED=0
FAILED=0

for exp in "${RUN_EXPERIMENTS[@]}"; do
  IDX=$((IDX + 1))
  script="${EXPERIMENT_SCRIPTS[$exp]}"
  name="${EXPERIMENT_NAMES[$exp]}"
  outfile="$RESDIR/$(printf '%02d' "$exp")_${name}.csv"

  if [[ ! -f "$ABLDIR/$script" ]]; then
    echo "[$IDX/$TOTAL] Experiment $exp: script $script not found, skipping"
    FAILED=$((FAILED + 1))
    continue
  fi

  echo "[$IDX/$TOTAL] Running experiment $exp: $name"

  # Build extra args: pass --timeout and any instance paths
  extra_args=(--timeout "$TIMEOUT")
  extra_args+=("${PATHS[@]}")

  if bash "$ABLDIR/$script" "${extra_args[@]}" > "$outfile"; then
    rows=$(( $(wc -l < "$outfile") - 1 ))
    echo "  -> $outfile ($rows data rows)"
    PASSED=$((PASSED + 1))
  else
    echo "  -> FAILED (output in $outfile)"
    FAILED=$((FAILED + 1))
  fi
done

echo ""
echo "Ablation study complete: $PASSED passed, $FAILED failed out of $TOTAL experiments."
echo "Results in $RESDIR/"
