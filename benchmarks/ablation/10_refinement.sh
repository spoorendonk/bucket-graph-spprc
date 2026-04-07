#!/usr/bin/env bash
# 10_refinement.sh — A+ refinement predictability ablation.
#
# Measures how initial bucket step sizes affect convergence. Runs with
# different starting steps to observe where auto-refinement (xi-doubling)
# would fire. Compares coarse steps (where refinement is more likely
# needed) vs fine steps (less refinement needed).
#
# Usage:
#   ./benchmarks/ablation/10_refinement.sh [--timeout S] [PATH...]
#
# Output:
#   CSV to stdout: instance,set,initial_step,cost,paths,time_ms,status
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPTDIR/common.sh"

TIMEOUT="${DEFAULT_TIMEOUT}"
PATHS=()
# Coarse to fine: refinement is more likely with coarse steps
INITIAL_STEPS=(50 30 20 15 10 5)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout) TIMEOUT="$2"; shift 2 ;;
    -h|--help) sed -n '2,/^$/s/^# \?//p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)         PATHS+=("$1"); shift ;;
  esac
done

check_solver

if [[ ${#PATHS[@]} -eq 0 ]]; then
  while IFS= read -r p; do PATHS+=("$p"); done < <(default_instance_paths)
fi

FILES=()
while IFS= read -r f; do
  [[ -n "$f" ]] && FILES+=("$f")
done < <(collect_files "${PATHS[@]}")

if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "No instance files found." >&2
  exit 1
fi

echo "instance,set,initial_step,cost,paths,time_ms,status"

for file in "${FILES[@]}"; do
  stem="$(instance_stem "$file")"
  iset="$(instance_set "$file")"

  for step in "${INITIAL_STEPS[@]}"; do
    run_solver "$TIMEOUT" --steps "${step},${step}" --stage exact -- "$file"
    parse_cost_time "$OUT" "$STATUS"
    parse_paths "$OUT"
    echo "${stem},${iset},${step},${COST},${PATHS_COUNT},${TIME_MS},${STATUS}"
  done
done
