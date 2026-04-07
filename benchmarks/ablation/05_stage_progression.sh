#!/usr/bin/env bash
# 05_stage_progression.sh — Stage progression ablation.
#
# Compares Heuristic1 -> H2 -> Exact progression vs straight-to-Exact.
# Uses --stage to control the starting stage.
#
# Usage:
#   ./benchmarks/ablation/05_stage_progression.sh [--timeout S] [PATH...]
#
# Output:
#   CSV to stdout: instance,set,stage,cost,paths,time_ms,fw_ms,bw_ms,concat_ms,n_labels_created,n_non_dominated,status
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPTDIR/common.sh"

TIMEOUT="${DEFAULT_TIMEOUT}"
PATHS=()
STAGES=(heuristic1 heuristic2 exact)

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

echo "instance,set,stage,cost,paths,time_ms,fw_ms,bw_ms,concat_ms,n_labels_created,n_non_dominated,status"

for file in "${FILES[@]}"; do
  stem="$(instance_stem "$file")"
  iset="$(instance_set "$file")"

  for stage in "${STAGES[@]}"; do
    run_solver "$TIMEOUT" --stage "$stage" -- "$file"
    parse_cost_time "$OUT" "$STATUS"
    parse_paths "$OUT"
    parse_timing "$OUT"
    parse_stats "$OUT"
    echo "${stem},${iset},${stage},${COST},${PATHS_COUNT},${TIME_MS},${FW_MS},${BW_MS},${CONCAT_MS},${N_LABELS_CREATED},${N_NON_DOMINATED},${STATUS}"
  done
done
