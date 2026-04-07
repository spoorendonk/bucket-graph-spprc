#!/usr/bin/env bash
# 03_jump_arcs.sh — Jump arc effectiveness ablation.
#
# Compares solver performance with and without jump arcs.
#
# Usage:
#   ./benchmarks/ablation/03_jump_arcs.sh [--timeout S] [PATH...]
#
# Output:
#   CSV to stdout: instance,set,jump_arcs,cost,paths,time_ms,fw_ms,bw_ms,concat_ms,n_labels_created,n_dominance_checks,status
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPTDIR/common.sh"

TIMEOUT="${DEFAULT_TIMEOUT}"
PATHS=()

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

echo "instance,set,jump_arcs,cost,paths,time_ms,fw_ms,bw_ms,concat_ms,n_labels_created,n_dominance_checks,status"

for file in "${FILES[@]}"; do
  stem="$(instance_stem "$file")"
  iset="$(instance_set "$file")"

  # With jump arcs (default)
  run_solver "$TIMEOUT" --stage exact -- "$file"
  parse_cost_time "$OUT" "$STATUS"
  parse_paths "$OUT"
  parse_timing "$OUT"
  parse_stats "$OUT"
  echo "${stem},${iset},on,${COST},${PATHS_COUNT},${TIME_MS},${FW_MS},${BW_MS},${CONCAT_MS},${N_LABELS_CREATED},${N_DOMINANCE_CHECKS},${STATUS}"

  # Without jump arcs
  run_solver "$TIMEOUT" --no-jump-arcs --stage exact -- "$file"
  parse_cost_time "$OUT" "$STATUS"
  parse_paths "$OUT"
  parse_timing "$OUT"
  parse_stats "$OUT"
  echo "${stem},${iset},off,${COST},${PATHS_COUNT},${TIME_MS},${FW_MS},${BW_MS},${CONCAT_MS},${N_LABELS_CREATED},${N_DOMINANCE_CHECKS},${STATUS}"
done
