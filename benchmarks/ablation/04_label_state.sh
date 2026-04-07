#!/usr/bin/env bash
# 04_label_state.sh — Label state scaling ablation.
#
# Compares ng-path with different neighborhood sizes to measure the impact
# of compressed bitvector label state. Tests ng=0 (no ng-path), 4, 8, 12,
# 16, 24 to show how label state size affects performance.
#
# Usage:
#   ./benchmarks/ablation/04_label_state.sh [--timeout S] [PATH...]
#
# Output:
#   CSV to stdout: instance,set,ng_size,cost,paths,time_ms,status
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPTDIR/common.sh"

TIMEOUT="${DEFAULT_TIMEOUT}"
PATHS=()
NG_SIZES=(0 4 8 12 16 24)

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

echo "instance,set,ng_size,cost,paths,time_ms,status"

for file in "${FILES[@]}"; do
  stem="$(instance_stem "$file")"
  iset="$(instance_set "$file")"

  for ng in "${NG_SIZES[@]}"; do
    run_solver "$TIMEOUT" --ng "$ng" --stage exact -- "$file"
    parse_cost_time "$OUT" "$STATUS"
    parse_paths "$OUT"
    echo "${stem},${iset},${ng},${COST},${PATHS_COUNT},${TIME_MS},${STATUS}"
  done
done
