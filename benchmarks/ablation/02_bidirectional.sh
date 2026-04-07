#!/usr/bin/env bash
# 02_bidirectional.sh — Bidirectional crossover point ablation.
#
# Compares mono vs bidir solver across all instances.
#
# Usage:
#   ./benchmarks/ablation/02_bidirectional.sh [--timeout S] [PATH...]
#
# Output:
#   CSV to stdout: instance,set,mode,cost,paths,time_ms,status
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

echo "instance,set,mode,cost,paths,time_ms,status"

for file in "${FILES[@]}"; do
  stem="$(instance_stem "$file")"
  iset="$(instance_set "$file")"

  # Mono
  run_solver "$TIMEOUT" --mono --stage exact -- "$file"
  parse_cost_time "$OUT" "$STATUS"
  parse_paths "$OUT"
  echo "${stem},${iset},mono,${COST},${PATHS_COUNT},${TIME_MS},${STATUS}"

  # Bidir
  run_solver "$TIMEOUT" --stage exact -- "$file"
  parse_cost_time "$OUT" "$STATUS"
  parse_paths "$OUT"
  echo "${stem},${iset},bidir,${COST},${PATHS_COUNT},${TIME_MS},${STATUS}"
done
