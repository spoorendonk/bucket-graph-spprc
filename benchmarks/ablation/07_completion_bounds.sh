#!/usr/bin/env bash
# 07_completion_bounds.sh — Completion bound effectiveness ablation.
#
# Measures the impact of completion bounds by comparing solver performance
# with different theta (pricing threshold) values. Tighter theta values
# allow more aggressive arc elimination via completion bounds.
#
# Usage:
#   ./benchmarks/ablation/07_completion_bounds.sh [--timeout S] [PATH...]
#
# Output:
#   CSV to stdout: instance,set,theta,cost,paths,time_ms,status
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPTDIR/common.sh"

TIMEOUT="${DEFAULT_TIMEOUT}"
PATHS=()
# Theta values from loose to tight
THETAS=("0" "-0.001" "-0.01" "-0.1" "-1" "-10")

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

echo "instance,set,theta,cost,paths,time_ms,status"

for file in "${FILES[@]}"; do
  stem="$(instance_stem "$file")"
  iset="$(instance_set "$file")"

  for theta in "${THETAS[@]}"; do
    run_solver "$TIMEOUT" --theta "$theta" --stage exact -- "$file"
    parse_cost_time "$OUT" "$STATUS"
    parse_paths "$OUT"
    echo "${stem},${iset},${theta},${COST},${PATHS_COUNT},${TIME_MS},${STATUS}"
  done
done
