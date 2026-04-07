#!/usr/bin/env bash
# 06_dominance.sh — Dominance tradeoffs ablation.
#
# Compares dominance behavior by running with different ng-path sizes.
# Larger ng neighborhoods increase dominance checking cost but produce
# better (more constrained) paths. ng=0 uses cost-only dominance.
#
# Usage:
#   ./benchmarks/ablation/06_dominance.sh [--timeout S] [PATH...]
#
# Output:
#   CSV to stdout: instance,set,ng_size,mode,cost,paths,time_ms,fw_ms,bw_ms,concat_ms,n_dominance_checks,n_non_dominated,status
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPTDIR/common.sh"

TIMEOUT="${DEFAULT_TIMEOUT}"
PATHS=()
NG_SIZES=(0 4 8 16)

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

echo "instance,set,ng_size,mode,cost,paths,time_ms,fw_ms,bw_ms,concat_ms,n_dominance_checks,n_non_dominated,status"

for file in "${FILES[@]}"; do
  stem="$(instance_stem "$file")"
  iset="$(instance_set "$file")"

  for ng in "${NG_SIZES[@]}"; do
    # Mono — no backward dominance
    run_solver "$TIMEOUT" --ng "$ng" --mono --stage exact -- "$file"
    parse_cost_time "$OUT" "$STATUS"
    parse_paths "$OUT"
    parse_timing "$OUT"
    parse_stats "$OUT"
    echo "${stem},${iset},${ng},mono,${COST},${PATHS_COUNT},${TIME_MS},${FW_MS},${BW_MS},${CONCAT_MS},${N_DOMINANCE_CHECKS},${N_NON_DOMINATED},${STATUS}"

    # Bidir — bidirectional dominance
    run_solver "$TIMEOUT" --ng "$ng" --stage exact -- "$file"
    parse_cost_time "$OUT" "$STATUS"
    parse_paths "$OUT"
    parse_timing "$OUT"
    parse_stats "$OUT"
    echo "${stem},${iset},${ng},bidir,${COST},${PATHS_COUNT},${TIME_MS},${FW_MS},${BW_MS},${CONCAT_MS},${N_DOMINANCE_CHECKS},${N_NON_DOMINATED},${STATUS}"
  done
done
