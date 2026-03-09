#!/usr/bin/env bash
# check_optimal.sh — Compare benchmark results against reference optimal values.
#
# Reads results from benchmarks/bgspprc.csv (produced by run_benchmarks.sh),
# looks up reference optima in benchmarks/instances/<set>/optimal.csv, and
# reports PASS/FAIL/SKIP per instance.
#
# Usage:
#   ./benchmarks/check_optimal.sh [--ng K] [--csv FILE] [PATH...]
#
# Arguments:
#   PATH           Instance file or directory — filter CSV to these instances.
#                  If omitted, all CSV rows are checked.
#   --ng K         Filter CSV to rows with this ng value.
#   --csv FILE     Results CSV (default: benchmarks/bgspprc.csv).
#
# Verdict:
#   PASS   cost ≤ optimal + 0.001
#   FAIL   cost > optimal + 0.001, or no result
#   SKIP   no reference optimal available
#
# Output:
#   Per-instance table and PASS/FAIL/SKIP summary counts.
set -euo pipefail

usage() {
  sed -n '2,/^$/s/^# \?//p' "${BASH_SOURCE[0]}"
  exit "${1:-0}"
}

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
CSV="${SCRIPTDIR}/bgspprc.csv"
NG_FILTER=""
PATHS=()
EPS=0.001

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ng)   NG_FILTER="$2"; shift 2 ;;
    --csv)  CSV="$2"; shift 2 ;;
    -h|--help) usage ;;
    *)      PATHS+=("$1"); shift ;;
  esac
done

if [[ ! -f "$CSV" ]]; then
  echo "CSV not found: $CSV" >&2
  exit 1
fi

# ── Collect instance stems to filter ──
FILTER_STEMS=()
if [[ ${#PATHS[@]} -gt 0 ]]; then
  for p in "${PATHS[@]}"; do
    if [[ -f "$p" ]]; then
      stem="$(basename "$p")"
      FILTER_STEMS+=("${stem%.*}")
    elif [[ -d "$p" ]]; then
      while IFS= read -r -d '' f; do
        stem="$(basename "$f")"
        FILTER_STEMS+=("${stem%.*}")
      done < <(find "$p" -maxdepth 2 -type f \( -name '*.sppcc' -o -name '*.vrp' -o -name '*.graph' \) -print0)
    fi
  done
fi

# ── Load optimal values ──
declare -A OPTIMAL
for optfile in "$SCRIPTDIR"/instances/*/optimal.csv; do
  [[ -f "$optfile" ]] || continue
  while IFS=, read -r inst opt; do
    [[ "$inst" == "instance" || "$inst" == \#* ]] && continue
    OPTIMAL["$inst"]="$opt"
  done < "$optfile"
done

# ── Check results ──
PASS=0 FAIL=0 SKIP=0 TOTAL=0

printf "%-30s  %10s  %10s  %s\n" "Instance" "Got" "Optimal" "Result"
printf '%.0s-' {1..65}; echo

# Read CSV, skip header
while IFS=, read -r inst set ng cost paths time_s ts; do
  # Apply ng filter
  if [[ -n "$NG_FILTER" && "$ng" != "$NG_FILTER" ]]; then
    continue
  fi

  # Apply instance filter
  if [[ ${#FILTER_STEMS[@]} -gt 0 ]]; then
    match=0
    for s in "${FILTER_STEMS[@]}"; do
      if [[ "$inst" == "$s" ]]; then match=1; break; fi
    done
    [[ $match -eq 0 ]] && continue
  fi

  TOTAL=$((TOTAL + 1))

  # Look up optimal
  opt="${OPTIMAL[$inst]:-}"

  if [[ -z "$opt" ]]; then
    printf "%-30s  %10s  %10s  SKIP\n" "$inst" "${cost:--}" "-"
    SKIP=$((SKIP + 1))
    continue
  fi

  if [[ -z "$cost" ]]; then
    printf "%-30s  %10s  %10s  FAIL (no result)\n" "$inst" "-" "$opt"
    FAIL=$((FAIL + 1))
    continue
  fi

  # Compare: cost <= optimal + eps
  result="$(awk -v c="$cost" -v o="$opt" -v e="$EPS" 'BEGIN{print (c <= o + e) ? "PASS" : "FAIL"}')"
  printf "%-30s  %10s  %10s  %s\n" "$inst" "$cost" "$opt" "$result"
  if [[ "$result" == "PASS" ]]; then
    PASS=$((PASS + 1))
  else
    FAIL=$((FAIL + 1))
  fi
done < <(tail -n +2 "$CSV")

echo
echo "Summary: PASS=$PASS  FAIL=$FAIL  SKIP=$SKIP  TOTAL=$TOTAL"
