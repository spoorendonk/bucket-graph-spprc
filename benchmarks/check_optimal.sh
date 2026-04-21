#!/usr/bin/env bash
# check_optimal.sh — Compare benchmark results against reference optimal values.
#
# Reads results from benchmarks/bgspprc.csv (produced by run_benchmarks.sh),
# looks up reference optima, and reports PASS/FAIL/SKIP per instance.
#
# Optimal lookup (in benchmarks/instances/<set>/):
#   Each row's ng value determines the optimal file:
#     ng non-empty: prefer optimal-ng<ng>.csv, fall back to optimal.csv
#     ng empty: use optimal.csv
#   rcspp instances (set matches ng[0-9]+) always have optimal=0.
#
# Usage:
#   ./benchmarks/check_optimal.sh [--ng K] [--mode M] [--csv FILE] [PATH...]
#
# Arguments:
#   PATH           Instance file or directory — filter CSV to these instances.
#                  If omitted, all CSV rows are checked.
#   --ng K         Filter CSV to rows with this ng value.
#   --mode M       Filter CSV to rows with this solver mode
#                  (mono | bidir | para-bidir).
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
MODE_FILTER=""
PATHS=()
EPS=0.001

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ng)   NG_FILTER="$2"; shift 2 ;;
    --mode) MODE_FILTER="$2"; shift 2 ;;
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

# ── On-demand optimal lookup ──
# For each row, use the row's ng value to pick the right optimal file.
# rcspp instances (set matches ng[0-9]+) always have optimal=0.
# Cache loaded files to avoid re-parsing.
declare -A OPTIMAL   # keyed as "set:ng:instance"
declare -A LOADED    # keyed as "set:ng", value=1 when loaded

load_optimal() {
  local set="$1" ng="$2" key="${1}:${2}"
  [[ -n "${LOADED[$key]:-}" ]] && return
  LOADED["$key"]=1

  # rcspp ng* sets: optimal is always 0 — handled per-lookup, not here
  [[ "$set" =~ ^ng[0-9]+$ ]] && return

  local optfile=""
  local setdir="$SCRIPTDIR/instances/$set"
  if [[ -n "$ng" && -f "$setdir/optimal-ng${ng}.csv" ]]; then
    optfile="$setdir/optimal-ng${ng}.csv"
  elif [[ -f "$setdir/optimal.csv" ]]; then
    optfile="$setdir/optimal.csv"
  fi
  [[ -z "$optfile" ]] && return
  while IFS=, read -r _oinst _oopt; do
    [[ "$_oinst" == "instance" || "$_oinst" == \#* ]] && continue
    OPTIMAL["${set}:${ng}:${_oinst}"]="$_oopt"
  done < "$optfile"
}

lookup_optimal() {
  local _inst="$1" _set="$2" _ng="$3"
  # rcspp instances: optimal is always 0
  if [[ "$_set" =~ ^ng[0-9]+$ ]]; then
    _lookup_result=0; return
  fi
  load_optimal "$_set" "$_ng"
  _lookup_result="${OPTIMAL["${_set}:${_ng}:${_inst}"]:-}"
}

# ── Check results ──
PASS=0 FAIL=0 SKIP=0 TOTAL=0

printf "%-30s  %4s  %-10s  %10s  %10s  %s\n" "Instance" "ng" "Mode" "Got" "Optimal" "Result"
printf '%.0s-' {1..82}; echo

# Read CSV, skip header
while IFS=, read -r inst set ng mode cost paths time_s ts; do
  # Apply ng filter
  if [[ -n "$NG_FILTER" && "$ng" != "$NG_FILTER" ]]; then
    continue
  fi

  # Apply mode filter
  if [[ -n "$MODE_FILTER" && "$mode" != "$MODE_FILTER" ]]; then
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

  # Look up optimal using row's own ng value
  lookup_optimal "$inst" "$set" "$ng"
  opt="$_lookup_result"

  if [[ -z "$opt" ]]; then
    printf "%-30s  %4s  %-10s  %10s  %10s  SKIP\n" "$inst" "${ng:--}" "${mode:--}" "${cost:--}" "-"
    SKIP=$((SKIP + 1))
    continue
  fi

  if [[ -z "$cost" ]]; then
    printf "%-30s  %4s  %-10s  %10s  %10s  FAIL (no result)\n" "$inst" "${ng:--}" "${mode:--}" "-" "$opt"
    FAIL=$((FAIL + 1))
    continue
  fi

  # Compare: cost <= optimal + eps
  result="$(awk -v c="$cost" -v o="$opt" -v e="$EPS" 'BEGIN{print (c <= o + e) ? "PASS" : "FAIL"}')"
  printf "%-30s  %4s  %-10s  %10s  %10s  %s\n" "$inst" "${ng:--}" "${mode:--}" "$cost" "$opt" "$result"
  if [[ "$result" == "PASS" ]]; then
    PASS=$((PASS + 1))
  else
    FAIL=$((FAIL + 1))
  fi
done < <(tail -n +2 "$CSV")

echo
echo "Summary: PASS=$PASS  FAIL=$FAIL  SKIP=$SKIP  TOTAL=$TOTAL"
