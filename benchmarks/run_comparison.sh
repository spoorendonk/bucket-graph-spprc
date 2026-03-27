#!/usr/bin/env bash
# run_comparison.sh — Compare bgspprc runtimes against Spoorendonk et al. 2025 (arXiv:2511.01397) "Base" column.
#
# Runs bgspprc-solve on all 56 Solomon RCSPP instances × {ng8, ng16, ng24},
# compares wall-clock times against the paper's sequential scalar baseline,
# and produces a ratio table plus shifted geometric mean summaries.
#
# Usage:
#   ./benchmarks/run_comparison.sh [--ng 8|16|24] [--timeout S]
#
# Prerequisites:
#   cmake -B build -DCMAKE_CXX_COMPILER=g++-14 && cmake --build build
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
REPODIR="$(cd "$SCRIPTDIR/.." && pwd)"
SOLVE="${SOLVE:-$REPODIR/build/bgspprc-solve}"
PAPER_CSV="$SCRIPTDIR/pull_algo_runtimes.csv"
OUT_CSV="$SCRIPTDIR/comparison_rcspp.csv"
TIMEOUT=120
NG_GROUPS=(8 16 24)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ng)      NG_GROUPS=("$2"); shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,/^$/s/^# \?//p' "${BASH_SOURCE[0]}"
      exit 0 ;;
    *) echo "Unknown arg: $1" >&2; exit 1 ;;
  esac
done

if [[ ! -x "$SOLVE" ]]; then
  echo "Error: solver not found at $SOLVE" >&2
  echo "Build first: cmake -B build -DCMAKE_CXX_COMPILER=g++-14 && cmake --build build" >&2
  exit 1
fi

if [[ ! -f "$PAPER_CSV" ]]; then
  echo "Error: paper data not found at $PAPER_CSV" >&2
  exit 1
fi

# Load paper data into associative array: key="instance,ng" -> base_s
declare -A PAPER_BASE
while IFS=, read -r inst ng base_s _rest; do
  [[ "$inst" == "instance" ]] && continue
  PAPER_BASE["${inst},${ng}"]="$base_s"
done < "$PAPER_CSV"

# Write CSV header
echo "instance,ng,bgspprc_s,paper_base_s,ratio" > "$OUT_CSV"

# Print table header
printf "\n"
printf "%-10s  %3s  %10s  %10s  %8s  %s\n" "Instance" "ng" "bgspprc(s)" "paper(s)" "ratio" "status"
printf '%.0s─' {1..60}; echo

# Accumulators for shifted geometric mean per ng group
declare -A SUM_LOG  # sum of ln(t + shift)
declare -A COUNT    # count per ng group
SHIFT=10

for ng in "${NG_GROUPS[@]}"; do
  INSTDIR="$SCRIPTDIR/instances/rcspp/ng${ng}"
  if [[ ! -d "$INSTDIR" ]]; then
    echo "Warning: instance directory $INSTDIR not found, skipping ng$ng" >&2
    continue
  fi

  for file in "$INSTDIR"/*.graph; do
    stem="$(basename "$file" .graph)"
    paper_key="${stem},${ng}"
    paper_val="${PAPER_BASE[$paper_key]:-}"

    if [[ -z "$paper_val" ]]; then
      echo "Warning: no paper data for $paper_key, skipping" >&2
      continue
    fi

    # Run solver
    status="OK"
    time_s=""
    if output=$(timeout "${TIMEOUT}s" "$SOLVE" --ng "$ng" "$file" 2>&1); then
      # Parse time from "...X.Xms" on first line
      line="$(echo "$output" | head -1)"
      if [[ "$line" =~ ([0-9.]+)ms ]]; then
        time_ms="${BASH_REMATCH[1]}"
        time_s="$(awk "BEGIN{printf \"%.3f\", $time_ms/1000}")"
      else
        status="PARSE_ERR"
      fi
    else
      rc=$?
      if [[ $rc -eq 124 ]]; then
        status="TIMEOUT"
        time_s="TL"
      else
        status="ERROR($rc)"
      fi
    fi

    # Compute ratio (skip timeouts)
    ratio=""
    if [[ -n "$time_s" && "$time_s" != "TL" && -n "$paper_val" ]]; then
      ratio="$(awk "BEGIN{printf \"%.2f\", $time_s / $paper_val}")"
    fi

    # Print row
    printf "%-10s  %3d  %10s  %10s  %8s  %s\n" \
      "$stem" "$ng" "${time_s:--}" "$paper_val" "${ratio:--}" "$status"

    # Write CSV row
    echo "${stem},${ng},${time_s},${paper_val},${ratio}" >> "$OUT_CSV"

    # Accumulate for geometric mean (only if we have a valid time)
    if [[ -n "$time_s" && "$status" == "OK" ]]; then
      log_ours="$(awk "BEGIN{print log($time_s + $SHIFT)}")"
      log_paper="$(awk "BEGIN{print log($paper_val + $SHIFT)}")"
      SUM_LOG["ours_$ng"]="$(awk "BEGIN{print ${SUM_LOG[ours_$ng]:-0} + $log_ours}")"
      SUM_LOG["paper_$ng"]="$(awk "BEGIN{print ${SUM_LOG[paper_$ng]:-0} + $log_paper}")"
      COUNT[$ng]="$(( ${COUNT[$ng]:-0} + 1 ))"
    fi
  done
done

# Print geometric mean summary
printf "\n"
printf '%.0s─' {1..60}; echo
printf "Shifted geometric mean (shift=%ds):\n\n" "$SHIFT"
printf "%-10s  %10s  %10s  %8s  %5s\n" "Group" "bgspprc(s)" "paper(s)" "ratio" "n"
printf '%.0s─' {1..50}; echo

for ng in "${NG_GROUPS[@]}"; do
  n="${COUNT[$ng]:-0}"
  if [[ "$n" -eq 0 ]]; then
    printf "%-10s  %10s  %10s  %8s  %5d\n" "ng$ng" "-" "-" "-" 0
    continue
  fi
  geo_ours="$(awk "BEGIN{printf \"%.3f\", exp(${SUM_LOG[ours_$ng]} / $n) - $SHIFT}")"
  geo_paper="$(awk "BEGIN{printf \"%.3f\", exp(${SUM_LOG[paper_$ng]} / $n) - $SHIFT}")"
  geo_ratio="$(awk "BEGIN{printf \"%.2f\", ($geo_ours + $SHIFT) / ($geo_paper + $SHIFT)}")"
  printf "%-10s  %10s  %10s  %8s  %5d\n" "ng$ng" "$geo_ours" "$geo_paper" "$geo_ratio" "$n"
done

printf "\nResults written to %s\n" "$OUT_CSV"
