#!/usr/bin/env bash
# 08_instance_structure.sh — Instance structure vs configuration ablation.
#
# Compares performance across C-class (clustered) vs R-class (random)
# vs RC-class (mixed) Solomon instances with different configurations.
# Instance class is inferred from the filename prefix (C/R/RC).
#
# Usage:
#   ./benchmarks/ablation/08_instance_structure.sh [--timeout S] [PATH...]
#
# Output:
#   CSV to stdout: instance,set,class,step,mode,cost,paths,time_ms,fw_ms,bw_ms,concat_ms,status
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPTDIR/common.sh"

TIMEOUT="${DEFAULT_TIMEOUT}"
PATHS=()
STEPS=(10 20)

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

# Infer instance class from filename
infer_class() {
  local stem="$1"
  if [[ "$stem" =~ ^RC ]]; then echo "RC"
  elif [[ "$stem" =~ ^C ]]; then echo "C"
  elif [[ "$stem" =~ ^R ]]; then echo "R"
  else echo "other"
  fi
}

echo "instance,set,class,step,mode,cost,paths,time_ms,fw_ms,bw_ms,concat_ms,status"

for file in "${FILES[@]}"; do
  stem="$(instance_stem "$file")"
  iset="$(instance_set "$file")"
  iclass="$(infer_class "$stem")"

  for step in "${STEPS[@]}"; do
    for mode in mono bidir; do
      mode_flag=()
      [[ "$mode" == "mono" ]] && mode_flag=(--mono)

      run_solver "$TIMEOUT" "${mode_flag[@]}" --steps "${step},${step}" --stage exact -- "$file"
      parse_cost_time "$OUT" "$STATUS"
      parse_paths "$OUT"
      parse_timing "$OUT"
      echo "${stem},${iset},${iclass},${step},${mode},${COST},${PATHS_COUNT},${TIME_MS},${FW_MS},${BW_MS},${CONCAT_MS},${STATUS}"
    done
  done
done
