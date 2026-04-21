#!/usr/bin/env bash
# run_pathwyse.sh — Build Pathwyse, convert instances, run Pathwyse, append rows
# to benchmarks/pathwyse.csv. One row per (instance, ng) with cost + runtime.
#
# No comparison logic — use build_comparison_pathwyse.py to join pathwyse.csv
# against bgspprc.csv into benchmarks/comparison_pathwyse.csv.
#
# Usage:
#   ./benchmarks/run_pathwyse.sh [--ng K] [--timeout S] [--skip-build] [--append]
#                                [PATH...]
#
# Arguments:
#   PATH              Instance file or directory of instances.
#                     Default: benchmarks/instances/rcspp/ng8
#   --ng K            ng-neighborhood size (default: 8).
#   --timeout S       Per-instance Pathwyse timeout in seconds (default: 120).
#   --skip-build      Skip cloning/building Pathwyse (reuse existing build).
#   --append          Append rows to existing CSV (skip header write + overwrite).
#
# Environment:
#   PATHWYSE_DIR      Path to Pathwyse repo (default: ./build/pathwyse).
#
# Output:
#   benchmarks/pathwyse.csv — CSV with columns:
#     instance, set, ng, cost, time_s, cost_scale, timestamp
#   cost_scale is the per-instance int32 scaling used by convert_to_pathwyse.py;
#   downstream consumers use it to size float-comparison tolerances.
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
REPODIR="$(cd "$SCRIPTDIR/.." && pwd)"
PATHWYSE_DIR="${PATHWYSE_DIR:-$REPODIR/build/pathwyse}"
PATHWYSE_BIN="$PATHWYSE_DIR/bin/pathwyse"
OUT_CSV="$SCRIPTDIR/pathwyse.csv"
CONVERTED_DIR="$SCRIPTDIR/instances/pathwyse"
TIMEOUT=120
NG=8
SKIP_BUILD=0
APPEND=0
PATHS=()

# ── Usage ──
usage() {
  sed -n '2,/^$/s/^# \?//p' "${BASH_SOURCE[0]}"
  exit "${1:-0}"
}

# ── Args ──
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ng)         NG="$2"; shift 2 ;;
    --timeout)    TIMEOUT="$2"; shift 2 ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    --append)     APPEND=1; shift ;;
    -h|--help)    usage ;;
    *)            PATHS+=("$1"); shift ;;
  esac
done

# Default paths
if [[ ${#PATHS[@]} -eq 0 ]]; then
  PATHS=("$SCRIPTDIR/instances/rcspp/ng${NG}")
fi

# ── Build Pathwyse ──
build_pathwyse() {
  if [[ -x "$PATHWYSE_BIN" && $SKIP_BUILD -eq 1 ]]; then
    echo "Pathwyse: reusing existing build at $PATHWYSE_BIN"
    return
  fi

  echo "Pathwyse: cloning repository..."
  if [[ -d "$PATHWYSE_DIR" ]]; then
    rm -rf "$PATHWYSE_DIR"
  fi
  git clone --depth 1 https://github.com/pathwyse/pathwyse.git "$PATHWYSE_DIR"

  echo "Pathwyse: building..."
  mkdir -p "$PATHWYSE_DIR/build"
  cmake -B "$PATHWYSE_DIR/build" -S "$PATHWYSE_DIR" \
    -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
  cmake --build "$PATHWYSE_DIR/build" -j"$(nproc)" 2>&1 | tail -5

  if [[ ! -x "$PATHWYSE_BIN" ]]; then
    echo "Error: Pathwyse binary not found at $PATHWYSE_BIN after build" >&2
    echo "Checking build output location..." >&2
    find "$PATHWYSE_DIR" -name pathwyse -type f -executable 2>/dev/null
    exit 1
  fi
  echo "Pathwyse: built successfully at $PATHWYSE_BIN"
}

build_pathwyse

# ── Convert instances to Pathwyse format ──
echo
echo "Converting instances to Pathwyse format..."
python3 "$SCRIPTDIR/convert_to_pathwyse.py" --outdir "$CONVERTED_DIR" "${PATHS[@]}"

# ── Collect instance files ──
collect_files() {
  local files=()
  for p in "$@"; do
    if [[ -f "$p" ]]; then
      files+=("$p")
    elif [[ -d "$p" ]]; then
      while IFS= read -r -d '' f; do
        files+=("$f")
      done < <(find "$p" -maxdepth 2 -type f \( -name '*.sppcc' -o -name '*.vrp' -o -name '*.graph' \) -print0 | sort -z)
    else
      echo "Warning: $p not found, skipping" >&2
    fi
  done
  printf '%s\n' "${files[@]}"
}

FILES=()
while IFS= read -r f; do
  [[ -n "$f" ]] && FILES+=("$f")
done < <(collect_files "${PATHS[@]}")

if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "No instance files found." >&2
  exit 1
fi

# ── Pathwyse settings file for ng-path ──
# Pathwyse reads every numeric field with std::stoi (truncates at the decimal
# point). convert_to_pathwyse.py pre-multiplies EDGE_COST by cost_scale
# (default 1e6) and writes a sidecar `.scales` file; we read it per-instance
# and divide the reported Obj by cost_scale to recover the true objective.
# `problem/scaling` is forced to 1.0 (no additional scaling by Pathwyse)
# since our own pre-scaling already carries all the precision we need.
write_pathwyse_settings() {
  local ng_val="$1"
  local ng_mode="off"
  [[ "$ng_val" -gt 0 ]] && ng_mode="standard"
  local settingsfile="$PATHWYSE_DIR/pathwyse.set"

  cat > "$settingsfile" <<SETTINGS
verbosity = 0
problem/scaling/override = 1
problem/scaling = 1.0
main_algorithm = PWDefault
algo/default/timelimit = ${TIMEOUT}.0
algo/default/bidirectional = 0
algo/default/dssr = standard
algo/default/ng = ${ng_mode}
algo/default/ng/set_size = ${ng_val}
algo/default/reserve = 10000000
algo/default/use_visited = 1
algo/default/compare_unreachables = 1
data_collection/level = -1
output/write = 0
SETTINGS
}

# Read cost_scale from the sidecar next to a converted .txt. Falls back to 1
# for legacy files without a sidecar.
read_cost_scale() {
  local pw_file="$1"
  local scales_file="${pw_file%.txt}.scales"
  local val=""
  if [[ -f "$scales_file" ]]; then
    val="$(awk -F= '$1=="cost_scale" {print $2; exit}' "$scales_file")"
  fi
  echo "${val:-1}"
}

# ── Write CSV header ──
if [[ $APPEND -eq 0 ]]; then
  echo "instance,set,ng,cost,time_s,cost_scale,timestamp" > "$OUT_CSV"
fi

# Pathwyse settings are ng-dependent, not instance-dependent, so write once.
# Concurrent script invocations would race on this file.
write_pathwyse_settings "$NG"

# ── Run ──
echo
TOTAL=${#FILES[@]}
IDX=0

printf "%-20s  %3s  %10s  %12s  %s\n" "Instance" "ng" "time(s)" "cost" "status"
printf '%.0s-' {1..70}; echo

# Accumulate for geometric mean
SUM_LOG=0
COUNT=0
SHIFT=1

for file in "${FILES[@]}"; do
  IDX=$((IDX + 1))
  stem="$(basename "$file")"
  stem="${stem%.*}"
  parent="$(basename "$(dirname "$file")")"
  pw_file="$CONVERTED_DIR/$parent/${stem}.txt"

  if [[ ! -f "$pw_file" ]]; then
    echo "Warning: Pathwyse instance not found: $pw_file, skipping" >&2
    continue
  fi

  pw_cost="" pw_time_s="" pw_status="OK"
  pw_raw=""
  if pw_raw=$(cd "$PATHWYSE_DIR" && timeout "${TIMEOUT}s" "$PATHWYSE_BIN" "$pw_file" 2>&1); then
    :
  else
    rc=$?
    if [[ $rc -eq 124 ]]; then
      pw_status="TIMEOUT"
    else
      pw_status="ERROR($rc)"
    fi
  fi

  pw_cost_scale="$(read_cost_scale "$pw_file")"
  if [[ "$pw_status" == "OK" && -n "$pw_raw" ]]; then
    if [[ "$pw_raw" =~ Obj:[[:space:]]*(-?[0-9]+) ]]; then
      pw_cost_raw="${BASH_REMATCH[1]}"
      pw_cost="$(awk -v raw="$pw_cost_raw" -v scale="$pw_cost_scale" 'BEGIN{printf "%.6f", raw / scale}')"
    fi
    if [[ "$pw_raw" =~ global\ time:[[:space:]]*([0-9.eE+-]+) ]]; then
      pw_time_s="${BASH_REMATCH[1]}"
    fi
  fi

  # Set label from parent directory (matches bgspprc.csv convention:
  # spprclib/roberti/ng8/ng16/ng24).
  set_label="$parent"

  ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

  printf "%-20s  %3d  %10s  %12s  %s  [%d/%d]\n" \
    "$stem" "$NG" "${pw_time_s:--}" "${pw_cost:--}" "$pw_status" "$IDX" "$TOTAL"

  echo "${stem},${set_label},${NG},${pw_cost},${pw_time_s},${pw_cost_scale},${ts}" >> "$OUT_CSV"

  if [[ -n "$pw_time_s" && "$pw_status" == "OK" ]]; then
    SUM_LOG="$(awk -v s="$SUM_LOG" -v t="$pw_time_s" -v sh="$SHIFT" 'BEGIN{print s + log(t + sh)}')"
    COUNT=$((COUNT + 1))
  fi
done

# ── Geometric mean summary ──
echo
printf '%.0s-' {1..70}; echo
printf "Pathwyse shifted geometric mean (shift=%ds, n=%d):\n" "$SHIFT" "$COUNT"

if [[ "$COUNT" -gt 0 ]]; then
  geo="$(awk -v s="$SUM_LOG" -v n="$COUNT" -v sh="$SHIFT" 'BEGIN{printf "%.3f", exp(s / n) - sh}')"
  printf "  %ss\n" "$geo"
else
  printf "  No valid results to summarize.\n"
fi

printf "\nResults written to %s\n" "$OUT_CSV"
