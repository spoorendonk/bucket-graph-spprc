#!/usr/bin/env bash
# run_pathwyse.sh — Build Pathwyse, convert instances, run Pathwyse, join against
# pre-computed bgspprc results, produce comparison CSV.
#
# bgspprc rows are read from benchmarks/bgspprc.csv (mode=para-bidir) — this
# script does NOT invoke bgspprc-solve. Re-run benchmarks/run_benchmarks.sh
# first if bgspprc.csv is stale.
#
# Usage:
#   ./benchmarks/run_pathwyse.sh [--ng K] [--timeout S] [--skip-build] [--append]
#                                [--bgspprc-csv FILE] [PATH...]
#
# Arguments:
#   PATH              Instance file or directory of instances.
#                     Default: benchmarks/instances/rcspp/ng8
#   --ng K            ng-neighborhood size (default: 8).
#   --timeout S       Per-instance Pathwyse timeout in seconds (default: 120).
#   --skip-build      Skip cloning/building Pathwyse (reuse existing build).
#   --append          Append rows to existing CSV (skip header write + overwrite).
#   --bgspprc-csv F   Source of bgspprc rows (default: benchmarks/bgspprc.csv).
#
# Environment:
#   PATHWYSE_DIR      Path to Pathwyse repo (default: ./build/pathwyse).
#
# Output:
#   benchmarks/comparison_pathwyse.csv — CSV with columns:
#     instance, ng, bgspprc_s, pathwyse_s, bgspprc_cost, pathwyse_cost,
#     bg_leq, ratio
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
REPODIR="$(cd "$SCRIPTDIR/.." && pwd)"
PATHWYSE_DIR="${PATHWYSE_DIR:-$REPODIR/build/pathwyse}"
PATHWYSE_BIN="$PATHWYSE_DIR/bin/pathwyse"
OUT_CSV="$SCRIPTDIR/comparison_pathwyse.csv"
CONVERTED_DIR="$SCRIPTDIR/instances/pathwyse"
BG_CSV="$SCRIPTDIR/bgspprc.csv"
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
    --bgspprc-csv) BG_CSV="$2"; shift 2 ;;
    -h|--help)    usage ;;
    *)            PATHS+=("$1"); shift ;;
  esac
done

# Default paths
if [[ ${#PATHS[@]} -eq 0 ]]; then
  PATHS=("$SCRIPTDIR/instances/rcspp/ng${NG}")
fi

# ── Verify bgspprc results CSV ──
if [[ ! -f "$BG_CSV" ]]; then
  echo "Error: bgspprc results CSV not found at $BG_CSV" >&2
  echo "Run benchmarks/run_benchmarks.sh first, or pass --bgspprc-csv FILE." >&2
  exit 1
fi
BG_CSV_HEADER_EXPECTED="instance,set,ng,mode,cost,paths,time_s,timestamp"
if [[ "$(head -1 "$BG_CSV")" != "$BG_CSV_HEADER_EXPECTED" ]]; then
  echo "Error: $BG_CSV header mismatch. Expected: $BG_CSV_HEADER_EXPECTED" >&2
  exit 1
fi

# Look up (cost,time_s) for a (stem, ng) pair from bgspprc.csv (mode=para-bidir).
# Prints "cost,time_s" or empty string on miss.
lookup_bg() {
  local stem="$1" ng="$2"
  awk -F, -v s="$stem" -v n="$ng" '
    NR>1 && $1==s && $3==n && $4=="para-bidir" {print $5","$7; exit}
  ' "$BG_CSV"
}

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
  echo "instance,ng,bgspprc_s,pathwyse_s,bgspprc_cost,pathwyse_cost,bg_leq,ratio" > "$OUT_CSV"
fi

# Pathwyse settings are ng-dependent, not instance-dependent, so write once.
# Concurrent script invocations would race on this file.
write_pathwyse_settings "$NG"

# ── Run comparison ──
echo
TOTAL=${#FILES[@]}
IDX=0
# Costs are NOT expected to match: bgspprc uses compressed ng-path (weaker
# dominance, more negative paths found), while Pathwyse uses full O(|V|)
# unreachable vectors (tighter dominance, fewer paths). bgspprc cost <= Pathwyse
# cost is the expected relationship. We report bg_leq (bg <= pw) as a sanity check.

printf "%-20s  %3s  %10s  %10s  %12s  %10s  %10s  %s\n" \
  "Instance" "ng" "bg(s)" "pw(s)" "bg_cost" "pw_cost" "ratio" "bg<=pw"
printf '%.0s-' {1..90}; echo

# Accumulate for geometric mean
SUM_LOG_BG=0
SUM_LOG_PW=0
COUNT=0
SHIFT=1

for file in "${FILES[@]}"; do
  IDX=$((IDX + 1))
  stem="$(basename "$file")"
  stem="${stem%.*}"

  # Determine Pathwyse converted file
  parent="$(basename "$(dirname "$file")")"
  pw_file="$CONVERTED_DIR/$parent/${stem}.txt"

  if [[ ! -f "$pw_file" ]]; then
    echo "Warning: Pathwyse instance not found: $pw_file, skipping" >&2
    continue
  fi

  # ── Look up bgspprc result ──
  # Three cases: row absent (MISSING — skip pathwyse), row present with empty
  # cost/time (BG_TIMEOUT — bgspprc itself didn't finish; still run pathwyse
  # to see if it finds a path bgspprc missed), or populated row (OK).
  bg_cost="" bg_time_s="" bg_status="OK"
  bg_row="$(lookup_bg "$stem" "$NG")"
  if [[ -z "$bg_row" ]]; then
    echo "Warning: no bgspprc row for $stem ng=$NG, skipping" >&2
    continue
  elif [[ "$bg_row" == "," ]]; then
    bg_status="BG_TIMEOUT"
  else
    bg_cost="${bg_row%,*}"
    bg_time_s="${bg_row#*,}"
  fi

  # ── Run Pathwyse ──
  pw_cost="" pw_time_s="" pw_status="OK"
  # Run Pathwyse from its directory so it picks up pathwyse.set
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

  # Parse Pathwyse output. Obj is an integer; divide by per-instance cost_scale
  # (read from the sidecar written by convert_to_pathwyse.py) to recover the
  # true objective.
  pw_cost_scale="$(read_cost_scale "$pw_file")"
  if [[ "$pw_status" == "OK" && -n "$pw_raw" ]]; then
    if [[ "$pw_raw" =~ Obj:[[:space:]]*(-?[0-9]+) ]]; then
      pw_cost_raw="${BASH_REMATCH[1]}"
      pw_cost="$(awk -v raw="$pw_cost_raw" -v scale="$pw_cost_scale" 'BEGIN{printf "%.6f", raw / scale}')"
    fi
    # Extract global time
    if [[ "$pw_raw" =~ global\ time:[[:space:]]*([0-9.eE+-]+) ]]; then
      pw_time_s="${BASH_REMATCH[1]}"
    fi
  fi

  # Cost comparison: bg cost <= pw cost expected (weaker dominance finds more
  # paths). Tolerance accounts for accumulated per-arc rounding in the int
  # scaling used by the converter (≈ n_arcs / cost_scale). A bg cost of 0
  # typically means bgspprc found no improving path — label `bg_zero` so these
  # rows don't get lumped with real `bg_gt` regressions.
  bg_leq=""
  if [[ -n "$bg_cost" && -n "$pw_cost" ]]; then
    bg_leq="$(awk -v bg="$bg_cost" -v pw="$pw_cost" -v s="$pw_cost_scale" '
      BEGIN{
        tol = 100.0 / s
        if (bg+0 > -1e-9 && bg+0 < 1e-9) print "bg_zero"
        else print (bg <= pw + tol) ? "bg_leq" : "bg_gt"
      }')"
  fi

  # Ratio
  ratio=""
  if [[ -n "$bg_time_s" && -n "$pw_time_s" && "$bg_time_s" != "0.000" && "$pw_time_s" != "0.000" ]]; then
    ratio="$(awk -v bg="$bg_time_s" -v pw="$pw_time_s" 'BEGIN{printf "%.2f", bg / pw}')"
  fi

  # Print row
  printf "%-20s  %3d  %10s  %10s  %12s  %10s  %10s  %s  [%d/%d]\n" \
    "$stem" "$NG" "${bg_time_s:--}" "${pw_time_s:--}" \
    "${bg_cost:--}" "${pw_cost:--}" "${ratio:--}" "${bg_leq:--}" "$IDX" "$TOTAL"

  # Write CSV row
  echo "${stem},${NG},${bg_time_s},${pw_time_s},${bg_cost},${pw_cost},${bg_leq},${ratio}" >> "$OUT_CSV"

  # Accumulate for geometric mean (only if both have valid times)
  if [[ -n "$bg_time_s" && -n "$pw_time_s" && "$bg_status" == "OK" && "$pw_status" == "OK" ]]; then
    SUM_LOG_BG="$(awk -v s="$SUM_LOG_BG" -v t="$bg_time_s" -v sh="$SHIFT" 'BEGIN{print s + log(t + sh)}')"
    SUM_LOG_PW="$(awk -v s="$SUM_LOG_PW" -v t="$pw_time_s" -v sh="$SHIFT" 'BEGIN{print s + log(t + sh)}')"
    COUNT=$((COUNT + 1))
  fi
done

# ── Geometric mean summary ──
echo
printf '%.0s-' {1..90}; echo
printf "Shifted geometric mean (shift=%ds, n=%d):\n" "$SHIFT" "$COUNT"

if [[ "$COUNT" -gt 0 ]]; then
  geo_bg="$(awk -v s="$SUM_LOG_BG" -v n="$COUNT" -v sh="$SHIFT" 'BEGIN{printf "%.3f", exp(s / n) - sh}')"
  geo_pw="$(awk -v s="$SUM_LOG_PW" -v n="$COUNT" -v sh="$SHIFT" 'BEGIN{printf "%.3f", exp(s / n) - sh}')"
  geo_ratio="$(awk -v bg="$geo_bg" -v pw="$geo_pw" -v sh="$SHIFT" 'BEGIN{printf "%.2f", (bg + sh) / (pw + sh)}')"
  printf "  bgspprc: %ss  pathwyse: %ss  ratio: %s\n" "$geo_bg" "$geo_pw" "$geo_ratio"
else
  printf "  No valid results to summarize.\n"
fi

printf "\nResults written to %s\n" "$OUT_CSV"
