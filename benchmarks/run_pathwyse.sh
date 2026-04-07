#!/usr/bin/env bash
# run_pathwyse.sh — Build Pathwyse, convert instances, run both solvers, produce comparison CSV.
#
# Clones, builds, and runs Pathwyse (https://github.com/pathwyse/pathwyse) on
# benchmark instances alongside bgspprc-solve. Produces a comparison CSV with
# runtime, label count (bgspprc only — Pathwyse does not expose label counts),
# and cost verification.
#
# Usage:
#   ./benchmarks/run_pathwyse.sh [--ng K] [--timeout S] [--skip-build] [PATH...]
#
# Arguments:
#   PATH            Instance file or directory of instances.
#                   Default: benchmarks/instances/rcspp/ng8
#   --ng K          ng-neighborhood size (default: 8).
#   --timeout S     Per-instance timeout in seconds (default: 120).
#   --skip-build    Skip cloning/building Pathwyse (reuse existing build).
#
# Environment:
#   SOLVE           Path to bgspprc-solve binary (default: ./build/bgspprc-solve).
#   PATHWYSE_DIR    Path to Pathwyse repo (default: ./build/pathwyse).
#
# Output:
#   benchmarks/comparison_pathwyse.csv — CSV with columns:
#     instance, ng, bgspprc_s, pathwyse_s, bgspprc_labels, pathwyse_labels,
#     bgspprc_cost, pathwyse_cost, cost_match, ratio
#
# Prerequisites:
#   bgspprc-solve must be built:
#     cmake -B build -DCMAKE_CXX_COMPILER=g++-14 && cmake --build build
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
REPODIR="$(cd "$SCRIPTDIR/.." && pwd)"
SOLVE="${SOLVE:-$REPODIR/build/bgspprc-solve}"
PATHWYSE_DIR="${PATHWYSE_DIR:-$REPODIR/build/pathwyse}"
PATHWYSE_BIN="$PATHWYSE_DIR/bin/pathwyse"
OUT_CSV="$SCRIPTDIR/comparison_pathwyse.csv"
CONVERTED_DIR="$SCRIPTDIR/instances/pathwyse"
TIMEOUT=120
NG=8
SKIP_BUILD=0
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
    -h|--help)    usage ;;
    *)            PATHS+=("$1"); shift ;;
  esac
done

# Default paths
if [[ ${#PATHS[@]} -eq 0 ]]; then
  PATHS=("$SCRIPTDIR/instances/rcspp/ng${NG}")
fi

# ── Verify bgspprc-solve ──
if [[ ! -x "$SOLVE" ]]; then
  echo "Error: bgspprc-solve not found at $SOLVE" >&2
  echo "Build first: cmake -B build -DCMAKE_CXX_COMPILER=g++-14 && cmake --build build" >&2
  exit 1
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
python3 "$SCRIPTDIR/convert_to_pathwyse.py" --ng "$NG" --outdir "$CONVERTED_DIR" "${PATHS[@]}"

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
# Pathwyse uses int objective internally. We force scaling=1 so the reported
# Obj equals int(actual_cost). For integer-cost instances this is exact; for
# fractional costs the error is bounded by the number of arcs in the path.
PW_SCALE=1

write_pathwyse_settings() {
  local ng_val="$1"
  local settingsfile="$PATHWYSE_DIR/pathwyse.set"

  cat > "$settingsfile" <<SETTINGS
verbosity = 0
problem/scaling/override = 1
problem/scaling = 1.0
main_algorithm = PWDefault
algo/default/timelimit = ${TIMEOUT}.0
algo/default/bidirectional = 0
algo/default/dssr = standard
algo/default/ng = off
algo/default/ng/set_size = ${ng_val}
algo/default/reserve = 10000000
algo/default/use_visited = 1
algo/default/compare_unreachables = 1
data_collection/level = -1
output/write = 0
SETTINGS
}

write_pathwyse_settings_with_ng() {
  local ng_val="$1"
  local settingsfile="$PATHWYSE_DIR/pathwyse.set"

  cat > "$settingsfile" <<SETTINGS
verbosity = 0
problem/scaling/override = 1
problem/scaling = 1.0
main_algorithm = PWDefault
algo/default/timelimit = ${TIMEOUT}.0
algo/default/bidirectional = 0
algo/default/dssr = standard
algo/default/ng = standard
algo/default/ng/set_size = ${ng_val}
algo/default/reserve = 10000000
algo/default/use_visited = 1
algo/default/compare_unreachables = 1
data_collection/level = -1
output/write = 0
SETTINGS
}

# ── Write CSV header ──
echo "instance,ng,bgspprc_s,pathwyse_s,bgspprc_labels,pathwyse_labels,bgspprc_cost,pathwyse_cost,cost_match,ratio" > "$OUT_CSV"

# ── Run comparison ──
echo
TOTAL=${#FILES[@]}
IDX=0
# Pathwyse uses int objective (scale=1). Int truncation introduces error up to
# 1 per arc. For paths with ~10 arcs, total error bounded by ~10.
COST_EPS=10.0

printf "%-20s  %3s  %10s  %10s  %12s  %10s  %10s  %s\n" \
  "Instance" "ng" "bg(s)" "pw(s)" "bg_cost" "pw_cost" "ratio" "match"
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
  ext="${file##*.}"

  # Determine Pathwyse converted file
  parent="$(basename "$(dirname "$file")")"
  pw_file="$CONVERTED_DIR/$parent/${stem}.txt"

  if [[ ! -f "$pw_file" ]]; then
    echo "Warning: Pathwyse instance not found: $pw_file, skipping" >&2
    continue
  fi

  # ── Run bgspprc-solve ──
  bg_cost="" bg_time_s="" bg_labels="" bg_status="OK"
  if bg_output=$(timeout "${TIMEOUT}s" "$SOLVE" --ng "$NG" "$file" 2>&1); then
    bg_line="$(echo "$bg_output" | head -1)"
    if [[ "$bg_line" =~ cost=([0-9.eE+-]+) ]]; then
      bg_cost="${BASH_REMATCH[1]}"
    fi
    if [[ "$bg_line" =~ ([0-9.]+)ms ]]; then
      bg_time_ms="${BASH_REMATCH[1]}"
      bg_time_s="$(awk "BEGIN{printf \"%.3f\", $bg_time_ms/1000}")"
    fi
    if [[ "$bg_line" =~ paths=([0-9]+) ]]; then
      bg_labels="${BASH_REMATCH[1]}"
    fi
  else
    rc=$?
    if [[ $rc -eq 124 ]]; then
      bg_status="TIMEOUT"
    else
      bg_status="ERROR($rc)"
    fi
  fi

  # ── Run Pathwyse ──
  # Configure Pathwyse with ng settings
  if [[ "$NG" -gt 0 ]]; then
    write_pathwyse_settings_with_ng "$NG"
  else
    write_pathwyse_settings "$NG"
  fi

  pw_cost="" pw_time_s="" pw_labels="" pw_status="OK"
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

  # Parse Pathwyse output
  # Pathwyse output at verbosity=0: "Status Obj: VALUE" and "PWDefault global time: TIME"
  # We force scaling=1, so Obj = int(actual_cost).
  if [[ "$pw_status" == "OK" && -n "$pw_raw" ]]; then
    # Extract objective (may be negative)
    if [[ "$pw_raw" =~ Obj:[[:space:]]*(-?[0-9.eE+-]+) ]]; then
      pw_cost_raw="${BASH_REMATCH[1]}"
      pw_cost="$(awk "BEGIN{printf \"%.3f\", $pw_cost_raw / $PW_SCALE}")"
    fi
    # Extract global time
    if [[ "$pw_raw" =~ global\ time:[[:space:]]*([0-9.eE+-]+) ]]; then
      pw_time_s="${BASH_REMATCH[1]}"
    fi
  fi

  # Labels: Pathwyse does not expose label counts at verbosity=0
  pw_labels=""

  # Cost comparison
  cost_match=""
  if [[ -n "$bg_cost" && -n "$pw_cost" ]]; then
    cost_match="$(awk -v a="$bg_cost" -v b="$pw_cost" -v e="$COST_EPS" \
      'BEGIN{d=a-b; if(d<0)d=-d; print (d<=e) ? "YES" : "NO"}')"
  fi

  # Ratio
  ratio=""
  if [[ -n "$bg_time_s" && -n "$pw_time_s" && "$bg_time_s" != "0.000" && "$pw_time_s" != "0.000" ]]; then
    ratio="$(awk "BEGIN{printf \"%.2f\", $bg_time_s / $pw_time_s}")"
  fi

  # Print row
  printf "%-20s  %3d  %10s  %10s  %12s  %10s  %10s  %s  [%d/%d]\n" \
    "$stem" "$NG" "${bg_time_s:--}" "${pw_time_s:--}" \
    "${bg_cost:--}" "${pw_cost:--}" "${ratio:--}" "${cost_match:--}" "$IDX" "$TOTAL"

  # Write CSV row
  echo "${stem},${NG},${bg_time_s},${pw_time_s},${bg_labels},${pw_labels},${bg_cost},${pw_cost},${cost_match},${ratio}" >> "$OUT_CSV"

  # Accumulate for geometric mean (only if both have valid times)
  if [[ -n "$bg_time_s" && -n "$pw_time_s" && "$bg_status" == "OK" && "$pw_status" == "OK" ]]; then
    SUM_LOG_BG="$(awk "BEGIN{print $SUM_LOG_BG + log($bg_time_s + $SHIFT)}")"
    SUM_LOG_PW="$(awk "BEGIN{print $SUM_LOG_PW + log($pw_time_s + $SHIFT)}")"
    COUNT=$((COUNT + 1))
  fi
done

# ── Geometric mean summary ──
echo
printf '%.0s-' {1..90}; echo
printf "Shifted geometric mean (shift=%ds, n=%d):\n" "$SHIFT" "$COUNT"

if [[ "$COUNT" -gt 0 ]]; then
  geo_bg="$(awk "BEGIN{printf \"%.3f\", exp($SUM_LOG_BG / $COUNT) - $SHIFT}")"
  geo_pw="$(awk "BEGIN{printf \"%.3f\", exp($SUM_LOG_PW / $COUNT) - $SHIFT}")"
  geo_ratio="$(awk "BEGIN{printf \"%.2f\", ($geo_bg + $SHIFT) / ($geo_pw + $SHIFT)}")"
  printf "  bgspprc: %ss  pathwyse: %ss  ratio: %s\n" "$geo_bg" "$geo_pw" "$geo_ratio"
else
  printf "  No valid results to summarize.\n"
fi

printf "\nResults written to %s\n" "$OUT_CSV"
