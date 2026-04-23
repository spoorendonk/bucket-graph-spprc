#!/usr/bin/env bash
# run_benchmarks.sh — Run bgspprc-solve on benchmark instances and produce CSV results.
#
# Runs bgspprc-solve on instance files (.sppcc/.vrp/.graph), parses stdout,
# and writes rows to benchmarks/bgspprc.csv (one row per instance+set+ng+mode
# combo, replaced on re-run).
#
# Usage:
#   ./benchmarks/run_benchmarks.sh [--ng K] [--mode M] [--timeout S] [PATH...]
#
# Arguments:
#   PATH           Instance file or directory of instances.
#                  Default: benchmarks/instances/spprclib and
#                  benchmarks/instances/roberti.
#   --ng K         Pass --ng K to solver (ng-neighborhood size).
#   --mode M       Solver mode (default: para_bidir). Two orthogonal axes:
#                    bidir axis: mono | bidir | para_bidir
#                    SIMD axis:  _base (SIMD off) | _vec (SIMD on)
#                  Data-parallelism is always on.
#
#                    mono_base        → bgspprc-solve-nosimd --mono
#                    mono_vec         → bgspprc-solve        --mono
#                    bidir_base       → bgspprc-solve-nosimd --no-parallel-bidir
#                    bidir_vec        → bgspprc-solve        --no-parallel-bidir
#                    para_bidir_base  → bgspprc-solve-nosimd (defaults)
#                    para_bidir       → bgspprc-solve        (defaults)
#   --max-paths N  Pass --max-paths N to solver.
#   --timeout S    Per-instance timeout in seconds (default: 120).
#
# Environment:
#   SOLVE          Path to SIMD-on solver    (default: ./build/bgspprc-solve).
#   SOLVE_NOSIMD   Path to SIMD-off solver   (default: ./build/bgspprc-solve-nosimd).
#
# Output:
#   benchmarks/bgspprc.csv — CSV with columns:
#     instance, set, ng, mode, cost, paths, time_s, timestamp
#   Existing rows for the same (instance, set, ng, mode) are replaced;
#   other rows preserved.
#
# Parsed fields from bgspprc-solve stdout:
#   "name  type  n=NN  arcs=NN  cost=X.XXX  paths=N  X.Xms"
#   → cost, paths, time_ms (converted to time_s for CSV)
#
# The "set" column is inferred from the parent directory name.
# The "ng" column uses --ng K if given, else inferred from grandparent
# directory name matching ng[0-9]+ (e.g. rcspp/ng16/ → ng=16).
#
# Prerequisites:
#   bgspprc-solve must be built:
#     cmake -B build -DCMAKE_CXX_COMPILER=g++-14 && cmake --build build
set -euo pipefail

# ── Usage ──
usage() {
  sed -n '2,/^$/s/^# \?//p' "${BASH_SOURCE[0]}"
  exit "${1:-0}"
}

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
REPODIR="$(cd "$SCRIPTDIR/.." && pwd)"
SOLVE="${SOLVE:-$REPODIR/build/bgspprc-solve}"
SOLVE_NOSIMD="${SOLVE_NOSIMD:-$REPODIR/build/bgspprc-solve-nosimd}"
CSV="${SCRIPTDIR}/bgspprc.csv"

# ── Args ──
NG_FLAG=()
MAX_PATHS_FLAG=()
TIMEOUT=120
MODE="para_bidir"
PATHS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ng)        NG_FLAG=(--ng "$2"); shift 2 ;;
    --mode)      MODE="$2"; shift 2 ;;
    --max-paths) MAX_PATHS_FLAG=(--max-paths "$2"); shift 2 ;;
    --timeout)   TIMEOUT="$2"; shift 2 ;;
    -h|--help)   usage ;;
    *)           PATHS+=("$1"); shift ;;
  esac
done

# Map mode to (binary, flags). SIMD axis selects which binary; bidir axis
# selects the --mono / --no-parallel-bidir flag.
SOLVE_BIN="$SOLVE"
MODE_FLAGS=()
case "$MODE" in
  mono_base)       SOLVE_BIN="$SOLVE_NOSIMD"; MODE_FLAGS=(--mono) ;;
  mono_vec)        SOLVE_BIN="$SOLVE";        MODE_FLAGS=(--mono) ;;
  bidir_base)      SOLVE_BIN="$SOLVE_NOSIMD"; MODE_FLAGS=(--no-parallel-bidir) ;;
  bidir_vec)       SOLVE_BIN="$SOLVE";        MODE_FLAGS=(--no-parallel-bidir) ;;
  para_bidir_base) SOLVE_BIN="$SOLVE_NOSIMD"; MODE_FLAGS=() ;;
  para_bidir)      SOLVE_BIN="$SOLVE";        MODE_FLAGS=() ;;
  *)
    echo "Invalid --mode: $MODE" >&2
    echo "Expected: mono_base | mono_vec | bidir_base | bidir_vec | para_bidir_base | para_bidir" >&2
    exit 1
    ;;
esac

if [[ ! -x "$SOLVE_BIN" ]]; then
  echo "Solver binary not found or not executable: $SOLVE_BIN" >&2
  echo "Build with: cmake --build build --target $(basename "$SOLVE_BIN")" >&2
  exit 1
fi

# Default paths
if [[ ${#PATHS[@]} -eq 0 ]]; then
  PATHS=("$SCRIPTDIR/instances/spprclib" "$SCRIPTDIR/instances/roberti")
fi

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

# ── Ensure CSV header ──
EXPECTED_HEADER="instance,set,ng,mode,cost,paths,time_s,timestamp"
if [[ ! -f "$CSV" ]]; then
  echo "$EXPECTED_HEADER" > "$CSV"
else
  existing_header="$(head -1 "$CSV")"
  if [[ "$existing_header" != "$EXPECTED_HEADER" ]]; then
    echo "Error: $CSV has an outdated header." >&2
    echo "  expected: $EXPECTED_HEADER" >&2
    echo "  found:    $existing_header" >&2
    echo "Remove the file and re-run to regenerate with the current schema." >&2
    exit 1
  fi
fi

# ── Infer set and ng from path ──
infer_set() {
  basename "$(dirname "$1")"
}

infer_ng() {
  # If --ng was passed, use that
  if [[ ${#NG_FLAG[@]} -gt 0 ]]; then
    echo "${NG_FLAG[1]}"
    return
  fi
  # Infer from parent or grandparent dir name matching ng[0-9]+
  local parent grandparent
  parent="$(basename "$(dirname "$1")")"
  grandparent="$(basename "$(dirname "$(dirname "$1")")")"
  if [[ "$parent" =~ ^ng([0-9]+)$ ]]; then
    echo "${BASH_REMATCH[1]}"
  elif [[ "$grandparent" =~ ^ng([0-9]+)$ ]]; then
    echo "${BASH_REMATCH[1]}"
  else
    echo ""
  fi
}

# ── Run ──
TIMESTAMP="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
TOTAL=${#FILES[@]}
IDX=0

printf "%-30s  %8s  %5s  %8s  %s\n" "Instance" "Cost" "Paths" "Time(ms)" "Status"
printf '%.0s-' {1..75}; echo

for file in "${FILES[@]}"; do
  IDX=$((IDX + 1))
  stem="$(basename "$file")"
  stem="${stem%.*}"
  set_name="$(infer_set "$file")"
  ng_val="$(infer_ng "$file")"

  # Run solver with timeout
  output=""
  status="OK"
  if output=$(timeout "${TIMEOUT}s" "$SOLVE_BIN" "${MODE_FLAGS[@]}" "${NG_FLAG[@]}" "${MAX_PATHS_FLAG[@]}" "$file" 2>&1); then
    :
  else
    rc=$?
    if [[ $rc -eq 124 ]]; then
      status="TIMEOUT"
    else
      status="ERROR($rc)"
    fi
  fi

  # Parse output: "name  type  n=NN  arcs=NN  cost=X.XXX  paths=N  X.Xms"
  cost="" paths="" time_ms=""
  if [[ "$status" == "OK" && -n "$output" ]]; then
    # Extract from first non-path line
    line="$(echo "$output" | head -1)"
    if [[ "$line" =~ cost=([0-9.eE+-]+) ]]; then
      cost="${BASH_REMATCH[1]}"
    fi
    if [[ "$line" =~ paths=([0-9]+) ]]; then
      paths="${BASH_REMATCH[1]}"
    fi
    if [[ "$line" =~ ([0-9.]+)ms ]]; then
      time_ms="${BASH_REMATCH[1]}"
    fi
  fi

  # Convert ms to seconds for CSV
  time_s=""
  if [[ -n "$time_ms" ]]; then
    time_s="$(awk "BEGIN{printf \"%.3f\", $time_ms/1000}")"
  fi

  # Progress output
  printf "%-30s  %8s  %5s  %8s  %s  [%d/%d]\n" \
    "$stem" "${cost:--}" "${paths:--}" "${time_ms:--}" "$status" "$IDX" "$TOTAL"

  # Deduplicate: remove existing rows for this instance+set+ng+mode
  if [[ -f "$CSV" ]]; then
    tmp="$(mktemp)"
    awk -F, -v inst="$stem" -v s="$set_name" -v n="$ng_val" -v m="$MODE" \
      'NR==1 || !($1==inst && $2==s && $3==n && $4==m)' "$CSV" > "$tmp"
    mv "$tmp" "$CSV"
  fi

  # Append result (even for failures, with empty fields)
  echo "${stem},${set_name},${ng_val},${MODE},${cost},${paths},${time_s},${TIMESTAMP}" >> "$CSV"
done

echo
echo "Results written to $CSV"
