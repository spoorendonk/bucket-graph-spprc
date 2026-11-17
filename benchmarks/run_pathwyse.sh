#!/usr/bin/env bash
# run_pathwyse.sh — Build Pathwyse, convert instances, run Pathwyse, write rows
# to a CSV. One row per (instance, ng[, algo, parallel]) with cost + runtime.
#
# No comparison logic — run `compute_means.py pathwyse` (1.0) or
# `compute_means.py pathwyse11` (1.1.0) to join the rows against bgspprc's
# in-memory and print the comparison table.
#
# Usage:
#   ./benchmarks/run_pathwyse.sh [--pathwyse V] [--algo A] [--parallel P]
#                                [--ng K] [--timeout S] [--skip-build]
#                                [--append] [PATH...]
#
# Arguments:
#   PATH              Instance file or directory of instances.
#                     Default: benchmarks/instances/rcspp/ng8 (1.0),
#                     benchmarks/instances/{spprclib,roberti} (1.1.0, which
#                     takes .sppcc and .vrp only).
#   --pathwyse V      1.1.0 (default) or 1.0 — the release to build and run.
#                     1.0 is pinned commit d53c01b with the two parity patches
#                     in benchmarks/patches/, the paper's comparison; 1.1.0 is
#                     the release tag, unpatched (see benchmarks/README.md,
#                     "Pathwyse parity").
#   --algo A          1.1.0 only: PWBucket (default) or PWDefault.
#   --parallel P      1.1.0 only: 0 (default) runs the forward and backward
#                     searches in turn, 1 runs them concurrently
#                     (`algo/default/parallel`).
#   --ng K            ng-neighborhood size (default: 8).
#   --timeout S       Per-instance Pathwyse timeout in seconds (default: 120).
#   --skip-build      Skip cloning/building Pathwyse (reuse existing build).
#   --append          1.0 only: append rows to the existing CSV (skip the
#                     header write + overwrite). 1.1.0 always keeps the CSV
#                     and replaces only rows of the same (instance, set, ng,
#                     algo, parallel).
#
# Environment:
#   PATHWYSE_DIR      Path to the Pathwyse checkout (default:
#                     ./build/pathwyse for 1.0, ./build/pathwyse-1.1.0).
#   PATHWYSE_CSV      Output CSV (default: benchmarks/pathwyse.csv for 1.0,
#                     benchmarks/pathwyse-1.1.0/pathwyse.csv for 1.1.0).
#   SOLVE             bgspprc-solve, which writes the ng sets 1.1.0 reads
#                     (default: ./build/bgspprc-solve).
#   RUN_PREFIX        Command each Pathwyse run goes under, word-split
#                     (default: none), e.g. RUN_PREFIX="xk lock --" to take a
#                     shared machine's lock per instance.
#
# Output (1.0): benchmarks/pathwyse.csv — CSV with columns:
#     instance, set, ng, cost, time_s, cost_scale, timestamp
# Output (1.1.0): benchmarks/pathwyse-1.1.0/pathwyse.csv — CSV with columns:
#     instance, set, ng, algo, parallel, status, cost, time_s, cost_scale,
#     timestamp
#   cost_scale is the per-instance integer scaling used by
#   convert_to_pathwyse.py; downstream consumers use it to size
#   float-comparison tolerances. 1.1.0's status is OK, TIMEOUT, ERROR(rc)
#   or UNPARSED; cost/time_s are empty unless it is OK.
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
REPODIR="$(cd "$SCRIPTDIR/.." && pwd)"
SOLVE="${SOLVE:-$REPODIR/build/bgspprc-solve}"
TIMEOUT=120
NG=8
SKIP_BUILD=0
APPEND=0
VERSION="1.1.0"
ALGO="PWBucket"
PARALLEL=0
ALGO_GIVEN=0
PATHS=()

# ── Usage ──
usage() {
  sed -n '2,/^$/s/^# \?//p' "${BASH_SOURCE[0]}"
  exit "${1:-0}"
}

# ── Args ──
while [[ $# -gt 0 ]]; do
  case "$1" in
    --pathwyse)   VERSION="$2"; shift 2 ;;
    --algo)       ALGO="$2"; ALGO_GIVEN=1; shift 2 ;;
    --parallel)   PARALLEL="$2"; ALGO_GIVEN=1; shift 2 ;;
    --ng)         NG="$2"; shift 2 ;;
    --timeout)    TIMEOUT="$2"; shift 2 ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    --append)     APPEND=1; shift ;;
    -h|--help)    usage ;;
    *)            PATHS+=("$1"); shift ;;
  esac
done

case "$VERSION" in
  1.0|1.1.0) ;;
  *) echo "Error: --pathwyse must be 1.0 or 1.1.0, not $VERSION" >&2; exit 1 ;;
esac
case "$ALGO" in
  PWBucket|PWDefault) ;;
  *) echo "Error: --algo must be PWBucket or PWDefault, not $ALGO" >&2; exit 1 ;;
esac
case "$PARALLEL" in
  0|1) ;;
  *) echo "Error: --parallel must be 0 or 1, not $PARALLEL" >&2; exit 1 ;;
esac
if [[ "$VERSION" == "1.0" && $ALGO_GIVEN -eq 1 ]]; then
  # 1.0 runs the paper's configuration only: PWDefault, sequential.
  echo "Error: --algo and --parallel are for --pathwyse 1.1.0" >&2
  exit 1
fi
if [[ "$VERSION" == "1.1.0" && $APPEND -eq 1 ]]; then
  echo "Error: --append is for --pathwyse 1.0 (1.1.0 always keeps the CSV)" >&2
  exit 1
fi

read -r -a RUN_PREFIX_ARR <<<"${RUN_PREFIX:-}"

# Default paths. 1.1.0 compares on .sppcc/.vrp only: its ng sets come from
# `bgspprc-solve --write-ng`, which takes no .graph.
if [[ ${#PATHS[@]} -eq 0 ]]; then
  if [[ "$VERSION" == "1.0" ]]; then
    PATHS=("$SCRIPTDIR/instances/rcspp/ng${NG}")
  else
    PATHS=("$SCRIPTDIR/instances/spprclib" "$SCRIPTDIR/instances/roberti")
  fi
fi

# ── Build Pathwyse ──
# 1.0: pinned upstream commit. The patches in benchmarks/patches/ are
# line-anchored against this revision.
# 1.1.0: the release tag's commit, unpatched: two-cycle elimination is off by
# configuration (`algo/two_cycle_elimination = 0`) and the ng sets are read
# from bgspprc (`algo/default/ng/set_path`), which replace the two patches.
if [[ "$VERSION" == "1.0" ]]; then
  PATHWYSE_REV="d53c01b2c02743ad2def2c5a6f0f2bfebef1d1b1"
  PATCHES=(
    "$SCRIPTDIR/patches/pathwyse-skip-terminals-in-buildng.patch"
    "$SCRIPTDIR/patches/pathwyse-pure-ng.patch"
  )
  PATHWYSE_DIR="${PATHWYSE_DIR:-$REPODIR/build/pathwyse}"
  OUT_CSV="${PATHWYSE_CSV:-$SCRIPTDIR/pathwyse.csv}"
  CONVERTED_DIR="$SCRIPTDIR/instances/pathwyse"
  CONVERT_FLAGS=()
else
  PATHWYSE_REV="15a7468fc13891d77bf25bbc9235a07101f271ba"  # tag 1.1.0
  PATCHES=()
  PATHWYSE_DIR="${PATHWYSE_DIR:-$REPODIR/build/pathwyse-1.1.0}"
  OUT_CSV="${PATHWYSE_CSV:-$SCRIPTDIR/pathwyse-1.1.0/pathwyse.csv}"
  CONVERTED_DIR="$SCRIPTDIR/instances/pathwyse-1.1.0"
  CONVERT_FLAGS=(--wide-costs)
fi
PATHWYSE_BIN="$PATHWYSE_DIR/bin/pathwyse"

build_pathwyse() {
  if [[ -x "$PATHWYSE_BIN" && $SKIP_BUILD -eq 1 ]]; then
    echo "Pathwyse: reusing existing build at $PATHWYSE_BIN"
    return
  fi

  echo "Pathwyse: cloning repository..."
  if [[ -d "$PATHWYSE_DIR" ]]; then
    rm -rf "$PATHWYSE_DIR"
  fi
  # Full clone (no --depth 1) so we can checkout the pinned revision below.
  git clone https://github.com/pathwyse/pathwyse.git "$PATHWYSE_DIR"

  echo "Pathwyse: checking out pinned revision $PATHWYSE_REV..."
  git -C "$PATHWYSE_DIR" checkout --detach "$PATHWYSE_REV"

  if [[ ${#PATCHES[@]} -gt 0 ]]; then
    echo "Pathwyse: applying parity patches..."
  fi
  for p in "${PATCHES[@]}"; do
    if [[ ! -f "$p" ]]; then
      echo "Error: patch not found: $p" >&2
      exit 1
    fi
    echo "  applying $(basename "$p")"
    ( cd "$PATHWYSE_DIR" && patch -p1 ) <"$p"
  done

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
  echo "Pathwyse: built successfully at $PATHWYSE_BIN ($VERSION, $PATHWYSE_REV)"
}

build_pathwyse

# ── Convert instances to Pathwyse format ──
echo
echo "Converting instances to Pathwyse format..."
python3 "$SCRIPTDIR/convert_to_pathwyse.py" "${CONVERT_FLAGS[@]}" \
  --outdir "$CONVERTED_DIR" "${PATHS[@]}"

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
# 1.0 reads every numeric field with std::stoi (truncates at the decimal
# point). convert_to_pathwyse.py pre-multiplies EDGE_COST by a per-extension
# cost_scale (sppcc=1, vrp=1000, graph=1000) and writes a sidecar `.scales`
# file; we read it per-instance and divide the reported Obj by cost_scale
# to recover the true objective.
# `problem/scaling` is forced to 1.0 (no additional scaling by Pathwyse)
# since our own pre-scaling already carries all the precision we need.
write_pathwyse_settings_1_0() {
  local settingsfile="$1" ng_val="$2"
  local ng_mode="off"
  [[ "$ng_val" -gt 0 ]] && ng_mode="standard"

  cat > "$settingsfile" <<SETTINGS
verbosity = 0
problem/scaling/override = 1
problem/scaling = 1.0
main_algorithm = PWDefault
algo/default/timelimit = ${TIMEOUT}.0
algo/default/bidirectional = 1
algo/default/dssr = off
algo/default/ng = ${ng_mode}
algo/default/ng/set_size = ${ng_val}
algo/default/reserve = 10000000
algo/default/use_visited = 1
algo/default/compare_unreachables = 1
data_collection/level = -1
output/write = 0
SETTINGS
}

# 1.1.0: the release's suggested settings for cyclic networks
# (docs/documentation/SUGGESTED_SETTINGS.md: PWBucket, bidirectional, bucket
# join, min_res_consumption buckets), with three changes for a pure-ng
# relaxation, the one bgspprc computes: DSSR off (the search stops at the ng
# relaxation), two-cycle elimination off, and the ng sets read from
# bgspprc's --write-ng file instead of built from set_size. PWDefault runs
# the same settings with its own ordered join (the shipped pathwyse.set).
# Costs come pre-scaled to integers (--wide-costs), so decimal_digits = 0
# keeps PathWyse from scaling them again. No time limit inside PathWyse: the
# outer `timeout` decides, so a run it cuts off is never a returned cost.
write_pathwyse_settings_1_1() {
  local settingsfile="$1" ng_file="$2"
  local join="bucket_node"
  [[ "$ALGO" == "PWDefault" ]] && join="ordered"

  cat > "$settingsfile" <<SETTINGS
verbosity = 1
problem/decimal_digits = 0
algo_names = ${ALGO}
algo_active = 1
algo_parallel = 0
algo/default/autoconfig = 1
algo/default/timelimit = 0
algo/default/search = bidirectional
algo/default/parallel = ${PARALLEL}
algo/default/reserve = 10000000
algo/default/use_visited = 1
algo/default/compare_unreachables = 1
algo/two_cycle_elimination = 0
algo/default/ng = standard
algo/default/ng/set_path = ${ng_file}
algo/default/dssr = off
algo/default/join/type = ${join}
algo/bucket/size_mode = min_res_consumption
data_collection/level = -1
output/write = 0
SETTINGS
}

# Read cost_scale from the sidecar next to a converted .txt. Hard-errors if
# the sidecar is missing — silently falling back to 1 would under-scale the
# recovered objective by up to 1000× under the per-extension defaults
# (sppcc=1, vrp=1000, graph=1000) and corrupt pathwyse.csv.
read_cost_scale() {
  local pw_file="$1"
  local scales_file="${pw_file%.txt}.scales"
  if [[ ! -f "$scales_file" ]]; then
    echo "Error: missing sidecar $scales_file (regenerate via convert_to_pathwyse.py)" >&2
    exit 1
  fi
  local val
  val="$(awk -F= '$1=="cost_scale" {print $2; exit}' "$scales_file")"
  if [[ -z "$val" ]]; then
    echo "Error: $scales_file has no cost_scale entry" >&2
    exit 1
  fi
  echo "$val"
}

# ng sets for 1.1.0, as the bgspprc-solve of this checkout builds them for the
# same --ng (and its default ng metric, cost). Written afresh by every
# invocation into its own run directory, so a change to bgspprc's ng
# construction never meets sets cached from before it. Exits on failure: a
# missing file would make PathWyse abort on every instance.
ng_set_file() {
  local file="$1" stem="$2"
  local dir="$RUN_DIR/ngsets"
  local out="$dir/${stem}.ng${NG}.csv"
  rm -f "$out"
  mkdir -p "$dir"
  if ! "$SOLVE" --write-ng "$dir" --ng "$NG" "$file" >/dev/null 2>"$dir/write-ng.err"; then
    echo "Error: $SOLVE --write-ng failed on $file:" >&2
    cat "$dir/write-ng.err" >&2
    exit 1
  fi
  if [[ ! -f "$out" ]]; then
    echo "Error: $SOLVE --write-ng wrote no $out" >&2
    exit 1
  fi
  echo "$out"
}

# ── CSV header ──
if [[ "$VERSION" == "1.0" ]]; then
  HEADER="instance,set,ng,cost,time_s,cost_scale,timestamp"
  if [[ $APPEND -eq 0 ]]; then
    echo "$HEADER" > "$OUT_CSV"
  fi
else
  HEADER="instance,set,ng,algo,parallel,status,cost,time_s,cost_scale,timestamp"
  mkdir -p "$(dirname "$OUT_CSV")"
  if [[ ! -f "$OUT_CSV" ]]; then
    echo "$HEADER" > "$OUT_CSV"
  elif [[ "$(head -1 "$OUT_CSV")" != "$HEADER" ]]; then
    echo "Error: $OUT_CSV has another header than $HEADER" >&2
    exit 1
  fi
fi

# Pathwyse reads pathwyse.set from its working directory. 1.0's settings are
# ng-dependent, not instance-dependent, so are written once into the checkout
# (concurrent 1.0 invocations would race on it). 1.1.0's name the instance's
# ng-set file, so are written per instance, into a run directory of this
# invocation's own: concurrent invocations waiting on RUN_PREFIX's lock never
# overwrite each other's settings.
if [[ "$VERSION" == "1.0" ]]; then
  RUN_DIR="$PATHWYSE_DIR"
  write_pathwyse_settings_1_0 "$RUN_DIR/pathwyse.set" "$NG"
else
  RUN_DIR="$(mktemp -d "${TMPDIR:-/tmp}/run_pathwyse.XXXXXX")"
  trap 'rm -rf "$RUN_DIR"' EXIT
fi

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

  if [[ "$VERSION" == "1.1.0" ]]; then
    ng_file="$(ng_set_file "$file" "$stem")"  # a plain assignment: set -e sees a failure
    write_pathwyse_settings_1_1 "$RUN_DIR/pathwyse.set" "$ng_file"
  fi

  pw_cost="" pw_time_s="" pw_status="OK"
  pw_raw=""
  if pw_raw=$(cd "$RUN_DIR" && "${RUN_PREFIX_ARR[@]}" timeout "${TIMEOUT}s" "$PATHWYSE_BIN" "$pw_file" 2>&1); then
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
    if [[ "$VERSION" == "1.0" ]]; then
      cost_re='Obj:[[:space:]]*(-?[0-9]+)'
      time_re='global time:[[:space:]]*([0-9.eE+-]+)'
    else
      # 1.1.0 prints `Obj : -135733.000000` (std::to_string of the scaled
      # integer cost, so never in exponent form) and `<algo> Time : 0.077926`.
      cost_re='Obj[[:space:]]*:[[:space:]]*(-?[0-9]+)(\.[0-9]*)?'
      time_re="${ALGO} Time[[:space:]]*:[[:space:]]*([0-9.eE+-]+)"
    fi
    if [[ "$pw_raw" =~ $cost_re ]]; then
      pw_cost_raw="${BASH_REMATCH[1]}"
      pw_cost="$(awk -v raw="$pw_cost_raw" -v scale="$pw_cost_scale" 'BEGIN{printf "%.6f", raw / scale}')"
    fi
    if [[ "$pw_raw" =~ $time_re ]]; then
      pw_time_s="${BASH_REMATCH[1]}"
    fi
    # 1.0 keeps its old behaviour (a half-parsed row is written as is).
    if [[ "$VERSION" == "1.1.0" && ( -z "$pw_cost" || -z "$pw_time_s" ) ]]; then
      pw_status="UNPARSED"
      pw_cost="" pw_time_s=""
    fi
  fi

  # Set label from parent directory (matches bgspprc.csv convention:
  # spprclib/roberti/ng8/ng16/ng24).
  set_label="$parent"

  ts="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

  printf "%-20s  %3d  %10s  %12s  %s  [%d/%d]\n" \
    "$stem" "$NG" "${pw_time_s:--}" "${pw_cost:--}" "$pw_status" "$IDX" "$TOTAL"

  if [[ "$VERSION" == "1.0" ]]; then
    echo "${stem},${set_label},${NG},${pw_cost},${pw_time_s},${pw_cost_scale},${ts}" >> "$OUT_CSV"
  else
    # Replace an earlier row of the same configuration; keep every other.
    tmp="$(mktemp)"
    awk -F, -v i="$stem" -v s="$set_label" -v n="$NG" -v a="$ALGO" -v p="$PARALLEL" \
      'NR==1 || !($1==i && $2==s && $3==n && $4==a && $5==p)' "$OUT_CSV" > "$tmp"
    mv "$tmp" "$OUT_CSV"
    echo "${stem},${set_label},${NG},${ALGO},${PARALLEL},${pw_status},${pw_cost},${pw_time_s},${pw_cost_scale},${ts}" >> "$OUT_CSV"
  fi

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
