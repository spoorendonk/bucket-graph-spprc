#!/usr/bin/env bash
# common.sh — Shared helpers for ablation study scripts.
# Source this file; do not execute directly.

SCRIPTDIR="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)"
REPODIR="$(cd "$SCRIPTDIR/../.." && pwd)"
SOLVE="${SOLVE:-$REPODIR/build/bgspprc-solve}"
INSTDIR="${INSTDIR:-$REPODIR/benchmarks/instances}"
DEFAULT_TIMEOUT=120

# ── Collect instance files from paths ──
# Usage: collect_files PATH...
# Prints one file path per line.
collect_files() {
  for p in "$@"; do
    if [[ -f "$p" ]]; then
      echo "$p"
    elif [[ -d "$p" ]]; then
      find "$p" -maxdepth 2 -type f \( -name '*.sppcc' -o -name '*.vrp' -o -name '*.graph' \) | sort
    else
      echo "Warning: $p not found, skipping" >&2
    fi
  done
}

# ── Default instance paths ──
# Returns default paths if none were given on the command line.
default_instance_paths() {
  local paths=()
  for d in "$INSTDIR/spprclib" "$INSTDIR/roberti"; do
    [[ -d "$d" ]] && paths+=("$d")
  done
  if [[ ${#paths[@]} -eq 0 ]]; then
    echo "No default instance directories found in $INSTDIR" >&2
    exit 1
  fi
  printf '%s\n' "${paths[@]}"
}

# ── Run solver and capture output ──
# Usage: run_solver TIMEOUT EXTRA_FLAGS... -- FILE
# Sets: OUT (stdout+stderr), STATUS (OK|TIMEOUT|ERROR(N))
# Always passes --stats --timing for richer metrics.
run_solver() {
  local timeout_s="$1"; shift
  local args=(--stats --timing)
  while [[ $# -gt 0 && "$1" != "--" ]]; do
    args+=("$1"); shift
  done
  [[ "${1:-}" == "--" ]] && shift
  local file="$1"

  OUT=""
  STATUS="OK"
  if OUT=$(timeout "${timeout_s}s" "$SOLVE" "${args[@]}" "$file" 2>&1); then
    :
  else
    local rc=$?
    if [[ $rc -eq 124 ]]; then
      STATUS="TIMEOUT"
    else
      STATUS="ERROR($rc)"
    fi
  fi
}

# ── Parse solver output ──
# Usage: parse_cost_time "$OUT" "$STATUS"
# Sets: COST, TIME_MS
parse_cost_time() {
  local output="$1" status="$2"
  COST="" TIME_MS=""
  if [[ "$status" == "OK" && -n "$output" ]]; then
    local line
    line="$(echo "$output" | head -1)"
    if [[ "$line" =~ cost=([0-9.eE+-]+) ]]; then COST="${BASH_REMATCH[1]}"; fi
    if [[ "$line" =~ ([0-9.]+)ms ]]; then TIME_MS="${BASH_REMATCH[1]}"; fi
  fi
}

# ── Parse paths count from solver output ──
# Usage: parse_paths "$OUT"
# Sets: PATHS_COUNT
parse_paths() {
  local output="$1"
  PATHS_COUNT=""
  if [[ -n "$output" ]]; then
    local line
    line="$(echo "$output" | head -1)"
    if [[ "$line" =~ paths=([0-9]+) ]]; then PATHS_COUNT="${BASH_REMATCH[1]}"; fi
  fi
}

# ── Parse solve statistics from --stats output ──
# Usage: parse_stats "$OUT"
# Sets: N_BUCKETS, N_LABELS_CREATED, N_DOMINANCE_CHECKS, N_NON_DOMINATED,
#       N_FIXED_BUCKETS, N_ELIMINATED_ARCS, LABEL_STATE_BYTES
parse_stats() {
  local output="$1"
  N_BUCKETS="" N_LABELS_CREATED="" N_DOMINANCE_CHECKS="" N_NON_DOMINATED=""
  N_FIXED_BUCKETS="" N_ELIMINATED_ARCS="" LABEL_STATE_BYTES=""
  if [[ -n "$output" ]]; then
    if [[ "$output" =~ n_buckets=([0-9]+) ]]; then N_BUCKETS="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ n_labels_created=([0-9]+) ]]; then N_LABELS_CREATED="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ n_dominance_checks=([0-9]+) ]]; then N_DOMINANCE_CHECKS="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ n_non_dominated=([0-9]+) ]]; then N_NON_DOMINATED="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ n_fixed_buckets=([0-9]+) ]]; then N_FIXED_BUCKETS="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ n_eliminated_arcs=([0-9]+) ]]; then N_ELIMINATED_ARCS="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ label_state_bytes=([0-9]+) ]]; then LABEL_STATE_BYTES="${BASH_REMATCH[1]}"; fi
  fi
}

# ── Parse phase timing from --timing output ──
# Usage: parse_timing "$OUT"
# Sets: FW_MS, BW_MS, COMPLETION_MS, CONCAT_MS, PATHS_MS, SUM_MS
parse_timing() {
  local output="$1"
  FW_MS="" BW_MS="" COMPLETION_MS="" CONCAT_MS="" PATHS_MS="" SUM_MS=""
  if [[ -n "$output" ]]; then
    if [[ "$output" =~ fw=([0-9.]+)ms ]]; then FW_MS="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ bw=([0-9.]+)ms ]]; then BW_MS="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ completion=([0-9.]+)ms ]]; then COMPLETION_MS="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ concat=([0-9.]+)ms ]]; then CONCAT_MS="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ paths=([0-9.]+)ms ]]; then PATHS_MS="${BASH_REMATCH[1]}"; fi
    if [[ "$output" =~ sum=([0-9.]+)ms ]]; then SUM_MS="${BASH_REMATCH[1]}"; fi
  fi
}

# ── Instance stem ──
instance_stem() {
  local f="$1"
  local stem
  stem="$(basename "$f")"
  echo "${stem%.*}"
}

# ── Instance set ──
instance_set() {
  basename "$(dirname "$1")"
}

# ── Check solver binary exists ──
check_solver() {
  if [[ ! -x "$SOLVE" ]]; then
    echo "Error: solver not found at $SOLVE" >&2
    echo "Build first: cmake -B build -DCMAKE_CXX_COMPILER=g++-14 && cmake --build build" >&2
    exit 1
  fi
}
