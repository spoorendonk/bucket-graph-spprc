#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: compare_mono_bidir.sh [--ng K] [--timeout S] [PATH...]
  PATH    Instance file or directory
  --ng K      Pass --ng K to solver
  --timeout S Per-instance timeout in seconds (default: 120)
EOF
  exit 1
}

SOLVE="${SOLVE:-./build/bgspprc-solve}"
NG_FLAG=()
TIMEOUT=120
PATHS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ng)      NG_FLAG=(--ng "$2"); shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    -h|--help) usage ;;
    *)         PATHS+=("$1"); shift ;;
  esac
done

if [[ ${#PATHS[@]} -eq 0 ]]; then
  echo "Error: at least one PATH required" >&2
  usage
fi

# ── Collect instance files ──
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

FILES=()
while IFS= read -r f; do
  [[ -n "$f" ]] && FILES+=("$f")
done < <(collect_files "${PATHS[@]}")

if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "No instance files found." >&2
  exit 1
fi

# ── Parse solver output ──
parse_result() {
  local output="$1" status="$2"
  local cost="-" time_ms="-"
  if [[ "$status" == "OK" && -n "$output" ]]; then
    local line
    line="$(echo "$output" | head -1)"
    if [[ "$line" =~ cost=([0-9.eE+-]+) ]]; then cost="${BASH_REMATCH[1]}"; fi
    if [[ "$line" =~ ([0-9.]+)ms ]]; then time_ms="${BASH_REMATCH[1]}"; fi
  elif [[ "$status" != "OK" ]]; then
    cost="$status"
  fi
  echo "$cost $time_ms"
}

# ── Run ──
printf "%-30s  %12s  %10s  %12s  %10s\n" "Instance" "Mono Cost" "Mono ms" "Bidir Cost" "Bidir ms"
printf '%.0s-' {1..80}; echo

for file in "${FILES[@]}"; do
  stem="$(basename "$file")"
  stem="${stem%.*}"

  # Mono run
  mono_out="" mono_status="OK"
  if mono_out=$(timeout "${TIMEOUT}s" "$SOLVE" --mono "${NG_FLAG[@]}" "$file" 2>&1); then :
  else
    rc=$?
    if [[ $rc -eq 124 ]]; then mono_status="TIMEOUT"; else mono_status="ERR($rc)"; fi
  fi
  read -r mono_cost mono_ms <<< "$(parse_result "$mono_out" "$mono_status")"

  # Bidir run
  bidir_out="" bidir_status="OK"
  if bidir_out=$(timeout "${TIMEOUT}s" "$SOLVE" "${NG_FLAG[@]}" "$file" 2>&1); then :
  else
    rc=$?
    if [[ $rc -eq 124 ]]; then bidir_status="TIMEOUT"; else bidir_status="ERR($rc)"; fi
  fi
  read -r bidir_cost bidir_ms <<< "$(parse_result "$bidir_out" "$bidir_status")"

  printf "%-30s  %12s  %10s  %12s  %10s\n" "$stem" "$mono_cost" "$mono_ms" "$bidir_cost" "$bidir_ms"
done
