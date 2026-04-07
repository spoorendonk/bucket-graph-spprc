#!/usr/bin/env bash
# 09_simd.sh — SIMD impact ablation.
#
# Compares solver performance with and without SIMD/march=native.
# Builds two separate binaries:
#   build-simd/bgspprc-solve    — with -march=native (default)
#   build-nosimd/bgspprc-solve  — with BGSPPRC_DISABLE_SIMD=ON, no -march=native
#
# Usage:
#   ./benchmarks/ablation/09_simd.sh [--timeout S] [--skip-build] [PATH...]
#
# Output:
#   CSV to stdout: instance,set,simd,cost,paths,time_ms,status
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPTDIR/common.sh"

TIMEOUT="${DEFAULT_TIMEOUT}"
PATHS=()
SKIP_BUILD=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout)    TIMEOUT="$2"; shift 2 ;;
    --skip-build) SKIP_BUILD=true; shift ;;
    -h|--help)    sed -n '2,/^$/s/^# \?//p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)            PATHS+=("$1"); shift ;;
  esac
done

BUILD_SIMD="$REPODIR/build-simd"
BUILD_NOSIMD="$REPODIR/build-nosimd"
SOLVE_SIMD="$BUILD_SIMD/bgspprc-solve"
SOLVE_NOSIMD="$BUILD_NOSIMD/bgspprc-solve"

if [[ "$SKIP_BUILD" == false ]]; then
  echo "Building with SIMD..." >&2
  cmake -B "$BUILD_SIMD" -DCMAKE_CXX_COMPILER=g++-14 "$REPODIR" >&2
  cmake --build "$BUILD_SIMD" --target bgspprc-solve >&2

  echo "Building without SIMD..." >&2
  cmake -B "$BUILD_NOSIMD" -DCMAKE_CXX_COMPILER=g++-14 \
    -DBGSPPRC_DISABLE_SIMD=ON \
    -DCMAKE_CXX_FLAGS="-mno-avx -mno-avx2 -mno-sse4.2" \
    "$REPODIR" >&2
  cmake --build "$BUILD_NOSIMD" --target bgspprc-solve >&2
fi

for bin in "$SOLVE_SIMD" "$SOLVE_NOSIMD"; do
  if [[ ! -x "$bin" ]]; then
    echo "Error: binary not found at $bin" >&2
    echo "Run without --skip-build to build both variants." >&2
    exit 1
  fi
done

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

echo "instance,set,simd,cost,paths,time_ms,status"

for file in "${FILES[@]}"; do
  stem="$(instance_stem "$file")"
  iset="$(instance_set "$file")"

  # With SIMD
  saved_solve="$SOLVE"
  SOLVE="$SOLVE_SIMD"
  run_solver "$TIMEOUT" --stage exact -- "$file"
  parse_cost_time "$OUT" "$STATUS"
  parse_paths "$OUT"
  echo "${stem},${iset},on,${COST},${PATHS_COUNT},${TIME_MS},${STATUS}"

  # Without SIMD
  SOLVE="$SOLVE_NOSIMD"
  run_solver "$TIMEOUT" --stage exact -- "$file"
  parse_cost_time "$OUT" "$STATUS"
  parse_paths "$OUT"
  echo "${stem},${iset},off,${COST},${PATHS_COUNT},${TIME_MS},${STATUS}"
  SOLVE="$saved_solve"
done
