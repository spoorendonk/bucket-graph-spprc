#!/usr/bin/env bash
# regen_all_modes.sh — Regenerate bgspprc.csv over the 6-mode × 3-ng grid.
#
# Driver for the full 2376-row regeneration from issue #91. Calls
# run_benchmarks.sh sequentially for each (mode, ng) pair. The per-ng rcspp
# directory is passed explicitly because run_benchmarks.sh's default paths
# are spprclib+roberti only.
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"

MODES=(mono_base mono_vec bidir_base bidir_vec para_bidir_base para_bidir)
NGS=(8 16 24)

TIMEOUT="${TIMEOUT:-120}"

for mode in "${MODES[@]}"; do
  for ng in "${NGS[@]}"; do
    echo "=================================================="
    echo "mode=$mode  ng=$ng"
    echo "=================================================="
    "$SCRIPTDIR/run_benchmarks.sh" --mode "$mode" --ng "$ng" --timeout "$TIMEOUT" \
      "$SCRIPTDIR/instances/spprclib" \
      "$SCRIPTDIR/instances/roberti" \
      "$SCRIPTDIR/instances/rcspp/ng${ng}"
  done
done

echo
echo "Regen complete. Rows: $(($(wc -l < "$SCRIPTDIR/bgspprc.csv") - 1))"
