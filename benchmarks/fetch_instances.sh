#!/usr/bin/env bash
set -euo pipefail

# Download benchmark instances into benchmarks/instances/ at project root.
# Requires: curl, unzip

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DATA_DIR="$ROOT_DIR/benchmarks/instances"

mkdir -p "$DATA_DIR"

has_files() { compgen -G "$1" >/dev/null; }

# ── RCSPP (ng8/ng16/ng24 .graph files) ──
# The rcspp_dataset repo contains a rcspp.zip with rcspp/{ng8,ng16,ng24}/

if has_files "$DATA_DIR/rcspp/ng8/*.graph" && has_files "$DATA_DIR/rcspp/ng16/*.graph" &&
    has_files "$DATA_DIR/rcspp/ng24/*.graph"; then
    echo "rcspp: already present, skipping"
else
    echo "rcspp: downloading..."
    TMP="$(mktemp -d)"
    trap 'rm -rf "$TMP"' EXIT
    curl -sfL "https://raw.githubusercontent.com/spoorendonk/rcspp_dataset/main/rcspp.zip" \
        -o "$TMP/rcspp.zip"
    unzip -q "$TMP/rcspp.zip" -d "$TMP"
    mkdir -p "$DATA_DIR/rcspp"
    cp -r "$TMP"/rcspp/ng8 "$DATA_DIR/rcspp/"
    cp -r "$TMP"/rcspp/ng16 "$DATA_DIR/rcspp/"
    cp -r "$TMP"/rcspp/ng24 "$DATA_DIR/rcspp/"
    rm -rf "$TMP"
    trap - EXIT
    echo "rcspp: done"
fi

# ── SPPRCLIB + Roberti (from cptp repo) ──
# These dirs are tracked in git for their optimal*.csv files, so they exist on a
# clean clone: test for the instance files themselves, not the directories. Copy
# only the instance files, so the committed CSVs are never overwritten by
# cptp's copies (its roberti/optimal.csv differs from ours).

if has_files "$DATA_DIR/spprclib/*.sppcc" && has_files "$DATA_DIR/roberti/*.vrp"; then
    echo "spprclib + roberti: already present, skipping"
else
    echo "spprclib + roberti: downloading..."
    TMP="$(mktemp -d)"
    trap 'rm -rf "$TMP"' EXIT
    curl -sfL "https://github.com/spoorendonk/cptp/archive/refs/heads/main.tar.gz" \
        -o "$TMP/cptp.tar.gz"
    tar xzf "$TMP/cptp.tar.gz" -C "$TMP"
    SRC="$TMP/cptp-main/benchmarks/instances"
    mkdir -p "$DATA_DIR/spprclib" "$DATA_DIR/roberti"
    cp "$SRC"/spprclib/*.sppcc "$DATA_DIR/spprclib/"
    cp "$SRC"/roberti/*.vrp "$DATA_DIR/roberti/"
    rm -rf "$TMP"
    trap - EXIT
    echo "spprclib + roberti: done"
fi

echo "All instances in $DATA_DIR"
