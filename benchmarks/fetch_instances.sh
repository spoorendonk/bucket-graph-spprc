#!/usr/bin/env bash
set -euo pipefail

# Download benchmark instances into benchmarks/instances/ at project root.
# Requires: curl, unzip

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DATA_DIR="$ROOT_DIR/benchmarks/instances"

mkdir -p "$DATA_DIR"

# ── RCSPP (ng8/ng16/ng24 .graph files) ──
# The rcspp_dataset repo contains a rcspp.zip with rcspp/{ng8,ng16,ng24}/

if [ -d "$DATA_DIR/rcspp/ng8" ] && [ -d "$DATA_DIR/rcspp/ng16" ] && [ -d "$DATA_DIR/rcspp/ng24" ]; then
    echo "rcspp: already present, skipping"
else
    echo "rcspp: downloading..."
    TMP="$(mktemp -d)"
    trap 'rm -rf "$TMP"' EXIT
    curl -sL "https://raw.githubusercontent.com/spoorendonk/rcspp_dataset/main/rcspp.zip" \
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

if [ -d "$DATA_DIR/spprclib" ] && [ -d "$DATA_DIR/roberti" ]; then
    echo "spprclib + roberti: already present, skipping"
else
    echo "spprclib + roberti: downloading..."
    TMP="$(mktemp -d)"
    trap 'rm -rf "$TMP"' EXIT
    curl -sL "https://github.com/spoorendonk/cptp/archive/refs/heads/main.tar.gz" \
        -o "$TMP/cptp.tar.gz"
    tar xzf "$TMP/cptp.tar.gz" -C "$TMP"
    cp -r "$TMP"/cptp-main/benchmarks/instances/spprclib "$DATA_DIR/"
    cp -r "$TMP"/cptp-main/benchmarks/instances/roberti "$DATA_DIR/"
    rm -rf "$TMP"
    trap - EXIT
    echo "spprclib + roberti: done"
fi

echo "All instances in $DATA_DIR"
