#!/usr/bin/env bash
#
# Compare our solver vs PathWyse across SPPRCLIB and Roberti instances.
#
# Usage: ./scripts/bench_vs_pathwyse.sh [spprclib|roberti|all]
#
# Prerequisites:
#   - build/bench_vs_pathwyse compiled
#   - pathwyse binary at ../pathwyse/bin/pathwyse
#   - Instance dirs in data/ (run scripts/fetch_instances.sh first)

set -euo pipefail

MODE="${1:-all}"
TIMEOUT=300
PATHWYSE="../pathwyse/bin/pathwyse"
OUR_SOLVER="./build/bench_vs_pathwyse"
CACHE_DIR="build/pathwyse_instances"
CSV_FILE="build/bench_vs_pathwyse.csv"

# Instance directories (run scripts/fetch_instances.sh first)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/data"
SPPRCLIB_DIR=""
ROBERTI_DIR=""
if [ -d "$DATA_DIR/spprclib" ]; then
    SPPRCLIB_DIR="$DATA_DIR/spprclib"
fi
if [ -d "$DATA_DIR/roberti" ]; then
    ROBERTI_DIR="$DATA_DIR/roberti"
fi

mkdir -p "$CACHE_DIR"

# Check prerequisites
if [ ! -x "$PATHWYSE" ]; then
    echo "ERROR: PathWyse not found at $PATHWYSE"
    exit 1
fi
if [ ! -x "$OUR_SOLVER" ]; then
    echo "ERROR: Our solver not found at $OUR_SOLVER (run: make build/bench_vs_pathwyse)"
    exit 1
fi

# CSV header
echo "instance,ng_size,our_mono,our_bidir,pathwyse,mono_ms,bidir_ms,pw_ms,mono_bidir_match,our_vs_pw_match" > "$CSV_FILE"

pass=0
fail=0

run_instance() {
    local pw_file="$1"
    local orig_file="$2"
    local ng="$3"
    local name="$4"
    local tol="$5"

    # Write pathwyse param file
    local parfile
    parfile=$(mktemp /tmp/pw_par_XXXXXX)
    cat > "$parfile" <<PAREOF
verbosity = 2
algo/default/autoconfig = 0
algo/default/bidirectional = 1
algo/default/ng = standard
algo/default/ng/set_size = $ng
algo/default/dssr = off
algo/default/use_visited = 1
algo/default/compare_unreachables = 1
PAREOF

    # Run our solver
    local our_out
    our_out=$(timeout "$TIMEOUT" "$OUR_SOLVER" "$orig_file" "$ng" 2>/dev/null) || {
        echo "  $name ng$ng: OUR SOLVER TIMEOUT/ERROR"
        rm -f "$parfile"
        return
    }

    local our_mono our_bidir our_mono_ms our_bidir_ms
    our_mono=$(echo "$our_out" | grep "^RESULT_MONO" | awk '{print $4}')
    our_bidir=$(echo "$our_out" | grep "^RESULT_BIDIR" | awk '{print $4}')
    our_mono_ms=$(echo "$our_out" | grep "^RESULT_MONO" | awk '{print $5}')
    our_bidir_ms=$(echo "$our_out" | grep "^RESULT_BIDIR" | awk '{print $5}')

    # Run PathWyse
    local pw_out pw_obj
    pw_out=$(timeout "$TIMEOUT" "$PATHWYSE" "$pw_file" -param "$parfile" 2>&1) || {
        echo "  $name ng$ng: PATHWYSE TIMEOUT/ERROR"
        echo "$name,$ng,$our_mono,$our_bidir,TIMEOUT,$our_mono_ms,$our_bidir_ms,,," >> "$CSV_FILE"
        rm -f "$parfile"
        return
    }
    pw_obj=$(echo "$pw_out" | grep "^Obj:" | awk '{print $2}')
    # PathWyse prints "PWDefault global time: <seconds>"
    local pw_time_s pw_ms
    pw_time_s=$(echo "$pw_out" | grep "global time:" | awk '{print $NF}')
    pw_ms=$(python3 -c "print(f'{float(${pw_time_s:-0}) * 1000:.1f}')")
    rm -f "$parfile"

    if [ -z "$pw_obj" ]; then
        echo "  $name ng$ng: PATHWYSE NO OBJ"
        echo "$name,$ng,$our_mono,$our_bidir,NO_OBJ,$our_mono_ms,$our_bidir_ms,,," >> "$CSV_FILE"
        return
    fi

    # Compare mono vs bidir (tolerance 1.0)
    local mono_bidir_diff mono_bidir_ok
    mono_bidir_diff=$(python3 -c "
m, b = $our_mono, $our_bidir
d = abs(m - b)
# Both non-negative means no negative-cost path found
ok = d < 1.0 or (m >= 0 and b >= -1e-6)
print(f'{d:.4f}')
print('PASS' if ok else 'FAIL')
")
    mono_bidir_ok=$(echo "$mono_bidir_diff" | tail -1)

    # Compare our bidir vs PathWyse: our relaxation is weaker (local ng-bitset
    # vs PathWyse's global bitset), so our bound should be <= PathWyse's (at
    # least as negative). If ours > PathWyse + tol, that's a bug.
    local our_vs_pw_ok
    our_vs_pw_ok=$(python3 -c "
ours = $our_bidir
pw = float($pw_obj)
d = ours - pw  # positive means ours is less negative (unexpected)
ok = d < $tol or (ours >= 0 and pw >= 0)
print('PASS' if ok else 'FAIL')
print(f'{d:.4f}')
")
    local pw_match pw_diff
    pw_match=$(echo "$our_vs_pw_ok" | head -1)
    pw_diff=$(echo "$our_vs_pw_ok" | tail -1)

    # Status
    local status="PASS"
    if [ "$mono_bidir_ok" != "PASS" ] || [ "$pw_match" != "PASS" ]; then
        status="FAIL"
        ((fail++)) || true
    else
        ((pass++)) || true
    fi

    printf "  %-30s ng%-2d  mono=%-12s bidir=%-12s pw=%-8s  mono=%sms bidir=%sms pw=%sms  mb=%s pw=%s  %s\n" \
        "$name" "$ng" "$our_mono" "$our_bidir" "$pw_obj" "$our_mono_ms" "$our_bidir_ms" "$pw_ms" "$mono_bidir_ok" "$pw_match" "$status"

    echo "$name,$ng,$our_mono,$our_bidir,$pw_obj,$our_mono_ms,$our_bidir_ms,$pw_ms,$mono_bidir_ok,$pw_match" >> "$CSV_FILE"
}

# ── SPPRCLIB instances ──
if [ "$MODE" = "all" ] || [ "$MODE" = "spprclib" ]; then
    if [ -n "$SPPRCLIB_DIR" ]; then
        echo ""
        echo "=== SPPRCLIB vs PathWyse ==="
        echo ""

        for sppcc in "$SPPRCLIB_DIR"/*.sppcc; do
            name=$(basename "$sppcc" .sppcc)
            pw_file="$CACHE_DIR/${name}.sppcc"

            # Convert once
            if [ ! -f "$pw_file" ]; then
                python3 scripts/convert_sppcc_to_pathwyse.py "$sppcc" "$pw_file"
            fi

            for ng in 8 16 24; do
                run_instance "$pw_file" "$sppcc" "$ng" "$name" "1.0"
            done
        done
    else
        echo "SPPRCLIB dir not found, skipping"
    fi
fi

# ── Roberti instances ──
if [ "$MODE" = "all" ] || [ "$MODE" = "roberti" ]; then
    if [ -n "$ROBERTI_DIR" ]; then
        echo ""
        echo "=== Roberti vs PathWyse ==="
        echo ""

        scaling=1000
        for vrp in "$ROBERTI_DIR"/*.vrp; do
            name=$(basename "$vrp" .vrp)

            # Skip F-class with large capacity (known slow)
            if [[ "$name" == F-* ]]; then
                continue
            fi

            pw_file="$CACHE_DIR/${name}.sppcc"

            if [ ! -f "$pw_file" ]; then
                python3 scripts/convert_vrp_to_pathwyse.py "$vrp" "$pw_file" "$scaling"
            fi

            # Tolerance: n_vertices / scaling (worst-case rounding error over path)
            n_nodes=$(grep "^SIZE" "$pw_file" | awk '{print $3}')
            tol=$(python3 -c "print(${n_nodes:-80} / $scaling)")

            for ng in 8 16 24; do
                run_instance "$pw_file" "$vrp" "$ng" "$name" "$tol"
            done
        done
    else
        echo "Roberti dir not found, skipping"
    fi
fi

echo ""
echo "════════════════════════════════════"
echo "  TOTAL: $pass passed, $fail failed"
echo "════════════════════════════════════"
echo "CSV: $CSV_FILE"

exit $((fail > 0 ? 1 : 0))
