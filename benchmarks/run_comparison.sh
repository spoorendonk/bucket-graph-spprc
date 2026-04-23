#!/usr/bin/env bash
# run_comparison.sh — Compare bgspprc para_bidir vs paper `all_s` on rcspp instances.
#
# Pure CSV join over existing data:
#   - bgspprc times: `bgspprc.csv`, mode=para_bidir, set∈{ng8,ng16,ng24}
#   - paper times:   `pull_algo_runtimes.csv`, column `all_s` (all optimizations)
#
# Timeouts on either side are shown in the output as "TL" and substituted with
# TIMEOUT seconds when computing the shifted geometric mean — a conservative
# bound (actual time is ≥ TIMEOUT). Both sides use the same timeout budget, so
# the ratio is apples-to-apples.
#
# Usage:
#   ./benchmarks/run_comparison.sh [--ng 8|16|24] [--timeout S]
#
# Does NOT run the solver. Refresh bgspprc.csv with run_benchmarks.sh first
# if you want updated bgspprc times. --timeout should match the timeout that
# was used to generate bgspprc.csv — substituting a different value for TL
# bgspprc rows would misreport their runtime.
set -euo pipefail

SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"
BGSPPRC_CSV="$SCRIPTDIR/bgspprc.csv"
PAPER_CSV="$SCRIPTDIR/pull_algo_runtimes.csv"
OUT_CSV="$SCRIPTDIR/comparison_rcspp.csv"
TIMEOUT=120
NG_GROUPS=(8 16 24)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ng)      NG_GROUPS=("$2"); shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,/^$/s/^# \?//p' "${BASH_SOURCE[0]}"
      exit 0 ;;
    *) echo "Unknown arg: $1" >&2; exit 1 ;;
  esac
done

if [[ ! -f "$BGSPPRC_CSV" ]]; then
  echo "Error: $BGSPPRC_CSV not found — run run_benchmarks.sh first" >&2
  exit 1
fi
if [[ ! -f "$PAPER_CSV" ]]; then
  echo "Error: $PAPER_CSV not found" >&2
  exit 1
fi

BGSPPRC_CSV="$BGSPPRC_CSV" PAPER_CSV="$PAPER_CSV" OUT_CSV="$OUT_CSV" \
TIMEOUT="$TIMEOUT" NG_GROUPS="${NG_GROUPS[*]}" \
python3 <<'PY'
import csv, math, os

bgspprc_csv = os.environ['BGSPPRC_CSV']
paper_csv   = os.environ['PAPER_CSV']
out_csv     = os.environ['OUT_CSV']
timeout     = float(os.environ['TIMEOUT'])
ng_groups   = [int(x) for x in os.environ['NG_GROUPS'].split()]
SHIFT = 10

# Paper all_s — mark >timeout as TL (None).
paper = {}  # (inst, ng) -> float or None (TL)
with open(paper_csv) as f:
    for r in csv.DictReader(f):
        ng = int(r['ng'])
        if ng not in ng_groups:
            continue
        try:
            t = float(r['all_s'])
        except (TypeError, ValueError):
            continue
        paper[(r['instance'], ng)] = None if t > timeout else t

# bgspprc para_bidir rows on rcspp sets (set prefix 'ng') — empty cost = TL (None).
bg = {}  # (inst, ng) -> float or None (TL)
with open(bgspprc_csv) as f:
    for r in csv.DictReader(f):
        if r['mode'] != 'para_bidir' or not r['set'].startswith('ng'):
            continue
        try:
            ng = int(r['ng'])
        except ValueError:
            continue
        if ng not in ng_groups:
            continue
        key = (r['instance'], ng)
        if not r['cost'] or not r['time_s']:
            bg[key] = None
        else:
            bg[key] = float(r['time_s'])

# Join — only instances present in both CSVs.
rows = []
for key in sorted(set(bg) & set(paper), key=lambda k: (k[1], k[0])):
    bg_v, paper_v = bg[key], paper[key]
    bg_sgm    = timeout if bg_v    is None else bg_v
    paper_sgm = timeout if paper_v is None else paper_v
    ratio = bg_sgm / paper_sgm
    rows.append((key[0], key[1], bg_v, paper_v, bg_sgm, paper_sgm, ratio))

def fmt(v):
    return 'TL' if v is None else f'{v:.3f}'

with open(out_csv, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['instance', 'ng', 'bgspprc_s', 'paper_all_s', 'ratio'])
    for inst, ng, bg_v, paper_v, _, _, ratio in rows:
        w.writerow([inst, ng, fmt(bg_v), fmt(paper_v), f'{ratio:.2f}'])

print()
print(f'{"Instance":<12} {"ng":>3} {"bgspprc(s)":>12} {"paper(s)":>10} {"ratio":>8}')
print('─' * 52)
for inst, ng, bg_v, paper_v, _, _, ratio in rows:
    print(f'{inst:<12} {ng:>3} {fmt(bg_v):>12} {fmt(paper_v):>10} {ratio:>8.2f}')

print()
print('─' * 52)
print(f'Shifted geometric mean (shift={SHIFT}s, TL substituted with {timeout:g}s):')
print()
print(f'{"Group":<10} {"bgspprc(s)":>12} {"paper(s)":>10} {"ratio":>8} {"n":>5} {"bg_TL":>6} {"p_TL":>5}')
print('─' * 66)
bg_tl_total = paper_tl_total = 0
for ng in ng_groups:
    grp = [r for r in rows if r[1] == ng]
    n = len(grp)
    if not n:
        print(f'{"ng"+str(ng):<10} {"-":>12} {"-":>10} {"-":>8} {n:>5}')
        continue
    bg_tl    = sum(1 for r in grp if r[2] is None)
    paper_tl = sum(1 for r in grp if r[3] is None)
    bg_tl_total    += bg_tl
    paper_tl_total += paper_tl
    go = math.exp(sum(math.log(r[4] + SHIFT) for r in grp) / n) - SHIFT
    gp = math.exp(sum(math.log(r[5] + SHIFT) for r in grp) / n) - SHIFT
    gr = (go + SHIFT) / (gp + SHIFT)
    print(f'{"ng"+str(ng):<10} {go:>12.3f} {gp:>10.3f} {gr:>8.2f} {n:>5} {bg_tl:>6} {paper_tl:>5}')

print()
print(f'Results written to {out_csv}')
print(f'Timeouts: bgspprc={bg_tl_total}, paper={paper_tl_total} (both substituted with {timeout:g}s in SGM)')
PY
