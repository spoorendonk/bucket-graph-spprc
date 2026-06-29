#!/usr/bin/env python3
"""Regenerate the reference ng-optima from the bgspprc.csv 6-mode consensus.

For each spprclib/roberti instance and ng size, the optimal reduced cost is the
value agreed by every solver mode that finished within the timeout (all 6 modes
must agree — verify first with check_consistency.py). This writes one
`optimal-ng<ng>.csv` (`instance,optimal`) per set under
`benchmarks/instances/<set>/`, replacing the previous ad-hoc process (see commit
d2c2beb). Instances no mode solved are omitted (check_optimal.sh then SKIPs them).

rcspp `.graph` instances are not handled here — their optimum is always 0
(check_optimal.sh treats `set` matching `ng[0-9]+` specially).

Usage:
    python3 benchmarks/gen_optimal_ng.py            # regenerate the files
    python3 benchmarks/gen_optimal_ng.py --check    # verify files are up to date (exit 1 if not)
"""
from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SETS = ["spprclib", "roberti"]
NGS = ["8", "16", "24"]
# cost is written to 3 decimals in bgspprc.csv; agree on that quantum.
QUANT = 3


def consensus(bg_path: Path) -> dict[tuple[str, str], dict[str, str]]:
    """(set, ng) -> {instance: optimal} from the agreed cost across solved modes.

    Raises on a genuine cross-mode disagreement (a correctness red flag —
    check_consistency.py covers this, but we re-assert it here)."""
    seen: dict[tuple[str, str, str], set[float]] = defaultdict(set)
    for r in csv.DictReader(open(bg_path)):
        if r["set"] in SETS and r["cost"]:
            seen[(r["set"], r["ng"], r["instance"])].add(round(float(r["cost"]), QUANT))

    out: dict[tuple[str, str], dict[str, str]] = defaultdict(dict)
    conflicts = []
    for (s, ng, inst), vals in seen.items():
        if len(vals) > 1:
            conflicts.append((s, ng, inst, sorted(vals)))
            continue
        out[(s, ng)][inst] = f"{vals.pop():.{QUANT}f}"
    if conflicts:
        for s, ng, inst, vals in conflicts:
            print(f"CONFLICT {s} ng{ng} {inst}: modes disagree {vals}", file=sys.stderr)
        raise SystemExit(
            f"{len(conflicts)} cross-mode cost conflict(s); run check_consistency.py"
        )
    return out


def render(rows: dict[str, str]) -> str:
    out = ["instance,optimal"]
    out += [f"{inst},{rows[inst]}" for inst in sorted(rows)]
    return "\n".join(out) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    ap.add_argument("--bg", default=str(ROOT / "bgspprc.csv"))
    ap.add_argument("--check", action="store_true",
                    help="verify on-disk files match the consensus; exit 1 if not")
    args = ap.parse_args()

    cons = consensus(Path(args.bg))
    stale = False
    for s in SETS:
        for ng in NGS:
            f = ROOT / "instances" / s / f"optimal-ng{ng}.csv"
            content = render(cons.get((s, ng), {}))
            if args.check:
                cur = f.read_text() if f.exists() else ""
                status = "ok" if cur == content else "STALE"
                if cur != content:
                    stale = True
                print(f"  {s:9} ng{ng:<2} {len(cons.get((s, ng), {})):2} rows  {status}")
            else:
                f.write_text(content)
                print(f"  wrote {f.relative_to(ROOT.parent)}  ({len(cons.get((s, ng), {}))} rows)")
    if args.check and stale:
        print("optimal-ng files are stale — run gen_optimal_ng.py", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
