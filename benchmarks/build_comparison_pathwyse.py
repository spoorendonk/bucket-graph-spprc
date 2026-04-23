#!/usr/bin/env python3
"""Build benchmarks/comparison_pathwyse.csv from bgspprc.csv + pathwyse.csv.

Inner-joins on (instance, ng). Filters to sppcc/vrp instances (sets
`spprclib` and `roberti`); rcspp `.graph` instances are excluded because
bgspprc's auto-default ng-metric is `cost` on .graph while Pathwyse builds
ng neighborhoods from distance — the ng-relaxations differ and the rows
aren't strictly apples-to-apples.

The bgspprc side picks `mode=para_bidir` (consistently fastest across
ng=16/24 per bgspprc.csv analysis; mono wins at ng=8 only).

Usage:
    python3 benchmarks/build_comparison_pathwyse.py
"""
import argparse
import csv
import math
import sys
from collections import Counter
from pathlib import Path

BG_MODE = "para_bidir"
VALID_BG_MODES = [
    "mono_base",
    "mono_vec",
    "bidir_base",
    "bidir_vec",
    "para_bidir_base",
    "para_bidir",
]
INCLUDED_SETS = {"spprclib", "roberti"}


def read_bg(path: Path, mode: str) -> dict[tuple[str, str], dict]:
    """Index bgspprc.csv rows by (instance, ng) filtered to `mode`."""
    out: dict[tuple[str, str], dict] = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["mode"] != mode:
                continue
            out[(r["instance"], r["ng"])] = r
    return out


def read_pw(path: Path) -> dict[tuple[str, str], dict]:
    """Index pathwyse.csv rows by (instance, ng). Keeps the latest row when
    the file contains duplicates (e.g. re-run of the same instance)."""
    out: dict[tuple[str, str], dict] = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            out[(r["instance"], r["ng"])] = r
    return out


def classify(bg_cost: str, pw_cost: str, cost_scale: int) -> str:
    """Return bg_leq / bg_gt / bg_zero / '' for a row.

    Tolerance = max(1e-3, 100/cost_scale) covers the 3-decimal quantum
    used for `cost` in bgspprc.csv plus per-arc int-scaling rounding on
    the Pathwyse side. At cost_scale=1e6 this is 1e-3; at cost_scale=1e3
    it widens to 0.1 to absorb scaled-rounding drift on instances with
    large raw arc costs."""
    if not bg_cost or not pw_cost:
        return ""
    bg, pw = float(bg_cost), float(pw_cost)
    if -1e-9 < bg < 1e-9:
        return "bg_zero"
    tol = max(1e-3, 100.0 / cost_scale)
    return "bg_leq" if bg <= pw + tol else "bg_gt"


def shifted_geomean(vals: list[float], shift: float = 1.0) -> float:
    if not vals:
        return float("nan")
    return math.exp(sum(math.log(v + shift) for v in vals) / len(vals)) - shift


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bg", default="benchmarks/bgspprc.csv")
    ap.add_argument("--pw", default="benchmarks/pathwyse.csv")
    ap.add_argument("--out", default="benchmarks/comparison_pathwyse.csv")
    ap.add_argument("--bg-mode", default=BG_MODE, choices=VALID_BG_MODES)
    args = ap.parse_args()

    bg = read_bg(Path(args.bg), args.bg_mode)
    pw = read_pw(Path(args.pw))

    keys = [
        (inst, ng)
        for (inst, ng), r in bg.items()
        if r["set"] in INCLUDED_SETS and (inst, ng) in pw
    ]
    keys.sort(key=lambda k: (int(k[1]), k[0]))

    rows = []
    for inst, ng in keys:
        b, p = bg[(inst, ng)], pw[(inst, ng)]
        bg_cost, bg_time = b["cost"], b["time_s"]
        pw_cost, pw_time = p["cost"], p["time_s"]
        cost_scale = int(p.get("cost_scale") or 1_000_000)
        tag = classify(bg_cost, pw_cost, cost_scale)
        ratio = ""
        if bg_time and pw_time:
            try:
                bt, pt = float(bg_time), float(pw_time)
                if bt > 0 and pt > 0:
                    ratio = f"{bt / pt:.2f}"
            except ValueError:
                pass
        rows.append(
            [inst, ng, bg_time, pw_time, bg_cost, pw_cost, tag, ratio]
        )

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["instance", "ng", "bgspprc_s", "pathwyse_s", "bgspprc_cost", "pathwyse_cost", "bg_leq", "ratio"]
        )
        w.writerows(rows)

    # Summary
    tags = Counter(r[6] for r in rows)
    print(f"Wrote {len(rows)} rows to {args.out}")
    print(
        f"  bg_leq={tags.get('bg_leq', 0)}  bg_gt={tags.get('bg_gt', 0)}  "
        f"bg_zero={tags.get('bg_zero', 0)}  blank={tags.get('', 0)}"
    )

    bg_times, pw_times = [], []
    for r in rows:
        if r[2] and r[3]:
            try:
                bg_times.append(float(r[2]))
                pw_times.append(float(r[3]))
            except ValueError:
                pass
    if bg_times:
        gb, gp = shifted_geomean(bg_times), shifted_geomean(pw_times)
        print(
            f"  shifted geomean (n={len(bg_times)}): "
            f"bgspprc={gb:.3f}s  pathwyse={gp:.3f}s  ratio={ (gb + 1) / (gp + 1):.2f}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
