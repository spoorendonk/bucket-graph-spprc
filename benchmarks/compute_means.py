#!/usr/bin/env python3
"""Compute the benchmark tables published in the README.

Reads the source CSVs directly and joins them in-memory — there are no
intermediate `comparison_*.csv` files. The joins reproduce the ones that
`run_comparison.sh` and `build_comparison_pathwyse.py` used to materialise:

  pathwyse   bgspprc.csv (mode=para_bidir, set∈{spprclib,roberti})
             ⋈ pathwyse.csv on (instance, ng); sgm shift = 1 s. Also prints
             a bgspprc-vs-Pathwyse cost-agreement summary to stderr.
  rcspp      bgspprc.csv (mode=para_bidir, set∈{ng8,ng16,ng24})
             ⋈ pull_algo_runtimes.csv (`all_s`) on (instance, ng); shift = 1 s.
  modes      bgspprc.csv per (set, ng, mode) — the 6-mode/SIMD ablation,
             bgspprc-only.

Each row reports `sgm (s)` (shifted geometric mean, `exp(mean(log(t + s))) − s`,
with `s` = 1 s), `mean (s)` (arithmetic mean), and `solved` (count of instances
completed within the 120 s timeout). Timeouts substitute as 120 s in both means
(never dropped). Each table is printed as github-flavored markdown so the output
can be pasted directly into `benchmarks/README.md`.

Usage:
    python3 benchmarks/compute_means.py [pathwyse|rcspp|modes|all] [--rows]

Defaults to `all`. `--rows` dumps the per-row joined data as CSV (the
spot-check replacement for the former comparison_*.csv files) instead of the
aggregate tables; it applies to the `pathwyse` and `rcspp` joins.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent

SET_ORDER = ["spprclib", "roberti", "rcspp"]
NG_ORDER = ["8", "16", "24"]
MODE_ORDER = [
    "mono_base",
    "mono_vec",
    "bidir_base",
    "bidir_vec",
    "para_bidir_base",
    "para_bidir",
]

# Map bgspprc.csv `set` field to display name. rcspp is binned per-ng under
# directory names ng8/ng16/ng24.
SET_DISPLAY = {"ng8": "rcspp", "ng16": "rcspp", "ng24": "rcspp"}

# The bgspprc mode used for both head-to-head comparisons (consistently fastest
# across ng=16/24; mono wins at ng=8 only — see the mode ablation table).
BG_MODE = "para_bidir"
# Pathwyse comparison covers sppcc + vrp only. The rcspp `.graph` set has its
# own comparison axis (vs Petersen & Spoorendonk 2025 — the rcspp table).
INCLUDED_SETS = {"spprclib", "roberti"}
RCSPP_NG_GROUPS = (8, 16, 24)
TIMEOUT = 120.0


def shifted_geomean(values: list[float], shift: float) -> float:
    if not values:
        return float("nan")
    return math.exp(sum(math.log(v + shift) for v in values) / len(values)) - shift


def arithmetic_mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def classify(bg_cost: str, pw_cost: str, cost_scale: int) -> str:
    """Return bg_leq / bg_gt / bg_zero / '' for a row.

    Tolerance = max(1e-3, 100/cost_scale) covers the 3-decimal quantum used for
    `cost` in bgspprc.csv plus per-arc int-scaling rounding on the Pathwyse
    side. At cost_scale=1e6 this is 1e-3; at cost_scale=1e3 it widens to 0.1 to
    absorb scaled-rounding drift on instances with large raw arc costs."""
    if not bg_cost or not pw_cost:
        return ""
    bg, pw = float(bg_cost), float(pw_cost)
    if -1e-9 < bg < 1e-9:
        return "bg_zero"
    tol = max(1e-3, 100.0 / cost_scale)
    return "bg_leq" if bg <= pw + tol else "bg_gt"


def join_rcspp(
    bg_path: Path,
    paper_path: Path,
    ng_groups: tuple[int, ...] = RCSPP_NG_GROUPS,
    timeout: float = TIMEOUT,
) -> list[dict]:
    """bgspprc `para_bidir` (rcspp sets) ⋈ paper `all_s` on (instance, ng).

    Pure join over existing data — no solver run. Cells that timed out on
    either side are `'TL'` and substitute as `timeout` seconds downstream.
    Returns one dict per joined (instance, ng), sorted by (ng, instance)."""
    ng_set = set(ng_groups)

    # Paper all_s — mark >timeout as TL; skip unparseable.
    paper: dict[tuple[str, int], str] = {}
    with open(paper_path) as f:
        for r in csv.DictReader(f):
            try:
                ng = int(r["ng"])
            except (TypeError, ValueError):
                continue
            if ng not in ng_set:
                continue
            try:
                t = float(r["all_s"])
            except (TypeError, ValueError):
                continue
            paper[(r["instance"], ng)] = "TL" if t > timeout else f"{t:.3f}"

    # bgspprc para_bidir rows on rcspp sets — empty cost/time = TL.
    bg: dict[tuple[str, int], str] = {}
    with open(bg_path) as f:
        for r in csv.DictReader(f):
            if r["mode"] != BG_MODE or not r["set"].startswith("ng"):
                continue
            try:
                ng = int(r["ng"])
            except (TypeError, ValueError):
                continue
            if ng not in ng_set:
                continue
            key = (r["instance"], ng)
            if not r["cost"] or not r["time_s"]:
                bg[key] = "TL"
            else:
                bg[key] = f"{float(r['time_s']):.3f}"

    rows = []
    for inst, ng in sorted(set(bg) & set(paper), key=lambda k: (k[1], k[0])):
        bg_v, paper_v = bg[(inst, ng)], paper[(inst, ng)]
        bg_sgm = timeout if bg_v == "TL" else float(bg_v)
        paper_sgm = timeout if paper_v == "TL" else float(paper_v)
        rows.append(
            {
                "instance": inst,
                "ng": str(ng),
                "bgspprc_s": bg_v,
                "paper_all_s": paper_v,
                "ratio": f"{bg_sgm / paper_sgm:.2f}",
            }
        )
    return rows


def join_pathwyse(
    bg_path: Path,
    pw_path: Path,
    bg_mode: str = BG_MODE,
    included_sets: set[str] = INCLUDED_SETS,
) -> list[dict]:
    """bgspprc (`bg_mode`) ⋈ pathwyse on (instance, ng), filtered to
    `included_sets`. Pure join over existing data. Missing times stay `''`
    (downstream treats them as timeouts). Returns one dict per joined
    (instance, ng), sorted by (ng, instance)."""
    bg: dict[tuple[str, str], dict] = {}
    with open(bg_path) as f:
        for r in csv.DictReader(f):
            if r["mode"] != bg_mode:
                continue
            bg[(r["instance"], r["ng"])] = r

    pw: dict[tuple[str, str], dict] = {}
    with open(pw_path) as f:
        for r in csv.DictReader(f):
            pw[(r["instance"], r["ng"])] = r  # last-seen row wins on dups

    keys = [k for k, r in bg.items() if r["set"] in included_sets and k in pw]
    keys.sort(key=lambda k: (int(k[1]), k[0]))

    rows = []
    for inst, ng in keys:
        b, p = bg[(inst, ng)], pw[(inst, ng)]
        cost_scale = int(p.get("cost_scale") or 1_000_000)
        ratio = ""
        if b["time_s"] and p["time_s"]:
            try:
                bt, pt = float(b["time_s"]), float(p["time_s"])
                if bt > 0 and pt > 0:
                    ratio = f"{bt / pt:.2f}"
            except ValueError:
                pass
        rows.append(
            {
                "instance": inst,
                "ng": ng,
                "set": b["set"],
                "bgspprc_s": b["time_s"],
                "pathwyse_s": p["time_s"],
                "bgspprc_cost": b["cost"],
                "pathwyse_cost": p["cost"],
                "bg_leq": classify(b["cost"], p["cost"], cost_scale),
                "ratio": ratio,
            }
        )
    return rows


def modes_table(csv_path: Path, shift: float = 1.0, tl: float = TIMEOUT) -> str:
    """bgspprc-only ablation across (set, ng, mode). Empty cost = timeout
    substituted with `tl` seconds in both the sgm and the arithmetic mean.
    `solved` is the count of instances that finished without TL."""
    rows = list(csv.DictReader(open(csv_path)))
    grouped: dict[tuple[str, str, str], list[dict]] = defaultdict(list)
    for r in rows:
        s = SET_DISPLAY.get(r["set"], r["set"])
        grouped[(s, r["ng"], r["mode"])].append(r)

    lines = [
        "| set       | ng | mode             | sgm (s) | mean (s) | solved |",
        "|-----------|---:|------------------|--------:|---------:|-------:|",
    ]
    for s in SET_ORDER:
        for ng in NG_ORDER:
            for m in MODE_ORDER:
                rs = grouped.get((s, ng, m), [])
                if not rs:
                    continue
                ts = [tl if r["cost"] == "" else float(r["time_s"]) for r in rs]
                solved = sum(1 for r in rs if r["cost"] != "")
                lines.append(
                    f"| {s:<9} | {int(ng):>2} | {m:<16} | "
                    f"{shifted_geomean(ts, shift):>7.3f} | "
                    f"{arithmetic_mean(ts):>8.3f} | "
                    f"{solved:>3}/{len(rs):<3} |"
                )
    return "\n".join(lines)


def pathwyse_table(
    bg_path: Path, pw_path: Path, shift: float = 1.0, tl: float = TIMEOUT
) -> str:
    """bgspprc para_bidir vs patched Pathwyse on `.sppcc`/`.vrp`, in Plato
    style: one row per (set, ng, solver). Timeouts substitute as `tl` seconds
    (never dropped). Cost equality is checked per row; the bg_leq/bg_gt/bg_zero
    tally is printed to stderr (inspect `--rows` for the per-row costs)."""
    rows = join_pathwyse(bg_path, pw_path)

    grouped: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in rows:
        grouped[(r["set"], r["ng"])].append(r)

    lines = [
        "| set      | ng | solver   | sgm (s) | mean (s) | solved |",
        "|----------|---:|----------|--------:|---------:|-------:|",
    ]
    for s in [x for x in SET_ORDER if x in INCLUDED_SETS]:
        for ng in NG_ORDER:
            rs = grouped.get((s, ng), [])
            if not rs:
                continue
            for solver, col in (("bgspprc", "bgspprc_s"),
                                ("pathwyse", "pathwyse_s")):
                ts = [tl if r[col] == "" else float(r[col]) for r in rs]
                solved = sum(1 for r in rs if r[col] != "")
                lines.append(
                    f"| {s:<8} | {int(ng):>2} | {solver:<8} | "
                    f"{shifted_geomean(ts, shift):>7.3f} | "
                    f"{arithmetic_mean(ts):>8.3f} | "
                    f"{solved:>3}/{len(rs):<3} |"
                )

    # Cost-agreement check (folded in from the former build_comparison_pathwyse.py).
    # Printed to stderr so stdout stays a clean, pasteable markdown table.
    tags = Counter(r["bg_leq"] for r in rows)
    print(
        f"cost agreement (pathwyse, n={len(rows)}): "
        f"bg_leq={tags.get('bg_leq', 0)}  bg_gt={tags.get('bg_gt', 0)}  "
        f"bg_zero={tags.get('bg_zero', 0)}  blank={tags.get('', 0)}",
        file=sys.stderr,
    )
    return "\n".join(lines)


def rcspp_table(
    bg_path: Path, paper_path: Path, shift: float = 1.0, tl: float = TIMEOUT
) -> str:
    """bgspprc para_bidir vs Petersen & Spoorendonk 2025 paper runtimes per
    ng, Plato-style. `TL` cells substitute as `tl` seconds."""
    rows = join_rcspp(bg_path, paper_path)

    def to_s(v: str) -> float:
        return tl if v == "TL" else float(v)

    grouped: dict[str, list[dict]] = defaultdict(list)
    for r in rows:
        grouped[r["ng"]].append(r)

    lines = [
        "| ng | solver   | sgm (s) | mean (s) | solved |",
        "|---:|----------|--------:|---------:|-------:|",
    ]
    for ng in sorted(grouped, key=int):
        rs = grouped[ng]
        for solver, col in (("bgspprc", "bgspprc_s"),
                            ("paper", "paper_all_s")):
            ts = [to_s(r[col]) for r in rs]
            solved = sum(1 for r in rs if r[col] != "TL")
            lines.append(
                f"| {int(ng):>2} | {solver:<8} | "
                f"{shifted_geomean(ts, shift):>7.3f} | "
                f"{arithmetic_mean(ts):>8.3f} | "
                f"{solved:>3}/{len(rs):<3} |"
            )
    return "\n".join(lines)


def dump_rows(rows: list[dict], columns: list[str]) -> None:
    """Write joined rows as CSV to stdout (spot-check / per-row inspection)."""
    w = csv.DictWriter(sys.stdout, fieldnames=columns, extrasaction="ignore")
    w.writeheader()
    w.writerows(rows)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    ap.add_argument(
        "table",
        nargs="?",
        default="all",
        choices=["pathwyse", "rcspp", "modes", "all"],
    )
    ap.add_argument("--bg", default=str(ROOT / "bgspprc.csv"))
    ap.add_argument("--pw", default=str(ROOT / "pathwyse.csv"))
    ap.add_argument("--paper", default=str(ROOT / "pull_algo_runtimes.csv"))
    ap.add_argument(
        "--rows",
        action="store_true",
        help="dump per-row joined data as CSV instead of aggregate tables",
    )
    args = ap.parse_args()
    bg, pw, paper = Path(args.bg), Path(args.pw), Path(args.paper)

    # Validate the inputs the chosen table actually reads, with a hint on how to
    # generate each one — beats a raw FileNotFoundError traceback.
    needed = {"bg": bg}
    if args.table in ("pathwyse", "all"):
        needed["pw"] = pw
    if args.table in ("rcspp", "all"):
        needed["paper"] = paper
    hint = {
        "bg": "generate it with benchmarks/run_benchmarks.sh",
        "pw": "generate it with benchmarks/run_pathwyse.sh",
        "paper": "checked-in reference data — see benchmarks/README.md",
    }
    missing = [(k, p) for k, p in needed.items() if not p.exists()]
    if missing:
        for k, p in missing:
            print(f"error: {p} not found ({hint[k]})", file=sys.stderr)
        return 1

    if args.rows:
        # Each join has its own column schema, so a row dump is for one table.
        if args.table == "pathwyse":
            dump_rows(
                join_pathwyse(bg, pw),
                ["instance", "ng", "bgspprc_s", "pathwyse_s",
                 "bgspprc_cost", "pathwyse_cost", "bg_leq", "ratio"],
            )
        elif args.table == "rcspp":
            dump_rows(
                join_rcspp(bg, paper),
                ["instance", "ng", "bgspprc_s", "paper_all_s", "ratio"],
            )
        else:
            print("--rows requires an explicit table: pathwyse or rcspp",
                  file=sys.stderr)
            return 2
        return 0

    if args.table in ("pathwyse", "all"):
        if args.table == "all":
            print("### Pathwyse comparison (sppcc + vrp)\n")
        print(pathwyse_table(bg, pw))
        if args.table == "all":
            print()

    if args.table in ("rcspp", "all"):
        if args.table == "all":
            print("### Paper comparison (rcspp)\n")
        print(rcspp_table(bg, paper))
        if args.table == "all":
            print()

    if args.table in ("modes", "all"):
        if args.table == "all":
            print("### bgspprc mode/SIMD ablation\n")
        print(modes_table(bg))

    return 0


if __name__ == "__main__":
    sys.exit(main())
