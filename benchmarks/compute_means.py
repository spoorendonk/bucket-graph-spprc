#!/usr/bin/env python3
"""Compute the benchmark tables published in the README.

Reads the source CSVs directly and joins them in-memory — there are no
intermediate `comparison_*.csv` files. The joins reproduce the ones that
`run_comparison.sh` and `build_comparison_pathwyse.py` used to materialise:

  pathwyse   bgspprc.csv (mode=para_bidir_vec, set∈{spprclib,roberti})
             ⋈ pathwyse.csv on (instance, ng); sgm shift = 1 s. Also prints
             a bgspprc-vs-Pathwyse cost-agreement summary to stderr.
  rcspp      bgspprc.csv (mode=para_bidir_vec, set∈{ng8,ng16,ng24})
             ⋈ pull_algo_runtimes.csv (`all_s`) on (instance, ng); shift = 1 s.
  pathwyse11 pathwyse-1.1.0/bgspprc.csv ⋈ pathwyse-1.1.0/pathwyse.csv on
             (instance, ng): bgspprc vs PathWyse 1.1.0 PWDefault and PWBucket
             per thread line (bidir_vec vs parallel=0, para_bidir_vec vs
             parallel=1), with each PathWyse algorithm's time ratio to
             bgspprc and its spread. Not part of `all`.
  modes      bgspprc.csv per (set, ng, mode) — the 6-mode/SIMD ablation,
             bgspprc-only.

Each row reports `sgm (s)` (shifted geometric mean, `exp(mean(log(t + s))) − s`,
with `s` = 1 s), `mean (s)` (arithmetic mean), and `solved` (count of instances
completed within the 120 s timeout). Timeouts substitute as 120 s in both means
(never dropped). A failed run (status `ERROR(rc)` or `UNPARSED`, of bgspprc or
PathWyse 1.1.0) is no result: its row is left out of the table and named on
stderr. Each table is printed as github-flavored markdown so the output
can be pasted directly into `benchmarks/README.md`.

Usage:
    python3 benchmarks/compute_means.py [pathwyse|pathwyse11|rcspp|modes|all] [--rows]

Defaults to `all`. `--rows` dumps the per-row joined data as CSV (the
spot-check replacement for the former comparison_*.csv files) instead of the
aggregate tables; it applies to the `pathwyse` and `rcspp` joins.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from collections import Counter, defaultdict
from collections.abc import Callable
from pathlib import Path

ROOT = Path(__file__).resolve().parent

# A CSV row, or a joined row built from them: column -> cell.
Row = dict[str, str]

SET_ORDER = ["spprclib", "roberti", "rcspp"]
NG_ORDER = ["8", "16", "24"]
MODE_ORDER = [
    "mono_base",
    "mono_vec",
    "bidir_base",
    "bidir_vec",
    "para_bidir_base",
    "para_bidir_vec",
]

# Map bgspprc.csv `set` field to display name. rcspp is binned per-ng under
# directory names ng8/ng16/ng24.
SET_DISPLAY = {"ng8": "rcspp", "ng16": "rcspp", "ng24": "rcspp"}

# The bgspprc mode used for both head-to-head comparisons: the default CLI
# build. It is not the fastest everywhere (mono leads on rcspp at every ng) —
# see the mode ablation table.
BG_MODE = "para_bidir_vec"
# Pathwyse comparison covers sppcc + vrp only. The rcspp `.graph` set has its
# own comparison axis (vs Petersen & Spoorendonk 2025 — the rcspp table).
INCLUDED_SETS = {"spprclib", "roberti"}
RCSPP_NG_GROUPS = (8, 16, 24)
TIMEOUT = 120.0
# Row statuses that are results: a solve, or a timeout (scored as unsolved at
# the time limit). ERROR(rc) and UNPARSED rows are failed runs, not timeouts:
# scoring one as a timeout would inflate that solver's times, so the tables
# leave its row out, loudly (`drop_failed`).
RESULT_STATUSES = {"OK", "TIMEOUT"}


def bg_status(r: Row) -> str:
    """A bgspprc.csv row's status. A row from before the column (#157) reads
    OK when it has a cost and a time, else TIMEOUT: the runner then wrote a
    failed run like a timeout, so an old failure cannot be told apart."""
    return r.get("status") or ("OK" if r["cost"] and r["time_s"] else "TIMEOUT")


def drop_failed(
    rows: list[Row], solvers: tuple[str, ...], where: Callable[[Row], str]
) -> list[Row]:
    """The rows on which no solver's run failed, by its `<solver>_status`
    ('' is a CSV without the column, read as a result). Each row left out is
    named on stderr by `where(row)`, with every solver's status."""
    kept = []
    for r in rows:
        statuses = [(s, r[f"{s}_status"]) for s in solvers]
        if all(st in RESULT_STATUSES or st == "" for _, st in statuses):
            kept.append(r)
            continue
        listed = ", ".join(f"{s} {st}" for s, st in statuses)
        print(f"excluded (failed run): {where(r)}: {listed}", file=sys.stderr)
    return kept


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
) -> list[Row]:
    """bgspprc `para_bidir_vec` (rcspp sets) ⋈ paper `all_s` on (instance, ng).

    Pure join over existing data — no solver run. Cells that timed out on
    either side are `'TL'` and substitute as `timeout` seconds downstream;
    a failed bgspprc run is `'TL'` too, and its `bgspprc_status` tells it
    apart. Returns one dict per joined (instance, ng), sorted by (ng,
    instance)."""
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

    # bgspprc para_bidir_vec rows on rcspp sets — empty cost/time = TL.
    bg: dict[tuple[str, int], tuple[str, str]] = {}
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
                bg[key] = ("TL", bg_status(r))
            else:
                bg[key] = (f"{float(r['time_s']):.3f}", bg_status(r))

    rows = []
    for inst, ng in sorted(set(bg) & set(paper), key=lambda k: (k[1], k[0])):
        (bg_v, bg_st), paper_v = bg[(inst, ng)], paper[(inst, ng)]
        bg_sgm = timeout if bg_v == "TL" else float(bg_v)
        paper_sgm = timeout if paper_v == "TL" else float(paper_v)
        rows.append(
            {
                "instance": inst,
                "ng": str(ng),
                "bgspprc_s": bg_v,
                "bgspprc_status": bg_st,
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
) -> list[Row]:
    """bgspprc (`bg_mode`) ⋈ pathwyse on (instance, ng), filtered to
    `included_sets`. Pure join over existing data. Missing times stay `''`
    (downstream treats them as timeouts). Returns one dict per joined
    (instance, ng), sorted by (ng, instance)."""
    bg: dict[tuple[str, str], Row] = {}
    with open(bg_path) as f:
        for r in csv.DictReader(f):
            if r["mode"] != bg_mode:
                continue
            bg[(r["instance"], r["ng"])] = r

    pw: dict[tuple[str, str], Row] = {}
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
                "bgspprc_status": bg_status(b),
                "pathwyse_s": p["time_s"],
                "bgspprc_cost": b["cost"],
                "pathwyse_cost": p["cost"],
                "bg_leq": classify(b["cost"], p["cost"], cost_scale),
                "ratio": ratio,
            }
        )
    return rows


# PathWyse 1.1.0 head-to-head (#153): each thread line pairs a bgspprc mode with
# PathWyse's `algo/default/parallel` setting, and every line compares bgspprc
# against both PathWyse algorithms on the same instances.
PW11_LINES = (("seq", "bidir_vec", "0"), ("par", "para_bidir_vec", "1"))
PW11_ALGOS = ("PWDefault", "PWBucket")
PW11_DIR = ROOT / "pathwyse-1.1.0"


# Sets whose arc costs are integers, so convert to PathWyse at any cost_scale
# without rounding (spprclib's `.sppcc`).
INTEGER_COST_SETS = {"spprclib"}


def costs_agree(bg_cost: str, pw_cost: str, cost_scale: int, integer_costs: bool) -> str:
    """Return eq / ne / '' for a bgspprc and a PathWyse 1.1.0 cost.

    Both solve the same pure ng relaxation over the same ng sets, so their
    optima must match, not merely order: within 1e-3, the 3-decimal quantum
    of bgspprc.csv, widened to 100/cost_scale for the per-arc integer
    rounding of the converted instance unless its costs are integers.
    '' when either side has no cost (a timeout)."""
    if not bg_cost or not pw_cost:
        return ""
    tol = 1e-3 if integer_costs else max(1e-3, 100.0 / cost_scale)
    return "eq" if abs(float(bg_cost) - float(pw_cost)) <= tol else "ne"


def join_pathwyse11(bg_path: Path, pw_path: Path) -> list[Row]:
    """bgspprc ⋈ PathWyse 1.1.0 (PWDefault and PWBucket) on (instance, ng),
    per thread line of `PW11_LINES`, filtered to `INCLUDED_SETS`.

    A (instance, ng, line) is joined only when bgspprc and both algorithms
    have a row for it, so every solver in a table line covers the same
    instances. Empty times stay '' (downstream substitutes the timeout).
    Each solver's row status is kept (`bgspprc_status`, `<algo>_status`; ''
    for a PathWyse CSV without the column); the table drops failed rows. Sorted by (line, ng,
    instance)."""
    bg: dict[tuple[str, str, str], Row] = {}
    with open(bg_path) as f:
        for r in csv.DictReader(f):
            bg[(r["instance"], r["ng"], r["mode"])] = r

    pw: dict[tuple[str, str, str, str], Row] = {}
    with open(pw_path) as f:
        for r in csv.DictReader(f):
            pw[(r["instance"], r["ng"], r["algo"], r["parallel"])] = r  # last wins

    rows = []
    for line, mode, parallel in PW11_LINES:
        # Filter before sorting: only joined keys need an integer ng.
        keys = sorted(
            (
                (inst, ng)
                for inst, ng, m in bg
                if m == mode
                and bg[(inst, ng, m)]["set"] in INCLUDED_SETS
                and all((inst, ng, a, parallel) in pw for a in PW11_ALGOS)
            ),
            key=lambda k: (int(k[1]), k[0]),
        )
        for inst, ng in keys:
            b = bg[(inst, ng, mode)]
            ps = [pw[(inst, ng, a, parallel)] for a in PW11_ALGOS]
            row = {
                "instance": inst,
                "ng": ng,
                "set": b["set"],
                "line": line,
                "bgspprc_s": b["time_s"] if b["cost"] else "",
                "bgspprc_status": bg_status(b),
                "bgspprc_cost": b["cost"],
            }
            for a, p in zip(PW11_ALGOS, ps, strict=True):
                cost_scale = int(p.get("cost_scale") or 1_000_000)
                row[f"{a}_status"] = p.get("status") or ""
                row[f"{a}_s"] = p["time_s"] if p["cost"] else ""
                row[f"{a}_cost"] = p["cost"]
                row[f"{a}_agree"] = costs_agree(
                    b["cost"], p["cost"], cost_scale, b["set"] in INTEGER_COST_SETS
                )
            rows.append(row)
    return rows


def quartiles(values: list[float]) -> tuple[float, float, float]:
    """(q1, median, q3), inclusive method; one value gives it three times."""
    if len(values) == 1:
        return values[0], values[0], values[0]
    q1, q2, q3 = statistics.quantiles(values, n=4, method="inclusive")
    return q1, q2, q3


def pathwyse11_table(bg_path: Path, pw_path: Path, shift: float = 1.0, tl: float = TIMEOUT) -> str:
    """bgspprc vs PathWyse 1.1.0 PWDefault and PWBucket, one row per (set, ng,
    line, solver). Timeouts substitute as `tl` seconds. `ratio` is the
    solver's time over bgspprc's as a shifted geometric mean of the
    per-instance ratios (t + shift) / (t_bg + shift), so > 1 means bgspprc is
    faster; `q1–q3` is the interquartile range of those per-instance ratios,
    the spread laptop timings call for. Cost agreement goes to stderr."""
    # A failed run of any solver leaves its (instance, ng, line) out, so every
    # solver in a table line still covers the same instances.
    rows = drop_failed(
        join_pathwyse11(bg_path, pw_path),
        ("bgspprc", *PW11_ALGOS),
        lambda r: f"{r['instance']} ng{r['ng']} {r['line']}",
    )

    grouped: dict[tuple[str, str, str], list[Row]] = defaultdict(list)
    for r in rows:
        grouped[(r["set"], r["ng"], r["line"])].append(r)

    def secs(v: str) -> float:
        return tl if v == "" else float(v)

    lines = [
        "| set      | ng | line | solver    | sgm (s) | solved | ratio | q1–q3       |",
        "|----------|---:|------|-----------|--------:|-------:|------:|-------------|",
    ]
    for s in [x for x in SET_ORDER if x in INCLUDED_SETS]:
        for ng in NG_ORDER:
            for line, _, _ in PW11_LINES:
                rs = grouped.get((s, ng, line), [])
                if not rs:
                    continue
                bg_ts = [secs(r["bgspprc_s"]) for r in rs]
                for solver in ("bgspprc", *PW11_ALGOS):
                    col = f"{solver}_s"
                    ts = [secs(r[col]) for r in rs]
                    solved = sum(1 for r in rs if r[col] != "")
                    if solver == "bgspprc":
                        ratio, spread = "", ""
                    else:
                        rel = [(t + shift) / (b + shift) for t, b in zip(ts, bg_ts, strict=True)]
                        q1, _, q3 = quartiles(rel)
                        ratio = f"{shifted_geomean(rel, 0.0):.2f}"
                        spread = f"{q1:.2f}–{q3:.2f}"
                    lines.append(
                        f"| {s:<8} | {int(ng):>2} | {line:<4} | {solver:<9} | "
                        f"{shifted_geomean(ts, shift):>7.3f} | "
                        f"{solved:>3}/{len(rs):<3} | {ratio:>5} | {spread:<11} |"
                    )

    for a in PW11_ALGOS:
        tags = Counter(r[f"{a}_agree"] for r in rows)
        print(
            f"cost agreement ({a}, n={len(rows)}): eq={tags.get('eq', 0)}  "
            f"ne={tags.get('ne', 0)}  blank={tags.get('', 0)}",
            file=sys.stderr,
        )
    return "\n".join(lines)


def modes_table(csv_path: Path, shift: float = 1.0, tl: float = TIMEOUT) -> str:
    """bgspprc-only ablation across (set, ng, mode). Empty cost = timeout
    substituted with `tl` seconds in both the sgm and the arithmetic mean.
    `solved` is the count of instances that finished without TL. A failed
    run (`drop_failed`) leaves its instance out of every mode of its (set,
    ng), so the modes are still compared on the same instances."""
    with open(csv_path) as f:
        rows = [{**r, "bgspprc_status": bg_status(r)} for r in csv.DictReader(f)]
    failed = {
        (r["set"], r["ng"], r["instance"])
        for r in rows
        if r["bgspprc_status"] not in RESULT_STATUSES
    }
    rows = [
        r
        for r in drop_failed(
            rows,
            ("bgspprc",),
            lambda r: f"{r['instance']} {r['set']} ng{r['ng']} {r['mode']} (all modes)",
        )
        if (r["set"], r["ng"], r["instance"]) not in failed
    ]
    grouped: dict[tuple[str, str, str], list[Row]] = defaultdict(list)
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


def pathwyse_table(bg_path: Path, pw_path: Path, shift: float = 1.0, tl: float = TIMEOUT) -> str:
    """bgspprc para_bidir_vec vs patched Pathwyse on `.sppcc`/`.vrp`, in Plato
    style: one row per (set, ng, solver). Timeouts substitute as `tl` seconds
    (never dropped). Cost equality is checked per row; the bg_leq/bg_gt/bg_zero
    tally is printed to stderr (inspect `--rows` for the per-row costs).
    Failed bgspprc runs are left out (`drop_failed`)."""
    rows = drop_failed(
        join_pathwyse(bg_path, pw_path),
        ("bgspprc",),
        lambda r: f"{r['instance']} ng{r['ng']}",
    )

    grouped: dict[tuple[str, str], list[Row]] = defaultdict(list)
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
            for solver, col in (("bgspprc", "bgspprc_s"), ("pathwyse", "pathwyse_s")):
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


def rcspp_table(bg_path: Path, paper_path: Path, shift: float = 1.0, tl: float = TIMEOUT) -> str:
    """bgspprc para_bidir_vec vs Petersen & Spoorendonk 2025 paper runtimes per
    ng, Plato-style. `TL` cells substitute as `tl` seconds. Failed bgspprc
    runs are left out (`drop_failed`)."""
    rows = drop_failed(
        join_rcspp(bg_path, paper_path),
        ("bgspprc",),
        lambda r: f"{r['instance']} ng{r['ng']}",
    )

    def to_s(v: str) -> float:
        return tl if v == "TL" else float(v)

    grouped: dict[str, list[Row]] = defaultdict(list)
    for r in rows:
        grouped[r["ng"]].append(r)

    lines = [
        "| ng | solver   | sgm (s) | mean (s) | solved |",
        "|---:|----------|--------:|---------:|-------:|",
    ]
    for ng in sorted(grouped, key=int):
        rs = grouped[ng]
        for solver, col in (("bgspprc", "bgspprc_s"), ("paper", "paper_all_s")):
            ts = [to_s(r[col]) for r in rs]
            solved = sum(1 for r in rs if r[col] != "TL")
            lines.append(
                f"| {int(ng):>2} | {solver:<8} | "
                f"{shifted_geomean(ts, shift):>7.3f} | "
                f"{arithmetic_mean(ts):>8.3f} | "
                f"{solved:>3}/{len(rs):<3} |"
            )
    return "\n".join(lines)


def dump_rows(rows: list[Row], columns: list[str]) -> None:
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
        choices=["pathwyse", "pathwyse11", "rcspp", "modes", "all"],
    )
    ap.add_argument("--bg", default=str(ROOT / "bgspprc.csv"))
    ap.add_argument("--pw", default=str(ROOT / "pathwyse.csv"))
    ap.add_argument("--paper", default=str(ROOT / "pull_algo_runtimes.csv"))
    ap.add_argument("--bg11", default=str(PW11_DIR / "bgspprc.csv"))
    ap.add_argument("--pw11", default=str(PW11_DIR / "pathwyse.csv"))
    ap.add_argument(
        "--rows",
        action="store_true",
        help="dump per-row joined data as CSV instead of aggregate tables",
    )
    args = ap.parse_args()
    bg, pw, paper = Path(args.bg), Path(args.pw), Path(args.paper)
    bg11, pw11 = Path(args.bg11), Path(args.pw11)

    # Validate the inputs the chosen table actually reads, with a hint on how to
    # generate each one — beats a raw FileNotFoundError traceback.
    needed = {"bg": bg}
    if args.table == "pathwyse11":
        needed = {"bg11": bg11, "pw11": pw11}
    if args.table in ("pathwyse", "all"):
        needed["pw"] = pw
    if args.table in ("rcspp", "all"):
        needed["paper"] = paper
    hint = {
        "bg": "generate it with benchmarks/run_benchmarks.sh",
        "pw": "generate it with benchmarks/run_pathwyse.sh --pathwyse 1.0",
        "bg11": "generate it with BGSPPRC_CSV=<it> benchmarks/run_benchmarks.sh",
        "pw11": "generate it with benchmarks/run_pathwyse.sh",
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
                [
                    "instance",
                    "ng",
                    "bgspprc_s",
                    "bgspprc_status",
                    "pathwyse_s",
                    "bgspprc_cost",
                    "pathwyse_cost",
                    "bg_leq",
                    "ratio",
                ],
            )
        elif args.table == "pathwyse11":
            dump_rows(
                join_pathwyse11(bg11, pw11),
                ["instance", "set", "ng", "line", "bgspprc_s", "bgspprc_status", "bgspprc_cost"]
                + [f"{a}_{c}" for a in PW11_ALGOS for c in ("status", "s", "cost", "agree")],
            )
        elif args.table == "rcspp":
            dump_rows(
                join_rcspp(bg, paper),
                ["instance", "ng", "bgspprc_s", "bgspprc_status", "paper_all_s", "ratio"],
            )
        else:
            print(
                "--rows requires an explicit table: pathwyse, pathwyse11 or rcspp",
                file=sys.stderr,
            )
            return 2
        return 0

    if args.table == "pathwyse11":
        print(pathwyse11_table(bg11, pw11))
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
