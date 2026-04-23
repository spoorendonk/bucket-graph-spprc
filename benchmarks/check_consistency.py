#!/usr/bin/env python3
"""check_consistency.py — Cross-mode sanity check on bgspprc.csv.

For each (instance, set, ng) group, verify that every solver mode that
produced a cost agrees within tolerance across all six modes (bidir axis
× SIMD axis). A mismatch means one of the modes is returning an
infeasible or wrong optimum — a correctness bug, not a performance issue.

Usage:
  ./benchmarks/check_consistency.py [--csv FILE] [--eps EPS]

Exit code:
  0 — all solved groups consistent
  1 — at least one mismatch
"""
from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

MODES = (
    "mono_base",
    "mono_vec",
    "bidir_base",
    "bidir_vec",
    "para_bidir_base",
    "para_bidir",
)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--csv", default=Path(__file__).parent / "bgspprc.csv", type=Path)
    p.add_argument("--eps", type=float, default=1e-3)
    args = p.parse_args()

    groups: dict[tuple[str, str, str], dict[str, str]] = defaultdict(dict)
    with args.csv.open() as f:
        for r in csv.DictReader(f):
            groups[(r["instance"], r["set"], r["ng"])][r["mode"]] = r["cost"]

    full_ok = full_bad = partial_ok = partial_bad = unsolved = 0
    mismatches: list[tuple[tuple[str, str, str], dict[str, float]]] = []

    for key, m2c in sorted(groups.items()):
        solved = {m: float(c) for m, c in m2c.items() if c}
        if not solved:
            unsolved += 1
            continue
        consistent = len(solved) < 2 or max(solved.values()) - min(solved.values()) <= args.eps
        full = len(solved) == len(MODES)
        if consistent and full:
            full_ok += 1
        elif consistent:
            partial_ok += 1
        elif full:
            full_bad += 1
            mismatches.append((key, solved))
        else:
            partial_bad += 1
            mismatches.append((key, solved))

    total = len(groups)
    print(f"groups: {total}")
    print(f"  all-modes-agree:     {full_ok}")
    print(f"  partial-agree:       {partial_ok}  (some modes timed out, rest agree)")
    print(f"  all-modes-timeout:   {unsolved}")
    print(f"  MISMATCH (full):     {full_bad}")
    print(f"  MISMATCH (partial):  {partial_bad}")

    if mismatches:
        print("\nMismatches:")
        for (inst, sset, ng), solved in mismatches:
            spread = max(solved.values()) - min(solved.values())
            print(f"  {inst} [{sset}, ng={ng}] spread={spread:.4g}  {solved}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
