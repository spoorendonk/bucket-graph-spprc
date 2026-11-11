#!/usr/bin/env python3
"""Convert bgspprc instance files (.sppcc, .vrp, .graph) to Pathwyse format.

Usage:
    python3 benchmarks/convert_to_pathwyse.py [--outdir DIR] [--cost-scale N]
        [--time-scale N] [--cap-scale N] PATH...

Arguments:
    PATH            Instance file or directory of instances.
    --outdir DIR    Output directory (default: benchmarks/instances/pathwyse).
    --cost-scale N  Multiplier applied to EDGE_COST before int truncation.
                    Default: per-extension auto (sppcc=1, vrp=1000, graph=1000).
                    Pathwyse accumulates path cost in int32, so the scale must
                    leave headroom for summing along paths, not just for one arc.
    --time-scale N  Multiplier for TW bounds and arc times (default: 1000).
    --cap-scale N   Multiplier for capacity bounds and demands (default: 1).

Converts:
    .sppcc  → Pathwyse format with CAP resource (capacity-constrained).
    .vrp    → Pathwyse format with CAP resource (CVRP pricing).
    .graph  → Pathwyse format with CAP + TW resources (time windows + capacity).

Pathwyse reads every numeric field with std::stoi, so fractional values are
truncated at the decimal point. To preserve precision, we pre-multiply each
field by the appropriate scale and round to int. Consumers must divide the
reported Obj by cost-scale to recover the true objective value.

The scales used are written to a sidecar `<stem>.scales` file next to each
converted `.txt` so downstream scripts can read them back.

The conversion models the same graph structure as bgspprc's loaders:
    - Source/sink split: depot becomes source (vertex 0), sink is added as last vertex.
    - Arc costs include duals/profits as per bgspprc's instance_io.h.
    - Ng-neighborhoods are computed by Pathwyse internally via algo/default/ng/set_size.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import TypedDict

# An arc as the writer reads it: (from, to, cost, demand at `to`).
Arc = tuple[int, int, float, float]


class _InstanceBase(TypedDict):
    name: str
    n_vertices: int
    source: int
    sink: int
    arcs: list[Arc]
    cap_ub: float
    demands: list[float]
    has_tw: bool
    tw_lb: list[float] | None
    tw_ub: list[float] | None
    arc_times: list[float] | None


class Instance(_InstanceBase, total=False):
    """A parsed instance; only `.graph` files carry per-vertex capacities."""

    cap_ub_per_vertex: list[float]


# Per-format cost-scale defaults. Pathwyse reads every field with std::stoi
# and accumulates path costs in int32, so the scale must leave headroom for
# summing along long paths — not just for fitting one arc. Per-format:
#   .sppcc   → 1     (spprclib costs are integers natively)
#   .vrp     → 1000  (Roberti uses Euclidean distances, 3 decimals are plenty)
#   .graph   → 1000  (rcspp reduced costs, 3 decimals are plenty)
DEFAULT_COST_SCALES = {".sppcc": 1, ".vrp": 1000, ".graph": 1000}
DEFAULT_TIME_SCALE = 1_000
DEFAULT_CAP_SCALE = 1


_INT32_MAX = 2_147_483_647


def _scale_int(value: float | str, scale: float) -> int:
    """Multiply by scale and round to nearest int. Pathwyse stores all fields
    as int (std::stoi → 32-bit), so callers must scale back on output side."""
    result = round(float(value) * scale)
    if abs(result) > _INT32_MAX:
        raise OverflowError(
            f"scaled value {result} exceeds int32 range "
            f"(value={value}, scale={scale}). Use a smaller --cost-scale."
        )
    return result


def parse_sppcc(filepath: str | Path) -> Instance:
    """Parse SPPCC file (spprclib format) — matches instance_io.h load_sppcc."""
    with open(filepath) as f:
        lines = f.readlines()

    name = ""
    dimension = 0
    capacity = 0.0
    dist_matrix: list[list[float]] = []
    node_weights: list[float] = []
    demands: list[float] = []

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("NAME"):
            pos = line.find(":")
            if pos >= 0:
                name = line[pos + 2 :].strip()
        elif line.startswith("DIMENSION"):
            dimension = int(line.split(":")[1].strip())
        elif line.startswith("CAPACITY"):
            capacity = float(line.split(":")[1].strip())
        elif line.startswith("EDGE_WEIGHT_SECTION"):
            dist_matrix = []
            for _ in range(dimension):
                vals: list[float] = []
                while len(vals) < dimension:
                    i += 1
                    vals.extend(float(x) for x in lines[i].split())
                dist_matrix.append(vals)
        elif line.startswith("NODE_WEIGHT_SECTION"):
            node_weights = []
            while len(node_weights) < dimension:
                i += 1
                node_weights.extend(float(x) for x in lines[i].split())
        elif line.startswith("DEMAND_SECTION"):
            demands = [0.0] * dimension
            for _ in range(dimension):
                i += 1
                parts = lines[i].split()
                idx = int(parts[0]) - 1  # 1-indexed to 0-indexed
                demands[idx] = float(parts[1])
        i += 1

    # Build graph: source=0 (depot), sink=N (copy of depot)
    N = dimension
    n_vertices = N + 1
    source = 0
    sink = N

    arcs: list[Arc] = []
    for ii in range(N):
        for jj in range(1, N):
            if ii == jj:
                continue
            cost = dist_matrix[ii][jj] + node_weights[jj]
            arcs.append((ii, jj, cost, demands[jj]))
        # Arc from customer i to sink (return to depot)
        if ii > 0:
            arcs.append((ii, sink, dist_matrix[ii][0], 0.0))

    # Add depot dual to arcs leaving source
    depot_dual = node_weights[0]
    for idx in range(len(arcs)):
        if arcs[idx][0] == 0:
            frm, to, c, d = arcs[idx]
            arcs[idx] = (frm, to, c + depot_dual, d)

    return {
        "name": name,
        "n_vertices": n_vertices,
        "source": source,
        "sink": sink,
        "arcs": arcs,
        "cap_ub": capacity,
        "demands": [*demands, 0.0],  # sink has zero demand
        "has_tw": False,
        "tw_lb": None,
        "tw_ub": None,
        "arc_times": None,
    }


def parse_vrp(filepath: str | Path) -> Instance:
    """Parse Roberti VRP file — matches instance_io.h load_roberti_vrp."""
    with open(filepath) as f:
        lines = f.readlines()

    name = ""
    dimension = 0
    capacity = 0.0
    x: list[float] = []
    y: list[float] = []
    demands: list[float] = []
    profits: list[float] = []
    depot = 1  # 1-indexed

    NONE, COORDS, DEMANDS, DEPOT, PROFIT = range(5)
    section = NONE

    for line in lines:
        line = line.strip()
        if line.startswith("NAME"):
            pos = line.find(":")
            if pos >= 0:
                name = line[pos + 2 :].strip()
        elif line.startswith("DIMENSION"):
            dimension = int(line.split(":")[1].strip())
            x = [0.0] * dimension
            y = [0.0] * dimension
            demands = [0.0] * dimension
            profits = [0.0] * dimension
        elif line.startswith("CAPACITY"):
            capacity = float(line.split(":")[1].strip())
        elif line.startswith("NODE_COORD_SECTION"):
            section = COORDS
        elif line.startswith("DEMAND_SECTION"):
            section = DEMANDS
        elif line.startswith("DEPOT_SECTION"):
            section = DEPOT
        elif line.startswith("PROFIT_SECTION"):
            section = PROFIT
        elif line.startswith("EOF") or line.startswith("EDGE_WEIGHT"):
            section = NONE
        elif section == COORDS:
            parts = line.split()
            if len(parts) == 3:
                idx = int(parts[0]) - 1
                x[idx] = float(parts[1])
                y[idx] = float(parts[2])
        elif section == DEMANDS:
            parts = line.split()
            if len(parts) == 2:
                idx = int(parts[0]) - 1
                demands[idx] = float(parts[1])
        elif section == DEPOT:
            parts = line.split()
            if parts and int(parts[0]) > 0:
                depot = int(parts[0])
        elif section == PROFIT:
            parts = line.split()
            if len(parts) == 2:
                idx = int(parts[0]) - 1
                profits[idx] = float(parts[1])

    def dist(i: int, j: int) -> float:
        dx = x[i] - x[j]
        dy = y[i] - y[j]
        return math.sqrt(dx * dx + dy * dy)

    dep0 = depot - 1  # 0-indexed depot
    N = dimension
    n_vertices = N + 1
    source = dep0
    sink = N

    arcs: list[Arc] = []
    for ii in range(N):
        for jj in range(N):
            if ii == jj:
                continue
            if jj == dep0:
                continue  # use sink instead
            arcs.append((ii, jj, dist(ii, jj) - profits[jj], demands[jj]))
        # Arc to sink (return to depot)
        if ii != dep0:
            arcs.append((ii, sink, dist(ii, dep0) - profits[dep0], 0.0))

    return {
        "name": name,
        "n_vertices": n_vertices,
        "source": source,
        "sink": sink,
        "arcs": arcs,
        "cap_ub": capacity,
        "demands": [*demands, 0.0],  # sink has zero demand
        "has_tw": False,
        "tw_lb": None,
        "tw_ub": None,
        "arc_times": None,
    }


def parse_graph(filepath: str | Path) -> Instance:
    """Parse .graph file (rcspp_dataset format) — matches instance_io.h load_rcspp_graph."""
    with open(filepath) as f:
        lines = f.readlines()

    n_vertices = 0
    name = ""
    # Per vertex: time window [a, b], demand d, capacity Q.
    vertices: list[dict[str, float]] = []
    arcs: list[tuple[int, int, float, float, float]] = []  # (from, to, cost, demand, time)

    for line in lines:
        line = line.strip()
        if not line or line.startswith("c"):
            continue

        if line.startswith("p "):
            parts = line.split()
            name = parts[1]
            n_vertices = int(parts[2])
            vertices = [{} for _ in range(n_vertices)]
        elif line.startswith("v "):
            parts = line.split()
            vid = int(parts[1])
            a = float(parts[2])
            b = float(parts[3])
            d = float(parts[4])
            Q = float(parts[5])
            vertices[vid] = {"a": a, "b": b, "d": d, "Q": Q}
        elif line.startswith("e "):
            parts = line.split()
            src = int(parts[2])
            tgt = int(parts[3])
            cost = float(parts[4])
            time = float(parts[5])
            arcs.append((src, tgt, cost, vertices[tgt]["d"], time))

    source = 0
    sink = n_vertices - 1

    tw_lb = [v["a"] for v in vertices]
    tw_ub = [v["b"] for v in vertices]
    cap_ub_per_vertex = [v["Q"] for v in vertices]

    return {
        "name": name,
        "n_vertices": n_vertices,
        "source": source,
        "sink": sink,
        "arcs": [(a[0], a[1], a[2], a[3]) for a in arcs],
        "cap_ub": max(cap_ub_per_vertex),  # global capacity (max; per-vertex bounds override)
        "cap_ub_per_vertex": cap_ub_per_vertex,
        "demands": [v["d"] for v in vertices],
        "has_tw": True,
        "tw_lb": tw_lb,
        "tw_ub": tw_ub,
        "arc_times": [a[4] for a in arcs],
    }


def write_pathwyse(
    inst: Instance,
    outpath: str | Path,
    cost_scale: int = 1,
    time_scale: int = DEFAULT_TIME_SCALE,
    cap_scale: int = DEFAULT_CAP_SCALE,
) -> None:
    """Write instance in Pathwyse format. All numeric fields are scaled and
    rounded to int since Pathwyse parses every value with std::stoi.

    Pathwyse accumulates label costs in int32, so an aggressively large
    cost_scale will saturate the accumulator over long paths even when
    individual arcs fit. Callers should choose cost_scale via
    DEFAULT_COST_SCALES[ext]; this function only enforces the per-arc bound
    as a final safety check.
    """
    nv = inst["n_vertices"]
    source = inst["source"]
    sink = inst["sink"]
    has_tw = inst["has_tw"]

    max_abs_cost = max((abs(float(a[2])) for a in inst["arcs"]), default=0.0)
    if max_abs_cost * cost_scale > _INT32_MAX:
        raise OverflowError(
            f"cost_scale={cost_scale} overflows int32 per-arc for "
            f"|cost|={max_abs_cost} on instance {inst['name']}"
        )
    # Soft path-sum bound: a worst-case all-customers path has nv-1 arcs;
    # if even that saturates int32 we'll silently truncate inside Pathwyse.
    if max_abs_cost * cost_scale * max(nv - 1, 1) > _INT32_MAX:
        raise OverflowError(
            f"cost_scale={cost_scale} overflows int32 for "
            f"path-sum bound (|cost|={max_abs_cost}, nv={nv}) on "
            f"instance {inst['name']}; reduce --cost-scale"
        )

    # Determine resources
    n_res = 0
    res_types: list[str] = []
    # Resource 0: capacity (CAP)
    res_types.append("CAP")
    n_res += 1
    # Resource 1: time window (TW) if present
    if has_tw:
        res_types.append("TW")
        n_res += 1

    with open(outpath, "w") as f:
        # Header
        f.write(f"NAME : {inst['name']}\n")
        f.write(f"SIZE : {nv}\n")
        f.write("DIRECTED : 1\n")
        f.write("CYCLIC : 1\n")
        f.write(f"ORIGIN : {source}\n")
        f.write(f"DESTINATION : {sink}\n")
        f.write(f"RESOURCES : {n_res}\n")
        f.write(f"RES_NAMES : {' '.join(str(i) for i in range(n_res))}\n")

        # Resource types
        f.write("RES_TYPE\n")
        f.writelines(f"{i} {rt}\n" for i, rt in enumerate(res_types))
        f.write("END\n")

        # The time windows, which the parsers set exactly when they set has_tw.
        tw: tuple[list[float], list[float]] | None = None
        if has_tw:
            tw_lb, tw_ub = inst["tw_lb"], inst["tw_ub"]
            if tw_lb is None or tw_ub is None:
                raise ValueError(f"instance {inst['name']} has has_tw but no time windows")
            tw = (tw_lb, tw_ub)

        # Global resource bounds
        f.write("RES_BOUND\n")
        f.write(f"0 0 {_scale_int(inst['cap_ub'], cap_scale)}\n")
        if tw:
            tw_lb, tw_ub = tw
            # Global time window: [min_tw_lb, max_tw_ub]
            f.write(
                f"1 {_scale_int(tw_lb[source], time_scale)} {_scale_int(tw_ub[sink], time_scale)}\n"
            )
        f.write("END\n")

        # Per-node resource bounds (time windows)
        if tw:
            tw_lb, tw_ub = tw
            f.write("RES_NODE_BOUND\n")
            f.writelines(
                f"1 {v} {_scale_int(tw_lb[v], time_scale)} {_scale_int(tw_ub[v], time_scale)}\n"
                for v in range(nv)
            )
            f.write("END\n")

        # Per-node capacity bounds (if per-vertex caps differ)
        cap_per_v = inst.get("cap_ub_per_vertex")
        if cap_per_v:
            f.write("RES_NODE_BOUND\n")
            for v in range(nv):
                f.write(f"0 {v} 0 {_scale_int(cap_per_v[v], cap_scale)}\n")
            f.write("END\n")

        # Edge costs
        f.write("EDGE_COST\n")
        f.writelines(
            f"{arc[0]} {arc[1]} {_scale_int(arc[2], cost_scale)}\n" for arc in inst["arcs"]
        )
        f.write("END\n")

        # Node costs: all zero (costs are on arcs, which already include duals)
        # Pathwyse adds node cost of target to arc traversal cost,
        # but our arc costs already include target duals. So all node costs = 0.
        # No NODE_COST section needed (defaults to 0).

        # Node consumption: capacity (resource 0)
        # Pathwyse CAP resources use node consumption (demand at each vertex).
        f.write("NODE_CONSUMPTION\n")
        demands = inst["demands"]
        for v in range(nv):
            if demands[v] != 0:
                f.write(f"0 {v} {_scale_int(demands[v], cap_scale)}\n")
        f.write("END\n")

        # Edge consumption: time (resource 1)
        arc_times = inst["arc_times"]
        if has_tw and arc_times:
            f.write("EDGE_CONSUMPTION\n")
            for i, arc in enumerate(inst["arcs"]):
                time_val = arc_times[i]
                if time_val != 0:
                    f.write(f"1 {arc[0]} {arc[1]} {_scale_int(time_val, time_scale)}\n")
            f.write("END\n")

    # Sidecar file with the scales used, so compare_pathwyse.sh can divide Obj back.
    scales_path = str(outpath).rsplit(".", 1)[0] + ".scales"
    with open(scales_path, "w") as f:
        f.write(f"cost_scale={cost_scale}\n")
        f.write(f"time_scale={time_scale}\n")
        f.write(f"cap_scale={cap_scale}\n")


def convert_file(
    path: str | Path,
    outdir: str | Path,
    cost_scale: int | None,
    time_scale: int,
    cap_scale: int,
) -> str | None:
    """Convert a single instance file to Pathwyse format.

    cost_scale=None selects the per-extension default from DEFAULT_COST_SCALES.
    """
    filepath = Path(path)
    ext = filepath.suffix

    if ext == ".sppcc":
        inst = parse_sppcc(str(filepath))
    elif ext == ".vrp":
        inst = parse_vrp(str(filepath))
    elif ext == ".graph":
        inst = parse_graph(str(filepath))
    else:
        print(f"  Skipping unknown extension: {filepath}", file=sys.stderr)
        return None

    if cost_scale is None:
        cost_scale = DEFAULT_COST_SCALES[ext]

    # Determine output subdirectory based on parent/grandparent dir
    parent_name = filepath.parent.name
    outsubdir = Path(outdir) / parent_name
    os.makedirs(outsubdir, exist_ok=True)

    outpath = outsubdir / (filepath.stem + ".txt")
    write_pathwyse(inst, str(outpath), cost_scale, time_scale, cap_scale)
    return str(outpath)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Convert bgspprc instances to Pathwyse format")
    parser.add_argument("paths", nargs="+", help="Instance files or directories")
    parser.add_argument(
        "--outdir",
        default=None,
        help="Output directory (default: benchmarks/instances/pathwyse)",
    )
    parser.add_argument(
        "--cost-scale",
        type=int,
        default=None,
        help="Multiplier for EDGE_COST before int truncation. "
        "Default: per-extension auto (sppcc=1, vrp=1000, graph=1000).",
    )
    parser.add_argument(
        "--time-scale",
        type=int,
        default=DEFAULT_TIME_SCALE,
        help=f"Multiplier for TW bounds and arc times (default: {DEFAULT_TIME_SCALE})",
    )
    parser.add_argument(
        "--cap-scale",
        type=int,
        default=DEFAULT_CAP_SCALE,
        help=f"Multiplier for capacity bounds and demands (default: {DEFAULT_CAP_SCALE})",
    )
    args = parser.parse_args()

    if args.outdir is None:
        # Find benchmarks dir relative to script
        script_dir = Path(__file__).resolve().parent
        args.outdir = str(script_dir / "instances" / "pathwyse")

    converted = 0
    used_scales: dict[str, int] = {}  # ext -> cost_scale actually applied
    for path in args.paths:
        p = Path(path)
        if p.is_file():
            out = convert_file(
                str(p), args.outdir, args.cost_scale, args.time_scale, args.cap_scale
            )
            if out:
                print(f"  {p.name} -> {out}")
                converted += 1
                used_scales[p.suffix] = (
                    args.cost_scale
                    if args.cost_scale is not None
                    else DEFAULT_COST_SCALES[p.suffix]
                )
        elif p.is_dir():
            for ext in ("*.sppcc", "*.vrp", "*.graph"):
                for f in sorted(p.rglob(ext)):
                    out = convert_file(
                        str(f), args.outdir, args.cost_scale, args.time_scale, args.cap_scale
                    )
                    if out:
                        print(f"  {f.name} -> {out}")
                        converted += 1
                        used_scales[f.suffix] = (
                            args.cost_scale
                            if args.cost_scale is not None
                            else DEFAULT_COST_SCALES[f.suffix]
                        )
        else:
            print(f"  Warning: {path} not found, skipping", file=sys.stderr)

    if args.cost_scale is None:
        # Show the per-extension scales actually applied this run.
        scale_str = (
            ", ".join(f"{ext.lstrip('.')}={s}" for ext, s in sorted(used_scales.items())) or "auto"
        )
        cost_scale_label = f"auto ({scale_str})" if used_scales else "auto"
    else:
        cost_scale_label = str(args.cost_scale)
    print(
        f"\nConverted {converted} instances to {args.outdir} "
        f"(cost_scale={cost_scale_label}, time_scale={args.time_scale}, "
        f"cap_scale={args.cap_scale})"
    )


if __name__ == "__main__":
    main()
