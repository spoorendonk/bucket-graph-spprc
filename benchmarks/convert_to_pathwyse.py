#!/usr/bin/env python3
"""Convert bgspprc instance files (.sppcc, .vrp, .graph) to Pathwyse format.

Usage:
    python3 benchmarks/convert_to_pathwyse.py [--outdir DIR] [--cost-scale N]
        [--time-scale N] [--cap-scale N] PATH...

Arguments:
    PATH            Instance file or directory of instances.
    --outdir DIR    Output directory (default: benchmarks/instances/pathwyse).
    --cost-scale N  Multiplier applied to EDGE_COST before int truncation
                    (default: 1000000 = 6-digit precision).
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
import math
import os
import sys
from pathlib import Path


# Default scales. Pathwyse reads every field with std::stoi, so fractional
# values are lost unless we pre-multiply by a scale and round to int.
DEFAULT_COST_SCALE = 1_000_000
DEFAULT_TIME_SCALE = 1_000
DEFAULT_CAP_SCALE = 1


_INT32_MAX = 2_147_483_647


def _scale_int(value, scale):
    """Multiply by scale and round to nearest int. Pathwyse stores all fields
    as int (std::stoi → 32-bit), so callers must scale back on output side."""
    result = int(round(float(value) * scale))
    if abs(result) > _INT32_MAX:
        raise OverflowError(
            f"scaled value {result} exceeds int32 range "
            f"(value={value}, scale={scale}). Use a smaller --cost-scale."
        )
    return result


def parse_sppcc(filepath):
    """Parse SPPCC file (spprclib format) — matches instance_io.h load_sppcc."""
    with open(filepath) as f:
        lines = f.readlines()

    name = ""
    dimension = 0
    capacity = 0.0
    dist_matrix = []
    node_weights = []
    demands = []

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
            dist_matrix = [None] * dimension
            for r in range(dimension):
                vals = []
                while len(vals) < dimension:
                    i += 1
                    vals.extend(float(x) for x in lines[i].split())
                dist_matrix[r] = vals
        elif line.startswith("NODE_WEIGHT_SECTION"):
            node_weights = []
            while len(node_weights) < dimension:
                i += 1
                node_weights.extend(float(x) for x in lines[i].split())
        elif line.startswith("DEMAND_SECTION"):
            demands = [0.0] * dimension
            for r in range(dimension):
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

    arcs = []  # (from, to, cost, demand)
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
            f, t, c, d = arcs[idx]
            arcs[idx] = (f, t, c + depot_dual, d)

    return {
        "name": name,
        "n_vertices": n_vertices,
        "source": source,
        "sink": sink,
        "arcs": arcs,
        "cap_ub": capacity,
        "demands": demands + [0.0],  # sink has zero demand
        "has_tw": False,
        "tw_lb": None,
        "tw_ub": None,
        "arc_times": None,
    }


def parse_vrp(filepath):
    """Parse Roberti VRP file — matches instance_io.h load_roberti_vrp."""
    with open(filepath) as f:
        lines = f.readlines()

    name = ""
    dimension = 0
    capacity = 0.0
    x = []
    y = []
    demands = []
    profits = []
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

    def dist(i, j):
        dx = x[i] - x[j]
        dy = y[i] - y[j]
        return math.sqrt(dx * dx + dy * dy)

    dep0 = depot - 1  # 0-indexed depot
    N = dimension
    n_vertices = N + 1
    source = dep0
    sink = N

    arcs = []
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
        "demands": demands + [0.0],  # sink has zero demand
        "has_tw": False,
        "tw_lb": None,
        "tw_ub": None,
        "arc_times": None,
    }


def parse_graph(filepath):
    """Parse .graph file (rcspp_dataset format) — matches instance_io.h load_rcspp_graph."""
    with open(filepath) as f:
        lines = f.readlines()

    n_vertices = 0
    name = ""
    vertices = []
    arcs = []

    for line in lines:
        line = line.strip()
        if not line or line.startswith("c"):
            continue

        if line.startswith("p "):
            parts = line.split()
            name = parts[1]
            n_vertices = int(parts[2])
            vertices = [None] * n_vertices
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
    inst,
    outpath,
    cost_scale=DEFAULT_COST_SCALE,
    time_scale=DEFAULT_TIME_SCALE,
    cap_scale=DEFAULT_CAP_SCALE,
):
    """Write instance in Pathwyse format. All numeric fields are scaled and
    rounded to int since Pathwyse parses every value with std::stoi."""
    nv = inst["n_vertices"]
    source = inst["source"]
    sink = inst["sink"]
    has_tw = inst["has_tw"]

    # Auto-adapt cost_scale per instance so |cost * scale| fits in int32. Raw
    # costs on spprclib distance instances reach ~1e5; with the default 1e6
    # scale that overflows. Reducing the scale (powers of 10) keeps the sidecar
    # decode side simple and round-trips cleanly.
    orig_cost_scale = cost_scale
    max_abs_cost = max((abs(float(a[2])) for a in inst["arcs"]), default=0.0)
    if max_abs_cost > 0:
        while cost_scale > 1 and max_abs_cost * cost_scale > _INT32_MAX:
            new_scale = max(cost_scale // 10, 1)
            if new_scale == cost_scale:
                break
            cost_scale = new_scale
        if max_abs_cost * cost_scale > _INT32_MAX:
            raise OverflowError(
                f"cannot fit |cost|={max_abs_cost} in int32 even at scale=1"
            )
    if cost_scale != orig_cost_scale:
        print(
            f"  note: {inst['name']}: reduced cost_scale "
            f"{orig_cost_scale}→{cost_scale} (max|c|={max_abs_cost:.0f})",
            file=sys.stderr,
        )

    # Determine resources
    n_res = 0
    res_types = []
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
        f.write(f"DIRECTED : 1\n")
        f.write(f"CYCLIC : 1\n")
        f.write(f"ORIGIN : {source}\n")
        f.write(f"DESTINATION : {sink}\n")
        f.write(f"RESOURCES : {n_res}\n")
        f.write(f"RES_NAMES : {' '.join(str(i) for i in range(n_res))}\n")

        # Resource types
        f.write("RES_TYPE\n")
        for i, rt in enumerate(res_types):
            f.write(f"{i} {rt}\n")
        f.write("END\n")

        # Global resource bounds
        f.write("RES_BOUND\n")
        f.write(f"0 0 {_scale_int(inst['cap_ub'], cap_scale)}\n")
        if has_tw:
            # Global time window: [min_tw_lb, max_tw_ub]
            f.write(
                f"1 {_scale_int(inst['tw_lb'][source], time_scale)} "
                f"{_scale_int(inst['tw_ub'][sink], time_scale)}\n"
            )
        f.write("END\n")

        # Per-node resource bounds (time windows)
        if has_tw:
            f.write("RES_NODE_BOUND\n")
            for v in range(nv):
                f.write(
                    f"1 {v} {_scale_int(inst['tw_lb'][v], time_scale)} "
                    f"{_scale_int(inst['tw_ub'][v], time_scale)}\n"
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
        for arc in inst["arcs"]:
            f.write(f"{arc[0]} {arc[1]} {_scale_int(arc[2], cost_scale)}\n")
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
        if has_tw and inst["arc_times"]:
            f.write("EDGE_CONSUMPTION\n")
            for i, arc in enumerate(inst["arcs"]):
                time_val = inst["arc_times"][i]
                if time_val != 0:
                    f.write(f"1 {arc[0]} {arc[1]} {_scale_int(time_val, time_scale)}\n")
            f.write("END\n")

    # Sidecar file with the scales used, so run_pathwyse.sh can divide Obj back.
    scales_path = str(outpath).rsplit(".", 1)[0] + ".scales"
    with open(scales_path, "w") as f:
        f.write(f"cost_scale={cost_scale}\n")
        f.write(f"time_scale={time_scale}\n")
        f.write(f"cap_scale={cap_scale}\n")


def convert_file(filepath, outdir, cost_scale, time_scale, cap_scale):
    """Convert a single instance file to Pathwyse format."""
    filepath = Path(filepath)
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

    # Determine output subdirectory based on parent/grandparent dir
    parent_name = filepath.parent.name
    outsubdir = Path(outdir) / parent_name
    os.makedirs(outsubdir, exist_ok=True)

    outpath = outsubdir / (filepath.stem + ".txt")
    write_pathwyse(inst, str(outpath), cost_scale, time_scale, cap_scale)
    return str(outpath)


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert bgspprc instances to Pathwyse format"
    )
    parser.add_argument("paths", nargs="+", help="Instance files or directories")
    parser.add_argument(
        "--outdir",
        default=None,
        help="Output directory (default: benchmarks/instances/pathwyse)",
    )
    parser.add_argument(
        "--cost-scale",
        type=int,
        default=DEFAULT_COST_SCALE,
        help=f"Multiplier for EDGE_COST before int truncation (default: {DEFAULT_COST_SCALE})",
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
    for path in args.paths:
        p = Path(path)
        if p.is_file():
            out = convert_file(
                str(p), args.outdir, args.cost_scale, args.time_scale, args.cap_scale
            )
            if out:
                print(f"  {p.name} -> {out}")
                converted += 1
        elif p.is_dir():
            for ext in ("*.sppcc", "*.vrp", "*.graph"):
                for f in sorted(p.rglob(ext)):
                    out = convert_file(
                        str(f), args.outdir, args.cost_scale, args.time_scale, args.cap_scale
                    )
                    if out:
                        print(f"  {f.name} -> {out}")
                        converted += 1
        else:
            print(f"  Warning: {path} not found, skipping", file=sys.stderr)

    print(
        f"\nConverted {converted} instances to {args.outdir} "
        f"(cost_scale={args.cost_scale}, time_scale={args.time_scale}, "
        f"cap_scale={args.cap_scale})"
    )


if __name__ == "__main__":
    main()
