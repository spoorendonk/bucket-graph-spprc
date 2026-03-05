#!/usr/bin/env python3
"""Convert TSPLIB .sppcc files to PathWyse format.

TSPLIB format has: DIMENSION, CAPACITY, EDGE_WEIGHT_SECTION (full matrix),
NODE_WEIGHT_SECTION, DEMAND_SECTION.

PathWyse format has: SIZE, DIRECTED, CYCLIC, RESOURCES, EDGE_COST,
NODE_COST, NODE_CONSUMPTION.

PathWyse uses SIZE=N nodes (0..N-1). PathWyse internally creates the
destination node N. Edge costs and node costs are raw integer values
from the original instance.
"""

import sys
import re


def parse_sppcc(path):
    with open(path) as f:
        lines = f.readlines()

    name = ""
    dimension = 0
    capacity = 0
    dist_matrix = []
    node_weights = []
    demands = []

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("NAME"):
            m = re.search(r":\s*(.*)", line)
            if m:
                name = m.group(1).strip()
        elif line.startswith("DIMENSION"):
            m = re.search(r":\s*(\d+)", line)
            if m:
                dimension = int(m.group(1))
        elif line.startswith("CAPACITY"):
            m = re.search(r":\s*(\S+)", line)
            if m:
                capacity = float(m.group(1))
        elif line.startswith("EDGE_WEIGHT_SECTION"):
            # Read full matrix: dimension x dimension values
            vals = []
            i += 1
            while len(vals) < dimension * dimension and i < len(lines):
                vals.extend(lines[i].split())
                i += 1
            i -= 1  # will be incremented below
            dist_matrix = []
            for r in range(dimension):
                row = [float(vals[r * dimension + c]) for c in range(dimension)]
                dist_matrix.append(row)
        elif line.startswith("NODE_WEIGHT_SECTION"):
            vals = []
            i += 1
            while len(vals) < dimension and i < len(lines):
                vals.extend(lines[i].split())
                i += 1
            i -= 1
            node_weights = [float(v) for v in vals]
        elif line.startswith("DEMAND_SECTION"):
            demands = [0.0] * dimension
            i += 1
            for _ in range(dimension):
                if i >= len(lines):
                    break
                parts = lines[i].split()
                if len(parts) >= 2:
                    idx = int(parts[0]) - 1  # 1-indexed to 0-indexed
                    demands[idx] = float(parts[1])
                i += 1
            i -= 1
        i += 1

    return name, dimension, capacity, dist_matrix, node_weights, demands


def write_pathwyse(outpath, name, dimension, capacity, dist_matrix, node_weights, demands):
    N = dimension  # original nodes: 0..N-1
    # PathWyse uses SIZE=N. It internally creates destination node N.

    with open(outpath, "w") as f:
        # Header
        # Append "path problem" only if not already present
        if "path problem" not in name:
            name_str = f"{name} path problem"
        else:
            name_str = name
        f.write(f"NAME : {name_str}\n")
        f.write(f"COMMENT : (converted from TSPLIB)\n")
        f.write(f"SIZE : {N}\n")
        f.write(f"DIRECTED : 1\n")
        f.write(f"CYCLIC : 1\n")
        f.write(f"RESOURCES : 1\n")
        f.write(f"RES_NAMES : 0\n")
        f.write(f"RES_TYPE\n")
        f.write(f"0 CAP\n")
        f.write(f"END\n")
        f.write(f"RES_BOUND\n")
        f.write(f"0 0 {int(capacity)}\n")
        f.write(f"END\n")

        # EDGE_COST: N x N matrix (raw distances as integers)
        f.write(f"EDGE_COST\n")
        for i in range(N):
            for j in range(N):
                cost = int(round(dist_matrix[i][j]))
                f.write(f"{i} {j} {cost}\n")
        f.write(f"END\n")

        # NODE_COST: node_weight per node (integer)
        f.write(f"NODE_COST\n")
        for i in range(N):
            cost = int(round(node_weights[i]))
            f.write(f"{i} {cost}\n")
        f.write(f"END\n")

        # NODE_CONSUMPTION: demand at each node for resource 0
        f.write(f"NODE_CONSUMPTION\n")
        for i in range(N):
            d = int(round(demands[i]))
            f.write(f"0 {i} {d}\n")
        f.write(f"END\n")


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <input.sppcc> <output.sppcc>")
        print(f"       {sys.argv[0]} --validate <reference.sppcc> <converted.sppcc>")
        sys.exit(1)

    if sys.argv[1] == "--validate":
        # Compare two PathWyse files
        with open(sys.argv[2]) as f:
            ref = f.read()
        with open(sys.argv[3]) as f:
            conv = f.read()
        if ref.strip() == conv.strip():
            print("MATCH: files are identical")
        else:
            # Compare line by line
            ref_lines = ref.strip().split("\n")
            conv_lines = conv.strip().split("\n")
            diffs = 0
            for i, (r, c) in enumerate(zip(ref_lines, conv_lines)):
                if r.strip() != c.strip():
                    if diffs < 10:
                        print(f"  line {i+1}: ref={r.strip()!r}  conv={c.strip()!r}")
                    diffs += 1
            if len(ref_lines) != len(conv_lines):
                print(f"  line count: ref={len(ref_lines)} conv={len(conv_lines)}")
            print(f"DIFF: {diffs} lines differ")
        sys.exit(0)

    inpath = sys.argv[1]
    outpath = sys.argv[2]

    name, dimension, capacity, dist_matrix, node_weights, demands = parse_sppcc(inpath)
    write_pathwyse(outpath, name, dimension, capacity, dist_matrix, node_weights, demands)
    print(f"Converted {inpath} -> {outpath} ({dimension} nodes)")


if __name__ == "__main__":
    main()
