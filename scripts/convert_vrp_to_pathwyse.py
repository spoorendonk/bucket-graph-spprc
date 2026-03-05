#!/usr/bin/env python3
"""Convert Roberti .vrp files to PathWyse format with integer distances.

Roberti VRP format: CVRP with EUC_2D coordinates, demands, profits.
PathWyse uses integer distances: floor(sqrt(dx^2+dy^2) * scaling).

Usage: convert_vrp_to_pathwyse.py <input.vrp> <output.sppcc> [scaling]
  scaling defaults to 1000.
"""

import sys
import math


def parse_vrp(path):
    name = ""
    dimension = 0
    capacity = 0
    coords = {}
    demands = {}
    profits = {}
    depot = 1

    section = None

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.startswith("NAME"):
                parts = line.split(":", 1)
                if len(parts) > 1:
                    name = parts[1].strip()
            elif line.startswith("DIMENSION"):
                parts = line.split(":", 1)
                if len(parts) > 1:
                    dimension = int(parts[1].strip())
            elif line.startswith("CAPACITY"):
                parts = line.split(":", 1)
                if len(parts) > 1:
                    capacity = float(parts[1].strip())
            elif line.startswith("NODE_COORD_SECTION"):
                section = "coords"
            elif line.startswith("DEMAND_SECTION"):
                section = "demands"
            elif line.startswith("DEPOT_SECTION"):
                section = "depot"
            elif line.startswith("PROFIT_SECTION"):
                section = "profit"
            elif line.startswith("EOF") or line.startswith("EDGE_WEIGHT"):
                section = None
            elif section == "coords":
                parts = line.split()
                if len(parts) >= 3:
                    idx = int(parts[0]) - 1
                    coords[idx] = (float(parts[1]), float(parts[2]))
            elif section == "demands":
                parts = line.split()
                if len(parts) >= 2:
                    idx = int(parts[0]) - 1
                    demands[idx] = float(parts[1])
            elif section == "depot":
                try:
                    d = int(line)
                    if d > 0:
                        depot = d
                except ValueError:
                    pass
            elif section == "profit":
                parts = line.split()
                if len(parts) >= 2:
                    idx = int(parts[0]) - 1
                    profits[idx] = float(parts[1])

    return name, dimension, capacity, coords, demands, profits, depot


def write_pathwyse(outpath, name, dimension, capacity, coords, demands, profits, depot, scaling):
    dep0 = depot - 1  # 0-indexed
    N = dimension

    def dist_int(i, j):
        dx = coords[i][0] - coords[j][0]
        dy = coords[i][1] - coords[j][1]
        return int(math.floor(math.sqrt(dx * dx + dy * dy) * scaling))

    with open(outpath, "w") as f:
        f.write(f"NAME : {name} path problem\n")
        f.write(f"COMMENT : (converted from Roberti VRP, scaling={scaling})\n")
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

        # EDGE_COST: N x N integer distances * scaling
        f.write(f"EDGE_COST\n")
        for i in range(N):
            for j in range(N):
                cost = dist_int(i, j)
                f.write(f"{i} {j} {cost}\n")
        f.write(f"END\n")

        # NODE_COST: -profit[i] * scaling (integer)
        f.write(f"NODE_COST\n")
        for i in range(N):
            p = profits.get(i, 0.0)
            cost = int(round(-p * scaling))
            f.write(f"{i} {cost}\n")
        f.write(f"END\n")

        # NODE_CONSUMPTION
        f.write(f"NODE_CONSUMPTION\n")
        for i in range(N):
            d = int(round(demands.get(i, 0)))
            f.write(f"0 {i} {d}\n")
        f.write(f"END\n")


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <input.vrp> <output.sppcc> [scaling]")
        sys.exit(1)

    inpath = sys.argv[1]
    outpath = sys.argv[2]
    scaling = int(sys.argv[3]) if len(sys.argv) > 3 else 1000

    name, dimension, capacity, coords, demands, profits, depot = parse_vrp(inpath)
    write_pathwyse(outpath, name, dimension, capacity, coords, demands, profits, depot, scaling)
    print(f"Converted {inpath} -> {outpath} ({dimension} nodes, scaling={scaling})")


if __name__ == "__main__":
    main()
