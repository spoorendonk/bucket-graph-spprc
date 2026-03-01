#pragma once

/// Parsers for benchmark instance formats:
///   - SPPCC (spprclib): TSPLIB-like, complete graph, capacity + demands
///   - RCSPP (.graph):   DIMACS-like, sparse arcs, time windows + capacity

#include <bgspprc/problem_view.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace bgspprc::io {

/// Fully-owned instance data (parsers fill this, then we build a ProblemView).
struct Instance {
    int n_vertices = 0;  // includes source and sink
    int source = 0;
    int sink = 0;

    std::vector<int> arc_from;
    std::vector<int> arc_to;
    std::vector<double> arc_cost;        // reduced cost (with duals baked in)
    std::vector<double> arc_time;        // time consumption
    std::vector<double> arc_demand;      // demand consumption

    // Per-vertex bounds
    std::vector<double> tw_lb;           // time window lower bound
    std::vector<double> tw_ub;           // time window upper bound
    std::vector<double> cap_lb;          // capacity lower bound (always 0)
    std::vector<double> cap_ub;          // capacity upper bound

    int n_main_resources = 0;            // 1 (time-only) or 2 (time + capacity)

    // Owning storage for ProblemView pointers
    std::vector<const double*> arc_res_ptrs;
    std::vector<const double*> v_lb_ptrs;
    std::vector<const double*> v_ub_ptrs;

    std::string name;

    ProblemView problem_view() {
        arc_res_ptrs.clear();
        v_lb_ptrs.clear();
        v_ub_ptrs.clear();

        // Always have capacity as resource 0
        arc_res_ptrs.push_back(arc_demand.data());
        v_lb_ptrs.push_back(cap_lb.data());
        v_ub_ptrs.push_back(cap_ub.data());

        if (!arc_time.empty()) {
            arc_res_ptrs.push_back(arc_time.data());
            v_lb_ptrs.push_back(tw_lb.data());
            v_ub_ptrs.push_back(tw_ub.data());
        }

        ProblemView pv;
        pv.n_vertices = n_vertices;
        pv.source = source;
        pv.sink = sink;
        pv.n_arcs = static_cast<int>(arc_from.size());
        pv.arc_from = arc_from.data();
        pv.arc_to = arc_to.data();
        pv.arc_base_cost = arc_cost.data();
        pv.n_resources = static_cast<int>(arc_res_ptrs.size());
        pv.arc_resource = arc_res_ptrs.data();
        pv.vertex_lb = v_lb_ptrs.data();
        pv.vertex_ub = v_ub_ptrs.data();
        pv.n_main_resources = n_main_resources;
        return pv;
    }
};

/// Parse SPPCC file (spprclib format).
///
/// Tour-based: depot is both source and sink.
/// We model this with source=0 and sink=N (a copy of depot with zero demand).
/// Arc cost for (i,j) = distance[i][j] + node_weight[j]
///   (node_weight is negative for customers = profit subtracted)
///
/// The depot's node_weight is the vehicle dual — applied as initial label cost.
inline Instance load_sppcc(const std::string& path) {
    Instance inst;
    std::ifstream file(path);
    assert(file.is_open());

    int dimension = 0;
    double capacity = 0;
    std::vector<std::vector<double>> dist_matrix;
    std::vector<double> node_weights;
    std::vector<double> demands;

    std::string line;
    while (std::getline(file, line)) {
        if (line.starts_with("NAME")) {
            auto pos = line.find(':');
            if (pos != std::string::npos)
                inst.name = line.substr(pos + 2);
        } else if (line.starts_with("DIMENSION")) {
            std::sscanf(line.c_str(), "DIMENSION : %d", &dimension);
        } else if (line.starts_with("CAPACITY")) {
            std::sscanf(line.c_str(), "CAPACITY : %lf", &capacity);
        } else if (line.starts_with("EDGE_WEIGHT_SECTION")) {
            dist_matrix.resize(dimension);
            for (int i = 0; i < dimension; ++i) {
                dist_matrix[i].resize(dimension);
                for (int j = 0; j < dimension; ++j) {
                    file >> dist_matrix[i][j];
                }
            }
        } else if (line.starts_with("NODE_WEIGHT_SECTION")) {
            node_weights.resize(dimension);
            for (int i = 0; i < dimension; ++i) {
                file >> node_weights[i];
            }
        } else if (line.starts_with("DEMAND_SECTION")) {
            demands.resize(dimension, 0);
            for (int i = 0; i < dimension; ++i) {
                int id;
                double d;
                file >> id >> d;
                demands[id - 1] = d;  // 1-indexed → 0-indexed
            }
        }
    }

    // Build instance: source=0 (depot), sink=dimension (copy of depot)
    // Vertices: 0..dimension-1 are original, dimension is the sink (depot copy)
    int N = dimension;
    inst.n_vertices = N + 1;
    inst.source = 0;
    inst.sink = N;

    // Profits: profit[i] = -node_weight[i]
    // Arc cost for (i,j) customer: distance[i][j] + node_weight[j]
    //                              = distance[i][j] - profit[j]

    // Build arcs: source→customers, customer→customer, customer→sink
    for (int i = 0; i < N; ++i) {
        for (int j = 1; j < N; ++j) {
            if (i == j) continue;
            inst.arc_from.push_back(i);
            inst.arc_to.push_back(j);
            inst.arc_cost.push_back(dist_matrix[i][j] + node_weights[j]);
            inst.arc_demand.push_back(demands[j]);
        }
        // Arc from customer i to sink (= return to depot)
        if (i > 0) {  // not from depot to itself
            inst.arc_from.push_back(i);
            inst.arc_to.push_back(N);  // sink
            inst.arc_cost.push_back(dist_matrix[i][0]);  // distance back to depot
            inst.arc_demand.push_back(0);  // sink has no demand
        }
    }

    // Capacity bounds per vertex
    inst.cap_lb.assign(inst.n_vertices, 0.0);
    inst.cap_ub.assign(inst.n_vertices, capacity);

    // No time windows in SPPCC — use capacity as the only main resource
    inst.n_main_resources = 1;

    // Initial cost from depot node weight (vehicle dual)
    // We bake this into arcs leaving source:
    // Actually it should be a constant added once. Since it's the same for all
    // paths, we add it to all arcs leaving the source.
    // Wait: node_weight[0] is already large positive (vehicle dual). The optimal
    // path cost = sum of (distance + node_weight_target) + initial depot weight.
    // For the depot: we don't visit it as a "target" (we leave from it), so
    // depot node weight = vehicle dual added once to the path cost.
    // Model: source label starts with cost = node_weight[0] (depot dual).
    // Since our solver starts labels at cost=0, bake it into one dummy arc:
    // Add node_weight[0] to cost of all arcs from source.
    double depot_dual = node_weights[0];
    int n_arcs = static_cast<int>(inst.arc_from.size());
    for (int a = 0; a < n_arcs; ++a) {
        if (inst.arc_from[a] == 0) {
            inst.arc_cost[a] += depot_dual;
        }
    }

    return inst;
}

/// Parse .graph file (rcspp_dataset format).
///
/// Sparse graph with explicit arcs, time windows, capacity, neighborhoods.
/// Vertex 0 = source, last vertex = sink.
inline Instance load_rcspp_graph(const std::string& path) {
    Instance inst;
    std::ifstream file(path);
    assert(file.is_open());

    int n_vertices = 0, n_edges = 0, ng_size = 0;

    struct Vertex {
        double a, b, d, Q;
    };
    std::vector<Vertex> vertices;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == 'c') continue;

        if (line[0] == 'p') {
            // p name n_vertices n_edges ng_size
            char name[256];
            std::sscanf(line.c_str(), "p %255s %d %d %d",
                        name, &n_vertices, &n_edges, &ng_size);
            inst.name = name;
            vertices.resize(n_vertices);
        } else if (line[0] == 'v') {
            // v id a b d Q
            int id;
            double a, b, d, Q;
            std::sscanf(line.c_str(), "v %d %lf %lf %lf %lf",
                        &id, &a, &b, &d, &Q);
            assert(id >= 0 && id < n_vertices);
            vertices[id] = {a, b, d, Q};
        } else if (line[0] == 'e') {
            // e id source target cost time
            int eid, src, tgt;
            double cost, time;
            std::sscanf(line.c_str(), "e %d %d %d %lf %lf",
                        &eid, &src, &tgt, &cost, &time);
            inst.arc_from.push_back(src);
            inst.arc_to.push_back(tgt);
            inst.arc_cost.push_back(cost);
            inst.arc_time.push_back(time);
            inst.arc_demand.push_back(vertices[tgt].d);
        } else if (line[0] == 'n') {
            // n vertex neighbor1 neighbor2 ...
            // (neighborhoods — we'll use these later for ng-path resource)
        }
    }

    inst.n_vertices = n_vertices;
    inst.source = 0;
    inst.sink = n_vertices - 1;

    // Build per-vertex bounds
    inst.tw_lb.resize(n_vertices);
    inst.tw_ub.resize(n_vertices);
    inst.cap_lb.assign(n_vertices, 0.0);
    inst.cap_ub.resize(n_vertices);

    for (int i = 0; i < n_vertices; ++i) {
        inst.tw_lb[i] = vertices[i].a;
        inst.tw_ub[i] = vertices[i].b;
        inst.cap_ub[i] = vertices[i].Q;
    }

    // Two main resources: capacity + time
    inst.n_main_resources = 2;

    return inst;
}

}  // namespace bgspprc::io
