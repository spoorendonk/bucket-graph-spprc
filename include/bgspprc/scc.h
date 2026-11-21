#pragma once

#include <algorithm>
#include <utility>
#include <vector>

namespace bgspprc::detail {

/// Tarjan's strongly connected components over nodes 0..n-1, without
/// recursion and without a copy of the graph (#162). succ(v, k) gives the
/// k-th successor of v, or a negative value once k is past the last; it is
/// called once per edge and once more per node, in increasing k. Nodes are
/// rooted in increasing order and successors taken in succ's order, exactly
/// as the recursive formulation does, so the numbering is that one's: comp[v]
/// is v's component, and components are numbered in reverse topological
/// order (an edge u -> w has comp[u] >= comp[w]). Returns the number of
/// components. Storage is O(n) whatever the edge count, and the DFS depth
/// lives on the heap, so a path through millions of nodes is fine.
template <typename Succ>
int tarjan_scc(int n, Succ&& succ, std::vector<int>& comp) {
    comp.assign(n, -1);
    std::vector<int> index(n, -1);
    std::vector<int> lowlink(n, -1);
    // Nodes visited and not yet given a component: Tarjan's stack. A
    // visited node is on it exactly while its comp is still -1.
    std::vector<int> stack;
    // The DFS path: each node with the index of its next successor.
    std::vector<std::pair<int, int>> path;
    int next_index = 0;
    int n_comp = 0;

    auto visit = [&](int v) {
        index[v] = lowlink[v] = next_index++;
        stack.push_back(v);
        path.emplace_back(v, 0);
    };

    for (int root = 0; root < n; ++root) {
        if (index[root] != -1) {
            continue;
        }
        visit(root);
        while (!path.empty()) {
            const int v = path.back().first;
            const int w = succ(v, path.back().second);
            if (w >= 0) {
                ++path.back().second;
                if (index[w] == -1) {
                    visit(w);
                } else if (comp[w] == -1) {
                    lowlink[v] = std::min(lowlink[v], index[w]);
                }
                continue;
            }
            path.pop_back();
            if (lowlink[v] == index[v]) {
                int u = -1;
                do {
                    u = stack.back();
                    stack.pop_back();
                    comp[u] = n_comp;
                } while (u != v);
                ++n_comp;
            }
            if (!path.empty()) {
                const int parent = path.back().first;
                lowlink[parent] = std::min(lowlink[parent], lowlink[v]);
            }
        }
    }
    return n_comp;
}

}  // namespace bgspprc::detail
