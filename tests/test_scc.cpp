#include <algorithm>
#include <bgspprc/scc.h>
#include <cstdint>
#include <doctest/doctest.h>
#include <random>
#include <vector>

using bgspprc::detail::tarjan_scc;

namespace {

using Graph = std::vector<std::vector<int>>;

/// The recursive Tarjan compute_sccs used before #162, over a materialized
/// adjacency: the numbering tarjan_scc must reproduce exactly.
struct RecursiveTarjan {
    const Graph& adj;
    std::vector<int> index;
    std::vector<int> lowlink;
    std::vector<bool> on_stack;
    std::vector<int> stack;
    std::vector<int> comp;
    int idx = 0;
    int n_comp = 0;

    explicit RecursiveTarjan(const Graph& g)
        : adj(g),
          index(g.size(), -1),
          lowlink(g.size(), -1),
          on_stack(g.size(), false),
          comp(g.size(), -1) {
        for (int v = 0; v < static_cast<int>(g.size()); ++v) {
            if (index[v] == -1) {
                strongconnect(v);
            }
        }
    }

    void strongconnect(int v) {
        index[v] = lowlink[v] = idx++;
        stack.push_back(v);
        on_stack[v] = true;
        for (int w : adj[v]) {
            if (index[w] == -1) {
                strongconnect(w);
                lowlink[v] = std::min(lowlink[v], lowlink[w]);
            } else if (on_stack[w]) {
                lowlink[v] = std::min(lowlink[v], index[w]);
            }
        }
        if (lowlink[v] == index[v]) {
            int w = -1;
            do {
                w = stack.back();
                stack.pop_back();
                on_stack[w] = false;
                comp[w] = n_comp;
            } while (w != v);
            ++n_comp;
        }
    }
};

/// tarjan_scc over adj, counting succ calls: hits return an edge, all
/// include the one past-the-end call per node.
struct CountedRun {
    std::vector<int> comp;
    int n_comp = 0;
    std::int64_t hits = 0;
    std::int64_t calls = 0;
};

CountedRun run_iterative(const Graph& adj) {
    CountedRun r;
    r.n_comp = tarjan_scc(
        static_cast<int>(adj.size()),
        [&](int v, int k) {
            ++r.calls;
            if (k < static_cast<int>(adj[v].size())) {
                ++r.hits;
                return adj[v][k];
            }
            return -1;
        },
        r.comp);
    return r;
}

Graph random_graph(std::mt19937& rng, int n, double p) {
    std::bernoulli_distribution edge(p);
    Graph g(n);
    for (int u = 0; u < n; ++u) {
        for (int v = 0; v < n; ++v) {
            if (edge(rng)) {
                g[u].push_back(v);  // self loops included
            }
        }
    }
    // Shuffle successor order and add a duplicate now and then: the
    // numbering depends on that order, so cover more than ascending lists.
    for (auto& succ : g) {
        std::ranges::shuffle(succ, rng);
        if (!succ.empty() && edge(rng)) {
            succ.push_back(succ.front());
        }
    }
    return g;
}

/// reach[u][v]: v reachable from u (reflexive), by Floyd–Warshall.
std::vector<std::vector<bool>> reachability(const Graph& g) {
    const int n = static_cast<int>(g.size());
    std::vector<std::vector<bool>> reach(n, std::vector<bool>(n, false));
    for (int u = 0; u < n; ++u) {
        reach[u][u] = true;
        for (int v : g[u]) {
            reach[u][v] = true;
        }
    }
    for (int k = 0; k < n; ++k) {
        for (int u = 0; u < n; ++u) {
            if (reach[u][k]) {
                for (int v = 0; v < n; ++v) {
                    if (reach[k][v]) {
                        reach[u][v] = true;
                    }
                }
            }
        }
    }
    return reach;
}

}  // namespace

TEST_CASE("tarjan_scc matches the recursive Tarjan and the SCC partition") {
    std::mt19937 rng(162);
    for (int trial = 0; trial < 300; ++trial) {
        const int n = 1 + (trial % 40);
        const double p = (trial % 5 == 0) ? 0.3 : 2.0 / n;
        const Graph g = random_graph(rng, n, p);
        std::int64_t m = 0;
        for (const auto& succ : g) {
            m += static_cast<std::int64_t>(succ.size());
        }

        const RecursiveTarjan ref(g);
        const CountedRun it = run_iterative(g);
        CAPTURE(trial);
        CHECK(it.n_comp == ref.n_comp);
        CHECK(it.comp == ref.comp);

        // Each edge read once, plus one past-the-end call per node.
        CHECK(it.hits == m);
        CHECK(it.calls == m + n);

        // Partition: same component iff mutually reachable.
        const auto reach = reachability(g);
        for (int u = 0; u < n; ++u) {
            REQUIRE(it.comp[u] >= 0);
            REQUIRE(it.comp[u] < it.n_comp);
            for (int v = 0; v < n; ++v) {
                CHECK((it.comp[u] == it.comp[v]) == (reach[u][v] && reach[v][u]));
            }
        }
        // Reverse topological numbering: an edge never goes to a higher id.
        for (int u = 0; u < n; ++u) {
            for (int v : g[u]) {
                CHECK(it.comp[u] >= it.comp[v]);
            }
        }
    }
}

TEST_CASE("tarjan_scc handles a deep chain without recursion") {
    // Two million nodes on one DFS path: the recursive version needs a stack
    // frame per node (a std::function frame each), far past a default stack.
    constexpr int kN = 2'000'000;
    std::vector<int> comp;
    std::int64_t hits = 0;
    std::int64_t calls = 0;

    SUBCASE("acyclic chain: one component per node, sink first") {
        const int n_comp = tarjan_scc(
            kN,
            [&](int v, int k) {
                ++calls;
                if (k == 0 && v + 1 < kN) {
                    ++hits;
                    return v + 1;
                }
                return -1;
            },
            comp);
        CHECK(n_comp == kN);
        CHECK(hits == kN - 1);
        CHECK(calls == (kN - 1) + kN);
        bool reverse_topo = true;
        for (int v = 0; v < kN; ++v) {
            reverse_topo = reverse_topo && (comp[v] == kN - 1 - v);
        }
        CHECK(reverse_topo);
    }

    SUBCASE("chain closed into a cycle: one component") {
        const int n_comp = tarjan_scc(
            kN,
            [&](int v, int k) {
                ++calls;
                if (k == 0) {
                    ++hits;
                    return (v + 1) % kN;
                }
                return -1;
            },
            comp);
        CHECK(n_comp == 1);
        CHECK(hits == kN);
        CHECK(calls == 2 * static_cast<std::int64_t>(kN));
        CHECK(std::ranges::all_of(comp, [](int c) { return c == 0; }));
    }
}

TEST_CASE("tarjan_scc on an empty graph") {
    std::vector<int> comp{7, 7};
    CHECK(tarjan_scc(0, [](int, int) { return -1; }, comp) == 0);
    CHECK(comp.empty());
}
