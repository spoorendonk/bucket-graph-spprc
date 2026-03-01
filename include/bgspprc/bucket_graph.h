#pragma once

#include "arc.h"
#include "bucket.h"
#include "label.h"
#include "problem_view.h"
#include "r1c.h"
#include "resource.h"
#include "types.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <span>
#include <stack>
#include <tuple>
#include <vector>

namespace bgspprc {

/// Core bucket graph engine for SPPRC labeling.
///
/// Template parameter Pack is a ResourcePack<Rs...>.
template <typename Pack>
class BucketGraph {
public:
    struct Options {
        std::array<double, 2> bucket_steps = {1.0, 1.0};
        int max_paths = 100;
        double tolerance = -1e-6;
        bool bidirectional = false;  // mono-directional first
    };

    BucketGraph(const ProblemView& pv, Pack pack, Options opts = {})
        : pv_(pv), pack_(std::move(pack)), opts_(opts) {
        adj_.build(pv_);
        n_main_ = std::min(pv_.n_main_resources, 2);
    }

    /// Build the bucket graph structure. Call once or after step size changes.
    void build() {
        build_buckets();
        build_bucket_arcs();
        compute_sccs();
    }

    /// Update reduced costs (fast, O(1) — just stores the pointer).
    void update_arc_costs(const double* reduced_costs) {
        reduced_costs_ = reduced_costs;
    }

    /// Set R1C cuts (rebuilds masks, resizes label R1C state).
    void set_r1c_cuts(std::span<const R1Cut> cuts) {
        r1c_.set_cuts(cuts, pv_.n_vertices, pv_.n_arcs);
        pool_.set_r1c_words(r1c_.n_words());
    }

    /// Solve: mono-directional forward labeling.
    /// Returns labels at sink with negative reduced cost.
    struct Path {
        std::vector<int> vertices;
        std::vector<int> arcs;
        double reduced_cost;
        double original_cost;
    };

    std::vector<Path> solve() {
        pool_.clear();
        reset_labels();

        // Initialize source label
        auto* src_label = pool_.allocate();
        src_label->vertex = pv_.source;
        src_label->cost = 0.0;
        src_label->real_cost = 0.0;
        src_label->q = {0.0, 0.0};
        for (int r = 0; r < n_main_; ++r) {
            src_label->q[r] = pv_.vertex_lb[r][pv_.source];
        }
        src_label->resource_states = pack_.init_states(Direction::Forward);
        if (r1c_.has_cuts()) {
            r1c_.init_state({src_label->r1c_states,
                             static_cast<std::size_t>(src_label->n_r1c_words)});
        }

        int src_bucket = vertex_bucket_index(pv_.source, src_label->q);
        src_label->bucket = src_bucket;
        bucket_labels_[src_bucket].push_back(src_label);

        // Process SCCs in topological order
        for (int scc : scc_topo_order_) {
            process_scc(scc);
        }

        // Extract paths at sink
        return extract_paths();
    }

    // Accessors
    int n_buckets() const { return static_cast<int>(buckets_.size()); }
    const Bucket& bucket(int i) const { return buckets_[i]; }
    const std::vector<Bucket>& buckets() const { return buckets_; }

    /// Arc elimination: remove bucket arcs where no negative-cost path can use them.
    void eliminate_arcs(double theta) {
        for (auto& b : buckets_) {
            std::erase_if(b.bucket_arcs, [&](const BucketArc& ba) {
                return b.c_best + completion_bound(ba.to_bucket) > theta + EPS;
            });
        }
    }

    /// Bucket fixing: mark entire buckets as fixed if no useful labels can exist.
    void fix_buckets(double theta) {
        for (auto& b : buckets_) {
            if (b.c_best > theta + EPS) {
                b.fixed = true;
            }
        }
    }

    void reset_elimination() {
        for (auto& b : buckets_) {
            b.fixed = false;
        }
        // Rebuild arcs
        build_bucket_arcs();
    }

private:
    // ── Bucket construction ──

    void build_buckets() {
        buckets_.clear();
        vertex_bucket_start_.assign(pv_.n_vertices, -1);
        vertex_n_buckets_.assign(pv_.n_vertices, {0, 0});

        int total = 0;
        for (int v = 0; v < pv_.n_vertices; ++v) {
            vertex_bucket_start_[v] = total;

            std::array<int, 2> nb = {1, 1};
            for (int r = 0; r < n_main_; ++r) {
                double range = pv_.vertex_ub[r][v] - pv_.vertex_lb[r][v];
                nb[r] = std::max(1, static_cast<int>(
                    std::ceil(range / opts_.bucket_steps[r])));
            }
            vertex_n_buckets_[v] = nb;

            for (int k1 = 0; k1 < nb[0]; ++k1) {
                for (int k2 = 0; k2 < nb[1]; ++k2) {
                    Bucket b;
                    b.vertex = v;

                    for (int r = 0; r < n_main_; ++r) {
                        int k = (r == 0) ? k1 : k2;
                        b.lb[r] = pv_.vertex_lb[r][v] + k * opts_.bucket_steps[r];
                        b.ub[r] = std::min(
                            b.lb[r] + opts_.bucket_steps[r],
                            pv_.vertex_ub[r][v]);
                    }
                    // For unused main resources (n_main_ < 2), set full range
                    for (int r = n_main_; r < 2; ++r) {
                        b.lb[r] = 0.0;
                        b.ub[r] = INF;
                    }

                    buckets_.push_back(std::move(b));
                }
            }
            total = static_cast<int>(buckets_.size());
        }

        bucket_labels_.resize(buckets_.size());
    }

    /// Get bucket index for a vertex and main resource values.
    int vertex_bucket_index(int vertex, const std::array<double, 2>& q) const {
        int start = vertex_bucket_start_[vertex];
        auto& nb = vertex_n_buckets_[vertex];

        std::array<int, 2> k = {0, 0};
        for (int r = 0; r < n_main_; ++r) {
            double offset = q[r] - pv_.vertex_lb[r][vertex];
            k[r] = std::min(
                static_cast<int>(offset / opts_.bucket_steps[r]),
                nb[r] - 1);
            k[r] = std::max(k[r], 0);
        }

        return start + k[0] * nb[1] + k[1];
    }

    /// Get all bucket indices for a vertex (for iteration).
    std::pair<int, int> vertex_bucket_range(int vertex) const {
        int start = vertex_bucket_start_[vertex];
        int count = vertex_n_buckets_[vertex][0] * vertex_n_buckets_[vertex][1];
        return {start, start + count};
    }

    // ── Bucket arc generation ──

    void build_bucket_arcs() {
        for (auto& b : buckets_) {
            b.bucket_arcs.clear();
            b.jump_arcs.clear();
        }

        for (int a = 0; a < pv_.n_arcs; ++a) {
            int from_v = pv_.arc_from[a];
            int to_v = pv_.arc_to[a];

            auto [from_start, from_end] = vertex_bucket_range(from_v);

            for (int bi = from_start; bi < from_end; ++bi) {
                const auto& src_b = buckets_[bi];

                // Compute minimum resource at target after extension
                std::array<double, 2> q_target;
                bool feasible = true;
                for (int r = 0; r < n_main_; ++r) {
                    double d = pv_.arc_resource[r][a];
                    q_target[r] = std::max(
                        src_b.lb[r] + d,
                        pv_.vertex_lb[r][to_v]);
                    if (q_target[r] > pv_.vertex_ub[r][to_v]) {
                        feasible = false;
                        break;
                    }
                }
                if (!feasible) continue;

                int target_bi = vertex_bucket_index(to_v, q_target);
                buckets_[bi].bucket_arcs.push_back({target_bi, a});
            }
        }
    }

    // ── SCC computation (Tarjan's) ──

    void compute_sccs() {
        int n = static_cast<int>(buckets_.size());
        scc_topo_order_.clear();

        // Build adjacency for extended bucket graph
        // (bucket arcs + edges between adjacent buckets of same vertex)
        std::vector<std::vector<int>> adj(n);

        for (int bi = 0; bi < n; ++bi) {
            for (const auto& ba : buckets_[bi].bucket_arcs) {
                adj[bi].push_back(ba.to_bucket);
            }
        }

        // Add edges between adjacent buckets of same vertex (extended bucket graph)
        for (int v = 0; v < pv_.n_vertices; ++v) {
            auto [start, end] = vertex_bucket_range(v);
            for (int bi = start; bi < end; ++bi) {
                // Connect to next bucket in each dimension
                auto& nb = vertex_n_buckets_[v];
                int k0 = (bi - start) / nb[1];
                int k1 = (bi - start) % nb[1];

                if (k0 + 1 < nb[0]) adj[bi].push_back(start + (k0 + 1) * nb[1] + k1);
                if (k1 + 1 < nb[1]) adj[bi].push_back(start + k0 * nb[1] + k1 + 1);
            }
        }

        // Tarjan's SCC
        std::vector<int> index(n, -1), lowlink(n, -1), comp(n, -1);
        std::vector<bool> on_stack(n, false);
        std::stack<int> stack;
        int idx = 0;
        int n_scc = 0;

        std::function<void(int)> strongconnect = [&](int v) {
            index[v] = lowlink[v] = idx++;
            stack.push(v);
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
                int w;
                do {
                    w = stack.top();
                    stack.pop();
                    on_stack[w] = false;
                    comp[w] = n_scc;
                    buckets_[w].scc_id = n_scc;
                } while (w != v);
                ++n_scc;
            }
        };

        for (int i = 0; i < n; ++i) {
            if (index[i] == -1) strongconnect(i);
        }

        // Build SCC membership
        scc_buckets_.assign(n_scc, {});
        for (int i = 0; i < n; ++i) {
            scc_buckets_[comp[i]].push_back(i);
        }

        // Tarjan's produces SCCs in reverse topological order
        scc_topo_order_.resize(n_scc);
        std::iota(scc_topo_order_.begin(), scc_topo_order_.end(), 0);
        std::reverse(scc_topo_order_.begin(), scc_topo_order_.end());
    }

    // ── Labeling ──

    void reset_labels() {
        bucket_labels_.clear();
        bucket_labels_.resize(buckets_.size());
        for (auto& b : buckets_) {
            b.c_best = INF;
        }
    }

    void process_scc(int scc_id) {
        auto& scc_bs = scc_buckets_[scc_id];
        if (scc_bs.empty()) return;

        // Label limit to prevent runaway on large SCCs
        constexpr int MAX_LABELS_PER_SCC = 500000;
        int label_count = 0;

        bool changed = true;
        while (changed) {
            changed = false;
            for (int bi : scc_bs) {
                if (buckets_[bi].fixed) continue;
                if (label_count >= MAX_LABELS_PER_SCC) break;

                auto& labels = bucket_labels_[bi];
                // Use index-based iteration since vector may grow
                // (only for intra-SCC arcs targeting same bucket)
                int n_labels = static_cast<int>(labels.size());
                for (int li = 0; li < n_labels; ++li) {
                    auto* label = labels[li];
                    if (label->extended || label->dominated) continue;

                    // Check dominance in component-wise smaller buckets
                    if (dominated_in_smaller_buckets(label, bi)) {
                        label->dominated = true;
                        continue;
                    }

                    // Extend along bucket arcs
                    for (const auto& ba : buckets_[bi].bucket_arcs) {
                        if (buckets_[ba.to_bucket].fixed) continue;

                        auto* new_label = extend_label(
                            label, ba.arc_id, ba.to_bucket);
                        if (new_label) {
                            if (!dominated_in_bucket(new_label, ba.to_bucket)) {
                                remove_dominated(new_label, ba.to_bucket);
                                bucket_labels_[ba.to_bucket].push_back(new_label);
                                ++label_count;
                                changed = true;
                            }
                        }
                    }

                    // Extend along jump arcs
                    for (const auto& ja : buckets_[bi].jump_arcs) {
                        if (buckets_[ja.jump_bucket].fixed) continue;

                        auto* new_label = extend_label(
                            label, ja.arc_id, ja.jump_bucket);
                        if (new_label) {
                            if (!dominated_in_bucket(new_label, ja.jump_bucket)) {
                                remove_dominated(new_label, ja.jump_bucket);
                                bucket_labels_[ja.jump_bucket].push_back(new_label);
                                ++label_count;
                                changed = true;
                            }
                        }
                    }

                    label->extended = true;
                }
            }
            if (label_count >= MAX_LABELS_PER_SCC) break;
        }

        // Update c_best for this SCC
        update_c_best(scc_id);
    }

    Label<Pack>* extend_label(const Label<Pack>* parent, int arc_id,
                               int target_bucket) {
        int to_v = pv_.arc_to[arc_id];

        auto* L = pool_.allocate();
        L->vertex = to_v;
        L->parent = const_cast<Label<Pack>*>(parent);
        L->parent_arc = arc_id;

        // Cost
        double arc_cost = reduced_costs_ ? reduced_costs_[arc_id]
                                         : pv_.arc_base_cost[arc_id];
        L->cost = parent->cost + arc_cost;
        L->real_cost = parent->real_cost + pv_.arc_base_cost[arc_id];

        // Main resource update
        for (int r = 0; r < n_main_; ++r) {
            double d = pv_.arc_resource[r][arc_id];
            L->q[r] = std::max(parent->q[r] + d, pv_.vertex_lb[r][to_v]);
            if (L->q[r] > pv_.vertex_ub[r][to_v]) {
                return nullptr;  // infeasible
            }
        }

        // Meta-Solver resource extension
        if constexpr (Pack::size > 0) {
            auto [new_states, extra_cost] = pack_.extend(
                Direction::Forward, parent->resource_states, arc_id);
            if (extra_cost >= INF) return nullptr;
            L->resource_states = new_states;
            L->cost += extra_cost;
        }

        // R1C extension
        if (r1c_.has_cuts() && parent->r1c_states && L->r1c_states) {
            double r1c_cost = r1c_.extend(
                Direction::Forward,
                {parent->r1c_states, static_cast<std::size_t>(parent->n_r1c_words)},
                {L->r1c_states, static_cast<std::size_t>(L->n_r1c_words)},
                arc_id, to_v);
            L->cost += r1c_cost;
        }

        // c_best pruning
        if (L->cost + buckets_[target_bucket].c_best > opts_.tolerance) {
            // This label + best completion exceeds tolerance → prune
            // (only useful if c_best has been computed for downstream buckets)
        }

        L->bucket = target_bucket;
        return L;
    }

    bool dominates(const Label<Pack>* L1, const Label<Pack>* L2) const {
        if (L1->vertex != L2->vertex) return false;

        // Main resources: L1 must be <= L2 (forward)
        for (int r = 0; r < n_main_; ++r) {
            if (L1->q[r] > L2->q[r] + EPS) return false;
        }

        // Cost with resource domination adjustments
        double dom_cost = L1->cost;

        // Meta-Solver resource domination cost
        if constexpr (Pack::size > 0) {
            dom_cost += pack_.domination_cost(
                Direction::Forward, L1->vertex,
                L1->resource_states, L2->resource_states);
        }

        // R1C domination cost
        if (r1c_.has_cuts() && L1->r1c_states && L2->r1c_states) {
            dom_cost += r1c_.domination_cost(
                Direction::Forward, L1->vertex,
                {L1->r1c_states, static_cast<std::size_t>(L1->n_r1c_words)},
                {L2->r1c_states, static_cast<std::size_t>(L2->n_r1c_words)});
        }

        return dom_cost <= L2->cost + EPS;
    }

    bool dominated_in_bucket(const Label<Pack>* L, int bi) const {
        for (const auto* existing : bucket_labels_[bi]) {
            if (existing->dominated) continue;
            if (dominates(existing, L)) return true;
        }
        return false;
    }

    bool dominated_in_smaller_buckets(const Label<Pack>* L, int bi) const {
        // Check component-wise smaller buckets for the same vertex
        int v = L->vertex;
        auto [start, end] = vertex_bucket_range(v);
        auto& nb = vertex_n_buckets_[v];

        int k0 = (bi - start) / nb[1];
        int k1 = (bi - start) % nb[1];

        // Iterate over buckets with smaller or equal indices in each dimension
        for (int i0 = 0; i0 <= k0; ++i0) {
            for (int i1 = 0; i1 <= k1; ++i1) {
                int other = start + i0 * nb[1] + i1;
                if (other == bi) continue;

                for (const auto* existing : bucket_labels_[other]) {
                    if (existing->dominated) continue;
                    if (dominates(existing, L)) return true;
                }
            }
        }
        return false;
    }

    void remove_dominated(const Label<Pack>* new_label, int bi) {
        for (auto* existing : bucket_labels_[bi]) {
            if (existing->dominated) continue;
            if (dominates(new_label, existing)) {
                existing->dominated = true;
            }
        }
    }

    void update_c_best(int scc_id) {
        auto& scc_bs = scc_buckets_[scc_id];

        // First pass: c_best from labels in bucket
        for (int bi : scc_bs) {
            double best = INF;
            for (const auto* L : bucket_labels_[bi]) {
                if (!L->dominated && L->cost < best) {
                    best = L->cost;
                }
            }
            buckets_[bi].c_best = best;
        }

        // Second pass: propagate from component-wise smaller buckets
        for (int v = 0; v < pv_.n_vertices; ++v) {
            auto [start, end] = vertex_bucket_range(v);
            auto& nb = vertex_n_buckets_[v];

            for (int k0 = 0; k0 < nb[0]; ++k0) {
                for (int k1 = 0; k1 < nb[1]; ++k1) {
                    int bi = start + k0 * nb[1] + k1;
                    if (buckets_[bi].scc_id != scc_id) continue;

                    // Check smaller neighbors
                    if (k0 > 0) {
                        int prev = start + (k0 - 1) * nb[1] + k1;
                        buckets_[bi].c_best = std::min(
                            buckets_[bi].c_best, buckets_[prev].c_best);
                    }
                    if (k1 > 0) {
                        int prev = start + k0 * nb[1] + (k1 - 1);
                        buckets_[bi].c_best = std::min(
                            buckets_[bi].c_best, buckets_[prev].c_best);
                    }
                }
            }
        }
    }

    double completion_bound(int bi) const {
        return buckets_[bi].c_best;
    }

    std::vector<Path> extract_paths() {
        std::vector<Path> paths;

        // Find all non-dominated labels at the sink
        auto [start, end] = vertex_bucket_range(pv_.sink);
        for (int bi = start; bi < end; ++bi) {
            for (const auto* L : bucket_labels_[bi]) {
                if (L->dominated) continue;
                if (L->cost < opts_.tolerance) {
                    Path p;
                    L->get_path(p.vertices, p.arcs);
                    p.reduced_cost = L->cost;
                    p.original_cost = L->real_cost;
                    paths.push_back(std::move(p));
                }
            }
        }

        // Sort by reduced cost (most negative first)
        std::sort(paths.begin(), paths.end(),
                  [](const Path& a, const Path& b) {
                      return a.reduced_cost < b.reduced_cost;
                  });

        // Limit to max_paths
        if (static_cast<int>(paths.size()) > opts_.max_paths) {
            paths.resize(opts_.max_paths);
        }

        return paths;
    }

    // ── Data ──

    const ProblemView& pv_;
    Pack pack_;
    Options opts_;
    Adjacency adj_;
    int n_main_ = 1;

    const double* reduced_costs_ = nullptr;

    std::vector<Bucket> buckets_;
    std::vector<int> vertex_bucket_start_;        // [vertex] → first bucket index
    std::vector<std::array<int, 2>> vertex_n_buckets_;  // [vertex] → (n0, n1)

    std::vector<std::vector<int>> scc_buckets_;   // [scc_id] → bucket indices
    std::vector<int> scc_topo_order_;             // topological order of SCCs

    std::vector<std::vector<Label<Pack>*>> bucket_labels_;  // [bucket] → labels

    LabelPool<Pack> pool_;
    R1CManager r1c_;
};

}  // namespace bgspprc
