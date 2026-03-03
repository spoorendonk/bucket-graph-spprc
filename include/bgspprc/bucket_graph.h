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
        bool bidirectional = false;
        Stage stage = Stage::Exact;
    };

    BucketGraph(const ProblemView& pv, Pack pack, Options opts = {})
        : pv_(pv), pack_(std::move(pack)), opts_(opts) {
        adj_.build(pv_);
        n_main_ = std::min(pv_.n_main_resources, 2);
    }

    /// Build the bucket graph structure.
    void build() {
        build_buckets();
        build_bucket_arcs(Direction::Forward);
        compute_sccs(Direction::Forward);
        if (opts_.bidirectional) {
            build_bucket_arcs(Direction::Backward);
            compute_sccs(Direction::Backward);
        }
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

    struct Path {
        std::vector<int> vertices;
        std::vector<int> arcs;
        double reduced_cost;
        double original_cost;
    };

    /// Solve: returns paths with negative reduced cost.
    std::vector<Path> solve() {
        if (opts_.bidirectional) {
            return solve_bidirectional();
        }
        return solve_mono();
    }

    // Accessors
    int n_buckets() const { return static_cast<int>(buckets_.size()); }
    const Bucket& bucket(int i) const { return buckets_[i]; }
    const std::vector<Bucket>& buckets() const { return buckets_; }

    void set_stage(Stage s) { opts_.stage = s; }
    void set_tolerance(double t) { opts_.tolerance = t; }

    /// Save best labels for warm starting the next solve.
    /// Keeps the top `fraction` of non-dominated labels by cost.
    void save_warm_labels(double fraction = 0.7) {
        warm_labels_.clear();
        collect_warm_labels(fw_bucket_labels_, Direction::Forward, fraction);
    }

    /// Adaptive bucket step size: halve steps for vertices where the ratio
    /// (resource range / step) exceeds `threshold`. Returns true if any
    /// step was changed (requiring rebuild).
    bool adapt_bucket_steps(double threshold = 20.0) {
        bool changed = false;
        for (int r = 0; r < n_main_; ++r) {
            double max_ratio = 0.0;
            for (int v = 0; v < pv_.n_vertices; ++v) {
                double range = pv_.vertex_ub[r][v] - pv_.vertex_lb[r][v];
                if (range > 0) {
                    max_ratio = std::max(max_ratio, range / opts_.bucket_steps[r]);
                }
            }
            if (max_ratio > threshold) {
                opts_.bucket_steps[r] *= 0.5;
                changed = true;
            }
        }
        return changed;
    }

    /// Get current bucket steps (for inspection).
    const std::array<double, 2>& bucket_steps() const { return opts_.bucket_steps; }

    /// Arc elimination: remove bucket arcs incompatible with gap theta.
    /// Uses c_best (cost-to-b from source) + bw_completion (cost-to-go to sink).
    /// Then generates jump arcs to maintain reachability.
    void eliminate_arcs(double theta) {
        compute_completion_bounds(Direction::Forward);

        int nb = static_cast<int>(buckets_.size());
        for (int bi = 0; bi < nb; ++bi) {
            auto& b = buckets_[bi];
            std::erase_if(b.bucket_arcs, [&](const BucketArc& ba) {
                double arc_cost = reduced_costs_ ?
                    reduced_costs_[ba.arc_id] : pv_.arc_base_cost[ba.arc_id];
                return b.c_best + arc_cost +
                       fw_completion_[ba.to_bucket] > theta + EPS;
            });
        }
        obtain_jump_arcs(Direction::Forward);

        if (opts_.bidirectional) {
            compute_completion_bounds(Direction::Backward);

            for (int bi = 0; bi < nb; ++bi) {
                auto& b = buckets_[bi];
                std::erase_if(b.bw_bucket_arcs, [&](const BucketArc& ba) {
                    double arc_cost = reduced_costs_ ?
                        reduced_costs_[ba.arc_id] : pv_.arc_base_cost[ba.arc_id];
                    return b.bw_c_best + arc_cost +
                           bw_completion_[ba.to_bucket] > theta + EPS;
                });
            }
            obtain_jump_arcs(Direction::Backward);
        }
    }

    /// Bucket fixing: mark buckets as fixed using completion bounds.
    ///
    /// Fix bucket b if c_best(b) + completion(b) > theta, meaning no
    /// source-to-sink path through b can have cost within the gap.
    ///
    /// Returns number of newly fixed buckets.
    int fix_buckets(double theta) {
        // Ensure completion bounds are computed
        if (fw_completion_.empty())
            compute_completion_bounds(Direction::Forward);
        if (opts_.bidirectional && bw_completion_.empty())
            compute_completion_bounds(Direction::Backward);

        int nb = static_cast<int>(buckets_.size());
        int newly_fixed = 0;

        for (int bi = 0; bi < nb; ++bi) {
            if (fixed_.test(bi)) continue;

            // Forward path bound: cost-to-b + cost-from-b-to-sink > theta
            bool fw_bad =
                (buckets_[bi].c_best + fw_completion_[bi] > theta + EPS);

            bool should_fix = fw_bad;

            if (opts_.bidirectional) {
                // Backward path bound
                bool bw_bad =
                    (buckets_[bi].bw_c_best + bw_completion_[bi] > theta + EPS);
                // Concatenation bound: fw_cost_at_b + bw_cost_at_b > theta
                bool concat_bad =
                    (buckets_[bi].c_best + buckets_[bi].bw_c_best > theta + EPS);

                // Fix only if ALL path types through b are bad
                should_fix = fw_bad && bw_bad && concat_bad;
            }

            if (should_fix) {
                fixed_.set(bi);
                ++newly_fixed;
            }
        }

        return newly_fixed;
    }

    /// Query whether a bucket is fixed.
    bool is_bucket_fixed(int bi) const { return fixed_.test(bi); }

    /// Number of fixed buckets.
    int n_fixed_buckets() const { return fixed_.n_fixed(); }


    void reset_elimination() {
        fixed_.clear();
        fw_completion_.clear();
        bw_completion_.clear();
        for (auto& b : buckets_) {
            b.jump_arcs.clear();
            b.bw_jump_arcs.clear();
        }
        build_bucket_arcs(Direction::Forward);
        if (opts_.bidirectional) {
            build_bucket_arcs(Direction::Backward);
        }
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
                    for (int r = n_main_; r < 2; ++r) {
                        b.lb[r] = 0.0;
                        b.ub[r] = INF;
                    }

                    buckets_.push_back(std::move(b));
                }
            }
            total = static_cast<int>(buckets_.size());
        }

        fw_bucket_labels_.resize(buckets_.size());
        bw_bucket_labels_.resize(buckets_.size());
        fixed_.resize(static_cast<int>(buckets_.size()));
    }

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

    std::pair<int, int> vertex_bucket_range(int vertex) const {
        int start = vertex_bucket_start_[vertex];
        int count = vertex_n_buckets_[vertex][0] * vertex_n_buckets_[vertex][1];
        return {start, start + count};
    }

    // ── Bucket arc generation ──

    void build_bucket_arcs(Direction dir) {
        for (auto& b : buckets_) {
            if (dir == Direction::Forward)
                b.bucket_arcs.clear();
            else
                b.bw_bucket_arcs.clear();
            b.jump_arcs.clear();
        }

        for (int a = 0; a < pv_.n_arcs; ++a) {
            int from_v = pv_.arc_from[a];
            int to_v = pv_.arc_to[a];

            if (dir == Direction::Forward) {
                // Forward: iterate buckets of from_v, target at to_v
                auto [start, end] = vertex_bucket_range(from_v);
                for (int bi = start; bi < end; ++bi) {
                    const auto& src_b = buckets_[bi];
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
            } else {
                // Backward: iterate buckets of to_v, target at from_v
                auto [start, end] = vertex_bucket_range(to_v);
                for (int bi = start; bi < end; ++bi) {
                    const auto& src_b = buckets_[bi];
                    std::array<double, 2> q_target;
                    bool feasible = true;
                    for (int r = 0; r < n_main_; ++r) {
                        double d = pv_.arc_resource[r][a];
                        q_target[r] = std::min(
                            src_b.ub[r] - d,
                            pv_.vertex_ub[r][from_v]);
                        if (q_target[r] < pv_.vertex_lb[r][from_v]) {
                            feasible = false;
                            break;
                        }
                    }
                    if (!feasible) continue;
                    int target_bi = vertex_bucket_index(from_v, q_target);
                    buckets_[bi].bw_bucket_arcs.push_back({target_bi, a});
                }
            }
        }
    }

    // ── SCC computation (Tarjan's) ──

    void compute_sccs(Direction dir) {
        int n = static_cast<int>(buckets_.size());

        auto& scc_buckets = (dir == Direction::Forward) ?
            fw_scc_buckets_ : bw_scc_buckets_;
        auto& scc_topo = (dir == Direction::Forward) ?
            fw_scc_topo_order_ : bw_scc_topo_order_;
        auto& bucket_scc_id = (dir == Direction::Forward) ?
            fw_bucket_scc_id_ : bw_bucket_scc_id_;

        scc_topo.clear();
        bucket_scc_id.assign(n, -1);

        // Build adjacency
        std::vector<std::vector<int>> adj(n);
        for (int bi = 0; bi < n; ++bi) {
            auto& arcs = (dir == Direction::Forward) ?
                buckets_[bi].bucket_arcs : buckets_[bi].bw_bucket_arcs;
            for (const auto& ba : arcs) {
                adj[bi].push_back(ba.to_bucket);
            }
        }

        // Same-vertex adjacent edges (direction-dependent)
        for (int v = 0; v < pv_.n_vertices; ++v) {
            auto [start, end] = vertex_bucket_range(v);
            auto& nb = vertex_n_buckets_[v];
            for (int bi = start; bi < end; ++bi) {
                int k0 = (bi - start) / nb[1];
                int k1 = (bi - start) % nb[1];
                if (dir == Direction::Forward) {
                    // Forward: resource increases → edge to next bucket
                    if (k0 + 1 < nb[0])
                        adj[bi].push_back(start + (k0 + 1) * nb[1] + k1);
                    if (k1 + 1 < nb[1])
                        adj[bi].push_back(start + k0 * nb[1] + k1 + 1);
                } else {
                    // Backward: resource decreases → edge to previous bucket
                    if (k0 > 0)
                        adj[bi].push_back(start + (k0 - 1) * nb[1] + k1);
                    if (k1 > 0)
                        adj[bi].push_back(start + k0 * nb[1] + (k1 - 1));
                }
            }
        }

        // Tarjan's SCC
        std::vector<int> index(n, -1), lowlink(n, -1);
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
                    bucket_scc_id[w] = n_scc;
                } while (w != v);
                ++n_scc;
            }
        };

        for (int i = 0; i < n; ++i) {
            if (index[i] == -1) strongconnect(i);
        }

        scc_buckets.assign(n_scc, {});
        for (int i = 0; i < n; ++i) {
            scc_buckets[bucket_scc_id[i]].push_back(i);
        }

        // Tarjan's produces SCCs in reverse topological order
        scc_topo.resize(n_scc);
        std::iota(scc_topo.begin(), scc_topo.end(), 0);
        std::reverse(scc_topo.begin(), scc_topo.end());
    }

    // ── Jump arc generation ──

    /// Generate jump arcs after arc elimination.
    /// For each original arc a=(u,v), if the bucket arc from some bucket b(u)
    /// to bucket b(v) was eliminated, create a jump arc from b(u) that targets
    /// the next non-fixed, non-eliminated reachable bucket at v.
    void obtain_jump_arcs(Direction dir) {
        // Clear existing jump arcs
        for (auto& b : buckets_) {
            if (dir == Direction::Forward)
                b.jump_arcs.clear();
            else
                b.bw_jump_arcs.clear();
        }

        // Build a set of existing bucket arcs for fast lookup
        // For each bucket, collect the set of arc_ids that have bucket arcs
        int nb = static_cast<int>(buckets_.size());
        std::vector<std::vector<int>> existing_arc_ids(nb);
        for (int bi = 0; bi < nb; ++bi) {
            auto& arcs = (dir == Direction::Forward) ?
                buckets_[bi].bucket_arcs : buckets_[bi].bw_bucket_arcs;
            for (const auto& ba : arcs) {
                existing_arc_ids[bi].push_back(ba.arc_id);
            }
            std::sort(existing_arc_ids[bi].begin(), existing_arc_ids[bi].end());
        }

        // For each arc in the problem, check if any source bucket lost its
        // bucket arc due to elimination. If so, create a jump arc.
        for (int a = 0; a < pv_.n_arcs; ++a) {
            int src_v = (dir == Direction::Forward) ?
                pv_.arc_from[a] : pv_.arc_to[a];
            int tgt_v = (dir == Direction::Forward) ?
                pv_.arc_to[a] : pv_.arc_from[a];

            auto [src_start, src_end] = vertex_bucket_range(src_v);

            for (int bi = src_start; bi < src_end; ++bi) {
                if (fixed_.test(bi)) continue;

                // Check if this bucket already has a bucket arc for this arc
                auto& eids = existing_arc_ids[bi];
                if (std::binary_search(eids.begin(), eids.end(), a)) continue;

                // This arc was eliminated from this bucket.
                // Try to find a reachable non-fixed bucket at target vertex.
                std::array<double, 2> q_target;
                bool feasible = true;
                for (int r = 0; r < n_main_; ++r) {
                    double d = pv_.arc_resource[r][a];
                    if (dir == Direction::Forward) {
                        q_target[r] = std::max(
                            buckets_[bi].lb[r] + d,
                            pv_.vertex_lb[r][tgt_v]);
                        if (q_target[r] > pv_.vertex_ub[r][tgt_v]) {
                            feasible = false;
                            break;
                        }
                    } else {
                        q_target[r] = std::min(
                            buckets_[bi].ub[r] - d,
                            pv_.vertex_ub[r][tgt_v]);
                        if (q_target[r] < pv_.vertex_lb[r][tgt_v]) {
                            feasible = false;
                            break;
                        }
                    }
                }
                if (!feasible) continue;

                // Find first non-fixed bucket at target
                int target_bi = vertex_bucket_index(tgt_v, q_target);
                if (!fixed_.test(target_bi)) {
                    if (dir == Direction::Forward)
                        buckets_[bi].jump_arcs.push_back({target_bi, a});
                    else
                        buckets_[bi].bw_jump_arcs.push_back({target_bi, a});
                }
            }
        }
    }

    // ── Label extension (direction-aware) ──

    Label<Pack>* extend_label(const Label<Pack>* parent, int arc_id,
                               Direction dir) {
        int new_v = (dir == Direction::Forward) ?
            pv_.arc_to[arc_id] : pv_.arc_from[arc_id];

        auto* L = pool_.allocate();
        L->vertex = new_v;
        L->dir = dir;
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
            if (dir == Direction::Forward) {
                L->q[r] = std::max(parent->q[r] + d, pv_.vertex_lb[r][new_v]);
                if (L->q[r] > pv_.vertex_ub[r][new_v])
                    return nullptr;
            } else {
                L->q[r] = std::min(parent->q[r] - d, pv_.vertex_ub[r][new_v]);
                if (L->q[r] < pv_.vertex_lb[r][new_v])
                    return nullptr;
            }
        }

        // Meta-Solver resource extension
        if constexpr (Pack::size > 0) {
            auto [new_states, extra_cost] = pack_.extend(
                dir, parent->resource_states, arc_id);
            if (extra_cost >= INF) return nullptr;
            L->resource_states = new_states;
            L->cost += extra_cost;
        }

        // R1C extension
        if (r1c_.has_cuts() && parent->r1c_states && L->r1c_states) {
            double r1c_cost = r1c_.extend(
                dir,
                {parent->r1c_states, static_cast<std::size_t>(parent->n_r1c_words)},
                {L->r1c_states, static_cast<std::size_t>(L->n_r1c_words)},
                arc_id, new_v);
            L->cost += r1c_cost;
        }

        // Compute correct target bucket
        L->bucket = vertex_bucket_index(new_v, L->q);
        return L;
    }

    // ── Dominance (direction-aware) ──

    bool dominates(const Label<Pack>* L1, const Label<Pack>* L2,
                   Direction dir) const {
        if (L1->vertex != L2->vertex) return false;

        for (int r = 0; r < n_main_; ++r) {
            if (dir == Direction::Forward) {
                if (L1->q[r] > L2->q[r] + EPS) return false;
            } else {
                if (L1->q[r] < L2->q[r] - EPS) return false;
            }
        }

        // Heuristic1: cost-only dominance (ignore resource states)
        if (opts_.stage == Stage::Heuristic1) {
            return L1->cost <= L2->cost + EPS;
        }

        double dom_cost = L1->cost;

        if constexpr (Pack::size > 0) {
            dom_cost += pack_.domination_cost(
                dir, L1->vertex,
                L1->resource_states, L2->resource_states);
        }

        if (r1c_.has_cuts() && L1->r1c_states && L2->r1c_states) {
            dom_cost += r1c_.domination_cost(
                dir, L1->vertex,
                {L1->r1c_states, static_cast<std::size_t>(L1->n_r1c_words)},
                {L2->r1c_states, static_cast<std::size_t>(L2->n_r1c_words)});
        }

        return dom_cost <= L2->cost + EPS;
    }

    bool dominated_in_bucket(const Label<Pack>* L, int bi, Direction dir,
                             std::vector<std::vector<Label<Pack>*>>& labels) const {
        for (const auto* existing : labels[bi]) {
            if (existing->dominated) continue;
            if (dominates(existing, L, dir)) return true;
        }
        return false;
    }

    bool dominated_in_adjacent_buckets(const Label<Pack>* L, int bi,
                                       Direction dir,
                                       std::vector<std::vector<Label<Pack>*>>& labels) const {
        int v = L->vertex;
        auto [start, end] = vertex_bucket_range(v);
        auto& nb = vertex_n_buckets_[v];

        int k0 = (bi - start) / nb[1];
        int k1 = (bi - start) % nb[1];

        if (dir == Direction::Forward) {
            // Check component-wise smaller buckets (lower q = better)
            for (int i0 = 0; i0 <= k0; ++i0) {
                for (int i1 = 0; i1 <= k1; ++i1) {
                    int other = start + i0 * nb[1] + i1;
                    if (other == bi) continue;
                    for (const auto* existing : labels[other]) {
                        if (existing->dominated) continue;
                        if (dominates(existing, L, dir)) return true;
                    }
                }
            }
        } else {
            // Check component-wise larger buckets (higher q = better)
            for (int i0 = k0; i0 < nb[0]; ++i0) {
                for (int i1 = k1; i1 < nb[1]; ++i1) {
                    int other = start + i0 * nb[1] + i1;
                    if (other == bi) continue;
                    for (const auto* existing : labels[other]) {
                        if (existing->dominated) continue;
                        if (dominates(existing, L, dir)) return true;
                    }
                }
            }
        }
        return false;
    }

    void remove_dominated(const Label<Pack>* new_label, int bi, Direction dir,
                          std::vector<std::vector<Label<Pack>*>>& labels) {
        for (auto* existing : labels[bi]) {
            if (existing->dominated) continue;
            if (dominates(new_label, existing, dir)) {
                existing->dominated = true;
            }
        }
    }

    // ── SCC processing (unified for both directions) ──

    void process_scc(int scc_id, Direction dir,
                     const std::vector<std::vector<int>>& scc_buckets,
                     std::vector<std::vector<Label<Pack>*>>& labels,
                     double midpoint) {
        auto& scc_bs = scc_buckets[scc_id];
        if (scc_bs.empty()) return;

        constexpr int MAX_LABELS_PER_SCC = 500000;
        int label_count = 0;

        bool changed = true;
        while (changed) {
            changed = false;
            for (int bi : scc_bs) {
                if (fixed_.test(bi)) continue;
                if (label_count >= MAX_LABELS_PER_SCC) break;

                auto& bucket_labels = labels[bi];
                int n_labels = static_cast<int>(bucket_labels.size());
                for (int li = 0; li < n_labels; ++li) {
                    auto* label = bucket_labels[li];
                    if (label->extended || label->dominated) continue;

                    // Midpoint cutoff for bidirectional
                    if (dir == Direction::Forward && label->q[0] > midpoint) {
                        continue;
                    }
                    if (dir == Direction::Backward && label->q[0] < midpoint) {
                        continue;
                    }

                    if (dominated_in_adjacent_buckets(label, bi, dir, labels)) {
                        label->dominated = true;
                        continue;
                    }

                    // Extend along bucket arcs
                    auto& arcs = (dir == Direction::Forward) ?
                        buckets_[bi].bucket_arcs : buckets_[bi].bw_bucket_arcs;

                    for (const auto& ba : arcs) {
                        auto* new_label = extend_label(label, ba.arc_id, dir);
                        if (new_label) {
                            int actual_bi = new_label->bucket;
                            // Check actual landing bucket, not bucket arc target
                            if (fixed_.test(actual_bi)) continue;
                            if (!dominated_in_bucket(new_label, actual_bi,
                                                     dir, labels)) {
                                remove_dominated(new_label, actual_bi,
                                                 dir, labels);
                                labels[actual_bi].push_back(new_label);
                                ++label_count;
                                changed = true;
                            }
                        }
                    }

                    // Jump arcs
                    auto& jarcs = (dir == Direction::Forward) ?
                        buckets_[bi].jump_arcs : buckets_[bi].bw_jump_arcs;
                    for (const auto& ja : jarcs) {
                        auto* new_label = extend_label(
                            label, ja.arc_id, dir);
                        if (new_label) {
                            int actual_bi = new_label->bucket;
                            if (fixed_.test(actual_bi)) continue;
                            if (!dominated_in_bucket(new_label, actual_bi,
                                                     dir, labels)) {
                                remove_dominated(new_label, actual_bi,
                                                 dir, labels);
                                labels[actual_bi].push_back(new_label);
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

        update_c_best(scc_id, dir, scc_buckets, labels);
    }

    // ── c_best update ──

    void update_c_best(int scc_id, Direction dir,
                       const std::vector<std::vector<int>>& scc_buckets,
                       const std::vector<std::vector<Label<Pack>*>>& labels) {
        auto& scc_bs = scc_buckets[scc_id];

        // First pass: c_best from labels in bucket
        for (int bi : scc_bs) {
            double best = INF;
            for (const auto* L : labels[bi]) {
                if (!L->dominated && L->cost < best) {
                    best = L->cost;
                }
            }
            if (dir == Direction::Forward)
                buckets_[bi].c_best = best;
            else
                buckets_[bi].bw_c_best = best;
        }

        // Second pass: propagate
        for (int v = 0; v < pv_.n_vertices; ++v) {
            auto [start, end] = vertex_bucket_range(v);
            auto& nb = vertex_n_buckets_[v];

            if (dir == Direction::Forward) {
                // Propagate from smaller to larger
                for (int k0 = 0; k0 < nb[0]; ++k0) {
                    for (int k1 = 0; k1 < nb[1]; ++k1) {
                        int bi = start + k0 * nb[1] + k1;
                        auto& scc_id_bi = fw_bucket_scc_id_[bi];
                        if (scc_id_bi != scc_id) continue;
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
            } else {
                // Propagate from larger to smaller
                for (int k0 = nb[0] - 1; k0 >= 0; --k0) {
                    for (int k1 = nb[1] - 1; k1 >= 0; --k1) {
                        int bi = start + k0 * nb[1] + k1;
                        auto& scc_id_bi = bw_bucket_scc_id_[bi];
                        if (scc_id_bi != scc_id) continue;
                        if (k0 + 1 < nb[0]) {
                            int next = start + (k0 + 1) * nb[1] + k1;
                            buckets_[bi].bw_c_best = std::min(
                                buckets_[bi].bw_c_best,
                                buckets_[next].bw_c_best);
                        }
                        if (k1 + 1 < nb[1]) {
                            int next = start + k0 * nb[1] + (k1 + 1);
                            buckets_[bi].bw_c_best = std::min(
                                buckets_[bi].bw_c_best,
                                buckets_[next].bw_c_best);
                        }
                    }
                }
            }
        }
    }

    double completion_bound(int bi) const {
        return buckets_[bi].c_best;
    }

    // ── Completion bound computation (cost-to-go) ──

    /// Compute backward completion bounds for arc elimination / bucket fixing.
    /// completion(b) = lower bound on cost from bucket b to the terminal
    ///   (sink for forward, source for backward).
    /// Uses reverse topological order on the DAG of SCCs.
    void compute_completion_bounds(Direction dir) {
        int nb = static_cast<int>(buckets_.size());
        auto& completion = (dir == Direction::Forward) ?
            fw_completion_ : bw_completion_;
        completion.assign(nb, INF);

        // Terminal buckets: cost-to-go = 0
        int terminal = (dir == Direction::Forward) ? pv_.sink : pv_.source;
        auto [t_start, t_end] = vertex_bucket_range(terminal);
        for (int bi = t_start; bi < t_end; ++bi) {
            completion[bi] = 0.0;
        }

        auto& scc_topo = (dir == Direction::Forward) ?
            fw_scc_topo_order_ : bw_scc_topo_order_;
        auto& scc_buckets = (dir == Direction::Forward) ?
            fw_scc_buckets_ : bw_scc_buckets_;

        // Process SCCs in reverse topological order (from sink toward source).
        // For DAG SCCs (single bucket), one pass suffices.
        // For SCCs with cycles, iterate until convergence.
        for (int i = static_cast<int>(scc_topo.size()) - 1; i >= 0; --i) {
            int scc = scc_topo[i];
            auto& scc_bs = scc_buckets[scc];

            bool has_cycle = (scc_bs.size() > 1);
            int max_iters = has_cycle ? static_cast<int>(scc_bs.size()) + 1 : 1;

            for (int iter = 0; iter < max_iters; ++iter) {
                bool changed = false;
                for (int bi : scc_bs) {
                    auto& arcs = (dir == Direction::Forward) ?
                        buckets_[bi].bucket_arcs :
                        buckets_[bi].bw_bucket_arcs;
                    for (const auto& ba : arcs) {
                        if (completion[ba.to_bucket] >= INF) continue;
                        double arc_cost = reduced_costs_ ?
                            reduced_costs_[ba.arc_id] :
                            pv_.arc_base_cost[ba.arc_id];
                        double candidate = arc_cost + completion[ba.to_bucket];
                        if (candidate < completion[bi] - EPS) {
                            completion[bi] = candidate;
                            changed = true;
                        }
                    }
                }
                if (!changed) break;
            }
        }

        // Within-vertex propagation: lower resource bucket has more slack,
        // so its cost-to-go is at least as good as higher resource buckets.
        for (int v = 0; v < pv_.n_vertices; ++v) {
            auto [start, end] = vertex_bucket_range(v);
            auto& vnb = vertex_n_buckets_[v];
            if (dir == Direction::Forward) {
                // Forward: lower k = more slack → propagate higher→lower
                for (int k0 = vnb[0] - 2; k0 >= 0; --k0) {
                    for (int k1 = vnb[1] - 1; k1 >= 0; --k1) {
                        int bi = start + k0 * vnb[1] + k1;
                        int next = start + (k0 + 1) * vnb[1] + k1;
                        completion[bi] = std::min(completion[bi],
                                                  completion[next]);
                    }
                }
                for (int k0 = vnb[0] - 1; k0 >= 0; --k0) {
                    for (int k1 = vnb[1] - 2; k1 >= 0; --k1) {
                        int bi = start + k0 * vnb[1] + k1;
                        int next = start + k0 * vnb[1] + (k1 + 1);
                        completion[bi] = std::min(completion[bi],
                                                  completion[next]);
                    }
                }
            } else {
                // Backward: higher k = more slack → propagate lower→higher
                for (int k0 = 1; k0 < vnb[0]; ++k0) {
                    for (int k1 = 0; k1 < vnb[1]; ++k1) {
                        int bi = start + k0 * vnb[1] + k1;
                        int prev = start + (k0 - 1) * vnb[1] + k1;
                        completion[bi] = std::min(completion[bi],
                                                  completion[prev]);
                    }
                }
                for (int k0 = 0; k0 < vnb[0]; ++k0) {
                    for (int k1 = 1; k1 < vnb[1]; ++k1) {
                        int bi = start + k0 * vnb[1] + k1;
                        int prev = start + k0 * vnb[1] + (k1 - 1);
                        completion[bi] = std::min(completion[bi],
                                                  completion[prev]);
                    }
                }
            }
        }
    }

    // ── Label storage management ──

    void reset_label_storage(std::vector<std::vector<Label<Pack>*>>& labels) {
        labels.clear();
        labels.resize(buckets_.size());
    }

    void reset_c_best() {
        for (auto& b : buckets_) {
            b.c_best = INF;
            b.bw_c_best = INF;
        }
    }

    Label<Pack>* create_initial_label(Direction dir) {
        auto* L = pool_.allocate();
        L->dir = dir;
        L->cost = 0.0;
        L->real_cost = 0.0;

        if (dir == Direction::Forward) {
            L->vertex = pv_.source;
            for (int r = 0; r < n_main_; ++r)
                L->q[r] = pv_.vertex_lb[r][pv_.source];
        } else {
            L->vertex = pv_.sink;
            for (int r = 0; r < n_main_; ++r)
                L->q[r] = pv_.vertex_ub[r][pv_.sink];
        }

        L->resource_states = pack_.init_states(dir);
        if (r1c_.has_cuts()) {
            r1c_.init_state({L->r1c_states,
                static_cast<std::size_t>(L->n_r1c_words)});
        }
        return L;
    }

    // ── Mono-directional solve ──

    std::vector<Path> solve_mono() {
        pool_.clear();
        reset_label_storage(fw_bucket_labels_);
        reset_c_best();

        auto* src = create_initial_label(Direction::Forward);
        int src_bi = vertex_bucket_index(pv_.source, src->q);
        src->bucket = src_bi;
        fw_bucket_labels_[src_bi].push_back(src);

        // Inject warm labels from previous solve
        if (!warm_labels_.empty()) {
            inject_warm_labels(fw_bucket_labels_, Direction::Forward);
        }

        for (int scc : fw_scc_topo_order_) {
            process_scc(scc, Direction::Forward, fw_scc_buckets_,
                        fw_bucket_labels_, INF);
        }

        return extract_paths(fw_bucket_labels_);
    }

    // ── Bi-directional solve ──

    double compute_midpoint() const {
        double min_lb = INF, max_ub = -INF;
        for (int v = 0; v < pv_.n_vertices; ++v) {
            min_lb = std::min(min_lb, pv_.vertex_lb[0][v]);
            max_ub = std::max(max_ub, pv_.vertex_ub[0][v]);
        }
        return (min_lb + max_ub) / 2.0;
    }

    std::vector<Path> solve_bidirectional() {
        pool_.clear();
        reset_label_storage(fw_bucket_labels_);
        reset_label_storage(bw_bucket_labels_);
        reset_c_best();

        double mu = compute_midpoint();

        // Forward labeling
        auto* src = create_initial_label(Direction::Forward);
        int src_bi = vertex_bucket_index(pv_.source, src->q);
        src->bucket = src_bi;
        fw_bucket_labels_[src_bi].push_back(src);

        if (!warm_labels_.empty()) {
            inject_warm_labels(fw_bucket_labels_, Direction::Forward);
        }

        for (int scc : fw_scc_topo_order_) {
            process_scc(scc, Direction::Forward, fw_scc_buckets_,
                        fw_bucket_labels_, mu);
        }

        // Backward labeling
        auto* snk = create_initial_label(Direction::Backward);
        int snk_bi = vertex_bucket_index(pv_.sink, snk->q);
        snk->bucket = snk_bi;
        bw_bucket_labels_[snk_bi].push_back(snk);

        for (int scc : bw_scc_topo_order_) {
            process_scc(scc, Direction::Backward, bw_scc_buckets_,
                        bw_bucket_labels_, mu);
        }

        // Collect paths: forward labels that reached sink + concatenations
        auto paths = extract_paths(fw_bucket_labels_);
        auto concat_paths = concatenate_and_extract();
        paths.insert(paths.end(), concat_paths.begin(), concat_paths.end());

        // Sort and limit
        std::sort(paths.begin(), paths.end(),
                  [](const Path& a, const Path& b) {
                      return a.reduced_cost < b.reduced_cost;
                  });
        if (static_cast<int>(paths.size()) > opts_.max_paths) {
            paths.resize(opts_.max_paths);
        }
        return paths;
    }

    // ── Concatenation ──

    std::vector<Path> concatenate_and_extract() {
        std::vector<Path> paths;

        for (int v = 0; v < pv_.n_vertices; ++v) {
            if (v == pv_.source || v == pv_.sink) continue;

            // Collect non-dominated forward and backward labels at v
            auto [start, end] = vertex_bucket_range(v);

            std::vector<const Label<Pack>*> fw_labels;
            std::vector<const Label<Pack>*> bw_labels;

            for (int bi = start; bi < end; ++bi) {
                for (const auto* L : fw_bucket_labels_[bi]) {
                    if (!L->dominated) fw_labels.push_back(L);
                }
                for (const auto* L : bw_bucket_labels_[bi]) {
                    if (!L->dominated) bw_labels.push_back(L);
                }
            }

            if (fw_labels.empty() || bw_labels.empty()) continue;

            // Try all pairs
            for (const auto* fw : fw_labels) {
                for (const auto* bw : bw_labels) {
                    // Resource feasibility: fw.q[r] <= bw.q[r]
                    bool feasible = true;
                    for (int r = 0; r < n_main_; ++r) {
                        if (fw->q[r] > bw->q[r] + EPS) {
                            feasible = false;
                            break;
                        }
                    }
                    if (!feasible) continue;

                    double total_cost = fw->cost + bw->cost;
                    double total_real_cost = fw->real_cost + bw->real_cost;

                    // Resource concatenation cost
                    if constexpr (Pack::size > 0) {
                        total_cost += pack_.concatenation_cost(
                            Symmetry::Asymmetric, v,
                            fw->resource_states, bw->resource_states);
                    }

                    // R1C concatenation cost
                    if (r1c_.has_cuts() && fw->r1c_states && bw->r1c_states) {
                        total_cost += r1c_.concatenation_cost(
                            Symmetry::Asymmetric, v,
                            {fw->r1c_states,
                             static_cast<std::size_t>(fw->n_r1c_words)},
                            {bw->r1c_states,
                             static_cast<std::size_t>(bw->n_r1c_words)});
                    }

                    if (total_cost < opts_.tolerance) {
                        Path p;
                        // Forward subpath: source → ... → v
                        fw->get_path(p.vertices, p.arcs);

                        // Backward subpath: v → ... → sink
                        std::vector<int> bw_verts, bw_arcs;
                        bw->get_backward_subpath(bw_verts, bw_arcs);

                        // Append backward (skip first vertex = v, already in fw)
                        for (std::size_t i = 1; i < bw_verts.size(); ++i) {
                            p.vertices.push_back(bw_verts[i]);
                        }
                        p.arcs.insert(p.arcs.end(),
                                      bw_arcs.begin(), bw_arcs.end());

                        p.reduced_cost = total_cost;
                        p.original_cost = total_real_cost;
                        paths.push_back(std::move(p));
                    }
                }
            }
        }

        return paths;
    }

    // ── Path extraction ──

    std::vector<Path> extract_paths(
            const std::vector<std::vector<Label<Pack>*>>& labels) {
        std::vector<Path> paths;

        auto [start, end] = vertex_bucket_range(pv_.sink);
        for (int bi = start; bi < end; ++bi) {
            for (const auto* L : labels[bi]) {
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

        std::sort(paths.begin(), paths.end(),
                  [](const Path& a, const Path& b) {
                      return a.reduced_cost < b.reduced_cost;
                  });

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
    BucketFixBitmap fixed_;
    std::vector<double> fw_completion_;   // forward cost-to-go (b → sink)
    std::vector<double> bw_completion_;   // backward cost-to-go (b → source)
    std::vector<int> vertex_bucket_start_;
    std::vector<std::array<int, 2>> vertex_n_buckets_;

    // Forward SCC data
    std::vector<std::vector<int>> fw_scc_buckets_;
    std::vector<int> fw_scc_topo_order_;
    std::vector<int> fw_bucket_scc_id_;

    // Backward SCC data
    std::vector<std::vector<int>> bw_scc_buckets_;
    std::vector<int> bw_scc_topo_order_;
    std::vector<int> bw_bucket_scc_id_;

    // Label storage (forward and backward)
    std::vector<std::vector<Label<Pack>*>> fw_bucket_labels_;
    std::vector<std::vector<Label<Pack>*>> bw_bucket_labels_;

    // Warm label storage: saved state from previous solve
    struct WarmLabel {
        int vertex;
        Direction dir;
        double cost;
        double real_cost;
        std::array<double, 2> q;
        int parent_arc;  // last arc (for path, not critical for warm start)
        typename Pack::StatesTuple resource_states;
        std::vector<uint64_t> r1c_state;
    };
    std::vector<WarmLabel> warm_labels_;

    void collect_warm_labels(const std::vector<std::vector<Label<Pack>*>>& labels,
                             Direction dir, double fraction) {
        // Gather all non-dominated labels
        std::vector<const Label<Pack>*> all_labels;
        for (const auto& bucket : labels) {
            for (const auto* L : bucket) {
                if (!L->dominated) {
                    all_labels.push_back(L);
                }
            }
        }

        // Sort by cost
        std::sort(all_labels.begin(), all_labels.end(),
                  [](const Label<Pack>* a, const Label<Pack>* b) {
                      return a->cost < b->cost;
                  });

        // Keep top fraction
        int keep = std::max(1, static_cast<int>(all_labels.size() * fraction));
        keep = std::min(keep, static_cast<int>(all_labels.size()));

        for (int i = 0; i < keep; ++i) {
            const auto* L = all_labels[i];
            WarmLabel wl;
            wl.vertex = L->vertex;
            wl.dir = dir;
            wl.cost = L->cost;
            wl.real_cost = L->real_cost;
            wl.q = L->q;
            wl.parent_arc = L->parent_arc;
            wl.resource_states = L->resource_states;
            if (L->r1c_states && L->n_r1c_words > 0) {
                wl.r1c_state.assign(L->r1c_states,
                                     L->r1c_states + L->n_r1c_words);
            }
            warm_labels_.push_back(std::move(wl));
        }
    }

    void inject_warm_labels(std::vector<std::vector<Label<Pack>*>>& labels,
                            Direction dir) {
        for (const auto& wl : warm_labels_) {
            if (wl.dir != dir) continue;

            auto* L = pool_.allocate();
            L->vertex = wl.vertex;
            L->dir = dir;
            L->cost = wl.cost;
            L->real_cost = wl.real_cost;
            L->q = wl.q;
            L->parent = nullptr;  // warm labels have no parent chain
            L->parent_arc = wl.parent_arc;
            L->resource_states = wl.resource_states;

            if (!wl.r1c_state.empty() && L->r1c_states) {
                int words = std::min(static_cast<int>(wl.r1c_state.size()),
                                      L->n_r1c_words);
                std::copy_n(wl.r1c_state.data(), words, L->r1c_states);
            }

            int bi = vertex_bucket_index(wl.vertex, wl.q);
            L->bucket = bi;

            if (!dominated_in_bucket(L, bi, dir, labels)) {
                labels[bi].push_back(L);
            }
        }
    }

    LabelPool<Pack> pool_;
    R1CManager r1c_;
};

}  // namespace bgspprc
