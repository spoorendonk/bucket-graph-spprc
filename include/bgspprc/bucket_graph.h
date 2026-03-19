#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <numeric>
#include <span>
#include <stack>
#include <tuple>
#include <unordered_map>
#include <vector>

#if defined(__AVX2__) && __has_include(<experimental/simd>)
#include <experimental/simd>
#define BGSPPRC_HAS_SIMD 1
#endif

#include "arc.h"
#include "bucket.h"
#include "label.h"
#include "problem_view.h"
#include "resource.h"
#include "types.h"

namespace bgspprc {

namespace detail {

// Detect a resource with compress_state(uint32_t, int) → uint32_t
template <typename R>
concept HasCompressState = requires(const R& r, typename R::State s, int v) {
  { r.compress_state(s, v) } -> std::same_as<uint32_t>;
};

// Find the index of the first resource with compress_state in a ResourcePack.
// Returns sizeof...(Rs) if none found.
template <typename Pack>
struct NgResourceIndex;

template <Resource... Rs>
struct NgResourceIndex<ResourcePack<Rs...>> {
  static constexpr std::size_t find() {
    std::size_t idx = 0;
    bool found = false;
    ((found || (HasCompressState<Rs> ? (found = true, true) : (++idx, false))), ...);
    return found ? idx : sizeof...(Rs);
  }
  static constexpr std::size_t value = find();
  static constexpr bool has_ng = value < sizeof...(Rs);
};

}  // namespace detail

/// Core bucket graph engine for SPPRC labeling.
///
/// Template parameter Pack is a ResourcePack<Rs...>.
template <typename Pack>
class BucketGraph {
 public:
  struct Options {
    std::array<double, 2> bucket_steps = {1.0, 1.0};
    int max_paths = 0;  // 0 = unlimited
    double theta = -1e-6;
    bool bidirectional = false;
    bool symmetric = false;  // skip backward labeling, use fw labels as bw
    Stage stage = Stage::Exact;
    int max_enum_labels =
        5000000;  // safety cap on total labels during enumeration
  };

  BucketGraph(const ProblemView& pv, Pack pack, Options opts = {})
      : pv_(pv), pack_(std::move(pack)), opts_(opts) {
    if (opts_.symmetric) {
      opts_.bidirectional = true;
      if constexpr (Pack::size > 0)
        assert(pack_.symmetric() &&
               "symmetric mode requires all resources to be symmetric");
    }
    adj_.build(pv_);
    n_main_ = std::min(pv_.n_main_resources, 2);
    if constexpr (Pack::size > 0) {
      min_dom_cost_ = pack_.min_domination_cost();
    }
    for (int r = 0; r < n_main_; ++r)
      main_nondisposable_[r] =
          pv_.resource_nondisposable && pv_.resource_nondisposable[r];
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
    if (opts_.symmetric) {
      build_arc_lookup();
    }
    midpoint_initialized_ = false;
  }

  /// Update reduced costs (fast, O(1) — just stores the pointer).
  void update_arc_costs(const double* reduced_costs) {
    reduced_costs_ = reduced_costs;
  }

  /// Access a resource in the pack (for runtime reconfiguration).
  template <typename R>
  R& resource() {
    return std::get<R>(pack_.resources);
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
  void set_theta(double t) { opts_.theta = t; }
  bool enumeration_complete() const { return enum_complete_; }

  void reset_midpoint() { midpoint_initialized_ = false; }
  double midpoint() const { return midpoint_; }

  /// Save best labels for warm starting the next solve.
  /// Keeps the top `fraction` of non-dominated labels by cost.
  void save_warm_labels(double fraction = 0.7) {
    warm_labels_.clear();
    collect_warm_labels(fw_labels_, Direction::Forward, fraction);
  }

  /// Adaptive bucket step size: halve steps for vertices where the ratio
  /// (resource range / step) exceeds `threshold`. Returns true if any
  /// step was changed (requiring rebuild).
  bool adapt_bucket_steps(double threshold = 20.0) {
    bool changed = false;
    if (!vertex_bucket_steps_.empty()) {
      // Per-vertex mode: halve steps for vertices exceeding threshold
      for (int v = 0; v < pv_.n_vertices; ++v) {
        for (int r = 0; r < n_main_; ++r) {
          double range = pv_.vertex_ub[r][v] - pv_.vertex_lb[r][v];
          if (range > 0 &&
              range / vertex_bucket_steps_[v][r] > threshold) {
            vertex_bucket_steps_[v][r] *= 0.5;
            changed = true;
          }
        }
      }
    } else {
      // Uniform mode
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
    }
    return changed;
  }

  /// Get current bucket steps (for inspection).
  const std::array<double, 2>& bucket_steps() const {
    return opts_.bucket_steps;
  }

  /// Set per-vertex bucket step sizes. Overrides uniform opts_.bucket_steps
  /// during build. Size must equal n_vertices.
  void set_vertex_bucket_steps(
      std::vector<std::array<double, 2>> steps) {
    assert(static_cast<int>(steps.size()) == pv_.n_vertices);
    vertex_bucket_steps_ = std::move(steps);
  }

  /// Compute per-vertex bucket steps from minimum positive inbound arc resource.
  /// Vertices get finer steps where arcs have small resource consumption,
  /// capped at max_buckets_per_vertex buckets per resource dimension.
  std::vector<std::array<double, 2>> compute_min_inbound_arc_resource(
      int max_buckets_per_vertex = 200) const {
    std::vector<std::array<double, 2>> steps(pv_.n_vertices, {INF, INF});
    for (int a = 0; a < pv_.n_arcs; ++a) {
      int to = pv_.arc_to[a];
      for (int r = 0; r < n_main_; ++r) {
        double d = pv_.arc_resource[r][a];
        if (d > EPS) steps[to][r] = std::min(steps[to][r], d);
      }
    }
    for (int v = 0; v < pv_.n_vertices; ++v) {
      for (int r = 0; r < n_main_; ++r) {
        double range = pv_.vertex_ub[r][v] - pv_.vertex_lb[r][v];
        if (range < EPS) {
          steps[v][r] = 1.0;  // single bucket
          continue;
        }
        double min_step = range / max_buckets_per_vertex;
        steps[v][r] = std::max(steps[v][r], min_step);
        if (steps[v][r] >= INF) steps[v][r] = range;  // fallback: 1 bucket
      }
    }
    return steps;
  }

  /// Arc elimination: remove bucket arcs incompatible with gap theta.
  /// Uses c_best (cost-to-b from source) + bw_completion (cost-to-go to sink).
  /// Then generates jump arcs to maintain reachability.
  void eliminate_arcs(double theta) {
    compute_completion_bounds(Direction::Forward);

    int nb = static_cast<int>(buckets_.size());
    for (int bi = 0; bi < nb; ++bi) {
      auto& b = buckets_[bi];
      std::erase_if(b.bucket_arcs, [&](const BucketArc& ba) {
        double arc_cost = reduced_costs_ ? reduced_costs_[ba.arc_id]
                                         : pv_.arc_base_cost[ba.arc_id];
        return b.c_best + arc_cost + fw_completion_[ba.to_bucket] > theta + EPS;
      });
    }
    obtain_jump_arcs(Direction::Forward);

    if (opts_.bidirectional) {
      compute_completion_bounds(Direction::Backward);

      for (int bi = 0; bi < nb; ++bi) {
        auto& b = buckets_[bi];
        std::erase_if(b.bw_bucket_arcs, [&](const BucketArc& ba) {
          double arc_cost = reduced_costs_ ? reduced_costs_[ba.arc_id]
                                           : pv_.arc_base_cost[ba.arc_id];
          return b.bw_c_best + arc_cost + bw_completion_[ba.to_bucket] >
                 theta + EPS;
        });
      }
      obtain_jump_arcs(Direction::Backward);
    }
  }

  /// Label-based arc elimination per Sadykov et al. 2021, Section 4.2.
  /// Requires labels from a prior solve(). Tighter than bound-based
  /// eliminate_arcs because it checks actual θ-compatible (L, L̃) pairs.
  /// Falls back to bound-based for mono-directional (no opposite labels).
  void eliminate_arcs_label_based(double theta) {
    if (!opts_.bidirectional || opts_.symmetric) {
      // Mono or symmetric: no backward labels → fall back to bound-based
      eliminate_arcs(theta);
      return;
    }

    // Phase 1: compute jump arcs (needed by BucketArcElimination)
    obtain_jump_arcs(Direction::Forward);
    obtain_jump_arcs(Direction::Backward);

    // Phase 2: forward elimination
    bucket_arc_elimination(theta, Direction::Forward);

    // Phase 3: backward elimination
    bucket_arc_elimination(theta, Direction::Backward);

    // Phase 4: recompute jump arcs after elimination
    obtain_jump_arcs(Direction::Forward);
    obtain_jump_arcs(Direction::Backward);
  }

  /// Bucket fixing: mark buckets as fixed using completion bounds.
  ///
  /// Fix bucket b if c_best(b) + completion(b) > theta, meaning no
  /// source-to-sink path through b can have cost within the gap.
  ///
  /// Returns number of newly fixed buckets.
  int fix_buckets(double theta) {
    fixing_theta_ = theta;
    // Ensure completion bounds are computed
    if (fw_completion_.empty()) compute_completion_bounds(Direction::Forward);
    if (opts_.bidirectional && bw_completion_.empty())
      compute_completion_bounds(Direction::Backward);

    int nb = static_cast<int>(buckets_.size());
    int newly_fixed = 0;

    for (int bi = 0; bi < nb; ++bi) {
      if (fixed_.test(bi)) continue;

      // Forward path bound: cost-to-b + cost-from-b-to-sink > theta
      bool fw_bad = (buckets_[bi].c_best + fw_completion_[bi] > theta + EPS);

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

  /// Number of bw labels pruned by post-pass exact completion bounds.
  int n_bw_labels_pruned() const { return bw_labels_pruned_; }

  void reset_elimination() {
    fixed_.clear();
    fixing_theta_ = INF;
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
  // ── SoA label storage ──

  static constexpr bool has_ng_ = detail::NgResourceIndex<Pack>::has_ng;
  static constexpr std::size_t ng_idx_ = detail::NgResourceIndex<Pack>::value;

  struct BucketLabels {
    std::vector<std::vector<Label<Pack>*>> labels;
    std::vector<std::vector<double>> costs;
    std::vector<std::vector<double>> q0;
    std::vector<std::vector<double>> q1;
    std::vector<std::vector<uint32_t>> ng_bits;  // self-bit-stripped ng states

    void resize(std::size_t n) {
      labels.resize(n);
      costs.resize(n);
      q0.resize(n);
      q1.resize(n);
      if constexpr (has_ng_) ng_bits.resize(n);
    }

    std::size_t size() const { return labels.size(); }
  };

  // ── Bucket construction ──

  const std::array<double, 2>& step_for_vertex(int v) const {
    if (!vertex_bucket_steps_.empty())
      return vertex_bucket_steps_[v];
    return opts_.bucket_steps;
  }

  void build_buckets() {
    buckets_.clear();
    vertex_bucket_start_.assign(pv_.n_vertices, -1);
    vertex_n_buckets_.assign(pv_.n_vertices, {0, 0});

    int total = 0;
    for (int v = 0; v < pv_.n_vertices; ++v) {
      vertex_bucket_start_[v] = total;

      auto& vstep = step_for_vertex(v);
      std::array<int, 2> nb = {1, 1};
      for (int r = 0; r < n_main_; ++r) {
        double range = pv_.vertex_ub[r][v] - pv_.vertex_lb[r][v];
        nb[r] = std::max(
            1, static_cast<int>(std::ceil(range / vstep[r])));
      }
      vertex_n_buckets_[v] = nb;

      for (int k1 = 0; k1 < nb[0]; ++k1) {
        for (int k2 = 0; k2 < nb[1]; ++k2) {
          Bucket b;
          b.vertex = v;

          for (int r = 0; r < n_main_; ++r) {
            int k = (r == 0) ? k1 : k2;
            b.lb[r] = pv_.vertex_lb[r][v] + k * vstep[r];
            b.ub[r] =
                std::min(b.lb[r] + vstep[r], pv_.vertex_ub[r][v]);
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

    fw_labels_.resize(buckets_.size());
    bw_labels_.resize(buckets_.size());
    fixed_.resize(static_cast<int>(buckets_.size()));

    // Size scratch_visited_ to max buckets per vertex
    int max_bv = 0;
    for (int v = 0; v < pv_.n_vertices; ++v) {
      auto [vs, ve] = vertex_bucket_range(v);
      max_bv = std::max(max_bv, ve - vs);
    }
    scratch_visited_.resize(max_bv);
  }

  int vertex_bucket_index(int vertex, const std::array<double, 2>& q) const {
    int start = vertex_bucket_start_[vertex];
    auto& nb = vertex_n_buckets_[vertex];
    auto& vstep = step_for_vertex(vertex);

    std::array<int, 2> k = {0, 0};
    for (int r = 0; r < n_main_; ++r) {
      double offset = q[r] - pv_.vertex_lb[r][vertex];
      k[r] =
          std::min(static_cast<int>(offset / vstep[r]), nb[r] - 1);
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
            q_target[r] = std::max(src_b.lb[r] + d, pv_.vertex_lb[r][to_v]);
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
            q_target[r] = std::min(src_b.ub[r] - d, pv_.vertex_ub[r][from_v]);
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

    auto& scc_buckets =
        (dir == Direction::Forward) ? fw_scc_buckets_ : bw_scc_buckets_;
    auto& scc_topo =
        (dir == Direction::Forward) ? fw_scc_topo_order_ : bw_scc_topo_order_;
    auto& bucket_scc_id =
        (dir == Direction::Forward) ? fw_bucket_scc_id_ : bw_bucket_scc_id_;

    scc_topo.clear();
    bucket_scc_id.assign(n, -1);

    // Build adjacency
    std::vector<std::vector<int>> adj(n);
    for (int bi = 0; bi < n; ++bi) {
      auto& arcs = (dir == Direction::Forward) ? buckets_[bi].bucket_arcs
                                               : buckets_[bi].bw_bucket_arcs;
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
          if (k0 + 1 < nb[0]) adj[bi].push_back(start + (k0 + 1) * nb[1] + k1);
          if (k1 + 1 < nb[1]) adj[bi].push_back(start + k0 * nb[1] + k1 + 1);
        } else {
          // Backward: resource decreases → edge to previous bucket
          if (k0 > 0) adj[bi].push_back(start + (k0 - 1) * nb[1] + k1);
          if (k1 > 0) adj[bi].push_back(start + k0 * nb[1] + (k1 - 1));
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

  // ── Jump arc generation (Section 4.1, ObtainJumpBucketArcs) ──

  /// Generate jump arcs after arc elimination per Sadykov et al. 2021 §4.1.
  /// Jump arcs are SAME-VERTEX arcs: when a bucket arc for arc a is eliminated
  /// from bucket b, create a jump arc from b to a higher (fw) / lower (bw)
  /// same-vertex bucket b' that still has the bucket arc for a.
  /// During extension, the label's resource is boosted to b'.lb (fw) / b'.ub
  /// (bw).
  void obtain_jump_arcs(Direction dir) {
    for (auto& b : buckets_) {
      if (dir == Direction::Forward)
        b.jump_arcs.clear();
      else
        b.bw_jump_arcs.clear();
    }

    int nb = static_cast<int>(buckets_.size());
    int na = pv_.n_arcs;

    // CSR inverted index: arc_id → bucket indices that contain it
    scratch_arc_offset_.assign(na + 1, 0);
    for (int bi = 0; bi < nb; ++bi) {
      auto& arcs = (dir == Direction::Forward) ? buckets_[bi].bucket_arcs
                                               : buckets_[bi].bw_bucket_arcs;
      for (const auto& ba : arcs) scratch_arc_offset_[ba.arc_id + 1]++;
    }
    for (int a = 0; a < na; ++a) scratch_arc_offset_[a + 1] += scratch_arc_offset_[a];
    scratch_arc_bucket_data_.resize(scratch_arc_offset_[na]);
    {
      std::vector<int> arc_pos(scratch_arc_offset_.begin(), scratch_arc_offset_.end() - 1);
      for (int bi = 0; bi < nb; ++bi) {
        auto& arcs = (dir == Direction::Forward) ? buckets_[bi].bucket_arcs
                                                 : buckets_[bi].bw_bucket_arcs;
        for (const auto& ba : arcs) {
          scratch_arc_bucket_data_[arc_pos[ba.arc_id]++] = bi;
        }
      }
    }

    // Use precomputed adjacency lists
    const auto& arcs_by_vertex =
        (dir == Direction::Forward) ? adj_.outgoing : adj_.incoming;

    struct Candidate {
      int k0, k1, bi;
    };
    std::vector<Candidate> candidates;

    for (int v = 0; v < pv_.n_vertices; ++v) {
      if (arcs_by_vertex[v].empty()) continue;

      auto [vstart, vend] = vertex_bucket_range(v);
      auto& nb_v = vertex_n_buckets_[v];
      int n_buckets_v = vend - vstart;
      if (n_buckets_v <= 1) continue;

      for (int bi = vstart; bi < vend; ++bi) {
        if (fixed_.test(bi)) continue;
        int k0 = (bi - vstart) / nb_v[1];
        int k1 = (bi - vstart) % nb_v[1];

        for (int a : arcs_by_vertex[v]) {
          // CSR range of buckets that have arc a (sorted by bucket index)
          const int* csr_begin = scratch_arc_bucket_data_.data() + scratch_arc_offset_[a];
          const int* csr_end = scratch_arc_bucket_data_.data() + scratch_arc_offset_[a + 1];

          // Does bucket bi already have a bucket arc for arc a?
          if (std::binary_search(csr_begin, csr_end, bi)) continue;

          // Find B̄ = {b' at same vertex : b' ≻ b AND b' has arc a}
          // Forward: b' ≻ b means k' component-wise > k (jump UP)
          // Backward: b' ≻ b means k' component-wise < k (jump DOWN)
          candidates.clear();

          for (const int* p = csr_begin; p != csr_end; ++p) {
            int bi2 = *p;
            if (bi2 < vstart || bi2 >= vend || bi2 == bi) continue;
            int k0_2 = (bi2 - vstart) / nb_v[1];
            int k1_2 = (bi2 - vstart) % nb_v[1];

            bool succeeds;
            if (dir == Direction::Forward) {
              succeeds = (k0_2 >= k0 && k1_2 >= k1 && (k0_2 > k0 || k1_2 > k1));
            } else {
              succeeds = (k0_2 <= k0 && k1_2 <= k1 && (k0_2 < k0 || k1_2 < k1));
            }
            if (!succeeds) continue;

            candidates.push_back({k0_2, k1_2, bi2});
          }

          // Filter to Pareto front: component-wise minimal (fw) / maximal (bw)
          // O(n log n) sort+sweep instead of O(n²) pairwise check
          if (dir == Direction::Forward) {
            std::sort(candidates.begin(), candidates.end(),
                      [](const auto& lhs, const auto& rhs) {
                        return lhs.k0 < rhs.k0 ||
                               (lhs.k0 == rhs.k0 && lhs.k1 < rhs.k1);
                      });
            int min_k1 = std::numeric_limits<int>::max();
            for (auto& c : candidates) {
              if (c.k1 < min_k1) {
                buckets_[bi].jump_arcs.push_back({c.bi, a});
                min_k1 = c.k1;
              }
            }
          } else {
            std::sort(candidates.begin(), candidates.end(),
                      [](const auto& lhs, const auto& rhs) {
                        return lhs.k0 > rhs.k0 ||
                               (lhs.k0 == rhs.k0 && lhs.k1 > rhs.k1);
                      });
            int max_k1 = std::numeric_limits<int>::min();
            for (auto& c : candidates) {
              if (c.k1 > max_k1) {
                buckets_[bi].bw_jump_arcs.push_back({c.bi, a});
                max_k1 = c.k1;
              }
            }
          }
        }
      }
    }
  }

  // ── Label extension (direction-aware) ──

  Label<Pack>* extend_label(const Label<Pack>* parent, int arc_id,
                            Direction dir,
                            const std::array<double, 2>* q_boost = nullptr) {
    int new_v =
        (dir == Direction::Forward) ? pv_.arc_to[arc_id] : pv_.arc_from[arc_id];

    // Cost — compute on stack before allocating
    double extra_arc_cost =
        reduced_costs_ ? reduced_costs_[arc_id] : pv_.arc_base_cost[arc_id];
    double new_cost = parent->cost + extra_arc_cost;
    double new_real_cost = parent->real_cost + pv_.arc_base_cost[arc_id];

    // Main resource update — modified for jump arcs (paper §4.1)
    std::array<double, 2> new_q{};
    for (int r = 0; r < n_main_; ++r) {
      double d = pv_.arc_resource[r][arc_id];
      double q_base = parent->q[r];
      if (q_boost) {
        if (dir == Direction::Forward)
          q_base = std::max(q_base, (*q_boost)[r]);
        else
          q_base = std::min(q_base, (*q_boost)[r]);
      }
      if (dir == Direction::Forward) {
        new_q[r] = std::max(q_base + d, pv_.vertex_lb[r][new_v]);
        if (new_q[r] > pv_.vertex_ub[r][new_v]) return nullptr;
      } else {
        new_q[r] = std::min(q_base - d, pv_.vertex_ub[r][new_v]);
        if (new_q[r] < pv_.vertex_lb[r][new_v]) return nullptr;
      }
    }

    // Meta-Solver resource extension: extendAlongArc + extendToVertex
    typename Pack::StatesTuple new_resource_states{};
    if constexpr (Pack::size > 0) {
      auto [arc_states, arc_cost] =
          pack_.extend_along_arc(dir, parent->resource_states, arc_id);
      if (arc_cost >= INF) return nullptr;
      auto [vtx_states, vtx_cost] =
          pack_.extend_to_vertex(dir, arc_states, new_v);
      if (vtx_cost >= INF) return nullptr;
      new_resource_states = vtx_states;
      new_cost += arc_cost + vtx_cost;
    }

    // All checks passed — allocate and fill
    auto* L = pool_.allocate();
    L->vertex = new_v;
    L->dir = dir;
    L->parent = const_cast<Label<Pack>*>(parent);
    L->parent_arc = arc_id;
    L->cost = new_cost;
    L->real_cost = new_real_cost;
    L->q = new_q;
    L->extended = false;
    L->dominated = false;
    if constexpr (Pack::size > 0) {
      L->resource_states = new_resource_states;
    }
    L->bucket = vertex_bucket_index(new_v, new_q);
    return L;
  }

  // ── Dominance (direction-aware) ──

  bool dominates(const Label<Pack>* L1, const Label<Pack>* L2,
                 Direction dir) const {
    if (L1->vertex != L2->vertex) return false;

    for (int r = 0; r < n_main_; ++r) {
      if (main_nondisposable_[r]) {
        // Non-disposable: require equality
        if (std::abs(L1->q[r] - L2->q[r]) > EPS) return false;
      } else if (dir == Direction::Forward) {
        if (L1->q[r] > L2->q[r] + EPS) return false;
      } else {
        if (L1->q[r] < L2->q[r] - EPS) return false;
      }
    }

    // Heuristic stages: cost-only dominance (ignore ng/R1C states)
    if (opts_.stage == Stage::Heuristic1 || opts_.stage == Stage::Heuristic2) {
      return L1->cost <= L2->cost + EPS;
    }

    if constexpr (Pack::size == 0) {
      return L1->cost <= L2->cost + EPS;
    }

    // Cost pre-check: if L1->cost already exceeds L2->cost by more than the
    // maximum possible domination_cost reduction, L1 can never dominate.
    // Avoids expensive pack domination_cost() (ng bit ops, R1C state ops).
    if (L1->cost > L2->cost + EPS + (-min_dom_cost_)) return false;

    double dom_cost = L1->cost;

    if constexpr (Pack::size > 0) {
      dom_cost += pack_.domination_cost(dir, L1->vertex, L1->resource_states,
                                        L2->resource_states);
    }

    return dom_cost <= L2->cost + EPS;
  }

  /// Extract compressed ng bits from a label (self bit stripped).
  uint32_t label_ng_bits(const Label<Pack>* L) const {
    if constexpr (has_ng_) {
      auto& ng_res = std::get<ng_idx_>(pack_.resources);
      auto state = std::get<ng_idx_>(L->resource_states);
      return ng_res.compress_state(state, L->vertex);
    } else {
      return 0;
    }
  }

  void insert_sorted(BucketLabels& bl, int bi, Label<Pack>* L) const {
    auto& bucket_costs = bl.costs[bi];
    auto pos = std::lower_bound(bucket_costs.begin(), bucket_costs.end(), L->cost);
    auto idx = pos - bucket_costs.begin();
    bucket_costs.insert(pos, L->cost);
    bl.labels[bi].insert(bl.labels[bi].begin() + idx, L);
    bl.q0[bi].insert(bl.q0[bi].begin() + idx, L->q[0]);
    bl.q1[bi].insert(bl.q1[bi].begin() + idx, L->q[1]);
    if constexpr (has_ng_) {
      bl.ng_bits[bi].insert(bl.ng_bits[bi].begin() + idx, label_ng_bits(L));
    }
  }

#ifdef BGSPPRC_HAS_SIMD
  bool dominated_in_bucket_simd(
      const Label<Pack>* L, int bi, Direction dir,
      const BucketLabels& bl) const {
    namespace stdx = std::experimental;
    using simd_d = stdx::native_simd<double>;
    constexpr std::size_t W = simd_d::size();

    auto& bucket = bl.labels[bi];
    auto& bucket_costs = bl.costs[bi];
    auto& bucket_q0 = bl.q0[bi];
    auto& bucket_q1 = bl.q1[bi];
    const std::size_t n = bucket.size();
    const double threshold = L->cost + EPS;

    // Precompute new label's compressed ng bits for subset check
    [[maybe_unused]] uint32_t new_ng = 0;
    [[maybe_unused]] const uint32_t* ng_data = nullptr;
    if constexpr (has_ng_) {
      new_ng = label_ng_bits(L);
      ng_data = bl.ng_bits[bi].data();
    }

    // In heuristic stages, ng check is skipped (cost-only dominance)
    [[maybe_unused]] const bool check_ng =
        has_ng_ && opts_.stage != Stage::Heuristic1 &&
        opts_.stage != Stage::Heuristic2;

    std::size_t i = 0;
    for (; i + W <= n; i += W) {
      // Direct SIMD load from contiguous cost array — no gather needed
      simd_d cost_vec(bucket_costs.data() + i, stdx::element_aligned);
      auto mask = (cost_vec + min_dom_cost_) <= threshold;
      if (!stdx::any_of(mask)) return false;  // sorted: all remaining fail too

      // SIMD resource pre-filter on q[0]
      if (n_main_ >= 1) {
        simd_d q0_vec(bucket_q0.data() + i, stdx::element_aligned);
        if (dir == Direction::Forward) {
          mask = mask & (q0_vec <= simd_d(L->q[0] + EPS));
        } else {
          mask = mask & (q0_vec >= simd_d(L->q[0] - EPS));
        }
        if (!stdx::any_of(mask)) continue;
      }

      // SIMD resource pre-filter on q[1]
      if (n_main_ >= 2) {
        simd_d q1_vec(bucket_q1.data() + i, stdx::element_aligned);
        if (dir == Direction::Forward) {
          mask = mask & (q1_vec <= simd_d(L->q[1] + EPS));
        } else {
          mask = mask & (q1_vec >= simd_d(L->q[1] - EPS));
        }
        if (!stdx::any_of(mask)) continue;
      }

      for (std::size_t j = 0; j < W; ++j) {
        if (!mask[j]) continue;
        auto* existing = bucket[i + j];
        if (existing->dominated) continue;
        // Scalar ng subset check from SoA — avoids chasing label pointer
        if (check_ng && (ng_data[i + j] & ~new_ng)) continue;
        if (dominates(existing, L, dir)) return true;
      }
    }

    for (; i < n; ++i) {
      if (bucket_costs[i] + min_dom_cost_ > threshold) break;
      auto* existing = bucket[i];
      if (existing->dominated) continue;
      if (check_ng && (ng_data[i] & ~new_ng)) continue;
      if (dominates(existing, L, dir)) return true;
    }
    return false;
  }
#endif

  bool dominated_in_bucket(
      const Label<Pack>* L, int bi, Direction dir,
      const BucketLabels& bl) const {
#ifdef BGSPPRC_HAS_SIMD
    return dominated_in_bucket_simd(L, bi, dir, bl);
#else
    const double threshold = L->cost + EPS;
    auto& bucket_costs = bl.costs[bi];
    auto& bucket = bl.labels[bi];

    [[maybe_unused]] uint32_t new_ng = 0;
    [[maybe_unused]] const uint32_t* ng_data = nullptr;
    if constexpr (has_ng_) {
      new_ng = label_ng_bits(L);
      ng_data = bl.ng_bits[bi].data();
    }
    [[maybe_unused]] const bool check_ng =
        has_ng_ && opts_.stage != Stage::Heuristic1 &&
        opts_.stage != Stage::Heuristic2;

    for (std::size_t i = 0; i < bucket.size(); ++i) {
      if (bucket_costs[i] + min_dom_cost_ > threshold) break;  // sorted
      auto* existing = bucket[i];
      if (existing->dominated) continue;
      if (check_ng && (ng_data[i] & ~new_ng)) continue;
      if (dominates(existing, L, dir)) return true;
    }
    return false;
#endif
  }

  bool dominated_in_adjacent_buckets(
      const Label<Pack>* L, int bi, Direction dir,
      const BucketLabels& bl) const {
    int v = L->vertex;
    auto [start, end] = vertex_bucket_range(v);
    auto& nb = vertex_n_buckets_[v];

    int k0 = (bi - start) / nb[1];
    int k1 = (bi - start) % nb[1];
    const double threshold = L->cost + EPS;

    if (dir == Direction::Forward) {
      for (int i0 = 0; i0 <= k0; ++i0) {
        for (int i1 = 0; i1 <= k1; ++i1) {
          int other = start + i0 * nb[1] + i1;
          if (other == bi) continue;
          if (buckets_[other].c_best + min_dom_cost_ > threshold) continue;
          auto& ocosts = bl.costs[other];
          auto& obucket = bl.labels[other];
          for (std::size_t idx = 0; idx < obucket.size(); ++idx) {
            if (ocosts[idx] + min_dom_cost_ > threshold) break;
            auto* existing = obucket[idx];
            if (existing->dominated) continue;
            if (dominates(existing, L, dir)) return true;
          }
        }
      }
    } else {
      for (int i0 = k0; i0 < nb[0]; ++i0) {
        for (int i1 = k1; i1 < nb[1]; ++i1) {
          int other = start + i0 * nb[1] + i1;
          if (other == bi) continue;
          if (buckets_[other].bw_c_best + min_dom_cost_ > threshold)
            continue;
          auto& ocosts = bl.costs[other];
          auto& obucket = bl.labels[other];
          for (std::size_t idx = 0; idx < obucket.size(); ++idx) {
            if (ocosts[idx] + min_dom_cost_ > threshold) break;
            auto* existing = obucket[idx];
            if (existing->dominated) continue;
            if (dominates(existing, L, dir)) return true;
          }
        }
      }
    }
    return false;
  }

  void remove_dominated(const Label<Pack>* new_label, int bi, Direction dir,
                        BucketLabels& bl) {
    for (auto* existing : bl.labels[bi]) {
      if (!existing->dominated && dominates(new_label, existing, dir)) {
        existing->dominated = true;
      }
    }
  }

  // ── SCC processing (unified for both directions) ──

  /// Try to insert a label into its bucket. In Enumerate stage, uses
  /// completion-bound pruning instead of dominance.
  bool try_insert_label(Label<Pack>* new_label, int actual_bi, Direction dir,
                        BucketLabels& bl,
                        int& label_count) {
    if (fixed_.test(actual_bi)) return false;

    if (opts_.stage == Stage::Enumerate) {
      auto& completion =
          (dir == Direction::Forward) ? fw_completion_ : bw_completion_;
      if (!completion.empty() && completion[actual_bi] < INF &&
          new_label->cost + completion[actual_bi] >= opts_.theta)
        return false;
      bl.labels[actual_bi].push_back(new_label);
      bl.costs[actual_bi].push_back(new_label->cost);
      bl.q0[actual_bi].push_back(new_label->q[0]);
      bl.q1[actual_bi].push_back(new_label->q[1]);
      if constexpr (has_ng_) {
        bl.ng_bits[actual_bi].push_back(label_ng_bits(new_label));
      }
      ++label_count;
      ++total_enum_labels_;
      if (buckets_[actual_bi].vertex == pv_.sink) ++enum_sink_labels_;
      return true;
    }

    // Normal: dominance check
    if (!dominated_in_bucket(new_label, actual_bi, dir, bl)) {
      remove_dominated(new_label, actual_bi, dir, bl);
      insert_sorted(bl, actual_bi, new_label);
      ++label_count;
      if (dir == Direction::Forward)
        ++fw_label_count_;
      else
        ++bw_label_count_;
      return true;
    }
    return false;
  }

  void process_scc(int scc_id, Direction dir,
                   const std::vector<std::vector<int>>& scc_buckets,
                   BucketLabels& bl,
                   double midpoint) {
    auto& scc_bs = scc_buckets[scc_id];
    if (scc_bs.empty()) return;

    const bool enumerating = (opts_.stage == Stage::Enumerate);
    // Enumeration needs more labels per SCC since dominance is disabled.
    const int scc_cap = enumerating ? 2000000 : 500000;
    int label_count = 0;

    // Check all label caps; marks enumeration incomplete on hit.
    auto at_label_cap = [&]() -> bool {
      if (label_count >= scc_cap) {
        if (enumerating) enum_complete_ = false;
        return true;
      }
      if (enumerating && total_enum_labels_ >= opts_.max_enum_labels) {
        enum_complete_ = false;
        return true;
      }
      if (enumerating && opts_.max_paths > 0 && enum_sink_labels_ >= opts_.max_paths) {
        enum_complete_ = false;
        return true;
      }
      return false;
    };

    bool changed = true;
    while (changed) {
      changed = false;
      for (int bi : scc_bs) {
        if (fixed_.test(bi)) continue;
        if (at_label_cap()) break;

        auto& bucket_labels = bl.labels[bi];
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

          if (enumerating) {
            // Completion-bound pruning on label before extending
            auto& completion =
                (dir == Direction::Forward) ? fw_completion_ : bw_completion_;
            if (!completion.empty() && completion[bi] < INF &&
                label->cost + completion[bi] >= opts_.theta) {
              label->extended = true;
              continue;
            }
          } else {
            if (dominated_in_adjacent_buckets(label, bi, dir, bl)) {
              label->dominated = true;
              continue;
            }
          }

          // Exact completion bound pruning (§5): discard fw labels
          // past q* that have no θ-compatible bw label.
          const bool prune_past_mid = opts_.bidirectional && !opts_.symmetric &&
                                      !enumerating && dir == Direction::Forward;
          auto try_insert = [&](Label<Pack>* new_label) {
            if (!new_label) return;
            if (prune_past_mid && new_label->q[0] > midpoint &&
                !has_compatible_opposite(new_label, opts_.theta,
                                         bw_labels_))
              return;
            if (try_insert_label(new_label, new_label->bucket, dir, bl,
                                 label_count))
              changed = true;
          };

          // Extend along bucket arcs
          auto& arcs = (dir == Direction::Forward)
                           ? buckets_[bi].bucket_arcs
                           : buckets_[bi].bw_bucket_arcs;

          for (const auto& ba : arcs) {
            try_insert(extend_label(label, ba.arc_id, dir));
          }

          // Jump arcs (paper §4.1): extend with resource boost
          // Boost: fw → max(q, lb of jump bucket)
          //        bw → min(q, ub of jump bucket)
          auto& jarcs = (dir == Direction::Forward) ? buckets_[bi].jump_arcs
                                                    : buckets_[bi].bw_jump_arcs;
          for (const auto& ja : jarcs) {
            std::array<double, 2> boost = (dir == Direction::Forward)
                                              ? buckets_[ja.jump_bucket].lb
                                              : buckets_[ja.jump_bucket].ub;
            try_insert(extend_label(label, ja.arc_id, dir, &boost));
          }

          label->extended = true;
        }
      }
      if (at_label_cap()) break;
    }

    // Batch compaction: physically remove dominated labels at SCC boundary
    for (int bi : scc_bs) {
      std::erase_if(bl.labels[bi],
                     [](const Label<Pack>* L) { return L->dominated; });
      // Rebuild SoA arrays from surviving labels
      auto& surviving = bl.labels[bi];
      auto& c = bl.costs[bi];
      auto& q0 = bl.q0[bi];
      auto& q1 = bl.q1[bi];
      c.resize(surviving.size());
      q0.resize(surviving.size());
      q1.resize(surviving.size());
      for (std::size_t i = 0; i < surviving.size(); ++i) {
        c[i] = surviving[i]->cost;
        q0[i] = surviving[i]->q[0];
        q1[i] = surviving[i]->q[1];
      }
      if constexpr (has_ng_) {
        auto& ng = bl.ng_bits[bi];
        ng.resize(surviving.size());
        for (std::size_t i = 0; i < surviving.size(); ++i) {
          ng[i] = label_ng_bits(surviving[i]);
        }
      }
    }

    update_c_best(scc_id, dir, scc_buckets, bl);
  }

  // ── c_best update ──

  void update_c_best(int scc_id, Direction dir,
                     const std::vector<std::vector<int>>& scc_buckets,
                     const BucketLabels& bl) {
    auto& scc_bs = scc_buckets[scc_id];

    // First pass: c_best from labels in bucket
    for (int bi : scc_bs) {
      double best = INF;
      for (const auto* L : bl.labels[bi]) {
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
              buckets_[bi].c_best =
                  std::min(buckets_[bi].c_best, buckets_[prev].c_best);
            }
            if (k1 > 0) {
              int prev = start + k0 * nb[1] + (k1 - 1);
              buckets_[bi].c_best =
                  std::min(buckets_[bi].c_best, buckets_[prev].c_best);
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
              buckets_[bi].bw_c_best =
                  std::min(buckets_[bi].bw_c_best, buckets_[next].bw_c_best);
            }
            if (k1 + 1 < nb[1]) {
              int next = start + k0 * nb[1] + (k1 + 1);
              buckets_[bi].bw_c_best =
                  std::min(buckets_[bi].bw_c_best, buckets_[next].bw_c_best);
            }
          }
        }
      }
    }
  }

  double completion_bound(int bi) const { return buckets_[bi].c_best; }

  // ── Completion bound computation (cost-to-go) ──

  /// Compute backward completion bounds for arc elimination / bucket fixing.
  /// completion(b) = lower bound on cost from bucket b to the terminal
  ///   (sink for forward, source for backward).
  /// Uses reverse topological order on the DAG of SCCs.
  void compute_completion_bounds(Direction dir) {
    int nb = static_cast<int>(buckets_.size());
    auto& completion =
        (dir == Direction::Forward) ? fw_completion_ : bw_completion_;
    completion.assign(nb, INF);

    // Terminal buckets: cost-to-go = 0
    int terminal = (dir == Direction::Forward) ? pv_.sink : pv_.source;
    auto [t_start, t_end] = vertex_bucket_range(terminal);
    for (int bi = t_start; bi < t_end; ++bi) {
      completion[bi] = 0.0;
    }

    auto& scc_topo =
        (dir == Direction::Forward) ? fw_scc_topo_order_ : bw_scc_topo_order_;
    auto& scc_buckets =
        (dir == Direction::Forward) ? fw_scc_buckets_ : bw_scc_buckets_;

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
          auto& arcs = (dir == Direction::Forward)
                           ? buckets_[bi].bucket_arcs
                           : buckets_[bi].bw_bucket_arcs;
          for (const auto& ba : arcs) {
            if (completion[ba.to_bucket] >= INF) continue;
            double arc_cost = reduced_costs_ ? reduced_costs_[ba.arc_id]
                                             : pv_.arc_base_cost[ba.arc_id];
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
            completion[bi] = std::min(completion[bi], completion[next]);
          }
        }
        for (int k0 = vnb[0] - 1; k0 >= 0; --k0) {
          for (int k1 = vnb[1] - 2; k1 >= 0; --k1) {
            int bi = start + k0 * vnb[1] + k1;
            int next = start + k0 * vnb[1] + (k1 + 1);
            completion[bi] = std::min(completion[bi], completion[next]);
          }
        }
      } else {
        // Backward: higher k = more slack → propagate lower→higher
        for (int k0 = 1; k0 < vnb[0]; ++k0) {
          for (int k1 = 0; k1 < vnb[1]; ++k1) {
            int bi = start + k0 * vnb[1] + k1;
            int prev = start + (k0 - 1) * vnb[1] + k1;
            completion[bi] = std::min(completion[bi], completion[prev]);
          }
        }
        for (int k0 = 0; k0 < vnb[0]; ++k0) {
          for (int k1 = 1; k1 < vnb[1]; ++k1) {
            int bi = start + k0 * vnb[1] + k1;
            int prev = start + k0 * vnb[1] + (k1 - 1);
            completion[bi] = std::min(completion[bi], completion[prev]);
          }
        }
      }
    }
  }

  // ── Label-based arc elimination (Section 4.2) ──

  /// Check if a forward label L and backward label L_tilde form a
  /// θ-compatible pair when joined through arc a.
  bool is_theta_compatible(const Label<Pack>* fw, const Label<Pack>* bw,
                           int arc_id, double theta) const {
    // Cheapest check first: pure cost bound (no resource/pack ops).
    // Account for min_dom_cost_ as lower bound on pack concatenation_cost
    // (can be negative for R1C with negative duals; 0 for ng-path).
    double arc_cost =
        reduced_costs_ ? reduced_costs_[arc_id] : pv_.arc_base_cost[arc_id];
    double total_cost = fw->cost + bw->cost + arc_cost;
    if (total_cost + min_dom_cost_ >= theta) return false;

    // Resource feasibility
    int j = pv_.arc_to[arc_id];
    for (int r = 0; r < n_main_; ++r) {
      double fw_after = fw->q[r] + pv_.arc_resource[r][arc_id];
      fw_after = std::max(fw_after, pv_.vertex_lb[r][j]);
      if (fw_after > bw->q[r] + EPS) return false;
    }

    if constexpr (Pack::size > 0) {
      auto [ext_states, ext_cost] = pack_.extend_along_arc(
          Direction::Forward, fw->resource_states, arc_id);
      if (ext_cost >= INF) return false;
      total_cost += ext_cost;

      double cc = pack_.concatenation_cost(Symmetry::Asymmetric, j, ext_states,
                                           bw->resource_states);
      if (cc >= INF) return false;
      total_cost += cc;
    }

    return total_cost < theta;
  }

  /// Post-pass: mark bw labels past midpoint as dominated if no θ-compatible
  /// fw label exists via incoming arcs.  Mirror of has_compatible_opposite.
  void prune_bw_past_midpoint(double mu, double theta) {
    bw_labels_pruned_ = 0;
    for (int v = 0; v < pv_.n_vertices; ++v) {
      if (v == pv_.source || v == pv_.sink) continue;
      auto [start, end] = vertex_bucket_range(v);
      for (int bi = start; bi < end; ++bi) {
        for (auto* bw : bw_labels_.labels[bi]) {
          if (bw->dominated || bw->q[0] >= mu) continue;
          bool found = false;
          for (int arc_id : adj_.incoming[v]) {
            double arc_cost = reduced_costs_ ? reduced_costs_[arc_id]
                                             : pv_.arc_base_cost[arc_id];
            double bw_base = bw->cost + arc_cost;
            int i = pv_.arc_from[arc_id];
            auto [istart, iend] = vertex_bucket_range(i);
            for (int fbi = istart; fbi < iend && !found; ++fbi) {
              // Bucket-level c_best skip
              if (buckets_[fbi].c_best + bw_base + min_dom_cost_ >= theta)
                continue;
              // Cost-sorted early break within bucket
              auto& fw_costs = fw_labels_.costs[fbi];
              auto& fw_labs = fw_labels_.labels[fbi];
              for (std::size_t idx = 0; idx < fw_labs.size(); ++idx) {
                if (fw_costs[idx] + bw_base + min_dom_cost_ >= theta)
                  break;  // sorted
                if (fw_labs[idx]->dominated) continue;
                if (is_theta_compatible(fw_labs[idx], bw, arc_id, theta)) {
                  found = true;
                  break;
                }
              }
            }
            if (found) break;
          }
          if (!found) {
            bw->dominated = true;
            ++bw_labels_pruned_;
          }
        }
      }
    }
  }

  /// Check if a forward label past q* has any θ-compatible backward label
  /// via across-arc concatenation (BucketGraph 2021 §5 exact completion
  /// bounds). Iterates outgoing arcs (v,j), checks bw labels at j.
  bool has_compatible_opposite(
      const Label<Pack>* L, double theta,
      const BucketLabels& bw_bl) const {
    int v = L->vertex;
    if (v == pv_.source || v == pv_.sink) return true;

    for (int arc_id : adj_.outgoing[v]) {
      double arc_cost = reduced_costs_ ? reduced_costs_[arc_id]
                                       : pv_.arc_base_cost[arc_id];
      double base = L->cost + arc_cost;
      int j = pv_.arc_to[arc_id];
      auto [jstart, jend] = vertex_bucket_range(j);
      for (int bi = jstart; bi < jend; ++bi) {
        // Bucket-level c_best skip
        if (base + buckets_[bi].bw_c_best + min_dom_cost_ >= theta) continue;
        // Cost-sorted early break within bucket
        auto& bw_costs = bw_bl.costs[bi];
        auto& bw_labels = bw_bl.labels[bi];
        for (std::size_t idx = 0; idx < bw_labels.size(); ++idx) {
          if (base + bw_costs[idx] + min_dom_cost_ >= theta) break;  // sorted
          if (bw_labels[idx]->dominated) continue;
          if (is_theta_compatible(L, bw_labels[idx], arc_id, theta))
            return true;
        }
      }
    }
    return false;
  }

  /// Compute the arrival bucket index for a bucket arc or jump arc.
  /// For forward: q_arr = max(src_lb + arc_resource, vertex_lb of head)
  ///   then find which backward bucket at head contains q_arr.
  /// Returns -1 if infeasible.
  int compute_arrival_bucket(int src_bi, int arc_id, Direction dir,
                             bool is_jump, int jump_bi = -1) const {
    int head_v =
        (dir == Direction::Forward) ? pv_.arc_to[arc_id] : pv_.arc_from[arc_id];

    std::array<double, 2> q_arr = {};
    for (int r = 0; r < n_main_; ++r) {
      double d = pv_.arc_resource[r][arc_id];
      double base_q;
      if (dir == Direction::Forward) {
        base_q = is_jump ? buckets_[jump_bi].lb[r] : buckets_[src_bi].lb[r];
        q_arr[r] = std::max(base_q + d, pv_.vertex_lb[r][head_v]);
        if (q_arr[r] > pv_.vertex_ub[r][head_v]) return -1;
      } else {
        base_q = is_jump ? buckets_[jump_bi].ub[r] : buckets_[src_bi].ub[r];
        q_arr[r] = std::min(base_q - d, pv_.vertex_ub[r][head_v]);
        if (q_arr[r] < pv_.vertex_lb[r][head_v]) return -1;
      }
    }
    return vertex_bucket_index(head_v, q_arr);
  }

  /// Paper's UpdateBucketsSet procedure (Section 4.2, page 19).
  /// Recursively checks for θ-compatible pairs starting from bucket b_tilde
  /// in the opposite direction's label set. `visited` is indexed by
  /// vertex-local offset (bi - vstart), not global bucket index.
  void update_buckets_set(
      double theta, const Label<Pack>* L, int arc_id,
      std::span<uint64_t>
          b_bar,      // per opposite-bucket bitset: found compatible?
      int b_tilde,    // current opposite-sense bucket to check
      Direction dir,  // direction of L (fw → checking bw buckets)
      int vstart, int n_buckets_v, uint8_t* visited) const {
    int local = b_tilde - vstart;
    if (visited[local]) return;
    visited[local] = 1;

    // Cost bound prune: c̄^L + c̄_{arc} + c̃^best_{b̃} ≥ θ
    double arc_cost =
        reduced_costs_ ? reduced_costs_[arc_id] : pv_.arc_base_cost[arc_id];
    double opp_c_best = (dir == Direction::Forward)
                            ? buckets_[b_tilde].bw_c_best
                            : buckets_[b_tilde].c_best;
    if (L->cost + arc_cost + opp_c_best + min_dom_cost_ >= theta) return;

    // Check labels at b_tilde for θ-compatibility
    if (!((b_bar[b_tilde / 64] >> (b_tilde % 64)) & 1)) {
      auto& opp_labels = (dir == Direction::Forward)
                             ? bw_labels_.labels[b_tilde]
                             : fw_labels_.labels[b_tilde];
      auto& opp_costs = (dir == Direction::Forward)
                             ? bw_labels_.costs[b_tilde]
                             : fw_labels_.costs[b_tilde];
      double base = L->cost + arc_cost;
      for (std::size_t idx = 0; idx < opp_labels.size(); ++idx) {
        if (base + opp_costs[idx] + min_dom_cost_ >= theta) break;  // sorted
        if (opp_labels[idx]->dominated) continue;
        bool compat = (dir == Direction::Forward)
                          ? is_theta_compatible(L, opp_labels[idx], arc_id, theta)
                          : is_theta_compatible(opp_labels[idx], L, arc_id, theta);
        if (compat) {
          b_bar[b_tilde / 64] |= (1ULL << (b_tilde % 64));
          break;
        }
      }
    }

    // Recurse to adjacent smaller (opposite-sense) buckets: Φ_{b̃}
    int v = buckets_[b_tilde].vertex;
    auto& nb_v = vertex_n_buckets_[v];
    int k0 = (b_tilde - vstart) / nb_v[1];
    int k1 = (b_tilde - vstart) % nb_v[1];

    // Φ for opposite-sense buckets:
    // Eliminating forward arcs → b̃ is backward bucket → Φ goes to higher k
    // Eliminating backward arcs → b̃ is forward bucket → Φ goes to lower k
    if (dir == Direction::Forward) {
      if (k0 + 1 < nb_v[0]) {
        int adj = vstart + (k0 + 1) * nb_v[1] + k1;
        update_buckets_set(theta, L, arc_id, b_bar, adj, dir, vstart,
                           n_buckets_v, visited);
      }
      if (k1 + 1 < nb_v[1]) {
        int adj = vstart + k0 * nb_v[1] + (k1 + 1);
        update_buckets_set(theta, L, arc_id, b_bar, adj, dir, vstart,
                           n_buckets_v, visited);
      }
    } else {
      if (k0 > 0) {
        int adj = vstart + (k0 - 1) * nb_v[1] + k1;
        update_buckets_set(theta, L, arc_id, b_bar, adj, dir, vstart,
                           n_buckets_v, visited);
      }
      if (k1 > 0) {
        int adj = vstart + k0 * nb_v[1] + (k1 - 1);
        update_buckets_set(theta, L, arc_id, b_bar, adj, dir, vstart,
                           n_buckets_v, visited);
      }
    }
  }

  /// Check if a bucket arc should be eliminated (no θ-compatible pair found).
  bool should_eliminate_arc(int arr_bi, std::span<const uint64_t> b_bar,
                            Direction dir) const {
    int arr_v = buckets_[arr_bi].vertex;
    auto [arr_vstart, arr_vend] = vertex_bucket_range(arr_v);
    auto& arr_nb = vertex_n_buckets_[arr_v];
    int arr_k0 = (arr_bi - arr_vstart) / arr_nb[1];
    int arr_k1 = (arr_bi - arr_vstart) % arr_nb[1];

    for (int bi2 = arr_vstart; bi2 < arr_vend; ++bi2) {
      if (!((b_bar[bi2 / 64] >> (bi2 % 64)) & 1)) continue;
      int k0_2 = (bi2 - arr_vstart) / arr_nb[1];
      int k1_2 = (bi2 - arr_vstart) % arr_nb[1];

      if (dir == Direction::Forward) {
        if (k0_2 >= arr_k0 && k1_2 >= arr_k1) return false;
      } else {
        if (k0_2 <= arr_k0 && k1_2 <= arr_k1) return false;
      }
    }
    return true;
  }

  /// Process a single bucket during label-based elimination.
  /// GetBBar: callable(int arc_id) → std::span<uint64_t>
  template <typename GetBBar>
  void process_bucket_elimination(int bi, Direction dir, double theta,
                                  GetBBar&& get_bbar) {
    if (fixed_.test(bi)) return;

    // Get labels at bucket b
    auto& labels_b = (dir == Direction::Forward) ? fw_labels_.labels[bi]
                                                 : bw_labels_.labels[bi];

    // Helper: call update_buckets_set with vertex-local visited
    auto do_update = [&](const Label<Pack>* L, int arc_id, int arr_bi) {
      int arr_v = buckets_[arr_bi].vertex;
      auto [vs, ve] = vertex_bucket_range(arr_v);
      int nbv = ve - vs;
      std::fill_n(scratch_visited_.data(), nbv, uint8_t(0));
      update_buckets_set(theta, L, arc_id, get_bbar(arc_id), arr_bi, dir, vs,
                         nbv, scratch_visited_.data());
    };

    // Process jump arcs: UpdateBucketsSet for each (L, ψ)
    auto& jarcs = (dir == Direction::Forward) ? buckets_[bi].jump_arcs
                                              : buckets_[bi].bw_jump_arcs;
    for (const auto& ja : jarcs) {
      int arr_bi =
          compute_arrival_bucket(bi, ja.arc_id, dir, true, ja.jump_bucket);
      if (arr_bi < 0) continue;

      for (const auto* L : labels_b) {
        if (L->dominated) continue;
        do_update(L, ja.arc_id, arr_bi);
      }
    }

    // Process bucket arcs: UpdateBucketsSet, then check elimination
    auto& bucket_arcs = (dir == Direction::Forward)
                            ? buckets_[bi].bucket_arcs
                            : buckets_[bi].bw_bucket_arcs;

    int write = 0;
    for (int read = 0; read < static_cast<int>(bucket_arcs.size()); ++read) {
      const auto& ba = bucket_arcs[read];
      int arr_bi = compute_arrival_bucket(bi, ba.arc_id, dir, false);

      bool eliminate = (arr_bi < 0);
      if (!eliminate) {
        for (const auto* L : labels_b) {
          if (L->dominated) continue;
          do_update(L, ba.arc_id, arr_bi);
        }
        eliminate = should_eliminate_arc(arr_bi, get_bbar(ba.arc_id), dir);
      }

      if (!eliminate) {
        bucket_arcs[write++] = ba;
      }
    }
    bucket_arcs.resize(write);
  }

  /// Paper's BucketArcElimination procedure (Section 4.2, page 20).
  /// B̄ sets are per-(arc, source bucket), scoped per vertex and propagated
  /// through Φ_b predecessors within the same vertex.
  void bucket_arc_elimination(double theta, Direction dir) {
    int nb = static_cast<int>(buckets_.size());
    int n_words = (nb + 63) / 64;

    // Use adjacency lists and compute arc_id → local index mapping
    const auto& arcs_by_vertex =
        (dir == Direction::Forward) ? adj_.outgoing : adj_.incoming;
    scratch_arc_local_.resize(pv_.n_arcs);
    for (int v = 0; v < pv_.n_vertices; ++v) {
      for (int i = 0; i < static_cast<int>(arcs_by_vertex[v].size()); ++i)
        scratch_arc_local_[arcs_by_vertex[v][i]] = i;
    }

    // Reusable flat buffer for B̄ bitsets — sized to max across all vertices
    int max_flat_size = 0;
    for (int v = 0; v < pv_.n_vertices; ++v) {
      const auto& arcs_v = arcs_by_vertex[v];
      if (arcs_v.empty()) continue;
      auto [vs, ve] = vertex_bucket_range(v);
      int n_bv = ve - vs;
      int entries = static_cast<int>(arcs_v.size()) * n_bv;
      max_flat_size = std::max(max_flat_size, entries * n_words);
    }
    if (static_cast<int>(scratch_b_bar_.size()) < max_flat_size)
      scratch_b_bar_.resize(max_flat_size);

    for (int v = 0; v < pv_.n_vertices; ++v) {
      const auto& arcs_from_v = arcs_by_vertex[v];
      if (arcs_from_v.empty()) continue;

      auto [vstart, vend] = vertex_bucket_range(v);
      auto& nb_v = vertex_n_buckets_[v];
      int n_bv = vend - vstart;

      // Per-(local_arc, local_bucket) B̄ bitsets — flat, reused
      int n_arcs_v = static_cast<int>(arcs_from_v.size());
      int n_entries = n_arcs_v * n_bv;
      int flat_size = n_entries * n_words;
      std::fill_n(scratch_b_bar_.data(), flat_size, uint64_t(0));
      auto b_bar = [&](int idx) -> std::span<uint64_t> {
        return {scratch_b_bar_.data() + idx * n_words,
                static_cast<size_t>(n_words)};
      };

      // Process a single bucket: propagate Φ, then run elimination
      auto process = [&](int k0, int k1) {
        int bi = vstart + k0 * nb_v[1] + k1;
        int local_bi = bi - vstart;
        if (fixed_.test(bi)) return;

        // Propagate B̄ from Φ_b predecessors (source-sense)
        for (int lai = 0; lai < n_arcs_v; ++lai) {
          auto cur = b_bar(lai * n_bv + local_bi);
          if (dir == Direction::Forward) {
            if (k0 > 0) {
              auto pred = b_bar(lai * n_bv + ((k0 - 1) * nb_v[1] + k1));
              for (int w = 0; w < n_words; ++w) cur[w] |= pred[w];
            }
            if (k1 > 0) {
              auto pred = b_bar(lai * n_bv + (k0 * nb_v[1] + (k1 - 1)));
              for (int w = 0; w < n_words; ++w) cur[w] |= pred[w];
            }
          } else {
            if (k0 + 1 < nb_v[0]) {
              auto pred = b_bar(lai * n_bv + ((k0 + 1) * nb_v[1] + k1));
              for (int w = 0; w < n_words; ++w) cur[w] |= pred[w];
            }
            if (k1 + 1 < nb_v[1]) {
              auto pred = b_bar(lai * n_bv + (k0 * nb_v[1] + (k1 + 1)));
              for (int w = 0; w < n_words; ++w) cur[w] |= pred[w];
            }
          }
        }

        // Process arcs at this bucket using per-bucket B̄
        auto get_bbar = [&](int arc_id) -> std::span<uint64_t> {
          return b_bar(scratch_arc_local_[arc_id] * n_bv + local_bi);
        };
        process_bucket_elimination(bi, dir, theta, get_bbar);
      };

      if (dir == Direction::Forward) {
        for (int k0 = 0; k0 < nb_v[0]; ++k0)
          for (int k1 = 0; k1 < nb_v[1]; ++k1) process(k0, k1);
      } else {
        for (int k0 = nb_v[0] - 1; k0 >= 0; --k0)
          for (int k1 = nb_v[1] - 1; k1 >= 0; --k1) process(k0, k1);
      }
    }
  }

  // ── Label storage management ──

  void reset_label_storage(BucketLabels& bl) {
    bl.resize(buckets_.size());
    for (std::size_t i = 0; i < bl.size(); ++i) {
      bl.labels[i].clear();
      bl.costs[i].clear();
      bl.q0[i].clear();
      bl.q1[i].clear();
      if constexpr (has_ng_) bl.ng_bits[i].clear();
    }
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
    L->parent = nullptr;
    L->parent_arc = -1;
    L->extended = false;
    L->dominated = false;

    if (dir == Direction::Forward) {
      L->vertex = pv_.source;
      for (int r = 0; r < n_main_; ++r) L->q[r] = pv_.vertex_lb[r][pv_.source];
    } else {
      L->vertex = pv_.sink;
      for (int r = 0; r < n_main_; ++r) L->q[r] = pv_.vertex_ub[r][pv_.sink];
    }

    L->resource_states = pack_.init_states(dir);
    if constexpr (Pack::size > 0) {
      auto [vtx_states, vtx_cost] =
          pack_.extend_to_vertex(dir, L->resource_states, L->vertex);
      if (vtx_cost >= INF) return nullptr;
      L->resource_states = vtx_states;
      L->cost += vtx_cost;
    }
    return L;
  }

  // ── Mono-directional solve ──

  std::vector<Path> solve_mono() {
    pool_.clear();
    reset_label_storage(fw_labels_);
    reset_c_best();
    if constexpr (Pack::size > 0)
      min_dom_cost_ = pack_.min_domination_cost();

    total_enum_labels_ = 0;
    enum_complete_ = true;
    enum_sink_labels_ = 0;
    if (opts_.stage == Stage::Enumerate) {
      compute_completion_bounds(Direction::Forward);
      if (fixed_.n_fixed() > 0 &&
          std::abs(opts_.theta - fixing_theta_) > EPS) {
        fprintf(stderr,
                "bgspprc: warning: enumerating with gap=%.6g but "
                "buckets were fixed with theta=%.6g; consider "
                "reset_elimination()\n",
                opts_.theta, fixing_theta_);
      }
    }

    auto* src = create_initial_label(Direction::Forward);
    if (!src) return {};  // infeasible source state
    int src_bi = vertex_bucket_index(pv_.source, src->q);
    src->bucket = src_bi;
    fw_labels_.labels[src_bi].push_back(src);
    fw_labels_.costs[src_bi].push_back(src->cost);
    fw_labels_.q0[src_bi].push_back(src->q[0]);
    fw_labels_.q1[src_bi].push_back(src->q[1]);
    if constexpr (has_ng_) fw_labels_.ng_bits[src_bi].push_back(label_ng_bits(src));
    if (opts_.stage == Stage::Enumerate) ++total_enum_labels_;

    // Inject warm labels from previous solve
    if (!warm_labels_.empty()) {
      inject_warm_labels(fw_labels_, Direction::Forward);
    }

    for (int scc : fw_scc_topo_order_) {
      process_scc(scc, Direction::Forward, fw_scc_buckets_, fw_labels_,
                  INF);
    }

    auto paths = extract_paths(fw_labels_);
    std::sort(paths.begin(), paths.end(), [](const Path& a, const Path& b) {
      return a.reduced_cost < b.reduced_cost;
    });
    if (opts_.max_paths > 0 && static_cast<int>(paths.size()) > opts_.max_paths) {
      paths.resize(opts_.max_paths);
      if (opts_.stage == Stage::Enumerate) enum_complete_ = false;
    }
    return paths;
  }

  // ── Bi-directional solve ──

  double get_midpoint() {
    if (!midpoint_initialized_) {
      double min_lb = INF, max_ub = -INF;
      for (int v = 0; v < pv_.n_vertices; ++v) {
        min_lb = std::min(min_lb, pv_.vertex_lb[0][v]);
        max_ub = std::max(max_ub, pv_.vertex_ub[0][v]);
      }
      midpoint_ = (min_lb + max_ub) / 2.0;
      midpoint_initialized_ = true;
    }
    return midpoint_;
  }

  void adjust_midpoint() {
    if (fw_label_count_ == 0 && bw_label_count_ == 0) return;
    double min_lb = INF, max_ub = -INF;
    for (int v = 0; v < pv_.n_vertices; ++v) {
      min_lb = std::min(min_lb, pv_.vertex_lb[0][v]);
      max_ub = std::max(max_ub, pv_.vertex_ub[0][v]);
    }
    if (fw_label_count_ > 1.2 * bw_label_count_) {
      midpoint_ += 0.05 * (max_ub - midpoint_);
    } else if (bw_label_count_ > 1.2 * fw_label_count_) {
      midpoint_ -= 0.05 * (midpoint_ - min_lb);
    }
  }

  std::vector<Path> solve_bidirectional() {
    pool_.clear();
    reset_label_storage(fw_labels_);
    reset_label_storage(bw_labels_);
    reset_c_best();
    if constexpr (Pack::size > 0)
      min_dom_cost_ = pack_.min_domination_cost();

    fw_label_count_ = 0;
    bw_label_count_ = 0;
    total_enum_labels_ = 0;
    enum_complete_ = true;
    enum_sink_labels_ = 0;
    if (opts_.stage == Stage::Enumerate) {
      compute_completion_bounds(Direction::Forward);
      compute_completion_bounds(Direction::Backward);
      if (fixed_.n_fixed() > 0 &&
          std::abs(opts_.theta - fixing_theta_) > EPS) {
        fprintf(stderr,
                "bgspprc: warning: enumerating with gap=%.6g but "
                "buckets were fixed with theta=%.6g; consider "
                "reset_elimination()\n",
                opts_.theta, fixing_theta_);
      }
    }

    double mu = get_midpoint();

    if (!opts_.symmetric) {
      // Backward labeling (run first so bw labels exist for fw
      // exact completion bound pruning — BucketGraph 2021 §5)
      auto* snk = create_initial_label(Direction::Backward);
      if (!snk) return {};  // infeasible sink state
      int snk_bi = vertex_bucket_index(pv_.sink, snk->q);
      snk->bucket = snk_bi;
      bw_labels_.labels[snk_bi].push_back(snk);
      bw_labels_.costs[snk_bi].push_back(snk->cost);
      bw_labels_.q0[snk_bi].push_back(snk->q[0]);
      bw_labels_.q1[snk_bi].push_back(snk->q[1]);
      if constexpr (has_ng_) bw_labels_.ng_bits[snk_bi].push_back(label_ng_bits(snk));
      if (opts_.stage == Stage::Enumerate) {
        ++total_enum_labels_;
        ++enum_sink_labels_;  // seed label is at sink
      }

      for (int scc : bw_scc_topo_order_) {
        process_scc(scc, Direction::Backward, bw_scc_buckets_,
                    bw_labels_, mu);
      }
    }

    // Forward labeling
    auto* src = create_initial_label(Direction::Forward);
    if (!src) return {};  // infeasible source state
    int src_bi = vertex_bucket_index(pv_.source, src->q);
    src->bucket = src_bi;
    fw_labels_.labels[src_bi].push_back(src);
    fw_labels_.costs[src_bi].push_back(src->cost);
    fw_labels_.q0[src_bi].push_back(src->q[0]);
    fw_labels_.q1[src_bi].push_back(src->q[1]);
    if constexpr (has_ng_) fw_labels_.ng_bits[src_bi].push_back(label_ng_bits(src));
    if (opts_.stage == Stage::Enumerate) ++total_enum_labels_;

    if (!warm_labels_.empty()) {
      inject_warm_labels(fw_labels_, Direction::Forward);
    }

    for (int scc : fw_scc_topo_order_) {
      process_scc(scc, Direction::Forward, fw_scc_buckets_, fw_labels_,
                  mu);
    }

    // Post-pass: prune bw labels past midpoint with no compatible fw label
    if (!opts_.symmetric && opts_.stage != Stage::Enumerate)
      prune_bw_past_midpoint(mu, opts_.theta);

    if (opts_.symmetric) {
      // Symmetric: populate bw_c_best from fw c_best via mirror
      populate_symmetric_bw_c_best();
    }

    if (opts_.stage == Stage::Exact && !opts_.symmetric) adjust_midpoint();

    // Collect paths: forward labels that reached sink + concatenations
    auto paths = extract_paths(fw_labels_);
    auto concat_paths = concatenate_and_extract();
    paths.insert(paths.end(), concat_paths.begin(), concat_paths.end());

    // Sort and limit
    std::sort(paths.begin(), paths.end(), [](const Path& a, const Path& b) {
      return a.reduced_cost < b.reduced_cost;
    });
    if (opts_.max_paths > 0 && static_cast<int>(paths.size()) > opts_.max_paths) {
      paths.resize(opts_.max_paths);
      if (opts_.stage == Stage::Enumerate) enum_complete_ = false;
    }
    return paths;
  }

  // ── Concatenation ──

  std::vector<Path> concatenate_and_extract() {
    std::vector<Path> paths;

    Symmetry sym = opts_.symmetric ? Symmetry::Symmetric : Symmetry::Asymmetric;

    auto append_bw_subpath = [this](Path& p, const Label<Pack>* bw) {
      scratch_path_verts_.clear();
      scratch_path_arcs_.clear();
      bw->get_backward_subpath(scratch_path_verts_, scratch_path_arcs_);
      for (std::size_t k = 1; k < scratch_path_verts_.size(); ++k)
        p.vertices.push_back(scratch_path_verts_[k]);
      p.arcs.insert(p.arcs.end(), scratch_path_arcs_.begin(), scratch_path_arcs_.end());
    };

    // Across-arc concatenation: for each arc a = (i,j), join fw labels at i
    // with bw labels at j using on-the-fly extend through arc a.
    // This finds paths crossing the bidir midpoint on a single arc.
    [&] {
      for (int a = 0; a < pv_.n_arcs; ++a) {
        int i = pv_.arc_from[a];
        int j = pv_.arc_to[a];
        if (i == pv_.source || i == pv_.sink) continue;
        if (j == pv_.source || j == pv_.sink) continue;

        double arc_cost =
            reduced_costs_ ? reduced_costs_[a] : pv_.arc_base_cost[a];
        double arc_real_cost = pv_.arc_base_cost[a];

        auto [fw_start, fw_end] = vertex_bucket_range(i);
        auto [bw_start, bw_end] = vertex_bucket_range(j);

        for (int fbi = fw_start; fbi < fw_end; ++fbi) {
          for (const auto* fw : fw_labels_.labels[fbi]) {
            if (fw->dominated) continue;

            // For EmptyPack, bw costs and resource adjustments are
            // non-negative, so fw + arc alone exceeding theta is sufficient.
            if constexpr (Pack::size == 0) {
              if (fw->cost + arc_cost >= opts_.theta) continue;
            }

            for (int bbi = bw_start; bbi < bw_end; ++bbi) {
              auto& bw_labels = opts_.symmetric
                  ? fw_labels_.labels[bbi] : bw_labels_.labels[bbi];
              for (const auto* bw : bw_labels) {
                if (bw->dominated) continue;

                // Cost pre-check before expensive resource feasibility and
                // pack ops. min_dom_cost_ is a lower bound on the combined
                // extend_along_arc + concatenation_cost from the pack (0 for
                // ng/standard resources, negative sum of betas for R1C).
                // NOTE: this reuses min_dom_cost_ (a domination_cost bound)
                // as a concatenation bound. Valid because extend_along_arc
                // returns cost >= 0 for all resources, and concatenation_cost
                // shares the same worst-case bound as domination_cost. If a
                // future resource breaks either assumption, add a dedicated
                // min_concatenation_cost() to the Resource concept.
                double total_cost = fw->cost + bw->cost + arc_cost;
                if (total_cost + min_dom_cost_ >= opts_.theta) continue;

                bool feasible = true;
                for (int r = 0; r < n_main_; ++r) {
                  double fw_after_arc = fw->q[r] + pv_.arc_resource[r][a];
                  fw_after_arc =
                      std::max(fw_after_arc, pv_.vertex_lb[r][j]);
                  double q_bw =
                      opts_.symmetric
                          ? (pv_.vertex_lb[r][j] + pv_.vertex_ub[r][j] -
                             bw->q[r])
                          : bw->q[r];
                  if (fw_after_arc > q_bw + EPS) {
                    feasible = false;
                    break;
                  }
                }
                if (!feasible) continue;
                double total_real_cost =
                    fw->real_cost + bw->real_cost + arc_real_cost;

                if constexpr (Pack::size > 0) {
                  auto [ext_states, ext_cost] = pack_.extend_along_arc(
                      Direction::Forward, fw->resource_states, a);
                  if (ext_cost >= INF) continue;
                  total_cost += ext_cost;

                  double cc = pack_.concatenation_cost(
                      sym, j, ext_states, bw->resource_states);
                  if (cc >= INF) continue;
                  total_cost += cc;
                }

                if (total_cost < opts_.theta) {
                  Path p;
                  fw->get_path(p.vertices, p.arcs);
                  p.arcs.push_back(a);
                  p.vertices.push_back(j);
                  if (opts_.symmetric) {
                    append_symmetric_bw_subpath(p, bw);
                  } else {
                    append_bw_subpath(p, bw);
                  }
                  p.reduced_cost = total_cost;
                  p.original_cost = total_real_cost;
                  paths.push_back(std::move(p));
                }
              }
            }
          }
        }
      }
    }();

    return paths;
  }

  // ── Path extraction ──

  std::vector<Path> extract_paths(
      const BucketLabels& bl) {
    std::vector<Path> paths;

    auto [start, end] = vertex_bucket_range(pv_.sink);
    for (int bi = start; bi < end; ++bi) {
      for (const auto* L : bl.labels[bi]) {
        if (L->dominated) continue;
        if (L->cost < opts_.theta) {
          Path p;
          L->get_path(p.vertices, p.arcs);
          p.reduced_cost = L->cost;
          p.original_cost = L->real_cost;
          paths.push_back(std::move(p));
        }
      }
    }

    return paths;
  }

  // ── Symmetric mode helpers ──

  /// Map a bucket to its resource-mirrored counterpart at the same vertex.
  /// With uniform windows, bucket κ = (k0, k1) mirrors to
  /// (nb[0]-1-k0, nb[1]-1-k1).
  int mirror_bucket(int bi) const {
    int v = buckets_[bi].vertex;
    int vstart = vertex_bucket_start_[v];
    auto& nb_v = vertex_n_buckets_[v];
    int k0 = (bi - vstart) / nb_v[1];
    int k1 = (bi - vstart) % nb_v[1];
    int mk0 = nb_v[0] - 1 - k0;
    int mk1 = nb_v[1] - 1 - k1;
    return vstart + mk0 * nb_v[1] + mk1;
  }

  /// After forward labeling, mirror c_best into bw_c_best.
  void populate_symmetric_bw_c_best() {
    for (int bi = 0; bi < static_cast<int>(buckets_.size()); ++bi) {
      int mbi = mirror_bucket(bi);
      buckets_[bi].bw_c_best = buckets_[mbi].c_best;
    }
  }

  /// Build arc lookup table for symmetric path reconstruction.
  void build_arc_lookup() {
    arc_lookup_.clear();
    arc_lookup_.reserve(pv_.n_arcs);
    for (int a = 0; a < pv_.n_arcs; ++a) {
      int64_t key = static_cast<int64_t>(pv_.arc_from[a]) * pv_.n_vertices +
                    pv_.arc_to[a];
      arc_lookup_[key] = a;
    }
  }

  /// Find arc id for (from, to). Returns -1 if not found.
  int find_arc(int from, int to) const {
    auto it =
        arc_lookup_.find(static_cast<int64_t>(from) * pv_.n_vertices + to);
    return (it != arc_lookup_.end()) ? it->second : -1;
  }

  /// Append the reversed forward path of a symmetric "bw" label.
  /// The bw label is actually a forward label; we reverse its path
  /// and look up opposite arcs.
  void append_symmetric_bw_subpath(Path& p, const Label<Pack>* bw) {
    scratch_path_verts_.clear();
    scratch_path_arcs_.clear();
    bw->get_path(scratch_path_verts_, scratch_path_arcs_);
    // scratch_path_verts_ = [source, ..., j], we need j→...→source reversed
    // j is already appended, so start from index size()-2
    for (int k = static_cast<int>(scratch_path_verts_.size()) - 2; k >= 0; --k) {
      p.vertices.push_back(scratch_path_verts_[k]);
      int opp = find_arc(scratch_path_verts_[k + 1], scratch_path_verts_[k]);
      assert(opp >= 0 && "symmetric mode requires opposite arc for every arc");
      p.arcs.push_back(opp);
    }
  }

  // ── Data ──

  const ProblemView& pv_;
  Pack pack_;
  Options opts_;
  Adjacency adj_;
  int n_main_ = 1;
  std::array<bool, 2> main_nondisposable_ = {false, false};

  const double* reduced_costs_ = nullptr;

  std::vector<Bucket> buckets_;
  std::vector<std::array<double, 2>> vertex_bucket_steps_;  // per-vertex steps (optional)
  BucketFixBitmap fixed_;
  std::vector<double> fw_completion_;  // forward cost-to-go (b → sink)
  std::vector<double> bw_completion_;  // backward cost-to-go (b → source)
  std::vector<int> vertex_bucket_start_;
  std::vector<std::array<int, 2>> vertex_n_buckets_;

  // Arc lookup for symmetric path reconstruction: (from*V + to) → arc_id
  std::unordered_map<int64_t, int> arc_lookup_;

  // Forward SCC data
  std::vector<std::vector<int>> fw_scc_buckets_;
  std::vector<int> fw_scc_topo_order_;
  std::vector<int> fw_bucket_scc_id_;

  // Backward SCC data
  std::vector<std::vector<int>> bw_scc_buckets_;
  std::vector<int> bw_scc_topo_order_;
  std::vector<int> bw_bucket_scc_id_;

  // Label storage (forward and backward)
  BucketLabels fw_labels_;
  BucketLabels bw_labels_;

  // Warm label storage: saved state from previous solve
  struct WarmLabel {
    int vertex;
    Direction dir;
    double cost;
    double real_cost;
    std::array<double, 2> q;
    int parent_arc;  // last arc (for path, not critical for warm start)
    typename Pack::StatesTuple resource_states;
  };
  std::vector<WarmLabel> warm_labels_;

  void collect_warm_labels(const BucketLabels& bl,
                           Direction dir, double fraction) {
    // Gather all non-dominated labels
    std::vector<const Label<Pack>*> all_labels;
    for (const auto& bucket : bl.labels) {
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
      warm_labels_.push_back(std::move(wl));
    }
  }

  void inject_warm_labels(BucketLabels& bl, Direction dir) {
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
      L->extended = false;
      L->dominated = false;
      L->resource_states = wl.resource_states;

      int bi = vertex_bucket_index(wl.vertex, wl.q);
      L->bucket = bi;

      if (!dominated_in_bucket(L, bi, dir, bl)) {
        insert_sorted(bl, bi, L);
      }
    }
  }

  int total_enum_labels_ = 0;  // global label counter for enumeration
  bool enum_complete_ = true;
  int enum_sink_labels_ = 0;   // labels that landed in sink buckets
  double fixing_theta_ = INF;  // theta used in last fix_buckets call

  // Adaptive midpoint for bidirectional labeling
  int fw_label_count_ = 0;
  int bw_label_count_ = 0;
  int bw_labels_pruned_ = 0;
  double midpoint_ = 0.0;
  bool midpoint_initialized_ = false;

  double min_dom_cost_ = 0.0;  // cached pack_.min_domination_cost()

  LabelPool<Pack> pool_;

  // Scratch buffers — reused across calls
  std::vector<uint8_t> scratch_visited_;
  std::vector<uint64_t> scratch_b_bar_;
  std::vector<int> scratch_arc_local_;
  std::vector<int> scratch_arc_offset_;        // obtain_jump_arcs CSR
  std::vector<int> scratch_arc_bucket_data_;   // obtain_jump_arcs CSR
  std::vector<int> scratch_path_verts_;        // path reconstruction
  std::vector<int> scratch_path_arcs_;         // path reconstruction
};

}  // namespace bgspprc
