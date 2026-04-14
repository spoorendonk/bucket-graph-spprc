#pragma once

#include "bucket_graph.h"
#include "executor.h"
#include "problem_view.h"
#include "resource.h"
#include "types.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace bgspprc {

/// Top-level solver wrapping BucketGraph with multi-stage control.
template <typename Pack, Executor Exec = SequentialExecutor>
class Solver {
public:
    using Path = typename BucketGraph<Pack, Exec>::Path;

    struct Options {
        std::array<double, 2> bucket_steps = {1.0, 1.0};
        bool bidirectional = true;
        bool symmetric = false;
        bool no_jump_arcs = false;   // disable jump arcs (for ablation studies)
        bool parallel_bidir = true;  // concurrent fw/bw labeling (requires parallel executor)
        int max_paths = 0;           // 0 = unlimited
        double theta = -1e-6;
        int max_enum_labels = 5000000;
    };

    Solver(const ProblemView& problem, Pack resources, Options opts = {}, Exec executor = {})
        : pv_(problem),
          pack_(std::move(resources)),
          opts_(opts),
          bg_(pv_, pack_,
              typename BucketGraph<Pack, Exec>::Options{
                  .bucket_steps = opts_.bucket_steps,
                  .max_paths = opts_.max_paths,
                  .theta = opts_.theta,
                  .bidirectional = opts_.bidirectional,
                  .symmetric = opts_.symmetric,
                  .no_jump_arcs = opts_.no_jump_arcs,
                  .parallel_bidir = opts_.parallel_bidir,
                  .stage = Stage::Exact,
                  .max_enum_labels = opts_.max_enum_labels,
              },
              std::move(executor)) {}

    /// Build bucket graph (call once, or after step size changes).
    void build() { bg_.build(); }

    /// Fast cost update — O(1), just stores the pointer.
    void update_arc_costs(std::span<const double> reduced_costs) {
        bg_.update_arc_costs(reduced_costs.data());
    }

    /// Solve SPPRC — returns paths with negative reduced cost.
    /// Uses multi-stage progression: Heuristic1 → Heuristic2 → Exact.
    /// In Exact stage, accumulates dominance stats for BG2021 §6.3 A+
    /// ξ-doubling: on CG convergence (empty exact pricing), checks if finer
    /// buckets would help and retries if so.
    std::vector<Path> solve() {
        accumulated_timings_.reset();
        bg_.set_stage(stage_);
        auto paths = bg_.solve();
        accumulated_timings_ += bg_.solve_timings();

        // Accumulate stats for A+ criterion during exact pricing
        if (stage_ == Stage::Exact) {
            total_dom_checks_ += bg_.dominance_checks();
            total_nondom_labels_ += bg_.non_dominated_labels();
            ++exact_pricing_calls_;

            // BG2021 §6.3 A+: on CG convergence (empty exact pricing),
            // check if finer buckets would help
            if (paths.empty() && try_refine_buckets()) {
                bg_.set_stage(stage_);
                paths = bg_.solve();
                accumulated_timings_ += bg_.solve_timings();
                total_dom_checks_ += bg_.dominance_checks();
                total_nondom_labels_ += bg_.non_dominated_labels();
                ++exact_pricing_calls_;
            }
        }

        update_stage(paths);
        return paths;
    }

    /// Arc elimination using optimality gap (bound-based).
    void eliminate_arcs(double theta) { bg_.eliminate_arcs(theta); }

    /// Label-based arc elimination (Section 4.2). Tighter but requires prior
    /// solve.
    void eliminate_arcs_label_based(double theta) { bg_.eliminate_arcs_label_based(theta); }

    /// Bucket fixing using optimality gap. Returns number of newly fixed buckets.
    int fix_buckets(double theta) { return bg_.fix_buckets(theta); }

    /// Number of fixed buckets.
    int n_fixed_buckets() const { return bg_.n_fixed_buckets(); }

    // ── Solve statistics ──

    /// Total bucket count.
    int n_buckets() const { return bg_.n_buckets(); }

    /// Labels allocated by the pool during the last solve.
    int64_t labels_created() const { return bg_.labels_created(); }

    /// Dominance comparisons performed in the last solve.
    int64_t dominance_checks() const { return bg_.dominance_checks(); }
    int64_t dominance_checks(Direction d) const { return bg_.dominance_checks(d); }

    /// Labels surviving dominance in the last solve.
    int64_t non_dominated_labels() const { return bg_.non_dominated_labels(); }
    int64_t non_dominated_labels(Direction d) const { return bg_.non_dominated_labels(d); }

    /// Bucket arcs eliminated since last build.
    int64_t eliminated_bucket_arcs() const { return bg_.eliminated_bucket_arcs(); }

    /// Size of the compile-time resource state tuple in bytes.
    static constexpr std::size_t label_state_size() { return Pack::label_state_size(); }

    /// Reset all elimination/fixing.
    void reset_elimination() { bg_.reset_elimination(); }

    /// Adaptive midpoint control for bidirectional labeling.
    void reset_midpoint() { bg_.reset_midpoint(); }
    double midpoint() const { return bg_.midpoint(); }

    /// Access a resource in the pack (for runtime reconfiguration).
    template <typename R>
    R& resource() {
        return bg_.template resource<R>();
    }

    /// Whether the last enumeration was complete (no caps hit).
    bool enumeration_complete() const { return bg_.enumeration_complete(); }

    /// Phase timing breakdown accumulated across all solve iterations
    /// (including A+ refinement retries).
    const SolveTimings& solve_timings() const { return accumulated_timings_; }

    /// Stage management.
    void set_stage(Stage stage) { stage_ = stage; }
    Stage current_stage() const { return stage_; }

    /// Path enumeration within gap.
    std::vector<Path> enumerate(double gap) {
        bg_.set_stage(Stage::Enumerate);
        bg_.set_theta(gap);
        auto paths = bg_.solve();
        bg_.set_theta(opts_.theta);
        bg_.set_stage(stage_);
        return paths;
    }

    /// Save warm labels for next solve (top fraction by cost).
    void save_warm_labels(double fraction = 0.7) { bg_.save_warm_labels(fraction); }

    /// Adaptive bucket step sizes. Returns true if steps changed (needs rebuild).
    bool adapt_bucket_steps(double threshold = 20.0) {
        bool changed = bg_.adapt_bucket_steps(threshold);
        if (changed) {
            bg_.build();
        }
        return changed;
    }

    /// Get current bucket steps.
    const std::array<double, 2>& bucket_steps() const { return bg_.bucket_steps(); }

    /// Set per-vertex bucket step sizes.
    void set_vertex_bucket_steps(std::vector<std::array<double, 2>> steps) {
        bg_.set_vertex_bucket_steps(std::move(steps));
    }

    /// Compute per-vertex steps from minimum inbound arc resource consumption.
    std::vector<std::array<double, 2>> compute_min_inbound_arc_resource(
        int max_buckets = 200) const {
        return bg_.compute_min_inbound_arc_resource(max_buckets);
    }

private:
    /// BG2021 §6.3 A+: check if bucket steps should be halved.
    /// Returns true if bucket graph was rebuilt with finer steps.
    bool try_refine_buckets() {
        if (exact_pricing_calls_ == 0) {
            return false;
        }

        double avg_ratio =
            static_cast<double>(total_dom_checks_) / std::max(total_nondom_labels_, int64_t{1});
        double arcs_per_vertex = static_cast<double>(bg_.non_fixed_arc_count()) / pv_.n_vertices;

        // Reset accumulators for next convergence cycle
        total_dom_checks_ = 0;
        total_nondom_labels_ = 0;
        exact_pricing_calls_ = 0;

        if (avg_ratio > 500.0 && arcs_per_vertex < 10000.0) {
            bg_.halve_bucket_steps();
            bg_.build();
            return true;
        }
        return false;
    }

    void update_stage(const std::vector<Path>& paths) {
        ++iteration_;

        if (stage_ == Stage::Heuristic1) {
            // Move to Heuristic2 if no neg-cost paths found or after 3 iters
            if (paths.empty() || iteration_ > 3) {
                stage_ = Stage::Heuristic2;
                iteration_ = 0;
            }
        } else if (stage_ == Stage::Heuristic2) {
            // Move to Exact if no neg-cost paths found or after 10 iters
            if (paths.empty() || iteration_ > 10) {
                stage_ = Stage::Exact;
            }
        }
    }

    const ProblemView& pv_;
    Pack pack_;
    Options opts_;
    BucketGraph<Pack, Exec> bg_;
    Stage stage_ = Stage::Heuristic1;
    int iteration_ = 0;

    // BG2021 §6.3 A+: accumulated stats across exact pricing calls
    int64_t total_dom_checks_ = 0;
    int64_t total_nondom_labels_ = 0;
    int exact_pricing_calls_ = 0;

    // Accumulated timings across solve iterations (including A+ retries)
    SolveTimings accumulated_timings_;
};

}  // namespace bgspprc
