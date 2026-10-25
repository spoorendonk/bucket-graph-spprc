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
        // A step <= 0 is derived from xi (BG2021 §6.3); see BucketGraph::Options.
        std::array<double, 2> bucket_steps = {0.0, 0.0};
        double xi = 25.0;  // BG2021 §6.3: initial maximum number of buckets per vertex
        bool bidirectional = true;
        bool symmetric = false;
        bool no_jump_arcs = false;   // disable jump arcs (for ablation studies)
        bool parallel_bidir = true;  // concurrent fw/bw labeling (requires parallel executor)
        int max_paths = 0;           // 0 = unlimited
        double theta = -1e-6;
        int max_enum_labels = 5000000;
        int max_scc_labels = 500000;  // see BucketGraph::Options
    };

    Solver(const ProblemView& problem, Pack resources, Options opts = {}, Exec executor = {})
        : pv_(problem),
          pack_(std::move(resources)),
          opts_(opts),
          bg_(pv_, pack_,
              typename BucketGraph<Pack, Exec>::Options{
                  .bucket_steps = opts_.bucket_steps,
                  .xi = opts_.xi,
                  .max_paths = opts_.max_paths,
                  .theta = opts_.theta,
                  .bidirectional = opts_.bidirectional,
                  .symmetric = opts_.symmetric,
                  .no_jump_arcs = opts_.no_jump_arcs,
                  .parallel_bidir = opts_.parallel_bidir,
                  .stage = Stage::Exact,
                  .max_enum_labels = opts_.max_enum_labels,
                  .max_scc_labels = opts_.max_scc_labels,
              },
              std::move(executor)) {}

    /// Build bucket graph (call once, or after step size changes).
    void build() {
        bg_.build();
        exact_complete_ = false;
        labelled_gap_ = NEG_INF;
    }

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
            // check if finer buckets would help. A capped solve proves no
            // convergence.
            if (paths.empty() && bg_.exact_complete() && try_refine_buckets()) {
                bg_.set_stage(stage_);
                paths = bg_.solve();
                accumulated_timings_ += bg_.solve_timings();
                total_dom_checks_ += bg_.dominance_checks();
                total_nondom_labels_ += bg_.non_dominated_labels();
                ++exact_pricing_calls_;
            }
        }

        exact_complete_ = bg_.exact_complete();
        labelled_gap_ = NEG_INF;
        update_stage(paths);
        return paths;
    }

    // Elimination and fixing need an exact prior solve. When it pruned below
    // the gap (BucketGraph::bounds_cover_gap), they first label again at the
    // gap (BucketGraph::label_bounds, #116): the per-solve statistics then
    // describe that labelling, save_warm_labels() keeps its labels, and the
    // reduced costs passed to update_arc_costs() must still be alive. A gap
    // labelling that hits max_scc_labels leaves them doing nothing, and is
    // not run again at that gap before the next solve.

    /// Arc elimination using optimality gap (bound-based).
    void eliminate_arcs(double theta) {
        cover_gap(theta, false);
        bg_.eliminate_arcs(theta);
    }

    /// Label-based arc elimination (Section 4.2). Tighter but requires prior
    /// solve.
    void eliminate_arcs_label_based(double theta) {
        cover_gap(theta, true);
        bg_.eliminate_arcs_label_based(theta);
    }

    /// Bucket fixing using optimality gap. Returns number of newly fixed buckets.
    int fix_buckets(double theta) {
        cover_gap(theta, false);
        return bg_.fix_buckets(theta);
    }

    /// Number of fixed buckets.
    [[nodiscard]] int n_fixed_buckets() const { return bg_.n_fixed_buckets(); }

    // ── Solve statistics ──

    /// Total bucket count.
    [[nodiscard]] int n_buckets() const { return bg_.n_buckets(); }

    /// Labels allocated by the pool during the last solve.
    [[nodiscard]] int64_t labels_created() const { return bg_.labels_created(); }

    /// Dominance comparisons performed in the last solve.
    [[nodiscard]] int64_t dominance_checks() const { return bg_.dominance_checks(); }
    [[nodiscard]] int64_t dominance_checks(Direction d) const { return bg_.dominance_checks(d); }

    /// Labels surviving dominance in the last solve.
    [[nodiscard]] int64_t non_dominated_labels() const { return bg_.non_dominated_labels(); }
    [[nodiscard]] int64_t non_dominated_labels(Direction d) const {
        return bg_.non_dominated_labels(d);
    }

    /// Bucket arcs eliminated since last build.
    [[nodiscard]] int64_t eliminated_bucket_arcs() const { return bg_.eliminated_bucket_arcs(); }

    /// Size of the compile-time resource state tuple in bytes.
    static constexpr std::size_t label_state_size() { return Pack::label_state_size(); }

    /// Reset all elimination/fixing.
    void reset_elimination() {
        bg_.reset_elimination();
        labelled_gap_ = NEG_INF;
    }

    /// Adaptive midpoint control for bidirectional labeling.
    void reset_midpoint() { bg_.reset_midpoint(); }
    [[nodiscard]] double midpoint() const { return bg_.midpoint(); }

    /// Access a resource in the pack (for runtime reconfiguration).
    template <typename R>
    R& resource() {
        return bg_.template resource<R>();
    }

    /// Whether the last enumeration was complete (no caps hit).
    [[nodiscard]] bool enumeration_complete() const { return bg_.enumeration_complete(); }

    /// Whether the last solve was exact and hit no label cap, so an empty
    /// result proves no path within theta exists; see BucketGraph::exact_complete().
    [[nodiscard]] bool exact_complete() const { return exact_complete_; }

    /// Phase timing breakdown accumulated across all solve iterations
    /// (including A+ refinement retries).
    [[nodiscard]] const SolveTimings& solve_timings() const { return accumulated_timings_; }

    /// Stage management.
    void set_stage(Stage stage) { stage_ = stage; }
    [[nodiscard]] Stage current_stage() const { return stage_; }

    /// Path enumeration within gap.
    std::vector<Path> enumerate(double gap) {
        bg_.set_stage(Stage::Enumerate);
        bg_.set_theta(gap);
        auto paths = bg_.solve();
        bg_.set_theta(opts_.theta);
        bg_.set_stage(stage_);
        exact_complete_ = bg_.exact_complete();
        labelled_gap_ = NEG_INF;
        return paths;
    }

    /// Save warm labels for next solve (top fraction by cost).
    void save_warm_labels(double fraction = 0.7) { bg_.save_warm_labels(fraction); }

    /// Adaptive bucket step sizes. Returns true if steps changed (needs rebuild).
    bool adapt_bucket_steps(double threshold = 20.0) {
        bool changed = bg_.adapt_bucket_steps(threshold);
        if (changed) {
            bg_.build();
            exact_complete_ = false;
            labelled_gap_ = NEG_INF;
        }
        return changed;
    }

    /// Get current bucket steps.
    [[nodiscard]] const std::array<double, 2>& bucket_steps() const { return bg_.bucket_steps(); }

    /// Current ξ (BG2021 §6.3), the nominal bucket count per vertex; see BucketGraph::xi().
    [[nodiscard]] double xi() const { return bg_.xi(); }

    /// Set per-vertex bucket step sizes.
    void set_vertex_bucket_steps(std::vector<std::array<double, 2>> steps) {
        bg_.set_vertex_bucket_steps(std::move(steps));
    }

    /// Compute per-vertex steps from minimum inbound arc resource consumption.
    [[nodiscard]] std::vector<std::array<double, 2>> compute_min_inbound_arc_resource(
        int max_buckets = 200) const {
        return bg_.compute_min_inbound_arc_resource(max_buckets);
    }

private:
    /// Label at `gap` when the last solve was exact but its bounds do not
    /// cover the gap, so elimination and fixing there keep every path below it.
    void cover_gap(double gap, bool label_based) {
        if (exact_complete_ && gap > labelled_gap_ && !bg_.bounds_cover_gap(gap, label_based)) {
            bg_.label_bounds(gap);
            labelled_gap_ = gap;
        }
    }

    /// BG2021 §6.3 A+: check if ξ should be doubled.
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
            bg_.double_xi();
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

    // exact_complete() of the last solve() or enumerate(), which the labelling
    // behind elimination and fixing (cover_gap) leaves as it was.
    bool exact_complete_ = false;
    // Largest gap cover_gap labelled at since the last solve, enumerate,
    // build or reset_elimination; NEG_INF when none.
    double labelled_gap_ = NEG_INF;

    // Accumulated timings across solve iterations (including A+ retries)
    SolveTimings accumulated_timings_;
};

}  // namespace bgspprc
