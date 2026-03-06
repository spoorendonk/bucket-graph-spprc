#pragma once

#include "bucket_graph.h"
#include "problem_view.h"
#include "r1c.h"
#include "resource.h"
#include "types.h"

#include <algorithm>
#include <span>
#include <vector>

namespace bgspprc {

/// Top-level solver wrapping BucketGraph with multi-stage control.
template <typename Pack>
class Solver {
public:
    using Path = typename BucketGraph<Pack>::Path;

    struct Options {
        std::array<double, 2> bucket_steps = {1.0, 1.0};
        bool bidirectional = false;
        bool symmetric = false;
        int max_paths = 100;
        double tolerance = -1e-6;
        int max_enum_labels = 5000000;
    };

    Solver(const ProblemView& problem, Pack resources, Options opts = {})
        : pv_(problem)
        , pack_(std::move(resources))
        , opts_(opts)
        , bg_(pv_, pack_,
              typename BucketGraph<Pack>::Options{
                  .bucket_steps = opts_.bucket_steps,
                  .max_paths = opts_.max_paths,
                  .tolerance = opts_.tolerance,
                  .bidirectional = opts_.bidirectional,
                  .symmetric = opts_.symmetric,
                  .stage = Stage::Exact,
                  .max_enum_labels = opts_.max_enum_labels,
              }) {}

    /// Build bucket graph (call once, or after step size changes).
    void build() { bg_.build(); }

    /// Fast cost update — O(1), just stores the pointer.
    void update_arc_costs(std::span<const double> reduced_costs) {
        bg_.update_arc_costs(reduced_costs.data());
    }

    /// Solve SPPRC — returns paths with negative reduced cost.
    /// Uses multi-stage progression: Heuristic1 → Heuristic2 → Exact.
    std::vector<Path> solve() {
        bg_.set_stage(stage_);
        auto paths = bg_.solve();
        update_stage(paths);
        return paths;
    }

    /// Arc elimination using optimality gap (bound-based).
    void eliminate_arcs(double theta) { bg_.eliminate_arcs(theta); }

    /// Label-based arc elimination (Section 4.2). Tighter but requires prior solve.
    void eliminate_arcs_label_based(double theta) { bg_.eliminate_arcs_label_based(theta); }

    /// Bucket fixing using optimality gap. Returns number of newly fixed buckets.
    int fix_buckets(double theta) { return bg_.fix_buckets(theta); }

    /// Number of fixed buckets.
    int n_fixed_buckets() const { return bg_.n_fixed_buckets(); }

    /// Reset all elimination/fixing.
    void reset_elimination() { bg_.reset_elimination(); }

    /// Adaptive midpoint control for bidirectional labeling.
    void reset_midpoint() { bg_.reset_midpoint(); }
    double midpoint() const { return bg_.midpoint(); }

    /// Set R1C cuts.
    void set_r1c_cuts(std::span<const R1Cut> cuts) { bg_.set_r1c_cuts(cuts); }

    /// Whether the last enumeration was complete (no caps hit).
    bool enumeration_complete() const { return bg_.enumeration_complete(); }

    /// Stage management.
    void set_stage(Stage stage) { stage_ = stage; }
    Stage current_stage() const { return stage_; }

    /// Path enumeration within gap.
    std::vector<Path> enumerate(double gap) {
        bg_.set_stage(Stage::Enumerate);
        bg_.set_tolerance(gap);
        auto paths = bg_.solve();
        bg_.set_tolerance(opts_.tolerance);
        bg_.set_stage(stage_);
        return paths;
    }

    /// Save warm labels for next solve (top fraction by cost).
    void save_warm_labels(double fraction = 0.7) {
        bg_.save_warm_labels(fraction);
    }

    /// Adaptive bucket step sizes. Returns true if steps changed (needs rebuild).
    bool adapt_bucket_steps(double threshold = 20.0) {
        bool changed = bg_.adapt_bucket_steps(threshold);
        if (changed) bg_.build();
        return changed;
    }

    /// Get current bucket steps.
    const std::array<double, 2>& bucket_steps() const {
        return bg_.bucket_steps();
    }

private:
    void update_stage(const std::vector<Path>& paths) {
        ++iteration_;

        if (stage_ == Stage::Heuristic1) {
            // Move to Heuristic2 if no neg-cost paths found or after 3 iters
            if (paths.empty() || iteration_ > 3) {
                stage_ = Stage::Heuristic2;
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
    BucketGraph<Pack> bg_;
    Stage stage_ = Stage::Heuristic1;
    int iteration_ = 0;
};

}  // namespace bgspprc
