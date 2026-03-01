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
              }) {}

    /// Build bucket graph (call once, or after step size changes).
    void build() { bg_.build(); }

    /// Fast cost update — O(1), just stores the pointer.
    void update_arc_costs(std::span<const double> reduced_costs) {
        bg_.update_arc_costs(reduced_costs.data());
    }

    /// Solve SPPRC — returns paths with negative reduced cost.
    std::vector<Path> solve() {
        auto paths = bg_.solve();
        update_stage(paths);
        return paths;
    }

    /// Arc elimination using optimality gap.
    void eliminate_arcs(double theta) { bg_.eliminate_arcs(theta); }

    /// Bucket fixing using optimality gap.
    void fix_buckets(double theta) { bg_.fix_buckets(theta); }

    /// Reset all elimination/fixing.
    void reset_elimination() { bg_.reset_elimination(); }

    /// Set R1C cuts.
    void set_r1c_cuts(std::span<const R1Cut> cuts) { bg_.set_r1c_cuts(cuts); }

    /// Stage management.
    void set_stage(Stage stage) { stage_ = stage; }
    Stage current_stage() const { return stage_; }

    /// Path enumeration within gap.
    std::vector<Path> enumerate(double gap) {
        auto saved_tol = opts_.tolerance;
        opts_.tolerance = gap;
        // Rebuild with exact settings for enumeration
        auto paths = bg_.solve();
        opts_.tolerance = saved_tol;
        return paths;
    }

private:
    void update_stage(const std::vector<Path>& /*paths*/) {
        ++iteration_;
        // Auto-progression: advance stage based on iteration count
        // and whether paths were found
        if (stage_ == Stage::Heuristic1 && iteration_ > 3) {
            stage_ = Stage::Heuristic2;
        }
        if (stage_ == Stage::Heuristic2 && iteration_ > 10) {
            stage_ = Stage::Exact;
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
