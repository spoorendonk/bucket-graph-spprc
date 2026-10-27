#pragma once

#include "../problem_view.h"
#include "../types.h"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <utility>

namespace bgspprc {

inline constexpr int kMaxSideResources = 8;

/// The resources of a ProblemView past its main ones, as one pack resource.
///
/// The bucket grid takes at most two main resources from the view and never
/// reads the rest: resources [min(n_main_resources, 2), n_resources) are the
/// pack's to enforce. This resource enforces resources [first, first + n) of
/// the view, each a standard resource (StandardResource's extension,
/// domination and concatenation), with equality in dominance for a
/// non-disposable one. The count is a runtime value up to kMaxSideResources,
/// so one instantiation serves every view.
struct SideResources {
    using State = std::array<double, kMaxSideResources>;

    // The view's arrays, borrowed: the caller keeps them alive, not the view.
    const double* const* arc_resource;
    const double* const* vertex_lb;
    const double* const* vertex_ub;
    const bool* resource_nondisposable;
    int source;
    int sink;
    int first;  // the view's index of side resource 0
    int n;      // side resources in use, <= kMaxSideResources

    /// Throws std::invalid_argument unless [first, first + n) lies in the
    /// view's resources and n <= kMaxSideResources.
    SideResources(const ProblemView& pv, int first_, int n_)
        : arc_resource(pv.arc_resource),
          vertex_lb(pv.vertex_lb),
          vertex_ub(pv.vertex_ub),
          resource_nondisposable(pv.resource_nondisposable),
          source(pv.source),
          sink(pv.sink),
          first(first_),
          n(n_) {
        if (first < 0 || n < 0 || n > kMaxSideResources || first > pv.n_resources - n) {
            throw std::invalid_argument(
                "SideResources: [first, first + n) must lie in the view's resources, with n <= "
                "kMaxSideResources");
        }
        if (n > 0 && (arc_resource == nullptr || vertex_lb == nullptr || vertex_ub == nullptr)) {
            throw std::invalid_argument(
                "SideResources: the view needs arc consumptions and vertex windows");
        }
    }

    [[nodiscard]] bool symmetric() const { return false; }

    [[nodiscard]] State init_state(Direction dir) const {
        State s{};
        for (int i = 0; i < n; ++i) {
            s[i] = dir == Direction::Forward ? vertex_lb[first + i][source]
                                             : vertex_ub[first + i][sink];
        }
        return s;
    }

    [[nodiscard]] std::pair<State, double> extend_along_arc(Direction dir, State q,
                                                            int arc_id) const {
        for (int i = 0; i < n; ++i) {
            const double d = arc_resource[first + i][arc_id];
            q[i] = dir == Direction::Forward ? q[i] + d : q[i] - d;
        }
        return {q, 0.0};
    }

    [[nodiscard]] std::pair<State, double> extend_to_vertex(Direction dir, State q,
                                                            int vertex) const {
        for (int i = 0; i < n; ++i) {
            const double lb = vertex_lb[first + i][vertex];
            const double ub = vertex_ub[first + i][vertex];
            if (dir == Direction::Forward) {
                q[i] = std::max(q[i], lb);
                if (q[i] > ub) {
                    return {q, INF};
                }
            } else {
                q[i] = std::min(q[i], ub);
                if (q[i] < lb) {
                    return {q, INF};
                }
            }
        }
        return {q, 0.0};
    }

    /// Exact, as StandardResource's (#114).
    [[nodiscard]] double domination_cost(Direction dir, int /*vertex*/, const State& s1,
                                         const State& s2) const {
        for (int i = 0; i < n; ++i) {
            const bool nondisposable =
                resource_nondisposable != nullptr && resource_nondisposable[first + i];
            if (nondisposable) {
                if (s1[i] != s2[i]) {
                    return INF;
                }
            } else if (dir == Direction::Forward ? s1[i] > s2[i] : s1[i] < s2[i]) {
                return INF;
            }
        }
        return 0.0;
    }

    [[nodiscard]] double min_domination_cost() const { return 0.0; }

    [[nodiscard]] double concatenation_cost(Symmetry sym, int vertex, const State& s_fw,
                                            const State& s_bw) const {
        for (int i = 0; i < n; ++i) {
            if (sym == Symmetry::Symmetric) {
                if (s_fw[i] + s_bw[i] > vertex_ub[first + i][vertex] + EPS) {
                    return INF;
                }
            } else if (s_fw[i] > s_bw[i] + EPS) {
                return INF;
            }
        }
        return 0.0;
    }
};

}  // namespace bgspprc
