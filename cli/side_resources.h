#pragma once

/// The resources of a ProblemView past its main ones, as one pack resource.
///
/// bgspprc's bucket grid takes at most two main resources from the view and
/// ignores the rest; a general instance (.sppg) may carry more. This resource
/// enforces resources [first, first + n) of the view, each a standard resource
/// (StandardResource's extension, domination and concatenation), with
/// equality in dominance for a non-disposable one. The count is a runtime
/// value up to kMaxSideResources, so one instantiation serves every file.

#include <algorithm>
#include <array>
#include <bgspprc/problem_view.h>
#include <bgspprc/types.h>
#include <cassert>
#include <utility>

namespace bgspprc::io {

inline constexpr int kMaxSideResources = 8;

struct SideResources {
    using State = std::array<double, kMaxSideResources>;

    const ProblemView* pv;  // borrowed: the caller keeps the view alive
    int first;              // the view's index of side resource 0
    int n;                  // side resources in use, <= kMaxSideResources

    SideResources(const ProblemView& pv_, int first_, int n_) : pv(&pv_), first(first_), n(n_) {
        assert(n >= 0 && n <= kMaxSideResources && first + n <= pv->n_resources);
    }

    [[nodiscard]] bool symmetric() const { return false; }

    [[nodiscard]] State init_state(Direction dir) const {
        State s{};
        for (int i = 0; i < n; ++i) {
            s[i] = dir == Direction::Forward ? pv->vertex_lb[first + i][pv->source]
                                             : pv->vertex_ub[first + i][pv->sink];
        }
        return s;
    }

    [[nodiscard]] std::pair<State, double> extend_along_arc(Direction dir, State q,
                                                            int arc_id) const {
        for (int i = 0; i < n; ++i) {
            const double d = pv->arc_resource[first + i][arc_id];
            q[i] = dir == Direction::Forward ? q[i] + d : q[i] - d;
        }
        return {q, 0.0};
    }

    [[nodiscard]] std::pair<State, double> extend_to_vertex(Direction dir, State q,
                                                            int vertex) const {
        for (int i = 0; i < n; ++i) {
            const double lb = pv->vertex_lb[first + i][vertex];
            const double ub = pv->vertex_ub[first + i][vertex];
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
                pv->resource_nondisposable != nullptr && pv->resource_nondisposable[first + i];
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
                if (s_fw[i] + s_bw[i] > pv->vertex_ub[first + i][vertex] + EPS) {
                    return INF;
                }
            } else if (s_fw[i] > s_bw[i] + EPS) {
                return INF;
            }
        }
        return 0.0;
    }
};

}  // namespace bgspprc::io
