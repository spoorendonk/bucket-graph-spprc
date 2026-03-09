#pragma once

#include "types.h"

#include <concepts>
#include <tuple>
#include <utility>

namespace bgspprc {

/// Concept for a Meta-Solver resource type.
///
/// Each resource carries per-label state and defines:
///  - symmetric: whether this resource supports symmetric labeling (§4.1)
///  - extend_along_arc: how state changes along an arc
///  - domination_cost: extra cost penalty when L1's state is "worse" than L2's
///  - concatenation_cost: cost adjustment when joining forward/backward labels
template <typename R>
concept Resource = requires(const R& r, Direction dir, Symmetry sym,
                            typename R::State s, typename R::State s2,
                            int arc_id, int vertex) {
    typename R::State;
    { r.symmetric() } -> std::same_as<bool>;
    { r.init_state(dir) } -> std::same_as<typename R::State>;
    { r.extend_along_arc(dir, s, arc_id) } -> std::same_as<std::pair<typename R::State, double>>;
    { r.extend_to_vertex(dir, s, vertex) } -> std::same_as<std::pair<typename R::State, double>>;
    { r.domination_cost(dir, vertex, s, s2) } -> std::same_as<double>;
    { r.concatenation_cost(sym, vertex, s, s2) } -> std::same_as<double>;
};

/// Compile-time pack of resources. Labels carry a tuple of all states.
template <Resource... Rs>
struct ResourcePack {
    std::tuple<Rs...> resources;
    using StatesTuple = std::tuple<typename Rs::State...>;

    static constexpr std::size_t size = sizeof...(Rs);

    explicit ResourcePack(Rs... rs) : resources(std::move(rs)...) {}

    /// True if all resources in the pack support symmetric labeling.
    bool symmetric() const {
        return std::apply(
            [](const auto&... rs) { return (rs.symmetric() && ...); },
            resources);
    }

    StatesTuple init_states(Direction dir) const {
        return std::apply(
            [dir](const auto&... rs) {
                return StatesTuple{rs.init_state(dir)...};
            },
            resources);
    }

    /// Extend all resources along an arc (extendAlongArc). Returns (new_states, total_extra_cost).
    /// If any resource returns INF cost, the extension is infeasible.
    std::pair<StatesTuple, double> extend_along_arc(Direction dir,
                                          const StatesTuple& states,
                                          int arc_id) const {
        StatesTuple new_states;
        double total_cost = 0.0;
        bool feasible = true;

        auto do_extend = [&]<std::size_t... Is>(std::index_sequence<Is...>) {
            ((
                [&] {
                    if (!feasible) return;
                    auto [s, c] = std::get<Is>(resources).extend_along_arc(
                        dir, std::get<Is>(states), arc_id);
                    if (c >= INF) {
                        feasible = false;
                        return;
                    }
                    std::get<Is>(new_states) = s;
                    total_cost += c;
                }()
            ), ...);
        };
        do_extend(std::index_sequence_for<Rs...>{});

        if (!feasible) return {StatesTuple{}, INF};
        return {new_states, total_cost};
    }

    /// Extend all resources to a vertex (extendToVertex). Returns (new_states, total_extra_cost).
    std::pair<StatesTuple, double> extend_to_vertex(Direction dir,
                                                     const StatesTuple& states,
                                                     int vertex) const {
        StatesTuple new_states;
        double total_cost = 0.0;
        bool feasible = true;

        auto do_ext = [&]<std::size_t... Is>(std::index_sequence<Is...>) {
            ((
                [&] {
                    if (!feasible) return;
                    auto [s, c] = std::get<Is>(resources).extend_to_vertex(
                        dir, std::get<Is>(states), vertex);
                    if (c >= INF) {
                        feasible = false;
                        return;
                    }
                    std::get<Is>(new_states) = s;
                    total_cost += c;
                }()
            ), ...);
        };
        do_ext(std::index_sequence_for<Rs...>{});

        if (!feasible) return {StatesTuple{}, INF};
        return {new_states, total_cost};
    }

    /// Compute domination cost: how much extra cost L1 pays vs L2 due to resource states.
    double domination_cost(Direction dir, int vertex,
                           const StatesTuple& s1, const StatesTuple& s2) const {
        double total = 0.0;
        auto do_dom = [&]<std::size_t... Is>(std::index_sequence<Is...>) {
            ((total += std::get<Is>(resources).domination_cost(
                  dir, vertex, std::get<Is>(s1), std::get<Is>(s2))),
             ...);
        };
        do_dom(std::index_sequence_for<Rs...>{});
        return total;
    }

    /// Compute concatenation cost adjustment for joining forward/backward labels.
    double concatenation_cost(Symmetry sym, int vertex,
                              const StatesTuple& s_fw,
                              const StatesTuple& s_bw) const {
        double total = 0.0;
        auto do_cat = [&]<std::size_t... Is>(std::index_sequence<Is...>) {
            ((total += std::get<Is>(resources).concatenation_cost(
                  sym, vertex, std::get<Is>(s_fw), std::get<Is>(s_bw))),
             ...);
        };
        do_cat(std::index_sequence_for<Rs...>{});
        return total;
    }
};

template <Resource... Rs>
auto make_resource_pack(Rs... rs) {
    return ResourcePack<Rs...>(std::move(rs)...);
}

/// Empty resource pack (no extra resources beyond main).
using EmptyPack = ResourcePack<>;

}  // namespace bgspprc
