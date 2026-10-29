#include <algorithm>
#include <bgspprc/executor_thread.h>
#include <bgspprc/resource.h>
#include <bgspprc/solver.h>
#include <doctest/doctest.h>
#include <utility>
#include <vector>

using namespace bgspprc;

// End-to-end test on a small known instance.
//
// Graph: 5 vertices (0=source, 4=sink)
//
//   0 --a0(c=10,t=2)--> 1 --a2(c=5,t=3)--> 3 --a5(c=2,t=1)--> 4
//   0 --a1(c=3,t=4)---> 2 --a3(c=4,t=2)--> 3 --a5(c=2,t=1)--> 4
//   1 --a4(c=8,t=5)---> 4
//   2 --a6(c=7,t=3)---> 4
//
// Paths:
//   0→1→3→4: cost = 10+5+2 = 17, time = 2+3+1 = 6
//   0→2→3→4: cost = 3+4+2 = 9,   time = 4+2+1 = 7
//   0→1→4:   cost = 10+8 = 18,    time = 2+5 = 7
//   0→2→4:   cost = 3+7 = 10,     time = 4+3 = 7
//
// Optimal (min cost): 0→2→3→4, cost=9

TEST_CASE("End-to-end: 5-vertex graph optimal path") {
    int from[] = {0, 0, 1, 2, 1, 3, 2};
    int to[] = {1, 2, 3, 3, 4, 4, 4};
    double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
    double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 4;
    pv.n_arcs = 7;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    Solver<EmptyPack> solver(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    solver.build();

    auto paths = solver.solve();
    REQUIRE(!paths.empty());

    // Best path cost = 9 (0→2→3→4)
    CHECK(paths[0].reduced_cost == doctest::Approx(9.0));
    CHECK(paths[0].original_cost == doctest::Approx(9.0));

    CHECK(paths[0].vertices.size() == 4);
    CHECK(paths[0].vertices[0] == 0);
    CHECK(paths[0].vertices[1] == 2);
    CHECK(paths[0].vertices[2] == 3);
    CHECK(paths[0].vertices[3] == 4);
}

TEST_CASE("End-to-end: with reduced costs (pricing)") {
    int from[] = {0, 0, 1, 2, 1, 3, 2};
    int to[] = {1, 2, 3, 3, 4, 4, 4};
    double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
    double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 4;
    pv.n_arcs = 7;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    Solver<EmptyPack> solver(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 0.0});
    solver.build();

    // Reduced costs: make 0→1→3→4 have negative reduced cost
    double red[] = {-15.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
    solver.update_arc_costs(red);

    auto paths = solver.solve();
    REQUIRE(!paths.empty());

    // 0→1→3→4: -15+5+2 = -8 (negative reduced cost)
    CHECK(paths[0].reduced_cost == doctest::Approx(-8.0));
}

TEST_CASE("End-to-end: tight time windows") {
    int from[] = {0, 0, 1, 2, 1, 3, 2};
    int to[] = {1, 2, 3, 3, 4, 4, 4};
    double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
    double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 6.5};  // sink tw: [0, 6.5]

    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 4;
    pv.n_arcs = 7;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    Solver<EmptyPack> solver(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    solver.set_stage(Stage::Exact);
    solver.build();

    auto paths = solver.solve();
    REQUIRE(!paths.empty());

    // With sink time window [0, 6.5]:
    //   0→1→3→4: time=6 ≤ 6.5 → feasible, cost=17
    //   0→2→3→4: time=7 > 6.5 → INFEASIBLE
    //   0→1→4:   time=7 > 6.5 → INFEASIBLE
    //   0→2→4:   time=7 > 6.5 → INFEASIBLE
    // Only feasible path: 0→1→3→4, cost=17
    CHECK(paths[0].reduced_cost == doctest::Approx(17.0));
    CHECK(paths[0].vertices[1] == 1);
    CHECK(paths[0].vertices[2] == 3);
}

// ── Bi-directional tests ──

TEST_CASE("Bidirectional: same optimal as mono-directional") {
    int from[] = {0, 0, 1, 2, 1, 3, 2};
    int to[] = {1, 2, 3, 3, 4, 4, 4};
    double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
    double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 4;
    pv.n_arcs = 7;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // Mono-directional
    Solver<EmptyPack> mono(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    mono.build();
    auto mono_paths = mono.solve();

    // Bi-directional
    Solver<EmptyPack> bidir(pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .bidirectional = true, .theta = 1e9});
    bidir.build();
    auto bidir_paths = bidir.solve();

    REQUIRE(!mono_paths.empty());
    REQUIRE(!bidir_paths.empty());

    // Same best cost
    CHECK(bidir_paths[0].reduced_cost == doctest::Approx(mono_paths[0].reduced_cost));

    // Best path should be 0→2→3→4, cost=9
    CHECK(bidir_paths[0].reduced_cost == doctest::Approx(9.0));
    CHECK(bidir_paths[0].vertices.front() == 0);
    CHECK(bidir_paths[0].vertices.back() == 4);
}

TEST_CASE("Bidirectional: with reduced costs") {
    int from[] = {0, 0, 1, 2, 1, 3, 2};
    int to[] = {1, 2, 3, 3, 4, 4, 4};
    double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
    double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};

    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 4;
    pv.n_arcs = 7;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    Solver<EmptyPack> bidir(pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .bidirectional = true, .theta = 0.0});
    bidir.build();

    double red[] = {-15.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
    bidir.update_arc_costs(red);

    auto paths = bidir.solve();
    REQUIRE(!paths.empty());

    // 0→1→3→4: -15+5+2 = -8
    CHECK(paths[0].reduced_cost == doctest::Approx(-8.0));
    CHECK(paths[0].vertices.front() == 0);
    CHECK(paths[0].vertices.back() == 4);
}

TEST_CASE("Bidirectional: tight time windows") {
    int from[] = {0, 0, 1, 2, 1, 3, 2};
    int to[] = {1, 2, 3, 3, 4, 4, 4};
    double cost[] = {10.0, 3.0, 5.0, 4.0, 8.0, 2.0, 7.0};
    double time_d[] = {2.0, 4.0, 3.0, 2.0, 5.0, 1.0, 3.0};

    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 6.5};

    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 4;
    pv.n_arcs = 7;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // Mono
    Solver<EmptyPack> mono(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    mono.set_stage(Stage::Exact);
    mono.build();
    auto mono_paths = mono.solve();

    // Bidir
    Solver<EmptyPack> bidir(pv, EmptyPack{},
                            {.bucket_steps = {5.0, 1.0}, .bidirectional = true, .theta = 1e9});
    bidir.set_stage(Stage::Exact);
    bidir.build();
    auto bidir_paths = bidir.solve();

    REQUIRE(!mono_paths.empty());
    REQUIRE(!bidir_paths.empty());

    // Same best: 0→1→3→4, cost=17 (only feasible path with sink tw [0, 6.5])
    CHECK(bidir_paths[0].reduced_cost == doctest::Approx(mono_paths[0].reduced_cost));
    CHECK(bidir_paths[0].reduced_cost == doctest::Approx(17.0));
}

// Chain 0 → 1 → … → n, windows [0, 10] (midpoint 5), first arc cost -5 and
// the rest +1: one path, rc -4 for two arcs. The source window starts at
// source_lb. Returns {mono, bidir} best reduced costs, INF when no path is
// found, and checks the path is found at most once (#109).
template <typename Exec>
static std::pair<double, double> chain_best(std::vector<double> time_d, double source_lb = 0.0) {
    const int n_arcs = static_cast<int>(time_d.size());
    const int n = n_arcs + 1;
    std::vector<int> from(n_arcs);
    std::vector<int> to(n_arcs);
    std::vector<double> cost(n_arcs, 1.0);
    for (int a = 0; a < n_arcs; ++a) {
        from[a] = a;
        to[a] = a + 1;
    }
    cost[0] = -5.0;
    std::vector<double> tw_lb(n, 0.0);
    std::vector<double> tw_ub(n, 10.0);
    tw_lb[0] = source_lb;

    const double* arc_res[] = {time_d.data()};
    const double* v_lb[] = {tw_lb.data()};
    const double* v_ub[] = {tw_ub.data()};

    ProblemView pv;
    pv.n_vertices = n;
    pv.source = 0;
    pv.sink = n - 1;
    pv.n_arcs = n_arcs;
    pv.arc_from = from.data();
    pv.arc_to = to.data();
    pv.arc_base_cost = cost.data();
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    double best[2] = {INF, INF};
    for (bool bidir : {false, true}) {
        Solver<EmptyPack, Exec> s(pv, EmptyPack{}, {.bidirectional = bidir});
        s.set_stage(Stage::Exact);
        s.build();
        auto paths = s.solve();
        CHECK(paths.size() <= 1);
        if (!paths.empty()) {
            best[bidir ? 1 : 0] = paths[0].reduced_cost;
        }
    }
    return {best[0], best[1]};
}

template <typename Exec>
static void check_chain(std::vector<double> time_d, double source_lb = 0.0) {
    const double rc = -5.0 + static_cast<double>(time_d.size() - 1);
    auto [mono, bidir] = chain_best<Exec>(std::move(time_d), source_lb);
    CHECK(mono == doctest::Approx(rc));
    CHECK(bidir == doctest::Approx(rc));
}

TEST_CASE("Bidirectional: finds a path whose arc out of the source crosses the midpoint") {
    // The fw label at 1 lands past the midpoint and is never extended: the
    // path is joined across the arc out of 1, into the sink or not, and
    // must come back once.
    for (double t0 : {1.0, 5.0, 6.0, 9.0}) {
        CAPTURE(t0);
        check_chain<SequentialExecutor>({t0, 1.0});
        check_chain<StdThreadExecutor>({t0, 1.0});
    }
    for (double t0 : {1.0, 5.0, 6.0, 8.0}) {
        CAPTURE(t0);
        check_chain<SequentialExecutor>({t0, 1.0, 1.0});
        check_chain<StdThreadExecutor>({t0, 1.0, 1.0});
        check_chain<SequentialExecutor>({t0 - 1.0, 1.0, 1.0, 1.0});
        check_chain<StdThreadExecutor>({t0 - 1.0, 1.0, 1.0, 1.0});
    }
}

TEST_CASE("Bidirectional: finds a path once when the source starts past the midpoint") {
    // The source label is never extended, so the path is joined across the
    // arc out of the source whatever its length.
    for (std::vector<double> time_d :
         {std::vector<double>{1.0, 1.0}, {1.0, 1.0, 1.0}, {0.0, 1.0, 1.0, 1.0}}) {
        CAPTURE(time_d.size());
        check_chain<SequentialExecutor>(time_d, 6.0);
        check_chain<StdThreadExecutor>(time_d, 6.0);
    }
}

TEST_CASE("Bidirectional: finds a path whose arc into the sink crosses the midpoint") {
    // The bw label at the last inner vertex lands below the midpoint and is
    // never extended.
    for (double t : {1.0, 5.0, 6.0, 8.0}) {
        CAPTURE(t);
        check_chain<SequentialExecutor>({1.0, t});
        check_chain<StdThreadExecutor>({1.0, t});
        check_chain<SequentialExecutor>({1.0, 1.0, t});
        check_chain<StdThreadExecutor>({1.0, 1.0, t});
    }
}

// Exact-stage paths for one ProblemView, mono then bidirectional.
template <typename Exec>
static auto solve_mono_bidir(const ProblemView& pv) {
    auto solve = [&](bool bidir) {
        Solver<EmptyPack, Exec> s(pv, EmptyPack{}, {.bidirectional = bidir});
        s.set_stage(Stage::Exact);
        s.build();
        return s.solve();
    };
    return std::pair{solve(false), solve(true)};
}

TEST_CASE("source == sink is the empty path in mono and bidirectional (#110)") {
    // A depot with one tour 0 → 1 → 0 of cost -4, repeatable while time
    // lasts. Outside symmetric mode no path leaves the depot: no tour, and
    // no multi-trip tour, only the empty path of cost 0.
    int from[] = {0, 1};
    int to[] = {1, 0};
    double cost[] = {-5.0, 1.0};
    double time_d[] = {1.0, 1.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 0;
    pv.n_arcs = 2;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // Default theta < 0: the empty path is not improving, so nothing.
    auto [mono, bidir] = solve_mono_bidir<SequentialExecutor>(pv);
    CHECK(mono.empty());
    CHECK(bidir.empty());
    auto [par_mono, par_bidir] = solve_mono_bidir<StdThreadExecutor>(pv);
    CHECK(par_mono.empty());
    CHECK(par_bidir.empty());

    // theta > 0 admits the empty path, and it is the only one.
    for (bool bidir_mode : {false, true}) {
        CAPTURE(bidir_mode);
        Solver<EmptyPack> s(pv, EmptyPack{}, {.bidirectional = bidir_mode, .theta = 1.0});
        s.set_stage(Stage::Exact);
        s.build();
        auto paths = s.solve();
        REQUIRE(paths.size() == 1);
        CHECK(paths[0].reduced_cost == doctest::Approx(0.0));
        CHECK(paths[0].arcs.empty());
        CHECK(paths[0].vertices == std::vector<int>{0});

        // cost_of agrees: the empty walk is the path, a tour is not one.
        auto empty = s.cost_of(std::vector<int>{});
        REQUIRE(empty.has_value());
        const auto w =
            empty.value_or(Solver<EmptyPack>::WalkCost{.reduced_cost = INF, .original_cost = INF});
        CHECK(w.reduced_cost == doctest::Approx(0.0));
        CHECK_FALSE(s.cost_of(std::vector<int>{0, 1}).has_value());
    }
}

TEST_CASE("Bidirectional: finds a split-depot tour whose first arc crosses the midpoint") {
    // The depot split into source 0 and sink copy 2: 0 → 1 → 2, first arc
    // past the midpoint 5.
    int from[] = {0, 1};
    int to[] = {1, 2};
    double cost[] = {-5.0, 1.0};
    double time_d[] = {6.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 3;
    pv.source = 0;
    pv.sink = 2;
    pv.n_arcs = 2;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    auto check = [](const auto& paths) {
        REQUIRE(paths.size() == 1);
        CHECK(paths[0].reduced_cost == doctest::Approx(-4.0));
        CHECK(paths[0].vertices == std::vector<int>{0, 1, 2});
    };
    auto [mono, bidir] = solve_mono_bidir<SequentialExecutor>(pv);
    check(mono);
    check(bidir);
    auto [par_mono, par_bidir] = solve_mono_bidir<StdThreadExecutor>(pv);
    check(par_mono);
    check(par_bidir);
}

TEST_CASE("Bidirectional: source-arc join keeps a one-arc and a longer suffix apart") {
    // 0 → 1 → 3 and 0 → 1 → 2 → 3, both negative, first arc past the
    // midpoint 5: bw@1 holds a one-arc and a two-arc label, and each path
    // must come back exactly once.
    int from[] = {0, 1, 1, 2};
    int to[] = {1, 3, 2, 3};
    double cost[] = {-5.0, 2.0, 1.0, 0.0};
    double time_d[] = {6.0, 1.0, 1.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 4;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    auto check = [](const auto& paths) {
        REQUIRE(paths.size() == 2);
        CHECK(paths[0].reduced_cost == doctest::Approx(-4.0));
        CHECK(paths[0].vertices == std::vector<int>{0, 1, 2, 3});
        CHECK(paths[1].reduced_cost == doctest::Approx(-3.0));
        CHECK(paths[1].vertices == std::vector<int>{0, 1, 3});
    };
    auto [mono, bidir] = solve_mono_bidir<SequentialExecutor>(pv);
    check(mono);
    check(bidir);
    auto [par_mono, par_bidir] = solve_mono_bidir<StdThreadExecutor>(pv);
    check(par_mono);
    check(par_bidir);
}

// Dominance on a main resource must be exact: windows are checked without
// EPS, so a dominator a rounding error past the label it prunes can be cut
// at a window that label meets (#114).
static ProblemView rounding_view(const int* from, const int* to, const double* cost,
                                 const double** arc_res, const double** v_lb, const double** v_ub) {
    ProblemView pv;
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 4;
    pv.n_arcs = 5;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;
    return pv;
}

TEST_CASE("Dominance: a forward label one rounding error later does not prune a feasible one") {
    // At 2, 0 → 1 → 2 arrives at 0.1 + 0.2 = 0.30000000000000004 for -5 and
    // 0 → 2 at 0.3 for 0. Vertex 3 closes at 0.3, below the midpoint 10, so
    // only 0 → 2 → 3 → 4 (-1) is feasible.
    int from[] = {0, 1, 0, 2, 3};
    int to[] = {1, 2, 2, 3, 4};
    double cost[] = {-5.0, 0.0, 0.0, -1.0, 0.0};
    double time_d[] = {0.1, 0.2, 0.3, 0.0, 0.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 0.3, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};
    ProblemView pv = rounding_view(from, to, cost, arc_res, v_lb, v_ub);

    auto check = [](const auto& paths) {
        REQUIRE(paths.size() == 1);
        CHECK(paths[0].reduced_cost == doctest::Approx(-1.0));
        CHECK(paths[0].vertices == std::vector<int>{0, 2, 3, 4});
    };
    auto [mono, bidir] = solve_mono_bidir<SequentialExecutor>(pv);
    check(mono);
    check(bidir);
}

TEST_CASE("Dominance: a backward label one rounding error earlier does not prune a feasible one") {
    // Backward from the sink at 20, 2 → 1 → 4 reaches 2 at
    // 20 - 0.1 - 0.1 = 19.799999999999997 for -5 and 2 → 4 at 19.8 for 0.
    // Vertex 3 opens at 19.8, above the midpoint 10, so only 0 → 3 → 2 → 4
    // (-1) is feasible; forward, 0 → 3 → 2 → 1 → 4 ends at
    // 19.8 + 0.1 + 0.1 > 20 too. The bidir join checks q within EPS, as fw
    // and bw arithmetic round apart, so it may also return that path (-6).
    int from[] = {0, 3, 2, 2, 1};
    int to[] = {3, 2, 4, 1, 4};
    double cost[] = {0.0, -1.0, 0.0, -5.0, 0.0};
    double time_d[] = {0.0, 0.0, 0.2, 0.1, 0.1};
    double tw_lb[] = {0.0, 0.0, 0.0, 19.8, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};
    ProblemView pv = rounding_view(from, to, cost, arc_res, v_lb, v_ub);

    auto check = [](const auto& paths) {
        REQUIRE(paths.size() == 1);
        CHECK(paths[0].reduced_cost == doctest::Approx(-1.0));
        CHECK(paths[0].vertices == std::vector<int>{0, 3, 2, 4});
    };
    auto [mono, bidir] = solve_mono_bidir<SequentialExecutor>(pv);
    check(mono);
    CHECK(std::ranges::any_of(bidir, [](const auto& p) {
        return p.vertices == std::vector<int>{0, 3, 2, 4} &&
               p.reduced_cost == doctest::Approx(-1.0);
    }));
}
