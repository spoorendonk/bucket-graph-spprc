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
// the rest +1: one path, rc -4 for two arcs. Returns {mono, bidir} best
// reduced costs, INF when no path is found, and checks the path is found once
// where bidirectional labelling is already exact about it: two arcs, or a
// first arc past the midpoint. Other chains get the inner-arc duplicates of
// #109.
template <typename Exec>
static std::pair<double, double> chain_best(std::vector<double> time_d) {
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

    const bool once = n_arcs == 2 || time_d[0] > 5.0;
    double best[2] = {INF, INF};
    for (bool bidir : {false, true}) {
        Solver<EmptyPack, Exec> s(pv, EmptyPack{}, {.bidirectional = bidir});
        s.set_stage(Stage::Exact);
        s.build();
        auto paths = s.solve();
        if (!bidir || once) {
            CHECK(paths.size() <= 1);
        }
        if (!paths.empty()) {
            best[bidir ? 1 : 0] = paths[0].reduced_cost;
        }
    }
    return {best[0], best[1]};
}

template <typename Exec>
static void check_chain(std::vector<double> time_d) {
    const double rc = -5.0 + static_cast<double>(time_d.size() - 1);
    auto [mono, bidir] = chain_best<Exec>(std::move(time_d));
    CHECK(mono == doctest::Approx(rc));
    CHECK(bidir == doctest::Approx(rc));
}

TEST_CASE("Bidirectional: finds a path whose arc out of the source crosses the midpoint") {
    // The fw label at 1 lands past the midpoint and is never extended. With
    // two arcs the path can only be joined across the arc out of the source;
    // with more it is joined at the next arc and must not come back twice.
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

TEST_CASE("Bidirectional: finds a depot tour whose first arc crosses the midpoint") {
    // source == sink == 0: 0 → 1 → 0, first arc past the midpoint 5.
    int from[] = {0, 1};
    int to[] = {1, 0};
    double cost[] = {-5.0, 1.0};
    double time_d[] = {6.0, 1.0};
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

    auto check = [](const auto& paths) {
        REQUIRE(paths.size() == 1);
        CHECK(paths[0].reduced_cost == doctest::Approx(-4.0));
        CHECK(paths[0].vertices == std::vector<int>{0, 1, 0});
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
