#include <algorithm>
#include <array>
#include <atomic>
#include <bgspprc/bucket_graph.h>
#include <bgspprc/executor_thread.h>
#include <bgspprc/r1c.h>
#include <bgspprc/resource.h>
#include <bgspprc/resources/cumulative_cost.h>
#include <bgspprc/resources/ng_path.h>
#include <bgspprc/resources/pickup_delivery.h>
#include <bgspprc/solver.h>
#include <cmath>
#include <doctest/doctest.h>
#include <random>
#include <vector>

using namespace bgspprc;

// ════════════════════════════════════════════════════════════════════
// Helpers
// ════════════════════════════════════════════════════════════════════

/// Compare sequential bidir and parallel bidir: same best cost and
/// same set of path arc sequences.
static void check_parallel_bidir_agree(const ProblemView& pv, double tol = 1e9, double step = 5.0) {
    BucketGraph<EmptyPack> seq(
        pv, EmptyPack{},
        {.bucket_steps = {step, 1.0}, .theta = tol, .bidirectional = true, .stage = Stage::Exact});
    seq.build();
    auto sp = seq.solve();

    BucketGraph<EmptyPack, StdThreadExecutor> par(
        pv, EmptyPack{},
        {.bucket_steps = {step, 1.0}, .theta = tol, .bidirectional = true, .stage = Stage::Exact});
    par.build();
    auto pp = par.solve();

    if (!sp.empty() && !pp.empty()) {
        CHECK(sp[0].reduced_cost == doctest::Approx(pp[0].reduced_cost).epsilon(1e-6));
    } else {
        CHECK(sp.empty() == pp.empty());
    }

    // Verify all sequential paths appear in parallel results
    auto to_set = [](const auto& paths) {
        std::vector<std::vector<int>> vs;
        vs.reserve(paths.size());
        for (auto& p : paths) {
            vs.push_back(p.arcs);
        }
        std::ranges::sort(vs);
        vs.erase(std::ranges::unique(vs).begin(), vs.end());
        return vs;
    };
    auto seq_set = to_set(sp);
    auto par_set = to_set(pp);
    // Each path once (#109).
    CHECK(seq_set.size() == sp.size());
    CHECK(par_set.size() == pp.size());
    for (auto& path : seq_set) {
        CHECK(std::ranges::find(par_set, path) != par_set.end());
    }
}

/// Compare mono and bidir: same best cost, and both produce the same
/// set of path arc sequences (order-independent).
static void check_mono_bidir_agree(const ProblemView& pv, double tol = 1e9, double step = 5.0) {
    BucketGraph<EmptyPack> mono(pv, EmptyPack{},
                                {.bucket_steps = {step, 1.0}, .theta = tol, .stage = Stage::Exact});
    mono.build();
    auto mp = mono.solve();

    BucketGraph<EmptyPack> bidir(
        pv, EmptyPack{},
        {.bucket_steps = {step, 1.0}, .theta = tol, .bidirectional = true, .stage = Stage::Exact});
    bidir.build();
    auto bp = bidir.solve();

    if (!mp.empty() && !bp.empty()) {
        CHECK(mp[0].reduced_cost == doctest::Approx(bp[0].reduced_cost).epsilon(1e-9));
    } else {
        CHECK(mp.empty() == bp.empty());
    }

    // Verify all mono paths appear in bidir results, each
    // path once (#109). Dominance can differ between the two, so bidir may
    // return paths mono does not.
    auto to_set = [](const auto& paths) {
        std::vector<std::vector<int>> vs;
        vs.reserve(paths.size());
        for (auto& p : paths) {
            vs.push_back(p.arcs);
        }
        std::ranges::sort(vs);
        vs.erase(std::ranges::unique(vs).begin(), vs.end());
        return vs;
    };
    auto mono_set = to_set(mp);
    auto bidir_set = to_set(bp);
    CHECK(mono_set.size() == mp.size());
    CHECK(bidir_set.size() == bp.size());
    // Every mono path must appear in bidir
    for (auto& path : mono_set) {
        CHECK(std::ranges::find(bidir_set, path) != bidir_set.end());
    }
}

// ════════════════════════════════════════════════════════════════════
// 1. Single-arc graph (minimal case)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: single-arc source→sink") {
    int from[] = {0};
    int to[] = {1};
    double cost[] = {7.0};
    double time_d[] = {3.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_mono_bidir_agree(pv);

    // Also verify the actual path
    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    bg.build();
    auto paths = bg.solve();
    REQUIRE(paths.size() == 1);
    CHECK(paths[0].reduced_cost == doctest::Approx(7.0));
    CHECK(paths[0].vertices == std::vector<int>{0, 1});
    CHECK(paths[0].arcs == std::vector<int>{0});
}

// ════════════════════════════════════════════════════════════════════
// 2. All paths infeasible (resource constraints block everything)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: all paths infeasible") {
    // Arc 0→1 needs time=15, but sink ub=10. No feasible path.
    int from[] = {0};
    int to[] = {1};
    double cost[] = {1.0};
    double time_d[] = {15.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    BucketGraph<EmptyPack> mono(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    mono.build();
    auto mp = mono.solve();
    CHECK(mp.empty());

    BucketGraph<EmptyPack> bidir(
        pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    bidir.build();
    auto bp = bidir.solve();
    CHECK(bp.empty());
}

TEST_CASE("Stress: multi-hop all infeasible") {
    // 0→1→2, each arc time=6, vertex ub=10. Path needs 12 > 10.
    int from[] = {0, 1};
    int to[] = {1, 2};
    double cost[] = {1.0, 1.0};
    double time_d[] = {6.0, 6.0};
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

    check_mono_bidir_agree(pv);

    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    bg.build();
    CHECK(bg.solve().empty());
}

// ════════════════════════════════════════════════════════════════════
// 3. Parallel arcs between same vertex pair
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: parallel arcs") {
    // 0→1 via two arcs: fast+expensive (t=2,c=10) and slow+cheap (t=5,c=1)
    // 1→2 single arc
    int from[] = {0, 0, 1};
    int to[] = {1, 1, 2};
    double cost[] = {10.0, 1.0, 1.0};
    double time_d[] = {2.0, 5.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 3;
    pv.source = 0;
    pv.sink = 2;
    pv.n_arcs = 3;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_mono_bidir_agree(pv);

    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    bg.build();
    auto paths = bg.solve();
    REQUIRE(paths.size() >= 1);
    // Cheap path uses slow arc: cost = 1+1 = 2
    CHECK(paths[0].reduced_cost == doctest::Approx(2.0));
}

TEST_CASE("Stress: parallel arcs, tight window forces expensive arc") {
    // Same setup but sink ub=4: slow arc (t=5+1=6) infeasible, must use fast
    int from[] = {0, 0, 1};
    int to[] = {1, 1, 2};
    double cost[] = {10.0, 1.0, 1.0};
    double time_d[] = {2.0, 5.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 4.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 3;
    pv.source = 0;
    pv.sink = 2;
    pv.n_arcs = 3;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_mono_bidir_agree(pv);

    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    bg.build();
    auto paths = bg.solve();
    REQUIRE(paths.size() == 1);
    CHECK(paths[0].reduced_cost == doctest::Approx(11.0));  // fast arc: 10+1
}

// ════════════════════════════════════════════════════════════════════
// 4. Zero-width time windows
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: zero-width time window at intermediate vertex") {
    // 0→1→2. Vertex 1 has tw=[3,3] — forced arrival at exactly 3.
    int from[] = {0, 1};
    int to[] = {1, 2};
    double cost[] = {1.0, 2.0};
    double time_d[] = {2.0, 1.0};  // arrive at 1: max(0+2, 3) = 3. arrive at 2: 4.
    double tw_lb[] = {0.0, 3.0, 0.0};
    double tw_ub[] = {10.0, 3.0, 10.0};
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

    check_mono_bidir_agree(pv);

    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {1.0, 1.0}, .theta = 1e9});
    bg.build();
    auto paths = bg.solve();
    REQUIRE(paths.size() == 1);
    CHECK(paths[0].reduced_cost == doctest::Approx(3.0));
}

TEST_CASE("Stress: zero-width window makes path infeasible") {
    // 0→1→2. Vertex 1 tw=[5,5], but arc 0→1 has t=3, so arrive at max(3,5)=5 ✓.
    // Now change to tw=[1,1]: arrive max(3,1)=3 > 1 → infeasible.
    int from[] = {0, 1};
    int to[] = {1, 2};
    double cost[] = {1.0, 1.0};
    double time_d[] = {3.0, 1.0};
    double tw_lb[] = {0.0, 1.0, 0.0};
    double tw_ub[] = {10.0, 1.0, 10.0};
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

    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {1.0, 1.0}, .theta = 1e9});
    bg.build();
    CHECK(bg.solve().empty());
}

// ════════════════════════════════════════════════════════════════════
// 5. Diamond with asymmetric resources — neither path dominates
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: diamond, neither path dominates the other") {
    // 0→1→3: cost=5, time=8
    // 0→2→3: cost=6, time=4
    // Path 1 is cheaper but slower; path 2 is more expensive but faster.
    // With wide windows, both should be found (neither dominates).
    int from[] = {0, 0, 1, 2};
    int to[] = {1, 2, 3, 3};
    double cost[] = {2.0, 3.0, 3.0, 3.0};
    double time_d[] = {4.0, 2.0, 4.0, 2.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0};
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

    check_mono_bidir_agree(pv, 1e9, 1.0);

    BucketGraph<EmptyPack> bg(pv, EmptyPack{},
                              {.bucket_steps = {1.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    bg.build();
    auto paths = bg.solve();
    REQUIRE(paths.size() == 2);
    CHECK(paths[0].reduced_cost == doctest::Approx(5.0));  // cheaper path
    CHECK(paths[1].reduced_cost == doctest::Approx(6.0));
}

// ════════════════════════════════════════════════════════════════════
// 6. Domination with equal cost and equal resource
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: identical labels — only one survives") {
    // Two identical parallel arcs 0→1, same cost and time.
    // Both produce identical labels at vertex 1. One should dominate the other.
    int from[] = {0, 0, 1};
    int to[] = {1, 1, 2};
    double cost[] = {5.0, 5.0, 1.0};
    double time_d[] = {3.0, 3.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 3;
    pv.source = 0;
    pv.sink = 2;
    pv.n_arcs = 3;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    bg.build();
    auto paths = bg.solve();
    // Both arcs lead to identical labels; dominance kills one → single path
    CHECK(paths.size() == 1);
    CHECK(paths[0].reduced_cost == doctest::Approx(6.0));
}

// ════════════════════════════════════════════════════════════════════
// 7. Large negative reduced costs (pricing scenario)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: large negative reduced costs") {
    // Diamond: 0→1→3, 0→2→3, with very negative reduced costs.
    int from[] = {0, 0, 1, 2};
    int to[] = {1, 2, 3, 3};
    double base_cost[] = {1.0, 1.0, 1.0, 1.0};
    double time_d[] = {2.0, 3.0, 2.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0};
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
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    BucketGraph<EmptyPack> mono(pv, EmptyPack{},
                                {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    mono.build();

    double red_cost[] = {-1000.0, -500.0, -2000.0, -100.0};
    mono.update_arc_costs(red_cost);
    auto mp = mono.solve();
    REQUIRE(!mp.empty());
    CHECK(mp[0].reduced_cost == doctest::Approx(-3000.0));  // -1000 + -2000

    BucketGraph<EmptyPack> bidir(
        pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    bidir.build();
    bidir.update_arc_costs(red_cost);
    auto bp = bidir.solve();
    REQUIRE(!bp.empty());
    CHECK(bp[0].reduced_cost == doctest::Approx(mp[0].reduced_cost));
}

// ════════════════════════════════════════════════════════════════════
// 8. Concatenation at exact resource boundary
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: concatenation at exact resource midpoint") {
    // 0→1→2→3, tw [0,10]. Midpoint = 5.
    // Arc 0→1: t=3, Arc 1→2: t=2 (fw arrives at 2 with q=5 = exactly midpoint)
    // Arc 2→3: t=3
    // The concatenation at arc 1→2 should work: fw_after_arc = 3+2=5, bw at 2
    // has q=ub-3=7, and 5 ≤ 7. At exact boundary.
    int from[] = {0, 1, 2};
    int to[] = {1, 2, 3};
    double cost[] = {1.0, 1.0, 1.0};
    double time_d[] = {3.0, 2.0, 3.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 3;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_mono_bidir_agree(pv, 1e9, 1.0);
}

// ════════════════════════════════════════════════════════════════════
// 9. Negative-cost self-loop (label cap prevents infinite loop)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: ng-path blocks self-loop") {
    // Self-loop at vertex 1 with very negative cost.
    // With ng-path (v ∈ N(v)), extend_to_vertex marks v in its own
    // ng-neighborhood, so extend_along_arc detects the self-loop.
    int from[] = {0, 1, 1};
    int to[] = {1, 1, 2};
    double base_cost[] = {1.0, 1.0, 1.0};
    double time_d[] = {1.0, 0.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 3;
    pv.source = 0;
    pv.sink = 2;
    pv.n_arcs = 3;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    std::vector<std::vector<int>> neighbors = {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}};
    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, neighbors);
    using Pack = ResourcePack<NgPathResource>;

    double red_cost[] = {1.0, -100.0, 1.0};
    BucketGraph<Pack> bg(pv, Pack(ng),
                         {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    bg.build();
    bg.update_arc_costs(red_cost);
    auto paths = bg.solve();

    REQUIRE(!paths.empty());
    CHECK(paths[0].reduced_cost == doctest::Approx(2.0));  // 0→1→2, self-loop blocked
    CHECK(paths[0].vertices == std::vector<int>{0, 1, 2});
}

TEST_CASE("Stress: ng-path blocks cycle on triangle") {
    // Triangle: 0→1→2→0 with back-edge. Cycle blocked because 0 ∈ N(2).
    int from[] = {0, 1, 2, 0, 1};
    int to[] = {1, 2, 0, 3, 3};
    double base_cost[] = {1.0, 1.0, 1.0, 1.0, 5.0};
    double time_d[] = {1.0, 1.0, 1.0, 1.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 5;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    std::vector<std::vector<int>> neighbors = {
        {0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}};
    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, neighbors);
    using Pack = ResourcePack<NgPathResource>;

    double red_cost[] = {1.0, 1.0, -100.0, 1.0, 5.0};

    // Mono
    BucketGraph<Pack> mono(pv, Pack(ng),
                           {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    mono.build();
    mono.update_arc_costs(red_cost);
    auto mp = mono.solve();
    REQUIRE(!mp.empty());
    CHECK(mp[0].reduced_cost == doctest::Approx(1.0));  // 0→3

    // Bidir
    BucketGraph<Pack> bidir(
        pv, Pack(ng),
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    bidir.build();
    bidir.update_arc_costs(red_cost);
    auto bp = bidir.solve();
    REQUIRE(!bp.empty());
    CHECK(bp[0].reduced_cost == doctest::Approx(mp[0].reduced_cost));
}

// ════════════════════════════════════════════════════════════════════
// 10. Infeasible initial label (tests the null-ptr fix)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: infeasible source via resource returns empty") {
    // PickupDeliveryResource with source having impossible pickup demand.
    int from[] = {0};
    int to[] = {1};
    double base_cost[] = {1.0};
    double time_d[] = {1.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // Source pickup demand = 999, capacity Q = 10 → infeasible at source
    double pickup[] = {999.0, 0.0};
    double delivery[] = {0.0, 0.0};
    PickupDeliveryResource pd(pickup, delivery, 10.0, 2);
    using Pack = ResourcePack<PickupDeliveryResource>;

    BucketGraph<Pack> bg(pv, Pack(pd), {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    bg.build();
    auto paths = bg.solve();
    CHECK(paths.empty());  // graceful empty, no crash
}

TEST_CASE("Stress: infeasible source via resource, bidir returns empty") {
    int from[] = {0};
    int to[] = {1};
    double base_cost[] = {1.0};
    double time_d[] = {1.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    double pickup[] = {999.0, 0.0};
    double delivery[] = {0.0, 0.0};
    PickupDeliveryResource pd(pickup, delivery, 10.0, 2);
    using Pack = ResourcePack<PickupDeliveryResource>;

    BucketGraph<Pack> bg(
        pv, Pack(pd),
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    bg.build();
    auto paths = bg.solve();
    CHECK(paths.empty());
}

TEST_CASE("Stress: infeasible sink in bidir returns empty") {
    int from[] = {0};
    int to[] = {1};
    double base_cost[] = {1.0};
    double time_d[] = {1.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // Sink has impossible pickup demand
    double pickup[] = {0.0, 999.0};
    double delivery[] = {0.0, 0.0};
    PickupDeliveryResource pd(pickup, delivery, 10.0, 2);
    using Pack = ResourcePack<PickupDeliveryResource>;

    BucketGraph<Pack> bg(
        pv, Pack(pd),
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    bg.build();
    auto paths = bg.solve();
    CHECK(paths.empty());
}

TEST_CASE("Parallel bidir: fixing after a solve with an infeasible sink") {
    // The parallel solve computes completion bounds before seeding, and an
    // infeasible sink returns before the bw_c_best bounds are saved; fixing
    // and elimination must not read them (#111, #118).
    int from[] = {0};
    int to[] = {1};
    double base_cost[] = {1.0};
    double time_d[] = {1.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    double pickup[] = {0.0, 999.0};
    double delivery[] = {0.0, 0.0};
    PickupDeliveryResource pd(pickup, delivery, 10.0, 2);
    using Pack = ResourcePack<PickupDeliveryResource>;

    BucketGraph<Pack, StdThreadExecutor> bg(
        pv, Pack(pd),
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    bg.build();
    CHECK(bg.solve().empty());
    CHECK(bg.fix_buckets(1e9) == 0);
    bg.eliminate_arcs(1e9);
}

TEST_CASE("Sequential: fixing and elimination after a first solve with an infeasible end") {
    // A solve with an infeasible source or sink returns before computing
    // completion bounds; fixing and elimination then leave the graph as it
    // is rather than read the empty bounds (#118).
    int from[] = {0};
    int to[] = {1};
    double base_cost[] = {1.0};
    double time_d[] = {1.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    double pickup[] = {0.0, 0.0};
    double delivery[] = {0.0, 0.0};
    bool bidir = false;
    SUBCASE("mono, infeasible source") {
        pickup[0] = 999.0;
    }
    SUBCASE("bidir, infeasible source") {
        pickup[0] = 999.0;
        bidir = true;
    }
    SUBCASE("bidir, infeasible sink") {
        pickup[1] = 999.0;
        bidir = true;
    }
    PickupDeliveryResource pd(pickup, delivery, 10.0, 2);
    using Pack = ResourcePack<PickupDeliveryResource>;

    BucketGraph<Pack> bg(
        pv, Pack(pd),
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = bidir, .stage = Stage::Exact});
    bg.build();
    const auto arcs_before = bg.non_fixed_arc_count();
    CHECK(bg.solve().empty());
    CHECK(bg.fix_buckets(0.0) == 0);
    CHECK(bg.n_fixed_buckets() == 0);
    bg.eliminate_arcs(0.0);
    bg.eliminate_arcs_label_based(0.0);
    CHECK(bg.non_fixed_arc_count() == arcs_before);
}

TEST_CASE("Fixing after a rebuild waits for the next solve") {
    // build() replaces the grid the completion bounds were computed for.
    int from[] = {0};
    int to[] = {1};
    double base_cost[] = {1.0};
    double time_d[] = {1.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    BucketGraph<ResourcePack<>> bg(pv, ResourcePack<>{},
                                   {.bucket_steps = {5.0, 1.0}, .stage = Stage::Exact});
    bg.build();
    bg.solve();
    const auto n_before = bg.buckets().size();
    bg.double_xi();
    bg.build();
    REQUIRE(bg.buckets().size() > n_before);
    CHECK(bg.fix_buckets(-1.0) == 0);
    bg.eliminate_arcs(-1.0);
    bg.solve();
    CHECK(bg.fix_buckets(-1.0) > 0);
}

// ════════════════════════════════════════════════════════════════════
// 11. Mono vs bidir path contents match (not just cost)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: mono and bidir produce identical paths on larger graph") {
    // 6-vertex graph with multiple paths at different costs.
    //   0→1→3→5  cost=5+4+2=11, time=1+2+1=4
    //   0→2→4→5  cost=3+6+1=10, time=2+1+2=5
    //   0→1→4→5  cost=5+7+1=13, time=1+3+2=6
    //   0→2→3→5  cost=3+8+2=13, time=2+2+1=5
    int from[] = {0, 0, 1, 2, 3, 4, 1, 2};
    int to[] = {1, 2, 3, 4, 5, 5, 4, 3};
    double cost[] = {5.0, 3.0, 4.0, 6.0, 2.0, 1.0, 7.0, 8.0};
    double time_d[] = {1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 3.0, 2.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 6;
    pv.source = 0;
    pv.sink = 5;
    pv.n_arcs = 8;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_mono_bidir_agree(pv, 1e9, 1.0);
}

// ════════════════════════════════════════════════════════════════════
// 12. Waiting at vertex (time window lb > arrival time)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: forced waiting at vertex") {
    // 0→1→2. Arc 0→1 time=1, vertex 1 tw=[5,10]. Must wait until 5.
    // Arc 1→2 time=2. Arrive at 2: q=7.
    int from[] = {0, 1};
    int to[] = {1, 2};
    double cost[] = {1.0, 1.0};
    double time_d[] = {1.0, 2.0};
    double tw_lb[] = {0.0, 5.0, 0.0};
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

    check_mono_bidir_agree(pv, 1e9, 1.0);

    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {1.0, 1.0}, .theta = 1e9});
    bg.build();
    auto paths = bg.solve();
    REQUIRE(paths.size() == 1);
    CHECK(paths[0].reduced_cost == doctest::Approx(2.0));
}

// ════════════════════════════════════════════════════════════════════
// 13. Longer chain — bidir concatenation in the middle
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: long chain, bidir must concatenate in middle") {
    // 0→1→2→3→4→5, each arc cost=1, time=2. Total cost=5, time=10.
    // tw=[0,10] everywhere. Midpoint=5. Concat at arc 2→3 (fw q=6 > mid).
    constexpr int N = 6;
    constexpr int A = 5;
    int from[A];
    int to[A];
    double cost[A];
    double time_d[A];
    for (int i = 0; i < A; ++i) {
        from[i] = i;
        to[i] = i + 1;
        cost[i] = 1.0;
        time_d[i] = 2.0;
    }
    double tw_lb[N];
    double tw_ub[N];
    for (int i = 0; i < N; ++i) {
        tw_lb[i] = 0.0;
        tw_ub[i] = 10.0;
    }
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = N;
    pv.source = 0;
    pv.sink = N - 1;
    pv.n_arcs = A;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_mono_bidir_agree(pv, 1e9, 1.0);
}

// ════════════════════════════════════════════════════════════════════
// 14. Ng-path with cycle that dominance alone wouldn't catch
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: ng-path prevents revisit on triangle") {
    // Triangle: 0→1→2→0→1→3. Without ng-path, cycle 0→1→2→0 has cost=3.
    // With ng-path, 0→1→2→0 is blocked (0 revisited).
    // Only valid path: 0→1→3 (cost=1+5=6) and 0→2→1→3 (cost=2+4+5=11)
    int from[] = {0, 0, 1, 2, 1};
    int to[] = {1, 2, 2, 0, 3};
    double base_cost[] = {1.0, 2.0, 1.0, 1.0, 5.0};
    double time_d[] = {1.0, 1.0, 1.0, 1.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 5;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // Full neighborhoods including self: prevents revisiting any vertex
    std::vector<std::vector<int>> neighbors = {
        {0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}};
    NgPathResource ng(pv.n_vertices, pv.n_arcs, pv.arc_from, pv.arc_to, neighbors);
    using Pack = ResourcePack<NgPathResource>;

    // Mono
    BucketGraph<Pack> mono(pv, Pack(ng),
                           {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    mono.build();
    auto mp = mono.solve();
    REQUIRE(!mp.empty());
    CHECK(mp[0].reduced_cost == doctest::Approx(6.0));
    CHECK(mp[0].vertices == std::vector<int>{0, 1, 3});

    // Bidir
    BucketGraph<Pack> bidir(
        pv, Pack(ng),
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    bidir.build();
    auto bp = bidir.solve();
    REQUIRE(!bp.empty());
    CHECK(bp[0].reduced_cost == doctest::Approx(6.0));
}

// ════════════════════════════════════════════════════════════════════
// 15. Tolerance exactly at best path cost
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: theta exactly at best cost excludes path") {
    // Path cost = 4.0. Tolerance < 4.0 means no paths returned.
    int from[] = {0, 1};
    int to[] = {1, 2};
    double cost[] = {2.0, 2.0};
    double time_d[] = {1.0, 1.0};
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

    // theta = 4.0 → path with cost 4.0 NOT included (strictly <)
    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 4.0});
    bg.build();
    CHECK(bg.solve().empty());

    // theta = 4.0 + eps → included
    BucketGraph<EmptyPack> bg2(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 4.001});
    bg2.build();
    auto paths = bg2.solve();
    CHECK(paths.size() == 1);
}

// ════════════════════════════════════════════════════════════════════
// 16. Non-disposable resource (exact equality required for dominance)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: non-disposable resource with equal values allows dominance") {
    // Diamond 0→1→3, 0→2→3. Both paths have cost=2, time=5, cap=2.
    // With nondisposable cap, dominance requires exact equality on cap.
    // Both arrive at sink with identical (cost, time, cap) → one dominates.
    int from[] = {0, 0, 1, 2};
    int to[] = {1, 2, 3, 3};
    double cost[] = {1.0, 1.0, 1.0, 1.0};
    double time_d[] = {3.0, 5.0, 2.0, 0.0};
    double cap_d[] = {1.0, 2.0, 1.0, 0.0};  // path1: cap=1+1=2, path2: cap=2+0=2
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0, 10.0};
    double cap_lb[] = {0.0, 0.0, 0.0, 0.0};
    double cap_ub[] = {10.0, 10.0, 10.0, 10.0};
    const double* arc_res[] = {time_d, cap_d};
    const double* v_lb[] = {tw_lb, cap_lb};
    const double* v_ub[] = {tw_ub, cap_ub};

    bool nondisposable[] = {false, true};

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 4;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 2;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 2;
    pv.resource_nondisposable = nondisposable;

    BucketGraph<EmptyPack> bg(pv, EmptyPack{},
                              {.bucket_steps = {5.0, 5.0}, .theta = 1e9, .stage = Stage::Exact});
    bg.build();
    auto paths = bg.solve();
    CHECK(!paths.empty());
    CHECK(paths[0].reduced_cost == doctest::Approx(2.0));
}

TEST_CASE("Stress: non-disposable resource prevents dominance") {
    // Two paths to sink with same cost but different non-disposable values.
    int from[] = {0, 0, 1, 2};
    int to[] = {1, 2, 3, 3};
    double cost[] = {1.0, 1.0, 1.0, 1.0};
    double time_d[] = {3.0, 5.0, 2.0, 0.0};
    double cap_d[] = {1.0, 3.0, 1.0, 0.0};  // path1: cap=2, path2: cap=3
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0, 10.0};
    double cap_lb[] = {0.0, 0.0, 0.0, 0.0};
    double cap_ub[] = {10.0, 10.0, 10.0, 10.0};
    const double* arc_res[] = {time_d, cap_d};
    const double* v_lb[] = {tw_lb, cap_lb};
    const double* v_ub[] = {tw_ub, cap_ub};

    bool nondisposable[] = {false, true};

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 4;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 2;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 2;
    pv.resource_nondisposable = nondisposable;

    BucketGraph<EmptyPack> bg(pv, EmptyPack{},
                              {.bucket_steps = {5.0, 5.0}, .theta = 1e9, .stage = Stage::Exact});
    bg.build();
    auto paths = bg.solve();
    // Both paths cost=2 but different nondisposable resource → both survive
    CHECK(paths.size() == 2);
}

// ════════════════════════════════════════════════════════════════════
// 17. Wider stress: 10-vertex grid with many paths
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: 10-vertex layered graph, mono==bidir") {
    // Layer 0: {0}
    // Layer 1: {1, 2, 3}
    // Layer 2: {4, 5, 6}
    // Layer 3: {7, 8}
    // Layer 4: {9} (sink)
    // Full connections between adjacent layers.
    constexpr int N = 10;
    std::vector<int> vfrom;
    std::vector<int> vto;
    std::vector<double> vcost;
    std::vector<double> vtime;

    auto add_arc = [&](int f, int t, double c, double d) {
        vfrom.push_back(f);
        vto.push_back(t);
        vcost.push_back(c);
        vtime.push_back(d);
    };

    // Layer 0→1
    add_arc(0, 1, 2.0, 1.0);
    add_arc(0, 2, 3.0, 2.0);
    add_arc(0, 3, 4.0, 1.0);
    // Layer 1→2
    for (int i = 1; i <= 3; ++i) {
        for (int j = 4; j <= 6; ++j) {
            add_arc(i, j, 1.0 + (i + j) % 3, 1.0 + (i * j) % 2);
        }
    }
    // Layer 2→3
    for (int i = 4; i <= 6; ++i) {
        for (int j = 7; j <= 8; ++j) {
            add_arc(i, j, 2.0 + (i - j) % 2, 1.0);
        }
    }
    // Layer 3→4
    add_arc(7, 9, 1.0, 2.0);
    add_arc(8, 9, 3.0, 1.0);

    int A = static_cast<int>(vfrom.size());
    double tw_lb[N];
    double tw_ub[N];
    for (int i = 0; i < N; ++i) {
        tw_lb[i] = 0.0;
        tw_ub[i] = 30.0;
    }

    const double* arc_res[] = {vtime.data()};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = N;
    pv.source = 0;
    pv.sink = 9;
    pv.n_arcs = A;
    pv.arc_from = vfrom.data();
    pv.arc_to = vto.data();
    pv.arc_base_cost = vcost.data();
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_mono_bidir_agree(pv, 1e9, 2.0);
}

// ════════════════════════════════════════════════════════════════════
// CumulativeCostResource integration (Meta-Solver §5)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: CumulativeCostResource end-to-end solve") {
    // 4 vertices: 0=source, 3=sink
    // Two paths: 0→1→3 and 0→2→3
    //
    // Arc table:
    //   arc 0: 0→1, base_cost=1, time=2, travel_time=2, weight=3
    //   arc 1: 0→2, base_cost=2, time=1, travel_time=1, weight=1
    //   arc 2: 1→3, base_cost=1, time=1, travel_time=1, weight=2
    //   arc 3: 2→3, base_cost=1, time=2, travel_time=2, weight=4
    //
    // Path 0→1→3: base_cost = 1+1 = 2
    //   CCR forward: arc0: T=0+2=2, S=3*(0+2)=6, delta=6
    //                arc2: T=2+1=3, S=6+2*(2+1)=12, delta=6
    //   total cost = 2 + 12 = 14
    //
    // Path 0→2→3: base_cost = 2+1 = 3
    //   CCR forward: arc1: T=0+1=1, S=1*(0+1)=1, delta=1
    //                arc3: T=1+2=3, S=1+4*(1+2)=13, delta=12
    //   total cost = 3 + 13 = 16
    //
    // Best path: 0→1→3 with cost 14.

    int from[] = {0, 0, 1, 2};
    int to[] = {1, 2, 3, 3};
    double base_cost[] = {1.0, 2.0, 1.0, 1.0};
    double time_d[] = {2.0, 1.0, 1.0, 2.0};  // main resource
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
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    double travel_time[] = {2.0, 1.0, 1.0, 2.0};
    double weight[] = {3.0, 1.0, 2.0, 4.0};
    CumulativeCostResource ccr(travel_time, weight, /*W_max=*/10.0,
                               /*T_max=*/10.0, /*n_arcs=*/4);
    using Pack = ResourcePack<CumulativeCostResource>;

    BucketGraph<Pack> bg(pv, Pack(ccr),
                         {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    bg.build();
    auto paths = bg.solve();

    REQUIRE(!paths.empty());
    CHECK(paths[0].reduced_cost == doctest::Approx(14.0).epsilon(1e-6));
    CHECK(paths[0].vertices.front() == 0);
    CHECK(paths[0].vertices.back() == 3);
}

TEST_CASE("Stress: CumulativeCostResource bidir agrees with mono") {
    int from[] = {0, 0, 1, 2};
    int to[] = {1, 2, 3, 3};
    double base_cost[] = {1.0, 2.0, 1.0, 1.0};
    double time_d[] = {2.0, 1.0, 1.0, 2.0};
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
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    double travel_time[] = {2.0, 1.0, 1.0, 2.0};
    double weight[] = {3.0, 1.0, 2.0, 4.0};
    CumulativeCostResource ccr(travel_time, weight, 10.0, 10.0, 4);
    using Pack = ResourcePack<CumulativeCostResource>;

    BucketGraph<Pack> mono(pv, Pack(ccr),
                           {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    mono.build();
    auto mp = mono.solve();

    BucketGraph<Pack> bidir(
        pv, Pack(ccr),
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    bidir.build();
    auto bp = bidir.solve();

    REQUIRE(!mp.empty());
    REQUIRE(!bp.empty());
    CHECK(mp[0].reduced_cost == doctest::Approx(bp[0].reduced_cost).epsilon(1e-6));
}

// ════════════════════════════════════════════════════════════════════
// PickupDeliveryResource integration with feasible paths (Meta-Solver §6)
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: PickupDeliveryResource feasible solve") {
    // 4 vertices: 0=source(depot), 3=sink(depot)
    // Customers 1 and 2 have pickup/delivery demands.
    // Vehicle capacity Q = 10.
    //
    // Path 0→1→3: pickup 3, delivery 2 at vertex 1. Feasible.
    // Path 0→2→3: pickup 8, delivery 1 at vertex 2. Feasible.
    // Path 0→1→2→3: cumulative pickup = 3+8 = 11 > Q=10. Infeasible!

    int from[] = {0, 0, 1, 2, 1};
    int to[] = {1, 2, 3, 3, 2};
    double base_cost[] = {1.0, 2.0, 3.0, 1.0, 1.0};
    double time_d[] = {1.0, 1.0, 1.0, 1.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0, 10.0};

    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 5;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = base_cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // Depot: no demand. Customer 1: pickup=3, delivery=2. Customer 2: pickup=8,
    // delivery=1.
    double pickup[] = {0.0, 3.0, 8.0, 0.0};
    double delivery[] = {0.0, 2.0, 1.0, 0.0};
    PickupDeliveryResource pd(pickup, delivery, /*Q=*/10.0, /*n_vertices=*/4);
    using Pack = ResourcePack<PickupDeliveryResource>;

    BucketGraph<Pack> bg(pv, Pack(pd),
                         {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    bg.build();
    auto paths = bg.solve();

    REQUIRE(!paths.empty());
    // Best feasible path: 0→2→3 cost=3 or 0→1→3 cost=4
    CHECK(paths[0].reduced_cost == doctest::Approx(3.0).epsilon(1e-6));

    // All returned paths must be feasible (no path with cumulative pickup > Q)
    for (const auto& p : paths) {
        double total_pickup = 0.0;
        for (int v : p.vertices) {
            total_pickup += pickup[v];
        }
        CHECK(total_pickup <= 10.0 + 1e-6);
    }
}

// ════════════════════════════════════════════════════════════════════
// Warm label injection across Solver stage transitions
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Stress: warm labels across Heuristic1 to Exact transition") {
    // Use a graph with multiple paths so Heuristic1 (1 label/bucket) may
    // miss the optimal, but warm labels help Exact converge.
    //
    // LargerGraph-style: 6 vertices, 8 arcs.
    int from[] = {0, 0, 1, 2, 3, 4, 1, 2};
    int to[] = {1, 2, 3, 4, 5, 5, 4, 3};
    double cost[] = {5.0, 3.0, 4.0, 6.0, 2.0, 1.0, 7.0, 8.0};
    double time_d[] = {1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 3.0, 2.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};

    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 6;
    pv.source = 0;
    pv.sink = 5;
    pv.n_arcs = 8;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // Get exact optimal for reference
    BucketGraph<EmptyPack> ref(pv, EmptyPack{},
                               {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    ref.build();
    auto exact_paths = ref.solve();
    REQUIRE(!exact_paths.empty());
    double exact_best = exact_paths[0].reduced_cost;

    // Solver starts at Heuristic1, transitions through stages
    Solver<EmptyPack> solver(pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    solver.build();

    CHECK(solver.current_stage() == Stage::Heuristic1);

    // First solve in Heuristic1
    auto paths1 = solver.solve();
    REQUIRE(!paths1.empty());
    // Heuristic1 must find cost >= exact (it's a heuristic)
    CHECK(paths1[0].reduced_cost >= exact_best - 1e-6);

    // Save warm labels
    solver.save_warm_labels(0.7);

    // Force to Exact for deterministic comparison
    solver.set_stage(Stage::Exact);

    // Solve in Exact with warm labels injected
    auto paths2 = solver.solve();
    REQUIRE(!paths2.empty());
    CHECK(paths2[0].reduced_cost == doctest::Approx(exact_best).epsilon(1e-6));
}

TEST_CASE("Stress: warm labels with ng-path resource across stages") {
    // Verify warm labels preserve ng-path resource states correctly.
    int from[] = {0, 0, 1, 2, 1, 2};
    int to[] = {1, 2, 3, 3, 2, 1};
    double cost[] = {1.0, 2.0, 3.0, 1.0, 1.0, 1.0};
    double time_d[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0, 10.0};

    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 6;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // ng-neighborhoods: each vertex sees all others (k=3)
    std::vector<std::vector<int>> neighbors = {{1, 2, 3}, {0, 2, 3}, {0, 1, 3}, {0, 1, 2}};
    NgPathResource ng(4, 6, from, to, neighbors);
    using Pack = ResourcePack<NgPathResource>;

    // Exact solve for reference
    BucketGraph<Pack> ref(pv, Pack(ng),
                          {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .stage = Stage::Exact});
    ref.build();
    auto exact_paths = ref.solve();
    REQUIRE(!exact_paths.empty());
    double exact_best = exact_paths[0].reduced_cost;

    // Solver with warm labels across Heuristic1 → Exact
    Solver<Pack> solver(pv, Pack(ng), {.bucket_steps = {5.0, 1.0}, .theta = 1e9});
    solver.build();

    auto paths1 = solver.solve();
    REQUIRE(!paths1.empty());

    solver.save_warm_labels(0.7);
    solver.set_stage(Stage::Exact);

    auto paths2 = solver.solve();
    REQUIRE(!paths2.empty());
    CHECK(paths2[0].reduced_cost == doctest::Approx(exact_best).epsilon(1e-6));
}

// ════════════════════════════════════════════════════════════════════
// Parallel bidirectional labeling
// ════════════════════════════════════════════════════════════════════

TEST_CASE("Parallel bidir: single-arc source→sink") {
    int from[] = {0};
    int to[] = {1};
    double cost[] = {7.0};
    double time_d[] = {3.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_parallel_bidir_agree(pv);
}

TEST_CASE("Parallel bidir: 5-vertex diamond graph") {
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

    check_parallel_bidir_agree(pv, 1e9, 5.0);
    check_parallel_bidir_agree(pv, 1e9, 1.0);
}

TEST_CASE("Parallel bidir: 6-vertex with cross arcs") {
    int from[] = {0, 0, 1, 2, 3, 4, 1, 2};
    int to[] = {1, 2, 3, 4, 5, 5, 4, 3};
    double cost[] = {5.0, 3.0, 4.0, 6.0, 2.0, 1.0, 7.0, 8.0};
    double time_d[] = {1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 3.0, 2.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 6;
    pv.source = 0;
    pv.sink = 5;
    pv.n_arcs = 8;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_parallel_bidir_agree(pv, 1e9, 1.0);
}

TEST_CASE("Parallel bidir: random 8-vertex graph") {
    constexpr int N = 8;
    std::vector<int> from_v;
    std::vector<int> to_v;
    std::vector<double> cost_v;
    std::vector<double> time_v;
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            from_v.push_back(i);
            to_v.push_back(j);
            cost_v.push_back(1.0 + (i * 7 + j * 3) % 10);
            time_v.push_back(1.0 + (i + j) % 4);
        }
    }
    int n_arcs = static_cast<int>(from_v.size());
    std::vector<double> tw_lb(N, 0.0);
    std::vector<double> tw_ub(N, 30.0);
    const double* arc_res[] = {time_v.data()};
    const double* v_lb[] = {tw_lb.data()};
    const double* v_ub[] = {tw_ub.data()};

    ProblemView pv;
    pv.n_vertices = N;
    pv.source = 0;
    pv.sink = N - 1;
    pv.n_arcs = n_arcs;
    pv.arc_from = from_v.data();
    pv.arc_to = to_v.data();
    pv.arc_base_cost = cost_v.data();
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    check_parallel_bidir_agree(pv, 1e9, 3.0);
    check_parallel_bidir_agree(pv, 1e9, 1.0);
}

TEST_CASE("Parallel bidir: with NgPathResource") {
    constexpr int N = 6;
    int from[] = {0, 0, 1, 2, 3, 4, 1, 2};
    int to[] = {1, 2, 3, 4, 5, 5, 4, 3};
    double cost[] = {5.0, 3.0, 4.0, 6.0, 2.0, 1.0, 7.0, 8.0};
    double time_d[] = {1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 3.0, 2.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = N;
    pv.source = 0;
    pv.sink = N - 1;
    pv.n_arcs = 8;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    // Build ng-neighborhoods: 3 nearest neighbors per vertex
    std::vector<std::vector<int>> neighbors = {{0, 1, 2}, {1, 0, 3}, {2, 0, 4},
                                               {3, 1, 4}, {4, 2, 3}, {5, 3, 4}};
    NgPathResource ng(N, 8, from, to, neighbors);
    using Pack = ResourcePack<NgPathResource>;

    BucketGraph<Pack> seq(
        pv, Pack(ng),
        {.bucket_steps = {1.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    seq.build();
    auto sp = seq.solve();

    BucketGraph<Pack, StdThreadExecutor> par(
        pv, Pack(ng),
        {.bucket_steps = {1.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    par.build();
    auto pp = par.solve();

    REQUIRE(!sp.empty());
    REQUIRE(!pp.empty());
    CHECK(sp[0].reduced_cost == doctest::Approx(pp[0].reduced_cost).epsilon(1e-6));
}

TEST_CASE("Parallel bidir: via Solver API") {
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

    Solver<EmptyPack> seq_solver(pv, EmptyPack{},
                                 {.bucket_steps = {5.0, 1.0}, .bidirectional = true, .theta = 1e9});
    seq_solver.set_stage(Stage::Exact);
    seq_solver.build();
    auto sp = seq_solver.solve();

    Solver<EmptyPack, StdThreadExecutor> par_solver(
        pv, EmptyPack{}, {.bucket_steps = {5.0, 1.0}, .bidirectional = true, .theta = 1e9});
    par_solver.set_stage(Stage::Exact);
    par_solver.build();
    auto pp = par_solver.solve();

    REQUIRE(!sp.empty());
    REQUIRE(!pp.empty());
    CHECK(sp[0].reduced_cost == doctest::Approx(pp[0].reduced_cost).epsilon(1e-6));

    // Check timing field is populated
    CHECK(par_solver.solve_timings().parallel_labeling.count() > 0.0);
}

TEST_CASE("Parallel bidir: infeasible instance") {
    int from[] = {0};
    int to[] = {1};
    double cost[] = {1.0};
    double time_d[] = {15.0};
    double tw_lb[] = {0.0, 0.0};
    double tw_ub[] = {10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 2;
    pv.source = 0;
    pv.sink = 1;
    pv.n_arcs = 1;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    BucketGraph<EmptyPack, StdThreadExecutor> par(
        pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    par.build();
    auto pp = par.solve();
    CHECK(pp.empty());
}

TEST_CASE("Parallel bidir: with reduced costs") {
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

    double reduced[] = {-5.0, 3.0, 5.0, 4.0, -3.0, 2.0, 7.0};

    BucketGraph<EmptyPack> seq(
        pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    seq.build();
    seq.update_arc_costs(reduced);
    auto sp = seq.solve();

    BucketGraph<EmptyPack, StdThreadExecutor> par(
        pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    par.build();
    par.update_arc_costs(reduced);
    auto pp = par.solve();

    REQUIRE(!sp.empty());
    REQUIRE(!pp.empty());
    CHECK(sp[0].reduced_cost == doctest::Approx(pp[0].reduced_cost).epsilon(1e-6));
}

TEST_CASE("Parallel bidir: with R1CResource") {
    constexpr int N = 6;
    int from[] = {0, 0, 1, 2, 3, 4, 1, 2};
    int to[] = {1, 2, 3, 4, 5, 5, 4, 3};
    double cost[] = {5.0, 3.0, 4.0, 6.0, 2.0, 1.0, 7.0, 8.0};
    double time_d[] = {1.0, 2.0, 2.0, 1.0, 1.0, 2.0, 3.0, 2.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = N;
    pv.source = 0;
    pv.sink = N - 1;
    pv.n_arcs = 8;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    using R1CPack = ResourcePack<R1CResource>;

    R1Cut cut;
    cut.base_set = {3};
    cut.multipliers = {0.5};
    cut.memory_arcs = {{2, 0}, {7, 0}};
    cut.dual_value = -5.0;

    R1CResource r1c_seq;
    BucketGraph<R1CPack> seq(
        pv, R1CPack{r1c_seq},
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    seq.resource<R1CResource>().set_cuts({{cut}}, N, 8);
    seq.build();
    auto sp = seq.solve();

    R1CResource r1c_par;
    BucketGraph<R1CPack, StdThreadExecutor> par(
        pv, R1CPack{r1c_par},
        {.bucket_steps = {5.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    par.resource<R1CResource>().set_cuts({{cut}}, N, 8);
    par.build();
    auto pp = par.solve();

    REQUIRE(!sp.empty());
    REQUIRE(!pp.empty());
    CHECK(sp[0].reduced_cost == doctest::Approx(pp[0].reduced_cost).epsilon(1e-6));
}

TEST_CASE("Parallel bidir: symmetric + StdThreadExecutor falls back to sequential") {
    int from[] = {0, 0, 1, 2};
    int to[] = {1, 2, 3, 3};
    double cost[] = {5.0, 3.0, 4.0, 6.0};
    double time_d[] = {1.0, 2.0, 2.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0};
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

    BucketGraph<EmptyPack> seq(pv, EmptyPack{},
                               {.bucket_steps = {5.0, 1.0},
                                .theta = 1e9,
                                .bidirectional = true,
                                .symmetric = true,
                                .stage = Stage::Exact});
    seq.build();
    auto sp = seq.solve();

    BucketGraph<EmptyPack, StdThreadExecutor> par(pv, EmptyPack{},
                                                  {.bucket_steps = {5.0, 1.0},
                                                   .theta = 1e9,
                                                   .bidirectional = true,
                                                   .symmetric = true,
                                                   .stage = Stage::Exact});
    par.build();
    auto pp = par.solve();

    REQUIRE(!sp.empty());
    REQUIRE(!pp.empty());
    CHECK(sp[0].reduced_cost == doctest::Approx(pp[0].reduced_cost).epsilon(1e-6));
    CHECK(par.solve_timings().parallel_labeling.count() == 0.0);
}

TEST_CASE("Parallel bidir: Enumerate stage") {
    int from[] = {0, 0, 1, 2, 1};
    int to[] = {1, 2, 3, 3, 2};
    double cost[] = {2.0, 3.0, 4.0, 1.0, 5.0};
    double time_d[] = {1.0, 2.0, 2.0, 1.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 4;
    pv.source = 0;
    pv.sink = 3;
    pv.n_arcs = 5;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    BucketGraph<EmptyPack> seq(pv, EmptyPack{},
                               {.bucket_steps = {5.0, 1.0},
                                .theta = 1e9,
                                .bidirectional = true,
                                .stage = Stage::Enumerate});
    seq.build();
    auto sp = seq.solve();

    BucketGraph<EmptyPack, StdThreadExecutor> par(pv, EmptyPack{},
                                                  {.bucket_steps = {5.0, 1.0},
                                                   .theta = 1e9,
                                                   .bidirectional = true,
                                                   .stage = Stage::Enumerate});
    par.build();
    auto pp = par.solve();

    REQUIRE(!sp.empty());
    REQUIRE(!pp.empty());
    CHECK(sp[0].reduced_cost == doctest::Approx(pp[0].reduced_cost).epsilon(1e-6));
    CHECK(sp.size() == pp.size());
}

TEST_CASE("Bidir: a label cap leaves fw labels to join, each path once") {
    // Chain 0 -> 1 -> 2 -> 3 -> 4, windows [0, 10] (midpoint 5), unit times: no
    // fw label passes the midpoint. A low enumeration cap stops the labeling
    // with fw labels below it unextended; they still join the bw labels, and
    // the one path comes back once.
    int from[] = {0, 1, 2, 3};
    int to[] = {1, 2, 3, 4};
    double cost[] = {-5.0, 1.0, 1.0, 1.0};
    double time_d[] = {1.0, 1.0, 1.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0, 10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 4;
    pv.n_arcs = 4;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    for (int cap = 3; cap <= 8; ++cap) {
        CAPTURE(cap);
        BucketGraph<EmptyPack> bg(pv, EmptyPack{},
                                  {.theta = 1e9,
                                   .bidirectional = true,
                                   .stage = Stage::Enumerate,
                                   .max_enum_labels = cap});
        bg.build();
        auto paths = bg.solve();
        REQUIRE(paths.size() == 1);
        CHECK(paths[0].vertices == std::vector<int>{0, 1, 2, 3, 4});
        CHECK(paths[0].reduced_cost == doctest::Approx(-2.0));
    }
}

/// Random time-window (and capacity) instance with negative arc costs, so bw
/// labels can be cheaper than 0: 0 is the source, n - 1 the sink.
struct RandomTwInstance {
    std::vector<int> from_v;
    std::vector<int> to_v;
    std::vector<double> cost_v;
    std::vector<double> time_v;
    std::vector<double> load_v;
    std::vector<double> lb0;
    std::vector<double> ub0;
    std::vector<double> lb1;
    std::vector<double> ub1;
    std::array<const double*, 2> arc_res{};
    std::array<const double*, 2> v_lb{};
    std::array<const double*, 2> v_ub{};
    ProblemView pv;

    RandomTwInstance(int seed, int n_main) {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> U(0, 1);
        const int n = 6 + seed % 6;
        std::vector<double> x(n);
        std::vector<double> y(n);
        std::vector<double> du(n);
        std::vector<double> dem(n);
        lb0.resize(n);
        ub0.resize(n);
        lb1.assign(n, 0.0);
        ub1.resize(n);
        for (int v = 0; v < n; ++v) {
            x[v] = U(rng) * 10;
            y[v] = U(rng) * 10;
            du[v] = (v > 0 && v < n - 1) ? U(rng) * 14 : 0;
            dem[v] = (v > 0 && v < n - 1) ? 1 + static_cast<int>(U(rng) * 3) : 0;
            double c = U(rng) * 40;
            lb0[v] = c;
            ub0[v] = c + 5 + U(rng) * 20;
        }
        x[n - 1] = x[0];
        y[n - 1] = y[0];
        lb0[0] = lb0[n - 1] = 0;
        ub0[0] = ub0[n - 1] = 60;
        std::ranges::fill(ub1, 6 + static_cast<int>(U(rng) * 6));

        for (int u = 0; u <= n - 2; ++u) {
            for (int v = 1; v <= n - 1; ++v) {
                if (u == v || (u == 0 && v == n - 1)) {
                    continue;
                }
                double d = std::hypot(x[u] - x[v], y[u] - y[v]);
                from_v.push_back(u);
                to_v.push_back(v);
                cost_v.push_back(d - du[u] / 2 - du[v] / 2);
                time_v.push_back(d + 1);
                load_v.push_back(dem[v]);
            }
        }
        arc_res = {time_v.data(), load_v.data()};
        v_lb = {lb0.data(), lb1.data()};
        v_ub = {ub0.data(), ub1.data()};

        pv.n_vertices = n;
        pv.source = 0;
        pv.sink = n - 1;
        pv.n_arcs = static_cast<int>(from_v.size());
        pv.arc_from = from_v.data();
        pv.arc_to = to_v.data();
        pv.arc_base_cost = cost_v.data();
        pv.n_resources = 2;
        pv.arc_resource = arc_res.data();
        pv.vertex_lb = v_lb.data();
        pv.vertex_ub = v_ub.data();
        pv.n_main_resources = n_main;
    }

    // pv points into the vectors above.
    RandomTwInstance(const RandomTwInstance&) = delete;
    RandomTwInstance& operator=(const RandomTwInstance&) = delete;
};

template <typename Paths>
static double best_reduced_cost(const Paths& paths) {
    double b = INF;
    for (const auto& p : paths) {
        b = std::min(b, p.reduced_cost);
    }
    return b;
}

TEST_CASE("Bidir finds the mono best at the default theta on random instances") {
    // Bidir at the default theta must return the mono best; a join skip that
    // took bw costs as non-negative lost it on seeds 936 and 1099 (1D) and
    // 459 (2D).
    for (int n_main = 1; n_main <= 2; ++n_main) {
        for (int seed = 0; seed < 1200; ++seed) {
            CAPTURE(n_main);
            CAPTURE(seed);
            RandomTwInstance inst(seed, n_main);

            auto best = [&](bool bidir) {
                BucketGraph<EmptyPack> bg(
                    inst.pv, EmptyPack{},
                    {.bucket_steps = {2.0, n_main == 2 ? 2.0 : 0.0}, .bidirectional = bidir});
                bg.build();
                return best_reduced_cost(bg.solve());
            };
            const double mono = best(false);
            const double bidir = best(true);
            if (mono >= INF) {
                CHECK(bidir >= INF);
            } else {
                CHECK(bidir == doctest::Approx(mono));
            }
        }
    }
}

TEST_CASE("Bidir arc elimination keeps the arcs of an improving path (#111)") {
    // Chain 0 -> 1 -> 2 -> 3 -> 4, windows [0, 10] (midpoint 5): the first
    // arc takes 5.5, so the fw labels at 2 and 3 lie past the midpoint and
    // were never there to give c_best. Elimination took that as no path and
    // removed the arcs out of 2 and 3; every later solve came back empty.
    int from[] = {0, 1, 2, 3};
    int to[] = {1, 2, 3, 4};
    double cost[] = {-5.0, 1.0, 1.0, 1.0};
    double time_d[] = {5.5, 0.1, 0.1, 0.1};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 10.0, 10.0, 10.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 5;
    pv.source = 0;
    pv.sink = 4;
    pv.n_arcs = 4;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    auto run = [&]<typename Exec>(Exec) {
        for (bool label_based : {false, true}) {
            CAPTURE(label_based);
            BucketGraph<EmptyPack, Exec> bg(pv, EmptyPack{},
                                            {.bucket_steps = {0.25, 1.0},
                                             .bidirectional = true,
                                             .parallel_bidir = false,
                                             .stage = Stage::Exact});
            bg.build();
            for (int k = 0; k < 6; ++k) {
                CAPTURE(k);
                auto paths = bg.solve();
                REQUIRE(paths.size() == 1);
                CHECK(paths[0].vertices == std::vector<int>{0, 1, 2, 3, 4});
                CHECK(paths[0].reduced_cost == doctest::Approx(-2.0));
                if (label_based) {
                    bg.eliminate_arcs_label_based(-1e-6);
                    bg.fix_buckets(-1e-6);
                } else {
                    bg.eliminate_arcs(-1e-6);
                }
            }
        }
    };
    run(SequentialExecutor{});
    run(StdThreadExecutor{});
}

TEST_CASE("Bidir elimination and fixing keep the mono best on random instances (#111)") {
    // Repeated bidir solves, each followed by elimination (bound- or
    // label-based, the latter with bucket fixing), must keep returning the
    // mono best: the midpoint moves between solves, and elimination may drop
    // only arcs no improving path uses. Before #111, 11 (bound-based) and 35
    // (label-based with fixing) of the 597 improving instances here lost it.
    auto run = [&]<typename Exec>(Exec, bool parallel_bidir) {
        for (int n_main = 1; n_main <= 2; ++n_main) {
            for (int seed = 0; seed < 300; ++seed) {
                RandomTwInstance inst(seed, n_main);
                typename BucketGraph<EmptyPack, Exec>::Options opts{
                    .bucket_steps = {2.0, n_main == 2 ? 2.0 : 0.0}};
                BucketGraph<EmptyPack, Exec> mono(inst.pv, EmptyPack{}, opts);
                mono.build();
                const double best = best_reduced_cost(mono.solve());
                if (best >= opts.theta) {
                    continue;  // nothing improving: elimination may drop it all
                }
                for (bool label_based : {false, true}) {
                    CAPTURE(parallel_bidir);
                    CAPTURE(n_main);
                    CAPTURE(seed);
                    CAPTURE(label_based);
                    opts.bidirectional = true;
                    opts.parallel_bidir = parallel_bidir;
                    BucketGraph<EmptyPack, Exec> bg(inst.pv, EmptyPack{}, opts);
                    bg.build();
                    for (int k = 0; k < 6; ++k) {
                        CAPTURE(k);
                        REQUIRE(best_reduced_cost(bg.solve()) == doctest::Approx(best));
                        if (label_based) {
                            bg.eliminate_arcs_label_based(opts.theta);
                            bg.fix_buckets(opts.theta);
                        } else {
                            bg.eliminate_arcs(opts.theta);
                        }
                    }
                }
            }
        }
    };
    run(SequentialExecutor{}, false);
    run(StdThreadExecutor{}, true);
}

TEST_CASE("Parallel bidir: parallel_bidir=false gives sequential bidir with StdThreadExecutor") {
    int from[] = {0, 0, 1, 2};
    int to[] = {1, 2, 3, 3};
    double cost[] = {5.0, 3.0, 4.0, 6.0};
    double time_d[] = {1.0, 2.0, 2.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0, 20.0};
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

    // parallel_bidir=true: concurrent fw/bw
    BucketGraph<EmptyPack, StdThreadExecutor> par_bidir(pv, EmptyPack{},
                                                        {.bucket_steps = {5.0, 1.0},
                                                         .theta = 1e9,
                                                         .bidirectional = true,
                                                         .parallel_bidir = true,
                                                         .stage = Stage::Exact});
    par_bidir.build();
    auto pp = par_bidir.solve();

    // parallel_bidir=false: data-parallel only, sequential bidir
    BucketGraph<EmptyPack, StdThreadExecutor> seq_bidir(pv, EmptyPack{},
                                                        {.bucket_steps = {5.0, 1.0},
                                                         .theta = 1e9,
                                                         .bidirectional = true,
                                                         .parallel_bidir = false,
                                                         .stage = Stage::Exact});
    seq_bidir.build();
    auto sp = seq_bidir.solve();

    REQUIRE(!pp.empty());
    REQUIRE(!sp.empty());
    CHECK(pp[0].reduced_cost == doctest::Approx(sp[0].reduced_cost).epsilon(1e-6));
    // parallel_bidir=false should not use parallel labeling
    CHECK(seq_bidir.solve_timings().parallel_labeling.count() == 0.0);
    // parallel_bidir=true should use parallel labeling
    CHECK(par_bidir.solve_timings().parallel_labeling.count() > 0.0);
}

// ════════════════════════════════════════════════════════════════════
// Shared pricer contract (flowty/flowty#58): a core-owned ThreadPool and
// borrowed derived data must solve exactly like the owning, sequential setup.
// ════════════════════════════════════════════════════════════════════

TEST_CASE("ThreadPoolExecutor: borrowed ng and R1C views match owned sequential") {
    // Customers 1..4 with a negative cycle 1 -> 2 -> 3 -> 1 (and 2 -> 1,
    // 3 -> 2), so ng has cycles to block, and a 3-SRC cut on {1, 2, 3} that the
    // best elementary path covers more than once, so the cut has a cost to add.
    constexpr int N = 6;
    std::vector<int> from = {0, 0, 0, 1, 2, 3, 2, 3, 1, 2, 3, 4};
    std::vector<int> to = {1, 2, 3, 2, 3, 1, 1, 2, 4, 4, 4, 5};
    std::vector<double> cost = {1.0, 1.0, 1.0, -4.0, -4.0, -4.0, -3.0, -3.0, 1.0, 1.0, 1.0, 0.0};
    const int n_arcs = static_cast<int>(from.size());
    std::vector<double> time_d(n_arcs, 1.0);
    std::vector<double> tw_lb(N, 0.0);
    std::vector<double> tw_ub(N, 20.0);
    const double* arc_res[] = {time_d.data()};
    const double* v_lb[] = {tw_lb.data()};
    const double* v_ub[] = {tw_ub.data()};

    ProblemView pv;
    pv.n_vertices = N;
    pv.source = 0;
    pv.sink = N - 1;
    pv.n_arcs = n_arcs;
    pv.arc_from = from.data();
    pv.arc_to = to.data();
    pv.arc_base_cost = cost.data();
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    std::vector<std::vector<int>> neighbors(N);
    for (int v = 0; v < N; ++v) {
        for (int w = 0; w < N; ++w) {
            neighbors[v].push_back(w);
        }
    }
    R1Cut cut;
    cut.base_set = {1, 2, 3};
    cut.multipliers = {0.5, 0.5, 0.5};
    for (int a = 0; a < n_arcs; ++a) {
        cut.memory_arcs.emplace_back(a, 0);  // full memory
    }
    cut.dual_value = -5.0;
    const std::vector<R1Cut> cuts = {cut};

    // Core side: derived data built once, one pool for every solve.
    const NgPathData ng_data(N, n_arcs, from.data(), to.data(), neighbors);
    R1CData r1c_data;
    r1c_data.set_cuts(cuts, N, n_arcs);
    R1CData no_cuts;
    ThreadPool pool(4);
    const ThreadPoolExecutor exec(pool);

    using OwnedPack = ResourcePack<NgPathResource, R1CResource>;
    using ViewPack = ResourcePack<NgPathView, R1CView>;
    static_assert(detail::NgResourceIndex<ViewPack>::has_ng);
    static_assert(detail::R1CResourceIndex<ViewPack>::has_r1c);

    auto best = [](auto& bg) {
        bg.build();
        auto paths = bg.solve();
        REQUIRE(!paths.empty());
        return paths[0].reduced_cost;
    };

    for (const bool bidirectional : {false, true}) {
        CAPTURE(bidirectional);
        const BucketGraph<OwnedPack>::Options opts{.bucket_steps = {5.0, 1.0},
                                                   .theta = 1e9,
                                                   .bidirectional = bidirectional,
                                                   .stage = Stage::Exact};
        const BucketGraph<ViewPack>::Options view_opts{.bucket_steps = {5.0, 1.0},
                                                       .theta = 1e9,
                                                       .bidirectional = bidirectional,
                                                       .stage = Stage::Exact};
        const BucketGraph<ViewPack, ThreadPoolExecutor>::Options pool_opts{
            .bucket_steps = {5.0, 1.0},
            .theta = 1e9,
            .bidirectional = bidirectional,
            .stage = Stage::Exact};

        BucketGraph<OwnedPack> owned(
            pv, OwnedPack(NgPathResource(N, n_arcs, from.data(), to.data(), neighbors), {}), opts);
        owned.resource<R1CResource>().set_cuts(cuts, N, n_arcs);
        const double owned_cost = best(owned);

        BucketGraph<ViewPack> view_seq(pv, ViewPack(NgPathView(ng_data), R1CView(r1c_data)),
                                       view_opts);
        BucketGraph<ViewPack, ThreadPoolExecutor> view_pool(
            pv, ViewPack(NgPathView(ng_data), R1CView(r1c_data)), pool_opts, exec);
        CHECK(best(view_seq) == doctest::Approx(owned_cost).epsilon(1e-9));
        CHECK(best(view_pool) == doctest::Approx(owned_cost).epsilon(1e-9));

        // Both resources bite: without the cut the path is cheaper, and
        // without ng-sets the negative cycle makes it cheaper still.
        BucketGraph<ViewPack> uncut(pv, ViewPack(NgPathView(ng_data), R1CView(no_cuts)), view_opts);
        const double uncut_cost = best(uncut);
        CHECK(uncut_cost < owned_cost - 1.0);
        BucketGraph<EmptyPack> relaxed(pv, EmptyPack{},
                                       {.bucket_steps = {5.0, 1.0},
                                        .theta = 1e9,
                                        .bidirectional = bidirectional,
                                        .stage = Stage::Exact});
        CHECK(best(relaxed) < uncut_cost - 1.0);
    }
}

/// Forwards to a ThreadPoolExecutor and counts pool-sized dispatches made
/// while a parallel_invoke is running, i.e. the nested case.
struct NestingProbeExecutor {
    ThreadPoolExecutor inner;
    std::atomic<int>* in_invoke;
    std::atomic<int>* nested;

    void note(int begin, int end) const {
        if (in_invoke->load() > 0 && end - begin >= kParallelForMinChunk) {
            nested->fetch_add(1);
        }
    }
    void parallel_for(int begin, int end, auto&& f) const {
        note(begin, end);
        inner.parallel_for(begin, end, f);
    }
    void parallel_for_chunked(int begin, int end, auto&& f) const {
        note(begin, end);
        inner.parallel_for_chunked(begin, end, f);
    }
    void parallel_invoke(auto&& f, auto&& g) const {
        in_invoke->fetch_add(1);
        inner.parallel_invoke(f, g);
        in_invoke->fetch_sub(1);
    }
    [[nodiscard]] int n_threads() const { return inner.n_threads(); }
};
static_assert(Executor<NestingProbeExecutor>);

TEST_CASE("ThreadPoolExecutor: bidir with nested pool dispatch matches sequential") {
    // compact_labels dispatches once a bucket SCC has >= 1024 buckets. With a
    // bucket step wider than the horizon each vertex has one bucket, and the
    // customer rings (i -> i+1, i -> i+7) make all customers one SCC, so the
    // bidirectional solve dispatches to the pool inside parallel_invoke.
    constexpr int kCustomers = 1100;
    constexpr int N = kCustomers + 2;
    constexpr double kHorizon = 8.0;
    std::mt19937 rng(2026);
    std::uniform_real_distribution<double> cost_dist(-3.0, 6.0);
    std::vector<int> from;
    std::vector<int> to;
    std::vector<double> cost;
    auto add_arc = [&](int i, int j) {
        from.push_back(i);
        to.push_back(j);
        cost.push_back(cost_dist(rng));
    };
    for (int c = 0; c < kCustomers; ++c) {
        const int v = c + 1;
        add_arc(0, v);
        add_arc(v, N - 1);
        add_arc(v, (c + 1) % kCustomers + 1);
        add_arc(v, (c + 7) % kCustomers + 1);
    }
    const int n_arcs = static_cast<int>(from.size());
    std::vector<double> time_d(n_arcs, 1.0);
    std::vector<double> tw_lb(N, 0.0);
    std::vector<double> tw_ub(N, kHorizon);
    const double* arc_res[] = {time_d.data()};
    const double* v_lb[] = {tw_lb.data()};
    const double* v_ub[] = {tw_ub.data()};

    ProblemView pv;
    pv.n_vertices = N;
    pv.source = 0;
    pv.sink = N - 1;
    pv.n_arcs = n_arcs;
    pv.arc_from = from.data();
    pv.arc_to = to.data();
    pv.arc_base_cost = cost.data();
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    std::vector<std::vector<int>> neighbors(N);
    for (int c = 0; c < kCustomers; ++c) {
        for (int k = 0; k < 4; ++k) {
            neighbors[c + 1].push_back((c + k) % kCustomers + 1);
        }
    }
    const NgPathData ng_data(N, n_arcs, from.data(), to.data(), neighbors);
    using ViewPack = ResourcePack<NgPathView>;

    BucketGraph<ViewPack> seq(
        pv, ViewPack(NgPathView(ng_data)),
        {.bucket_steps = {100.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact});
    seq.build();
    auto sp = seq.solve();

    ThreadPool pool(4);
    std::atomic<int> in_invoke{0};
    std::atomic<int> nested{0};
    BucketGraph<ViewPack, NestingProbeExecutor> par(
        pv, ViewPack(NgPathView(ng_data)),
        {.bucket_steps = {100.0, 1.0}, .theta = 1e9, .bidirectional = true, .stage = Stage::Exact},
        NestingProbeExecutor{
            .inner = ThreadPoolExecutor(pool), .in_invoke = &in_invoke, .nested = &nested});
    par.build();
    auto pp = par.solve();

    CHECK(nested.load() > 0);  // the case under test was actually reached
    REQUIRE(!sp.empty());
    REQUIRE(!pp.empty());
    CHECK(sp[0].reduced_cost == doctest::Approx(pp[0].reduced_cost).epsilon(1e-6));
}
