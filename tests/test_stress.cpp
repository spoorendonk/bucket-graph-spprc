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
#include <set>
#include <stdexcept>
#include <type_traits>
#include <utility>
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
    // Symmetric mode is the depot convention (#126): depot 0, every arc with
    // its opposite.
    int from[] = {0, 1, 0, 2, 1, 2};
    int to[] = {1, 0, 2, 0, 2, 1};
    double cost[] = {5.0, 5.0, 3.0, 3.0, 4.0, 4.0};
    double time_d[] = {1.0, 1.0, 2.0, 2.0, 2.0, 2.0};
    double tw_lb[] = {0.0, 0.0, 0.0};
    double tw_ub[] = {20.0, 20.0, 20.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 3;
    pv.source = 0;
    pv.sink = 0;
    pv.n_arcs = 6;
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

// Every path the solve returns costs, under cost_of, what labelling gave it:
// exactly for a forward label (mono), up to rounding for a bidir join.
template <typename BG>
static void check_cost_of_paths(const BG& bg, const std::vector<typename BG::Path>& paths,
                                bool exact) {
    for (const auto& p : paths) {
        CAPTURE(p.arcs.size());
        auto c = bg.cost_of(p.arcs);
        REQUIRE(c.has_value());
        const auto w = c.value_or(typename BG::WalkCost{.reduced_cost = INF, .original_cost = INF});
        if (exact) {
            CHECK(w.reduced_cost == p.reduced_cost);
            CHECK(w.original_cost == p.original_cost);
        } else {
            CHECK(w.reduced_cost == doctest::Approx(p.reduced_cost).epsilon(1e-9));
            CHECK(w.original_cost == doctest::Approx(p.original_cost).epsilon(1e-9));
        }
    }
}

TEST_CASE("cost_of matches labelling on random instances with ng-paths") {
    using Pack = ResourcePack<NgPathResource>;
    int checked = 0;
    for (int n_main = 1; n_main <= 2; ++n_main) {
        for (int seed = 0; seed < 150; ++seed) {
            CAPTURE(n_main);
            CAPTURE(seed);
            RandomTwInstance inst(seed, n_main);
            const int n = inst.pv.n_vertices;
            std::vector<std::vector<int>> neighbors(n);
            for (int v = 0; v < n; ++v) {
                neighbors[v] = {v, (v + 1) % n, (v + 2) % n};
            }
            NgPathResource ng(n, inst.pv.n_arcs, inst.from_v.data(), inst.to_v.data(), neighbors);
            // Duals on the customers make cycles negative, which ng must cut.
            std::mt19937 rng(seed);
            std::uniform_real_distribution<double> U(0, 1);
            std::vector<double> dual(n, 0.0);
            for (int v = 1; v < n - 1; ++v) {
                dual[v] = U(rng) * 15;
            }
            std::vector<double> reduced(inst.cost_v);
            for (int a = 0; a < inst.pv.n_arcs; ++a) {
                reduced[a] -= dual[inst.to_v[a]];
            }
            for (bool bidir : {false, true}) {
                CAPTURE(bidir);
                BucketGraph<Pack> bg(inst.pv, Pack(ng),
                                     {.bucket_steps = {2.0, n_main == 2 ? 2.0 : 0.0},
                                      .theta = 1e9,
                                      .bidirectional = bidir});
                bg.build();
                bg.update_arc_costs(reduced.data());
                auto paths = bg.solve();
                check_cost_of_paths(bg, paths, !bidir);
                checked += static_cast<int>(paths.size());
            }
        }
    }
    CHECK(checked > 1000);
}

// The jump arcs of BG2021 §4.1 straight from the definition, as
// obtain_jump_arcs built them before #128: for each unfixed bucket b and each
// arc a at its vertex that b lacks, the component-wise minimal (fw) / maximal
// (bw) buckets b' ≻ b of the same vertex that hold a. Entries are
// (arc_id, jump_bucket, to_vertex), sorted per bucket.
using JumpKey = std::array<int, 3>;

template <typename BG>
static std::vector<std::vector<JumpKey>> reference_jump_arcs(const BG& bg, const ProblemView& pv,
                                                             bool fw) {
    const int nb = bg.n_buckets();
    std::vector<std::vector<JumpKey>> out(nb);
    // Buckets of a vertex are contiguous and row-major in (k0, k1).
    std::vector<int> start(pv.n_vertices, -1);
    std::vector<int> count(pv.n_vertices, 0);
    std::vector<int> n1(pv.n_vertices, 0);
    for (int bi = 0; bi < nb; ++bi) {
        const auto& b = bg.bucket(bi);
        if (start[b.vertex] < 0) {
            start[b.vertex] = bi;
        }
        ++count[b.vertex];
        if (b.lb[0] == bg.bucket(start[b.vertex]).lb[0]) {
            ++n1[b.vertex];
        }
    }
    auto holds = [&](int bi, int a) {
        const auto& arcs = fw ? bg.bucket(bi).bucket_arcs : bg.bucket(bi).bw_bucket_arcs;
        return std::ranges::contains(arcs, a, &BucketArc::arc_id);
    };
    for (int bi = 0; bi < nb; ++bi) {
        const int v = bg.bucket(bi).vertex;
        if (bg.is_bucket_fixed(bi) || count[v] <= 1) {
            continue;
        }
        const int k0 = (bi - start[v]) / n1[v];
        const int k1 = (bi - start[v]) % n1[v];
        for (int a = 0; a < pv.n_arcs; ++a) {
            if ((fw ? pv.arc_from[a] : pv.arc_to[a]) != v || holds(bi, a)) {
                continue;
            }
            std::vector<std::array<int, 2>> succ;
            std::vector<int> succ_bi;
            for (int b2 = start[v]; b2 < start[v] + count[v]; ++b2) {
                const int j0 = (b2 - start[v]) / n1[v];
                const int j1 = (b2 - start[v]) % n1[v];
                const bool after = fw ? (j0 >= k0 && j1 >= k1 && (j0 > k0 || j1 > k1))
                                      : (j0 <= k0 && j1 <= k1 && (j0 < k0 || j1 < k1));
                if (after && holds(b2, a)) {
                    succ.push_back({j0, j1});
                    succ_bi.push_back(b2);
                }
            }
            for (size_t i = 0; i < succ.size(); ++i) {
                bool dominated = false;
                for (size_t j = 0; j < succ.size() && !dominated; ++j) {
                    dominated =
                        j != i && (fw ? (succ[j][0] <= succ[i][0] && succ[j][1] <= succ[i][1])
                                      : (succ[j][0] >= succ[i][0] && succ[j][1] >= succ[i][1]));
                }
                if (!dominated) {
                    out[bi].push_back({a, succ_bi[i], fw ? pv.arc_to[a] : pv.arc_from[a]});
                }
            }
        }
        std::ranges::sort(out[bi]);
    }
    return out;
}

// Jump arcs checked, and the (bucket, arc) pairs whose front has two or more.
struct JumpCount {
    int arcs = 0;
    int multi = 0;
};

template <typename BG>
static JumpCount check_jump_arcs(const BG& bg, const ProblemView& pv, bool fw) {
    const auto want = reference_jump_arcs(bg, pv, fw);
    JumpCount n;
    for (int bi = 0; bi < bg.n_buckets(); ++bi) {
        CAPTURE(bi);
        std::vector<JumpKey> got;
        for (const auto& ja : fw ? bg.bucket(bi).jump_arcs : bg.bucket(bi).bw_jump_arcs) {
            got.push_back({ja.arc_id, ja.jump_bucket, ja.to_vertex});
            CHECK(ja.real_cost == pv.arc_base_cost[ja.arc_id]);
            CHECK(ja.resource[0] == pv.arc_resource[0][ja.arc_id]);
        }
        std::ranges::sort(got);
        CHECK(got == want[bi]);
        n.arcs += static_cast<int>(got.size());
        for (size_t i = 1; i < got.size(); ++i) {
            // got is sorted, so a run of one arc id is one front.
            n.multi +=
                (got[i][0] == got[i - 1][0] && (i == 1 || got[i - 2][0] != got[i][0])) ? 1 : 0;
        }
    }
    return n;
}

TEST_CASE("jump arcs after elimination match their definition (#128)") {
    int jumps = 0;
    // Fronts of two or more buckets exist only in 2D: count them per direction.
    std::array<int, 2> multi_2d{};
    for (int n_main = 1; n_main <= 2; ++n_main) {
        for (int seed = 0; seed < 60; ++seed) {
            CAPTURE(n_main);
            CAPTURE(seed);
            RandomTwInstance inst(seed, n_main);
            const int n = inst.pv.n_vertices;
            std::mt19937 rng(seed);
            std::uniform_real_distribution<double> U(0, 1);
            std::vector<double> dual(n, 0.0);
            for (int v = 1; v < n - 1; ++v) {
                dual[v] = U(rng) * 15;
            }
            std::vector<double> reduced(inst.cost_v);
            for (int a = 0; a < inst.pv.n_arcs; ++a) {
                reduced[a] -= dual[inst.to_v[a]];
            }
            for (int mode = 0; mode < 3; ++mode) {
                CAPTURE(mode);
                const bool bidir = mode > 0;
                BucketGraph<ResourcePack<>> bg(inst.pv, ResourcePack<>{},
                                               {.bucket_steps = {1.5, n_main == 2 ? 1.0 : 0.0},
                                                .theta = 1e9,
                                                .bidirectional = bidir,
                                                .stage = Stage::Exact});
                bg.build();
                bg.update_arc_costs(reduced.data());
                auto paths = bg.solve();
                if (paths.empty()) {
                    continue;
                }
                const double gap = 2.0 + U(rng) * 6.0;
                bg.fix_buckets(gap);
                if (mode == 2) {
                    bg.eliminate_arcs_label_based(gap);
                } else {
                    bg.eliminate_arcs(gap);
                }
                for (bool fw : {true, false}) {
                    if (!fw && !bidir) {
                        continue;
                    }
                    const auto n = check_jump_arcs(bg, inst.pv, fw);
                    jumps += n.arcs;
                    if (n_main == 2) {
                        multi_2d[fw ? 0 : 1] += n.multi;
                    }
                }
            }
        }
    }
    CHECK(jumps > 1000);
    CHECK(multi_2d[0] > 0);
    CHECK(multi_2d[1] > 0);
}

TEST_CASE("cost_of matches labelling with an R1C dual, through Solver") {
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
    // A 3-row subset cut over {1, 3, 4}, remembered on every arc: a path
    // through two of them pays beta = 5.
    R1Cut cut;
    cut.base_set = {1, 3, 4};
    cut.multipliers = {0.5, 0.5, 0.5};
    for (int a = 0; a < 8; ++a) {
        cut.memory_arcs.emplace_back(a, 0);
    }
    cut.dual_value = -5.0;

    for (bool bidir : {false, true}) {
        CAPTURE(bidir);
        Solver<R1CPack> solver(pv, R1CPack{R1CResource{}},
                               {.bucket_steps = {5.0, 0.0}, .bidirectional = bidir, .theta = 1e9});
        solver.resource<R1CResource>().set_cuts({{cut}}, N, 8);
        solver.build();
        solver.set_stage(Stage::Exact);
        auto paths = solver.solve();
        REQUIRE(!paths.empty());
        bool saw_cut = false;
        for (const auto& p : paths) {
            auto c = solver.cost_of(p.arcs);
            REQUIRE(c.has_value());
            const auto w =
                c.value_or(Solver<R1CPack>::WalkCost{.reduced_cost = INF, .original_cost = INF});
            CHECK(w.reduced_cost == doctest::Approx(p.reduced_cost).epsilon(1e-9));
            CHECK(w.original_cost == doctest::Approx(p.original_cost).epsilon(1e-9));
            saw_cut = saw_cut || w.reduced_cost > w.original_cost + 1.0;
        }
        CHECK(saw_cut);
        // 0 -> 1 -> 3 -> 5 visits 1 and 3.
        auto c = solver.cost_of(std::vector<int>{0, 2, 4});
        REQUIRE(c.has_value());
        const auto w =
            c.value_or(Solver<R1CPack>::WalkCost{.reduced_cost = INF, .original_cost = INF});
        CHECK(w.reduced_cost == doctest::Approx(16.0));
        CHECK(w.original_cost == doctest::Approx(11.0));
    }
}

// ════════════════════════════════════════════════════════════════════
// Borrowed adjacency (#133)
// ════════════════════════════════════════════════════════════════════

template <typename Path>
static void check_same_paths(const std::vector<Path>& a, const std::vector<Path>& b) {
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CAPTURE(i);
        CHECK(a[i].arcs == b[i].arcs);
        CHECK(a[i].vertices == b[i].vertices);
        CHECK(a[i].reduced_cost == b[i].reduced_cost);
        CHECK(a[i].original_cost == b[i].original_cost);
    }
}

// A temporary CsrAdjacency would dangle once the constructor returns, so it
// does not convert; an lvalue one and a plain view do.
static_assert(!std::is_constructible_v<BucketGraph<ResourcePack<>>, const ProblemView&,
                                       CsrAdjacency, ResourcePack<>>);
static_assert(std::is_constructible_v<BucketGraph<ResourcePack<>>, const ProblemView&,
                                      const CsrAdjacency&, ResourcePack<>>);
static_assert(std::is_constructible_v<Solver<ResourcePack<>>, const ProblemView&, AdjacencyView,
                                      ResourcePack<>>);
static_assert(!std::is_constructible_v<Solver<ResourcePack<>>, const ProblemView&, CsrAdjacency,
                                       ResourcePack<>>);

/// The CSR of `pv`'s arcs with each vertex's arcs in reverse arc-index order,
/// as a caller's own CSR might list them.
struct ReversedCsr {
    std::vector<int> out_begin, out_arcs, in_begin, in_arcs;

    explicit ReversedCsr(const ProblemView& pv) {
        CsrAdjacency csr;
        csr.build(pv);
        for (int v = 0; v < pv.n_vertices; ++v) {
            out_begin.push_back(static_cast<int>(out_arcs.size()));
            in_begin.push_back(static_cast<int>(in_arcs.size()));
            const auto o = csr.outgoing(v);
            const auto i = csr.incoming(v);
            out_arcs.insert(out_arcs.end(), o.rbegin(), o.rend());
            in_arcs.insert(in_arcs.end(), i.rbegin(), i.rend());
        }
        out_begin.push_back(pv.n_arcs);
        in_begin.push_back(pv.n_arcs);
    }

    [[nodiscard]] AdjacencyView view() const {
        return {.out_begin = out_begin.data(),
                .out_arcs = out_arcs.data(),
                .in_begin = in_begin.data(),
                .in_arcs = in_arcs.data(),
                .n_vertices = static_cast<int>(out_begin.size()) - 1,
                .n_arcs = static_cast<int>(out_arcs.size())};
    }
};

TEST_CASE("a borrowed adjacency gives the same paths as an owned one (#133)") {
    using Pack = ResourcePack<NgPathResource>;
    using BG = BucketGraph<Pack>;
    int checked = 0;
    for (int n_main = 1; n_main <= 2; ++n_main) {
        for (int seed = 0; seed < 40; ++seed) {
            CAPTURE(n_main);
            CAPTURE(seed);
            RandomTwInstance inst(seed, n_main);
            const int n = inst.pv.n_vertices;
            std::vector<std::vector<int>> neighbors(n);
            for (int v = 0; v < n; ++v) {
                neighbors[v] = {v, (v + 1) % n, (v + 2) % n};
            }
            NgPathResource ng(n, inst.pv.n_arcs, inst.from_v.data(), inst.to_v.data(), neighbors);
            std::mt19937 rng(seed);
            std::uniform_real_distribution<double> U(0, 1);
            std::vector<double> reduced(inst.cost_v);
            for (int a = 0; a < inst.pv.n_arcs; ++a) {
                reduced[a] -= inst.to_v[a] < n - 1 ? U(rng) * 15 : 0.0;
            }
            CsrAdjacency csr;
            csr.build(inst.pv);
            const ReversedCsr reversed(inst.pv);
            for (bool bidir : {false, true}) {
                CAPTURE(bidir);
                const BG::Options opts{.bucket_steps = {2.0, n_main == 2 ? 2.0 : 0.0},
                                       .theta = 1e9,
                                       .bidirectional = bidir};
                BG owned(inst.pv, Pack(ng), opts);
                BG borrowed(inst.pv, csr, Pack(ng), opts);
                BG reordered(inst.pv, reversed.view(), Pack(ng), opts);
                CHECK(owned.owns_adjacency());
                CHECK_FALSE(borrowed.owns_adjacency());
                CHECK_FALSE(reordered.owns_adjacency());
                std::vector<std::vector<BG::Path>> paths;
                for (BG* bg : {&owned, &borrowed, &reordered}) {
                    bg->build();
                    bg->update_arc_costs(reduced.data());
                    paths.push_back(bg->solve());
                }
                // The same CSR order labels identically; another order may
                // break ties apart but finds the same optimum.
                check_same_paths(paths[0], paths[1]);
                CHECK(best_reduced_cost(paths[2]) ==
                      doctest::Approx(best_reduced_cost(paths[0])).epsilon(1e-9));
                CHECK(paths[2].empty() == paths[0].empty());
                checked += static_cast<int>(paths[0].size());
            }
        }
    }
    CHECK(checked > 100);
}

TEST_CASE("Solver with a borrowed adjacency matches the owning one (#133)") {
    using Pack = ResourcePack<>;
    using S = Solver<Pack>;
    for (int seed = 0; seed < 20; ++seed) {
        CAPTURE(seed);
        RandomTwInstance inst(seed, 1);
        std::vector<double> reduced(inst.cost_v);
        for (int a = 0; a < inst.pv.n_arcs; ++a) {
            reduced[a] -= 6.0;
        }
        CsrAdjacency csr;
        csr.build(inst.pv);
        const S::Options opts{.bucket_steps = {2.0, 0.0}, .theta = 1e9};
        S owned(inst.pv, Pack{}, opts);
        S borrowed(inst.pv, csr.view(), Pack{}, opts);
        CHECK(owned.owns_adjacency());
        CHECK_FALSE(borrowed.owns_adjacency());
        for (S* s : {&owned, &borrowed}) {
            s->set_stage(Stage::Exact);
            s->build();
            s->update_arc_costs(reduced);
        }
        check_same_paths(owned.solve(), borrowed.solve());
    }
}

TEST_CASE("a borrowed adjacency that does not match the view is refused (#133)") {
    using Pack = ResourcePack<>;
    RandomTwInstance inst(3, 1);
    CsrAdjacency csr;
    csr.build(inst.pv);

    AdjacencyView fewer_arcs = csr.view();
    --fewer_arcs.n_arcs;
    CHECK_THROWS_AS(BucketGraph<Pack>(inst.pv, fewer_arcs, Pack{}), std::invalid_argument);

    AdjacencyView fewer_vertices = csr.view();
    --fewer_vertices.n_vertices;
    CHECK_THROWS_AS(BucketGraph<Pack>(inst.pv, fewer_vertices, Pack{}), std::invalid_argument);

    AdjacencyView no_arrays = csr.view();
    no_arrays.in_arcs = nullptr;
    CHECK_THROWS_AS(BucketGraph<Pack>(inst.pv, no_arrays, Pack{}), std::invalid_argument);
    CHECK_THROWS_AS(Solver<Pack>(inst.pv, AdjacencyView{}, Pack{}), std::invalid_argument);
}

TEST_CASE("cost_of refuses walks labelling would not produce") {
    // 0 -> 1 -> 2 (sink), and 0 -> 2 directly; vertex 1 closes at 1.
    int from[] = {0, 1, 0, 1};
    int to[] = {1, 2, 2, 1};
    double cost[] = {1.0, 2.0, 5.0, 0.5};
    double time_d[] = {1.0, 1.0, 1.0, 1.0};
    double tw_lb[] = {0.0, 0.0, 0.0};
    double tw_ub[] = {10.0, 1.0, 10.0};
    const double* arc_res[] = {time_d};
    const double* v_lb[] = {tw_lb};
    const double* v_ub[] = {tw_ub};

    ProblemView pv;
    pv.n_vertices = 3;
    pv.source = 0;
    pv.sink = 2;
    pv.n_arcs = 4;
    pv.arc_from = from;
    pv.arc_to = to;
    pv.arc_base_cost = cost;
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.bucket_steps = {1.0, 0.0}});
    bg.build();

    auto ok = bg.cost_of(std::vector<int>{0, 1});
    REQUIRE(ok.has_value());
    const auto w =
        ok.value_or(BucketGraph<EmptyPack>::WalkCost{.reduced_cost = INF, .original_cost = INF});
    CHECK(w.reduced_cost == 3.0);
    CHECK(w.original_cost == 3.0);

    CHECK_FALSE(bg.cost_of(std::vector<int>{}).has_value());         // never leaves the source
    CHECK_FALSE(bg.cost_of(std::vector<int>{0}).has_value());        // ends short of the sink
    CHECK_FALSE(bg.cost_of(std::vector<int>{1, 0}).has_value());     // not a walk from the source
    CHECK_FALSE(bg.cost_of(std::vector<int>{0, 3, 1}).has_value());  // closes vertex 1's window
    CHECK_FALSE(bg.cost_of(std::vector<int>{0, 4}).has_value());     // arc id out of range
    CHECK_FALSE(bg.cost_of(std::vector<int>{-1}).has_value());

    // Reduced costs replace the base ones; an INF one forbids its arc.
    double reduced[] = {-4.0, 1.0, INF, 0.0};
    bg.update_arc_costs(reduced);
    auto rc = bg.cost_of(std::vector<int>{0, 1});
    REQUIRE(rc.has_value());
    const auto wr =
        rc.value_or(BucketGraph<EmptyPack>::WalkCost{.reduced_cost = INF, .original_cost = INF});
    CHECK(wr.reduced_cost == -3.0);
    CHECK(wr.original_cost == 3.0);
    CHECK_FALSE(bg.cost_of(std::vector<int>{2}).has_value());
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

TEST_CASE("Bidir Enumerate returns the mono paths on random instances (#117)") {
    // Enumerate keeps every label below theta, so mono and bidir must return
    // the same paths. Bidir lost some on over half of these instances:
    // Enumerate stored a bucket's labels unsorted, and the concatenation
    // stopped at the first one over theta.
    auto paths = [&]<typename Exec>(Exec, const ProblemView& pv, int n_main, double theta,
                                    bool bidir) {
        BucketGraph<EmptyPack, Exec> bg(pv, EmptyPack{},
                                        {.bucket_steps = {2.0, n_main == 2 ? 2.0 : 0.0},
                                         .theta = theta,
                                         .bidirectional = bidir,
                                         .stage = Stage::Enumerate});
        bg.build();
        std::vector<std::vector<int>> v;
        for (const auto& p : bg.solve()) {
            v.push_back(p.vertices);
        }
        CHECK(bg.enumeration_complete());
        std::ranges::sort(v);
        return v;
    };
    for (double theta : {-1e-6, 5.0}) {
        for (int n_main = 1; n_main <= 2; ++n_main) {
            for (int seed = 0; seed < 200; ++seed) {
                CAPTURE(theta);
                CAPTURE(n_main);
                CAPTURE(seed);
                RandomTwInstance inst(seed, n_main);
                const auto mono = paths(SequentialExecutor{}, inst.pv, n_main, theta, false);
                CHECK(paths(SequentialExecutor{}, inst.pv, n_main, theta, true) == mono);
                CHECK(paths(StdThreadExecutor{}, inst.pv, n_main, theta, true) == mono);
            }
        }
    }
}

TEST_CASE("Elimination keeps the paths that need a jump arc (#115)") {
    // After elimination a bucket can reach an arc only by a jump arc to a
    // later bucket of its vertex. The completion bounds took the min over
    // those later buckets only after the SCC pass, so the buckets before it
    // never saw them, and the next elimination (or Enumerate's pruning) cut
    // improving paths: on seed 104, Enumerate after three rounds lost 4 of
    // its 6 paths (e.g. 0 3 5 3 5 2 4 7).
    auto enumerate = [](BucketGraph<EmptyPack>& bg) {
        bg.set_stage(Stage::Enumerate);
        std::vector<std::vector<int>> v;
        for (const auto& p : bg.solve()) {
            v.push_back(p.vertices);
        }
        CHECK(bg.enumeration_complete());
        std::ranges::sort(v);
        return v;
    };
    for (bool bidir : {false, true}) {
        for (int seed = 0; seed < 200; ++seed) {
            CAPTURE(bidir);
            CAPTURE(seed);
            RandomTwInstance inst(seed, 1);
            const BucketGraph<EmptyPack>::Options opts{.bucket_steps = {2.0, 0.0},
                                                       .bidirectional = bidir};
            BucketGraph<EmptyPack> fresh(inst.pv, EmptyPack{}, opts);
            fresh.build();
            const auto all = enumerate(fresh);

            BucketGraph<EmptyPack> bg(inst.pv, EmptyPack{}, opts);
            bg.build();
            for (int k = 0; k < 3; ++k) {
                bg.solve();
                CHECK(bg.exact_complete());
                bg.eliminate_arcs(-1e-6);
                bg.fix_buckets(-1e-6);
            }
            CHECK(enumerate(bg) == all);
        }
    }
}

TEST_CASE("Completion bounds stay lower bounds in a negative bucket cycle (#115)") {
    // 0 -> 1 <-> 2 -> 3, windows [0, 10], one bucket per vertex: 1 and 2 form
    // one SCC whose cycle 1 2 1 costs -2 and takes 0.2, so the best path
    // (-79) takes 89 cycle arcs. |SCC| + 1 relaxation passes left completion
    // bounds far above that instead of unbounded below: parallel bidir
    // pruned every label by them, and elimination cut the path's arcs.
    int from[] = {0, 1, 2, 2};
    int to[] = {1, 2, 1, 3};
    double cost[] = {10.0, -1.0, -1.0, 0.0};
    double time_d[] = {1.0, 0.1, 0.1, 0.0};
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

    auto run = [&]<typename Exec>(Exec, bool bidir) {
        BucketGraph<EmptyPack, Exec> bg(pv, EmptyPack{},
                                        {.bucket_steps = {20.0, 0.0}, .bidirectional = bidir});
        bg.build();
        for (int k = 0; k < 3; ++k) {
            CAPTURE(k);
            auto paths = bg.solve();
            CHECK(bg.exact_complete());
            CHECK(best_reduced_cost(paths) == doctest::Approx(-79.0));
            bg.eliminate_arcs(-1e-6);
            bg.fix_buckets(-1e-6);
        }
    };
    for (bool bidir : {false, true}) {
        CAPTURE(bidir);
        run(SequentialExecutor{}, bidir);
        run(StdThreadExecutor{}, bidir);
    }
}

namespace {

/// The #115 cycle 0 -> 1 <-> 2 -> 3 plus an arc 0 -> 2 (arc 4) costing 1000,
/// with the arcs' consumption of up to two main resources given.
struct LapCycleInstance {
    int from[5] = {0, 1, 2, 2, 0};
    int to[5] = {1, 2, 1, 3, 2};
    double cost[5] = {10.0, -1.0, -1.0, 0.0, 1000.0};
    std::array<std::array<double, 5>, 2> d{};
    std::array<std::array<double, 4>, 2> lb{};
    std::array<std::array<double, 4>, 2> ub{};
    std::array<const double*, 2> arc_res{};
    std::array<const double*, 2> v_lb{};
    std::array<const double*, 2> v_ub{};
    ProblemView pv;

    LapCycleInstance(const LapCycleInstance&) = delete;  // pv points into this
    LapCycleInstance& operator=(const LapCycleInstance&) = delete;
    LapCycleInstance(std::array<double, 5> d0, std::array<double, 5> d1, int n_main, double w)
        : d{d0, d1} {
        for (int r = 0; r < 2; ++r) {
            ub[r].fill(w);
            arc_res[r] = d[r].data();
            v_lb[r] = lb[r].data();
            v_ub[r] = ub[r].data();
        }
        pv.n_vertices = 4;
        pv.source = 0;
        pv.sink = 3;
        pv.n_arcs = 5;
        pv.arc_from = from;
        pv.arc_to = to;
        pv.arc_base_cost = cost;
        pv.n_resources = n_main;
        pv.arc_resource = arc_res.data();
        pv.vertex_lb = v_lb.data();
        pv.vertex_ub = v_ub.data();
        pv.n_main_resources = n_main;
    }
};

/// True when some bucket of the source still has arc 4 (0 -> 2).
template <typename BG>
bool source_keeps_arc4(const BG& bg) {
    for (int bi = 0; bi < bg.n_buckets(); ++bi) {
        const auto& b = bg.bucket(bi);
        if (b.vertex == 0 && std::ranges::contains(b.bucket_arcs, 4, &BucketArc::arc_id)) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST_CASE("A negative bucket cycle gets a finite lap bound that still eliminates (#121)") {
    // Any path over arc 4 costs more than 1000 minus what the cycle can
    // give back in the windows, so a finite bound eliminates the arc from
    // every source bucket, where NEG_INF eliminated it from none. Finer
    // steps let labels jump out of the cycle's SCCs, and in 2D the second
    // resource bounds the laps more tightly than the first.
    struct Case {
        int n_main;
        std::array<double, 5> d1;
        double best;
    };
    const Case cases[] = {
        // 1D: 45 laps of cost -2 fit time 10.
        {.n_main = 1, .d1 = {}, .best = -79.0},
        // 2D: a load of 0.5 per cycle arc fits 19 of them in 10.
        {.n_main = 2, .d1 = {0.0, 0.5, 0.5, 0.0, 0.0}, .best = -9.0},
    };
    for (const auto& c : cases) {
        const LapCycleInstance inst({1.0, 0.1, 0.1, 0.0, 1.0}, c.d1, c.n_main, 10.0);
        // Elimination and fixing must keep every path Enumerate finds below
        // theta, not only the best: a bound too high cuts the paths with
        // fewer laps first, and the tight theta leaves it no slack.
        auto run = [&]<typename Exec>(Exec, bool bidir, double step, double theta) {
            const typename BucketGraph<EmptyPack, Exec>::Options opts{
                .bucket_steps = {step, c.n_main == 2 ? step : 0.0},
                .theta = theta,
                .bidirectional = bidir};
            auto enumerate = [](BucketGraph<EmptyPack, Exec>& bg) {
                bg.set_stage(Stage::Enumerate);
                std::vector<std::vector<int>> v;
                for (const auto& p : bg.solve()) {
                    v.push_back(p.vertices);
                }
                CHECK(bg.enumeration_complete());
                std::ranges::sort(v);
                return v;
            };
            BucketGraph<EmptyPack, Exec> fresh(inst.pv, EmptyPack{}, opts);
            fresh.build();
            const auto all = enumerate(fresh);
            CHECK(!all.empty());

            BucketGraph<EmptyPack, Exec> bg(inst.pv, EmptyPack{}, opts);
            bg.build();
            for (int k = 0; k < 3; ++k) {
                CAPTURE(k);
                auto paths = bg.solve();
                CHECK(bg.exact_complete());
                CHECK(best_reduced_cost(paths) == doctest::Approx(c.best));
                bg.eliminate_arcs(theta);
                bg.fix_buckets(theta);
            }
            CHECK_FALSE(source_keeps_arc4(bg));
            CHECK(enumerate(bg) == all);
        };
        for (double step : {20.0, 1.0, 0.3}) {
            for (bool bidir : {false, true}) {
                for (double theta : {-1e-6, c.best + 2.5}) {
                    CAPTURE(c.n_main);
                    CAPTURE(step);
                    CAPTURE(bidir);
                    CAPTURE(theta);
                    run(SequentialExecutor{}, bidir, step, theta);
                    run(StdThreadExecutor{}, bidir, step, theta);
                }
            }
        }
    }
}

TEST_CASE("A negative bucket cycle no resource bounds keeps NEG_INF (#121)") {
    // Arc 1 -> 2 takes no time at cost -1, so time gives no lap bound for
    // the cycle and its SCC stays NEG_INF: mono elimination keeps arc 4
    // (only bidir's labels can rule it out). The cycle still laps 90 times
    // in the window, through 2 -> 1: best 10 - 1 - 2 * 90.
    const LapCycleInstance inst({1.0, 0.0, 0.1, 0.0, 1.0}, {}, 1, 10.05);
    auto run = [&]<typename Exec>(Exec, bool bidir, double step) {
        BucketGraph<EmptyPack, Exec> bg(inst.pv, EmptyPack{},
                                        {.bucket_steps = {step, 0.0}, .bidirectional = bidir});
        bg.build();
        for (int k = 0; k < 3; ++k) {
            CAPTURE(k);
            auto paths = bg.solve();
            CHECK(bg.exact_complete());
            CHECK(best_reduced_cost(paths) == doctest::Approx(-171.0));
            bg.eliminate_arcs(-1e-6);
            bg.fix_buckets(-1e-6);
        }
        if (!bidir) {
            CHECK(source_keeps_arc4(bg));
        }
    };
    for (double step : {20.0, 1.0}) {
        for (bool bidir : {false, true}) {
            CAPTURE(step);
            CAPTURE(bidir);
            run(SequentialExecutor{}, bidir, step);
            run(StdThreadExecutor{}, bidir, step);
        }
    }
}

TEST_CASE("A long negative ring stops Bellman-Ford early and keeps the lap bound (#129)") {
    // Source 0 -> ring 1 -> 2 -> ... -> kRing -> 1, exit 1 -> sink, and an
    // arc 0 -> 2 costing 1000 to eliminate. Every ring arc costs -1 and
    // takes 0.1 of time 10, so the ring's buckets form one SCC holding a
    // negative cycle, which the predecessor graph shows long before
    // |SCC| + 1 passes. The lap bound it then gets must still eliminate the
    // costly arc and keep every path below theta: 11 laps fit after 0 -> 1.
    constexpr int kRing = 8;
    constexpr int kSink = kRing + 1;
    std::vector<int> from{0, 0, 1};
    std::vector<int> to{1, 2, kSink};
    std::vector<double> cost{10.0, 1000.0, 0.0};
    std::vector<double> time{1.0, 1.0, 0.0};
    for (int v = 1; v <= kRing; ++v) {
        from.push_back(v);
        to.push_back(v == kRing ? 1 : v + 1);
        cost.push_back(-1.0);
        time.push_back(0.1);
    }
    const std::vector<double> lb(kSink + 1, 0.0);
    const std::vector<double> ub(kSink + 1, 10.0);
    const double* arc_res[] = {time.data()};
    const double* v_lb[] = {lb.data()};
    const double* v_ub[] = {ub.data()};
    ProblemView pv;
    pv.n_vertices = kSink + 1;
    pv.source = 0;
    pv.sink = kSink;
    pv.n_arcs = static_cast<int>(from.size());
    pv.arc_from = from.data();
    pv.arc_to = to.data();
    pv.arc_base_cost = cost.data();
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    auto run = [&]<typename Exec>(Exec, bool bidir, double step, double theta) {
        const typename BucketGraph<EmptyPack, Exec>::Options opts{
            .bucket_steps = {step, 0.0}, .theta = theta, .bidirectional = bidir};
        auto enumerate = [](BucketGraph<EmptyPack, Exec>& bg) {
            bg.set_stage(Stage::Enumerate);
            std::vector<std::vector<int>> v;
            for (const auto& p : bg.solve()) {
                v.push_back(p.vertices);
            }
            CHECK(bg.enumeration_complete());
            std::ranges::sort(v);
            return v;
        };
        BucketGraph<EmptyPack, Exec> fresh(pv, EmptyPack{}, opts);
        fresh.build();
        const auto all = enumerate(fresh);
        CHECK(!all.empty());

        BucketGraph<EmptyPack, Exec> bg(pv, EmptyPack{}, opts);
        bg.build();
        for (int k = 0; k < 3; ++k) {
            CAPTURE(k);
            auto paths = bg.solve();
            CHECK(bg.exact_complete());
            CHECK(best_reduced_cost(paths) == doctest::Approx(10.0 - (kRing * 11.0)));
            bg.eliminate_arcs(theta);
            bg.fix_buckets(theta);
        }
        bool keeps_arc1 = false;
        for (int bi = 0; bi < bg.n_buckets(); ++bi) {
            const auto& b = bg.bucket(bi);
            keeps_arc1 = keeps_arc1 || (b.vertex == 0 && std::ranges::contains(b.bucket_arcs, 1,
                                                                               &BucketArc::arc_id));
        }
        CHECK_FALSE(keeps_arc1);
        CHECK(enumerate(bg) == all);
    };
    for (double step : {20.0, 1.0}) {
        for (bool bidir : {false, true}) {
            for (double theta : {-1e-6, -69.0}) {
                CAPTURE(step);
                CAPTURE(bidir);
                CAPTURE(theta);
                run(SequentialExecutor{}, bidir, step, theta);
                run(StdThreadExecutor{}, bidir, step, theta);
            }
        }
    }
}

TEST_CASE(
    "exact_complete is false and label_cap_hit true after a label cap, and elimination waits "
    "(#115, #96)") {
    RandomTwInstance inst(104, 1);
    for (bool bidir : {false, true}) {
        CAPTURE(bidir);
        auto make = [&](int cap) {
            return BucketGraph<EmptyPack>(
                inst.pv, EmptyPack{},
                {.bucket_steps = {2.0, 0.0}, .bidirectional = bidir, .max_scc_labels = cap});
        };
        auto bg = make(500000);
        bg.build();
        CHECK_FALSE(bg.exact_complete());  // nothing solved yet
        bg.solve();
        CHECK(bg.exact_complete());
        CHECK_FALSE(bg.label_cap_hit());

        auto capped = make(1);
        capped.build();
        capped.solve();
        CHECK_FALSE(capped.exact_complete());
        CHECK(capped.label_cap_hit());  // #96
        capped.eliminate_arcs(-1e9);
        CHECK(capped.eliminated_bucket_arcs() == 0);
        CHECK(capped.fix_buckets(-1e9) == 0);

        // A later uncapped solve is complete again; a heuristic one is not,
        // and elimination and fixing wait for an exact one.
        bg.set_stage(Stage::Heuristic1);
        bg.solve();
        CHECK_FALSE(bg.exact_complete());
        CHECK_FALSE(bg.label_cap_hit());
        bg.eliminate_arcs(-1e9);
        CHECK(bg.eliminated_bucket_arcs() == 0);
        CHECK(bg.fix_buckets(-1e9) == 0);
        bg.set_stage(Stage::Exact);
        bg.solve();
        CHECK(bg.exact_complete());

        // The cap cuts a heuristic solve too, and nothing is set before a solve (#96).
        capped.build();
        CHECK_FALSE(capped.label_cap_hit());
        capped.set_stage(Stage::Heuristic1);
        capped.solve();
        CHECK(capped.label_cap_hit());
        auto fresh = make(500000);
        fresh.build();
        CHECK_FALSE(fresh.label_cap_hit());

        // Enumeration's own caps clear enumeration_complete(), not the SCC flag.
        auto enum_capped = BucketGraph<EmptyPack>(inst.pv, EmptyPack{},
                                                  {.bucket_steps = {2.0, 0.0},
                                                   .theta = 1e9,
                                                   .bidirectional = bidir,
                                                   .stage = Stage::Enumerate,
                                                   .max_enum_labels = 1});
        enum_capped.build();
        enum_capped.solve();
        CHECK_FALSE(enum_capped.enumeration_complete());
        CHECK_FALSE(enum_capped.exact_complete());
        CHECK_FALSE(enum_capped.label_cap_hit());
    }

    Solver<EmptyPack> solver(inst.pv, EmptyPack{},
                             {.bucket_steps = {2.0, 0.0}, .max_scc_labels = 1});
    solver.set_stage(Stage::Exact);
    solver.build();
    solver.solve();
    CHECK_FALSE(solver.exact_complete());
    CHECK(solver.label_cap_hit());

    Solver<EmptyPack> uncapped(inst.pv, EmptyPack{}, {.bucket_steps = {2.0, 0.0}});
    uncapped.set_stage(Stage::Exact);
    uncapped.build();
    uncapped.solve();
    CHECK(uncapped.exact_complete());
    CHECK_FALSE(uncapped.label_cap_hit());

    // enumerate() reports its own labelling, and a rebuild clears the flag.
    solver.enumerate(0.0);
    CHECK(solver.enumeration_complete());
    CHECK_FALSE(solver.label_cap_hit());
    solver.solve();
    CHECK(solver.label_cap_hit());
    solver.build();
    CHECK_FALSE(solver.label_cap_hit());

    // Parallel bidir: the two directions share the flag.
    ThreadPool pool(4);
    BucketGraph<EmptyPack, ThreadPoolExecutor> par(
        inst.pv, EmptyPack{},
        {.bucket_steps = {2.0, 0.0}, .bidirectional = true, .max_scc_labels = 1},
        ThreadPoolExecutor(pool));
    par.build();
    par.solve();
    CHECK(par.label_cap_hit());
    CHECK_FALSE(par.exact_complete());
    par.build();
    CHECK_FALSE(par.label_cap_hit());
}

TEST_CASE("A label cap reached only by inserts downstream leaves the solve exact (#115)") {
    // Star 0 -> {1..5} -> 6: the source's SCC is one bucket whose one label
    // inserts 5 labels at other vertices. That reaches a cap of 3, but the
    // SCC has nothing left to extend, so nothing was cut off.
    std::vector<int> from;
    std::vector<int> to;
    for (int v = 1; v <= 5; ++v) {
        from.push_back(0);
        to.push_back(v);
        from.push_back(v);
        to.push_back(6);
    }
    std::vector<double> cost(from.size(), -1.0);
    std::vector<double> time_d(from.size(), 1.0);
    std::vector<double> tw_lb(7, 0.0);
    std::vector<double> tw_ub(7, 10.0);
    const double* arc_res[] = {time_d.data()};
    const double* v_lb[] = {tw_lb.data()};
    const double* v_ub[] = {tw_ub.data()};

    ProblemView pv;
    pv.n_vertices = 7;
    pv.source = 0;
    pv.sink = 6;
    pv.n_arcs = static_cast<int>(from.size());
    pv.arc_from = from.data();
    pv.arc_to = to.data();
    pv.arc_base_cost = cost.data();
    pv.n_resources = 1;
    pv.arc_resource = arc_res;
    pv.vertex_lb = v_lb;
    pv.vertex_ub = v_ub;
    pv.n_main_resources = 1;

    BucketGraph<EmptyPack> bg(pv, EmptyPack{}, {.max_paths = 0, .max_scc_labels = 3});
    bg.build();
    auto paths = bg.solve();
    CHECK(best_reduced_cost(paths) == doctest::Approx(-2.0));
    CHECK(bg.exact_complete());
    CHECK_FALSE(bg.label_cap_hit());  // #96: set only when the cap cut labelling
}

TEST_CASE(
    "Elimination and fixing at a gap above the solve's theta keep the paths below it (#116)") {
    // Pricing solves at theta = -1e-6 and eliminates or fixes at gap 5. The
    // parallel completion prune and the bw post-pass prune dropped labels
    // against the solve's theta, so bounds and partners missed paths between
    // it and the gap: 23 (bound-based), 31 (label-based with fixing) and 16
    // (fixing) of these 400 instances lost one in parallel bidir, 5
    // (label-based with fixing) in sequential. Enumerate at the gap
    // afterwards must still return every path below it.
    constexpr double kGap = 5.0;
    auto enumerate = [](auto& solver) {
        std::vector<std::vector<int>> v;
        for (const auto& p : solver.enumerate(kGap)) {
            v.push_back(p.vertices);
        }
        CHECK(solver.enumeration_complete());
        std::ranges::sort(v);
        return v;
    };
    auto run = [&]<typename Exec>(Exec exec, bool bidirectional, bool parallel_bidir) {
        for (int mode = 0; mode < 3; ++mode) {
            int64_t reduced = 0;
            for (int n_main = 1; n_main <= 2; ++n_main) {
                for (int seed = 0; seed < 200; ++seed) {
                    CAPTURE(bidirectional);
                    CAPTURE(parallel_bidir);
                    CAPTURE(mode);
                    CAPTURE(n_main);
                    CAPTURE(seed);
                    RandomTwInstance inst(seed, n_main);
                    const typename Solver<EmptyPack, Exec>::Options opts{
                        .bucket_steps = {2.0, n_main == 2 ? 2.0 : 0.0},
                        .bidirectional = bidirectional,
                        .parallel_bidir = parallel_bidir};
                    Solver<EmptyPack> fresh(
                        inst.pv, EmptyPack{},
                        {.bucket_steps = opts.bucket_steps, .bidirectional = false});
                    fresh.build();
                    const auto all = enumerate(fresh);

                    Solver<EmptyPack, Exec> solver(inst.pv, EmptyPack{}, opts, exec);
                    solver.set_stage(Stage::Exact);
                    solver.build();
                    for (int k = 0; k < 4; ++k) {
                        solver.solve();
                        if (mode == 0) {
                            solver.eliminate_arcs(kGap);
                        } else if (mode == 1) {
                            solver.eliminate_arcs_label_based(kGap);
                            reduced += solver.fix_buckets(kGap);
                        } else {
                            reduced += solver.fix_buckets(kGap);
                        }
                    }
                    reduced += solver.eliminated_bucket_arcs();
                    CHECK(enumerate(solver) == all);
                }
            }
            CHECK(reduced > 0);  // the gap still lets something go
        }
    };
    run(SequentialExecutor{}, true, false);
    run(StdThreadExecutor{}, true, true);
    // The executor flowty passes in, over a caller-owned pool (#135): no
    // other test instantiates Solver<Pack, ThreadPoolExecutor>.
    ThreadPool pool(4);
    run(ThreadPoolExecutor(pool), false, false);
    run(ThreadPoolExecutor(pool), true, true);
}

TEST_CASE("bounds_cover_gap: the theta a solve pruned against bounds the gap (#116)") {
    RandomTwInstance inst(42, 1);
    auto make = [&]<typename Exec>(Exec, bool parallel_bidir, double theta) {
        return BucketGraph<EmptyPack, Exec>(inst.pv, EmptyPack{},
                                            {.bucket_steps = {2.0, 0.0},
                                             .theta = theta,
                                             .bidirectional = true,
                                             .parallel_bidir = parallel_bidir});
    };
    auto solve = [](auto& bg) {
        bg.build();
        CHECK_FALSE(bg.bounds_cover_gap(-1e9));  // nothing solved yet
        bg.solve();
        CHECK(bg.exact_complete());
    };

    // Sequential bidir prunes only the bw partners against theta.
    auto seq = make(SequentialExecutor{}, false, -1e-6);
    solve(seq);
    CHECK(seq.bounds_cover_gap(5.0));
    CHECK(seq.bounds_cover_gap(-1e-6, true));
    CHECK_FALSE(seq.bounds_cover_gap(5.0, true));

    // Parallel bidir prunes the labels behind c_best too, and elimination and
    // fixing at a gap above theta do nothing.
    auto par = make(StdThreadExecutor{}, true, -1e-6);
    solve(par);
    CHECK(par.bounds_cover_gap(-1e-6));
    CHECK_FALSE(par.bounds_cover_gap(5.0));
    CHECK_FALSE(par.bounds_cover_gap(5.0, true));
    par.eliminate_arcs(5.0);
    par.eliminate_arcs_label_based(5.0);
    CHECK(par.eliminated_bucket_arcs() == 0);
    CHECK(par.fix_buckets(5.0) == 0);

    // label_bounds labels at the gap and keeps pricing's midpoint.
    const double mid = par.midpoint();
    par.label_bounds(5.0);
    CHECK(par.midpoint() == mid);
    CHECK(par.bounds_cover_gap(5.0));
    CHECK(par.bounds_cover_gap(5.0, true));
    CHECK_FALSE(par.bounds_cover_gap(5.0 + 1e-3));

    auto at_gap = make(StdThreadExecutor{}, true, 5.0);
    solve(at_gap);
    CHECK(at_gap.bounds_cover_gap(5.0));
    CHECK(at_gap.bounds_cover_gap(5.0, true));
}

// ════════════════════════════════════════════════════════════════════
// Generalized R1C: denominators 2-8 (#86, #142)
// ════════════════════════════════════════════════════════════════════

namespace {

/// A DAG on `n` vertices (source 0, sink n-1), every arc taking one unit of
/// time, with rank-1 cuts given both as `R1Cut`s and as their definition
/// (d, numerators, the arcs that keep the sum), from which it prices paths
/// by enumeration. A subclass fills the arcs and cuts, then `finish_view`.
struct R1CBruteForce {
    int n = 0;
    std::vector<int> from, to;
    std::vector<double> cost, time_d;
    std::vector<double> tw_lb, tw_ub;
    const double* arc_res[1] = {};
    const double* v_lb[1] = {};
    const double* v_ub[1] = {};
    std::vector<R1Cut> cuts;
    std::vector<R1VertexCut> vertex_cuts;
    // Per cut: d and the numerator of each vertex (0 outside C).
    std::vector<int> den;
    std::vector<std::vector<int>> num;
    std::vector<std::vector<bool>> remembers;  // [cut][arc]
    ProblemView pv;

    R1CBruteForce() = default;
    R1CBruteForce(const R1CBruteForce&) = delete;
    R1CBruteForce& operator=(const R1CBruteForce&) = delete;

    /// Every arc takes 1, so a path's time is its arc count: an ub of n - 1
    /// puts the bidir midpoint mid-path, and joins carry bw cut state.
    void finish_view() {
        tw_lb.assign(static_cast<std::size_t>(n), 0.0);
        tw_ub.assign(static_cast<std::size_t>(n), n - 1.0);
        arc_res[0] = time_d.data();
        v_lb[0] = tw_lb.data();
        v_ub[0] = tw_ub.data();
        pv.n_vertices = n;
        pv.source = 0;
        pv.sink = n - 1;
        pv.n_arcs = static_cast<int>(from.size());
        pv.arc_from = from.data();
        pv.arc_to = to.data();
        pv.arc_base_cost = cost.data();
        pv.n_resources = 1;
        pv.arc_resource = arc_res;
        pv.vertex_lb = v_lb;
        pv.vertex_ub = v_ub;
        pv.n_main_resources = 1;
    }

    /// Reduced cost of a path from the cut definition: per memory segment,
    /// ⌊Σ k_v / d⌋, priced at β = max(0, -σ).
    [[nodiscard]] double reduced_cost(const std::vector<int>& arcs) const {
        double rc = 0.0;
        for (int a : arcs) {
            rc += cost[static_cast<std::size_t>(a)];
        }
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            int acc = 0;
            int coef = 0;
            for (int a : arcs) {
                if (!remembers[c][static_cast<std::size_t>(a)]) {
                    coef += acc / den[c];
                    acc = 0;
                }
                acc += num[c][static_cast<std::size_t>(to[static_cast<std::size_t>(a)])];
            }
            coef += acc / den[c];
            rc += std::max(0.0, -cuts[c].dual_value) * coef;
        }
        return rc;
    }

    /// Min reduced cost over every source-sink path, by enumeration.
    [[nodiscard]] double brute_force_min() const {
        double best = INF;
        std::vector<int> arcs;
        auto dfs = [&](auto&& self, int v) -> void {
            if (v == n - 1) {
                best = std::min(best, reduced_cost(arcs));
                return;
            }
            for (std::size_t a = 0; a < from.size(); ++a) {
                if (from[a] == v) {
                    arcs.push_back(static_cast<int>(a));
                    self(self, to[a]);
                    arcs.pop_back();
                }
            }
        };
        dfs(dfs, 0);
        return best;
    }

    /// Min cost over every source-sink path without the cuts.
    [[nodiscard]] double plain_min() const {
        double best = INF;
        auto dfs = [&](auto&& self, int v, double c) -> void {
            if (v == n - 1) {
                best = std::min(best, c);
                return;
            }
            for (std::size_t a = 0; a < from.size(); ++a) {
                if (from[a] == v) {
                    self(self, to[a], c + cost[a]);
                }
            }
        };
        dfs(dfs, 0, 0.0);
        return best;
    }
};

/// A random DAG on N vertices (source 0, sink N-1) with a mixed family of
/// rank-1 cuts — 1-, 2- and 3-bit — over random arc memories, or with
/// `vertex_memory` over random vertex memories, posed both as `R1VertexCut`s
/// and as `R1Cut`s on the arcs those memories induce (#144).
struct MixedR1CInstance : R1CBruteForce {
    static constexpr int N = 9;  // bidir midpoint 4

    explicit MixedR1CInstance(unsigned seed, bool vertex_memory = false) {
        n = N;
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        for (int i = 0; i < N - 1; ++i) {
            for (int j = i + 1; j < N; ++j) {
                if (j == i + 1 || unit(rng) < 0.45) {
                    from.push_back(i);
                    to.push_back(j);
                    cost.push_back(std::round(unit(rng) * 20.0) - 14.0);
                    time_d.push_back(1.0);
                }
            }
        }
        const int n_arcs = static_cast<int>(from.size());
        const int denominators[] = {2, 3, 4, 5, 7, 8, 2, 3};
        for (int d : denominators) {
            R1Cut cut;
            std::vector<int> k(N, 0);
            for (int v = 1; v < N - 1; ++v) {
                if (unit(rng) < 0.5) {
                    k[v] = 1 + static_cast<int>(rng() % static_cast<unsigned>(d - 1));
                    cut.base_set.push_back(v);
                    cut.multipliers.push_back(static_cast<double>(k[v]) / d);
                }
            }
            std::vector<bool> mem(static_cast<std::size_t>(n_arcs), false);
            if (vertex_memory) {
                // M' = memory ∪ C; an arc keeps the sum iff both ends are in M'.
                R1VertexCut vcut{.denominator = d};
                std::vector<bool> in_m(N, false);
                for (int v = 0; v < N; ++v) {
                    if (k[v] != 0) {
                        vcut.coefficients.emplace_back(v, k[v]);
                        in_m[v] = true;
                    } else if (unit(rng) < 0.5) {
                        vcut.memory.push_back(v);
                        in_m[v] = true;
                    }
                }
                for (int a = 0; a < n_arcs; ++a) {
                    const auto ua = static_cast<std::size_t>(a);
                    if (in_m[static_cast<std::size_t>(from[ua])] &&
                        in_m[static_cast<std::size_t>(to[ua])]) {
                        mem[ua] = true;
                        cut.memory_arcs.emplace_back(a, 0);
                    }
                }
                cut.dual_value = -std::round(unit(rng) * 12.0);
                vcut.dual = cut.dual_value;
                vertex_cuts.push_back(vcut);
            } else {
                for (int a = 0; a < n_arcs; ++a) {
                    if (unit(rng) < 0.75) {
                        mem[static_cast<std::size_t>(a)] = true;
                        cut.memory_arcs.emplace_back(a, 0);
                    }
                }
                cut.dual_value = -std::round(unit(rng) * 12.0);
            }
            cuts.push_back(cut);
            // The cut's own d: multipliers sharing a smaller one reduce to it,
            // which prices the same.
            den.push_back(d);
            num.push_back(k);
            remembers.push_back(mem);
        }
        finish_view();
    }
};

/// Labelling's best path prices as the enumeration's best, and every path it
/// returns prices as the cut definition says. Returns false, checking
/// nothing, when no path has a negative reduced cost. Counts in `long_paths`
/// the returned paths of 7 or more arcs. In bidir, a fw label stops past the
/// midpoint 4, at its fifth arc, and joins across its next arc with the bw
/// label there; from the seventh arc on, that bw label has visited an
/// interior vertex, so the join prices a nonzero bw cut state. With
/// `vertex_form`, prices `inst.vertex_cuts` instead of `inst.cuts`; with
/// `per_vertex`, sets `inst.cuts` through the `ProblemView` overload (the
/// per-vertex layout) instead of the global one. `best` gets the best path's
/// reduced cost.
template <std::size_t Words, bool vertex_form = false, bool per_vertex = false>
bool check_mixed_r1c(const MixedR1CInstance& inst, bool bidir, int& long_paths,
                     double* best_out = nullptr) {
    using Res = BasicR1CResource<Words>;
    using Pack = ResourcePack<Res>;
    Solver<Pack> solver(inst.pv, Pack{Res{}},
                        {.bucket_steps = {2.0, 0.0}, .bidirectional = bidir, .theta = 1e9});
    // With two words, 21 3-bit cuts at β = 0 fill bits 0-62 first, so on the
    // global layout the family's 1-bit cut takes bit 63 and the rest spill
    // into word 1.
    auto& r1c = solver.template resource<Res>();
    if constexpr (vertex_form) {
        std::vector<R1VertexCut> cuts;
        if constexpr (Words > 1) {
            cuts.assign(21, {.coefficients = {{1, 1}}, .denominator = 8});
        }
        cuts.insert(cuts.end(), inst.vertex_cuts.begin(), inst.vertex_cuts.end());
        r1c.set_cuts(cuts, inst.pv);
    } else {
        std::vector<R1Cut> cuts;
        if constexpr (Words > 1) {
            for (int i = 0; i < 21; ++i) {
                cuts.push_back({.base_set = {1}, .multipliers = {1.0 / 8}, .memory_arcs = {}});
            }
        }
        cuts.insert(cuts.end(), inst.cuts.begin(), inst.cuts.end());
        if constexpr (per_vertex) {
            r1c.set_cuts(cuts, inst.pv);
        } else {
            r1c.set_cuts(cuts, inst.pv.n_vertices, inst.pv.n_arcs);
        }
    }
    if constexpr (Words > 1 && (vertex_form || per_vertex)) {
        // Per vertex: the 21 3-bit cuts fill bits 0-62 at vertex 1 alone.
        REQUIRE(r1c.field(20, 1).word == 0);
        REQUIRE(r1c.field(20, 1).shift == 60);
        REQUIRE(r1c.field(20, 2).word == -1);
    } else if constexpr (Words > 1) {
        REQUIRE(r1c.field(21).word == 0);
        REQUIRE(r1c.field(21).shift == 63);
        REQUIRE(r1c.field(r1c.n_active() - 1).word == 1);
    }
    solver.build();
    solver.set_stage(Stage::Exact);
    const auto paths = solver.solve();
    const double want = inst.brute_force_min();
    if (want >= 0.0) {
        return false;
    }
    REQUIRE(!paths.empty());
    double best = INF;
    for (const auto& p : paths) {
        CHECK(p.reduced_cost == doctest::Approx(inst.reduced_cost(p.arcs)));
        best = std::min(best, p.reduced_cost);
        long_paths += p.arcs.size() >= 7 ? 1 : 0;
    }
    CHECK(best == doctest::Approx(want));
    if (best_out != nullptr) {
        *best_out = best;
    }
    return true;
}

}  // namespace

TEST_CASE("R1C with denominators 2-8: labelling finds the brute-force best path") {
    int with_cut_cost = 0;
    int checked = 0;
    int long_mono = 0;
    int long_bidir = 0;
    int long_mono_pv = 0;  // per layout, so neither can lean on the other
    int long_bidir_pv = 0;
    for (unsigned seed = 1; seed <= 25; ++seed) {
        CAPTURE(seed);
        const MixedR1CInstance inst(seed);
        // Every width is in the family.
        R1CData data;
        data.set_cuts(inst.cuts, inst.pv.n_vertices, inst.pv.n_arcs);
        bool widths[4] = {};
        for (int c = 0; c < data.n_active(); ++c) {
            widths[data.field(c).width] = true;
        }
        CHECK((widths[1] && widths[2] && widths[3]));
        for (bool bidir : {false, true}) {
            CAPTURE(bidir);
            int& long_paths = bidir ? long_bidir : long_mono;
            int& long_paths_pv = bidir ? long_bidir_pv : long_mono_pv;
            checked += check_mixed_r1c<1>(inst, bidir, long_paths) ? 1 : 0;
            checked += check_mixed_r1c<2>(inst, bidir, long_paths) ? 1 : 0;
            // Per-vertex layout: random arc memories leave arcs between two
            // vertices of a cut's M' that reset its sum.
            checked += check_mixed_r1c<1, false, true>(inst, bidir, long_paths_pv) ? 1 : 0;
            checked += check_mixed_r1c<2, false, true>(inst, bidir, long_paths_pv) ? 1 : 0;
        }
        // Count instances where the cuts change the best path's cost.
        if (inst.brute_force_min() > inst.plain_min() + 0.5) {
            ++with_cut_cost;
        }
    }
    CHECK(checked >= 160);      // most instances have a negative path
    CHECK(with_cut_cost >= 5);  // the cuts matter on a fair share of instances
    CHECK(long_bidir >= 5);     // bidir prices joins with a nonzero bw state
    CHECK(long_mono >= 5);
    CHECK(long_bidir_pv >= 5);
    CHECK(long_mono_pv >= 5);
}

TEST_CASE("R1C vertex memory: both cut forms price as the brute-force best path") {
    int checked = 0;
    int with_cut_cost = 0;
    int long_vertex = 0;  // per form, so neither can lean on the other
    int long_arc = 0;
    int long_vertex2 = 0;
    for (unsigned seed = 1; seed <= 25; ++seed) {
        CAPTURE(seed);
        const MixedR1CInstance inst(seed, true);
        REQUIRE(inst.vertex_cuts.size() == inst.cuts.size());
        for (bool bidir : {false, true}) {
            CAPTURE(bidir);
            double by_vertex = INF;
            double by_arc = INF;
            double by_vertex2 = INF;
            const bool any = check_mixed_r1c<1, true>(inst, bidir, long_vertex, &by_vertex);
            check_mixed_r1c<1, false>(inst, bidir, long_arc, &by_arc);
            check_mixed_r1c<2, true>(inst, bidir, long_vertex2, &by_vertex2);
            if (any) {
                ++checked;
                CHECK(by_vertex == doctest::Approx(by_arc));
                CHECK(by_vertex2 == doctest::Approx(by_arc));
            }
        }
        // Count instances where the cuts change the best path's cost.
        if (inst.brute_force_min() > inst.plain_min() + 0.5) {
            ++with_cut_cost;
        }
    }
    CHECK(checked >= 40);
    CHECK(with_cut_cost >= 5);
    CHECK(long_vertex >= 5);
    CHECK(long_arc >= 5);
    CHECK(long_vertex2 >= 5);
}

namespace {

/// A DAG on 14 vertices, arcs i → j for 1 ≤ j - i ≤ 3, with many local cuts
/// (#146): each over 2 or 3 consecutive interior vertices, its memory the
/// vertex on either side or not. Posed in both forms, the `R1Cut` one
/// remembering the arcs with both ends in M'. `tries` cuts are drawn, and
/// the ones `fit` finds no room for are dropped, so the kept ones fit per
/// vertex but need more than `64·Words` bits in all. A memory vertex is next
/// to a base-set one, and arc i → i + 1 is always there, so the arc form has
/// the same M', and a d no larger, and fits too.
struct LocalR1CInstance : R1CBruteForce {
    static constexpr int N = 14;
    int total_bits = 0;  // over the kept cuts

    LocalR1CInstance(unsigned seed, std::size_t words, int tries) {
        n = N;
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        for (int i = 0; i < N - 1; ++i) {
            for (int j = i + 1; j <= std::min(i + 3, N - 1); ++j) {
                if (j == i + 1 || unit(rng) < 0.7) {
                    from.push_back(i);
                    to.push_back(j);
                    cost.push_back(std::round(unit(rng) * 20.0) - 14.0);
                    time_d.push_back(1.0);
                }
            }
        }
        const int n_arcs = static_cast<int>(from.size());
        finish_view();
        const int denominators[] = {2, 3, 4, 5, 6, 7, 8};
        for (int t = 0; t < tries; ++t) {
            const int d = denominators[rng() % 7];
            const int len = 2 + static_cast<int>(rng() % 2);
            const int first = 1 + static_cast<int>(rng() % static_cast<unsigned>(N - 1 - len));
            std::vector<int> k(N, 0);
            std::vector<bool> in_m(N, false);
            R1Cut cut;
            R1VertexCut vcut{.denominator = d};
            for (int v = first; v < first + len; ++v) {
                k[static_cast<std::size_t>(v)] =
                    1 + static_cast<int>(rng() % static_cast<unsigned>(d - 1));
                vcut.coefficients.emplace_back(v, k[static_cast<std::size_t>(v)]);
                cut.base_set.push_back(v);
                cut.multipliers.push_back(static_cast<double>(k[static_cast<std::size_t>(v)]) / d);
                in_m[static_cast<std::size_t>(v)] = true;
            }
            for (int v : {first - 1, first + len}) {
                if (unit(rng) < 0.5) {
                    vcut.memory.push_back(v);
                    in_m[static_cast<std::size_t>(v)] = true;
                }
            }
            const double dual = -std::floor(unit(rng) * 3.0) / 2.0;  // β ∈ {0, 0.5, 1}
            std::vector<bool> mem(static_cast<std::size_t>(n_arcs), false);
            for (int a = 0; a < n_arcs; ++a) {
                const auto ua = static_cast<std::size_t>(a);
                if (in_m[static_cast<std::size_t>(from[ua])] &&
                    in_m[static_cast<std::size_t>(to[ua])]) {
                    mem[ua] = true;
                    cut.memory_arcs.emplace_back(a, 0);
                }
            }
            cut.dual_value = dual;
            vcut.dual = dual;
            cuts.push_back(cut);
            vertex_cuts.push_back(vcut);
            den.push_back(d);
            num.push_back(k);
            remembers.push_back(mem);
        }
        // Keep the cuts the per-vertex layout has room for (#147's fit).
        const R1CFit fit = words == 1 ? BasicR1CData<1>::fit(vertex_cuts, pv)
                                      : BasicR1CData<2>::fit(vertex_cuts, pv);
        std::size_t out = 0;
        for (std::size_t c = 0, u = 0; c < cuts.size(); ++c) {
            if (u < fit.unplaced.size() && std::cmp_equal(fit.unplaced[u], c)) {
                ++u;
                continue;
            }
            cuts[out] = cuts[c];
            vertex_cuts[out] = vertex_cuts[c];
            den[out] = den[c];
            num[out] = num[c];
            remembers[out] = remembers[c];
            constexpr int kWidth[] = {0, 0, 1, 2, 2, 3, 3, 3, 3};  // ⌈log2 d⌉, by hand
            total_bits += kWidth[den[c]];
            ++out;
        }
        cuts.resize(out);
        vertex_cuts.resize(out);
        den.resize(out);
        num.resize(out);
        remembers.resize(out);
    }
};

/// Labelling's best path on the per-vertex layout prices as the
/// enumeration's best, and every path it returns prices as the cut
/// definition says. Returns the best reduced cost (INF without paths).
template <std::size_t Words, bool vertex_form>
double check_local_r1c(const LocalR1CInstance& inst, bool bidir) {
    using Res = BasicR1CResource<Words>;
    using Pack = ResourcePack<Res>;
    Solver<Pack> solver(inst.pv, Pack{Res{}},
                        {.bucket_steps = {2.0, 0.0}, .bidirectional = bidir, .theta = 1e9});
    auto& r1c = solver.template resource<Res>();
    if constexpr (vertex_form) {
        r1c.set_cuts(inst.vertex_cuts, inst.pv);
    } else {
        r1c.set_cuts(inst.cuts, inst.pv);
    }
    REQUIRE(r1c.n_active() == static_cast<int>(inst.cuts.size()));
    solver.build();
    solver.set_stage(Stage::Exact);
    const auto paths = solver.solve();
    double best = INF;
    for (const auto& p : paths) {
        CHECK(p.reduced_cost == doctest::Approx(inst.reduced_cost(p.arcs)));
        best = std::min(best, p.reduced_cost);
    }
    const double want = inst.brute_force_min();
    if (want < 0.0) {
        CHECK(best == doctest::Approx(want));
    }
    return best;
}

/// The cuts that sit at different places at two vertices of their M', which
/// an arc between those vertices moves, and those at one place across it.
template <std::size_t Words>
std::pair<int, int> moved_kept_cuts(const LocalR1CInstance& inst) {
    BasicR1CData<Words> data;
    data.set_cuts(inst.vertex_cuts, inst.pv);
    int moved = 0;
    int kept = 0;
    for (int c = 0; c < data.n_active(); ++c) {
        std::set<std::pair<int, int>> places;
        for (int v = 0; v < inst.pv.n_vertices; ++v) {
            const auto f = data.field(c, v);
            if (f.word >= 0) {
                places.emplace(f.word, f.shift);
            }
        }
        moved += places.size() > 1 ? 1 : 0;
        kept += places.size() == 1 ? 1 : 0;
    }
    return {moved, kept};
}

}  // namespace

TEST_CASE("R1C per-vertex layout: more cuts than the words hold price as the brute force") {
    // 400 tries fill the vertices, so the gap-free packing lays most of them
    // out; 120 leave room, so the shared places do.
    int full_moved = 0;
    int light_moved = 0;
    int light_kept = 0;
    int full_kept = 0;
    int checked = 0;
    int with_cut_cost = 0;
    for (unsigned seed = 1; seed <= 8; ++seed) {
        CAPTURE(seed);
        for (std::size_t words : {1U, 2U}) {
            for (int tries : {400, 120}) {
                CAPTURE(words);
                CAPTURE(tries);
                const LocalR1CInstance inst(seed, words, tries);
                REQUIRE(inst.total_bits > static_cast<int>(64 * words));
                const auto [mv, kp] =
                    words == 1 ? moved_kept_cuts<1>(inst) : moved_kept_cuts<2>(inst);
                (tries == 400 ? full_moved : light_moved) += mv;
                (tries == 400 ? full_kept : light_kept) += kp;
                const double want = inst.brute_force_min();
                if (want < 0.0) {
                    ++checked;
                }
                if (want > inst.plain_min() + 0.25) {
                    ++with_cut_cost;
                }
                for (bool bidir : {false, true}) {
                    CAPTURE(bidir);
                    if (words == 1) {
                        check_local_r1c<1, true>(inst, bidir);
                        check_local_r1c<1, false>(inst, bidir);
                    } else {
                        check_local_r1c<2, true>(inst, bidir);
                        check_local_r1c<2, false>(inst, bidir);
                    }
                }
            }
        }
    }
    CAPTURE(full_moved);
    CAPTURE(light_moved);
    CAPTURE(light_kept);
    CAPTURE(full_kept);
    CHECK(checked >= 10);
    CHECK(with_cut_cost >= 4);
    CHECK(full_moved >= 20);  // arcs move fields, not only keep them in place
    // With room, the shared places keep cuts in one place across M' far more
    // often than the gap-free packing does (about 1.2 kept per move against
    // 0.04 on these seeds).
    CHECK(light_kept * full_moved > 10 * full_kept * light_moved);
}
