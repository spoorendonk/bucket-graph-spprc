#include "doctest.h"
#include <bgspprc/bucket_graph.h>
#include <bgspprc/resource.h>

using namespace bgspprc;

// Helper to build a simple 4-vertex graph:
//   0 → 1 → 3
//   0 → 2 → 3
// with time windows and costs.
struct SimpleGraph {
    // 4 vertices: 0=source, 3=sink
    // 4 arcs: 0→1, 0→2, 1→3, 2→3
    int from[4]      = {0, 0, 1, 2};
    int to[4]        = {1, 2, 3, 3};
    double cost[4]   = {1.0, 2.0, 3.0, 1.0};  // base costs
    double time_d[4] = {1.0, 2.0, 1.0, 1.0};  // time consumption

    // Time windows per vertex
    double tw_lb[4] = {0.0, 0.0, 0.0, 0.0};
    double tw_ub[4] = {10.0, 10.0, 10.0, 10.0};

    const double* arc_res[1]   = {time_d};
    const double* v_lb[1]      = {tw_lb};
    const double* v_ub[1]      = {tw_ub};

    ProblemView pv;

    SimpleGraph() {
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
    }
};

TEST_CASE("Bucket construction") {
    SimpleGraph g;
    BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0}});
    bg.build();

    // With step=5, tw=[0,10] for each vertex: ceil(10/5)=2 buckets per vertex
    // 4 vertices × 2 buckets = 8 total
    CHECK(bg.n_buckets() == 8);

    // Verify bucket properties
    for (int i = 0; i < bg.n_buckets(); ++i) {
        CHECK(bg.bucket(i).vertex >= 0);
        CHECK(bg.bucket(i).vertex < 4);
    }
}

TEST_CASE("Bucket construction step=1") {
    SimpleGraph g;
    BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
        {.bucket_steps = {1.0, 1.0}});
    bg.build();

    // With step=1, tw=[0,10]: ceil(10/1)=10 buckets per vertex
    // 4 vertices × 10 buckets = 40 total
    CHECK(bg.n_buckets() == 40);
}

TEST_CASE("Bucket arcs exist") {
    SimpleGraph g;
    BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0}});
    bg.build();

    // Check that source buckets have outgoing arcs
    int total_arcs = 0;
    for (int i = 0; i < bg.n_buckets(); ++i) {
        total_arcs += static_cast<int>(bg.bucket(i).bucket_arcs.size());
    }
    CHECK(total_arcs > 0);
}

TEST_CASE("Solve simple graph") {
    SimpleGraph g;
    BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});  // accept all paths
    bg.build();

    auto paths = bg.solve();
    CHECK(!paths.empty());

    // Best path should be 0→2→3 with cost 2+1=3 or 0→1→3 with cost 1+3=4
    // So best is 0→2→3 = 3.0
    CHECK(paths[0].reduced_cost == doctest::Approx(3.0));
    CHECK(paths[0].vertices.size() == 3);
}

TEST_CASE("Solve with reduced costs") {
    SimpleGraph g;
    BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
        {.bucket_steps = {5.0, 1.0}, .tolerance = 1e9});
    bg.build();

    // Set reduced costs that make path 0→1→3 cheaper
    double red_cost[4] = {-5.0, 2.0, 3.0, 1.0};
    bg.update_arc_costs(red_cost);

    auto paths = bg.solve();
    CHECK(!paths.empty());

    // 0→1→3: -5+3 = -2
    // 0→2→3: 2+1 = 3
    CHECK(paths[0].reduced_cost == doctest::Approx(-2.0));
}

TEST_CASE("Time window feasibility") {
    // Make tight time windows so some paths are infeasible
    SimpleGraph g;
    g.tw_ub[1] = 0.5;  // vertex 1 can only be reached at time ≤ 0.5
    // Arc 0→1 has time consumption 1.0, so arrival at 1 = 1.0 > 0.5 → infeasible

    BucketGraph<EmptyPack> bg(g.pv, EmptyPack{},
        {.bucket_steps = {1.0, 1.0}, .tolerance = 1e9});
    bg.build();

    auto paths = bg.solve();

    // Only path 0→2→3 should be feasible
    for (const auto& p : paths) {
        // No path should go through vertex 1
        bool has_v1 = false;
        for (int v : p.vertices) {
            if (v == 1) has_v1 = true;
        }
        CHECK(!has_v1);
    }
}
