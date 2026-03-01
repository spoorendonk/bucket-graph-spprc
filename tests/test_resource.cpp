#include "doctest.h"
#include <bgspprc/resource.h>
#include <bgspprc/resources/standard.h>
#include <bgspprc/resources/ng_path.h>

using namespace bgspprc;

// ── StandardResource ──

TEST_CASE("StandardResource init_state forward") {
    //  source=0, sink=2, 3 vertices
    double consumption[] = {1.0, 2.0, 3.0};
    double lb[] = {0.0, 5.0, 10.0};
    double ub[] = {100.0, 50.0, 30.0};

    StandardResource res(consumption, lb, ub, 0, 2, 3);

    CHECK(res.init_state(Direction::Forward) == 0.0);    // lb[source]
    CHECK(res.init_state(Direction::Backward) == 10.0);  // lb[sink]
}

TEST_CASE("StandardResource extend") {
    double consumption[] = {5.0, 3.0};
    double lb[] = {0.0, 0.0, 0.0};
    double ub[] = {100.0, 100.0, 100.0};

    StandardResource res(consumption, lb, ub, 0, 2, 2);

    auto [s1, c1] = res.extend(Direction::Forward, 10.0, 0);
    CHECK(s1 == 15.0);
    CHECK(c1 == 0.0);

    auto [s2, c2] = res.extend(Direction::Forward, 10.0, 1);
    CHECK(s2 == 13.0);
    CHECK(c2 == 0.0);
}

// ── ResourcePack ──

TEST_CASE("EmptyPack operations") {
    EmptyPack pack;
    auto states = pack.init_states(Direction::Forward);
    static_assert(std::tuple_size_v<decltype(states)> == 0);

    auto [new_states, cost] = pack.extend(Direction::Forward, states, 0);
    CHECK(cost == 0.0);
}

TEST_CASE("ResourcePack with StandardResource") {
    double consumption[] = {2.0, 3.0};
    double lb[] = {0.0, 0.0, 0.0};
    double ub[] = {100.0, 100.0, 100.0};

    auto pack = make_resource_pack(
        StandardResource(consumption, lb, ub, 0, 2, 2));

    auto states = pack.init_states(Direction::Forward);
    CHECK(std::get<0>(states) == 0.0);

    auto [new_states, cost] = pack.extend(Direction::Forward, states, 0);
    CHECK(std::get<0>(new_states) == 2.0);
    CHECK(cost == 0.0);
}

// ── NgPathResource ──

TEST_CASE("NgPathResource basic") {
    // 4 vertices: 0=source, 3=sink
    // ng_sets: each vertex's neighborhood
    uint64_t ng_sets[] = {
        0b1111,  // NG(0) = {0,1,2,3}
        0b1111,  // NG(1) = {0,1,2,3}
        0b1111,  // NG(2) = {0,1,2,3}
        0b1111,  // NG(3) = {0,1,2,3}
    };
    int arc_to[] = {1, 2, 3, 1};  // 4 arcs

    NgPathResource ng(4, arc_to, ng_sets);

    auto s0 = ng.init_state(Direction::Forward);
    CHECK(s0 == 0ULL);

    // Extend to vertex 1 via arc 0
    auto [s1, c1] = ng.extend(Direction::Forward, s0, 0);
    CHECK(c1 == 0.0);
    CHECK((s1 & (1ULL << 1)) != 0);  // vertex 1 is now forbidden

    // Extend to vertex 2 via arc 1
    auto [s2, c2] = ng.extend(Direction::Forward, s1, 1);
    CHECK(c2 == 0.0);
    CHECK((s2 & (1ULL << 2)) != 0);  // vertex 2 forbidden
    CHECK((s2 & (1ULL << 1)) != 0);  // vertex 1 still forbidden (in NG(2))

    // Try to revisit vertex 1 via arc 3 — should be infeasible
    auto [s3, c3] = ng.extend(Direction::Forward, s2, 3);
    CHECK(c3 == INF);
}

TEST_CASE("NgPathResource domination") {
    uint64_t ng_sets[] = {0b1111, 0b1111, 0b1111, 0b1111};
    int arc_to[] = {1, 2};

    NgPathResource ng(4, arc_to, ng_sets);

    // s1 is subset of s2 → s1 can dominate
    uint64_t s1 = 0b0010;  // vertex 1 forbidden
    uint64_t s2 = 0b0110;  // vertices 1,2 forbidden

    CHECK(ng.domination_cost(Direction::Forward, 0, s1, s2) == 0.0);

    // s1 has extra forbidden → cannot dominate
    CHECK(ng.domination_cost(Direction::Forward, 0, s2, s1) == INF);
}
