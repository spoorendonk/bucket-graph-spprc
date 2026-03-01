#pragma once

#include "arc.h"
#include "types.h"

#include <array>
#include <vector>

namespace bgspprc {

struct Bucket {
    int vertex = -1;
    int scc_id = -1;

    // Main resource interval for this bucket
    std::array<double, 2> lb = {0.0, 0.0};
    std::array<double, 2> ub = {0.0, 0.0};

    // Completion bound: best cost of any label in this bucket or reachable buckets
    double c_best = INF;

    bool fixed = false;  // eliminated by bucket fixing

    std::vector<BucketArc> bucket_arcs;
    std::vector<JumpArc> jump_arcs;
};

}  // namespace bgspprc
