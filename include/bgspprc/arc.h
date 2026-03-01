#pragma once

#include <cstdint>

namespace bgspprc {

struct BucketArc {
    int to_bucket;
    int arc_id;
};

struct JumpArc {
    int jump_bucket;
    int arc_id;
};

}  // namespace bgspprc
