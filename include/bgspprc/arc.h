#pragma once

#include <cstdint>

namespace bgspprc {

/// Bucket arc with inlined hot fields to avoid cache misses on extend_label.
/// ~48 bytes — fits one 64B cache line.
struct BucketArc {
  int to_bucket;      // 4B
  int arc_id;         // 4B — kept for pack_.extend_along_arc() and path reconstruction
  int to_vertex;      // 4B — pv_.arc_to[arc_id] (fw) / pv_.arc_from[arc_id] (bw)
  double cost;        // 8B — reduced_costs_[arc_id] or arc_base_cost[arc_id]
  double real_cost;   // 8B — always pv_.arc_base_cost[arc_id]
  double resource[2]; // 16B — pv_.arc_resource[0][arc_id], pv_.arc_resource[1][arc_id]
};

/// Jump arc with inlined hot fields — same layout as BucketArc for shared fields.
struct JumpArc {
  int jump_bucket;    // 4B
  int arc_id;         // 4B — kept for pack_.extend_along_arc() and path reconstruction
  int to_vertex;      // 4B — pv_.arc_to[arc_id] (fw) / pv_.arc_from[arc_id] (bw)
  double cost;        // 8B — reduced_costs_[arc_id] or arc_base_cost[arc_id]
  double real_cost;   // 8B — always pv_.arc_base_cost[arc_id]
  double resource[2]; // 16B — pv_.arc_resource[0][arc_id], pv_.arc_resource[1][arc_id]
};

}  // namespace bgspprc
