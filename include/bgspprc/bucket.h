#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "arc.h"
#include "types.h"

namespace bgspprc {

struct Bucket {
  int vertex = -1;
  int scc_id = -1;

  // Main resource interval for this bucket
  std::array<double, 2> lb = {0.0, 0.0};
  std::array<double, 2> ub = {0.0, 0.0};

  // Completion bound: best cost of any label in this bucket or reachable
  // buckets
  double c_best = INF;
  double bw_c_best = INF;  // backward completion bound

  std::vector<BucketArc> bucket_arcs;     // forward bucket arcs
  std::vector<BucketArc> bw_bucket_arcs;  // backward bucket arcs
  std::vector<JumpArc> jump_arcs;         // forward jump arcs
  std::vector<JumpArc> bw_jump_arcs;      // backward jump arcs
};

/// Bitmap for fast bucket-fixed queries.  O(1) test, cache-friendly.
class BucketFixBitmap {
 public:
  void resize(int n_buckets) {
    n_ = n_buckets;
    int n_words = (n_buckets + 63) / 64;
    bits_.assign(n_words, 0);
    n_fixed_ = 0;
  }

  void clear() {
    std::fill(bits_.begin(), bits_.end(), 0ULL);
    n_fixed_ = 0;
  }

  void set(int i) {
    int w = i / 64;
    int b = i % 64;
    if (!(bits_[w] & (1ULL << b))) {
      bits_[w] |= (1ULL << b);
      ++n_fixed_;
    }
  }

  bool test(int i) const { return (bits_[i / 64] >> (i % 64)) & 1; }

  int n_fixed() const { return n_fixed_; }
  int n_total() const { return n_; }

 private:
  std::vector<uint64_t> bits_;
  int n_ = 0;
  int n_fixed_ = 0;
};

}  // namespace bgspprc
