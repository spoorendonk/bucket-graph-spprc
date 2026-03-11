#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "types.h"

namespace bgspprc {

/// Label for the SPPRC labeling algorithm.
///
/// Template parameter Pack is a ResourcePack<Rs...>.
/// Labels are allocated from a pool (LabelPool) for cache efficiency.
template <typename Pack>
struct Label {
  int vertex = -1;
  int bucket = -1;
  Direction dir = Direction::Forward;
  double cost = INF;       // reduced cost
  double real_cost = INF;  // original cost (without duals)

  // Main resource consumptions (up to 2 main resources define bucket grid)
  std::array<double, 2> q = {0.0, 0.0};

  Label* parent = nullptr;
  int parent_arc = -1;

  bool extended = false;
  bool dominated = false;

  // Compile-time resource states
  typename Pack::StatesTuple resource_states{};

  /// Reconstruct path from source to this label (forward labels).
  void get_path(std::vector<int>& vertices, std::vector<int>& arcs) const {
    vertices.clear();
    arcs.clear();
    for (const Label* l = this; l != nullptr; l = l->parent) {
      vertices.push_back(l->vertex);
      if (l->parent_arc >= 0) arcs.push_back(l->parent_arc);
    }
    std::reverse(vertices.begin(), vertices.end());
    std::reverse(arcs.begin(), arcs.end());
  }

  /// Get subpath for backward labels: returns vertices/arcs in forward
  /// order (current_vertex → ... → sink). No reversal since backward
  /// parent chains naturally point toward the sink.
  void get_backward_subpath(std::vector<int>& vertices,
                            std::vector<int>& arcs) const {
    vertices.clear();
    arcs.clear();
    for (const Label* l = this; l != nullptr; l = l->parent) {
      vertices.push_back(l->vertex);
      if (l->parent_arc >= 0) arcs.push_back(l->parent_arc);
    }
  }
};

/// Arena-style pool allocator for labels.
///
/// Allocates labels in large blocks for cache locality.
template <typename Pack>
class LabelPool {
 public:
  explicit LabelPool(std::size_t block_size = 4096)
      : block_size_(block_size) {}

  ~LabelPool() { clear(); }

  LabelPool(const LabelPool&) = delete;
  LabelPool& operator=(const LabelPool&) = delete;
  LabelPool(LabelPool&&) = default;
  LabelPool& operator=(LabelPool&&) = default;

  Label<Pack>* allocate() {
    if (free_pos_ >= current_block_end_) {
      allocate_block();
    }
    auto* label = reinterpret_cast<Label<Pack>*>(free_pos_);
    new (label) Label<Pack>{};

    free_pos_ += label_size();
    ++count_;
    return label;
  }

  void clear() {
    for (auto* block : blocks_) {
      std::free(block);
    }
    blocks_.clear();
    free_pos_ = nullptr;
    current_block_end_ = nullptr;
    count_ = 0;
  }

  std::size_t count() const { return count_; }

 private:
  std::size_t label_size() const {
    // Align to 8 bytes
    std::size_t sz = sizeof(Label<Pack>);
    return (sz + 7) & ~std::size_t{7};
  }

  void allocate_block() {
    std::size_t ls = label_size();
    std::size_t bytes = ls * block_size_;
    // Align blocks to 64-byte cache lines for better locality
    constexpr std::size_t cache_line = 64;
    std::size_t alignment = std::max(alignof(Label<Pack>), cache_line);
    // aligned_alloc requires bytes to be a multiple of alignment
    bytes = (bytes + alignment - 1) & ~(alignment - 1);
    auto* block = static_cast<char*>(std::aligned_alloc(alignment, bytes));
    assert(block);
    blocks_.push_back(block);
    free_pos_ = block;
    current_block_end_ = block + bytes;
  }

  std::size_t block_size_ = 4096;
  std::vector<char*> blocks_;
  char* free_pos_ = nullptr;
  char* current_block_end_ = nullptr;
  std::size_t count_ = 0;
};

}  // namespace bgspprc
