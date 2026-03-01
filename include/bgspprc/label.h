#pragma once

#include "types.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

namespace bgspprc {

/// Label for the SPPRC labeling algorithm.
///
/// Template parameter Pack is a ResourcePack<Rs...>.
/// Labels are allocated from a pool (LabelPool) for cache efficiency.
template <typename Pack>
struct Label {
    int vertex = -1;
    int bucket = -1;
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

    // lm-R1C states: dynamically sized, allocated alongside label
    // n_r1c_words is set by the solver based on active cuts
    uint64_t* r1c_states = nullptr;
    int n_r1c_words = 0;

    /// Reconstruct path from source to this label.
    void get_path(std::vector<int>& vertices, std::vector<int>& arcs) const {
        // Collect in reverse
        vertices.clear();
        arcs.clear();
        for (const Label* l = this; l != nullptr; l = l->parent) {
            vertices.push_back(l->vertex);
            if (l->parent_arc >= 0) arcs.push_back(l->parent_arc);
        }
        // Reverse to get source→sink order
        std::reverse(vertices.begin(), vertices.end());
        std::reverse(arcs.begin(), arcs.end());
    }
};

/// Arena-style pool allocator for labels.
///
/// Allocates labels in large blocks for cache locality.
/// Labels can carry extra R1C state words allocated inline.
template <typename Pack>
class LabelPool {
public:
    explicit LabelPool(int r1c_words = 0, std::size_t block_size = 4096)
        : r1c_words_(r1c_words), block_size_(block_size) {}

    ~LabelPool() { clear(); }

    LabelPool(const LabelPool&) = delete;
    LabelPool& operator=(const LabelPool&) = delete;
    LabelPool(LabelPool&&) = default;
    LabelPool& operator=(LabelPool&&) = default;

    void set_r1c_words(int n) { r1c_words_ = n; }
    int r1c_words() const { return r1c_words_; }

    Label<Pack>* allocate() {
        if (free_pos_ >= current_block_end_) {
            allocate_block();
        }
        auto* label = reinterpret_cast<Label<Pack>*>(free_pos_);
        new (label) Label<Pack>{};

        auto* r1c_ptr = reinterpret_cast<uint64_t*>(
            free_pos_ + sizeof(Label<Pack>));

        if (r1c_words_ > 0) {
            std::memset(r1c_ptr, 0, r1c_words_ * sizeof(uint64_t));
            label->r1c_states = r1c_ptr;
            label->n_r1c_words = r1c_words_;
        }

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
        // Label + inline R1C words, aligned to 8 bytes
        std::size_t sz = sizeof(Label<Pack>) + r1c_words_ * sizeof(uint64_t);
        return (sz + 7) & ~std::size_t{7};
    }

    void allocate_block() {
        std::size_t ls = label_size();
        std::size_t bytes = ls * block_size_;
        auto* block = static_cast<char*>(std::aligned_alloc(
            alignof(Label<Pack>), bytes));
        assert(block);
        blocks_.push_back(block);
        free_pos_ = block;
        current_block_end_ = block + bytes;
    }

    int r1c_words_ = 0;
    std::size_t block_size_ = 4096;
    std::vector<char*> blocks_;
    char* free_pos_ = nullptr;
    char* current_block_end_ = nullptr;
    std::size_t count_ = 0;
};

}  // namespace bgspprc
