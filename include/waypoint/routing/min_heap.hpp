#pragma once

#include <algorithm>
#include <cassert>
#include <compare>
#include <functional>
#include <vector>

#include "waypoint/core/types.hpp"

namespace waypoint::routing {

/// Priority queue of intersections, lowest priority first.
///
/// A thin wrapper over std::push_heap / std::pop_heap. Unlike
/// std::priority_queue it can be cleared while keeping its memory, so a
/// search engine running thousands of queries allocates only once.
class MinHeap {
public:
    void push(NodeId node, Seconds priority) {
        entries_.push_back({priority, node});
        std::push_heap(entries_.begin(), entries_.end(), std::greater<>{});
    }

    /// Removes and returns the node with the lowest priority.
    NodeId popMin() {
        assert(!entries_.empty());
        std::pop_heap(entries_.begin(), entries_.end(), std::greater<>{});
        const NodeId node = entries_.back().node;
        entries_.pop_back();
        return node;
    }

    [[nodiscard]] bool empty() const noexcept { return entries_.empty(); }
    void clear() noexcept { entries_.clear(); }

private:
    struct Entry {
        Seconds priority;
        NodeId node;
        auto operator<=>(const Entry&) const = default;  // by priority, then node
    };

    std::vector<Entry> entries_;
};

}  // namespace waypoint::routing
