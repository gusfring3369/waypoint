#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "waypoint/core/types.hpp"

namespace waypoint::routing {

/// Per-intersection bookkeeping for one shortest-path query: the best travel
/// time found so far, the previous intersection on that route, which starting
/// point (seed) the route began at, and whether the time is final ("settled").
///
/// The storage is allocated once and reused by every query. Clearing V labels
/// before each query would cost O(V) even when the query only looks at a few
/// streets, so instead each label records the query number ("generation")
/// that wrote it. Labels from older generations count as empty, and clear()
/// just increments the generation: O(1).
class SearchLabels {
public:
    explicit SearchLabels(std::size_t nodeCount) : labels_(nodeCount) {}

    /// Forgets every label.
    void clear() {
        if (++generation_ == 0) {  // wrapped around after 2^32 queries: reset for real
            std::ranges::fill(labels_, Label{});
            generation_ = 1;
        }
    }

    /// Records a route to `node` if it is faster than the best one known.
    /// Returns true when the label changed.
    bool improve(NodeId node, Seconds cost, NodeId parent, std::uint32_t seed) {
        Label& label = at(node);
        if (label.reached == generation_ && label.cost <= cost) return false;
        label = {cost, parent, seed, generation_, label.settled};
        return true;
    }

    void markSettled(NodeId node) { at(node).settled = generation_; }

    [[nodiscard]] bool isSettled(NodeId node) const { return at(node).settled == generation_; }

    [[nodiscard]] Seconds cost(NodeId node) const {
        const Label& label = at(node);
        return label.reached == generation_ ? label.cost : kUnreachable;
    }

    [[nodiscard]] NodeId parent(NodeId node) const { return at(node).parent; }
    [[nodiscard]] std::uint32_t seed(NodeId node) const { return at(node).seed; }

private:
    struct Label {
        Seconds cost = kUnreachable;
        NodeId parent = kInvalidNode;
        std::uint32_t seed = 0;
        std::uint32_t reached = 0;  // generation that wrote `cost`
        std::uint32_t settled = 0;  // generation in which `cost` became final
    };

    Label& at(NodeId node) {
        assert(node < labels_.size());
        return labels_[node];
    }
    const Label& at(NodeId node) const {
        assert(node < labels_.size());
        return labels_[node];
    }

    std::vector<Label> labels_;
    std::uint32_t generation_ = 1;
};

}  // namespace waypoint::routing
