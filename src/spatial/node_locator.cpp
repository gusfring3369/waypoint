#include "waypoint/spatial/node_locator.hpp"

#include <cassert>

namespace waypoint::spatial {

NodeLocator::NodeLocator(const graph::RoadNetwork& network) : index_(network.bounds()) {
    for (NodeId node = 0; node < network.nodeCount(); ++node) {
        [[maybe_unused]] const bool inserted = index_.insert(node, network.position(node));
        assert(inserted && "network bounds must contain every intersection");
    }
}

std::optional<NodeId> NodeLocator::nearestNode(geo::Point point) const {
    const auto hits = index_.nearest(point, 1);
    if (hits.empty()) return std::nullopt;
    return hits.front().id;
}

}  // namespace waypoint::spatial
