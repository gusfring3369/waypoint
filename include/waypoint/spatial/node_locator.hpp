#pragma once

#include <optional>

#include "waypoint/core/types.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/graph/road_network.hpp"
#include "waypoint/spatial/quad_tree.hpp"

namespace waypoint::spatial {

/// Snaps arbitrary map positions (a customer's GPS fix, a rider's location)
/// onto the nearest road-network intersection.
class NodeLocator {
public:
    explicit NodeLocator(const graph::RoadNetwork& network);

    /// Nearest intersection to `point`, or nullopt for an empty network.
    [[nodiscard]] std::optional<NodeId> nearestNode(geo::Point point) const;

private:
    QuadTree index_;
};

}  // namespace waypoint::spatial
