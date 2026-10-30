#pragma once

#include <cstddef>
#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/graph/road.hpp"
#include "waypoint/graph/road_network.hpp"

namespace waypoint::graph {

/// Collects intersections and roads, rejects anything invalid, and turns the
/// result into a RoadNetwork (Builder pattern).
///
///   RoadNetworkBuilder builder;
///   NodeId a = builder.addIntersection({0.0, 0.0});
///   NodeId b = builder.addIntersection({1.0, 0.0});
///   builder.addTwoWayRoad(a, b, {.speedKmh = 40.0});
///   RoadNetwork city = std::move(builder).build();
class RoadNetworkBuilder {
public:
    /// Adds an intersection at `position` (in km) and returns its id.
    NodeId addIntersection(geo::Point position);

    /// Adds a one-way road. Throws std::invalid_argument for unknown
    /// intersections, a road from a node to itself, a non-positive speed,
    /// traffic below 1, or a length shorter than the straight-line distance.
    EdgeId addRoad(NodeId from, NodeId to, const RoadAttributes& attributes = {});

    /// Adds the road in both directions with the same attributes.
    void addTwoWayRoad(NodeId a, NodeId b, const RoadAttributes& attributes = {});

    [[nodiscard]] std::size_t nodeCount() const noexcept { return positions_.size(); }

    /// Hands the collected data to a new RoadNetwork. Leaves the builder empty,
    /// hence the `std::move(builder).build()` call style.
    [[nodiscard]] RoadNetwork build() &&;

private:
    std::vector<geo::Point> positions_;
    std::vector<Road> roads_;
};

}  // namespace waypoint::graph
