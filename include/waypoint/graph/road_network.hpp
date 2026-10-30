#pragma once

#include <cassert>
#include <cstddef>
#include <span>
#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/geo/bounding_box.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/graph/adjacency_index.hpp"
#include "waypoint/graph/road.hpp"

namespace waypoint::graph {

/// A city's road graph: intersections (nodes) joined by one-way roads (edges).
///
/// Created only by RoadNetworkBuilder, which validates every road, so a
/// RoadNetwork is always valid. After that the layout of roads is fixed; the
/// only thing that can change is traffic.
///
/// Every road satisfies  length >= straight-line distance,  speed <= maxSpeedKmh()
/// and  traffic >= 1. These make lowerBoundTravelTime() a safe A* heuristic.
class RoadNetwork {
public:
    [[nodiscard]] std::size_t nodeCount() const noexcept { return positions_.size(); }
    [[nodiscard]] std::size_t edgeCount() const noexcept { return roads_.size(); }
    [[nodiscard]] const geo::BoundingBox& bounds() const noexcept { return bounds_; }
    [[nodiscard]] double maxSpeedKmh() const noexcept { return maxSpeedKmh_; }

    // Accessors below expect valid ids (node < nodeCount(), edge < edgeCount()).

    [[nodiscard]] geo::Point position(NodeId node) const noexcept {
        assert(node < positions_.size());
        return positions_[node];
    }

    [[nodiscard]] const Road& road(EdgeId edge) const noexcept {
        assert(edge < roads_.size());
        return roads_[edge];
    }

    /// Roads leaving `node` (Forward) or entering it (Reverse).
    [[nodiscard]] std::span<const Arc> arcs(NodeId node, Direction direction) const noexcept {
        return direction == Direction::Forward ? outgoing_.arcsOf(node) : incoming_.arcsOf(node);
    }

    /// Current travel time across a road, traffic included.
    [[nodiscard]] Seconds travelTime(EdgeId edge) const noexcept {
        assert(edge < travelTimes_.size());
        return travelTimes_[edge];
    }

    /// A travel time that no route between the two points can beat:
    /// the straight-line distance driven at the network's top speed.
    [[nodiscard]] Seconds lowerBoundTravelTime(geo::Point from, geo::Point to) const noexcept;

    /// Applies a live traffic update. Throws std::invalid_argument if the
    /// multiplier is below 1 (see isValidTrafficMultiplier).
    void setTrafficMultiplier(EdgeId edge, double multiplier);

private:
    friend class RoadNetworkBuilder;  // the only way to create a network

    RoadNetwork(std::vector<geo::Point> positions, std::vector<Road> roads);

    std::vector<geo::Point> positions_;
    std::vector<Road> roads_;
    std::vector<Seconds> travelTimes_;  // Road::travelTime() cached for the search loop
    AdjacencyIndex outgoing_;
    AdjacencyIndex incoming_;
    geo::BoundingBox bounds_;
    double maxSpeedKmh_ = 0.0;
};

}  // namespace waypoint::graph
