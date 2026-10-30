#include "waypoint/graph/road_network_builder.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace waypoint::graph {
namespace {

// Allows for rounding when a caller passes a length equal to the straight-line
// distance computed a slightly different way.
constexpr double kLengthTolerance = 1e-9;

}  // namespace

NodeId RoadNetworkBuilder::addIntersection(geo::Point position) {
    if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
        throw std::invalid_argument("intersection coordinates must be finite");
    }
    positions_.push_back(position);
    return static_cast<NodeId>(positions_.size() - 1);
}

EdgeId RoadNetworkBuilder::addRoad(NodeId from, NodeId to, const RoadAttributes& attributes) {
    if (from >= positions_.size() || to >= positions_.size()) {
        throw std::invalid_argument("road references an unknown intersection");
    }
    if (from == to) throw std::invalid_argument("a road must connect two different intersections");
    if (!std::isfinite(attributes.speedKmh) || attributes.speedKmh <= 0.0) {
        throw std::invalid_argument("road speed must be positive");
    }
    if (!isValidTrafficMultiplier(attributes.trafficMultiplier)) {
        throw std::invalid_argument("traffic multiplier must be >= 1");
    }

    const double straightLine = geo::euclideanDistance(positions_[from], positions_[to]);
    const double length = attributes.lengthKm.value_or(straightLine);
    if (!std::isfinite(length) || length < straightLine * (1.0 - kLengthTolerance)) {
        throw std::invalid_argument("a road cannot be shorter than the straight-line distance");
    }

    roads_.push_back({from, to, length, attributes.speedKmh, attributes.trafficMultiplier});
    return static_cast<EdgeId>(roads_.size() - 1);
}

void RoadNetworkBuilder::addTwoWayRoad(NodeId a, NodeId b, const RoadAttributes& attributes) {
    addRoad(a, b, attributes);
    addRoad(b, a, attributes);
}

RoadNetwork RoadNetworkBuilder::build() && {
    return RoadNetwork(std::move(positions_), std::move(roads_));
}

}  // namespace waypoint::graph
