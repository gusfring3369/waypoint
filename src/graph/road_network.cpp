#include "waypoint/graph/road_network.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace waypoint::graph {
namespace {

geo::BoundingBox boundsOf(const std::vector<geo::Point>& points) {
    if (points.empty()) return {};
    const auto [minX, maxX] = std::ranges::minmax(points, {}, &geo::Point::x);
    const auto [minY, maxY] = std::ranges::minmax(points, {}, &geo::Point::y);
    return {{minX.x, minY.y}, {maxX.x, maxY.y}};
}

}  // namespace

RoadNetwork::RoadNetwork(std::vector<geo::Point> positions, std::vector<Road> roads)
    : positions_(std::move(positions)),
      roads_(std::move(roads)),
      outgoing_(positions_.size(), roads_, Direction::Forward),
      incoming_(positions_.size(), roads_, Direction::Reverse),
      bounds_(boundsOf(positions_)) {
    travelTimes_.reserve(roads_.size());
    for (const Road& road : roads_) {
        travelTimes_.push_back(road.travelTime());
        maxSpeedKmh_ = std::max(maxSpeedKmh_, road.speedKmh);
    }
}

Seconds RoadNetwork::lowerBoundTravelTime(geo::Point from, geo::Point to) const noexcept {
    constexpr double kSecondsPerHour = 3600.0;
    if (maxSpeedKmh_ <= 0.0) return 0.0;  // no roads at all
    return geo::euclideanDistance(from, to) / maxSpeedKmh_ * kSecondsPerHour;
}

void RoadNetwork::setTrafficMultiplier(EdgeId edge, double multiplier) {
    if (edge >= roads_.size()) throw std::out_of_range("unknown road " + std::to_string(edge));
    if (!isValidTrafficMultiplier(multiplier)) {
        throw std::invalid_argument("traffic multiplier must be >= 1, got " + std::to_string(multiplier));
    }
    roads_[edge].trafficMultiplier = multiplier;
    travelTimes_[edge] = roads_[edge].travelTime();
}

}  // namespace waypoint::graph
