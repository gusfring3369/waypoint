#include "waypoint/graph/road_network.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace waypoint::graph {
namespace {

constexpr double kSecondsPerHour = 3600.0;

// Tolerates floating-point noise when a caller passes a length equal to the
// straight-line distance computed a different way.
constexpr double kLengthTolerance = 1e-9;

Seconds computeTravelTime(const Road& road) noexcept {
    return road.lengthKm / road.speedKmh * kSecondsPerHour * road.trafficMultiplier;
}

void validateTrafficMultiplier(double multiplier) {
    if (!std::isfinite(multiplier) || multiplier < 1.0) {
        throw std::invalid_argument("traffic multiplier must be finite and >= 1, got " +
                                    std::to_string(multiplier));
    }
}

}  // namespace

Seconds RoadNetwork::lowerBoundTravelTime(geo::Point from, geo::Point to) const noexcept {
    if (maxSpeedKmh_ <= 0.0) return 0.0;
    return geo::euclideanDistance(from, to) / maxSpeedKmh_ * kSecondsPerHour;
}

void RoadNetwork::setTrafficMultiplier(EdgeId edge, double multiplier) {
    validateTrafficMultiplier(multiplier);
    Road& road = roads_.at(edge);
    road.trafficMultiplier = multiplier;
    travelTime_[edge] = computeTravelTime(road);
}

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
    if (from == to) throw std::invalid_argument("self-loop roads are not allowed");
    if (!std::isfinite(attributes.speedKmh) || attributes.speedKmh <= 0.0) {
        throw std::invalid_argument("road speed must be finite and positive");
    }
    validateTrafficMultiplier(attributes.trafficMultiplier);

    const double straightLine = geo::euclideanDistance(positions_[from], positions_[to]);
    const double length = attributes.lengthKm.value_or(straightLine);
    if (!std::isfinite(length) || length < straightLine * (1.0 - kLengthTolerance)) {
        throw std::invalid_argument("road length cannot be shorter than the straight-line distance");
    }

    roads_.push_back({from, to, length, attributes.speedKmh, attributes.trafficMultiplier});
    return static_cast<EdgeId>(roads_.size() - 1);
}

void RoadNetworkBuilder::addTwoWayRoad(NodeId a, NodeId b, const RoadAttributes& attributes) {
    addRoad(a, b, attributes);
    addRoad(b, a, attributes);
}

RoadNetwork RoadNetworkBuilder::build() && {
    RoadNetwork network;
    const std::size_t nodes = positions_.size();

    // Counting sort of edges into per-node buckets, once keyed by tail and once by head.
    const auto buildCsr = [&](auto keyOf, auto neighborOf) {
        RoadNetwork::Csr csr;
        csr.offsets.assign(nodes + 1, 0);
        for (const Road& road : roads_) ++csr.offsets[keyOf(road) + 1];
        for (std::size_t n = 0; n < nodes; ++n) csr.offsets[n + 1] += csr.offsets[n];

        csr.arcs.resize(roads_.size());
        std::vector<std::size_t> cursor(csr.offsets.begin(), csr.offsets.end() - 1);
        for (EdgeId e = 0; e < roads_.size(); ++e) {
            csr.arcs[cursor[keyOf(roads_[e])]++] = {neighborOf(roads_[e]), e};
        }
        return csr;
    };
    network.outgoing_ = buildCsr([](const Road& r) { return r.from; }, [](const Road& r) { return r.to; });
    network.incoming_ = buildCsr([](const Road& r) { return r.to; }, [](const Road& r) { return r.from; });

    network.travelTime_.reserve(roads_.size());
    for (const Road& road : roads_) {
        network.travelTime_.push_back(computeTravelTime(road));
        network.maxSpeedKmh_ = std::max(network.maxSpeedKmh_, road.speedKmh);
    }

    if (!positions_.empty()) {
        const auto [minX, maxX] = std::ranges::minmax(positions_, {}, &geo::Point::x);
        const auto [minY, maxY] = std::ranges::minmax(positions_, {}, &geo::Point::y);
        network.bounds_ = {{minX.x, minY.y}, {maxX.x, maxY.y}};
    }

    network.positions_ = std::move(positions_);
    network.roads_ = std::move(roads_);
    return network;
}

}  // namespace waypoint::graph
