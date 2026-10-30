#pragma once

#include <cstddef>
#include <cstdint>

#include "waypoint/core/types.hpp"
#include "waypoint/graph/road_network.hpp"

namespace waypoint::sim {

/// Settings for a synthetic city. The same settings (including `seed`) always
/// produce the same city, which keeps tests and benchmarks reproducible.
struct CityConfig {
    std::size_t rows = 100;
    std::size_t cols = 100;
    double blockKm = 0.2;  ///< distance between neighbouring intersections
    double jitter = 0.25;  ///< random shift of each intersection, as a fraction of a block
    double localSpeedKmh = 30.0;
    double arterialSpeedKmh = 60.0;
    std::size_t arterialEvery = 8;      ///< every n-th street (both directions) is a fast arterial
    double oneWayRatio = 0.2;           ///< share of local streets that are one-way
    double closedRatio = 0.03;          ///< share of local street segments missing (parks, works)
    double maxTrafficMultiplier = 2.0;  ///< each road gets random congestion in [1, max]
    std::size_t bridgeEvery = 12;       ///< a river splits the city, crossable every n columns; 0 = no river
    std::uint64_t seed = 42;
};

/// Builds a grid-like city with local streets, faster arterials, one-way
/// streets, closures and a river crossable only at bridges, so that
/// "closest in a straight line" and "fastest by road" often disagree.
[[nodiscard]] graph::RoadNetwork generateCity(const CityConfig& config);

/// Id of the intersection at grid position (row, col).
[[nodiscard]] inline NodeId intersectionAt(const CityConfig& config, std::size_t row, std::size_t col) {
    return static_cast<NodeId>(row * config.cols + col);
}

/// The river flows between this row and the row below it.
[[nodiscard]] inline std::size_t firstRowNorthOfRiver(const CityConfig& config) {
    return config.rows / 2;
}

}  // namespace waypoint::sim
