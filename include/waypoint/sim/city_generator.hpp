#pragma once

#include <cstddef>
#include <cstdint>

#include "waypoint/graph/road_network.hpp"

namespace waypoint::sim {

/// Parameters for a synthetic, reproducible city used by the demo, tests and benchmarks.
struct CityConfig {
    std::size_t rows = 100;
    std::size_t cols = 100;
    double blockKm = 0.2;  ///< spacing between neighbouring intersections
    double jitter = 0.25;  ///< random intersection offset, as a fraction of a block
    double localSpeedKmh = 30.0;
    double arterialSpeedKmh = 60.0;
    std::size_t arterialEvery = 8;      ///< every n-th street (both axes) is a fast arterial
    double oneWayRatio = 0.2;           ///< share of local streets that are one-way
    double closedRatio = 0.03;          ///< share of local segments removed (parks, works)
    double maxTrafficMultiplier = 2.0;  ///< per-road congestion drawn from [1, max]
    std::size_t bridgeEvery = 12;       ///< a river splits the city; 0 disables it
    std::uint64_t seed = 42;
};

/// Builds a perturbed grid city: local streets, faster arterials, one-way
/// streets, closures and a river crossable only at bridges (so that straight-
/// line proximity and road proximity genuinely disagree). Intersection
/// (row, col) has id row * cols + col.
[[nodiscard]] graph::RoadNetwork generateCity(const CityConfig& config);

}  // namespace waypoint::sim
