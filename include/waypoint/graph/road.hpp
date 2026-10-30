#pragma once

#include <cmath>
#include <optional>

#include "waypoint/core/types.hpp"

namespace waypoint::graph {

/// Which way a search walks the roads. Reverse follows every road backwards,
/// answering "how long from X to here?" instead of "how long from here to X?".
enum class Direction { Forward, Reverse };

/// Settings for a new road. Fields left out keep their defaults, e.g.
/// `builder.addRoad(a, b, {.speedKmh = 50.0})`.
struct RoadAttributes {
    double speedKmh = 30.0;          ///< free-flow speed limit
    double trafficMultiplier = 1.0;  ///< congestion slowdown: 1 = free flow, 2 = twice as slow
    std::optional<double> lengthKm;  ///< defaults to the straight-line distance
};

/// A one-way road segment between two intersections.
struct Road {
    NodeId from;
    NodeId to;
    double lengthKm;
    double speedKmh;
    double trafficMultiplier;

    /// Time to drive this road right now, in seconds.
    [[nodiscard]] Seconds travelTime() const noexcept {
        constexpr double kSecondsPerHour = 3600.0;
        return lengthKm / speedKmh * kSecondsPerHour * trafficMultiplier;
    }
};

/// Traffic can only slow a road down. A multiplier below 1 would let a road
/// beat its speed limit and break the A* heuristic (see docs/DESIGN.md).
[[nodiscard]] inline bool isValidTrafficMultiplier(double multiplier) noexcept {
    return std::isfinite(multiplier) && multiplier >= 1.0;
}

}  // namespace waypoint::graph
