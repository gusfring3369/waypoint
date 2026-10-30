#pragma once

#include <cstddef>
#include <vector>

#include "waypoint/core/types.hpp"

namespace waypoint::routing {

/// How much work a search did. Used to compare algorithms fairly.
struct SearchStats {
    std::size_t nodesSettled = 0;  ///< intersections whose travel time was finalised
    std::size_t edgesRelaxed = 0;  ///< roads examined while doing so
};

/// Result of a point-to-point query.
struct Route {
    Seconds travelTime = kUnreachable;
    std::vector<NodeId> path;  ///< source first, target last; empty when unreachable
    SearchStats stats;

    [[nodiscard]] bool found() const noexcept { return !path.empty(); }
};

/// Result of a multi-source query: which source reached the target first.
struct NearestSource {
    std::size_t sourceIndex;  ///< position in the `sources` list passed in
    Seconds travelTime;
};

}  // namespace waypoint::routing
