#pragma once

#include "waypoint/core/types.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/graph/road_network.hpp"

namespace waypoint::routing {

// A heuristic estimates the remaining travel time from an intersection to the
// target. A* explores intersections in order of  (time so far + estimate),
// which pulls the search toward the target. If the estimate never exceeds the
// real remaining time, A* still returns the optimal route.
//
// Heuristics are plain function objects (not virtual classes) because they are
// called for every road the search looks at; templates let the compiler inline them.

/// Estimates zero everywhere. A* with this heuristic is exactly Dijkstra.
struct NoHeuristic {
    Seconds operator()(NodeId /*node*/) const noexcept { return 0.0; }
};

/// Straight-line distance to the target driven at the network's top speed:
/// no real route can be faster, so the estimate never overshoots.
/// A weight above 1 (weighted A*) searches less but may return a route up to
/// `weight` times slower than the best one.
class StraightLineHeuristic {
public:
    StraightLineHeuristic(const graph::RoadNetwork& network, NodeId target, double weight = 1.0)
        : network_(network), target_(network.position(target)), weight_(weight) {}

    Seconds operator()(NodeId node) const noexcept {
        return weight_ * network_.lowerBoundTravelTime(network_.position(node), target_);
    }

private:
    const graph::RoadNetwork& network_;
    geo::Point target_;
    double weight_;
};

}  // namespace waypoint::routing
