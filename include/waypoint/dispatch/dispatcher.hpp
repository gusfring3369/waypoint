#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/dispatch/fleet_index.hpp"
#include "waypoint/dispatch/matching.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/graph/road_network.hpp"
#include "waypoint/routing/shortest_path_engine.hpp"

namespace waypoint::dispatch {

struct Order {
    OrderId id;
    geo::Point pickup;
    NodeId pickupNode;  ///< pickup snapped to the road network
};

struct Assignment {
    OrderId order;
    RiderId rider;
    Seconds pickupEta;  ///< rider's road travel time to the pickup
};

struct DispatchPlan {
    std::vector<Assignment> assignments;
    std::vector<OrderId> unassigned;  ///< no reachable rider within the search radius

    [[nodiscard]] Seconds totalPickupEta() const noexcept;
};

enum class MatchingStrategy {
    Greedy,   ///< orders take their fastest free rider in arrival order
    Optimal,  ///< minimum total pickup ETA across the whole batch
};

struct DispatchConfig {
    double searchRadiusKm = 3.0;             ///< straight-line shortlist radius
    std::size_t maxCandidatesPerOrder = 16;  ///< shortlist size cap per order
};

/// Matches orders to riders in a funnel of increasingly expensive stages:
///
///   1. Shortlist  - QuadTree k-nearest query: cheap, straight-line distance.
///   2. Price      - road-network search: exact ETAs, but only for the shortlist.
///   3. Assign     - pick the rider (single order) or solve the batch matching.
///
/// The planner never mutates the fleet: callers commit a plan by removing the
/// assigned riders from the FleetIndex, which keeps planning side-effect free
/// and easy to test or re-run.
class Dispatcher {
public:
    Dispatcher(const graph::RoadNetwork& network, const FleetIndex& fleet, DispatchConfig config = {});

    /// Fastest-arriving rider for one order. The shortlist is priced with a
    /// single multi-source A* search instead of one search per candidate.
    [[nodiscard]] std::optional<Assignment> dispatch(const Order& order);

    /// Assigns a batch of orders to distinct riders. Each order's shortlist is
    /// priced with one reverse one-to-many search from its pickup.
    [[nodiscard]] DispatchPlan dispatchBatch(std::span<const Order> orders, MatchingStrategy strategy);

private:
    std::vector<Rider> shortlist(const Order& order) const;

    const FleetIndex& fleet_;
    DispatchConfig config_;
    routing::ShortestPathEngine engine_;
};

}  // namespace waypoint::dispatch
