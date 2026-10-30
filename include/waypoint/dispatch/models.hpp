#pragma once

#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/geo/point.hpp"

namespace waypoint::dispatch {

/// A delivery rider who is free to take an order.
struct Rider {
    RiderId id;
    geo::Point position;  ///< last reported GPS position
    NodeId node;          ///< nearest intersection, used for routing
};

/// A customer order waiting for a rider.
struct Order {
    OrderId id;
    geo::Point pickup;  ///< restaurant location
    NodeId pickupNode;  ///< nearest intersection to the pickup
};

/// One rider sent to one order.
struct Assignment {
    OrderId order;
    RiderId rider;
    Seconds pickupEta;  ///< rider's driving time to the pickup
};

/// The outcome of dispatching a batch of orders.
struct DispatchPlan {
    std::vector<Assignment> assignments;
    std::vector<OrderId> unassigned;  ///< no reachable rider within the search radius

    [[nodiscard]] Seconds totalPickupEta() const noexcept {
        Seconds total = 0.0;
        for (const Assignment& assignment : assignments) total += assignment.pickupEta;
        return total;
    }
};

}  // namespace waypoint::dispatch
