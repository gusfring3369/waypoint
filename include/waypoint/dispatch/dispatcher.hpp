#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "waypoint/dispatch/cost_matrix.hpp"
#include "waypoint/dispatch/fleet_index.hpp"
#include "waypoint/dispatch/matcher.hpp"
#include "waypoint/dispatch/models.hpp"
#include "waypoint/graph/road_network.hpp"
#include "waypoint/routing/shortest_path_engine.hpp"

namespace waypoint::dispatch {

struct DispatchConfig {
    double searchRadiusKm = 3.0;             ///< only consider riders this close (straight line)
    std::size_t maxCandidatesPerOrder = 16;  ///< and at most this many of them
};

/// Chooses riders for orders in three stages, each more expensive per rider
/// than the last, so the expensive stages only see a few riders:
///
///   1. Shortlist - QuadTree: the nearest riders by straight-line distance.
///   2. Price     - road search: exact driving times for the shortlist only.
///   3. Assign    - pick the fastest rider, or hand the batch to a Matcher.
///
/// The Dispatcher only plans; it never changes the fleet. The caller commits a
/// plan by removing the assigned riders from the FleetIndex.
class Dispatcher {
public:
    Dispatcher(const graph::RoadNetwork& network, const FleetIndex& fleet, DispatchConfig config = {});

    /// The rider who can reach this order's pickup soonest, if any is in range.
    [[nodiscard]] std::optional<Assignment> dispatch(const Order& order);

    /// Assigns a batch of orders to distinct riders using `matcher`'s policy.
    [[nodiscard]] DispatchPlan dispatchBatch(std::span<const Order> orders, const Matcher& matcher);

private:
    /// Stage 1 for a batch: every shortlisted rider gets one column; each order
    /// remembers which columns are its candidates.
    struct BatchShortlist {
        std::vector<Rider> riders;
        std::vector<std::vector<std::size_t>> candidateColumns;  // per order
    };

    [[nodiscard]] std::vector<Rider> shortlist(const Order& order) const;
    [[nodiscard]] BatchShortlist shortlistBatch(std::span<const Order> orders) const;
    [[nodiscard]] CostMatrix priceShortlist(std::span<const Order> orders, const BatchShortlist& shortlist);
    [[nodiscard]] static DispatchPlan toPlan(std::span<const Order> orders, const BatchShortlist& shortlist,
                                             const CostMatrix& etas, const Matching& matching);

    const FleetIndex& fleet_;
    DispatchConfig config_;
    routing::ShortestPathEngine engine_;
};

}  // namespace waypoint::dispatch
