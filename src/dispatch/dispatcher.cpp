#include "waypoint/dispatch/dispatcher.hpp"

#include <algorithm>
#include <iterator>
#include <unordered_map>

namespace waypoint::dispatch {

Seconds DispatchPlan::totalPickupEta() const noexcept {
    Seconds total = 0.0;
    for (const Assignment& assignment : assignments) total += assignment.pickupEta;
    return total;
}

Dispatcher::Dispatcher(const graph::RoadNetwork& network, const FleetIndex& fleet, DispatchConfig config)
    : fleet_(fleet), config_(config), engine_(network) {}

std::optional<Assignment> Dispatcher::dispatch(const Order& order) {
    const std::vector<Rider> candidates = shortlist(order);

    std::vector<NodeId> sources;
    sources.reserve(candidates.size());
    std::ranges::transform(candidates, std::back_inserter(sources), &Rider::node);

    const auto nearest = engine_.nearestSource(sources, order.pickupNode);
    if (!nearest) return std::nullopt;
    return Assignment{order.id, candidates[nearest->sourceIndex].id, nearest->travelTime};
}

DispatchPlan Dispatcher::dispatchBatch(std::span<const Order> orders, MatchingStrategy strategy) {
    // Stage 1: shortlist each order; the union of shortlisted riders becomes the matrix columns.
    std::vector<Rider> riders;
    std::unordered_map<RiderId, std::size_t> columnOf;
    std::vector<std::vector<std::size_t>> columnsOfOrder(orders.size());
    for (std::size_t row = 0; row < orders.size(); ++row) {
        for (const Rider& rider : shortlist(orders[row])) {
            const auto [it, inserted] = columnOf.try_emplace(rider.id, riders.size());
            if (inserted) riders.push_back(rider);
            columnsOfOrder[row].push_back(it->second);
        }
    }

    // Stage 2: price each order's shortlist with one reverse search from its pickup.
    CostMatrix eta(orders.size(), riders.size());
    std::vector<NodeId> riderNodes;
    for (std::size_t row = 0; row < orders.size(); ++row) {
        riderNodes.clear();
        for (const std::size_t col : columnsOfOrder[row]) riderNodes.push_back(riders[col].node);

        const auto times = engine_.travelTimes(orders[row].pickupNode, riderNodes, graph::Direction::Reverse);
        for (std::size_t i = 0; i < times.size(); ++i) eta(row, columnsOfOrder[row][i]) = times[i];
    }

    // Stage 3: assign.
    const Matching matching = strategy == MatchingStrategy::Optimal ? matchOptimal(eta) : matchGreedy(eta);

    DispatchPlan plan;
    for (std::size_t row = 0; row < orders.size(); ++row) {
        if (const auto col = matching[row]) {
            plan.assignments.push_back({orders[row].id, riders[*col].id, eta(row, *col)});
        } else {
            plan.unassigned.push_back(orders[row].id);
        }
    }
    return plan;
}

std::vector<Rider> Dispatcher::shortlist(const Order& order) const {
    return fleet_.nearby(order.pickup, config_.searchRadiusKm, config_.maxCandidatesPerOrder);
}

}  // namespace waypoint::dispatch
