#include "waypoint/dispatch/dispatcher.hpp"

#include <unordered_map>

namespace waypoint::dispatch {

Dispatcher::Dispatcher(const graph::RoadNetwork& network, const FleetIndex& fleet, DispatchConfig config)
    : fleet_(fleet), config_(config), engine_(network) {}

std::optional<Assignment> Dispatcher::dispatch(const Order& order) {
    const std::vector<Rider> candidates = shortlist(order);

    std::vector<NodeId> riderNodes;
    riderNodes.reserve(candidates.size());
    for (const Rider& rider : candidates) riderNodes.push_back(rider.node);

    // One multi-source search prices every candidate at once.
    const auto fastest = engine_.nearestSource(riderNodes, order.pickupNode);
    if (!fastest) return std::nullopt;
    return Assignment{order.id, candidates[fastest->sourceIndex].id, fastest->travelTime};
}

DispatchPlan Dispatcher::dispatchBatch(std::span<const Order> orders, const Matcher& matcher) {
    const BatchShortlist candidates = shortlistBatch(orders);
    const CostMatrix etas = priceShortlist(orders, candidates);
    const Matching matching = matcher.match(etas);
    return toPlan(orders, candidates, etas, matching);
}

std::vector<Rider> Dispatcher::shortlist(const Order& order) const {
    return fleet_.nearby(order.pickup, config_.searchRadiusKm, config_.maxCandidatesPerOrder);
}

Dispatcher::BatchShortlist Dispatcher::shortlistBatch(std::span<const Order> orders) const {
    BatchShortlist result;
    result.candidateColumns.resize(orders.size());
    std::unordered_map<RiderId, std::size_t> columnOfRider;  // a rider near two orders gets one column

    for (std::size_t row = 0; row < orders.size(); ++row) {
        for (const Rider& rider : shortlist(orders[row])) {
            const auto [it, isNew] = columnOfRider.try_emplace(rider.id, result.riders.size());
            if (isNew) result.riders.push_back(rider);
            result.candidateColumns[row].push_back(it->second);
        }
    }
    return result;
}

// One reverse search from each pickup gives every candidate's driving time to it.
CostMatrix Dispatcher::priceShortlist(std::span<const Order> orders, const BatchShortlist& shortlist) {
    CostMatrix etas(orders.size(), shortlist.riders.size());  // pairs not shortlisted stay forbidden
    std::vector<NodeId> riderNodes;

    for (std::size_t row = 0; row < orders.size(); ++row) {
        const auto& columns = shortlist.candidateColumns[row];
        riderNodes.clear();
        for (const std::size_t col : columns) riderNodes.push_back(shortlist.riders[col].node);

        const auto times = engine_.travelTimes(orders[row].pickupNode, riderNodes, graph::Direction::Reverse);
        for (std::size_t i = 0; i < columns.size(); ++i) etas(row, columns[i]) = times[i];
    }
    return etas;
}

DispatchPlan Dispatcher::toPlan(std::span<const Order> orders, const BatchShortlist& shortlist,
                                const CostMatrix& etas, const Matching& matching) {
    DispatchPlan plan;
    for (std::size_t row = 0; row < orders.size(); ++row) {
        if (const auto col = matching[row]) {
            plan.assignments.push_back({orders[row].id, shortlist.riders[*col].id, etas(row, *col)});
        } else {
            plan.unassigned.push_back(orders[row].id);
        }
    }
    return plan;
}

}  // namespace waypoint::dispatch
