#include "waypoint/dispatch/greedy_matcher.hpp"

namespace waypoint::dispatch {

Matching GreedyMatcher::match(const CostMatrix& costs) const {
    Matching matching(costs.rows());
    std::vector<bool> riderTaken(costs.cols(), false);

    for (std::size_t order = 0; order < costs.rows(); ++order) {
        std::optional<std::size_t> fastest;
        for (std::size_t rider = 0; rider < costs.cols(); ++rider) {
            if (riderTaken[rider] || !costs.isAllowed(order, rider)) continue;
            if (!fastest || costs(order, rider) < costs(order, *fastest)) fastest = rider;
        }
        if (fastest) riderTaken[*fastest] = true;
        matching[order] = fastest;
    }
    return matching;
}

}  // namespace waypoint::dispatch
