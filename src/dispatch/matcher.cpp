#include "waypoint/dispatch/matcher.hpp"

namespace waypoint::dispatch {

double totalCost(const CostMatrix& costs, const Matching& matching) {
    double total = 0.0;
    for (std::size_t row = 0; row < matching.size(); ++row) {
        if (const auto col = matching[row]) total += costs(row, *col);
    }
    return total;
}

}  // namespace waypoint::dispatch
