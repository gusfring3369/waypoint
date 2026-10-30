#pragma once

#include <cstddef>
#include <optional>
#include <string_view>
#include <vector>

#include "waypoint/dispatch/cost_matrix.hpp"

namespace waypoint::dispatch {

/// For each row (order), the column (rider) it was given, if any.
/// No column appears twice: a rider takes at most one order.
using Matching = std::vector<std::optional<std::size_t>>;

/// A policy for pairing orders with riders (Strategy pattern).
///
/// The Dispatcher only knows this interface, so a new policy, such as one that
/// prioritises premium customers, can be added without changing the Dispatcher.
class Matcher {
public:
    virtual ~Matcher() = default;

    [[nodiscard]] virtual Matching match(const CostMatrix& costs) const = 0;
    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

/// Sum of the costs of the matched pairs.
[[nodiscard]] double totalCost(const CostMatrix& costs, const Matching& matching);

}  // namespace waypoint::dispatch
