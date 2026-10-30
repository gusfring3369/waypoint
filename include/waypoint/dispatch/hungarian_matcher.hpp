#pragma once

#include <string_view>

#include "waypoint/dispatch/matcher.hpp"

namespace waypoint::dispatch {

/// The best possible pairing for the whole batch, found with the Hungarian
/// algorithm (Kuhn-Munkres), O(n^2 m) for n = min(rows, cols).
///
/// "Best" means: serve as many orders as possible, and among those pairings
/// pick the one with the smallest total pickup time. Costs must be >= 0;
/// otherwise std::invalid_argument is thrown.
class HungarianMatcher final : public Matcher {
public:
    [[nodiscard]] Matching match(const CostMatrix& costs) const override;
    [[nodiscard]] std::string_view name() const noexcept override { return "Optimal (Hungarian)"; }
};

}  // namespace waypoint::dispatch
