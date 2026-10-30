#pragma once

#include <string_view>

#include "waypoint/dispatch/matcher.hpp"

namespace waypoint::dispatch {

/// First come, first served: orders are handled in order, and each takes its
/// fastest rider who is still free.
///
/// Simple and fast, O(rows x cols), but short-sighted: an early order can take
/// the only rider a later order could have used.
class GreedyMatcher final : public Matcher {
public:
    [[nodiscard]] Matching match(const CostMatrix& costs) const override;
    [[nodiscard]] std::string_view name() const noexcept override {
        return "Greedy (first-come-first-served)";
    }
};

}  // namespace waypoint::dispatch
