#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <stdexcept>

#include "test_framework.hpp"
#include "waypoint/dispatch/greedy_matcher.hpp"
#include "waypoint/dispatch/hungarian_matcher.hpp"

using namespace waypoint;
using namespace waypoint::dispatch;

namespace {

struct Score {
    std::size_t matched = 0;
    double cost = 0.0;

    [[nodiscard]] bool betterThan(const Score& other) const {
        return matched > other.matched || (matched == other.matched && cost < other.cost);
    }
};

// Reference answer by trying every possible matching (only for tiny matrices).
class BruteForceMatcher {
public:
    explicit BruteForceMatcher(const CostMatrix& costs) : costs_(costs), used_(costs.cols(), false) {}

    Score best() {
        tryRow(0, {});
        return best_;
    }

private:
    void tryRow(std::size_t row, Score current) {
        if (row == costs_.rows()) {
            if (current.betterThan(best_)) best_ = current;
            return;
        }
        tryRow(row + 1, current);  // leave this row unmatched
        for (std::size_t col = 0; col < costs_.cols(); ++col) {
            if (used_[col] || !costs_.isAllowed(row, col)) continue;
            used_[col] = true;
            tryRow(row + 1, {current.matched + 1, current.cost + costs_(row, col)});
            used_[col] = false;
        }
    }

    const CostMatrix& costs_;
    std::vector<bool> used_;
    Score best_;
};

bool isValid(const CostMatrix& costs, const Matching& matching) {
    std::set<std::size_t> usedColumns;
    for (std::size_t row = 0; row < matching.size(); ++row) {
        if (!matching[row]) continue;
        if (!costs.isAllowed(row, *matching[row])) return false;
        if (!usedColumns.insert(*matching[row]).second) return false;
    }
    return true;
}

std::size_t matchedCount(const Matching& matching) {
    return static_cast<std::size_t>(
        std::ranges::count_if(matching, [](const auto& m) { return m.has_value(); }));
}

CostMatrix randomMatrix(std::mt19937& rng) {
    std::uniform_int_distribution<std::size_t> dimension(1, 6);
    std::uniform_real_distribution<double> cost(0.0, 1000.0);
    std::bernoulli_distribution forbidden(0.3);

    CostMatrix costs(dimension(rng), dimension(rng));
    for (std::size_t r = 0; r < costs.rows(); ++r) {
        for (std::size_t c = 0; c < costs.cols(); ++c)
            costs(r, c) = forbidden(rng) ? kUnreachable : cost(rng);
    }
    return costs;
}

}  // namespace

TEST_CASE("greedy matching is locally optimal but globally wasteful") {
    // Order 0 grabs rider 0, leaving order 1 a 100 s pickup.
    CostMatrix costs(2, 2);
    costs(0, 0) = 1.0;
    costs(0, 1) = 2.0;
    costs(1, 0) = 2.0;
    costs(1, 1) = 100.0;

    CHECK_APPROX(totalCost(costs, GreedyMatcher{}.match(costs)), 101.0);
    CHECK_APPROX(totalCost(costs, HungarianMatcher{}.match(costs)), 4.0);
}

TEST_CASE("optimal matching serves the most orders before minimising cost") {
    // Order 1 can only use rider 0, but greedy hands rider 0 to order 0 first.
    CostMatrix costs(2, 2);
    costs(0, 0) = 1.0;
    costs(0, 1) = 50.0;
    costs(1, 0) = 5.0;

    const Matching greedy = GreedyMatcher{}.match(costs);
    const Matching optimal = HungarianMatcher{}.match(costs);
    CHECK(matchedCount(greedy) == 1);
    CHECK(matchedCount(optimal) == 2);
    CHECK(optimal[0] == std::optional<std::size_t>{1});
    CHECK(optimal[1] == std::optional<std::size_t>{0});
}

TEST_CASE("optimal matching agrees with exhaustive search on random instances") {
    std::mt19937 rng(17);
    const GreedyMatcher greedyMatcher;
    const HungarianMatcher optimalMatcher;

    for (int trial = 0; trial < 300; ++trial) {
        const CostMatrix costs = randomMatrix(rng);
        const Matching optimal = optimalMatcher.match(costs);
        const Matching greedy = greedyMatcher.match(costs);
        REQUIRE(isValid(costs, optimal));
        REQUIRE(isValid(costs, greedy));

        const Score expected = BruteForceMatcher(costs).best();
        CHECK(matchedCount(optimal) == expected.matched);
        CHECK(std::fabs(totalCost(costs, optimal) - expected.cost) < 1e-6);
        CHECK(matchedCount(greedy) <= expected.matched);
    }
}

TEST_CASE("matchers are interchangeable through the Matcher interface") {
    CostMatrix costs(1, 1);
    costs(0, 0) = 3.0;
    const GreedyMatcher greedy;
    const HungarianMatcher optimal;
    const Matcher* matchers[] = {&greedy, &optimal};
    for (const Matcher* matcher : matchers) {
        CHECK(matcher->match(costs) == Matching{0});
        CHECK(!matcher->name().empty());
    }
}

TEST_CASE("hungarian matcher handles empty and invalid input") {
    const HungarianMatcher matcher;
    CHECK(matcher.match(CostMatrix(0, 3)).empty());
    CHECK(matcher.match(CostMatrix(2, 0)) == Matching(2));
    CHECK(matcher.match(CostMatrix(2, 2)) == Matching(2));  // every pair forbidden

    CostMatrix negative(1, 1);
    negative(0, 0) = -1.0;
    CHECK_THROWS_AS(matcher.match(negative), std::invalid_argument);
}
