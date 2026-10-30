#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <stdexcept>

#include "test_framework.hpp"
#include "waypoint/dispatch/matching.hpp"

using namespace waypoint;
using namespace waypoint::dispatch;

namespace {

struct Score {
    std::size_t matched = 0;
    double cost = 0.0;
};

// Exhaustive search: most rows matched first, then least total cost.
void bruteForce(const CostMatrix& costs, std::size_t row, std::vector<bool>& used, Score current,
                Score& best) {
    if (row == costs.rows()) {
        if (current.matched > best.matched || (current.matched == best.matched && current.cost < best.cost)) {
            best = current;
        }
        return;
    }
    bruteForce(costs, row + 1, used, current, best);  // leave this row unmatched
    for (std::size_t col = 0; col < costs.cols(); ++col) {
        if (used[col] || costs(row, col) == kUnreachable) continue;
        used[col] = true;
        bruteForce(costs, row + 1, used, {current.matched + 1, current.cost + costs(row, col)}, best);
        used[col] = false;
    }
}

Score optimum(const CostMatrix& costs) {
    std::vector<bool> used(costs.cols(), false);
    Score best;
    bruteForce(costs, 0, used, {}, best);
    return best;
}

bool isValid(const CostMatrix& costs, const Matching& matching) {
    std::set<std::size_t> seen;
    for (std::size_t row = 0; row < matching.size(); ++row) {
        if (!matching[row]) continue;
        if (costs(row, *matching[row]) == kUnreachable) return false;
        if (!seen.insert(*matching[row]).second) return false;
    }
    return true;
}

std::size_t matchedCount(const Matching& matching) {
    return static_cast<std::size_t>(
        std::ranges::count_if(matching, [](const auto& m) { return m.has_value(); }));
}

}  // namespace

TEST_CASE("greedy matching is locally optimal but globally wasteful") {
    // Order 0 grabs rider 0, leaving order 1 a 100 s pickup.
    CostMatrix costs(2, 2);
    costs(0, 0) = 1.0;
    costs(0, 1) = 2.0;
    costs(1, 0) = 2.0;
    costs(1, 1) = 100.0;

    CHECK_APPROX(totalCost(costs, matchGreedy(costs)), 101.0);
    CHECK_APPROX(totalCost(costs, matchOptimal(costs)), 4.0);
}

TEST_CASE("optimal matching serves the most orders before minimising cost") {
    // Order 1 can only use rider 0, but greedy hands rider 0 to order 0 first.
    CostMatrix costs(2, 2);
    costs(0, 0) = 1.0;
    costs(0, 1) = 50.0;
    costs(1, 0) = 5.0;

    const Matching greedy = matchGreedy(costs);
    const Matching optimal = matchOptimal(costs);
    CHECK(matchedCount(greedy) == 1);
    CHECK(matchedCount(optimal) == 2);
    CHECK(optimal[0] == std::optional<std::size_t>{1});
    CHECK(optimal[1] == std::optional<std::size_t>{0});
}

TEST_CASE("optimal matching agrees with exhaustive search on random instances") {
    std::mt19937 rng(17);
    std::uniform_int_distribution<std::size_t> dimension(1, 6);
    std::uniform_real_distribution<double> cost(0.0, 1000.0);
    std::bernoulli_distribution forbidden(0.3);

    for (int trial = 0; trial < 300; ++trial) {
        CostMatrix costs(dimension(rng), dimension(rng));
        for (std::size_t r = 0; r < costs.rows(); ++r) {
            for (std::size_t c = 0; c < costs.cols(); ++c) {
                costs(r, c) = forbidden(rng) ? kUnreachable : cost(rng);
            }
        }

        const Matching optimal = matchOptimal(costs);
        const Matching greedy = matchGreedy(costs);
        REQUIRE(isValid(costs, optimal));
        REQUIRE(isValid(costs, greedy));

        const Score expected = optimum(costs);
        CHECK(matchedCount(optimal) == expected.matched);
        CHECK(std::fabs(totalCost(costs, optimal) - expected.cost) < 1e-6);
        CHECK(matchedCount(greedy) <= expected.matched);
    }
}

TEST_CASE("matching handles empty and invalid input") {
    CHECK(matchOptimal(CostMatrix(0, 3)).empty());
    CHECK(matchOptimal(CostMatrix(2, 0)) == Matching(2));
    CHECK(matchOptimal(CostMatrix(2, 2)) == Matching(2));  // everything forbidden

    CostMatrix negative(1, 1);
    negative(0, 0) = -1.0;
    CHECK_THROWS_AS(matchOptimal(negative), std::invalid_argument);
}
