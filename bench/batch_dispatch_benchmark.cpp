// Section 4: greedy vs Hungarian matching for batches of competing orders.

#include <cstdio>
#include <random>
#include <string_view>
#include <vector>

#include "bench_common.hpp"
#include "waypoint/dispatch/dispatcher.hpp"
#include "waypoint/dispatch/fleet_index.hpp"
#include "waypoint/dispatch/greedy_matcher.hpp"
#include "waypoint/dispatch/hungarian_matcher.hpp"
#include "waypoint/spatial/node_locator.hpp"

namespace bench {
namespace {

using namespace waypoint::dispatch;

constexpr std::size_t kOrdersPerBatch = 30;
constexpr std::size_t kRidersPerBatch = 40;

struct MatcherTotals {
    std::size_t served = 0;
    double totalEta = 0.0;
    double etaInComparableRounds = 0.0;  // rounds where both matchers served equally many orders
};

}  // namespace

void runBatchDispatchBenchmark(const waypoint::graph::RoadNetwork& city, const Scale& scale) {
    // A dense 3 x 3 km neighbourhood so that orders genuinely compete for riders.
    const auto zone = waypoint::geo::BoundingBox::around(city.bounds().center(), 1.5, 1.5);
    const waypoint::spatial::NodeLocator locator(city);
    std::mt19937_64 rng(kSeed + 3);

    const GreedyMatcher greedyMatcher;
    const HungarianMatcher optimalMatcher;
    MatcherTotals greedy, optimal;
    std::size_t comparableRounds = 0;
    double optimalMicros = 0.0;

    for (std::size_t round = 0; round < scale.batchRounds; ++round) {
        FleetIndex fleet(city.bounds());
        for (waypoint::RiderId id = 0; id < kRidersPerBatch; ++id) {
            const auto position = randomPoint(zone, rng);
            fleet.upsert({id, position, *locator.nearestNode(position)});
        }
        std::vector<Order> orders;
        for (waypoint::OrderId id = 0; id < kOrdersPerBatch; ++id) {
            const auto pickup = randomPoint(zone, rng);
            orders.push_back({id, pickup, *locator.nearestNode(pickup)});
        }

        Dispatcher dispatcher(city, fleet, {.searchRadiusKm = 2.0, .maxCandidatesPerOrder = 12});
        const DispatchPlan greedyPlan = dispatcher.dispatchBatch(orders, greedyMatcher);
        const Stopwatch optimalTimer;
        const DispatchPlan optimalPlan = dispatcher.dispatchBatch(orders, optimalMatcher);
        optimalMicros += optimalTimer.elapsedMicros();

        greedy.served += greedyPlan.assignments.size();
        greedy.totalEta += greedyPlan.totalPickupEta();
        optimal.served += optimalPlan.assignments.size();
        optimal.totalEta += optimalPlan.totalPickupEta();
        if (greedyPlan.assignments.size() == optimalPlan.assignments.size()) {
            ++comparableRounds;
            greedy.etaInComparableRounds += greedyPlan.totalPickupEta();
            optimal.etaInComparableRounds += optimalPlan.totalPickupEta();
        }
    }

    const auto rounds = static_cast<double>(scale.batchRounds);
    const double ordersPlaced = rounds * static_cast<double>(kOrdersPerBatch);
    const auto printRow = [&](std::string_view name, const MatcherTotals& totals) {
        std::printf("| %.*s | %.1f%% | %.0f s |\n", static_cast<int>(name.size()), name.data(),
                    100.0 * static_cast<double>(totals.served) / ordersPlaced,
                    totals.totalEta / static_cast<double>(totals.served));
    };

    std::printf("## 4. Batch dispatch (%zu rounds of %zu orders vs %zu riders in a 3 x 3 km zone)\n\n",
                scale.batchRounds, kOrdersPerBatch, kRidersPerBatch);
    std::printf("| Strategy | Orders served | Avg pickup ETA per order |\n|---|---:|---:|\n");
    printRow(greedyMatcher.name(), greedy);
    printRow(optimalMatcher.name(), optimal);
    std::printf(
        "\nIn the %zu rounds where both served the same number of orders, optimal matching cut total "
        "pickup time by %.1f%%. Avg end-to-end batch time (shortlist + price + match): %.2f ms.\n\n",
        comparableRounds, percentLess(greedy.etaInComparableRounds, optimal.etaInComparableRounds),
        optimalMicros / rounds / 1000.0);
}

}  // namespace bench
