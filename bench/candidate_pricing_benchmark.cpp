// Section 3: three ways to get road ETAs for a shortlist of riders.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "bench_common.hpp"
#include "waypoint/dispatch/fleet_index.hpp"
#include "waypoint/routing/shortest_path_engine.hpp"
#include "waypoint/spatial/node_locator.hpp"

namespace bench {
namespace {

using waypoint::NodeId;
using waypoint::Seconds;

constexpr std::size_t kCandidates = 16;
constexpr double kRadiusKm = 3.0;

struct StrategyTotals {
    double nodesSettled = 0.0;
    double micros = 0.0;
};

void printRow(const char* name, std::size_t searches, const StrategyTotals& totals, double orders,
              const char* output) {
    std::printf("| %s | %zu | %.0f | %.0f us | %s |\n", name, searches, totals.nodesSettled / orders,
                totals.micros / orders, output);
}

}  // namespace

void runCandidatePricingBenchmark(const waypoint::graph::RoadNetwork& city, const Scale& scale) {
    // Most of the fleet is busy at any moment; only free riders are candidates.
    const std::size_t availableRiders = scale.riders / 25;

    const waypoint::spatial::NodeLocator locator(city);
    std::mt19937_64 rng(kSeed + 2);
    waypoint::dispatch::FleetIndex fleet(city.bounds());
    for (waypoint::RiderId id = 0; id < availableRiders; ++id) {
        const auto position = randomPoint(city.bounds(), rng);
        fleet.upsert({id, position, *locator.nearestNode(position)});
    }

    waypoint::routing::ShortestPathEngine engine(city);
    StrategyTotals perCandidate, oneToMany, multiSource;
    std::size_t orders = 0;

    for (std::size_t i = 0; i < scale.routeQueries; ++i) {
        const auto pickup = randomPoint(city.bounds(), rng);
        const NodeId pickupNode = *locator.nearestNode(pickup);
        std::vector<NodeId> riderNodes;
        for (const auto& rider : fleet.nearby(pickup, kRadiusKm, kCandidates))
            riderNodes.push_back(rider.node);
        if (riderNodes.size() < kCandidates) continue;

        // a) One A* search per rider.
        Seconds fastestSeparate = waypoint::kUnreachable;
        const Stopwatch perCandidateTimer;
        for (const NodeId riderNode : riderNodes) {
            const auto route = engine.aStar(riderNode, pickupNode);
            fastestSeparate = std::min(fastestSeparate, route.travelTime);
            perCandidate.nodesSettled += static_cast<double>(route.stats.nodesSettled);
        }
        perCandidate.micros += perCandidateTimer.elapsedMicros();

        // b) One reverse search from the pickup that reaches every rider.
        const Stopwatch oneToManyTimer;
        const auto times = engine.travelTimes(pickupNode, riderNodes, waypoint::graph::Direction::Reverse);
        oneToMany.micros += oneToManyTimer.elapsedMicros();
        oneToMany.nodesSettled += static_cast<double>(engine.lastStats().nodesSettled);

        // c) One search started from all riders at once.
        const Stopwatch multiSourceTimer;
        const auto fastest = engine.nearestSource(riderNodes, pickupNode);
        multiSource.micros += multiSourceTimer.elapsedMicros();
        multiSource.nodesSettled += static_cast<double>(engine.lastStats().nodesSettled);

        const Seconds fastestOneToMany = *std::ranges::min_element(times);
        const Seconds fastestMultiSource = fastest ? fastest->travelTime : waypoint::kUnreachable;
        failIf(std::abs(fastestSeparate - fastestOneToMany) > 1e-6 ||
                   std::abs(fastestSeparate - fastestMultiSource) > 1e-6,
               "pricing strategies disagree on the fastest rider");
        ++orders;
    }

    const auto n = static_cast<double>(orders);
    std::printf("## 3. Pricing a %zu-rider shortlist for one order (%zu available riders, %zu orders)\n\n",
                kCandidates, availableRiders, orders);
    std::printf(
        "| Strategy | Searches | Avg nodes settled | Avg time | Output |\n|---|---:|---:|---:|---|\n");
    printRow("A* per candidate", kCandidates, perCandidate, n, "every ETA");
    printRow("Reverse one-to-many Dijkstra", 1, oneToMany, n, "every ETA");
    printRow("Multi-source A*", 1, multiSource, n, "fastest rider only");
    std::printf(
        "\nMulti-source A* finds the fastest rider settling %.1f%% fewer nodes than one A* per "
        "candidate.\n\n",
        percentLess(perCandidate.nodesSettled, multiSource.nodesSettled));
}

}  // namespace bench
