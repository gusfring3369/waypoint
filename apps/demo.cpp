// A guided tour of the engine on a generated city:
//   1. route planning with Dijkstra and A*
//   2. dispatching one order when a river is in the way
//   3. re-routing around live traffic
//   4. greedy vs optimal matching for a batch of orders

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "waypoint/dispatch/dispatcher.hpp"
#include "waypoint/dispatch/fleet_index.hpp"
#include "waypoint/dispatch/greedy_matcher.hpp"
#include "waypoint/dispatch/hungarian_matcher.hpp"
#include "waypoint/routing/shortest_path_engine.hpp"
#include "waypoint/sim/city_generator.hpp"
#include "waypoint/spatial/node_locator.hpp"

// Every library type lives in namespace `waypoint`. Pulling it in here, in an
// application .cpp file, keeps the demo short; library headers never do this.
using namespace waypoint;

namespace {

/// 125.4 seconds -> "2:05"
std::string formatEta(Seconds seconds) {
    const long total = std::lround(seconds);
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%ld:%02ld", total / 60, total % 60);
    return buffer;
}

void printHeading(const char* title) {
    std::printf("\n=== %s ===\n", title);
}

void compareSearchAlgorithms(const graph::RoadNetwork& city, NodeId from, NodeId to) {
    printHeading("1. Route planning: Dijkstra vs A*");
    routing::ShortestPathEngine engine(city);
    const routing::Route dijkstra = engine.dijkstra(from, to);
    const routing::Route aStar = engine.aStar(from, to);

    const double saved = 1.0 - static_cast<double>(aStar.stats.nodesSettled) /
                                   static_cast<double>(dijkstra.stats.nodesSettled);
    std::printf("  Dijkstra : ETA %s, %zu nodes settled\n", formatEta(dijkstra.travelTime).c_str(),
                dijkstra.stats.nodesSettled);
    std::printf("  A*       : ETA %s, %zu nodes settled (%.0f%% fewer, same route cost)\n",
                formatEta(aStar.travelTime).c_str(), aStar.stats.nodesSettled, 100.0 * saved);
    std::printf("  Route passes through %zu intersections\n", aStar.path.size());
}

void dispatchAcrossRiver(const graph::RoadNetwork& city, const sim::CityConfig& config) {
    printHeading("2. Single dispatch: straight-line nearest is not road nearest");

    // Restaurant on the south bank, halfway between two bridges.
    const std::size_t northBank = sim::firstRowNorthOfRiver(config);
    const std::size_t column = config.bridgeEvery + config.bridgeEvery / 2;
    const NodeId restaurant = sim::intersectionAt(config, northBank - 1, column);
    const NodeId acrossTheWater = sim::intersectionAt(config, northBank, column);
    const NodeId sameBank = sim::intersectionAt(config, northBank - 4, column);

    const std::vector<dispatch::Rider> riders = {
        {101, city.position(acrossTheWater), acrossTheWater},
        {102, city.position(sameBank), sameBank},
    };
    dispatch::FleetIndex fleet(city.bounds());
    for (const dispatch::Rider& rider : riders) fleet.upsert(rider);

    routing::ShortestPathEngine engine(city);
    const geo::Point pickup = city.position(restaurant);
    for (const dispatch::Rider& rider : riders) {
        std::printf("  Rider #%u: %.2f km away in a straight line, ETA by road %s\n", rider.id,
                    geo::euclideanDistance(rider.position, pickup),
                    formatEta(engine.aStar(rider.node, restaurant).travelTime).c_str());
    }

    dispatch::Dispatcher dispatcher(city, fleet, {.searchRadiusKm = 2.0});
    if (const auto assignment = dispatcher.dispatch({1, pickup, restaurant})) {
        std::printf("  -> Dispatched rider #%u (pickup ETA %s)\n", assignment->rider,
                    formatEta(assignment->pickupEta).c_str());
    }
}

void rerouteAroundTraffic(graph::RoadNetwork& city, NodeId from, NodeId to) {
    printHeading("3. Live traffic: congestion on the current route");
    routing::ShortestPathEngine engine(city);
    const routing::Route before = engine.aStar(from, to);
    std::printf("  Before : ETA %s\n", formatEta(before.travelTime).c_str());

    // A traffic jam (4x slower) on every road of the middle third of the route.
    const std::size_t jamStart = before.path.size() / 3;
    const std::size_t jamEnd = 2 * before.path.size() / 3;
    for (std::size_t i = jamStart; i < jamEnd; ++i) {
        for (const graph::Arc& arc : city.arcs(before.path[i], graph::Direction::Forward)) {
            if (arc.neighbor == before.path[i + 1]) city.setTrafficMultiplier(arc.edge, 4.0);
        }
    }

    const routing::Route after = engine.aStar(from, to);
    const auto changed = std::ranges::count_if(
        after.path, [&](NodeId node) { return std::ranges::find(before.path, node) == before.path.end(); });
    std::printf("  After  : ETA %s, rerouted (%td of %zu intersections differ from the old route)\n",
                formatEta(after.travelTime).c_str(), changed, after.path.size());
}

void compareBatchMatching(const graph::RoadNetwork& city) {
    printHeading("4. Batch dispatch: greedy vs optimal matching");
    const spatial::NodeLocator locator(city);
    const auto zone = geo::BoundingBox::around(city.bounds().center(), 1.0, 1.0);
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> x(zone.min.x, zone.max.x);
    std::uniform_real_distribution<double> y(zone.min.y, zone.max.y);

    dispatch::FleetIndex fleet(city.bounds());
    for (RiderId id = 1; id <= 10; ++id) {
        const geo::Point position{x(rng), y(rng)};
        fleet.upsert({id, position, *locator.nearestNode(position)});
    }
    std::vector<dispatch::Order> orders;
    for (OrderId id = 1; id <= 8; ++id) {
        const geo::Point pickup{x(rng), y(rng)};
        orders.push_back({id, pickup, *locator.nearestNode(pickup)});
    }
    std::printf("  %zu orders, %zu riders in a 2 x 2 km zone\n", orders.size(), fleet.size());

    dispatch::Dispatcher dispatcher(city, fleet, {.searchRadiusKm = 2.0, .maxCandidatesPerOrder = 6});
    const dispatch::GreedyMatcher greedy;
    const dispatch::HungarianMatcher optimal;
    const dispatch::Matcher* matchers[] = {&greedy, &optimal};  // same interface, different policy
    for (const dispatch::Matcher* matcher : matchers) {
        const dispatch::DispatchPlan plan = dispatcher.dispatchBatch(orders, *matcher);
        std::printf("  %-34s %zu served, total pickup time %s\n", std::string(matcher->name()).c_str(),
                    plan.assignments.size(), formatEta(plan.totalPickupEta()).c_str());
    }
}

}  // namespace

int main() {
    const sim::CityConfig config{.rows = 80, .cols = 80, .blockKm = 0.15, .seed = 11};
    graph::RoadNetwork city = sim::generateCity(config);
    std::printf("Generated city: %zu intersections, %zu road segments, %.0f x %.0f km\n", city.nodeCount(),
                city.edgeCount(), city.bounds().max.x, city.bounds().max.y);

    // A cross-town trip on the south bank of the river.
    const NodeId from = sim::intersectionAt(config, 8, 6);
    const NodeId to = sim::intersectionAt(config, 30, 66);

    compareSearchAlgorithms(city, from, to);
    dispatchAcrossRiver(city, config);
    rerouteAroundTraffic(city, from, to);
    compareBatchMatching(city);
    return 0;
}
