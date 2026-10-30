// Walks through the engine end to end on a generated city:
// routing, rider dispatch across a river, live traffic, and batch matching.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "waypoint/dispatch/dispatcher.hpp"
#include "waypoint/dispatch/fleet_index.hpp"
#include "waypoint/routing/shortest_path_engine.hpp"
#include "waypoint/sim/city_generator.hpp"
#include "waypoint/spatial/node_locator.hpp"

using namespace waypoint;

namespace {

std::string formatEta(Seconds seconds) {
    const long total = std::lround(seconds);
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%ld:%02ld", total / 60, total % 60);
    return buffer;
}

void printHeading(const char* title) {
    std::printf("\n=== %s ===\n", title);
}

void compareSearches(const graph::RoadNetwork& city, NodeId from, NodeId to) {
    printHeading("1. Route planning: Dijkstra vs A*");
    routing::ShortestPathEngine engine(city);
    const auto dijkstra = engine.dijkstra(from, to);
    const auto aStar = engine.aStar(from, to);
    std::printf("  Dijkstra : ETA %s, %zu nodes settled\n", formatEta(dijkstra.travelTime).c_str(),
                dijkstra.stats.nodesSettled);
    std::printf("  A*       : ETA %s, %zu nodes settled (%.0f%% fewer, same route cost)\n",
                formatEta(aStar.travelTime).c_str(), aStar.stats.nodesSettled,
                100.0 * (1.0 - static_cast<double>(aStar.stats.nodesSettled) /
                                   static_cast<double>(dijkstra.stats.nodesSettled)));
    std::printf("  Route passes through %zu intersections\n", aStar.path.size());
}

void dispatchAcrossRiver(const graph::RoadNetwork& city, const sim::CityConfig& config) {
    printHeading("2. Single dispatch: straight-line nearest is not road nearest");
    // The river runs between rows riverRow - 1 and riverRow, with bridges every
    // `bridgeEvery` columns. Put the restaurant midway between two bridges.
    const std::size_t riverRow = config.rows / 2;
    const std::size_t column = config.bridgeEvery + config.bridgeEvery / 2;
    const auto nodeAt = [&](std::size_t r, std::size_t c) {
        return static_cast<NodeId>(r * config.cols + c);
    };

    const NodeId restaurant = nodeAt(riverRow - 1, column);
    const std::vector<dispatch::Rider> riders = {
        {101, city.position(nodeAt(riverRow, column)), nodeAt(riverRow, column)},          // across the water
        {102, city.position(nodeAt(riverRow - 4, column)), nodeAt(riverRow - 4, column)},  // same bank
    };

    dispatch::FleetIndex fleet(city.bounds());
    for (const auto& rider : riders) fleet.upsert(rider);

    routing::ShortestPathEngine engine(city);
    const geo::Point pickup = city.position(restaurant);
    for (const auto& rider : riders) {
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

void liveTraffic(graph::RoadNetwork& city, NodeId from, NodeId to) {
    printHeading("3. Live traffic: congestion on the current route");
    routing::ShortestPathEngine engine(city);
    const auto before = engine.aStar(from, to);
    std::printf("  Before : ETA %s\n", formatEta(before.travelTime).c_str());

    // A jam on every road of the middle third of the route.
    const std::size_t begin = before.path.size() / 3;
    const std::size_t end = 2 * before.path.size() / 3;
    for (std::size_t i = begin; i < end; ++i) {
        for (const auto& arc : city.arcs(before.path[i], graph::Direction::Forward)) {
            if (arc.neighbor == before.path[i + 1]) city.setTrafficMultiplier(arc.edge, 4.0);
        }
    }

    const auto after = engine.aStar(from, to);
    std::size_t shared = 0;
    for (const NodeId node : after.path) shared += std::ranges::count(before.path, node) > 0 ? 1U : 0U;
    std::printf("  After  : ETA %s, rerouted (%zu of %zu intersections differ from the old route)\n",
                formatEta(after.travelTime).c_str(), after.path.size() - shared, after.path.size());
}

void batchDispatch(const graph::RoadNetwork& city) {
    printHeading("4. Batch dispatch: greedy vs optimal matching");
    const spatial::NodeLocator locator(city);
    const auto zone = geo::BoundingBox::around(city.bounds().center(), 1.0, 1.0);
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> x(zone.min.x, zone.max.x);
    std::uniform_real_distribution<double> y(zone.min.y, zone.max.y);

    dispatch::FleetIndex fleet(city.bounds());
    for (RiderId id = 1; id <= 10; ++id) {
        const geo::Point p{x(rng), y(rng)};
        fleet.upsert({id, p, *locator.nearestNode(p)});
    }
    std::vector<dispatch::Order> orders;
    for (OrderId id = 1; id <= 8; ++id) {
        const geo::Point p{x(rng), y(rng)};
        orders.push_back({id, p, *locator.nearestNode(p)});
    }

    dispatch::Dispatcher dispatcher(city, fleet, {.searchRadiusKm = 2.0, .maxCandidatesPerOrder = 6});
    const auto greedy = dispatcher.dispatchBatch(orders, dispatch::MatchingStrategy::Greedy);
    const auto optimal = dispatcher.dispatchBatch(orders, dispatch::MatchingStrategy::Optimal);

    std::printf("  %zu orders, %zu riders in a 2 x 2 km zone\n", orders.size(), fleet.size());
    std::printf("  Greedy : %zu served, total pickup time %s\n", greedy.assignments.size(),
                formatEta(greedy.totalPickupEta()).c_str());
    std::printf("  Optimal: %zu served, total pickup time %s\n", optimal.assignments.size(),
                formatEta(optimal.totalPickupEta()).c_str());
    for (const auto& a : optimal.assignments) {
        std::printf("    order %u <- rider #%u (%s)\n", a.order, a.rider, formatEta(a.pickupEta).c_str());
    }
}

}  // namespace

int main() {
    const sim::CityConfig config{.rows = 80, .cols = 80, .blockKm = 0.15, .seed = 11};
    auto city = sim::generateCity(config);
    std::printf("Generated city: %zu intersections, %zu road segments, %.0f x %.0f km\n", city.nodeCount(),
                city.edgeCount(), city.bounds().max.x, city.bounds().max.y);

    // A cross-town trip on the south bank of the river.
    const NodeId from = static_cast<NodeId>(8 * config.cols + 6);
    const NodeId to = static_cast<NodeId>(30 * config.cols + 66);

    compareSearches(city, from, to);
    dispatchAcrossRiver(city, config);
    liveTraffic(city, from, to);
    batchDispatch(city);
    return 0;
}
