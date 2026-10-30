#include <random>
#include <set>
#include <vector>

#include "test_framework.hpp"
#include "waypoint/dispatch/dispatcher.hpp"
#include "waypoint/dispatch/fleet_index.hpp"
#include "waypoint/sim/city_generator.hpp"
#include "waypoint/spatial/node_locator.hpp"

using namespace waypoint;
using namespace waypoint::dispatch;

namespace {

// Two river banks 0.3 km apart, joined by a single bridge 5 km east.
//
//   N0 ------------------------------ N5
//   ~~~~~~~~~~~~~~~~ river ~~~~~~~~~~~ |   (bridge)
//   S0 ---- S1 ---------------------- S5
struct RiverTown {
    graph::RoadNetwork network;
    NodeId s0, s1, n0;
};

RiverTown makeRiverTown() {
    graph::RoadNetworkBuilder builder;
    const NodeId s0 = builder.addIntersection({0.0, 0.0});
    const NodeId s1 = builder.addIntersection({1.0, 0.0});
    const NodeId s5 = builder.addIntersection({5.0, 0.0});
    const NodeId n0 = builder.addIntersection({0.0, 0.3});
    const NodeId n5 = builder.addIntersection({5.0, 0.3});
    builder.addTwoWayRoad(s0, s1);
    builder.addTwoWayRoad(s1, s5);
    builder.addTwoWayRoad(s5, n5);
    builder.addTwoWayRoad(n5, n0);
    return {std::move(builder).build(), s0, s1, n0};
}

const geo::BoundingBox kTownArea{{-1.0, -1.0}, {6.0, 1.0}};

}  // namespace

TEST_CASE("dispatch prefers the rider with the fastest road route, not the closest one") {
    const RiverTown town = makeRiverTown();
    FleetIndex fleet(kTownArea);
    REQUIRE(fleet.upsert({.id = 1, .position = {0.0, 0.3}, .node = town.n0}));  // 0.3 km, across the river
    REQUIRE(fleet.upsert({.id = 2, .position = {1.0, 0.0}, .node = town.s1}));  // 1 km, same bank

    Dispatcher dispatcher(town.network, fleet, {.searchRadiusKm = 2.0});
    const auto assignment = dispatcher.dispatch({.id = 7, .pickup = {0.0, 0.0}, .pickupNode = town.s0});
    REQUIRE(assignment.has_value());
    CHECK(assignment->order == 7);
    CHECK(assignment->rider == 2);
    CHECK_APPROX(assignment->pickupEta, 120.0);  // 1 km at the default 30 km/h
}

TEST_CASE("dispatch ignores riders outside the search radius") {
    const RiverTown town = makeRiverTown();
    FleetIndex fleet(kTownArea);
    REQUIRE(fleet.upsert({.id = 1, .position = {1.0, 0.0}, .node = town.s1}));

    Dispatcher dispatcher(town.network, fleet, {.searchRadiusKm = 0.5});
    CHECK(!dispatcher.dispatch({.id = 1, .pickup = {0.0, 0.0}, .pickupNode = town.s0}).has_value());
}

TEST_CASE("fleet index applies location updates and drops riders leaving the area") {
    FleetIndex fleet(kTownArea);
    REQUIRE(fleet.upsert({.id = 1, .position = {0.0, 0.0}, .node = 0}));
    REQUIRE(fleet.upsert({.id = 1, .position = {5.0, 0.0}, .node = 2}));  // moved
    CHECK(fleet.size() == 1);
    CHECK(fleet.nearby({0.0, 0.0}, 1.0, 10).empty());
    CHECK(fleet.nearby({5.0, 0.0}, 1.0, 10).size() == 1);

    CHECK(!fleet.upsert({.id = 1, .position = {50.0, 0.0}, .node = 2}));  // left the service area
    CHECK(fleet.find(1) == nullptr);
    CHECK(!fleet.remove(1));
}

TEST_CASE("node locator snaps to the nearest intersection") {
    const auto city = sim::generateCity({.rows = 30, .cols = 30, .seed = 9});
    const spatial::NodeLocator locator(city);
    std::mt19937 rng(10);
    std::uniform_real_distribution<double> coordinate(-0.5, 6.5);
    for (int i = 0; i < 100; ++i) {
        const geo::Point query{coordinate(rng), coordinate(rng)};
        double best = kUnreachable;
        for (NodeId node = 0; node < city.nodeCount(); ++node) {
            best = std::min(best, geo::euclideanDistance(city.position(node), query));
        }
        const auto snapped = locator.nearestNode(query);
        REQUIRE(snapped.has_value());
        CHECK_APPROX(geo::euclideanDistance(city.position(*snapped), query), best);
    }
}

TEST_CASE("optimal batch dispatch never does worse than greedy") {
    const auto city = sim::generateCity({.rows = 50, .cols = 50, .seed = 12});
    const spatial::NodeLocator locator(city);
    std::mt19937 rng(13);
    std::uniform_real_distribution<double> coordinate(0.0, 9.8);

    for (int round = 0; round < 20; ++round) {
        FleetIndex fleet(city.bounds());
        for (RiderId id = 0; id < 25; ++id) {
            const geo::Point position{coordinate(rng), coordinate(rng)};
            fleet.upsert({id, position, *locator.nearestNode(position)});
        }
        std::vector<Order> orders;
        for (OrderId id = 0; id < 15; ++id) {
            const geo::Point pickup{coordinate(rng), coordinate(rng)};
            orders.push_back({id, pickup, *locator.nearestNode(pickup)});
        }

        Dispatcher dispatcher(city, fleet, {.searchRadiusKm = 3.0, .maxCandidatesPerOrder = 8});
        const auto greedy = dispatcher.dispatchBatch(orders, MatchingStrategy::Greedy);
        const auto optimal = dispatcher.dispatchBatch(orders, MatchingStrategy::Optimal);

        CHECK(optimal.assignments.size() + optimal.unassigned.size() == orders.size());
        CHECK(optimal.assignments.size() >= greedy.assignments.size());
        if (optimal.assignments.size() == greedy.assignments.size()) {
            CHECK(optimal.totalPickupEta() <= greedy.totalPickupEta() + 1e-6);
        }

        std::set<RiderId> riders;
        for (const auto& assignment : optimal.assignments) {
            CHECK(riders.insert(assignment.rider).second);  // each rider used at most once
        }
    }
}
