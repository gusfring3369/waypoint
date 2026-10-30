#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

#include "test_framework.hpp"
#include "waypoint/graph/road_network.hpp"
#include "waypoint/graph/road_network_builder.hpp"
#include "waypoint/sim/city_generator.hpp"

using namespace waypoint;
using graph::Direction;
using graph::RoadNetworkBuilder;

namespace {

std::vector<NodeId> neighbors(const graph::RoadNetwork& network, NodeId node, Direction direction) {
    std::vector<NodeId> result;
    for (const auto& arc : network.arcs(node, direction)) result.push_back(arc.neighbor);
    return result;
}

}  // namespace

TEST_CASE("builder rejects roads that would break routing invariants") {
    RoadNetworkBuilder builder;
    const NodeId a = builder.addIntersection({0.0, 0.0});
    const NodeId b = builder.addIntersection({3.0, 4.0});  // 5 km apart

    CHECK_THROWS_AS(builder.addRoad(a, 7), std::invalid_argument);
    CHECK_THROWS_AS(builder.addRoad(a, a), std::invalid_argument);
    CHECK_THROWS_AS(builder.addRoad(a, b, {.speedKmh = 0.0}), std::invalid_argument);
    CHECK_THROWS_AS(builder.addRoad(a, b, {.trafficMultiplier = 0.9}), std::invalid_argument);
    CHECK_THROWS_AS(builder.addRoad(a, b, {.lengthKm = 4.9}), std::invalid_argument);
    CHECK_THROWS_AS(builder.addIntersection({std::nan(""), 0.0}), std::invalid_argument);

    builder.addRoad(a, b, {.lengthKm = 5.0});  // exactly straight is fine
    builder.addRoad(b, a, {.lengthKm = 7.5});  // a winding road is fine
}

TEST_CASE("network exposes forward and reverse adjacency") {
    RoadNetworkBuilder builder;
    const NodeId a = builder.addIntersection({0.0, 0.0});
    const NodeId b = builder.addIntersection({1.0, 0.0});
    const NodeId c = builder.addIntersection({1.0, 1.0});
    builder.addRoad(a, b);
    builder.addRoad(a, c);
    builder.addTwoWayRoad(b, c);
    const auto network = std::move(builder).build();

    CHECK(network.nodeCount() == 3);
    CHECK(network.edgeCount() == 4);
    CHECK(neighbors(network, a, Direction::Forward) == (std::vector<NodeId>{b, c}));
    CHECK(neighbors(network, a, Direction::Reverse).empty());
    CHECK(neighbors(network, c, Direction::Reverse) == (std::vector<NodeId>{a, b}));
    CHECK(network.bounds().min == (geo::Point{0.0, 0.0}));
    CHECK(network.bounds().max == (geo::Point{1.0, 1.0}));
}

TEST_CASE("travel times include speed and live traffic") {
    RoadNetworkBuilder builder;
    const NodeId a = builder.addIntersection({0.0, 0.0});
    const NodeId b = builder.addIntersection({1.0, 0.0});
    const EdgeId road = builder.addRoad(a, b, {.speedKmh = 60.0, .trafficMultiplier = 1.5});
    auto network = std::move(builder).build();

    CHECK_APPROX(network.travelTime(road), 90.0);  // 1 km at 60 km/h = 60 s, x1.5 traffic
    network.setTrafficMultiplier(road, 3.0);
    CHECK_APPROX(network.travelTime(road), 180.0);
    CHECK_THROWS_AS(network.setTrafficMultiplier(road, 0.5), std::invalid_argument);
    CHECK_APPROX(network.lowerBoundTravelTime({0.0, 0.0}, {1.0, 0.0}), 60.0);
}

TEST_CASE("lower bound never exceeds any road's travel time on a generated city") {
    const auto network = sim::generateCity({.rows = 40, .cols = 40, .seed = 3});
    for (EdgeId e = 0; e < network.edgeCount(); ++e) {
        const auto& road = network.road(e);
        const Seconds bound =
            network.lowerBoundTravelTime(network.position(road.from), network.position(road.to));
        CHECK(bound <= network.travelTime(e) + 1e-9);
    }
}

TEST_CASE("generated city is deterministic for a given seed") {
    const auto first = sim::generateCity({.rows = 20, .cols = 20, .seed = 99});
    const auto second = sim::generateCity({.rows = 20, .cols = 20, .seed = 99});
    REQUIRE(first.edgeCount() == second.edgeCount());
    for (EdgeId e = 0; e < first.edgeCount(); ++e) {
        CHECK(first.travelTime(e) == second.travelTime(e));
    }
}
