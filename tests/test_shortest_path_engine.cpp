#include <algorithm>
#include <random>
#include <stdexcept>
#include <vector>

#include "test_framework.hpp"
#include "waypoint/graph/road_network.hpp"
#include "waypoint/routing/shortest_path_engine.hpp"
#include "waypoint/sim/city_generator.hpp"

using namespace waypoint;
using graph::Direction;
using routing::ShortestPathEngine;

namespace {

// a --slow highway-- d, and a fast detour a -> b -> c -> d.
//
//   b ---- c
//   |      |
//   a ---- d
struct Square {
    graph::RoadNetwork network;
    NodeId a, b, c, d;
    EdgeId direct;
};

Square makeSquare() {
    graph::RoadNetworkBuilder builder;
    const NodeId a = builder.addIntersection({0.0, 0.0});
    const NodeId b = builder.addIntersection({0.0, 1.0});
    const NodeId c = builder.addIntersection({1.0, 1.0});
    const NodeId d = builder.addIntersection({1.0, 0.0});
    const EdgeId direct = builder.addRoad(a, d, {.speedKmh = 10.0});  // 360 s
    builder.addRoad(a, b, {.speedKmh = 60.0});                        // 60 s each
    builder.addRoad(b, c, {.speedKmh = 60.0});
    builder.addRoad(c, d, {.speedKmh = 60.0});
    return {std::move(builder).build(), a, b, c, d, direct};
}

std::vector<NodeId> randomNodes(const graph::RoadNetwork& network, std::size_t count, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<NodeId> pick(0, static_cast<NodeId>(network.nodeCount() - 1));
    std::vector<NodeId> nodes(count);
    for (NodeId& node : nodes) node = pick(rng);
    return nodes;
}

}  // namespace

TEST_CASE("dijkstra and A* take the faster detour over the shorter slow road") {
    const Square s = makeSquare();
    ShortestPathEngine engine(s.network);
    for (const auto& route : {engine.dijkstra(s.a, s.d), engine.aStar(s.a, s.d)}) {
        REQUIRE(route.found());
        CHECK_APPROX(route.travelTime, 180.0);
        CHECK(route.path == (std::vector<NodeId>{s.a, s.b, s.c, s.d}));
    }
}

TEST_CASE("one-way roads are respected and unreachable targets are reported") {
    const Square s = makeSquare();
    ShortestPathEngine engine(s.network);
    const auto route = engine.aStar(s.d, s.a);
    CHECK(!route.found());
    CHECK(route.travelTime == kUnreachable);
}

TEST_CASE("a route from a node to itself is free") {
    const Square s = makeSquare();
    ShortestPathEngine engine(s.network);
    const auto route = engine.dijkstra(s.b, s.b);
    CHECK(route.travelTime == 0.0);
    CHECK(route.path == std::vector<NodeId>{s.b});
}

TEST_CASE("live traffic updates change the chosen route") {
    Square s = makeSquare();
    ShortestPathEngine engine(s.network);
    // Congest every detour road until the direct road wins.
    for (EdgeId e = 0; e < s.network.edgeCount(); ++e) {
        if (e != s.direct) s.network.setTrafficMultiplier(e, 3.0);
    }
    const auto route = engine.aStar(s.a, s.d);
    CHECK_APPROX(route.travelTime, 360.0);
    CHECK(route.path == (std::vector<NodeId>{s.a, s.d}));
}

TEST_CASE("A* is exact and settles fewer nodes than dijkstra on a generated city") {
    const auto city = sim::generateCity({.rows = 60, .cols = 60, .seed = 5});
    ShortestPathEngine engine(city);
    const auto sources = randomNodes(city, 150, 1);
    const auto targets = randomNodes(city, 150, 2);

    std::size_t dijkstraSettled = 0;
    std::size_t aStarSettled = 0;
    for (std::size_t i = 0; i < sources.size(); ++i) {
        const auto exact = engine.dijkstra(sources[i], targets[i]);
        const auto guided = engine.aStar(sources[i], targets[i]);
        REQUIRE(exact.found() == guided.found());
        if (!exact.found()) continue;
        CHECK_APPROX(guided.travelTime, exact.travelTime);
        CHECK(guided.path.front() == sources[i]);
        CHECK(guided.path.back() == targets[i]);
        dijkstraSettled += exact.stats.nodesSettled;
        aStarSettled += guided.stats.nodesSettled;
    }
    CHECK(aStarSettled < dijkstraSettled);
}

TEST_CASE("weighted A* stays within its suboptimality bound") {
    const auto city = sim::generateCity({.rows = 60, .cols = 60, .seed = 6});
    ShortestPathEngine engine(city);
    const auto sources = randomNodes(city, 100, 3);
    const auto targets = randomNodes(city, 100, 4);
    for (std::size_t i = 0; i < sources.size(); ++i) {
        const auto exact = engine.aStar(sources[i], targets[i]);
        const auto approx = engine.aStar(sources[i], targets[i], 2.0);
        if (!exact.found()) continue;
        CHECK(approx.travelTime >= exact.travelTime - 1e-9);
        CHECK(approx.travelTime <= 2.0 * exact.travelTime + 1e-9);
    }
}

TEST_CASE("one-to-many search agrees with individual searches in both directions") {
    const auto city = sim::generateCity({.rows = 40, .cols = 40, .seed = 7});
    ShortestPathEngine engine(city);
    const NodeId origin = 820;
    auto targets = randomNodes(city, 25, 5);
    targets.push_back(targets.front());  // duplicates are allowed
    targets.push_back(origin);

    const auto forward = engine.travelTimes(origin, targets, Direction::Forward);
    const auto reverse = engine.travelTimes(origin, targets, Direction::Reverse);
    REQUIRE(forward.size() == targets.size());
    for (std::size_t i = 0; i < targets.size(); ++i) {
        CHECK_APPROX(forward[i], engine.dijkstra(origin, targets[i]).travelTime);
        CHECK_APPROX(reverse[i], engine.dijkstra(targets[i], origin).travelTime);
    }
}

TEST_CASE("multi-source A* finds the source with the fastest route") {
    const auto city = sim::generateCity({.rows = 40, .cols = 40, .seed = 8});
    ShortestPathEngine engine(city);
    const auto targets = randomNodes(city, 30, 6);
    for (std::size_t round = 0; round < targets.size(); ++round) {
        const auto sources = randomNodes(city, 12, static_cast<unsigned>(100 + round));

        Seconds best = kUnreachable;
        for (const NodeId source : sources) {
            best = std::min(best, engine.dijkstra(source, targets[round]).travelTime);
        }

        const auto nearest = engine.nearestSource(sources, targets[round]);
        REQUIRE(nearest.has_value() == (best != kUnreachable));
        if (!nearest) continue;
        CHECK_APPROX(nearest->travelTime, best);
        CHECK_APPROX(engine.dijkstra(sources[nearest->sourceIndex], targets[round]).travelTime, best);
    }
}

TEST_CASE("engine validates node ids") {
    const Square s = makeSquare();
    ShortestPathEngine engine(s.network);
    CHECK_THROWS_AS(engine.dijkstra(s.a, 99), std::out_of_range);
    CHECK_THROWS_AS(engine.aStar(99, s.a), std::out_of_range);
    const NodeId bad[] = {99};
    CHECK_THROWS_AS(engine.nearestSource(bad, s.a), std::out_of_range);
    CHECK(!engine.nearestSource({}, s.a).has_value());
}
