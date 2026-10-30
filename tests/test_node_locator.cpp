#include <algorithm>
#include <random>

#include "test_framework.hpp"
#include "waypoint/sim/city_generator.hpp"
#include "waypoint/spatial/node_locator.hpp"

using namespace waypoint;

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
