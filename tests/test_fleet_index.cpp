#include "test_framework.hpp"
#include "waypoint/dispatch/fleet_index.hpp"

using namespace waypoint;
using namespace waypoint::dispatch;

namespace {

const geo::BoundingBox kTownArea{{-1.0, -1.0}, {6.0, 1.0}};

}  // namespace

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

TEST_CASE("fleet index returns nearby riders nearest first, up to the limit") {
    FleetIndex fleet(kTownArea);
    REQUIRE(fleet.upsert({.id = 1, .position = {3.0, 0.0}, .node = 0}));
    REQUIRE(fleet.upsert({.id = 2, .position = {1.0, 0.0}, .node = 0}));
    REQUIRE(fleet.upsert({.id = 3, .position = {2.0, 0.0}, .node = 0}));

    const auto nearest = fleet.nearby({0.0, 0.0}, 10.0, 2);
    REQUIRE(nearest.size() == 2);
    CHECK(nearest[0].id == 2);
    CHECK(nearest[1].id == 3);
}
