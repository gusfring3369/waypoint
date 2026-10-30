#include <cmath>

#include "test_framework.hpp"
#include "waypoint/geo/bounding_box.hpp"
#include "waypoint/geo/geodesy.hpp"
#include "waypoint/geo/point.hpp"

using namespace waypoint::geo;

TEST_CASE("haversine matches the known London-Paris distance") {
    const LatLng london{51.5074, -0.1278};
    const LatLng paris{48.8566, 2.3522};
    CHECK(std::fabs(haversineKm(london, paris) - 343.5) < 1.0);
    CHECK(haversineKm(london, london) == 0.0);
}

TEST_CASE("local projection round-trips and stays accurate at city scale") {
    const LocalProjection projection({12.9716, 77.5946});  // Bengaluru
    const LatLng a{12.9716, 77.5946};
    const LatLng b{13.0500, 77.6500};

    const LatLng roundTrip = projection.toLatLng(projection.toPlanar(b));
    CHECK(std::fabs(roundTrip.lat - b.lat) < 1e-9);
    CHECK(std::fabs(roundTrip.lng - b.lng) < 1e-9);

    const double planar = euclideanDistance(projection.toPlanar(a), projection.toPlanar(b));
    const double geodesic = haversineKm(a, b);
    CHECK(std::fabs(planar - geodesic) / geodesic < 0.005);
}

TEST_CASE("bounding box quadrants partition the box consistently with quadrantOf") {
    const BoundingBox box{{0.0, 0.0}, {4.0, 2.0}};
    const auto quads = box.quadrants();
    const Point samples[] = {{0.0, 0.0}, {2.0, 1.0}, {3.9, 0.1}, {0.1, 1.9}, {4.0, 2.0}, {2.0, 0.0}};
    for (const Point p : samples) {
        CHECK(quads[box.quadrantOf(p)].contains(p));
    }
    CHECK(box.quadrantOf({1.0, 0.5}) == 0);  // SW
    CHECK(box.quadrantOf({3.0, 0.5}) == 1);  // SE
    CHECK(box.quadrantOf({1.0, 1.5}) == 2);  // NW
    CHECK(box.quadrantOf({3.0, 1.5}) == 3);  // NE
}

TEST_CASE("bounding box distance is zero inside and Euclidean outside") {
    const BoundingBox box = BoundingBox::around({0.0, 0.0}, 1.0, 1.0);
    CHECK(box.squaredDistanceTo({0.5, -0.5}) == 0.0);
    CHECK_APPROX(box.squaredDistanceTo({4.0, 0.0}), 9.0);
    CHECK_APPROX(box.squaredDistanceTo({4.0, 5.0}), 9.0 + 16.0);
}
