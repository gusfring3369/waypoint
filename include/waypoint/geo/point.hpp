#pragma once

#include <cmath>

namespace waypoint::geo {

/// A position on a planar map, in kilometres.
struct Point {
    double x = 0.0;
    double y = 0.0;

    friend bool operator==(const Point&, const Point&) = default;
};

[[nodiscard]] inline double squaredDistance(Point a, Point b) noexcept {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

[[nodiscard]] inline double euclideanDistance(Point a, Point b) noexcept {
    return std::sqrt(squaredDistance(a, b));
}

}  // namespace waypoint::geo
