#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

#include "waypoint/geo/point.hpp"

namespace waypoint::geo {

/// Axis-aligned rectangle with inclusive bounds.
struct BoundingBox {
    Point min;
    Point max;

    [[nodiscard]] static BoundingBox around(Point center, double halfWidth, double halfHeight) noexcept {
        return {{center.x - halfWidth, center.y - halfHeight}, {center.x + halfWidth, center.y + halfHeight}};
    }

    [[nodiscard]] Point center() const noexcept { return {(min.x + max.x) / 2.0, (min.y + max.y) / 2.0}; }

    [[nodiscard]] bool contains(Point p) const noexcept {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y;
    }

    /// Squared distance from `p` to the closest point of the box (0 when inside).
    [[nodiscard]] double squaredDistanceTo(Point p) const noexcept {
        const double dx = p.x < min.x ? min.x - p.x : (p.x > max.x ? p.x - max.x : 0.0);
        const double dy = p.y < min.y ? min.y - p.y : (p.y > max.y ? p.y - max.y : 0.0);
        return dx * dx + dy * dy;
    }

    /// Squared distance from `p` to the farthest point of the box.
    [[nodiscard]] double squaredDistanceToFarthest(Point p) const noexcept {
        const double dx = std::max(p.x - min.x, max.x - p.x);
        const double dy = std::max(p.y - min.y, max.y - p.y);
        return dx * dx + dy * dy;
    }

    /// Quadrant of `p` relative to the centre: 0 = SW, 1 = SE, 2 = NW, 3 = NE.
    /// Points on a split line go to the east/north side, so every point has exactly one home.
    [[nodiscard]] std::size_t quadrantOf(Point p) const noexcept {
        const Point c = center();
        return (p.x >= c.x ? 1U : 0U) | (p.y >= c.y ? 2U : 0U);
    }

    /// The four children, indexed consistently with quadrantOf().
    [[nodiscard]] std::array<BoundingBox, 4> quadrants() const noexcept {
        const Point c = center();
        return {{
            {{min.x, min.y}, {c.x, c.y}},
            {{c.x, min.y}, {max.x, c.y}},
            {{min.x, c.y}, {c.x, max.y}},
            {{c.x, c.y}, {max.x, max.y}},
        }};
    }
};

}  // namespace waypoint::geo
