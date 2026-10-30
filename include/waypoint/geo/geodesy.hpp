#pragma once

#include "waypoint/geo/point.hpp"

namespace waypoint::geo {

/// A WGS-84 coordinate in degrees.
struct LatLng {
    double lat = 0.0;
    double lng = 0.0;
};

inline constexpr double kEarthRadiusKm = 6371.0088;

/// Great-circle distance in kilometres.
[[nodiscard]] double haversineKm(LatLng a, LatLng b) noexcept;

/// Equirectangular projection around a fixed origin.
///
/// The engine works in planar kilometres so the Euclidean heuristic and the
/// QuadTree share one metric. At city scale (tens of km) the projection error
/// is well under 1%; ingest real map data through this once, then route on the
/// projected coordinates so edge lengths and heuristics stay consistent.
class LocalProjection {
public:
    explicit LocalProjection(LatLng origin) noexcept;

    [[nodiscard]] Point toPlanar(LatLng coordinate) const noexcept;
    [[nodiscard]] LatLng toLatLng(Point point) const noexcept;

private:
    LatLng origin_;
    double kmPerDegreeLat_;
    double kmPerDegreeLng_;
};

}  // namespace waypoint::geo
