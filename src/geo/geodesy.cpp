#include "waypoint/geo/geodesy.hpp"

#include <cmath>
#include <numbers>

namespace waypoint::geo {
namespace {

constexpr double toRadians(double degrees) noexcept {
    return degrees * std::numbers::pi / 180.0;
}

}  // namespace

double haversineKm(LatLng a, LatLng b) noexcept {
    const double dLat = toRadians(b.lat - a.lat);
    const double dLng = toRadians(b.lng - a.lng);
    const double sinLat = std::sin(dLat / 2.0);
    const double sinLng = std::sin(dLng / 2.0);
    const double h =
        sinLat * sinLat + std::cos(toRadians(a.lat)) * std::cos(toRadians(b.lat)) * sinLng * sinLng;
    return 2.0 * kEarthRadiusKm * std::asin(std::sqrt(h));
}

LocalProjection::LocalProjection(LatLng origin) noexcept
    : origin_(origin),
      kmPerDegreeLat_(toRadians(1.0) * kEarthRadiusKm),
      kmPerDegreeLng_(toRadians(1.0) * kEarthRadiusKm * std::cos(toRadians(origin.lat))) {}

Point LocalProjection::toPlanar(LatLng coordinate) const noexcept {
    return {(coordinate.lng - origin_.lng) * kmPerDegreeLng_,
            (coordinate.lat - origin_.lat) * kmPerDegreeLat_};
}

LatLng LocalProjection::toLatLng(Point point) const noexcept {
    return {origin_.lat + point.y / kmPerDegreeLat_, origin_.lng + point.x / kmPerDegreeLng_};
}

}  // namespace waypoint::geo
