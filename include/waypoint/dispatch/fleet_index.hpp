#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/dispatch/models.hpp"
#include "waypoint/geo/bounding_box.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/spatial/quad_tree.hpp"

namespace waypoint::dispatch {

/// The riders currently free to take orders, searchable by location.
///
/// Riders send location updates far more often than orders arrive, so an
/// update is cheap: remove the rider from its old QuadTree cell and insert it
/// into the new one.
class FleetIndex {
public:
    /// `serviceArea` should cover every position riders can report from, not just
    /// the road network's bounds: riders reported outside it cannot be indexed.
    explicit FleetIndex(geo::BoundingBox serviceArea);

    /// Adds a rider or applies a location update. A rider reported outside the
    /// service area cannot be dispatched, so they are dropped and false returned.
    bool upsert(const Rider& rider);

    /// Takes a rider out of the pool (e.g. once assigned). False if not present.
    bool remove(RiderId id);

    [[nodiscard]] const Rider* find(RiderId id) const;

    /// Up to `limit` riders within `radiusKm` of `point`, nearest first.
    [[nodiscard]] std::vector<Rider> nearby(geo::Point point, double radiusKm, std::size_t limit) const;

    [[nodiscard]] std::size_t size() const noexcept { return riders_.size(); }

private:
    spatial::QuadTree index_;
    std::unordered_map<RiderId, Rider> riders_;
};

}  // namespace waypoint::dispatch
