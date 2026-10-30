#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/geo/bounding_box.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/spatial/quad_tree.hpp"

namespace waypoint::dispatch {

struct Rider {
    RiderId id;
    geo::Point position;  ///< last reported GPS position
    NodeId node;          ///< intersection the rider is snapped to for routing
};

/// The pool of riders available for new orders, indexed by position.
///
/// Location pings arrive far more often than orders, so updates are
/// O(tree depth): remove from the old leaf, insert into the new one.
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
    spatial::QuadTree<RiderId> index_;
    std::unordered_map<RiderId, Rider> riders_;
};

}  // namespace waypoint::dispatch
