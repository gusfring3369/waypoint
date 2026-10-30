#include "waypoint/dispatch/fleet_index.hpp"

namespace waypoint::dispatch {

FleetIndex::FleetIndex(geo::BoundingBox serviceArea) : index_(serviceArea) {}

bool FleetIndex::upsert(const Rider& rider) {
    remove(rider.id);
    if (!index_.insert(rider.id, rider.position)) return false;
    riders_.emplace(rider.id, rider);
    return true;
}

bool FleetIndex::remove(RiderId id) {
    const auto it = riders_.find(id);
    if (it == riders_.end()) return false;
    index_.remove(id, it->second.position);
    riders_.erase(it);
    return true;
}

const Rider* FleetIndex::find(RiderId id) const {
    const auto it = riders_.find(id);
    return it == riders_.end() ? nullptr : &it->second;
}

std::vector<Rider> FleetIndex::nearby(geo::Point point, double radiusKm, std::size_t limit) const {
    std::vector<Rider> found;
    for (const auto& hit : index_.nearest(point, limit, radiusKm)) {
        found.push_back(riders_.at(hit.id));
    }
    return found;
}

}  // namespace waypoint::dispatch
