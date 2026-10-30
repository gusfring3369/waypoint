#include "waypoint/sim/city_generator.hpp"

#include <random>
#include <stdexcept>
#include <utility>

#include "waypoint/graph/road_network_builder.hpp"

namespace waypoint::sim {
namespace {

/// Lays out one city according to a CityConfig. Random choices are drawn in a
/// fixed order, so a given seed always yields the same city.
class CityBuilder {
public:
    explicit CityBuilder(const CityConfig& config) : config_(config), rng_(config.seed) {}

    graph::RoadNetwork build() && {
        placeIntersections();
        addStreets();
        return std::move(network_).build();
    }

private:
    void placeIntersections() {
        const double maxShift = config_.jitter * config_.blockKm;
        for (std::size_t row = 0; row < config_.rows; ++row) {
            for (std::size_t col = 0; col < config_.cols; ++col) {
                const double x =
                    static_cast<double>(col) * config_.blockKm + randomBetween(-maxShift, maxShift);
                const double y =
                    static_cast<double>(row) * config_.blockKm + randomBetween(-maxShift, maxShift);
                network_.addIntersection({x, y});
            }
        }
    }

    // Each intersection connects east along its row and north along its column.
    // Streets crossing the river exist only at bridges.
    void addStreets() {
        const std::size_t riverRow = firstRowNorthOfRiver(config_);
        for (std::size_t row = 0; row < config_.rows; ++row) {
            for (std::size_t col = 0; col < config_.cols; ++col) {
                const NodeId here = intersectionAt(config_, row, col);
                if (col + 1 < config_.cols)
                    addStreet(here, intersectionAt(config_, row, col + 1), isArterial(row));
                if (row + 1 >= config_.rows) continue;

                const NodeId north = intersectionAt(config_, row + 1, col);
                const bool crossesRiver = config_.bridgeEvery > 0 && row + 1 == riverRow;
                if (!crossesRiver) {
                    addStreet(here, north, isArterial(col));
                } else if (col % config_.bridgeEvery == 0) {
                    addTwoWayRoad(here, north, config_.localSpeedKmh);  // bridge
                }
            }
        }
    }

    // Arterials are always open and two-way. Local streets may be closed or one-way.
    void addStreet(NodeId a, NodeId b, bool arterial) {
        if (arterial) {
            addTwoWayRoad(a, b, config_.arterialSpeedKmh);
            return;
        }
        if (chance(config_.closedRatio)) return;
        if (chance(config_.oneWayRatio)) {
            if (chance(0.5)) std::swap(a, b);  // random direction
            network_.addRoad(a, b, {.speedKmh = config_.localSpeedKmh, .trafficMultiplier = randomTraffic()});
            return;
        }
        addTwoWayRoad(a, b, config_.localSpeedKmh);
    }

    // Each direction gets its own congestion level.
    void addTwoWayRoad(NodeId a, NodeId b, double speedKmh) {
        network_.addRoad(a, b, {.speedKmh = speedKmh, .trafficMultiplier = randomTraffic()});
        network_.addRoad(b, a, {.speedKmh = speedKmh, .trafficMultiplier = randomTraffic()});
    }

    [[nodiscard]] bool isArterial(std::size_t streetIndex) const {
        return config_.arterialEvery > 0 && streetIndex % config_.arterialEvery == 0;
    }

    double randomTraffic() { return randomBetween(1.0, config_.maxTrafficMultiplier); }
    bool chance(double probability) { return unit_(rng_) < probability; }
    double randomBetween(double low, double high) { return low + unit_(rng_) * (high - low); }

    const CityConfig& config_;
    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> unit_{0.0, 1.0};
    graph::RoadNetworkBuilder network_;
};

}  // namespace

graph::RoadNetwork generateCity(const CityConfig& config) {
    if (config.blockKm <= 0.0 || config.jitter < 0.0 || config.jitter >= 0.5) {
        throw std::invalid_argument("blockKm must be > 0 and jitter in [0, 0.5)");
    }
    if (config.maxTrafficMultiplier < 1.0) throw std::invalid_argument("maxTrafficMultiplier must be >= 1");
    return CityBuilder(config).build();
}

}  // namespace waypoint::sim
