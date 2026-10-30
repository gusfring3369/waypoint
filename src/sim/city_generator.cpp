#include "waypoint/sim/city_generator.hpp"

#include <random>
#include <stdexcept>
#include <utility>

namespace waypoint::sim {

graph::RoadNetwork generateCity(const CityConfig& config) {
    if (config.blockKm <= 0.0 || config.jitter < 0.0 || config.jitter >= 0.5) {
        throw std::invalid_argument("blockKm must be > 0 and jitter in [0, 0.5)");
    }
    if (config.maxTrafficMultiplier < 1.0) {
        throw std::invalid_argument("maxTrafficMultiplier must be >= 1");
    }

    std::mt19937_64 rng(config.seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const auto chance = [&](double probability) { return unit(rng) < probability; };
    const auto traffic = [&] { return 1.0 + unit(rng) * (config.maxTrafficMultiplier - 1.0); };

    graph::RoadNetworkBuilder builder;
    const double maxOffset = config.jitter * config.blockKm;
    for (std::size_t r = 0; r < config.rows; ++r) {
        for (std::size_t c = 0; c < config.cols; ++c) {
            builder.addIntersection(
                {static_cast<double>(c) * config.blockKm + (2.0 * unit(rng) - 1.0) * maxOffset,
                 static_cast<double>(r) * config.blockKm + (2.0 * unit(rng) - 1.0) * maxOffset});
        }
    }

    const auto nodeAt = [&](std::size_t r, std::size_t c) {
        return static_cast<NodeId>(r * config.cols + c);
    };
    const auto isArterial = [&](std::size_t streetIndex) {
        return config.arterialEvery > 0 && streetIndex % config.arterialEvery == 0;
    };
    // Each direction of a street gets its own congestion level.
    const auto addTwoWay = [&](NodeId a, NodeId b, double speedKmh) {
        builder.addRoad(a, b, {.speedKmh = speedKmh, .trafficMultiplier = traffic()});
        builder.addRoad(b, a, {.speedKmh = speedKmh, .trafficMultiplier = traffic()});
    };
    const auto addStreet = [&](NodeId a, NodeId b, bool arterial) {
        if (arterial) {
            addTwoWay(a, b, config.arterialSpeedKmh);
            return;
        }
        if (chance(config.closedRatio)) return;
        if (chance(config.oneWayRatio)) {
            if (chance(0.5)) std::swap(a, b);
            builder.addRoad(a, b, {.speedKmh = config.localSpeedKmh, .trafficMultiplier = traffic()});
            return;
        }
        addTwoWay(a, b, config.localSpeedKmh);
    };

    // The river runs between rows riverRow - 1 and riverRow.
    const std::size_t riverRow = config.rows / 2;
    for (std::size_t r = 0; r < config.rows; ++r) {
        for (std::size_t c = 0; c < config.cols; ++c) {
            if (c + 1 < config.cols) addStreet(nodeAt(r, c), nodeAt(r, c + 1), isArterial(r));
            if (r + 1 >= config.rows) continue;

            const bool crossesRiver = config.bridgeEvery > 0 && r + 1 == riverRow;
            if (!crossesRiver) {
                addStreet(nodeAt(r, c), nodeAt(r + 1, c), isArterial(c));
            } else if (c % config.bridgeEvery == 0) {
                addTwoWay(nodeAt(r, c), nodeAt(r + 1, c), config.localSpeedKmh);
            }
        }
    }

    return std::move(builder).build();
}

}  // namespace waypoint::sim
