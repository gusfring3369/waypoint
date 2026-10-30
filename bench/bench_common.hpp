#pragma once

// Shared helpers and the list of benchmark sections. Each section lives in its
// own .cpp file and prints a Markdown table.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <random>

#include "waypoint/geo/bounding_box.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/graph/road_network.hpp"

namespace bench {

/// How big a run is: the full run for the README numbers, a quick one for CI.
struct Scale {
    std::size_t gridSide;        ///< city is gridSide x gridSide intersections
    std::size_t routeQueries;    ///< route / pricing queries
    std::size_t riders;          ///< riders in the spatial index
    std::size_t spatialQueries;  ///< radius and k-nearest queries
    std::size_t batchRounds;     ///< batch dispatch rounds
};

inline constexpr Scale kFullScale{200, 1000, 50'000, 10'000, 200};
inline constexpr Scale kQuickScale{60, 100, 5'000, 500, 20};

inline constexpr std::uint64_t kSeed = 2024;

/// Measures elapsed wall time from construction.
class Stopwatch {
public:
    [[nodiscard]] double elapsedMicros() const {
        return std::chrono::duration<double, std::micro>(Clock::now() - start_).count();
    }

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point start_ = Clock::now();
};

[[nodiscard]] inline double percentLess(double baseline, double improved) {
    return 100.0 * (baseline - improved) / baseline;
}

[[nodiscard]] inline waypoint::geo::Point randomPoint(const waypoint::geo::BoundingBox& area,
                                                      std::mt19937_64& rng) {
    std::uniform_real_distribution<double> x(area.min.x, area.max.x);
    std::uniform_real_distribution<double> y(area.min.y, area.max.y);
    return {x(rng), y(rng)};
}

/// Prints a message and exits with an error if two algorithms disagree.
void failIf(bool condition, const char* message);

// Benchmark sections, one per file.
void runRoutingBenchmark(const waypoint::graph::RoadNetwork& city, const Scale& scale);
void runSpatialIndexBenchmark(const waypoint::graph::RoadNetwork& city, const Scale& scale);
void runCandidatePricingBenchmark(const waypoint::graph::RoadNetwork& city, const Scale& scale);
void runBatchDispatchBenchmark(const waypoint::graph::RoadNetwork& city, const Scale& scale);

}  // namespace bench
