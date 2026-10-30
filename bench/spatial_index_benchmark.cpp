// Section 2: QuadTree vs linear scan for finding riders near a point.

#include <algorithm>
#include <cstdio>
#include <random>
#include <vector>

#include "bench_common.hpp"
#include "waypoint/spatial/quad_tree.hpp"

namespace bench {
namespace {

using waypoint::geo::Point;
using waypoint::spatial::QuadTree;

constexpr double kRadiusKm = 2.0;
constexpr std::size_t kNearest = 16;

std::vector<Point> randomPoints(const waypoint::geo::BoundingBox& area, std::size_t count,
                                std::mt19937_64& rng) {
    std::vector<Point> points(count);
    for (Point& p : points) p = randomPoint(area, rng);
    return points;
}

// Baselines: what you would write without an index.
std::vector<QuadTree::Id> scanWithinRadius(const std::vector<Point>& riders, Point center) {
    std::vector<QuadTree::Id> found;
    for (QuadTree::Id i = 0; i < riders.size(); ++i) {
        if (waypoint::geo::squaredDistance(riders[i], center) <= kRadiusKm * kRadiusKm) found.push_back(i);
    }
    return found;
}

void scanNearest(const std::vector<Point>& riders, Point center, std::vector<double>& distances) {
    for (std::size_t i = 0; i < riders.size(); ++i)
        distances[i] = waypoint::geo::squaredDistance(riders[i], center);
    std::ranges::nth_element(distances, distances.begin() + kNearest - 1);
}

}  // namespace

void runSpatialIndexBenchmark(const waypoint::graph::RoadNetwork& city, const Scale& scale) {
    std::mt19937_64 rng(kSeed + 1);
    const auto riders = randomPoints(city.bounds(), scale.riders, rng);
    const auto queries = randomPoints(city.bounds(), scale.spatialQueries, rng);

    QuadTree tree(city.bounds());
    const Stopwatch buildTimer;
    for (QuadTree::Id i = 0; i < riders.size(); ++i) static_cast<void>(tree.insert(i, riders[i]));
    const double buildMicros = buildTimer.elapsedMicros();

    std::size_t treeHits = 0;
    const Stopwatch treeRadiusTimer;
    for (const Point q : queries) treeHits += tree.withinRadius(q, kRadiusKm).size();
    const double treeRadiusMicros = treeRadiusTimer.elapsedMicros();

    std::size_t scanHits = 0;
    const Stopwatch scanRadiusTimer;
    for (const Point q : queries) scanHits += scanWithinRadius(riders, q).size();
    const double scanRadiusMicros = scanRadiusTimer.elapsedMicros();

    const Stopwatch treeNearestTimer;
    for (const Point q : queries) static_cast<void>(tree.nearest(q, kNearest));
    const double treeNearestMicros = treeNearestTimer.elapsedMicros();

    std::vector<double> distances(riders.size());
    const Stopwatch scanNearestTimer;
    for (const Point q : queries) scanNearest(riders, q, distances);
    const double scanNearestMicros = scanNearestTimer.elapsedMicros();

    failIf(treeHits != scanHits, "QuadTree and linear scan found different riders");

    const auto n = static_cast<double>(queries.size());
    std::printf("## 2. Rider shortlist (%zu riders, %zu queries)\n\n", riders.size(), queries.size());
    std::printf("Index build: %.1f ms. Avg riders within %.0f km: %.0f.\n\n", buildMicros / 1000.0, kRadiusKm,
                static_cast<double>(treeHits) / n);
    std::printf("| Query | Linear scan | QuadTree | Speed-up |\n|---|---:|---:|---:|\n");
    std::printf("| Radius %.0f km | %.1f us | %.1f us | %.0fx |\n", kRadiusKm, scanRadiusMicros / n,
                treeRadiusMicros / n, scanRadiusMicros / treeRadiusMicros);
    std::printf("| %zu nearest | %.1f us | %.1f us | %.0fx |\n\n", kNearest, scanNearestMicros / n,
                treeNearestMicros / n, scanNearestMicros / treeNearestMicros);
}

}  // namespace bench
