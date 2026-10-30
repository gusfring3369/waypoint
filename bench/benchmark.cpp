// Reproducible benchmark behind every number quoted in the README.
//
//   waypoint_bench           full run (40,000-intersection city, 50,000 riders)
//   waypoint_bench --quick   smaller run for CI smoke tests

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "waypoint/dispatch/dispatcher.hpp"
#include "waypoint/dispatch/fleet_index.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/routing/shortest_path_engine.hpp"
#include "waypoint/sim/city_generator.hpp"
#include "waypoint/spatial/node_locator.hpp"
#include "waypoint/spatial/quad_tree.hpp"

using namespace waypoint;

namespace {

struct Scale {
    std::size_t gridSide;
    std::size_t routeQueries;
    std::size_t riders;
    std::size_t spatialQueries;
    std::size_t batchRounds;
};

constexpr Scale kFull{200, 1000, 50'000, 10'000, 200};
constexpr Scale kQuick{60, 100, 5'000, 500, 20};

constexpr double kBlockKm = 0.1;
constexpr std::uint64_t kSeed = 2024;

class Stopwatch {
public:
    [[nodiscard]] double elapsedMicros() const {
        return std::chrono::duration<double, std::micro>(Clock::now() - start_).count();
    }

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point start_ = Clock::now();
};

double percentLess(double baseline, double improved) {
    return 100.0 * (baseline - improved) / baseline;
}

geo::Point randomPoint(const geo::BoundingBox& area, std::mt19937_64& rng) {
    std::uniform_real_distribution<double> x(area.min.x, area.max.x);
    std::uniform_real_distribution<double> y(area.min.y, area.max.y);
    return {x(rng), y(rng)};
}

// ---------------------------------------------------------------------------
void benchmarkRouting(const graph::RoadNetwork& city, const Scale& scale) {
    struct Totals {
        double settled = 0.0;
        double micros = 0.0;
        double worstRatio = 1.0;
    };
    Totals dijkstra, aStar, weighted;

    routing::ShortestPathEngine engine(city);
    std::mt19937_64 rng(kSeed);
    std::uniform_int_distribution<NodeId> anyNode(0, static_cast<NodeId>(city.nodeCount() - 1));

    std::size_t measured = 0;
    while (measured < scale.routeQueries) {
        const NodeId source = anyNode(rng);
        const NodeId target = anyNode(rng);

        Stopwatch t1;
        const auto exact = engine.dijkstra(source, target);
        const double dijkstraMicros = t1.elapsedMicros();
        if (!exact.found() || source == target) continue;

        Stopwatch t2;
        const auto guided = engine.aStar(source, target);
        const double aStarMicros = t2.elapsedMicros();

        Stopwatch t3;
        const auto greedy = engine.aStar(source, target, 1.5);
        const double weightedMicros = t3.elapsedMicros();

        if (std::abs(guided.travelTime - exact.travelTime) > 1e-6) {
            std::printf("A* returned a non-optimal route; aborting\n");
            std::exit(1);
        }
        dijkstra.settled += static_cast<double>(exact.stats.nodesSettled);
        dijkstra.micros += dijkstraMicros;
        aStar.settled += static_cast<double>(guided.stats.nodesSettled);
        aStar.micros += aStarMicros;
        weighted.settled += static_cast<double>(greedy.stats.nodesSettled);
        weighted.micros += weightedMicros;
        weighted.worstRatio = std::max(weighted.worstRatio, greedy.travelTime / exact.travelTime);
        ++measured;
    }

    const auto n = static_cast<double>(measured);
    std::printf("## 1. Point-to-point routing (%zu random reachable pairs)\n\n", measured);
    std::printf("| Algorithm | Avg nodes settled | Avg query time | Worst route vs optimal |\n");
    std::printf("|---|---:|---:|---:|\n");
    std::printf("| Dijkstra | %.0f | %.0f us | 1.000x |\n", dijkstra.settled / n, dijkstra.micros / n);
    std::printf("| A* (admissible) | %.0f | %.0f us | 1.000x |\n", aStar.settled / n, aStar.micros / n);
    std::printf("| Weighted A* (w = 1.5) | %.0f | %.0f us | %.3fx |\n\n", weighted.settled / n,
                weighted.micros / n, weighted.worstRatio);
    std::printf("A* settles %.1f%% fewer nodes than Dijkstra (%.1f%% less time), with identical costs.\n\n",
                percentLess(dijkstra.settled, aStar.settled), percentLess(dijkstra.micros, aStar.micros));
}

// ---------------------------------------------------------------------------
void benchmarkSpatialIndex(const graph::RoadNetwork& city, const Scale& scale) {
    constexpr double kRadiusKm = 2.0;
    constexpr std::size_t kNearest = 16;

    std::mt19937_64 rng(kSeed + 1);
    std::vector<geo::Point> riders(scale.riders);
    for (auto& p : riders) p = randomPoint(city.bounds(), rng);

    spatial::QuadTree<std::size_t> tree(city.bounds());
    Stopwatch build;
    for (std::size_t i = 0; i < riders.size(); ++i) static_cast<void>(tree.insert(i, riders[i]));
    const double buildMicros = build.elapsedMicros();

    std::vector<geo::Point> queries(scale.spatialQueries);
    for (auto& q : queries) q = randomPoint(city.bounds(), rng);

    std::size_t treeHits = 0;
    Stopwatch treeRadius;
    for (const auto q : queries) treeHits += tree.withinRadius(q, kRadiusKm).size();
    const double treeRadiusMicros = treeRadius.elapsedMicros();

    // Both sides materialise the matching riders, as a caller would need them.
    std::size_t scanHits = 0;
    std::vector<std::size_t> scanResult;
    Stopwatch scanRadius;
    for (const auto q : queries) {
        scanResult.clear();
        for (std::size_t i = 0; i < riders.size(); ++i) {
            if (geo::squaredDistance(riders[i], q) <= kRadiusKm * kRadiusKm) scanResult.push_back(i);
        }
        scanHits += scanResult.size();
    }
    const double scanRadiusMicros = scanRadius.elapsedMicros();

    Stopwatch treeKnn;
    std::size_t knnChecksum = 0;
    for (const auto q : queries) knnChecksum += tree.nearest(q, kNearest).size();
    const double treeKnnMicros = treeKnn.elapsedMicros();

    Stopwatch scanKnn;
    std::vector<double> distances(riders.size());
    for (const auto q : queries) {
        for (std::size_t i = 0; i < riders.size(); ++i) distances[i] = geo::squaredDistance(riders[i], q);
        std::ranges::nth_element(distances, distances.begin() + kNearest - 1);
        knnChecksum += kNearest;
    }
    const double scanKnnMicros = scanKnn.elapsedMicros();

    if (treeHits != scanHits) {
        std::printf("QuadTree and linear scan disagree; aborting\n");
        std::exit(1);
    }

    const auto n = static_cast<double>(queries.size());
    std::printf("## 2. Rider shortlist (%zu riders, %zu queries)\n\n", riders.size(), queries.size());
    std::printf("Index build: %.1f ms. Avg riders within %.0f km: %.0f.\n\n", buildMicros / 1000.0, kRadiusKm,
                static_cast<double>(treeHits) / n);
    std::printf("| Query | Linear scan | QuadTree | Speed-up |\n|---|---:|---:|---:|\n");
    std::printf("| Radius %.0f km | %.1f us | %.1f us | %.0fx |\n", kRadiusKm, scanRadiusMicros / n,
                treeRadiusMicros / n, scanRadiusMicros / treeRadiusMicros);
    std::printf("| %zu nearest | %.1f us | %.1f us | %.0fx |\n\n", kNearest, scanKnnMicros / n,
                treeKnnMicros / n, scanKnnMicros / treeKnnMicros);
    static_cast<void>(knnChecksum);
}

// ---------------------------------------------------------------------------
void benchmarkCandidatePricing(const graph::RoadNetwork& city, const Scale& scale) {
    constexpr std::size_t kCandidates = 16;
    constexpr double kRadiusKm = 3.0;
    // Most of the fleet is busy at any moment; only free riders are candidates.
    const std::size_t availableRiders = scale.riders / 25;

    const spatial::NodeLocator locator(city);
    std::mt19937_64 rng(kSeed + 2);
    dispatch::FleetIndex fleet(city.bounds());
    for (RiderId id = 0; id < availableRiders; ++id) {
        const auto p = randomPoint(city.bounds(), rng);
        fleet.upsert({id, p, *locator.nearestNode(p)});
    }

    routing::ShortestPathEngine engine(city);
    double perCandidateSettled = 0.0, oneToManySettled = 0.0, multiSourceSettled = 0.0;
    double perCandidateMicros = 0.0, oneToManyMicros = 0.0, multiSourceMicros = 0.0;
    std::size_t orders = 0;

    for (std::size_t i = 0; i < scale.routeQueries; ++i) {
        const auto pickup = randomPoint(city.bounds(), rng);
        const NodeId pickupNode = *locator.nearestNode(pickup);
        std::vector<NodeId> sources;
        for (const auto& rider : fleet.nearby(pickup, kRadiusKm, kCandidates)) sources.push_back(rider.node);
        if (sources.size() < kCandidates) continue;

        Seconds bestSeparate = kUnreachable;
        Stopwatch t1;
        for (const NodeId source : sources) {
            const auto route = engine.aStar(source, pickupNode);
            bestSeparate = std::min(bestSeparate, route.travelTime);
            perCandidateSettled += static_cast<double>(route.stats.nodesSettled);
        }
        perCandidateMicros += t1.elapsedMicros();

        Stopwatch t2;
        const auto times = engine.travelTimes(pickupNode, sources, graph::Direction::Reverse);
        oneToManyMicros += t2.elapsedMicros();
        oneToManySettled += static_cast<double>(engine.lastStats().nodesSettled);

        Stopwatch t3;
        const auto nearest = engine.nearestSource(sources, pickupNode);
        multiSourceMicros += t3.elapsedMicros();
        multiSourceSettled += static_cast<double>(engine.lastStats().nodesSettled);

        const Seconds bestOneToMany = *std::ranges::min_element(times);
        const Seconds bestMultiSource = nearest ? nearest->travelTime : kUnreachable;
        if (std::abs(bestSeparate - bestOneToMany) > 1e-6 ||
            std::abs(bestSeparate - bestMultiSource) > 1e-6) {
            std::printf("Pricing strategies disagree; aborting\n");
            std::exit(1);
        }
        ++orders;
    }

    const auto n = static_cast<double>(orders);
    std::printf("## 3. Pricing a %zu-rider shortlist for one order (%zu available riders, %zu orders)\n\n",
                kCandidates, availableRiders, orders);
    std::printf(
        "| Strategy | Searches | Avg nodes settled | Avg time | Output |\n|---|---:|---:|---:|---|\n");
    std::printf("| A* per candidate | %zu | %.0f | %.0f us | every ETA |\n", kCandidates,
                perCandidateSettled / n, perCandidateMicros / n);
    std::printf("| Reverse one-to-many Dijkstra | 1 | %.0f | %.0f us | every ETA |\n", oneToManySettled / n,
                oneToManyMicros / n);
    std::printf("| Multi-source A* | 1 | %.0f | %.0f us | fastest rider only |\n\n", multiSourceSettled / n,
                multiSourceMicros / n);
    std::printf(
        "Multi-source A* finds the fastest rider settling %.1f%% fewer nodes than one A* per candidate.\n\n",
        percentLess(perCandidateSettled, multiSourceSettled));
}

// ---------------------------------------------------------------------------
void benchmarkBatchMatching(const graph::RoadNetwork& city, const Scale& scale) {
    constexpr std::size_t kOrders = 30;
    constexpr std::size_t kRiders = 40;
    // A dense neighbourhood (3 x 3 km) so orders genuinely compete for riders.
    const geo::Point centre = city.bounds().center();
    const auto zone = geo::BoundingBox::around(centre, 1.5, 1.5);

    const spatial::NodeLocator locator(city);
    std::mt19937_64 rng(kSeed + 3);

    double greedyEta = 0.0, optimalEta = 0.0;
    std::size_t greedyServed = 0, optimalServed = 0, comparableRounds = 0;
    double greedyComparableEta = 0.0, optimalComparableEta = 0.0;
    double optimalMicros = 0.0;

    for (std::size_t round = 0; round < scale.batchRounds; ++round) {
        dispatch::FleetIndex fleet(city.bounds());
        for (RiderId id = 0; id < kRiders; ++id) {
            const auto p = randomPoint(zone, rng);
            fleet.upsert({id, p, *locator.nearestNode(p)});
        }
        std::vector<dispatch::Order> orders;
        for (OrderId id = 0; id < kOrders; ++id) {
            const auto p = randomPoint(zone, rng);
            orders.push_back({id, p, *locator.nearestNode(p)});
        }

        dispatch::Dispatcher dispatcher(city, fleet, {.searchRadiusKm = 2.0, .maxCandidatesPerOrder = 12});
        const auto greedy = dispatcher.dispatchBatch(orders, dispatch::MatchingStrategy::Greedy);
        Stopwatch timer;
        const auto optimal = dispatcher.dispatchBatch(orders, dispatch::MatchingStrategy::Optimal);
        optimalMicros += timer.elapsedMicros();

        greedyEta += greedy.totalPickupEta();
        optimalEta += optimal.totalPickupEta();
        greedyServed += greedy.assignments.size();
        optimalServed += optimal.assignments.size();
        if (greedy.assignments.size() == optimal.assignments.size()) {
            ++comparableRounds;
            greedyComparableEta += greedy.totalPickupEta();
            optimalComparableEta += optimal.totalPickupEta();
        }
    }

    const auto rounds = static_cast<double>(scale.batchRounds);
    std::printf("## 4. Batch dispatch (%zu rounds of %zu orders vs %zu riders in a 3 x 3 km zone)\n\n",
                scale.batchRounds, kOrders, kRiders);
    std::printf("| Strategy | Orders served | Avg pickup ETA per order |\n|---|---:|---:|\n");
    std::printf("| Greedy (first-come-first-served) | %.1f%% | %.0f s |\n",
                100.0 * static_cast<double>(greedyServed) / (rounds * kOrders),
                greedyEta / static_cast<double>(greedyServed));
    std::printf("| Optimal (Hungarian) | %.1f%% | %.0f s |\n\n",
                100.0 * static_cast<double>(optimalServed) / (rounds * kOrders),
                optimalEta / static_cast<double>(optimalServed));
    std::printf(
        "In the %zu rounds where both served the same number of orders, optimal matching cut total "
        "pickup time by %.1f%%. Avg end-to-end batch time (shortlist + price + match): %.2f ms.\n\n",
        comparableRounds, percentLess(greedyComparableEta, optimalComparableEta),
        optimalMicros / rounds / 1000.0);
}

}  // namespace

int main(int argc, char** argv) {
    const bool quick = argc > 1 && std::strcmp(argv[1], "--quick") == 0;
    const Scale scale = quick ? kQuick : kFull;

    sim::CityConfig config;
    config.rows = config.cols = scale.gridSide;
    config.blockKm = kBlockKm;
    config.seed = kSeed;

    Stopwatch build;
    const auto city = sim::generateCity(config);
    std::printf("# Waypoint benchmark%s\n\n", quick ? " (quick)" : "");
    std::printf("City: %zu intersections, %zu directed road segments, %.0f x %.0f km, built in %.1f ms.\n\n",
                city.nodeCount(), city.edgeCount(), city.bounds().max.x - city.bounds().min.x,
                city.bounds().max.y - city.bounds().min.y, build.elapsedMicros() / 1000.0);

    benchmarkRouting(city, scale);
    benchmarkSpatialIndex(city, scale);
    benchmarkCandidatePricing(city, scale);
    benchmarkBatchMatching(city, scale);
    return 0;
}
