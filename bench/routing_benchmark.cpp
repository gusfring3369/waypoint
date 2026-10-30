// Section 1: Dijkstra vs A* vs weighted A* on random point-to-point queries.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "bench_common.hpp"
#include "waypoint/routing/shortest_path_engine.hpp"

namespace bench {
namespace {

struct AlgorithmTotals {
    double nodesSettled = 0.0;
    double micros = 0.0;
    double worstRatioToOptimal = 1.0;

    void add(const waypoint::routing::Route& route, double elapsedMicros, double optimalTime) {
        nodesSettled += static_cast<double>(route.stats.nodesSettled);
        micros += elapsedMicros;
        worstRatioToOptimal = std::max(worstRatioToOptimal, route.travelTime / optimalTime);
    }
};

void printRow(const char* name, const AlgorithmTotals& totals, double queries) {
    std::printf("| %s | %.0f | %.0f us | %.3fx |\n", name, totals.nodesSettled / queries,
                totals.micros / queries, totals.worstRatioToOptimal);
}

}  // namespace

void runRoutingBenchmark(const waypoint::graph::RoadNetwork& city, const Scale& scale) {
    using waypoint::NodeId;

    waypoint::routing::ShortestPathEngine engine(city);
    std::mt19937_64 rng(kSeed);
    std::uniform_int_distribution<NodeId> anyNode(0, static_cast<NodeId>(city.nodeCount() - 1));
    AlgorithmTotals dijkstra, aStar, weightedAStar;

    std::size_t measured = 0;
    while (measured < scale.routeQueries) {
        const NodeId source = anyNode(rng);
        const NodeId target = anyNode(rng);

        const Stopwatch dijkstraTimer;
        const auto exact = engine.dijkstra(source, target);
        const double dijkstraMicros = dijkstraTimer.elapsedMicros();
        if (!exact.found() || source == target) continue;  // only measure real trips

        const Stopwatch aStarTimer;
        const auto guided = engine.aStar(source, target);
        const double aStarMicros = aStarTimer.elapsedMicros();

        const Stopwatch weightedTimer;
        const auto approximate = engine.aStar(source, target, 1.5);
        const double weightedMicros = weightedTimer.elapsedMicros();

        failIf(std::abs(guided.travelTime - exact.travelTime) > 1e-6, "A* returned a non-optimal route");
        dijkstra.add(exact, dijkstraMicros, exact.travelTime);
        aStar.add(guided, aStarMicros, exact.travelTime);
        weightedAStar.add(approximate, weightedMicros, exact.travelTime);
        ++measured;
    }

    const auto queries = static_cast<double>(measured);
    std::printf("## 1. Point-to-point routing (%zu random reachable pairs)\n\n", measured);
    std::printf("| Algorithm | Avg nodes settled | Avg query time | Worst route vs optimal |\n");
    std::printf("|---|---:|---:|---:|\n");
    printRow("Dijkstra", dijkstra, queries);
    printRow("A* (admissible)", aStar, queries);
    printRow("Weighted A* (w = 1.5)", weightedAStar, queries);
    std::printf("\nA* settles %.1f%% fewer nodes than Dijkstra (%.1f%% less time), with identical costs.\n\n",
                percentLess(dijkstra.nodesSettled, aStar.nodesSettled),
                percentLess(dijkstra.micros, aStar.micros));
}

}  // namespace bench
