// Reproducible benchmark behind every number quoted in the README.
//
//   waypoint_bench           full run (40,000-intersection city, 50,000 riders)
//   waypoint_bench --quick   smaller run for CI

#include <cstdio>
#include <cstdlib>
#include <string_view>

#include "bench_common.hpp"
#include "waypoint/sim/city_generator.hpp"

namespace bench {

void failIf(bool condition, const char* message) {
    if (!condition) return;
    std::printf("FAILED: %s\n", message);
    std::exit(EXIT_FAILURE);
}

}  // namespace bench

int main(int argc, char** argv) {
    const bool quick = argc > 1 && std::string_view(argv[1]) == "--quick";
    const bench::Scale& scale = quick ? bench::kQuickScale : bench::kFullScale;

    waypoint::sim::CityConfig config;
    config.rows = scale.gridSide;
    config.cols = scale.gridSide;
    config.blockKm = 0.1;
    config.seed = bench::kSeed;

    const bench::Stopwatch buildTimer;
    const auto city = waypoint::sim::generateCity(config);
    const auto& area = city.bounds();
    std::printf("# Waypoint benchmark%s\n\n", quick ? " (quick)" : "");
    std::printf("City: %zu intersections, %zu directed road segments, %.0f x %.0f km, built in %.1f ms.\n\n",
                city.nodeCount(), city.edgeCount(), area.max.x - area.min.x, area.max.y - area.min.y,
                buildTimer.elapsedMicros() / 1000.0);

    bench::runRoutingBenchmark(city, scale);
    bench::runSpatialIndexBenchmark(city, scale);
    bench::runCandidatePricingBenchmark(city, scale);
    bench::runBatchDispatchBenchmark(city, scale);
    return EXIT_SUCCESS;
}
