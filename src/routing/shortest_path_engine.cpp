#include "waypoint/routing/shortest_path_engine.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "waypoint/routing/heuristics.hpp"

namespace waypoint::routing {

using graph::Direction;

ShortestPathEngine::ShortestPathEngine(const graph::RoadNetwork& network)
    : network_(network), labels_(network.nodeCount()) {}

// ---------------------------------------------------------------------------
// The search loop shared by every query
// ---------------------------------------------------------------------------

// Best-first search. The frontier holds reached intersections ordered by
// (travel time so far + heuristic estimate). Each step takes the most promising
// one, marks its travel time as final ("settles" it), and checks whether going
// through it gives any neighbour a faster route.
//
// When a neighbour's time improves it is pushed again rather than updated in
// place, so the frontier can hold outdated copies; they are skipped when popped.
template <typename Heuristic, typename StopCondition>
void ShortestPathEngine::search(std::span<const NodeId> seeds, Direction direction,
                                const Heuristic& heuristic, StopCondition shouldStop) {
    labels_.clear();
    frontier_.clear();
    stats_ = {};

    for (std::uint32_t seed = 0; seed < seeds.size(); ++seed) {
        const NodeId node = seeds[seed];
        if (labels_.improve(node, 0.0, kInvalidNode, seed)) frontier_.push(node, heuristic(node));
    }

    while (!frontier_.empty()) {
        const NodeId node = frontier_.popMin();
        if (labels_.isSettled(node)) continue;  // outdated copy

        labels_.markSettled(node);
        ++stats_.nodesSettled;
        if (shouldStop(node)) return;

        relaxArcs(node, direction, heuristic);
    }
}

template <typename Heuristic>
void ShortestPathEngine::relaxArcs(NodeId node, Direction direction, const Heuristic& heuristic) {
    const Seconds timeToNode = labels_.cost(node);
    const std::uint32_t seed = labels_.seed(node);

    for (const graph::Arc& arc : network_.arcs(node, direction)) {
        if (labels_.isSettled(arc.neighbor)) continue;
        ++stats_.edgesRelaxed;

        const Seconds timeToNeighbor = timeToNode + network_.travelTime(arc.edge);
        if (labels_.improve(arc.neighbor, timeToNeighbor, node, seed)) {
            frontier_.push(arc.neighbor, timeToNeighbor + heuristic(arc.neighbor));
        }
    }
}

template <typename Heuristic>
Route ShortestPathEngine::findRoute(NodeId source, NodeId target, const Heuristic& heuristic) {
    const NodeId seeds[] = {source};
    search(seeds, Direction::Forward, heuristic, [target](NodeId node) { return node == target; });

    Route route;
    route.stats = stats_;
    if (labels_.isSettled(target)) {
        route.travelTime = labels_.cost(target);
        route.path = pathTo(target);
    }
    return route;
}

// ---------------------------------------------------------------------------
// Public queries
// ---------------------------------------------------------------------------

Route ShortestPathEngine::dijkstra(NodeId source, NodeId target) {
    requireNode(source);
    requireNode(target);
    return findRoute(source, target, NoHeuristic{});
}

Route ShortestPathEngine::aStar(NodeId source, NodeId target, double heuristicWeight) {
    requireNode(source);
    requireNode(target);
    if (!(heuristicWeight >= 0.0)) throw std::invalid_argument("heuristic weight must be >= 0");
    return findRoute(source, target, StraightLineHeuristic(network_, target, heuristicWeight));
}

std::vector<Seconds> ShortestPathEngine::travelTimes(NodeId origin, std::span<const NodeId> targets,
                                                     Direction direction) {
    requireNode(origin);
    for (const NodeId target : targets) requireNode(target);

    std::unordered_set<NodeId> unsettledTargets(targets.begin(), targets.end());
    const NodeId seeds[] = {origin};
    search(seeds, direction, NoHeuristic{}, [&unsettledTargets](NodeId node) {
        unsettledTargets.erase(node);
        return unsettledTargets.empty();
    });

    std::vector<Seconds> times;
    times.reserve(targets.size());
    for (const NodeId target : targets) {
        times.push_back(labels_.isSettled(target) ? labels_.cost(target) : kUnreachable);
    }
    return times;
}

std::optional<NearestSource> ShortestPathEngine::nearestSource(std::span<const NodeId> sources,
                                                               NodeId target) {
    requireNode(target);
    for (const NodeId source : sources) requireNode(source);

    search(sources, Direction::Forward, StraightLineHeuristic(network_, target),
           [target](NodeId node) { return node == target; });

    if (!labels_.isSettled(target)) return std::nullopt;
    return NearestSource{labels_.seed(target), labels_.cost(target)};
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

std::vector<NodeId> ShortestPathEngine::pathTo(NodeId target) const {
    std::vector<NodeId> path;
    for (NodeId node = target; node != kInvalidNode; node = labels_.parent(node)) path.push_back(node);
    std::ranges::reverse(path);
    return path;
}

void ShortestPathEngine::requireNode(NodeId node) const {
    if (node >= network_.nodeCount()) {
        throw std::out_of_range("intersection " + std::to_string(node) + " is not in the road network");
    }
}

}  // namespace waypoint::routing
