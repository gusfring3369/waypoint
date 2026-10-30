#pragma once

#include <optional>
#include <span>
#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/graph/road_network.hpp"
#include "waypoint/routing/min_heap.hpp"
#include "waypoint/routing/route.hpp"
#include "waypoint/routing/search_labels.hpp"

namespace waypoint::routing {

/// Answers shortest-path questions on one RoadNetwork.
///
/// All four query types run the same search loop and differ only in where
/// they start, which heuristic guides them, and when they stop:
///
///   query            starts at             heuristic            stops when
///   dijkstra         source                none                 target settled
///   aStar            source                straight line        target settled
///   travelTimes      origin                none                 all targets settled
///   nearestSource    every source at once  straight line        target settled
///
/// The engine keeps reusable scratch memory, so give each thread its own.
class ShortestPathEngine {
public:
    explicit ShortestPathEngine(const graph::RoadNetwork& network);

    /// Fastest route, exploring outward evenly in every direction.
    [[nodiscard]] Route dijkstra(NodeId source, NodeId target);

    /// Fastest route, exploring toward the target first. Same result as
    /// dijkstra() with far less work. A heuristicWeight above 1 trades
    /// accuracy for speed (see StraightLineHeuristic).
    [[nodiscard]] Route aStar(NodeId source, NodeId target, double heuristicWeight = 1.0);

    /// Travel times between `origin` and each target from a single search
    /// (kUnreachable where there is no route). Forward measures origin -> target;
    /// Reverse measures target -> origin, e.g. riders driving to one pickup.
    [[nodiscard]] std::vector<Seconds> travelTimes(NodeId origin, std::span<const NodeId> targets,
                                                   graph::Direction direction);

    /// Which source can reach `target` fastest. All sources start searching
    /// at once and the first to arrive wins: one search instead of one per source.
    [[nodiscard]] std::optional<NearestSource> nearestSource(std::span<const NodeId> sources, NodeId target);

    /// Work done by the most recent query.
    [[nodiscard]] const SearchStats& lastStats() const noexcept { return stats_; }

private:
    template <typename Heuristic, typename StopCondition>
    void search(std::span<const NodeId> seeds, graph::Direction direction, const Heuristic& heuristic,
                StopCondition shouldStop);

    template <typename Heuristic>
    void relaxArcs(NodeId node, graph::Direction direction, const Heuristic& heuristic);

    template <typename Heuristic>
    Route findRoute(NodeId source, NodeId target, const Heuristic& heuristic);

    [[nodiscard]] std::vector<NodeId> pathTo(NodeId target) const;
    void requireNode(NodeId node) const;

    const graph::RoadNetwork& network_;
    SearchLabels labels_;
    MinHeap frontier_;
    SearchStats stats_;
};

}  // namespace waypoint::routing
