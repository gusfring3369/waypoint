#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/graph/road_network.hpp"

namespace waypoint::routing {

struct SearchStats {
    std::size_t nodesSettled = 0;  ///< nodes popped from the heap with a final cost
    std::size_t edgesRelaxed = 0;  ///< arcs examined while expanding settled nodes
};

struct Route {
    Seconds travelTime = kUnreachable;
    std::vector<NodeId> path;  ///< source first, target last; empty when unreachable
    SearchStats stats;

    [[nodiscard]] bool found() const noexcept { return !path.empty(); }
};

struct NearestSource {
    std::size_t sourceIndex;  ///< index into the `sources` span that won
    Seconds travelTime;
};

/// Label-setting shortest path solver bound to one RoadNetwork.
///
/// All query types share one search loop (binary heap with lazy deletion) and
/// differ only in their seeds, heuristic and stopping rule. Per-node labels are
/// allocated once and tagged with a query generation, so starting a query is
/// O(1) instead of O(V): cost scales with the region explored, not the map.
///
/// Holds mutable scratch space, so use one engine per thread.
class ShortestPathEngine {
public:
    explicit ShortestPathEngine(const graph::RoadNetwork& network);

    /// Uninformed baseline: settles nodes in order of distance from `source`.
    [[nodiscard]] Route dijkstra(NodeId source, NodeId target);

    /// Goal-directed search guided by RoadNetwork::lowerBoundTravelTime().
    /// heuristicWeight == 1 is exact. A weight w > 1 (weighted A*) settles fewer
    /// nodes but may return a route up to w times the optimum.
    [[nodiscard]] Route aStar(NodeId source, NodeId target, double heuristicWeight = 1.0);

    /// One-to-many: a single search from `origin` that prices every target and
    /// stops once all are settled. Forward gives origin -> target times; Reverse
    /// gives target -> origin times. Unreachable targets get kUnreachable.
    [[nodiscard]] std::vector<Seconds> travelTimes(NodeId origin, std::span<const NodeId> targets,
                                                   graph::Direction direction);

    /// Multi-source A*: every source starts at cost 0, and the first label to
    /// reach `target` identifies the source with the fastest route. Prices N
    /// sources for roughly the cost of one search.
    [[nodiscard]] std::optional<NearestSource> nearestSource(std::span<const NodeId> sources, NodeId target);

    /// Work done by the most recent query.
    [[nodiscard]] const SearchStats& lastStats() const noexcept { return stats_; }

private:
    enum class Visit { Continue, Stop };

    struct Label {
        Seconds cost = 0.0;
        NodeId parent = kInvalidNode;
        std::uint32_t origin = 0;   // index of the seed this label grew from
        std::uint32_t reached = 0;  // generation in which `cost` was last written
        std::uint32_t settled = 0;  // generation in which `cost` became final
    };

    struct HeapEntry {
        Seconds priority;
        NodeId node;
    };

    template <typename Heuristic, typename OnSettle>
    void search(std::span<const NodeId> seeds, graph::Direction direction, const Heuristic& heuristic,
                OnSettle&& onSettle);

    template <typename Heuristic>
    [[nodiscard]] Route pointToPoint(NodeId source, NodeId target, const Heuristic& heuristic);
    void beginQuery();
    bool improve(NodeId node, Seconds cost, NodeId parent, std::uint32_t origin);
    [[nodiscard]] bool isSettled(NodeId node) const noexcept;
    [[nodiscard]] std::vector<NodeId> pathTo(NodeId target) const;
    void requireNode(NodeId node) const;

    const graph::RoadNetwork& network_;
    std::vector<Label> labels_;
    std::vector<std::uint32_t> targetMarks_;  // generation-tagged membership for travelTimes()
    std::vector<HeapEntry> heap_;
    std::uint32_t generation_ = 0;
    SearchStats stats_;
};

}  // namespace waypoint::routing
