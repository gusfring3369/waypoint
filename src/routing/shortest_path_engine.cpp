#include "waypoint/routing/shortest_path_engine.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace waypoint::routing {
namespace {

using graph::Direction;

struct ZeroHeuristic {
    Seconds operator()(NodeId /*node*/) const noexcept { return 0.0; }
};

// Admissible and consistent given the RoadNetwork invariants; scaling it by
// w > 1 trades optimality for a narrower search.
class TravelTimeLowerBound {
public:
    TravelTimeLowerBound(const graph::RoadNetwork& network, NodeId target, double weight)
        : network_(network), target_(network.position(target)), weight_(weight) {}

    Seconds operator()(NodeId node) const noexcept {
        return weight_ * network_.lowerBoundTravelTime(network_.position(node), target_);
    }

private:
    const graph::RoadNetwork& network_;
    geo::Point target_;
    double weight_;
};

}  // namespace

ShortestPathEngine::ShortestPathEngine(const graph::RoadNetwork& network)
    : network_(network), labels_(network.nodeCount()), targetMarks_(network.nodeCount(), 0) {}

template <typename Heuristic>
Route ShortestPathEngine::pointToPoint(NodeId source, NodeId target, const Heuristic& heuristic) {
    requireNode(source);
    requireNode(target);

    beginQuery();
    const NodeId seeds[] = {source};
    search(seeds, Direction::Forward, heuristic,
           [target](NodeId node) { return node == target ? Visit::Stop : Visit::Continue; });

    Route route;
    route.stats = stats_;
    if (isSettled(target)) {
        route.travelTime = labels_[target].cost;
        route.path = pathTo(target);
    }
    return route;
}

// The single search loop behind every query. Priorities are cost + heuristic;
// with a consistent heuristic a node's cost is final the first time it is
// popped, so later (stale) heap copies of it are skipped instead of being
// removed eagerly with a decrease-key.
template <typename Heuristic, typename OnSettle>
void ShortestPathEngine::search(std::span<const NodeId> seeds, Direction direction,
                                const Heuristic& heuristic, OnSettle&& onSettle) {
    const auto byPriority = [](const HeapEntry& a, const HeapEntry& b) { return a.priority > b.priority; };
    const auto push = [&](NodeId node, Seconds cost) {
        heap_.push_back({cost + heuristic(node), node});
        std::ranges::push_heap(heap_, byPriority);
    };

    for (std::uint32_t i = 0; i < seeds.size(); ++i) {
        if (improve(seeds[i], 0.0, kInvalidNode, i)) push(seeds[i], 0.0);
    }

    while (!heap_.empty()) {
        std::ranges::pop_heap(heap_, byPriority);
        const NodeId node = heap_.back().node;
        heap_.pop_back();

        Label& label = labels_[node];
        if (label.settled == generation_) continue;
        label.settled = generation_;
        ++stats_.nodesSettled;

        if (onSettle(node) == Visit::Stop) return;

        for (const auto& arc : network_.arcs(node, direction)) {
            if (isSettled(arc.neighbor)) continue;
            ++stats_.edgesRelaxed;
            const Seconds cost = label.cost + network_.travelTime(arc.edge);
            if (improve(arc.neighbor, cost, node, label.origin)) push(arc.neighbor, cost);
        }
    }
}

Route ShortestPathEngine::dijkstra(NodeId source, NodeId target) {
    return pointToPoint(source, target, ZeroHeuristic{});
}

Route ShortestPathEngine::aStar(NodeId source, NodeId target, double heuristicWeight) {
    if (!(heuristicWeight >= 0.0)) throw std::invalid_argument("heuristic weight must be >= 0");
    requireNode(target);
    return pointToPoint(source, target, TravelTimeLowerBound(network_, target, heuristicWeight));
}

std::vector<Seconds> ShortestPathEngine::travelTimes(NodeId origin, std::span<const NodeId> targets,
                                                     Direction direction) {
    requireNode(origin);
    for (const NodeId target : targets) requireNode(target);

    beginQuery();
    std::size_t pending = 0;
    for (const NodeId target : targets) {
        if (targetMarks_[target] != generation_) {
            targetMarks_[target] = generation_;
            ++pending;
        }
    }

    const NodeId seeds[] = {origin};
    if (pending > 0) {
        search(seeds, direction, ZeroHeuristic{}, [&](NodeId node) {
            const bool isTarget = targetMarks_[node] == generation_;
            return isTarget && --pending == 0 ? Visit::Stop : Visit::Continue;
        });
    }

    std::vector<Seconds> times;
    times.reserve(targets.size());
    for (const NodeId target : targets) {
        times.push_back(isSettled(target) ? labels_[target].cost : kUnreachable);
    }
    return times;
}

std::optional<NearestSource> ShortestPathEngine::nearestSource(std::span<const NodeId> sources,
                                                               NodeId target) {
    requireNode(target);
    for (const NodeId source : sources) requireNode(source);

    beginQuery();
    search(sources, Direction::Forward, TravelTimeLowerBound(network_, target, 1.0),
           [target](NodeId node) { return node == target ? Visit::Stop : Visit::Continue; });

    if (!isSettled(target)) return std::nullopt;
    return NearestSource{labels_[target].origin, labels_[target].cost};
}

void ShortestPathEngine::beginQuery() {
    heap_.clear();
    stats_ = {};
    if (++generation_ == 0) {  // wrapped after 2^32 queries: invalidate every stale tag
        std::ranges::fill(labels_, Label{});
        std::ranges::fill(targetMarks_, 0U);
        generation_ = 1;
    }
}

bool ShortestPathEngine::improve(NodeId node, Seconds cost, NodeId parent, std::uint32_t origin) {
    Label& label = labels_[node];
    if (label.reached == generation_ && label.cost <= cost) return false;
    label.cost = cost;
    label.parent = parent;
    label.origin = origin;
    label.reached = generation_;
    return true;
}

bool ShortestPathEngine::isSettled(NodeId node) const noexcept {
    return labels_[node].settled == generation_;
}

std::vector<NodeId> ShortestPathEngine::pathTo(NodeId target) const {
    std::vector<NodeId> path;
    for (NodeId node = target; node != kInvalidNode; node = labels_[node].parent) {
        path.push_back(node);
    }
    std::ranges::reverse(path);
    return path;
}

void ShortestPathEngine::requireNode(NodeId node) const {
    if (node >= network_.nodeCount()) {
        throw std::out_of_range("node " + std::to_string(node) + " is not in the road network");
    }
}

}  // namespace waypoint::routing
