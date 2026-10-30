#include "waypoint/graph/adjacency_index.hpp"

namespace waypoint::graph {

AdjacencyIndex::AdjacencyIndex(std::size_t nodeCount, std::span<const Road> roads, Direction direction)
    : offsets_(nodeCount + 1, 0), arcs_(roads.size()) {
    const bool forward = direction == Direction::Forward;
    const auto ownerOf = [forward](const Road& road) { return forward ? road.from : road.to; };
    const auto neighborOf = [forward](const Road& road) { return forward ? road.to : road.from; };

    // Counting sort of the roads by owning node:
    // 1. count each node's arcs,
    for (const Road& road : roads) ++offsets_[ownerOf(road) + 1];
    // 2. turn the counts into start positions (prefix sums),
    for (std::size_t node = 0; node < nodeCount; ++node) offsets_[node + 1] += offsets_[node];
    // 3. drop each arc into the next free slot of its node.
    std::vector<std::size_t> nextSlot(offsets_.begin(), offsets_.end() - 1);
    for (EdgeId edge = 0; edge < roads.size(); ++edge) {
        const Road& road = roads[edge];
        arcs_[nextSlot[ownerOf(road)]++] = {neighborOf(road), edge};
    }
}

}  // namespace waypoint::graph
