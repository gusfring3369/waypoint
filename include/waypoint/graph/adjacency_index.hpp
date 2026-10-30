#pragma once

#include <cassert>
#include <cstddef>
#include <span>
#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/graph/road.hpp"

namespace waypoint::graph {

/// One step out of an intersection: where it leads and which road it uses.
struct Arc {
    NodeId neighbor;
    EdgeId edge;
};

/// For every intersection, the roads touching it in one direction (leaving it
/// for Forward, entering it for Reverse).
///
/// Stored in "compressed sparse row" (CSR) form: all per-node lists are packed
/// back to back into one array, and node n owns the slice
/// arcs_[offsets_[n] .. offsets_[n + 1]). Compared with vector<vector<Arc>>
/// that is one allocation instead of one per node, and a node's arcs are
/// contiguous in memory, which keeps the search loop cache-friendly.
class AdjacencyIndex {
public:
    AdjacencyIndex() = default;
    AdjacencyIndex(std::size_t nodeCount, std::span<const Road> roads, Direction direction);

    [[nodiscard]] std::span<const Arc> arcsOf(NodeId node) const noexcept {
        assert(node + 1 < offsets_.size());
        return std::span(arcs_).subspan(offsets_[node], offsets_[node + 1] - offsets_[node]);
    }

private:
    std::vector<std::size_t> offsets_;  // nodeCount + 1 entries
    std::vector<Arc> arcs_;
};

}  // namespace waypoint::graph
