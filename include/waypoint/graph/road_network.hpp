#pragma once

#include <cassert>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "waypoint/core/types.hpp"
#include "waypoint/geo/bounding_box.hpp"
#include "waypoint/geo/point.hpp"

namespace waypoint::graph {

/// Which way a search walks the road graph. Reverse search follows edges
/// backwards, answering "how long from X to here" instead of "from here to X".
enum class Direction { Forward, Reverse };

struct RoadAttributes {
    double speedKmh = 30.0;          ///< free-flow speed limit
    double trafficMultiplier = 1.0;  ///< congestion slowdown, always >= 1
    std::optional<double> lengthKm;  ///< defaults to the straight-line length
};

/// A directed road segment.
struct Road {
    NodeId from;
    NodeId to;
    double lengthKm;
    double speedKmh;
    double trafficMultiplier;
};

/// Immutable-topology road graph stored as two compressed sparse rows (CSR):
/// one indexed by tail for forward search, one indexed by head for reverse
/// search. Each node's arcs are contiguous, so relaxing a node is a linear scan.
///
/// Traffic is the only mutable state. Edge costs are travel times, and every
/// road obeys  length >= straight-line distance,  speed <= maxSpeedKmh(),
/// trafficMultiplier >= 1.  Together these make lowerBoundTravelTime() an
/// admissible and consistent A* heuristic, which the class enforces on input.
class RoadNetwork {
public:
    struct Arc {
        NodeId neighbor;
        EdgeId edge;
    };

    [[nodiscard]] std::size_t nodeCount() const noexcept { return positions_.size(); }
    [[nodiscard]] std::size_t edgeCount() const noexcept { return roads_.size(); }

    /// Precondition for the accessors below: ids are < nodeCount() / edgeCount().
    [[nodiscard]] geo::Point position(NodeId node) const noexcept {
        assert(node < positions_.size());
        return positions_[node];
    }
    [[nodiscard]] const Road& road(EdgeId edge) const { return roads_.at(edge); }
    [[nodiscard]] const geo::BoundingBox& bounds() const noexcept { return bounds_; }
    [[nodiscard]] double maxSpeedKmh() const noexcept { return maxSpeedKmh_; }

    /// Outgoing arcs for Forward, incoming arcs for Reverse.
    [[nodiscard]] std::span<const Arc> arcs(NodeId node, Direction direction) const noexcept {
        assert(node < positions_.size());
        const Csr& csr = direction == Direction::Forward ? outgoing_ : incoming_;
        return std::span(csr.arcs).subspan(csr.offsets[node], csr.offsets[node + 1] - csr.offsets[node]);
    }

    /// Current travel time across an edge, traffic included.
    [[nodiscard]] Seconds travelTime(EdgeId edge) const noexcept {
        assert(edge < travelTime_.size());
        return travelTime_[edge];
    }

    /// A travel time no route between the two points can beat.
    [[nodiscard]] Seconds lowerBoundTravelTime(geo::Point from, geo::Point to) const noexcept;

    /// Applies a live traffic update. Throws std::invalid_argument if multiplier < 1,
    /// since a speed-up beyond free flow would break heuristic admissibility.
    void setTrafficMultiplier(EdgeId edge, double multiplier);

private:
    friend class RoadNetworkBuilder;

    struct Csr {
        std::vector<std::size_t> offsets;  // nodeCount + 1 entries
        std::vector<Arc> arcs;
    };

    RoadNetwork() = default;

    std::vector<geo::Point> positions_;
    std::vector<Road> roads_;
    std::vector<Seconds> travelTime_;
    Csr outgoing_;
    Csr incoming_;
    geo::BoundingBox bounds_;
    double maxSpeedKmh_ = 0.0;
};

/// Collects intersections and roads, validates them, and freezes them into a RoadNetwork.
class RoadNetworkBuilder {
public:
    NodeId addIntersection(geo::Point position);

    /// Adds a one-way road. Throws std::invalid_argument on unknown nodes,
    /// self-loops, non-positive speed, traffic < 1, or a length shorter than
    /// the straight-line distance between the endpoints.
    EdgeId addRoad(NodeId from, NodeId to, const RoadAttributes& attributes = {});

    /// Adds a pair of opposing one-way roads with identical attributes.
    void addTwoWayRoad(NodeId a, NodeId b, const RoadAttributes& attributes = {});

    [[nodiscard]] std::size_t nodeCount() const noexcept { return positions_.size(); }

    [[nodiscard]] RoadNetwork build() &&;

private:
    std::vector<geo::Point> positions_;
    std::vector<Road> roads_;
};

}  // namespace waypoint::graph
