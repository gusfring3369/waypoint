#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "waypoint/geo/bounding_box.hpp"
#include "waypoint/geo/point.hpp"

namespace waypoint::spatial {

/// Tuning knobs for QuadTree.
struct QuadTreeConfig {
    std::size_t leafCapacity = 8;  ///< points a cell may hold before splitting
    std::size_t maxDepth = 24;     ///< stops endless splitting when many points share one spot
};

/// Finds points near a location without looking at every point.
///
/// The map area is a rectangle ("cell"). When a cell holds more than
/// `leafCapacity` points it splits into four equal quadrants, and so on, so
/// busy areas get small cells and quiet areas stay as one large cell. A query
/// skips every cell that cannot contain an answer.
///
/// Built for moving objects such as riders: remove + insert moves a point,
/// and when removals leave four sibling cells nearly empty they merge back.
class QuadTree {
public:
    using Id = std::uint32_t;

    struct Entry {
        Id id;
        geo::Point position;
    };

    using Config = QuadTreeConfig;

    explicit QuadTree(geo::BoundingBox bounds, Config config = {});

    /// Returns false (and stores nothing) when `position` is outside the bounds.
    [[nodiscard]] bool insert(Id id, geo::Point position);

    /// `position` must be the one the entry was inserted with. False if not found.
    bool remove(Id id, geo::Point position);

    /// Every entry within `radius` of `center`, in no particular order.
    [[nodiscard]] std::vector<Entry> withinRadius(geo::Point center, double radius) const;

    /// Up to `k` entries closest to `center` and within `maxRadius`, nearest first.
    [[nodiscard]] std::vector<Entry> nearest(
        geo::Point center, std::size_t k, double maxRadius = std::numeric_limits<double>::infinity()) const;

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
    [[nodiscard]] const geo::BoundingBox& bounds() const noexcept { return root_.box; }

private:
    struct Node {
        geo::BoundingBox box;
        std::vector<Entry> entries;        // only leaves hold entries
        std::unique_ptr<Node[]> children;  // null for a leaf, otherwise 4 quadrants

        [[nodiscard]] bool isLeaf() const noexcept { return children == nullptr; }
        [[nodiscard]] Node& childFor(geo::Point p) { return children[box.quadrantOf(p)]; }
    };

    static constexpr std::size_t kQuadrants = 4;

    void insertInto(Node& node, const Entry& entry, std::size_t depth);
    void split(Node& node, std::size_t depth);
    bool removeFrom(Node& node, Id id, geo::Point position);
    void mergeIfSparse(Node& node);
    static void collectWithinRadius(const Node& node, geo::Point center, double radiusSq,
                                    std::vector<Entry>& out);
    static void collectAll(const Node& node, std::vector<Entry>& out);

    Config config_;
    Node root_;
    std::size_t size_ = 0;
};

}  // namespace waypoint::spatial
