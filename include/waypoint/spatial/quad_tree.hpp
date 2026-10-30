#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "waypoint/geo/bounding_box.hpp"
#include "waypoint/geo/point.hpp"

namespace waypoint::spatial {

/// Point-region QuadTree for proximity queries over moving objects.
///
/// Entries live only in leaves. A leaf splits into four quadrants once it holds
/// more than `leafCapacity` entries, and four sibling leaves merge back into
/// their parent once a removal brings them under capacity, so the tree tracks
/// the current distribution of points rather than its history. `maxDepth`
/// bounds the recursion when many points share one coordinate.
///
/// Move-only (children are owned through unique_ptr); not thread-safe.
template <std::equality_comparable Id>
class QuadTree {
public:
    struct Entry {
        Id id;
        geo::Point position;
    };

    struct Config {
        std::size_t leafCapacity = 8;
        std::size_t maxDepth = 24;
    };

    explicit QuadTree(geo::BoundingBox bounds, Config config = {})
        : config_(config), root_{bounds, {}, nullptr} {}

    /// Returns false (and stores nothing) when `position` lies outside the tree bounds.
    [[nodiscard]] bool insert(Id id, geo::Point position) {
        if (!root_.box.contains(position)) return false;
        insertInto(root_, Entry{std::move(id), position}, 0);
        ++size_;
        return true;
    }

    /// `position` must be the position the entry was inserted with.
    bool remove(const Id& id, geo::Point position) {
        if (!root_.box.contains(position) || !removeFrom(root_, id, position)) return false;
        --size_;
        return true;
    }

    /// All entries within `radius` of `center`, in no particular order.
    [[nodiscard]] std::vector<Entry> withinRadius(geo::Point center, double radius) const {
        std::vector<Entry> found;
        collectWithinRadius(root_, center, radius * radius, found);
        return found;
    }

    /// Up to `k` entries closest to `center` and within `maxRadius`, nearest first.
    [[nodiscard]] std::vector<Entry> nearest(
        geo::Point center, std::size_t k, double maxRadius = std::numeric_limits<double>::infinity()) const;

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
    [[nodiscard]] const geo::BoundingBox& bounds() const noexcept { return root_.box; }

private:
    struct Node {
        geo::BoundingBox box;
        std::vector<Entry> entries;
        std::unique_ptr<Node[]> children;  // null for leaves, otherwise exactly 4

        [[nodiscard]] bool isLeaf() const noexcept { return children == nullptr; }
        [[nodiscard]] Node& childFor(geo::Point p) { return children[box.quadrantOf(p)]; }
        [[nodiscard]] const Node& childFor(geo::Point p) const { return children[box.quadrantOf(p)]; }
    };

    static constexpr std::size_t kQuadrants = 4;

    void insertInto(Node& node, Entry entry, std::size_t depth) {
        if (!node.isLeaf()) {
            insertInto(node.childFor(entry.position), std::move(entry), depth + 1);
            return;
        }
        node.entries.push_back(std::move(entry));
        if (node.entries.size() > config_.leafCapacity && depth < config_.maxDepth) {
            split(node, depth);
        }
    }

    void split(Node& node, std::size_t depth) {
        const auto boxes = node.box.quadrants();
        node.children = std::make_unique<Node[]>(kQuadrants);
        for (std::size_t q = 0; q < kQuadrants; ++q) node.children[q].box = boxes[q];

        for (Entry& entry : std::exchange(node.entries, {})) {
            insertInto(node.childFor(entry.position), std::move(entry), depth + 1);
        }
    }

    bool removeFrom(Node& node, const Id& id, geo::Point position) {
        if (node.isLeaf()) {
            auto it = std::ranges::find(node.entries, id, &Entry::id);
            if (it == node.entries.end()) return false;
            *it = std::move(node.entries.back());
            node.entries.pop_back();
            return true;
        }
        if (!removeFrom(node.childFor(position), id, position)) return false;
        collapseIfSparse(node);
        return true;
    }

    void collapseIfSparse(Node& node) {
        std::size_t total = 0;
        for (std::size_t q = 0; q < kQuadrants; ++q) {
            const Node& child = node.children[q];
            if (!child.isLeaf()) return;
            total += child.entries.size();
        }
        if (total > config_.leafCapacity) return;

        node.entries.reserve(total);
        for (std::size_t q = 0; q < kQuadrants; ++q) {
            auto& childEntries = node.children[q].entries;
            std::ranges::move(childEntries, std::back_inserter(node.entries));
        }
        node.children.reset();
    }

    void collectWithinRadius(const Node& node, geo::Point center, double radiusSq,
                             std::vector<Entry>& out) const {
        if (node.box.squaredDistanceTo(center) > radiusSq) return;     // disjoint: prune subtree
        if (node.box.squaredDistanceToFarthest(center) <= radiusSq) {  // inside: no checks needed
            collectAll(node, out);
            return;
        }
        if (node.isLeaf()) {
            for (const Entry& entry : node.entries) {
                if (geo::squaredDistance(entry.position, center) <= radiusSq) out.push_back(entry);
            }
            return;
        }
        for (std::size_t q = 0; q < kQuadrants; ++q) {
            collectWithinRadius(node.children[q], center, radiusSq, out);
        }
    }

    void collectAll(const Node& node, std::vector<Entry>& out) const {
        out.insert(out.end(), node.entries.begin(), node.entries.end());
        if (node.isLeaf()) return;
        for (std::size_t q = 0; q < kQuadrants; ++q) collectAll(node.children[q], out);
    }

    Config config_;
    Node root_;
    std::size_t size_ = 0;
};

// Best-first traversal: quadrants are visited in order of their distance to
// `center`, and the search stops as soon as the closest unvisited quadrant is
// farther away than the k-th best entry found so far.
template <std::equality_comparable Id>
auto QuadTree<Id>::nearest(geo::Point center, std::size_t k, double maxRadius) const -> std::vector<Entry> {
    if (k == 0 || empty()) return {};

    struct PendingNode {
        double distanceSq;
        const Node* node;
    };
    struct Candidate {
        double distanceSq;
        Entry entry;
    };
    const auto fartherNode = [](const PendingNode& a, const PendingNode& b) {
        return a.distanceSq > b.distanceSq;
    };
    const auto closerCandidate = [](const Candidate& a, const Candidate& b) {
        return a.distanceSq < b.distanceSq;
    };

    std::priority_queue<PendingNode, std::vector<PendingNode>, decltype(fartherNode)> frontier(fartherNode);
    std::priority_queue<Candidate, std::vector<Candidate>, decltype(closerCandidate)> best(
        closerCandidate);  // max-heap: top is the worst of the current k best

    double limitSq = maxRadius * maxRadius;
    frontier.push({root_.box.squaredDistanceTo(center), &root_});

    while (!frontier.empty() && frontier.top().distanceSq <= limitSq) {
        const Node& node = *frontier.top().node;
        frontier.pop();

        if (!node.isLeaf()) {
            for (std::size_t q = 0; q < kQuadrants; ++q) {
                const double d = node.children[q].box.squaredDistanceTo(center);
                if (d <= limitSq) frontier.push({d, &node.children[q]});
            }
            continue;
        }
        for (const Entry& entry : node.entries) {
            const double d = geo::squaredDistance(entry.position, center);
            if (d > limitSq) continue;
            best.push({d, entry});
            if (best.size() > k) best.pop();
            if (best.size() == k) limitSq = best.top().distanceSq;
        }
    }

    std::vector<Entry> result;
    result.reserve(best.size());
    for (; !best.empty(); best.pop()) result.push_back(best.top().entry);
    std::ranges::reverse(result);
    return result;
}

}  // namespace waypoint::spatial
