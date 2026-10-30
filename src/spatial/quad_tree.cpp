#include "waypoint/spatial/quad_tree.hpp"

#include <algorithm>
#include <iterator>
#include <queue>
#include <utility>

#include "nearest_collector.hpp"

namespace waypoint::spatial {

QuadTree::QuadTree(geo::BoundingBox bounds, Config config) : config_(config), root_{bounds, {}, nullptr} {}

// ---------------------------------------------------------------------------
// Insert
// ---------------------------------------------------------------------------

bool QuadTree::insert(Id id, geo::Point position) {
    if (!root_.box.contains(position)) return false;
    insertInto(root_, {id, position}, 0);
    ++size_;
    return true;
}

void QuadTree::insertInto(Node& node, const Entry& entry, std::size_t depth) {
    if (!node.isLeaf()) {
        insertInto(node.childFor(entry.position), entry, depth + 1);
        return;
    }
    node.entries.push_back(entry);
    if (node.entries.size() > config_.leafCapacity && depth < config_.maxDepth) split(node, depth);
}

// Turns a full leaf into four child cells and moves its entries down into them.
void QuadTree::split(Node& node, std::size_t depth) {
    const auto quadrants = node.box.quadrants();
    node.children = std::make_unique<Node[]>(kQuadrants);
    for (std::size_t q = 0; q < kQuadrants; ++q) node.children[q].box = quadrants[q];

    for (const Entry& entry : std::exchange(node.entries, {})) {
        insertInto(node.childFor(entry.position), entry, depth + 1);
    }
}

// ---------------------------------------------------------------------------
// Remove
// ---------------------------------------------------------------------------

bool QuadTree::remove(Id id, geo::Point position) {
    if (!root_.box.contains(position) || !removeFrom(root_, id, position)) return false;
    --size_;
    return true;
}

bool QuadTree::removeFrom(Node& node, Id id, geo::Point position) {
    if (node.isLeaf()) {
        const auto it = std::ranges::find(node.entries, id, &Entry::id);
        if (it == node.entries.end()) return false;
        *it = node.entries.back();  // order inside a leaf doesn't matter
        node.entries.pop_back();
        return true;
    }
    if (!removeFrom(node.childFor(position), id, position)) return false;
    mergeIfSparse(node);
    return true;
}

// If all four children are leaves and together fit in one leaf, fold them back in.
void QuadTree::mergeIfSparse(Node& node) {
    std::size_t total = 0;
    for (std::size_t q = 0; q < kQuadrants; ++q) {
        if (!node.children[q].isLeaf()) return;
        total += node.children[q].entries.size();
    }
    if (total > config_.leafCapacity) return;

    node.entries.reserve(total);
    for (std::size_t q = 0; q < kQuadrants; ++q) {
        std::ranges::copy(node.children[q].entries, std::back_inserter(node.entries));
    }
    node.children.reset();
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

std::vector<QuadTree::Entry> QuadTree::withinRadius(geo::Point center, double radius) const {
    std::vector<Entry> found;
    collectWithinRadius(root_, center, radius * radius, found);
    return found;
}

void QuadTree::collectWithinRadius(const Node& node, geo::Point center, double radiusSq,
                                   std::vector<Entry>& out) {
    if (node.box.squaredDistanceTo(center) > radiusSq) return;     // cell entirely outside the circle
    if (node.box.squaredDistanceToFarthest(center) <= radiusSq) {  // cell entirely inside
        collectAll(node, out);
        return;
    }
    if (node.isLeaf()) {
        for (const Entry& entry : node.entries) {
            if (geo::squaredDistance(entry.position, center) <= radiusSq) out.push_back(entry);
        }
        return;
    }
    for (std::size_t q = 0; q < kQuadrants; ++q) collectWithinRadius(node.children[q], center, radiusSq, out);
}

void QuadTree::collectAll(const Node& node, std::vector<Entry>& out) {
    out.insert(out.end(), node.entries.begin(), node.entries.end());
    if (node.isLeaf()) return;
    for (std::size_t q = 0; q < kQuadrants; ++q) collectAll(node.children[q], out);
}

// Visits cells closest-first. Once the closest unvisited cell is farther away
// than the k-th best entry found so far, nothing left can improve the answer.
std::vector<QuadTree::Entry> QuadTree::nearest(geo::Point center, std::size_t k, double maxRadius) const {
    if (k == 0) return {};
    NearestCollector best(center, k, maxRadius);

    using CellByDistance = std::pair<double, const Node*>;  // (squared distance to cell, cell)
    const auto fartherFirst = [](const CellByDistance& a, const CellByDistance& b) {
        return a.first > b.first;
    };
    std::priority_queue<CellByDistance, std::vector<CellByDistance>, decltype(fartherFirst)> cells(
        fartherFirst);
    cells.emplace(root_.box.squaredDistanceTo(center), &root_);

    while (!cells.empty() && cells.top().first <= best.cutoffSq()) {
        const Node& node = *cells.top().second;
        cells.pop();

        if (node.isLeaf()) {
            for (const Entry& entry : node.entries) best.offer(entry);
            continue;
        }
        for (std::size_t q = 0; q < kQuadrants; ++q) {
            const Node& child = node.children[q];
            cells.emplace(child.box.squaredDistanceTo(center), &child);
        }
    }
    return best.takeSorted();
}

}  // namespace waypoint::spatial
