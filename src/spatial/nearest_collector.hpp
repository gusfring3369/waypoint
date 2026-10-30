#pragma once

#include <algorithm>
#include <cstddef>
#include <queue>
#include <vector>

#include "waypoint/geo/point.hpp"
#include "waypoint/spatial/quad_tree.hpp"

namespace waypoint::spatial {

/// Keeps the `k` entries closest to `center` out of all entries offered.
/// Internal helper for QuadTree::nearest().
class NearestCollector {
public:
    NearestCollector(geo::Point center, std::size_t k, double maxRadius)
        : center_(center), k_(k), maxRadiusSq_(maxRadius * maxRadius) {}

    void offer(const QuadTree::Entry& entry) {
        const double distanceSq = geo::squaredDistance(entry.position, center_);
        if (distanceSq > cutoffSq()) return;
        best_.push({distanceSq, entry});
        if (best_.size() > k_) best_.pop();  // drop the farthest
    }

    /// Anything farther than this (squared) cannot make it into the result.
    [[nodiscard]] double cutoffSq() const noexcept {
        return best_.size() == k_ ? best_.top().distanceSq : maxRadiusSq_;
    }

    /// The collected entries, nearest first. Empties the collector.
    [[nodiscard]] std::vector<QuadTree::Entry> takeSorted() {
        std::vector<QuadTree::Entry> result;
        result.reserve(best_.size());
        for (; !best_.empty(); best_.pop()) result.push_back(best_.top().entry);
        std::ranges::reverse(result);
        return result;
    }

private:
    struct Candidate {
        double distanceSq;
        QuadTree::Entry entry;
        bool operator<(const Candidate& other) const noexcept { return distanceSq < other.distanceSq; }
    };

    geo::Point center_;
    std::size_t k_;
    double maxRadiusSq_;
    std::priority_queue<Candidate> best_;  // max-heap: top() is the farthest kept so far
};

}  // namespace waypoint::spatial
