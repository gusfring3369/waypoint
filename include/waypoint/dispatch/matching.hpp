#pragma once

#include <cassert>
#include <cstddef>
#include <optional>
#include <vector>

#include "waypoint/core/types.hpp"

namespace waypoint::dispatch {

/// Dense row-major cost matrix (rows = orders, columns = riders).
/// kUnreachable marks a pair that must not be matched.
class CostMatrix {
public:
    CostMatrix(std::size_t rows, std::size_t cols, double fill = kUnreachable)
        : rows_(rows), cols_(cols), cells_(rows * cols, fill) {}

    [[nodiscard]] std::size_t rows() const noexcept { return rows_; }
    [[nodiscard]] std::size_t cols() const noexcept { return cols_; }

    [[nodiscard]] double& operator()(std::size_t row, std::size_t col) noexcept {
        assert(row < rows_ && col < cols_);
        return cells_[row * cols_ + col];
    }
    [[nodiscard]] double operator()(std::size_t row, std::size_t col) const noexcept {
        assert(row < rows_ && col < cols_);
        return cells_[row * cols_ + col];
    }

private:
    std::size_t rows_;
    std::size_t cols_;
    std::vector<double> cells_;
};

/// For each row, the column matched to it, if any. No column appears twice.
using Matching = std::vector<std::optional<std::size_t>>;

/// First-come-first-served baseline: rows in order, each takes its cheapest
/// still-free column. O(R * C), but an early row can take the only rider a
/// later row could use, so the total cost is not minimal.
[[nodiscard]] Matching matchGreedy(const CostMatrix& costs);

/// Globally optimal assignment (Hungarian / Kuhn-Munkres with potentials,
/// O(n^2 m) for n = min(R, C)). Maximises the number of matched rows first,
/// then minimises total cost among those matchings. Costs must be >= 0.
[[nodiscard]] Matching matchOptimal(const CostMatrix& costs);

/// Sum of the costs of the matched pairs.
[[nodiscard]] double totalCost(const CostMatrix& costs, const Matching& matching);

}  // namespace waypoint::dispatch
