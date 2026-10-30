#pragma once

#include <cassert>
#include <cstddef>
#include <vector>

#include "waypoint/core/types.hpp"

namespace waypoint::dispatch {

/// A table of costs: rows are orders, columns are riders, each cell is the
/// rider's pickup ETA for that order. kUnreachable marks a pair that is not
/// allowed (rider too far away or no route).
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

    [[nodiscard]] bool isAllowed(std::size_t row, std::size_t col) const noexcept {
        return (*this)(row, col) != kUnreachable;
    }

private:
    std::size_t rows_;
    std::size_t cols_;
    std::vector<double> cells_;  // row by row in one block
};

}  // namespace waypoint::dispatch
