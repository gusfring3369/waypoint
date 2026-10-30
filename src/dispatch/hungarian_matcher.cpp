#include "waypoint/dispatch/hungarian_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace waypoint::dispatch {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Largest allowed cost in the matrix. Rejects negative or NaN costs.
double largestAllowedCost(const CostMatrix& costs) {
    double largest = 0.0;
    for (std::size_t r = 0; r < costs.rows(); ++r) {
        for (std::size_t c = 0; c < costs.cols(); ++c) {
            if (!costs.isAllowed(r, c)) continue;
            const double cost = costs(r, c);
            if (!std::isfinite(cost) || cost < 0.0) {
                throw std::invalid_argument("matching costs must be finite and >= 0");
            }
            largest = std::max(largest, cost);
        }
    }
    return largest;
}

// The solver needs every cell finite and no more rows than columns. Forbidden
// pairs get a penalty bigger than any complete set of allowed pairs, so the
// solver uses as few of them as it can, which means serving as many orders as
// possible. Transposing swaps rows and columns when there are more rows.
CostMatrix toSolvableForm(const CostMatrix& costs, bool transpose) {
    const std::size_t pairsInAnyMatching = std::min(costs.rows(), costs.cols());
    const double penalty = static_cast<double>(pairsInAnyMatching) * largestAllowedCost(costs) + 1.0;

    CostMatrix solvable(transpose ? costs.cols() : costs.rows(), transpose ? costs.rows() : costs.cols());
    for (std::size_t r = 0; r < costs.rows(); ++r) {
        for (std::size_t c = 0; c < costs.cols(); ++c) {
            const double cost = costs.isAllowed(r, c) ? costs(r, c) : penalty;
            (transpose ? solvable(c, r) : solvable(r, c)) = cost;
        }
    }
    return solvable;
}

// Kuhn-Munkres for an n x m matrix with n <= m: every row gets its own column.
//
// Each row r and column c carries a "potential" (rowPotential, colPotential),
// kept so that  cost(r, c) - rowPotential[r] - colPotential[c] >= 0  everywhere.
// Pairs where this "reduced cost" is 0 are called tight, and a matching made
// only of tight pairs is guaranteed optimal.
//
// Rows are added one at a time. From the new row, the search grows a tree of
// tight pairs (much like Dijkstra) until it reaches a free column, shifting
// potentials whenever it gets stuck. Then it walks back along the tree and
// moves every row on that path to its new column.
//
// Indices are 1-based so that column 0 can be the virtual start of each path.
Matching solveRowsAtMostCols(const CostMatrix& cost) {
    const std::size_t n = cost.rows();
    const std::size_t m = cost.cols();
    std::vector<double> rowPotential(n + 1, 0.0);
    std::vector<double> colPotential(m + 1, 0.0);
    std::vector<std::size_t> rowOfCol(m + 1, 0);  // 0 = column still free
    std::vector<std::size_t> previousCol(m + 1, 0);
    std::vector<double> minSlack(m + 1);
    std::vector<bool> inTree(m + 1);

    for (std::size_t newRow = 1; newRow <= n; ++newRow) {
        rowOfCol[0] = newRow;
        std::size_t col = 0;
        std::fill(minSlack.begin(), minSlack.end(), kInfinity);
        std::fill(inTree.begin(), inTree.end(), false);

        // Grow the tree until it reaches a free column.
        do {
            inTree[col] = true;
            const std::size_t row = rowOfCol[col];
            double delta = kInfinity;
            std::size_t nextCol = 0;
            for (std::size_t j = 1; j <= m; ++j) {
                if (inTree[j]) continue;
                const double slack = cost(row - 1, j - 1) - rowPotential[row] - colPotential[j];
                if (slack < minSlack[j]) {
                    minSlack[j] = slack;
                    previousCol[j] = col;
                }
                if (minSlack[j] < delta) {
                    delta = minSlack[j];
                    nextCol = j;
                }
            }
            for (std::size_t j = 0; j <= m; ++j) {  // shift potentials so nextCol becomes tight
                if (inTree[j]) {
                    rowPotential[rowOfCol[j]] += delta;
                    colPotential[j] -= delta;
                } else {
                    minSlack[j] -= delta;
                }
            }
            col = nextCol;
        } while (rowOfCol[col] != 0);

        // Walk back along the path, moving each row one column along.
        do {
            const std::size_t previous = previousCol[col];
            rowOfCol[col] = rowOfCol[previous];
            col = previous;
        } while (col != 0);
    }

    Matching matching(n);
    for (std::size_t j = 1; j <= m; ++j) {
        if (rowOfCol[j] != 0) matching[rowOfCol[j] - 1] = j - 1;
    }
    return matching;
}

}  // namespace

Matching HungarianMatcher::match(const CostMatrix& costs) const {
    Matching matching(costs.rows());
    if (costs.rows() == 0 || costs.cols() == 0) return matching;

    const bool transpose = costs.rows() > costs.cols();
    const Matching solved = solveRowsAtMostCols(toSolvableForm(costs, transpose));

    // Undo the transpose and drop any forbidden pairs the solver was forced to use.
    for (std::size_t i = 0; i < solved.size(); ++i) {
        const std::optional<std::size_t> partner = solved[i];
        if (!partner) continue;
        const std::size_t order = transpose ? *partner : i;
        const std::size_t rider = transpose ? i : *partner;
        if (costs.isAllowed(order, rider)) matching[order] = rider;
    }
    return matching;
}

}  // namespace waypoint::dispatch
