#include "waypoint/dispatch/matching.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace waypoint::dispatch {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

bool isForbidden(double cost) noexcept {
    return cost == kUnreachable;
}

// Kuhn-Munkres for a dense n x m matrix with n <= m (every row gets a column).
// Rows are added one at a time; each addition grows a shortest augmenting path
// over reduced costs a[i][j] - u[i] - v[j], keeping the dual potentials u, v
// feasible so the final assignment is optimal. Indices are 1-based internally
// so that column 0 can act as the virtual root of each augmenting path.
Matching hungarian(const CostMatrix& a) {
    const std::size_t n = a.rows();
    const std::size_t m = a.cols();
    std::vector<double> u(n + 1, 0.0);
    std::vector<double> v(m + 1, 0.0);
    std::vector<std::size_t> rowOfCol(m + 1, 0);  // 0 = column unmatched
    std::vector<std::size_t> prevCol(m + 1, 0);
    std::vector<double> minSlack(m + 1);
    std::vector<bool> visited(m + 1);

    for (std::size_t row = 1; row <= n; ++row) {
        rowOfCol[0] = row;
        std::size_t col = 0;
        std::ranges::fill(minSlack, kInfinity);
        std::fill(visited.begin(), visited.end(), false);

        do {  // Dijkstra-like growth of the alternating tree until a free column is reached
            visited[col] = true;
            const std::size_t current = rowOfCol[col];
            double delta = kInfinity;
            std::size_t nextCol = 0;
            for (std::size_t j = 1; j <= m; ++j) {
                if (visited[j]) continue;
                const double slack = a(current - 1, j - 1) - u[current] - v[j];
                if (slack < minSlack[j]) {
                    minSlack[j] = slack;
                    prevCol[j] = col;
                }
                if (minSlack[j] < delta) {
                    delta = minSlack[j];
                    nextCol = j;
                }
            }
            for (std::size_t j = 0; j <= m; ++j) {
                if (visited[j]) {
                    u[rowOfCol[j]] += delta;
                    v[j] -= delta;
                } else {
                    minSlack[j] -= delta;
                }
            }
            col = nextCol;
        } while (rowOfCol[col] != 0);

        do {  // flip the augmenting path
            const std::size_t prev = prevCol[col];
            rowOfCol[col] = rowOfCol[prev];
            col = prev;
        } while (col != 0);
    }

    Matching matching(n);
    for (std::size_t j = 1; j <= m; ++j) {
        if (rowOfCol[j] != 0) matching[rowOfCol[j] - 1] = j - 1;
    }
    return matching;
}

}  // namespace

Matching matchGreedy(const CostMatrix& costs) {
    Matching matching(costs.rows());
    std::vector<bool> taken(costs.cols(), false);
    for (std::size_t row = 0; row < costs.rows(); ++row) {
        std::optional<std::size_t> best;
        for (std::size_t col = 0; col < costs.cols(); ++col) {
            if (taken[col] || isForbidden(costs(row, col))) continue;
            if (!best || costs(row, col) < costs(row, *best)) best = col;
        }
        if (best) taken[*best] = true;
        matching[row] = best;
    }
    return matching;
}

Matching matchOptimal(const CostMatrix& costs) {
    const std::size_t rows = costs.rows();
    const std::size_t cols = costs.cols();
    if (rows == 0 || cols == 0) return Matching(rows);

    double maxAllowed = 0.0;
    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t c = 0; c < cols; ++c) {
            const double cost = costs(r, c);
            if (isForbidden(cost)) continue;
            if (std::isnan(cost) || cost < 0.0 || std::isinf(cost)) {
                throw std::invalid_argument("matching costs must be finite and >= 0");
            }
            maxAllowed = std::max(maxAllowed, cost);
        }
    }

    // Forbidden pairs get a penalty larger than any complete set of allowed
    // pairs, so the optimum uses as few of them as possible (i.e. serves as
    // many rows as possible); they are dropped from the result afterwards.
    const bool transpose = rows > cols;
    const std::size_t n = transpose ? cols : rows;
    const std::size_t m = transpose ? rows : cols;
    const double penalty = static_cast<double>(n) * maxAllowed + 1.0;

    CostMatrix square(n, m);
    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t c = 0; c < cols; ++c) {
            const double cost = isForbidden(costs(r, c)) ? penalty : costs(r, c);
            if (transpose) {
                square(c, r) = cost;
            } else {
                square(r, c) = cost;
            }
        }
    }

    const Matching solved = hungarian(square);
    Matching matching(rows);
    for (std::size_t i = 0; i < n; ++i) {
        const std::optional<std::size_t> partner = solved[i];
        if (!partner) continue;
        const std::size_t r = transpose ? *partner : i;
        const std::size_t c = transpose ? i : *partner;
        if (!isForbidden(costs(r, c))) matching[r] = c;
    }
    return matching;
}

double totalCost(const CostMatrix& costs, const Matching& matching) {
    double total = 0.0;
    for (std::size_t row = 0; row < matching.size(); ++row) {
        if (const auto col = matching[row]) total += costs(row, *col);
    }
    return total;
}

}  // namespace waypoint::dispatch
