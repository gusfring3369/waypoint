#pragma once

#include <cstdint>
#include <limits>

namespace waypoint {

using NodeId = std::uint32_t;
using EdgeId = std::uint32_t;
using RiderId = std::uint32_t;
using OrderId = std::uint32_t;

/// All routing costs are travel times in seconds.
using Seconds = double;

inline constexpr NodeId kInvalidNode = std::numeric_limits<NodeId>::max();
inline constexpr Seconds kUnreachable = std::numeric_limits<Seconds>::infinity();

}  // namespace waypoint
