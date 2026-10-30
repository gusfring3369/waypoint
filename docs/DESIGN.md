# Design notes

This document explains *why* Waypoint is built the way it is: the trade-offs behind each decision, what was rejected, and how the system would change at production scale.

## 1. Costs are travel times, not distances

A dispatcher cares about *when* a rider arrives, not how far they drive. A 3 km arterial at 60 km/h beats a 2 km side street at 30 km/h with traffic. Every edge therefore stores `length / speed × traffic` in seconds, and every API returns `Seconds`.

This choice shapes the heuristic. Straight-line *distance* is not a lower bound on travel *time*. Straight-line distance **divided by the fastest speed on the network** is.

## 2. Admissibility is enforced, not assumed

A* returns optimal routes only if the heuristic never overestimates. With h(u) = ‖u − t‖ / v_max:

- For any road (u, v): ‖u − v‖ ≤ length(u, v), because `RoadNetworkBuilder` rejects shorter lengths.
- Its travel time is length / speed × traffic ≥ ‖u − v‖ / v_max, because speed ≤ v_max and traffic ≥ 1.
- By the triangle inequality, h(u) ≤ ‖u − v‖ / v_max + h(v) ≤ c(u, v) + h(v).

So h is **consistent**, which also implies admissible. Consistency means a node's cost is final the first time it is popped, which lets the search loop keep a simple "settled" flag and never re-open nodes.

The code enforces each premise at the boundary where it could be broken:

| Premise | Enforced by |
|---|---|
| length ≥ straight line | `RoadNetworkBuilder::addRoad` throws |
| traffic ≥ 1 | builder and `RoadNetwork::setTrafficMultiplier` throw |
| v_max is the true maximum | computed in `build()` from every road |

**Trade-off:** a city with one 120 km/h highway weakens the heuristic everywhere, because v_max is global. ALT landmarks (precomputed distances to a few landmark nodes plus the triangle inequality) give much tighter bounds. That is the natural next step.

## 3. Builder, then an immutable-topology graph

`RoadNetworkBuilder` validates input and produces a `RoadNetwork` whose topology cannot change afterwards.

- **Invalid states are unrepresentable.** Every edge in a `RoadNetwork` has already passed validation, so search code needs no defensive checks.
- **CSR storage.** Freezing the topology allows compressed sparse rows: one `offsets` array plus one contiguous `arcs` array per direction. Relaxing a node is a linear scan over adjacent memory, unlike `vector<vector<Edge>>` with one heap allocation per node.
- **Both directions.** The reverse CSR makes "how long from each rider *to* this pickup" a single search from the pickup, instead of one search per rider.
- **Traffic stays mutable.** Travel times live in a separate array indexed by `EdgeId`, so a live update is O(1) and never touches topology.

An adjacency matrix was never an option. Road graphs average about 3.5 arcs per node, so a 40,000-node matrix would hold 1.6 × 10⁹ cells, almost all of them empty.

## 4. One search loop

Dijkstra, A\*, one-to-many and multi-source search are the same algorithm with different seeds, heuristics and stopping rules. `ShortestPathEngine::search` is written once, as a template over the heuristic and a stop callback. Each public method is a few lines that configure it.

- **Lazy deletion instead of decrease-key.** `std::priority_queue` has no decrease-key. When a node's cost improves we push it again, and the stale copy is skipped when popped (`settled == generation`). This keeps a cache-friendly binary heap and the same O((V + E) log V) bound.
- **Generation stamps.** Labels (`cost`, `parent`, `origin`) are allocated once per engine. Instead of resetting O(V) arrays per query, each query increments a 32-bit generation, and a label counts only if its stamp matches. On wrap-around (after 2³² queries) everything is reset once.
- **Multi-source A\*.** All candidate riders start at cost 0, each remembering its index as `origin`. Because the heuristic depends only on the node, the first label to settle the pickup comes from the rider with the fastest route. The result is exact, for about the cost of one search.

**Thread-safety contract:** an engine owns mutable scratch space, so use one per thread. The `RoadNetwork` can be shared read-only.

## 5. QuadTree design

| Decision | Reason |
|---|---|
| Entries only in leaves | Internal nodes are pure routing, so queries and removals follow one path |
| `quadrantOf` arithmetic instead of "try each child" | O(1) child choice. A point on a split line has exactly one home, so it cannot be duplicated or lost |
| Merge on removal | Riders move constantly. Without merging the tree keeps every split it ever made |
| `maxDepth` | Many riders at one coordinate (a restaurant pickup point) would otherwise split forever |
| `unique_ptr<Node[]>` children | No manual `delete`. The tree is move-only by construction (Rule of Zero) |
| Best-first k-nearest | Stops as soon as no unexplored quadrant can beat the current k-th best |

**Alternatives considered**

- **Uniform grid or geohash buckets.** Simpler, with O(1) updates, but the cell size must be tuned to density, and dense downtown cells degrade. A good choice when density is predictable.
- **k-d tree.** Excellent for static points, but rebalancing under constant updates is awkward.
- **R-tree.** The right choice for indexing *shapes* (delivery zones, geofences). Unnecessary for points.

## 6. The dispatch funnel and its trade-off

Shortlist → price → assign keeps the expensive work (graph search) proportional to the shortlist size, not the fleet size.

**The trade-off is recall.** The shortlist uses straight-line distance, so a rider just outside the radius with a fast arterial route can be missed. The two knobs are `searchRadiusKm` and `maxCandidatesPerOrder`. A larger shortlist improves recall at linear pricing cost. Production systems tune these per zone and time of day.

## 7. Matching: greedy vs Hungarian

Greedy first-come-first-served is O(R·C) and simple, but an early order can take the only rider a later order could use. The Hungarian algorithm solves the assignment exactly in O(n²m).

Infeasible pairs (out of radius or unreachable) are modelled as a penalty P larger than any complete set of feasible pairs (P = n · max cost + 1). Using one fewer infeasible pair always lowers the total, so the optimum serves as many orders as possible and only then minimises ETA. Infeasible pairs are dropped from the result.

**Batching window trade-off:** collecting orders for a few seconds before matching improves the global result, but adds latency to every order. The benchmark's 30-order batches complete in about 4.5 ms end-to-end, so the window is limited by order arrival rate, not by compute.

## 8. Testing strategy

Every non-trivial algorithm is tested against a brute-force oracle on randomised inputs:

| Component | Oracle |
|---|---|
| QuadTree radius / k-nearest | Linear scan |
| A\*, weighted A\* | Dijkstra (exact match; within w× bound) |
| One-to-many (both directions) | Individual Dijkstra runs |
| Multi-source A\* | Minimum over individual Dijkstra runs |
| Hungarian | Exhaustive search over all partial matchings |
| NodeLocator | Linear scan over intersections |

Invariant tests cover the heuristic lower bound on every edge of a generated city, generator determinism, builder validation, and live traffic changing the chosen route. CI runs everything under AddressSanitizer and UBSan on GCC and Clang.

## 9. Toward production

| Concern | Approach |
|---|---|
| Throughput | One `ShortestPathEngine` per worker thread over a shared graph |
| Concurrent traffic updates | Double-buffered travel-time arrays swapped atomically per update batch |
| Faster queries at country scale | Contraction Hierarchies (static) or Customizable CH (traffic-friendly) |
| Tighter heuristic | ALT landmarks |
| Fleet size beyond one machine | Shard `FleetIndex` geographically and fan out shortlist queries to neighbouring shards |
| Real maps | Feed an OpenStreetMap parser into `RoadNetworkBuilder` via `LocalProjection` |
