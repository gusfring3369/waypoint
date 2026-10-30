# Waypoint

**A real-time spatial routing and fleet dispatch engine in modern C++20.**

Waypoint answers the two questions behind every food-delivery or ride-hailing request:

1. **Which rider should take this order?** It shortlists nearby riders with a QuadTree, prices the shortlist with exact road-network ETAs, and assigns riders to orders individually or as a globally optimal batch.
2. **What is the fastest route?** It runs A* search on a directed, traffic-aware road graph, using a heuristic that is provably admissible.

It has no dependencies beyond a C++20 compiler and CMake. Every algorithm is checked against a brute-force reference in the test suite, and every performance number below comes from the benchmark in this repository.

---

## Highlights

| | Result (from [`bench/`](bench), 40,000-intersection city) |
|---|---|
| **A\* vs Dijkstra** | **48.5% fewer nodes settled**, ~36% faster, identical route costs over 1,000 random queries |
| **QuadTree k-nearest** | **~100x faster** than a linear scan over 50,000 riders (~3.4 µs vs ~350 µs per query) |
| **Multi-source A\*** | Finds the fastest of 16 candidate riders while settling **96.4% fewer nodes** than one A\* per candidate |
| **Hungarian batch matching** | Serves **100% vs 98.8%** of orders and cuts total pickup time by **11.6%** compared with greedy dispatch |
| **End-to-end batch** | 30 orders shortlisted, priced and optimally matched in **~3.5 ms** |

---

## Architecture

The code is split into five modules. Each module only uses the modules below it:

```mermaid
flowchart TD
    dispatch["<b>dispatch</b><br/>Dispatcher · FleetIndex<br/>Matcher ◁ GreedyMatcher, HungarianMatcher"]
    routing["<b>routing</b><br/>ShortestPathEngine<br/>SearchLabels · MinHeap · heuristics"]
    spatial["<b>spatial</b><br/>QuadTree · NodeLocator"]
    graph["<b>graph</b><br/>RoadNetworkBuilder → RoadNetwork<br/>AdjacencyIndex (CSR)"]
    geo["<b>geo</b><br/>Point · BoundingBox · Haversine"]
    sim["<b>sim</b><br/>generateCity"]

    dispatch --> routing
    dispatch --> spatial
    routing --> graph
    spatial --> graph
    spatial --> geo
    graph --> geo
    sim --> graph
```

| Module | What it does | Main classes |
|---|---|---|
| `geo` | Points, rectangles and distances in kilometres; latitude/longitude conversion | `Point`, `BoundingBox`, `LocalProjection` |
| `graph` | The road map: validated on input, stored compactly, traffic can be updated live | `RoadNetworkBuilder`, `RoadNetwork`, `AdjacencyIndex` |
| `routing` | All shortest-path searches, sharing one search loop | `ShortestPathEngine`, `SearchLabels`, `MinHeap` |
| `spatial` | "What is near this point?" for riders and intersections | `QuadTree`, `NodeLocator` |
| `dispatch` | The business layer: riders, orders, assignment policies | `Dispatcher`, `FleetIndex`, `Matcher` |
| `sim` | Reproducible synthetic cities for tests, the demo and benchmarks | `generateCity` |

### Design patterns used

| Pattern | Where | Why |
|---|---|---|
| **Builder** | `RoadNetworkBuilder` → `RoadNetwork` | All validation happens in one place, so a `RoadNetwork` can never be invalid |
| **Strategy** | `Matcher` interface, implemented by `GreedyMatcher` and `HungarianMatcher` | The `Dispatcher` doesn't know which policy it runs, so new policies plug in without changing it |
| **Dependency injection** | `Dispatcher` receives the network, fleet and matcher | Each part can be tested on its own with small, hand-built inputs |
| **Template method / policy** | `ShortestPathEngine::search` takes a heuristic and a stop condition | Four query types share one loop, and the compiler inlines the hot path |
| **RAII / Rule of Zero** | `QuadTree` children are `unique_ptr`, containers own everything | No manual `new`/`delete` anywhere |

### The dispatch funnel

Each stage costs more per rider than the one before, so each stage only sees the riders that survived the previous one:

```
  Order at (x, y)
        │
        ▼
 ┌──────────────────────┐   QuadTree: nearest riders within radius R
 │ 1. SHORTLIST         │   cheap, straight-line distance
 └──────────┬───────────┘   50,000 riders  ->  ≤ 16 candidates
            ▼
 ┌──────────────────────┐   single order : one multi-source A* (fastest rider wins)
 │ 2. PRICE             │   batch        : one reverse search per order
 └──────────┬───────────┘   exact road ETAs: one-way streets, traffic, river crossings
            ▼
 ┌──────────────────────┐   single order : take the winner
 │ 3. ASSIGN            │   batch        : a Matcher (greedy or Hungarian)
 └──────────────────────┘
```

Why not just pick the closest rider in a straight line? Rivers, one-way streets and highways make straight-line distance a poor stand-in for travel time. The demo shows this directly:

```
=== 2. Single dispatch: straight-line nearest is not road nearest ===
  Rider #101: 0.16 km away in a straight line, ETA by road 4:33
  Rider #102: 0.45 km away in a straight line, ETA by road 1:17
  -> Dispatched rider #102 (pickup ETA 1:17)
```

---

## How the algorithms work

### Road graph: compact adjacency lists in both directions
`AdjacencyIndex` stores, for every intersection, the roads leaving it (or entering it), in **compressed sparse row (CSR)** form. All the lists sit back to back in one array, so a node's roads are next to each other in memory. `RoadNetwork` keeps one index for each direction. The reverse index lets one search answer "how long does each rider take to reach this pickup?".

Costs are **travel times in seconds** (`length / speed × traffic`), because a dispatcher cares about arrival time, not distance.

### A* with a heuristic that is guaranteed safe
The heuristic is the straight-line distance to the target divided by the network's top speed: no real route can be faster than that. `RoadNetworkBuilder` **enforces** the rules that make this true:

- a road is never shorter than the straight line between its ends,
- traffic can only slow a road down (`setTrafficMultiplier(0.5)` throws).

So A* returns exactly the same routes as Dijkstra (`docs/DESIGN.md` has the proof). Weighted A* (`aStar(s, t, 1.5)`) is also available. It settles another 45% fewer nodes, and in the benchmark its worst route was 2.2% slower than optimal.

### One search loop, four kinds of query
`ShortestPathEngine` runs Dijkstra, A\*, one-to-many and multi-source searches through one loop. They differ only in where they start, which heuristic they use and when they stop. Two helper classes keep the loop short:

- **`MinHeap`**: a priority queue that keeps its memory between queries. A node whose time improves is pushed again, and the outdated copy is skipped later. This replaces the "decrease-key" operation, which `std::priority_queue` doesn't have.
- **`SearchLabels`**: per-intersection bookkeeping (best time, previous node, settled flag). Each label is stamped with a query number, so starting a new query is O(1) instead of clearing 40,000 entries.

### QuadTree for moving riders
- A cell splits into four when it gets crowded, and **four quiet sibling cells merge back**, so the tree follows where riders are now.
- `maxDepth` stops endless splitting when many riders report the same spot.
- **Radius queries** skip cells entirely outside the circle and take cells entirely inside it without checking each point.
- **k-nearest** visits cells closest-first and stops once no unvisited cell can beat the k-th best rider found (`NearestCollector`).

### Optimal batch assignment
`GreedyMatcher` handles orders first-come-first-served, so an early order can take the only rider a later order could use. `HungarianMatcher` solves the whole batch at once with the **Hungarian algorithm** (O(n²m)). It first serves as many orders as possible, then minimises total pickup time. Tests check it against brute force on 300 random cases.

---

## Benchmarks

```bash
./build/waypoint_bench          # full run, about 10 s
./build/waypoint_bench --quick  # CI smoke run
```

Measured on one core of an Intel Xeon @ 2.10 GHz, GCC 13, `-O3` (CMake Release). Synthetic city: 200 × 200 jittered grid (40,000 intersections, 141,045 directed segments, 20 × 20 km), 20% one-way streets, arterials, closures, random congestion up to 2×, and a river with bridges. Seeds are fixed, so node counts and match quality are exact. Timings vary by about ±10% between runs and by machine.

**1. Point-to-point routing** (1,000 random reachable pairs)

| Algorithm | Avg nodes settled | Avg query time | Worst route vs optimal |
|---|---:|---:|---:|
| Dijkstra | 19,886 | 2,154 µs | 1.000× |
| A* | 10,234 | 1,364 µs | 1.000× |
| Weighted A* (w = 1.5) | 5,599 | 757 µs | 1.022× |

**2. Rider shortlist** (50,000 riders, 10,000 queries)

| Query | Linear scan | QuadTree | Speed-up |
|---|---:|---:|---:|
| Radius 2 km (≈1,445 hits) | 100.7 µs | 21.7 µs | ~4–5× |
| 16 nearest | 353.7 µs | 3.4 µs | ~100× |

The radius query gains less because a 2 km circle holds about 1,445 riders that must be copied either way. k-nearest, which the dispatcher actually uses, touches only a few cells.

**3. Pricing a 16-rider shortlist** (2,000 available riders, 1,000 orders)

| Strategy | Searches | Avg nodes settled | Avg time | Output |
|---|---:|---:|---:|---|
| A* per candidate | 16 | 1,192 | 121 µs | every ETA |
| Reverse one-to-many Dijkstra | 1 | 940 | 100 µs | every ETA |
| Multi-source A* | 1 | 43 | 5 µs | fastest rider only |

**4. Batch dispatch** (200 rounds of 30 orders vs 40 riders in a 3 × 3 km zone)

| Strategy | Orders served | Avg pickup ETA per order |
|---|---:|---:|
| Greedy (first-come-first-served) | 98.8% | 70 s |
| Optimal (Hungarian) | 100.0% | 63 s |

In rounds where both strategies served the same number of orders, optimal matching cut total pickup time by 11.6%.

---

## Complexity

V = intersections, E = road segments, N = indexed riders, k = results.

| Operation | Time | Notes |
|---|---|---|
| Dijkstra / A* | O((V + E) log V) worst case | A* explores a goal-directed subset in practice |
| One-to-many / multi-source | O((V + E) log V) worst case | one search instead of k |
| Traffic update | O(1) | |
| QuadTree insert / remove | O(depth) | depth ≤ `maxDepth`; ~log₄ N for spread-out data |
| QuadTree radius query | O(depth + cells visited + k) | worst case O(N) |
| QuadTree k-nearest | O(k log k + cells visited) | typically a handful of cells |
| Hungarian matching | O(n² m), n = min(rows, cols) | |
| Graph memory | O(V + E) | an adjacency matrix would need V² cells: 1.6 × 10⁹ here |

---

## Build and run

Requires CMake ≥ 3.20 and a C++20 compiler (tested with GCC 13 and Clang 18).

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/waypoint_demo
./build/waypoint_bench
```

| CMake option | Default | |
|---|---|---|
| `WAYPOINT_BUILD_TESTS` | `ON` | unit tests |
| `WAYPOINT_BUILD_APPS` | `ON` | demo and benchmark |
| `WAYPOINT_WARNINGS_AS_ERRORS` | `OFF` | `-Werror` / `/WX` |
| `WAYPOINT_SANITIZE` | `OFF` | AddressSanitizer + UndefinedBehaviorSanitizer |

The included CI workflow builds with GCC and Clang, in Debug with sanitizers and in Release, with warnings treated as errors. It also runs the tests, demo, quick benchmark and a `clang-format` check.

## Usage

```cpp
#include "waypoint/dispatch/dispatcher.hpp"
#include "waypoint/dispatch/hungarian_matcher.hpp"
#include "waypoint/graph/road_network_builder.hpp"
#include "waypoint/spatial/node_locator.hpp"

using namespace waypoint;

// 1. Build the road map (coordinates in km).
graph::RoadNetworkBuilder builder;
const NodeId a = builder.addIntersection({0.0, 0.0});
const NodeId b = builder.addIntersection({1.0, 0.0});
builder.addTwoWayRoad(a, b, {.speedKmh = 40.0});
const EdgeId oneWay = builder.addRoad(b, a, {.speedKmh = 60.0, .lengthKm = 1.3});
graph::RoadNetwork city = std::move(builder).build();

// 2. Route (one engine per thread).
routing::ShortestPathEngine engine(city);
routing::Route route = engine.aStar(a, b);                  // route.travelTime, route.path
city.setTrafficMultiplier(oneWay, 1.8);                     // live traffic update

// 3. Track riders and dispatch orders.
const spatial::NodeLocator locator(city);                   // GPS position -> nearest intersection
dispatch::FleetIndex fleet(geo::BoundingBox::around({0.5, 0.0}, 10.0, 10.0));  // service area
fleet.upsert({.id = 7, .position = {0.9, 0.1}, .node = *locator.nearestNode({0.9, 0.1})});

dispatch::Dispatcher dispatcher(city, fleet, {.searchRadiusKm = 3.0});
auto single = dispatcher.dispatch({.id = 1, .pickup = {0.0, 0.0}, .pickupNode = a});

const dispatch::Order batch[] = {{.id = 2, .pickup = {0.0, 0.0}, .pickupNode = a}};
auto plan = dispatcher.dispatchBatch(batch, dispatch::HungarianMatcher{});
```

For real map data, convert latitude/longitude to kilometres once with `geo::LocalProjection`, then build the network from the converted points. At city scale the conversion error is under 0.5% (tested).

## Code conventions

- **Namespaces.** Everything lives in `waypoint::<module>` (for example `waypoint::routing::ShortestPathEngine`), so names can't clash with other libraries. Headers never contain `using namespace`. Application and test `.cpp` files may write `using namespace waypoint;` for brevity. Standard library names are always written `std::...`, without `using namespace std;`, following the Google and LLVM style guides and the C++ Core Guidelines.
- **One class per file.** `include/waypoint/<module>/<name>.hpp` declares it, and `src/<module>/<name>.cpp` implements it. A header only contains code that must be there: small inline functions on the search hot path, or templates.
- **Naming.** `PascalCase` types, `camelCase` functions and variables, `trailingUnderscore_` for private members, `kConstant` for constants.
- **Errors.** Invalid input to public functions throws (`std::invalid_argument`, `std::out_of_range`). Internal assumptions are checked with `assert` in debug builds.
- **Formatting.** `clang-format` with the repository's `.clang-format`.

## Project layout

```
include/waypoint/
  core/      types.hpp                       ids, Seconds, constants
  geo/       point, bounding_box, geodesy    geometry and lat/lng helpers
  graph/     road, adjacency_index,          the road map
             road_network, road_network_builder
  routing/   route, heuristics, min_heap,    shortest paths
             search_labels, shortest_path_engine
  spatial/   quad_tree, node_locator         proximity search
  dispatch/  models, fleet_index, cost_matrix, matcher,
             greedy_matcher, hungarian_matcher, dispatcher
  sim/       city_generator                  synthetic cities
src/         one .cpp per class above (+ spatial/nearest_collector.hpp, internal)
tests/       one test file per class, plus a tiny test harness
apps/        demo.cpp: guided walkthrough
bench/       one file per benchmark section + main.cpp
docs/        DESIGN.md: design decisions and trade-offs
```

## Limitations and next steps

- **Single-threaded queries.** Engines are cheap to construct, so a production service would keep one per worker thread over a shared, read-mostly `RoadNetwork`. Concurrent traffic updates would then need double-buffered travel-time arrays.
- **Static topology.** Continental-scale graphs would call for preprocessing-based speed-ups such as Contraction Hierarchies or ALT landmarks. ALT would also give a much tighter heuristic than straight-line distance at top speed.
- **Synthetic cities.** Ingesting OpenStreetMap extracts would only need a parser that feeds `RoadNetworkBuilder`.

See [`docs/DESIGN.md`](docs/DESIGN.md) for the reasoning behind each design choice.
