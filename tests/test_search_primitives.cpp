#include <vector>

#include "test_framework.hpp"
#include "waypoint/routing/min_heap.hpp"
#include "waypoint/routing/search_labels.hpp"

using namespace waypoint;
using namespace waypoint::routing;

TEST_CASE("min heap pops in priority order and can be reused after clear") {
    MinHeap heap;
    heap.push(10, 5.0);
    heap.push(11, 1.0);
    heap.push(12, 3.0);

    std::vector<NodeId> order;
    while (!heap.empty()) order.push_back(heap.popMin());
    CHECK(order == (std::vector<NodeId>{11, 12, 10}));

    heap.push(1, 2.0);
    heap.clear();
    CHECK(heap.empty());
}

TEST_CASE("search labels keep only improvements") {
    SearchLabels labels(4);
    CHECK(labels.cost(2) == kUnreachable);
    CHECK(labels.improve(2, 10.0, 1, 0));
    CHECK(!labels.improve(2, 12.0, 3, 0));  // slower: ignored
    CHECK(labels.improve(2, 8.0, 3, 1));    // faster: replaces
    CHECK(labels.cost(2) == 8.0);
    CHECK(labels.parent(2) == 3);
    CHECK(labels.seed(2) == 1);
}

TEST_CASE("clearing search labels forgets costs and settled flags") {
    SearchLabels labels(4);
    REQUIRE(labels.improve(0, 5.0, kInvalidNode, 0));
    labels.markSettled(0);
    CHECK(labels.isSettled(0));

    labels.clear();
    CHECK(!labels.isSettled(0));
    CHECK(labels.cost(0) == kUnreachable);
    CHECK(labels.improve(0, 50.0, kInvalidNode, 0));  // any cost beats "unreached"
}
