#include <algorithm>
#include <random>
#include <vector>

#include "test_framework.hpp"
#include "waypoint/geo/point.hpp"
#include "waypoint/spatial/quad_tree.hpp"

using waypoint::geo::BoundingBox;
using waypoint::geo::Point;
using Tree = waypoint::spatial::QuadTree;
using Id = Tree::Id;

namespace {

const BoundingBox kArea{{0.0, 0.0}, {100.0, 100.0}};

std::vector<Point> randomPoints(std::size_t count, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> coordinate(0.0, 100.0);
    std::vector<Point> points(count);
    for (Point& p : points) p = {coordinate(rng), coordinate(rng)};
    return points;
}

std::vector<Id> sortedIds(const std::vector<Tree::Entry>& entries) {
    std::vector<Id> ids;
    for (const auto& e : entries) ids.push_back(e.id);
    std::ranges::sort(ids);
    return ids;
}

}  // namespace

TEST_CASE("quadtree rejects points outside its bounds") {
    Tree tree(kArea);
    CHECK(!tree.insert(1, {-0.1, 50.0}));
    CHECK(!tree.insert(2, {50.0, 100.1}));
    CHECK(tree.insert(3, {100.0, 100.0}));  // bounds are inclusive
    CHECK(tree.size() == 1);
}

TEST_CASE("quadtree radius query matches a brute-force scan") {
    const auto points = randomPoints(3000, 7);
    Tree tree(kArea, {.leafCapacity = 4});
    for (Id i = 0; i < points.size(); ++i) REQUIRE(tree.insert(i, points[i]));

    for (const Point center : randomPoints(200, 8)) {
        const double radius = 7.5;
        std::vector<Id> expected;
        for (Id i = 0; i < points.size(); ++i) {
            if (waypoint::geo::euclideanDistance(points[i], center) <= radius) expected.push_back(i);
        }
        CHECK(sortedIds(tree.withinRadius(center, radius)) == expected);
    }
}

TEST_CASE("quadtree k-nearest matches a brute-force scan") {
    const auto points = randomPoints(2000, 11);
    Tree tree(kArea);
    for (Id i = 0; i < points.size(); ++i) REQUIRE(tree.insert(i, points[i]));

    for (const Point center : randomPoints(200, 12)) {
        std::vector<double> expected;
        for (const Point p : points) expected.push_back(waypoint::geo::euclideanDistance(p, center));
        std::ranges::sort(expected);
        expected.resize(10);

        const auto hits = tree.nearest(center, 10);
        REQUIRE(hits.size() == 10);
        for (Id i = 0; i < hits.size(); ++i) {
            CHECK_APPROX(waypoint::geo::euclideanDistance(hits[i].position, center), expected[i]);
        }
    }
}

TEST_CASE("quadtree k-nearest honours the radius cap") {
    Tree tree(kArea);
    REQUIRE(tree.insert(1, {50.0, 50.0}));
    REQUIRE(tree.insert(2, {52.0, 50.0}));
    REQUIRE(tree.insert(3, {60.0, 50.0}));

    const auto hits = tree.nearest({50.0, 50.0}, 10, 5.0);
    CHECK(sortedIds(hits) == (std::vector<Id>{1, 2}));
    CHECK(tree.nearest({50.0, 50.0}, 0).empty());
}

TEST_CASE("quadtree removal and re-insertion track moving objects") {
    const auto start = randomPoints(500, 21);
    const auto finish = randomPoints(500, 22);
    Tree tree(kArea, {.leafCapacity = 4});
    for (Id i = 0; i < 500; ++i) REQUIRE(tree.insert(i, start[i]));

    for (Id i = 0; i < 500; ++i) {
        REQUIRE(tree.remove(i, start[i]));
        REQUIRE(tree.insert(i, finish[i]));
    }
    CHECK(tree.size() == 500);
    CHECK(!tree.remove(0, start[0]));  // no longer stored at its old position

    // Every object is found at its new position only.
    for (Id i = 0; i < 500; ++i) {
        const auto hits = tree.nearest(finish[i], 1);
        REQUIRE(!hits.empty());
        CHECK(hits.front().position == finish[i]);
    }

    for (Id i = 0; i < 500; ++i) REQUIRE(tree.remove(i, finish[i]));
    CHECK(tree.empty());
    CHECK(tree.withinRadius({50.0, 50.0}, 1000.0).empty());
}

TEST_CASE("quadtree survives many entries at one coordinate") {
    Tree tree(kArea, {.leafCapacity = 2, .maxDepth = 8});
    for (Id i = 0; i < 1000; ++i) REQUIRE(tree.insert(i, {33.3, 66.6}));
    CHECK(tree.withinRadius({33.3, 66.6}, 0.0).size() == 1000);
    for (Id i = 0; i < 1000; ++i) REQUIRE(tree.remove(i, {33.3, 66.6}));
    CHECK(tree.empty());
}
