#include "algorithms/convex_hull.hpp"
#include "algorithms/line_segment_intersection.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <ostream>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

std::unordered_set<Point> pointSet(const std::vector<Point>& points) {
    return std::unordered_set<Point>(points.begin(), points.end());
}

std::string pointSetString(const std::unordered_set<Point>& points) {
    std::vector<Point> sorted_points(points.begin(), points.end());
    std::sort(sorted_points.begin(), sorted_points.end());

    std::string ret = "{";
    for (std::size_t i = 0; i < sorted_points.size(); ++i) {
        if (i > 0) {
            ret += ", ";
        }
        ret += sorted_points[i].toString();
    }
    ret += "}";
    return ret;
}

void expectPointSetsEqual(const std::string& algorithm_name,
                          const std::unordered_set<Point>& actual,
                          const std::unordered_set<Point>& expected) {
    EXPECT_EQ(actual, expected) << algorithm_name << " intersections\n"
                                << "  actual:   " << pointSetString(actual) << "\n"
                                << "  expected: " << pointSetString(expected);
}

void expectIntersections(const std::vector<Segment>& segments,
                         const std::unordered_set<Point>& expected) {
    expectPointSetsEqual("brute force", bruteForceLineSegmentIntersection(segments), expected);
    expectPointSetsEqual("sweep line", lineSegmentIntersection(segments), expected);
}

} // namespace

TEST(ConvexHullTest, BuildsSquareAroundInteriorPoint) {
    const std::vector<Point> points = {
        Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1), Point(Rational(1, 2), Rational(1, 2)),
    };

    const Cycle hull = convexHull(points);

    EXPECT_EQ(hull.points,
              std::vector<Point>({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)}));
    EXPECT_TRUE(hull.isOuter());
}

TEST(ConvexHullTest, KeepsOnlyEndpointsForCollinearPoints) {
    const std::vector<Point> points = {Point(0, 0), Point(1, 0), Point(2, 0), Point(3, 0)};

    const Cycle hull = convexHull(points);

    EXPECT_EQ(hull.points, std::vector<Point>({Point(0, 0), Point(3, 0)}));
}

TEST(ConvexHullTest, SlowAndFastHullUseSameBoundaryPoints) {
    const std::vector<Point> points = {Point(0, 0), Point(2, 0), Point(2, 2),
                                       Point(0, 2), Point(1, 1), Point(1, 0)};

    const Cycle hull = convexHull(points);
    const std::vector<Segment> slow_hull = slowConvexHull(points);

    std::vector<Point> slow_points;
    for (const Segment& segment : slow_hull) {
        slow_points.push_back(segment.start);
    }

    EXPECT_EQ(pointSet(hull.points), pointSet(slow_points));
}

TEST(LineSegmentIntersectionTest, FindsSingleCrossingPoint) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 4)),
        Segment(Point(0, 4), Point(4, 0)),
        Segment(Point(5, 5), Point(6, 6)),
    };

    expectIntersections(segments, {Point(2, 2)});
}

TEST(LineSegmentIntersectionTest, FindsEndpointAndInteriorIntersections) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 0)),
        Segment(Point(2, -1), Point(2, 1)),
        Segment(Point(4, 0), Point(5, 1)),
    };

    expectIntersections(segments, {Point(2, 0), Point(4, 0)});
}

TEST(LineSegmentIntersectionTest, ReportsOverlapEndpoints) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 0)),
        Segment(Point(2, 0), Point(6, 0)),
    };

    expectIntersections(segments, {Point(2, 0), Point(4, 0)});
}
