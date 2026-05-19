#include "algorithms/assemble.hpp"
#include "algorithms/convex_hull.hpp"
#include "algorithms/line_segment_intersection.hpp"
#include "geometry/polygon.hpp"
#include <algorithm>
#include <gtest/gtest.h>
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

std::vector<Segment> rectangleSegments(const Point& lower_left, const Point& upper_right) {
    const Point lower_right(upper_right.x, lower_left.y);
    const Point upper_left(lower_left.x, upper_right.y);
    return {
        Segment(lower_left, lower_right),
        Segment(lower_right, upper_right),
        Segment(upper_right, upper_left),
        Segment(upper_left, lower_left),
    };
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

TEST(AssembleCyclesTest, BuildsOuterCycleFromUnorderedSegments) {
    const std::vector<Segment> segments = {
        Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(0, 1), Point(0, 0)),
        Segment(Point(1, 1), Point(0, 1)),
        Segment(Point(0, 0), Point(1, 0)),
    };

    const std::vector<Cycle> cycles = assembleCycles(segments);

    ASSERT_EQ(cycles.size(), 1);
    EXPECT_TRUE(cycles[0].isOuter());
    EXPECT_DOUBLE_EQ(cycles[0].area(), 1.0);
    EXPECT_EQ(pointSet(cycles[0].points),
              pointSet({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)}));
}

TEST(AssembleCyclesTest, BuildsMultipleDisjointOuterCycles) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(1, 0)), Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(1, 1), Point(0, 1)), Segment(Point(0, 1), Point(0, 0)),
        Segment(Point(3, 0), Point(5, 0)), Segment(Point(5, 0), Point(5, 2)),
        Segment(Point(5, 2), Point(3, 2)), Segment(Point(3, 2), Point(3, 0)),
    };

    const std::vector<Cycle> cycles = assembleCycles(segments);

    ASSERT_EQ(cycles.size(), 2);
    EXPECT_TRUE(cycles[0].isOuter());
    EXPECT_TRUE(cycles[1].isOuter());
    EXPECT_DOUBLE_EQ(cycles[0].area() + cycles[1].area(), 5.0);
}

TEST(AssembleCyclesTest, RejectsDuplicateReversedAndDegenerateSegments) {
    EXPECT_THROW(
        assembleCycles({Segment(Point(0, 0), Point(1, 0)), Segment(Point(1, 0), Point(0, 0))}),
        std::invalid_argument);

    EXPECT_THROW(assembleCycles({Segment(Point(0, 0), Point(0, 0))}), std::invalid_argument);
}

TEST(AssembleCyclesTest, RejectsOpenChains) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(1, 0)),
        Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(1, 1), Point(0, 1)),
    };

    EXPECT_THROW(assembleCycles(segments), std::invalid_argument);
}

TEST(AssemblePolygonsTest, BuildsDonutWithIsland) {
    std::vector<Segment> segments;
    const std::vector<Segment> outer_segments = rectangleSegments(Point(0, 0), Point(10, 10));
    const std::vector<Segment> hole_segments = rectangleSegments(Point(3, 3), Point(7, 7));
    const std::vector<Segment> island_segments = rectangleSegments(Point(4, 4), Point(6, 6));
    segments.insert(segments.end(), outer_segments.begin(), outer_segments.end());
    segments.insert(segments.end(), hole_segments.begin(), hole_segments.end());
    segments.insert(segments.end(), island_segments.begin(), island_segments.end());

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 2);

    const auto donut = std::find_if(polygons.begin(), polygons.end(), [](const Polygon& polygon) {
        return polygon.outer_cycle.area() == 100.0;
    });
    ASSERT_NE(donut, polygons.end());
    ASSERT_EQ(donut->inner_cycles.size(), 1);
    EXPECT_DOUBLE_EQ(donut->area(), 84.0);

    const auto island = std::find_if(polygons.begin(), polygons.end(), [](const Polygon& polygon) {
        return polygon.outer_cycle.area() == 4.0;
    });
    ASSERT_NE(island, polygons.end());
    EXPECT_TRUE(island->inner_cycles.empty());
    EXPECT_DOUBLE_EQ(island->area(), 4.0);
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
