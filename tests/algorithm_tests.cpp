#include "algorithms/assemble.hpp"
#include "algorithms/convex_hull.hpp"
#include "algorithms/line_segment_intersection.hpp"
#include "algorithms/overlay.hpp"
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

std::unordered_set<Segment> segmentSet(const std::vector<Segment>& segments) {
    return std::unordered_set<Segment>(segments.begin(), segments.end());
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

std::string segmentSetString(const std::unordered_set<Segment>& segments) {
    std::vector<Segment> sorted_segments(segments.begin(), segments.end());
    std::sort(sorted_segments.begin(), sorted_segments.end());

    std::string ret = "{";
    for (std::size_t i = 0; i < sorted_segments.size(); ++i) {
        if (i > 0) {
            ret += ", ";
        }
        ret += sorted_segments[i].toString();
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

void expectSegmentSetsEqual(const std::string& algorithm_name,
                            const std::unordered_set<Segment>& actual,
                            const std::unordered_set<Segment>& expected) {
    EXPECT_EQ(actual, expected) << algorithm_name << " segments\n"
                                << "  actual:   " << segmentSetString(actual) << "\n"
                                << "  expected: " << segmentSetString(expected);
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

    const LinearRing hull = convexHull(points);

    EXPECT_EQ(hull.points,
              std::vector<Point>({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)}));
    EXPECT_TRUE(hull.isOuter());
}

TEST(ConvexHullTest, KeepsOnlyEndpointsForCollinearPoints) {
    const std::vector<Point> points = {Point(0, 0), Point(1, 0), Point(2, 0), Point(3, 0)};

    const LinearRing hull = convexHull(points);

    EXPECT_EQ(hull.points, std::vector<Point>({Point(0, 0), Point(3, 0)}));
}

TEST(ConvexHullTest, SlowAndFastHullUseSameBoundaryPoints) {
    const std::vector<Point> points = {Point(0, 0), Point(2, 0), Point(2, 2),
                                       Point(0, 2), Point(1, 1), Point(1, 0)};

    const LinearRing hull = convexHull(points);
    const std::vector<Segment> slow_hull = slowConvexHull(points);

    std::vector<Point> slow_points;
    for (const Segment& segment : slow_hull) {
        slow_points.push_back(segment.start);
    }

    EXPECT_EQ(pointSet(hull.points), pointSet(slow_points));
}

TEST(AssembleRingsTest, BuildsOuterRingFromUnorderedSegments) {
    const std::vector<Segment> segments = {
        Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(0, 1), Point(0, 0)),
        Segment(Point(1, 1), Point(0, 1)),
        Segment(Point(0, 0), Point(1, 0)),
    };

    const std::vector<LinearRing> rings = assembleRings(segments);

    ASSERT_EQ(rings.size(), 1);
    EXPECT_TRUE(rings[0].isOuter());
    EXPECT_DOUBLE_EQ(rings[0].area(), 1.0);
    EXPECT_EQ(pointSet(rings[0].points),
              pointSet({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)}));
}

TEST(AssembleRingsTest, BuildsMultipleDisjointOuterRings) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(1, 0)), Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(1, 1), Point(0, 1)), Segment(Point(0, 1), Point(0, 0)),
        Segment(Point(3, 0), Point(5, 0)), Segment(Point(5, 0), Point(5, 2)),
        Segment(Point(5, 2), Point(3, 2)), Segment(Point(3, 2), Point(3, 0)),
    };

    const std::vector<LinearRing> rings = assembleRings(segments);

    ASSERT_EQ(rings.size(), 2);
    EXPECT_TRUE(rings[0].isOuter());
    EXPECT_TRUE(rings[1].isOuter());
    EXPECT_DOUBLE_EQ(rings[0].area() + rings[1].area(), 5.0);
}

TEST(AssembleRingsTest, AllowsReversedDuplicateSegments) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(1, 0)),
        Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(1, 1), Point(0, 1)),
        Segment(Point(0, 1), Point(0, 0)),
        Segment(Point(1, 0), Point(0, 0)),
    };

    const std::vector<LinearRing> rings = assembleRings(segments);

    ASSERT_EQ(rings.size(), 1);
    EXPECT_TRUE(rings[0].isOuter());
    EXPECT_DOUBLE_EQ(rings[0].area(), 1.0);
    EXPECT_EQ(pointSet(rings[0].points),
              pointSet({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)}));
}

TEST(AssembleRingsTest, IgnoresOpenChains) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(1, 0)),
        Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(1, 1), Point(0, 1)),
    };

    EXPECT_TRUE(assembleRings(segments).empty());
}

TEST(AssembleRingsTest, IgnoresDanglingChainAttachedToRing) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(1, 0)),
        Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(1, 1), Point(0, 1)),
        Segment(Point(0, 1), Point(0, 0)),
        Segment(Point(1, 0), Point(2, 0)),
    };

    const std::vector<LinearRing> rings = assembleRings(segments);

    ASSERT_EQ(rings.size(), 1);
    EXPECT_TRUE(rings[0].isOuter());
    EXPECT_DOUBLE_EQ(rings[0].area(), 1.0);
    EXPECT_EQ(pointSet(rings[0].points),
              pointSet({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)}));
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
        return polygon.outer_ring.area() == 100.0;
    });
    ASSERT_NE(donut, polygons.end());
    ASSERT_EQ(donut->inner_rings.size(), 1);
    EXPECT_DOUBLE_EQ(donut->area(), 84.0);

    const auto island = std::find_if(polygons.begin(), polygons.end(), [](const Polygon& polygon) {
        return polygon.outer_ring.area() == 4.0;
    });
    ASSERT_NE(island, polygons.end());
    EXPECT_TRUE(island->inner_rings.empty());
    EXPECT_DOUBLE_EQ(island->area(), 4.0);
}

TEST(AssemblePolygonsTest, BuildsPolygonWithTwoHoles) {
    std::vector<Segment> segments;
    const std::vector<Segment> outer_segments = rectangleSegments(Point(15, 0), Point(25, 10));
    const std::vector<Segment> left_hole_segments = rectangleSegments(Point(18, 3), Point(20, 8));
    const std::vector<Segment> right_hole_segments = rectangleSegments(Point(21, 3), Point(23, 8));
    segments.insert(segments.end(), outer_segments.begin(), outer_segments.end());
    segments.insert(segments.end(), left_hole_segments.begin(), left_hole_segments.end());
    segments.insert(segments.end(), right_hole_segments.begin(), right_hole_segments.end());

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    const Polygon& polygon = polygons[0];
    EXPECT_DOUBLE_EQ(polygon.outer_ring.area(), 100.0);
    ASSERT_EQ(polygon.inner_rings.size(), 2);
    EXPECT_DOUBLE_EQ(polygon.inner_rings[0].area() + polygon.inner_rings[1].area(), 20.0);
    EXPECT_DOUBLE_EQ(polygon.area(), 80.0);
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

TEST(LineSegmentIntersectionTest, GroupsSortedIntersectionPointsBySegment) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(6, 0)),
        Segment(Point(4, -1), Point(4, 1)),
        Segment(Point(2, -1), Point(2, 1)),
        Segment(Point(10, 0), Point(11, 0)),
    };

    const std::vector<std::vector<Point>> intersections =
        lineSegmentIntersectionBySegments(segments);

    ASSERT_EQ(intersections.size(), segments.size());
    EXPECT_EQ(intersections[0], std::vector<Point>({Point(2, 0), Point(4, 0)}));
    EXPECT_EQ(intersections[1], std::vector<Point>({Point(4, 0)}));
    EXPECT_EQ(intersections[2], std::vector<Point>({Point(2, 0)}));
    EXPECT_TRUE(intersections[3].empty());
}

TEST(LineSegmentIntersectionTest, GroupsIntersectionPointsInOriginalSegmentDirection) {
    const std::vector<Segment> segments = {
        Segment(Point(6, 0), Point(0, 0)),
        Segment(Point(4, -1), Point(4, 1)),
        Segment(Point(2, -1), Point(2, 1)),
    };

    const std::vector<std::vector<Point>> intersections =
        lineSegmentIntersectionBySegments(segments);

    ASSERT_EQ(intersections.size(), segments.size());
    EXPECT_EQ(intersections[0], std::vector<Point>({Point(4, 0), Point(2, 0)}));
    EXPECT_EQ(intersections[1], std::vector<Point>({Point(4, 0)}));
    EXPECT_EQ(intersections[2], std::vector<Point>({Point(2, 0)}));
}

TEST(LineSegmentIntersectionTest, KeepsDuplicateSegmentsDistinctInternally) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 0)),
        Segment(Point(0, 0), Point(4, 0)),
    };

    expectIntersections(segments, {Point(0, 0), Point(4, 0)});
}

TEST(LineSegmentIntersectionTest, KeepsReversedDuplicateSegmentsDistinctInternally) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 0)),
        Segment(Point(4, 0), Point(0, 0)),
    };

    expectIntersections(segments, {Point(0, 0), Point(4, 0)});
}

TEST(LineSegmentIntersectionTest, IgnoresDegenerateSegments) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(0, 0)),
        Segment(Point(0, 0), Point(4, 4)),
        Segment(Point(0, 4), Point(4, 0)),
    };

    expectIntersections(segments, {Point(2, 2)});
}

TEST(PlanarizeSegmentsTest, SplitsCrossingSegmentsAtIntersection) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 4)),
        Segment(Point(0, 4), Point(4, 0)),
    };

    const std::vector<Segment> planarized = planarizeSegments(segments);

    expectSegmentSetsEqual("planarizeSegments", segmentSet(planarized),
                           segmentSet({
                               Segment(Point(0, 0), Point(2, 2)),
                               Segment(Point(2, 2), Point(4, 4)),
                               Segment(Point(0, 4), Point(2, 2)),
                               Segment(Point(2, 2), Point(4, 0)),
                           }));
}
