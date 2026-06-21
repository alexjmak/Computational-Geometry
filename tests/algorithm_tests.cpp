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

std::vector<Segment> rectangleHoleSegments(const Point& lower_left, const Point& upper_right) {
    const Point lower_right(upper_right.x, lower_left.y);
    const Point upper_left(lower_left.x, upper_right.y);
    return {
        Segment(lower_left, upper_left),
        Segment(upper_left, upper_right),
        Segment(upper_right, lower_right),
        Segment(lower_right, lower_left),
    };
}

Segment segment(long long ax, long long ay, long long bx, long long by) {
    return Segment(Point(ax, ay), Point(bx, by));
}

struct OverlayBucketCounts {
    std::size_t left_only = 0;
    std::size_t right_only = 0;
    std::size_t both = 0;
    std::size_t neither = 0;
    double left_only_area = 0.0;
    double right_only_area = 0.0;
    double both_area = 0.0;
};

OverlayBucketCounts countOverlayBuckets(const OverlayResult& overlay) {
    OverlayBucketCounts counts;
    for (const OverlayFacePolygon& face : overlayFacePolygons(overlay)) {
        const bool in_left = face.label.left_face != DCEL::unbounded_face_index;
        const bool in_right = face.label.right_face != DCEL::unbounded_face_index;
        const double area = face.polygon.area();

        EXPECT_GT(area, 0.0);
        if (in_left && in_right) {
            ++counts.both;
            counts.both_area += area;
        } else if (in_left) {
            ++counts.left_only;
            counts.left_only_area += area;
        } else if (in_right) {
            ++counts.right_only;
            counts.right_only_area += area;
        } else {
            ++counts.neither;
        }
    }
    return counts;
}

std::vector<Segment> showcaseSolidLayer() {
    return {
        segment(20, 72, 82, 71), segment(82, 71, 92, 46),
        segment(92, 46, 74, 21), segment(74, 21, 26, 28),
        segment(26, 28, 13, 57), segment(13, 57, 20, 72),
        segment(20, 72, 35, 62), segment(35, 62, 60, 60),
        segment(60, 60, 82, 71), segment(35, 62, 40, 38),
        segment(40, 38, 64, 45), segment(60, 60, 64, 45),
        segment(64, 45, 82, 48), segment(74, 21, 82, 48),
        segment(26, 28, 40, 38),
    };
}

std::vector<Segment> showcaseDottedLayer() {
    return {
        segment(3, 70, 42, 86), segment(42, 86, 74, 58),
        segment(74, 58, 70, 43), segment(70, 43, 28, 9),
        segment(28, 9, 3, 35),   segment(3, 35, 3, 70),
        segment(9, 60, 38, 65),  segment(38, 65, 52, 53),
        segment(52, 53, 47, 44), segment(47, 44, 11, 44),
        segment(11, 44, 9, 60),
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
        Segment(Point(0, 0), Point(1, 0)), Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(1, 1), Point(0, 1)), Segment(Point(0, 1), Point(0, 0)),
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
        Segment(Point(0, 0), Point(1, 0)), Segment(Point(1, 0), Point(1, 1)),
        Segment(Point(1, 1), Point(0, 1)), Segment(Point(0, 1), Point(0, 0)),
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

TEST(SegmentOverlayTest, LabelsContainedPolygonShowcaseFaces) {
    const std::vector<Segment> layer1 = rectangleSegments(Point(3, 2), Point(7, 5));
    const std::vector<Segment> layer2 = rectangleSegments(Point(0, 0), Point(10, 8));

    const OverlayResult overlay = segmentOverlay(layer1, layer2);
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 0);
    EXPECT_EQ(counts.right_only, 1);
    EXPECT_EQ(counts.both, 1);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_DOUBLE_EQ(counts.right_only_area, 68.0);
    EXPECT_DOUBLE_EQ(counts.both_area, 12.0);
}

TEST(SegmentOverlayTest, LabelsOverlayShowcaseFaces) {
    const OverlayResult overlay = segmentOverlay(showcaseSolidLayer(), showcaseDottedLayer());
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 3);
    EXPECT_EQ(counts.right_only, 2);
    EXPECT_EQ(counts.both, 9);
    EXPECT_EQ(counts.neither, 0);
}

TEST(PolygonAndTest, BuildsIntersectionOfTwoDonuts) {
    std::vector<Segment> layer1 = rectangleSegments(Point(1, 1), Point(7, 6));
    const std::vector<Segment> layer1_hole =
        rectangleHoleSegments(Point(Rational(11, 2), Rational(9, 2)),
                              Point(Rational(13, 2), Rational(11, 2)));
    layer1.insert(layer1.end(), layer1_hole.begin(), layer1_hole.end());

    std::vector<Segment> layer2 = rectangleSegments(Point(4, 3), Point(10, 8));
    const std::vector<Segment> layer2_hole =
        rectangleHoleSegments(Point(5, 4), Point(6, 5));
    layer2.insert(layer2.end(), layer2_hole.begin(), layer2_hole.end());

    const std::vector<Segment> intersection_segments = polygon_and(layer1, layer2);
    const std::vector<Polygon> polygons = assemblePolygons(intersection_segments);

    EXPECT_EQ(intersection_segments.size(), 12);
    ASSERT_EQ(polygons.size(), 1);
    EXPECT_DOUBLE_EQ(polygons[0].area(), 7.25);
    EXPECT_DOUBLE_EQ(polygons[0].outer_ring.area(), 9.0);
    ASSERT_EQ(polygons[0].inner_rings.size(), 1);
    EXPECT_DOUBLE_EQ(polygons[0].inner_rings[0].area(), 1.75);
}
