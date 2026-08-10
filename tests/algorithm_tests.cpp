#include "algorithms/assemble.hpp"
#include "algorithms/convex_hull.hpp"
#include "algorithms/horizontal_ray_query.hpp"
#include "algorithms/line_segment_intersection.hpp"
#include "algorithms/monotone_partition.hpp"
#include "algorithms/overlay.hpp"
#include "algorithms/triangulation.hpp"
#include "geometry/polygon.hpp"
#include <algorithm>
#include <gtest/gtest.h>
#include <optional>
#include <stdexcept>
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

std::vector<Segment> ringSegments(const std::vector<Point>& points) {
    LinearRing ring(points);
    return ring.segments();
}

void appendSegments(std::vector<Segment>& segments, const std::vector<Segment>& extra_segments) {
    segments.insert(segments.end(), extra_segments.begin(), extra_segments.end());
}

Rational totalPolygonArea(const std::vector<Polygon>& polygons) {
    Rational area = 0;
    for (const Polygon& polygon : polygons) {
        area += polygon.area();
    }
    return area;
}

Segment segment(long long ax, long long ay, long long bx, long long by) {
    return Segment(Point(ax, ay), Point(bx, by));
}

struct OverlayBucketCounts {
    std::size_t left_only = 0;
    std::size_t right_only = 0;
    std::size_t both = 0;
    std::size_t neither = 0;
    Rational left_only_area = 0;
    Rational right_only_area = 0;
    Rational both_area = 0;
};

OverlayBucketCounts countOverlayBuckets(const OverlayResult& overlay) {
    OverlayBucketCounts counts;
    for (const OverlayFacePolygon& face : overlayFacePolygons(overlay)) {
        const bool in_left = face.label.left_face != DCEL::unbounded_face_index;
        const bool in_right = face.label.right_face != DCEL::unbounded_face_index;
        const Rational area = face.polygon.area();

        EXPECT_GT(area, 0);
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

bool isInteriorFace(const std::vector<DCEL::FaceParity>& face_parities, std::size_t face_index) {
    return face_index < face_parities.size() &&
           face_parities[face_index] == DCEL::FaceParity::Interior;
}

OverlayBucketCounts countFilledOverlayBuckets(const OverlayResult& overlay, const DCEL& left,
                                              const DCEL& right) {
    const std::vector<DCEL::FaceParity> left_face_parities = left.faceParities();
    const std::vector<DCEL::FaceParity> right_face_parities = right.faceParities();

    OverlayBucketCounts counts;
    for (const OverlayFacePolygon& face : overlayFacePolygons(overlay)) {
        const bool in_left = isInteriorFace(left_face_parities, face.label.left_face);
        const bool in_right = isInteriorFace(right_face_parities, face.label.right_face);
        const Rational area = face.polygon.area();

        EXPECT_GT(area, 0);
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
        segment(20, 72, 82, 71), segment(82, 71, 92, 46), segment(92, 46, 74, 21),
        segment(74, 21, 26, 28), segment(26, 28, 13, 57), segment(13, 57, 20, 72),
        segment(20, 72, 35, 62), segment(35, 62, 60, 60), segment(60, 60, 82, 71),
        segment(35, 62, 40, 38), segment(40, 38, 64, 45), segment(60, 60, 64, 45),
        segment(64, 45, 82, 48), segment(74, 21, 82, 48), segment(26, 28, 40, 38),
    };
}

std::vector<Segment> showcaseDottedLayer() {
    return {
        segment(3, 70, 42, 86),  segment(42, 86, 74, 58), segment(74, 58, 70, 43),
        segment(70, 43, 28, 9),  segment(28, 9, 3, 35),   segment(3, 35, 3, 70),
        segment(9, 60, 38, 65),  segment(38, 65, 52, 53), segment(52, 53, 47, 44),
        segment(47, 44, 11, 44), segment(11, 44, 9, 60),
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

TEST(ConvexHullTest, PreservesSingletonPoint) {
    const std::vector<Point> points = {Point(2, 3)};

    const LinearRing hull = convexHull(points);

    EXPECT_EQ(hull.points, std::vector<Point>({Point(2, 3)}));
}

TEST(ConvexHullTest, DeduplicatesRepeatedSingletonPoint) {
    const std::vector<Point> points = {Point(2, 3), Point(2, 3), Point(2, 3)};

    const LinearRing hull = convexHull(points);

    EXPECT_EQ(hull.points, std::vector<Point>({Point(2, 3)}));
}

TEST(ConvexHullTest, PreservesTwoUniquePoints) {
    const std::vector<Point> points = {Point(2, 3), Point(5, 7), Point(2, 3)};

    const LinearRing hull = convexHull(points);

    EXPECT_EQ(hull.points, std::vector<Point>({Point(2, 3), Point(5, 7)}));
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
    EXPECT_EQ(rings[0].area(), 1);
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
    EXPECT_EQ(rings[0].area() + rings[1].area(), 5);
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
    EXPECT_EQ(rings[0].area(), 1);
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
    EXPECT_EQ(rings[0].area(), 1);
    EXPECT_EQ(pointSet(rings[0].points),
              pointSet({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)}));
}

TEST(AssembleRingsTest, BuildsPointTouchingRingsSeparately) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(1, 1));
    appendSegments(segments, rectangleSegments(Point(1, 1), Point(2, 2)));

    const std::vector<LinearRing> rings = assembleRings(segments);

    ASSERT_EQ(rings.size(), 2);
    EXPECT_EQ(rings[0].area() + rings[1].area(), 2);
    EXPECT_EQ(rings[0].points.size(), 4);
    EXPECT_EQ(rings[1].points.size(), 4);
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
        return polygon.outer_ring.area() == 100;
    });
    ASSERT_NE(donut, polygons.end());
    ASSERT_EQ(donut->inner_rings.size(), 1);
    EXPECT_EQ(donut->area(), 84);

    const auto island = std::find_if(polygons.begin(), polygons.end(), [](const Polygon& polygon) {
        return polygon.outer_ring.area() == 4;
    });
    ASSERT_NE(island, polygons.end());
    EXPECT_TRUE(island->inner_rings.empty());
    EXPECT_EQ(island->area(), 4);
}

TEST(AssemblePolygonsTest, BuildsNestedAlternatingRingsWithIslandHole) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(10, 10));
    appendSegments(segments, rectangleSegments(Point(1, 1), Point(9, 9)));
    appendSegments(segments, rectangleSegments(Point(2, 2), Point(8, 8)));
    appendSegments(segments, rectangleSegments(Point(3, 3), Point(7, 7)));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 2);
    EXPECT_EQ(totalPolygonArea(polygons), 56);

    const auto outer_donut = std::find_if(polygons.begin(), polygons.end(), [](const Polygon& p) {
        return p.outer_ring.area() == 100;
    });
    ASSERT_NE(outer_donut, polygons.end());
    ASSERT_EQ(outer_donut->inner_rings.size(), 1);
    EXPECT_EQ(outer_donut->area(), 36);

    const auto island_with_lake =
        std::find_if(polygons.begin(), polygons.end(),
                     [](const Polygon& p) { return p.outer_ring.area() == 36; });
    ASSERT_NE(island_with_lake, polygons.end());
    ASSERT_EQ(island_with_lake->inner_rings.size(), 1);
    EXPECT_EQ(island_with_lake->area(), 20);
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
    EXPECT_EQ(polygon.outer_ring.area(), 100);
    ASSERT_EQ(polygon.inner_rings.size(), 2);
    EXPECT_EQ(polygon.inner_rings[0].area() + polygon.inner_rings[1].area(), 20);
    EXPECT_EQ(polygon.area(), 80);
}

TEST(AssemblePolygonsTest, ReturnsNoPolygonsForEmptySegmentSet) {
    EXPECT_TRUE(assemblePolygons({}).empty());
}

TEST(AssemblePolygonsTest, PreservesDuplicateBoundaryAsSinglePolygon) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> duplicate_segments = rectangleSegments(Point(0, 0), Point(4, 4));
    appendSegments(segments, duplicate_segments);

    const std::vector<Polygon> polygons = assemblePolygons(segments);
    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(polygons), 16);
}

TEST(AssemblePolygonsTest, PreservesReversedDuplicateBoundaryAsSinglePolygon) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    appendSegments(segments, rectangleHoleSegments(Point(0, 0), Point(4, 4)));

    const std::vector<Polygon> polygons = assemblePolygons(segments);
    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(polygons), 16);
}

TEST(AssemblePolygonsTest, BuildsTwoRegionsFromSelfCrossingBowtieBoundary) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(2, 2)),
        Segment(Point(2, 2), Point(0, 2)),
        Segment(Point(0, 2), Point(2, 0)),
        Segment(Point(2, 0), Point(0, 0)),
    };

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 2);
    EXPECT_EQ(totalPolygonArea(polygons), 2);
}

TEST(AssemblePolygonsTest, KeepsPointTouchingRectanglesAsSeparatePolygons) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(1, 1));
    appendSegments(segments, rectangleSegments(Point(1, 1), Point(2, 2)));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 2);
    EXPECT_EQ(totalPolygonArea(polygons), 2);
}

TEST(AssemblePolygonsTest, MergesSharedEdgeRectanglesIntoOnePolygon) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(1, 1));
    appendSegments(segments, rectangleSegments(Point(1, 0), Point(2, 1)));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(polygons), 2);
    EXPECT_TRUE(polygons[0].inner_rings.empty());
}

TEST(AssemblePolygonsTest, MergesPartiallySharedEdgeRectanglesIntoOnePolygon) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    appendSegments(segments, rectangleSegments(Point(2, 4), Point(6, 6)));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(polygons), 24);
    EXPECT_TRUE(polygons[0].inner_rings.empty());
}

TEST(AssemblePolygonsTest, IgnoresInteriorChordAcrossSquare) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    segments.emplace_back(Point(0, 0), Point(4, 4));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(polygons[0].outer_ring.points.size(), 4);
    EXPECT_EQ(totalPolygonArea(polygons), 16);
    EXPECT_TRUE(polygons[0].inner_rings.empty());
}

TEST(AssemblePolygonsTest, IgnoresFreeSegmentOutsideSquare) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    segments.emplace_back(Point(6, 1), Point(8, 3));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(polygons[0].outer_ring.points.size(), 4);
    EXPECT_EQ(totalPolygonArea(polygons), 16);
    EXPECT_TRUE(polygons[0].inner_rings.empty());
}

TEST(AssemblePolygonsTest, IgnoresDanglingSegmentAttachedToSquare) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    segments.emplace_back(Point(4, 2), Point(6, 2));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(polygons), 16);
    EXPECT_TRUE(polygons[0].inner_rings.empty());
}

TEST(AssemblePolygonsTest, RemovesDanglingAttachmentVertexFromSquareBoundary) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    segments.emplace_back(Point(4, 2), Point(6, 2));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(polygons[0].outer_ring.points.size(), 4);
    EXPECT_EQ(totalPolygonArea(polygons), 16);
    EXPECT_TRUE(polygons[0].inner_rings.empty());
}

TEST(AssemblePolygonsTest, IgnoresFreeSegmentInsideSquare) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    segments.emplace_back(Point(1, 1), Point(3, 3));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(polygons[0].outer_ring.points.size(), 4);
    EXPECT_EQ(totalPolygonArea(polygons), 16);
    EXPECT_TRUE(polygons[0].inner_rings.empty());
}

TEST(AssemblePolygonsTest, IgnoresDanglingSegmentInsideSquare) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    segments.emplace_back(Point(1, 1), Point(3, 1));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(polygons[0].outer_ring.points.size(), 4);
    EXPECT_EQ(totalPolygonArea(polygons), 16);
    EXPECT_TRUE(polygons[0].inner_rings.empty());
}

TEST(AssemblePolygonsTest, IgnoresMultipleInteriorChordsAcrossSquare) {
    std::vector<Segment> segments = rectangleSegments(Point(0, 0), Point(4, 4));
    segments.emplace_back(Point(0, 0), Point(4, 4));
    segments.emplace_back(Point(0, 4), Point(4, 0));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(polygons[0].outer_ring.points.size(), 4);
    EXPECT_EQ(totalPolygonArea(polygons), 16);
    EXPECT_TRUE(polygons[0].inner_rings.empty());
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

TEST(LineSegmentIntersectionTest, ReportsEndpointsForVerticalCollinearOverlap) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(0, 6)),
        Segment(Point(0, 2), Point(0, 4)),
    };

    expectIntersections(segments, {Point(0, 2), Point(0, 4)});
}

TEST(LineSegmentIntersectionTest, ReportsEndpointsForDiagonalCollinearOverlap) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(6, 6)),
        Segment(Point(2, 2), Point(4, 4)),
    };

    expectIntersections(segments, {Point(2, 2), Point(4, 4)});
}

TEST(LineSegmentIntersectionTest, DeduplicatesOverlappingChainEndpoints) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 0)),
        Segment(Point(2, 0), Point(6, 0)),
        Segment(Point(4, 0), Point(8, 0)),
    };

    expectIntersections(segments, {Point(2, 0), Point(4, 0), Point(6, 0)});
}

TEST(LineSegmentIntersectionTest, ReportsRationalCrossingPoint) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(3, 2)),
        Segment(Point(0, 2), Point(3, 0)),
    };

    expectIntersections(segments, {Point(Rational(3, 2), 1)});
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

TEST(LineSegmentIntersectionTest, GroupsDiagonalIntersectionsInOriginalSegmentDirection) {
    const std::vector<Segment> segments = {
        Segment(Point(6, 6), Point(0, 0)),
        Segment(Point(4, 0), Point(4, 6)),
        Segment(Point(2, 0), Point(2, 6)),
    };

    const std::vector<std::vector<Point>> intersections =
        lineSegmentIntersectionBySegments(segments);

    ASSERT_EQ(intersections.size(), segments.size());
    EXPECT_EQ(intersections[0], std::vector<Point>({Point(4, 4), Point(2, 2)}));
    EXPECT_EQ(intersections[1], std::vector<Point>({Point(4, 4)}));
    EXPECT_EQ(intersections[2], std::vector<Point>({Point(2, 2)}));
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

TEST(LineSegmentIntersectionTest, HandlesManySegmentsSharingOneEndpoint) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 0)),
        Segment(Point(0, 0), Point(0, 4)),
        Segment(Point(0, 0), Point(4, 4)),
        Segment(Point(0, 0), Point(4, 2)),
    };

    expectIntersections(segments, {Point(0, 0)});
}

TEST(LineSegmentIntersectionTest, DeduplicatesSeveralIntersectionsAtSamePoint) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 4)),
        Segment(Point(0, 4), Point(4, 0)),
        Segment(Point(2, -1), Point(2, 5)),
        Segment(Point(-1, 2), Point(5, 2)),
    };

    expectIntersections(segments, {Point(2, 2)});
}

TEST(LineSegmentIntersectionTest, HandlesDuplicateReversedSegmentAndCrossingSegment) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(4, 0)),
        Segment(Point(4, 0), Point(0, 0)),
        Segment(Point(2, -1), Point(2, 1)),
    };

    expectIntersections(segments, {Point(0, 0), Point(2, 0), Point(4, 0)});
}

TEST(LineSegmentIntersectionTest, ReportsEndpointsForContainedCollinearOverlap) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(6, 0)),
        Segment(Point(2, 0), Point(4, 0)),
        Segment(Point(3, -1), Point(3, 1)),
    };

    expectIntersections(segments, {Point(2, 0), Point(3, 0), Point(4, 0)});
}

TEST(LeftRayQueryTest, FindsNearestSegmentStrictlyLeftOfQuery) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(0, 5)),
        Segment(Point(3, 0), Point(3, 5)),
        Segment(Point(6, 0), Point(6, 5)),
    };
    const std::vector<Point> queries = {Point(4, 2)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 1);
    ASSERT_TRUE(hits[0].has_value());
    EXPECT_EQ(hits[0].value(), 1);
}

TEST(LeftRayQueryTest, ReportsNoHitWhenAllSegmentsAreRightOfQuery) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(0, 5)),
        Segment(Point(3, 0), Point(3, 5)),
    };
    const std::vector<Point> queries = {Point(-1, 2)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 1);
    EXPECT_FALSE(hits[0].has_value());
}

TEST(LeftRayQueryTest, KeepsQueryResultsIndexedWhenQueriesSharePoint) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(0, 5)),
        Segment(Point(3, 0), Point(3, 5)),
    };
    const std::vector<Point> queries = {Point(4, 2), Point(4, 2)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 2);
    ASSERT_TRUE(hits[0].has_value());
    ASSERT_TRUE(hits[1].has_value());
    EXPECT_EQ(hits[0].value(), 1);
    EXPECT_EQ(hits[1].value(), 1);
}

TEST(LeftRayQueryTest, HandlesReversedInputSegments) {
    const std::vector<Segment> segments = {
        Segment(Point(3, 5), Point(3, 0)),
    };
    const std::vector<Point> queries = {Point(4, 2)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 1);
    ASSERT_TRUE(hits[0].has_value());
    EXPECT_EQ(hits[0].value(), 0);
}

TEST(LeftRayQueryTest, CountsLowerEndpointStrictlyLeftOfQuery) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(5, 5)),
    };
    const std::vector<Point> queries = {Point(10, 0)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 1);
    ASSERT_TRUE(hits[0].has_value());
    EXPECT_EQ(hits[0].value(), 0);
}

TEST(LeftRayQueryTest, PicksCloserEdgeAtLowerEndpointTie) {
    const std::vector<Segment> segments = {
        Segment(Point(-5, 5), Point(0, 0)),
        Segment(Point(0, 0), Point(5, 5)),
    };
    const std::vector<Point> queries = {Point(10, 0)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 1);
    ASSERT_TRUE(hits[0].has_value());
    EXPECT_EQ(hits[0].value(), 1);
}

TEST(LeftRayQueryTest, CountsUpperEndpointStrictlyLeftOfQuery) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(5, 5)),
    };
    const std::vector<Point> queries = {Point(10, 5)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 1);
    ASSERT_TRUE(hits[0].has_value());
    EXPECT_EQ(hits[0].value(), 0);
}

TEST(LeftRayQueryTest, PicksCloserEdgeAtUpperEndpointTie) {
    const std::vector<Segment> segments = {
        Segment(Point(-5, 0), Point(0, 5)),
        Segment(Point(5, 0), Point(0, 5)),
    };
    const std::vector<Point> queries = {Point(10, 5)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 1);
    ASSERT_TRUE(hits[0].has_value());
    EXPECT_EQ(hits[0].value(), 1);
}

TEST(LeftRayQueryTest, AllowsEitherDuplicateSegmentIdForDuplicateGeometry) {
    const std::vector<Segment> segments = {
        Segment(Point(1, 0), Point(1, 5)),
        Segment(Point(1, 0), Point(1, 5)),
    };
    const std::vector<Point> queries = {Point(2, 2)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 1);
    ASSERT_TRUE(hits[0].has_value());
    EXPECT_TRUE(hits[0].value() == 0 || hits[0].value() == 1);
}

TEST(LeftRayQueryTest, SkipsHorizontalSegmentIncidentToQueryPoint) {
    const std::vector<Segment> segments = {
        Segment(Point(-2, 0), Point(0, 0)),
        Segment(Point(-3, -1), Point(-3, 1)),
    };
    const std::vector<Point> queries = {Point(0, 0)};

    const std::vector<std::optional<sweep::SegmentId>> hits = leftRayQuery(segments, queries);

    ASSERT_EQ(hits.size(), 1);
    ASSERT_TRUE(hits[0].has_value());
    EXPECT_EQ(hits[0].value(), 1);
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

TEST(PlanarizeSegmentsTest, SplitsContainedCollinearOverlapIntoAtomicSegments) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(6, 0)),
        Segment(Point(2, 0), Point(4, 0)),
    };

    const std::vector<Segment> planarized = planarizeSegments(segments);

    expectSegmentSetsEqual("planarizeSegments", segmentSet(planarized),
                           segmentSet({
                               Segment(Point(0, 0), Point(2, 0)),
                               Segment(Point(2, 0), Point(4, 0)),
                               Segment(Point(4, 0), Point(6, 0)),
                           }));
}

TEST(PlanarizeSegmentsTest, PreservesOverlappingCoverageMultiplicity) {
    const std::vector<Segment> segments = {
        Segment(Point(0, 0), Point(6, 0)),
        Segment(Point(2, 0), Point(4, 0)),
    };

    const std::vector<Segment> planarized = planarizeSegments(segments);

    EXPECT_EQ(std::count(planarized.begin(), planarized.end(), Segment(Point(0, 0), Point(2, 0))),
              1);
    EXPECT_EQ(std::count(planarized.begin(), planarized.end(), Segment(Point(2, 0), Point(4, 0))),
              2);
    EXPECT_EQ(std::count(planarized.begin(), planarized.end(), Segment(Point(4, 0), Point(6, 0))),
              1);
    EXPECT_EQ(planarized.size(), 4);
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
    EXPECT_EQ(counts.right_only_area, 68);
    EXPECT_EQ(counts.both_area, 12);
}

TEST(SegmentOverlayTest, LabelsDisjointRectanglesSideBySide) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = rectangleSegments(Point(6, 0), Point(10, 4));

    const OverlayResult overlay = segmentOverlay(left, right);
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 1);
    EXPECT_EQ(counts.right_only, 1);
    EXPECT_EQ(counts.both, 0);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_EQ(counts.left_only_area, 16);
    EXPECT_EQ(counts.right_only_area, 16);
}

TEST(SegmentOverlayTest, LabelsSingleSidedOverlayAgainstEmptyArrangement) {
    const std::vector<Segment> left;
    const std::vector<Segment> right = rectangleSegments(Point(0, 0), Point(4, 4));

    const OverlayResult overlay = segmentOverlay(left, right);
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 0);
    EXPECT_EQ(counts.right_only, 1);
    EXPECT_EQ(counts.both, 0);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_EQ(counts.right_only_area, 16);
}

TEST(SegmentOverlayTest, ProducesNoFiniteFacesForTwoEmptyArrangements) {
    const std::vector<Segment> left;
    const std::vector<Segment> right;

    const OverlayResult overlay = segmentOverlay(left, right);

    EXPECT_EQ(overlay.dcel.pointCount(), 0);
    EXPECT_EQ(overlay.dcel.halfEdgeCount(), 0);
    ASSERT_EQ(overlay.dcel.faceCount(), 1);
    ASSERT_EQ(overlay.face_labels.size(), 1);
    EXPECT_EQ(overlay.face_labels[DCEL::unbounded_face_index].left_face,
              DCEL::unbounded_face_index);
    EXPECT_EQ(overlay.face_labels[DCEL::unbounded_face_index].right_face,
              DCEL::unbounded_face_index);
    EXPECT_TRUE(overlayFacePolygons(overlay).empty());
}

TEST(SegmentOverlayTest, TreatsDuplicateLeftBoundaryAsFilledAgainstRightPolygon) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    appendSegments(left, rectangleSegments(Point(0, 0), Point(4, 4)));
    const std::vector<Segment> right = rectangleSegments(Point(0, 0), Point(4, 4));

    const DCEL left_dcel = DCEL::fromSegments(left);
    const DCEL right_dcel = DCEL::fromSegments(right);
    const OverlayResult overlay = segmentOverlay(left_dcel, right_dcel);
    const OverlayBucketCounts counts = countFilledOverlayBuckets(overlay, left_dcel, right_dcel);

    EXPECT_EQ(counts.left_only, 0);
    EXPECT_EQ(counts.right_only, 0);
    EXPECT_EQ(counts.both, 1);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_EQ(counts.both_area, 16);
}

TEST(SegmentOverlayTest, LabelsOpenRightChordAsNonFilledOverlaySplitter) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = {
        Segment(Point(0, 0), Point(4, 4)),
    };

    const OverlayResult overlay = segmentOverlay(left, right);
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 2);
    EXPECT_EQ(counts.right_only, 0);
    EXPECT_EQ(counts.both, 0);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_EQ(counts.left_only_area, 16);
}

TEST(SegmentOverlayTest, LabelsPointTouchingRectanglesAsSeparateInteriors) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(1, 1));
    const std::vector<Segment> right = rectangleSegments(Point(1, 1), Point(2, 2));

    const OverlayResult overlay = segmentOverlay(left, right);
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 1);
    EXPECT_EQ(counts.right_only, 1);
    EXPECT_EQ(counts.both, 0);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_EQ(counts.left_only_area, 1);
    EXPECT_EQ(counts.right_only_area, 1);
}

TEST(SegmentOverlayTest, LabelsSharedEdgeRectanglesAsSeparateInteriors) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(1, 1));
    const std::vector<Segment> right = rectangleSegments(Point(1, 0), Point(2, 1));

    const OverlayResult overlay = segmentOverlay(left, right);
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 1);
    EXPECT_EQ(counts.right_only, 1);
    EXPECT_EQ(counts.both, 0);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_EQ(counts.left_only_area, 1);
    EXPECT_EQ(counts.right_only_area, 1);
}

TEST(SegmentOverlayTest, LabelsPartialSharedEdgeRectanglesAsSeparateInteriors) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = rectangleSegments(Point(2, 4), Point(6, 6));

    const OverlayResult overlay = segmentOverlay(left, right);
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 1);
    EXPECT_EQ(counts.right_only, 1);
    EXPECT_EQ(counts.both, 0);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_EQ(counts.left_only_area, 16);
    EXPECT_EQ(counts.right_only_area, 8);
}

TEST(SegmentOverlayTest, LabelsContainedStripSplittingRectangle) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(6, 4));
    const std::vector<Segment> right = rectangleSegments(Point(2, 0), Point(4, 4));

    const OverlayResult overlay = segmentOverlay(left, right);
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 2);
    EXPECT_EQ(counts.right_only, 0);
    EXPECT_EQ(counts.both, 1);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_EQ(counts.left_only_area, 16);
    EXPECT_EQ(counts.both_area, 8);
}

TEST(SegmentOverlayTest, LabelsContainedRectangleTouchingShellBoundary) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(6, 4));
    const std::vector<Segment> right = rectangleSegments(Point(2, 0), Point(4, 2));

    const OverlayResult overlay = segmentOverlay(left, right);
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 1);
    EXPECT_EQ(counts.right_only, 0);
    EXPECT_EQ(counts.both, 1);
    EXPECT_EQ(counts.neither, 0);
    EXPECT_EQ(counts.left_only_area, 20);
    EXPECT_EQ(counts.both_area, 4);
}

TEST(SegmentOverlayTest, LabelsPolygonInsideHoleAsRightOnly) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(10, 10));
    appendSegments(left, rectangleHoleSegments(Point(3, 3), Point(7, 7)));
    const std::vector<Segment> right = rectangleSegments(Point(4, 4), Point(6, 6));

    const DCEL left_dcel = DCEL::fromSegments(left);
    const DCEL right_dcel = DCEL::fromSegments(right);
    const OverlayResult overlay = segmentOverlay(left_dcel, right_dcel);
    const OverlayBucketCounts counts = countFilledOverlayBuckets(overlay, left_dcel, right_dcel);

    EXPECT_EQ(counts.left_only, 1);
    EXPECT_EQ(counts.right_only, 1);
    EXPECT_EQ(counts.both, 0);
    EXPECT_EQ(counts.neither, 1);
    EXPECT_EQ(counts.left_only_area, 84);
    EXPECT_EQ(counts.right_only_area, 4);
}

TEST(SegmentOverlayTest, LabelsRectangleCrossingHoleBoundaryByFilledParity) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(10, 10));
    appendSegments(left, rectangleHoleSegments(Point(3, 3), Point(7, 7)));
    const std::vector<Segment> right = rectangleSegments(Point(5, 5), Point(9, 9));

    const DCEL left_dcel = DCEL::fromSegments(left);
    const DCEL right_dcel = DCEL::fromSegments(right);
    const OverlayResult overlay = segmentOverlay(left_dcel, right_dcel);
    const OverlayBucketCounts counts = countFilledOverlayBuckets(overlay, left_dcel, right_dcel);

    EXPECT_EQ(counts.left_only, 1);
    EXPECT_EQ(counts.right_only, 1);
    EXPECT_EQ(counts.both, 1);
    EXPECT_EQ(counts.neither, 1);
    EXPECT_EQ(counts.left_only_area, 72);
    EXPECT_EQ(counts.right_only_area, 4);
    EXPECT_EQ(counts.both_area, 12);
}

TEST(SegmentOverlayTest, LabelsOverlayShowcaseFaces) {
    const OverlayResult overlay = segmentOverlay(showcaseSolidLayer(), showcaseDottedLayer());
    const OverlayBucketCounts counts = countOverlayBuckets(overlay);

    EXPECT_EQ(counts.left_only, 3);
    EXPECT_EQ(counts.right_only, 2);
    EXPECT_EQ(counts.both, 9);
    EXPECT_EQ(counts.neither, 0);
}

TEST(PolygonBooleanTest, AppliesTruthTableToOverlappingRectangles) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = rectangleSegments(Point(2, 0), Point(6, 4));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    ASSERT_EQ(intersection.size(), 1);
    EXPECT_EQ(totalPolygonArea(intersection), 8);

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 24);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 8);

    ASSERT_EQ(symmetric_difference.size(), 2);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 16);
}

TEST(PolygonBooleanTest, AppliesTruthTableWithEmptyLeftInput) {
    const std::vector<Segment> left;
    const std::vector<Segment> right = rectangleSegments(Point(0, 0), Point(4, 4));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    EXPECT_TRUE(difference.empty());

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 16);
}

TEST(PolygonBooleanTest, AppliesTruthTableWithEmptyRightInput) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right;

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 16);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 16);
}

TEST(PolygonBooleanTest, AppliesTruthTableWithBothInputsEmpty) {
    const std::vector<Segment> left;
    const std::vector<Segment> right;

    EXPECT_TRUE(assemblePolygons(polygonAnd(left, right)).empty());
    EXPECT_TRUE(assemblePolygons(polygonOr(left, right)).empty());
    EXPECT_TRUE(assemblePolygons(polygonDifference(left, right)).empty());
    EXPECT_TRUE(assemblePolygons(polygonXor(left, right)).empty());
}

TEST(PolygonBooleanTest, AppliesTruthTableWithDuplicateLeftBoundary) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    appendSegments(left, rectangleSegments(Point(0, 0), Point(4, 4)));
    const std::vector<Segment> right = rectangleSegments(Point(0, 0), Point(4, 4));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    ASSERT_EQ(intersection.size(), 1);
    EXPECT_EQ(totalPolygonArea(intersection), 16);

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    EXPECT_TRUE(difference.empty());

    EXPECT_TRUE(symmetric_difference.empty());
}

TEST(PolygonBooleanTest, AppliesTruthTableWithDuplicateBoundaryOnBothSides) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    appendSegments(left, rectangleSegments(Point(0, 0), Point(4, 4)));
    std::vector<Segment> right = rectangleSegments(Point(0, 0), Point(4, 4));
    appendSegments(right, rectangleSegments(Point(0, 0), Point(4, 4)));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    ASSERT_EQ(intersection.size(), 1);
    EXPECT_EQ(totalPolygonArea(intersection), 16);

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    EXPECT_TRUE(difference.empty());
    EXPECT_TRUE(symmetric_difference.empty());
}

TEST(PolygonBooleanTest, IgnoresOpenRightChordAsZeroAreaInput) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = {
        Segment(Point(0, 0), Point(4, 4)),
    };

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 16);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 16);
}

TEST(PolygonBooleanTest, IgnoresOpenRightVerticalChordAsZeroAreaInput) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = {
        Segment(Point(2, 0), Point(2, 4)),
    };

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 16);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 16);
}

TEST(PolygonBooleanTest, IgnoresOpenRightHorizontalChordAsZeroAreaInput) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = {
        Segment(Point(0, 2), Point(4, 2)),
    };

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 16);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 16);
}

TEST(PolygonBooleanTest, IgnoresOpenRightPolylineAsZeroAreaInput) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = {
        Segment(Point(1, 1), Point(3, 1)),
        Segment(Point(3, 1), Point(3, 3)),
    };

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 16);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 16);
}

TEST(PolygonBooleanTest, AppliesTruthTableToDisjointRectanglesSideBySide) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = rectangleSegments(Point(6, 0), Point(10, 4));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 2);
    EXPECT_EQ(totalPolygonArea(union_polygons), 32);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 16);

    ASSERT_EQ(symmetric_difference.size(), 2);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 32);
}

TEST(PolygonBooleanTest, AppliesTruthTableToContainedRectangles) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(10, 10));
    const std::vector<Segment> right = rectangleSegments(Point(3, 3), Point(7, 7));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    ASSERT_EQ(intersection.size(), 1);
    EXPECT_EQ(totalPolygonArea(intersection), 16);

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 100);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 84);
    ASSERT_EQ(difference[0].inner_rings.size(), 1);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 84);
    ASSERT_EQ(symmetric_difference[0].inner_rings.size(), 1);
}

TEST(PolygonBooleanTest, AppliesTruthTableToIdenticalRectangles) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = rectangleSegments(Point(0, 0), Point(4, 4));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    ASSERT_EQ(intersection.size(), 1);
    EXPECT_EQ(totalPolygonArea(intersection), 16);

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    EXPECT_TRUE(difference.empty());
    EXPECT_TRUE(symmetric_difference.empty());
}

TEST(PolygonBooleanTest, AppliesTruthTableToIdenticalReversedRectangles) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = rectangleHoleSegments(Point(0, 0), Point(4, 4));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    ASSERT_EQ(intersection.size(), 1);
    EXPECT_EQ(totalPolygonArea(intersection), 16);

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 16);

    EXPECT_TRUE(difference.empty());
    EXPECT_TRUE(symmetric_difference.empty());
}

TEST(PolygonBooleanTest, AppliesTruthTableToPointTouchingRectangles) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(1, 1));
    const std::vector<Segment> right = rectangleSegments(Point(1, 1), Point(2, 2));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 2);
    EXPECT_EQ(totalPolygonArea(union_polygons), 2);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 1);

    ASSERT_EQ(symmetric_difference.size(), 2);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 2);
}

TEST(PolygonBooleanTest, AppliesTruthTableToSharedEdgeRectangles) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(1, 1));
    const std::vector<Segment> right = rectangleSegments(Point(1, 0), Point(2, 1));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 2);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 1);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 2);
}

TEST(PolygonBooleanTest, AppliesTruthTableToPartialSharedEdgeRectangles) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(4, 4));
    const std::vector<Segment> right = rectangleSegments(Point(2, 4), Point(6, 6));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 24);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 16);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 24);
}

TEST(PolygonBooleanTest, DifferenceCanSplitRectangleIntoTwoPolygons) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(6, 4));
    const std::vector<Segment> right = rectangleSegments(Point(2, 0), Point(4, 4));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    ASSERT_EQ(intersection.size(), 1);
    EXPECT_EQ(totalPolygonArea(intersection), 8);

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 24);

    ASSERT_EQ(difference.size(), 2);
    EXPECT_EQ(totalPolygonArea(difference), 16);

    ASSERT_EQ(symmetric_difference.size(), 2);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 16);
}

TEST(PolygonBooleanTest, AppliesTruthTableToContainedRectangleTouchingShellBoundary) {
    const std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(6, 4));
    const std::vector<Segment> right = rectangleSegments(Point(2, 0), Point(4, 2));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    ASSERT_EQ(intersection.size(), 1);
    EXPECT_EQ(totalPolygonArea(intersection), 4);

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 24);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 20);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 20);
}

TEST(PolygonBooleanTest, AppliesTruthTableToPolygonInsideHole) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(10, 10));
    appendSegments(left, rectangleHoleSegments(Point(3, 3), Point(7, 7)));
    const std::vector<Segment> right = rectangleSegments(Point(4, 4), Point(6, 6));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 2);
    EXPECT_EQ(totalPolygonArea(union_polygons), 88);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 84);
    ASSERT_EQ(difference[0].inner_rings.size(), 1);

    ASSERT_EQ(symmetric_difference.size(), 2);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 88);
}

TEST(PolygonBooleanTest, AppliesTruthTableToPolygonPointTouchingHoleBoundary) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(10, 10));
    appendSegments(left, rectangleHoleSegments(Point(3, 3), Point(7, 7)));
    const std::vector<Segment> right = ringSegments({
        Point(7, 7),
        Point(6, Rational(13, 2)),
        Point(Rational(11, 2), 6),
        Point(Rational(13, 2), Rational(11, 2)),
    });

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 2);
    EXPECT_EQ(totalPolygonArea(union_polygons), 85);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 84);
    ASSERT_EQ(difference[0].inner_rings.size(), 1);

    ASSERT_EQ(symmetric_difference.size(), 2);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 85);
}

TEST(PolygonBooleanTest, AppliesTruthTableToPolygonSharingHoleBoundaryEdges) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(10, 10));
    appendSegments(left, rectangleHoleSegments(Point(3, 3), Point(7, 7)));
    const std::vector<Segment> right = rectangleSegments(Point(6, 6), Point(7, 7));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 85);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 84);
    ASSERT_EQ(difference[0].inner_rings.size(), 1);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 85);
}

TEST(PolygonBooleanTest, AppliesTruthTableToRectangleCrossingHoleBoundary) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(10, 10));
    appendSegments(left, rectangleHoleSegments(Point(3, 3), Point(7, 7)));
    const std::vector<Segment> right = rectangleSegments(Point(5, 5), Point(9, 9));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    ASSERT_EQ(intersection.size(), 1);
    EXPECT_EQ(totalPolygonArea(intersection), 12);

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 88);

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 72);

    ASSERT_EQ(symmetric_difference.size(), 2);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 76);
}

TEST(PolygonBooleanTest, AppliesTruthTableToPolygonMatchingHoleBoundary) {
    std::vector<Segment> left = rectangleSegments(Point(0, 0), Point(10, 10));
    appendSegments(left, rectangleHoleSegments(Point(3, 3), Point(7, 7)));
    const std::vector<Segment> right = rectangleSegments(Point(3, 3), Point(7, 7));

    const std::vector<Polygon> intersection = assemblePolygons(polygonAnd(left, right));
    const std::vector<Polygon> union_polygons = assemblePolygons(polygonOr(left, right));
    const std::vector<Polygon> difference = assemblePolygons(polygonDifference(left, right));
    const std::vector<Polygon> symmetric_difference = assemblePolygons(polygonXor(left, right));

    EXPECT_TRUE(intersection.empty());

    ASSERT_EQ(union_polygons.size(), 1);
    EXPECT_EQ(totalPolygonArea(union_polygons), 100);
    EXPECT_TRUE(union_polygons[0].inner_rings.empty());

    ASSERT_EQ(difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(difference), 84);
    ASSERT_EQ(difference[0].inner_rings.size(), 1);

    ASSERT_EQ(symmetric_difference.size(), 1);
    EXPECT_EQ(totalPolygonArea(symmetric_difference), 100);
    EXPECT_TRUE(symmetric_difference[0].inner_rings.empty());
}

TEST(PolygonBooleanTest, BuildsIntersectionOfTwoDonuts) {
    std::vector<Segment> layer1 = rectangleSegments(Point(1, 1), Point(7, 6));
    const std::vector<Segment> layer1_hole = rectangleHoleSegments(
        Point(Rational(11, 2), Rational(9, 2)), Point(Rational(13, 2), Rational(11, 2)));
    layer1.insert(layer1.end(), layer1_hole.begin(), layer1_hole.end());

    std::vector<Segment> layer2 = rectangleSegments(Point(4, 3), Point(10, 8));
    const std::vector<Segment> layer2_hole = rectangleHoleSegments(Point(5, 4), Point(6, 5));
    layer2.insert(layer2.end(), layer2_hole.begin(), layer2_hole.end());

    const std::vector<Segment> intersection_segments = polygonAnd(layer1, layer2);
    const std::vector<Polygon> polygons = assemblePolygons(intersection_segments);

    EXPECT_EQ(intersection_segments.size(), 12);
    ASSERT_EQ(polygons.size(), 1);
    EXPECT_EQ(polygons[0].area(), Rational(29, 4));
    EXPECT_EQ(polygons[0].outer_ring.area(), 9);
    ASSERT_EQ(polygons[0].inner_rings.size(), 1);
    EXPECT_EQ(polygons[0].inner_rings[0].area(), Rational(7, 4));
}

TEST(PolygonBooleanTest, AssemblesSharedEdgeRectangleAndConcaveArrow) {
    std::vector<Segment> segments = rectangleSegments(Point(28, 7), Point(33, 11));
    appendSegments(segments, ringSegments({
                                 Point(35, 1),
                                 Point(41, 5),
                                 Point(35, 11),
                                 Point(36, 7),
                                 Point(32, 7),
                                 Point(32, 3),
                                 Point(36, 3),
                             }));

    const std::vector<Polygon> polygons = assemblePolygons(segments);

    EXPECT_FALSE(polygons.empty());
}

TEST(PolygonBooleanTest, ComputesTwoDonutBooleanAreas) {
    std::vector<Segment> layer1 = rectangleSegments(Point(1, 1), Point(7, 6));
    const std::vector<Segment> layer1_hole = rectangleHoleSegments(
        Point(Rational(11, 2), Rational(9, 2)), Point(Rational(13, 2), Rational(11, 2)));
    layer1.insert(layer1.end(), layer1_hole.begin(), layer1_hole.end());

    std::vector<Segment> layer2 = rectangleSegments(Point(4, 3), Point(10, 8));
    const std::vector<Segment> layer2_hole = rectangleHoleSegments(Point(5, 4), Point(6, 5));
    layer2.insert(layer2.end(), layer2_hole.begin(), layer2_hole.end());

    EXPECT_EQ(totalPolygonArea(assemblePolygons(polygonAnd(layer1, layer2))), Rational(29, 4));
    EXPECT_EQ(totalPolygonArea(assemblePolygons(polygonOr(layer1, layer2))), Rational(203, 4));
    EXPECT_EQ(totalPolygonArea(assemblePolygons(polygonDifference(layer1, layer2))), Rational(87, 4));
    EXPECT_EQ(totalPolygonArea(assemblePolygons(polygonXor(layer1, layer2))), Rational(87, 2));
}

TEST(TriangulationTest, TriangulatesConvexQuadrilateral) {
    const LinearRing square({Point(0, 0), Point(4, 0), Point(4, 4), Point(0, 4)});

    const std::vector<LinearRing> triangles = earClippingTriangulation(square);

    ASSERT_EQ(triangles.size(), 2);
    EXPECT_EQ(triangles[0].area() + triangles[1].area(), square.area());
}

TEST(TriangulationTest, ReturnsTriangleForTriangleInput) {
    const LinearRing triangle({Point(0, 0), Point(4, 0), Point(1, 3)});

    const std::vector<LinearRing> triangles = earClippingTriangulation(triangle);

    ASSERT_EQ(triangles.size(), 1);
    EXPECT_EQ(triangles[0].points.size(), 3);
    EXPECT_TRUE(triangles[0].isOuter());
    EXPECT_EQ(triangles[0].area(), triangle.area());
}

TEST(TriangulationTest, RejectsRingsWithFewerThanTwoPoints) {
    EXPECT_THROW(earClippingTriangulation(LinearRing(std::vector<Point>{})), std::invalid_argument);
    EXPECT_THROW(earClippingTriangulation(LinearRing({Point(0, 0)})), std::invalid_argument);
}

TEST(TriangulationTest, TriangulatesConvexPentagon) {
    const LinearRing pentagon({Point(0, 0), Point(5, 0), Point(6, 3), Point(3, 5), Point(0, 3)});

    const std::vector<LinearRing> triangles = earClippingTriangulation(pentagon);

    ASSERT_EQ(triangles.size(), 3);
    Rational triangle_area = 0;
    for (const LinearRing& triangle : triangles) {
        EXPECT_TRUE(triangle.isOuter());
        triangle_area += triangle.area();
    }
    EXPECT_EQ(triangle_area, pentagon.area());
}

TEST(TriangulationTest, TriangulatesConcavePolygon) {
    const LinearRing polygon({Point(0, 0), Point(4, 0), Point(4, 4), Point(2, 2), Point(0, 4)});

    const std::vector<LinearRing> triangles = earClippingTriangulation(polygon);

    ASSERT_EQ(triangles.size(), 3);
    Rational triangle_area = 0;
    for (const LinearRing& triangle : triangles) {
        EXPECT_TRUE(triangle.isOuter());
        triangle_area += triangle.area();
    }
    EXPECT_EQ(triangle_area, polygon.area());
}

TEST(TriangulationTest, TriangulatesConcavePolygonWithMultipleEars) {
    const LinearRing polygon({
        Point(0, 0),
        Point(6, 0),
        Point(6, 6),
        Point(4, 6),
        Point(4, 2),
        Point(2, 2),
        Point(2, 6),
        Point(0, 6),
    });

    const std::vector<LinearRing> triangles = earClippingTriangulation(polygon);

    ASSERT_EQ(triangles.size(), 6);
    Rational triangle_area = 0;
    for (const LinearRing& triangle : triangles) {
        EXPECT_TRUE(triangle.isOuter());
        triangle_area += triangle.area();
    }
    EXPECT_EQ(triangle_area, polygon.area());
}

TEST(MonotoneTriangulationTest, ReturnsTriangleForTriangleInput) {
    const LinearRing ring({Point(0, 0), Point(3, 0), Point(1, 3)});

    const std::vector<LinearRing> triangles = triangulateMonotonePolygon(ring);

    ASSERT_EQ(triangles.size(), 1);
    EXPECT_EQ(triangles.front().points.size(), 3);
    EXPECT_TRUE(triangles.front().isOuter());
    EXPECT_EQ(triangles.front().area(), ring.area());
}

TEST(MonotoneTriangulationTest, ReturnsEmptyForFewerThanThreeVertices) {
    EXPECT_TRUE(triangulateMonotonePolygon(LinearRing(std::vector<Point>{})).empty());
    EXPECT_TRUE(triangulateMonotonePolygon(LinearRing({Point(0, 0)})).empty());
    EXPECT_TRUE(
        triangulateMonotonePolygon(LinearRing({Point(0, 0), Point(1, 1)})).empty());
}

TEST(MonotoneTriangulationTest, TriangulatesConcaveYMonotonePolygon) {
    const LinearRing ring({
        Point(0, 0),
        Point(3, 2),
        Point(2, 4),
        Point(0, 6),
        Point(-3, 4),
        Point(-1, 3),
        Point(-3, 2),
    });

    const std::vector<LinearRing> triangles = triangulateMonotonePolygon(ring);

    ASSERT_EQ(triangles.size(), ring.points.size() - 2);
    Rational triangulated_area = 0;
    for (const LinearRing& triangle : triangles) {
        EXPECT_EQ(triangle.points.size(), 3);
        EXPECT_TRUE(triangle.isOuter());
        triangulated_area += triangle.area();
    }
    EXPECT_EQ(triangulated_area, ring.area());
}

TEST(TriangulationTest, TriangulatesVertexClassificationExample) {
    // Counter-clockwise version of the start/end/split/merge-vertex example. The vertices are
    // deliberately assigned distinct y coordinates so sweep-event ordering is unambiguous.
    const LinearRing polygon({
        Point(10, 5),
        Point(8, 6),
        Point(8, 11),
        Point(6, 10),
        Point(4, 12),
        Point(0, 9),
        Point(2, 7),
        Point(1, 4),
        Point(-1, 6),
        Point(-2, 1),
        Point(0, -2),
        Point(3, -1),
        Point(5, -2),
        Point(4, 4),
        Point(10, 2),
    });

    const std::vector<LinearRing> triangles = earClippingTriangulation(polygon);

    ASSERT_EQ(triangles.size(), polygon.points.size() - 2);
    Rational triangle_area = 0;
    for (const LinearRing& triangle : triangles) {
        EXPECT_TRUE(triangle.isOuter());
        triangle_area += triangle.area();
    }
    EXPECT_EQ(triangle_area, polygon.area());
}

TEST(MonotonePartitionTest, PartitionsVertexClassificationExample) {
    // Counter-clockwise version of the start/end/split/merge-vertex example. The vertices are
    // deliberately assigned distinct y coordinates so sweep-event ordering is unambiguous.
    const LinearRing polygon({
        Point(10, 5),
        Point(8, 6),
        Point(8, 11),
        Point(6, 10),
        Point(4, 12),
        Point(0, 9),
        Point(2, 7),
        Point(1, 4),
        Point(-1, 6),
        Point(-2, 1),
        Point(0, -2),
        Point(3, -1),
        Point(5, -2),
        Point(4, 4),
        Point(10, 2),
    });

    const std::vector<LinearRing> partitions = monotonePartition(polygon);

    ASSERT_EQ(partitions.size(), 4);
    Rational partition_area = 0;
    for (const LinearRing& partition : partitions) {
        EXPECT_TRUE(partition.isOuter());
        partition_area += partition.area();
    }
    EXPECT_EQ(partition_area, polygon.area());
}

TEST(MonotoneChainsTest, ClassifiesVerticesAcrossRingBoundary) {
    // Counter-clockwise diamond with the bottom vertex at index zero. Classifying the left chain
    // therefore exercises the wrap from the final index back to the first index.
    const LinearRing ring({
        Point(0, 0),
        Point(2, 2),
        Point(0, 4),
        Point(-2, 2),
    });

    const std::vector<MonotoneChain> chains = extractMonotoneChains(ring);

    const std::vector<MonotoneChain> expected_chains({
        MonotoneChain::Left,
        MonotoneChain::Right,
        MonotoneChain::Right,
        MonotoneChain::Left,
    });
    EXPECT_EQ(chains, expected_chains);
}

TEST(MonotoneChainsTest, KeepsVertexAlignedLabelsForFewerThanThreeVertices) {
    const std::vector<MonotoneChain> chains =
        extractMonotoneChains(LinearRing({Point(0, 1), Point(0, 0)}));

    EXPECT_EQ(chains, std::vector<MonotoneChain>({MonotoneChain::Right, MonotoneChain::Right}));
}

TEST(MonotonePartitionTest, PartitionsTextbookExample) {
    // Counter-clockwise integer embedding of the pictured v1--v15 boundary. Interior diagonals
    // from the illustration are intentionally omitted; monotonePartition must create them.
    const LinearRing polygon({
        Point(10, 8),
        Point(7, 6),
        Point(7, 12),
        Point(5, 10),
        Point(4, 13),
        Point(0, 9),
        Point(2, 7),
        Point(1, 4),
        Point(-1, 5),
        Point(-2, 1),
        Point(0, -2),
        Point(3, -1),
        Point(6, -5),
        Point(5, 3),
        Point(8, 0),
    });

    const std::vector<LinearRing> partitions = monotonePartition(polygon);

    ASSERT_FALSE(partitions.empty());
    Rational partition_area = 0;
    for (const LinearRing& partition : partitions) {
        EXPECT_TRUE(partition.isOuter());
        partition_area += partition.area();
    }
    EXPECT_EQ(partition_area, polygon.area());
}

TEST(MonotonePartitionTest, DISABLED_RejectsClockwiseRing) {
    LinearRing polygon({
        Point(10, 8),
        Point(7, 6),
        Point(7, 12),
        Point(5, 10),
        Point(4, 13),
        Point(0, 9),
        Point(2, 7),
        Point(1, 4),
        Point(-1, 5),
        Point(-2, 1),
        Point(0, -2),
        Point(3, -1),
        Point(6, -5),
        Point(5, 3),
        Point(8, 0),
    });
    polygon.reverse();

    EXPECT_THROW(monotonePartition(polygon), std::invalid_argument);
}

TEST(MonotonePartitionTest, DISABLED_PreservesAreaWithHorizontalEdge) {
    const LinearRing polygon({
        Point(9, 5),
        Point(6, 5),
        Point(4, 14),
        Point(-3, 7),
        Point(-13, 9),
        Point(12, -8),
    });

    const std::vector<LinearRing> partitions = monotonePartition(polygon);

    Rational partition_area = 0;
    for (const LinearRing& partition : partitions) {
        partition_area += partition.area();
    }
    EXPECT_EQ(partition_area, polygon.area());
}

TEST(TriangulationTest, TriangulatesRingAfterRemovingCollinearVertices) {
    LinearRing ring({Point(0, 0), Point(2, 0), Point(4, 0), Point(4, 4), Point(0, 4)});
    ring.removeCollinearVertices();

    const std::vector<LinearRing> triangles = earClippingTriangulation(ring);

    ASSERT_EQ(ring.points.size(), 4);
    ASSERT_EQ(triangles.size(), 2);
    EXPECT_EQ(triangles[0].area() + triangles[1].area(), ring.area());
}
