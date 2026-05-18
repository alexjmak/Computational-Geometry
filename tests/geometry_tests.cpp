#include "geometry/dcel.hpp"
#include "geometry/intersection.hpp"
#include "geometry/polygon.hpp"
#include "geometry/random.hpp"
#include "geometry/segment.hpp"

#include <gtest/gtest.h>

#include <random>
#include <unordered_set>
#include <vector>

TEST(VectorTest, OrdersLexicographicallyByXThenY) {
    EXPECT_LT(Point(0, 10), Point(1, 0));
    EXPECT_LT(Point(1, 0), Point(1, 1));
    EXPECT_FALSE(Point(1, 1) < Point(1, 1));
}

TEST(VectorTest, SupportsExactRationalArithmetic) {
    const Point p(Rational(1, 2), Rational(2, 3));
    const Point q(Rational(1, 3), Rational(1, 6));

    EXPECT_EQ(p + q, Point(Rational(5, 6), Rational(5, 6)));
    EXPECT_EQ(p - q, Point(Rational(1, 6), Rational(1, 2)));
    EXPECT_EQ(p * Rational(6), Point(3, 4));
}

TEST(SegmentTest, ReversesAndCanonicalizesEndpoints) {
    const Segment segment(Point(2, 1), Point(0, 1));

    EXPECT_EQ(segment.reversed(), Segment(Point(0, 1), Point(2, 1)));
    EXPECT_EQ(segment.canonicalizedX(), Segment(Point(0, 1), Point(2, 1)));
    EXPECT_EQ(Segment(Point(0, 0), Point(0, 2)).canonicalizedY(),
              Segment(Point(0, 0), Point(0, 2)));
    EXPECT_EQ(Segment(Point(0, 2), Point(0, 0)).canonicalizedY(),
              Segment(Point(0, 0), Point(0, 2)));
}

TEST(SegmentTest, HashTreatsEqualSegmentsAsSameKey) {
    std::unordered_set<Segment> segments;
    segments.insert(Segment(Point(0, 0), Point(1, 1)));
    segments.insert(Segment(Point(0, 0), Point(1, 1)));
    segments.insert(Segment(Point(1, 1), Point(0, 0)));

    EXPECT_EQ(segments.size(), 2);
}

TEST(PredicateTest, ComputesOrientationSigns) {
    EXPECT_GT(orientation(Point(0, 0), Point(1, 0), Point(1, 1)), 0);
    EXPECT_LT(orientation(Point(0, 0), Point(1, 0), Point(1, -1)), 0);
    EXPECT_EQ(orientation(Point(0, 0), Point(1, 1), Point(2, 2)), 0);
}

TEST(PredicateTest, DetectsPointsOnSegments) {
    const Segment segment(Point(0, 0), Point(4, 0));

    EXPECT_TRUE(isPointOnSegment(Point(2, 0), segment));
    EXPECT_TRUE(isPointOnSegment(Point(0, 0), segment));
    EXPECT_TRUE(isPointOnSegment(Point(4, 0), segment));
    EXPECT_FALSE(isPointOnSegment(Point(5, 0), segment));
    EXPECT_FALSE(isPointOnSegment(Point(2, 1), segment));
}

TEST(IntersectionTest, ClassifiesDisjointCrossingTouchingAndOverlap) {
    EXPECT_EQ(
        intersectionType(Segment(Point(0, 0), Point(1, 0)), Segment(Point(0, 1), Point(1, 1))),
        IntersectionType::NONE);
    EXPECT_EQ(
        intersectionType(Segment(Point(0, 0), Point(4, 4)), Segment(Point(0, 4), Point(4, 0))),
        IntersectionType::POINT);
    EXPECT_EQ(
        intersectionType(Segment(Point(0, 0), Point(4, 0)), Segment(Point(4, 0), Point(5, 1))),
        IntersectionType::POINT);
    EXPECT_EQ(
        intersectionType(Segment(Point(0, 0), Point(4, 0)), Segment(Point(2, 0), Point(6, 0))),
        IntersectionType::SEGMENT);
}

TEST(IntersectionTest, ComputesUniqueIntersectionPoints) {
    EXPECT_EQ(
        intersectionPoint(Segment(Point(0, 0), Point(4, 4)), Segment(Point(0, 4), Point(4, 0))),
        Point(2, 2));
    EXPECT_EQ(
        intersectionPoint(Segment(Point(0, 0), Point(4, 0)), Segment(Point(4, 0), Point(5, 1))),
        Point(4, 0));
    EXPECT_EQ(
        intersectionPoint(Segment(Point(0, 0), Point(4, 0)), Segment(Point(2, 0), Point(6, 0))),
        std::nullopt);
}

TEST(IntersectionTest, IntersectsSegmentAtY) {
    EXPECT_EQ(intersectAtY(Segment(Point(0, 0), Point(4, 4)), Point(0, 2)), Point(2, 2));
    EXPECT_EQ(intersectAtY(Segment(Point(2, 0), Point(2, 4)), Point(0, 3)), Point(2, 3));
    EXPECT_EQ(intersectAtY(Segment(Point(0, 0), Point(4, 0)), Point(2, 0)), Point(2, 0));
    EXPECT_EQ(intersectAtY(Segment(Point(0, 0), Point(4, 4)), Point(0, 5)), std::nullopt);
}

TEST(CycleTest, ComputesSignedAreaOrientationAndSegments) {
    const Cycle outer({Point(0, 0), Point(2, 0), Point(2, 2), Point(0, 2)});
    const Cycle inner({Point(0, 0), Point(0, 2), Point(2, 2), Point(2, 0)});

    EXPECT_DOUBLE_EQ(outer.signedArea(), 4.0);
    EXPECT_DOUBLE_EQ(inner.signedArea(), -4.0);
    EXPECT_TRUE(outer.isOuter());
    EXPECT_FALSE(inner.isOuter());
    EXPECT_EQ(outer.segments(),
              std::vector<Segment>(
                  {Segment(Point(0, 2), Point(0, 0)), Segment(Point(0, 0), Point(2, 0)),
                   Segment(Point(2, 0), Point(2, 2)), Segment(Point(2, 2), Point(0, 2))}));
}

TEST(PolygonTest, ValidatesCycleOrientations) {
    const Cycle outer({Point(0, 0), Point(4, 0), Point(4, 4), Point(0, 4)});
    const Cycle hole({Point(1, 1), Point(1, 2), Point(2, 2), Point(2, 1)});

    EXPECT_NO_THROW(Polygon(outer, {hole}));
    EXPECT_THROW(
        [] { Polygon polygon(Cycle({Point(1, 1), Point(1, 2), Point(2, 2), Point(2, 1)})); }(),
        std::invalid_argument);
    EXPECT_THROW([&] { Polygon polygon(outer, {outer}); }(), std::invalid_argument);
}

TEST(PolygonTest, ReturnsOuterThenInnerCycles) {
    const Cycle outer({Point(0, 0), Point(4, 0), Point(4, 4), Point(0, 4)});
    const Cycle hole({Point(1, 1), Point(1, 2), Point(2, 2), Point(2, 1)});
    Polygon polygon(outer, {hole});

    const std::vector<const Cycle*> cycles = static_cast<const Polygon&>(polygon).cycles();

    ASSERT_EQ(cycles.size(), 2);
    EXPECT_EQ(*cycles[0], outer);
    EXPECT_EQ(*cycles[1], hole);
}

TEST(RandomTest, SeededGenerationIsDeterministic) {
    std::mt19937 rng_a(7);
    std::mt19937 rng_b(7);

    EXPECT_EQ(randomPoints(5, -10, 10, &rng_a), randomPoints(5, -10, 10, &rng_b));
}

TEST(RandomTest, RandomConvexPolygonHasPositiveArea) {
    std::mt19937 rng(3);
    const Cycle polygon = randomConvexPolygon(8, -20, 20, &rng);

    EXPECT_GE(polygon.points.size(), 3);
    EXPECT_TRUE(polygon.isOuter());
}

TEST(DCELTest, CreatesDCELFromPolygons) {
    const Cycle outer({Point(0, 0), Point(4, 0), Point(4, 4), Point(0, 4)});
    const Cycle hole({Point(1, 1), Point(1, 2), Point(2, 2), Point(2, 1)});
    const Polygon polygon(outer, {hole});

    const DCEL dcel = DCEL::fromPolygons({polygon});

    EXPECT_EQ(dcel.points.size(), 8);
    EXPECT_EQ(dcel.half_edges.size(), 16);
    EXPECT_EQ(dcel.faces.size(), 3);

    for (const DCEL::DCELPoint& point : dcel.points) {
        EXPECT_LT(point.half_edge, dcel.half_edges.size());
        EXPECT_EQ(dcel.originOf(dcel.half_edges[point.half_edge]), point);
    }

    for (std::size_t i = 0; i < dcel.half_edges.size(); ++i) {
        const DCEL::HalfEdge& half_edge = dcel.half_edges[i];

        EXPECT_LT(half_edge.origin, dcel.points.size());
        ASSERT_LT(half_edge.twin, dcel.half_edges.size());
        ASSERT_LT(half_edge.next, dcel.half_edges.size());
        ASSERT_LT(half_edge.prev, dcel.half_edges.size());

        EXPECT_EQ(dcel.twinOf(dcel.twinOf(half_edge)).origin, half_edge.origin);
        EXPECT_EQ(dcel.half_edges[half_edge.twin].twin, i);
        EXPECT_EQ(dcel.half_edges[half_edge.next].prev, i);
        EXPECT_EQ(dcel.half_edges[half_edge.prev].next, i);

        const Segment segment = dcel.segmentOf(half_edge);
        EXPECT_EQ(segment.start, dcel.originOf(half_edge));
        EXPECT_EQ(segment.end, dcel.originOf(dcel.twinOf(half_edge)));
    }

    EXPECT_EQ(dcel.faces[DCEL::unbounded_face_index].outer_component, DCEL::npos);
}

TEST(DCELTest, CreatesFacesForDonutWithIsland) {
    const Cycle donut_outer({Point(0, 0), Point(10, 0), Point(10, 10), Point(0, 10)});
    const Cycle donut_hole({Point(3, 3), Point(3, 7), Point(7, 7), Point(7, 3)});
    const Polygon donut(donut_outer, {donut_hole});

    const Cycle island_outer({Point(4, 4), Point(6, 4), Point(6, 6), Point(4, 6)});
    const Polygon island(island_outer);

    const DCEL dcel = DCEL::fromPolygons({donut, island});

    EXPECT_EQ(dcel.points.size(), 12);
    EXPECT_EQ(dcel.half_edges.size(), 24);
    ASSERT_EQ(dcel.faces.size(), 4);
    EXPECT_EQ(dcel.faces[DCEL::unbounded_face_index].outer_component, DCEL::npos);

    std::size_t outer_component_count = 0;
    std::size_t inner_component_count = 0;
    std::size_t faces_with_inner_components = 0;

    for (const DCEL::Face& face : dcel.faces) {
        if (face.outer_component != DCEL::npos) {
            ++outer_component_count;
            EXPECT_LT(face.outer_component, dcel.half_edges.size());
        }
        if (!face.inner_components.empty()) {
            ++faces_with_inner_components;
        }
        inner_component_count += face.inner_components.size();
        for (const std::size_t inner_component : face.inner_components) {
            EXPECT_LT(inner_component, dcel.half_edges.size());
        }
    }

    EXPECT_EQ(outer_component_count, 3);
    EXPECT_EQ(inner_component_count, 3);
    EXPECT_EQ(faces_with_inner_components, 3);
}

TEST(DCELTest, ReusesHalfEdgesForPolygonsSharingEdge) {
    const Polygon left(Rectangle(Point(0, 0), Point(1, 1)).cycle());
    const Polygon right(Rectangle(Point(1, 0), Point(2, 1)).cycle());

    const DCEL dcel = DCEL::fromPolygons({left, right});

    EXPECT_EQ(dcel.points.size(), 6);
    EXPECT_EQ(dcel.half_edges.size(), 14);
    EXPECT_EQ(dcel.faces.size(), 3);
    EXPECT_EQ(dcel.faces[DCEL::unbounded_face_index].outer_component, DCEL::npos);

    std::size_t shared_edge_half_edges = 0;
    for (const DCEL::HalfEdge& half_edge : dcel.half_edges) {
        const Segment segment = dcel.segmentOf(half_edge);
        if ((segment.start == Point(1, 0) && segment.end == Point(1, 1)) ||
            (segment.start == Point(1, 1) && segment.end == Point(1, 0))) {
            ++shared_edge_half_edges;
        }
    }

    EXPECT_EQ(shared_edge_half_edges, 2);
}
