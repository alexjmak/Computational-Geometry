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
    EXPECT_EQ(intersectionType(Segment(Point(0, 0), Point(1, 0)),
                               Segment(Point(0, 1), Point(1, 1))),
              IntersectionType::NONE);
    EXPECT_EQ(intersectionType(Segment(Point(0, 0), Point(4, 4)),
                               Segment(Point(0, 4), Point(4, 0))),
              IntersectionType::POINT);
    EXPECT_EQ(intersectionType(Segment(Point(0, 0), Point(4, 0)),
                               Segment(Point(4, 0), Point(5, 1))),
              IntersectionType::POINT);
    EXPECT_EQ(intersectionType(Segment(Point(0, 0), Point(4, 0)),
                               Segment(Point(2, 0), Point(6, 0))),
              IntersectionType::SEGMENT);
}

TEST(IntersectionTest, ComputesUniqueIntersectionPoints) {
    EXPECT_EQ(intersectionPoint(Segment(Point(0, 0), Point(4, 4)),
                                Segment(Point(0, 4), Point(4, 0))),
              Point(2, 2));
    EXPECT_EQ(intersectionPoint(Segment(Point(0, 0), Point(4, 0)),
                                Segment(Point(4, 0), Point(5, 1))),
              Point(4, 0));
    EXPECT_EQ(intersectionPoint(Segment(Point(0, 0), Point(4, 0)),
                                Segment(Point(2, 0), Point(6, 0))),
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
    EXPECT_EQ(outer.segments(), std::vector<Segment>({Segment(Point(0, 2), Point(0, 0)),
                                                      Segment(Point(0, 0), Point(2, 0)),
                                                      Segment(Point(2, 0), Point(2, 2)),
                                                      Segment(Point(2, 2), Point(0, 2))}));
}

TEST(PolygonTest, ValidatesCycleOrientations) {
    const Cycle outer({Point(0, 0), Point(4, 0), Point(4, 4), Point(0, 4)});
    const Cycle hole({Point(1, 1), Point(1, 2), Point(2, 2), Point(2, 1)});

    EXPECT_NO_THROW(Polygon(outer, {hole}));
    EXPECT_THROW([] { Polygon polygon(Cycle({Point(1, 1), Point(1, 2), Point(2, 2),
                                             Point(2, 1)})); }(),
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
