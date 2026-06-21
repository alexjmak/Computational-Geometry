#include "geometry/intersection.hpp"
#include "geometry/predicates.hpp"
#include "geometry/segment.hpp"
#include <optional>

IntersectionType intersectionType(const Segment& s, const Segment& t) {
    Rational o1 = orientation(s.start, s.end, t.start);
    Rational o2 = orientation(s.start, s.end, t.end);
    Rational o3 = orientation(t.start, t.end, s.start);
    Rational o4 = orientation(t.start, t.end, s.end);

    // Coincident segments.
    if (o1 == 0 && o2 == 0 && o3 == 0 && o4 == 0) {
        Segment s2 = s.canonicalizedX();
        Segment t2 = t.canonicalizedX();
        if (s2.end > t2.start && s2.start < t2.end) {
            return IntersectionType::Segment;
        }
    }

    // X intersection.
    if (o1 * o2 < 0 && o3 * o4 < 0) {
        return IntersectionType::Point;
    }

    // T intersection.
    if (o1 == 0 && isPointOnSegment(t.start, s))
        return IntersectionType::Point;
    if (o2 == 0 && isPointOnSegment(t.end, s))
        return IntersectionType::Point;
    if (o3 == 0 && isPointOnSegment(s.start, t))
        return IntersectionType::Point;
    if (o4 == 0 && isPointOnSegment(s.end, t))
        return IntersectionType::Point;

    return IntersectionType::None;
}

std::optional<Point> intersectionPoint(const Segment& s, const Segment& t) {
    if (intersectionType(s, t) != IntersectionType::Point) {
        return std::nullopt;
    }

    // Parametric form of the two lines:
    //     p = s.start + a * (s.end - s.start)
    //     p = t.start + b * (t.end - t.start)
    //
    // Rearranged into a 2x2 system:
    //     a * (s.end - s.start) + b * (t.start - t.end) = t.start - s.start
    Rational det = crossProduct(s.end - s.start, t.start - t.end);

    // Collinear endpoints touching.
    if (det == 0) {
        if (isPointOnSegment(t.start, s))
            return t.start;
        if (isPointOnSegment(t.end, s))
            return t.end;
        if (isPointOnSegment(s.start, t))
            return s.start;
        if (isPointOnSegment(s.end, t))
            return s.end;
    }

    // Cramer's Rule.
    Rational det_a = crossProduct(t.start - s.start, t.start - t.end);
    Rational a = det_a / det;
    return s.start + (s.end - s.start) * a;
}

std::optional<Point> intersectAtY(const Segment& s, const Point& p) {
    Segment cs = s.canonicalizedY();
    if (p.y < cs.start.y || p.y > cs.end.y) {
        return std::nullopt;
    }
    if (cs.start.y == cs.end.y) {
        if (p.x > cs.start.x || p.x < cs.end.x) {
            return cs.start;
        }
        return p;
    }
    Rational t = (p.y - cs.start.y) / (cs.end.y - cs.start.y);
    return cs.start + (cs.end - cs.start) * t;
}

std::optional<Point> leftRayIntersection(const Segment& segment, const Point& origin) {
    std::optional<Point> intersection = intersectAtY(segment, origin);
    if (!intersection) {
        return std::nullopt;
    }

    if (intersection->x >= origin.x) {
        return std::nullopt;
    }

    return intersection;
}

std::optional<Point> rightRayIntersection(const Segment& segment, const Point& origin) {
    std::optional<Point> intersection = intersectAtY(segment, origin);
    if (!intersection) {
        return std::nullopt;
    }

    if (intersection->x <= origin.x) {
        return std::nullopt;
    }

    return intersection;
}