#include "geometry/predicates.hpp"

Rational crossProduct(const Vect2& p, const Vect2& q) {
    return p.x * q.y - p.y * q.x;
}

Rational dotProduct(const Vect2& p, const Vect2& q) {
    return p.x * q.x + p.y * q.y;
}

Rational orientation(const Point& p, const Point& q, const Point& r) {
    return crossProduct(q - p, r - p);
}

bool isPointOnSegment(const Point& p, const Segment& s) {
    bool collinear = orientation(s.start, s.end, p) == 0;
    if (!collinear)
        return false;

    // Vectors from the segment endpoints to the point are in opposite directions.
    return dotProduct(p - s.end, p - s.start) <= 0;
}
