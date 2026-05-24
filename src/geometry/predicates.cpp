#include "geometry/predicates.hpp"
#include "geometry/segment.hpp"
#include <cmath>
#include <numbers>

Rational crossProduct(const Vect2& p, const Vect2& q) {
    return p.x * q.y - p.y * q.x;
}

Rational dotProduct(const Vect2& p, const Vect2& q) {
    return p.x * q.x + p.y * q.y;
}

double signedAngle(const Vect2& p, const Vect2& q) {
    return std::atan2(boost::rational_cast<double>(crossProduct(p, q)),
                      boost::rational_cast<double>(dotProduct(p, q)));
}

double ccwAngle(const Vect2& p, const Vect2& q) {
    double signed_angle = signedAngle(p, q);
    if (signed_angle < 0) {
        signed_angle += 2 * std::numbers::pi;
    }
    return signed_angle;
}

double cwAngle(const Vect2& p, const Vect2& q) {
    return ccwAngle(q, p);
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
