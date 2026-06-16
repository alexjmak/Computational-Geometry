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

int halfPlane(const Vect2& v) {
    // Split the circle at the positive x-axis so angular comparison becomes a
    // strict linear order: upper half-plane first, then lower half-plane.
    if (v.y > 0 || (v.y == 0 && v.x > 0)) {
        return 0;
    }
    return 1;
}

bool angleComparator(const Vect2& a, const Vect2& b) {
    // Check if two vectors are on different half-planes
    // ex: a's angle is between 0 and pi and b's angle is between pi and 2*pi
    int ha = halfPlane(a);
    int hb = halfPlane(b);
    if (ha != hb) {
        return ha < hb;
    }

    // Both vectors are on the same half-plane, use cross produce to determine order
    Rational cross = crossProduct(a, b);
    if (cross != 0) {
        return cross > 0;
    }

    // Both a and b are collinear and have the same ray, break ties with squared length
    Rational a_len2 = dotProduct(a, a);
    Rational b_len2 = dotProduct(b, b);
    return a_len2 < b_len2;
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
