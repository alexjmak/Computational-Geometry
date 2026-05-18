#ifndef PREDICATES_HPP
#define PREDICATES_HPP

#include "geometry/vector.hpp"

class Segment;

/// \brief Compute the 2D cross product of two vectors.
/// \param p The first vector.
/// \param q The second vector.
/// \returns The scalar cross product p x q.
Rational crossProduct(const Vect2& p, const Vect2& q);

/// \brief Compute the 2D dot product of two vectors.
/// \param p The first vector.
/// \param q The second vector.
/// \returns The scalar dot product p dot q.
Rational dotProduct(const Vect2& p, const Vect2& q);

/// \brief Determine the orientation of three points.
/// \param p The first point.
/// \param q The second point.
/// \param r The third point.
/// \returns A positive value for counter-clockwise, a negative value for clockwise,
/// or zero if the points are collinear.
Rational orientation(const Point& p, const Point& q, const Point& r);

/// \brief Check whether a point lies on a segment.
/// \param p The point to test.
/// \param s The segment to test against.
/// \returns True if the point is on the segment, otherwise false.
bool isPointOnSegment(const Point& p, const Segment& s);

#endif // PREDICATES_HPP
