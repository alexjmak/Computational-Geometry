#ifndef INTERSECTION_HPP
#define INTERSECTION_HPP

#include "geometry/vector.hpp"
#include <optional>

class Segment;

/// \brief Classify how two segments intersect.
enum class IntersectionType { None, Point, Segment };

/// \brief Return the intersection classification for two segments.
/// \param s The first segment.
/// \param t The second segment.
/// \returns IntersectionType::NONE if the segments do not intersect,
/// IntersectionType::POINT if they intersect at exactly one point, or
/// IntersectionType::Segment if they overlap along a non-zero-length collinear segment.
///
/// Endpoint touching and T-intersections are classified as IntersectionType::POINT.
/// Collinear segments that overlap only at a single endpoint are also classified as
/// IntersectionType::POINT, not IntersectionType::Segment.
IntersectionType intersectionType(const Segment& s, const Segment& t);

/// \brief Compute the intersection point of two segments when it is unique.
/// \param s The first segment.
/// \param t The second segment.
/// \returns The unique intersection point if one exists; std::nullopt if the
/// segments do not intersect or overlap along a segment.
std::optional<Point> intersectionPoint(const Segment& s, const Segment& t);

/// \brief Intersect a segment with the horizontal line through a point.
/// \param s The segment to intersect.
/// \param p A point whose y coordinate defines the horizontal line.
/// \returns The intersection point when the segment spans p.y, otherwise std::nullopt.
std::optional<Point> intersectAtY(const Segment& s, const Point& p);

/// \brief Intersect a segment with the horizontal ray to the left of a point.
/// \param s The segment to intersect.
/// \param p A point whose y coordinate defines the horizontal ray.
/// \returns The intersection point when the segment spans p.y and the intersection is to the left
/// of p, otherwise std::nullopt.
std::optional<Point> leftRayIntersection(const Segment& s, const Point& p);

/// \brief Intersect a segment with the horizontal ray to the right of a point.
/// \param s The segment to intersect.
/// \param p A point whose y coordinate defines the horizontal ray.
/// \returns The intersection point when the segment spans p.y and the intersection is to the right
/// of p, otherwise std::nullopt.
std::optional<Point> rightRayIntersection(const Segment& s, const Point& p);

#endif // INTERSECTION_HPP
