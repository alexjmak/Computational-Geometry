#ifndef LINE_SEGMENT_INTERSECTION_HPP
#define LINE_SEGMENT_INTERSECTION_HPP

#include "geometry/intersection.hpp"
#include "geometry/segment.hpp"
#include <unordered_set>
#include <vector>

/// \brief Return all pairwise intersection points by checking every segment pair.
/// \param segments The segments to test for intersections.
/// \returns A set containing every unique intersection point found.
std::unordered_set<Point> bruteForceLineSegmentIntersection(const std::vector<Segment>& segments);

/// \brief Find all segment intersection points using a Bentley-Ottmann-style sweep line.
/// \param segments The segments to test for intersections.
/// \returns A set containing every unique intersection point found.
std::unordered_set<Point> lineSegmentIntersection(const std::vector<Segment>& segments);

/// \brief Find intersection points grouped by input segment using a sweep line.
/// \param segments The segments to test for intersections.
/// \returns A vector indexed by segment ID, with each segment's intersection points sorted.
std::vector<std::vector<Point>>
lineSegmentIntersectionBySegments(const std::vector<Segment>& segments);

#endif // LINE_SEGMENT_INTERSECTION_HPP
