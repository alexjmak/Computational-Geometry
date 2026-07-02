#ifndef RAY_HITS_HPP
#define RAY_HITS_HPP

#include "algorithms/sweep_line.hpp"
#include <optional>
#include <vector>

/// \brief Find the nearest segment strictly to the left of each query point.
///
/// The sweep processes segment endpoints and query points from top to bottom. At a query point,
/// active segments are ordered by their intersection with the horizontal line through the query,
/// and the predecessor of the query x-coordinate is reported when it lies strictly to the left.
///
/// \param segments Input segments to test against the leftward rays.
/// \param queries Query points whose leftward rays should be evaluated.
/// \returns A query-indexed vector containing the hit segment ID, or std::nullopt when no segment
/// is strictly left of the query.
std::vector<std::optional<sweep::SegmentId>> leftRayQuery(const std::vector<Segment>& segments,
                                                          const std::vector<Point>& queries);

#endif // RAY_HITS_HPP
