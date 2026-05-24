#ifndef CONVEX_HULL_HPP
#define CONVEX_HULL_HPP

#include "geometry/polygon.hpp"
#include "geometry/segment.hpp"
#include "geometry/vector.hpp"
#include <vector>

/// \brief Compute convex hull boundary segments by checking every directed point pair.
/// \param points The input points.
/// \returns Directed hull boundary segments found by the brute-force algorithm.
std::vector<Segment> slowConvexHull(const std::vector<Point>& points);

/// \brief Compute the convex hull of a point set using a monotone-chain algorithm.
/// \param points The input points.
/// \returns A ring containing the convex hull vertices in boundary order.
Ring convexHull(const std::vector<Point>& points);

#endif // CONVEX_HULL_HPP
