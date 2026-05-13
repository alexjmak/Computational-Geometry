#include "algorithms/convex_hull.hpp"
#include "geometry/predicates.hpp"
#include <algorithm>

/// \brief Build one monotone half of a convex hull from sorted points.
/// \param points Points ordered in the direction the half hull should be traced.
/// \returns The points on the requested half hull, including both endpoints.
std::vector<Point> buildHalfHull(const std::vector<Point>& points) {
    std::vector<Point> hull_points;
    for (const auto& point : points) {
        hull_points.push_back(point);
        while (hull_points.size() >= 3 &&
               orientation(hull_points[hull_points.size() - 3], hull_points[hull_points.size() - 2],
                           hull_points.back()) <= 0) {
            hull_points.erase(hull_points.end() - 2);
        }
    }
    return hull_points;
}

std::vector<Segment> slowConvexHull(const std::vector<Point>& points) {
    std::vector<Segment> segments;
    for (const auto& p : points) {
        for (const auto& q : points) {
            if (p == q)
                continue;
            Segment s(p, q);
            bool valid = true;
            for (const auto& r : points) {
                if (orientation(p, q, r) > 0)
                    continue;
                if (isPointOnSegment(r, s))
                    continue;
                valid = false;
                break;
            }
            if (valid) {
                segments.push_back(s);
            }
        }
    }
    return segments;
}

Cycle convexHull(const std::vector<Point>& points) {
    std::vector<Point> sorted_points = points;
    std::sort(sorted_points.begin(), sorted_points.end());
    std::vector<Point> lower_hull = buildHalfHull(sorted_points);
    std::vector<Point> upper_hull =
        buildHalfHull(std::vector<Point>(sorted_points.rbegin(), sorted_points.rend()));
    std::vector<Point> hull;
    hull.reserve(lower_hull.size() + upper_hull.size() - 2);
    hull.insert(hull.end(), lower_hull.begin(), lower_hull.end() - 1);
    hull.insert(hull.end(), upper_hull.begin(), upper_hull.end() - 1);
    return Cycle(hull);
}
