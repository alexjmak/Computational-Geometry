#ifndef RANDOM_HPP
#define RANDOM_HPP

#include "geometry/polygon.hpp"
#include <random>

/// \brief Generate a repeatable set of random 2D points.
/// \param num_points The number of points to generate.
/// \param min_coordinate The minimum x and y coordinate value.
/// \param max_coordinate The maximum x and y coordinate value.
/// \param rng The random number generator used to make generation repeatable.
/// \returns A list of randomly generated points.
std::vector<Point> randomPoints(int num_points, int min_coordinate = 10, int max_coordinate = 100,
                                std::mt19937* rng = nullptr);

/// \brief Generate random line segments with bounded integer endpoints and lengths.
/// \param num_segments The number of segments to generate.
/// \param min_length The minimum segment length.
/// \param max_length The maximum segment length.
/// \param min_coordinate The minimum endpoint coordinate value.
/// \param max_coordinate The maximum endpoint coordinate value.
/// \param rng The random number generator used to make generation repeatable.
/// \returns A list of randomly generated segments.
std::vector<Segment> randomSegments(int num_segments, int min_length = 1, int max_length = 10,
                                    int min_coordinate = 10, int max_coordinate = 100,
                                    std::mt19937* rng = nullptr);

/// \brief Generate a random convex polygon from sampled points.
/// \param n The approximate target number of polygon vertices.
/// \param min_c The minimum coordinate value.
/// \param max_c The maximum coordinate value.
/// \param rng The random number generator used to make generation repeatable.
/// \returns The convex hull of the generated point sample.
Cycle randomConvexPolygon(int n, int min_c = 10, int max_c = 100, std::mt19937* rng = nullptr);

#endif // RANDOM_HPP
