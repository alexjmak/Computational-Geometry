#ifndef TRIANGULATION_HPP
#define TRIANGULATION_HPP

#include "geometry/polygon.hpp"

/// \brief Triangulate a simple counter-clockwise ring using the ear-clipping algorithm.
/// \details The input points must form a simple ring in counter-clockwise order. A two-point ring
/// produces an empty triangulation.
/// \param ring The ring to triangulate.
/// \returns Counter-clockwise triangles covering the same area as the input ring.
/// \throws std::invalid_argument If the ring contains fewer than two points.
std::vector<LinearRing> earClippingTriangulation(const LinearRing& ring);

/// \brief Triangulate a simple counter-clockwise y-monotone ring.
/// \param monotone_ring The y-monotone ring to triangulate.
/// \returns Counter-clockwise triangles covering the same area as the input ring.
std::vector<LinearRing> triangulateMonotonePolygon(const LinearRing& monotone_ring);

/// \brief Partition and triangulate a simple counter-clockwise ring.
/// \param ring The simple ring to triangulate.
/// \returns Counter-clockwise triangles covering the same area as the input ring.
std::vector<LinearRing> triangulate(const LinearRing& ring);

#endif // TRIANGULATION_HPP
