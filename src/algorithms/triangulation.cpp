#include "algorithms/triangulation.hpp"
#include "debug/logging.hpp"
#include "geometry/polygon.hpp"
#include "geometry/predicates.hpp"
#include <cassert>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

/// \brief Mutable adjacency state for one vertex during ear clipping.
struct TriangulationVertex {
    std::size_t prev; ///< Index of the previous active vertex.
    std::size_t next; ///< Index of the next active vertex.
    bool removed;     ///< Whether this vertex has already been clipped.
};

/// \brief Create circular adjacency state for a ring's vertices.
/// \param point_count Number of vertices in the ring.
/// \returns One initialized triangulation vertex per ring point.
std::vector<TriangulationVertex> createTriangulationVertices(std::size_t point_count) {
    std::vector<TriangulationVertex> vertices(point_count);
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        vertices[i].prev = (i + vertices.size() - 1) % vertices.size();
        vertices[i].next = (i + 1) % vertices.size();
        vertices[i].removed = false;
    }
    return vertices;
}

/// \brief Find the ear triangle at an active vertex.
/// \param ring Ring being triangulated.
/// \param vertices Current adjacency and removal state for the ring vertices.
/// \param index Index of the candidate ear vertex.
/// \returns The candidate triangle if it is convex and contains no other active vertex;
/// otherwise std::nullopt.
std::optional<LinearRing> findEarTriangle(const LinearRing& ring,
                                          const std::vector<TriangulationVertex>& vertices,
                                          std::size_t index) {
    const std::size_t prev_index = vertices[index].prev;
    const std::size_t next_index = vertices[index].next;
    const Point& prev = ring.points[prev_index];
    const Point& current = ring.points[index];
    const Point& next = ring.points[next_index];

    if (orientation(prev, current, next) <= 0) {
        return std::nullopt;
    }

    LinearRing triangle({prev, current, next});
    
    // Clipping an ear must not cut across the remaining polygon. For a convex triangle formed by
    // consecutive vertices, it is sufficient to check that no other active vertex lies on or in it.
    for (std::size_t i = 0; i < ring.points.size(); ++i) {
        if (vertices[i].removed || i == prev_index || i == index || i == next_index) {
            continue;
        }
        if (locatePoint(triangle, ring.points[i]) != PointContainment::Outside) {
            return std::nullopt;
        }
    }

    return triangle;
}

} // namespace

std::vector<LinearRing> earClippingTriangulation(const LinearRing& ring) {
    if (debug::triangulationEnabled()) {
        debug::triangulation() << "earClippingTriangulation input:" << std::endl;
        for (std::size_t i = 0; i < ring.points.size(); ++i) {
            debug::triangulation()
                << "  " << ring.points[i].x << ", " << ring.points[i].y << std::endl;
        }
    }

    if (ring.points.size() < 2) {
        throw std::invalid_argument("Cannot triangulate a ring with fewer than two points");
    }

    std::vector<TriangulationVertex> vertices = createTriangulationVertices(ring.points.size());

    std::vector<LinearRing> triangles;
    while (triangles.size() < ring.points.size() - 2) {
        bool found_ear = false;
        for (std::size_t curr = 0; curr < ring.points.size(); ++curr) {
            if (vertices[curr].removed) {
                continue;
            }

            std::optional<LinearRing> triangle = findEarTriangle(ring, vertices, curr);
            if (!triangle.has_value()) {
                continue;
            }

            found_ear = true;
            const std::size_t prev = vertices[curr].prev;
            const std::size_t next = vertices[curr].next;
            vertices[curr].removed = true;
            vertices[prev].next = next;
            vertices[next].prev = prev;

            if (debug::triangulationEnabled()) {
                debug::triangulation()
                    << "found triangle: " << triangle->points[0].x << ", " << triangle->points[0].y
                    << " | " << triangle->points[1].x << ", " << triangle->points[1].y << " | "
                    << triangle->points[2].x << ", " << triangle->points[2].y << std::endl;
            }

            triangles.push_back(std::move(*triangle));
            break;
        }
        assert(found_ear);
        if (!found_ear) {
            break; // Should not happen if the polygon is simple and valid
        }
    }

    return triangles;
}
