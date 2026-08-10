#include "algorithms/triangulation.hpp"
#include "algorithms/monotone_partition.hpp"
#include "algorithms/sweep_line.hpp"
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

/// \brief Merge a y-monotone ring's boundary chains into top-to-bottom sweep order.
std::vector<std::size_t> buildMonotoneVertexQueue(const LinearRing& ring) {
    const std::size_t point_count = ring.points.size();
    std::size_t top_index = 0;
    for (std::size_t i = 1; i < point_count; ++i) {
        if (sweep::EventPoint(ring.points[i]) < sweep::EventPoint(ring.points[top_index])) {
            top_index = i;
        }
    }

    std::vector<std::size_t> queue;
    queue.reserve(point_count);
    queue.push_back(top_index);

    std::size_t left_index = (top_index + 1) % point_count;
    std::size_t right_index = (top_index + point_count - 1) % point_count;
    while (left_index != right_index) {
        if (sweep::EventPoint(ring.points[left_index]) <
            sweep::EventPoint(ring.points[right_index])) {
            queue.push_back(left_index);
            left_index = (left_index + 1) % point_count;
        } else {
            queue.push_back(right_index);
            right_index = (right_index + point_count - 1) % point_count;
        }
    }
    queue.push_back(left_index);
    return queue;
}

/// \brief Append a triangle with counter-clockwise point order.
void appendMonotoneTriangle(std::vector<LinearRing>& triangles, const LinearRing& ring,
                            std::size_t current, std::size_t previous, std::size_t candidate,
                            MonotoneChain chain) {
    if (chain == MonotoneChain::Left) {
        std::swap(previous, candidate);
    }

    triangles.emplace_back(
        std::vector<Point>{ring.points[current], ring.points[previous], ring.points[candidate]});
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

std::vector<LinearRing> triangulateMonotonePolygon(const LinearRing& monotone_ring) {
    const std::size_t point_count = monotone_ring.points.size();
    if (point_count < 3) {
        return {};
    }

    const std::vector<MonotoneChain> chains = extractMonotoneChains(monotone_ring);
    const std::vector<std::size_t> queue = buildMonotoneVertexQueue(monotone_ring);
    assert(queue.size() == chains.size());

    std::vector<LinearRing> triangles;
    triangles.reserve(point_count - 2);

    // The stack is the boundary of the region not yet triangulated. Its vertices remain in sweep
    // order, with the most recently processed vertex at the back.
    std::vector<std::size_t> stack;
    stack.reserve(point_count);
    stack.push_back(queue[0]);
    stack.push_back(queue[1]);

    // The first two vertices initialize the stack; process every remaining vertex except the
    // bottom, which is connected after the loop.
    for (std::size_t i = 2; i < queue.size() - 1; ++i) {
        const MonotoneChain curr_chain = chains[queue[i]];
        if (curr_chain == chains[stack.back()]) {
            // On the same chain, connect the current vertex to consecutive stack vertices while
            // each new diagonal remains inside the polygon.
            std::size_t prev = stack.back();
            stack.pop_back();
            while (!stack.empty()) {
                const std::size_t candidate = stack.back();
                Segment top_segment =
                    curr_chain == MonotoneChain::Left
                        ? Segment(monotone_ring.points[candidate], monotone_ring.points[prev])
                        : Segment(monotone_ring.points[prev], monotone_ring.points[candidate]);

                if (orientation(monotone_ring.points[queue[i]], top_segment.start,
                                top_segment.end) <= 0) {
                    // Diagonal is not inside the polygon
                    break;
                }

                stack.pop_back();
                appendMonotoneTriangle(triangles, monotone_ring, queue[i], prev, candidate,
                                       curr_chain);
                prev = candidate;
            }

            stack.push_back(prev);
            stack.push_back(queue[i]);
        } else {
            while (stack.size() > 1) {
                const std::size_t candidate = stack.back();
                stack.pop_back();
                appendMonotoneTriangle(triangles, monotone_ring, queue[i], stack.back(), candidate,
                                       curr_chain);
            }
            stack.pop_back();

            // The fan consumes the old frontier.
            // The previous and current events become the first boundary edge of the region that
            // remains below the sweep.
            stack.push_back(queue[i - 1]);
            stack.push_back(queue[i]);
        }
    }

    // The bottom vertex sees every remaining stack vertex, completing the final triangle fan.
    std::size_t prev = stack.back();
    stack.pop_back();
    while (!stack.empty()) {
        const std::size_t candidate = stack.back();
        stack.pop_back();
        appendMonotoneTriangle(triangles, monotone_ring, queue.back(), prev, candidate,
                               chains[prev]);
        prev = candidate;
    }

    return triangles;
}

std::vector<LinearRing> triangulate(const LinearRing& ring) {
    std::vector<LinearRing> monotone_polygons;
    monotone_polygons = monotonePartition(ring);

    std::vector<LinearRing> triangles;
    for (const LinearRing& monotone_polygon : monotone_polygons) {
        std::vector<LinearRing> monotone_triangles = triangulateMonotonePolygon(monotone_polygon);
        triangles.insert(triangles.end(), monotone_triangles.begin(), monotone_triangles.end());
    }
    return triangles;
}
