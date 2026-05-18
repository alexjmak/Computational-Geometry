#include "algorithms/assemble.hpp"
#include "geometry/dcel.hpp"
#include <cassert>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

std::vector<Cycle> assembleCycles(const std::vector<Segment>& segments) {
    std::unordered_map<Point, std::vector<Point>> adjacency_list;
    std::unordered_set<Segment> seen;

    // Create adjacency list based on the segments
    for (const Segment& segment : segments) {
        if (segment.start == segment.end) {
            throw std::invalid_argument("Degenerate segment with identical endpoints: " +
                                        segment.toString());
        }

        Segment canonical_segment = segment.canonicalizedX();
        if (seen.contains(canonical_segment)) {
            throw std::invalid_argument("Duplicate segment: " + segment.toString());
        }

        seen.insert(canonical_segment);
        adjacency_list[canonical_segment.start].push_back(canonical_segment.end);
        adjacency_list[canonical_segment.end].push_back(canonical_segment.start);
    }

    // Validate that every endpoint has degree 2
    for (const auto& [point, neighbors] : adjacency_list) {
        if (neighbors.size() != 2) {
            throw std::invalid_argument("Endpoint with degree != 2: " +
                                        std::to_string(neighbors.size()) + ": " + point.toString());
        }
    }

    // Build cycles by walking the adjacency list
    std::vector<Cycle> cycles;
    std::unordered_set<Point> visited;
    for (const auto& [point, neighbors] : adjacency_list) {
        if (visited.contains(point)) {
            continue;
        }

        std::vector<Point> cycle_points;
        const Point start = point;
        Point prev = start;
        Point curr = start;
        do {
            if (visited.contains(curr)) {
                throw std::invalid_argument("Cycle walk reached an already visited point: " +
                                            curr.toString());
            }

            visited.insert(curr);
            cycle_points.push_back(curr);

            const std::vector<Point>& neighbors = adjacency_list[curr];
            const Point& next = neighbors[0] == prev ? neighbors[1] : neighbors[0];
            prev = curr;
            curr = next;
        } while (curr != start);

        Cycle cycle(std::move(cycle_points));
        if (!cycle.isOuter()) {
            cycle.reverse();
        }
        cycles.emplace_back(std::move(cycle));
    }

    return cycles;
}

std::vector<Polygon> assemblePolygons(const std::vector<Segment>& segments) {
    DCEL dcel = DCEL::fromSegments(segments);
    const std::vector<std::size_t> face_depths = dcel.faceDepths();
    std::vector<Polygon> polygons;

    // Face depth counts how many boundary cycles separate a face from the unbounded face.
    // By the odd-even fill rule, odd-depth faces are filled polygon interiors, while
    // even-depth faces are exterior/hole regions.
    for (std::size_t i = 0; i < dcel.faces.size(); ++i) {
        const std::size_t depth = face_depths[i];
        assert(depth != DCEL::npos);

        if (depth % 2 == 1) {
            const DCEL::Face& face = dcel.faces[i];
            assert(face.outer_component != DCEL::npos);
            std::optional<Polygon> polygon = dcel.polygonOf(face);
            assert(polygon.has_value());
            polygons.push_back(std::move(*polygon));
        }
    }

    return polygons;
}
