#include "algorithms/overlay.hpp"
#include "algorithms/line_segment_intersection.hpp"
#include "geometry/dcel.hpp"
#include "geometry/intersection.hpp"
#include <algorithm>
#include <cassert>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

enum class PolygonBooleanOp {
    And,
    Or,
    Difference,
    Xor,
};

std::optional<Point> nearestLeftBoundaryIntersection(const Polygon& polygon, const Point& origin) {
    std::optional<Point> nearest_left;
    for (const LinearRing* ring : polygon.rings()) {
        for (const Segment& segment : ring->segments()) {
            if (segment.start.y == segment.end.y) {
                continue;
            }

            std::optional<Point> intersection = leftRayIntersection(segment, origin);
            if (!intersection) {
                continue;
            }

            if (!nearest_left || intersection->x > nearest_left->x) {
                nearest_left = intersection;
            }
        }
    }
    return nearest_left;
}

Point representativePointInsideFace(const Polygon& polygon) {
    for (const Segment& segment : polygon.outer_ring.segments()) {
        if (segment.start == segment.end) {
            continue;
        }

        const Point midpoint = (segment.start + segment.end) * Rational(1, 2);
        std::optional<Point> left_intersection = nearestLeftBoundaryIntersection(polygon, midpoint);
        if (!left_intersection) {
            continue;
        }

        const Point candidate = (midpoint + *left_intersection) * Rational(1, 2);
        if (pointInPolygon(polygon, candidate)) {
            return candidate;
        }
    }

    throw std::logic_error("Unable to choose representative point inside overlay face");
}

bool isFilledFace(const std::vector<DCEL::FaceParity>& face_parities, std::size_t face_index) {
    return face_index < face_parities.size() &&
           face_parities[face_index] == DCEL::FaceParity::Interior;
}

bool selectFace(PolygonBooleanOp op, bool left_inside, bool right_inside) {
    switch (op) {
    case PolygonBooleanOp::And:
        return left_inside && right_inside;
    case PolygonBooleanOp::Or:
        return left_inside || right_inside;
    case PolygonBooleanOp::Difference:
        return left_inside && !right_inside;
    case PolygonBooleanOp::Xor:
        return left_inside != right_inside;
    }

    throw std::logic_error("Unknown polygon boolean operation");
}

std::vector<Segment> extractBoundarySegments(const DCEL& dcel,
                                             const std::vector<bool>& selected_faces) {
    std::vector<Segment> segments;
    for (std::size_t i = 0; i < dcel.halfEdgeCount(); ++i) {
        const DCEL::HalfEdge& half_edge = dcel.halfEdge(i);
        const DCEL::HalfEdge& twin = dcel.twinOf(half_edge);

        // Include only the boundary between the selected region and its complement.
        if (selected_faces[half_edge.face] && !selected_faces[twin.face]) {
            segments.push_back(dcel.segmentOf(half_edge));
        }
    }

    return segments;
}

std::vector<Segment> polygonBoolean(const std::vector<Segment>& left,
                                    const std::vector<Segment>& right, PolygonBooleanOp op) {
    DCEL left_dcel = DCEL::fromSegments(left);
    DCEL right_dcel = DCEL::fromSegments(right);
    OverlayResult overlay = segmentOverlay(left_dcel, right_dcel);

    const DCEL& dcel = overlay.dcel;
    const std::vector<DCEL::FaceParity> left_face_parities = left_dcel.faceParities();
    const std::vector<DCEL::FaceParity> right_face_parities = right_dcel.faceParities();
    std::vector<bool> selected_faces(dcel.faceCount(), false);

    assert(overlay.faceLabels.size() == dcel.faceCount());
    for (std::size_t i = 0; i < dcel.faceCount(); ++i) {
        const OverlayFaceLabel& faceLabel = overlay.faceLabels[i];
        const bool left_inside = isFilledFace(left_face_parities, faceLabel.left_face);
        const bool right_inside = isFilledFace(right_face_parities, faceLabel.right_face);
        selected_faces[i] = selectFace(op, left_inside, right_inside);
    }

    return extractBoundarySegments(dcel, selected_faces);
}

} // namespace

std::vector<Segment> planarizeSegments(const std::vector<Segment>& segments) {
    std::vector<std::vector<Point>> intersections = lineSegmentIntersectionBySegments(segments);

    std::unordered_set<Segment> split_segments;
    for (std::size_t i = 0; i < segments.size(); i++) {
        const Segment& segment = segments[i];
        const std::vector<Point>& split_pts = intersections[i];

        // Intersections list might not have the endpoints of segment
        const Point* prev = &segment.start;
        for (std::size_t j = 0; j < split_pts.size(); j++) {
            const Point* curr = &split_pts[j];
            if (*curr != *prev) {
                split_segments.insert(Segment(*prev, *curr));
            }
            prev = curr;
        }

        // Include last section until end
        if (segment.end != *prev) {
            split_segments.insert(Segment(*prev, segment.end));
        }
    }

    std::vector<Segment> result(split_segments.begin(), split_segments.end());
    std::sort(result.begin(), result.end());
    return result;
}

std::vector<OverlaySplitEdge> planarizeSegments(const std::vector<OverlaySourceEdge>& source_edges,
                                                bool canonicalize_direction = true) {
    std::vector<Segment> segments;
    for (const OverlaySourceEdge& source_edge : source_edges) {
        segments.push_back(source_edge.segment);
    }

    std::vector<std::vector<Point>> intersections = lineSegmentIntersectionBySegments(segments);

    std::unordered_map<Segment, std::vector<std::size_t>> source_map;
    for (std::size_t i = 0; i < segments.size(); i++) {
        const Segment& segment = segments[i];
        const std::vector<Point>& split_pts = intersections[i];

        // Intersections list might not have the endpoints of segment
        const Point* prev = &segment.start;
        for (std::size_t j = 0; j < split_pts.size(); j++) {
            const Point* curr = &split_pts[j];
            if (*curr != *prev) {
                Segment split_segment = Segment(*prev, *curr);
                if (canonicalize_direction) {
                    split_segment = split_segment.canonicalizedX();
                }
                source_map[split_segment].push_back(i);
            }
            prev = curr;
        }

        // Include last section until end
        if (segment.end != *prev) {
            Segment split_segment = Segment(*prev, segment.end);
            if (canonicalize_direction) {
                split_segment = split_segment.canonicalizedX();
            }
            source_map[split_segment].push_back(i);
        }
    }

    std::vector<OverlaySplitEdge> split_edges;
    for (const auto& [split_segment, sources] : source_map) {
        split_edges.push_back({.segment = split_segment, .source_edges = sources});
    }

    return split_edges;
}

std::vector<OverlaySourceEdge> collectSourceEdges(const DCEL& dcel, OverlaySource source) {
    std::vector<OverlaySourceEdge> source_edges;

    for (std::size_t i = 0; i < dcel.halfEdgeCount(); ++i) {
        const DCEL::HalfEdge& half_edge = dcel.halfEdge(i);

        if (i > half_edge.twin) {
            // Visit each geometric edge once
            continue;
        }

        source_edges.push_back(
            {.segment = dcel.segmentOf(half_edge), .source = source, .half_edge = i});
    }

    return source_edges;
}

std::size_t locateFaceContainingPoint(const DCEL& dcel, const Point& p) {
    for (std::size_t i = 0; i < dcel.faceCount(); ++i) {
        if (i == DCEL::unbounded_face_index) {
            continue;
        }

        std::optional<Polygon> polygon = dcel.polygonOf(dcel.face(i));
        if (!polygon) {
            continue;
        }

        if (pointInPolygon(*polygon, p)) {
            return i;
        }
    }

    return DCEL::unbounded_face_index;
}

OverlayResult segmentOverlay(const DCEL& left, const DCEL& right) {
    std::vector<OverlaySourceEdge> left_edges = collectSourceEdges(left, OverlaySource::Left);
    std::vector<OverlaySourceEdge> right_edges = collectSourceEdges(right, OverlaySource::Right);

    std::vector<OverlaySourceEdge> source_edges;
    source_edges.insert(source_edges.end(), left_edges.begin(), left_edges.end());
    source_edges.insert(source_edges.end(), right_edges.begin(), right_edges.end());

    std::vector<OverlaySplitEdge> split_edges = planarizeSegments(source_edges, true);

    std::vector<Segment> split_segments;
    for (const OverlaySplitEdge& split_edge : split_edges) {
        split_segments.push_back(split_edge.segment);
    }

    DCEL dcel = DCEL::fromSegments(split_segments);

    std::unordered_map<Segment, std::size_t> split_segment_to_id;
    for (std::size_t i = 0; i < split_segments.size(); ++i) {
        split_segment_to_id[split_segments[i]] = i;
        split_segment_to_id[split_segments[i].reversed()] = i;
    }

    std::vector<std::size_t> dcel_edge_to_segment_id(dcel.halfEdgeCount(), DCEL::npos);
    for (std::size_t i = 0; i < dcel.halfEdgeCount(); i++) {
        const DCEL::HalfEdge& half_edge = dcel.halfEdge(i);
        const Segment segment = dcel.segmentOf(half_edge);
        const auto found = split_segment_to_id.find(segment);
        assert(found != split_segment_to_id.end());
        dcel_edge_to_segment_id[i] = found->second;
    }

    std::vector<OverlayFaceLabel> face_labels;
    for (std::size_t i = 0; i < dcel.faceCount(); ++i) {
        const DCEL::Face& face = dcel.face(i);
        if (face.outer_component == DCEL::npos) {
            face_labels.push_back({.left_face = DCEL::unbounded_face_index,
                                   .right_face = DCEL::unbounded_face_index});
            continue;
        }

        std::optional<Polygon> polygon = dcel.polygonOf(face);
        assert(polygon.has_value());

        // TODO: Currently a brute force way to find face labels, should improve with scanline
        const Point point = representativePointInsideFace(*polygon);
        face_labels.push_back({.left_face = locateFaceContainingPoint(left, point),
                               .right_face = locateFaceContainingPoint(right, point)});
    }
    return {
        .dcel = std::move(dcel),
        .faceLabels = std::move(face_labels),
    };
}

OverlayResult segmentOverlay(const std::vector<Segment>& left, const std::vector<Segment>& right) {
    return segmentOverlay(DCEL::fromSegments(left), DCEL::fromSegments(right));
}

std::vector<OverlayFacePolygon> overlayFacePolygons(const OverlayResult& overlay) {
    std::vector<OverlayFacePolygon> polygons;
    for (std::size_t i = 0; i < overlay.dcel.faceCount(); ++i) {
        std::optional<Polygon> polygon = overlay.dcel.polygonOf(overlay.dcel.face(i));
        if (!polygon) {
            continue;
        }

        assert(i < overlay.faceLabels.size());
        polygons.push_back(
            {.face_index = i, .polygon = std::move(*polygon), .label = overlay.faceLabels[i]});
    }
    return polygons;
}

std::vector<Segment> polygonAnd(const std::vector<Segment>& left,
                                const std::vector<Segment>& right) {
    return polygonBoolean(left, right, PolygonBooleanOp::And);
}

std::vector<Segment> polygonOr(const std::vector<Segment>& left,
                               const std::vector<Segment>& right) {
    return polygonBoolean(left, right, PolygonBooleanOp::Or);
}

std::vector<Segment> polygonDifference(const std::vector<Segment>& left,
                                       const std::vector<Segment>& right) {
    return polygonBoolean(left, right, PolygonBooleanOp::Difference);
}

std::vector<Segment> polygonXor(const std::vector<Segment>& left,
                                const std::vector<Segment>& right) {
    return polygonBoolean(left, right, PolygonBooleanOp::Xor);
}
