#include "algorithms/overlay.hpp"
#include "algorithms/line_segment_intersection.hpp"
#include "debug/logging.hpp"
#include "geometry/dcel.hpp"
#include "geometry/intersection.hpp"
#include <cassert>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace {

/// \brief Boolean operation to apply to the filled regions of two source arrangements.
enum class PolygonBooleanOp {
    And,        ///< Keep faces inside both inputs.
    Or,         ///< Keep faces inside either input.
    Difference, ///< Keep faces inside the left input and outside the right input.
    Xor,        ///< Keep faces inside exactly one input.
};

/// \brief Test whether a source face is part of the filled polygon region.
/// \param face_parities Face parity values indexed by source face index.
/// \param face_index Source face index to inspect.
/// \returns True when the face exists and has interior parity.
bool isFilledFace(const std::vector<DCEL::FaceParity>& face_parities, std::size_t face_index) {
    return face_index < face_parities.size() &&
           face_parities[face_index] == DCEL::FaceParity::Interior;
}

/// \brief Apply a boolean operation to a pair of source containment flags.
/// \param op Boolean operation to evaluate.
/// \param left_inside Whether the overlay face is inside the left input.
/// \param right_inside Whether the overlay face is inside the right input.
/// \returns True when the overlay face should be included in the output region.
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

/// \brief Extract boundary segments between selected and unselected overlay faces.
/// \param dcel Overlay DCEL to inspect.
/// \param selected_faces Selection mask indexed by overlay face index.
/// \returns Directed boundary segments for the selected region.
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

/// \brief Compute a polygon boolean operation by overlaying both segment arrangements.
/// \param left Segments describing the left polygon arrangement.
/// \param right Segments describing the right polygon arrangement.
/// \param op Boolean operation to apply to overlay faces.
/// \returns Boundary segments for the selected result region.
std::vector<Segment> polygonBoolean(const std::vector<Segment>& left,
                                    const std::vector<Segment>& right, PolygonBooleanOp op) {
    DCEL left_dcel = DCEL::fromSegments(left);
    DCEL right_dcel = DCEL::fromSegments(right);
    OverlayResult overlay = segmentOverlay(left_dcel, right_dcel);

    const DCEL& dcel = overlay.dcel;
    const std::vector<DCEL::FaceParity> left_face_parities = left_dcel.faceParities();
    const std::vector<DCEL::FaceParity> right_face_parities = right_dcel.faceParities();
    std::vector<bool> selected_faces(dcel.faceCount(), false);

    assert(overlay.face_labels.size() == dcel.faceCount());
    for (std::size_t i = 0; i < dcel.faceCount(); ++i) {
        const OverlayFaceLabel& faceLabel = overlay.face_labels[i];
        const bool left_inside = isFilledFace(left_face_parities, faceLabel.left_face);
        const bool right_inside = isFilledFace(right_face_parities, faceLabel.right_face);
        selected_faces[i] = selectFace(op, left_inside, right_inside);
    }

    return extractBoundarySegments(dcel, selected_faces);
}

/// \brief Collect each geometric edge of a DCEL once and annotate it with its source DCEL.
/// \param dcel Source DCEL to inspect.
/// \param source Source label assigned to the collected edges.
/// \returns Source-edge records for one half-edge from each twin pair.
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

/// \brief Point-location queries used to fill overlay face labels for a source DCEL.
struct FaceQueries {
    const DCEL& source;                        ///< Source DCEL to query.
    OverlaySource source_label;                ///< The label of the source DCEL being filled.
    std::vector<Point> points;                 ///< Representative overlay-face points to locate.
    std::vector<std::size_t> overlay_face_ids; ///< Overlay face index for each query point.
};

/// \brief Convert a nearest-left half-edge hit into the containing source face.
/// \param dcel Source DCEL that produced the hit.
/// \param half_edge_index Half-edge hit returned by DCEL::computeNearestLeftHalfEdges.
/// \returns Containing source face index, or DCEL::unbounded_face_index for no hit.
std::size_t faceFromNearestLeftHalfEdge(const DCEL& dcel, const std::size_t half_edge_index) {
    if (half_edge_index == DCEL::npos) {
        return DCEL::unbounded_face_index;
    }

    return dcel.halfEdge(half_edge_index).face;
}

/// \brief Locate the missing source face by shooting left rays from representative overlay face
/// vertices into the opposite source DCEL. The hit half-edge is oriented so its incident face
/// contains the query point.
/// \param queries Point-location queries used to fill overlay face labels for a source DCEL.
/// \param face_labels Overlay face labels to update.
void fillMissingFaceLabels(const FaceQueries& queries, std::vector<OverlayFaceLabel>& face_labels) {
    assert(queries.points.size() == queries.overlay_face_ids.size());

    const std::vector<std::size_t> hits =
        queries.source.computeNearestLeftHalfEdges(queries.points);
    assert(hits.size() == queries.points.size());

    for (std::size_t i = 0; i < hits.size(); ++i) {
        OverlayFaceLabel& label = face_labels[queries.overlay_face_ids[i]];
        std::size_t& face_label =
            queries.source_label == OverlaySource::Left ? label.left_face : label.right_face;

        assert(face_label == DCEL::npos);
        face_label = faceFromNearestLeftHalfEdge(queries.source, hits[i]);
    }
}

/// \brief Collect representative points for overlay faces with exactly one missing source label.
/// \param overlay Overlay DCEL whose faces need labels.
/// \param face_labels Current labels after source-edge labeling.
/// \param left_face_queries Queries to fill missing left source labels.
/// \param right_face_queries Queries to fill missing right source labels.
void collectMissingFaceQueries(const DCEL& overlay,
                               const std::vector<OverlayFaceLabel>& face_labels,
                               FaceQueries& left_face_queries, FaceQueries& right_face_queries) {
    for (std::size_t i = 0; i < overlay.faceCount(); ++i) {
        const OverlayFaceLabel& face_label = face_labels[i];
        assert(face_label.left_face != DCEL::npos || face_label.right_face != DCEL::npos);

        if (face_label.left_face != DCEL::npos && face_label.right_face != DCEL::npos) {
            continue;
        }

        const DCEL::Face& face = overlay.face(i);
        assert(face.outer_component != DCEL::npos);
        const DCEL::HalfEdge& outer_half_edge = overlay.halfEdge(face.outer_component);

        LinearRing outer_ring = overlay.ringOf(outer_half_edge);
        const Point& representative_point = outer_ring.points[0];

        FaceQueries& queries =
            face_label.left_face == DCEL::npos ? left_face_queries : right_face_queries;
        queries.points.push_back(representative_point);
        queries.overlay_face_ids.push_back(i);
    }
}

/// \brief Split source edges at all intersections while preserving source-edge provenance.
/// \param source_edges Source edges from the input DCELs.
/// \param canonicalize_direction Whether split pieces should be canonicalized before grouping.
/// \returns Split edges with the source-edge indices that cover each split segment.
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

/// \brief Label each overlay face with the containing source faces from both inputs.
/// \param left Left source DCEL.
/// \param right Right source DCEL.
/// \param overlay Combined overlay DCEL.
/// \param overlay_edge_to_split_edge Split-edge index for each overlay half-edge.
/// \param split_edges Planarized split edges with source-edge provenance.
/// \param source_edges Original source edges from both input DCELs.
/// \returns Source face labels indexed by overlay face index.
std::vector<OverlayFaceLabel>
getFaceLabels(const DCEL& left, const DCEL& right, const DCEL& overlay,
              const std::vector<std::size_t>& overlay_edge_to_split_edge,
              const std::vector<OverlaySplitEdge>& split_edges,
              const std::vector<OverlaySourceEdge>& source_edges) {
    assert(overlay_edge_to_split_edge.size() == overlay.halfEdgeCount());

    std::vector<OverlayFaceLabel> face_labels(overlay.faceCount());
    face_labels[DCEL::unbounded_face_index] = {.left_face = DCEL::unbounded_face_index,
                                               .right_face = DCEL::unbounded_face_index};

    // Label the easy cases:
    // For each overlay half-edge backed by a source edge,
    // copy the source face on the same side into the overlay half-edge's incident face.
    for (std::size_t overlay_edge_index = 0; overlay_edge_index < overlay.halfEdgeCount();
         overlay_edge_index++) {
        const DCEL::HalfEdge& overlay_half_edge = overlay.halfEdge(overlay_edge_index);
        const Segment overlay_segment = overlay.segmentOf(overlay_half_edge);
        const std::size_t split_edge_index = overlay_edge_to_split_edge[overlay_edge_index];
        for (const std::size_t source_edge_index : split_edges[split_edge_index].source_edges) {
            const OverlaySourceEdge& source_edge = source_edges[source_edge_index];

            const DCEL& source = source_edge.source == OverlaySource::Left ? left : right;
            const DCEL::HalfEdge& source_half_edge = source.halfEdge(source_edge.half_edge);

            std::size_t& face_label = source_edge.source == OverlaySource::Left
                                          ? face_labels[overlay_half_edge.face].left_face
                                          : face_labels[overlay_half_edge.face].right_face;

            // Same direction means the source half-edge has the same incident side as the overlay
            // half-edge; reversed direction means the source twin does.
            const bool same_direction = overlay_segment.isCanonicalizedX() ==
                                        source.segmentOf(source_half_edge).isCanonicalizedX();
            if (same_direction) {
                assert(face_label == DCEL::npos || face_label == source_half_edge.face);
                face_label = source_half_edge.face;
            } else {
                const DCEL::HalfEdge& source_twin_half_edge = source.twinOf(source_half_edge);
                assert(face_label == DCEL::npos || face_label == source_twin_half_edge.face);
                face_label = source_twin_half_edge.face;
            }
        }
    }

    // At this point, each overlay face has a label from at least one source. For faces missing the
    // other source label, find it by shooting a left ray from a representative vertex into the
    // other source DCEL.
    FaceQueries left_face_queries{.source = left, .source_label = OverlaySource::Left};
    FaceQueries right_face_queries{.source = right, .source_label = OverlaySource::Right};
    collectMissingFaceQueries(overlay, face_labels, left_face_queries, right_face_queries);

    fillMissingFaceLabels(left_face_queries, face_labels);
    fillMissingFaceLabels(right_face_queries, face_labels);

    for (std::size_t i = 0; i < face_labels.size(); ++i) {
        const OverlayFaceLabel& face_label = face_labels[i];
        assert(face_label.left_face != DCEL::npos);
        assert(face_label.right_face != DCEL::npos);

        if (debug::overlayEnabled()) {
            debug::overlay() << "Overlay face " << i << " has label (left=" << face_label.left_face
                             << ", right=" << face_label.right_face << ")\n";
        }
    }

    return face_labels;
}

} // namespace

std::vector<Segment> planarizeSegments(const std::vector<Segment>& segments) {
    std::vector<std::vector<Point>> intersections = lineSegmentIntersectionBySegments(segments);

    std::vector<Segment> split_segments;
    for (std::size_t i = 0; i < segments.size(); i++) {
        const Segment& segment = segments[i];
        const std::vector<Point>& split_pts = intersections[i];

        // Intersections list might not have the endpoints of segment
        const Point* prev = &segment.start;
        for (std::size_t j = 0; j < split_pts.size(); j++) {
            const Point* curr = &split_pts[j];
            if (*curr != *prev) {
                split_segments.push_back(Segment(*prev, *curr));
            }
            prev = curr;
        }

        // Include last section until end
        if (segment.end != *prev) {
            split_segments.push_back(Segment(*prev, segment.end));
        }
    }

    return split_segments;
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

    std::vector<std::size_t> overlay_edge_to_split_edge(dcel.halfEdgeCount(), DCEL::npos);
    for (std::size_t i = 0; i < dcel.halfEdgeCount(); i++) {
        const DCEL::HalfEdge& half_edge = dcel.halfEdge(i);
        const Segment segment = dcel.segmentOf(half_edge);
        const auto found = split_segment_to_id.find(segment);
        assert(found != split_segment_to_id.end());
        overlay_edge_to_split_edge[i] = found->second;
    }

    std::vector<OverlayFaceLabel> face_labels =
        getFaceLabels(left, right, dcel, overlay_edge_to_split_edge, split_edges, source_edges);

    return {
        .dcel = std::move(dcel),
        .face_labels = std::move(face_labels),
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

        assert(i < overlay.face_labels.size());
        polygons.push_back(
            {.face_index = i, .polygon = std::move(*polygon), .label = overlay.face_labels[i]});
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
