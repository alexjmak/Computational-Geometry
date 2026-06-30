#ifndef OVERLAY_HPP
#define OVERLAY_HPP

#include "geometry/dcel.hpp"
#include "geometry/segment.hpp"
#include <vector>

/// \brief Face labels mapping an overlay face back to the source DCEL faces that contain it.
struct OverlayFaceLabel {
    std::size_t left_face = DCEL::npos;  ///< Face index in the left source DCEL.
    std::size_t right_face = DCEL::npos; ///< Face index in the right source DCEL.
};

/// \brief Result of overlaying two segment arrangements.
struct OverlayResult {
    DCEL dcel;                                 ///< DCEL for the combined planar subdivision.
    std::vector<OverlayFaceLabel> face_labels; ///< Labels indexed by overlay face index.
};

/// \brief Polygon geometry extracted from one finite overlay face with its source label.
struct OverlayFacePolygon {
    std::size_t face_index = DCEL::npos; ///< Face index in the overlay DCEL.
    Polygon polygon;                     ///< Polygon represented by the overlay face.
    OverlayFaceLabel label;              ///< Source-face label for this overlay face.
};

/// \brief Source arrangement for an edge used during overlay construction.
enum class OverlaySource {
    Left, ///< Edge comes from the left input arrangement.
    Right ///< Edge comes from the right input arrangement.
};

/// \brief Input edge annotated with its source arrangement and source half-edge index.
struct OverlaySourceEdge {
    Segment segment;                    ///< Geometry of the source edge.
    OverlaySource source;               ///< Source arrangement containing this edge.
    std::size_t half_edge = DCEL::npos; ///< Half-edge index in the source DCEL.
};

/// \brief Planarized segment piece and the source edges that contributed to it.
struct OverlaySplitEdge {
    Segment segment;                       ///< Geometry of the split edge.
    std::vector<std::size_t> source_edges; ///< Source-edge indices covering this split edge.
};

/// \brief Split segments at all intersections so the result forms a planar subdivision.
/// Duplicate and overlapping input coverage is preserved; this function does not filter duplicate
/// split segments.
/// \param segments Input segments to split.
/// \returns Segment pieces whose intersections occur only at shared endpoints.
std::vector<Segment> planarizeSegments(const std::vector<Segment>& segments);

/// \brief Overlay two existing DCEL arrangements into one combined subdivision.
/// \param left The left source DCEL.
/// \param right The right source DCEL.
/// \returns Combined overlay DCEL with each face labeled by containing source faces.
OverlayResult segmentOverlay(const DCEL& left, const DCEL& right);

/// \brief Build and overlay two segment arrangements.
/// \param left Segments for the left source arrangement.
/// \param right Segments for the right source arrangement.
/// \returns Combined overlay DCEL with each face labeled by containing source faces.
OverlayResult segmentOverlay(const std::vector<Segment>& left, const std::vector<Segment>& right);

/// \brief Extract finite overlay faces as polygons with their source labels.
/// \param overlay Overlay result to inspect.
/// \returns Polygon records for overlay faces that can be represented as polygons.
std::vector<OverlayFacePolygon> overlayFacePolygons(const OverlayResult& overlay);

/// \brief Compute the polygon intersection boundary under odd-even fill.
/// \param left Segments describing the left polygon arrangement.
/// \param right Segments describing the right polygon arrangement.
/// \returns Boundary segments for the region inside both inputs.
std::vector<Segment> polygonAnd(const std::vector<Segment>& left,
                                const std::vector<Segment>& right);

/// \brief Compute the polygon union boundary under odd-even fill.
/// \param left Segments describing the left polygon arrangement.
/// \param right Segments describing the right polygon arrangement.
/// \returns Boundary segments for the region inside either input.
std::vector<Segment> polygonOr(const std::vector<Segment>& left, const std::vector<Segment>& right);

/// \brief Compute the polygon difference boundary under odd-even fill.
/// \param left Segments describing the minuend polygon arrangement.
/// \param right Segments describing the subtrahend polygon arrangement.
/// \returns Boundary segments for the region inside left and outside right.
std::vector<Segment> polygonDifference(const std::vector<Segment>& left,
                                       const std::vector<Segment>& right);

/// \brief Compute the polygon symmetric-difference boundary under odd-even fill.
/// \param left Segments describing the left polygon arrangement.
/// \param right Segments describing the right polygon arrangement.
/// \returns Boundary segments for the region inside exactly one input.
std::vector<Segment> polygonXor(const std::vector<Segment>& left,
                                const std::vector<Segment>& right);

#endif // OVERLAY_HPP
