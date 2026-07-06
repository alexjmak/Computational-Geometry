#include "algorithms/assemble.hpp"
#include "geometry/dcel.hpp"
#include "geometry/polygon.hpp"
#include <cassert>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace {

/// \brief Extract the true boundary segments of the filled regions.
/// \param segments Input segments that may include shared edges, duplicate boundaries, or internal
/// edges.
/// \returns Segments whose incident faces have interior parity on one side and exterior parity on
/// the other. Same-parity edges are omitted so adjacent filled faces assemble as one polygon.
std::vector<Segment> extractFilledBoundarySegments(const std::vector<Segment>& segments) {
    const DCEL dcel = DCEL::fromSegments(segments);
    const std::vector<DCEL::FaceParity> face_parities = dcel.faceParities();
    std::vector<Segment> boundary_segments;

    for (std::size_t i = 0; i < dcel.halfEdgeCount(); ++i) {
        const DCEL::HalfEdge& half_edge = dcel.halfEdge(i);
        const DCEL::HalfEdge& twin_half_edge = dcel.twinOf(half_edge);

        const DCEL::FaceParity face_parity = face_parities[half_edge.face];
        const DCEL::FaceParity twin_face_parity = face_parities[twin_half_edge.face];
        assert(face_parity != DCEL::FaceParity::Unknown);
        assert(twin_face_parity != DCEL::FaceParity::Unknown);

        if (face_parity == DCEL::FaceParity::Interior &&
            twin_face_parity == DCEL::FaceParity::Exterior) {
            boundary_segments.push_back(dcel.segmentOf(half_edge));
        }
    }

    return boundary_segments;
}

} // namespace

std::vector<LinearRing> assembleRings(const std::vector<Segment>& segments) {
    DCEL dcel = DCEL::fromSegments(segments);
    std::vector<LinearRing> rings;

    for (std::size_t i = 0; i < dcel.faceCount(); ++i) {
        const DCEL::Face& face = dcel.face(i);
        if (face.outer_component == DCEL::npos) {
            continue;
        }

        const DCEL::HalfEdge& outer_edge = dcel.halfEdge(face.outer_component);
        rings.push_back(dcel.ringOf(outer_edge));
    }

    return rings;
}

std::vector<Polygon> assemblePolygons(const std::vector<Segment>& segments) {
    const std::vector<Segment> boundary_segments = extractFilledBoundarySegments(segments);

    DCEL dcel = DCEL::fromSegments(boundary_segments);
    const std::vector<DCEL::FaceParity> face_parities = dcel.faceParities();
    std::vector<Polygon> polygons;

    // By the odd-even fill rule, interior faces are filled polygon regions, while exterior
    // faces are unfilled background or holes.
    for (std::size_t i = 0; i < dcel.faceCount(); ++i) {
        const DCEL::FaceParity parity = face_parities[i];
        assert(parity != DCEL::FaceParity::Unknown);

        if (parity == DCEL::FaceParity::Interior) {
            const DCEL::Face& face = dcel.face(i);
            assert(face.outer_component != DCEL::npos);
            std::optional<Polygon> polygon = dcel.polygonOf(face);
            assert(polygon.has_value());
            polygons.push_back(std::move(*polygon));
        }
    }

    return polygons;
}
