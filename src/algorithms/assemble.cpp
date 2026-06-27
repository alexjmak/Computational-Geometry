#include "algorithms/assemble.hpp"
#include "geometry/dcel.hpp"
#include "geometry/polygon.hpp"
#include <cassert>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

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
    DCEL dcel = DCEL::fromSegments(segments);
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
