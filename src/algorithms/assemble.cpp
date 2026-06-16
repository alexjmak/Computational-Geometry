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
    const std::vector<std::size_t> face_depths = dcel.faceDepths();
    std::vector<Polygon> polygons;

    // Face depth counts how many boundary rings separate a face from the unbounded face.
    // By the odd-even fill rule, odd-depth faces are filled polygon interiors, while
    // even-depth faces are exterior/hole regions.
    for (std::size_t i = 0; i < dcel.faceCount(); ++i) {
        const std::size_t depth = face_depths[i];
        assert(depth != DCEL::npos);

        if (depth % 2 == 1) {
            const DCEL::Face& face = dcel.face(i);
            assert(face.outer_component != DCEL::npos);
            std::optional<Polygon> polygon = dcel.polygonOf(face);
            assert(polygon.has_value());
            polygons.push_back(std::move(*polygon));
        }
    }

    return polygons;
}
