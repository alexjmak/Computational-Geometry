#include "geometry/dcel.hpp"
#include "algorithms/overlay.hpp"
#include "geometry/intersection.hpp"
#include "geometry/polygon.hpp"
#include "geometry/predicates.hpp"
#include "geometry/segment.hpp"
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cstddef>
#ifndef NDEBUG
#include <iostream>
#endif
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

/*
 * DCEL construction overview
 *
 * This file builds a doubly-connected edge list (DCEL) from an arbitrary set
 * of input boundary segments. A DCEL stores each undirected edge as two directed
 * half-edges, links half-edges around vertices and faces, and records one outer
 * component plus zero or more inner components for each face. The structure is
 * used by polygon assembly, overlay, boolean operations, and face parity
 * classification.
 *
 * General algorithm created here:
 *
 * 1. Planarize the input segments. Intersecting segments are split so every
 *    crossing, touch, or overlap endpoint becomes a DCEL vertex. After this
 *    step, edges only meet at shared endpoints.
 * 2. Create a pair of twin half-edges for each directed planar segment
 *    and create the endpoint vertices.
 * 3. For every vertex, collect outgoing half-edges and sort them by
 *    counterclockwise angle around the vertex.
 * 4. Link next/prev pointers by walking from each half-edge to the predecessor
 *    of its twin in the destination vertex's angular order. This is the
 *    standard planar subdivision rule that keeps a boundary traversal with the
 *    incident face consistently on one side.
 * 5. Traverse unvisited next pointers to assemble directed boundary rings.
 *    Each ring is classified by orientation: counterclockwise rings are outer
 *    boundaries of bounded faces, and clockwise rings bound holes or the
 *    unbounded exterior.
 * 6. Group boundary rings that belong to the same face. For each non-outer
 *    ring, the builder finds the half-edge immediately to the left of that
 *    ring's leftmost vertex. The non-outer ring and the ring containing that
 *    left half-edge bound the same face; if no such edge exists, the ring is
 *    grouped with the unbounded face. This implementation answers that query
 *    by scanning the half-edges; it could later be replaced by a DCEL-specific
 *    sweep-line pass.
 * 7. Materialize DCEL face records from the ring groups, attach outer and inner
 *    component representatives, and assign the final face index to every
 *    half-edge in each boundary ring.
 *
 * Later helpers derive polygons from finite faces and compute face parity by
 * traversing the face-adjacency graph from the unbounded face.
 */

namespace {

/// \brief Disjoint-set structure for grouping boundary rings that bound the same face.
class UnionFind {
  public:
    /// \brief Create a new singleton set.
    /// \returns The index of the newly created set.
    std::size_t addSet() {
        const std::size_t index = parent.size();
        // A new set starts as its own root.
        parent.push_back(index);
        // Rank is an upper bound on the tree height. It is used only to keep
        // unions shallow; it is not the number of elements in the set.
        rank.push_back(0);
        return index;
    }

    /// \brief Find the root representative for a set.
    /// \param index The set element to query.
    /// \returns The root representative for index's set.
    std::size_t find(std::size_t index) {
        assert(index < parent.size());

        if (parent[index] != index) {
            // Path compression: after finding the root, point this node
            // directly at it so future finds are faster.
            parent[index] = find(parent[index]);
        }

        return parent[index];
    }

    /// \brief Merge the sets containing two elements.
    /// \param a The first set element.
    /// \param b The second set element.
    void unite(std::size_t a, std::size_t b) {
        std::size_t root_a = find(a);
        std::size_t root_b = find(b);

        if (root_a == root_b) {
            return;
        }

        // Attach the shallower tree under the deeper tree. If the ranks match,
        // either root can win, and the winning root's rank grows by one.
        if (rank[root_a] < rank[root_b]) {
            std::swap(root_a, root_b);
        }

        parent[root_b] = root_a;

        if (rank[root_a] == rank[root_b]) {
            ++rank[root_a];
        }
    }

  private:
    std::vector<std::size_t> parent; ///< Parent links for each union-find node.
    std::vector<std::size_t> rank;   ///< Upper-bound tree heights used for balanced unions.
};

/// \brief Builder metadata for one directed DCEL half edge.
struct DCELBuilderHalfEdge {
    /// \brief Initialize metadata for an unassigned builder half-edge.
    DCELBuilderHalfEdge() : boundary_ring(DCEL::npos) {}

    std::size_t boundary_ring; ///< Index of the boundary ring containing this half-edge.
};

/// \brief Builder metadata for one directed DCEL boundary ring.
struct DCELBuilderRing {
    std::vector<std::size_t> half_edges; ///< Ordered half-edge indices around the ring.
};

#ifndef NDEBUG
void debugPrintHalfEdgeList(const std::vector<std::size_t>& half_edges) {
    for (std::size_t half_edge : half_edges) {
        std::cerr << ' ' << half_edge;
    }
}
#endif

DCEL::FaceParity oppositeFaceParity(DCEL::FaceParity parity) {
    assert(parity != DCEL::FaceParity::Unknown);

    if (parity == DCEL::FaceParity::Exterior) {
        return DCEL::FaceParity::Interior;
    }
    return DCEL::FaceParity::Exterior;
}

#ifndef NDEBUG
std::string faceParityName(DCEL::FaceParity parity) {
    switch (parity) {
    case DCEL::FaceParity::Unknown:
        return "Unknown";
    case DCEL::FaceParity::Exterior:
        return "Exterior";
    case DCEL::FaceParity::Interior:
        return "Interior";
    }

    return "<invalid>";
}

void dumpFaceBoundary(const DCEL& dcel, std::size_t face_index) {
    const DCEL::Face& face = dcel.face(face_index);
    auto dump_component = [&](const char* label, std::size_t component) {
        std::cerr << "  " << label << " component " << component << ":\n";
        if (component == DCEL::npos) {
            std::cerr << "    <none>\n";
            return;
        }

        std::size_t curr_half_edge = component;
        do {
            const DCEL::HalfEdge& half_edge = dcel.halfEdge(curr_half_edge);
            std::cerr << "    edge " << curr_half_edge << " face=" << half_edge.face
                      << " twin=" << half_edge.twin << " next=" << half_edge.next
                      << " segment=" << dcel.segmentOf(half_edge).toString() << "\n";
            curr_half_edge = half_edge.next;
        } while (curr_half_edge != component);
    };

    dump_component("outer", face.outer_component);
    for (std::size_t inner_component : face.inner_components) {
        dump_component("inner", inner_component);
    }
}

void dumpFaceParityConflict(const DCEL& dcel, const std::vector<DCEL::FaceParity>& face_parities,
                            std::size_t face_index, std::size_t adjacent_face,
                            std::size_t half_edge_index, DCEL::FaceParity expected_parity) {
    const DCEL::HalfEdge& half_edge = dcel.halfEdge(half_edge_index);
    const DCEL::HalfEdge& twin_half_edge = dcel.twinOf(half_edge);
    std::cerr << "[dcel] face parity conflict\n";
    std::cerr << "  current_face=" << face_index
              << " parity=" << faceParityName(face_parities[face_index]) << "\n";
    std::cerr << "  adjacent_face=" << adjacent_face
              << " existing_parity=" << faceParityName(face_parities[adjacent_face])
              << " expected_parity=" << faceParityName(expected_parity) << "\n";
    std::cerr << "  crossing_edge=" << half_edge_index
              << " segment=" << dcel.segmentOf(half_edge).toString() << "\n";
    std::cerr << "  twin_edge=" << half_edge.twin
              << " segment=" << dcel.segmentOf(twin_half_edge).toString() << "\n";
    std::cerr << "  current face boundary:\n";
    dumpFaceBoundary(dcel, face_index);
    std::cerr << "  adjacent face boundary:\n";
    dumpFaceBoundary(dcel, adjacent_face);
}
#endif

} // namespace

/// \brief Incremental builder for constructing a DCEL from polygon boundaries.
class DCEL::Creator {
  public:
    /// \brief Initialize a builder around an empty DCEL supplied by a factory.
    /// \param dcel The empty DCEL to populate.
    explicit Creator(DCEL dcel);

    /// \brief Build a DCEL from segments.
    /// \param rings The segments used to build DCEL.
    /// \returns A populated DCEL.
    DCEL build(const std::vector<Segment>& segments);

  private:
    using OutgoingHalfEdges = std::unordered_map<std::size_t, std::vector<std::size_t>>;

    DCEL dcel;                                        ///< DCEL being incrementally constructed.
    std::unordered_map<Point, std::size_t> point_map; ///< Maps geometric points to vertex indices.
    std::unordered_map<Segment, std::size_t> segment_map; ///< Maps directed segments to half-edges.
    std::vector<DCELBuilderHalfEdge> half_edges;          ///< Builder metadata for half-edges.
    std::vector<DCELBuilderRing> boundary_rings;          ///< Builder metadata for directed rings.
    UnionFind ring_union_find; ///< Groups boundary rings that bound the same face.

    /// \brief Reserve storage for vertices and half-edges.
    /// \param polygons The polygons that will be inserted.
    void reserve(const std::vector<const LinearRing*>& rings);

    /// \brief Collect outgoing half-edges by origin vertex.
    /// \returns A map from vertex index to outgoing half-edge indices.
    OutgoingHalfEdges collectOutgoingHalfEdges() const;

    /// \brief Sort each vertex's outgoing half-edges by counterclockwise angle.
    /// \param outgoing_half_edges Outgoing half-edge adjacency lists to sort in-place.
    void sortOutgoingHalfEdges(OutgoingHalfEdges& outgoing_half_edges) const;

    /// \brief Link half-edge next/prev pointers using sorted outgoing angular order.
    /// \param outgoing_half_edges Counterclockwise-sorted outgoing half-edges by origin vertex.
    void linkHalfEdges(const OutgoingHalfEdges& outgoing_half_edges);

    /// \brief Walk linked half-edges into closed boundary rings.
    void assembleBoundaryRings();

    /// \brief Register builder metadata for a linked boundary ring.
    /// \param half_edge_indices Ordered half-edge indices around the boundary ring.
    void addBoundaryRing(const std::vector<std::size_t>& half_edge_indices);

    /// \brief Convert builder ring half-edge indices into a geometric ring.
    /// \param boundary_ring The builder boundary ring to convert.
    /// \returns A linear ring with the half-edge origins in boundary order.
    LinearRing linearRingOf(const DCELBuilderRing& boundary_ring) const;

    /// \brief Return true when a builder ring is an outer face boundary.
    /// \param boundary_ring The builder boundary ring to inspect.
    /// \returns True if the derived geometric ring is counter-clockwise.
    bool isOuter(const DCELBuilderRing& boundary_ring) const;

    /// \brief Create DCEL face records from the boundary-ring connectivity graph.
    void createFaces();

    /// \brief Union boundary rings that bound the same face.
    /// \param unbounded_ring_index The union-find node for the imaginary unbounded boundary.
    void groupBoundaryRingsByFace(std::size_t unbounded_ring_index);

    /// \brief Create the initial mapping from ring roots to face indices.
    /// \param unbounded_ring_index The union-find node for the imaginary unbounded boundary.
    /// \returns A map from union-find roots to DCEL face indices, seeded with the unbounded face.
    std::unordered_map<std::size_t, std::size_t>
    createFaceByRingRoot(std::size_t unbounded_ring_index);

    /// \brief Resolve or create the DCEL face for a ring-root group.
    /// \param root_ring_index The union-find root representing a face group.
    /// \param root_ring_to_face Map from ring roots to existing face indices.
    /// \returns The DCEL face index for the ring-root group.
    std::size_t getOrCreateFace(std::size_t root_ring_index,
                                std::unordered_map<std::size_t, std::size_t>& root_ring_to_face);

    /// \brief Attach a boundary ring representative to a DCEL face.
    /// \param boundary_ring The boundary ring metadata to attach.
    /// \param face_index The face receiving the boundary component.
    void addBoundaryRingToFace(const DCELBuilderRing& boundary_ring, std::size_t face_index);

    /// \brief Assign a face index to every half-edge in a boundary ring.
    /// \param boundary_ring The boundary ring whose half-edges should be assigned.
    /// \param face_index The face incident to the boundary ring.
    void assignFaceToBoundaryRing(const DCELBuilderRing& boundary_ring, std::size_t face_index);

    /// \brief Find the half-edge whose origin is the leftmost vertex in a ring.
    /// \param half_edge_indices Ordered half-edge indices for the ring.
    /// \returns The half-edge index with the lexicographically smallest origin.
    std::size_t leftmostHalfEdge(const std::vector<std::size_t>& half_edge_indices) const;

    /// \brief Find the half-edge whose segment intersects the horizontal ray to the left of a point
    /// and has the rightmost intersection among all such half-edges. Returns DCEL::npos if no such
    /// half-edge exists.
    /// \param point The point whose left ray we are intersecting with the half-edges.
    /// \returns The half-edge with the rightmost intersection with the horizontal ray to the left
    /// of
    /// point, or DCEL::npos if no half-edge intersects that ray.
    std::size_t findNearestHalfEdgeToLeft(const Point& point);

    /// \brief Return an existing half-edge pair for a segment or create a new pair.
    /// \param segment The directed segment for the requested half-edge.
    /// \returns The half-edge index whose segment matches segment.
    std::size_t getOrCreateHalfEdgePair(const Segment& segment);

    /// \brief Return an existing point index or create a new DCEL vertex.
    /// \param point The geometric point to look up.
    /// \returns The DCEL vertex index for point.
    std::size_t getOrCreatePoint(const Point& point);

    /// \brief Create one unlinked half-edge for a directed segment.
    /// \param segment The directed segment whose start point becomes the origin.
    /// \returns The index of the newly created half-edge.
    std::size_t createHalfEdge(const Segment& segment);
};

DCEL::Creator::Creator(DCEL dcel) : dcel(std::move(dcel)) {}

DCEL::Creator::OutgoingHalfEdges DCEL::Creator::collectOutgoingHalfEdges() const {
    OutgoingHalfEdges origin_to_edge;
    for (std::size_t edge_index = 0; edge_index < dcel.half_edges.size(); edge_index++) {
        const DCEL::HalfEdge& half_edge = dcel.half_edges[edge_index];
        origin_to_edge[half_edge.origin].push_back(edge_index);
    }

    return origin_to_edge;
}

void DCEL::Creator::sortOutgoingHalfEdges(OutgoingHalfEdges& outgoing_half_edges) const {
    for (auto& [vertex, edges] : outgoing_half_edges) {
        std::sort(edges.begin(), edges.end(), [&](std::size_t lhs, std::size_t rhs) {
            const auto& lhs_edge = dcel.half_edges[lhs];
            const auto& rhs_edge = dcel.half_edges[rhs];

            const Point& origin = dcel.originOf(lhs_edge);
            const Point& lhs_dest = dcel.originOf(dcel.twinOf(lhs_edge));
            const Point& rhs_dest = dcel.originOf(dcel.twinOf(rhs_edge));

            Vect2 lhs_vec = lhs_dest - origin;
            Vect2 rhs_vec = rhs_dest - origin;

            return angleComparator(lhs_vec, rhs_vec);
        });
    }
}

void DCEL::Creator::linkHalfEdges(const OutgoingHalfEdges& outgoing_half_edges) {
#ifndef NDEBUG
    std::cerr << "[dcel] linkHalfEdges: half_edges=" << dcel.half_edges.size()
              << " vertices=" << outgoing_half_edges.size() << '\n';
#endif

    for (std::size_t edge = 0; edge < dcel.half_edges.size(); edge++) {
        std::size_t twin = dcel.half_edges[edge].twin;
        std::size_t twin_origin = dcel.half_edges[twin].origin;

        // At e's destination, use the twin to find the incoming direction index i
        const auto& edges = outgoing_half_edges.at(twin_origin);
        auto it = std::find(edges.begin(), edges.end(), twin);
        assert(it != edges.end());
        std::size_t i = std::distance(edges.begin(), it);

        // Then step to the previous outgoing edge in CCW order (next edge clockwise)
        // That keeps the face consistently on one side of the traversal.
        std::size_t next_i = (i + edges.size() - 1) % edges.size();
        std::size_t next = edges[next_i];

        dcel.half_edges[edge].next = next;
        dcel.half_edges[next].prev = edge;

#ifndef NDEBUG
        std::cerr << "[dcel]   edge " << edge << " "
                  << dcel.segmentOf(dcel.half_edges[edge]).toString() << " twin=" << twin
                  << " dest_vertex=" << twin_origin << " incoming_slot=" << i
                  << " next_slot=" << next_i << " next=" << next << '\n';
#endif
    }
}

void DCEL::Creator::assembleBoundaryRings() {
#ifndef NDEBUG
    std::cerr << "[dcel] assembleBoundaryRings\n";
#endif

    std::unordered_set<std::size_t> visited;

    for (std::size_t edge = 0; edge < dcel.half_edges.size(); edge++) {
        if (visited.contains(edge)) {
            continue;
        }

        std::size_t first_edge = edge;
        std::size_t curr_edge = first_edge;
        std::vector<std::size_t> curr_ring_edges;

        do {
            if (visited.contains(curr_edge)) {
                throw std::logic_error("Boundary ring reached an already visited half-edge");
            }
            visited.insert(curr_edge);
            curr_ring_edges.push_back(curr_edge);

#ifndef NDEBUG
            std::cerr << "[dcel]   walk edge " << curr_edge << " -> "
                      << dcel.halfEdge(curr_edge).next << '\n';
#endif

            curr_edge = dcel.halfEdge(curr_edge).next;
        } while (curr_edge != first_edge);

        addBoundaryRing(curr_ring_edges);
    }
}

DCEL DCEL::Creator::build(const std::vector<Segment>& segments) {
    std::vector<Segment> planarized = planarizeSegments(segments);

    for (const Segment& segment : planarized) {
        getOrCreateHalfEdgePair(segment);
    }

    OutgoingHalfEdges outgoing_half_edges = collectOutgoingHalfEdges();
    sortOutgoingHalfEdges(outgoing_half_edges);

    linkHalfEdges(outgoing_half_edges);

    assembleBoundaryRings();

    createFaces();

    return std::move(dcel);
}

std::size_t DCEL::Creator::findNearestHalfEdgeToLeft(const Point& point) {
    std::size_t nearest_left_half_edge = DCEL::npos;
    std::optional<Point> nearest_intersection;

    for (std::size_t half_edge_index = 0; half_edge_index < dcel.half_edges.size();
         ++half_edge_index) {
        const DCEL::HalfEdge& half_edge = dcel.half_edges[half_edge_index];
        if (half_edge_index > half_edge.twin) {
            // Test each geometric edge once, then choose the directed twin whose
            // incident face is on the same side as point.
            continue;
        }

        Segment half_edge_segment = dcel.segmentOf(half_edge);
        if (half_edge_segment.start.y == point.y && half_edge_segment.end.y == point.y) {
            // A horizontal edge ending at the query point is the nearest possible left hit. Use
            // the opposite direction so point-touching exterior rings union with each other.
            const DCEL::HalfEdge& twin_half_edge = dcel.twinOf(half_edge);
            const Point& half_edge_origin = dcel.originOf(half_edge);
            const Point& twin_origin = dcel.originOf(twin_half_edge);

            if (half_edge_origin == point && twin_origin.x < point.x) {
                return half_edge.twin;
            }
            if (twin_origin == point && half_edge_origin.x < point.x) {
                return half_edge_index;
            }
        }

        std::optional<Point> intersection = leftRayIntersection(half_edge_segment, point);
        if (!intersection) {
            // This half edge does not intersect the hole's leftmost point, so we can skip it.
            continue;
        }

        // Collinear hits lie on the same horizontal line as the query point and do not identify
        // which incident face is on the same side as the point.
        Rational point_orientation =
            orientation(half_edge_segment.start, half_edge_segment.end, point);
        if (point_orientation == 0) {
            continue;
        }

        if (!nearest_intersection || intersection->x > nearest_intersection->x) {
            nearest_intersection = intersection;
            // We want the half-edge whose incident face is on the same side as point.
            // We can determine this by checking the orientation of the half-edge segment with
            // respect to point.
            if (point_orientation < 0) {
                nearest_left_half_edge = half_edge.twin;
            } else if (point_orientation > 0) {
                nearest_left_half_edge = half_edge_index;
            }
        }
    }
    return nearest_left_half_edge;
}

void DCEL::Creator::createFaces() {
#ifndef NDEBUG
    std::cerr << "[dcel] createFaces: rings=" << boundary_rings.size() << '\n';
#endif

    const std::size_t unbounded_ring_index = ring_union_find.addSet();

    groupBoundaryRingsByFace(unbounded_ring_index);

    std::unordered_map<std::size_t, std::size_t> root_ring_to_face =
        createFaceByRingRoot(unbounded_ring_index);

    for (std::size_t ring_index = 0; ring_index < boundary_rings.size(); ++ring_index) {
        const DCELBuilderRing& boundary_ring = boundary_rings[ring_index];
        const std::size_t root_ring_index = ring_union_find.find(ring_index);
        const std::size_t face_index = getOrCreateFace(root_ring_index, root_ring_to_face);

#ifndef NDEBUG
        std::cerr << "[dcel]   assign ring " << ring_index << " root=" << root_ring_index
                  << " face=" << face_index << '\n';
#endif

        addBoundaryRingToFace(boundary_ring, face_index);
    }

#ifndef NDEBUG
    for (std::size_t face_index = 0; face_index < dcel.faces.size(); ++face_index) {
        const DCEL::Face& face = dcel.faces[face_index];
        std::cerr << "[dcel]   face " << face_index << " outer=" << face.outer_component
                  << " inners=";
        debugPrintHalfEdgeList(face.inner_components);
        std::cerr << '\n';
    }
#endif
}

void DCEL::Creator::groupBoundaryRingsByFace(const std::size_t unbounded_ring_index) {
    // There is an arc between two rings if and only if one of the rings is the boundary of a hole
    // and the other ring has a half-edge immediately to the left of the leftmost vertex of that
    // hole ring.
    for (std::size_t ring_index = 0; ring_index < boundary_rings.size(); ++ring_index) {
        const DCELBuilderRing& boundary_ring = boundary_rings[ring_index];
        if (isOuter(boundary_ring)) {
            continue;
        }

        std::size_t leftmost_half_edge_index = leftmostHalfEdge(boundary_ring.half_edges);
        const DCEL::HalfEdge& leftmost_half_edge = dcel.half_edges[leftmost_half_edge_index];
        const Point& leftmost_point = dcel.originOf(leftmost_half_edge);
        std::size_t nearest_left_half_edge = findNearestHalfEdgeToLeft(leftmost_point);

        if (nearest_left_half_edge == DCEL::npos) {
#ifndef NDEBUG
            std::cerr << "[dcel]   group ring " << ring_index
                      << " with unbounded face: leftmost=" << leftmost_point.toString() << '\n';
#endif
            ring_union_find.unite(ring_index, unbounded_ring_index);
        } else {
            std::size_t nearest_left_ring = half_edges[nearest_left_half_edge].boundary_ring;
            assert(nearest_left_ring != DCEL::npos);
            assert(nearest_left_ring != ring_index);
#ifndef NDEBUG
            std::cerr << "[dcel]   group ring " << ring_index << " with ring " << nearest_left_ring
                      << ": leftmost=" << leftmost_point.toString()
                      << " nearest_left_edge=" << nearest_left_half_edge << ' '
                      << dcel.segmentOf(dcel.half_edges[nearest_left_half_edge]).toString() << '\n';
#endif
            ring_union_find.unite(ring_index, nearest_left_ring);
        }
    }
}

std::unordered_map<std::size_t, std::size_t>
DCEL::Creator::createFaceByRingRoot(const std::size_t unbounded_ring_index) {
    // Group rings into faces based on which rings are connected by unions.
    std::unordered_map<std::size_t, std::size_t> root_ring_to_face;

    // First add the unbounded face
    assert(dcel.faces.size() == DCEL::unbounded_face_index);
    dcel.faces.emplace_back(DCEL::Face()); // Start with the unbounded face.
    dcel.faces[DCEL::unbounded_face_index].outer_component = DCEL::npos;
    root_ring_to_face[ring_union_find.find(unbounded_ring_index)] = DCEL::unbounded_face_index;

    return root_ring_to_face;
}

std::size_t
DCEL::Creator::getOrCreateFace(const std::size_t root_ring_index,
                               std::unordered_map<std::size_t, std::size_t>& root_ring_to_face) {
    auto found = root_ring_to_face.find(root_ring_index);
    if (found != root_ring_to_face.end()) {
        return found->second;
    }

    const std::size_t face_index = dcel.faces.size();
    dcel.faces.emplace_back(DCEL::Face());
    root_ring_to_face[root_ring_index] = face_index;
    return face_index;
}

void DCEL::Creator::addBoundaryRingToFace(const DCELBuilderRing& boundary_ring,
                                          const std::size_t face_index) {

    // Assign this face to every half-edge around the boundary component.
    assignFaceToBoundaryRing(boundary_ring, face_index);

    // Store one representative half-edge for this boundary component.
    const std::size_t half_edge_index = leftmostHalfEdge(boundary_ring.half_edges);
    if (isOuter(boundary_ring)) {
        assert(dcel.faces[face_index].outer_component == DCEL::npos);
        dcel.faces[face_index].outer_component = half_edge_index;
#ifndef NDEBUG
        std::cerr << "[dcel]     face " << face_index << " outer_component=" << half_edge_index
                  << '\n';
#endif
    } else {
        dcel.faces[face_index].inner_components.push_back(half_edge_index);
#ifndef NDEBUG
        std::cerr << "[dcel]     face " << face_index << " inner_component=" << half_edge_index
                  << '\n';
#endif
    }
}

void DCEL::Creator::assignFaceToBoundaryRing(const DCELBuilderRing& boundary_ring,
                                             const std::size_t face_index) {
    for (const std::size_t half_edge_index : boundary_ring.half_edges) {
        dcel.half_edges[half_edge_index].face = face_index;
    }
}

void DCEL::Creator::reserve(const std::vector<const LinearRing*>& rings) {
    std::size_t segment_count = 0;
    std::size_t ring_count = 0;
    for (const LinearRing* ring : rings) {
        segment_count += ring->points.size();
        ring_count++;
    }

    dcel.points.reserve(segment_count * 2);
    dcel.half_edges.reserve(segment_count * 2);
    half_edges.reserve(segment_count * 2);
    boundary_rings.reserve(ring_count * 2);
}

std::size_t DCEL::Creator::getOrCreatePoint(const Point& point) {
    const auto found = point_map.find(point);
    if (found != point_map.end()) {
        return found->second;
    }

    dcel.points.emplace_back(DCEL::DCELPoint(point));
    const std::size_t point_index = dcel.points.size() - 1;
    point_map[point] = point_index;
    return point_index;
}

std::size_t DCEL::Creator::createHalfEdge(const Segment& segment) {
    const std::size_t start_index = getOrCreatePoint(segment.start);

    dcel.half_edges.emplace_back(DCEL::HalfEdge(start_index));

    // Initialize the half-edge ring index to npos to indicate that it is not yet part of any
    // ring. We will set this to the correct ring index in addRing
    half_edges.push_back(DCELBuilderHalfEdge());

    const std::size_t half_edge_index = dcel.half_edges.size() - 1;
    if (dcel.points[start_index].half_edge == DCEL::npos) {
        dcel.points[start_index].half_edge = half_edge_index;
    }
    return half_edge_index;
}

void DCEL::Creator::addBoundaryRing(const std::vector<std::size_t>& half_edge_indices) {
    const std::size_t ring_index = boundary_rings.size();
    boundary_rings.push_back({half_edge_indices});
    ring_union_find.addSet();

    for (const std::size_t ring_half_edge_index : half_edge_indices) {
        half_edges[ring_half_edge_index].boundary_ring = ring_index;
    }

#ifndef NDEBUG
    const LinearRing ring = linearRingOf(boundary_rings[ring_index]);
    std::cerr << "[dcel]   ring " << ring_index << " edges=";
    debugPrintHalfEdgeList(half_edge_indices);
    std::cerr << " area=" << ring.signedArea() << " outer=" << ring.isOuter() << " points=";
    for (const Point& point : ring.points) {
        std::cerr << ' ' << point.toString();
    }
    std::cerr << '\n';
#endif
}

LinearRing DCEL::Creator::linearRingOf(const DCELBuilderRing& boundary_ring) const {
    std::vector<Point> points;
    points.reserve(boundary_ring.half_edges.size());

    for (const std::size_t half_edge_index : boundary_ring.half_edges) {
        const DCEL::HalfEdge& half_edge = dcel.half_edges[half_edge_index];
        points.push_back(static_cast<Point>(dcel.originOf(half_edge)));
    }

    return LinearRing(std::move(points));
}

bool DCEL::Creator::isOuter(const DCELBuilderRing& boundary_ring) const {
    return linearRingOf(boundary_ring).isOuter();
}

std::size_t
DCEL::Creator::leftmostHalfEdge(const std::vector<std::size_t>& half_edge_indices) const {
    assert(!half_edge_indices.empty());

    std::size_t leftmost_half_edge = half_edge_indices.front();
    for (const std::size_t half_edge_index : half_edge_indices) {
        const DCEL::HalfEdge& half_edge = dcel.half_edges[half_edge_index];
        if (dcel.originOf(half_edge) < dcel.originOf(dcel.half_edges[leftmost_half_edge])) {
            leftmost_half_edge = half_edge_index;
        }
    }

    return leftmost_half_edge;
}

std::size_t DCEL::Creator::getOrCreateHalfEdgePair(const Segment& segment) {
    const Segment reversed_segment = segment.reversed();
    const auto found = segment_map.find(segment);
    if (found != segment_map.end()) {
        assert(segment_map.contains(reversed_segment));
        return found->second;
    }

    const std::size_t half_edge_index = createHalfEdge(segment);
    const std::size_t twin_half_edge_index = createHalfEdge(reversed_segment);

    DCEL::HalfEdge& half_edge = dcel.half_edges[half_edge_index];
    DCEL::HalfEdge& twin_half_edge = dcel.half_edges[twin_half_edge_index];

    half_edge.twin = twin_half_edge_index;
    twin_half_edge.twin = half_edge_index;

    segment_map[segment] = half_edge_index;
    segment_map[reversed_segment] = twin_half_edge_index;

    return half_edge_index;
}

DCEL::DCELPoint::DCELPoint(Rational x, Rational y) : Point(x, y), half_edge(DCEL::npos) {}

DCEL::DCELPoint::DCELPoint(const Point& p) : Point(p), half_edge(DCEL::npos) {}

DCEL::HalfEdge::HalfEdge(std::size_t origin)
    : origin(origin), twin(DCEL::npos), next(DCEL::npos), prev(DCEL::npos), face(DCEL::npos) {}

DCEL::Face::Face() : outer_component(DCEL::npos), inner_components() {}

std::size_t DCEL::pointCount() const {
    return points.size();
}

std::size_t DCEL::halfEdgeCount() const {
    return half_edges.size();
}

std::size_t DCEL::faceCount() const {
    return faces.size();
}

const DCEL::DCELPoint& DCEL::point(std::size_t index) const {
    assert(index < points.size());
    return points[index];
}

const DCEL::HalfEdge& DCEL::halfEdge(std::size_t index) const {
    assert(index < half_edges.size());
    return half_edges[index];
}

const DCEL::Face& DCEL::face(std::size_t index) const {
    assert(index < faces.size());
    return faces[index];
}

DCEL::DCELPoint& DCEL::originOf(const DCEL::HalfEdge& half_edge) {
    assert(half_edge.origin != npos);
    return points[half_edge.origin];
}

const DCEL::DCELPoint& DCEL::originOf(const DCEL::HalfEdge& half_edge) const {
    assert(half_edge.origin != npos);
    return points[half_edge.origin];
}

DCEL::HalfEdge& DCEL::twinOf(const DCEL::HalfEdge& half_edge) {
    assert(half_edge.twin != npos);
    return half_edges[half_edge.twin];
}

const DCEL::HalfEdge& DCEL::twinOf(const DCEL::HalfEdge& half_edge) const {
    assert(half_edge.twin != npos);
    return half_edges[half_edge.twin];
}

DCEL::HalfEdge& DCEL::nextOf(const DCEL::HalfEdge& half_edge) {
    assert(half_edge.next != npos);
    return half_edges[half_edge.next];
}

const DCEL::HalfEdge& DCEL::nextOf(const DCEL::HalfEdge& half_edge) const {
    assert(half_edge.next != npos);
    return half_edges[half_edge.next];
}

DCEL::HalfEdge& DCEL::prevOf(const DCEL::HalfEdge& half_edge) {
    assert(half_edge.prev != npos);
    return half_edges[half_edge.prev];
}

const DCEL::HalfEdge& DCEL::prevOf(const DCEL::HalfEdge& half_edge) const {
    assert(half_edge.prev != npos);
    return half_edges[half_edge.prev];
}

DCEL::Face& DCEL::faceOf(const DCEL::HalfEdge& half_edge) {
    assert(half_edge.face != npos);
    return faces[half_edge.face];
}

const DCEL::Face& DCEL::faceOf(const DCEL::HalfEdge& half_edge) const {
    assert(half_edge.face != npos);
    return faces[half_edge.face];
}

Segment DCEL::segmentOf(const DCEL::HalfEdge& half_edge) const {
    return Segment(originOf(half_edge), originOf(twinOf(half_edge)));
}

LinearRing DCEL::ringOf(const HalfEdge& half_edge) const {
    std::vector<Point> points;
    const HalfEdge* current_half_edge = &half_edge;
    do {
        points.push_back(originOf(*current_half_edge));
        current_half_edge = &nextOf(*current_half_edge);
    } while (current_half_edge != &half_edge);
    return LinearRing(std::move(points));
}

std::optional<Polygon> DCEL::polygonOf(const Face& face) const {
    if (face.outer_component == npos) {
        // Unbounded face is not a finite polygon, so forcing it into Polygon would be semantically
        // wrong.
        return std::nullopt;
    }

    const HalfEdge& outer_half_edge = half_edges[face.outer_component];
    LinearRing outer_ring = ringOf(outer_half_edge);

    std::vector<LinearRing> inner_rings;
    for (const std::size_t inner_component : face.inner_components) {
        const HalfEdge& inner_half_edge = half_edges[inner_component];
        inner_rings.push_back(ringOf(inner_half_edge));
    }

    return Polygon(std::move(outer_ring), std::move(inner_rings));
}

DCEL DCEL::fromSegments(const std::vector<Segment>& segments) {
    return DCEL::Creator(DCEL()).build(segments);
}

const DCEL::Face& DCEL::unboundedFace() const {
    return faces[unbounded_face_index];
}

std::vector<DCEL::FaceParity> DCEL::faceParities() const {
    std::vector<FaceParity> face_parities(faces.size(), FaceParity::Unknown);
    face_parities[unbounded_face_index] = FaceParity::Exterior;

    // Traverse the face-adjacency graph from the unbounded face using BFS.
    // Crossing a boundary through a half-edge twin flips the fill parity.
    std::queue<std::size_t> queue;
    queue.push(unbounded_face_index);

    while (!queue.empty()) {
        std::size_t count = queue.size();
        for (std::size_t i = 0; i < count; i++) {
            std::size_t face_index = queue.front();
            queue.pop();

            const Face& face = faces[face_index];
            const FaceParity adjacent_parity = oppositeFaceParity(face_parities[face_index]);

            auto visit_boundary_component = [&](std::size_t component) {
                std::size_t curr_half_edge = component;
                do {
                    const HalfEdge& half_edge = half_edges[curr_half_edge];
                    const HalfEdge& twin_half_edge = twinOf(half_edge);
                    const std::size_t adjacent_face = twin_half_edge.face;
                    assert(adjacent_face != npos);

                    if (face_parities[adjacent_face] == FaceParity::Unknown) {
                        face_parities[adjacent_face] = adjacent_parity;
                        queue.push(adjacent_face);
                    } else {
#ifndef NDEBUG
                        if (face_parities[adjacent_face] != adjacent_parity) {
                            dumpFaceParityConflict(*this, face_parities, face_index, adjacent_face,
                                                   curr_half_edge, adjacent_parity);
                        }
#endif
                        assert(face_parities[adjacent_face] == adjacent_parity);
                    }

                    curr_half_edge = half_edge.next;
                } while (curr_half_edge != component);
            };

            if (face.outer_component != DCEL::npos) {
                visit_boundary_component(face.outer_component);
            }
            for (std::size_t component : face.inner_components) {
                visit_boundary_component(component);
            }
        }
    }

    return face_parities;
}
