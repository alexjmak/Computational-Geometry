#include "geometry/dcel.hpp"
#include "algorithms/assemble.hpp"
#include "geometry/intersection.hpp"
#include "geometry/polygon.hpp"
#include "geometry/predicates.hpp"
#include "geometry/segment.hpp"
#include <cassert>
#include <cfloat>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

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
    DCELBuilderHalfEdge() : boundary_ring(DCEL::npos), is_input_ring_edge(false) {}

    std::size_t boundary_ring; ///< Index of the boundary ring containing this half-edge.
    bool is_input_ring_edge;   ///< True if this half-edge belongs to an input ring.
};

/// \brief Builder metadata for one directed DCEL boundary ring.
struct DCELBuilderRing {

    std::size_t leftmost_half_edge; ///< Representative half-edge at the ring's leftmost vertex.
    bool is_outer;                  ///< True when the ring is an outer face boundary.
};

} // namespace

/// \brief Incremental builder for constructing a DCEL from polygon boundaries.
class DCEL::Creator {
  public:
    /// \brief Initialize a builder around an empty DCEL supplied by a factory.
    /// \param dcel The empty DCEL to populate.
    explicit Creator(DCEL dcel);

    /// \brief Build a DCEL from polygon boundary rings.
    /// \param rings The rings whose boundaries should be inserted.
    /// \returns A populated DCEL.
    DCEL build(const std::vector<const LinearRing*>& rings);

  private:
    DCEL dcel;                                        ///< DCEL being incrementally constructed.
    std::unordered_map<Point, std::size_t> point_map; ///< Maps geometric points to vertex indices.
    std::unordered_map<Segment, std::size_t> segment_map; ///< Maps directed segments to half-edges.
    std::vector<DCELBuilderHalfEdge> half_edges;          ///< Builder metadata for half-edges.
    std::vector<DCELBuilderRing> boundary_rings;          ///< Builder metadata for directed rings.
    UnionFind ring_union_find; ///< Groups boundary rings that bound the same face.

    /// \brief Reserve storage for vertices and half-edges.
    /// \param polygons The polygons that will be inserted.
    void reserve(const std::vector<const LinearRing*>& rings);

    /// \brief Add a boundary ring.
    /// \param ring The boundary ring to add.
    /// \param is_input_ring True if the ring came from caller input.
    void processBoundaryRing(const LinearRing& ring, bool is_input_ring);

    /// \brief Assemble directed half-edges into closed non-input boundary rings.
    /// \param edges Half-edge indices that are not part of an input boundary ring.
    /// \returns Closed rings formed by walking the supplied half-edges.
    std::vector<LinearRing> assembleBoundaryRing(const std::vector<std::size_t>& edges);

    /// \brief Register builder metadata for a linked boundary ring.
    /// \param half_edge_indices Ordered half-edge indices around the boundary ring.
    /// \param isOuter True if the ring is counter-clockwise and bounds a finite face.
    /// \returns The builder boundary-ring index.
    std::size_t addBoundaryRing(const std::vector<std::size_t>& half_edge_indices, bool isOuter);

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

    /// \brief Link the next/previous pointers for a directed boundary ring.
    /// \param half_edge_indices Ordered half-edge indices for the directed ring.
    void linkBoundaryRingHalfEdges(const std::vector<std::size_t>& half_edge_indices);

    /// \brief Find the half-edge whose origin is the leftmost vertex in a ring.
    /// \param half_edge_indices Ordered half-edge indices for the ring.
    /// \returns The half-edge index with the lexicographically smallest origin.
    std::size_t leftmostHalfEdge(const std::vector<std::size_t>& half_edge_indices);

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

    /// \brief Choose the next non-input half-edge while assembling a boundary ring.
    /// \param prev_edge_idx The half-edge used to reach the current vertex, or DCEL::npos if the
    /// walk is starting.
    /// \param next_edges Candidate outgoing half-edges from the current vertex.
    /// \param visited Set of half-edges already consumed by assembled boundary rings.
    /// \returns The selected outgoing half-edge, or DCEL::npos if no unvisited candidate remains.
    std::size_t nextNonInputBoundaryEdge(std::size_t prev_edge_idx,
                                         const std::vector<std::size_t>& next_edges,
                                         std::unordered_set<std::size_t>& visited);
};

DCEL::Creator::Creator(DCEL dcel) : dcel(std::move(dcel)) {}

std::size_t DCEL::Creator::nextNonInputBoundaryEdge(std::size_t prev_edge_idx,
                                                  const std::vector<std::size_t>& next_edges,
                                                  std::unordered_set<std::size_t>& visited) {
    if (next_edges.size() == 0) {
        return DCEL::npos;
    }

    std::size_t next_edge = DCEL::npos;

    if (prev_edge_idx != DCEL::npos && next_edges.size() > 1) {
        DCEL::HalfEdge prev_edge = dcel.half_edges[prev_edge_idx];
        DCEL::HalfEdge prev_twin_edge = dcel.twinOf(prev_edge);
        Vect2 prev_reversed = dcel.originOf(prev_edge) - dcel.originOf(prev_twin_edge);

        DCELBuilderHalfEdge prev_twin_builder_edge = half_edges[prev_edge.twin];
        DCELBuilderRing prev_twin_ring = boundary_rings[prev_twin_builder_edge.boundary_ring];
        bool choose_clockwise_turn = !prev_twin_ring.is_outer;

        double best_edge_angle = std::numeric_limits<double>::max();
        for (const std::size_t half_edge_index : next_edges) {
            if (visited.contains(half_edge_index)) {
                continue;
            }

            DCEL::HalfEdge candidate_edge = dcel.half_edges[half_edge_index];
            DCEL::HalfEdge candidate_twin_edge = dcel.twinOf(candidate_edge);
            Vect2 candidate = dcel.originOf(candidate_twin_edge) - dcel.originOf(candidate_edge);

            // Pick the non-visited candidate edge with the smallest angle
            double angle;
            if (choose_clockwise_turn) {
                angle = cwAngle(prev_reversed, candidate);
            } else {
                angle = ccwAngle(prev_reversed, candidate);
            }

            if (angle < best_edge_angle) {
                best_edge_angle = angle;
                next_edge = half_edge_index;
            }
        }
    } else {
        // Take the first unvisited edge
        next_edge = DCEL::npos;
        for (const std::size_t half_edge_index : next_edges) {
            if (visited.contains(half_edge_index)) {
                continue;
            }
            next_edge = half_edge_index;
            break;
        }
    }

    if (next_edge != DCEL::npos) {
        visited.insert(next_edge);
    }

    return next_edge;
}

std::vector<LinearRing> DCEL::Creator::assembleBoundaryRing(const std::vector<std::size_t>& edges) {
    std::unordered_map<std::size_t, std::vector<std::size_t>> origin_to_edge;
    for (const std::size_t edge_index : edges) {
        DCEL::HalfEdge half_edge = dcel.half_edges[edge_index];

        origin_to_edge[half_edge.origin].push_back(edge_index);
    }

    std::unordered_set<std::size_t> visited;

    std::vector<LinearRing> rings;
    std::vector<Point> curr_ring_pts;

    for (const std::size_t half_edge_index : edges) {
        if (visited.contains(half_edge_index)) {
            continue;
        }

        std::size_t first = dcel.half_edges[half_edge_index].origin;
        std::size_t curr = first;
        Point curr_pt = dcel.points[curr];
        std::size_t prev_edge_idx = DCEL::npos;

        do {
            // find next unvisited edge or if all visited then break
            auto found = origin_to_edge.find(curr);
            if (found == origin_to_edge.end()) {
                throw std::logic_error("Boundary ring reached vertex with no outgoing edge");
            }

            std::size_t curr_edge_idx =
                nextNonInputBoundaryEdge(prev_edge_idx, found->second, visited);
            if (curr_edge_idx == DCEL::npos) {
                throw std::logic_error("Boundary ring is unclosed");
            }

            prev_edge_idx = curr_edge_idx;

            curr_ring_pts.push_back(curr_pt);

            DCEL::HalfEdge twin_edge = dcel.twinOf(dcel.half_edges[curr_edge_idx]);
            std::size_t next = twin_edge.origin;
            Point next_pt = dcel.points[next];
            curr = next;
            curr_pt = next_pt;
        } while (curr != first);

        rings.emplace_back(LinearRing(std::move(curr_ring_pts)));
        curr_ring_pts.clear();
    }

    return rings;
}

DCEL DCEL::Creator::build(const std::vector<const LinearRing*>& rings) {
    reserve(rings);

    for (const LinearRing* ring : rings) {
        processBoundaryRing(*ring, true);
    }

    // collectNonInputEdges();
    std::vector<std::size_t> non_input_edges;
    for (std::size_t i = 0; i < half_edges.size(); ++i) {
        DCELBuilderHalfEdge half_edge = half_edges[i];
        if (!half_edge.is_input_ring_edge) {
            non_input_edges.push_back(i);
        }
    }

    std::vector<LinearRing> non_input_rings = assembleBoundaryRing(non_input_edges);

    for (const LinearRing& non_input_ring : non_input_rings) {
        processBoundaryRing(non_input_ring, false);
    }

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
    const std::size_t unbounded_ring_index = ring_union_find.addSet();

    groupBoundaryRingsByFace(unbounded_ring_index);

    std::unordered_map<std::size_t, std::size_t> root_ring_to_face =
        createFaceByRingRoot(unbounded_ring_index);

    for (std::size_t ring_index = 0; ring_index < boundary_rings.size(); ++ring_index) {
        const DCELBuilderRing& boundary_ring = boundary_rings[ring_index];
        const std::size_t root_ring_index = ring_union_find.find(ring_index);
        const std::size_t face_index = getOrCreateFace(root_ring_index, root_ring_to_face);

        addBoundaryRingToFace(boundary_ring, face_index);
    }
}

void DCEL::Creator::groupBoundaryRingsByFace(const std::size_t unbounded_ring_index) {
    // There is an arc between two rings if and only if one of the rings is the boundary of a hole
    // and the other ring has a half-edge immediately to the left of the leftmost vertex of that
    // hole ring.
    for (std::size_t ring_index = 0; ring_index < boundary_rings.size(); ++ring_index) {
        const DCELBuilderRing& boundary_ring = boundary_rings[ring_index];
        if (boundary_ring.is_outer) {
            continue;
        }

        std::size_t leftmost_half_edge_index = boundary_ring.leftmost_half_edge;
        DCEL::HalfEdge& leftmost_half_edge = dcel.half_edges[leftmost_half_edge_index];
        Point& leftmost_point = static_cast<Point&>(dcel.originOf(leftmost_half_edge));
        std::size_t nearest_left_half_edge = findNearestHalfEdgeToLeft(leftmost_point);

        if (nearest_left_half_edge == DCEL::npos) {
            ring_union_find.unite(ring_index, unbounded_ring_index);
        } else {
            std::size_t nearest_left_ring = half_edges[nearest_left_half_edge].boundary_ring;
            assert(nearest_left_ring != DCEL::npos);
            assert(nearest_left_ring != ring_index);
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
    const std::size_t half_edge_index = boundary_ring.leftmost_half_edge;
    if (boundary_ring.is_outer) {
        assert(dcel.faces[face_index].outer_component == DCEL::npos);
        dcel.faces[face_index].outer_component = half_edge_index;
    } else {
        dcel.faces[face_index].inner_components.push_back(half_edge_index);
    }
}

void DCEL::Creator::assignFaceToBoundaryRing(const DCELBuilderRing& boundary_ring,
                                           const std::size_t face_index) {
    const std::size_t start = boundary_ring.leftmost_half_edge;
    std::size_t current = start;
    for (std::size_t i = 0; i < dcel.half_edges.size(); ++i) {
        dcel.half_edges[current].face = face_index;
        current = dcel.half_edges[current].next;
        if (current == start) {
            return;
        }
    }

    assert(false && "Boundary ring does not close while assigning face");
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

std::size_t DCEL::Creator::addBoundaryRing(const std::vector<std::size_t>& half_edge_indices,
                                         bool isOuter) {
    const std::size_t ring_index = boundary_rings.size();
    boundary_rings.push_back({DCEL::npos, isOuter});
    ring_union_find.addSet();
    // Add the half edge containing the leftmost origin vertex as the representative for each
    // boundary ring, which we will use to determine face connectivity.
    const std::size_t leftmost_half_edge = leftmostHalfEdge(half_edge_indices);
    boundary_rings[ring_index].leftmost_half_edge = leftmost_half_edge;
    return ring_index;
}

void DCEL::Creator::processBoundaryRing(const LinearRing& ring, bool is_input_ring) {
    const std::vector<Segment> segments = ring.segments();
    if (segments.empty()) {
        return;
    }

    std::vector<std::size_t> half_edge_indices;
    half_edge_indices.reserve(segments.size());

    for (const Segment& segment : segments) {
        if (!is_input_ring) {
            // The segment should have been created when processing the input rings
            assert(segment_map.contains(segment));
        }

        const std::size_t half_edge_index = getOrCreateHalfEdgePair(segment);
        half_edge_indices.push_back(half_edge_index);
    }

    linkBoundaryRingHalfEdges(half_edge_indices);

    const std::size_t ring_index = addBoundaryRing(half_edge_indices, ring.isOuter());

    for (const std::size_t half_edge_index : half_edge_indices) {
        half_edges[half_edge_index].boundary_ring = ring_index;
        if (is_input_ring) {
            half_edges[half_edge_index].is_input_ring_edge = true;
        }
    }
}

void DCEL::Creator::linkBoundaryRingHalfEdges(const std::vector<std::size_t>& half_edge_indices) {
    for (std::size_t i = 0; i < half_edge_indices.size(); ++i) {
        const std::size_t prev =
            half_edge_indices[(i + half_edge_indices.size() - 1) % half_edge_indices.size()];
        const std::size_t curr = half_edge_indices[i];
        const std::size_t next = half_edge_indices[(i + 1) % half_edge_indices.size()];

        DCEL::HalfEdge& half_edge = dcel.half_edges[curr];
        half_edge.prev = prev;
        half_edge.next = next;
    }
}

std::size_t DCEL::Creator::leftmostHalfEdge(const std::vector<std::size_t>& half_edge_indices) {
    assert(!half_edge_indices.empty());

    std::size_t leftmost_half_edge = half_edge_indices.front();
    for (const std::size_t half_edge_index : half_edge_indices) {
        DCEL::HalfEdge& half_edge = dcel.half_edges[half_edge_index];
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

DCEL DCEL::fromPolygons(const std::vector<Polygon>& polygons) {
    std::vector<const LinearRing*> rings;
    for (const Polygon& polygon : polygons) {
        for (const LinearRing* ring : polygon.rings()) {
            rings.push_back(ring);
        }
    }
    return DCEL::Creator(DCEL()).build(rings);
}

DCEL DCEL::fromRings(const std::vector<const LinearRing*>& rings) {
    return DCEL::Creator(DCEL()).build(rings);
}

DCEL DCEL::fromRings(const std::vector<LinearRing>& rings) {
    std::vector<const LinearRing*> ring_pointers;
    for (const LinearRing& ring : rings) {
        ring_pointers.push_back(&ring);
    }
    return DCEL::Creator(DCEL()).build(ring_pointers);
}

DCEL DCEL::fromSegments(const std::vector<Segment>& segments) {
    std::vector<const LinearRing*> rings;
    std::vector<LinearRing> assembled_rings = assembleRings(segments);
    for (const LinearRing& ring : assembled_rings) {
        rings.push_back(&ring);
    }
    return DCEL::Creator(DCEL()).build(rings);
}

const DCEL::Face& DCEL::unboundedFace() const {
    return faces[unbounded_face_index];
}

std::vector<std::size_t> DCEL::faceDepths() const {
    std::size_t depth = 0;
    std::vector<std::size_t> face_depths(faces.size(), DCEL::npos);
    face_depths[unbounded_face_index] = depth;

    std::queue<std::size_t> queue;
    queue.push(unbounded_face_index);

    while (!queue.empty()) {
        std::size_t count = queue.size();
        for (std::size_t i = 0; i < count; i++) {
            std::size_t face_index = queue.front();
            queue.pop();

            const Face& face = faces[face_index];
            for (std::size_t inner_component : face.inner_components) {
                const HalfEdge& half_edge = half_edges[inner_component];
                const HalfEdge& twin_half_edge = twinOf(half_edge);
                const std::size_t next_face = twin_half_edge.face;
                assert(next_face != npos);

                if (face_depths[next_face] != npos) {
                    continue;
                }

                face_depths[next_face] = depth + 1;
                queue.push(next_face);
            }
        }
        ++depth;
    }

    return face_depths;
}
