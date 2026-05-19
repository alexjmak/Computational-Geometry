#include "geometry/dcel.hpp"
#include "algorithms/assemble.hpp"
#include "geometry/intersection.hpp"
#include "geometry/polygon.hpp"
#include "geometry/predicates.hpp"
#include <cassert>
#include <cstddef>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

/// \brief Disjoint-set structure for grouping boundary cycles that bound the same face.
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

/// \brief Builder metadata for one directed DCEL boundary cycle.
struct DCELBoundaryCycle {
    std::size_t leftmost_half_edge; ///< Representative half-edge at the cycle's leftmost vertex.
    bool is_outer;                  ///< True when the cycle is an outer face boundary.
};

/// \brief Incremental builder for constructing a DCEL from polygon boundaries.
class DCELCreator {
  public:
    /// \brief Initialize a builder around an empty DCEL supplied by a factory.
    /// \param dcel The empty DCEL to populate.
    explicit DCELCreator(DCEL dcel);

    /// \brief Build a DCEL from polygon boundary cycles.
    /// \param cycles The cycles whose boundaries should be inserted.
    /// \returns A populated DCEL.
    DCEL build(const std::vector<const Cycle*>& cycles);

  private:
    DCEL dcel;                                        ///< DCEL being incrementally constructed.
    std::unordered_map<Point, std::size_t> point_map; ///< Maps geometric points to vertex indices.
    std::unordered_map<Segment, std::size_t> segment_map; ///< Maps directed segments to half-edges.
    std::vector<std::size_t> half_edge_to_cycle_indices;  ///< Maps half-edges to boundary cycles.
    std::vector<DCELBoundaryCycle> boundary_cycles;       ///< Builder metadata for directed cycles.
    UnionFind cycle_union_find; ///< Groups boundary cycles that bound the same face.

    /// \brief Reserve storage for vertices and half-edges.
    /// \param polygons The polygons that will be inserted.
    void reserve(const std::vector<const Cycle*>& cycles);

    /// \brief Add a polygon boundary cycle and its twin boundary cycle.
    /// \param cycle The input boundary cycle to add.
    void addCycle(const Cycle& cycle);

    /// \brief Create DCEL face records from the boundary-cycle connectivity graph.
    void createFaces();

    /// \brief Union boundary cycles that bound the same face.
    /// \param unbounded_cycle_index The union-find node for the imaginary unbounded boundary.
    void groupBoundaryCyclesByFace(std::size_t unbounded_cycle_index);

    /// \brief Create the initial mapping from cycle roots to face indices.
    /// \param unbounded_cycle_index The union-find node for the imaginary unbounded boundary.
    /// \returns A map from union-find roots to DCEL face indices, seeded with the unbounded face.
    std::unordered_map<std::size_t, std::size_t>
    createFaceByCycleRoot(std::size_t unbounded_cycle_index);

    /// \brief Resolve or create the DCEL face for a cycle-root group.
    /// \param root_cycle_index The union-find root representing a face group.
    /// \param root_cycle_to_face Map from cycle roots to existing face indices.
    /// \returns The DCEL face index for the cycle-root group.
    std::size_t getOrCreateFace(std::size_t root_cycle_index,
                                std::unordered_map<std::size_t, std::size_t>& root_cycle_to_face);

    /// \brief Attach a boundary cycle representative to a DCEL face.
    /// \param boundary_cycle The boundary cycle metadata to attach.
    /// \param face_index The face receiving the boundary component.
    void addBoundaryCycleToFace(const DCELBoundaryCycle& boundary_cycle, std::size_t face_index);

    /// \brief Assign a face index to every half-edge in a boundary cycle.
    /// \param boundary_cycle The boundary cycle whose half-edges should be assigned.
    /// \param face_index The face incident to the boundary cycle.
    void assignFaceToBoundaryCycle(const DCELBoundaryCycle& boundary_cycle, std::size_t face_index);

    /// \brief Link the next/previous pointers for a directed cycle and its twin cycle.
    /// \param half_edge_indices Ordered half-edge indices for the directed cycle.
    void linkCycleHalfEdges(const std::vector<std::size_t>& half_edge_indices);

    /// \brief Find the half-edge whose origin is the leftmost vertex in a cycle.
    /// \param half_edge_indices Ordered half-edge indices for the cycle.
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
};

DCELCreator::DCELCreator(DCEL dcel) : dcel(std::move(dcel)) {}

DCEL DCELCreator::build(const std::vector<const Cycle*>& cycles) {
    reserve(cycles);

    for (const Cycle* cycle : cycles) {
        addCycle(*cycle);
    }

    createFaces();

    return std::move(dcel);
}

std::size_t DCELCreator::findNearestHalfEdgeToLeft(const Point& point) {
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

void DCELCreator::createFaces() {
    const std::size_t unbounded_cycle_index = cycle_union_find.addSet();

    groupBoundaryCyclesByFace(unbounded_cycle_index);

    std::unordered_map<std::size_t, std::size_t> root_cycle_to_face =
        createFaceByCycleRoot(unbounded_cycle_index);

    for (std::size_t cycle_index = 0; cycle_index < boundary_cycles.size(); ++cycle_index) {
        const DCELBoundaryCycle& boundary_cycle = boundary_cycles[cycle_index];
        const std::size_t root_cycle_index = cycle_union_find.find(cycle_index);
        const std::size_t face_index = getOrCreateFace(root_cycle_index, root_cycle_to_face);

        addBoundaryCycleToFace(boundary_cycle, face_index);
    }
}

void DCELCreator::groupBoundaryCyclesByFace(const std::size_t unbounded_cycle_index) {
    // There is an arc between two cycles if and only if one of the cycles is the boundary of a hole
    // and the other cycle has a half-edge immediately to the left of the leftmost vertex of that
    // hole cycle.
    for (std::size_t cycle_index = 0; cycle_index < boundary_cycles.size(); ++cycle_index) {
        const DCELBoundaryCycle& boundary_cycle = boundary_cycles[cycle_index];
        if (boundary_cycle.is_outer) {
            continue;
        }

        std::size_t leftmost_half_edge_index = boundary_cycle.leftmost_half_edge;
        DCEL::HalfEdge& leftmost_half_edge = dcel.half_edges[leftmost_half_edge_index];
        Point& leftmost_point = static_cast<Point&>(dcel.originOf(leftmost_half_edge));
        std::size_t nearest_left_half_edge = findNearestHalfEdgeToLeft(leftmost_point);

        if (nearest_left_half_edge == DCEL::npos) {
            cycle_union_find.unite(cycle_index, unbounded_cycle_index);
        } else {
            std::size_t nearest_left_cycle = half_edge_to_cycle_indices[nearest_left_half_edge];
            assert(nearest_left_cycle != DCEL::npos);
            assert(nearest_left_cycle != cycle_index);
            cycle_union_find.unite(cycle_index, nearest_left_cycle);
        }
    }
}

std::unordered_map<std::size_t, std::size_t>
DCELCreator::createFaceByCycleRoot(const std::size_t unbounded_cycle_index) {
    // Group cycles into faces based on which cycles are connected by unions.
    std::unordered_map<std::size_t, std::size_t> root_cycle_to_face;

    // First add the unbounded face
    assert(dcel.faces.size() == DCEL::unbounded_face_index);
    dcel.faces.emplace_back(DCEL::Face()); // Start with the unbounded face.
    dcel.faces[DCEL::unbounded_face_index].outer_component = DCEL::npos;
    root_cycle_to_face[cycle_union_find.find(unbounded_cycle_index)] = DCEL::unbounded_face_index;

    return root_cycle_to_face;
}

std::size_t
DCELCreator::getOrCreateFace(const std::size_t root_cycle_index,
                             std::unordered_map<std::size_t, std::size_t>& root_cycle_to_face) {
    auto found = root_cycle_to_face.find(root_cycle_index);
    if (found != root_cycle_to_face.end()) {
        return found->second;
    }

    const std::size_t face_index = dcel.faces.size();
    dcel.faces.emplace_back(DCEL::Face());
    root_cycle_to_face[root_cycle_index] = face_index;
    return face_index;
}

void DCELCreator::addBoundaryCycleToFace(const DCELBoundaryCycle& boundary_cycle,
                                         const std::size_t face_index) {

    // Assign this face to every half-edge around the boundary component.
    assignFaceToBoundaryCycle(boundary_cycle, face_index);

    // Store one representative half-edge for this boundary component.
    const std::size_t half_edge_index = boundary_cycle.leftmost_half_edge;
    if (boundary_cycle.is_outer) {
        assert(dcel.faces[face_index].outer_component == DCEL::npos);
        dcel.faces[face_index].outer_component = half_edge_index;
    } else {
        dcel.faces[face_index].inner_components.push_back(half_edge_index);
    }
}

void DCELCreator::assignFaceToBoundaryCycle(const DCELBoundaryCycle& boundary_cycle,
                                            const std::size_t face_index) {
    const std::size_t start = boundary_cycle.leftmost_half_edge;
    std::size_t current = start;
    for (std::size_t i = 0; i < dcel.half_edges.size(); ++i) {
        dcel.half_edges[current].face = face_index;
        current = dcel.half_edges[current].next;
        if (current == start) {
            return;
        }
    }

    assert(false && "Boundary cycle does not close while assigning face");
}

void DCELCreator::reserve(const std::vector<const Cycle*>& cycles) {
    std::size_t segment_count = 0;
    std::size_t cycle_count = 0;
    for (const Cycle* cycle : cycles) {
        segment_count += cycle->points.size();
        cycle_count++;
    }

    dcel.points.reserve(segment_count * 2);
    dcel.half_edges.reserve(segment_count * 2);
    half_edge_to_cycle_indices.reserve(segment_count * 2);
    boundary_cycles.reserve(cycle_count * 2);
}

std::size_t DCELCreator::getOrCreatePoint(const Point& point) {
    const auto found = point_map.find(point);
    if (found != point_map.end()) {
        return found->second;
    }

    dcel.points.emplace_back(DCEL::DCELPoint(point));
    const std::size_t point_index = dcel.points.size() - 1;
    point_map[point] = point_index;
    return point_index;
}

std::size_t DCELCreator::createHalfEdge(const Segment& segment) {
    const std::size_t start_index = getOrCreatePoint(segment.start);

    dcel.half_edges.emplace_back(DCEL::HalfEdge(start_index));

    // Initialize the half-edge cycle index to npos to indicate that it is not yet part of any
    // cycle. We will set this to the correct cycle index in addCycle
    half_edge_to_cycle_indices.push_back(DCEL::npos);

    const std::size_t half_edge_index = dcel.half_edges.size() - 1;
    if (dcel.points[start_index].half_edge == DCEL::npos) {
        dcel.points[start_index].half_edge = half_edge_index;
    }
    return half_edge_index;
}

void DCELCreator::addCycle(const Cycle& cycle) {
    const std::vector<Segment> segments = cycle.segments();
    if (segments.empty()) {
        return;
    }

    std::vector<std::size_t> half_edge_indices;
    half_edge_indices.reserve(segments.size());

    const std::size_t cycle_index = boundary_cycles.size();
    boundary_cycles.push_back({DCEL::npos, cycle.isOuter()});
    cycle_union_find.addSet();

    const std::size_t twin_cycle_index = boundary_cycles.size();
    boundary_cycles.push_back({DCEL::npos, !boundary_cycles[cycle_index].is_outer});
    cycle_union_find.addSet();

    for (const Segment& segment : segments) {
        const std::size_t half_edge_index = getOrCreateHalfEdgePair(segment);
        half_edge_indices.push_back(half_edge_index);

        const std::size_t twin_half_edge_index = dcel.half_edges[half_edge_index].twin;
        half_edge_to_cycle_indices[half_edge_index] = cycle_index;
        half_edge_to_cycle_indices[twin_half_edge_index] = twin_cycle_index;
    }

    linkCycleHalfEdges(half_edge_indices);

    // Add the half edge containing the leftmost origin vertex as the representative for each
    // boundary cycle, which we will use to determine face connectivity.
    const std::size_t leftmost_half_edge = leftmostHalfEdge(half_edge_indices);
    const std::size_t twin_leftmost_half_edge =
        dcel.half_edges[dcel.half_edges[leftmost_half_edge].twin].next;
    boundary_cycles[cycle_index].leftmost_half_edge = leftmost_half_edge;
    boundary_cycles[twin_cycle_index].leftmost_half_edge = twin_leftmost_half_edge;
}

void DCELCreator::linkCycleHalfEdges(const std::vector<std::size_t>& half_edge_indices) {
    for (std::size_t i = 0; i < half_edge_indices.size(); ++i) {
        const std::size_t prev =
            half_edge_indices[(i + half_edge_indices.size() - 1) % half_edge_indices.size()];
        const std::size_t curr = half_edge_indices[i];
        const std::size_t next = half_edge_indices[(i + 1) % half_edge_indices.size()];

        DCEL::HalfEdge& half_edge = dcel.half_edges[curr];
        half_edge.prev = prev;
        half_edge.next = next;

        DCEL::HalfEdge& twin_half_edge = dcel.half_edges[half_edge.twin];
        twin_half_edge.prev = dcel.half_edges[next].twin;
        twin_half_edge.next = dcel.half_edges[prev].twin;
    }
}

std::size_t DCELCreator::leftmostHalfEdge(const std::vector<std::size_t>& half_edge_indices) {
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

std::size_t DCELCreator::getOrCreateHalfEdgePair(const Segment& segment) {
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

} // namespace

DCEL::DCELPoint::DCELPoint(Rational x, Rational y) : Point(x, y), half_edge(DCEL::npos) {}

DCEL::DCELPoint::DCELPoint(const Point& p) : Point(p), half_edge(DCEL::npos) {}

DCEL::HalfEdge::HalfEdge(std::size_t origin)
    : origin(origin), twin(DCEL::npos), next(DCEL::npos), prev(DCEL::npos), face(DCEL::npos) {}

DCEL::Face::Face() : outer_component(DCEL::npos), inner_components() {}

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

Cycle DCEL::cycleOf(const HalfEdge& half_edge) const {
    std::vector<Point> points;
    const HalfEdge* current_half_edge = &half_edge;
    do {
        points.push_back(originOf(*current_half_edge));
        current_half_edge = &nextOf(*current_half_edge);
    } while (current_half_edge != &half_edge);
    return Cycle(std::move(points));
}

std::optional<Polygon> DCEL::polygonOf(const Face& face) const {
    if (face.outer_component == npos) {
        // Unbounded face is not a finite polygon, so forcing it into Polygon would be semantically
        // wrong.
        return std::nullopt;
    }

    const HalfEdge& outer_half_edge = half_edges[face.outer_component];
    Cycle outer_cycle = cycleOf(outer_half_edge);

    std::vector<Cycle> inner_cycles;
    for (const std::size_t inner_component : face.inner_components) {
        const HalfEdge& inner_half_edge = half_edges[inner_component];
        inner_cycles.push_back(cycleOf(inner_half_edge));
    }

    return Polygon(std::move(outer_cycle), std::move(inner_cycles));
}

DCEL DCEL::fromPolygons(const std::vector<Polygon>& polygons) {
    std::vector<const Cycle*> cycles;
    for (const Polygon& polygon : polygons) {
        for (const Cycle* cycle : polygon.cycles()) {
            cycles.push_back(cycle);
        }
    }
    return DCELCreator(DCEL()).build(cycles);
}

DCEL DCEL::fromCycles(const std::vector<const Cycle*>& cycles) {
    return DCELCreator(DCEL()).build(cycles);
}

DCEL DCEL::fromCycles(const std::vector<Cycle>& cycles) {
    std::vector<const Cycle*> cycle_pointers;
    for (const Cycle& cycle : cycles) {
        cycle_pointers.push_back(&cycle);
    }
    return DCELCreator(DCEL()).build(cycle_pointers);
}

DCEL DCEL::fromSegments(const std::vector<Segment>& segments) {
    std::vector<const Cycle*> cycles;
    std::vector<Cycle> assembled_cycles = assembleCycles(segments);
    for (const Cycle& cycle : assembled_cycles) {
        cycles.push_back(&cycle);
    }
    return DCELCreator(DCEL()).build(cycles);
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
