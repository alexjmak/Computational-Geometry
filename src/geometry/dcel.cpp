#include "geometry/dcel.hpp"
#include <cassert>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace {

class DCELCreator {
  public:
    DCEL build(const std::vector<Polygon>& polygons);

  private:
    DCEL dcel;
    std::unordered_map<Point, std::size_t> point_map;
    std::unordered_map<Segment, std::size_t> segment_map;
    std::vector<DCEL::HalfEdge*> cycle_leftmost_half_edges;

    void reserve(const std::vector<Polygon>& polygons);
    void addCycle(const Cycle& cycle);
    void linkCycleHalfEdges(const std::vector<std::size_t>& half_edge_indices);
    DCEL::HalfEdge* leftmostHalfEdge(const std::vector<std::size_t>& half_edge_indices);
    std::size_t getOrCreateHalfEdgePair(const Segment& segment);
    std::size_t getOrCreatePoint(const Point& point);
    std::size_t createHalfEdge(const Segment& segment);
};

DCEL DCELCreator::build(const std::vector<Polygon>& polygons) {
    reserve(polygons);

    for (const Polygon& polygon : polygons) {
        for (const Cycle* cycle : polygon.cycles()) {
            addCycle(*cycle);
        }
    }

    return std::move(dcel);
}

void DCELCreator::reserve(const std::vector<Polygon>& polygons) {
    std::size_t segment_count = 0;
    for (const Polygon& polygon : polygons) {
        for (const Cycle* cycle : polygon.cycles()) {
            segment_count += cycle->points.size();
        }
    }

    dcel.points.reserve(segment_count * 2);
    dcel.half_edges.reserve(segment_count * 2);
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

    dcel.half_edges.emplace_back(DCEL::HalfEdge(&dcel.points[start_index]));
    const std::size_t half_edge_index = dcel.half_edges.size() - 1;
    if (!dcel.points[start_index].half_edge) {
        dcel.points[start_index].half_edge = &dcel.half_edges[half_edge_index];
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

    for (const Segment& segment : segments) {
        half_edge_indices.push_back(getOrCreateHalfEdgePair(segment));
    }

    linkCycleHalfEdges(half_edge_indices);

    DCEL::HalfEdge* leftmost_half_edge = leftmostHalfEdge(half_edge_indices);
    cycle_leftmost_half_edges.push_back(leftmost_half_edge);
    cycle_leftmost_half_edges.push_back(leftmost_half_edge->twin->next);
}

void DCELCreator::linkCycleHalfEdges(const std::vector<std::size_t>& half_edge_indices) {
    for (std::size_t i = 0; i < half_edge_indices.size(); ++i) {
        const std::size_t prev =
            half_edge_indices[(i + half_edge_indices.size() - 1) % half_edge_indices.size()];
        const std::size_t curr = half_edge_indices[i];
        const std::size_t next = half_edge_indices[(i + 1) % half_edge_indices.size()];

        DCEL::HalfEdge& half_edge = dcel.half_edges[curr];
        half_edge.prev = &dcel.half_edges[prev];
        half_edge.next = &dcel.half_edges[next];

        DCEL::HalfEdge& twin_half_edge = *half_edge.twin;
        twin_half_edge.prev = dcel.half_edges[next].twin;
        twin_half_edge.next = dcel.half_edges[prev].twin;
    }
}

DCEL::HalfEdge* DCELCreator::leftmostHalfEdge(const std::vector<std::size_t>& half_edge_indices) {
    assert(!half_edge_indices.empty());

    DCEL::HalfEdge* leftmost_half_edge = &dcel.half_edges[half_edge_indices.front()];
    for (const std::size_t half_edge_index : half_edge_indices) {
        DCEL::HalfEdge& half_edge = dcel.half_edges[half_edge_index];
        if (*half_edge.origin < *leftmost_half_edge->origin) {
            leftmost_half_edge = &half_edge;
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

    half_edge.twin = &twin_half_edge;
    twin_half_edge.twin = &half_edge;

    segment_map[segment] = half_edge_index;
    segment_map[reversed_segment] = twin_half_edge_index;

    return half_edge_index;
}

} // namespace

DCEL::DCELPoint::DCELPoint(Rational x, Rational y) : Point(x, y), half_edge(nullptr) {}

DCEL::DCELPoint::DCELPoint(const Point& p) : Point(p), half_edge(nullptr) {}

DCEL::HalfEdge::HalfEdge(DCEL::DCELPoint* origin)
    : origin(origin), twin(nullptr), next(nullptr), prev(nullptr), face(nullptr) {}

DCEL::Face::Face() : outer_component(nullptr), inner_components() {}

DCEL DCEL::fromPolygons(const std::vector<Polygon>& polygons) {
    return DCELCreator().build(polygons);
}
