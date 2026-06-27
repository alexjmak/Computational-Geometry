#ifndef OVERLAY_HPP
#define OVERLAY_HPP

#include "geometry/dcel.hpp"
#include "geometry/segment.hpp"
#include <vector>

struct OverlayFaceLabel {
    std::size_t left_face = DCEL::npos;
    std::size_t right_face = DCEL::npos;
};

struct OverlayResult {
    DCEL dcel;
    std::vector<OverlayFaceLabel> faceLabels;
};

struct OverlayFacePolygon {
    std::size_t face_index = DCEL::npos;
    Polygon polygon;
    OverlayFaceLabel label;
};

enum class OverlaySource { Left, Right };

struct OverlaySourceEdge {
    Segment segment;
    OverlaySource source;
    std::size_t half_edge = DCEL::npos;
};

struct OverlaySplitEdge {
    Segment segment;
    std::vector<std::size_t> source_edges;
};

std::vector<Segment> planarizeSegments(const std::vector<Segment>& segments);

OverlayResult segmentOverlay(const DCEL& left, const DCEL& right);

OverlayResult segmentOverlay(const std::vector<Segment>& left, const std::vector<Segment>& right);

std::vector<OverlayFacePolygon> overlayFacePolygons(const OverlayResult& overlay);

std::vector<Segment> polygonAnd(const std::vector<Segment>& left,
                                const std::vector<Segment>& right);

std::vector<Segment> polygonOr(const std::vector<Segment>& left,
                               const std::vector<Segment>& right);

std::vector<Segment> polygonDifference(const std::vector<Segment>& left,
                                       const std::vector<Segment>& right);

std::vector<Segment> polygonXor(const std::vector<Segment>& left,
                                const std::vector<Segment>& right);

#endif // OVERLAY_HPP
