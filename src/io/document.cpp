#include "io/document.hpp"
#include <algorithm>

bool Layer::operator==(const Layer& other) const {
    std::vector<Point> lhs_points = points;
    std::vector<Point> rhs_points = other.points;
    std::vector<Segment> lhs_segments = segments;
    std::vector<Segment> rhs_segments = other.segments;
    std::sort(lhs_points.begin(), lhs_points.end());
    std::sort(rhs_points.begin(), rhs_points.end());
    std::sort(lhs_segments.begin(), lhs_segments.end());
    std::sort(rhs_segments.begin(), rhs_segments.end());

    return name == other.name && lhs_points == rhs_points && lhs_segments == rhs_segments &&
           polygons == other.polygons;
}

void Layer::filter(const Rectangle& rectangle) {
    auto point_outside = [&](const Point& point) { return !rectangle.contains(point); };
    auto segment_outside = [&](const Segment& segment) { return !rectangle.contains(segment); };
    auto polygon_outside = [&](const Polygon& polygon) { return !rectangle.contains(polygon); };

    points.erase(std::remove_if(points.begin(), points.end(), point_outside), points.end());
    segments.erase(std::remove_if(segments.begin(), segments.end(), segment_outside),
                   segments.end());
    polygons.erase(std::remove_if(polygons.begin(), polygons.end(), polygon_outside),
                   polygons.end());
}

std::vector<Point> Document::collectPoints() const {
    std::vector<Point> points;
    for (const Layer& layer : layers) {
        points.insert(points.end(), layer.points.begin(), layer.points.end());
        for (const Segment& segment : layer.segments) {
            points.push_back(segment.start);
            points.push_back(segment.end);
        }
        for (const Polygon& polygon : layer.polygons) {
            for (const Cycle& cycle : polygon.cycles) {
                points.insert(points.end(), cycle.points.begin(), cycle.points.end());
            }
        }
    }
    return points;
}

std::vector<Segment> Document::collectSegments() const {
    std::vector<Segment> segments;
    for (const Layer& layer : layers) {
        segments.insert(segments.end(), layer.segments.begin(), layer.segments.end());
        for (const Polygon& polygon : layer.polygons) {
            for (const Cycle& cycle : polygon.cycles) {
                std::vector<Segment> cycle_segments = cycle.segments();
                segments.insert(segments.end(), cycle_segments.begin(), cycle_segments.end());
            }
        }
    }
    return segments;
}

bool Document::operator==(const Document& other) const {
    return layers == other.layers;
}
