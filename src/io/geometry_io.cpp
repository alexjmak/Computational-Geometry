#include "io/geometry_io.hpp"
#include <fstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace {

Rational parseRational(const YAML::Node& node) {
    if (node.IsScalar()) {
        return Rational(node.as<long long>());
    }

    // Optional support for exact rationals as [numerator, denominator].
    if (node.IsSequence() && node.size() == 2) {
        return Rational(node[0].as<long long>(), node[1].as<long long>());
    }

    throw std::runtime_error("Invalid rational value");
}

YAML::Node emitRational(const Rational& r) {
    if (r.denominator() == 1) {
        return YAML::Node(r.numerator());
    }

    YAML::Node node;
    node.push_back(r.numerator());
    node.push_back(r.denominator());
    node.SetStyle(YAML::EmitterStyle::Flow);
    return node;
}

Point parsePoint(const YAML::Node& node) {
    if (!node.IsSequence() || node.size() != 2) {
        throw std::runtime_error("Point must be [x, y]");
    }

    return Point(parseRational(node[0]), parseRational(node[1]));
}

YAML::Node emitPoint(const Point& p) {
    YAML::Node node;
    node.push_back(emitRational(p.x));
    node.push_back(emitRational(p.y));
    node.SetStyle(YAML::EmitterStyle::Flow);
    return node;
}

Ring parseRing(const YAML::Node& node) {
    if (!node.IsSequence()) {
        throw std::runtime_error("Ring must be a list of points");
    }

    std::vector<Point> points;
    points.reserve(node.size());

    for (const auto& pointNode : node) {
        points.push_back(parsePoint(pointNode));
    }

    return Ring(std::move(points));
}

YAML::Node emitRing(const Ring& ring) {
    YAML::Node node;

    for (const Point& point : ring.points) {
        node.push_back(emitPoint(point));
    }

    return node;
}

Segment parseSegment(const YAML::Node& node) {
    if (!node.IsSequence() || node.size() != 2) {
        throw std::runtime_error("Segment must be [[x1, y1], [x2, y2]]");
    }

    return Segment(parsePoint(node[0]), parsePoint(node[1]));
}

YAML::Node emitSegment(const Segment& segment) {
    YAML::Node node;
    node.push_back(emitPoint(segment.start));
    node.push_back(emitPoint(segment.end));
    node.SetStyle(YAML::EmitterStyle::Flow);
    return node;
}

Polygon parsePolygon(const YAML::Node& node) {
    if (!node["outer"]) {
        throw std::runtime_error("Polygon is missing required 'outer' ring");
    }

    Ring outer_ring = parseRing(node["outer"]);
    std::vector<Ring> inner_rings;

    if (node["holes"]) {
        for (const auto& holeNode : node["holes"]) {
            inner_rings.push_back(parseRing(holeNode));
        }
    }

    return Polygon(std::move(outer_ring), std::move(inner_rings));
}

YAML::Node emitPolygon(const Polygon& polygon) {
    YAML::Node node;

    node["outer"] = emitRing(polygon.outer_ring);

    if (!polygon.inner_rings.empty()) {
        YAML::Node holes;

        for (const Ring& inner_ring : polygon.inner_rings) {
            holes.push_back(emitRing(inner_ring));
        }

        node["holes"] = holes;
    }

    return node;
}

Layer parseLayer(const YAML::Node& node) {
    Layer layer;

    if (!node["name"]) {
        throw std::runtime_error("Layer is missing required 'name'");
    }

    layer.name = node["name"].as<std::string>();

    if (node["points"]) {
        for (const auto& pointNode : node["points"]) {
            layer.points.push_back(parsePoint(pointNode));
        }
    }

    if (node["polygons"]) {
        for (const auto& polygonNode : node["polygons"]) {
            layer.polygons.push_back(parsePolygon(polygonNode));
        }
    }

    if (node["segments"]) {
        for (const auto& segmentNode : node["segments"]) {
            layer.segments.push_back(parseSegment(segmentNode));
        }
    }

    return layer;
}

YAML::Node emitLayer(const Layer& layer) {
    YAML::Node node;
    node["name"] = layer.name;

    if (!layer.points.empty()) {
        YAML::Node points;

        for (const Point& point : layer.points) {
            points.push_back(emitPoint(point));
        }

        node["points"] = points;
    }

    if (!layer.polygons.empty()) {
        YAML::Node polygons;

        for (const Polygon& polygon : layer.polygons) {
            polygons.push_back(emitPolygon(polygon));
        }

        node["polygons"] = polygons;
    }

    if (!layer.segments.empty()) {
        YAML::Node segments;

        for (const Segment& segment : layer.segments) {
            segments.push_back(emitSegment(segment));
        }

        node["segments"] = segments;
    }

    return node;
}

} // namespace

Document loadYaml(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);

    if (!root["layers"] || !root["layers"].IsSequence()) {
        throw std::runtime_error("Root must contain a 'layers' list");
    }

    Document document;

    for (const auto& layerNode : root["layers"]) {
        document.layers.push_back(parseLayer(layerNode));
    }

    return document;
}

void saveYaml(const Document& document, const std::string& path) {
    YAML::Node root;
    YAML::Node layers;

    for (const Layer& layer : document.layers) {
        layers.push_back(emitLayer(layer));
    }

    root["layers"] = layers;

    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Could not open file for writing: " + path);
    }

    out << root;
}
