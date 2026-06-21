#include "geometry/polygon.hpp"
#include "geometry/intersection.hpp"
#include "geometry/predicates.hpp"
#include <cmath>
#include <stdexcept>

double Ring::area() const {
    return std::abs(signedArea());
}

bool Ring::isOuter() const {
    return signedArea() > 0.0;
}

LinearRing::LinearRing(std::vector<Point> points) : points(std::move(points)) {}

double LinearRing::signedArea() const {
    if (points.size() < 3) {
        return 0.0;
    }
    double ret = 0.0;
    Point prev = points.back();
    for (const auto& point : points) {
        ret += boost::rational_cast<double>(crossProduct(prev, point));
        prev = point;
    }
    return 0.5 * ret;
}

std::vector<Segment> LinearRing::segments() const {
    std::vector<Segment> segments;
    Point prev = points.back();
    for (const auto& point : points) {
        segments.emplace_back(prev, point);
        prev = point;
    }
    return segments;
}

bool LinearRing::operator==(const LinearRing& other) const {
    return points == other.points;
}

void LinearRing::reverse() {
    std::reverse(points.begin(), points.end());
}

Polygon::Polygon(LinearRing outer_ring, std::vector<LinearRing> inner_rings)
    : outer_ring(outer_ring), inner_rings(inner_rings) {
    if (!this->outer_ring.isOuter()) {
        throw std::invalid_argument("Polygon outer ring must be counter-clockwise");
    }
    for (const LinearRing& inner_ring : this->inner_rings) {
        if (inner_ring.isOuter()) {
            throw std::invalid_argument("Polygon inner rings must be clockwise");
        }
    }
}

std::vector<LinearRing*> Polygon::rings() {
    std::vector<const LinearRing*> const_rings = static_cast<const Polygon&>(*this).rings();
    std::vector<LinearRing*> ret;
    ret.reserve(const_rings.size());

    for (const LinearRing* ring : const_rings) {
        ret.push_back(const_cast<LinearRing*>(ring));
    }

    return ret;
}

std::vector<const LinearRing*> Polygon::rings() const {
    std::vector<const LinearRing*> ret;
    ret.reserve(1 + inner_rings.size());

    ret.push_back(&outer_ring);
    for (const LinearRing& inner_ring : inner_rings) {
        ret.push_back(&inner_ring);
    }

    return ret;
}

double Polygon::area() const {
    double ret = outer_ring.signedArea();
    for (const LinearRing& inner_ring : inner_rings) {
        ret += inner_ring.signedArea();
    }
    return ret;
}

bool Polygon::operator==(const Polygon& other) const {
    return outer_ring == other.outer_ring && inner_rings == other.inner_rings;
}

std::vector<Segment> toSegments(const std::vector<Polygon>& polygons) {
    std::vector<Segment> segments;
    for (const Polygon& polygon : polygons) {
        for (const LinearRing* ring : polygon.rings()) {
            std::vector<Segment> ring_segments = ring->segments();
            segments.insert(segments.end(), ring_segments.begin(), ring_segments.end());
        }
    }
    return segments;
}

Rectangle::Rectangle(Point lower_left, Point upper_right)
    : lower_left(lower_left), upper_right(upper_right) {}

LinearRing Rectangle::ring() const {
    Point lower_right(upper_right.x, lower_left.y);
    Point upper_left(lower_left.x, upper_right.y);
    return LinearRing({lower_left, lower_right, upper_right, upper_left});
}

Polygon Rectangle::polygon() const {
    return Polygon(ring());
}

bool Rectangle::contains(const Point& p) const {
    return p.x >= lower_left.x && p.x <= upper_right.x && p.y >= lower_left.y &&
           p.y <= upper_right.y;
}

bool Rectangle::contains(const Segment& s) const {
    return contains(s.start) && contains(s.end);
}

bool Rectangle::contains(const Polygon& polygon) const {
    for (const Point& point : polygon.outer_ring.points) {
        if (!contains(point)) {
            return false;
        }
    }
    for (const LinearRing& inner_ring : polygon.inner_rings) {
        for (const Point& point : inner_ring.points) {
            if (!contains(point)) {
                return false;
            }
        }
    }
    return true;
}

double Rectangle::area() const {
    Rational width = upper_right.x - lower_left.x;
    Rational height = upper_right.y - lower_left.y;
    return boost::rational_cast<double>(width * height);
}

PointContainment locatePoint(const LinearRing& ring, const Point& point) {
    bool inside = false;

    for (const Segment& segment : ring.segments()) {
        if (isPointOnSegment(point, segment)) {
            return PointContainment::Boundary;
        }

        // Don't count horizontal edges which have infinite intersection points
        bool crosses = segment.start.y > point.y != segment.end.y > point.y;
        if (crosses && rightRayIntersection(segment, point)) {
            // Odd-even fill rule
            inside = !inside;
        }
    }

    if (inside) {
        return PointContainment::Inside;
    }

    return PointContainment::Outside;
}

PointContainment locatePoint(const Polygon& polygon, const Point& point) {
    PointContainment outer_point_containment = locatePoint(polygon.outer_ring, point);
    if (outer_point_containment != PointContainment::Inside) {
        return outer_point_containment;
    }

    for (const LinearRing& inner_ring : polygon.inner_rings) {
        PointContainment inner_point_containment = locatePoint(inner_ring, point);
        if (inner_point_containment == PointContainment::Boundary) {
            return PointContainment::Boundary;
        }

        if (inner_point_containment == PointContainment::Inside) {
            return PointContainment::Outside;
        }
    }

    return PointContainment::Inside;
}

bool pointInPolygon(const Polygon& polygon, const Point& point, bool include_boundary) {
    PointContainment point_containment = locatePoint(polygon, point);
    if (point_containment == PointContainment::Inside) {
        return true;
    }

    if (include_boundary && point_containment == PointContainment::Boundary) {
        return true;
    }

    return false;
}