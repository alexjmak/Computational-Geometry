#include "geometry/polygon.hpp"
#include "geometry/predicates.hpp"
#include <cmath>
#include <stdexcept>

Ring::Ring(std::vector<Point> points) : points(std::move(points)) {}

double Ring::signedArea() const {
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

double Ring::area() const {
    return std::abs(signedArea());
}

bool Ring::isOuter() const {
    return signedArea() > 0.0;
}

std::vector<Segment> Ring::segments() const {
    std::vector<Segment> segments;
    Point prev = points.back();
    for (const auto& point : points) {
        segments.emplace_back(prev, point);
        prev = point;
    }
    return segments;
}

bool Ring::operator==(const Ring& other) const {
    return points == other.points;
}

void Ring::reverse() {
    std::reverse(points.begin(), points.end());
}

Polygon::Polygon(Ring outer_ring, std::vector<Ring> inner_rings)
    : outer_ring(outer_ring), inner_rings(inner_rings) {
    if (!this->outer_ring.isOuter()) {
        throw std::invalid_argument("Polygon outer ring must be counter-clockwise");
    }
    for (const Ring& inner_ring : this->inner_rings) {
        if (inner_ring.isOuter()) {
            throw std::invalid_argument("Polygon inner rings must be clockwise");
        }
    }
}

std::vector<Ring*> Polygon::rings() {
    std::vector<const Ring*> const_rings = static_cast<const Polygon&>(*this).rings();
    std::vector<Ring*> ret;
    ret.reserve(const_rings.size());

    for (const Ring* ring : const_rings) {
        ret.push_back(const_cast<Ring*>(ring));
    }

    return ret;
}

std::vector<const Ring*> Polygon::rings() const {
    std::vector<const Ring*> ret;
    ret.reserve(1 + inner_rings.size());

    ret.push_back(&outer_ring);
    for (const Ring& inner_ring : inner_rings) {
        ret.push_back(&inner_ring);
    }

    return ret;
}

double Polygon::area() const {
    double ret = outer_ring.signedArea();
    for (const Ring& inner_ring : inner_rings) {
        ret += inner_ring.signedArea();
    }
    return ret;
}

bool Polygon::operator==(const Polygon& other) const {
    return outer_ring == other.outer_ring && inner_rings == other.inner_rings;
}

Rectangle::Rectangle(Point lower_left, Point upper_right)
    : lower_left(lower_left), upper_right(upper_right) {}

Ring Rectangle::ring() const {
    Point lower_right(upper_right.x, lower_left.y);
    Point upper_left(lower_left.x, upper_right.y);
    return Ring({lower_left, lower_right, upper_right, upper_left});
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
    for (const Ring& inner_ring : polygon.inner_rings) {
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
