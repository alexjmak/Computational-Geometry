#include "geometry/polygon.hpp"
#include "geometry/predicates.hpp"
#include <cmath>
#include <stdexcept>

Cycle::Cycle(std::vector<Point> points) : points(std::move(points)) {}

double Cycle::signedArea() const {
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

double Cycle::area() const {
    return std::abs(signedArea());
}

bool Cycle::isOuter() const {
    return signedArea() > 0.0;
}

std::vector<Segment> Cycle::segments() const {
    std::vector<Segment> segments;
    Point prev = points.back();
    for (const auto& point : points) {
        segments.emplace_back(prev, point);
        prev = point;
    }
    return segments;
}

bool Cycle::operator==(const Cycle& other) const {
    return points == other.points;
}

void Cycle::reverse() {
    std::reverse(points.begin(), points.end());
}

Polygon::Polygon(Cycle outer_cycle, std::vector<Cycle> inner_cycles)
    : outer_cycle(outer_cycle), inner_cycles(inner_cycles) {
    if (!this->outer_cycle.isOuter()) {
        throw std::invalid_argument("Polygon outer cycle must be counter-clockwise");
    }
    for (const Cycle& inner_cycle : this->inner_cycles) {
        if (inner_cycle.isOuter()) {
            throw std::invalid_argument("Polygon inner cycles must be clockwise");
        }
    }
}

std::vector<Cycle*> Polygon::cycles() {
    std::vector<const Cycle*> const_cycles = static_cast<const Polygon&>(*this).cycles();
    std::vector<Cycle*> ret;
    ret.reserve(const_cycles.size());

    for (const Cycle* cycle : const_cycles) {
        ret.push_back(const_cast<Cycle*>(cycle));
    }

    return ret;
}

std::vector<const Cycle*> Polygon::cycles() const {
    std::vector<const Cycle*> ret;
    ret.reserve(1 + inner_cycles.size());

    ret.push_back(&outer_cycle);
    for (const Cycle& inner_cycle : inner_cycles) {
        ret.push_back(&inner_cycle);
    }

    return ret;
}

double Polygon::area() const {
    double ret = outer_cycle.signedArea();
    for (const Cycle& inner_cycle : inner_cycles) {
        ret += inner_cycle.signedArea();
    }
    return ret;
}

bool Polygon::operator==(const Polygon& other) const {
    return outer_cycle == other.outer_cycle && inner_cycles == other.inner_cycles;
}

Rectangle::Rectangle(Point lower_left, Point upper_right)
    : lower_left(lower_left), upper_right(upper_right) {}

Cycle Rectangle::cycle() const {
    Point lower_right(upper_right.x, lower_left.y);
    Point upper_left(lower_left.x, upper_right.y);
    return Cycle({lower_left, lower_right, upper_right, upper_left});
}

Polygon Rectangle::polygon() const {
    return Polygon(cycle());
}

bool Rectangle::contains(const Point& p) const {
    return p.x >= lower_left.x && p.x <= upper_right.x && p.y >= lower_left.y &&
           p.y <= upper_right.y;
}

bool Rectangle::contains(const Segment& s) const {
    return contains(s.start) && contains(s.end);
}

bool Rectangle::contains(const Polygon& polygon) const {
    for (const Point& point : polygon.outer_cycle.points) {
        if (!contains(point)) {
            return false;
        }
    }
    for (const Cycle& inner_cycle : polygon.inner_cycles) {
        for (const Point& point : inner_cycle.points) {
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
