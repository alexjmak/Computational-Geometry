#include "geometry/polygon.hpp"
#include "geometry/predicates.hpp"
#include <cmath>

Cycle::Cycle(std::vector<Point> points) : points(points) {}

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

Polygon::Polygon(std::vector<Cycle> cycles) : cycles(cycles) {}

double Polygon::area() const {
    double ret = 0.0;
    for (const auto& cycle : cycles) {
        ret += cycle.signedArea();
    }
    return ret;
}

bool Polygon::operator==(const Polygon& other) const {
    return cycles == other.cycles;
}

Rectangle::Rectangle(Point lower_left, Point upper_right)
    : lower_left(lower_left), upper_right(upper_right) {}

Cycle Rectangle::cycle() const {
    Point lower_right(upper_right.x, lower_left.y);
    Point upper_left(lower_left.x, upper_right.y);
    return Cycle({lower_left, lower_right, upper_right, upper_left});
}

Polygon Rectangle::polygon() const {
    return Polygon({cycle()});
}

bool Rectangle::contains(const Point& p) const {
    return p.x >= lower_left.x && p.x <= upper_right.x && p.y >= lower_left.y &&
           p.y <= upper_right.y;
}

bool Rectangle::contains(const Segment& s) const {
    return contains(s.start) && contains(s.end);
}

bool Rectangle::contains(const Polygon& polygon) const {
    for (const Cycle& cycle : polygon.cycles) {
        for (const Point& point : cycle.points) {
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
