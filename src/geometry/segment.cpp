#include "geometry/segment.hpp"

Segment::Segment(Point start, Point end) : start(start), end(end) {}

bool Segment::isCanonicalizedX() const {
    return start <= end;
}

bool Segment::isCanonicalizedY() const {
    if (start.y == end.y) {
        return start >= end;
    }
    return start.y < end.y;
}

Segment Segment::canonicalizedX() const {
    if (isCanonicalizedX()) {
        return *this;
    }
    return reversed();
}

Segment Segment::canonicalizedY() const {
    if (isCanonicalizedY()) {
        return *this;
    }
    return reversed();
}

Segment Segment::reversed() const {
    return Segment(end, start);
}

bool Segment::operator==(const Segment& other) const {
    return start == other.start && end == other.end;
}

bool Segment::operator<(const Segment& other) const {
    if (start == other.start) {
        return end < other.end;
    }
    return start < other.start;
}

std::string Segment::toString() const {
    return "S(" + start.toString() + ", " + end.toString() + ")";
}

size_t std::hash<Segment>::operator()(const Segment& s) const {
    size_t h = 0;

    auto combine = [&](size_t v) { h ^= v + 0x9e3779b9 + (h << 6) + (h >> 2); };

    combine(std::hash<Vect2>{}(s.start));
    combine(std::hash<Vect2>{}(s.end));

    return h;
}
