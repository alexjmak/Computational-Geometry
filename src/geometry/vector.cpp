#include "geometry/vector.hpp"

Vect2::Vect2(Rational x, Rational y) : x(x), y(y) {}

Vect2::Vect2(int x, int y) : x(x), y(y) {}

bool Vect2::operator==(const Vect2& other) const {
    return x == other.x && y == other.y;
}

bool Vect2::operator<(const Vect2& other) const {
    if (x == other.x) {
        return y < other.y;
    }
    return x < other.x;
}

Vect2 Vect2::operator+(const Vect2& other) const {
    return Vect2(x + other.x, y + other.y);
}

Vect2 Vect2::operator-(const Vect2& other) const {
    return Vect2(x - other.x, y - other.y);
}

Vect2 Vect2::operator*(Rational scalar) const {
    return Vect2(x * scalar, y * scalar);
}

Vect2 Vect2::operator-() const {
    return Vect2(-x, -y);
}

std::string Vect2::toString() const {
    return "(" + std::to_string(boost::rational_cast<double>(x)) + ", " +
           std::to_string(boost::rational_cast<double>(y)) + ")";
}

size_t std::hash<Vect2>::operator()(const Vect2& p) const {
    size_t h = 0;

    auto combine = [&](size_t v) { h ^= v + 0x9e3779b9 + (h << 6) + (h >> 2); };

    combine(std::hash<long long>{}(p.x.numerator()));
    combine(std::hash<long long>{}(p.x.denominator()));
    combine(std::hash<long long>{}(p.y.numerator()));
    combine(std::hash<long long>{}(p.y.denominator()));

    return h;
}
