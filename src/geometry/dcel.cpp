#include "geometry/dcel.hpp"

DCEL::Point::Point(Rational x, Rational y) : ::Point(x, y), half_edge(nullptr) {}

DCEL::HalfEdge::HalfEdge()
    : origin(nullptr), twin(nullptr), next(nullptr), prev(nullptr), face(nullptr) {}

DCEL::Face::Face() : outer_component(nullptr), inner_components() {}
