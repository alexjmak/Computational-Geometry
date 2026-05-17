#include "python/bindings.hpp"

PYBIND11_MODULE(cgeom, m) {
    m.doc() = "Python bindings for the Computational Geometry library";
    bindGeometry(m);
    bindDocument(m);
    bindGui(m);
    bindAlgorithms(m);
}
