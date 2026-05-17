#ifndef CGEOM_PYTHON_BINDINGS_HPP
#define CGEOM_PYTHON_BINDINGS_HPP

#include <pybind11/pybind11.h>

void bindGeometry(pybind11::module_& module);
void bindDocument(pybind11::module_& module);
void bindAlgorithms(pybind11::module_& module);
void bindGui(pybind11::module_& module);

#endif // CGEOM_PYTHON_BINDINGS_HPP
