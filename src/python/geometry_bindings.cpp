#include "python/bindings.hpp"

#include "geometry/polygon.hpp"

#include <pybind11/stl.h>

#include <vector>

namespace py = pybind11;

namespace pybind11::detail {

template <> struct type_caster<Rational> {
    PYBIND11_TYPE_CASTER(Rational, const_name("fractions.Fraction"));

    bool load(handle src, bool) {
        if (PyLong_Check(src.ptr())) {
            value = Rational(PyLong_AsLongLong(src.ptr()));
            return !PyErr_Occurred();
        }

        py::object object = py::reinterpret_borrow<py::object>(src);
        if (!py::hasattr(object, "numerator") || !py::hasattr(object, "denominator")) {
            return false;
        }

        long long numerator = py::cast<long long>(object.attr("numerator"));
        long long denominator = py::cast<long long>(object.attr("denominator"));
        if (denominator == 0) {
            return false;
        }

        value = Rational(numerator, denominator);
        return true;
    }

    static handle cast(const Rational& rational, return_value_policy, handle) {
        py::object fraction = py::module_::import("fractions").attr("Fraction");
        return fraction(rational.numerator(), rational.denominator()).release();
    }
};

} // namespace pybind11::detail

void bindGeometry(py::module_& module) {
    py::object total_ordering = py::module_::import("functools").attr("total_ordering");

    py::class_<Vect2>(module, "Vect2")
        .def(py::init<Rational, Rational>(), py::arg("x"), py::arg("y"))
        .def_readwrite("x", &Vect2::x)
        .def_readwrite("y", &Vect2::y)
        .def("__repr__", [](const Vect2& vect2) { return vect2.toString(); })
        .def("__eq__", &Vect2::operator==)
        .def("__lt__", &Vect2::operator<)
        .def("__add__", &Vect2::operator+)
        .def("__sub__", [](const Vect2& lhs, const Vect2& rhs) { return lhs - rhs; })
        .def("__mul__", &Vect2::operator*, py::is_operator())
        .def("__neg__", [](const Vect2& vect2) { return -vect2; });
    total_ordering(module.attr("Vect2"));
    module.attr("Point") = module.attr("Vect2");

    py::class_<Segment>(module, "Segment")
        .def(py::init<Point, Point>(), py::arg("start"), py::arg("end"))
        .def_readwrite("start", &Segment::start)
        .def_readwrite("end", &Segment::end)
        .def("canonicalized_x", &Segment::canonicalizedX)
        .def("canonicalized_y", &Segment::canonicalizedY)
        .def("__repr__", [](const Segment& segment) { return segment.toString(); })
        .def("__eq__", &Segment::operator==)
        .def("__lt__", &Segment::operator<);
    total_ordering(module.attr("Segment"));

    py::class_<Cycle>(module, "Cycle")
        .def(py::init<std::vector<Point>>(), py::arg("points"))
        .def_readwrite("points", &Cycle::points)
        .def("signed_area", &Cycle::signedArea)
        .def("area", &Cycle::area)
        .def("is_outer", &Cycle::isOuter)
        .def("segments", &Cycle::segments)
        .def("__eq__", &Cycle::operator==);

    py::class_<Polygon>(module, "Polygon")
        .def(py::init<Cycle, std::vector<Cycle>>(), py::arg("outer_cycle"),
             py::arg("inner_cycles") = std::vector<Cycle>{})
        .def_readwrite("outer_cycle", &Polygon::outer_cycle)
        .def_readwrite("inner_cycles", &Polygon::inner_cycles)
        .def("cycles",
             [](const Polygon& polygon) {
                 std::vector<Cycle> cycles;
                 cycles.push_back(polygon.outer_cycle);
                 cycles.insert(cycles.end(), polygon.inner_cycles.begin(),
                               polygon.inner_cycles.end());
                 return cycles;
             })
        .def("area", &Polygon::area)
        .def("__eq__", &Polygon::operator==);

    py::class_<Rectangle>(module, "Rectangle")
        .def(py::init<Point, Point>(), py::arg("lower_left"), py::arg("upper_right"))
        .def_readwrite("lower_left", &Rectangle::lower_left)
        .def_readwrite("upper_right", &Rectangle::upper_right)
        .def("cycle", &Rectangle::cycle)
        .def("polygon", &Rectangle::polygon)
        .def("contains", py::overload_cast<const Point&>(&Rectangle::contains, py::const_))
        .def("contains", py::overload_cast<const Segment&>(&Rectangle::contains, py::const_))
        .def("contains", py::overload_cast<const Polygon&>(&Rectangle::contains, py::const_))
        .def("area", &Rectangle::area);
}
