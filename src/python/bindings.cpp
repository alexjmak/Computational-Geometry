#include "algorithms/convex_hull.hpp"
#include "algorithms/line_segment_intersection.hpp"
#include "geometry/random.hpp"
#include "io/geometry_io.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <optional>
#include <random>
#include <string>
#include <unordered_set>
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

namespace {

std::vector<Point> sortedPoints(std::unordered_set<Point> points) {
    std::vector<Point> ret(points.begin(), points.end());
    std::ranges::sort(ret);
    return ret;
}

std::vector<Point> lineSegmentIntersectionPython(const std::vector<Segment>& segments,
                                                 const std::string& algorithm) {
    if (algorithm == "line-sweep") {
        return sortedPoints(lineSegmentIntersection(segments));
    }
    if (algorithm == "brute-force") {
        return sortedPoints(bruteForceLineSegmentIntersection(segments));
    }
    throw py::value_error("algorithm must be 'line-sweep' or 'brute-force'");
}

std::vector<Point> randomPointsPython(int count, std::optional<unsigned int> seed, int min_coord,
                                      int max_coord) {
    if (seed.has_value()) {
        std::mt19937 rng(*seed);
        return randomPoints(count, min_coord, max_coord, &rng);
    }
    return randomPoints(count, min_coord, max_coord);
}

std::vector<Segment> randomSegmentsPython(int count, std::optional<unsigned int> seed,
                                          int min_length, int max_length, int min_coord,
                                          int max_coord) {
    if (seed.has_value()) {
        std::mt19937 rng(*seed);
        return randomSegments(count, min_length, max_length, min_coord, max_coord, &rng);
    }
    return randomSegments(count, min_length, max_length, min_coord, max_coord);
}

void addLayer(Document& document, std::string name, std::vector<Point> points,
              std::vector<Segment> segments, std::vector<Polygon> polygons) {
    Layer layer;
    layer.name = std::move(name);
    layer.points = std::move(points);
    layer.segments = std::move(segments);
    layer.polygons = std::move(polygons);
    document.layers.push_back(std::move(layer));
}

} // namespace

PYBIND11_MODULE(cgeom, m) {
    m.doc() = "Python bindings for the Computational Geometry library";
    py::object total_ordering = py::module_::import("functools").attr("total_ordering");

    py::class_<Vect2>(m, "Vect2")
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
    total_ordering(m.attr("Vect2"));
    m.attr("Point") = m.attr("Vect2");

    py::class_<Segment>(m, "Segment")
        .def(py::init<Point, Point>(), py::arg("start"), py::arg("end"))
        .def_readwrite("start", &Segment::start)
        .def_readwrite("end", &Segment::end)
        .def("canonicalized_x", &Segment::canonicalizedX)
        .def("canonicalized_y", &Segment::canonicalizedY)
        .def("__repr__", [](const Segment& segment) { return segment.toString(); })
        .def("__eq__", &Segment::operator==)
        .def("__lt__", &Segment::operator<);
    total_ordering(m.attr("Segment"));

    py::class_<Cycle>(m, "Cycle")
        .def(py::init<std::vector<Point>>(), py::arg("points"))
        .def_readwrite("points", &Cycle::points)
        .def("signed_area", &Cycle::signedArea)
        .def("area", &Cycle::area)
        .def("is_outer", &Cycle::isOuter)
        .def("segments", &Cycle::segments)
        .def("__eq__", &Cycle::operator==);

    py::class_<Polygon>(m, "Polygon")
        .def(py::init<Cycle, std::vector<Cycle>>(), py::arg("outer_cycle"),
             py::arg("inner_cycles") = std::vector<Cycle>{})
        .def_readwrite("outer_cycle", &Polygon::outer_cycle)
        .def_readwrite("inner_cycles", &Polygon::inner_cycles)
        .def("area", &Polygon::area)
        .def("__eq__", &Polygon::operator==);

    py::class_<Rectangle>(m, "Rectangle")
        .def(py::init<Point, Point>(), py::arg("lower_left"), py::arg("upper_right"))
        .def_readwrite("lower_left", &Rectangle::lower_left)
        .def_readwrite("upper_right", &Rectangle::upper_right)
        .def("cycle", &Rectangle::cycle)
        .def("polygon", &Rectangle::polygon)
        .def("contains", py::overload_cast<const Point&>(&Rectangle::contains, py::const_))
        .def("contains", py::overload_cast<const Segment&>(&Rectangle::contains, py::const_))
        .def("contains", py::overload_cast<const Polygon&>(&Rectangle::contains, py::const_))
        .def("area", &Rectangle::area);

    py::class_<Layer>(m, "Layer")
        .def(py::init<>())
        .def_readwrite("name", &Layer::name)
        .def_readwrite("polygons", &Layer::polygons)
        .def_readwrite("segments", &Layer::segments)
        .def_readwrite("points", &Layer::points)
        .def("filter", &Layer::filter)
        .def("__eq__", &Layer::operator==);

    py::class_<Document>(m, "Document")
        .def(py::init<>())
        .def_readwrite("layers", &Document::layers)
        .def("collect_points", &Document::collectPoints)
        .def("collect_segments", &Document::collectSegments)
        .def("add_layer", &addLayer, py::arg("name"), py::arg("points") = std::vector<Point>{},
             py::arg("segments") = std::vector<Segment>{},
             py::arg("polygons") = std::vector<Polygon>{})
        .def("__eq__", &Document::operator==);

    m.def("load_yaml", &loadYaml, py::arg("path"));
    m.def("save_yaml", &saveYaml, py::arg("document"), py::arg("path"));
    m.def("convex_hull", &convexHull, py::arg("points"));
    m.def("line_segment_intersection", &lineSegmentIntersectionPython, py::arg("segments"),
          py::arg("algorithm") = "line-sweep");
    m.def("random_points", &randomPointsPython, py::arg("count"), py::arg("seed") = py::none(),
          py::arg("min_coord") = 10, py::arg("max_coord") = 100);
    m.def("random_segments", &randomSegmentsPython, py::arg("count"), py::arg("seed") = py::none(),
          py::arg("min_length") = 1, py::arg("max_length") = 40, py::arg("min_coord") = 1,
          py::arg("max_coord") = 1000);
}
