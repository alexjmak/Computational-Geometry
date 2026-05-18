#include "io/document.hpp"
#include "io/geometry_io.hpp"
#include "python/bindings.hpp"
#include <pybind11/stl.h>
#include <string>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace {

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

void bindDocument(py::module_& module) {
    py::class_<Layer>(module, "Layer")
        .def(py::init<>())
        .def_readwrite("name", &Layer::name)
        .def_readwrite("polygons", &Layer::polygons)
        .def_readwrite("segments", &Layer::segments)
        .def_readwrite("points", &Layer::points)
        .def("filter", &Layer::filter)
        .def("__eq__", &Layer::operator==);

    py::class_<Document>(module, "Document")
        .def(py::init<>())
        .def_readwrite("layers", &Document::layers)
        .def("collect_points", &Document::collectPoints)
        .def("collect_segments", &Document::collectSegments)
        .def("add_layer", &addLayer, py::arg("name"), py::arg("points") = std::vector<Point>{},
             py::arg("segments") = std::vector<Segment>{},
             py::arg("polygons") = std::vector<Polygon>{})
        .def("__eq__", &Document::operator==);

    module.def("load_yaml", &loadYaml, py::arg("path"));
    module.def("save_yaml", &saveYaml, py::arg("document"), py::arg("path"));
}
