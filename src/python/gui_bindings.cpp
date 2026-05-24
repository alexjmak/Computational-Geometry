#define QT_NO_KEYWORDS

#include "gui/controller.hpp"
#include "python/bindings.hpp"
#include <QApplication>
#include <pybind11/stl.h>
#include <string>

namespace py = pybind11;

namespace {

QApplication* ensureQApplication() {
    if (QApplication::instance() != nullptr) {
        return qobject_cast<QApplication*>(QApplication::instance());
    }

    static int argc = 1;
    static char app_name[] = "cgeom";
    static char* argv[] = {app_name, nullptr};
    static QApplication* application = new QApplication(argc, argv);
    return application;
}

int showPython(std::string file) {
    QApplication* app = ensureQApplication();
    GuiController controller(file);
    controller.show();
    return app->exec();
}

int showDocumentPython(const Document& document) {
    QApplication* app = ensureQApplication();
    Plot plot("Document");
    plot.setDocument(document);
    plot.show();
    return app->exec();
}

int showPlotPython(Plot& plot) {
    QApplication* app = ensureQApplication();
    plot.show();
    return app->exec();
}

} // namespace

void bindGui(py::module_& module) {
    py::class_<Plot>(module, "Plot")
        .def(py::init([](const std::string& title, const std::string& x_label,
                         const std::string& y_label) {
                 ensureQApplication();
                 return std::make_unique<Plot>(title, x_label, y_label);
             }),
             py::arg("title") = "", py::arg("x_label") = "x", py::arg("y_label") = "y")
        .def("add_points", &Plot::addPoints, py::arg("points"), py::arg("color") = "red",
             py::arg("s") = 5)
        .def("add_segments", &Plot::addSegments, py::arg("segments"), py::arg("color") = "blue",
             py::arg("line_width") = 1, py::arg("show_arrows") = true)
        .def("add_ring", &Plot::addRing, py::arg("ring"), py::arg("face_color") = "lightblue",
             py::arg("edge_color") = "blue", py::arg("alpha") = 0.35)
        .def("add_polygon", &Plot::addPolygon, py::arg("polygon"),
             py::arg("face_color") = "lightblue", py::arg("edge_color") = "blue",
             py::arg("alpha") = 0.35)
        .def("set_document", &Plot::setDocument, py::arg("document"))
        .def("clear", &Plot::clear)
        .def("show", &showPlotPython);

    module.def("show", &showPython, py::arg("file"));
    module.def("show", &showDocumentPython, py::arg("document"));
}
