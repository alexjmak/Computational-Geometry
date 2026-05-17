#define QT_NO_KEYWORDS

#include "gui/controller.hpp"
#include "python/bindings.hpp"
#include <QApplication>
#include <string>

namespace py = pybind11;

namespace {

int showPython(std::string file) {
    int argc = 0;
    char* argv[] = {nullptr};
    QApplication app(argc, argv);

    GuiController controller(file);
    controller.show();
    return app.exec();
}

int showDocumentPython(const Document& document) {
    int argc = 0;
    char* argv[] = {nullptr};
    QApplication app(argc, argv);

    Plot plot("Document");
    plot.setDocument(document);
    plot.show();
    return app.exec();
}

} // namespace

void bindGui(py::module_& module) {
    module.def("show", &showPython, py::arg("file"));
    module.def("show", &showDocumentPython, py::arg("document"));
}
