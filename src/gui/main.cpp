#include "gui/controller.hpp"
#include <QApplication>
#include <exception>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    if (argc > 2) {
        std::cerr << "Usage: " << argv[0] << " [input]" << std::endl;
        return 1;
    }

    QApplication app(argc, argv);

    try {
        GuiController controller(argc == 2 ? argv[1] : "");
        controller.show();
        return app.exec();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
