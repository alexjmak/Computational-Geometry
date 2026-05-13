#include "gui/controller.hpp"
#include "io/geometry_io.hpp"
#include <QApplication>
#include <QEvent>
#include <QKeyEvent>
#include <QtConcurrent/QtConcurrentRun>
#include <iostream>
#include <utility>

GuiController::GuiController(std::string input_path)
    : input_path(std::move(input_path)), plot(this->input_path) {
    qApp->installEventFilter(this);
    connect(&reload_watcher, &QFutureWatcher<Document>::finished, this,
            &GuiController::finishReload);
    reload();
}

void GuiController::reload() {
    if (reload_in_progress) {
        return;
    }

    reload_in_progress = true;
    reload_watcher.setFuture(QtConcurrent::run([path = input_path] { return loadYaml(path); }));
}

void GuiController::finishReload() {
    reload_in_progress = false;
    try {
        current_document = reload_watcher.result();
        plot.setDocument(current_document);
        plot.show();
        std::cout << "Loaded " << input_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Reload failed: " << e.what() << std::endl;
    }
}

void GuiController::show() {
    plot.show();
}

bool GuiController::eventFilter(QObject* watched, QEvent* event) {
    if (event->type() == QEvent::KeyPress) {
        auto* key_event = static_cast<QKeyEvent*>(event);
        if (key_event->key() == Qt::Key_R && key_event->modifiers() == Qt::NoModifier) {
            reload();
            return true;
        }
    }

    return QObject::eventFilter(watched, event);
}
