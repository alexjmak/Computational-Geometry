#include "gui/controller.hpp"
#include "io/geometry_io.hpp"
#include <QAction>
#include <QApplication>
#include <QDir>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeyEvent>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QtConcurrent/QtConcurrentRun>
#include <iostream>
#include <utility>

GuiController::GuiController(std::string input_path)
    : input_path(std::move(input_path)) {
    plot = new Plot(this->input_path.empty() ? "Computational Geometry Viewer" : this->input_path);
    window.setCentralWidget(plot);
    window.resize(900, 650);
    createMenus();
    updateWindowState();

    qApp->installEventFilter(this);
    connect(&reload_watcher, &QFutureWatcher<Document>::finished, this,
            &GuiController::finishReload);
    if (!this->input_path.empty()) {
        reload();
    }
}

void GuiController::reload() {
    if (reload_in_progress) {
        return;
    }
    if (input_path.empty()) {
        window.statusBar()->showMessage("Open a YAML geometry document to begin.");
        return;
    }

    reload_in_progress = true;
    loading_path = input_path;
    updateWindowState();
    reload_watcher.setFuture(QtConcurrent::run([path = loading_path] { return loadYaml(path); }));
}

void GuiController::finishReload() {
    reload_in_progress = false;
    try {
        current_document = reload_watcher.result();
        plot->setDocument(current_document);
        plot->show();
        std::cout << "Loaded " << loading_path << std::endl;
        window.statusBar()->showMessage(
            QString("Loaded %1").arg(QString::fromStdString(loading_path)), 5000);
    } catch (const std::exception& e) {
        std::cerr << "Reload failed: " << e.what() << std::endl;
        window.statusBar()->showMessage(QString("Load failed: %1").arg(e.what()), 8000);
        QMessageBox::warning(&window, "Load Failed",
                             QString("Could not load '%1'.\n\n%2")
                                 .arg(QString::fromStdString(loading_path), e.what()));
    }
    loading_path.clear();
    updateWindowState();
}

void GuiController::show() {
    window.show();
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

void GuiController::createMenus() {
    window.menuBar()->setNativeMenuBar(false);
    QMenu* file_menu = window.menuBar()->addMenu("&File");

    open_action = file_menu->addAction("&Open...");
    open_action->setShortcut(QKeySequence::Open);
    connect(open_action, &QAction::triggered, this, [this] { openFile(); });

    reload_action = file_menu->addAction("&Reload");
    reload_action->setShortcut(QKeySequence::Refresh);
    connect(reload_action, &QAction::triggered, this, [this] { reload(); });

    file_menu->addSeparator();
    exit_action = file_menu->addAction("E&xit");
    exit_action->setShortcut(QKeySequence::Quit);
    connect(exit_action, &QAction::triggered, &window, &QMainWindow::close);

    QMenu* help_menu = window.menuBar()->addMenu("&Help");
    about_action = help_menu->addAction("&About");
    connect(about_action, &QAction::triggered, this, [this] {
        QMessageBox::about(&window, "About Computational Geometry Viewer",
                           "Computational Geometry Viewer\n\n"
                           "Open YAML geometry documents and inspect their layers.");
    });
}

void GuiController::openFile() {
    if (reload_in_progress) {
        window.statusBar()->showMessage("Please wait for the current load to finish.", 3000);
        return;
    }

    QString start_directory = QDir::currentPath();
    if (!input_path.empty()) {
        start_directory = QFileInfo(QString::fromStdString(input_path)).absolutePath();
    }

    const QString file_name = QFileDialog::getOpenFileName(
        &window, "Open Geometry YAML", start_directory,
        "YAML files (*.yaml *.yml);;All files (*)");
    if (file_name.isEmpty()) {
        return;
    }

    input_path = file_name.toStdString();
    reload();
}

void GuiController::updateWindowState() {
    const bool has_input = !input_path.empty();
    const QString title =
        has_input ? QString("%1 - Computational Geometry Viewer")
                        .arg(QFileInfo(QString::fromStdString(input_path)).fileName())
                  : QString("Computational Geometry Viewer");
    window.setWindowTitle(title);

    if (reload_action != nullptr) {
        reload_action->setEnabled(has_input && !reload_in_progress);
    }
    if (open_action != nullptr) {
        open_action->setEnabled(!reload_in_progress);
    }

    if (reload_in_progress) {
        window.statusBar()->showMessage(
            QString("Loading %1...").arg(QString::fromStdString(loading_path)));
    } else if (!has_input) {
        window.statusBar()->showMessage("Open a YAML geometry document to begin.");
    }
}
