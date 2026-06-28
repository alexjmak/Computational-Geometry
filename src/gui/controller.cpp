#include "gui/controller.hpp"
#include "io/geometry_io.hpp"
#include <QAction>
#include <QApplication>
#include <QDir>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QStatusBar>
#include <QtConcurrent/QtConcurrentRun>
#include <iostream>
#include <utility>

GuiController::GuiController(std::string input_path) : input_path(std::move(input_path)) {
    plot = new Plot(this->input_path.empty() ? "Computational Geometry Viewer" : this->input_path);
    window.setCentralWidget(plot);
    window.resize(900, 650);
    createMenus();
    createLayerPanel();
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
        layer_visibility.assign(current_document.layers.size(), true);
        populateLayerPanel();
        renderCurrentDocument();
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

void GuiController::createLayerPanel() {
    layers_dock = new QDockWidget("Layers", &window);
    layers_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    layers_dock->setMinimumWidth(260);

    QScrollArea* scroll_area = new QScrollArea(layers_dock);
    scroll_area->setWidgetResizable(true);

    layers_widget = new QWidget(scroll_area);
    layers_layout = new QVBoxLayout(layers_widget);
    layers_layout->setContentsMargins(8, 8, 8, 8);
    layers_layout->setSpacing(6);
    layers_widget->setLayout(layers_layout);

    scroll_area->setWidget(layers_widget);
    layers_dock->setWidget(scroll_area);
    window.addDockWidget(Qt::RightDockWidgetArea, layers_dock);
    window.resizeDocks({layers_dock}, {260}, Qt::Horizontal);
    populateLayerPanel();
}

void GuiController::populateLayerPanel() {
    if (layers_layout == nullptr) {
        return;
    }

    while (QLayoutItem* item = layers_layout->takeAt(0)) {
        if (QWidget* widget = item->widget()) {
            widget->deleteLater();
        }
        delete item;
    }

    if (current_document.layers.empty()) {
        QLabel* empty_label = new QLabel("No layers loaded.", layers_widget);
        empty_label->setWordWrap(true);
        layers_layout->addWidget(empty_label);
        layers_layout->addStretch();
        return;
    }

    for (std::size_t i = 0; i < current_document.layers.size(); ++i) {
        const Layer& layer = current_document.layers[i];
        const bool visible = i >= layer_visibility.size() || layer_visibility[i];

        QWidget* row = new QWidget(layers_widget);
        QHBoxLayout* row_layout = new QHBoxLayout(row);
        row_layout->setContentsMargins(0, 0, 0, 0);
        row_layout->setSpacing(6);

        QLabel* name_label = new QLabel(QString::fromStdString(layer.name), row);
        name_label->setWordWrap(true);

        QPushButton* toggle_button = new QPushButton(visible ? "Hide" : "Show", row);
        toggle_button->setCheckable(true);
        toggle_button->setChecked(visible);
        toggle_button->setFixedWidth(72);
        connect(toggle_button, &QPushButton::clicked, this,
                [this, i](bool checked) { setLayerVisible(i, checked); });

        row_layout->addWidget(name_label, 1);
        row_layout->addWidget(toggle_button, 0);
        row->setLayout(row_layout);
        layers_layout->addWidget(row);
    }

    layers_layout->addStretch();
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

    QFileDialog dialog(&window, "Open Geometry YAML", start_directory,
                       "YAML files (*.yaml *.yml);;All files (*)");
    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setViewMode(QFileDialog::Detail);

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    const QStringList selected_files = dialog.selectedFiles();
    if (selected_files.empty()) {
        return;
    }

    const QString file_name = selected_files.first();
    input_path = file_name.toStdString();
    reload();
}

void GuiController::setLayerVisible(std::size_t layer_index, bool visible) {
    if (layer_index >= layer_visibility.size()) {
        return;
    }

    layer_visibility[layer_index] = visible;
    renderCurrentDocument();
    populateLayerPanel();
}

void GuiController::renderCurrentDocument() {
    plot->setDocument(current_document, layer_visibility);
}

void GuiController::updateWindowState() {
    const bool has_input = !input_path.empty();
    const QString title = has_input
                              ? QString("%1 - Computational Geometry Viewer")
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
