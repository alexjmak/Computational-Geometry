#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "gui/plot.hpp"
#include "io/document.hpp"
#include <QAction>
#include <QFutureWatcher>
#include <QMainWindow>
#include <QObject>
#include <string>

/// \brief Controller that owns a plot window and reloads it from YAML on demand.
class GuiController : public QObject {
  public:
    /// \brief Create a controller for one YAML-backed plot.
    /// \param input_path The YAML document path to load and reload. Empty opens a blank viewer.
    explicit GuiController(std::string input_path = "");

    /// \brief Start loading the YAML document and redraw the plot when loading finishes.
    void reload();

    /// \brief Show the controlled plot window.
    void show();

  protected:
    /// \brief Reload the plot when the user presses R.
    /// \param watched The object receiving the event.
    /// \param event The event to inspect.
    /// \returns True when the reload key was handled, otherwise false.
    bool eventFilter(QObject* watched, QEvent* event) override;

  private:
    /// \brief Build the menu bar and window-level actions.
    void createMenus();

    /// \brief Ask the user for a YAML document and load it.
    void openFile();

    /// \brief Apply the completed reload result on the GUI thread.
    void finishReload();

    /// \brief Refresh title, status text, and action enabled state.
    void updateWindowState();

    std::string input_path; ///< The YAML document currently backing the plot.
    std::string loading_path; ///< The YAML document currently being loaded.
    Document current_document; ///< The currently loaded document model.
    QMainWindow window;    ///< The top-level application window.
    Plot* plot = nullptr;  ///< The plot widget controlled by this object.
    QAction* open_action = nullptr;   ///< Opens a YAML document from disk.
    QAction* reload_action = nullptr; ///< Reloads the current YAML document.
    QAction* exit_action = nullptr;   ///< Closes the application window.
    QAction* about_action = nullptr;  ///< Shows application information.
    QFutureWatcher<Document> reload_watcher; ///< Watches asynchronous YAML reload work.
    bool reload_in_progress = false; ///< Whether a reload is currently running.
};

#endif // CONTROLLER_HPP
