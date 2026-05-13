#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "gui/plot.hpp"
#include "io/document.hpp"
#include <QObject>
#include <QFutureWatcher>
#include <string>

/// \brief Controller that owns a plot window and reloads it from YAML on demand.
class GuiController : public QObject {
  public:
    /// \brief Create a controller for one YAML-backed plot.
    /// \param input_path The YAML document path to load and reload.
    explicit GuiController(std::string input_path);

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
    /// \brief Apply the completed reload result on the GUI thread.
    void finishReload();

    std::string input_path; ///< The YAML document currently backing the plot.
    Document current_document; ///< The currently loaded document model.
    Plot plot;             ///< The plot window controlled by this object.
    QFutureWatcher<Document> reload_watcher; ///< Watches asynchronous YAML reload work.
    bool reload_in_progress = false; ///< Whether a reload is currently running.
};

#endif // CONTROLLER_HPP
