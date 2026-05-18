#ifndef PLOT_HPP
#define PLOT_HPP

#include "geometry/polygon.hpp"
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtGui/QResizeEvent>
#include <QtWidgets/QGraphicsPolygonItem>
#include <QtWidgets/QWidget>
#include <optional>
#include <string>
#include <vector>

QT_USE_NAMESPACE

struct Document;

/// \brief Qt widget for plotting exact geometry primitives.
class Plot : public QWidget {
  public:
    /// \brief Create a Qt chart widget configured for geometry plots.
    /// \param title The plot title.
    /// \param x_label The x-axis label.
    /// \param y_label The y-axis label.
    Plot(const std::string& title = "", const std::string& x_label = "x",
         const std::string& y_label = "y");

    /// \brief Scatter plot a collection of points on the current chart.
    /// \param points The points to draw.
    /// \param color The marker color.
    /// \param s The marker size.
    void addPoints(const std::vector<Point>& points, const std::string& color = "red", int s = 5);

    /// \brief Draw each segment in the iterable as a straight line on the chart.
    /// \param segments The segments to draw.
    /// \param color The line color.
    /// \param line_width The width of each plotted segment.
    void addSegments(const std::vector<Segment>& segments, const std::string& color = "blue",
                     int line_width = 1);

    /// \brief Draw a single closed cycle as a polygonal region.
    /// \param cycle The cycle to draw.
    /// \param face_color The fill color.
    /// \param edge_color The boundary color.
    /// \param alpha The fill opacity.
    void addCycle(const Cycle& cycle, const std::string& face_color = "lightblue",
                  const std::string& edge_color = "blue", double alpha = 0.35);

    /// \brief Draw the outer cycle of a polygon.
    /// \param polygon The polygon to draw.
    /// \param face_color The fill color.
    /// \param edge_color The boundary color.
    /// \param alpha The fill opacity.
    void addPolygon(const Polygon& polygon, const std::string& face_color = "lightblue",
                    const std::string& edge_color = "blue", double alpha = 0.35);

    /// \brief Replace the current plot contents with every layer in a document.
    /// \param document The document to render.
    void setDocument(const Document& document);

    /// \brief Remove all plotted geometry and reset the tracked bounds.
    void clear();

    /// \brief Show the plot widget.
    void show();

  protected:
    /// \brief Reposition filled cycle overlays after the plot is resized.
    /// \param event The Qt resize event.
    void resizeEvent(QResizeEvent* event) override;

  private:
    /// \brief Filled polygon overlay tracked in data coordinates.
    struct CycleFill {
        std::vector<Point> points;  ///< The cycle vertices in data coordinates.
        QGraphicsPolygonItem* item; ///< The scene item used to render the fill.
    };

    /// \brief Expand the tracked plot bounds to include a point.
    /// \param point The point to include.
    void includePoint(const Point& point);

    /// \brief Recompute chart axes using the tracked bounds and 10 percent padding.
    void updateAxes();

    /// \brief Reposition all cycle fill scene items after axes or geometry changes.
    void updateCycleFills();

    QChart* chart;                      ///< The Qt chart storing plotted series.
    QChartView* chartView;              ///< The Qt widget used to display the chart.
    std::optional<double> min_x;        ///< The minimum plotted x coordinate.
    std::optional<double> max_x;        ///< The maximum plotted x coordinate.
    std::optional<double> min_y;        ///< The minimum plotted y coordinate.
    std::optional<double> max_y;        ///< The maximum plotted y coordinate.
    std::vector<CycleFill> cycle_fills; ///< Filled cycle overlays in data coordinates.
};

#endif // PLOT_HPP
