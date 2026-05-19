#include "gui/plot.hpp"
#include "io/document.hpp"
#include <QApplication>
#include <QPainterPath>
#include <QVBoxLayout>
#include <QtCharts/QValueAxis>
#include <algorithm>

namespace {

double paddedMin(double min_value, double max_value) {
    double span = max_value - min_value;
    if (span == 0.0) {
        span = std::max(1.0, std::abs(min_value));
    }
    return min_value - span * 0.05;
}

double paddedMax(double min_value, double max_value) {
    double span = max_value - min_value;
    if (span == 0.0) {
        span = std::max(1.0, std::abs(max_value));
    }
    return max_value + span * 0.05;
}

} // namespace

Plot::Plot(const std::string& title, const std::string& x_label, const std::string& y_label)
    : chart(nullptr), chartView(nullptr), min_x(), max_x(), min_y(), max_y(), cycle_fills() {
    chart = new QChart();
    chart->setTitle(QString::fromStdString(title));
    chart->setAnimationOptions(QChart::SeriesAnimations);
    chart->legend()->hide();
    connect(chart, &QChart::plotAreaChanged, this, [this](const QRectF&) { updateCycleFills(); });

    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(chartView);
    setLayout(layout);
    setWindowTitle(QString::fromStdString(title));
    resize(800, 600);
}

void Plot::addPoints(const std::vector<Point>& points, const std::string& color, int s) {
    if (points.empty())
        return;
    QScatterSeries* series = new QScatterSeries();
    series->setBorderColor(Qt::transparent);
    series->setMarkerSize(s);
    series->setColor(QColor(QString::fromStdString(color)));
    for (const auto& p : points) {
        series->append(boost::rational_cast<double>(p.x), boost::rational_cast<double>(p.y));
        includePoint(p);
    }
    chart->addSeries(series);
    updateAxes();
}

void Plot::addSegments(const std::vector<Segment>& segments, const std::string& color,
                       int line_width) {
    for (const auto& seg : segments) {
        QLineSeries* series = new QLineSeries();
        series->setColor(QColor(QString::fromStdString(color)));
        series->setPen(QPen(QColor(QString::fromStdString(color)), line_width));
        series->append(boost::rational_cast<double>(seg.start.x),
                       boost::rational_cast<double>(seg.start.y));
        series->append(boost::rational_cast<double>(seg.end.x),
                       boost::rational_cast<double>(seg.end.y));
        includePoint(seg.start);
        includePoint(seg.end);
        chart->addSeries(series);
    }
    updateAxes();
}

void Plot::addCycle(const Cycle& cycle, const std::string& face_color,
                    const std::string& edge_color, double alpha) {
    if (cycle.points.empty())
        return;
    addCycleBoundary(cycle, edge_color);
    addCycleFill({cycle.points}, face_color, alpha);
    updateAxes();
}

void Plot::addPolygon(const Polygon& polygon, const std::string& face_color,
                      const std::string& edge_color, double alpha) {
    std::vector<std::vector<Point>> fill_cycles;
    fill_cycles.push_back(polygon.outer_cycle.points);
    addCycleBoundary(polygon.outer_cycle, edge_color);

    for (const Cycle& inner_cycle : polygon.inner_cycles) {
        fill_cycles.push_back(inner_cycle.points);
        addCycleBoundary(inner_cycle, edge_color);
    }

    addCycleFill(fill_cycles, face_color, alpha);
    updateAxes();
}

void Plot::addCycleBoundary(const Cycle& cycle, const std::string& edge_color) {
    if (cycle.points.empty())
        return;
    QLineSeries* series = new QLineSeries();
    series->setColor(QColor(QString::fromStdString(edge_color)));
    for (const auto& p : cycle.points) {
        series->append(boost::rational_cast<double>(p.x), boost::rational_cast<double>(p.y));
        includePoint(p);
    }
    // Close the cycle
    series->append(boost::rational_cast<double>(cycle.points[0].x),
                   boost::rational_cast<double>(cycle.points[0].y));
    chart->addSeries(series);
}

void Plot::addCycleFill(const std::vector<std::vector<Point>>& cycles,
                        const std::string& face_color, double alpha) {
    QColor fill_color(QString::fromStdString(face_color));
    fill_color.setAlphaF(std::clamp(alpha, 0.0, 1.0));
    QGraphicsPathItem* fill_item = new QGraphicsPathItem(chart);
    fill_item->setBrush(QBrush(fill_color));
    fill_item->setPen(Qt::NoPen);
    fill_item->setZValue(-1.0);
    cycle_fills.push_back({cycles, fill_item});
}

void Plot::setDocument(const Document& document) {
    clear();
    for (const Layer& layer : document.layers) {
        if (!layer.segments.empty()) {
            addSegments(layer.segments);
        }
        if (!layer.points.empty()) {
            addPoints(layer.points);
        }
        for (const Polygon& polygon : layer.polygons) {
            addPolygon(polygon);
        }
    }
}

void Plot::clear() {
    chart->removeAllSeries();
    for (CycleFill& fill : cycle_fills) {
        delete fill.item;
    }
    cycle_fills.clear();
    min_x.reset();
    max_x.reset();
    min_y.reset();
    max_y.reset();

    const auto axes = chart->axes();
    for (QAbstractAxis* axis : axes) {
        chart->removeAxis(axis);
        delete axis;
    }
}

void Plot::show() {
    QWidget::show();
    updateCycleFills();
}

void Plot::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    updateCycleFills();
}

void Plot::includePoint(const Point& point) {
    double x = boost::rational_cast<double>(point.x);
    double y = boost::rational_cast<double>(point.y);

    min_x = min_x ? std::min(*min_x, x) : x;
    max_x = max_x ? std::max(*max_x, x) : x;
    min_y = min_y ? std::min(*min_y, y) : y;
    max_y = max_y ? std::max(*max_y, y) : y;
}

void Plot::updateAxes() {
    chart->createDefaultAxes();
    if (!min_x || !max_x || !min_y || !max_y) {
        return;
    }

    QList<QAbstractAxis*> horizontal_axes = chart->axes(Qt::Horizontal);
    QList<QAbstractAxis*> vertical_axes = chart->axes(Qt::Vertical);
    if (!horizontal_axes.empty()) {
        if (auto* axis = qobject_cast<QValueAxis*>(horizontal_axes.first())) {
            axis->setRange(paddedMin(*min_x, *max_x), paddedMax(*min_x, *max_x));
        }
    }
    if (!vertical_axes.empty()) {
        if (auto* axis = qobject_cast<QValueAxis*>(vertical_axes.first())) {
            axis->setRange(paddedMin(*min_y, *max_y), paddedMax(*min_y, *max_y));
        }
    }

    updateCycleFills();
}

void Plot::updateCycleFills() {
    for (CycleFill& fill : cycle_fills) {
        QPainterPath path;
        path.setFillRule(Qt::OddEvenFill);

        for (const std::vector<Point>& cycle : fill.cycles) {
            if (cycle.empty()) {
                continue;
            }

            QPointF first_point =
                chart->mapToPosition(QPointF(boost::rational_cast<double>(cycle[0].x),
                                             boost::rational_cast<double>(cycle[0].y)));
            path.moveTo(first_point);

            for (std::size_t i = 1; i < cycle.size(); ++i) {
                QPointF data_point(boost::rational_cast<double>(cycle[i].x),
                                   boost::rational_cast<double>(cycle[i].y));
                path.lineTo(chart->mapToPosition(data_point));
            }
            path.closeSubpath();
        }

        fill.item->setPath(path);
    }
}
