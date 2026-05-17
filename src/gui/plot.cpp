#include "gui/plot.hpp"
#include <QApplication>
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

    QColor fill_color(QString::fromStdString(face_color));
    fill_color.setAlphaF(std::clamp(alpha, 0.0, 1.0));
    QGraphicsPolygonItem* fill_item = new QGraphicsPolygonItem(chart);
    fill_item->setBrush(QBrush(fill_color));
    fill_item->setPen(Qt::NoPen);
    fill_item->setZValue(-1.0);
    cycle_fills.push_back({cycle.points, fill_item});

    updateAxes();
}

void Plot::addPolygon(const Polygon& polygon, const std::string& face_color,
                      const std::string& edge_color, double alpha) {
    // Simplified, draw only the outer cycle.
    addCycle(polygon.outer_cycle, face_color, edge_color, alpha);
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
        QPolygonF polygon;
        polygon.reserve(static_cast<int>(fill.points.size()));
        for (const Point& point : fill.points) {
            QPointF data_point(boost::rational_cast<double>(point.x),
                               boost::rational_cast<double>(point.y));
            polygon << chart->mapToPosition(data_point);
        }
        fill.item->setPolygon(polygon);
    }
}
