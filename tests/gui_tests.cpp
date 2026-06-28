#include "gui/plot.hpp"
#include "io/document.hpp"
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsPathItem>
#include <QtWidgets/QGraphicsPolygonItem>
#include <gtest/gtest.h>
#include <vector>

namespace {

QChart* chartFor(const Plot& plot) {
    auto* chart_view = plot.findChild<QChartView*>();
    if (!chart_view) {
        return nullptr;
    }
    return chart_view->chart();
}

std::vector<QGraphicsPathItem*> pathItemsFor(const QChart& chart) {
    std::vector<QGraphicsPathItem*> path_items;
    for (QGraphicsItem* item : chart.childItems()) {
        if (auto* path_item = qgraphicsitem_cast<QGraphicsPathItem*>(item)) {
            path_items.push_back(path_item);
        }
    }
    return path_items;
}

std::vector<QGraphicsPolygonItem*> polygonItemsFor(const QChart& chart) {
    std::vector<QGraphicsPolygonItem*> polygon_items;
    for (QGraphicsItem* item : chart.childItems()) {
        if (auto* polygon_item = qgraphicsitem_cast<QGraphicsPolygonItem*>(item)) {
            polygon_items.push_back(polygon_item);
        }
    }
    return polygon_items;
}

} // namespace

TEST(PlotTest, AddPointsCreatesScatterSeries) {
    Plot plot;

    plot.addPoints({Point(0, 0), Point(1, 1)});

    QChart* chart = chartFor(plot);
    ASSERT_NE(chart, nullptr);
    ASSERT_EQ(chart->series().size(), 1);
    EXPECT_NE(qobject_cast<QScatterSeries*>(chart->series().first()), nullptr);
}

TEST(PlotTest, AddSegmentsCreatesOneSeriesPerSegment) {
    Plot plot;

    plot.addSegments({
        Segment(Point(0, 0), Point(1, 1)),
        Segment(Point(1, 0), Point(0, 1)),
    });
    plot.show();
    QApplication::processEvents();

    QChart* chart = chartFor(plot);
    ASSERT_NE(chart, nullptr);
    EXPECT_EQ(chart->series().size(), 2);

    const std::vector<QGraphicsPolygonItem*> polygon_items = polygonItemsFor(*chart);
    ASSERT_EQ(polygon_items.size(), 2);
    EXPECT_EQ(polygon_items[0]->polygon().size(), 3);
    EXPECT_EQ(polygon_items[1]->polygon().size(), 3);
}

TEST(PlotTest, AddSegmentsCanHideArrows) {
    Plot plot;

    plot.addSegments({Segment(Point(0, 0), Point(1, 1))}, "blue", 1, false);
    plot.show();
    QApplication::processEvents();

    QChart* chart = chartFor(plot);
    ASSERT_NE(chart, nullptr);
    EXPECT_EQ(chart->series().size(), 1);
    EXPECT_TRUE(polygonItemsFor(*chart).empty());
}

TEST(PlotTest, SetDocumentRendersAllLayerGeometry) {
    Document document;
    Layer layer;
    layer.points = {Point(0, 0)};
    layer.segments = {Segment(Point(0, 0), Point(1, 1))};
    layer.polygons = {Polygon(LinearRing({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)}))};
    document.layers.push_back(layer);

    Plot plot;
    plot.setDocument(document);

    QChart* chart = chartFor(plot);
    ASSERT_NE(chart, nullptr);
    EXPECT_EQ(chart->series().size(), 3);
}

TEST(PlotTest, SetDocumentCanHideLayers) {
    Document document;
    Layer visible_layer;
    visible_layer.points = {Point(0, 0)};
    Layer hidden_layer;
    hidden_layer.points = {Point(1, 1)};
    document.layers = {visible_layer, hidden_layer};

    Plot plot;
    plot.setDocument(document, {true, false});

    QChart* chart = chartFor(plot);
    ASSERT_NE(chart, nullptr);
    ASSERT_EQ(chart->series().size(), 1);
    auto* series = qobject_cast<QScatterSeries*>(chart->series().first());
    ASSERT_NE(series, nullptr);
    ASSERT_EQ(series->points().size(), 1);
    EXPECT_EQ(series->points().first(), QPointF(0, 0));
}

TEST(PlotTest, AddPolygonUsesCompoundFillForHoles) {
    const LinearRing outer({Point(0, 0), Point(4, 0), Point(4, 4), Point(0, 4)});
    const LinearRing hole({Point(1, 1), Point(1, 3), Point(3, 3), Point(3, 1)});
    const Polygon polygon(outer, {hole});

    Plot plot;
    plot.addPolygon(polygon);
    plot.show();
    QApplication::processEvents();

    QChart* chart = chartFor(plot);
    ASSERT_NE(chart, nullptr);
    EXPECT_EQ(chart->series().size(), 2);

    const std::vector<QGraphicsPathItem*> path_items = pathItemsFor(*chart);
    ASSERT_EQ(path_items.size(), 1);
    EXPECT_EQ(path_items[0]->path().fillRule(), Qt::OddEvenFill);
    EXPECT_TRUE(path_items[0]->path().contains(chart->mapToPosition(QPointF(0.5, 0.5))));
    EXPECT_FALSE(path_items[0]->path().contains(chart->mapToPosition(QPointF(2.0, 2.0))));
}

TEST(PlotTest, ClearRemovesSeriesAndAxes) {
    Plot plot;
    plot.addPoints({Point(0, 0), Point(1, 1)});

    plot.clear();

    QChart* chart = chartFor(plot);
    ASSERT_NE(chart, nullptr);
    EXPECT_TRUE(chart->series().empty());
    EXPECT_TRUE(chart->axes().empty());
}

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
