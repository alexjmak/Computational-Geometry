#include "gui/plot.hpp"
#include "io/document.hpp"
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtWidgets/QApplication>
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

    QChart* chart = chartFor(plot);
    ASSERT_NE(chart, nullptr);
    EXPECT_EQ(chart->series().size(), 2);
}

TEST(PlotTest, SetDocumentRendersAllLayerGeometry) {
    Document document;
    Layer layer;
    layer.points = {Point(0, 0)};
    layer.segments = {Segment(Point(0, 0), Point(1, 1))};
    layer.polygons = {Polygon(Cycle({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)}))};
    document.layers.push_back(layer);

    Plot plot;
    plot.setDocument(document);

    QChart* chart = chartFor(plot);
    ASSERT_NE(chart, nullptr);
    EXPECT_EQ(chart->series().size(), 3);
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
