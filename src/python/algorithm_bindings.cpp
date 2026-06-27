#include "algorithms/assemble.hpp"
#include "algorithms/convex_hull.hpp"
#include "algorithms/line_segment_intersection.hpp"
#include "algorithms/overlay.hpp"
#include "geometry/random.hpp"
#include "python/bindings.hpp"
#include <algorithm>
#include <optional>
#include <pybind11/stl.h>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

namespace py = pybind11;

namespace {

std::vector<Point> sortedPoints(std::unordered_set<Point> points) {
    std::vector<Point> ret(points.begin(), points.end());
    std::ranges::sort(ret);
    return ret;
}

std::vector<Point> lineSegmentIntersectionPython(const std::vector<Segment>& segments,
                                                 const std::string& algorithm) {
    if (algorithm == "line-sweep") {
        return sortedPoints(lineSegmentIntersection(segments));
    }
    if (algorithm == "brute-force") {
        return sortedPoints(bruteForceLineSegmentIntersection(segments));
    }
    throw py::value_error("algorithm must be 'line-sweep' or 'brute-force'");
}

std::vector<Point> randomPointsPython(int count, std::optional<unsigned int> seed, int min_coord,
                                      int max_coord) {
    if (seed.has_value()) {
        std::mt19937 rng(*seed);
        return randomPoints(count, min_coord, max_coord, &rng);
    }
    return randomPoints(count, min_coord, max_coord);
}

std::vector<Segment> randomSegmentsPython(int count, std::optional<unsigned int> seed,
                                          int min_length, int max_length, int min_coord,
                                          int max_coord) {
    if (seed.has_value()) {
        std::mt19937 rng(*seed);
        return randomSegments(count, min_length, max_length, min_coord, max_coord, &rng);
    }
    return randomSegments(count, min_length, max_length, min_coord, max_coord);
}

} // namespace

void bindAlgorithms(py::module_& module) {
    py::class_<OverlayFaceLabel>(module, "OverlayFaceLabel")
        .def_readonly("left_face", &OverlayFaceLabel::left_face)
        .def_readonly("right_face", &OverlayFaceLabel::right_face);

    py::class_<OverlayFacePolygon>(module, "OverlayFacePolygon")
        .def_readonly("face_index", &OverlayFacePolygon::face_index)
        .def_readonly("polygon", &OverlayFacePolygon::polygon)
        .def_readonly("label", &OverlayFacePolygon::label);

    py::class_<OverlayResult>(module, "OverlayResult")
        .def_readonly("face_labels", &OverlayResult::faceLabels)
        .def("polygons", &overlayFacePolygons);

    module.def("assemble_rings", &assembleRings, py::arg("segments"));
    module.def("assemble_polygons", &assemblePolygons, py::arg("segments"));
    module.def("convex_hull", &convexHull, py::arg("points"));
    module.def("line_segment_intersection", &lineSegmentIntersectionPython, py::arg("segments"),
               py::arg("algorithm") = "line-sweep");
    module.def("random_points", &randomPointsPython, py::arg("count"), py::arg("seed") = py::none(),
               py::arg("min_coord") = 10, py::arg("max_coord") = 100);
    module.def("random_segments", &randomSegmentsPython, py::arg("count"),
               py::arg("seed") = py::none(), py::arg("min_length") = 1, py::arg("max_length") = 40,
               py::arg("min_coord") = 1, py::arg("max_coord") = 1000);
    module.def("planarize_segments",
               py::overload_cast<const std::vector<Segment>&>(&planarizeSegments),
               py::arg("segments"));
    module.def("segment_overlay",
               py::overload_cast<const std::vector<Segment>&, const std::vector<Segment>&>(
                   &segmentOverlay),
               py::arg("left"), py::arg("right"));
    module.def("polygon_and", &polygonAnd, py::arg("left"), py::arg("right"));
    module.def("polygon_or", &polygonOr, py::arg("left"), py::arg("right"));
    module.def("polygon_difference", &polygonDifference, py::arg("left"), py::arg("right"));
    module.def("polygon_xor", &polygonXor, py::arg("left"), py::arg("right"));
}
