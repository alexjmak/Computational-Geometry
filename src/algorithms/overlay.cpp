#include "algorithms/overlay.hpp"
#include "algorithms/line_segment_intersection.hpp"
#include <algorithm>
#include <unordered_set>
#include <vector>

std::vector<Segment> planarizeSegments(const std::vector<Segment>& segments) {
    std::vector<std::vector<Point>> intersections = lineSegmentIntersectionBySegments(segments);

    std::unordered_set<Segment> split_segments;
    for (std::size_t i = 0; i < segments.size(); i++) {
        const Segment& segment = segments[i];
        const std::vector<Point>& split_pts = intersections[i];

        // Intersections list might not have the endpoints of segment
        const Point* prev = &segment.start;
        for (std::size_t j = 0; j < split_pts.size(); j++) {
            const Point* curr = &split_pts[j];
            if (*curr != *prev) {
                split_segments.insert(Segment(*prev, *curr));
            }
            prev = curr;
        }

        // Include last section until end
        if (segment.end != *prev) {
            split_segments.insert(Segment(*prev, segment.end));
        }
    }

    std::vector<Segment> result(split_segments.begin(), split_segments.end());
    std::sort(result.begin(), result.end());
    return result;
}
