#include "algorithms/sweep_line.hpp"
#include "geometry/intersection.hpp"
#include <stdexcept>

namespace sweep {

EventPoint::EventPoint(const Point& point) : point(point) {}

bool EventPoint::operator==(const EventPoint& other) const {
    return point == other.point;
}

bool EventPoint::operator<(const EventPoint& other) const {
    if (point.y == other.point.y) {
        return point.x < other.point.x;
    }
    return point.y > other.point.y;
}

ActiveSegment::ActiveSegment(SegmentId id, const Segment& segment)
    : id(id), segment(segment), slope_inverse(0), slope_inverse_infinity(0), cached_event(),
      cached_point_at_event() {
    Rational dx = segment.start.x - segment.end.x;
    Rational dy = segment.start.y - segment.end.y;
    if (dy != 0) {
        slope_inverse = dx / -dy;
        slope_inverse_infinity = 0;
    } else {
        slope_inverse_infinity = (dx >= 0) ? 1 : -1;
    }
}

std::optional<Point> ActiveSegment::pointAtEvent(const Point& event_point) const {
    if (cached_event && *cached_event == event_point) {
        return cached_point_at_event;
    }
    auto point_at_y = intersectAtY(segment, event_point);
    cached_event = event_point;
    cached_point_at_event = point_at_y;
    return point_at_y;
}

bool ActiveSegment::operator==(const ActiveSegment& other) const {
    return id == other.id;
}

int compareSlopeInverse(const ActiveSegment& a, const ActiveSegment& b) {
    if (a.slope_inverse_infinity != b.slope_inverse_infinity) {
        return a.slope_inverse_infinity < b.slope_inverse_infinity ? -1 : 1;
    }
    if (a.slope_inverse_infinity != 0) {
        return 0;
    }
    if (a.slope_inverse < b.slope_inverse) {
        return -1;
    }
    if (b.slope_inverse < a.slope_inverse) {
        return 1;
    }
    return 0;
}

ActiveSegmentCompare::ActiveSegmentCompare(const EventPoint* curr_event,
                                           const std::vector<ActiveSegment>* segments_by_id,
                                           const bool use_canonical_tie_order)
    : curr_event(curr_event), segments_by_id(segments_by_id),
      use_canonical_tie_order(use_canonical_tie_order) {}

bool ActiveSegmentCompare::operator()(SegmentId a_id, SegmentId b_id) const {
    const ActiveSegment& a = (*segments_by_id)[a_id];
    const ActiveSegment& b = (*segments_by_id)[b_id];
    const Point& event_point = curr_event->point;
    auto point_at_y = a.pointAtEvent(event_point);
    auto other_point_at_y = b.pointAtEvent(event_point);

    if (!point_at_y || !other_point_at_y) {
        throw std::runtime_error(
            "Failed to compute intersection point for active segment comparison");
    }

    if (*point_at_y == *other_point_at_y) {
        int slope_cmp = compareSlopeInverse(a, b);

        if (slope_cmp == 0 || use_canonical_tie_order) {
            if (a.segment == b.segment) {
                return a.id < b.id;
            }
            return a.segment < b.segment;
        }

        // Before and including the current event, the segments are ordered in the
        // order just below curr_event.point.y. If the intersection is after the
        // current event, then it has not been processed yet, and they are ordered
        // in the order just above curr_event.point.y.
        if (point_at_y->x <= event_point.x) {
            return slope_cmp < 0;
        }
        return slope_cmp > 0;
    }

    return *point_at_y < *other_point_at_y;
}

std::optional<SegmentId>
nearestActiveSegmentToLeft(const Point& point,
                           const std::set<SegmentId, ActiveSegmentCompare>& active_segments,
                           std::vector<ActiveSegment>& segments_by_id) {
    const SegmentId dummy_segment_id = segments_by_id.size();
    segments_by_id.emplace_back(dummy_segment_id, Segment(point, point));

    // The dummy sorts at the query x-coordinate. Its predecessor is the nearest candidate to
    // the left in the active set. Canonical tie ordering resolves shared-endpoint ties.
    auto it = active_segments.lower_bound(dummy_segment_id);
    std::optional<SegmentId> result;

    while (it != active_segments.begin()) {
        --it;
        const ActiveSegment& candidate = segments_by_id[*it];
        std::optional<Point> hit = candidate.pointAtEvent(point);
        if (hit && hit->x < point.x) {
            result = *it;
            break;
        }
    }

    segments_by_id.pop_back();
    return result;
}

} // namespace sweep
