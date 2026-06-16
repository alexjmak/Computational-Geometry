#include "algorithms/line_segment_intersection.hpp"
#include "geometry/predicates.hpp"
#include <climits>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace sweep {

using SegmentId = std::size_t;

/// \brief Event point wrapper ordered in sweep processing order.
class EventPoint {
  public:
    Point point; ///< The geometric point represented by this sweep event.

    /// \brief Wrap a point with the event-queue ordering used by the sweep.
    /// \param point The geometric point represented by this event.
    EventPoint(const Point& point);

    /// \brief Check whether two event points refer to the same geometric point.
    /// \param other The event point to compare against.
    /// \returns True when both event points contain the same point.
    bool operator==(const EventPoint& other) const;

    /// \brief Order events from top to bottom, breaking ties from left to right.
    /// \param other The event point to compare against.
    /// \returns True if this event should be processed before the other event.
    bool operator<(const EventPoint& other) const;

    /// \brief Check whether this event is at or before another event in sweep order.
    /// \param other The event point to compare against.
    /// \returns True if this event is ordered before or equal to the other event.
    bool operator<=(const EventPoint& other) const {
        return *this < other || *this == other;
    }
};

class State;

/// \brief Segment wrapper with sweep-state-dependent cached ordering data.
class ActiveSegment {
  public:
    SegmentId id;               ///< The original input segment index.
    Segment segment;            ///< The canonicalized segment represented by this ID.
    Rational slope_inverse;     ///< The finite inverse slope used for tie-breaking.
    int slope_inverse_infinity; ///< Direction marker for horizontal segment ordering.
    mutable std::optional<Point> cached_event; ///< The event point for the cached intersection.
    mutable std::optional<Point> cached_point_at_event; ///< Cached intersection at curr_event.

    /// \brief Wrap a segment with sweep-specific cached data.
    /// \param id The original input segment index.
    /// \param segment The segment represented in the active set.
    ActiveSegment(SegmentId id, const Segment& segment);

    /// \brief Return this segment's intersection with the sweep line through an event.
    /// \param event_point The event point whose horizontal sweep line is being evaluated.
    /// \returns The point where the segment meets the current event's horizontal line.
    std::optional<Point> pointAtEvent(const Point& event_point) const;

    /// \brief Check whether two active segments represent the same sweep segment by checking id.
    /// \param other The active segment to compare against.
    /// \returns True if both have the same id.
    bool operator==(const ActiveSegment& other) const;
};

/// \brief Compare active segments at the current sweep event.
struct ActiveSegmentCompare {
    const State* line_sweep; ///< The sweep-line state used to resolve segment IDs.

    /// \brief Initialize a comparator for one sweep-line state.
    /// \param line_sweep The state containing segment geometry and the current event.
    explicit ActiveSegmentCompare(const State* line_sweep);

    /// \brief Order active segments at the current sweep event.
    /// \param a_id The first active segment ID.
    /// \param b_id The second active segment ID.
    /// \returns True if a_id should appear before b_id in the active set.
    bool operator()(SegmentId a_id, SegmentId b_id) const;
};

/// \brief Event-queue payload for one geometric event point.
class Event {
  public:
    std::vector<SegmentId> upper_segments;   ///< Segments whose upper endpoint is this event.
    std::unordered_set<SegmentId> witnesses; ///< Segments that imply this event.

    /// \brief Store segments that start at an event and segments witnessing the event.
    Event();
};

/// \brief Mutable Bentley-Ottmann sweep-line state.
class State {
  public:
    mutable EventPoint curr_event; ///< The event point currently used for active-set ordering.
    std::map<EventPoint, Event> event_queue; ///< Pending events ordered in sweep order.
    /// \brief Active segments ordered by curr_event.
    ///
    /// Remove affected segments before moving curr_event, then insert/reinsert segments
    /// that remain active below the event.
    std::vector<ActiveSegment> segments_by_id; ///< Canonicalized active segment by input index.
    std::set<SegmentId, ActiveSegmentCompare> curr_segments;

    /// \brief Initialize the sweep-line state and active/event containers.
    State();

    /// \brief Seed the event queue with segment endpoint events.
    /// \param segments The input segments to add to the sweep.
    void populateEventQueue(const std::vector<Segment>& segments);

    /// \brief Print an active segment and its current sweep-line intersection point.
    /// \param id The active segment ID to print.
    void printElement(SegmentId id);
};

/// \brief Compare the inverse slopes of two active segments.
/// \param a The first active segment.
/// \param b The second active segment.
/// \returns -1 if a's inverse slope is smaller, 1 if larger, or 0 if equal.
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

State::State()
    : curr_event(EventPoint(Point(INT_MAX, INT_MAX))), segments_by_id(),
      curr_segments(ActiveSegmentCompare(this)) {}

void State::populateEventQueue(const std::vector<Segment>& segments) {
    // Insert segment's upper endpoints into the event queue, and keep track of which
    // segments they belong to.
    segments_by_id.reserve(segments.size());
    for (SegmentId id = 0; id < segments.size(); ++id) {
        segments_by_id.emplace_back(id, segments[id].canonicalizedY());
        const ActiveSegment& lss = segments_by_id.back();
        EventPoint upper_endpoint(lss.segment.end);
        EventPoint lower_endpoint(lss.segment.start);
        auto& upper_event = event_queue.try_emplace(upper_endpoint).first->second;
        upper_event.upper_segments.push_back(id);
        auto& lower_event = event_queue.try_emplace(lower_endpoint).first->second;
        lower_event.witnesses.insert(id);
    }
    if (!event_queue.empty()) {
        curr_event = event_queue.begin()->first;
    }
}

void State::printElement(SegmentId id) {
    const ActiveSegment& key = segments_by_id[id];
    auto point = key.pointAtEvent(curr_event.point);
    if (point) {
        std::cout << "#" << key.id << " " << key.segment.toString() << ": " << point->toString()
                  << std::endl;
    }
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

ActiveSegmentCompare::ActiveSegmentCompare(const State* line_sweep) : line_sweep(line_sweep) {}

bool ActiveSegmentCompare::operator()(SegmentId a_id, SegmentId b_id) const {
    const ActiveSegment& a = line_sweep->segments_by_id[a_id];
    const ActiveSegment& b = line_sweep->segments_by_id[b_id];
    const Point& event_point = line_sweep->curr_event.point;
    auto point_at_y = a.pointAtEvent(event_point);
    auto other_point_at_y = b.pointAtEvent(event_point);

    if (!point_at_y || !other_point_at_y) {
        throw std::runtime_error(
            "Failed to compute intersection point for active segment comparison");
    }

    if (*point_at_y == *other_point_at_y) {
        int slope_cmp = compareSlopeInverse(a, b);

        if (slope_cmp == 0) {
            if (a.segment == b.segment) {
                return a.id < b.id;
            }
            return a.segment < b.segment;
        }

        // Before and including the current event, the segments are ordered in the
        // order just below curr_event.point.y. If the intersection is after the
        // current event, then it hasn't been processed yet, and they are ordered in
        // the order just above curr_event.point.y.
        if (point_at_y->x <= event_point.x) {
            return slope_cmp < 0;
        } else {
            return slope_cmp > 0;
        }
    }

    bool ans = *point_at_y < *other_point_at_y;
    return ans;
}

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

Event::Event() : upper_segments(), witnesses() {}

/// \brief Insert a future intersection event for adjacent active segments when one exists.
/// \param predecessor One active segment adjacent to the candidate event.
/// \param successor The other active segment adjacent to the candidate event.
/// \param ls_point The current event point.
/// \param line_sweep The sweep-line state to update.
void findNewEvent(const ActiveSegment& predecessor, const ActiveSegment& successor,
                  const EventPoint& ls_point, State& line_sweep) {
    std::optional<Point> intersect_point =
        intersectionPoint(predecessor.segment, successor.segment);
    if (!intersect_point)
        return;
    EventPoint ls_intersect_point(*intersect_point);

    if (ls_intersect_point <= ls_point)
        return;

    if (line_sweep.event_queue.find(ls_intersect_point) != line_sweep.event_queue.end()) {
        auto& event = line_sweep.event_queue[ls_intersect_point];
        event.witnesses.insert(successor.id);
        event.witnesses.insert(predecessor.id);
        return;
    }

    // Event queue only contains upper segments.
#ifndef NDEBUG
    std::cout << "Found new event: " << ls_intersect_point.point.toString() << std::endl;
#endif

    auto& event = line_sweep.event_queue.try_emplace(ls_intersect_point).first->second;
    event.witnesses.insert(successor.id);
    event.witnesses.insert(predecessor.id);
}

/// \brief Walk neighboring active segments and collect those touching an event point.
/// \param ls_point The event point being processed.
/// \param seed A known active segment touching the event point.
/// \param mid_segments Segments containing this event in their interior.
/// \param lower_segments Segments whose lower endpoint is this event.
/// \param find_succ True to walk successors, false to walk predecessors.
void collectTouchingNeighbors(const EventPoint& ls_point, const ActiveSegment& seed,
                              const State& line_sweep, std::vector<SegmentId>& mid_segments,
                              std::vector<SegmentId>& lower_segments, bool find_succ) {
    const auto& curr_segments = line_sweep.curr_segments;
    const auto& segments_by_id = line_sweep.segments_by_id;
    SegmentId curr_segment_id = seed.id;
    while (true) {
        auto it = find_succ ? curr_segments.upper_bound(curr_segment_id)
                            : curr_segments.lower_bound(curr_segment_id);
        if (find_succ) {
            if (it == curr_segments.end()) {
                break;
            }
        } else {
            if (it == curr_segments.begin()) {
                break;
            }
            --it;
        }

        curr_segment_id = *it;
        const ActiveSegment& curr_segment = segments_by_id[curr_segment_id];

        const Point& lower_endpoint = curr_segment.segment.start;
        if (lower_endpoint == ls_point.point) {
#ifndef NDEBUG
            std::cout << "Found lower segment " << curr_segment.segment.toString() << std::endl;
#endif
            lower_segments.push_back(curr_segment_id);
        } else if (isPointOnSegment(ls_point.point, curr_segment.segment)) {
#ifndef NDEBUG
            std::cout << "Found mid segment " << curr_segment.segment.toString() << std::endl;
#endif
            mid_segments.push_back(curr_segment_id);
        } else {
            break;
        }
    }
}

/// \brief Process one sweep event and update the active set and future event queue.
/// \param ls_point The event point being processed.
/// \param event The event data associated with the point.
/// \param line_sweep The sweep-line state to update.
/// \returns Segment IDs touching the event point.
std::vector<SegmentId> handleEventPoint(EventPoint& ls_point, Event& event, State& line_sweep) {
#ifndef NDEBUG
    std::cout << std::endl;
    std::cout << "Event: " << ls_point.point.toString() << std::endl;
#endif

    auto& curr_segments = line_sweep.curr_segments;
    const auto& segments_by_id = line_sweep.segments_by_id;
    const auto& upper_segments = event.upper_segments;
    std::vector<SegmentId> mid_segments;
    std::vector<SegmentId> lower_segments;
#ifndef NDEBUG
    std::cout << "Active segments before event:" << std::endl;
    for (SegmentId id : curr_segments) {
        line_sweep.printElement(id);
    }
#endif

    // Populate all segments touching the event point on the lower point or anywhere in
    // the middle. These are all adjacent in curr_segments, and we use a known probe
    // segment to query the remaining ones.
    std::optional<SegmentId> probe_segment_id;
    if (!event.witnesses.empty()) {
        probe_segment_id = *event.witnesses.begin();
    } else {
        const SegmentId dummy_segment_id = segments_by_id.size();
        line_sweep.segments_by_id.emplace_back(dummy_segment_id,
                                               Segment(ls_point.point, ls_point.point));
        EventPoint prev_event = line_sweep.curr_event;
        line_sweep.curr_event = ls_point;

        // Use a zero-length dummy at the event point to locate where that point would
        // fall in the active-set ordering. A touching segment should be adjacent to that
        // position, but it may sort just before or just after the dummy because ties are
        // broken by slope.
        auto it = curr_segments.lower_bound(dummy_segment_id);
        if (it != curr_segments.begin()) {
            auto floor_it = std::prev(it);
            if (isPointOnSegment(ls_point.point, segments_by_id[*floor_it].segment)) {
                probe_segment_id = *floor_it;
            }
        }

        if (!probe_segment_id) {
            if (it != curr_segments.end() &&
                isPointOnSegment(ls_point.point, segments_by_id[*it].segment)) {
                probe_segment_id = *it;
            }
        }

        line_sweep.curr_event = prev_event;
        line_sweep.segments_by_id.pop_back();
    }

    if (probe_segment_id) {
        const ActiveSegment& probe_segment = segments_by_id[*probe_segment_id];
        const Point& lower_endpoint = probe_segment.segment.start;
        if (lower_endpoint == ls_point.point) {
            lower_segments.push_back(*probe_segment_id);
        } else if (isPointOnSegment(ls_point.point, probe_segment.segment)) {
            mid_segments.push_back(*probe_segment_id);
        }

        collectTouchingNeighbors(ls_point, probe_segment, line_sweep, mid_segments, lower_segments,
                                 true);
        collectTouchingNeighbors(ls_point, probe_segment, line_sweep, mid_segments, lower_segments,
                                 false);
    }

    for (SegmentId id : lower_segments) {
        const ActiveSegment& s = segments_by_id[id];
#ifndef NDEBUG
        std::cout << "Remove lower " << s.segment.toString() << std::endl;
#endif
        [[maybe_unused]] size_t count = curr_segments.erase(id);
        assert(count == 1);
    }
    for (SegmentId id : mid_segments) {
        const ActiveSegment& s = segments_by_id[id];
#ifndef NDEBUG
        std::cout << "Remove mid " << s.segment.toString() << std::endl;
#endif
        [[maybe_unused]] size_t count = curr_segments.erase(id);
        assert(count == 1);
    }

    // Move the sweep line.
    line_sweep.curr_event = ls_point;

    // Add in the upper segments and re-add the mid_segments so that they will be in
    // the correct order in curr_segments.
    for (SegmentId id : upper_segments) {
        const ActiveSegment& s = segments_by_id[id];
#ifndef NDEBUG
        std::cout << "Insert upper " << s.segment.toString() << std::endl;
#endif
        curr_segments.insert(id);
    }
    for (SegmentId id : mid_segments) {
        const ActiveSegment& s = segments_by_id[id];
#ifndef NDEBUG
        std::cout << "Insert mid " << s.segment.toString() << std::endl;
#endif
        curr_segments.insert(id);
    }

#ifndef NDEBUG
    std::cout << "Active segments after event:" << std::endl;
    for (SegmentId id : curr_segments) {
        line_sweep.printElement(id);
    }
#endif

    // Look for new events.
    if (upper_segments.empty() && mid_segments.empty()) {
        if (probe_segment_id) {
            auto it = curr_segments.lower_bound(*probe_segment_id);
            if (it != curr_segments.begin() && it != curr_segments.end()) {
                auto prev_it = std::prev(it);
                findNewEvent(segments_by_id[*prev_it], segments_by_id[*it], ls_point, line_sweep);
            }
        }
    } else {
        ActiveSegmentCompare compare_segment = curr_segments.key_comp();
        std::optional<SegmentId> leftmost;
        std::optional<SegmentId> rightmost;
        auto updateBoundarySegments = [&](SegmentId id) {
            if (!leftmost || compare_segment(id, *leftmost))
                leftmost = id;
            if (!rightmost || compare_segment(*rightmost, id))
                rightmost = id;
        };
        for (SegmentId id : upper_segments) {
            updateBoundarySegments(id);
        }
        for (SegmentId id : mid_segments) {
            updateBoundarySegments(id);
        }

        auto it = curr_segments.find(*leftmost);
        if (it != curr_segments.begin()) {
            --it;
            findNewEvent(segments_by_id[*it], segments_by_id[*leftmost], ls_point, line_sweep);
        }

        it = curr_segments.find(*rightmost);
        ++it;
        if (it != curr_segments.end()) {
            findNewEvent(segments_by_id[*rightmost], segments_by_id[*it], ls_point, line_sweep);
        }
    }

    std::vector<SegmentId> touching_segments;
    touching_segments.reserve(upper_segments.size() + mid_segments.size() + lower_segments.size());
    touching_segments.insert(touching_segments.end(), upper_segments.begin(), upper_segments.end());
    touching_segments.insert(touching_segments.end(), mid_segments.begin(), mid_segments.end());
    touching_segments.insert(touching_segments.end(), lower_segments.begin(), lower_segments.end());

    return touching_segments;
}

} // namespace sweep

template <typename IntersectionCallback>
void forEachLineSegmentIntersection(const std::vector<Segment>& segments,
                                    IntersectionCallback on_intersection) {
    std::vector<Segment> non_degenerate_segments;
    std::vector<sweep::SegmentId> original_ids;
    non_degenerate_segments.reserve(segments.size());
    original_ids.reserve(segments.size());
    for (sweep::SegmentId id = 0; id < segments.size(); ++id) {
        if (segments[id].start == segments[id].end) {
            continue;
        }
        original_ids.push_back(id);
        non_degenerate_segments.push_back(segments[id]);
    }

    sweep::State ls;
    ls.populateEventQueue(non_degenerate_segments);

    while (!ls.event_queue.empty()) {
        auto it = ls.event_queue.begin();
        sweep::EventPoint p = it->first;
        sweep::Event e = std::move(it->second);
        ls.event_queue.erase(it);

        std::vector<sweep::SegmentId> touching_segments = sweep::handleEventPoint(p, e, ls);
        if (touching_segments.size() > 1) {
            std::vector<sweep::SegmentId> original_touching_segments;
            original_touching_segments.reserve(touching_segments.size());
            for (sweep::SegmentId id : touching_segments) {
                original_touching_segments.push_back(original_ids[id]);
            }
#ifndef NDEBUG
            std::cout << "Intersection at " << p.point.toString() << std::endl;
#endif
            on_intersection(p.point, original_touching_segments);
        }
    }
}

std::unordered_set<Point> bruteForceLineSegmentIntersection(const std::vector<Segment>& segments) {
    std::unordered_set<Point> intersections;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (segments[i].start == segments[i].end) {
            continue;
        }
        for (size_t j = i + 1; j < segments.size(); ++j) {
            if (segments[j].start == segments[j].end) {
                continue;
            }
            std::optional<Point> intersection = intersectionPoint(segments[i], segments[j]);
            if (intersection) {
                intersections.insert(*intersection);
            } else if (intersectionType(segments[i], segments[j]) == IntersectionType::SEGMENT) {
                // To make it consistent with the sweep-line output, we include endpoints of
                // collinear overlaps.
                if (isPointOnSegment(segments[i].start, segments[j])) {
                    intersections.insert(segments[i].start);
                }
                if (isPointOnSegment(segments[i].end, segments[j])) {
                    intersections.insert(segments[i].end);
                }
                if (isPointOnSegment(segments[j].start, segments[i])) {
                    intersections.insert(segments[j].start);
                }
                if (isPointOnSegment(segments[j].end, segments[i])) {
                    intersections.insert(segments[j].end);
                }
            }
        }
    }
    return intersections;
}

std::unordered_set<Point> lineSegmentIntersection(const std::vector<Segment>& segments) {
    std::unordered_set<Point> intersection_points;
    forEachLineSegmentIntersection(segments,
                                   [&](const Point& point, const std::vector<sweep::SegmentId>&) {
                                       intersection_points.insert(point);
                                   });
    return intersection_points;
}

std::vector<std::vector<Point>>
lineSegmentIntersectionBySegments(const std::vector<Segment>& segments) {
    std::vector<std::vector<Point>> intersection_points_by_segments;
    intersection_points_by_segments.resize(segments.size());

    forEachLineSegmentIntersection(
        segments, [&](const Point& point, const std::vector<sweep::SegmentId>& touching_segments) {
            for (sweep::SegmentId id : touching_segments) {
                intersection_points_by_segments[id].push_back(point);
            }
        });

    // Sort points so that they are in order
    for (std::size_t id = 0; id < intersection_points_by_segments.size(); id++) {
        std::vector<Point>& points = intersection_points_by_segments[id];
        std::sort(points.begin(), points.end());
        points.erase(std::unique(points.begin(), points.end()), points.end());
        if (!segments[id].isCanonicalizedX()) {
            std::reverse(points.begin(), points.end());
        }
    }

    return intersection_points_by_segments;
}
