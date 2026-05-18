#include "algorithms/line_segment_intersection.hpp"
#include "geometry/predicates.hpp"
#include <climits>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>

namespace sweep {

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
    Segment segment;            ///< The canonicalized segment represented in the active set.
    State* line_sweep;          ///< The owning sweep-line state.
    Rational slope_inverse;     ///< The finite inverse slope used for tie-breaking.
    int slope_inverse_infinity; ///< Direction marker for horizontal segment ordering.
    mutable std::optional<Point> cached_event; ///< The event point for the cached intersection.
    mutable std::optional<Point> cached_point_at_event; ///< Cached intersection at curr_event.

    /// \brief Wrap a segment with sweep-specific cached data.
    /// \param segment The segment represented in the active set.
    /// \param line_sweep The owning sweep-line state.
    ActiveSegment(const Segment& segment, State* line_sweep);

    /// \brief Return this segment's intersection with the current sweep line.
    /// \returns The point where the segment meets the current event's horizontal line.
    std::optional<Point> pointAtCurrEvent() const;

    /// \brief Check whether two active segments represent the same sweep segment.
    /// \param other The active segment to compare against.
    /// \returns True if both wrappers contain the same segment and sweep state.
    bool operator==(const ActiveSegment& other) const;
};

/// \brief Hash active segments by their wrapped directed geometry.
struct ActiveSegmentHash {
    /// \brief Hash an active segment by its underlying geometry.
    /// \param s The active segment to hash.
    /// \returns A hash derived from the wrapped segment.
    size_t operator()(const ActiveSegment& s) const {
        return std::hash<Segment>{}(s.segment);
    }
};

/// \brief Compare active segments at the current sweep event.
struct ActiveSegmentCompare {
    /// \brief Order active segments at the current sweep event.
    /// \param a The first active segment.
    /// \param b The second active segment.
    /// \returns True if a should appear before b in the active set.
    bool operator()(const ActiveSegment& a, const ActiveSegment& b) const;
};

/// \brief Event-queue payload for one geometric event point.
class Event {
  public:
    std::vector<ActiveSegment> upper_segments; ///< Segments whose upper endpoint is this event.
    std::unordered_set<ActiveSegment, ActiveSegmentHash>
        witnesses; ///< Segments that imply this event.

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
    std::set<ActiveSegment, ActiveSegmentCompare> curr_segments;

    /// \brief Initialize the sweep-line state and active/event containers.
    State();

    /// \brief Seed the event queue with segment endpoint events.
    /// \param segments The input segments to add to the sweep.
    void populateEventQueue(const std::vector<Segment>& segments);

    /// \brief Print an active segment and its current sweep-line intersection point.
    /// \param key The active segment to print.
    void printElement(const ActiveSegment& key);
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

State::State() : curr_event(EventPoint(Point(INT_MAX, INT_MAX))) {}

void State::populateEventQueue(const std::vector<Segment>& segments) {
    // Insert segment's upper endpoints into the event queue, and keep track of which
    // segments they belong to.
    for (const Segment& s : segments) {
        ActiveSegment lss(s.canonicalizedY(), this);
        EventPoint upper_endpoint(lss.segment.end);
        EventPoint lower_endpoint(lss.segment.start);
        auto& upper_event = event_queue.try_emplace(upper_endpoint).first->second;
        upper_event.upper_segments.push_back(lss);
        auto& lower_event = event_queue.try_emplace(lower_endpoint).first->second;
        lower_event.witnesses.insert(lss);
    }
    if (!event_queue.empty()) {
        curr_event = event_queue.begin()->first;
    }
}

void State::printElement(const ActiveSegment& key) {
    auto point = key.pointAtCurrEvent();
    if (point) {
        std::cout << key.segment.toString() << ": " << point->toString() << std::endl;
    }
}

ActiveSegment::ActiveSegment(const Segment& segment, State* line_sweep)
    : segment(segment), line_sweep(line_sweep), slope_inverse(0), slope_inverse_infinity(0),
      cached_event(), cached_point_at_event() {
    Rational dx = segment.start.x - segment.end.x;
    Rational dy = segment.start.y - segment.end.y;
    if (dy != 0) {
        slope_inverse = dx / -dy;
        slope_inverse_infinity = 0;
    } else {
        slope_inverse_infinity = (dx >= 0) ? 1 : -1;
    }
}

std::optional<Point> ActiveSegment::pointAtCurrEvent() const {
    const Point& event_point = line_sweep->curr_event.point;
    if (cached_event && *cached_event == event_point) {
        return cached_point_at_event;
    }
    auto point_at_y = intersectAtY(segment, event_point);
    cached_event = event_point;
    cached_point_at_event = point_at_y;
    return point_at_y;
}

bool ActiveSegment::operator==(const ActiveSegment& other) const {
    return segment == other.segment && line_sweep == other.line_sweep;
}

bool ActiveSegmentCompare::operator()(const ActiveSegment& a, const ActiveSegment& b) const {
    auto point_at_y = a.pointAtCurrEvent();
    auto other_point_at_y = b.pointAtCurrEvent();
    const Point& event_point = a.line_sweep->curr_event.point;

    if (!point_at_y || !other_point_at_y) {
        throw std::runtime_error(
            "Failed to compute intersection point for active segment comparison");
    }

    if (*point_at_y == *other_point_at_y) {
        int slope_cmp = compareSlopeInverse(a, b);

        if (slope_cmp == 0) {
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
        event.witnesses.insert(successor);
        event.witnesses.insert(predecessor);
        return;
    }

    // Event queue only contains upper segments.
#ifndef NDEBUG
    std::cout << "Found new event: " << ls_intersect_point.point.toString() << std::endl;
#endif

    auto& event = line_sweep.event_queue.try_emplace(ls_intersect_point).first->second;
    event.witnesses.insert(successor);
    event.witnesses.insert(predecessor);
}

/// \brief Walk neighboring active segments and collect those touching an event point.
/// \param ls_point The event point being processed.
/// \param seed A known active segment touching the event point.
/// \param curr_segments The active segment set to search.
/// \param mid_segments Output list for segments containing the event in their interior.
/// \param lower_segments Output list for segments whose lower endpoint is the event.
/// \param find_succ True to walk successors, false to walk predecessors.
void collectTouchingNeighbors(const EventPoint& ls_point, const ActiveSegment& seed,
                              std::set<ActiveSegment, ActiveSegmentCompare>& curr_segments,
                              std::vector<ActiveSegment>& mid_segments,
                              std::vector<ActiveSegment>& lower_segments, bool find_succ) {
    ActiveSegment curr_segment = seed;
    while (true) {
        auto it = find_succ ? curr_segments.upper_bound(curr_segment)
                            : curr_segments.lower_bound(curr_segment);
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

        curr_segment = *it;

        const Point& lower_endpoint = curr_segment.segment.start;
        if (lower_endpoint == ls_point.point) {
#ifndef NDEBUG
            std::cout << "Found lower segment " << curr_segment.segment.toString() << std::endl;
#endif
            lower_segments.push_back(curr_segment);
        } else if (isPointOnSegment(ls_point.point, curr_segment.segment)) {
#ifndef NDEBUG
            std::cout << "Found mid segment " << curr_segment.segment.toString() << std::endl;
#endif
            mid_segments.push_back(curr_segment);
        } else {
            break;
        }
    }
}

/// \brief Process one sweep event and update the active set and future event queue.
/// \param ls_point The event point being processed.
/// \param event The event data associated with the point.
/// \param line_sweep The sweep-line state to update.
/// \returns True if more than one segment touches the event point.
bool handleEventPoint(EventPoint& ls_point, Event& event, State& line_sweep) {
#ifndef NDEBUG
    std::cout << std::endl;
    std::cout << "Event: " << ls_point.point.toString() << std::endl;
#endif

    std::vector<ActiveSegment> upper_segments = std::move(event.upper_segments);
    std::vector<ActiveSegment> mid_segments;
    std::vector<ActiveSegment> lower_segments;

    auto& curr_segments = line_sweep.curr_segments;
#ifndef NDEBUG
    std::cout << "Active segments before event:" << std::endl;
    for (const auto& segment : curr_segments) {
        line_sweep.printElement(segment);
    }
#endif

    // Populate all segments touching the event point on the lower point or anywhere in
    // the middle. These are all adjacent in curr_segments, and we use a known probe
    // segment to query the remaining ones.
    std::optional<ActiveSegment> probe_segment;
    if (!event.witnesses.empty()) {
        probe_segment = *event.witnesses.begin();
    } else {
        ActiveSegment dummy_segment(Segment(ls_point.point, ls_point.point), &line_sweep);
        EventPoint prev_event = line_sweep.curr_event;
        line_sweep.curr_event = ls_point;

        // Use a zero-length dummy at the event point to locate where that point would
        // fall in the active-set ordering. A touching segment should be adjacent to that
        // position, but it may sort just before or just after the dummy because ties are
        // broken by slope.
        auto it = curr_segments.lower_bound(dummy_segment);
        if (it != curr_segments.begin()) {
            auto floor_it = std::prev(it);
            if (isPointOnSegment(ls_point.point, floor_it->segment)) {
                probe_segment = *floor_it;
            }
        }

        if (!probe_segment) {
            if (it != curr_segments.end() && isPointOnSegment(ls_point.point, it->segment)) {
                probe_segment = *it;
            }
        }

        line_sweep.curr_event = prev_event;
    }

    if (probe_segment) {
        const Point& lower_endpoint = probe_segment->segment.start;
        if (lower_endpoint == ls_point.point) {
            lower_segments.push_back(*probe_segment);
        } else if (isPointOnSegment(ls_point.point, probe_segment->segment)) {
            mid_segments.push_back(*probe_segment);
        }

        collectTouchingNeighbors(ls_point, *probe_segment, curr_segments, mid_segments,
                                 lower_segments, true);
        collectTouchingNeighbors(ls_point, *probe_segment, curr_segments, mid_segments,
                                 lower_segments, false);
    }

    for (ActiveSegment& s : lower_segments) {
#ifndef NDEBUG
        std::cout << "Remove lower " << s.segment.toString() << std::endl;
#endif
        [[maybe_unused]] size_t count = curr_segments.erase(s);
        assert(count == 1);
    }
    for (ActiveSegment& s : mid_segments) {
#ifndef NDEBUG
        std::cout << "Remove mid " << s.segment.toString() << std::endl;
#endif
        [[maybe_unused]] size_t count = curr_segments.erase(s);
        assert(count == 1);
    }

    // Move the sweep line.
    line_sweep.curr_event = ls_point;

    // Add in the upper segments and re-add the mid_segments so that they will be in
    // the correct order in curr_segments.
    for (auto& s : upper_segments) {
#ifndef NDEBUG
        std::cout << "Insert upper " << s.segment.toString() << std::endl;
#endif
        curr_segments.insert(s);
    }
    for (auto& s : mid_segments) {
#ifndef NDEBUG
        std::cout << "Insert mid " << s.segment.toString() << std::endl;
#endif
        curr_segments.insert(s);
    }

#ifndef NDEBUG
    std::cout << "Active segments after event:" << std::endl;
    for (const auto& segment : curr_segments) {
        line_sweep.printElement(segment);
    }
#endif

    // Look for new events.
    if (upper_segments.empty() && mid_segments.empty()) {
        if (probe_segment) {
            auto it = curr_segments.lower_bound(*probe_segment);
            if (it != curr_segments.begin() && it != curr_segments.end()) {
                auto prev_it = std::prev(it);
                findNewEvent(*prev_it, *it, ls_point, line_sweep);
            }
        }
    } else {
        ActiveSegmentCompare compare_segment;
        ActiveSegment* leftmost = nullptr;
        ActiveSegment* rightmost = nullptr;
        auto updateBoundarySegments = [&](ActiveSegment& s) {
            if (!leftmost || compare_segment(s, *leftmost))
                leftmost = &s;
            if (!rightmost || compare_segment(*rightmost, s))
                rightmost = &s;
        };
        for (ActiveSegment& s : upper_segments) {
            updateBoundarySegments(s);
        }
        for (ActiveSegment& s : mid_segments) {
            updateBoundarySegments(s);
        }

        auto it = curr_segments.find(*leftmost);
        if (it != curr_segments.begin()) {
            --it;
            findNewEvent(*it, *leftmost, ls_point, line_sweep);
        }

        it = curr_segments.find(*rightmost);
        ++it;
        if (it != curr_segments.end()) {
            findNewEvent(*rightmost, *it, ls_point, line_sweep);
        }
    }

    if (upper_segments.size() + mid_segments.size() + lower_segments.size() > 1) {
#ifndef NDEBUG
        std::cout << "Intersection at " << ls_point.point.toString() << std::endl;
#endif
        return true;
    }

    return false;
}

} // namespace sweep

std::unordered_set<Point> bruteForceLineSegmentIntersection(const std::vector<Segment>& segments) {
    std::unordered_set<Point> intersections;
    for (size_t i = 0; i < segments.size(); ++i) {
        for (size_t j = i + 1; j < segments.size(); ++j) {
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
    sweep::State ls;
    ls.populateEventQueue(segments);

    std::unordered_set<Point> intersection_points;
    while (!ls.event_queue.empty()) {
        auto it = ls.event_queue.begin();
        sweep::EventPoint p = it->first;
        sweep::Event e = std::move(it->second);
        ls.event_queue.erase(it);

        if (sweep::handleEventPoint(p, e, ls)) {
            intersection_points.insert(p.point);
        }
    }
    return intersection_points;
}
