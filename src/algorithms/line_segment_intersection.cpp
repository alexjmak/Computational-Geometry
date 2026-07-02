#include "algorithms/line_segment_intersection.hpp"
#include "algorithms/sweep_line.hpp"
#include "debug/logging.hpp"
#include "geometry/predicates.hpp"
#include <climits>
#include <cstddef>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <unordered_set>
#include <vector>

/*
 * Line segment intersection overview
 *
 * This file implements two versions of the segment-intersection problem:
 * a straightforward brute-force checker and a sweep-line algorithm in the
 * Bentley-Ottmann family. Both return the set of geometric points where at
 * least two input segments intersect. Overlapping collinear segments are
 * reported by their overlap endpoints, matching the rest of the library's
 * point-set interface.
 *
 * The sweep-line implementation processes event points from top to bottom,
 * with left-to-right tie breaking. The event queue initially contains segment
 * endpoints. As the sweep discovers that neighboring active segments will
 * intersect below the current event, that intersection point is inserted into
 * the queue as a future event.
 *
 * High-level sweep steps:
 *
 * 1. Canonicalize each segment by y-order and seed the event queue with its
 *    upper and lower endpoints.
 * 2. Maintain an active set of segments intersecting the current sweep line.
 *    The active set is ordered by each segment's intersection with the
 *    horizontal line through the current event, with slope-based tie breaking
 *    for segments meeting at the same event point.
 * 3. At each event, collect:
 *    - segments whose upper endpoint is the event,
 *    - active segments that contain the event in their interior,
 *    - segments whose lower endpoint is the event.
 * 4. If at least two segments meet at the event, record that event point as an
 *    intersection.
 * 5. Remove affected active segments, advance the sweep event, then reinsert
 *    the segments that continue below the event.
 * 6. Only newly adjacent active segments can produce the next undiscovered
 *    intersection, so test those neighbor pairs and enqueue any valid future
 *    event.
 *
 * The implementation has more bookkeeping than the textbook sketch because it
 * needs deterministic behavior for exact rational coordinates, duplicate or
 * reversed input segments, endpoint touches, horizontal segments, and
 * collinear overlaps.
 */

namespace {

using sweep::ActiveSegment;
using sweep::ActiveSegmentCompare;
using sweep::EventPoint;
using sweep::SegmentId;

/// \brief Event-queue payload for one geometric event point.
class SegmentIntersectionEvent {
  public:
    std::vector<SegmentId> upper_segments;   ///< Segments whose upper endpoint is this event.
    std::unordered_set<SegmentId> witnesses; ///< Segments that imply this event.

    /// \brief Store segments that start at an event and segments witnessing the event.
    SegmentIntersectionEvent();
};

/// \brief Mutable Bentley-Ottmann sweep-line state.
class SegmentIntersectionState {
  public:
    mutable EventPoint curr_event; ///< The event point currently used for active-set ordering.
    std::map<EventPoint, SegmentIntersectionEvent> event_queue; ///< Pending intersection events.
    /// \brief Active segments ordered by curr_event.
    ///
    /// Remove affected segments before moving curr_event, then insert/reinsert segments
    /// that remain active below the event.
    std::vector<ActiveSegment> segments_by_id; ///< Canonicalized active segment by input index.
    std::set<SegmentId, ActiveSegmentCompare> curr_segments;

    /// \brief Initialize the sweep-line state and active/event containers.
    SegmentIntersectionState();

    /// \brief Seed the event queue with segment endpoint events.
    /// \param segments The input segments to add to the sweep.
    void populateEventQueue(const std::vector<Segment>& segments);

    /// \brief Print an active segment and its current sweep-line intersection point.
    /// \param id The active segment ID to print.
    void printElement(SegmentId id);
};

SegmentIntersectionState::SegmentIntersectionState()
    : curr_event(EventPoint(Point(INT_MAX, INT_MAX))), segments_by_id(),
      curr_segments(ActiveSegmentCompare(&curr_event, &segments_by_id)) {}

void SegmentIntersectionState::populateEventQueue(const std::vector<Segment>& segments) {
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

void SegmentIntersectionState::printElement(SegmentId id) {
    const ActiveSegment& key = segments_by_id[id];
    auto point = key.pointAtEvent(curr_event.point);
    if (point) {
        debug::segmentIntersection() << "#" << key.id << " " << key.segment.toString() << ": "
                                     << point->toString() << std::endl;
    }
}

SegmentIntersectionEvent::SegmentIntersectionEvent() : upper_segments(), witnesses() {}

/// \brief Insert a future intersection event for adjacent active segments when one exists.
/// \param predecessor One active segment adjacent to the candidate event.
/// \param successor The other active segment adjacent to the candidate event.
/// \param ls_point The current event point.
/// \param line_sweep The sweep-line state to update.
void findNewEvent(const ActiveSegment& predecessor, const ActiveSegment& successor,
                  const EventPoint& ls_point, SegmentIntersectionState& line_sweep) {
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
    if (debug::segmentIntersectionEnabled()) {
        debug::segmentIntersection()
            << "Found new event: " << ls_intersect_point.point.toString() << std::endl;
    }

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
                              const SegmentIntersectionState& line_sweep,
                              std::vector<SegmentId>& mid_segments,
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
            if (debug::segmentIntersectionEnabled()) {
                debug::segmentIntersection()
                    << "Found lower segment " << curr_segment.segment.toString() << std::endl;
            }
            lower_segments.push_back(curr_segment_id);
        } else if (isPointOnSegment(ls_point.point, curr_segment.segment)) {
            if (debug::segmentIntersectionEnabled()) {
                debug::segmentIntersection()
                    << "Found mid segment " << curr_segment.segment.toString() << std::endl;
            }
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
std::vector<SegmentId> handleEventPoint(EventPoint& ls_point, SegmentIntersectionEvent& event,
                                        SegmentIntersectionState& line_sweep) {
    if (debug::segmentIntersectionEnabled()) {
        debug::segmentIntersection() << std::endl;
        debug::segmentIntersection() << "Event: " << ls_point.point.toString() << std::endl;
    }

    auto& curr_segments = line_sweep.curr_segments;
    const auto& segments_by_id = line_sweep.segments_by_id;
    const auto& upper_segments = event.upper_segments;
    std::vector<SegmentId> mid_segments;
    std::vector<SegmentId> lower_segments;
    if (debug::segmentIntersectionEnabled()) {
        debug::segmentIntersection() << "Active segments before event:" << std::endl;
        for (SegmentId id : curr_segments) {
            line_sweep.printElement(id);
        }
    }

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
        if (debug::segmentIntersectionEnabled()) {
            debug::segmentIntersection() << "Remove lower " << s.segment.toString() << std::endl;
        }
        [[maybe_unused]] size_t count = curr_segments.erase(id);
        assert(count == 1);
    }
    for (SegmentId id : mid_segments) {
        const ActiveSegment& s = segments_by_id[id];
        if (debug::segmentIntersectionEnabled()) {
            debug::segmentIntersection() << "Remove mid " << s.segment.toString() << std::endl;
        }
        [[maybe_unused]] size_t count = curr_segments.erase(id);
        assert(count == 1);
    }

    // Move the sweep line.
    line_sweep.curr_event = ls_point;

    // Add in the upper segments and re-add the mid_segments so that they will be in
    // the correct order in curr_segments.
    for (SegmentId id : upper_segments) {
        const ActiveSegment& s = segments_by_id[id];
        if (debug::segmentIntersectionEnabled()) {
            debug::segmentIntersection() << "Insert upper " << s.segment.toString() << std::endl;
        }
        curr_segments.insert(id);
    }
    for (SegmentId id : mid_segments) {
        const ActiveSegment& s = segments_by_id[id];
        if (debug::segmentIntersectionEnabled()) {
            debug::segmentIntersection() << "Insert mid " << s.segment.toString() << std::endl;
        }
        curr_segments.insert(id);
    }

    if (debug::segmentIntersectionEnabled()) {
        debug::segmentIntersection() << "Active segments after event:" << std::endl;
        for (SegmentId id : curr_segments) {
            line_sweep.printElement(id);
        }
    }

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

} // namespace

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

    SegmentIntersectionState ls;
    ls.populateEventQueue(non_degenerate_segments);

    while (!ls.event_queue.empty()) {
        auto it = ls.event_queue.begin();
        sweep::EventPoint p = it->first;
        SegmentIntersectionEvent e = std::move(it->second);
        ls.event_queue.erase(it);

        std::vector<sweep::SegmentId> touching_segments = handleEventPoint(p, e, ls);
        if (touching_segments.size() > 1) {
            std::vector<sweep::SegmentId> original_touching_segments;
            original_touching_segments.reserve(touching_segments.size());
            for (sweep::SegmentId id : touching_segments) {
                original_touching_segments.push_back(original_ids[id]);
            }
            if (debug::segmentIntersectionEnabled()) {
                debug::segmentIntersection()
                    << "Intersection at " << p.point.toString() << std::endl;
            }
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
            } else if (intersectionType(segments[i], segments[j]) == IntersectionType::Segment) {
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
