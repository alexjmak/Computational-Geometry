#include "algorithms/horizontal_ray_query.hpp"
#include "algorithms/sweep_line.hpp"
#include "geometry/predicates.hpp"
#include <cassert>
#include <climits>
#include <cstddef>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <vector>

/*
 * Horizontal ray query
 *
 * This file answers a batch of horizontal ray queries against a segment set. It reuses the
 * shared sweep-line ordering helpers: events are processed from top to bottom, and active segments
 * are ordered by their x-coordinate on the horizontal line through the current event.
 *
 * For each query point, the algorithm temporarily inserts a zero-length dummy segment at the query
 * point into the ordering space. The predecessor of that dummy in the active set is the nearest
 * candidate to the left or right. Horizontal segments are ignored because they do not produce
 * strict left or right hits.
 */

namespace {

using sweep::ActiveSegment;
using sweep::ActiveSegmentCompare;
using sweep::EventPoint;
using sweep::SegmentId;

/// \brief Event-queue payload for one segment endpoint or query point.
class RayHitEvent {
  public:
    std::vector<SegmentId> upper_segments; ///< Segments whose upper endpoint is this event.
    std::vector<SegmentId> lower_segments; ///< Segments whose lower endpoint is this event.
    std::vector<std::size_t> query_ids;    ///< Query indices whose ray origin is this event.
};

/// \brief Mutable sweep-line state for batched left-ray queries.
class RayHitState {
  public:
    mutable EventPoint curr_event; ///< The event point currently used for active-set ordering.
    std::map<EventPoint, RayHitEvent> event_queue; ///< Pending endpoint and query events.
    /// \brief Active segments ordered by curr_event.
    ///
    /// Segments are removed at their lower endpoint and inserted at their upper endpoint. Query
    /// events read this set without changing it.
    std::vector<ActiveSegment> segments_by_id; ///< Canonicalized active segment by input index.
    std::set<SegmentId, ActiveSegmentCompare> curr_segments;

    /// \brief Initialize the sweep-line state and active/event containers.
    RayHitState();

    /// \brief Seed the event queue with segment endpoint events.
    /// \param segments The input segments to add to the sweep.
    /// \param queries Query points whose leftward rays should be answered.
    void populateEventQueue(const std::vector<Segment>& segments,
                            const std::vector<Point>& queries);

    /// \brief Print an active segment and its current sweep-line intersection point.
    /// \param id The active segment ID to print.
    void printElement(SegmentId id);
};

RayHitState::RayHitState()
    : curr_event(EventPoint(Point(INT_MAX, INT_MAX))), segments_by_id(),
      curr_segments(ActiveSegmentCompare(&curr_event, &segments_by_id)) {}

void RayHitState::populateEventQueue(const std::vector<Segment>& segments,
                                     const std::vector<Point>& queries) {
    segments_by_id.reserve(segments.size());
    for (SegmentId id = 0; id < segments.size(); ++id) {
        segments_by_id.emplace_back(id, segments[id].canonicalizedY());
        const ActiveSegment& lss = segments_by_id.back();
        EventPoint upper_endpoint(lss.segment.end);
        EventPoint lower_endpoint(lss.segment.start);
        auto& upper_event = event_queue.try_emplace(upper_endpoint).first->second;
        upper_event.upper_segments.push_back(id);
        auto& lower_event = event_queue.try_emplace(lower_endpoint).first->second;
        lower_event.lower_segments.push_back(id);
    }
    for (std::size_t id = 0; id < queries.size(); ++id) {
        EventPoint query_event_point(queries[id]);
        auto& query_event = event_queue.try_emplace(query_event_point).first->second;
        query_event.query_ids.push_back(id);
    }

    if (!event_queue.empty()) {
        curr_event = event_queue.begin()->first;
    }
}

void RayHitState::printElement(SegmentId id) {
    const ActiveSegment& key = segments_by_id[id];
    auto point = key.pointAtEvent(curr_event.point);
    if (point) {
        std::cout << "#" << key.id << " " << key.segment.toString() << ": " << point->toString()
                  << std::endl;
    }
}

/// \brief Process one sweep event and return a horizontal ray hit when the event is a query.
/// \param ls_point The endpoint or query point being processed.
/// \param event The event data associated with the point.
/// \param line_sweep The sweep-line state to update.
/// \returns The nearest active segment strictly left of a query event, or std::nullopt.
std::optional<SegmentId> handleEventPoint(EventPoint& ls_point, RayHitEvent& event,
                                          RayHitState& line_sweep) {
#ifndef NDEBUG
    std::cout << std::endl;
    std::cout << "Event: " << ls_point.point.toString() << std::endl;
#endif

    auto& curr_segments = line_sweep.curr_segments;
    const auto& segments_by_id = line_sweep.segments_by_id;
    const auto& upper_segments = event.upper_segments;
    const auto& lower_segments = event.lower_segments;
    const bool is_query = !event.query_ids.empty();

#ifndef NDEBUG
    std::cout << "Active segments before event:" << std::endl;
    for (SegmentId id : curr_segments) {
        line_sweep.printElement(id);
    }
#endif

    for (SegmentId id : lower_segments) {
        const ActiveSegment& s = segments_by_id[id];
#ifndef NDEBUG
        std::cout << "Remove lower " << s.segment.toString() << std::endl;
#endif
        [[maybe_unused]] size_t count = curr_segments.erase(id);
        assert(count == 1);
    }

    // Move the sweep line.
    line_sweep.curr_event = ls_point;

    // Insert segments that begin at this event so they are active below the current sweep line.
    for (SegmentId id : upper_segments) {
        const ActiveSegment& s = segments_by_id[id];
#ifndef NDEBUG
        std::cout << "Insert upper " << s.segment.toString() << std::endl;
#endif
        curr_segments.insert(id);
    }

#ifndef NDEBUG
    std::cout << "Active segments after event:" << std::endl;
    for (SegmentId id : curr_segments) {
        line_sweep.printElement(id);
    }
#endif

    std::optional<SegmentId> result;
    if (is_query) {
        const SegmentId dummy_segment_id = segments_by_id.size();
        line_sweep.segments_by_id.emplace_back(dummy_segment_id,
                                               Segment(ls_point.point, ls_point.point));

        // The dummy sorts at the query x-coordinate. Its predecessor is the nearest candidate to
        // the left in the active set; walk farther left when ties or endpoint touches are not
        // strict left hits.
        auto it = curr_segments.lower_bound(dummy_segment_id);

        while (it != curr_segments.begin()) {
            --it;
            const ActiveSegment& candidate = segments_by_id[*it];
            if (candidate.segment.start.y == candidate.segment.end.y) {
                // Horizontal segments are not strict left hits, so skip them.
                continue;
            }
            std::optional<Point> hit = candidate.pointAtEvent(ls_point.point);
            if (hit && hit->x < ls_point.point.x) {
                result = *it;
                break;
            }
        }
        line_sweep.segments_by_id.pop_back();
    }

    return result;
}
} // namespace

std::vector<std::optional<SegmentId>> leftRayQuery(const std::vector<Segment>& segments,
                                                   const std::vector<Point>& queries) {
    RayHitState ls;
    ls.populateEventQueue(segments, queries);

    std::vector<std::optional<SegmentId>> left_ray_hits(queries.size());
    while (!ls.event_queue.empty()) {
        auto it = ls.event_queue.begin();
        sweep::EventPoint p = it->first;
        RayHitEvent e = std::move(it->second);
        ls.event_queue.erase(it);

        std::optional<SegmentId> left_ray_hit = handleEventPoint(p, e, ls);

        for (std::size_t query_id : e.query_ids) {
            left_ray_hits[query_id] = left_ray_hit;
        }
    }

    return left_ray_hits;
}
