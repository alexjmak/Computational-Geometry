#include "algorithms/monotone_partition.hpp"
#include "algorithms/assemble.hpp"
#include "algorithms/sweep_line.hpp"
#include "geometry/polygon.hpp"
#include "geometry/predicates.hpp"
#include "geometry/vector.hpp"
#include <map>
#include <optional>
#include <set>

namespace {

using sweep::ActiveSegment;
using sweep::ActiveSegmentCompare;
using sweep::EventPoint;
using sweep::SegmentId;

enum class MonotoneVertexType { Start, End, Split, Merge, Regular };

/// \brief A sweep event annotated with its y-monotone vertex classification.
class MonotoneEventPoint : public EventPoint {
  public:
    MonotoneVertexType vertex_type;

    /// \brief Wrap a point with its classification while preserving sweep ordering.
    MonotoneEventPoint(const Point& point, MonotoneVertexType vertex_type)
        : EventPoint(point), vertex_type(vertex_type) {}
};

MonotoneVertexType classifyMonotoneVertex(const Point& prev, const Point& curr, const Point& next) {
    const Rational turn = orientation(prev, curr, next);

    // Event point comparator ordering is top-to-bottom, left-to-right.
    const EventPoint prev_event(prev);
    const EventPoint curr_event(curr);
    const EventPoint next_event(next);

    // curr is below both neighbors: local minimum
    if (prev_event < curr_event && next_event < curr_event) {
        return turn > 0 ? MonotoneVertexType::End : MonotoneVertexType::Merge;
    }

    // curr is above both neighbors: local maximum
    if (curr_event < prev_event && curr_event < next_event) {
        return turn > 0 ? MonotoneVertexType::Start : MonotoneVertexType::Split;
    }

    return MonotoneVertexType::Regular;
}

class MonotoneEvent {
  public:
    SegmentId prev_segment_id;
    SegmentId curr_segment_id;
    bool interior_lies_to_right;
};

/// \brief Mutable sweep-line state for batched left-ray queries.
class MonotoneState {
  public:
    mutable EventPoint curr_event; ///< The event point currently used for active-set ordering.
    std::map<MonotoneEventPoint, MonotoneEvent> event_queue; ///< Pending endpoint events.
    /// \brief Active segments ordered by curr_event.
    ///
    /// Segments are removed at their lower endpoint and inserted at their upper endpoint. Query
    /// events read this set without changing it.
    std::vector<ActiveSegment> segments_by_id; ///< Canonicalized active segment by input index.
    /// Helper vertex for each active segment, if the segment is active.
    std::vector<std::optional<MonotoneEventPoint>> helper_by_id;
    std::set<SegmentId, ActiveSegmentCompare> curr_segments;

    /// \brief Initialize the sweep-line state and active/event containers.
    MonotoneState();

    /// \brief Seed the event queue with segment endpoint events.
    /// \param ring The input ring to add to the sweep.
    void populateEventQueue(const LinearRing& ring);
};

MonotoneState::MonotoneState()
    : curr_event(EventPoint(Point(INT_MAX, INT_MAX))), segments_by_id(),
      curr_segments(ActiveSegmentCompare(&curr_event, &segments_by_id, true)), helper_by_id() {}

void MonotoneState::populateEventQueue(const LinearRing& ring) {
    segments_by_id.reserve(ring.points.size());
    helper_by_id.resize(ring.points.size());

    for (std::size_t i = 0; i < ring.points.size(); ++i) {
        const std::size_t prev_i = (i + ring.points.size() - 1) % ring.points.size();
        const std::size_t next_i = (i + 1) % ring.points.size();
        const Point& prev = ring.points[prev_i];
        const Point& curr = ring.points[i];
        const Point& next = ring.points[next_i];

        MonotoneVertexType vertex_type = classifyMonotoneVertex(prev, curr, next);

        SegmentId prev_segment = prev_i;
        SegmentId curr_segment = i;

        segments_by_id.emplace_back(curr_segment, Segment(curr, next));

        MonotoneEventPoint event_point(curr, vertex_type);
        const bool interior_lies_to_right = event_point < EventPoint(next);
        MonotoneEvent event{prev_segment, curr_segment, interior_lies_to_right};
        event_queue.try_emplace(event_point, event);
    }

    if (!event_queue.empty()) {
        curr_event = event_queue.begin()->first;
    }
}

void insertDiagonal(MonotoneState& line_sweep, const MonotoneEventPoint& pt1,
                    const MonotoneEventPoint& pt2) {
    SegmentId diagonal_id = line_sweep.segments_by_id.size();
    line_sweep.segments_by_id.emplace_back(diagonal_id, Segment(pt1.point, pt2.point));
    line_sweep.helper_by_id.emplace_back();
}

bool linkIfMerge(const MonotoneEventPoint& helper, const MonotoneEventPoint& ls_point,
                 MonotoneState& line_sweep) {
    if (helper.vertex_type == MonotoneVertexType::Merge) {
        insertDiagonal(line_sweep, ls_point, helper);
        return true;
    }

    return false;
}

void insertSegmentToStatus(const SegmentId& segment_id, const MonotoneEventPoint& ls_point,
                           MonotoneState& line_sweep) {
    line_sweep.curr_segments.insert(segment_id);
    line_sweep.helper_by_id[segment_id] = ls_point;
}

void handleStartVertex(const MonotoneEventPoint& ls_point, MonotoneEvent& event,
                       MonotoneState& line_sweep) {
    assert(ls_point.vertex_type == MonotoneVertexType::Start);
    insertSegmentToStatus(event.curr_segment_id, ls_point, line_sweep);
}

void handleEndVertex(const MonotoneEventPoint& ls_point, MonotoneEvent& event,
                     MonotoneState& line_sweep) {
    assert(ls_point.vertex_type == MonotoneVertexType::End);
    MonotoneEventPoint& helper = *line_sweep.helper_by_id[event.prev_segment_id];
    linkIfMerge(helper, ls_point, line_sweep);
    [[maybe_unused]] std::size_t count = line_sweep.curr_segments.erase(event.prev_segment_id);
    assert(count == 1);
}

void handleSplitVertex(const MonotoneEventPoint& ls_point, MonotoneEvent& event,
                       MonotoneState& line_sweep) {
    assert(ls_point.vertex_type == MonotoneVertexType::Split);
    std::optional<SegmentId> left_segment = sweep::nearestActiveSegmentToLeft(
        ls_point.point, line_sweep.curr_segments, line_sweep.segments_by_id);
    if (left_segment) {
        MonotoneEventPoint& helper = *line_sweep.helper_by_id[*left_segment];
        insertDiagonal(line_sweep, ls_point, helper);
        line_sweep.helper_by_id[*left_segment] = ls_point;
    }
    insertSegmentToStatus(event.curr_segment_id, ls_point, line_sweep);
}

void handleMergeVertex(const MonotoneEventPoint& ls_point, MonotoneEvent& event,
                       MonotoneState& line_sweep) {
    assert(ls_point.vertex_type == MonotoneVertexType::Merge);
    MonotoneEventPoint& prev_segment_helper = *line_sweep.helper_by_id[event.prev_segment_id];
    linkIfMerge(prev_segment_helper, ls_point, line_sweep);
    [[maybe_unused]] std::size_t count = line_sweep.curr_segments.erase(event.prev_segment_id);
    assert(count == 1);

    std::optional<SegmentId> left_segment = sweep::nearestActiveSegmentToLeft(
        ls_point.point, line_sweep.curr_segments, line_sweep.segments_by_id);
    if (left_segment) {
        MonotoneEventPoint& left_segment_helper = *line_sweep.helper_by_id[*left_segment];
        linkIfMerge(left_segment_helper, ls_point, line_sweep);
        left_segment_helper = ls_point;
    }
}

void handleRegularVertex(const MonotoneEventPoint& ls_point, MonotoneEvent& event,
                         MonotoneState& line_sweep) {
    assert(ls_point.vertex_type == MonotoneVertexType::Regular);
    if (event.interior_lies_to_right) {
        MonotoneEventPoint& prev_segment_helper = *line_sweep.helper_by_id[event.prev_segment_id];
        linkIfMerge(prev_segment_helper, ls_point, line_sweep);
        [[maybe_unused]] std::size_t count = line_sweep.curr_segments.erase(event.prev_segment_id);
        assert(count == 1);
        insertSegmentToStatus(event.curr_segment_id, ls_point, line_sweep);
    } else {
        std::optional<SegmentId> left_segment = sweep::nearestActiveSegmentToLeft(
            ls_point.point, line_sweep.curr_segments, line_sweep.segments_by_id);
        if (left_segment) {
            MonotoneEventPoint& left_segment_helper = *line_sweep.helper_by_id[*left_segment];
            linkIfMerge(left_segment_helper, ls_point, line_sweep);
            left_segment_helper = ls_point;
        }
    }
}

void handleEventPoint(const MonotoneEventPoint& ls_point, MonotoneEvent& event,
                      MonotoneState& line_sweep) {
    // Move the sweep line.
    line_sweep.curr_event = ls_point;

    switch (ls_point.vertex_type) {

    case MonotoneVertexType::Start:
        handleStartVertex(ls_point, event, line_sweep);
        break;
    case MonotoneVertexType::End:
        handleEndVertex(ls_point, event, line_sweep);
        break;
    case MonotoneVertexType::Split:
        handleSplitVertex(ls_point, event, line_sweep);
        break;
    case MonotoneVertexType::Merge:
        handleMergeVertex(ls_point, event, line_sweep);
        break;
    case MonotoneVertexType::Regular:
        handleRegularVertex(ls_point, event, line_sweep);
        break;
    }
}
} // namespace

std::vector<LinearRing> monotonePartition(LinearRing ring) {
    ring.removeCollinearVertices();

    MonotoneState ls;
    ls.populateEventQueue(ring);

    while (!ls.event_queue.empty()) {
        auto it = ls.event_queue.begin();
        MonotoneEventPoint p = it->first;
        MonotoneEvent e = std::move(it->second);
        ls.event_queue.erase(it);

        handleEventPoint(p, e, ls);
    }

    std::vector<Segment> monotone_segments;
    for (const ActiveSegment& s : ls.segments_by_id) {
        monotone_segments.push_back(s.segment);
    }

    return assembleRings(monotone_segments);
}
