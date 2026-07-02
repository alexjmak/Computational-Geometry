#ifndef SWEEP_LINE_HPP
#define SWEEP_LINE_HPP

#include "geometry/segment.hpp"
#include "geometry/vector.hpp"
#include <cstddef>
#include <optional>
#include <vector>

namespace sweep {

using SegmentId = std::size_t;

/// \brief Event point wrapper ordered in sweep processing order.
class EventPoint {
  public:
    Point point; ///< The geometric point represented by this sweep event.

    /// \brief Wrap a point with the event-queue ordering used by top-to-bottom sweeps.
    /// \param point The geometric point represented by this event.
    explicit EventPoint(const Point& point);

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

/// \brief Segment wrapper with cached sweep-line ordering data.
class ActiveSegment {
  public:
    SegmentId id;               ///< The original input segment index.
    Segment segment;            ///< The canonicalized segment represented by this ID.
    Rational slope_inverse;     ///< The finite inverse slope used for tie-breaking.
    int slope_inverse_infinity; ///< Direction marker for horizontal segment ordering.
    mutable std::optional<Point> cached_event; ///< Event point for the cached intersection.
    mutable std::optional<Point> cached_point_at_event; ///< Cached intersection at event.

    /// \brief Wrap a segment with sweep-specific cached data.
    /// \param id The original input segment index.
    /// \param segment The segment represented in the active set.
    ActiveSegment(SegmentId id, const Segment& segment);

    /// \brief Return this segment's intersection with the sweep line through an event.
    /// \param event_point The event point whose horizontal sweep line is being evaluated.
    /// \returns The point where the segment meets the current event's horizontal line.
    std::optional<Point> pointAtEvent(const Point& event_point) const;

    /// \brief Check whether two active segments represent the same sweep segment by ID.
    /// \param other The active segment to compare against.
    /// \returns True if both have the same id.
    bool operator==(const ActiveSegment& other) const;
};

/// \brief Compare the inverse slopes of two active segments.
/// \param a The first active segment.
/// \param b The second active segment.
/// \returns -1 if a's inverse slope is smaller, 1 if larger, or 0 if equal.
int compareSlopeInverse(const ActiveSegment& a, const ActiveSegment& b);

/// \brief Compare active segment IDs at the current sweep event.
struct ActiveSegmentCompare {
    const EventPoint* curr_event;                     ///< Current sweep event.
    const std::vector<ActiveSegment>* segments_by_id; ///< Segment geometry by ID.

    /// \brief Initialize a comparator for one sweep-line active set.
    /// \param curr_event The current event used for active-set ordering.
    /// \param segments_by_id The active segment geometry addressed by SegmentId.
    ActiveSegmentCompare(const EventPoint* curr_event,
                         const std::vector<ActiveSegment>* segments_by_id);

    /// \brief Order active segments at the current sweep event.
    /// \param a_id The first active segment ID.
    /// \param b_id The second active segment ID.
    /// \returns True if a_id should appear before b_id in the active set.
    bool operator()(SegmentId a_id, SegmentId b_id) const;
};

} // namespace sweep

#endif // SWEEP_LINE_HPP
