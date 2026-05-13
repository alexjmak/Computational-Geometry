#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include "geometry/vector.hpp"
#include <string>

/// \brief Directed line segment between two exact endpoints.
class Segment {
  public:
    Point start; ///< The directed segment's start endpoint.
    Point end;   ///< The directed segment's end endpoint.

    /// \brief Initialize a directed segment.
    /// \param start The starting endpoint.
    /// \param end The ending endpoint.
    Segment(Point start, Point end);

    /// \brief Check whether the segment endpoints are in lexicographic order.
    /// \returns True if start is less than or equal to end, otherwise false.
    bool isCanonicalizedX() const;

    /// \brief Check whether the segment endpoints are ordered bottom-to-top for sweep use.
    /// \returns True if start is below end, breaking horizontal ties from right to left.
    bool isCanonicalizedY() const;

    /// \brief Return a copy of the segment with lexicographically ordered endpoints.
    /// \returns A segment whose start endpoint is less than or equal to its end endpoint.
    Segment canonicalizedX() const;

    /// \brief Return a copy of the segment ordered bottom-to-top for sweep use.
    /// \returns A segment with endpoints ordered by y, with horizontal ties right-to-left.
    Segment canonicalizedY() const;

    /// \brief Check whether two directed segments have equal endpoints.
    /// \param other The segment to compare against.
    /// \returns True if both start and end endpoints are equal.
    bool operator==(const Segment& other) const;

    /// \brief Check whether two directed segments have different endpoints.
    /// \param other The segment to compare against.
    /// \returns True if either endpoint differs.
    bool operator!=(const Segment& other) const {
        return !(*this == other);
    }

    /// \brief Order segments lexicographically by start endpoint, then end endpoint.
    /// \param other The segment to compare against.
    /// \returns True if this segment is lexicographically smaller.
    bool operator<(const Segment& other) const;

    /// \brief Check whether this segment is lexicographically before or equal to another.
    /// \param other The segment to compare against.
    /// \returns True if this segment is less than or equal to other.
    bool operator<=(const Segment& other) const {
        return *this < other || *this == other;
    }

    /// \brief Check whether this segment is lexicographically greater than another.
    /// \param other The segment to compare against.
    /// \returns True if this segment is greater than other.
    bool operator>(const Segment& other) const {
        return other < *this;
    }

    /// \brief Check whether this segment is lexicographically after or equal to another.
    /// \param other The segment to compare against.
    /// \returns True if this segment is greater than or equal to other.
    bool operator>=(const Segment& other) const {
        return other <= *this;
    }

    /// \brief Format the segment as a human-readable endpoint pair.
    /// \returns A string representation of the segment.
    std::string toString() const;
};

namespace std {

/// \brief Hash specialization for directed segments.
template <> struct hash<Segment> {
    /// \brief Hash a segment by its directed endpoints.
    /// \param s The segment to hash.
    /// \returns A hash value derived from the segment endpoints.
    size_t operator()(const Segment& s) const;
};

} // namespace std

#endif // SEGMENT_HPP
