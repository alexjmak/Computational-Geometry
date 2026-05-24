#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "geometry/segment.hpp"
#include <vector>

/// \brief Closed polygonal boundary ring.
class Ring {
  public:
    std::vector<Point> points; ///< The ordered vertices of the closed ring.

    /// \brief Initialize a polygonal ring from an ordered list of points.
    /// \param points The points that define the closed ring boundary.
    Ring(std::vector<Point> points);

    /// \brief Reverse the point order of the ring. This changes the ring orientation and negates
    /// the signed area so outer rings become inner and vice versa.
    void reverse();

    /// \brief Compute the signed area of the ring.
    /// \returns The signed area. Positive values indicate counter-clockwise point order,
    /// and negative values indicate clockwise point order.
    double signedArea() const;

    /// \brief Compute the unsigned area of the ring.
    /// \returns The absolute area enclosed by the ring.
    double area() const;

    /// \brief Check whether the ring is oriented counter-clockwise.
    /// \returns True if the ring has positive signed area, otherwise false.
    bool isOuter() const;

    /// \brief Convert the ring into its directed boundary segments.
    /// \returns A list of segments connecting consecutive points in the ring.
    std::vector<Segment> segments() const;

    /// \brief Check whether two rings contain the same ordered vertices.
    /// \param other The ring to compare against.
    /// \returns True when both rings have identical point order.
    bool operator==(const Ring& other) const;
};

/// \brief Polygon represented by an outer boundary and optional inner hole rings.
class Polygon {
  public:
    Ring outer_ring;               ///< The outer boundary ring.
    std::vector<Ring> inner_rings; ///< The inner hole boundary rings.

    /// \brief Initialize a polygon from an outer ring and optional inner rings.
    /// \param outer_ring The outer boundary ring.
    /// \param inner_rings The inner hole boundary rings.
    Polygon(Ring outer_ring, std::vector<Ring> inner_rings = {});

    /// \brief Get pointers to all boundary rings, with the outer ring first.
    /// \returns A vector containing the outer ring followed by all inner rings.
    std::vector<Ring*> rings();

    /// \brief Get const pointers to all boundary rings, with the outer ring first.
    /// \returns A vector containing the outer ring followed by all inner rings.
    std::vector<const Ring*> rings() const;

    /// \brief Compute the signed area contributed by all polygon rings.
    /// \returns The sum of the signed areas of the polygon's rings.
    double area() const;

    /// \brief Check whether two polygons contain the same ordered rings.
    /// \param other The polygon to compare against.
    /// \returns True when both polygons have identical ring order and vertices.
    bool operator==(const Polygon& other) const;
};

/// \brief Axis-aligned rectangle represented by exact corner coordinates.
class Rectangle {
  public:
    Point lower_left;  ///< The lower-left corner of the rectangle.
    Point upper_right; ///< The upper-right corner of the rectangle.

    /// \brief Initialize an axis-aligned rectangle from lower-left and upper-right corners.
    /// \param lower_left The lower-left corner of the rectangle.
    /// \param upper_right The upper-right corner of the rectangle.
    Rectangle(Point lower_left, Point upper_right);

    /// \brief Convert the rectangle boundary into a counter-clockwise ring.
    /// \returns A ring containing the four rectangle corners.
    Ring ring() const;

    /// \brief Convert the rectangle into the polygon representation.
    /// \returns A polygon with one outer rectangular ring.
    Polygon polygon() const;

    /// \brief Check whether a point is inside or on the rectangle boundary.
    /// \param p The point to test.
    /// \returns True if the point is contained in the rectangle, otherwise false.
    bool contains(const Point& p) const;

    /// \brief Check whether a segment is fully contained in the rectangle.
    /// \param s The segment to test.
    /// \returns True if both segment endpoints are contained in the rectangle, otherwise false.
    bool contains(const Segment& s) const;

    /// \brief Check whether a polygon is fully contained in the rectangle.
    /// \param polygon The polygon to test.
    /// \returns True if every polygon vertex is contained in the rectangle, otherwise false.
    bool contains(const Polygon& polygon) const;

    /// \brief Compute the area of the rectangle.
    /// \returns The rectangle area.
    double area() const;
};

#endif // POLYGON_HPP
