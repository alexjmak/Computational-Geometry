#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "geometry/segment.hpp"
#include <vector>

/// \brief Closed polygonal boundary cycle.
class Cycle {
  public:
    std::vector<Point> points; ///< The ordered vertices of the closed cycle.

    /// \brief Initialize a polygonal cycle from an ordered list of points.
    /// \param points The points that define the closed cycle boundary.
    Cycle(std::vector<Point> points);

    /// \brief Compute the signed area of the cycle.
    /// \returns The signed area. Positive values indicate counter-clockwise point order,
    /// and negative values indicate clockwise point order.
    double signedArea() const;

    /// \brief Compute the unsigned area of the cycle.
    /// \returns The absolute area enclosed by the cycle.
    double area() const;

    /// \brief Check whether the cycle is oriented counter-clockwise.
    /// \returns True if the cycle has positive signed area, otherwise false.
    bool isOuter() const;

    /// \brief Convert the cycle into its directed boundary segments.
    /// \returns A list of segments connecting consecutive points in the cycle.
    std::vector<Segment> segments() const;

    /// \brief Check whether two cycles contain the same ordered vertices.
    /// \param other The cycle to compare against.
    /// \returns True when both cycles have identical point order.
    bool operator==(const Cycle& other) const;
};

/// \brief Polygon represented by one or more boundary cycles.
class Polygon {
  public:
    std::vector<Cycle> cycles; ///< The boundary cycles that make up the polygon.

    /// \brief Initialize a polygon from one or more boundary cycles.
    /// \param cycles The boundary cycles that make up the polygon.
    Polygon(std::vector<Cycle> cycles);

    /// \brief Compute the signed area contributed by all polygon cycles.
    /// \returns The sum of the signed areas of the polygon's cycles.
    double area() const;

    /// \brief Check whether two polygons contain the same ordered cycles.
    /// \param other The polygon to compare against.
    /// \returns True when both polygons have identical cycle order and vertices.
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

    /// \brief Convert the rectangle boundary into a counter-clockwise cycle.
    /// \returns A cycle containing the four rectangle corners.
    Cycle cycle() const;

    /// \brief Convert the rectangle into the polygon representation.
    /// \returns A polygon with one outer rectangular cycle.
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
