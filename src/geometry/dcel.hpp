#ifndef DCEL_HPP
#define DCEL_HPP

#include "geometry/polygon.hpp"
#include "geometry/vector.hpp"
#include <vector>

class DCEL {
  public:
    DCEL() = default;
    DCEL(const DCEL&) = delete;
    DCEL& operator=(const DCEL&) = delete;
    DCEL(DCEL&&) noexcept = default;
    DCEL& operator=(DCEL&&) noexcept = default;

    class HalfEdge;
    class Face;

    /// \brief A vertex with a pointer to one incident half-edge.
    class DCELPoint : public ::Point {
      public:
        /// \brief Initialize a DCEL vertex from exact coordinates.
        /// \param x The x coordinate.
        /// \param y The y coordinate.
        explicit DCELPoint(Rational x, Rational y);
        explicit DCELPoint(const Point& p);

        HalfEdge* half_edge; ///< One half-edge with this point as its origin.
    };

    /// \brief A directed edge record with adjacency links.
    class HalfEdge {
      public:
        /// \brief Initialize an unlinked half-edge.
        explicit HalfEdge(DCELPoint* origin);

        DCELPoint* origin; ///< The origin vertex of this directed half-edge.
        HalfEdge* twin;    ///< The oppositely directed half-edge.
        HalfEdge* next;    ///< The next half-edge around the incident face.
        HalfEdge* prev;    ///< The previous half-edge around the incident face.
        Face* face;        ///< The face incident to this half-edge.
    };

    /// \brief A face bounded by outer and optional inner components.
    class Face {
      public:
        /// \brief Initialize a face with no boundary components.
        Face();

        HalfEdge* outer_component;               ///< A half-edge on the outer boundary.
        std::vector<HalfEdge*> inner_components; ///< Half-edges on hole boundaries.
    };

    std::vector<DCELPoint> points;
    std::vector<HalfEdge> half_edges;
    std::vector<Face> faces;

    static DCEL fromPolygons(const std::vector<Polygon>& polygons);
};

#endif // DCEL_HPP
