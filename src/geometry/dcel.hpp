#ifndef DCEL_HPP
#define DCEL_HPP

#include "geometry/vector.hpp"
#include <vector>

class DCEL {
  public:
    class HalfEdge;
    class Face;

    /// \brief A vertex with a pointer to one incident half-edge.
    class Point : public ::Point {
      public:
        /// \brief Initialize a DCEL vertex from exact coordinates.
        /// \param x The x coordinate.
        /// \param y The y coordinate.
        Point(Rational x, Rational y);

        HalfEdge* half_edge; ///< One half-edge with this point as its origin.
    };

    /// \brief A directed edge record with adjacency links.
    class HalfEdge {
      public:
        /// \brief Initialize an unlinked half-edge.
        HalfEdge();

        Point* origin;   ///< The origin vertex of this directed half-edge.
        HalfEdge* twin;  ///< The oppositely directed half-edge.
        HalfEdge* next;  ///< The next half-edge around the incident face.
        HalfEdge* prev;  ///< The previous half-edge around the incident face.
        Face* face;      ///< The face incident to this half-edge.
    };

    /// \brief A face bounded by outer and optional inner components.
    class Face {
      public:
        /// \brief Initialize a face with no boundary components.
        Face();

        HalfEdge* outer_component; ///< A half-edge on the outer boundary.
        std::vector<HalfEdge*> inner_components; ///< Half-edges on hole boundaries.
    };
};

#endif // DCEL_HPP
