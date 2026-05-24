#ifndef DCEL_HPP
#define DCEL_HPP

#include "geometry/polygon.hpp"
#include "geometry/segment.hpp"
#include "geometry/vector.hpp"
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

class DCEL {
  public:
    /// \brief Sentinel index used for unset DCEL references.
    static constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();
    /// \brief Face index reserved for the unbounded face.
    static constexpr std::size_t unbounded_face_index = 0;

    // DCEL cannot be copied but can be moved
    DCEL(const DCEL&) = delete;
    DCEL& operator=(const DCEL&) = delete;
    DCEL(DCEL&&) noexcept = default;
    DCEL& operator=(DCEL&&) noexcept = default;

    class HalfEdge;
    class Face;

    /// \brief A vertex with an index to one incident half-edge.
    class DCELPoint : public ::Point {
      public:
        /// \brief Initialize a DCEL vertex from exact coordinates.
        /// \param x The x coordinate.
        /// \param y The y coordinate.
        explicit DCELPoint(Rational x, Rational y);

        /// \brief Initialize a DCEL vertex from a geometric point.
        /// \param p The point to copy.
        explicit DCELPoint(const Point& p);

        std::size_t half_edge; ///< One half-edge with this point as its origin.
    };

    /// \brief A directed edge record with adjacency links.
    class HalfEdge {
      public:
        /// \brief Initialize an unlinked half-edge.
        /// \param origin The index of this half-edge's origin vertex.
        explicit HalfEdge(std::size_t origin);

        std::size_t origin; ///< The origin vertex of this directed half-edge.
        std::size_t twin;   ///< The oppositely directed half-edge.
        std::size_t next;   ///< The next half-edge around the incident face.
        std::size_t prev;   ///< The previous half-edge around the incident face.
        std::size_t face;   ///< The face incident to this half-edge.
    };

    /// \brief A face bounded by outer and optional inner components.
    class Face {
      public:
        /// \brief Initialize a face with no boundary components.
        Face();

        std::size_t outer_component;               ///< A half-edge on the outer boundary.
        std::vector<std::size_t> inner_components; ///< Half-edges on hole boundaries.
    };

    std::vector<DCELPoint> points;
    std::vector<HalfEdge> half_edges;
    std::vector<Face> faces;

    /// \brief Resolve the origin vertex of a half-edge.
    /// \param half_edge The half-edge whose origin should be returned.
    /// \returns A mutable reference to the origin vertex.
    DCELPoint& originOf(const HalfEdge& half_edge);

    /// \brief Resolve the origin vertex of a half-edge.
    /// \param half_edge The half-edge whose origin should be returned.
    /// \returns A const reference to the origin vertex.
    const DCELPoint& originOf(const HalfEdge& half_edge) const;

    /// \brief Resolve the twin half-edge of a half-edge.
    /// \param half_edge The half-edge whose twin should be returned.
    /// \returns A mutable reference to the twin half-edge.
    HalfEdge& twinOf(const HalfEdge& half_edge);

    /// \brief Resolve the twin half-edge of a half-edge.
    /// \param half_edge The half-edge whose twin should be returned.
    /// \returns A const reference to the twin half-edge.
    const HalfEdge& twinOf(const HalfEdge& half_edge) const;

    /// \brief Resolve the next half-edge around the incident face.
    /// \param half_edge The half-edge whose successor should be returned.
    /// \returns A mutable reference to the next half-edge.
    HalfEdge& nextOf(const HalfEdge& half_edge);

    /// \brief Resolve the next half-edge around the incident face.
    /// \param half_edge The half-edge whose successor should be returned.
    /// \returns A const reference to the next half-edge.
    const HalfEdge& nextOf(const HalfEdge& half_edge) const;

    /// \brief Resolve the previous half-edge around the incident face.
    /// \param half_edge The half-edge whose predecessor should be returned.
    /// \returns A mutable reference to the previous half-edge.
    HalfEdge& prevOf(const HalfEdge& half_edge);

    /// \brief Resolve the previous half-edge around the incident face.
    /// \param half_edge The half-edge whose predecessor should be returned.
    /// \returns A const reference to the previous half-edge.
    const HalfEdge& prevOf(const HalfEdge& half_edge) const;

    /// \brief Resolve the face incident to a half-edge.
    /// \param half_edge The half-edge whose incident face should be returned.
    /// \returns A mutable reference to the incident face.
    Face& faceOf(const HalfEdge& half_edge);

    /// \brief Resolve the face incident to a half-edge.
    /// \param half_edge The half-edge whose incident face should be returned.
    /// \returns A const reference to the incident face.
    const Face& faceOf(const HalfEdge& half_edge) const;

    /// \brief Convert a half-edge into its directed geometric segment.
    /// \param half_edge The half-edge to convert.
    /// \returns The segment from the half-edge origin to its twin's origin.
    Segment segmentOf(const HalfEdge& half_edge) const;

    /// \brief Convert a half-edge cycle into a geometric cycle.
    /// \param half_edge The half-edge whose cycle should be converted.
    /// \returns A cycle containing the vertices of the half-edge cycle.
    Cycle cycleOf(const HalfEdge& half_edge) const;

    /// \brief Convert a face into a polygon by converting its outer and inner cycles.
    /// \param face The face to convert.
    /// \returns A polygon with an outer cycle from the face's outer component and inner cycles
    /// from the face's inner components. The unbounded face returns std::nullopt.
    std::optional<Polygon> polygonOf(const Face& face) const;

    /// \brief Access the reserved unbounded face.
    /// \returns The face at DCEL::unbounded_face_index.
    const Face& unboundedFace() const;

    /// \brief Compute nesting depths for all faces using the unbounded face as depth zero.
    /// \returns A face-indexed depth vector; unreachable faces keep DCEL::npos.
    std::vector<std::size_t> faceDepths() const;

    /// \brief Build a DCEL from polygon boundary cycles.
    /// \param polygons The polygons whose boundaries should populate the DCEL.
    /// \returns A DCEL containing vertices, half-edges, and faces for the polygons.
    static DCEL fromPolygons(const std::vector<Polygon>& polygons);

    /// \brief Build a DCEL from boundary cycles.
    /// \param cycles The boundary cycles to populate the DCEL.
    /// \returns A DCEL containing vertices, half-edges, and faces for the cycles.
    static DCEL fromCycles(const std::vector<const Cycle*>& cycles);

    /// \brief Build a DCEL from boundary cycles.
    /// \param cycles The boundary cycles to populate the DCEL.
    /// \returns A DCEL containing vertices, half-edges, and faces for the cycles.
    static DCEL fromCycles(const std::vector<Cycle>& cycles);

    /// \brief Build a DCEL from segments by assembling them into cycles.
    /// \param segments The segments to populate the DCEL.
    /// \returns A DCEL containing vertices, half-edges, and faces for the segments.
    static DCEL fromSegments(const std::vector<Segment>& segments);

  private:
    // External code cannot create an empty/unbuilt DCEL.
    // Only the static DCEL factory methods can create one.
    DCEL() = default;
};

#endif // DCEL_HPP
