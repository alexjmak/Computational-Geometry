#ifndef MONOTONE_PARTITION_HPP
#define MONOTONE_PARTITION_HPP

#include "geometry/polygon.hpp"
#include <vector>

/// \brief Boundary chain containing a vertex of a counter-clockwise y-monotone ring.
/// \details Left and right are viewed while walking from the ring's top vertex toward its bottom
/// vertex. Although the top and bottom are shared geometrically by both chains, each receives one
/// label so callers never need to handle an ambiguous chain: the top is Right and the bottom is
/// Left.
enum class MonotoneChain {
    Left,  ///< Chain reached by advancing through the counter-clockwise point order from the top.
    Right, ///< Chain reached by moving backward through the point order from the top.
};

/// \brief Partition a simple counter-clockwise ring into y-monotone rings.
/// \param ring Ring to partition; passed by value because collinear vertices may be removed.
/// \returns Counter-clockwise y-monotone rings whose union covers the input ring.
std::vector<LinearRing> monotonePartition(LinearRing ring);

/// \brief Classify each vertex of a counter-clockwise y-monotone ring by boundary chain.
/// \details The returned vector is index-aligned with `monotone_ring.points`: it has the same size,
/// and element `i` classifies point `i`. For a valid ring with at least three vertices, the top is
/// labeled Right, the bottom is labeled Left, and every other vertex belongs to the chain on which
/// it occurs. Rings with fewer than three vertices receive Right labels for every point.
/// \param monotone_ring The counter-clockwise y-monotone ring to classify.
/// \returns One chain label for every point in `monotone_ring`.
std::vector<MonotoneChain> extractMonotoneChains(const LinearRing& monotone_ring);

#endif
