#ifndef ASSEMBLE_HPP
#define ASSEMBLE_HPP

#include "geometry/polygon.hpp"

/// \brief Assembles cycles from unordered segments forming one or more closed boundaries.
/// Segment orientation is ignored during assembly.
/// \param segments Segments forming one or more closed cycles.
/// \returns Assembled cycles oriented with positive signed area.
std::vector<Cycle> assembleCycles(const std::vector<Segment>& segments);

/// \brief Assembles polygons from unordered segments forming one or more closed boundaries.
/// Segment orientation is ignored during assembly. Polygons are classified using the odd-even
/// fill rule, producing outer cycles and inner cycles (holes) as appropriate.
/// \param segments Segments forming one or more closed polygon boundaries.
/// \returns Assembled polygons with consistently oriented outer boundaries.
std::vector<Polygon> assemblePolygons(const std::vector<Segment>& segments);

#endif // ASSEMBLE_HPP
