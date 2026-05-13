#ifndef DOCUMENT_HPP
#define DOCUMENT_HPP

#include "geometry/polygon.hpp"
#include <string>
#include <vector>

/// \brief Named geometry layer in the YAML document schema.
struct Layer {
    std::string name;            ///< The display name of the layer.
    std::vector<Polygon> polygons; ///< Polygon geometry in this layer.
    std::vector<Segment> segments; ///< Segment geometry in this layer.
    std::vector<Point> points;     ///< Point geometry in this layer.

    /// \brief Compare layers by name and contained geometry.
    /// \param other The layer to compare against.
    /// \returns True if the layers contain equivalent geometry.
    bool operator==(const Layer& other) const;

    /// \brief Remove geometry that is not fully contained in a rectangle.
    /// \param rectangle The rectangle used to keep or discard geometry.
    void filter(const Rectangle& rectangle);
};

/// \brief Ordered collection of geometry layers loaded from or saved to YAML.
struct Document {
    std::vector<Layer> layers; ///< The ordered geometry layers in the document.

    /// \brief Collect points from explicit points, segment endpoints, and polygon vertices.
    /// \returns All points represented in the document.
    std::vector<Point> collectPoints() const;

    /// \brief Collect explicit segments and polygon boundary segments.
    /// \returns All segments represented in the document.
    std::vector<Segment> collectSegments() const;

    /// \brief Compare documents by ordered layers and layer contents.
    /// \param other The document to compare against.
    /// \returns True if both documents contain equivalent layers.
    bool operator==(const Document& other) const;
};

#endif // DOCUMENT_HPP
