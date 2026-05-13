#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <boost/rational.hpp>
#include <string>

using Rational = boost::rational<long long>;

/// \brief Exact two-dimensional vector or point with rational coordinates.
class Vect2 {
  public:
    Rational x; ///< The x coordinate.
    Rational y; ///< The y coordinate.

    /// \brief Initialize a 2D vector.
    /// \param x The x coordinate.
    /// \param y The y coordinate.
    Vect2(Rational x, Rational y);

    /// \brief Initialize a 2D vector from integer coordinates.
    /// \param x The x coordinate.
    /// \param y The y coordinate.
    Vect2(int x, int y);

    /// \brief Check whether two vectors have equal coordinates.
    /// \param other The vector to compare against.
    /// \returns True if both coordinates are equal, otherwise false.
    bool operator==(const Vect2& other) const;

    /// \brief Check whether two vectors have different coordinates.
    /// \param other The vector to compare against.
    /// \returns True if at least one coordinate differs, otherwise false.
    bool operator!=(const Vect2& other) const {
        return !(*this == other);
    }

    /// \brief Order vectors lexicographically by x, then y.
    /// \param other The vector to compare against.
    /// \returns True if this vector is lexicographically smaller.
    bool operator<(const Vect2& other) const;

    /// \brief Check whether this vector is lexicographically before or equal to another.
    /// \param other The vector to compare against.
    /// \returns True if this vector is less than or equal to other.
    bool operator<=(const Vect2& other) const {
        return *this < other || *this == other;
    }

    /// \brief Check whether this vector is lexicographically greater than another.
    /// \param other The vector to compare against.
    /// \returns True if this vector is greater than other.
    bool operator>(const Vect2& other) const {
        return other < *this;
    }

    /// \brief Check whether this vector is lexicographically after or equal to another.
    /// \param other The vector to compare against.
    /// \returns True if this vector is greater than or equal to other.
    bool operator>=(const Vect2& other) const {
        return other <= *this;
    }

    /// \brief Add two vectors component-wise.
    /// \param other The vector to add.
    /// \returns The component-wise sum.
    Vect2 operator+(const Vect2& other) const;

    /// \brief Subtract two vectors component-wise.
    /// \param other The vector to subtract.
    /// \returns The component-wise difference.
    Vect2 operator-(const Vect2& other) const;

    /// \brief Scale this vector by a rational value.
    /// \param scalar The value to multiply both coordinates by.
    /// \returns The scaled vector.
    Vect2 operator*(Rational scalar) const;

    /// \brief Negate both vector coordinates.
    /// \returns The vector pointing in the opposite direction.
    Vect2 operator-() const;

    /// \brief Format the vector as a human-readable coordinate pair.
    /// \returns A string representation of the vector.
    std::string toString() const;
};

using Point = Vect2;

namespace std {

/// \brief Hash specialization for exact 2D vectors.
template <> struct hash<Vect2> {
    /// \brief Hash a vector by its exact rational coordinates.
    /// \param p The vector to hash.
    /// \returns A hash value derived from the coordinate numerators and denominators.
    size_t operator()(const Vect2& p) const;
};

} // namespace std

#endif // VECTOR_HPP
