//! Points and vectors for geometric computations.
//!
//! This module defines the foundational geometric primitives:
//!
//! - [`Vector`] - a type in a linear (vector) space with algebraic operations
//!   (`Add`, `Sub`, `Mul` by scalar) plus `dot` and `norm`.
//! - [`Point`] - a type in an affine space with `displacement` (→ [`Vector`]) and
//!   `translate` (← [`Vector`]), plus a default `distance` implementation.
//!
//! Points and vectors are kept distinct because a position on a path is a point,
//! while derivatives (tangent, curvature vector) are vectors.

use crate::Scalar;

/// A displacement or derivative in Euclidean space.
///
/// Vectors represent displacements and derivatives (e.g. tangent vectors).
/// They form a linear space: addition, subtraction, and scalar multiplication
/// are required, along with `dot` product and Euclidean `norm`.
pub trait Vector:
    Copy
    + core::ops::Add<Output = Self>
    + core::ops::Sub<Output = Self>
    + core::ops::Mul<<Self as Vector>::Scalar, Output = Self>
{
    /// The scalar type used for components and magnitudes.
    type Scalar: Scalar;

    /// The zero vector (additive identity).
    fn zero() -> Self;

    /// Dot product of two vectors.
    fn dot(self, rhs: Self) -> Self::Scalar;

    /// Euclidean norm (magnitude) of the vector.
    fn norm(self) -> Self::Scalar;
}

/// A position in an affine space, parameterized by its scalar and vector types.
///
/// Points represent positions. They cannot be added, but a displacement from one
/// point to another yields a [`Vector`], and translating a point by a [`Vector`]
/// yields another point.
pub trait Point: Copy {
    /// Scalar type for distances and coordinates.
    type Scalar: Scalar;

    /// Vector type for displacements and derivatives.
    type Vector: Vector<Scalar = Self::Scalar>;

    /// Displacement from `self` to `other` (i.e. `other - self`).
    fn displacement(self, other: Self) -> Self::Vector;

    /// Translate this point by a vector, returning a new point.
    fn translate(self, v: Self::Vector) -> Self;

    /// Euclidean distance between two points.
    fn distance(self, other: Self) -> Self::Scalar {
        self.displacement(other).norm()
    }
}
