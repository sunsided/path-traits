//! Core traits for parametric curves.
//!
//! This module defines the two foundational traits of the crate:
//!
//! - [`Path`] - the core trait for arc-length-parameterized curves. It requires
//!   `length()`, `sample_at(s)`, and provides `start()`, `end()`, `domain()`.
//! - [`ParametricPath`] - extends [`Path`] with normalized-parameter sampling
//!   `sample_t(t) ∈ [0, 1]`, plus conversion helpers `t_to_s` / `s_to_t`.

use crate::PathError;
use crate::Point;
use crate::Scalar;

/// A curve that can be sampled by arc-length `s`.
///
/// Every path has a total `length`, and `sample_at(s)` returns a [`Point`](crate::Point)
/// for `s ∈ [0, length]`. The trait provides convenience methods `start()`, `end()`,
/// and `domain()`.
pub trait Path {
    /// The scalar type for arc-length, parameter, and distance computations.
    type Scalar: Scalar;
    /// The point type representing positions on the path.
    type Point: Point<Scalar = Self::Scalar>;
    /// The error type for fallible operations. Must be convertible from [`PathError`].
    type Error: From<PathError>;

    /// Total arc-length of the path.
    fn length(&self) -> Self::Scalar;

    /// Sample the path at arc-length `s ∈ [0, length]`.
    ///
    /// Returns an error when `s` is outside the valid domain.
    fn sample_at(&self, s: Self::Scalar) -> Result<Self::Point, Self::Error>;

    /// The start point of the path, equivalent to `sample_at(0)`.
    fn start(&self) -> Result<Self::Point, Self::Error> {
        self.sample_at(Self::Scalar::zero())
    }

    /// The end point of the path, equivalent to `sample_at(length)`.
    fn end(&self) -> Result<Self::Point, Self::Error> {
        self.sample_at(self.length())
    }

    /// The valid domain for arc-length sampling: `[0, length]`.
    fn domain(&self) -> core::ops::RangeInclusive<Self::Scalar> {
        Self::Scalar::zero()..=self.length()
    }
}

/// A curve that also supports sampling by a normalized parameter `t ∈ [0, 1]`.
///
/// Extends [`Path`] with `sample_t(t)`. Default implementations of `t_to_s` and
/// `s_to_t` use linear conversion (`s = t * length`), which is exact only for
/// arc-length-parameterized paths. Implementers may override these for exact
/// conversions.
pub trait ParametricPath: Path {
    /// Sample the path at normalized parameter `t ∈ [0, 1]`.
    ///
    /// Returns an error when `t` is outside the valid domain.
    fn sample_t(&self, t: Self::Scalar) -> Result<Self::Point, Self::Error>;

    /// Convert normalized parameter `t` to arc-length `s`.
    ///
    /// Default: `t * length()`. Override for exact conversions.
    fn t_to_s(&self, t: Self::Scalar) -> Self::Scalar {
        t * self.length()
    }

    /// Convert arc-length `s` to normalized parameter `t`.
    ///
    /// Default: `s / length()`. Override for exact conversions.
    fn s_to_t(&self, s: Self::Scalar) -> Self::Scalar {
        s / self.length()
    }
}
