//! Offset a path by a constant distance.
//!
//! [`Offset`] wraps a path and displaces every sampled point by a fixed
//! distance. Requires the inner path to implement
//! [`Tangent`](crate::Tangent) and [`Heading`](crate::Heading).

use crate::{Heading, Path, Point, Tangent};

/// A path displaced by a constant distance from the original.
///
/// Every sampled point is shifted by `distance` units. The offset direction
/// is derived from the path's heading; in 2D this produces a parallel curve.
///
/// **Note:** The current implementation offsets along the tangent direction as
/// a placeholder. A proper 2D-specific implementation should offset along the
/// surface normal (heading + π/2).
pub struct Offset<P, S> {
    /// The inner path being offset.
    inner: P,
    /// The constant offset distance.
    distance: S,
}

impl<P, S> Offset<P, S> {
    /// Create a new offset path.
    ///
    /// The `distance` is in the same units as the path's scalar type.
    pub fn new(inner: P, distance: S) -> Self {
        Self { inner, distance }
    }
}

impl<P: Path + Tangent + Heading> Path for Offset<P, P::Scalar> {
    type Scalar = P::Scalar;
    type Point = P::Point;
    type Error = P::Error;

    fn length(&self) -> Self::Scalar {
        self.inner.length()
    }

    fn sample_at(&self, s: Self::Scalar) -> Result<Self::Point, Self::Error> {
        let pos = self.inner.sample_at(s)?;
        let tan = self.inner.tangent_at(s)?;
        let offset_vec = tan * self.distance;
        Ok(pos.translate(offset_vec))
    }
}
