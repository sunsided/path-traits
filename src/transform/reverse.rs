//! Traverse a path in the reverse direction.
//!
//! [`Reverse`] wraps a path and samples it backwards: `sample_at(s)`
//! delegates to `inner.sample_at(length - s)`. Tangent vectors are negated
//! and (signed) curvature is negated.

use num_traits::NumCast;

use crate::{Curved, Path, Point, Tangent};

/// A path traversed in the opposite direction.
///
/// Sampling at arc-length `s` returns the position the inner path would have
/// at `length - s`. Tangent vectors are negated; signed curvature is negated.
pub struct Reverse<P> {
    /// The inner path being reversed.
    inner: P,
}

impl<P> Reverse<P> {
    /// Create a new reversed path.
    pub fn new(inner: P) -> Self {
        Self { inner }
    }
}

impl<P: Path> Path for Reverse<P> {
    type Scalar = P::Scalar;
    type Point = P::Point;
    type Error = P::Error;

    fn length(&self) -> Self::Scalar {
        self.inner.length()
    }

    fn sample_at(&self, s: Self::Scalar) -> Result<Self::Point, Self::Error> {
        self.inner.sample_at(self.inner.length() - s)
    }
}

impl<P: Tangent> Tangent for Reverse<P> {
    fn tangent_at(&self, s: Self::Scalar) -> Result<<Self::Point as Point>::Vector, Self::Error> {
        let t = self.inner.tangent_at(self.inner.length() - s)?;
        let neg_one = <P::Scalar as NumCast>::from(-1.0).unwrap();
        Ok(t * neg_one)
    }
}

impl<P: Curved> Curved for Reverse<P>
where
    P::Curvature: core::ops::Neg<Output = P::Curvature>,
{
    type Curvature = P::Curvature;

    fn curvature_at(&self, s: Self::Scalar) -> Result<Self::Curvature, Self::Error> {
        let k = self.inner.curvature_at(self.inner.length() - s)?;
        Ok(-k)
    }
}
