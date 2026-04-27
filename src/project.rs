//! Closest-point projection onto a path.
//!
//! This module defines the [`Project`] trait for finding the arc-length of the
//! point on a path nearest to a given query point, along with a convenience
//! method `closest_point` that returns the actual position.

use crate::Path;

/// Project a query point onto a path to find the nearest position.
///
/// Implementers should return the arc-length parameter of the closest point.
/// The default `closest_point` method combines `project` with `sample_at` to
/// produce the actual position.
pub trait Project: Path {
    /// Find the arc-length `s` of the point on the path closest to `p`.
    ///
    /// Returns an error when the projection cannot be computed (e.g. a
    /// degenerate path).
    fn project(&self, p: Self::Point) -> Result<Self::Scalar, Self::Error>;

    /// Return the closest point on the path to `p`.
    ///
    /// This default implementation calls `project` followed by `sample_at`.
    fn closest_point(&self, p: Self::Point) -> Result<Self::Point, Self::Error> {
        let s = self.project(p)?;
        self.sample_at(s)
    }
}
