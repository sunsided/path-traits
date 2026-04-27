//! Closest-point projection trait.
//!
//! This module defines the [`Project`] trait, which finds the arc-length of the
//! point on a path closest to a given query point, along with a convenience
//! method `closest_point` that returns the actual point.

use crate::Path;

/// Find the closest point on a path to a given query point.
///
/// Implementers should project the query point onto the path and return the
/// corresponding arc-length parameter. The default `closest_point` method
/// combines `project` with `sample_at` to produce the actual closest position.
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
