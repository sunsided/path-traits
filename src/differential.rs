//! Differential geometry queries for paths.
//!
//! Once you can sample a path's position, these opt-in traits let you query
//! how the path is oriented and bending at any point:
//!
//! - [`Tangent`] - unit tangent vector at any arc-length
//! - [`Heading`] - planar heading angle (radians) at any arc-length
//! - [`Curved`] - curvature (scalar in 2D, vector in 3D) at any arc-length
//! - [`FrenetFrame`] - full Frenet–Serret frame (T, N[, B]) at any arc-length

use crate::{Path, Point};

/// Query the unit tangent vector at any point along a path.
///
/// The returned vector is unit-length and points in the direction of
/// increasing arc-length.
pub trait Tangent: Path {
    /// Unit tangent vector at arc-length `s`.
    ///
    /// Returns an error when `s` is outside the domain or the path is not
    /// differentiable at `s`.
    fn tangent_at(&self, s: Self::Scalar) -> Result<<Self::Point as Point>::Vector, Self::Error>;
}

/// Query the planar heading angle at any point along a path.
///
/// Heading is meaningful only for 2D paths, where it represents the angle
/// (in radians, counter-clockwise from the positive x-axis) of the tangent.
pub trait Heading: Path {
    /// Planar heading angle (radians) at arc-length `s`.
    ///
    /// Returns an error when `s` is outside the domain or the heading is
    /// undefined at this point.
    fn heading_at(&self, s: Self::Scalar) -> Result<Self::Scalar, Self::Error>;
}

/// Query the curvature at any point along a path.
///
/// In 2D the curvature is a signed scalar (positive for left turns, negative
/// for right turns). In 3D it is a curvature vector (`κ · N`).
pub trait Curved: Path {
    /// The type representing curvature (scalar in 2D, vector in 3D).
    type Curvature;

    /// Curvature at arc-length `s`.
    ///
    /// Returns an error when `s` is outside the domain or curvature is
    /// undefined at this point.
    fn curvature_at(&self, s: Self::Scalar) -> Result<Self::Curvature, Self::Error>;
}

/// Query the full Frenet frame (T, N[, B]) at any point along a path.
///
/// In 2D the frame consists of the tangent and normal. In 3D it additionally
/// includes the binormal.
pub trait FrenetFrame: Tangent + Curved {
    /// Frame type, e.g. `(Tangent, Normal)` in 2D or `(T, N, B)` in 3D.
    type Frame;

    /// Frenet frame at arc-length `s`.
    ///
    /// Returns an error when `s` is outside the domain or the frame cannot be
    /// computed (e.g. zero curvature in 3D).
    fn frame_at(&self, s: Self::Scalar) -> Result<Self::Frame, Self::Error>;
}
