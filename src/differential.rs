//! Differential geometry traits.
//!
//! This module provides opt-in traits for geometric queries beyond position:
//!
//! - [`Tangent`] — unit tangent vector at any arc-length
//! - [`Heading`] — planar heading angle (radians) at any arc-length
//! - [`Curved`] — curvature (scalar in 2D, vector in 3D) at any arc-length
//! - [`FrenetFrame`] — full Frenet–Serret frame (T, N[, B]) at any arc-length

use crate::{Path, Point};

/// A path whose tangent vector can be queried at any arc-length.
///
/// The returned vector should be unit-length and point in the direction of
/// increasing arc-length.
pub trait Tangent: Path {
    /// Unit tangent vector at arc-length `s`.
    ///
    /// Returns an error when `s` is outside the domain or the path is not
    /// differentiable at `s`.
    fn tangent_at(&self, s: Self::Scalar) -> Result<<Self::Point as Point>::Vector, Self::Error>;
}

/// A path with a planar heading angle at any arc-length.
///
/// Heading is meaningful only for 2D embeddings, where it represents the angle
/// (in radians, counter-clockwise from the positive x-axis) of the tangent.
pub trait Heading: Path {
    /// Planar heading angle (radians) at arc-length `s`.
    ///
    /// Returns an error when `s` is outside the domain or the heading is
    /// undefined at this point.
    fn heading_at(&self, s: Self::Scalar) -> Result<Self::Scalar, Self::Error>;
}

/// A path whose curvature can be queried.
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

/// A path that can produce a Frenet frame (T, N[, B]) at any arc-length.
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
