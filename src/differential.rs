//! Differential geometry queries for paths.
//!
//! Once you can sample a path's position, these opt-in traits let you query
//! how the path is oriented and bending at any point:
//!
//! - [`Tangent`] - unit tangent vector at any arc-length
//! - [`Heading`] - planar heading angle (radians) at any arc-length
//! - [`Curved`] - curvature (scalar in 2D, vector in 3D) at any arc-length
//! - [`FrenetFrame`] - full Frenet–Serret frame (T, N[, B]) at any arc-length
//! - [`BishopFrame`] - rotation-minimizing (Bishop) frame stream in 3D

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

/// Rotation-minimizing (Bishop) frame along a path.
///
/// Unlike [`FrenetFrame`], a Bishop frame is *not* a local, pointwise query.
/// It depends on an initial frame (the **seed**) and on parallel transport
/// along the curve. Implementations therefore yield an ordered sequence of
/// frames at the given arc-length samples, starting from an explicit seed.
///
/// # When to use Bishop frames
///
/// Bishop frames (a.k.a. rotation-minimizing frames, RMF) are the right choice
/// when you need a *smooth* orthonormal frame along a curve — for sweep
/// surfaces, camera paths, ribbons, cable routing, or 6-DOF tool paths. They
/// are defined wherever the tangent is defined (unlike Frenet frames, which
/// break at zero-curvature points) and do not exhibit the discontinuous
/// "Frenet flip" at inflection points.
///
/// # Path dependence
///
/// A Bishop frame at arc-length `s` is only meaningful relative to the seed
/// frame and the ordered traversal from `s₀` to `s`. Two consumers querying
/// the same `s` with different seeds will receive different frames. This is
/// by design: the family of rotation-minimizing frames has one scalar degree
/// of freedom (the initial roll about the tangent).
///
/// # Sample ordering
///
/// The `samples` iterator **must** produce monotonically non-decreasing
/// arc-length values. Violating this precondition is a caller error;
/// implementations may return [`crate::PathError::not_differentiable`] or
/// [`crate::PathError::OutOfDomain`] for out-of-order samples.
///
/// # Two-dimensional paths
///
/// In 2D, a rotation-minimizing frame collapses to `(T, N)` — the tangent
/// rotated by 90°. This adds nothing beyond [`Tangent`], so this trait is
/// primarily meaningful for 3D paths. Implementors on 2D types may provide
/// a trivial implementation, but consumers should prefer `Tangent` directly.
pub trait BishopFrame: Tangent {
    /// Frame type — typically `(T, M1, M2)` in 3D, where `T` is the unit
    /// tangent and `M1`, `M2` span the normal plane.
    type Frame;

    /// Initial-frame seed required to disambiguate the family of
    /// rotation-minimizing frames. Often a full frame or an "up" hint
    /// combined with the start tangent.
    type Seed;

    /// Stream Bishop frames at the given monotonically non-decreasing
    /// arc-length samples, starting from `seed` at the first sample.
    ///
    /// # Arguments
    ///
    /// * `seed` — the initial frame at the first sample, which determines the
    ///   roll angle of the entire frame sequence.
    /// * `samples` — arc-length values in monotonically non-decreasing order.
    ///   If `samples` is empty, the returned iterator yields no frames.
    ///
    /// # Errors
    ///
    /// Returns an error when any sample is outside the path domain, or when
    /// the samples are not monotonically non-decreasing (caller precondition
    /// violation).
    ///
    /// # Example (schematic)
    ///
    /// ```ignore
    /// // `path` implements BishopFrame with Scalar = f64, Frame = (Vec3, Vec3, Vec3), Seed = Frame3
    /// let seed = Frame3::from_tangent_and_up(path.tangent_at(0.0)?, Vec3::new(0.0, 0.0, 1.0));
    /// let frames: Vec<_> = path
    ///     .bishop_frames(seed, equidistant(&path, 0.1))
    ///     .collect::<Result<Vec<_>, _>>()?;
    /// ```
    fn bishop_frames<I>(
        &self,
        seed: Self::Seed,
        samples: I,
    ) -> impl Iterator<Item = Result<Self::Frame, Self::Error>> + '_
    where
        I: IntoIterator<Item = Self::Scalar>;
}
