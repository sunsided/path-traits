//! Numeric scalar type alias.
//!
//! This module defines the [`Scalar`] trait, a marker alias over `num_traits::Float`
//! with additional bounds (`Debug`, `Copy`, `'static`) required for arc-length and
//! parameter computations throughout the crate.

use num_traits::Float;

/// A scalar type suitable for arc-length, parameter, and distance computations.
///
/// Requires `Float` from `num-traits` plus `Debug`, `Copy`, and `'static` bounds.
/// Implementations are provided automatically for any type satisfying those bounds
/// (e.g. `f32`, `f64`).
pub trait Scalar: Float + core::fmt::Debug + Copy + 'static {}

/// Blanket implementation of [`Scalar`] for any type that satisfies the required bounds.
impl<T: Float + core::fmt::Debug + Copy + 'static> Scalar for T {}
