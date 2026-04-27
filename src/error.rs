//! Error types for path operations.
//!
//! This module defines [`PathError<S>`], the canonical error enum for all path-related
//! fallible operations. Every trait in the hierarchy exposes
//! `type Error: From<PathError<Self::Scalar>>` so implementers may use their own error type.

use crate::Scalar;
use core::fmt;
use core::ops::RangeInclusive;

/// Errors that can occur during path sampling or geometric queries.
///
/// The scalar parameter `S` allows the error to carry the exact parameter value
/// and domain that caused the failure, making diagnostics precise.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum PathError<S> {
    /// Arc-length / parameter outside the valid domain.
    OutOfDomain {
        /// The parameter value that was out of domain.
        param: S,
        /// The valid domain `[start, end]`.
        domain: RangeInclusive<S>,
    },
    /// The path is not differentiable at `param` (e.g. a cusp or kink).
    NotDifferentiable {
        /// The parameter where differentiability fails.
        param: S,
        /// Human-readable reason.
        reason: &'static str,
    },
    /// The path is degenerate (e.g. zero-length segment, collinear control points).
    Degenerate {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// A custom error with a static description for implementer-specific failures.
    Custom(&'static str),
}

impl<S: Scalar> PathError<S> {
    /// Create an `OutOfDomain` error with the offending parameter and valid domain.
    pub fn out_of_domain(param: S, domain: RangeInclusive<S>) -> Self {
        PathError::OutOfDomain { param, domain }
    }

    /// Create a `NotDifferentiable` error with the parameter and reason.
    pub fn not_differentiable(param: S, reason: &'static str) -> Self {
        PathError::NotDifferentiable { param, reason }
    }

    /// Create a `Degenerate` error with a reason.
    pub fn degenerate(reason: &'static str) -> Self {
        PathError::Degenerate { reason }
    }
}

impl<S: fmt::Debug> fmt::Display for PathError<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PathError::OutOfDomain { param, domain } => {
                write!(
                    f,
                    "parameter {:?} is outside the valid domain {:?}",
                    param, domain
                )
            }
            PathError::NotDifferentiable { param, reason } => {
                write!(f, "path is not differentiable at {:?}: {}", param, reason)
            }
            PathError::Degenerate { reason } => {
                write!(f, "path is degenerate: {}", reason)
            }
            PathError::Custom(msg) => write!(f, "{}", msg),
        }
    }
}

impl<S: fmt::Debug> core::error::Error for PathError<S> {}
