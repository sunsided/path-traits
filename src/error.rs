//! Path error types.
//!
//! This module defines [`PathError`], the canonical error enum for all path-related
//! fallible operations. Every trait in the hierarchy exposes
//! `type Error: From<PathError>` so implementers may use their own error type.

/// Errors that can occur during path sampling or geometric queries.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum PathError {
    /// The parameter `s` or `t` is outside the valid domain `[0, length]` / `[0, 1]`.
    OutOfDomain,
    /// The path is not differentiable at the requested point (e.g. a cusp).
    NotDifferentiable,
    /// The path is degenerate (e.g. zero-length segment, making tangents undefined).
    Degenerate,
    /// A custom error with a static description for implementer-specific failures.
    Custom(&'static str),
}

impl core::fmt::Display for PathError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PathError::OutOfDomain => write!(f, "parameter out of valid domain"),
            PathError::NotDifferentiable => {
                write!(f, "path is not differentiable at this point")
            }
            PathError::Degenerate => write!(f, "path is degenerate"),
            PathError::Custom(msg) => write!(f, "{msg}"),
        }
    }
}

impl core::error::Error for PathError {}
