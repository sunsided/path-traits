//! Path composition and transformation.
//!
//! Build new paths from existing ones without modifying the original data:
//!
//! - [`Reverse`] - traverse a path backwards
//! - [`Concat`] - join two paths end-to-end
//! - [`Offset`] - displace a path by a constant distance
//!
//! The [`PathExt`] extension trait provides ergonomic `.reverse()`, `.concat()`,
//! and `.offset()` methods on any type implementing [`Path`](crate::Path).

mod concat;
mod offset;
mod reverse;

pub use concat::Concat;
pub use offset::Offset;
pub use reverse::Reverse;

use crate::{Heading, Path, Tangent};

/// Ergonomic methods for path composition and transformation.
///
/// Provides `.reverse()`, `.concat()`, and `.offset()` methods on any
/// type implementing [`Path`] (with additional bounds where required).
pub trait PathExt: Path + Sized {
    /// Reverse the path direction.
    fn reverse(self) -> Reverse<Self>;

    /// Concatenate this path with another, compatible path.
    fn concat<Q>(self, other: Q) -> Concat<Self, Q>
    where
        Q: Path<Scalar = Self::Scalar, Point = Self::Point, Error = Self::Error>;

    /// Offset the path by a distance `d`. Requires tangent and heading support.
    fn offset(self, d: Self::Scalar) -> Offset<Self, Self::Scalar>
    where
        Self: Tangent + Heading;
}

impl<P: Path + Sized> PathExt for P {
    fn reverse(self) -> Reverse<Self> {
        Reverse::new(self)
    }

    fn concat<Q>(self, other: Q) -> Concat<Self, Q>
    where
        Q: Path<Scalar = Self::Scalar, Point = Self::Point, Error = Self::Error>,
    {
        Concat::new(self, other)
    }

    fn offset(self, d: Self::Scalar) -> Offset<Self, Self::Scalar>
    where
        Self: Tangent + Heading,
    {
        Offset::new(self, d)
    }
}
