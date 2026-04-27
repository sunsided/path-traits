//! Scalar type alias for arc-length and parameter computations.
//!
//! This module defines the [`Scalar`] trait, a numeric trait with additional bounds
//! (`Debug`, `Copy`, `'static`) required for arc-length and parameter computations
//! throughout the crate. The supertraits vary by feature configuration.

#[cfg(feature = "num-traits")]
mod inner {
    use num_traits::float::FloatCore;

    /// A scalar type suitable for arc-length, parameter, and distance computations.
    ///
    /// Requires `FloatCore` from `num-traits` plus `Debug`, `Copy`, and `'static` bounds.
    /// Implementations are provided automatically for any type satisfying those bounds
    /// (e.g. `f32`, `f64`).
    pub trait Scalar: FloatCore + core::fmt::Debug + Copy + 'static {
        /// Returns the additive identity (0.0).
        fn zero() -> Self;
        /// Returns the multiplicative identity (1.0).
        fn one() -> Self;
        /// Cast a `usize` value to this scalar type.
        fn from_usize(n: usize) -> Self;
    }

    impl<T: FloatCore + core::fmt::Debug + Copy + 'static> Scalar for T {
        fn zero() -> Self {
            T::zero()
        }
        fn one() -> Self {
            T::one()
        }
        fn from_usize(n: usize) -> Self {
            num_traits::NumCast::from(n).unwrap_or(Self::one())
        }
    }
}

#[cfg(not(feature = "num-traits"))]
mod inner {
    /// A scalar type suitable for arc-length, parameter, and distance computations.
    ///
    /// Requires arithmetic operations (`Add`, `Sub`, `Mul`, `Div`, `Neg`),
    /// `PartialOrd`, `Debug`, `Copy`, and `'static` bounds.
    /// Manual implementations are provided for `f32` and `f64`.
    pub trait Scalar:
        core::ops::Add<Output = Self>
        + core::ops::Sub<Output = Self>
        + core::ops::Mul<Output = Self>
        + core::ops::Div<Output = Self>
        + core::ops::Neg<Output = Self>
        + PartialOrd
        + core::fmt::Debug
        + Copy
        + 'static
    {
        /// Returns the additive identity (0.0).
        fn zero() -> Self;
        /// Returns the multiplicative identity (1.0).
        fn one() -> Self;
        /// Cast a `usize` value to this scalar type.
        fn from_usize(n: usize) -> Self;
    }

    macro_rules! impl_scalar {
        ($($t:ty),+) => {
            $(
                impl Scalar for $t {
                    fn zero() -> Self { 0.0 }
                    fn one() -> Self { 1.0 }
                    fn from_usize(n: usize) -> Self { n as $t }
                }
            )+
        };
    }

    impl_scalar!(f32, f64);
}

pub use inner::Scalar;
