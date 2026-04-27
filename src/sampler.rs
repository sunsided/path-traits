//! Sampling functions.
//!
//! This module provides free functions for sampling paths at evenly-spaced
//! positions:
//!
//! - [`equidistant`] — samples at fixed arc-length steps
//! - [`n_samples`] — samples at `n` evenly-spaced arc-length positions
//! - [`uniform_t`] — samples at `n` evenly-spaced normalized-parameter values

use num_traits::{NumCast, One, Zero};

use crate::{ParametricPath, Path};

/// Sample a path at equidistant arc-length intervals.
///
/// Yields points at `s = 0, step, 2*step, …` until `s > length`. The final
/// sample may exceed `length` by less than `step`.
pub fn equidistant<P: Path>(
    path: &P,
    step: P::Scalar,
) -> impl Iterator<Item = Result<P::Point, P::Error>> + '_ {
    let length = path.length();
    let zero = P::Scalar::zero();
    let mut s = zero;
    core::iter::from_fn(move || {
        if s > length {
            None
        } else {
            let result = path.sample_at(s);
            s = s + step;
            Some(result)
        }
    })
}

/// Sample a path at `n` evenly-spaced arc-length positions.
///
/// The samples are at `s = 0, length/(n-1), 2*length/(n-1), …, length`.
/// For `n == 1`, returns a single sample at `s = 0`.
pub fn n_samples<P: Path>(
    path: &P,
    n: usize,
) -> impl Iterator<Item = Result<P::Point, P::Error>> + '_ {
    let length = path.length();
    let zero = P::Scalar::zero();
    let one = P::Scalar::one();
    let n_scalar = NumCast::from(n).unwrap_or(one);
    let step = if n > 1 {
        length / (n_scalar - one)
    } else {
        zero
    };
    (0..n).map(move |i| {
        let s = if n == 1 {
            zero
        } else {
            <P::Scalar as NumCast>::from(i).unwrap() * step
        };
        path.sample_at(s)
    })
}

/// Sample a parametric path at `n` evenly-spaced `t` values in `[0, 1]`.
///
/// The samples are at `t = 0, 1/(n-1), 2/(n-1), …, 1`.
/// For `n == 1`, returns a single sample at `t = 0`.
pub fn uniform_t<P: ParametricPath>(
    path: &P,
    n: usize,
) -> impl Iterator<Item = Result<P::Point, P::Error>> + '_ {
    let zero = P::Scalar::zero();
    let one = P::Scalar::one();
    let n_scalar = NumCast::from(n).unwrap_or(one);
    let dt = if n > 1 { one / (n_scalar - one) } else { zero };
    (0..n).map(move |i| {
        let t = if n == 1 {
            zero
        } else {
            <P::Scalar as NumCast>::from(i).unwrap() * dt
        };
        path.sample_t(t)
    })
}
