# path-traits

[![Crates.io](https://img.shields.io/crates/v/path-traits.svg)](https://crates.io/crates/path-traits)
[![Documentation](https://docs.rs/path-traits/badge.svg)](https://docs.rs/path-traits)
[![License: EUPL-1.2](https://img.shields.io/badge/License-EUPL--1.2-blue.svg)](https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12)
[![Rust Edition](https://img.shields.io/badge/edition-2024-dea584)](https://doc.rust-lang.org/edition-guide/)
[![Rust 1.85+](https://img.shields.io/badge/rust-1.85+-000000?logo=rust)](https://www.rust-lang.org)
[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

Tower-like generic traits for parametric paths, segments, and geometric queries.

## What is this?

`path-traits` is a small, dependency-free trait crate that provides a unified interface for parametric curves. It decouples *geometric queries* (sampling by arc-length, tangents, curvature, projection, composition) from *curve representation* (line segments, arcs, Béziers, B-splines, NURBS, polylines, and more).

The crate is `no_std` by default, has zero mandatory dependencies, is `#![forbid(unsafe_code)]`, and is generic over a `Scalar` type so it works with `f32`, `f64`, or custom scalar types via the optional `num-traits` feature.

## Who is it for?

- **Path planning / motion planning / robotics** libraries that need a shared vocabulary for curve queries.
- **Graphics and CAD code** that consumes curves generically, regardless of underlying representation.
- **Numerical libraries** that want arc-length-based sampling, tangents, and curvature without tying to a specific curve type.
- **Anyone** who wants to write `fn foo<P: Path>(p: &P)` once and have it work for every curve type in the ecosystem.

## Installation

```bash
# Default: no_std, zero deps
cargo add path-traits

# With num-traits for the Float/FloatCore Scalar backend
cargo add path-traits --features num-traits

# With std integrations
cargo add path-traits --features std

# Both
cargo add path-traits --features "std num-traits"
```

## The trait hierarchy at a glance

- [`Scalar`], [`Point`], [`Vector`] — numeric and geometric primitives.
- [`Path`], [`ParametricPath`] — sample curves by arc-length or normalized parameter.
- [`PathSegment`], [`SegmentedPath`] — work with multi-segment paths like polylines.
- [`Tangent`], [`Heading`], [`Curved`], [`FrenetFrame`] — differential geometry queries.
- [`Project`] — closest-point projection onto a path.
- [`PathExt`] + [`Reverse`], [`Concat`], [`Offset`] — composable path adapters.
- Free helpers: [`equidistant`], [`n_samples`], [`uniform_t`].
- [`PathError`] — canonical error enum (every `type Error` must be `From<PathError<Self::Scalar>>`).

[`Scalar`]: https://docs.rs/path-traits/latest/path_traits/trait.Scalar.html
[`Point`]: https://docs.rs/path-traits/latest/path_traits/trait.Point.html
[`Vector`]: https://docs.rs/path-traits/latest/path_traits/trait.Vector.html
[`Path`]: https://docs.rs/path-traits/latest/path_traits/trait.Path.html
[`ParametricPath`]: https://docs.rs/path-traits/latest/path_traits/trait.ParametricPath.html
[`PathSegment`]: https://docs.rs/path-traits/latest/path_traits/trait.PathSegment.html
[`SegmentedPath`]: https://docs.rs/path-traits/latest/path_traits/trait.SegmentedPath.html
[`Tangent`]: https://docs.rs/path-traits/latest/path_traits/trait.Tangent.html
[`Heading`]: https://docs.rs/path-traits/latest/path_traits/trait.Heading.html
[`Curved`]: https://docs.rs/path-traits/latest/path_traits/trait.Curved.html
[`FrenetFrame`]: https://docs.rs/path-traits/latest/path_traits/trait.FrenetFrame.html
[`Project`]: https://docs.rs/path-traits/latest/path_traits/trait.Project.html
[`PathExt`]: https://docs.rs/path-traits/latest/path_traits/trait.PathExt.html
[`Reverse`]: https://docs.rs/path-traits/latest/path_traits/struct.Reverse.html
[`Concat`]: https://docs.rs/path-traits/latest/path_traits/struct.Concat.html
[`Offset`]: https://docs.rs/path-traits/latest/path_traits/struct.Offset.html
[`equidistant`]: https://docs.rs/path-traits/latest/path_traits/fn.equidistant.html
[`n_samples`]: https://docs.rs/path-traits/latest/path_traits/fn.n_samples.html
[`uniform_t`]: https://docs.rs/path-traits/latest/path_traits/fn.uniform_t.html
[`PathError`]: https://docs.rs/path-traits/latest/path_traits/enum.PathError.html

## Using the traits (consumer guide)

### Writing a generic function

```rust
use path_traits::{Path, Scalar};

fn midpoint<P: Path>(p: &P) -> Result<P::Point, P::Error> {
    let half = p.length() / P::Scalar::from_usize(2);
    p.sample_at(half)
}
```

### Combining traits for richer queries

```rust
use path_traits::{Path, Tangent, Curved};

fn path_info<P: Path + Tangent + Curved>(p: &P, s: P::Scalar)
    -> Result<String, P::Error>
{
    let tangent = p.tangent_at(s)?;
    let curvature = p.curvature_at(s)?;
    Ok(format!("T=({:?}), k={:?}", tangent, curvature))
}
```

### Sampling with helper functions

```rust
use path_traits::{Path, equidistant, n_samples, uniform_t};

fn sample_demo<P: Path>(p: &P) {
    // Every 1.0 units of arc-length
    let _: Vec<_> = equidistant(p, 1.0).collect();

    // Exactly 10 points
    let _: Vec<_> = n_samples(p, 10).collect();
}

// uniform_t requires ParametricPath
use path_traits::ParametricPath;
fn uniform_demo<P: ParametricPath>(p: &P) {
    let _: Vec<_> = uniform_t(p, 5).collect();
}
```

### Path composition with PathExt

```rust
use path_traits::{Path, PathExt, Heading, Tangent};

fn compose_demo<P: Path + Clone>(a: P, b: P)
where
    P: Path<Scalar = f64, Point = P::Point>,
{
    // Reverse direction
    let _reversed = a.clone().reverse();

    // Join end-to-end
    let _concat = a.concat(b);

    // offset() requires Tangent + Heading bounds
    // let _offset = a.clone().offset(0.5);
}
```

### Closest-point projection

```rust
use path_traits::{Path, Project};

fn nearest<P: Path + Project>(p: &P, query: P::Point) -> Result<P::Point, P::Error> {
    p.closest_point(query)
}
```

## Implementing the traits (implementer guide)

This section describes what you must implement for each trait, what you get for free, and the invariants your implementation must uphold.

### Scalar

The [`Scalar`] trait is the numeric backbone of the crate. You typically **do not need to implement it** — blanket implementations exist for `f32` / `f64` in all feature configurations. Only implement `Scalar` manually for exotic numeric types.

**Feature-dependent supertraits:**
| Features | Supertrait |
|---|---|
| (none) | `Add + Sub + Mul + Div + Neg + PartialOrd + Debug + Copy + 'static` |
| `num-traits` | `FloatCore + Debug + Copy + 'static` |
| `num-traits` + `std` | `Float + Debug + Copy + 'static` |

**Required methods:** `zero()`, `one()`, `from_usize(n)`.

### Vector

[`Vector`] represents a displacement or derivative in Euclidean space.

**Required:** `zero()`, `dot(self, rhs)`, `norm(self)`.

**Required operator bounds:** `Add<Output=Self>`, `Sub<Output=Self>`, `Mul<Scalar, Output=Self>`.

**Invariant:** `norm() >= 0` and `(v * 0).norm() == 0`.

### Point

[`Point`] represents a position in an affine space.

**Required:** `displacement(self, other) -> Vector`, `translate(self, v) -> Point`.

**Provided:** `distance(self, other)` (delegates to `displacement().norm()`).

**Invariant:** `a.translate(a.displacement(b)) == b`.

### Path

[`Path`] is the core trait for arc-length-parameterized curves.

**Required associated types:**
- `type Scalar: Scalar`
- `type Point: Point<Scalar = Self::Scalar>`
- `type Error: From<PathError<Self::Scalar>>`

**Required methods:**
- `length(&self) -> Self::Scalar` — total arc-length.
- `sample_at(&self, s: Self::Scalar) -> Result<Self::Point, Self::Error>` — sample at arc-length `s ∈ [0, length]`.

**Provided:** `start()`, `end()`, `domain()`.

**Invariants:**
- `sample_at(0) == start()` and `sample_at(length()) == end()`.
- Return `PathError::OutOfDomain { param, domain }` when `s ∉ [0, length]` — use the `PathError::out_of_domain(s, self.domain())` helper. Use `PathError::degenerate(reason)` for zero-length paths and `PathError::not_differentiable(s, reason)` for cusps.
- `sample_at` should be arc-length-parameterized (constant speed). If it is not, also implement `ParametricPath` and override `t_to_s` / `s_to_t`.

**Error context:** `PathError<S>` carries the offending parameter and valid domain so consumers can produce precise diagnostics:

```rust
use path_traits::{Path, PathError};

fn handle_error<P: Path>(path: &P, result: Result<P::Point, P::Error>) {
    if let Err(err) = result {
        // Convert to PathError to inspect the payload
        if let PathError::OutOfDomain { param, domain } = err.into() {
            eprintln!("parameter {:?} is outside [{:?}, {:?}]", param, domain.start(), domain.end());
        }
    }
}
```

### ParametricPath

[`ParametricPath`] extends `Path` with normalized-parameter sampling.

**Required:**
- `sample_t(&self, t: Self::Scalar) -> Result<Self::Point, Self::Error>` — sample at `t ∈ [0, 1]`.

**Provided (default linear conversion):**
- `t_to_s(&self, t) -> Self::Scalar` — default: `t * length()`.
- `s_to_t(&self, s) -> Self::Scalar` — default: `s / length()`.

**Invariants:** `sample_t(0) == start()`, `sample_t(1) == end()`.

Override `t_to_s` / `s_to_t` if your path is not constant-speed.

### PathSegment

[`PathSegment`] is a **marker trait** for primitive, non-subdivided curves (a single line segment, a Bézier curve, etc.). Implement it on any type that already implements `Path`.

```rust
impl PathSegment for MyCurve {}
```

### SegmentedPath

[`SegmentedPath`] is for paths composed of multiple segments (polylines, chains).

**Required associated type:**
- `type Segment: PathSegment<Scalar = Self::Scalar, Point = Self::Point, Error = Self::Error>`

Note the same-type constraint: `Scalar`, `Point`, and `Error` must all match the parent path.

**Required methods:**
- `segment_count(&self) -> usize`
- `segments(&self) -> impl Iterator<Item = &Self::Segment> + '_`
- `locate(&self, s: Self::Scalar) -> Result<(usize, Self::Scalar), Self::Error>`

**Provided:** `segment(&self, i)` (by index, returns `Option`).

**Invariants:**
- Segment lengths sum to `length()`.
- `locate(s)` returns `local_s ∈ [0, segment_length]`.

### Tangent

[`Tangent`] provides the unit tangent vector at any arc-length.

**Required:**
- `tangent_at(&self, s) -> Result<Vector, Self::Error>`

The returned vector must be unit-length and point in the direction of increasing `s`. Use `PathError::degenerate(reason)` for zero-length paths and `PathError::not_differentiable(s, reason)` for cusps.

### Heading

[`Heading`] provides the planar heading angle (2D only).

**Required:**
- `heading_at(&self, s) -> Result<Self::Scalar, Self::Error>`

Returns radians, using the `atan2(y, x)` convention (counter-clockwise from the positive x-axis).

### Curved

[`Curved`] provides curvature at any arc-length.

**Required associated type:**
- `type Curvature` — scalar in 2D, vector in 3D.

**Required method:**
- `curvature_at(&self, s) -> Result<Self::Curvature, Self::Error>`

**Sign convention (2D):** positive for left turns (CCW), negative for right turns.

### FrenetFrame (advanced)

[`FrenetFrame`] provides the full Frenet-Serret frame. This is the most complex differential trait and is optional for basic implementations.

**Bounds:** `Tangent + Curved`.

**Required associated type:**
- `type Frame` — e.g. `(Tangent, Normal)` in 2D or `(T, N, B)` in 3D.

**Required method:**
- `frame_at(&self, s) -> Result<Self::Frame, Self::Error>`

### Project

[`Project`] provides closest-point projection onto a path.

**Required:**
- `project(&self, p: Self::Point) -> Result<Self::Scalar, Self::Error>` — returns the arc-length `s` of the closest point.

**Provided:** `closest_point(&self, p) -> Result<Self::Point, Self::Error>` (calls `project` then `sample_at`).

**Invariant:** the returned `s` minimizes `path.sample_at(s).distance(p)` over `[0, length]`.

### Minimal implementation example

Here is a complete, minimal implementation for a 2D line segment:

```rust
use path_traits::*;

// Vector
#[derive(Debug, Clone, Copy, PartialEq)]
struct Vec2(f64, f64);

impl core::ops::Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self { Vec2(self.0 + rhs.0, self.1 + rhs.1) }
}

impl core::ops::Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self { Vec2(self.0 - rhs.0, self.1 - rhs.1) }
}

impl core::ops::Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self { Vec2(self.0 * rhs, self.1 * rhs) }
}

impl Vector for Vec2 {
    type Scalar = f64;
    fn zero() -> Self { Vec2(0.0, 0.0) }
    fn dot(self, rhs: Self) -> f64 { self.0 * rhs.0 + self.1 * rhs.1 }
    fn norm(self) -> f64 { (self.0 * self.0 + self.1 * self.1).sqrt() }
}

// Point
#[derive(Debug, Clone, Copy, PartialEq)]
struct Pt2(f64, f64);

impl Point for Pt2 {
    type Scalar = f64;
    type Vector = Vec2;
    fn displacement(self, other: Self) -> Vec2 {
        Vec2(other.0 - self.0, other.1 - self.1)
    }
    fn translate(self, v: Vec2) -> Self { Pt2(self.0 + v.0, self.1 + v.1) }
}

// Path
struct LineSegment2 { a: Pt2, b: Pt2, len: f64 }

impl LineSegment2 {
    fn new(a: Pt2, b: Pt2) -> Self {
        Self { a, b, len: a.distance(b) }
    }
}

impl Path for LineSegment2 {
    type Scalar = f64;
    type Point = Pt2;
    type Error = PathError<f64>;

    fn length(&self) -> f64 { self.len }

    fn sample_at(&self, s: f64) -> Result<Pt2, PathError<f64>> {
        if s < 0.0 || s > self.len { return Err(PathError::out_of_domain(s, 0.0..=self.len)); }
        if self.len == 0.0 { return Ok(self.a); }
        let t = s / self.len;
        Ok(Pt2(
            self.a.0 + t * (self.b.0 - self.a.0),
            self.a.1 + t * (self.b.1 - self.a.1),
        ))
    }
}
```

### Adapter bounds note

The `.offset()` method on [`PathExt`] requires `Self: Tangent + Heading`. If your type does not implement these traits, the compiler will reject `.offset()` calls. This is intentional — offsetting requires knowledge of the tangent direction and heading to compute the displaced curve.

## Feature flags

| Feature | Description |
|---|---|
| *(default)* | `no_std`, zero deps. `f32`/`f64` work via manual `Scalar` impls. |
| `num-traits` | Uses `num-traits` as the `Scalar` backend. Without `std`, bounded by `FloatCore`; with `std`, bounded by `Float`. |
| `std` | Enables `std`-specific integrations. When combined with `num-traits`, forwards `std` to that crate. |

The core traits work in all three configurations. `f32` and `f64` are always available as `Scalar` types.

## MSRV / Edition

Rust 2024 edition, MSRV 1.85+.

## License

Licensed under any of:

- EUPL-1.2
- MIT
- Apache-2.0

Choose whichever suits your needs.

## Repository

Source code, issues, and pull requests: <https://github.com/sunsided/path-traits>
