#![cfg(all(not(feature = "std"), feature = "num-traits"))]

use path_traits::{
    Curved, Heading, ParametricPath, Path, PathError, PathExt, PathSegment, Point, Project,
    Tangent, Vector, equidistant, n_samples, uniform_t,
};

#[derive(Debug, Clone, Copy, PartialEq)]
struct Vec2(f32, f32);

impl core::ops::Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Vec2(self.0 + rhs.0, self.1 + rhs.1)
    }
}

impl core::ops::Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Vec2(self.0 - rhs.0, self.1 - rhs.1)
    }
}

impl core::ops::Mul<f32> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        Vec2(self.0 * rhs, self.1 * rhs)
    }
}

impl Vector for Vec2 {
    type Scalar = f32;
    fn zero() -> Self {
        Vec2(0.0, 0.0)
    }
    fn dot(self, rhs: Self) -> Self::Scalar {
        self.0 * rhs.0 + self.1 * rhs.1
    }
    fn norm(self) -> Self::Scalar {
        (self.0 * self.0 + self.1 * self.1).sqrt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Pt2(f32, f32);

impl Point for Pt2 {
    type Scalar = f32;
    type Vector = Vec2;
    fn displacement(self, other: Self) -> Self::Vector {
        Vec2(other.0 - self.0, other.1 - self.1)
    }
    fn translate(self, v: Self::Vector) -> Self {
        Pt2(self.0 + v.0, self.1 + v.1)
    }
}

#[derive(Debug, Clone)]
struct LineSegment2 {
    a: Pt2,
    b: Pt2,
    len: f32,
}

impl LineSegment2 {
    fn new(a: Pt2, b: Pt2) -> Self {
        let len = a.distance(b);
        Self { a, b, len }
    }
}

impl Path for LineSegment2 {
    type Scalar = f32;
    type Point = Pt2;
    type Error = PathError;

    fn length(&self) -> Self::Scalar {
        self.len
    }

    fn sample_at(&self, s: Self::Scalar) -> Result<Self::Point, Self::Error> {
        if s < 0.0 || s > self.len {
            return Err(PathError::OutOfDomain);
        }
        if self.len == 0.0 {
            return Ok(self.a);
        }
        let t = s / self.len;
        Ok(Pt2(
            self.a.0 + t * (self.b.0 - self.a.0),
            self.a.1 + t * (self.b.1 - self.a.1),
        ))
    }
}

impl PathSegment for LineSegment2 {}

impl ParametricPath for LineSegment2 {
    fn sample_t(&self, t: Self::Scalar) -> Result<Self::Point, Self::Error> {
        if !(0.0..=1.0).contains(&t) {
            return Err(PathError::OutOfDomain);
        }
        Ok(Pt2(
            self.a.0 + t * (self.b.0 - self.a.0),
            self.a.1 + t * (self.b.1 - self.a.1),
        ))
    }
}

impl Tangent for LineSegment2 {
    fn tangent_at(&self, s: Self::Scalar) -> Result<<Self::Point as Point>::Vector, Self::Error> {
        if s < 0.0 || s > self.len {
            return Err(PathError::OutOfDomain);
        }
        if self.len == 0.0 {
            return Err(PathError::Degenerate);
        }
        let dir = self.a.displacement(self.b);
        Ok(dir * (1.0 / self.len))
    }
}

impl Heading for LineSegment2 {
    fn heading_at(&self, s: Self::Scalar) -> Result<Self::Scalar, Self::Error> {
        if s < 0.0 || s > self.len {
            return Err(PathError::OutOfDomain);
        }
        if self.len == 0.0 {
            return Err(PathError::Degenerate);
        }
        let dir = self.a.displacement(self.b);
        Ok(dir.1.atan2(dir.0))
    }
}

impl Curved for LineSegment2 {
    type Curvature = f32;
    fn curvature_at(&self, s: Self::Scalar) -> Result<Self::Curvature, Self::Error> {
        if s < 0.0 || s > self.len {
            return Err(PathError::OutOfDomain);
        }
        Ok(0.0)
    }
}

impl Project for LineSegment2 {
    fn project(&self, p: Self::Point) -> Result<Self::Scalar, Self::Error> {
        if self.len == 0.0 {
            return Ok(0.0);
        }
        let dir = self.a.displacement(self.b);
        let to_p = self.a.displacement(p);
        let t = (to_p.dot(dir)) / (dir.dot(dir));
        let t_clamped = t.clamp(0.0, 1.0);
        Ok(t_clamped * self.len)
    }
}

#[test]
fn line_segment_length() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 4.0));
    assert!((seg.length() - 5.0).abs() < 1e-6);
}

#[test]
fn line_segment_start_end() {
    let seg = LineSegment2::new(Pt2(1.0, 2.0), Pt2(4.0, 6.0));
    let start = seg.start().unwrap();
    let end = seg.end().unwrap();
    assert_eq!(start, Pt2(1.0, 2.0));
    assert_eq!(end, Pt2(4.0, 6.0));
}

#[test]
fn line_segment_sample_mid() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(2.0, 0.0));
    let mid = seg.sample_at(1.0).unwrap();
    assert!((mid.0 - 1.0).abs() < 1e-6);
    assert!((mid.1 - 0.0).abs() < 1e-6);
}

#[test]
fn line_segment_out_of_domain() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(1.0, 0.0));
    assert_eq!(seg.sample_at(-0.1).unwrap_err(), PathError::OutOfDomain);
    assert_eq!(seg.sample_at(1.1).unwrap_err(), PathError::OutOfDomain);
}

#[test]
fn line_segment_tangent() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 4.0));
    let t = seg.tangent_at(2.5).unwrap();
    let expected = Vec2(0.6, 0.8);
    assert!((t.0 - expected.0).abs() < 1e-6);
    assert!((t.1 - expected.1).abs() < 1e-6);
}

#[test]
fn line_segment_heading() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(1.0, 1.0));
    let h = seg.heading_at(0.5).unwrap();
    assert!((h - core::f32::consts::FRAC_PI_4).abs() < 1e-6);
}

#[test]
fn line_segment_curvature_zero() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(5.0, 0.0));
    assert_eq!(seg.curvature_at(2.5).unwrap(), 0.0);
}

#[test]
fn equidistant_samples() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(4.0, 0.0));
    let count = equidistant(&seg, 1.0).count();
    assert_eq!(count, 5);
}

#[test]
fn n_samples_count() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(10.0, 0.0));
    let samples: Vec<_> = n_samples(&seg, 3).collect::<Result<Vec<_>, _>>().unwrap();
    assert_eq!(samples.len(), 3);
}

#[test]
fn uniform_t_samples() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(10.0, 0.0));
    let samples: Vec<_> = uniform_t(&seg, 5).collect::<Result<Vec<_>, _>>().unwrap();
    assert_eq!(samples.len(), 5);
}

#[test]
fn reverse_double_equals_original() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(5.0, 0.0));
    let rev = seg.clone().reverse();
    let rev2 = rev.reverse();
    for s in [0.0_f32, 1.25, 2.5, 3.75, 5.0] {
        let orig = seg.sample_at(s).unwrap();
        let double_rev = rev2.sample_at(s).unwrap();
        assert!((orig.0 - double_rev.0).abs() < 1e-6);
        assert!((orig.1 - double_rev.1).abs() < 1e-6);
    }
}

#[test]
fn concat_total_length() {
    let a = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 0.0));
    let b = LineSegment2::new(Pt2(3.0, 0.0), Pt2(3.0, 4.0));
    let c = a.concat(b);
    assert!((c.length() - 7.0).abs() < 1e-6);
}

#[test]
fn domain_matches_length() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 4.0));
    let domain = seg.domain();
    assert!(*domain.start() == 0.0);
    assert!((*domain.end() - 5.0).abs() < 1e-6);
}

#[test]
fn project_onto_segment() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(4.0, 0.0));
    let s = seg.project(Pt2(2.0, 3.0)).unwrap();
    assert!((s - 2.0).abs() < 1e-6);
}

#[test]
fn project_clamped_to_end() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 0.0));
    let s = seg.project(Pt2(5.0, 1.0)).unwrap();
    assert!((s - 3.0).abs() < 1e-6);
}

#[test]
fn project_clamped_to_start() {
    let seg = LineSegment2::new(Pt2(2.0, 0.0), Pt2(5.0, 0.0));
    let s = seg.project(Pt2(0.0, 1.0)).unwrap();
    assert!(s.abs() < 1e-6);
}

#[test]
fn closest_point_on_segment() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(4.0, 0.0));
    let cp = seg.closest_point(Pt2(1.0, 5.0)).unwrap();
    assert!((cp.0 - 1.0).abs() < 1e-6);
    assert!((cp.1 - 0.0).abs() < 1e-6);
}

#[test]
fn concat_sample_across_boundary() {
    let a = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 0.0));
    let b = LineSegment2::new(Pt2(3.0, 0.0), Pt2(3.0, 4.0));
    let c = a.concat(b);
    let junction = c.sample_at(3.0).unwrap();
    assert!((junction.0 - 3.0).abs() < 1e-6);
    assert!((junction.1 - 0.0).abs() < 1e-6);
    let into_b = c.sample_at(5.0).unwrap();
    assert!((into_b.0 - 3.0).abs() < 1e-6);
    assert!((into_b.1 - 2.0).abs() < 1e-6);
}

#[test]
fn reverse_tangent_negated() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 4.0));
    let rev = seg.clone().reverse();
    let orig_t = seg.tangent_at(2.0).unwrap();
    let rev_t = rev.tangent_at(seg.length() - 2.0).unwrap();
    assert!((rev_t.0 - (-orig_t.0)).abs() < 1e-6);
    assert!((rev_t.1 - (-orig_t.1)).abs() < 1e-6);
}

#[test]
fn reverse_curvature_negated() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(5.0, 0.0));
    let rev = seg.clone().reverse();
    let orig_k = seg.curvature_at(2.0).unwrap();
    let rev_k = rev.curvature_at(3.0).unwrap();
    assert!((rev_k - (-orig_k)).abs() < 1e-6);
}

#[test]
fn reverse_start_is_original_end() {
    let seg = LineSegment2::new(Pt2(1.0, 2.0), Pt2(4.0, 6.0));
    let rev = seg.clone().reverse();
    let rev_start = rev.start().unwrap();
    let orig_end = seg.end().unwrap();
    assert_eq!(rev_start, orig_end);
}
