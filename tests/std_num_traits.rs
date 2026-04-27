#![cfg(all(feature = "std", feature = "num-traits"))]

use path_traits::{
    Curved, Heading, ParametricPath, Path, PathError, PathExt, PathSegment, Point, Project,
    SegmentedPath, Tangent, Vector, equidistant, n_samples, uniform_t,
};

#[derive(Debug, Clone, Copy, PartialEq)]
struct Vec2(f64, f64);

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

impl core::ops::Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Vec2(self.0 * rhs, self.1 * rhs)
    }
}

impl Vector for Vec2 {
    type Scalar = f64;
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
struct Pt2(f64, f64);

impl Point for Pt2 {
    type Scalar = f64;
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
    len: f64,
}

impl LineSegment2 {
    fn new(a: Pt2, b: Pt2) -> Self {
        let len = a.distance(b);
        Self { a, b, len }
    }
}

impl Path for LineSegment2 {
    type Scalar = f64;
    type Point = Pt2;
    type Error = PathError<f64>;

    fn length(&self) -> Self::Scalar {
        self.len
    }

    fn sample_at(&self, s: Self::Scalar) -> Result<Self::Point, Self::Error> {
        if s < 0.0 || s > self.len {
            return Err(PathError::out_of_domain(s, self.domain()));
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
            return Err(PathError::out_of_domain(t, 0.0..=1.0));
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
            return Err(PathError::out_of_domain(s, self.domain()));
        }
        if self.len == 0.0 {
            return Err(PathError::degenerate("zero-length segment"));
        }
        let dir = self.a.displacement(self.b);
        Ok(dir * (1.0 / self.len))
    }
}

impl Heading for LineSegment2 {
    fn heading_at(&self, s: Self::Scalar) -> Result<Self::Scalar, Self::Error> {
        if s < 0.0 || s > self.len {
            return Err(PathError::out_of_domain(s, self.domain()));
        }
        if self.len == 0.0 {
            return Err(PathError::degenerate("zero-length segment"));
        }
        let dir = self.a.displacement(self.b);
        Ok(dir.1.atan2(dir.0))
    }
}

impl Curved for LineSegment2 {
    type Curvature = f64;
    fn curvature_at(&self, s: Self::Scalar) -> Result<Self::Curvature, Self::Error> {
        if s < 0.0 || s > self.len {
            return Err(PathError::out_of_domain(s, self.domain()));
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

#[derive(Debug, Clone)]
struct Polyline2 {
    #[allow(dead_code)]
    points: Vec<Pt2>,
    segments: Vec<LineSegment2>,
    seg_lengths: Vec<f64>,
    total_len: f64,
}

impl Polyline2 {
    fn new(points: Vec<Pt2>) -> Self {
        assert!(points.len() >= 2);
        let segments: Vec<_> = points
            .windows(2)
            .map(|w| LineSegment2::new(w[0], w[1]))
            .collect();
        let seg_lengths: Vec<_> = segments.iter().map(|s| s.len).collect();
        let total_len = seg_lengths.iter().sum();
        Self {
            points,
            segments,
            seg_lengths,
            total_len,
        }
    }
}

impl Path for Polyline2 {
    type Scalar = f64;
    type Point = Pt2;
    type Error = PathError<f64>;

    fn length(&self) -> Self::Scalar {
        self.total_len
    }

    fn sample_at(&self, s: Self::Scalar) -> Result<Self::Point, Self::Error> {
        if s < 0.0 || s > self.total_len {
            return Err(PathError::out_of_domain(s, self.domain()));
        }
        let (idx, local) = self.locate(s)?;
        self.segments[idx].sample_at(local)
    }
}

impl SegmentedPath for Polyline2 {
    type Segment = LineSegment2;

    fn segment_count(&self) -> usize {
        self.segments.len()
    }

    fn segments(&self) -> impl Iterator<Item = &Self::Segment> + '_ {
        self.segments.iter()
    }

    fn locate(&self, s: Self::Scalar) -> Result<(usize, Self::Scalar), Self::Error> {
        if s < 0.0 || s > self.total_len {
            return Err(PathError::out_of_domain(s, self.domain()));
        }
        let mut remaining = s;
        for (i, &seg_len) in self.seg_lengths.iter().enumerate() {
            if remaining <= seg_len {
                return Ok((i, remaining));
            }
            remaining -= seg_len;
        }
        Ok((
            self.segments.len() - 1,
            self.seg_lengths.last().copied().unwrap_or(0.0),
        ))
    }
}

#[test]
fn line_segment_length() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 4.0));
    assert!((seg.length() - 5.0).abs() < 1e-10);
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
    assert!((mid.0 - 1.0).abs() < 1e-10);
    assert!((mid.1 - 0.0).abs() < 1e-10);
}

#[test]
fn line_segment_out_of_domain() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(1.0, 0.0));
    let err = seg.sample_at(-0.1).unwrap_err();
    assert!(
        matches!(err, PathError::OutOfDomain { param, domain } if (param - -0.1).abs() < 1e-10 && *domain.start() == 0.0 && (*domain.end() - 1.0).abs() < 1e-10)
    );
    let err = seg.sample_at(1.1).unwrap_err();
    assert!(
        matches!(err, PathError::OutOfDomain { param, domain } if (param - 1.1).abs() < 1e-10 && *domain.start() == 0.0 && (*domain.end() - 1.0).abs() < 1e-10)
    );
}

#[test]
fn line_segment_tangent() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 4.0));
    let t = seg.tangent_at(2.5).unwrap();
    let expected = Vec2(0.6, 0.8);
    assert!((t.0 - expected.0).abs() < 1e-10);
    assert!((t.1 - expected.1).abs() < 1e-10);
}

#[test]
fn line_segment_heading() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(1.0, 1.0));
    let h = seg.heading_at(0.5).unwrap();
    assert!((h - core::f64::consts::FRAC_PI_4).abs() < 1e-10);
}

#[test]
fn line_segment_curvature_zero() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(5.0, 0.0));
    assert_eq!(seg.curvature_at(2.5).unwrap(), 0.0);
}

#[test]
fn polyline_segment_count() {
    let pts = vec![Pt2(0.0, 0.0), Pt2(1.0, 0.0), Pt2(1.0, 1.0)];
    let poly = Polyline2::new(pts);
    assert_eq!(poly.segment_count(), 2);
    assert_eq!(poly.segments().count(), 2);
}

#[test]
fn polyline_total_length() {
    let pts = vec![Pt2(0.0, 0.0), Pt2(3.0, 0.0), Pt2(3.0, 4.0)];
    let poly = Polyline2::new(pts);
    assert!((poly.length() - 7.0).abs() < 1e-10);
}

#[test]
fn polyline_locate_roundtrip() {
    let pts = vec![Pt2(0.0, 0.0), Pt2(2.0, 0.0), Pt2(2.0, 3.0)];
    let poly = Polyline2::new(pts);
    let (idx, local) = poly.locate(1.0).unwrap();
    assert_eq!(idx, 0);
    assert!((local - 1.0).abs() < 1e-10);
    let (idx, local) = poly.locate(3.0).unwrap();
    assert_eq!(idx, 1);
    assert!((local - 1.0).abs() < 1e-10);
}

#[test]
fn polyline_sample_via_locate() {
    let pts = vec![Pt2(0.0, 0.0), Pt2(2.0, 0.0), Pt2(2.0, 3.0)];
    let poly = Polyline2::new(pts);
    let (idx, local) = poly.locate(1.0).unwrap();
    let via_poly = poly.sample_at(1.0).unwrap();
    let via_seg = poly.segments[idx].sample_at(local).unwrap();
    assert_eq!(via_poly, via_seg);
}

#[test]
fn equidistant_samples() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(4.0, 0.0));
    let samples: Vec<_> = equidistant(&seg, 1.0).collect();
    assert_eq!(samples.len(), 5);
    for r in &samples {
        assert!(r.is_ok());
    }
}

#[test]
fn n_samples_count() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(10.0, 0.0));
    let samples: Vec<_> = n_samples(&seg, 3).collect();
    assert_eq!(samples.len(), 3);
}

#[test]
fn uniform_t_samples() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(10.0, 0.0));
    let samples: Vec<_> = uniform_t(&seg, 5).collect();
    assert_eq!(samples.len(), 5);
}

#[test]
fn reverse_double_equals_original() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(5.0, 0.0));
    let rev = seg.clone().reverse();
    let rev2 = rev.reverse();
    for s in [0.0, 1.25, 2.5, 3.75, 5.0] {
        let orig = seg.sample_at(s).unwrap();
        let double_rev = rev2.sample_at(s).unwrap();
        assert!((orig.0 - double_rev.0).abs() < 1e-10);
        assert!((orig.1 - double_rev.1).abs() < 1e-10);
    }
}

#[test]
fn concat_total_length() {
    let a = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 0.0));
    let b = LineSegment2::new(Pt2(3.0, 0.0), Pt2(3.0, 4.0));
    let c = a.concat(b);
    assert!((c.length() - 7.0).abs() < 1e-10);
}

#[test]
fn domain_matches_length() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 4.0));
    let domain = seg.domain();
    assert!(*domain.start() == 0.0);
    assert!((*domain.end() - 5.0).abs() < 1e-10);
}

#[test]
fn project_onto_segment() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(4.0, 0.0));
    let s = seg.project(Pt2(2.0, 3.0)).unwrap();
    assert!((s - 2.0).abs() < 1e-10);
}

#[test]
fn project_clamped_to_end() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 0.0));
    let s = seg.project(Pt2(5.0, 1.0)).unwrap();
    assert!((s - 3.0).abs() < 1e-10);
}

#[test]
fn project_clamped_to_start() {
    let seg = LineSegment2::new(Pt2(2.0, 0.0), Pt2(5.0, 0.0));
    let s = seg.project(Pt2(0.0, 1.0)).unwrap();
    assert!(s.abs() < 1e-10);
}

#[test]
fn closest_point_on_segment() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(4.0, 0.0));
    let cp = seg.closest_point(Pt2(1.0, 5.0)).unwrap();
    assert!((cp.0 - 1.0).abs() < 1e-10);
    assert!((cp.1 - 0.0).abs() < 1e-10);
}

#[test]
fn concat_sample_across_boundary() {
    let a = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 0.0));
    let b = LineSegment2::new(Pt2(3.0, 0.0), Pt2(3.0, 4.0));
    let c = a.concat(b);
    let junction = c.sample_at(3.0).unwrap();
    assert!((junction.0 - 3.0).abs() < 1e-10);
    assert!((junction.1 - 0.0).abs() < 1e-10);
    let into_b = c.sample_at(5.0).unwrap();
    assert!((into_b.0 - 3.0).abs() < 1e-10);
    assert!((into_b.1 - 2.0).abs() < 1e-10);
}

#[test]
fn reverse_tangent_negated() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(3.0, 4.0));
    let rev = seg.clone().reverse();
    let orig_t = seg.tangent_at(2.0).unwrap();
    let rev_t = rev.tangent_at(seg.length() - 2.0).unwrap();
    assert!((rev_t.0 - (-orig_t.0)).abs() < 1e-10);
    assert!((rev_t.1 - (-orig_t.1)).abs() < 1e-10);
}

#[test]
fn reverse_curvature_negated() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(5.0, 0.0));
    let rev = seg.clone().reverse();
    let orig_k = seg.curvature_at(2.0).unwrap();
    let rev_k = rev.curvature_at(3.0).unwrap();
    assert!((rev_k - (-orig_k)).abs() < 1e-10);
}

#[test]
fn reverse_start_is_original_end() {
    let seg = LineSegment2::new(Pt2(1.0, 2.0), Pt2(4.0, 6.0));
    let rev = seg.clone().reverse();
    let rev_start = rev.start().unwrap();
    let orig_end = seg.end().unwrap();
    assert_eq!(rev_start, orig_end);
}

#[test]
fn offset_horizontal_segment_shifts_x() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(5.0, 0.0));
    let d = 2.0;
    let offset_path = seg.clone().offset(d);
    let offset_mid = offset_path.sample_at(2.5).unwrap();
    let orig_mid = seg.sample_at(2.5).unwrap();
    assert!((offset_mid.0 - (orig_mid.0 + d)).abs() < 1e-10);
    assert!((offset_mid.1 - orig_mid.1).abs() < 1e-10);
}

#[test]
fn offset_negative_distance() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(5.0, 0.0));
    let d = -3.0;
    let offset_path = seg.clone().offset(d);
    let offset_mid = offset_path.sample_at(2.5).unwrap();
    let orig_mid = seg.sample_at(2.5).unwrap();
    assert!((offset_mid.0 - (orig_mid.0 + d)).abs() < 1e-10);
    assert!((offset_mid.1 - orig_mid.1).abs() < 1e-10);
}

#[test]
fn offset_preserves_length() {
    let seg = LineSegment2::new(Pt2(0.0, 0.0), Pt2(5.0, 0.0));
    let offset_path = seg.clone().offset(1.5);
    assert!((offset_path.length() - seg.length()).abs() < 1e-10);
}

#[test]
fn polyline_equidistant_iterator() {
    let pts = vec![Pt2(0.0, 0.0), Pt2(3.0, 0.0), Pt2(3.0, 4.0)];
    let poly = Polyline2::new(pts);
    let samples: Vec<_> = equidistant(&poly, 1.0).collect();
    assert_eq!(samples.len(), 8);
    for r in &samples {
        assert!(r.is_ok());
    }
}
