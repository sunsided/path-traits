//! Join two paths end-to-end.
//!
//! [`Concat`] connects two compatible paths into a single continuous path.
//! The combined length is the sum of the inner lengths, and sampling delegates
//! to the appropriate segment. When both inner paths are
//! [`SegmentedPath`](crate::SegmentedPath), the concatenation is itself a
//! [`SegmentedPath`](crate::SegmentedPath).

use crate::{Path, PathSegment, SegmentedPath};

/// Two paths joined end-to-end into a single continuous path.
///
/// The combined path has length `first.length() + second.length()`. Sampling
/// below `first.length()` delegates to the first path; the remainder delegates
/// to the second with a shifted arc-length.
pub struct Concat<A, B> {
    /// The first (prefix) path.
    first: A,
    /// The second (suffix) path.
    second: B,
}

impl<A, B> Concat<A, B> {
    /// Create a new concatenated path.
    pub fn new(first: A, second: B) -> Self {
        Self { first, second }
    }
}

impl<A: Path, B: Path<Scalar = A::Scalar, Point = A::Point, Error = A::Error>> Path
    for Concat<A, B>
{
    type Scalar = A::Scalar;
    type Point = A::Point;
    type Error = A::Error;

    fn length(&self) -> Self::Scalar {
        self.first.length() + self.second.length()
    }

    fn sample_at(&self, s: Self::Scalar) -> Result<Self::Point, Self::Error> {
        let first_len = self.first.length();
        if s <= first_len {
            self.first.sample_at(s)
        } else {
            self.second.sample_at(s - first_len)
        }
    }
}

impl<A: SegmentedPath, B: SegmentedPath<Segment = A::Segment>> SegmentedPath for Concat<A, B>
where
    A: Path,
    B: Path<Scalar = A::Scalar, Point = A::Point, Error = A::Error>,
    A::Segment: PathSegment<Scalar = A::Scalar, Point = A::Point, Error = A::Error>,
{
    type Segment = A::Segment;

    fn segment_count(&self) -> usize {
        self.first.segment_count() + self.second.segment_count()
    }

    fn segments(&self) -> impl Iterator<Item = &Self::Segment> + '_ {
        self.first.segments().chain(self.second.segments())
    }

    fn locate(&self, s: Self::Scalar) -> Result<(usize, Self::Scalar), Self::Error> {
        let first_len = self.first.length();
        if s <= first_len {
            self.first.locate(s)
        } else {
            let (idx, local) = self.second.locate(s - first_len)?;
            Ok((self.first.segment_count() + idx, local))
        }
    }
}
