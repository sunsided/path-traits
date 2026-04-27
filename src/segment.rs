//! Segmentation traits.
//!
//! This module provides:
//!
//! - [`PathSegment`] — a marker trait indicating a path is a primitive that is
//!   not further segmented.
//! - [`SegmentedPath`] — a path composed of multiple segments, offering
//!   enumeration, indexing, and arc-length-to-segment location.

use crate::Path;

/// Marker trait for a path primitive that is not further segmented.
///
/// Types implementing this trait represent atomic path building blocks (e.g. a
/// line segment, a Bézier curve). They behave exactly like a [`Path`] but signal
/// to [`SegmentedPath`] consumers that no further subdivision is expected.
pub trait PathSegment: Path {}

/// A path composed of multiple [`PathSegment`]s.
///
/// Provides methods to enumerate segments, access them by index, and map a
/// global arc-length parameter to a `(segment_index, local_s)` pair.
pub trait SegmentedPath: Path {
    /// The type of individual segments making up this path.
    type Segment: PathSegment<Scalar = Self::Scalar, Point = Self::Point, Error = Self::Error>;

    /// Number of segments in this path.
    fn segment_count(&self) -> usize;

    /// Iterator over all segments.
    fn segments(&self) -> impl Iterator<Item = &Self::Segment> + '_;

    /// Get the segment at index `i`, or `None` if out of bounds.
    fn segment(&self, i: usize) -> Option<&Self::Segment> {
        self.segments().nth(i)
    }

    /// Map global arc-length `s` to `(segment_index, local_s)`.
    ///
    /// Returns the index of the segment containing `s` and the local arc-length
    /// within that segment. Errors when `s` is outside the path's domain.
    fn locate(&self, s: Self::Scalar) -> Result<(usize, Self::Scalar), Self::Error>;
}
