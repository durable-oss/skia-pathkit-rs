//! Helper class for segment intersection testing
//!
//! Port of Skia's SkIntersectionHelper.h

use crate::core::{Point, Scalar};
use crate::pathops::{sk_op_contour::SkOpContour, sk_op_segment::SkOpSegment};

/// Helper class for accessing segment data during intersection tests
pub struct SkIntersectionHelper {
    segment: Option<&'static mut SkOpSegment>,
}

impl SkIntersectionHelper {
    /// Constructs a helper with no segment bound. Call [`init`](Self::init)
    /// before any accessor.
    pub fn new() -> Self {
        Self { segment: None }
    }

    /// Binds the helper to `contour`'s first segment.
    pub fn init(&mut self, contour: &mut SkOpContour) {
        self.segment = Some(unsafe {
            // SAFETY: Caller guarantees contour lifetime
            std::mem::transmute(contour.first())
        });
    }

    /// Returns the currently bound segment. Panics if `init` has not run.
    pub fn segment(&self) -> &SkOpSegment {
        self.segment.as_ref().unwrap()
    }

    /// Classifies the current segment by verb, refining a line into its
    /// horizontal or vertical form so axis-aligned intersections can take a
    /// cheaper path.
    pub fn segment_type(&self) -> SegmentType {
        let seg = self.segment();
        let verb = seg.verb();
        let mut type_ = match verb {
            super::sk_op_segment::Verb::Line => SegmentType::Line,
            super::sk_op_segment::Verb::Quad => SegmentType::Quad,
            super::sk_op_segment::Verb::Conic => SegmentType::Conic,
            super::sk_op_segment::Verb::Cubic => SegmentType::Cubic,
        };

        if type_ == SegmentType::Line {
            if seg.is_horizontal() {
                type_ = SegmentType::HorizontalLine;
            } else if seg.is_vertical() {
                type_ = SegmentType::VerticalLine;
            }
        }

        type_
    }

    /// The segment's control points. Only the first `verb.point_count()`
    /// entries are meaningful for verbs below a cubic.
    pub fn pts(&self) -> &[Point; 4] {
        // Use transmute to get fixed-size array
        unsafe { std::mem::transmute(self.segment().pts()) }
    }

    /// Conic weight of the segment, or 1.0 for non-conic verbs.
    pub fn weight(&self) -> Scalar {
        self.segment().weight()
    }

    /// Axis-aligned bounding box of the segment.
    pub fn bounds(&self) -> SkPathOpsBounds {
        *self.segment().bounds()
    }

    /// Left edge of the segment's bounds.
    pub fn left(&self) -> Scalar {
        self.bounds().left
    }

    /// Right edge of the segment's bounds.
    pub fn right(&self) -> Scalar {
        self.bounds().right
    }

    /// Top edge of the segment's bounds.
    pub fn top(&self) -> Scalar {
        self.bounds().top
    }

    /// Bottom edge of the segment's bounds.
    pub fn bottom(&self) -> Scalar {
        self.bounds().bottom
    }

    /// Smallest x of the segment's bounds.
    pub fn x(&self) -> Scalar {
        self.left()
    }

    /// Smallest y of the segment's bounds.
    pub fn y(&self) -> Scalar {
        self.top()
    }

    /// True when the segment runs right to left, so its first point is not
    /// the one at the bounds' smallest x.
    pub fn x_flipped(&self) -> bool {
        self.x() != self.pts()[0].x
    }

    /// True when the segment runs bottom to top, so its first point is not
    /// the one at the bounds' smallest y.
    pub fn y_flipped(&self) -> bool {
        self.y() != self.pts()[0].y
    }

    /// Repositions this helper to the segment following `after`. Not ported
    /// yet; always returns false.
    pub fn start_after(&mut self, _after: &SkIntersectionHelper) -> bool {
        // Simplified: not implemented in this version
        false
    }

    /// Steps to the next segment in the contour. Not ported yet; always
    /// returns false.
    pub fn advance(&mut self) -> bool {
        // Simplified: not implemented in this version
        false
    }
}

impl Default for SkIntersectionHelper {
    fn default() -> Self {
        Self::new()
    }
}

/// Segment type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentType {
    /// A line with equal y at both ends. Sorts below every other type so
    /// horizontal sweeps can be handled first.
    HorizontalLine = -1,
    /// A line with equal x at both ends.
    VerticalLine = 0,
    /// A line in general position.
    Line = 1,
    /// A quadratic Bezier.
    Quad = 2,
    /// A rational quadratic, carrying a weight alongside its points.
    Conic = 3,
    /// A cubic Bezier.
    Cubic = 4,
}

/// Bounds for path operations
#[derive(Debug, Clone, Copy, Default)]
pub struct SkPathOpsBounds {
    /// Smallest x.
    pub left: Scalar,
    /// Smallest y.
    pub top: Scalar,
    /// Largest x.
    pub right: Scalar,
    /// Largest y.
    pub bottom: Scalar,
}

impl SkPathOpsBounds {
    /// Returns inverted bounds, so the first [`add`](Self::add) sets every
    /// edge from the incoming box.
    pub fn empty() -> Self {
        Self {
            left: Scalar::MAX,
            top: Scalar::MAX,
            right: Scalar::MIN,
            bottom: Scalar::MIN,
        }
    }

    /// Grows these bounds to also cover `other`.
    pub fn add(&mut self, other: &Self) {
        self.left = self.left.min(other.left);
        self.top = self.top.min(other.top);
        self.right = self.right.max(other.right);
        self.bottom = self.bottom.max(other.bottom);
    }

    /// True if `a` and `b` overlap. Touching edges count as overlapping, and
    /// the comparison carries a tolerance so bounds that meet within rounding
    /// error are not missed.
    pub fn intersects(a: &Self, b: &Self) -> bool {
        almost_less_or_equal_ulps(a.left, b.right)
            && almost_less_or_equal_ulps(b.left, a.right)
            && almost_less_or_equal_ulps(a.top, b.bottom)
            && almost_less_or_equal_ulps(b.top, a.bottom)
    }
}

/// Compare with ULP tolerance
fn almost_less_or_equal_ulps(a: Scalar, b: Scalar) -> bool {
    a <= b + 1e-10
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bounds_intersects() {
        let a = SkPathOpsBounds {
            left: 0.0,
            top: 0.0,
            right: 10.0,
            bottom: 10.0,
        };
        let b = SkPathOpsBounds {
            left: 5.0,
            top: 5.0,
            right: 15.0,
            bottom: 15.0,
        };
        assert!(SkPathOpsBounds::intersects(&a, &b));

        let c = SkPathOpsBounds {
            left: 20.0,
            top: 20.0,
            right: 30.0,
            bottom: 30.0,
        };
        assert!(!SkPathOpsBounds::intersects(&a, &c));
    }
}
