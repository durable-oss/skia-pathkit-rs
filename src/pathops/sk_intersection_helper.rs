//! Helper class for segment intersection testing
//!
//! Port of Skia's SkIntersectionHelper.h

use crate::core::{Point, Scalar};
use crate::pathops::{
    sk_op_contour::SkOpContour,
    sk_op_segment::SkOpSegment,
};

/// Helper class for accessing segment data during intersection tests
pub struct SkIntersectionHelper {
    segment: Option<&'static mut SkOpSegment>,
}

impl SkIntersectionHelper {
    pub fn new() -> Self {
        Self { segment: None }
    }

    pub fn init(&mut self, contour: &mut SkOpContour) {
        self.segment = Some(unsafe {
            // SAFETY: Caller guarantees contour lifetime
            std::mem::transmute(contour.first())
        });
    }

    pub fn segment(&self) -> &SkOpSegment {
        self.segment.as_ref().unwrap()
    }

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

    pub fn pts(&self) -> &[Point; 4] {
        // Use transmute to get fixed-size array
        unsafe { std::mem::transmute(self.segment().pts()) }
    }

    pub fn weight(&self) -> Scalar {
        self.segment().weight()
    }

    pub fn bounds(&self) -> SkPathOpsBounds {
        *self.segment().bounds()
    }

    pub fn left(&self) -> Scalar {
        self.bounds().left
    }

    pub fn right(&self) -> Scalar {
        self.bounds().right
    }

    pub fn top(&self) -> Scalar {
        self.bounds().top
    }

    pub fn bottom(&self) -> Scalar {
        self.bounds().bottom
    }

    pub fn x(&self) -> Scalar {
        self.left()
    }

    pub fn y(&self) -> Scalar {
        self.top()
    }

    pub fn x_flipped(&self) -> bool {
        self.x() != self.pts()[0].x
    }

    pub fn y_flipped(&self) -> bool {
        self.y() != self.pts()[0].y
    }

    pub fn start_after(&mut self, _after: &SkIntersectionHelper) -> bool {
        // Simplified: not implemented in this version
        false
    }

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
    HorizontalLine = -1,
    VerticalLine = 0,
    Line = 1,
    Quad = 2,
    Conic = 3,
    Cubic = 4,
}

/// Bounds for path operations
#[derive(Debug, Clone, Copy, Default)]
pub struct SkPathOpsBounds {
    pub left: Scalar,
    pub top: Scalar,
    pub right: Scalar,
    pub bottom: Scalar,
}

impl SkPathOpsBounds {
    pub fn empty() -> Self {
        Self {
            left: Scalar::MAX,
            top: Scalar::MAX,
            right: Scalar::MIN,
            bottom: Scalar::MIN,
        }
    }

    pub fn add(&mut self, other: &Self) {
        self.left = self.left.min(other.left);
        self.top = self.top.min(other.top);
        self.right = self.right.max(other.right);
        self.bottom = self.bottom.max(other.bottom);
    }

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
            left: 0.0, top: 0.0, right: 10.0, bottom: 10.0,
        };
        let b = SkPathOpsBounds {
            left: 5.0, top: 5.0, right: 15.0, bottom: 15.0,
        };
        assert!(SkPathOpsBounds::intersects(&a, &b));

        let c = SkPathOpsBounds {
            left: 20.0, top: 20.0, right: 30.0, bottom: 30.0,
        };
        assert!(!SkPathOpsBounds::intersects(&a, &c));
    }
}
