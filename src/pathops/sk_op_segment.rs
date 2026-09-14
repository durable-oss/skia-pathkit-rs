//! SkOpSegment - represents a path segment for path operations
//!
//! Port of Skia's SkOpSegment.{h,cpp}

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

use super::sk_intersection_helper::SkPathOpsBounds;
use crate::core::{Point, Scalar};

/// Path segment verb types (matches Skia's SkPath::Verb)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verb {
    /// A straight segment between two points. The discriminant matches
    /// Skia's, and also equals the segment's polynomial degree.
    Line = 1,
    /// A quadratic Bezier.
    Quad = 2,
    /// A rational quadratic, carrying a weight alongside its points.
    Conic = 3,
    /// A cubic Bezier.
    Cubic = 4,
}

impl Verb {
    /// Returns the number of points needed for this verb
    pub fn point_count(&self) -> usize {
        match self {
            Verb::Line => 2,
            Verb::Quad => 3,
            Verb::Conic => 3,
            Verb::Cubic => 4,
        }
    }
}

/// Span base with t and point
#[derive(Debug, Clone, Copy)]
pub struct SkOpSpanBase {
    /// Parametric position along the owning segment, in [0, 1].
    pub t: Scalar,
    /// The point the segment reaches at `t`.
    pub pt: Point,
}

impl SkOpSpanBase {
    /// Creates a new span base at given t and point
    pub fn new(t: Scalar, pt: Point) -> Self {
        Self { t, pt }
    }
}

/// Span with winding data
#[derive(Debug, Clone)]
pub struct SkOpSpan {
    /// The span's position along its segment.
    pub base: SkOpSpanBase,
    /// This span's own winding contribution.
    pub wind_value: i32,
    /// This span's contribution to the opposite operand's winding.
    pub opp_value: i32,
    /// Winding accumulated from the start of the contour up to this span.
    pub wind_sum: i32,
    /// Accumulated opposite-operand winding.
    pub opp_sum: i32,
    /// True once this span has been resolved and needs no further work.
    pub done: bool,
    /// True once this span has been emitted into an output contour.
    pub already_added: bool,
    prev: Option<Box<SkOpSpanBase>>,
    next: Option<Box<SkOpSpanBase>>,
}

impl SkOpSpan {
    /// Creates a new span
    pub fn new(t: Scalar, pt: Point) -> Self {
        Self {
            base: SkOpSpanBase::new(t, pt),
            wind_value: 0,
            opp_value: 0,
            wind_sum: i32::MIN,
            opp_sum: i32::MIN,
            done: false,
            already_added: false,
            prev: None,
            next: None,
        }
    }

    /// Creates a new span with base
    pub fn with_base(base: SkOpSpanBase) -> Self {
        Self {
            base,
            wind_value: 0,
            opp_value: 0,
            wind_sum: i32::MIN,
            opp_sum: i32::MIN,
            done: false,
            already_added: false,
            prev: None,
            next: None,
        }
    }

    /// Returns the t value
    pub fn t(&self) -> Scalar {
        self.base.t
    }

    /// Returns the point
    pub fn pt(&self) -> Point {
        self.base.pt
    }
}

/// A path segment
#[derive(Debug)]
pub struct SkOpSegment {
    /// Points defining the segment
    pts: [Point; 4],
    /// Verb type
    verb: Verb,
    /// Weight (for conics)
    weight: Scalar,
    /// Bounds
    bounds: SkPathOpsBounds,
    /// Head span
    head: SkOpSpan,
    /// Tail span
    tail: SkOpSpanBase,
    /// Next segment in chain
    next: Option<Box<SkOpSegment>>,
    /// Previous segment in chain
    prev: Option<Box<SkOpSegment>>,
    /// Count of spans
    count: i32,
    /// Done span count
    done_count: i32,
    /// Debug ID
    #[cfg(debug_assertions)]
    id: i32,
}

impl SkOpSegment {
    /// Creates a new empty segment
    pub fn new() -> Self {
        Self {
            pts: [Point::new(0.0, 0.0); 4],
            verb: Verb::Line,
            weight: 1.0,
            bounds: SkPathOpsBounds::default(),
            head: SkOpSpan::with_base(SkOpSpanBase::new(0.0, Point::new(0.0, 0.0))),
            tail: SkOpSpanBase::new(1.0, Point::new(0.0, 0.0)),
            next: None,
            prev: None,
            count: 1,
            done_count: 0,
            #[cfg(debug_assertions)]
            id: -1,
        }
    }

    /// Creates a new line segment
    pub fn new_line(p0: Point, p1: Point) -> Self {
        let bounds = SkPathOpsBounds {
            left: p0.x.min(p1.x),
            top: p0.y.min(p1.y),
            right: p0.x.max(p1.x),
            bottom: p0.y.max(p1.y),
        };

        let mut seg = Self {
            pts: [Point::new(0.0, 0.0); 4],
            verb: Verb::Line,
            weight: 1.0,
            bounds,
            head: SkOpSpan::with_base(SkOpSpanBase::new(0.0, p0)),
            tail: SkOpSpanBase::new(1.0, p1),
            next: None,
            prev: None,
            count: 1,
            done_count: 0,
            #[cfg(debug_assertions)]
            id: -1,
        };
        seg.pts[0] = p0;
        seg.pts[1] = p1;
        seg
    }

    /// Adds a line to the segment
    pub fn add_line(&mut self, p0: Point, p1: Point) {
        self.pts[0] = p0;
        self.pts[1] = p1;
        self.pts[2] = Point::new(0.0, 0.0);
        self.pts[3] = Point::new(0.0, 0.0);
        self.verb = Verb::Line;
        self.weight = 1.0;
        self.bounds = SkPathOpsBounds {
            left: p0.x.min(p1.x),
            top: p0.y.min(p1.y),
            right: p0.x.max(p1.x),
            bottom: p0.y.max(p1.y),
        };
        self.head.base.t = 0.0;
        self.head.base.pt = p0;
        self.tail.t = 1.0;
        self.tail.pt = p1;
    }

    /// Adds a quad to the segment
    pub fn add_quad(&mut self, pts: [Point; 3]) {
        self.pts[0] = pts[0];
        self.pts[1] = pts[1];
        self.pts[2] = pts[2];
        self.pts[3] = Point::new(0.0, 0.0);
        self.verb = Verb::Quad;
        self.weight = 1.0;
        self.bounds = Self::compute_quad_bounds(pts);
        self.head.base.pt = pts[0];
        self.tail.pt = pts[2];
    }

    /// Adds a conic to the segment
    pub fn add_conic(&mut self, pts: [Point; 3], weight: Scalar) {
        self.pts[0] = pts[0];
        self.pts[1] = pts[1];
        self.pts[2] = pts[2];
        self.pts[3] = Point::new(0.0, 0.0);
        self.verb = Verb::Conic;
        self.weight = weight;
        self.bounds = Self::compute_conic_bounds(pts, weight);
        self.head.base.pt = pts[0];
        self.tail.pt = pts[2];
    }

    /// Adds a cubic to the segment
    pub fn add_cubic(&mut self, pts: [Point; 4]) {
        self.pts = pts;
        self.verb = Verb::Cubic;
        self.weight = 1.0;
        self.bounds = Self::compute_cubic_bounds(pts);
        self.head.base.pt = pts[0];
        self.tail.pt = pts[3];
    }

    /// Computes quad bounds
    fn compute_quad_bounds(pts: [Point; 3]) -> SkPathOpsBounds {
        let mut bounds = SkPathOpsBounds {
            left: pts[0].x,
            top: pts[0].y,
            right: pts[0].x,
            bottom: pts[0].y,
        };
        for pt in &pts[1..] {
            bounds.left = bounds.left.min(pt.x);
            bounds.top = bounds.top.min(pt.y);
            bounds.right = bounds.right.max(pt.x);
            bounds.bottom = bounds.bottom.max(pt.y);
        }
        // Add control point contribution
        bounds.left = bounds.left.min(pts[1].x);
        bounds.top = bounds.top.min(pts[1].y);
        bounds.right = bounds.right.max(pts[1].x);
        bounds.bottom = bounds.bottom.max(pts[1].y);
        bounds
    }

    /// Computes conic bounds
    fn compute_conic_bounds(pts: [Point; 3], _weight: Scalar) -> SkPathOpsBounds {
        let mut bounds = SkPathOpsBounds {
            left: pts[0].x.min(pts[2].x),
            top: pts[0].y.min(pts[2].y),
            right: pts[0].x.max(pts[2].x),
            bottom: pts[0].y.max(pts[2].y),
        };
        bounds.left = bounds.left.min(pts[1].x);
        bounds.top = bounds.top.min(pts[1].y);
        bounds.right = bounds.right.max(pts[1].x);
        bounds.bottom = bounds.bottom.max(pts[1].y);
        bounds
    }

    /// Computes cubic bounds
    fn compute_cubic_bounds(pts: [Point; 4]) -> SkPathOpsBounds {
        let mut bounds = SkPathOpsBounds {
            left: pts[0].x,
            top: pts[0].y,
            right: pts[0].x,
            bottom: pts[0].y,
        };
        for pt in &pts[1..] {
            bounds.left = bounds.left.min(pt.x);
            bounds.top = bounds.top.min(pt.y);
            bounds.right = bounds.right.max(pt.x);
            bounds.bottom = bounds.bottom.max(pt.y);
        }
        bounds
    }

    /// Returns the verb type
    pub fn verb(&self) -> Verb {
        self.verb
    }

    /// Returns the points
    pub fn pts(&self) -> &[Point; 4] {
        &self.pts
    }

    /// Returns the weight
    pub fn weight(&self) -> Scalar {
        self.weight
    }

    /// Returns the bounds
    pub fn bounds(&self) -> &SkPathOpsBounds {
        &self.bounds
    }

    /// Returns the head span
    pub fn head(&self) -> &SkOpSpan {
        &self.head
    }

    /// Returns the tail span
    pub fn tail(&self) -> &SkOpSpanBase {
        &self.tail
    }

    /// Returns the next segment
    pub fn next(&self) -> Option<&SkOpSegment> {
        self.next.as_ref().map(|n| n.as_ref())
    }

    /// Returns the previous segment
    pub fn prev(&self) -> Option<&SkOpSegment> {
        self.prev.as_ref().map(|p| p.as_ref())
    }

    /// Sets the next segment
    pub fn set_next(&mut self, next: Option<Box<SkOpSegment>>) {
        self.next = next;
    }

    /// Sets the previous segment
    pub fn set_prev(&mut self, prev: Option<Box<SkOpSegment>>) {
        self.prev = prev;
    }

    /// Returns true if the segment is horizontal
    pub fn is_horizontal(&self) -> bool {
        self.bounds.top == self.bounds.bottom
    }

    /// Returns true if the segment is vertical
    pub fn is_vertical(&self) -> bool {
        self.bounds.left == self.bounds.right
    }

    /// Returns true if the segment is done
    pub fn is_done(&self) -> bool {
        self.done_count >= self.count
    }

    /// Marks all spans as done
    pub fn mark_all_done(&mut self) {
        self.done_count = self.count;
    }

    /// Returns the first undone span
    pub fn undone_span(&self) -> Option<&SkOpSpan> {
        if self.done_count < self.count {
            Some(&self.head)
        } else {
            None
        }
    }

    /// Returns the point at t
    pub fn pt_at_t(&self, t: Scalar) -> Point {
        match self.verb {
            Verb::Line => {
                let p0 = self.pts[0];
                let p1 = self.pts[1];
                Point::new(p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t)
            }
            Verb::Quad => {
                let p0 = self.pts[0];
                let p1 = self.pts[1];
                let p2 = self.pts[2];
                let one_minus_t = 1.0 - t;
                Point::new(
                    p0.x * one_minus_t * one_minus_t + p1.x * 2.0 * one_minus_t * t + p2.x * t * t,
                    p0.y * one_minus_t * one_minus_t + p1.y * 2.0 * one_minus_t * t + p2.y * t * t,
                )
            }
            Verb::Cubic => {
                let p0 = self.pts[0];
                let p1 = self.pts[1];
                let p2 = self.pts[2];
                let p3 = self.pts[3];
                let one_minus_t = 1.0 - t;
                let one_minus_t2 = one_minus_t * one_minus_t;
                let t2 = t * t;
                let one_minus_t3 = one_minus_t2 * one_minus_t;
                let t3 = t2 * t;
                Point::new(
                    p0.x * one_minus_t3
                        + p1.x * 3.0 * one_minus_t2 * t
                        + p2.x * 3.0 * one_minus_t * t2
                        + p3.x * t3,
                    p0.y * one_minus_t3
                        + p1.y * 3.0 * one_minus_t2 * t
                        + p2.y * 3.0 * one_minus_t * t2
                        + p3.y * t3,
                )
            }
            Verb::Conic => {
                let p0 = self.pts[0];
                let p1 = self.pts[1];
                let p2 = self.pts[2];
                let w = self.weight;
                let one_minus_t = 1.0 - t;
                let denom = one_minus_t.powi(2) + 2.0 * one_minus_t * t * w + t.powi(2);
                let mt = one_minus_t / denom;
                let wt = t * w / denom;
                let t2 = t * t / denom;
                Point::new(
                    mt * p0.x + 2.0 * wt * p1.x + t2 * p2.x,
                    mt * p0.y + 2.0 * wt * p1.y + t2 * p2.y,
                )
            }
        }
    }

    /// Returns the last point
    pub fn last_pt(&self) -> Point {
        match self.verb {
            Verb::Line => self.pts[1],
            Verb::Quad => self.pts[2],
            Verb::Conic => self.pts[2],
            Verb::Cubic => self.pts[3],
        }
    }

    /// Returns whether the segment contains t
    pub fn contains(&self, t: Scalar) -> bool {
        t >= 0.0 && t <= 1.0
    }

    /// Superseded by [`OpArena::segment_add_t`].
    ///
    /// [`OpArena::segment_add_t`]: super::sk_op_arena::OpArena::segment_add_t
    ///
    /// Inserting a span means allocating one and relinking a shared graph, so
    /// it belongs on the arena rather than on a segment that owns its spans by
    /// value. This form remains only until the callers listed in
    /// `TODO/05-op-segment-winding.md` move across, and always returns `None`.
    #[deprecated(note = "use OpArena::segment_add_t")]
    pub fn add_t(&mut self, _t: Scalar, _pt: Point) -> Option<&mut SkOpSpan> {
        None
    }

    /// Not ported: builds the angles at each span. Needs `SkOpAngle::set`,
    /// which is item 04's remaining half. Does nothing.
    pub fn calc_angles(&mut self) {}

    /// Superseded by [`OpArena::mark_and_chase_done`].
    ///
    /// [`OpArena::mark_and_chase_done`]: super::sk_op_arena::OpArena::mark_and_chase_done
    ///
    /// Always reports success without marking anything.
    #[deprecated(note = "use OpArena::mark_and_chase_done")]
    pub fn mark_and_chase_done(
        &mut self,
        _start: &SkOpSpan,
        _end: &SkOpSpan,
        _chase: &mut Option<&SkOpSpan>,
    ) -> bool {
        true
    }

    /// Not ported: walks to the next segment of a boolean result. Needs the
    /// sorted angle loop, which is item 04. Always returns `None`.
    #[allow(clippy::too_many_arguments)] // mirrors the C++ signature
    pub fn find_next_op(
        &self,
        _chase: &mut Vec<&SkOpSpan>,
        _start: &SkOpSpan,
        _end: &SkOpSpan,
        _unsortable: &mut bool,
        _last_simple: &mut bool,
        _op: crate::pathops::PathOp,
        _xor_mask: i32,
        _xor_op_mask: i32,
    ) -> Option<&SkOpSegment> {
        None
    }

    /// Not ported: emits this segment's curve into a path writer. The other
    /// C++ overload, which fills a curve rather than a writer, is ported as
    /// [`sub_divide_curve`](super::sk_op_angle::sub_divide_curve). Always
    /// reports success without writing anything.
    pub fn sub_divide(
        &self,
        _start: &SkOpSpan,
        _end: &SkOpSpan,
        _writer: &mut crate::pathops::sk_path_writer::SkPathWriter,
    ) -> bool {
        true
    }

    /// Not ported: finds coincident runs this segment should have recorded.
    /// Item 06. Always reports none.
    pub fn missing_coincidence(&self) -> bool {
        false
    }

    /// Not ported: merges spans that share a point. Item 05 part 9. Always
    /// reports success.
    pub fn move_multiples(&mut self) -> bool {
        true
    }

    /// Not ported: merges spans that are nearly the same point. Item 05
    /// part 9. Always reports success.
    pub fn move_nearby(&mut self) -> bool {
        true
    }

    /// Not ported: sorts each span's angle loop. Needs `SkOpAngle::after`,
    /// which is item 04's remaining half. Always reports success.
    pub fn sort_angles(&mut self) -> bool {
        true
    }

    /// Validates the segment (debug builds only)
    #[cfg(debug_assertions)]
    pub fn debug_validate(&self) {
        assert!(self.count >= 1);
        assert!(self.done_count <= self.count);
    }
}

impl Default for SkOpSegment {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_line() {
        let seg = SkOpSegment::new_line(Point::new(0.0, 0.0), Point::new(10.0, 10.0));
        assert_eq!(seg.verb(), Verb::Line);
        assert_eq!(seg.pts()[0], Point::new(0.0, 0.0));
        assert_eq!(seg.pts()[1], Point::new(10.0, 10.0));
    }

    #[test]
    fn test_add_line() {
        let mut seg = SkOpSegment::new();
        seg.add_line(Point::new(5.0, 5.0), Point::new(15.0, 15.0));
        assert_eq!(seg.verb(), Verb::Line);
    }

    #[test]
    fn test_add_quad() {
        let mut seg = SkOpSegment::new();
        seg.add_quad([
            Point::new(0.0, 0.0),
            Point::new(5.0, 10.0),
            Point::new(10.0, 0.0),
        ]);
        assert_eq!(seg.verb(), Verb::Quad);
    }

    #[test]
    fn test_add_conic() {
        let mut seg = SkOpSegment::new();
        seg.add_conic(
            [
                Point::new(0.0, 0.0),
                Point::new(5.0, 10.0),
                Point::new(10.0, 0.0),
            ],
            1.0,
        );
        assert_eq!(seg.verb(), Verb::Conic);
        assert_eq!(seg.weight(), 1.0);
    }

    #[test]
    fn test_add_cubic() {
        let mut seg = SkOpSegment::new();
        seg.add_cubic([
            Point::new(0.0, 0.0),
            Point::new(3.3, 3.3),
            Point::new(6.6, 6.6),
            Point::new(10.0, 10.0),
        ]);
        assert_eq!(seg.verb(), Verb::Cubic);
    }

    #[test]
    fn test_is_horizontal() {
        let seg = SkOpSegment::new_line(Point::new(0.0, 5.0), Point::new(10.0, 5.0));
        assert!(seg.is_horizontal());
    }

    #[test]
    fn test_is_vertical() {
        let seg = SkOpSegment::new_line(Point::new(5.0, 0.0), Point::new(5.0, 10.0));
        assert!(seg.is_vertical());
    }

    #[test]
    fn test_verb_point_count() {
        assert_eq!(Verb::Line.point_count(), 2);
        assert_eq!(Verb::Quad.point_count(), 3);
        assert_eq!(Verb::Conic.point_count(), 3);
        assert_eq!(Verb::Cubic.point_count(), 4);
    }

    #[test]
    fn test_span_methods() {
        let span = SkOpSpan::new(0.5, Point::new(5.0, 5.0));
        assert!((span.t() - 0.5).abs() < 1e-10);
        assert_eq!(span.pt(), Point::new(5.0, 5.0));
        assert!(!span.done);
    }

    #[test]
    fn test_mark_all_done() {
        let mut seg = SkOpSegment::new();
        seg.done_count = 0;
        seg.count = 1;
        assert!(!seg.is_done());
        seg.mark_all_done();
        assert!(seg.is_done());
    }

    #[test]
    fn test_pt_at_t() {
        let seg = SkOpSegment::new_line(Point::new(0.0, 0.0), Point::new(10.0, 0.0));
        let pt = seg.pt_at_t(0.5);
        assert!((pt.x - 5.0).abs() < 1e-10);
        assert!((pt.y - 0.0).abs() < 1e-10);
    }
}
