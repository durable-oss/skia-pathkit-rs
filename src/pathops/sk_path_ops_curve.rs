//! The curve union used by the intersection code.
//!
//! Port of Skia's `SkPathOpsCurve.{h,cpp}`.
//!
//! [`SkDCurve`] is a tagged union over the double-precision curve types and
//! [`SkDCurveSweep`] wraps one with the hull sweep the intersection code uses
//! to decide whether a segment really bends. The curve types themselves live
//! in their own modules, matching the upstream header layout: this file
//! declares neither the points nor the curves, only the union over them.

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

use super::sk_path_ops_conic::SkDConic;
use super::sk_path_ops_cubic::SkDCubic;
use super::sk_path_ops_line::SkDLine;
use super::sk_path_ops_point::{SkDPoint, SkDVector};
use super::sk_path_ops_quad::SkDQuad;
use super::sk_path_ops_types::roughly_zero_when_compared_to;

/// Verb type for path segments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verb {
    /// Start a new contour at a single point.
    Move,
    /// Straight segment to the next point.
    Line,
    /// Quadratic Bezier: one off-curve control and an end point.
    Quad,
    /// Rational quadratic: like a quad, plus a weight.
    Conic,
    /// Cubic Bezier: two off-curve controls and an end point.
    Cubic,
    /// Close the current contour.
    Close,
}

impl Verb {
    /// Number of points the verb consumes, counting its starting point.
    #[must_use]
    pub fn point_count(&self) -> usize {
        match self {
            Verb::Move => 1,
            Verb::Line => 2,
            Verb::Quad | Verb::Conic => 3,
            Verb::Cubic => 4,
            Verb::Close => 0,
        }
    }
}

/// One curve of any type.
///
/// Upstream this is a union plus a separate verb tag; here the verb rides
/// along with the variant.
#[derive(Debug, Clone, Copy)]
pub enum SkDCurve {
    /// A straight segment between two points.
    Line(SkDLine),
    /// A quadratic Bezier.
    Quad(SkDQuad),
    /// A rational quadratic, that is, a weighted quadratic.
    Conic(SkDConic),
    /// A cubic Bezier.
    Cubic(SkDCubic),
}

impl Default for SkDCurve {
    fn default() -> Self {
        SkDCurve::Line(SkDLine::default())
    }
}

impl SkDCurve {
    /// The verb this curve was built from.
    #[must_use]
    pub fn verb(&self) -> Verb {
        match self {
            SkDCurve::Line(_) => Verb::Line,
            SkDCurve::Quad(_) => Verb::Quad,
            SkDCurve::Conic(_) => Verb::Conic,
            SkDCurve::Cubic(_) => Verb::Cubic,
        }
    }

    /// How many control points the curve carries.
    #[must_use]
    pub fn point_count(&self) -> usize {
        self.verb().point_count()
    }

    /// Control point `index`, counting from the start point.
    #[must_use]
    pub fn point(&self, index: usize) -> SkDPoint {
        match self {
            SkDCurve::Line(l) => l[index],
            SkDCurve::Quad(q) => q[index],
            SkDCurve::Conic(c) => c[index],
            SkDCurve::Cubic(c) => c[index],
        }
    }

    /// Mutable access to control point `index`.
    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        match self {
            SkDCurve::Line(l) => &mut l[index],
            SkDCurve::Quad(q) => &mut q[index],
            SkDCurve::Conic(c) => &mut c[index],
            SkDCurve::Cubic(c) => &mut c[index],
        }
    }

    /// Translates every control point by `off`.
    pub fn offset(&mut self, off: SkDVector) {
        for i in 0..self.point_count() {
            let moved = self.point(i) + off;
            *self.point_mut(i) = moved;
        }
    }

    /// The point the curve reaches at parameter `t`.
    #[must_use]
    pub fn pt_at_t(&self, t: f64) -> SkDPoint {
        match self {
            SkDCurve::Line(l) => l.pt_at_t(t),
            SkDCurve::Quad(q) => q.pt_at_t(t),
            SkDCurve::Conic(c) => c.pt_at_t(t),
            SkDCurve::Cubic(c) => c.pt_at_t(t),
        }
    }

    /// The curve's tangent at parameter `t`.
    ///
    /// A line's derivative is constant, so it is reported as the vector from
    /// its start to its end rather than as zero.
    #[must_use]
    pub fn dxdy_at_t(&self, t: f64) -> SkDVector {
        match self {
            SkDCurve::Line(l) => l[1] - l[0],
            SkDCurve::Quad(q) => q.dxdy_at_t(t),
            SkDCurve::Conic(c) => c.dxdy_at_t(t),
            SkDCurve::Cubic(c) => c.dxdy_at_t(t),
        }
    }

    /// True if the curve never reverses in x.
    #[must_use]
    pub fn monotonic_in_x(&self) -> bool {
        match self {
            SkDCurve::Line(_) => true,
            SkDCurve::Quad(q) => q.monotonic_in_x(),
            SkDCurve::Conic(c) => c.monotonic_in_x(),
            SkDCurve::Cubic(c) => c.monotonic_in_x(),
        }
    }

    /// True if the curve never reverses in y.
    #[must_use]
    pub fn monotonic_in_y(&self) -> bool {
        match self {
            SkDCurve::Line(_) => true,
            SkDCurve::Quad(q) => q.monotonic_in_y(),
            SkDCurve::Conic(c) => c.monotonic_in_y(),
            SkDCurve::Cubic(c) => c.monotonic_in_y(),
        }
    }
}

/// A curve plus the hull sweep bounding its direction of travel.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDCurveSweep {
    /// The curve the sweep describes.
    pub f_curve: SkDCurve,
    /// The two hull edges bounding the curve's direction of travel, each a
    /// vector from the start point towards a later control point. Together
    /// they span the wedge the curve stays inside.
    pub f_sweep: [SkDVector; 2],
    /// False when the control points are collinear with the endpoints, so the
    /// segment can be treated as a line rather than a curve.
    f_is_curve: bool,
    /// Cleared when a cubic's third vector falls outside the wedge the first
    /// two span, so the sweep vectors no longer run in the curve's own order.
    f_ordered: bool,
}

impl SkDCurveSweep {
    /// Empty sweep: a degenerate line, zero sweep vectors, not a curve, and
    /// ordered. Assign `f_curve`, then call
    /// [`set_curve_hull_sweep`](Self::set_curve_hull_sweep).
    #[must_use]
    pub fn new() -> Self {
        Self {
            f_curve: SkDCurve::default(),
            f_sweep: [SkDVector::default(); 2],
            f_is_curve: false,
            f_ordered: true,
        }
    }

    /// Whether the segment actually bends, as decided by the last
    /// [`set_curve_hull_sweep`](Self::set_curve_hull_sweep) call.
    #[must_use]
    pub fn is_curve(&self) -> bool {
        self.f_is_curve
    }

    /// Whether the sweep vectors are still in the curve's own order.
    #[must_use]
    pub fn is_ordered(&self) -> bool {
        self.f_ordered
    }

    /// Computes the sweep vectors for `f_curve`, read as `verb`.
    ///
    /// Port of `SkDCurveSweep::setCurveHullSweep`. The first vector runs from
    /// the start point to the first control point and the second to the next
    /// one along. A cubic may shift or swap them so the pair brackets the
    /// curve, clearing `f_ordered` when the swap happens.
    pub fn set_curve_hull_sweep(&mut self, verb: Verb) {
        self.f_ordered = true;
        self.f_sweep[0] = self.f_curve.point(1) - self.f_curve.point(0);

        if verb == Verb::Line {
            self.f_sweep[1] = self.f_sweep[0];
            self.f_is_curve = false;
            return;
        }

        self.f_sweep[1] = self.f_curve.point(2) - self.f_curve.point(0);

        // The largest coordinate present, so "small enough to be zero" is
        // judged against this curve's own scale.
        let mut max_val: f64 = 0.0;
        for index in 0..self.f_curve.point_count() {
            let pt = self.f_curve.point(index);
            max_val = max_val.max(pt.f_x.abs()).max(pt.f_y.abs());
        }

        if verb != Verb::Cubic {
            if roughly_zero_when_compared_to(self.f_sweep[0].f_x, max_val)
                && roughly_zero_when_compared_to(self.f_sweep[0].f_y, max_val)
            {
                self.f_sweep[0] = self.f_sweep[1];
            }
            self.set_is_curve();
            return;
        }

        let third_sweep = self.f_curve.point(3) - self.f_curve.point(0);

        if self.f_sweep[0].f_x == 0.0 && self.f_sweep[0].f_y == 0.0 {
            self.f_sweep[0] = self.f_sweep[1];
            self.f_sweep[1] = third_sweep;
            if roughly_zero_when_compared_to(self.f_sweep[0].f_x, max_val)
                && roughly_zero_when_compared_to(self.f_sweep[0].f_y, max_val)
            {
                self.f_sweep[0] = self.f_sweep[1];
                // Both leading vectors were degenerate, so the curve is
                // really the chord from its start to its end: pull the first
                // control point onto the end point to match.
                let end = self.f_curve.point(3);
                *self.f_curve.point_mut(1) = end;
            }
            self.set_is_curve();
            return;
        }

        let s1x3 = self.f_sweep[0].cross_check(third_sweep);
        let s3x2 = third_sweep.cross_check(self.f_sweep[1]);

        // The third vector already lies on or between the first two, so the
        // pair brackets the curve as it stands.
        if s1x3 * s3x2 >= 0.0 {
            self.set_is_curve();
            return;
        }

        let s2x1 = self.f_sweep[1].cross_check(self.f_sweep[0]);
        if s3x2 * s2x1 < 0.0 {
            self.f_sweep[0] = self.f_sweep[1];
            self.f_ordered = false;
        }
        self.f_sweep[1] = third_sweep;
        self.set_is_curve();
    }

    fn set_is_curve(&mut self) {
        self.f_is_curve = self.f_sweep[0].cross_check(self.f_sweep[1]) != 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(x0: f64, y0: f64, x1: f64, y1: f64) -> SkDLine {
        SkDLine::from_points(SkDPoint::new(x0, y0), SkDPoint::new(x1, y1))
    }

    #[test]
    fn verb_point_counts() {
        assert_eq!(Verb::Line.point_count(), 2);
        assert_eq!(Verb::Quad.point_count(), 3);
        assert_eq!(Verb::Conic.point_count(), 3);
        assert_eq!(Verb::Cubic.point_count(), 4);
    }

    #[test]
    fn curve_reports_the_verb_it_was_built_from() {
        assert_eq!(SkDCurve::Line(line(0.0, 0.0, 1.0, 1.0)).verb(), Verb::Line);
        assert_eq!(SkDCurve::Quad(SkDQuad::default()).verb(), Verb::Quad);
        assert_eq!(SkDCurve::Conic(SkDConic::default()).verb(), Verb::Conic);
        assert_eq!(SkDCurve::Cubic(SkDCubic::default()).verb(), Verb::Cubic);
    }

    #[test]
    fn curve_evaluates_through_the_wrapped_type() {
        let curve = SkDCurve::Line(line(0.0, 0.0, 1.0, 1.0));
        assert_eq!(curve.point_count(), 2);
        assert!((curve.pt_at_t(0.5).f_x - 0.5).abs() < 1e-12);

        let quad = SkDCurve::Quad(SkDQuad::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(1.0, 0.0),
            SkDPoint::new(1.0, 1.0),
        ]));
        // B(1/2) = (p0 + 2p1 + p2)/4, so y = (0 + 0 + 1)/4.
        assert!((quad.pt_at_t(0.5).f_y - 0.25).abs() < 1e-12);

        let cubic = SkDCurve::Cubic(SkDCubic::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(0.0, 1.0),
            SkDPoint::new(1.0, 1.0),
            SkDPoint::new(1.0, 0.0),
        ]));
        // B(1/2) = (p0 + 3p1 + 3p2 + p3)/8, so y = (0 + 3 + 3 + 0)/8.
        let pt = cubic.pt_at_t(0.5);
        assert!((pt.f_x - 0.5).abs() < 1e-12);
        assert!((pt.f_y - 0.75).abs() < 1e-12);
    }

    #[test]
    fn offset_moves_every_control_point() {
        let mut curve = SkDCurve::Cubic(SkDCubic::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(0.0, 1.0),
            SkDPoint::new(1.0, 1.0),
            SkDPoint::new(1.0, 0.0),
        ]));
        curve.offset(SkDVector::new(10.0, 20.0));
        assert_eq!(curve.point(0), SkDPoint::new(10.0, 20.0));
        assert_eq!(curve.point(1), SkDPoint::new(10.0, 21.0));
        assert_eq!(curve.point(2), SkDPoint::new(11.0, 21.0));
        assert_eq!(curve.point(3), SkDPoint::new(11.0, 20.0));
    }

    #[test]
    fn a_lines_tangent_is_its_own_direction() {
        // A line's derivative is constant, so every t reports the same
        // non-zero direction rather than zero.
        let curve = SkDCurve::Line(line(0.0, 0.0, 3.0, 4.0));
        for t in [0.0, 0.5, 1.0] {
            let d = curve.dxdy_at_t(t);
            assert!((d.f_x - 3.0).abs() < 1e-12);
            assert!((d.f_y - 4.0).abs() < 1e-12);
        }
    }

    #[test]
    fn sweep_of_a_line_is_not_a_curve() {
        let mut sweep = SkDCurveSweep::new();
        sweep.f_curve = SkDCurve::Line(line(0.0, 0.0, 10.0, 10.0));
        sweep.set_curve_hull_sweep(Verb::Line);

        assert!(!sweep.is_curve());
        assert!(sweep.is_ordered());
        // Both sweep vectors are the line's own direction.
        assert_eq!(sweep.f_sweep[0], sweep.f_sweep[1]);
    }

    #[test]
    fn sweep_of_a_bent_quad_is_a_curve() {
        let mut sweep = SkDCurveSweep::new();
        sweep.f_curve = SkDCurve::Quad(SkDQuad::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(1.0, 0.0),
            SkDPoint::new(1.0, 1.0),
        ]));
        sweep.set_curve_hull_sweep(Verb::Quad);

        assert!(sweep.is_curve());
        assert!(sweep.is_ordered());
    }

    #[test]
    fn sweep_of_a_collinear_quad_is_not_a_curve() {
        // Control point on the chord: the hull has no area, so the two sweep
        // vectors are parallel and the segment is a line in disguise.
        let mut sweep = SkDCurveSweep::new();
        sweep.f_curve = SkDCurve::Quad(SkDQuad::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(1.0, 1.0),
            SkDPoint::new(2.0, 2.0),
        ]));
        sweep.set_curve_hull_sweep(Verb::Quad);

        assert!(!sweep.is_curve());
    }

    #[test]
    fn a_quad_whose_first_vector_vanishes_falls_back_to_the_second() {
        // The control point sits on the start point, so sweep[0] is zero and
        // has to be replaced by the vector to the end point.
        let mut sweep = SkDCurveSweep::new();
        sweep.f_curve = SkDCurve::Quad(SkDQuad::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(10.0, 5.0),
        ]));
        sweep.set_curve_hull_sweep(Verb::Quad);

        assert_eq!(sweep.f_sweep[0], SkDVector::new(10.0, 5.0));
        assert_eq!(sweep.f_sweep[1], SkDVector::new(10.0, 5.0));
        assert!(!sweep.is_curve());
    }

    #[test]
    fn a_cubic_with_a_degenerate_first_vector_shifts_its_sweep_along() {
        // p1 == p0, so sweep[0] is zero: upstream shifts sweep[1] in and
        // takes the third vector as the new sweep[1].
        let mut sweep = SkDCurveSweep::new();
        sweep.f_curve = SkDCurve::Cubic(SkDCubic::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(10.0, 0.0),
            SkDPoint::new(10.0, 10.0),
        ]));
        sweep.set_curve_hull_sweep(Verb::Cubic);

        assert_eq!(sweep.f_sweep[0], SkDVector::new(10.0, 0.0));
        assert_eq!(sweep.f_sweep[1], SkDVector::new(10.0, 10.0));
        assert!(sweep.is_curve());
        assert!(sweep.is_ordered());
    }

    #[test]
    fn a_cubic_with_two_degenerate_vectors_collapses_to_its_chord() {
        // p1 and p2 both sit on p0, so neither leading vector says anything
        // about direction. Upstream falls back to the chord and pulls the
        // first control point onto the end point.
        let mut sweep = SkDCurveSweep::new();
        sweep.f_curve = SkDCurve::Cubic(SkDCubic::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(10.0, 10.0),
        ]));
        sweep.set_curve_hull_sweep(Verb::Cubic);

        assert_eq!(sweep.f_sweep[0], SkDVector::new(10.0, 10.0));
        assert_eq!(sweep.f_sweep[1], SkDVector::new(10.0, 10.0));
        assert_eq!(sweep.f_curve.point(1), SkDPoint::new(10.0, 10.0));
        assert!(!sweep.is_curve());
    }

    #[test]
    fn a_cubic_whose_third_vector_lies_between_the_first_two_keeps_them() {
        let mut sweep = SkDCurveSweep::new();
        sweep.f_curve = SkDCurve::Cubic(SkDCubic::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(10.0, 0.0),
            SkDPoint::new(10.0, 10.0),
            SkDPoint::new(5.0, 5.0),
        ]));
        sweep.set_curve_hull_sweep(Verb::Cubic);

        // The third vector is inside the wedge, so the original pair stands.
        assert_eq!(sweep.f_sweep[0], SkDVector::new(10.0, 0.0));
        assert_eq!(sweep.f_sweep[1], SkDVector::new(10.0, 10.0));
        assert!(sweep.is_ordered());
    }
}
