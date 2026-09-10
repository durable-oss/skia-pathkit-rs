//! Conic-line intersection computation.
//!
//! Port of Skia's `SkDConicLineIntersection.cpp`.
//!
//! A conic is a rational quadratic: its weight appears in both the numerator
//! and the denominator of the point at `t`, so the implicit equation solved by
//! [`LineConicIntersections::valid_t`] is *not* the quadratic one with a
//! substitution. The quad and cubic modules alongside this one look similar on
//! purpose, but the coefficient derivation here comes from the conic source
//! and must not be reconstructed by analogy from them.
//!
//! `add_circle`, `add_oval` and `add_round_rect` all emit conics, so without
//! this no intersection against circular geometry can be computed at all.

use super::sk_d_quad_line_intersection::DQuad;
use super::sk_intersections::SkIntersections;
use super::sk_path_ops_line::{pin_t, DLine};
use super::sk_path_ops_types::{
    approximately_equal, approximately_one_or_less, approximately_zero_or_more,
};
use crate::core::{Point, Scalar};

/// Maximum number of roots a conic-line intersection can produce.
const MAX_CONIC_ROOTS: usize = 2;

/// Number of control points in a conic.
const POINT_COUNT: usize = 3;

/// Index of a conic's last control point.
const POINT_LAST: usize = 2;

/// A conic curve: three control points and a weight.
///
/// Mirrors [`DQuad`] in the sibling module, with the weight added.
#[derive(Debug, Clone, Copy)]
pub struct DConic {
    /// The start point.
    pub p0: Point,
    /// The control point.
    pub p1: Point,
    /// The end point.
    pub p2: Point,
    /// The rational weight. A weight of 1 is exactly a quadratic.
    pub weight: Scalar,
}

impl DConic {
    /// Returns a conic through the three points with the given weight.
    #[must_use]
    pub fn new(p0: Point, p1: Point, p2: Point, weight: Scalar) -> Self {
        Self {
            p0,
            p1,
            p2,
            weight,
        }
    }

    /// Returns control point `index`.
    #[must_use]
    pub fn point(&self, index: usize) -> Point {
        match index {
            0 => self.p0,
            1 => self.p1,
            _ => self.p2,
        }
    }

    /// Evaluates the conic at `t`.
    ///
    /// The numerator and denominator are formed separately, as in
    /// `SkDConic::ptAtT`; collapsing them loses the weight.
    #[must_use]
    pub fn pt_at_t(&self, t: Scalar) -> Point {
        let one_minus_t = 1.0 - t;
        let one_minus_t2 = one_minus_t * one_minus_t;
        let t2 = t * t;
        let cross = 2.0 * one_minus_t * t * self.weight;
        let denom = one_minus_t2 + cross + t2;
        Point::new(
            (one_minus_t2 * self.p0.x + cross * self.p1.x + t2 * self.p2.x) / denom,
            (one_minus_t2 * self.p0.y + cross * self.p1.y + t2 * self.p2.y) / denom,
        )
    }

    /// Returns the t at which the conic passes nearest `xy`, or -1.
    ///
    /// Stands in for `SkDCurve::nearPoint` for the conic verb: samples the
    /// curve and refines around the closest sample. `opp` is the far end of
    /// the line, used to reject a point that is nearer that end instead.
    #[must_use]
    pub fn near_point(&self, xy: Point, opp: Point) -> Scalar {
        let mut best_t = -1.0;
        let mut best_dist = Scalar::MAX;
        const SAMPLES: usize = 64;
        for i in 0..=SAMPLES {
            let t = i as Scalar / SAMPLES as Scalar;
            let d = Point::distance(self.pt_at_t(t), xy);
            if d < best_dist {
                best_dist = d;
                best_t = t;
            }
        }
        if best_t < 0.0 {
            return -1.0;
        }
        // Refine with a few bisection passes around the best sample.
        let mut lo = (best_t - 1.0 / SAMPLES as Scalar).max(0.0);
        let mut hi = (best_t + 1.0 / SAMPLES as Scalar).min(1.0);
        for _ in 0..24 {
            let mid_lo = lo + (hi - lo) / 3.0;
            let mid_hi = hi - (hi - lo) / 3.0;
            if Point::distance(self.pt_at_t(mid_lo), xy)
                < Point::distance(self.pt_at_t(mid_hi), xy)
            {
                hi = mid_hi;
            } else {
                lo = mid_lo;
            }
        }
        let t = (lo + hi) / 2.0;
        let pt = self.pt_at_t(t);
        let dist = Point::distance(pt, xy);
        // The point has to actually sit on the curve, and be nearer to this
        // end of the line than to the other.
        let scale = Point::distance(self.p0, self.p2).max(1.0);
        if dist > scale * 1e-4 || dist >= Point::distance(pt, opp) {
            return -1.0;
        }
        t
    }
}

/// Whether the caller has already computed the intersection point.
///
/// Port of `LineConicIntersections::PinTPoint`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PinTPoint {
    /// `pin_ts` must compute the point itself.
    Uninitialized,
    /// The point is already set and should be kept where possible.
    Initialized,
}

/// Finds where a conic meets a line.
///
/// Port of `LineConicIntersections`.
pub struct LineConicIntersections<'a> {
    conic: DConic,
    line: DLine,
    intersections: &'a mut SkIntersections,
    allow_near: bool,
}

impl<'a> LineConicIntersections<'a> {
    /// Returns an intersector for `conic` against `line`.
    pub fn new(conic: DConic, line: DLine, intersections: &'a mut SkIntersections) -> Self {
        // Four, to allow a short partial coincidence plus a discrete crossing.
        intersections.set_max(4);
        Self {
            conic,
            line,
            intersections,
            allow_near: true,
        }
    }

    /// Sets whether near-endpoint matches are accepted.
    pub fn allow_near(&mut self, allow: bool) {
        self.allow_near = allow;
    }

    /// Solves the conic's implicit equation against `axis_intercept`.
    ///
    /// Port of `LineConicIntersections::validT`. `r` holds one coordinate of
    /// the three control points, or the rotated residuals from
    /// [`Self::intersect_ray`].
    ///
    /// The weight enters through `B` only: this is the rational form, and
    /// substituting into the plain quadratic gives different coefficients.
    fn valid_t(
        &self,
        r: &[Scalar; 3],
        axis_intercept: Scalar,
        roots: &mut [Scalar; MAX_CONIC_ROOTS],
    ) -> usize {
        let w = self.conic.weight;
        let mut a = r[2];
        let mut b = r[1] * w - axis_intercept * w + axis_intercept;
        let mut c = r[0];
        a += c - 2.0 * b; // A = a + c - 2*(b*w - xCept*w + xCept)
        b -= c; // B = b*w - w*xCept + xCept - a
        c -= axis_intercept;
        DQuad::roots_valid_t(a, 2.0 * b, c, roots)
    }

    /// Returns the conic t values where it crosses the horizontal `y`.
    fn horizontal_roots(
        &self,
        axis_intercept: Scalar,
        roots: &mut [Scalar; MAX_CONIC_ROOTS],
    ) -> usize {
        let vals = [self.conic.p0.y, self.conic.p1.y, self.conic.p2.y];
        self.valid_t(&vals, axis_intercept, roots)
    }

    /// Returns the conic t values where it crosses the vertical `x`.
    fn vertical_roots(
        &self,
        axis_intercept: Scalar,
        roots: &mut [Scalar; MAX_CONIC_ROOTS],
    ) -> usize {
        let vals = [self.conic.p0.x, self.conic.p1.x, self.conic.p2.x];
        self.valid_t(&vals, axis_intercept, roots)
    }

    /// Intersects the conic with the horizontal span `left`..`right` at `y`.
    ///
    /// Port of `horizontalIntersect`.
    pub fn horizontal_intersect(
        &mut self,
        axis_intercept: Scalar,
        left: Scalar,
        right: Scalar,
        flipped: bool,
    ) -> usize {
        self.add_exact_horizontal_end_points(left, right, axis_intercept);
        if self.allow_near {
            self.add_near_horizontal_end_points(left, right, axis_intercept);
        }
        let mut roots = [0.0; MAX_CONIC_ROOTS];
        let count = self.horizontal_roots(axis_intercept, &mut roots);
        for &root in roots.iter().take(count) {
            let mut conic_t = root;
            let mut pt = self.conic.pt_at_t(conic_t);
            let mut line_t = (pt.x - left) / (right - left);
            if self.pin_ts(&mut conic_t, &mut line_t, &mut pt, PinTPoint::Initialized)
                && self.unique_answer(conic_t, pt)
            {
                let _ = self.intersections.insert(conic_t, line_t, pt);
            }
        }
        if flipped {
            self.intersections.flip();
        }
        self.check_coincident();
        self.intersections.used()
    }

    /// Intersects the conic with the vertical span `top`..`bottom` at `x`.
    ///
    /// Port of `verticalIntersect`.
    pub fn vertical_intersect(
        &mut self,
        axis_intercept: Scalar,
        top: Scalar,
        bottom: Scalar,
        flipped: bool,
    ) -> usize {
        self.add_exact_vertical_end_points(top, bottom, axis_intercept);
        if self.allow_near {
            self.add_near_vertical_end_points(top, bottom, axis_intercept);
        }
        let mut roots = [0.0; MAX_CONIC_ROOTS];
        let count = self.vertical_roots(axis_intercept, &mut roots);
        for &root in roots.iter().take(count) {
            let mut conic_t = root;
            let mut pt = self.conic.pt_at_t(conic_t);
            let mut line_t = (pt.y - top) / (bottom - top);
            if self.pin_ts(&mut conic_t, &mut line_t, &mut pt, PinTPoint::Initialized)
                && self.unique_answer(conic_t, pt)
            {
                let _ = self.intersections.insert(conic_t, line_t, pt);
            }
        }
        if flipped {
            self.intersections.flip();
        }
        self.check_coincident();
        self.intersections.used()
    }

    /// Returns the conic t values where it crosses the line treated as a ray.
    ///
    /// Port of `intersectRay`. The line is rotated to the x axis and the
    /// residuals are fed through [`Self::valid_t`] with a zero intercept.
    pub fn intersect_ray(&self, roots: &mut [Scalar; MAX_CONIC_ROOTS]) -> usize {
        let adj = self.line.p[1].x - self.line.p[0].x;
        let opp = self.line.p[1].y - self.line.p[0].y;
        let mut r = [0.0; 3];
        for (n, slot) in r.iter_mut().enumerate() {
            let p = self.conic.point(n);
            *slot = (p.y - self.line.p[0].y) * adj - (p.x - self.line.p[0].x) * opp;
        }
        self.valid_t(&r, 0.0, roots)
    }

    /// Intersects the conic with the line segment.
    ///
    /// Port of `intersect`.
    pub fn intersect(&mut self) -> usize {
        self.add_exact_end_points();
        if self.allow_near {
            self.add_near_end_points();
        }
        let mut root_vals = [0.0; MAX_CONIC_ROOTS];
        let roots = self.intersect_ray(&mut root_vals);
        for &root in root_vals.iter().take(roots) {
            let mut conic_t = root;
            let mut line_t = self.find_line_t(conic_t);
            let mut pt = Point::default();
            if self.pin_ts(&mut conic_t, &mut line_t, &mut pt, PinTPoint::Uninitialized)
                && self.unique_answer(conic_t, pt)
            {
                let _ = self.intersections.insert(conic_t, line_t, pt);
            }
        }
        self.check_coincident();
        self.intersections.used()
    }

    /// Records conic endpoints that lie exactly on the line.
    ///
    /// Port of `addExactEndPoints`. Endpoints go in first so that t values of
    /// exactly 0 and 1 are recorded rather than approximated by the solver.
    fn add_exact_end_points(&mut self) {
        let mut c_index = 0;
        while c_index < POINT_COUNT {
            let pt = self.conic.point(c_index);
            let line_t = self.line.exact_point(pt);
            if line_t >= 0.0 {
                let conic_t = (c_index >> 1) as Scalar;
                let _ = self.intersections.insert(conic_t, line_t, pt);
            }
            c_index += POINT_LAST;
        }
    }

    /// Records conic endpoints that lie near the line.
    ///
    /// Port of `addNearEndPoints`.
    fn add_near_end_points(&mut self) {
        let mut c_index = 0;
        while c_index < POINT_COUNT {
            let conic_t = (c_index >> 1) as Scalar;
            if self.intersections.has_t(conic_t) {
                c_index += POINT_LAST;
                continue;
            }
            let pt = self.conic.point(c_index);
            let line_t = self.line.near_point(pt, None);
            if line_t >= 0.0 {
                let _ = self.intersections.insert(conic_t, line_t, pt);
            }
            c_index += POINT_LAST;
        }
        self.add_line_near_end_points();
    }

    /// Records line endpoints that lie near the conic.
    ///
    /// Port of `addLineNearEndPoints`.
    fn add_line_near_end_points(&mut self) {
        for l_index in 0..2 {
            let line_t = l_index as Scalar;
            if self.intersections.has_opposite_t(line_t) {
                continue;
            }
            let conic_t = self
                .conic
                .near_point(self.line.p[l_index], self.line.p[1 - l_index]);
            if conic_t < 0.0 {
                continue;
            }
            let _ = self
                .intersections
                .insert(conic_t, line_t, self.line.p[l_index]);
        }
    }

    /// Port of `addExactHorizontalEndPoints`.
    fn add_exact_horizontal_end_points(&mut self, left: Scalar, right: Scalar, y: Scalar) {
        let mut c_index = 0;
        while c_index < POINT_COUNT {
            let pt = self.conic.point(c_index);
            let line_t = DLine::exact_point_h(pt, left, right, y);
            if line_t >= 0.0 {
                let _ = self.intersections.insert((c_index >> 1) as Scalar, line_t, pt);
            }
            c_index += POINT_LAST;
        }
    }

    /// Port of `addNearHorizontalEndPoints`.
    fn add_near_horizontal_end_points(&mut self, left: Scalar, right: Scalar, y: Scalar) {
        let mut c_index = 0;
        while c_index < POINT_COUNT {
            let conic_t = (c_index >> 1) as Scalar;
            if self.intersections.has_t(conic_t) {
                c_index += POINT_LAST;
                continue;
            }
            let pt = self.conic.point(c_index);
            let line_t = DLine::near_point_h(pt, left, right, y);
            if line_t >= 0.0 {
                let _ = self.intersections.insert(conic_t, line_t, pt);
            }
            c_index += POINT_LAST;
        }
        self.add_line_near_end_points();
    }

    /// Port of `addExactVerticalEndPoints`.
    fn add_exact_vertical_end_points(&mut self, top: Scalar, bottom: Scalar, x: Scalar) {
        let mut c_index = 0;
        while c_index < POINT_COUNT {
            let pt = self.conic.point(c_index);
            let line_t = DLine::exact_point_v(pt, top, bottom, x);
            if line_t >= 0.0 {
                let _ = self.intersections.insert((c_index >> 1) as Scalar, line_t, pt);
            }
            c_index += POINT_LAST;
        }
    }

    /// Port of `addNearVerticalEndPoints`.
    fn add_near_vertical_end_points(&mut self, top: Scalar, bottom: Scalar, x: Scalar) {
        let mut c_index = 0;
        while c_index < POINT_COUNT {
            let conic_t = (c_index >> 1) as Scalar;
            if self.intersections.has_t(conic_t) {
                c_index += POINT_LAST;
                continue;
            }
            let pt = self.conic.point(c_index);
            let line_t = DLine::near_point_v(pt, top, bottom, x);
            if line_t >= 0.0 {
                let _ = self.intersections.insert(conic_t, line_t, pt);
            }
            c_index += POINT_LAST;
        }
        self.add_line_near_end_points();
    }

    /// Returns the line t matching conic t, projecting on the longer axis.
    ///
    /// Port of `findLineT`.
    fn find_line_t(&self, t: Scalar) -> Scalar {
        let xy = self.conic.pt_at_t(t);
        let dx = self.line.p[1].x - self.line.p[0].x;
        let dy = self.line.p[1].y - self.line.p[0].y;
        if dx.abs() > dy.abs() {
            (xy.x - self.line.p[0].x) / dx
        } else {
            (xy.y - self.line.p[0].y) / dy
        }
    }

    /// Clamps both t values into range and snaps the point to an endpoint.
    ///
    /// Port of `pinTs`. Returns false when the line t falls outside the
    /// segment, or when the result duplicates one already recorded.
    fn pin_ts(
        &self,
        conic_t: &mut Scalar,
        line_t: &mut Scalar,
        pt: &mut Point,
        pt_set: PinTPoint,
    ) -> bool {
        if !approximately_one_or_less(f64::from(*line_t)) {
            return false;
        }
        if !approximately_zero_or_more(f64::from(*line_t)) {
            return false;
        }
        let q_t = pin_t(*conic_t);
        *conic_t = q_t;
        let l_t = pin_t(*line_t);
        *line_t = l_t;
        if l_t == 0.0
            || l_t == 1.0
            || (pt_set == PinTPoint::Uninitialized && q_t != 0.0 && q_t != 1.0)
        {
            *pt = self.line.pt_at_t(l_t);
        } else if pt_set == PinTPoint::Uninitialized {
            *pt = self.conic.pt_at_t(q_t);
        }
        // Snap to a line endpoint when the grid point lands on one, so the
        // shared vertex between adjacent segments reads as exactly that.
        if points_approximately_equal(*pt, self.line.p[0]) {
            *pt = self.line.p[0];
            *line_t = 0.0;
        } else if points_approximately_equal(*pt, self.line.p[1]) {
            *pt = self.line.p[1];
            *line_t = 1.0;
        }
        if self.intersections.used() > 0
            && approximately_equal(f64::from(self.intersections.t(1, 0)), f64::from(*line_t))
        {
            return false;
        }
        if *pt == self.conic.p0 {
            *pt = self.conic.p0;
            *conic_t = 0.0;
        } else if *pt == self.conic.p2 {
            *pt = self.conic.p2;
            *conic_t = 1.0;
        }
        true
    }

    /// Returns false when this crossing repeats one already recorded.
    ///
    /// Port of `uniqueAnswer`. Two t values that map to the same point *and*
    /// whose midpoint also maps there describe one tangential touch, not two
    /// crossings.
    fn unique_answer(&self, conic_t: Scalar, pt: Point) -> bool {
        for inner in 0..self.intersections.used() {
            if self.intersections.pt(inner) != pt {
                continue;
            }
            let existing = self.intersections.t(0, inner);
            if conic_t == existing {
                return false;
            }
            let mid = self.conic.pt_at_t((existing + conic_t) / 2.0);
            if points_approximately_equal(mid, pt) {
                return false;
            }
        }
        true
    }

    /// Marks runs where the conic lies along the line as coincident.
    ///
    /// Port of `checkCoincident`. `last` must shrink with each removal or the
    /// walk does not terminate.
    fn check_coincident(&mut self) {
        let mut last = self.intersections.used() as isize - 1;
        let mut index: isize = 0;
        while index < last {
            let i = index as usize;
            let conic_mid_t = (self.intersections.t(0, i) + self.intersections.t(0, i + 1)) / 2.0;
            let conic_mid_pt = self.conic.pt_at_t(conic_mid_t);
            let t = self.line.near_point(conic_mid_pt, None);
            if t < 0.0 {
                index += 1;
                continue;
            }
            if self.intersections.is_coincident(i) {
                self.intersections.remove_one(i);
                last -= 1;
            } else if self.intersections.is_coincident(i + 1) {
                self.intersections.remove_one(i + 1);
                last -= 1;
            } else {
                self.intersections.set_coincident(i);
                index += 1;
            }
            self.intersections.set_coincident(index as usize);
        }
    }
}

/// Returns true when two points are equal to within the grid tolerance.
fn points_approximately_equal(a: Point, b: Point) -> bool {
    approximately_equal(f64::from(a.x), f64::from(b.x))
        && approximately_equal(f64::from(a.y), f64::from(b.y))
}

/// Intersects `conic` with the horizontal span `left`..`right` at `y`.
///
/// Port of `SkIntersections::horizontal(const SkDConic&, ...)`.
pub fn horizontal(
    intersections: &mut SkIntersections,
    conic: &DConic,
    left: Scalar,
    right: Scalar,
    y: Scalar,
    flipped: bool,
) -> usize {
    let line = DLine::new(Point::new(left, y), Point::new(right, y));
    let mut c = LineConicIntersections::new(*conic, line, intersections);
    c.horizontal_intersect(y, left, right, flipped)
}

/// Intersects `conic` with the vertical span `top`..`bottom` at `x`.
///
/// Port of `SkIntersections::vertical(const SkDConic&, ...)`.
pub fn vertical(
    intersections: &mut SkIntersections,
    conic: &DConic,
    top: Scalar,
    bottom: Scalar,
    x: Scalar,
    flipped: bool,
) -> usize {
    let line = DLine::new(Point::new(x, top), Point::new(x, bottom));
    let mut c = LineConicIntersections::new(*conic, line, intersections);
    c.vertical_intersect(x, top, bottom, flipped)
}

/// Intersects `conic` with `line`.
///
/// Port of `SkIntersections::intersect(const SkDConic&, const SkDLine&)`.
pub fn intersect(
    intersections: &mut SkIntersections,
    conic: &DConic,
    line: &DLine,
    allow_near: bool,
) -> usize {
    let mut c = LineConicIntersections::new(*conic, *line, intersections);
    c.allow_near(allow_near);
    c.intersect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The weight a quarter-circle arc uses.
    const CIRCLE_W: Scalar = std::f32::consts::FRAC_1_SQRT_2;

    /// A quarter arc of the unit-ish circle centred at (0, 100), radius 100,
    /// running from (0, 0) up to (100, 100) through the corner at (0, 100)...
    /// expressed the usual way: start (100,0), control (100,100), end (0,100)
    /// is the quarter centred on the origin.
    fn quarter_arc() -> DConic {
        DConic::new(
            Point::new(100.0, 0.0),
            Point::new(100.0, 100.0),
            Point::new(0.0, 100.0),
            CIRCLE_W,
        )
    }

    #[test]
    fn a_weighted_conic_is_not_the_quadratic_through_the_same_points() {
        // If this ever stops holding, every test below is testing nothing.
        let arc = quarter_arc();
        let as_quad = DConic::new(arc.p0, arc.p1, arc.p2, 1.0);
        let a = arc.pt_at_t(0.5);
        let b = as_quad.pt_at_t(0.5);
        assert!(Point::distance(a, b) > 1.0);
        // The arc's midpoint sits on the circle of radius 100.
        let r = (a.x * a.x + a.y * a.y).sqrt();
        assert!((r - 100.0).abs() < 0.01, "arc midpoint radius {r}");
    }

    #[test]
    fn a_line_through_the_arc_crosses_it_once() {
        // The quarter arc spans one quadrant, so a line through the origin
        // region meets it a single time.
        let arc = quarter_arc();
        let line = DLine::new(Point::new(0.0, 0.0), Point::new(200.0, 200.0));
        let mut ts = SkIntersections::new();
        let count = intersect(&mut ts, &arc, &line, true);
        assert_eq!(count, 1);
        // The crossing is at 45 degrees, on the circle.
        let pt = ts.pt(0);
        let r = (pt.x * pt.x + pt.y * pt.y).sqrt();
        assert!((r - 100.0).abs() < 0.5, "crossing radius {r}");
        assert!((pt.x - pt.y).abs() < 0.5, "crossing should be at 45 degrees");
    }

    #[test]
    fn a_line_clear_of_the_arc_misses_it() {
        let arc = quarter_arc();
        let line = DLine::new(Point::new(300.0, 0.0), Point::new(300.0, 200.0));
        let mut ts = SkIntersections::new();
        assert_eq!(intersect(&mut ts, &arc, &line, true), 0);
    }

    #[test]
    fn a_line_through_the_centre_of_a_half_circle_meets_it_twice() {
        // A half circle needs two conics in Skia, but a single conic bulging
        // past a chord is enough to be met twice by that chord's line.
        let bulge = DConic::new(
            Point::new(-100.0, 0.0),
            Point::new(0.0, 200.0),
            Point::new(100.0, 0.0),
            0.6,
        );
        let line = DLine::new(Point::new(-200.0, 50.0), Point::new(200.0, 50.0));
        let mut ts = SkIntersections::new();
        let count = intersect(&mut ts, &bulge, &line, true);
        assert_eq!(count, 2, "a chord below the apex cuts the curve twice");
        // Both crossings sit on the line.
        for i in 0..count {
            assert!((ts.pt(i).y - 50.0).abs() < 0.5);
        }
        // And they are on opposite sides of the apex.
        assert!(ts.pt(0).x * ts.pt(1).x < 0.0);
    }

    #[test]
    fn a_line_above_the_apex_misses_and_below_it_cuts_twice() {
        // The apex is a double root: at exactly that height the discriminant
        // is zero to within rounding, so which side of it a f32 line lands on
        // decides between 0 and 2. Test either side rather than the knife
        // edge, and check the crossings converge on the apex as the line
        // approaches it.
        let bulge = DConic::new(
            Point::new(-100.0, 0.0),
            Point::new(0.0, 200.0),
            Point::new(100.0, 0.0),
            0.5,
        );
        let apex = bulge.pt_at_t(0.5);

        let mut above = SkIntersections::new();
        let high = DLine::new(
            Point::new(-200.0, apex.y + 1.0),
            Point::new(200.0, apex.y + 1.0),
        );
        assert_eq!(intersect(&mut above, &bulge, &high, true), 0);

        let mut below = SkIntersections::new();
        let low = DLine::new(
            Point::new(-200.0, apex.y - 1.0),
            Point::new(200.0, apex.y - 1.0),
        );
        let count = intersect(&mut below, &bulge, &low, true);
        assert_eq!(count, 2, "a chord below the apex cuts twice");
        // The two crossings straddle the apex and sit close to it.
        assert!(below.pt(0).x * below.pt(1).x < 0.0);
        for i in 0..count {
            assert!((below.pt(i).x - apex.x).abs() < 30.0);
        }
    }

    #[test]
    fn an_endpoint_shared_with_the_line_registers_once() {
        // The line starts exactly where the arc ends.
        let arc = quarter_arc();
        let line = DLine::new(arc.p2, Point::new(-100.0, 100.0));
        let mut ts = SkIntersections::new();
        let count = intersect(&mut ts, &arc, &line, true);
        assert_eq!(count, 1);
        assert!((ts.t(0, 0) - 1.0).abs() < 1e-4, "end of the conic");
        assert!((ts.t(1, 0) - 0.0).abs() < 1e-4, "start of the line");
    }

    #[test]
    fn horizontal_fast_path_agrees_with_the_general_one() {
        let arc = quarter_arc();
        let y = 70.0;

        let mut fast = SkIntersections::new();
        let fast_count = horizontal(&mut fast, &arc, -200.0, 200.0, y, false);

        let mut general = SkIntersections::new();
        let line = DLine::new(Point::new(-200.0, y), Point::new(200.0, y));
        let general_count = intersect(&mut general, &arc, &line, true);

        assert_eq!(fast_count, general_count);
        assert!(fast_count > 0);
        for i in 0..fast_count {
            assert!(
                (fast.t(0, i) - general.t(0, i)).abs() < 1e-3,
                "conic t differs at {i}: {} vs {}",
                fast.t(0, i),
                general.t(0, i)
            );
            assert!((fast.pt(i).y - y).abs() < 0.5);
        }
    }

    #[test]
    fn vertical_fast_path_agrees_with_the_general_one() {
        let arc = quarter_arc();
        let x = 70.0;

        let mut fast = SkIntersections::new();
        let fast_count = vertical(&mut fast, &arc, -200.0, 200.0, x, false);

        let mut general = SkIntersections::new();
        let line = DLine::new(Point::new(x, -200.0), Point::new(x, 200.0));
        let general_count = intersect(&mut general, &arc, &line, true);

        assert_eq!(fast_count, general_count);
        assert!(fast_count > 0);
        for i in 0..fast_count {
            assert!(
                (fast.t(0, i) - general.t(0, i)).abs() < 1e-3,
                "conic t differs at {i}"
            );
            assert!((fast.pt(i).x - x).abs() < 0.5);
        }
    }

    #[test]
    fn horizontal_flipped_reverses_the_line_parameter() {
        let arc = quarter_arc();
        let y = 70.0;
        let mut plain = SkIntersections::new();
        let n = horizontal(&mut plain, &arc, -200.0, 200.0, y, false);
        assert!(n > 0);
        let straight: Vec<Scalar> = (0..n).map(|i| plain.t(1, i)).collect();

        let mut flipped = SkIntersections::new();
        assert_eq!(horizontal(&mut flipped, &arc, -200.0, 200.0, y, true), n);
        for (i, s) in straight.iter().enumerate() {
            assert!((flipped.t(1, i) - (1.0 - s)).abs() < 1e-4);
        }
    }

    #[test]
    fn valid_t_uses_the_weight() {
        // The same three points with two different weights must not give the
        // same roots; that is exactly the mistake of adapting the quad code.
        // Below both apexes (y = 57.1 at w = 0.4, y = 133.3 at w = 2.0), so
        // both weights genuinely cross the line.
        let line = DLine::new(Point::new(-200.0, 40.0), Point::new(200.0, 40.0));
        let pts = (
            Point::new(-100.0, 0.0),
            Point::new(0.0, 200.0),
            Point::new(100.0, 0.0),
        );

        let mut light = SkIntersections::new();
        let light_conic = DConic::new(pts.0, pts.1, pts.2, 0.4);
        let ln = intersect(&mut light, &light_conic, &line, true);

        let mut heavy = SkIntersections::new();
        let heavy_conic = DConic::new(pts.0, pts.1, pts.2, 2.0);
        let hn = intersect(&mut heavy, &heavy_conic, &line, true);

        assert!(ln > 0 && hn > 0);
        assert!(
            (light.t(0, 0) - heavy.t(0, 0)).abs() > 1e-3,
            "the weight must change where the line meets the curve"
        );
    }

    #[test]
    fn intersect_ray_reaches_past_the_segment() {
        let arc = quarter_arc();
        // A short segment nowhere near the arc, but its ray crosses.
        let line = DLine::new(Point::new(0.0, 0.0), Point::new(1.0, 1.0));
        let mut ts = SkIntersections::new();
        let c = LineConicIntersections::new(arc, line, &mut ts);
        let mut roots = [0.0; MAX_CONIC_ROOTS];
        let n = c.intersect_ray(&mut roots);
        assert!(n > 0, "the ray through the segment meets the arc");
    }

    #[test]
    fn pt_at_t_hits_both_endpoints() {
        let arc = quarter_arc();
        let start = arc.pt_at_t(0.0);
        let end = arc.pt_at_t(1.0);
        assert!(Point::distance(start, arc.p0) < 1e-4);
        assert!(Point::distance(end, arc.p2) < 1e-4);
    }
}
