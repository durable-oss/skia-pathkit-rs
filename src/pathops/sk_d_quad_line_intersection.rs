//! Quadratic-line intersection computation
//!
//! Port of Skia's SkDQuadLineIntersection.{h,cpp}
//!
//! This module implements intersection finding between a quadratic Bezier curve
//! and a line segment, using the mathematical approach of solving for valid
//! t values where the quadratic intersects the line.

use crate::core::{Point, Scalar};
use super::sk_intersections::SkIntersections;
use super::sk_path_ops_types::{almost_between_ulps, almost_equal_ulps_pin, between};

/// Maximum number of quadratic roots
const MAX_QUAD_ROOTS: usize = 2;

/// A quadratic curve represented by 3 control points
#[derive(Debug, Clone, Copy)]
pub struct DQuad {
    /// Start point, at t == 0.
    pub p0: Point,
    /// Control point.
    pub p1: Point,
    /// End point, at t == 1.
    pub p2: Point,
}

impl DQuad {
    /// Constructs a quadratic from its three control points, in order.
    pub fn new(p0: Point, p1: Point, p2: Point) -> Self {
        Self { p0, p1, p2 }
    }

    /// Evaluate quadratic at parameter t using Bernstein polynomials
    pub fn pt_at_t(&self, t: Scalar) -> Point {
        let one_minus_t = 1.0 - t;
        let one_minus_t2 = one_minus_t * one_minus_t;
        let t2 = t * t;

        Point {
            x: one_minus_t2 * self.p0.x
                + 2.0 * one_minus_t * t * self.p1.x
                + t2 * self.p2.x,
            y: one_minus_t2 * self.p0.y
                + 2.0 * one_minus_t * t * self.p1.y
                + t2 * self.p2.y,
        }
    }

    /// Get control point at index
    pub fn point(&self, index: usize) -> Point {
        match index {
            0 => self.p0,
            1 => self.p1,
            2 => self.p2,
            _ => self.p2,
        }
    }

    /// Find roots of the quadratic equation At^2 + Bt + C = 0 with valid t in `[0, 1]`
    pub fn roots_valid_t(a: Scalar, b: Scalar, c: Scalar, roots: &mut [Scalar; MAX_QUAD_ROOTS]) -> usize {
        let abs_a = a.abs();
        let abs_c = c.abs();

        // If quadratic coefficient is near zero, it's actually linear
        if abs_a < 1e-10 {
            if abs_c < 1e-10 {
                // c is zero, so t=0 is a root
                roots[0] = 0.0;
                1
            } else if b.abs() < 1e-10 {
                // both a and b are zero, no roots
                0
            } else {
                let t = -c / b;
                if (0.0..=1.0).contains(&t) {
                    roots[0] = t;
                    1
                } else {
                    0
                }
            }
        } else {
            let discriminant = b * b - 4.0 * a * c;
            if discriminant < 0.0 {
                0
            } else {
                let sqrt_disc = discriminant.sqrt();
                let t1 = (-b - sqrt_disc) / (2.0 * a);
                let t2 = (-b + sqrt_disc) / (2.0 * a);

                let mut count = 0;
                if (0.0..=1.0).contains(&t1) {
                    roots[count] = t1;
                    count += 1;
                }
                if (0.0..=1.0).contains(&t2) && (t1 - t2).abs() > 1e-10 {
                    roots[count] = t2;
                    count += 1;
                }
                count
            }
        }
    }

    /// Find roots for horizontal intersection at y = axis_intercept
    pub fn horizontal_intersect(&self, axis_intercept: Scalar, roots: &mut [Scalar; MAX_QUAD_ROOTS]) -> usize {
        // Using y-coordinates: D = f, E = e, F = d
        let d = self.p0.y;
        let e = self.p1.y;
        let f = self.p2.y;

        // Polynomial: (d - 2e + f)t^2 + 2(e - d)t + (d - axis_intercept) = 0
        let a = d - 2.0 * e + f;
        let b = 2.0 * (e - d);
        let c = d - axis_intercept;

        Self::roots_valid_t(a, b, c, roots)
    }

    /// Find roots for vertical intersection at x = axis_intercept
    pub fn vertical_intersect(&self, axis_intercept: Scalar, roots: &mut [Scalar; MAX_QUAD_ROOTS]) -> usize {
        // Using x-coordinates
        let d = self.p0.x;
        let e = self.p1.x;
        let f = self.p2.x;

        // Polynomial: (d - 2e + f)t^2 + 2(e - d)t + (d - axis_intercept) = 0
        let a = d - 2.0 * e + f;
        let b = 2.0 * (e - d);
        let c = d - axis_intercept;

        Self::roots_valid_t(a, b, c, roots)
    }
}

/// A line segment
#[derive(Debug, Clone, Copy)]
pub struct DLine {
    /// Start point, at t == 0.
    pub p0: Point,
    /// End point, at t == 1.
    pub p1: Point,
}

impl DLine {
    /// Constructs the segment running from `p0` to `p1`.
    pub fn new(p0: Point, p1: Point) -> Self {
        Self { p0, p1 }
    }

    /// Returns endpoint `index`. Any index other than 0 gives the end point.
    pub fn point(&self, index: usize) -> Point {
        match index {
            0 => self.p0,
            _ => self.p1,
        }
    }

    /// Evaluate line at parameter t
    pub fn pt_at_t(&self, t: Scalar) -> Point {
        Point {
            x: self.p0.x + t * (self.p1.x - self.p0.x),
            y: self.p0.y + t * (self.p1.y - self.p0.y),
        }
    }

    /// Returns 0 if `pt` is the line's first endpoint, 1 if it is the second,
    /// -1 otherwise.
    ///
    /// Port of `SkDLine::exactPoint` (`SkPathOpsLine.cpp:22`). This is an
    /// endpoint identity test, not a projection: it answers "is this point one
    /// of my two ends?", and the tolerance-based work belongs to
    /// [`near_point`](Self::near_point).
    pub fn exact_point(pt: Point, line: &DLine) -> Scalar {
        if pt == line.p0 {
            return 0.0;
        }
        if pt == line.p1 {
            return 1.0;
        }
        -1.0
    }

    /// Find t for point near line (returns -1.0 if outside segment)
    ///
    /// Port of `SkDLine::nearPoint`. Both coordinates have to land within the
    /// line's range before the perpendicular projection runs; projecting on
    /// the dominant axis alone would accept points well off the line.
    pub fn near_point(pt: Point, line: &DLine) -> Scalar {
        if !almost_between_ulps(line.p0.x, pt.x, line.p1.x)
            || !almost_between_ulps(line.p0.y, pt.y, line.p1.y)
        {
            return -1.0;
        }

        // Project a perpendicular ray from the point onto the line.
        let len_x = line.p1.x - line.p0.x;
        let len_y = line.p1.y - line.p0.y;
        let denom = len_x * len_x + len_y * len_y;
        let ab0_x = pt.x - line.p0.x;
        let ab0_y = pt.y - line.p0.y;
        let numer = len_x * ab0_x + len_y * ab0_y;

        if !between(0.0, numer, denom) {
            return -1.0;
        }
        if denom == 0.0 {
            return 0.0;
        }

        let t = numer / denom;
        let real_pt = line.pt_at_t(t);
        let dist = ((real_pt.x - pt.x).powi(2) + (real_pt.y - pt.y).powi(2)).sqrt();

        // Measure the distance against the largest magnitude in the line, so
        // the ULPS tolerance scales with the coordinates in play.
        let tiniest = line.p0.x.min(line.p0.y).min(line.p1.x).min(line.p1.y);
        let largest = line.p0.x.max(line.p0.y).max(line.p1.x).max(line.p1.y);
        let largest = largest.max(-tiniest);

        if !almost_equal_ulps_pin(largest, largest + dist) {
            return -1.0;
        }

        pin_t(t)
    }

    /// Find t for point on horizontal line y (returns -1.0 if not in range)
    pub fn exact_point_h(pt: Point, left: Scalar, right: Scalar, y: Scalar) -> Scalar {
        if approximately_equal(pt.y, y) {
            let t = (pt.x - left) / (right - left);
            if (0.0..=1.0).contains(&t) { t } else { -1.0 }
        } else {
            -1.0
        }
    }

    /// Find t for point near horizontal line y
    pub fn near_point_h(pt: Point, left: Scalar, right: Scalar, y: Scalar) -> Scalar {
        if approximately_equal(pt.y, y) {
            let t = (pt.x - left) / (right - left);
            if (0.0..=1.0).contains(&t) { t } else { -1.0 }
        } else {
            -1.0
        }
    }

    /// Find t for point on vertical line x (returns -1.0 if not in range)
    pub fn exact_point_v(pt: Point, top: Scalar, bottom: Scalar, x: Scalar) -> Scalar {
        if approximately_equal(pt.x, x) {
            let t = (pt.y - top) / (bottom - top);
            if (0.0..=1.0).contains(&t) { t } else { -1.0 }
        } else {
            -1.0
        }
    }

    /// Find t for point near vertical line x
    pub fn near_point_v(pt: Point, top: Scalar, bottom: Scalar, x: Scalar) -> Scalar {
        if approximately_equal(pt.x, x) {
            let t = (pt.y - top) / (bottom - top);
            if (0.0..=1.0).contains(&t) { t } else { -1.0 }
        } else {
            -1.0
        }
    }
}

/// Pin t value to [0,1] range
fn pin_t(t: Scalar) -> Scalar {
    if t < 0.0 {
        0.0
    } else if t > 1.0 {
        1.0
    } else {
        t
    }
}

/// Check if value is approximately 0 or more (including small negative due to floating point)
fn approximately_zero_or_more_double(t: Scalar) -> bool {
    t >= -1e-10
}

/// Check if value is approximately 1 or less
fn approximately_one_or_less_double(t: Scalar) -> bool {
    t <= 1.0 + 1e-10
}

/// Check if two values are approximately equal
fn approximately_equal(a: Scalar, b: Scalar) -> bool {
    (a - b).abs() < 1e-10
}

/// Line-quadratic intersection handler
struct LineQuadraticIntersections<'a> {
    quad: &'a DQuad,
    line: &'a DLine,
    intersections: &'a mut SkIntersections,
    allow_near: bool,
}

impl<'a> LineQuadraticIntersections<'a> {
    fn new(quad: &'a DQuad, line: &'a DLine, intersections: &'a mut SkIntersections) -> Self {
        intersections.set_max(5); // allow short partial coincidence plus discrete intersections
        Self {
            quad,
            line,
            intersections,
            allow_near: true,
        }
    }

    fn add_exact_end_points(&mut self) {
        // Add endpoints at quad t=0 and quad t=1
        for q_index in [0, 2] {
            let line_t = DLine::exact_point(self.quad.point(q_index), self.line);
            if line_t >= 0.0 {
                let quad_t = (q_index / 2) as Scalar;
                let _ = self.intersections.insert(quad_t, line_t, self.quad.point(q_index));
            }
        }
    }

    fn add_near_end_points(&mut self) {
        // Check quad endpoints
        for q_index in [0, 2] {
            let quad_t = (q_index / 2) as Scalar;
            if self.intersections.has_t(quad_t) {
                continue;
            }
            let line_t = DLine::near_point(self.quad.point(q_index), self.line);
            if line_t >= 0.0 {
                let _ = self.intersections.insert(quad_t, line_t, self.quad.point(q_index));
            }
        }

        // Check line endpoints
        for l_index in [0, 1] {
            let line_t = l_index as Scalar;
            if self.intersections.has_opposite_t(line_t) {
                continue;
            }
            let quad_t =
                self.quad_near_point(self.line.point(l_index), self.line.point(1 - l_index));
            if quad_t >= 0.0 {
                let _ = self.intersections.insert(quad_t, line_t, self.line.point(l_index));
            }
        }
    }

    /// Returns the quad t nearest to `pt`, or -1 if the quad does not come
    /// within ULPS tolerance of it.
    ///
    /// Port of `SkDCurve::nearPoint` (`SkPathOpsCurve.cpp:14`) for the quad
    /// verb. `opp` is the line's *other* endpoint; it only orients the
    /// perpendicular ray that is cast through `pt` to find where the curve
    /// crosses it.
    fn quad_near_point(&self, pt: Point, opp: Point) -> Scalar {
        // Reject early if the point is outside the control hull's box; the
        // curve is contained by it, so nothing inside can be near.
        let min_x = self.quad.p0.x.min(self.quad.p1.x).min(self.quad.p2.x);
        let max_x = self.quad.p0.x.max(self.quad.p1.x).max(self.quad.p2.x);
        if !almost_between_ulps(min_x, pt.x, max_x) {
            return -1.0;
        }
        let min_y = self.quad.p0.y.min(self.quad.p1.y).min(self.quad.p2.y);
        let max_y = self.quad.p0.y.max(self.quad.p1.y).max(self.quad.p2.y);
        if !almost_between_ulps(min_y, pt.y, max_y) {
            return -1.0;
        }

        // Cast a ray through `pt` perpendicular to the line, and take the
        // closest place the quad crosses it.
        let perp = DLine::new(
            pt,
            Point::new(pt.x + opp.y - pt.y, pt.y + pt.x - opp.x),
        );
        let mut roots = [0.0; MAX_QUAD_ROOTS];
        let mut scratch = SkIntersections::new();
        let count = LineQuadraticIntersections::new(self.quad, &perp, &mut scratch)
            .intersect_ray(&mut roots);

        let mut min_dist = Scalar::MAX;
        let mut min_t = -1.0;
        for &root in roots.iter().take(count) {
            let on_curve = self.quad.pt_at_t(root);
            let dist = ((on_curve.x - pt.x).powi(2) + (on_curve.y - pt.y).powi(2)).sqrt();
            if dist < min_dist {
                min_dist = dist;
                min_t = root;
            }
        }
        if min_t < 0.0 {
            return -1.0;
        }

        let largest = max_x.max(max_y).max(-min_x.min(min_y));
        if !almost_equal_ulps_pin(largest, largest + min_dist) {
            return -1.0;
        }
        pin_t(min_t)
    }

    fn find_line_t(&self, t: Scalar) -> Scalar {
        let xy = self.quad.pt_at_t(t);
        let dx = self.line.p1.x - self.line.p0.x;
        let dy = self.line.p1.y - self.line.p0.y;

        if dx.abs() > dy.abs() {
            (xy.x - self.line.p0.x) / dx
        } else {
            (xy.y - self.line.p0.y) / dy
        }
    }

    fn pin_ts(&self, quad_t: &mut Scalar, line_t: &mut Scalar, pt: &mut Point, pt_set: PinTPoint) -> bool {
        if !approximately_one_or_less_double(*line_t) || !approximately_zero_or_more_double(*line_t) {
            return false;
        }

        *quad_t = pin_t(*quad_t);
        *line_t = pin_t(*line_t);

        if *line_t == 0.0 || *line_t == 1.0 || (pt_set == PinTPoint::Uninitialized && *quad_t != 0.0 && *quad_t != 1.0) {
            *pt = self.line.pt_at_t(*line_t);
        } else if pt_set == PinTPoint::Uninitialized {
            *pt = self.quad.pt_at_t(*quad_t);
        }

        // Snap to endpoints if approximately equal
        if approximately_equal(pt.x, self.line.p0.x) && approximately_equal(pt.y, self.line.p0.y) {
            *pt = self.line.p0;
            *line_t = 0.0;
        } else if approximately_equal(pt.x, self.line.p1.x) && approximately_equal(pt.y, self.line.p1.y) {
            *pt = self.line.p1;
            *line_t = 1.0;
        }

        // Check for duplicate
        if self.intersections.used() > 0 && approximately_equal(self.intersections.t(1, 0), *line_t) {
            return false;
        }

        if approximately_equal(pt.x, self.quad.p0.x) && approximately_equal(pt.y, self.quad.p0.y) {
            *pt = self.quad.p0;
            *quad_t = 0.0;
        } else if approximately_equal(pt.x, self.quad.p2.x) && approximately_equal(pt.y, self.quad.p2.y) {
            *pt = self.quad.p2;
            *quad_t = 1.0;
        }

        true
    }

    fn unique_answer(&self, quad_t: Scalar, pt: Point) -> bool {
        for inner in 0..self.intersections.used() {
            if self.intersections.pt(inner) != pt {
                continue;
            }
            let existing_quad_t = self.intersections.t(0, inner);
            if quad_t == existing_quad_t {
                return false;
            }
            // Check if midway on quad is also same point (degenerate case)
            let quad_mid_t = (existing_quad_t + quad_t) / 2.0;
            let quad_mid_pt = self.quad.pt_at_t(quad_mid_t);
            if quad_mid_pt.approximately_equal(pt) {
                return false;
            }
        }
        true
    }

    fn add_exact_horizontal_end_points(&mut self, left: Scalar, right: Scalar, y: Scalar) {
        for q_index in [0, 2] {
            let line_t = DLine::exact_point_h(self.quad.point(q_index), left, right, y);
            if line_t >= 0.0 {
                let quad_t = (q_index / 2) as Scalar;
                let _ = self.intersections.insert(quad_t, line_t, self.quad.point(q_index));
            }
        }
    }

    fn add_near_horizontal_end_points(&mut self, left: Scalar, right: Scalar, y: Scalar) {
        for q_index in [0, 2] {
            let quad_t = (q_index / 2) as Scalar;
            if self.intersections.has_t(quad_t) {
                continue;
            }
            let line_t = DLine::near_point_h(self.quad.point(q_index), left, right, y);
            if line_t >= 0.0 {
                let _ = self.intersections.insert(quad_t, line_t, self.quad.point(q_index));
            }
        }
        Self::add_line_near_end_points(self.line);
    }

    fn add_exact_vertical_end_points(&mut self, top: Scalar, bottom: Scalar, x: Scalar) {
        for q_index in [0, 2] {
            let line_t = DLine::exact_point_v(self.quad.point(q_index), top, bottom, x);
            if line_t >= 0.0 {
                let quad_t = (q_index / 2) as Scalar;
                let _ = self.intersections.insert(quad_t, line_t, self.quad.point(q_index));
            }
        }
    }

    fn add_near_vertical_end_points(&mut self, top: Scalar, bottom: Scalar, x: Scalar) {
        for q_index in [0, 2] {
            let quad_t = (q_index / 2) as Scalar;
            if self.intersections.has_t(quad_t) {
                continue;
            }
            let line_t = DLine::near_point_v(self.quad.point(q_index), top, bottom, x);
            if line_t >= 0.0 {
                let _ = self.intersections.insert(quad_t, line_t, self.quad.point(q_index));
            }
        }
        Self::add_line_near_end_points(self.line);
    }

    fn add_line_near_end_points(_line: &DLine) {
        // Placeholder for adding line endpoint proximity checks
    }

    fn check_coincident(&mut self) {
        // The loop reads index and index + 1, so it stops one short of the
        // end. `last` also has to shrink with every removal, or the walk
        // never terminates.
        let mut last = self.intersections.used() as isize - 1;
        let mut index: isize = 0;
        while index < last {
            let i = index as usize;
            let quad_mid_t =
                (self.intersections.t(0, i) + self.intersections.t(0, i + 1)) / 2.0;
            let quad_mid_pt = self.quad.pt_at_t(quad_mid_t);
            let t = DLine::near_point(quad_mid_pt, self.line);
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

    fn intersect_ray(&self, roots: &mut [Scalar; MAX_QUAD_ROOTS]) -> usize {
        // Rotate line and quad so line is horizontal
        let adj = self.line.p1.x - self.line.p0.x;
        let opp = self.line.p1.y - self.line.p0.y;
        let mut r = [0.0; 3];

        for n in 0..3 {
            r[n] = (self.quad.point(n).y - self.line.p0.y) * adj
                - (self.quad.point(n).x - self.line.p0.x) * opp;
        }

        let a = r[2] + r[0] - 2.0 * r[1]; // A = a - 2b + c
        let b = r[1] - r[0];              // B = -(b - c)
        let c = r[0];

        DQuad::roots_valid_t(a, 2.0 * b, c, roots)
    }
}

impl<'a> LineQuadraticIntersections<'a> {
    fn intersect(&mut self) -> usize {
        self.add_exact_end_points();
        if self.allow_near {
            self.add_near_end_points();
        }

        let mut root_vals = [0.0; MAX_QUAD_ROOTS];
        let roots = self.intersect_ray(&mut root_vals);

        for index in 0..roots {
            let mut quad_t = root_vals[index];
            let mut line_t = self.find_line_t(quad_t);
            let mut pt = Point::new(0.0, 0.0);

            if self.pin_ts(&mut quad_t, &mut line_t, &mut pt, PinTPoint::Uninitialized)
                && self.unique_answer(quad_t, pt)
            {
                let _ = self.intersections.insert(quad_t, line_t, pt);
            }
        }

        self.check_coincident();
        self.intersections.used()
    }

    fn horizontal_intersect(
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

        let mut root_vals = [0.0; MAX_QUAD_ROOTS];
        let roots = self.quad.horizontal_intersect(axis_intercept, &mut root_vals);

        for index in 0..roots {
            let quad_t = root_vals[index];
            let pt = self.quad.pt_at_t(quad_t);
            let line_t = (pt.x - left) / (right - left);
            let mut pt = pt;
            let mut line_t = line_t;
            let mut quad_t = quad_t;

            if self.pin_ts(&mut quad_t, &mut line_t, &mut pt, PinTPoint::Initialized)
                && self.unique_answer(quad_t, pt)
            {
                let line_t = if flipped { 1.0 - line_t } else { line_t };
                let _ = self.intersections.insert(quad_t, line_t, pt);
            }
        }

        if flipped {
            self.intersections.flip();
        }

        self.check_coincident();
        self.intersections.used()
    }

    fn vertical_intersect(
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

        let mut root_vals = [0.0; MAX_QUAD_ROOTS];
        let roots = self.quad.vertical_intersect(axis_intercept, &mut root_vals);

        for index in 0..roots {
            let quad_t = root_vals[index];
            let pt = self.quad.pt_at_t(quad_t);
            let line_t = (pt.y - top) / (bottom - top);
            let mut pt = pt;
            let mut line_t = line_t;
            let mut quad_t = quad_t;

            if self.pin_ts(&mut quad_t, &mut line_t, &mut pt, PinTPoint::Initialized)
                && self.unique_answer(quad_t, pt)
            {
                let line_t = if flipped { 1.0 - line_t } else { line_t };
                let _ = self.intersections.insert(quad_t, line_t, pt);
            }
        }

        if flipped {
            self.intersections.flip();
        }

        self.check_coincident();
        self.intersections.used()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum PinTPoint {
    Uninitialized,
    Initialized,
}

impl Point {
    fn approximately_equal(&self, other: Point) -> bool {
        approximately_equal(self.x, other.x) && approximately_equal(self.y, other.y)
    }
}


// Public API functions

/// Intersect a quadratic with a horizontal line
pub fn horizontal(quad: &DQuad, left: Scalar, right: Scalar, y: Scalar, flipped: bool) -> usize {
    let line = DLine::new(Point::new(left, y), Point::new(right, y));
    let mut intersections = SkIntersections::new();
    let mut q = LineQuadraticIntersections::new(quad, &line, &mut intersections);
    q.horizontal_intersect(y, left, right, flipped)
}

/// Intersect a quadratic with a vertical line
pub fn vertical(quad: &DQuad, top: Scalar, bottom: Scalar, x: Scalar, flipped: bool) -> usize {
    let line = DLine::new(Point::new(x, top), Point::new(x, bottom));
    let mut intersections = SkIntersections::new();
    let mut q = LineQuadraticIntersections::new(quad, &line, &mut intersections);
    q.vertical_intersect(x, top, bottom, flipped)
}

/// Intersect a quadratic with a line
pub fn intersect(quad: &DQuad, line: &DLine, allow_near: bool) -> usize {
    let mut intersections = SkIntersections::new();
    let mut q = LineQuadraticIntersections::new(quad, line, &mut intersections);
    q.allow_near = allow_near;
    q.intersect()
}

/// Find intersection of quadratic with line (ray intersection)
pub fn intersect_ray(quad: &DQuad, line: &DLine, roots: &mut [Scalar; MAX_QUAD_ROOTS]) -> usize {
    let mut intersections = SkIntersections::new();
    let q = LineQuadraticIntersections::new(quad, line, &mut intersections);
    q.intersect_ray(roots)
}

/// Find horizontal intercepts of quadratic at y = y_value
pub fn horizontal_intercept(quad: &DQuad, y_value: Scalar, roots: &mut [Scalar; MAX_QUAD_ROOTS]) -> usize {
    quad.horizontal_intersect(y_value, roots)
}

/// Find vertical intercepts of quadratic at x = x_value
pub fn vertical_intercept(quad: &DQuad, x_value: Scalar, roots: &mut [Scalar; MAX_QUAD_ROOTS]) -> usize {
    quad.vertical_intersect(x_value, roots)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quad_eval() {
        let quad = DQuad::new(
            Point::new(0.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(2.0, 0.0),
        );

        let pt0 = quad.pt_at_t(0.0);
        let pt1 = quad.pt_at_t(0.5);
        let pt2 = quad.pt_at_t(1.0);

        assert!((pt0.x - 0.0).abs() < 1e-10);
        assert!((pt0.y - 0.0).abs() < 1e-10);
        assert!((pt1.x - 1.0).abs() < 1e-10);
        // B(1/2) = p0/4 + p1/2 + p2/4, so y = 0/4 + 1/2 + 0/4.
        assert!((pt1.y - 0.5).abs() < 1e-6);
        assert!((pt2.x - 2.0).abs() < 1e-10);
        assert!((pt2.y - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_horizontal_intersect() {
        let quad = DQuad::new(
            Point::new(0.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(2.0, 0.0),
        );

        let mut roots = [0.0; MAX_QUAD_ROOTS];
        let count = quad.horizontal_intersect(0.5, &mut roots);

        assert!(count > 0, "Should find intersection with y=0.5");
        for i in 0..count {
            assert!((0.0..=1.0).contains(&roots[i]), "t in [0,1]");
        }
    }

    #[test]
    fn test_vertical_intersect() {
        let quad = DQuad::new(
            Point::new(0.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(2.0, 0.0),
        );

        let mut roots = [0.0; MAX_QUAD_ROOTS];
        let count = quad.vertical_intersect(1.0, &mut roots);

        assert!(count > 0, "Should find intersection with x=1.0");
        assert!((roots[0] - 0.5).abs() < 1e-6, "t should be 0.5");
    }

    #[test]
    fn test_quad_line_intersect() {
        let quad = DQuad::new(
            Point::new(0.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(2.0, 0.0),
        );

        let line = DLine::new(
            Point::new(0.0, 0.5),
            Point::new(2.0, 0.5),
        );

        let count = intersect(&quad, &line, true);
        assert!(count > 0, "Should find intersections");
    }

    /// The `lineQuadTests` table from Skia's
    /// `tests/PathOpsQuadLineIntersectionTest.cpp`, with the intersection
    /// count upstream expects for each row.
    #[test]
    fn upstream_line_quad_tests_report_the_expected_counts() {
        let cases: [([(Scalar, Scalar); 3], [(Scalar, Scalar); 2], usize); 5] = [
            ([(1.0, 1.0), (2.0, 1.0), (0.0, 2.0)], [(0.0, 0.0), (1.0, 1.0)], 1),
            ([(0.0, 0.0), (1.0, 1.0), (3.0, 1.0)], [(0.0, 0.0), (3.0, 1.0)], 2),
            ([(2.0, 0.0), (1.0, 1.0), (2.0, 2.0)], [(0.0, 0.0), (0.0, 2.0)], 0),
            ([(4.0, 0.0), (0.0, 1.0), (4.0, 2.0)], [(3.0, 1.0), (4.0, 1.0)], 0),
            ([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)], [(0.0, 1.0), (1.0, 0.0)], 1),
        ];

        for (index, (q, l, expected)) in cases.iter().enumerate() {
            let quad = DQuad::new(
                Point::new(q[0].0, q[0].1),
                Point::new(q[1].0, q[1].1),
                Point::new(q[2].0, q[2].1),
            );
            let line = DLine::new(
                Point::new(l[0].0, l[0].1),
                Point::new(l[1].0, l[1].1),
            );
            assert_eq!(
                intersect(&quad, &line, true),
                *expected,
                "lineQuadTests[{index}]"
            );
        }
    }

    #[test]
    fn every_reported_hit_has_agreeing_points() {
        // Upstream's own check on this table: for each intersection, the two
        // curves evaluated at their reported t must land on the same point.
        // A hit invented by a bad endpoint test fails this even when the
        // count happens to look right.
        let cases: [([(Scalar, Scalar); 3], [(Scalar, Scalar); 2]); 5] = [
            ([(1.0, 1.0), (2.0, 1.0), (0.0, 2.0)], [(0.0, 0.0), (1.0, 1.0)]),
            ([(0.0, 0.0), (1.0, 1.0), (3.0, 1.0)], [(0.0, 0.0), (3.0, 1.0)]),
            ([(2.0, 0.0), (1.0, 1.0), (2.0, 2.0)], [(0.0, 0.0), (0.0, 2.0)]),
            ([(4.0, 0.0), (0.0, 1.0), (4.0, 2.0)], [(3.0, 1.0), (4.0, 1.0)]),
            ([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)], [(0.0, 1.0), (1.0, 0.0)]),
        ];

        for (index, (q, l)) in cases.iter().enumerate() {
            let quad = DQuad::new(
                Point::new(q[0].0, q[0].1),
                Point::new(q[1].0, q[1].1),
                Point::new(q[2].0, q[2].1),
            );
            let line = DLine::new(
                Point::new(l[0].0, l[0].1),
                Point::new(l[1].0, l[1].1),
            );

            let mut intersections = SkIntersections::new();
            let used = {
                let mut solver =
                    LineQuadraticIntersections::new(&quad, &line, &mut intersections);
                solver.intersect()
            };

            for i in 0..used {
                let on_quad = quad.pt_at_t(intersections.t(0, i));
                let on_line = line.pt_at_t(intersections.t(1, i));
                assert!(
                    (on_quad.x - on_line.x).abs() < 1e-4
                        && (on_quad.y - on_line.y).abs() < 1e-4,
                    "lineQuadTests[{index}] hit {i}: quad {on_quad:?} vs line {on_line:?}"
                );
            }
        }
    }

    #[test]
    fn exact_point_is_an_endpoint_test_not_a_projection() {
        // A point sharing the line's x but nowhere near it must not report a
        // t. This is what made every disjoint quad/line pair claim two hits.
        let line = DLine::new(Point::new(0.0, 0.0), Point::new(0.0, 2.0));
        assert_eq!(DLine::exact_point(Point::new(0.0, 0.0), &line), 0.0);
        assert_eq!(DLine::exact_point(Point::new(0.0, 2.0), &line), 1.0);
        assert_eq!(DLine::exact_point(Point::new(0.0, 1.0), &line), -1.0);

        let diagonal = DLine::new(Point::new(0.0, 0.0), Point::new(2.0, 2.0));
        // x = 1 lands inside the line's x-range, but (1, 50) is far off it.
        assert_eq!(DLine::exact_point(Point::new(1.0, 50.0), &diagonal), -1.0);
    }

    #[test]
    fn near_point_checks_both_coordinates() {
        // The old projection tested one axis and returned, so a point level
        // with the line but far above it passed.
        let line = DLine::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0));
        assert!((DLine::near_point(Point::new(5.0, 0.0), &line) - 0.5).abs() < 1e-6);
        assert_eq!(DLine::near_point(Point::new(5.0, 40.0), &line), -1.0);
        assert_eq!(DLine::near_point(Point::new(-5.0, 0.0), &line), -1.0);
    }

    #[test]
    fn test_line_point_projection() {
        let line = DLine::new(
            Point::new(0.0, 0.0),
            Point::new(2.0, 0.0),
        );

        let pt = Point::new(1.0, 0.0);
        let t = DLine::near_point(pt, &line);

        assert!((t - 0.5).abs() < 1e-6, "t should be 0.5");
    }

    #[test]
    fn test_horizontal_intercept() {
        let quad = DQuad::new(
            Point::new(0.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(2.0, 0.0),
        );

        let mut roots = [0.0; MAX_QUAD_ROOTS];
        let count = horizontal_intercept(&quad, 0.0, &mut roots);

        // Should find t=0 and t=1 (endpoints)
        assert!(count == 2, "Should find 2 roots for y=0");
    }
}
