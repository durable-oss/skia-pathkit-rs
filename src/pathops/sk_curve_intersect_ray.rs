//! Curve-against-infinite-line intersection, in f64.
//!
//! Port of the `CurveIntersectRay` dispatch table
//! (`SkPathOpsCurve.h`) together with the four `intersectRay` bodies it
//! points at, one per verb.
//!
//! # Why this is separate from the `sk_d_*_line_intersection` modules
//!
//! Those modules implement the full *segment* intersection: exact and near
//! endpoint handling, t pinning, coincidence checks, and they are written in
//! f32. `intersectRay` is the much smaller thing underneath — roots of the
//! curve against an unbounded line, with no endpoint fixup at all — and the
//! angle sorter needs it in f64.
//!
//! The precision matters. `SkOpAngle` decides curve order from the sign of
//! cross products between vectors it builds out of these roots; the whole
//! sector scheme exists so those sign decisions survive rounding. Narrowing
//! to f32 here would put the rounding back in front of the decision rather
//! than behind it. `sk_op_angle` and `sk_line_parameters` are f64 for the
//! same reason.
//!
//! # Rays, not segments
//!
//! The line is treated as infinite in both directions. Roots are reported in
//! the *curve's* parameter space only, and only those in `[0, 1]`: a hit past
//! the end of the caller's line is still a hit, which is exactly what
//! `endToSide` and `midToSide` want when they cast a perpendicular.

use super::sk_line_parameters::LinePoint;
use super::sk_op_angle::verb_to_points;
use super::sk_path_ops_cubic::SkDCubic;
use super::sk_path_ops_quad::SkDQuad;
use super::sk_path_ops_types::approximately_zero;
use crate::core::Verb;

/// Most roots any verb can produce against a line.
pub const MAX_RAY_ROOTS: usize = 3;

/// Where a ray crossed a curve.
#[derive(Debug, Clone, Copy, Default)]
pub struct RayHit {
    /// Parameter along the curve, in `[0, 1]`.
    pub t: f64,
    /// The point at `t`.
    pub pt: LinePoint,
}

/// The hits a ray made against one curve, in the order the roots were found.
#[derive(Debug, Clone, Copy, Default)]
pub struct RayHits {
    hits: [RayHit; MAX_RAY_ROOTS],
    used: usize,
}

impl RayHits {
    /// Returns an empty hit list.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns how many hits were recorded.
    #[must_use]
    pub fn used(&self) -> usize {
        self.used
    }

    /// Returns true when the ray missed entirely.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.used == 0
    }

    /// Returns hit `index`.
    ///
    /// # Panics
    ///
    /// Panics if `index` is at or past [`used`](Self::used).
    #[must_use]
    pub fn get(&self, index: usize) -> RayHit {
        assert!(index < self.used, "hit {index} of {}", self.used);
        self.hits[index]
    }

    /// Returns the curve parameter of hit `index`.
    ///
    /// # Panics
    ///
    /// Panics if `index` is at or past [`used`](Self::used).
    #[must_use]
    pub fn t(&self, index: usize) -> f64 {
        self.get(index).t
    }

    /// Returns the point of hit `index`.
    ///
    /// # Panics
    ///
    /// Panics if `index` is at or past [`used`](Self::used).
    #[must_use]
    pub fn pt(&self, index: usize) -> LinePoint {
        self.get(index).pt
    }

    /// Iterates the hits recorded so far.
    pub fn iter(&self) -> impl Iterator<Item = &RayHit> {
        self.hits[..self.used].iter()
    }

    fn push(&mut self, t: f64, pt: LinePoint) {
        if self.used < MAX_RAY_ROOTS {
            self.hits[self.used] = RayHit { t, pt };
            self.used += 1;
        }
    }
}

/// Returns the point `pts`/`verb` reaches at `t`, in f64.
///
/// The same evaluation as `SkDCurve::ptAtT`, kept here so the ray code does
/// not have to convert through the f32 curve types to get a point back.
#[must_use]
pub fn curve_pt_at_t(pts: &[LinePoint], verb: Verb, weight: f64, t: f64) -> LinePoint {
    let u = 1.0 - t;
    match verb {
        Verb::Line => [
            u * pts[0][0] + t * pts[1][0],
            u * pts[0][1] + t * pts[1][1],
        ],
        Verb::Quad => {
            let (a, b, c) = (u * u, 2.0 * u * t, t * t);
            [
                a * pts[0][0] + b * pts[1][0] + c * pts[2][0],
                a * pts[0][1] + b * pts[1][1] + c * pts[2][1],
            ]
        }
        Verb::Conic => {
            let (a, b, c) = (u * u, 2.0 * u * t * weight, t * t);
            let denom = a + b + c;
            [
                (a * pts[0][0] + b * pts[1][0] + c * pts[2][0]) / denom,
                (a * pts[0][1] + b * pts[1][1] + c * pts[2][1]) / denom,
            ]
        }
        Verb::Cubic => {
            let (a, b, c, d) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
            [
                a * pts[0][0] + b * pts[1][0] + c * pts[2][0] + d * pts[3][0],
                a * pts[0][1] + b * pts[1][1] + c * pts[2][1] + d * pts[3][1],
            ]
        }
        // Move and Close carry no curve.
        Verb::Move | Verb::Close => pts[0],
    }
}

/// Returns the curve's tangent at `t`, in f64.
///
/// Port of the `CurveDSlopeAtT` table. A line's derivative is constant, so it
/// reports the vector from its start to its end rather than zero, matching
/// `SkDCurve::dxdyAtT`.
#[must_use]
pub fn curve_d_slope_at_t(pts: &[LinePoint], verb: Verb, weight: f64, t: f64) -> LinePoint {
    match verb {
        Verb::Line => [pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]],
        Verb::Quad => {
            let a = t - 1.0;
            let b = 1.0 - 2.0 * t;
            [
                a * pts[0][0] + b * pts[1][0] + t * pts[2][0],
                a * pts[0][1] + b * pts[1][1] + t * pts[2][1],
            ]
        }
        Verb::Conic => conic_d_slope_at_t(pts, weight, t),
        Verb::Cubic => {
            let one_t = 1.0 - t;
            let a = -one_t * one_t;
            let b = (1.0 - 3.0 * t) * one_t;
            let c = (2.0 - 3.0 * t) * t;
            let d = t * t;
            [
                3.0 * (a * pts[0][0] + b * pts[1][0] + c * pts[2][0] + d * pts[3][0]),
                3.0 * (a * pts[0][1] + b * pts[1][1] + c * pts[2][1] + d * pts[3][1]),
            ]
        }
        Verb::Move | Verb::Close => [0.0, 0.0],
    }
}

/// Returns a conic's tangent at `t`.
///
/// Port of `SkDConic::dxdyAtT`. The rational form needs the quotient rule:
/// the numerator and denominator are each quadratics in `t`, so the
/// derivative is `(n' * d - n * d') / d^2`. The `d^2` is dropped, since only
/// the direction is used.
fn conic_d_slope_at_t(pts: &[LinePoint], weight: f64, t: f64) -> LinePoint {
    let mut out = [0.0; 2];
    for (axis, slot) in out.iter_mut().enumerate() {
        let (p0, p1, p2) = (pts[0][axis], pts[1][axis], pts[2][axis]);
        // Numerator and denominator in Bernstein form, weighted.
        let src = [p0, p1 * weight, p2];
        let w = [1.0, weight, 1.0];
        let num = conic_quad_at(&src, t);
        let den = conic_quad_at(&w, t);
        let d_num = conic_quad_slope(&src, t);
        let d_den = conic_quad_slope(&w, t);
        *slot = d_num * den - num * d_den;
    }
    out
}

/// Evaluates the quadratic Bernstein polynomial `p` at `t`.
fn conic_quad_at(p: &[f64; 3], t: f64) -> f64 {
    let u = 1.0 - t;
    u * u * p[0] + 2.0 * u * t * p[1] + t * t * p[2]
}

/// Evaluates the derivative of the quadratic Bernstein polynomial `p` at `t`.
fn conic_quad_slope(p: &[f64; 3], t: f64) -> f64 {
    2.0 * ((1.0 - t) * (p[1] - p[0]) + t * (p[2] - p[1]))
}

/// Intersects the curve `pts`/`verb` against the infinite line through `line`.
///
/// Port of `CurveIntersectRay`. The roots are in the curve's parameter space,
/// filtered to `[0, 1]`; the line is unbounded, so a hit beyond `line[1]` is
/// still reported.
///
/// Every verb rotates the problem so the line lies on the x axis, then solves
/// for where the rotated curve's y crosses zero. That avoids a division and
/// keeps a vertical line from being a special case.
#[must_use]
pub fn curve_intersect_ray(
    pts: &[LinePoint],
    verb: Verb,
    weight: f64,
    line: &[LinePoint; 2],
) -> RayHits {
    let mut hits = RayHits::new();
    let adj = line[1][0] - line[0][0];
    let opp = line[1][1] - line[0][1];
    // Rotating the curve so the line is horizontal: the rotated y of point n.
    let rotated = |n: usize| (pts[n][1] - line[0][1]) * adj - (pts[n][0] - line[0][0]) * opp;

    let mut roots = [0.0_f64; MAX_RAY_ROOTS];
    let count = match verb {
        Verb::Line => return line_intersect_ray(pts, line),
        Verb::Quad => {
            let (c, b, a) = (rotated(0), rotated(1), rotated(2));
            // A = a - 2b + c, B = -(b - c) doubled by the caller's convention.
            let qa = a + c - 2.0 * b;
            let qb = b - c;
            let mut two = [0.0_f64; 2];
            let n = SkDQuad::roots_valid_t(qa, 2.0 * qb, c, &mut two);
            roots[..n].copy_from_slice(&two[..n]);
            n
        }
        Verb::Conic => {
            let (c, b, a) = (rotated(0), rotated(1), rotated(2));
            // The axis intercept is 0 here, so B collapses to b * w.
            let cb = b * weight;
            let qa = a + c - 2.0 * cb;
            let qb = cb - c;
            let mut two = [0.0_f64; 2];
            let n = SkDQuad::roots_valid_t(qa, 2.0 * qb, c, &mut two);
            roots[..n].copy_from_slice(&two[..n]);
            n
        }
        Verb::Cubic => {
            let rot = [rotated(0), rotated(1), rotated(2), rotated(3)];
            let (a, b, c, d) = cubic_coefficients(&rot);
            let (found, n) = SkDCubic::roots_valid_t(a, b, c, d);
            roots[..n].copy_from_slice(&found[..n]);
            n
        }
        Verb::Move | Verb::Close => 0,
    };

    for &t in roots.iter().take(count) {
        hits.push(t, curve_pt_at_t(pts, verb, weight, t));
    }
    hits
}

/// Converts a cubic's Bernstein values to power-basis coefficients.
///
/// Port of `SkDCubic::Coefficients`, which strides by 2 because it reads one
/// axis out of an array of points; `src` here is already the four values of
/// that axis.
fn cubic_coefficients(src: &[f64; 4]) -> (f64, f64, f64, f64) {
    let d = src[0];
    let mut a = src[3];
    let mut b = src[2] * 3.0;
    let mut c = src[1] * 3.0;
    a -= d - c + b; // A =  -a + 3b - 3c + d
    b += 3.0 * d - 2.0 * c; // B =  3a - 6b + 3c
    c -= 3.0 * d; // C = -3a + 3b
    (a, b, c, d)
}

/// Intersects two infinite lines.
///
/// Port of `SkIntersections::intersectRay(SkDLine, SkDLine)`, reporting the
/// parameter on `pts` only. Parallel lines report nothing: the caller wants a
/// crossing point, and coincident lines have no single one.
fn line_intersect_ray(pts: &[LinePoint], line: &[LinePoint; 2]) -> RayHits {
    let mut hits = RayHits::new();
    let a_len = [pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]];
    let b_len = [line[1][0] - line[0][0], line[1][1] - line[0][1]];
    let denom = b_len[1] * a_len[0] - a_len[1] * b_len[0];
    if approximately_zero(denom) {
        return hits;
    }
    let ab0 = [pts[0][0] - line[0][0], pts[0][1] - line[0][1]];
    let numer_a = (ab0[1] * b_len[0] - b_len[1] * ab0[0]) / denom;
    hits.push(
        numer_a,
        [
            pts[0][0] + numer_a * a_len[0],
            pts[0][1] + numer_a * a_len[1],
        ],
    );
    hits
}

/// Returns the hit closest to `test_pt` whose t lies between `range_start`
/// and `range_end`, and its distance.
///
/// Port of `SkIntersections::closestTo`, in f64. Returns `None` when no hit
/// falls in the range. The range ends may be given in either order.
#[must_use]
pub fn closest_to(
    hits: &RayHits,
    range_start: f64,
    range_end: f64,
    test_pt: LinePoint,
) -> Option<(usize, f64)> {
    let (lo, hi) = if range_start <= range_end {
        (range_start, range_end)
    } else {
        (range_end, range_start)
    };
    let mut best: Option<(usize, f64)> = None;
    for (index, hit) in hits.iter().enumerate() {
        if !(lo..=hi).contains(&hit.t) {
            continue;
        }
        let dx = hit.pt[0] - test_pt[0];
        let dy = hit.pt[1] - test_pt[1];
        let dist = dx * dx + dy * dy;
        if best.map_or(true, |(_, b)| dist < b) {
            best = Some((index, dist));
        }
    }
    best.map(|(index, dist_sq)| (index, dist_sq.sqrt()))
}

/// Returns the hit furthest from `origin` whose t lies in the range.
///
/// Port of `SkIntersections::mostOutside`, in f64. `midToSide` uses it to
/// pick the crossing that best represents which side a curve is on; the
/// nearest crossing can be the shared origin itself.
#[must_use]
pub fn most_outside(
    hits: &RayHits,
    range_start: f64,
    range_end: f64,
    origin: LinePoint,
) -> Option<usize> {
    let (lo, hi) = if range_start <= range_end {
        (range_start, range_end)
    } else {
        (range_end, range_start)
    };
    let mut best: Option<(usize, f64)> = None;
    for (index, hit) in hits.iter().enumerate() {
        if !(lo..=hi).contains(&hit.t) {
            continue;
        }
        let dx = hit.pt[0] - origin[0];
        let dy = hit.pt[1] - origin[1];
        let dist = dx * dx + dy * dy;
        if best.map_or(true, |(_, b)| dist > b) {
            best = Some((index, dist));
        }
    }
    best.map(|(index, _)| index)
}

/// Returns the bounding box of `pts`, as `(min_x, min_y, max_x, max_y)`.
///
/// The angle code normalizes several distances by the larger box dimension;
/// this is that measurement, shared rather than written out four times.
#[must_use]
pub fn curve_extent(pts: &[LinePoint], verb: Verb) -> f64 {
    let count = verb_to_points(verb);
    let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
    let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in pts.iter().take(count + 1) {
        min_x = min_x.min(p[0]);
        min_y = min_y.min(p[1]);
        max_x = max_x.max(p[0]);
        max_y = max_y.max(p[1]);
    }
    (max_x - min_x).max(max_y - min_y)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Returns true when `a` and `b` agree to within `tol`.
    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn a_line_crossing_a_line_reports_the_crossing_t() {
        // The diagonal y = x, cut by the horizontal y = 1.
        let diag = [[0.0, 0.0], [2.0, 2.0]];
        let horiz = [[0.0, 1.0], [1.0, 1.0]];
        let hits = curve_intersect_ray(&diag, Verb::Line, 1.0, &horiz);
        assert_eq!(hits.used(), 1);
        assert!(near(hits.t(0), 0.5, 1e-12), "t was {}", hits.t(0));
        assert!(near(hits.pt(0)[0], 1.0, 1e-12));
    }

    #[test]
    fn the_ray_reaches_past_the_end_of_the_given_line() {
        // The line segment stops at x = 1, but the ray it defines does not.
        let diag = [[0.0, 0.0], [10.0, 10.0]];
        let stub = [[0.0, 8.0], [1.0, 8.0]];
        let hits = curve_intersect_ray(&diag, Verb::Line, 1.0, &stub);
        assert_eq!(hits.used(), 1, "a hit beyond the stub still counts");
        assert!(near(hits.t(0), 0.8, 1e-12), "t was {}", hits.t(0));
    }

    #[test]
    fn parallel_lines_report_no_crossing() {
        let a = [[0.0, 0.0], [1.0, 0.0]];
        let b = [[0.0, 5.0], [1.0, 5.0]];
        assert!(curve_intersect_ray(&a, Verb::Line, 1.0, &b).is_empty());
    }

    #[test]
    fn a_horizontal_ray_cuts_an_arch_quad_twice() {
        // An arch from (0,0) up over (1,2) and back to (2,0); y = 0.5 cuts it
        // on both sides.
        let quad = [[0.0, 0.0], [1.0, 2.0], [2.0, 0.0]];
        let ray = [[-1.0, 0.5], [3.0, 0.5]];
        let hits = curve_intersect_ray(&quad, Verb::Quad, 1.0, &ray);
        assert_eq!(hits.used(), 2, "an arch crossed below its apex has two");
        for i in 0..hits.used() {
            assert!(
                near(hits.pt(i)[1], 0.5, 1e-9),
                "hit {i} is off the ray at y = {}",
                hits.pt(i)[1]
            );
        }
    }

    #[test]
    fn a_ray_above_the_apex_misses_the_quad() {
        let quad = [[0.0, 0.0], [1.0, 2.0], [2.0, 0.0]];
        // The arch peaks at y = 1, so y = 1.5 is clear of it.
        let ray = [[-1.0, 1.5], [3.0, 1.5]];
        assert!(curve_intersect_ray(&quad, Verb::Quad, 1.0, &ray).is_empty());
    }

    #[test]
    fn a_ray_through_a_cubic_s_curve_finds_three_roots() {
        // An S that crosses y = 0 three times between its endpoints.
        let cubic = [[0.0, 0.0], [1.0, 3.0], [2.0, -3.0], [3.0, 0.0]];
        let ray = [[-1.0, 0.0], [4.0, 0.0]];
        let hits = curve_intersect_ray(&cubic, Verb::Cubic, 1.0, &ray);
        assert_eq!(hits.used(), 3, "an S crossing its own chord hits thrice");
        for i in 0..hits.used() {
            assert!(near(hits.pt(i)[1], 0.0, 1e-9));
        }
    }

    #[test]
    fn a_conics_hit_moves_with_its_weight() {
        let pts = [[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]];
        let ray = [[1.0, -1.0], [1.0, 2.0]];
        let light = curve_intersect_ray(&pts, Verb::Conic, 0.25, &ray);
        let heavy = curve_intersect_ray(&pts, Verb::Conic, 4.0, &ray);
        assert_eq!(light.used(), 1);
        assert_eq!(heavy.used(), 1);
        assert!(
            heavy.pt(0)[1] > light.pt(0)[1],
            "a heavier conic rides nearer its control point: {} vs {}",
            heavy.pt(0)[1],
            light.pt(0)[1]
        );
    }

    #[test]
    fn closest_to_honours_the_t_range() {
        let quad = [[0.0, 0.0], [1.0, 2.0], [2.0, 0.0]];
        let ray = [[-1.0, 0.5], [3.0, 0.5]];
        let hits = curve_intersect_ray(&quad, Verb::Quad, 1.0, &ray);
        assert_eq!(hits.used(), 2);
        // Restricted to the first half, only the ascending hit qualifies.
        let (index, _) = closest_to(&hits, 0.0, 0.5, [0.0, 0.5]).expect("a hit in range");
        assert!(hits.t(index) <= 0.5, "t was {}", hits.t(index));
        // With no hit in range at all, there is no answer.
        assert!(closest_to(&hits, 0.49, 0.5, [0.0, 0.5]).is_none());
    }

    #[test]
    fn most_outside_picks_the_further_hit_not_the_nearer() {
        let quad = [[0.0, 0.0], [1.0, 2.0], [2.0, 0.0]];
        let ray = [[-1.0, 0.5], [3.0, 0.5]];
        let hits = curve_intersect_ray(&quad, Verb::Quad, 1.0, &ray);
        let origin = hits.pt(0);
        let outside = most_outside(&hits, 0.0, 1.0, origin).expect("a hit");
        assert_ne!(
            outside, 0,
            "the hit at the origin is the nearest, so it must not win"
        );
    }

    #[test]
    fn a_lines_slope_is_its_whole_length_not_zero() {
        let line = [[1.0, 2.0], [4.0, 6.0]];
        let slope = curve_d_slope_at_t(&line, Verb::Line, 1.0, 0.5);
        assert!(near(slope[0], 3.0, 1e-12));
        assert!(near(slope[1], 4.0, 1e-12));
    }

    #[test]
    fn a_quads_slope_is_flat_at_its_apex() {
        // Symmetric arch: the tangent at t = 0.5 is horizontal.
        let quad = [[0.0, 0.0], [1.0, 2.0], [2.0, 0.0]];
        let slope = curve_d_slope_at_t(&quad, Verb::Quad, 1.0, 0.5);
        assert!(near(slope[1], 0.0, 1e-12), "dy was {}", slope[1]);
        assert!(slope[0] > 0.0, "still travelling in +x");
    }

    #[test]
    fn a_symmetric_conics_slope_is_flat_at_its_apex() {
        let pts = [[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]];
        for w in [0.25, 1.0, 4.0] {
            let slope = curve_d_slope_at_t(&pts, Verb::Conic, w, 0.5);
            assert!(
                near(slope[1], 0.0, 1e-9),
                "weight {w} gave dy = {}",
                slope[1]
            );
        }
    }

    #[test]
    fn curve_extent_is_the_larger_box_dimension() {
        let cubic = [[0.0, 0.0], [0.0, 10.0], [3.0, 10.0], [3.0, 0.0]];
        // Box is 3 wide and 10 tall, so 10 wins.
        assert!(near(curve_extent(&cubic, Verb::Cubic), 10.0, 1e-12));
    }
}
