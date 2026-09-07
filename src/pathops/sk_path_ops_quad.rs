//! Double-precision quadratic Bezier curve operations for path operations.
//!
//! Port of Skia's `SkPathOpsQuad.{h,cpp}`.

use super::sk_path_ops_point::{SkDPoint, SkDVector};
use super::sk_path_ops_types::{
    almost_dequal_ulps, approximately_equal, approximately_one_or_less, approximately_zero,
    approximately_zero_or_more, between_d,
};

/// The result of [`SkDQuad::chop_at`]: two quads sharing the split point,
/// packed as 5 points (`[a, b, c, d, e]` where `[a,b,c]` is the first quad
/// and `[c,d,e]` is the second).
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDQuadPair {
    /// The five packed control points.
    pub pts: [SkDPoint; 5],
}

impl SkDQuadPair {
    /// The first of the two sub-quads.
    pub fn first(&self) -> SkDQuad {
        SkDQuad {
            f_pts: [self.pts[0], self.pts[1], self.pts[2]],
        }
    }

    /// The second of the two sub-quads.
    pub fn second(&self) -> SkDQuad {
        SkDQuad {
            f_pts: [self.pts[2], self.pts[3], self.pts[4]],
        }
    }
}

/// A double-precision quadratic Bezier curve.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDQuad {
    /// The three control points: start, control, end.
    pub f_pts: [SkDPoint; 3],
}

impl SkDQuad {
    /// Number of control points in a quad.
    pub const K_POINT_COUNT: usize = 3;
    /// Index of the last control point.
    pub const K_POINT_LAST: usize = Self::K_POINT_COUNT - 1;
    /// Maximum number of intersections between two quads.
    pub const K_MAX_INTERSECTIONS: usize = 4;

    /// Creates a quad from its three control points.
    pub fn new(pts: [SkDPoint; 3]) -> Self {
        Self { f_pts: pts }
    }

    /// True if all three control points coincide.
    pub fn collapsed(&self) -> bool {
        self.f_pts[0].approximately_equal(self.f_pts[1])
            && self.f_pts[0].approximately_equal(self.f_pts[2])
    }

    /// True if the control point lies inside the wedge formed by the two
    /// endpoint tangent directions (i.e. the curve doesn't loop back on
    /// itself relative to its control point).
    pub fn controls_inside(&self) -> bool {
        let v01 = self.f_pts[0] - self.f_pts[1];
        let v02 = self.f_pts[0] - self.f_pts[2];
        let v12 = self.f_pts[1] - self.f_pts[2];
        v02.dot(v01) > 0.0 && v02.dot(v12) > 0.0
    }

    /// Returns this quad with its point order reversed.
    pub fn flip(&self) -> Self {
        Self {
            f_pts: [self.f_pts[2], self.f_pts[1], self.f_pts[0]],
        }
    }

    /// False: a quad is never a conic.
    pub fn is_conic() -> bool {
        false
    }

    /// Returns the `n`th control point.
    pub fn get(&self, n: usize) -> SkDPoint {
        self.f_pts[n]
    }

    /// Snaps `dst_pt` to `self[1]`'s coordinate(s) on any axis where
    /// `self[end_index]` already matches it exactly. Used after chopping to
    /// avoid reintroducing floating-point drift at shared endpoints.
    pub fn align(&self, end_index: usize, dst_pt: &mut SkDPoint) {
        if self.f_pts[end_index].f_x == self.f_pts[1].f_x {
            dst_pt.f_x = self.f_pts[end_index].f_x;
        }
        if self.f_pts[end_index].f_y == self.f_pts[1].f_y {
            dst_pt.f_y = self.f_pts[end_index].f_y;
        }
    }

    /// Returns the two control points other than `odd_man`, in a fixed
    /// order derived from Skia's bit-twiddling trick (see the C++ comment
    /// this is ported from in `SkPathOpsQuad.cpp`).
    pub fn other_pts(&self, odd_man: usize) -> [SkDPoint; 2] {
        let mut end_pt = [SkDPoint::default(); 2];
        for (i, opp) in (1..Self::K_POINT_COUNT).enumerate() {
            let mut end = (odd_man ^ opp) as isize - odd_man as isize;
            end &= !(end >> 2);
            end_pt[i] = self.f_pts[end as usize];
        }
        end_pt
    }

    /// True if the control points are monotonic (non-reversing) in x.
    pub fn monotonic_in_x(&self) -> bool {
        between_d(self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[2].f_x)
    }

    /// True if the control points are monotonic (non-reversing) in y.
    pub fn monotonic_in_y(&self) -> bool {
        between_d(self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[2].f_y)
    }

    /// Evaluates the curve at parameter `t` in `[0, 1]`.
    pub fn pt_at_t(&self, t: f64) -> SkDPoint {
        if t == 0.0 {
            return self.f_pts[0];
        }
        if t == 1.0 {
            return self.f_pts[2];
        }
        let one_t = 1.0 - t;
        let a = one_t * one_t;
        let b = 2.0 * one_t * t;
        let c = t * t;
        SkDPoint::new(
            a * self.f_pts[0].f_x + b * self.f_pts[1].f_x + c * self.f_pts[2].f_x,
            a * self.f_pts[0].f_y + b * self.f_pts[1].f_y + c * self.f_pts[2].f_y,
        )
    }

    /// Evaluates the curve's derivative (tangent direction, not
    /// necessarily normalized) at parameter `t`.
    pub fn dxdy_at_t(&self, t: f64) -> SkDVector {
        let a = t - 1.0;
        let b = 1.0 - 2.0 * t;
        let c = t;
        let mut result = SkDVector::new(
            a * self.f_pts[0].f_x + b * self.f_pts[1].f_x + c * self.f_pts[2].f_x,
            a * self.f_pts[0].f_y + b * self.f_pts[1].f_y + c * self.f_pts[2].f_y,
        );
        if result.f_x == 0.0 && result.f_y == 0.0 && (t == 0.0 || t == 1.0) {
            result = self.f_pts[2] - self.f_pts[0];
        }
        result
    }

    /// Quick-reject test for whether this quad's convex hull can possibly
    /// intersect `q2`'s. Returns `true` if an intersection is possible (in
    /// which case `is_linear` reports whether this quad degenerated to a
    /// line for the purposes of the test); returns `false` only when the
    /// quads provably share at most their endpoints.
    pub fn hull_intersects(&self, q2: &SkDQuad, is_linear: &mut bool) -> bool {
        let mut linear = true;
        for odd_man in 0..Self::K_POINT_COUNT {
            let end_pt = self.other_pts(odd_man);
            let orig_x = end_pt[0].f_x;
            let orig_y = end_pt[0].f_y;
            let adj = end_pt[1].f_x - orig_x;
            let opp = end_pt[1].f_y - orig_y;
            let sign =
                (self.f_pts[odd_man].f_y - orig_y) * adj - (self.f_pts[odd_man].f_x - orig_x) * opp;
            if approximately_zero(sign) {
                continue;
            }
            linear = false;
            let mut found_outlier = false;
            for n in 0..Self::K_POINT_COUNT {
                let test = (q2.f_pts[n].f_y - orig_y) * adj - (q2.f_pts[n].f_x - orig_x) * opp;
                if test * sign > 0.0 && !precisely_zero(test) {
                    found_outlier = true;
                    break;
                }
            }
            if !found_outlier {
                return false;
            }
        }
        *is_linear = linear;
        true
    }

    /// Finds the real roots of `A*t^2 + B*t + C == 0`, without discarding
    /// roots outside `[0, 1]` (see [`roots_valid_t`](Self::roots_valid_t)
    /// for that).
    pub fn roots_real(a: f64, b: f64, c: f64, s: &mut [f64; 2]) -> usize {
        if a == 0.0 {
            return handle_zero(b, c, s);
        }
        let p = b / (2.0 * a);
        let q = c / a;
        if approximately_zero(a) && (approximately_zero_inverse(p) || approximately_zero_inverse(q))
        {
            return handle_zero(b, c, s);
        }
        let p2 = p * p;
        if !almost_dequal_ulps(p2 as f32, q as f32) && p2 < q {
            return 0;
        }
        let sqrt_d = if p2 > q { (p2 - q).sqrt() } else { 0.0 };
        s[0] = sqrt_d - p;
        s[1] = -sqrt_d - p;
        1 + usize::from(!almost_dequal_ulps(s[0] as f32, s[1] as f32))
    }

    /// Like [`roots_real`](Self::roots_real), but keeps only roots that lie
    /// in (or snap to) `[0, 1]`.
    pub fn roots_valid_t(a: f64, b: f64, c: f64, t: &mut [f64; 2]) -> usize {
        let mut s = [0.0; 2];
        let real_roots = Self::roots_real(a, b, c, &mut s);
        Self::add_valid_ts(&s, real_roots, t)
    }

    /// Filters `s[0..real_roots]` down to those approximately in `[0, 1]`,
    /// clamping near-boundary values and deduplicating, writing the result
    /// into `t`. Shared by the cubic/conic root finders too.
    pub fn add_valid_ts(s: &[f64], real_roots: usize, t: &mut [f64]) -> usize {
        let mut found_roots = 0;
        for &s_val in s.iter().take(real_roots) {
            let mut t_value = s_val;
            if !(approximately_zero_or_more(t_value) && approximately_one_or_less(t_value)) {
                continue;
            }
            if super::sk_path_ops_types::approximately_less_than_zero(t_value) {
                t_value = 0.0;
            } else if super::sk_path_ops_types::approximately_greater_than_one(t_value) {
                t_value = 1.0;
            }
            if t.iter()
                .take(found_roots)
                .any(|&existing| approximately_equal(existing, t_value))
            {
                continue;
            }
            t[found_roots] = t_value;
            found_roots += 1;
        }
        found_roots
    }

    /// Splits the quad at `t1` and `t2`, returning the sub-quad spanning
    /// `[t1, t2]`.
    pub fn sub_divide(&self, t1: f64, t2: f64) -> Self {
        if t1 == 0.0 && t2 == 1.0 {
            return *self;
        }
        let ax = interp_quad_coords(self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[2].f_x, t1);
        let ay = interp_quad_coords(self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[2].f_y, t1);
        let mid_t = (t1 + t2) / 2.0;
        let dx = interp_quad_coords(
            self.f_pts[0].f_x,
            self.f_pts[1].f_x,
            self.f_pts[2].f_x,
            mid_t,
        );
        let dy = interp_quad_coords(
            self.f_pts[0].f_y,
            self.f_pts[1].f_y,
            self.f_pts[2].f_y,
            mid_t,
        );
        let cx = interp_quad_coords(self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[2].f_x, t2);
        let cy = interp_quad_coords(self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[2].f_y, t2);
        Self {
            f_pts: [
                SkDPoint::new(ax, ay),
                SkDPoint::new(2.0 * dx - (ax + cx) / 2.0, 2.0 * dy - (ay + cy) / 2.0),
                SkDPoint::new(cx, cy),
            ],
        }
    }

    /// Splits the quad into two quads meeting at parameter `t`.
    pub fn chop_at(&self, t: f64) -> SkDQuadPair {
        let mut pts = [SkDPoint::default(); 5];
        let (x0, x1, x2, x3, x4) =
            interp_quad_coords_chop(self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[2].f_x, t);
        let (y0, y1, y2, y3, y4) =
            interp_quad_coords_chop(self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[2].f_y, t);
        pts[0] = SkDPoint::new(x0, y0);
        pts[1] = SkDPoint::new(x1, y1);
        pts[2] = SkDPoint::new(x2, y2);
        pts[3] = SkDPoint::new(x3, y3);
        pts[4] = SkDPoint::new(x4, y4);
        SkDQuadPair { pts }
    }

    /// Finds where the derivative is zero (the curve's x/y extremum), if
    /// any, over `(0, 1)`. `src` is `[start, control, end]` for one axis.
    pub fn find_extrema(src: &[f64; 3]) -> Option<f64> {
        let a = src[0];
        let b = src[1];
        let c = src[2];
        valid_unit_divide(a - b, a - b - b + c)
    }

    /// Converts the quad's per-axis control values into the `A*t^2 +
    /// 2*B*t*(1-t) + C*(1-t)^2` parameterization's `(a, b, c)` coefficients.
    pub fn set_abc(quad: &[f64; 3]) -> (f64, f64, f64) {
        let mut a = quad[0];
        let mut b = 2.0 * quad[1];
        let c = quad[2];
        b -= c;
        a -= b;
        b -= c;
        (a, b, c)
    }
}

impl std::ops::Index<usize> for SkDQuad {
    type Output = SkDPoint;
    fn index(&self, index: usize) -> &Self::Output {
        &self.f_pts[index]
    }
}

impl std::ops::IndexMut<usize> for SkDQuad {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.f_pts[index]
    }
}

fn handle_zero(b: f64, c: f64, s: &mut [f64; 2]) -> usize {
    if approximately_zero(b) {
        s[0] = 0.0;
        return usize::from(c == 0.0);
    }
    s[0] = -c / b;
    1
}

fn approximately_zero_inverse(x: f64) -> bool {
    super::sk_path_ops_types::approximately_zero_inverse(x)
}

fn precisely_zero(x: f64) -> bool {
    super::sk_path_ops_types::precisely_zero(x)
}

/// Interpolates one axis of `[start, control, end]` at parameter `t` via
/// two nested lerps (De Casteljau's algorithm, one level).
fn interp_quad_coords(start: f64, control: f64, end: f64, t: f64) -> f64 {
    if t == 0.0 {
        return start;
    }
    if t == 1.0 {
        return end;
    }
    let ab = lerp(start, control, t);
    let bc = lerp(control, end, t);
    lerp(ab, bc, t)
}

/// Same De Casteljau split as [`interp_quad_coords`], but returns every
/// intermediate value: `(start, ab, abc, bc, end)`. `chop_at` needs the
/// full ladder (not just the final point) to build both sub-quads.
fn interp_quad_coords_chop(
    start: f64,
    control: f64,
    end: f64,
    t: f64,
) -> (f64, f64, f64, f64, f64) {
    let ab = lerp(start, control, t);
    let bc = lerp(control, end, t);
    let abc = lerp(ab, bc, t);
    (start, ab, abc, bc, end)
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Ported from Skia's `valid_unit_divide`: computes `numer/denom`, flipping
/// the sign of both if `numer` is negative so the result is compared
/// against `denom`'s original sign consistently, and rejects results
/// outside `(0, 1)` or that underflow to zero.
fn valid_unit_divide(mut numer: f64, mut denom: f64) -> Option<f64> {
    if numer < 0.0 {
        numer = -numer;
        denom = -denom;
    }
    if denom == 0.0 || numer == 0.0 || numer >= denom {
        return None;
    }
    let r = numer / denom;
    if r == 0.0 {
        return None;
    }
    Some(r)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad(pts: [(f64, f64); 3]) -> SkDQuad {
        SkDQuad::new([
            SkDPoint::new(pts[0].0, pts[0].1),
            SkDPoint::new(pts[1].0, pts[1].1),
            SkDPoint::new(pts[2].0, pts[2].1),
        ])
    }

    #[test]
    fn collapsed() {
        let q = quad([(1.0, 1.0), (1.0, 1.0), (1.0, 1.0)]);
        assert!(q.collapsed());
        let q2 = quad([(0.0, 0.0), (5.0, 5.0), (10.0, 0.0)]);
        assert!(!q2.collapsed());
    }

    #[test]
    fn flip() {
        let q = quad([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)]);
        let f = q.flip();
        assert_eq!(f.f_pts[0], q.f_pts[2]);
        assert_eq!(f.f_pts[2], q.f_pts[0]);
    }

    #[test]
    fn pt_at_t_endpoints() {
        let q = quad([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)]);
        assert_eq!(q.pt_at_t(0.0), q.f_pts[0]);
        assert_eq!(q.pt_at_t(1.0), q.f_pts[2]);
        let mid = q.pt_at_t(0.5);
        assert!((mid.f_x - 5.0).abs() < 1e-9);
        assert!((mid.f_y - 5.0).abs() < 1e-9);
    }

    #[test]
    fn monotonic() {
        let q = quad([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)]);
        assert!(q.monotonic_in_x());
        assert!(!q.monotonic_in_y());
    }

    #[test]
    fn find_extrema_parabola_apex() {
        // y goes 0 -> 10 -> 0: the derivative zero (apex) is at t=0.5.
        let t = SkDQuad::find_extrema(&[0.0, 10.0, 0.0]);
        assert!(t.is_some());
        assert!((t.unwrap() - 0.5).abs() < 1e-9);
    }

    #[test]
    fn find_extrema_monotonic_is_none() {
        let t = SkDQuad::find_extrema(&[0.0, 5.0, 10.0]);
        assert!(t.is_none());
    }

    #[test]
    fn sub_divide_identity() {
        let q = quad([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)]);
        let s = q.sub_divide(0.0, 1.0);
        assert_eq!(s.f_pts[0], q.f_pts[0]);
        assert_eq!(s.f_pts[2], q.f_pts[2]);
    }

    #[test]
    fn sub_divide_half() {
        let q = quad([(0.0, 0.0), (10.0, 10.0), (20.0, 0.0)]);
        let s = q.sub_divide(0.0, 0.5);
        // The half-curve should start where the original does...
        assert_eq!(s.f_pts[0], q.f_pts[0]);
        // ...and end at the original curve's t=0.5 point.
        let expected_end = q.pt_at_t(0.5);
        assert!((s.f_pts[2].f_x - expected_end.f_x).abs() < 1e-9);
        assert!((s.f_pts[2].f_y - expected_end.f_y).abs() < 1e-9);
    }

    #[test]
    fn chop_at_matches_pt_at_t() {
        let q = quad([(0.0, 0.0), (10.0, 10.0), (20.0, 0.0)]);
        let pair = q.chop_at(0.5);
        let expected = q.pt_at_t(0.5);
        assert!((pair.pts[2].f_x - expected.f_x).abs() < 1e-9);
        assert!((pair.pts[2].f_y - expected.f_y).abs() < 1e-9);
        assert_eq!(pair.pts[0], q.f_pts[0]);
        assert_eq!(pair.pts[4], q.f_pts[2]);
    }

    #[test]
    fn chop_at_first_second_reconstruct_original_endpoints() {
        let q = quad([(0.0, 0.0), (10.0, 10.0), (20.0, 0.0)]);
        let pair = q.chop_at(0.3);
        let first = pair.first();
        let second = pair.second();
        assert_eq!(first.f_pts[0], q.f_pts[0]);
        assert_eq!(first.f_pts[2], second.f_pts[0]);
        assert_eq!(second.f_pts[2], q.f_pts[2]);
    }

    #[test]
    fn roots_real_two_roots() {
        // t^2 - 3t + 2 = 0 -> t = 1, 2
        let mut s = [0.0; 2];
        let n = SkDQuad::roots_real(1.0, -3.0, 2.0, &mut s);
        assert_eq!(n, 2);
        let mut roots = s.to_vec();
        roots.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((roots[0] - 1.0).abs() < 1e-9);
        assert!((roots[1] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn roots_real_no_real_roots() {
        // t^2 + 1 = 0 has no real roots.
        let mut s = [0.0; 2];
        let n = SkDQuad::roots_real(1.0, 0.0, 1.0, &mut s);
        assert_eq!(n, 0);
    }

    #[test]
    fn roots_valid_t_filters_to_unit_range() {
        // t^2 - 3t + 2 = 0 -> t = 1, 2: only t=1 is in [0, 1].
        let mut t = [0.0; 2];
        let n = SkDQuad::roots_valid_t(1.0, -3.0, 2.0, &mut t);
        assert_eq!(n, 1);
        assert!((t[0] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn set_abc_roundtrip() {
        // Linear ramp 0, 4, 8 has a=0 (no curvature), matching a line.
        let (a, b, c) = SkDQuad::set_abc(&[0.0, 4.0, 8.0]);
        assert!((a - 0.0).abs() < 1e-9);
        assert!((c - 8.0).abs() < 1e-9);
        let _ = b;
    }

    #[test]
    fn hull_intersects_overlapping() {
        let q1 = quad([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)]);
        let q2 = quad([(0.0, 5.0), (5.0, -5.0), (10.0, 5.0)]);
        let mut is_linear = false;
        assert!(q1.hull_intersects(&q2, &mut is_linear));
    }

    #[test]
    fn hull_intersects_disjoint() {
        let q1 = quad([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)]);
        let q2 = quad([(100.0, 100.0), (105.0, 110.0), (110.0, 100.0)]);
        let mut is_linear = false;
        assert!(!q1.hull_intersects(&q2, &mut is_linear));
    }

    #[test]
    fn dxdy_at_t_direction() {
        let q = quad([(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)]);
        let d = q.dxdy_at_t(0.5);
        // A straight horizontal quad has a purely-x derivative.
        assert!(d.f_x > 0.0);
        assert!((d.f_y).abs() < 1e-9);
    }

    #[test]
    fn indexing() {
        let q = quad([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)]);
        assert_eq!(q[0], SkDPoint::new(0.0, 0.0));
        assert_eq!(q[2], SkDPoint::new(10.0, 0.0));
    }
}
