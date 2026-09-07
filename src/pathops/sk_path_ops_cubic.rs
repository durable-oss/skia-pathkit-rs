//! Double-precision cubic Bezier curve operations for path operations.
//!
//! Port of Skia's `SkPathOpsCubic.{h,cpp}`.

use super::sk_path_ops_point::{SkDPoint, SkDVector};
use super::sk_path_ops_quad::SkDQuad;
use super::sk_path_ops_types::{
    almost_dequal_ulps, approximately_equal, approximately_one_or_less, approximately_zero,
    approximately_zero_or_more, approximately_zero_when_compared_to, between_d,
};

/// Which axis a search (e.g. [`SkDCubic::binary_search`]) operates against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchAxis {
    /// Search against x.
    XAxis,
    /// Search against y.
    YAxis,
}

/// The result of [`SkDCubic::chop_at`]: two cubics sharing the split point,
/// packed as 7 points (`[a,b,c,d,e,f,g]` where `[a,b,c,d]` is the first
/// cubic and `[d,e,f,g]` is the second).
#[derive(Debug, Clone, Copy)]
pub struct SkDCubicPair {
    /// The seven packed control points.
    pub pts: [SkDPoint; 7],
}

impl Default for SkDCubicPair {
    fn default() -> Self {
        Self {
            pts: [SkDPoint::default(); 7],
        }
    }
}

impl SkDCubicPair {
    /// The first of the two sub-cubics.
    pub fn first(&self) -> SkDCubic {
        SkDCubic::new([self.pts[0], self.pts[1], self.pts[2], self.pts[3]])
    }

    /// The second of the two sub-cubics.
    pub fn second(&self) -> SkDCubic {
        SkDCubic::new([self.pts[3], self.pts[4], self.pts[5], self.pts[6]])
    }
}

/// A double-precision cubic Bezier curve.
#[derive(Debug, Clone, Copy)]
pub struct SkDCubic {
    /// The four control points: start, two control points, end.
    pub f_pts: [SkDPoint; 4],
}

impl Default for SkDCubic {
    fn default() -> Self {
        Self {
            f_pts: [SkDPoint::default(); 4],
        }
    }
}

/// Rough scale unit used by [`SkDCubic::calc_precision`], matching Skia's
/// `SkDCubic::gPrecisionUnit`.
const PRECISION_UNIT: f64 = 256.0;

impl SkDCubic {
    /// Number of control points in a cubic.
    pub const K_POINT_COUNT: usize = 4;
    /// Index of the last control point.
    pub const K_POINT_LAST: usize = Self::K_POINT_COUNT - 1;
    /// Maximum number of intersections between two cubics.
    pub const K_MAX_INTERSECTIONS: usize = 9;

    /// Creates a cubic from its four control points.
    pub fn new(pts: [SkDPoint; 4]) -> Self {
        Self { f_pts: pts }
    }

    /// The point at index `n`.
    pub fn at(&self, n: usize) -> SkDPoint {
        self.f_pts[n]
    }

    /// True if all four control points coincide.
    pub fn collapsed(&self) -> bool {
        self.f_pts[0].approximately_equal(self.f_pts[1])
            && self.f_pts[0].approximately_equal(self.f_pts[2])
            && self.f_pts[0].approximately_equal(self.f_pts[3])
    }

    /// True if the control points lie inside the wedge formed by the
    /// endpoint tangent directions.
    pub fn controls_inside(&self) -> bool {
        let v01 = self.f_pts[0] - self.f_pts[1];
        let v02 = self.f_pts[0] - self.f_pts[2];
        let v03 = self.f_pts[0] - self.f_pts[3];
        let v13 = self.f_pts[1] - self.f_pts[3];
        let v23 = self.f_pts[2] - self.f_pts[3];
        v03.dot(v01) > 0.0 && v03.dot(v02) > 0.0 && v03.dot(v13) > 0.0 && v03.dot(v23) > 0.0
    }

    /// False: a cubic is never a conic.
    pub fn is_conic() -> bool {
        false
    }

    /// Snaps `dst_pt` to `self[ctrl_index]`'s coordinate(s) on any axis
    /// where `self[end_index]` already matches it exactly.
    pub fn align(&self, end_index: usize, ctrl_index: usize, dst_pt: &mut SkDPoint) {
        if self.f_pts[end_index].f_x == self.f_pts[ctrl_index].f_x {
            dst_pt.f_x = self.f_pts[end_index].f_x;
        }
        if self.f_pts[end_index].f_y == self.f_pts[ctrl_index].f_y {
            dst_pt.f_y = self.f_pts[end_index].f_y;
        }
    }

    /// The rough scale of the cubic (average control-polygon leg length,
    /// normalized), used to judge whether curvature is extreme.
    pub fn calc_precision(&self) -> f64 {
        ((self.f_pts[1] - self.f_pts[0]).length()
            + (self.f_pts[2] - self.f_pts[1]).length()
            + (self.f_pts[3] - self.f_pts[2]).length())
            / PRECISION_UNIT
    }

    /// Evaluates the curve at parameter `t` in `[0, 1]`.
    pub fn pt_at_t(&self, t: f64) -> SkDPoint {
        if t == 0.0 {
            return self.f_pts[0];
        }
        if t == 1.0 {
            return self.f_pts[3];
        }
        let one_t = 1.0 - t;
        let one_t2 = one_t * one_t;
        let a = one_t2 * one_t;
        let b = 3.0 * one_t2 * t;
        let t2 = t * t;
        let c = 3.0 * one_t * t2;
        let d = t2 * t;
        SkDPoint::new(
            a * self.f_pts[0].f_x + b * self.f_pts[1].f_x + c * self.f_pts[2].f_x + d * self.f_pts[3].f_x,
            a * self.f_pts[0].f_y + b * self.f_pts[1].f_y + c * self.f_pts[2].f_y + d * self.f_pts[3].f_y,
        )
    }

    /// Evaluates the curve's derivative at parameter `t`.
    pub fn dxdy_at_t(&self, t: f64) -> SkDVector {
        let mut result = SkDVector::new(
            derivative_at_t(&x_coords(self), t),
            derivative_at_t(&y_coords(self), t),
        );
        if result.f_x == 0.0 && result.f_y == 0.0 {
            if t == 0.0 {
                result = self.f_pts[2] - self.f_pts[0];
            } else if t == 1.0 {
                result = self.f_pts[3] - self.f_pts[1];
            }
            if result.f_x == 0.0 && result.f_y == 0.0 && (t == 0.0 || t == 1.0) {
                result = self.f_pts[3] - self.f_pts[0];
            }
        }
        result
    }

    /// True if the control points are monotonic (non-reversing) in x.
    pub fn monotonic_in_x(&self) -> bool {
        between_d(self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[3].f_x)
            && between_d(self.f_pts[0].f_x, self.f_pts[2].f_x, self.f_pts[3].f_x)
    }

    /// True if the control points are monotonic (non-reversing) in y.
    pub fn monotonic_in_y(&self) -> bool {
        between_d(self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[3].f_y)
            && between_d(self.f_pts[0].f_y, self.f_pts[2].f_y, self.f_pts[3].f_y)
    }

    /// True if either endpoint pair already brackets the extrema in x or y
    /// (a quick check that avoids a full extrema search in the common
    /// case).
    pub fn ends_are_extrema_in_x_or_y(&self) -> bool {
        (between_d(self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[3].f_x)
            && between_d(self.f_pts[0].f_x, self.f_pts[2].f_x, self.f_pts[3].f_x))
            || (between_d(self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[3].f_y)
                && between_d(self.f_pts[0].f_y, self.f_pts[2].f_y, self.f_pts[3].f_y))
    }

    /// Finds the `t` values (up to 2) at which the curve's curvature
    /// changes sign (inflection points), via [`SkDQuad::roots_valid_t`] on
    /// the cross product of the first and second derivative coefficients.
    pub fn find_inflections(&self) -> ([f64; 2], usize) {
        let ax = self.f_pts[1].f_x - self.f_pts[0].f_x;
        let ay = self.f_pts[1].f_y - self.f_pts[0].f_y;
        let bx = self.f_pts[2].f_x - 2.0 * self.f_pts[1].f_x + self.f_pts[0].f_x;
        let by = self.f_pts[2].f_y - 2.0 * self.f_pts[1].f_y + self.f_pts[0].f_y;
        let cx = self.f_pts[3].f_x + 3.0 * (self.f_pts[1].f_x - self.f_pts[2].f_x) - self.f_pts[0].f_x;
        let cy = self.f_pts[3].f_y + 3.0 * (self.f_pts[1].f_y - self.f_pts[2].f_y) - self.f_pts[0].f_y;
        let mut t_values = [0.0; 2];
        let n = SkDQuad::roots_valid_t(
            bx * cy - by * cx,
            ax * cy - ay * cx,
            ax * by - ay * bx,
            &mut t_values,
        );
        (t_values, n)
    }

    /// Finds the `t` values (up to 3) at which curvature is at a local
    /// extremum (`F' . F'' == 0`), used to isolate a cusp or the point of
    /// maximum bend for splitting.
    pub fn find_max_curvature(&self) -> ([f64; 3], usize) {
        let mut coeff_x = formulate_f1_dot_f2(&x_coords(self));
        let coeff_y = formulate_f1_dot_f2(&y_coords(self));
        for i in 0..4 {
            coeff_x[i] += coeff_y[i];
        }
        Self::roots_valid_t(coeff_x[0], coeff_x[1], coeff_x[2], coeff_x[3])
    }

    /// Splits the cubic into two cubics meeting at parameter `t`.
    pub fn chop_at(&self, t: f64) -> SkDCubicPair {
        if t == 0.5 {
            let p = &self.f_pts;
            let mut pts = [SkDPoint::default(); 7];
            pts[0] = p[0];
            pts[1] = SkDPoint::new((p[0].f_x + p[1].f_x) / 2.0, (p[0].f_y + p[1].f_y) / 2.0);
            pts[2] = SkDPoint::new(
                (p[0].f_x + 2.0 * p[1].f_x + p[2].f_x) / 4.0,
                (p[0].f_y + 2.0 * p[1].f_y + p[2].f_y) / 4.0,
            );
            pts[3] = SkDPoint::new(
                (p[0].f_x + 3.0 * (p[1].f_x + p[2].f_x) + p[3].f_x) / 8.0,
                (p[0].f_y + 3.0 * (p[1].f_y + p[2].f_y) + p[3].f_y) / 8.0,
            );
            pts[4] = SkDPoint::new(
                (p[1].f_x + 2.0 * p[2].f_x + p[3].f_x) / 4.0,
                (p[1].f_y + 2.0 * p[2].f_y + p[3].f_y) / 4.0,
            );
            pts[5] = SkDPoint::new((p[2].f_x + p[3].f_x) / 2.0, (p[2].f_y + p[3].f_y) / 2.0);
            pts[6] = p[3];
            return SkDCubicPair { pts };
        }
        let mut pts = [SkDPoint::default(); 7];
        let xs = interp_cubic_coords_chop(&x_coords(self), t);
        let ys = interp_cubic_coords_chop(&y_coords(self), t);
        for i in 0..7 {
            pts[i] = SkDPoint::new(xs[i], ys[i]);
        }
        SkDCubicPair { pts }
    }

    /// Splits the cubic at `t1` and `t2`, returning the sub-cubic spanning
    /// `[t1, t2]`.
    pub fn sub_divide(&self, t1: f64, t2: f64) -> Self {
        if t1 == 0.0 || t2 == 1.0 {
            if t1 == 0.0 && t2 == 1.0 {
                return *self;
            }
            let pair = self.chop_at(if t1 == 0.0 { t2 } else { t1 });
            return if t1 == 0.0 { pair.first() } else { pair.second() };
        }
        let xs = x_coords(self);
        let ys = y_coords(self);
        let ax = interp_cubic_coords(&xs, t1);
        let ay = interp_cubic_coords(&ys, t1);
        let ex = interp_cubic_coords(&xs, (t1 * 2.0 + t2) / 3.0);
        let ey = interp_cubic_coords(&ys, (t1 * 2.0 + t2) / 3.0);
        let fx = interp_cubic_coords(&xs, (t1 + t2 * 2.0) / 3.0);
        let fy = interp_cubic_coords(&ys, (t1 + t2 * 2.0) / 3.0);
        let dx = interp_cubic_coords(&xs, t2);
        let dy = interp_cubic_coords(&ys, t2);
        let mx = ex * 27.0 - ax * 8.0 - dx;
        let my = ey * 27.0 - ay * 8.0 - dy;
        let nx = fx * 27.0 - ax - dx * 8.0;
        let ny = fy * 27.0 - ay - dy * 8.0;
        Self::new([
            SkDPoint::new(ax, ay),
            SkDPoint::new((mx * 2.0 - nx) / 18.0, (my * 2.0 - ny) / 18.0),
            SkDPoint::new((nx * 2.0 - mx) / 18.0, (ny * 2.0 - my) / 18.0),
            SkDPoint::new(dx, dy),
        ])
    }

    /// Finds the real roots of the cubic `A*t^3 + B*t^2 + C*t + D == 0`
    /// via Cardano's formula, without discarding roots outside `[0, 1]`.
    pub fn roots_real(a: f64, b: f64, c: f64, d: f64) -> ([f64; 3], usize) {
        if approximately_zero(a)
            && approximately_zero_when_compared_to(a, b)
            && approximately_zero_when_compared_to(a, c)
            && approximately_zero_when_compared_to(a, d)
        {
            // Degenerates to a quadratic.
            let mut s = [0.0; 2];
            let n = SkDQuad::roots_real(b, c, d, &mut s);
            return ([s[0], s[1], 0.0], n);
        }
        if approximately_zero_when_compared_to(d, a)
            && approximately_zero_when_compared_to(d, b)
            && approximately_zero_when_compared_to(d, c)
        {
            // 0 is one root.
            let mut s2 = [0.0; 2];
            let num = SkDQuad::roots_real(a, b, c, &mut s2);
            let mut s = [s2[0], s2[1], 0.0];
            if s[..num].iter().any(|&v| approximately_zero(v)) {
                return (s, num);
            }
            s[num] = 0.0;
            return (s, num + 1);
        }
        if approximately_zero(a + b + c + d) {
            // 1 is one root.
            let mut s2 = [0.0; 2];
            let num = SkDQuad::roots_real(a, a + b, -d, &mut s2);
            let mut s = [s2[0], s2[1], 0.0];
            if s[..num].iter().any(|&v| almost_dequal_ulps(v as f32, 1.0)) {
                return (s, num);
            }
            s[num] = 1.0;
            return (s, num + 1);
        }
        cardano_roots(a, b, c, d)
    }

    /// Like [`roots_real`](Self::roots_real), but keeps only roots in (or
    /// snapped to) `[0, 1]`.
    pub fn roots_valid_t(a: f64, b: f64, c: f64, d: f64) -> ([f64; 3], usize) {
        let (s, real_roots) = Self::roots_real(a, b, c, d);
        let mut t = [0.0; 3];
        let mut found_roots = SkDQuad::add_valid_ts(&s, real_roots, &mut t);
        for &t_value in s.iter().take(real_roots) {
            if !approximately_one_or_less(t_value) && between_d(1.0, t_value, 1.00005) {
                if t[..found_roots].iter().any(|&v| approximately_equal(v, 1.0)) {
                    continue;
                }
                t[found_roots] = 1.0;
                found_roots += 1;
            } else if !approximately_zero_or_more(t_value) && between_d(-0.00005, t_value, 0.0) {
                if t[..found_roots].iter().any(|&v| approximately_equal(v, 0.0)) {
                    continue;
                }
                t[found_roots] = 0.0;
                found_roots += 1;
            }
        }
        (t, found_roots)
    }

    /// Finds the `t` values (up to 2) where the derivative is zero on the
    /// given axis (its extrema). `src` is `[start, ctrl1, ctrl2, end]` for
    /// one axis.
    pub fn find_extrema(src: &[f64; 4]) -> ([f64; 2], usize) {
        let a = src[0];
        let b = src[1];
        let c = src[2];
        let d = src[3];
        let big_a = d - a + 3.0 * (b - c);
        let big_b = 2.0 * (a - b - b + c);
        let big_c = b - a;
        let mut t_values = [0.0; 2];
        let n = SkDQuad::roots_valid_t(big_a, big_b, big_c, &mut t_values);
        (t_values, n)
    }

    /// Quick-reject test for whether this cubic's convex hull can possibly
    /// intersect the hull formed by `pts` (a quad, conic, or another
    /// cubic's control points). See [`SkDQuad::hull_intersects`] for the
    /// semantics of the return value and `is_linear`.
    pub fn hull_intersects(&self, pts: &[SkDPoint], is_linear: &mut bool) -> bool {
        let mut linear = true;
        let mut order = [0u8; 4];
        let hull_count = self.convex_hull(&mut order);
        let mut end1 = order[0] as usize;
        let mut hull_index = 0usize;
        let mut end_pt0 = self.f_pts[end1];
        loop {
            hull_index = (hull_index + 1) % hull_count;
            let end2 = order[hull_index] as usize;
            let end_pt1 = self.f_pts[end2];
            let orig_x = end_pt0.f_x;
            let orig_y = end_pt0.f_y;
            let adj = end_pt1.f_x - orig_x;
            let opp = end_pt1.f_y - orig_y;
            let odd_man_mask = other_two(end1, end2);
            let odd_man = end1 ^ odd_man_mask;
            let mut sign =
                (self.f_pts[odd_man].f_y - orig_y) * adj - (self.f_pts[odd_man].f_x - orig_x) * opp;
            let odd_man2 = end2 ^ odd_man_mask;
            let sign2 =
                (self.f_pts[odd_man2].f_y - orig_y) * adj - (self.f_pts[odd_man2].f_x - orig_x) * opp;
            if sign * sign2 < 0.0 {
                end_pt0 = end_pt1;
                end1 = end2;
                if hull_index == 0 {
                    break;
                }
                continue;
            }
            if approximately_zero(sign) {
                sign = sign2;
                if approximately_zero(sign) {
                    end_pt0 = end_pt1;
                    end1 = end2;
                    if hull_index == 0 {
                        break;
                    }
                    continue;
                }
            }
            linear = false;
            let mut found_outlier = false;
            for pt in pts {
                let test = (pt.f_y - orig_y) * adj - (pt.f_x - orig_x) * opp;
                if test * sign > 0.0 && !precisely_zero(test) {
                    found_outlier = true;
                    break;
                }
            }
            if !found_outlier {
                return false;
            }
            end_pt0 = end_pt1;
            end1 = end2;
            if hull_index == 0 {
                break;
            }
        }
        *is_linear = linear;
        true
    }

    /// [`hull_intersects`](Self::hull_intersects) against another cubic's
    /// control points.
    pub fn hull_intersects_cubic(&self, c2: &SkDCubic, is_linear: &mut bool) -> bool {
        self.hull_intersects(&c2.f_pts, is_linear)
    }

    /// [`hull_intersects`](Self::hull_intersects) against a quad's control
    /// points.
    pub fn hull_intersects_quad(&self, quad: &SkDQuad, is_linear: &mut bool) -> bool {
        self.hull_intersects(&quad.f_pts, is_linear)
    }

    /// Computes the convex hull of the cubic's 4 control points. Returns
    /// the number of hull points (3 or 4) and fills `order` with their
    /// indices in counter-clockwise order.
    pub fn convex_hull(&self, order: &mut [u8; 4]) -> usize {
        let mut y_min = 0usize;
        for index in 1..4 {
            if self.f_pts[y_min].f_y > self.f_pts[index].f_y
                || (self.f_pts[y_min].f_y == self.f_pts[index].f_y
                    && self.f_pts[y_min].f_x > self.f_pts[index].f_x)
            {
                y_min = index;
            }
        }
        order[0] = y_min as u8;

        let mut mid_x: isize = -1;
        let mut backup_y_min: isize = -1;

        for _pass in 0..2 {
            for index in 0..4 {
                if index == y_min {
                    continue;
                }
                let mask = other_two(y_min, index);
                let side1 = y_min ^ mask;
                let side2 = index ^ mask;
                let mut rot_path = SkDCubic::default();

                if !self.rotate(y_min, index, &mut rot_path) {
                    order[1] = side1 as u8;
                    order[2] = side2 as u8;
                    return 3;
                }

                let sides = side(rot_path.at(side1).f_y - rot_path.at(y_min).f_y)
                    ^ side(rot_path.at(side2).f_y - rot_path.at(y_min).f_y);

                if sides == 2 {
                    if mid_x >= 0 {
                        order[0] = 0;
                        order[1] = 3;

                        if self.f_pts[1].approximately_zero_or_equal(self.f_pts[0])
                            || self.f_pts[1].approximately_zero_or_equal(self.f_pts[3])
                        {
                            order[2] = 2;
                            return 3;
                        }
                        if self.f_pts[2].approximately_zero_or_equal(self.f_pts[0])
                            || self.f_pts[2].approximately_zero_or_equal(self.f_pts[3])
                        {
                            order[2] = 1;
                            return 3;
                        }

                        let dist1_0 = self.f_pts[1].distance_squared(self.f_pts[0]);
                        let dist1_3 = self.f_pts[1].distance_squared(self.f_pts[3]);
                        let dist2_0 = self.f_pts[2].distance_squared(self.f_pts[0]);
                        let dist2_3 = self.f_pts[2].distance_squared(self.f_pts[3]);

                        let smallest1 = dist1_0.min(dist1_3);
                        let smallest2 = dist2_0.min(dist2_3);

                        if approximately_zero(smallest1.min(smallest2)) {
                            order[2] = if smallest1 < smallest2 { 2 } else { 1 };
                            return 3;
                        }
                    }
                    mid_x = index as isize;
                } else if sides == 0 {
                    backup_y_min = index as isize;
                }
            }

            if mid_x >= 0 {
                break;
            }
            if backup_y_min < 0 {
                break;
            }
            y_min = backup_y_min as usize;
            backup_y_min = -1;
        }

        if mid_x < 0 {
            mid_x = (y_min ^ 3) as isize;
        }

        let mask = other_two(y_min, mid_x as usize);
        let least = y_min ^ mask;
        let most = mid_x as usize ^ mask;

        order[0] = y_min as u8;
        order[1] = least as u8;

        let mut mid_path = SkDCubic::default();
        if !self.rotate(least, most, &mut mid_path) {
            order[2] = mid_x as u8;
            return 3;
        }

        let mid_sides = side(mid_path.at(y_min).f_y - mid_path.at(least).f_y)
            ^ side(mid_path.at(mid_x as usize).f_y - mid_path.at(least).f_y);

        if mid_sides != 2 {
            order[2] = most as u8;
            return 3;
        }

        order[2] = mid_x as u8;
        order[3] = most as u8;
        4
    }

    /// Rotates the cubic so the line from `self[zero]` to `self[index]`
    /// aligns with an axis, writing the result to `rot`. Returns `false`
    /// if the two points are coincident (rotation undefined).
    fn rotate(&self, zero: usize, index: usize, rot: &mut SkDCubic) -> bool {
        let dy = self.f_pts[index].f_y - self.f_pts[zero].f_y;
        let dx = self.f_pts[index].f_x - self.f_pts[zero].f_x;

        if approximately_zero(dy) {
            if approximately_zero(dx) {
                return false;
            }
            *rot = *self;
            let mask = other_two(index, zero);
            let side1 = index ^ mask;
            let side2 = zero ^ mask;
            if approximately_equal(self.f_pts[side1].f_y, self.f_pts[zero].f_y) {
                rot.f_pts[side1].f_y = self.f_pts[zero].f_y;
            }
            if approximately_equal(self.f_pts[side2].f_y, self.f_pts[zero].f_y) {
                rot.f_pts[side2].f_y = self.f_pts[zero].f_y;
            }
            return true;
        }

        for i in 0..4 {
            rot.f_pts[i].f_x = self.f_pts[i].f_x * dx + self.f_pts[i].f_y * dy;
            rot.f_pts[i].f_y = self.f_pts[i].f_y * dx - self.f_pts[i].f_x * dy;
        }
        true
    }
}

impl std::ops::Index<usize> for SkDCubic {
    type Output = SkDPoint;
    fn index(&self, index: usize) -> &Self::Output {
        &self.f_pts[index]
    }
}

impl std::ops::IndexMut<usize> for SkDCubic {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.f_pts[index]
    }
}

fn x_coords(c: &SkDCubic) -> [f64; 4] {
    [c.f_pts[0].f_x, c.f_pts[1].f_x, c.f_pts[2].f_x, c.f_pts[3].f_x]
}

fn y_coords(c: &SkDCubic) -> [f64; 4] {
    [c.f_pts[0].f_y, c.f_pts[1].f_y, c.f_pts[2].f_y, c.f_pts[3].f_y]
}

/// `c'(t) = 3[(b-a)(1-t)^2 + 2(c-b)t(1-t) + (d-c)t^2]`, evaluated for one
/// axis's control values `[a, b, c, d]`.
fn derivative_at_t(src: &[f64; 4], t: f64) -> f64 {
    let one_t = 1.0 - t;
    let a = src[0];
    let b = src[1];
    let c = src[2];
    let d = src[3];
    3.0 * ((b - a) * one_t * one_t + 2.0 * (c - b) * t * one_t + (d - c) * t * t)
}

/// Coefficients of `F'(t) . F''(t)` (a cubic in `t`) for one axis, used by
/// [`SkDCubic::find_max_curvature`].
fn formulate_f1_dot_f2(src: &[f64; 4]) -> [f64; 4] {
    let a = src[1] - src[0];
    let b = src[2] - 2.0 * src[1] + src[0];
    let c = src[3] + 3.0 * (src[1] - src[2]) - src[0];
    [c * c, 3.0 * b * c, 2.0 * b * b + c * a, a * b]
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// One level of De Casteljau subdivision for a cubic's single axis,
/// returning just the point at `t` (used by [`SkDCubic::sub_divide`]).
fn interp_cubic_coords(src: &[f64; 4], t: f64) -> f64 {
    let ab = lerp(src[0], src[1], t);
    let bc = lerp(src[1], src[2], t);
    let cd = lerp(src[2], src[3], t);
    let abc = lerp(ab, bc, t);
    let bcd = lerp(bc, cd, t);
    lerp(abc, bcd, t)
}

/// Full De Casteljau ladder for a cubic's single axis at `t`, returning
/// all 7 packed values `chop_at` needs to build both sub-cubics.
fn interp_cubic_coords_chop(src: &[f64; 4], t: f64) -> [f64; 7] {
    let ab = lerp(src[0], src[1], t);
    let bc = lerp(src[1], src[2], t);
    let cd = lerp(src[2], src[3], t);
    let abc = lerp(ab, bc, t);
    let bcd = lerp(bc, cd, t);
    let abcd = lerp(abc, bcd, t);
    [src[0], ab, abc, abcd, bcd, cd, src[3]]
}

fn precisely_zero(x: f64) -> bool {
    super::sk_path_ops_types::precisely_zero(x)
}

/// Returns the other two indices given two of `{0, 1, 2, 3}` (XOR trick;
/// see the C++ comment this is ported from in `SkPathOpsCubic.h`).
pub const fn other_two(one: usize, two: usize) -> usize {
    1 >> (3 - (one ^ two)) ^ 3
}

/// 0 if negative, 1 if zero, 2 if positive.
#[inline]
fn side(x: f64) -> usize {
    usize::from(x > 0.0) + usize::from(x >= 0.0)
}

const PI: f64 = std::f64::consts::PI;

/// Solves `A*t^3 + B*t^2 + C*t + D == 0` for real roots via Cardano's
/// formula (trigonometric form when there are 3 real roots, otherwise the
/// single-real-root form), ported from `SkDCubic::RootsReal`.
fn cardano_roots(mut big_a: f64, b: f64, c: f64, d: f64) -> ([f64; 3], usize) {
    let inv_a = 1.0 / big_a;
    let a = b * inv_a;
    let b_ = c * inv_a;
    let c_ = d * inv_a;
    let a2 = a * a;
    let q = (a2 - b_ * 3.0) / 9.0;
    let r = (2.0 * a2 * a - 9.0 * a * b_ + 27.0 * c_) / 54.0;
    let r2 = r * r;
    let q3 = q * q * q;
    let r2_minus_q3 = r2 - q3;
    let adiv3 = a / 3.0;
    let mut s = [0.0; 3];
    let mut n = 0usize;

    if r2_minus_q3 < 0.0 {
        // Three real roots.
        let theta = (r / q3.sqrt()).clamp(-1.0, 1.0).acos();
        let neg2_root_q = -2.0 * q.sqrt();

        let root0 = neg2_root_q * (theta / 3.0).cos() - adiv3;
        s[n] = root0;
        n += 1;

        let root1 = neg2_root_q * ((theta + 2.0 * PI) / 3.0).cos() - adiv3;
        if !almost_dequal_ulps(s[0] as f32, root1 as f32) {
            s[n] = root1;
            n += 1;
        }
        let root2 = neg2_root_q * ((theta - 2.0 * PI) / 3.0).cos() - adiv3;
        if !almost_dequal_ulps(s[0] as f32, root2 as f32)
            && (n == 1 || !almost_dequal_ulps(s[1] as f32, root2 as f32))
        {
            s[n] = root2;
            n += 1;
        }
    } else {
        // One real root.
        let sqrt_r2_minus_q3 = r2_minus_q3.sqrt();
        let mut aa = r.abs() + sqrt_r2_minus_q3;
        aa = aa.cbrt();
        if r > 0.0 {
            aa = -aa;
        }
        if aa != 0.0 {
            aa += q / aa;
        }
        big_a = aa;
        let root = aa - adiv3;
        s[n] = root;
        n += 1;
        if almost_dequal_ulps(r2 as f32, q3 as f32) {
            let root2 = -big_a / 2.0 - adiv3;
            if !almost_dequal_ulps(s[0] as f32, root2 as f32) {
                s[n] = root2;
                n += 1;
            }
        }
    }
    (s, n)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cubic(pts: [(f64, f64); 4]) -> SkDCubic {
        SkDCubic::new([
            SkDPoint::new(pts[0].0, pts[0].1),
            SkDPoint::new(pts[1].0, pts[1].1),
            SkDPoint::new(pts[2].0, pts[2].1),
            SkDPoint::new(pts[3].0, pts[3].1),
        ])
    }

    #[test]
    fn collapsed() {
        let c = cubic([(1.0, 1.0), (1.0, 1.0), (1.0, 1.0), (1.0, 1.0)]);
        assert!(c.collapsed());
        let c2 = cubic([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]);
        assert!(!c2.collapsed());
    }

    #[test]
    fn pt_at_t_endpoints() {
        let c = cubic([(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]);
        assert_eq!(c.pt_at_t(0.0), c.f_pts[0]);
        assert_eq!(c.pt_at_t(1.0), c.f_pts[3]);
    }

    #[test]
    fn monotonic() {
        let c = cubic([(0.0, 0.0), (3.0, 3.0), (7.0, 7.0), (10.0, 10.0)]);
        assert!(c.monotonic_in_x());
        assert!(c.monotonic_in_y());

        let c2 = cubic([(0.0, 0.0), (75.0, 300.0), (225.0, -300.0), (300.0, 0.0)]);
        assert!(!c2.monotonic_in_y());
    }

    #[test]
    fn chop_at_half_matches_pt_at_t() {
        let c = cubic([(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]);
        let pair = c.chop_at(0.5);
        let expected = c.pt_at_t(0.5);
        assert!((pair.pts[3].f_x - expected.f_x).abs() < 1e-9);
        assert!((pair.pts[3].f_y - expected.f_y).abs() < 1e-9);
    }

    #[test]
    fn chop_at_generic_matches_pt_at_t() {
        let c = cubic([(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]);
        let pair = c.chop_at(0.3);
        let expected = c.pt_at_t(0.3);
        assert!((pair.pts[3].f_x - expected.f_x).abs() < 1e-9);
        assert!((pair.pts[3].f_y - expected.f_y).abs() < 1e-9);
        assert_eq!(pair.pts[0], c.f_pts[0]);
        assert_eq!(pair.pts[6], c.f_pts[3]);
    }

    #[test]
    fn chop_at_first_second_share_split_point() {
        let c = cubic([(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]);
        let pair = c.chop_at(0.4);
        let first = pair.first();
        let second = pair.second();
        assert_eq!(first.f_pts[3], second.f_pts[0]);
        assert_eq!(first.f_pts[0], c.f_pts[0]);
        assert_eq!(second.f_pts[3], c.f_pts[3]);
    }

    #[test]
    fn sub_divide_identity() {
        let c = cubic([(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]);
        let s = c.sub_divide(0.0, 1.0);
        assert_eq!(s.f_pts[0], c.f_pts[0]);
        assert_eq!(s.f_pts[3], c.f_pts[3]);
    }

    #[test]
    fn sub_divide_matches_chop_at_endpoints() {
        let c = cubic([(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]);
        let s = c.sub_divide(0.2, 0.8);
        let p20 = c.pt_at_t(0.2);
        let p80 = c.pt_at_t(0.8);
        assert!((s.f_pts[0].f_x - p20.f_x).abs() < 1e-6);
        assert!((s.f_pts[0].f_y - p20.f_y).abs() < 1e-6);
        assert!((s.f_pts[3].f_x - p80.f_x).abs() < 1e-6);
        assert!((s.f_pts[3].f_y - p80.f_y).abs() < 1e-6);
    }

    #[test]
    fn roots_real_three_roots() {
        // (t-1)(t-2)(t-3) = t^3 - 6t^2 + 11t - 6
        let (roots, n) = SkDCubic::roots_real(1.0, -6.0, 11.0, -6.0);
        assert_eq!(n, 3);
        let mut sorted = roots[..n].to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((sorted[0] - 1.0).abs() < 1e-6);
        assert!((sorted[1] - 2.0).abs() < 1e-6);
        assert!((sorted[2] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn roots_real_one_root() {
        // t^3 - 1 = 0 has one real root (t=1) and two complex.
        let (roots, n) = SkDCubic::roots_real(1.0, 0.0, 0.0, -1.0);
        assert_eq!(n, 1);
        assert!((roots[0] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn roots_real_degenerates_to_quadratic() {
        // a == 0: t^2 - 3t + 2 = 0 -> t = 1, 2
        let (roots, n) = SkDCubic::roots_real(0.0, 1.0, -3.0, 2.0);
        assert_eq!(n, 2);
        let mut sorted = roots[..n].to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((sorted[0] - 1.0).abs() < 1e-9);
        assert!((sorted[1] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn find_extrema_symmetric_bump() {
        let c = cubic([(0.0, 0.0), (75.0, 300.0), (225.0, -300.0), (300.0, 0.0)]);
        let (t_values, n) = SkDCubic::find_extrema(&y_coords(&c));
        assert_eq!(n, 2);
        for &t in &t_values[..n] {
            assert!(t > 0.0 && t < 1.0);
        }
    }

    #[test]
    fn find_inflections_s_curve() {
        // An S-curve has exactly one inflection point.
        let c = cubic([(0.0, 0.0), (10.0, 0.0), (0.0, 10.0), (10.0, 10.0)]);
        let (_t_values, n) = c.find_inflections();
        assert!(n <= 2);
    }

    #[test]
    fn convex_hull_quadrilateral() {
        let c = cubic([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)]);
        let mut order = [0u8; 4];
        let result = c.convex_hull(&mut order);
        assert_eq!(result, 4);
    }

    #[test]
    fn convex_hull_degenerate_point() {
        let c = cubic([(1.0, 1.0), (1.0, 1.0), (1.0, 1.0), (1.0, 1.0)]);
        let mut order = [0u8; 4];
        let result = c.convex_hull(&mut order);
        assert_eq!(result, 3);
    }

    #[test]
    fn hull_intersects_overlapping_cubics() {
        let c1 = cubic([(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]);
        let c2 = cubic([(0.0, 5.0), (5.0, -5.0), (5.0, 15.0), (10.0, 5.0)]);
        let mut is_linear = false;
        assert!(c1.hull_intersects_cubic(&c2, &mut is_linear));
    }

    #[test]
    fn hull_intersects_disjoint_cubics() {
        let c1 = cubic([(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]);
        let c2 = cubic([(100.0, 100.0), (100.0, 110.0), (110.0, 110.0), (110.0, 100.0)]);
        let mut is_linear = false;
        assert!(!c1.hull_intersects_cubic(&c2, &mut is_linear));
    }

    #[test]
    fn indexing() {
        let c = cubic([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]);
        assert_eq!(c[0], SkDPoint::new(0.0, 0.0));
        assert_eq!(c[3], SkDPoint::new(3.0, 3.0));
    }

    #[test]
    fn calc_precision_nonzero_for_nondegenerate() {
        let c = cubic([(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]);
        assert!(c.calc_precision() > 0.0);
    }
}
