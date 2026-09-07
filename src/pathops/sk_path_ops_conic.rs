//! Double-precision conic (rational quadratic) curve operations for path
//! operations.
//!
//! Port of Skia's `SkPathOpsConic.{h,cpp}`. A conic shares its control
//! point storage and most predicates with [`SkDQuad`] (Skia's C++ literally
//! embeds an `SkDQuad` and delegates); only the weight-dependent evaluation
//! (`pt_at_t`, `dxdy_at_t`, `sub_divide`, `find_extrema`) has conic-specific
//! math.

use super::sk_path_ops_point::{SkDPoint, SkDVector};
use super::sk_path_ops_quad::SkDQuad;

/// A double-precision conic (rational quadratic Bezier) curve: a quad plus
/// a weight controlling how strongly the curve bends toward the control
/// point (`weight == 1` is exactly a parabola/quad).
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDConic {
    /// The three control points and their quad-shared predicates.
    pub f_pts: SkDQuad,
    /// The rational weight.
    pub f_weight: f64,
}

impl SkDConic {
    /// Number of control points in a conic.
    pub const K_POINT_COUNT: usize = 3;
    /// Index of the last control point.
    pub const K_POINT_LAST: usize = Self::K_POINT_COUNT - 1;
    /// Maximum number of intersections between two conics.
    pub const K_MAX_INTERSECTIONS: usize = 4;

    /// Creates a conic from its three control points and weight.
    pub fn new(pts: [SkDPoint; 3], weight: f64) -> Self {
        Self {
            f_pts: SkDQuad::new(pts),
            f_weight: weight,
        }
    }

    /// True: a conic is always a conic.
    pub fn is_conic() -> bool {
        true
    }

    /// True if all three control points coincide.
    pub fn collapsed(&self) -> bool {
        self.f_pts.collapsed()
    }

    /// True if the control point lies inside the wedge formed by the two
    /// endpoint tangent directions.
    pub fn controls_inside(&self) -> bool {
        self.f_pts.controls_inside()
    }

    /// Returns this conic with its point order reversed.
    pub fn flip(&self) -> Self {
        Self {
            f_pts: self.f_pts.flip(),
            f_weight: self.f_weight,
        }
    }

    /// True if the control points are monotonic (non-reversing) in x.
    pub fn monotonic_in_x(&self) -> bool {
        self.f_pts.monotonic_in_x()
    }

    /// True if the control points are monotonic (non-reversing) in y.
    pub fn monotonic_in_y(&self) -> bool {
        self.f_pts.monotonic_in_y()
    }

    /// Quick-reject test for whether this conic's hull can possibly
    /// intersect `quad`'s. See [`SkDQuad::hull_intersects`].
    pub fn hull_intersects_quad(&self, quad: &SkDQuad, is_linear: &mut bool) -> bool {
        self.f_pts.hull_intersects(quad, is_linear)
    }

    /// Quick-reject test for whether this conic's hull can possibly
    /// intersect `conic`'s.
    pub fn hull_intersects_conic(&self, conic: &SkDConic, is_linear: &mut bool) -> bool {
        self.f_pts.hull_intersects(&conic.f_pts, is_linear)
    }

    /// The derivative coefficients `[A, B, C]` of `d/dt` for one axis of
    /// the rational quadratic, matching Skia's `conic_deriv_coeff`.
    fn deriv_coeff(src: &[f64; 3], w: f64) -> [f64; 3] {
        let p20 = src[2] - src[0];
        let p10 = src[1] - src[0];
        let w_p10 = w * p10;
        [w * p20 - p20, p20 - 2.0 * w_p10, w_p10]
    }

    fn eval_tan(coord: &[f64; 3], w: f64, t: f64) -> f64 {
        let coeff = Self::deriv_coeff(coord, w);
        t * (t * coeff[0] + coeff[1]) + coeff[2]
    }

    /// Finds the `t` value (0 or 1 root) where the derivative is zero on
    /// the given axis (its extremum), if any, over `(0, 1)`.
    pub fn find_extrema(src: &[f64; 3], weight: f64) -> Option<f64> {
        let coeff = Self::deriv_coeff(src, weight);
        let mut t_values = [0.0; 2];
        let roots = SkDQuad::roots_valid_t(coeff[0], coeff[1], coeff[2], &mut t_values);
        if roots == 1 {
            Some(t_values[0])
        } else {
            None
        }
    }

    /// Evaluates the curve's derivative at parameter `t`.
    pub fn dxdy_at_t(&self, t: f64) -> SkDVector {
        let xs = [self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[2].f_x];
        let ys = [self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[2].f_y];
        let mut result = SkDVector::new(
            Self::eval_tan(&xs, self.f_weight, t),
            Self::eval_tan(&ys, self.f_weight, t),
        );
        if result.f_x == 0.0 && result.f_y == 0.0 && (t == 0.0 || t == 1.0) {
            result = self.f_pts[2] - self.f_pts[0];
        }
        result
    }

    fn eval_numerator(src: &[f64; 3], w: f64, t: f64) -> f64 {
        let src2w = src[1] * w;
        let c = src[0];
        let a = src[2] - 2.0 * src2w + c;
        let b = 2.0 * (src2w - c);
        (a * t + b) * t + c
    }

    fn eval_denominator(w: f64, t: f64) -> f64 {
        let b = 2.0 * (w - 1.0);
        let c = 1.0;
        let a = -b;
        (a * t + b) * t + c
    }

    /// Evaluates the curve at parameter `t` in `[0, 1]`.
    pub fn pt_at_t(&self, t: f64) -> SkDPoint {
        if t == 0.0 {
            return self.f_pts[0];
        }
        if t == 1.0 {
            return self.f_pts[2];
        }
        let xs = [self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[2].f_x];
        let ys = [self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[2].f_y];
        let denom = Self::eval_denominator(self.f_weight, t);
        SkDPoint::new(
            Self::eval_numerator(&xs, self.f_weight, t) / denom,
            Self::eval_numerator(&ys, self.f_weight, t) / denom,
        )
    }

    /// Splits the conic at `t1` and `t2`, returning the sub-conic spanning
    /// `[t1, t2]` (including its new weight).
    pub fn sub_divide(&self, t1: f64, t2: f64) -> Self {
        let xs = [self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[2].f_x];
        let ys = [self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[2].f_y];

        let (ax, ay, az) = if t1 == 0.0 {
            (self.f_pts[0].f_x, self.f_pts[0].f_y, 1.0)
        } else if t1 != 1.0 {
            (
                Self::eval_numerator(&xs, self.f_weight, t1),
                Self::eval_numerator(&ys, self.f_weight, t1),
                Self::eval_denominator(self.f_weight, t1),
            )
        } else {
            (self.f_pts[2].f_x, self.f_pts[2].f_y, 1.0)
        };

        let mid_t = (t1 + t2) / 2.0;
        let dx = Self::eval_numerator(&xs, self.f_weight, mid_t);
        let dy = Self::eval_numerator(&ys, self.f_weight, mid_t);
        let dz = Self::eval_denominator(self.f_weight, mid_t);

        let (cx, cy, cz) = if t2 == 1.0 {
            (self.f_pts[2].f_x, self.f_pts[2].f_y, 1.0)
        } else if t2 != 0.0 {
            (
                Self::eval_numerator(&xs, self.f_weight, t2),
                Self::eval_numerator(&ys, self.f_weight, t2),
                Self::eval_denominator(self.f_weight, t2),
            )
        } else {
            (self.f_pts[0].f_x, self.f_pts[0].f_y, 1.0)
        };

        let bx = 2.0 * dx - (ax + cx) / 2.0;
        let by = 2.0 * dy - (ay + cy) / 2.0;
        let mut bz = 2.0 * dz - (az + cz) / 2.0;
        if bz == 0.0 {
            bz = 1.0;
        }

        Self {
            f_pts: SkDQuad::new([
                SkDPoint::new(ax / az, ay / az),
                SkDPoint::new(bx / bz, by / bz),
                SkDPoint::new(cx / cz, cy / cz),
            ]),
            f_weight: bz / (az * cz).sqrt(),
        }
    }
}

impl std::ops::Index<usize> for SkDConic {
    type Output = SkDPoint;
    fn index(&self, index: usize) -> &Self::Output {
        &self.f_pts[index]
    }
}

impl std::ops::IndexMut<usize> for SkDConic {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.f_pts[index]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn conic(pts: [(f64, f64); 3], weight: f64) -> SkDConic {
        SkDConic::new(
            [
                SkDPoint::new(pts[0].0, pts[0].1),
                SkDPoint::new(pts[1].0, pts[1].1),
                SkDPoint::new(pts[2].0, pts[2].1),
            ],
            weight,
        )
    }

    #[test]
    fn weight_one_matches_quad_at_endpoints() {
        let c = conic([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)], 1.0);
        let q = SkDQuad::new(c.f_pts.f_pts);
        assert_eq!(c.pt_at_t(0.0), q.pt_at_t(0.0));
        assert_eq!(c.pt_at_t(1.0), q.pt_at_t(1.0));
    }

    #[test]
    fn weight_one_matches_quad_at_midpoint() {
        let c = conic([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)], 1.0);
        let q = SkDQuad::new(c.f_pts.f_pts);
        let cm = c.pt_at_t(0.5);
        let qm = q.pt_at_t(0.5);
        assert!((cm.f_x - qm.f_x).abs() < 1e-9);
        assert!((cm.f_y - qm.f_y).abs() < 1e-9);
    }

    #[test]
    fn pt_at_t_endpoints() {
        let c = conic([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)], 0.5);
        assert_eq!(c.pt_at_t(0.0), c.f_pts[0]);
        assert_eq!(c.pt_at_t(1.0), c.f_pts[2]);
    }

    #[test]
    fn sub_divide_identity_endpoints() {
        let c = conic([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)], 0.7);
        let s = c.sub_divide(0.0, 1.0);
        assert!((s.f_pts[0].f_x - c.f_pts[0].f_x).abs() < 1e-9);
        assert!((s.f_pts[2].f_x - c.f_pts[2].f_x).abs() < 1e-9);
    }

    #[test]
    fn sub_divide_endpoints_match_pt_at_t() {
        let c = conic([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)], 0.5);
        let s = c.sub_divide(0.2, 0.8);
        let p20 = c.pt_at_t(0.2);
        let p80 = c.pt_at_t(0.8);
        assert!((s.f_pts[0].f_x - p20.f_x).abs() < 1e-6);
        assert!((s.f_pts[0].f_y - p20.f_y).abs() < 1e-6);
        assert!((s.f_pts[2].f_x - p80.f_x).abs() < 1e-6);
        assert!((s.f_pts[2].f_y - p80.f_y).abs() < 1e-6);
    }

    #[test]
    fn find_extrema_symmetric_bump() {
        let t = SkDConic::find_extrema(&[0.0, 10.0, 0.0], 1.0);
        assert!(t.is_some());
        assert!((t.unwrap() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn find_extrema_monotonic_is_none() {
        let t = SkDConic::find_extrema(&[0.0, 5.0, 10.0], 1.0);
        assert!(t.is_none());
    }

    #[test]
    fn monotonic() {
        let c = conic([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)], 1.0);
        assert!(c.monotonic_in_x());
        assert!(!c.monotonic_in_y());
    }

    #[test]
    fn flip() {
        let c = conic([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)], 0.5);
        let f = c.flip();
        assert_eq!(f.f_pts[0], c.f_pts[2]);
        assert_eq!(f.f_pts[2], c.f_pts[0]);
        assert_eq!(f.f_weight, c.f_weight);
    }

    #[test]
    fn indexing() {
        let c = conic([(0.0, 0.0), (5.0, 10.0), (10.0, 0.0)], 0.5);
        assert_eq!(c[0], SkDPoint::new(0.0, 0.0));
        assert_eq!(c[2], SkDPoint::new(10.0, 0.0));
    }
}
