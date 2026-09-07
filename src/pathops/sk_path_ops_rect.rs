//! Double-precision rectangle utilities for path operations.
//!
//! Port of Skia's `SkPathOpsRect.{h,cpp}`. Computes tight (curve-aware)
//! bounding boxes for quad/conic/cubic curves by combining the endpoints
//! with the curve's x/y extrema, rather than just the convex hull of its
//! control points.

use super::sk_path_ops_conic::SkDConic;
use super::sk_path_ops_cubic::SkDCubic;
use super::sk_path_ops_point::SkDPoint;
use super::sk_path_ops_quad::SkDQuad;
use super::sk_path_ops_types::between as approximately_between_f32;

/// A double-precision rectangle representing a bounding box.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDRect {
    /// Left edge.
    pub f_left: f64,
    /// Top edge.
    pub f_top: f64,
    /// Right edge.
    pub f_right: f64,
    /// Bottom edge.
    pub f_bottom: f64,
}

impl SkDRect {
    /// Creates a new rectangle with explicit bounds.
    pub fn new(left: f64, top: f64, right: f64, bottom: f64) -> Self {
        Self {
            f_left: left,
            f_top: top,
            f_right: right,
            f_bottom: bottom,
        }
    }

    /// Creates an empty rectangle (its bounds are "inside out", so the
    /// first [`add`](Self::add) establishes real bounds).
    pub fn empty() -> Self {
        Self {
            f_left: f64::MAX,
            f_top: f64::MAX,
            f_right: f64::MIN,
            f_bottom: f64::MIN,
        }
    }

    /// Grows the rectangle's bounds to include `pt`.
    pub fn add(&mut self, pt: SkDPoint) {
        self.f_left = self.f_left.min(pt.f_x);
        self.f_top = self.f_top.min(pt.f_y);
        self.f_right = self.f_right.max(pt.f_x);
        self.f_bottom = self.f_bottom.max(pt.f_y);
    }

    /// Sets the rectangle to encompass exactly `pt`.
    pub fn set(&mut self, pt: SkDPoint) {
        self.f_left = pt.f_x;
        self.f_right = pt.f_x;
        self.f_top = pt.f_y;
        self.f_bottom = pt.f_y;
    }

    /// True if `pt` lies within the rectangle's bounds (within
    /// floating-point tolerance).
    pub fn contains(&self, pt: SkDPoint) -> bool {
        approximately_between(self.f_left, pt.f_x, self.f_right)
            && approximately_between(self.f_top, pt.f_y, self.f_bottom)
    }

    /// True if this rectangle and `r` overlap or touch.
    pub fn intersects(&self, r: &SkDRect) -> bool {
        r.f_left <= self.f_right
            && self.f_left <= r.f_right
            && r.f_top <= self.f_bottom
            && self.f_top <= r.f_bottom
    }

    /// The rectangle's width.
    pub fn width(&self) -> f64 {
        self.f_right - self.f_left
    }

    /// The rectangle's height.
    pub fn height(&self) -> f64 {
        self.f_bottom - self.f_top
    }

    /// True if the rectangle is well-formed (`left <= right`, `top <= bottom`).
    pub fn valid(&self) -> bool {
        self.f_left <= self.f_right && self.f_top <= self.f_bottom
    }

    /// Computes the tight bounding box of `curve` over its full `[0, 1]` range.
    pub fn set_bounds_quad(curve: &SkDQuad) -> Self {
        Self::set_bounds_quad_range(curve, curve, 0.0, 1.0)
    }

    /// Computes the tight bounding box of `curve` restricted to the
    /// sub-range `[start_t, end_t]`, where `sub` is `curve` already chopped
    /// to that range (so its own extrema are found in local `[0, 1]` terms
    /// and rescaled by the caller).
    pub fn set_bounds_quad_range(curve: &SkDQuad, sub: &SkDQuad, start_t: f64, end_t: f64) -> Self {
        let mut rect = SkDRect::default();
        rect.set(sub[0]);
        rect.add(sub[2]);

        let mut t_values = [0.0; 2];
        let mut roots = 0;

        if !sub.monotonic_in_x() {
            if let Some(t) = SkDQuad::find_extrema(&[sub[0].f_x, sub[1].f_x, sub[2].f_x]) {
                t_values[roots] = t;
                roots += 1;
            }
        }
        if !sub.monotonic_in_y() {
            if let Some(t) = SkDQuad::find_extrema(&[sub[0].f_y, sub[1].f_y, sub[2].f_y]) {
                t_values[roots] = t;
                roots += 1;
            }
        }

        for &t_value in t_values.iter().take(roots) {
            let t = start_t + (end_t - start_t) * t_value;
            rect.add(curve.pt_at_t(t));
        }
        rect
    }

    /// Computes the tight bounding box of `curve` over its full `[0, 1]` range.
    pub fn set_bounds_conic(curve: &SkDConic) -> Self {
        Self::set_bounds_conic_range(curve, curve, 0.0, 1.0)
    }

    /// Computes the tight bounding box of `curve` restricted to the
    /// sub-range `[start_t, end_t]`; see [`set_bounds_quad_range`](Self::set_bounds_quad_range).
    pub fn set_bounds_conic_range(
        curve: &SkDConic,
        sub: &SkDConic,
        start_t: f64,
        end_t: f64,
    ) -> Self {
        let mut rect = SkDRect::default();
        rect.set(sub[0]);
        rect.add(sub[2]);

        let mut t_values = [0.0; 2];
        let mut roots = 0;

        if !sub.monotonic_in_x() {
            if let Some(t) = SkDConic::find_extrema(&[sub[0].f_x, sub[1].f_x, sub[2].f_x], sub.f_weight) {
                t_values[roots] = t;
                roots += 1;
            }
        }
        if !sub.monotonic_in_y() {
            if let Some(t) = SkDConic::find_extrema(&[sub[0].f_y, sub[1].f_y, sub[2].f_y], sub.f_weight) {
                t_values[roots] = t;
                roots += 1;
            }
        }

        for &t_value in t_values.iter().take(roots) {
            let t = start_t + (end_t - start_t) * t_value;
            rect.add(curve.pt_at_t(t));
        }
        rect
    }

    /// Computes the tight bounding box of `curve` over its full `[0, 1]` range.
    pub fn set_bounds_cubic(curve: &SkDCubic) -> Self {
        Self::set_bounds_cubic_range(curve, curve, 0.0, 1.0)
    }

    /// Computes the tight bounding box of `curve` restricted to the
    /// sub-range `[start_t, end_t]`; see [`set_bounds_quad_range`](Self::set_bounds_quad_range).
    pub fn set_bounds_cubic_range(
        curve: &SkDCubic,
        sub: &SkDCubic,
        start_t: f64,
        end_t: f64,
    ) -> Self {
        let mut rect = SkDRect::default();
        rect.set(sub[0]);
        rect.add(sub[3]);

        let mut t_values = [0.0; 4];
        let mut roots = 0;

        if !sub.monotonic_in_x() {
            let (xs, n) = SkDCubic::find_extrema(&[sub[0].f_x, sub[1].f_x, sub[2].f_x, sub[3].f_x]);
            t_values[roots..roots + n].copy_from_slice(&xs[..n]);
            roots += n;
        }
        if !sub.monotonic_in_y() {
            let (ys, n) = SkDCubic::find_extrema(&[sub[0].f_y, sub[1].f_y, sub[2].f_y, sub[3].f_y]);
            t_values[roots..roots + n].copy_from_slice(&ys[..n]);
            roots += n;
        }

        for &t_value in t_values.iter().take(roots) {
            let t = start_t + (end_t - start_t) * t_value;
            rect.add(curve.pt_at_t(t));
        }
        rect
    }
}

fn approximately_between(a: f64, b: f64, c: f64) -> bool {
    approximately_between_f32(a as f32, b as f32, c as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rect_empty() {
        let rect = SkDRect::empty();
        assert!(rect.f_left == f64::MAX);
        assert!(rect.f_right == f64::MIN);
    }

    #[test]
    fn rect_add() {
        let mut rect = SkDRect::empty();
        rect.add(SkDPoint::new(0.0, 0.0));
        rect.add(SkDPoint::new(10.0, 10.0));
        assert!((rect.f_left - 0.0).abs() < 1e-10);
        assert!((rect.f_right - 10.0).abs() < 1e-10);
    }

    #[test]
    fn rect_contains() {
        let rect = SkDRect::new(0.0, 0.0, 10.0, 10.0);
        assert!(rect.contains(SkDPoint::new(5.0, 5.0)));
        assert!(!rect.contains(SkDPoint::new(15.0, 15.0)));
    }

    #[test]
    fn rect_intersects() {
        let rect1 = SkDRect::new(0.0, 0.0, 10.0, 10.0);
        let rect2 = SkDRect::new(5.0, 5.0, 15.0, 15.0);
        assert!(rect1.intersects(&rect2));
    }

    #[test]
    fn set_bounds_quad_apex() {
        // Quadratic with endpoints at y=0 and control point at y=10: the
        // curve's actual peak is at t=0.5, y=5 (not at the control point).
        let quad = SkDQuad::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(5.0, 10.0),
            SkDPoint::new(10.0, 0.0),
        ]);
        let rect = SkDRect::set_bounds_quad(&quad);
        assert!((rect.f_top - 0.0).abs() < 1e-6);
        assert!((rect.f_bottom - 5.0).abs() < 1e-6);
    }

    #[test]
    fn set_bounds_cubic_symmetric() {
        let cubic = SkDCubic::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(75.0, 300.0),
            SkDPoint::new(225.0, -300.0),
            SkDPoint::new(300.0, 0.0),
        ]);
        let rect = SkDRect::set_bounds_cubic(&cubic);
        assert!((rect.f_left - 0.0).abs() < 1e-6);
        assert!((rect.f_right - 300.0).abs() < 1e-6);
        // Symmetric S-curve: extrema push top above 0 and bottom below 0.
        assert!(rect.f_top < 0.0);
        assert!(rect.f_bottom > 0.0);
    }

    #[test]
    fn set_bounds_conic_weight_one_matches_quad() {
        let quad = SkDQuad::new([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(5.0, 10.0),
            SkDPoint::new(10.0, 0.0),
        ]);
        let conic = SkDConic::new(quad.f_pts, 1.0);
        let quad_rect = SkDRect::set_bounds_quad(&quad);
        let conic_rect = SkDRect::set_bounds_conic(&conic);
        assert!((quad_rect.f_bottom - conic_rect.f_bottom).abs() < 1e-6);
    }
}
