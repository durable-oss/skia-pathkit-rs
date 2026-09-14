//! 3x3 affine and perspective transforms.
//!
//! Ported from `old/pathkit/include/core/SkMatrix.h` and
//! `old/pathkit/src/core/SkMatrix.cpp`.
//!
//! Unlike the C++ original, this port does not cache a lazily computed
//! "type mask" for dispatch to specialized fast paths (that cache existed
//! to dodge virtual dispatch and redundant classification in a
//! single-threaded C++ hot loop). Every operation here goes through the
//! general 3x3 math directly; behavior is identical, only the
//! micro-optimization is dropped.

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

use super::point::Point;
use super::rect::Rect;
use super::scalar::{self, Scalar};
use crate::core::point::Point3;

/// Index of the horizontal scale factor within [`Matrix::as_slice`].
pub const M_SCALE_X: usize = 0;
/// Index of the horizontal skew factor within [`Matrix::as_slice`].
pub const M_SKEW_X: usize = 1;
/// Index of the horizontal translation within [`Matrix::as_slice`].
pub const M_TRANS_X: usize = 2;
/// Index of the vertical skew factor within [`Matrix::as_slice`].
pub const M_SKEW_Y: usize = 3;
/// Index of the vertical scale factor within [`Matrix::as_slice`].
pub const M_SCALE_Y: usize = 4;
/// Index of the vertical translation within [`Matrix::as_slice`].
pub const M_TRANS_Y: usize = 5;
/// Index of the input x-axis perspective factor within [`Matrix::as_slice`].
pub const M_PERSP_0: usize = 6;
/// Index of the input y-axis perspective factor within [`Matrix::as_slice`].
pub const M_PERSP_1: usize = 7;
/// Index of the perspective scale factor within [`Matrix::as_slice`].
pub const M_PERSP_2: usize = 8;

/// How [`Matrix::rect_to_rect`] aligns the source rectangle within the
/// destination when it restricts scaling to be uniform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScaleToFit {
    /// Scale in x and y independently to fill the destination rectangle.
    Fill,
    /// Scale uniformly and align to the top-left.
    Start,
    /// Scale uniformly and align to the center.
    Center,
    /// Scale uniformly and align to the bottom-right.
    End,
}

/// A 3x3 matrix for transforming 2D coordinates, stored in row-major order:
///
/// ```text
/// | scaleX  skewX  transX |
/// |  skewY scaleY  transY |
/// | persp0 persp1  persp2 |
/// ```
///
/// The default value is the identity matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix {
    m: [Scalar; 9],
}

impl Default for Matrix {
    fn default() -> Self {
        Self::identity()
    }
}

impl Matrix {
    /// Returns the identity matrix.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            m: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
    }

    /// Returns a matrix with all nine entries specified directly, in
    /// row-major order.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub const fn new_all(
        scale_x: Scalar,
        skew_x: Scalar,
        trans_x: Scalar,
        skew_y: Scalar,
        scale_y: Scalar,
        trans_y: Scalar,
        persp_0: Scalar,
        persp_1: Scalar,
        persp_2: Scalar,
    ) -> Self {
        Self {
            m: [
                scale_x, skew_x, trans_x, skew_y, scale_y, trans_y, persp_0, persp_1, persp_2,
            ],
        }
    }

    /// Returns a matrix that scales by `(sx, sy)` about the origin.
    #[must_use]
    pub const fn scale(sx: Scalar, sy: Scalar) -> Self {
        Self::new_all(sx, 0.0, 0.0, 0.0, sy, 0.0, 0.0, 0.0, 1.0)
    }

    /// Returns a matrix that translates by `(dx, dy)`.
    #[must_use]
    pub const fn translate(dx: Scalar, dy: Scalar) -> Self {
        Self::new_all(1.0, 0.0, dx, 0.0, 1.0, dy, 0.0, 0.0, 1.0)
    }

    /// Returns a matrix that rotates clockwise by `degrees` about the
    /// origin.
    #[must_use]
    pub fn rotate_deg(degrees: Scalar) -> Self {
        let mut m = Self::identity();
        m.set_rotate(degrees);
        m
    }

    /// Returns the raw nine matrix entries in row-major order.
    #[must_use]
    pub fn as_slice(&self) -> &[Scalar; 9] {
        &self.m
    }

    /// Returns the entry at `index` (0-8, row-major).
    #[must_use]
    pub fn get(&self, index: usize) -> Scalar {
        self.m[index]
    }

    /// Sets the entry at `index` (0-8, row-major).
    pub fn set(&mut self, index: usize, value: Scalar) {
        self.m[index] = value;
    }

    /// Returns the horizontal scale factor.
    #[must_use]
    pub fn scale_x(&self) -> Scalar {
        self.m[M_SCALE_X]
    }

    /// Returns the vertical scale factor.
    #[must_use]
    pub fn scale_y(&self) -> Scalar {
        self.m[M_SCALE_Y]
    }

    /// Returns the vertical skew factor.
    #[must_use]
    pub fn skew_y(&self) -> Scalar {
        self.m[M_SKEW_Y]
    }

    /// Returns the horizontal skew factor.
    #[must_use]
    pub fn skew_x(&self) -> Scalar {
        self.m[M_SKEW_X]
    }

    /// Returns the horizontal translation.
    #[must_use]
    pub fn translate_x(&self) -> Scalar {
        self.m[M_TRANS_X]
    }

    /// Returns the vertical translation.
    #[must_use]
    pub fn translate_y(&self) -> Scalar {
        self.m[M_TRANS_Y]
    }

    /// Returns `true` if this is the identity matrix.
    #[must_use]
    pub fn is_identity(&self) -> bool {
        self.m == Self::identity().m
    }

    /// Returns `true` if this matrix has a nonzero perspective component.
    #[must_use]
    pub fn has_perspective(&self) -> bool {
        self.m[M_PERSP_0] != 0.0 || self.m[M_PERSP_1] != 0.0 || self.m[M_PERSP_2] != 1.0
    }

    /// Returns `true` if this matrix maps any rectangle to another
    /// rectangle: identity, any combination of scale/translate, or a
    /// rotation by a multiple of 90 degrees.
    #[must_use]
    pub fn rect_stays_rect(&self) -> bool {
        if self.has_perspective() {
            return false;
        }
        let (sx, kx, ky, sy) = (
            self.m[M_SCALE_X],
            self.m[M_SKEW_X],
            self.m[M_SKEW_Y],
            self.m[M_SCALE_Y],
        );
        (sx != 0.0 && sy != 0.0 && kx == 0.0 && ky == 0.0)
            || (sx == 0.0 && sy == 0.0 && kx != 0.0 && ky != 0.0)
    }

    /// Resets this matrix to the identity in place.
    pub fn reset(&mut self) -> &mut Self {
        *self = Self::identity();
        self
    }

    /// Sets this matrix to translate by `(dx, dy)`.
    pub fn set_translate(&mut self, dx: Scalar, dy: Scalar) -> &mut Self {
        *self = Self::translate(dx, dy);
        self
    }

    /// Sets this matrix to scale by `(sx, sy)` about the pivot `(px, py)`.
    pub fn set_scale_pivot(&mut self, sx: Scalar, sy: Scalar, px: Scalar, py: Scalar) -> &mut Self {
        if sx == 1.0 && sy == 1.0 {
            self.reset();
        } else {
            *self = Self::new_all(sx, 0.0, px - sx * px, 0.0, sy, py - sy * py, 0.0, 0.0, 1.0);
        }
        self
    }

    /// Sets this matrix to scale by `(sx, sy)` about the origin.
    pub fn set_scale(&mut self, sx: Scalar, sy: Scalar) -> &mut Self {
        *self = Self::scale(sx, sy);
        self
    }

    /// Sets this matrix to rotate clockwise by `degrees` about the pivot
    /// `(px, py)`.
    pub fn set_rotate_pivot(&mut self, degrees: Scalar, px: Scalar, py: Scalar) -> &mut Self {
        let rad = degrees.to_radians();
        self.set_sin_cos_pivot(snap_to_zero(rad.sin()), snap_to_zero(rad.cos()), px, py)
    }

    /// Sets this matrix to rotate clockwise by `degrees` about the origin.
    pub fn set_rotate(&mut self, degrees: Scalar) -> &mut Self {
        let rad = degrees.to_radians();
        self.set_sin_cos(snap_to_zero(rad.sin()), snap_to_zero(rad.cos()))
    }

    /// Sets this matrix to a rotation by `sin_v`/`cos_v` (clockwise) about the
    /// pivot `(px, py)`.
    ///
    /// Used by [`crate::core::SkContourMeasure`] to build a rotation matrix
    /// from a unit tangent vector.
    pub fn set_sin_cos_pivot(
        &mut self,
        sin_v: Scalar,
        cos_v: Scalar,
        px: Scalar,
        py: Scalar,
    ) -> &mut Self {
        let one_minus_cos = 1.0 - cos_v;
        self.m[M_SCALE_X] = cos_v;
        self.m[M_SKEW_X] = -sin_v;
        self.m[M_TRANS_X] = sin_v * py + one_minus_cos * px;
        self.m[M_SKEW_Y] = sin_v;
        self.m[M_SCALE_Y] = cos_v;
        self.m[M_TRANS_Y] = -sin_v * px + one_minus_cos * py;
        self.m[M_PERSP_0] = 0.0;
        self.m[M_PERSP_1] = 0.0;
        self.m[M_PERSP_2] = 1.0;
        self
    }

    fn set_sin_cos(&mut self, sin_v: Scalar, cos_v: Scalar) -> &mut Self {
        self.m = [cos_v, -sin_v, 0.0, sin_v, cos_v, 0.0, 0.0, 0.0, 1.0];
        self
    }

    /// Sets this matrix to skew by `(kx, ky)` about the pivot `(px, py)`.
    pub fn set_skew_pivot(&mut self, kx: Scalar, ky: Scalar, px: Scalar, py: Scalar) -> &mut Self {
        *self = Self::new_all(1.0, kx, -kx * py, ky, 1.0, -ky * px, 0.0, 0.0, 1.0);
        self
    }

    /// Sets this matrix to skew by `(kx, ky)` about the origin.
    pub fn set_skew(&mut self, kx: Scalar, ky: Scalar) -> &mut Self {
        *self = Self::new_all(1.0, kx, 0.0, ky, 1.0, 0.0, 0.0, 0.0, 1.0);
        self
    }

    /// Sets this matrix to `a` concatenated with `b` (`self = a * b`):
    /// applying the result to a point is equivalent to applying `b` first,
    /// then `a`.
    pub fn set_concat(&mut self, a: &Self, b: &Self) -> &mut Self {
        if a.is_identity() {
            *self = *b;
            return self;
        }
        if b.is_identity() {
            *self = *a;
            return self;
        }

        let am = &a.m;
        let bm = &b.m;
        let mut r = [0.0; 9];
        for row in 0..3 {
            for col in 0..3 {
                r[row * 3 + col] = am[row * 3] * bm[col]
                    + am[row * 3 + 1] * bm[3 + col]
                    + am[row * 3 + 2] * bm[6 + col];
            }
        }
        self.m = r;
        self
    }

    /// Pre-concatenates `other` onto this matrix: `self = self * other`.
    pub fn pre_concat(&mut self, other: &Self) -> &mut Self {
        if !other.is_identity() {
            let this = *self;
            self.set_concat(&this, other);
        }
        self
    }

    /// Post-concatenates `other` onto this matrix: `self = other * self`.
    pub fn post_concat(&mut self, other: &Self) -> &mut Self {
        if !other.is_identity() {
            let this = *self;
            self.set_concat(other, &this);
        }
        self
    }

    /// Pre-translates this matrix by `(dx, dy)`.
    pub fn pre_translate(&mut self, dx: Scalar, dy: Scalar) -> &mut Self {
        let m = Self::translate(dx, dy);
        self.pre_concat(&m)
    }

    /// Post-translates this matrix by `(dx, dy)`.
    pub fn post_translate(&mut self, dx: Scalar, dy: Scalar) -> &mut Self {
        let m = Self::translate(dx, dy);
        self.post_concat(&m)
    }

    /// Pre-scales this matrix by `(sx, sy)` about the origin.
    pub fn pre_scale(&mut self, sx: Scalar, sy: Scalar) -> &mut Self {
        let m = Self::scale(sx, sy);
        self.pre_concat(&m)
    }

    /// Post-scales this matrix by `(sx, sy)` about the origin.
    pub fn post_scale(&mut self, sx: Scalar, sy: Scalar) -> &mut Self {
        let m = Self::scale(sx, sy);
        self.post_concat(&m)
    }

    /// Pre-rotates this matrix clockwise by `degrees` about the origin.
    pub fn pre_rotate(&mut self, degrees: Scalar) -> &mut Self {
        let m = Self::rotate_deg(degrees);
        self.pre_concat(&m)
    }

    /// Post-rotates this matrix clockwise by `degrees` about the origin.
    pub fn post_rotate(&mut self, degrees: Scalar) -> &mut Self {
        let m = Self::rotate_deg(degrees);
        self.post_concat(&m)
    }

    /// Returns the matrix determinant, using the perspective formula when
    /// [`Matrix::has_perspective`] is `true`.
    #[must_use]
    pub fn determinant(&self) -> f64 {
        let m = &self.m;
        let (m00, m01, m02) = (f64::from(m[0]), f64::from(m[1]), f64::from(m[2]));
        let (m10, m11, m12) = (f64::from(m[3]), f64::from(m[4]), f64::from(m[5]));
        let (m20, m21, m22) = (f64::from(m[6]), f64::from(m[7]), f64::from(m[8]));
        if self.has_perspective() {
            m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20)
                + m02 * (m10 * m21 - m11 * m20)
        } else {
            m00 * m11 - m01 * m10
        }
    }

    /// Computes the inverse of this matrix, returning `None` if it is not
    /// invertible (determinant nearly zero).
    #[must_use]
    pub fn invert(&self) -> Option<Self> {
        if self.is_identity() {
            return Some(Self::identity());
        }

        let det = self.determinant();
        let cube = f64::from(scalar::NEARLY_ZERO).powi(3);
        if det.abs() <= cube {
            return None;
        }
        let inv_det = 1.0 / det;

        let m = &self.m;
        let (m00, m01, m02) = (f64::from(m[0]), f64::from(m[1]), f64::from(m[2]));
        let (m10, m11, m12) = (f64::from(m[3]), f64::from(m[4]), f64::from(m[5]));
        let (m20, m21, m22) = (f64::from(m[6]), f64::from(m[7]), f64::from(m[8]));

        let out = if self.has_perspective() {
            [
                (m11 * m22 - m12 * m21) * inv_det,
                (m02 * m21 - m01 * m22) * inv_det,
                (m01 * m12 - m02 * m11) * inv_det,
                (m12 * m20 - m10 * m22) * inv_det,
                (m00 * m22 - m02 * m20) * inv_det,
                (m02 * m10 - m00 * m12) * inv_det,
                (m10 * m21 - m11 * m20) * inv_det,
                (m01 * m20 - m00 * m21) * inv_det,
                (m00 * m11 - m01 * m10) * inv_det,
            ]
        } else {
            [
                m11 * inv_det,
                -m01 * inv_det,
                (m01 * m12 - m11 * m02) * inv_det,
                -m10 * inv_det,
                m00 * inv_det,
                (m10 * m02 - m00 * m12) * inv_det,
                0.0,
                0.0,
                1.0,
            ]
        };

        Some(Self::new_all(
            out[0] as Scalar,
            out[1] as Scalar,
            out[2] as Scalar,
            out[3] as Scalar,
            out[4] as Scalar,
            out[5] as Scalar,
            out[6] as Scalar,
            out[7] as Scalar,
            out[8] as Scalar,
        ))
    }

    /// Transforms `src` by this matrix, applying the full perspective
    /// divide when [`Matrix::has_perspective`] is `true`.
    #[must_use]
    pub fn map_point(&self, src: Point) -> Point {
        let m = &self.m;
        let x = m[M_SCALE_X] * src.x + m[M_SKEW_X] * src.y + m[M_TRANS_X];
        let y = m[M_SKEW_Y] * src.x + m[M_SCALE_Y] * src.y + m[M_TRANS_Y];
        if self.has_perspective() {
            let w = m[M_PERSP_0] * src.x + m[M_PERSP_1] * src.y + m[M_PERSP_2];
            let inv_w = if w != 0.0 { 1.0 / w } else { 1.0 };
            Point::new(x * inv_w, y * inv_w)
        } else {
            Point::new(x, y)
        }
    }

    /// Transforms every point in `src`, writing results to `dst`.
    ///
    /// # Panics
    ///
    /// Panics if `dst.len() < src.len()`.
    pub fn map_points(&self, dst: &mut [Point], src: &[Point]) {
        assert!(dst.len() >= src.len());
        for (d, &s) in dst.iter_mut().zip(src) {
            *d = self.map_point(s);
        }
    }

    /// Transforms `points` in place.
    pub fn map_points_in_place(&self, points: &mut [Point]) {
        for p in points.iter_mut() {
            *p = self.map_point(*p);
        }
    }

    /// Transforms `(x, y)`.
    #[must_use]
    pub fn map_xy(&self, x: Scalar, y: Scalar) -> Point {
        self.map_point(Point::new(x, y))
    }

    /// Returns the destination rectangle produced by mapping the four
    /// corners of `src` and taking their bounds. Exact (not an
    /// approximation) even under rotation, skew, or perspective.
    #[must_use]
    pub fn map_rect(&self, src: &Rect) -> Rect {
        let corners = src.to_quad();
        let mut mapped = [Point::default(); 4];
        self.map_points(&mut mapped, &corners);
        Rect::from_points(&mapped)
    }

    /// Returns a matrix that maps `src` onto `dst` according to `fit`.
    ///
    /// Returns `None` if `src` is empty (the mapping is undefined).
    #[must_use]
    pub fn rect_to_rect(src: &Rect, dst: &Rect, fit: ScaleToFit) -> Option<Self> {
        if src.is_empty() {
            return None;
        }
        if dst.is_empty() {
            return Some(Self::new_all(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
        }

        let sx = dst.width() / src.width();
        let sy = dst.height() / src.height();
        let (sx, sy, tx_align, ty_align) = if fit == ScaleToFit::Fill {
            (sx, sy, 0.0, 0.0)
        } else {
            let s = sx.min(sy);
            let extra_x = dst.width() - src.width() * s;
            let extra_y = dst.height() - src.height() * s;
            let (ax, ay) = match fit {
                ScaleToFit::Start => (0.0, 0.0),
                ScaleToFit::Center => (extra_x * 0.5, extra_y * 0.5),
                ScaleToFit::End => (extra_x, extra_y),
                ScaleToFit::Fill => unreachable!(),
            };
            (s, s, ax, ay)
        };

        let tx = dst.left - src.left * sx + tx_align;
        let ty = dst.top - src.top * sy + ty_align;
        Some(Self::new_all(sx, 0.0, tx, 0.0, sy, ty, 0.0, 0.0, 1.0))
    }

    /// Copies matrix from a raw array of 9 scalars in row-major order.
    pub fn set9(&mut self, src: &[Scalar; 9]) -> &mut Self {
        self.m = *src;
        self
    }

    /// Pre-translates by `(dx, dy)` using optimized path for non-perspective matrices.
    pub fn pre_translate_optimized(&mut self, dx: Scalar, dy: Scalar) -> &mut Self {
        if self.has_perspective() {
            let m = Self::translate(dx, dy);
            self.pre_concat(&m);
        } else {
            let (sx, kx, ky, sy) = (
                self.m[M_SCALE_X],
                self.m[M_SKEW_X],
                self.m[M_SKEW_Y],
                self.m[M_SCALE_Y],
            );
            self.m[M_TRANS_X] = sx * dx + kx * dy + self.m[M_TRANS_X];
            self.m[M_TRANS_Y] = ky * dx + sy * dy + self.m[M_TRANS_Y];
        }
        self
    }

    /// Post-translates by `(dx, dy)` using optimized path for non-perspective matrices.
    pub fn post_translate_optimized(&mut self, dx: Scalar, dy: Scalar) -> &mut Self {
        if self.has_perspective() {
            let m = Self::translate(dx, dy);
            self.post_concat(&m);
        } else {
            self.m[M_TRANS_X] += dx;
            self.m[M_TRANS_Y] += dy;
        }
        self
    }

    /// Pre-scales by `(sx, sy)` using optimized path.
    pub fn pre_scale_optimized(&mut self, sx: Scalar, sy: Scalar) -> &mut Self {
        if sx == 1.0 && sy == 1.0 {
            return self;
        }
        self.m[M_SCALE_X] *= sx;
        self.m[M_SKEW_Y] *= sx;
        self.m[M_PERSP_0] *= sx;
        self.m[M_SKEW_X] *= sy;
        self.m[M_SCALE_Y] *= sy;
        self.m[M_PERSP_1] *= sy;
        self
    }

    /// Pre-rotates clockwise by `degrees` about the pivot `(px, py)`.
    pub fn pre_rotate_pivot(&mut self, degrees: Scalar, px: Scalar, py: Scalar) -> &mut Self {
        let rad = degrees.to_radians();
        let sin_v = snap_to_zero(rad.sin());
        let cos_v = snap_to_zero(rad.cos());
        let one_minus_cos = 1.0 - cos_v;
        let m = Self::new_all(
            cos_v,
            -sin_v,
            sin_v * py + one_minus_cos * px,
            sin_v,
            cos_v,
            -sin_v * px + one_minus_cos * py,
            0.0,
            0.0,
            1.0,
        );
        self.pre_concat(&m);
        self
    }

    /// Pre-skew by `(kx, ky)` about the origin.
    pub fn pre_skew(&mut self, kx: Scalar, ky: Scalar) -> &mut Self {
        let m = Self::new_all(1.0, kx, 0.0, ky, 1.0, 0.0, 0.0, 0.0, 1.0);
        self.pre_concat(&m);
        self
    }

    /// Post-skew by `(kx, ky)` about the origin.
    pub fn post_skew(&mut self, kx: Scalar, ky: Scalar) -> &mut Self {
        let m = Self::new_all(1.0, kx, 0.0, ky, 1.0, 0.0, 0.0, 0.0, 1.0);
        self.post_concat(&m);
        self
    }

    /// Post-skew by `(kx, ky)` about the pivot `(px, py)`.
    pub fn post_skew_pivot(&mut self, kx: Scalar, ky: Scalar, px: Scalar, py: Scalar) -> &mut Self {
        let m = Self::new_all(1.0, kx, -kx * py, ky, 1.0, -ky * px, 0.0, 0.0, 1.0);
        self.post_concat(&m);
        self
    }

    /// Divides scale/translation by integer values. Returns false if divisors are zero.
    pub fn post_idiv(&mut self, divx: i32, divy: i32) -> bool {
        if divx == 0 || divy == 0 {
            return false;
        }
        let invx = 1.0 / (divx as Scalar);
        let invy = 1.0 / (divy as Scalar);
        self.m[M_SCALE_X] *= invx;
        self.m[M_SKEW_X] *= invx;
        self.m[M_TRANS_X] *= invx;
        self.m[M_SCALE_Y] *= invy;
        self.m[M_SKEW_Y] *= invy;
        self.m[M_TRANS_Y] *= invy;
        true
    }

    /// Maps points using an optimized fast path based on matrix type.
    pub fn map_points_fast(&self, dst: &mut [Point], src: &[Point]) {
        if dst.len() < src.len() {
            return;
        }
        if src.is_empty() {
            return;
        }

        let m = &self.m;
        let (sx, kx, tx, ky, sy, ty, p0, p1, p2) = (
            m[M_SCALE_X],
            m[M_SKEW_X],
            m[M_TRANS_X],
            m[M_SKEW_Y],
            m[M_SCALE_Y],
            m[M_TRANS_Y],
            m[M_PERSP_0],
            m[M_PERSP_1],
            m[M_PERSP_2],
        );

        if self.has_perspective() {
            for (d, &s) in dst.iter_mut().zip(src) {
                let x = sx * s.x + kx * s.y + tx;
                let y = ky * s.x + sy * s.y + ty;
                let w = p0 * s.x + p1 * s.y + p2;
                let inv_w = if w != 0.0 { 1.0 / w } else { 1.0 };
                *d = Point::new(x * inv_w, y * inv_w);
            }
        } else if kx == 0.0 && ky == 0.0 && tx == 0.0 && ty == 0.0 {
            // Pure scale
            for (d, &s) in dst.iter_mut().zip(src) {
                *d = Point::new(s.x * sx, s.y * sy);
            }
        } else if kx == 0.0 && ky == 0.0 && sx == 1.0 && sy == 1.0 {
            // Pure translation
            for (d, &s) in dst.iter_mut().zip(src) {
                *d = Point::new(s.x + tx, s.y + ty);
            }
        } else if kx == 0.0 && ky == 0.0 {
            // Scale + translation
            for (d, &s) in dst.iter_mut().zip(src) {
                *d = Point::new(s.x * sx + tx, s.y * sy + ty);
            }
        } else {
            // Full affine
            for (d, &s) in dst.iter_mut().zip(src) {
                *d = Point::new(s.x * sx + s.y * kx + tx, s.x * ky + s.y * sy + ty);
            }
        }
    }

    /// Maps a single (x, y) point using the fast path.
    #[must_use]
    pub fn map_xy_fast(&self, x: Scalar, y: Scalar) -> Point {
        let m = &self.m;
        let x_out = m[M_SCALE_X] * x + m[M_SKEW_X] * y + m[M_TRANS_X];
        let y_out = m[M_SKEW_Y] * x + m[M_SCALE_Y] * y + m[M_TRANS_Y];
        if self.has_perspective() {
            let w = m[M_PERSP_0] * x + m[M_PERSP_1] * y + m[M_PERSP_2];
            let inv_w = if w != 0.0 { 1.0 / w } else { 1.0 };
            Point::new(x_out * inv_w, y_out * inv_w)
        } else {
            Point::new(x_out, y_out)
        }
    }

    /// Maps 3D homogeneous points.
    pub fn map_homogeneous_points(&self, dst: &mut [Point3], src: &[Point3]) {
        if dst.len() < src.len() {
            return;
        }
        for (d, &s) in dst.iter_mut().zip(src) {
            *d = self.map_homogeneous_point(s);
        }
    }

    /// Maps a single 3D homogeneous point.
    #[must_use]
    pub fn map_homogeneous_point(&self, p: Point3) -> Point3 {
        let m = &self.m;
        let x = m[M_SCALE_X] * p.x + m[M_SKEW_X] * p.y + m[M_TRANS_X] * p.z;
        let y = m[M_SKEW_Y] * p.x + m[M_SCALE_Y] * p.y + m[M_TRANS_Y] * p.z;
        let w = m[M_PERSP_0] * p.x + m[M_PERSP_1] * p.y + m[M_PERSP_2] * p.z;
        Point3::new(x, y, w)
    }

    /// Maps a rectangle using only scale and translation (fast path).
    pub fn map_rect_scale_translate(&self, dst: &mut Rect, src: &Rect) {
        let m = &self.m;
        let sx = m[M_SCALE_X];
        let sy = m[M_SCALE_Y];
        let tx = m[M_TRANS_X];
        let ty = m[M_TRANS_Y];
        dst.left = src.left * sx + tx;
        dst.top = src.top * sy + ty;
        dst.right = src.right * sx + tx;
        dst.bottom = src.bottom * sy + ty;
    }

    /// Returns the minimum scale factor. Returns -1 if perspective.
    pub fn get_min_scale(&self) -> Scalar {
        if self.has_perspective() {
            return -1.0;
        }
        let m = &self.m;
        let sx = m[M_SCALE_X].abs();
        let sy = m[M_SCALE_Y].abs();
        sx.min(sy)
    }

    /// Returns the maximum scale factor. Returns -1 if perspective.
    pub fn get_max_scale(&self) -> Scalar {
        if self.has_perspective() {
            return -1.0;
        }
        let m = &self.m;
        let sx = m[M_SCALE_X].abs();
        let sy = m[M_SCALE_Y].abs();
        sx.max(sy)
    }

    /// Returns both min and max scale factors. Returns None if perspective.
    pub fn get_min_max_scales(&self) -> Option<(Scalar, Scalar)> {
        if self.has_perspective() {
            return None;
        }
        let m = &self.m;
        let sx = m[M_SCALE_X].abs();
        let sy = m[M_SCALE_Y].abs();
        Some((sx.min(sy), sx.max(sy)))
    }

    /// Writes matrix to memory buffer (9 scalars * 4 bytes each). Returns bytes written.
    pub fn write_to_memory(&self, buffer: &mut [u8]) -> usize {
        let bytes = 36;
        if buffer.len() < bytes {
            return 0;
        }
        for i in 0..9 {
            let val = self.m[i].to_le_bytes();
            let offset = i * 4;
            buffer[offset] = val[0];
            buffer[offset + 1] = val[1];
            buffer[offset + 2] = val[2];
            buffer[offset + 3] = val[3];
        }
        bytes
    }

    /// Reads matrix from memory buffer. Returns bytes read.
    pub fn read_from_memory(&mut self, buffer: &[u8]) -> usize {
        let bytes = 36;
        if buffer.len() < bytes {
            return 0;
        }
        for i in 0..9 {
            let offset = i * 4;
            let val = [
                buffer[offset],
                buffer[offset + 1],
                buffer[offset + 2],
                buffer[offset + 3],
            ];
            self.m[i] = Scalar::from_le_bytes(val);
        }
        bytes
    }
}

/// Decomposes the upper-left 2x2 of the matrix via polar decomposition.
/// Returns (rotation1, scale, rotation2) such that the matrix ≈ R1 * S * R2.
pub fn decompose_upper_2x2(matrix: &Matrix) -> Option<(Point, Point, Point)> {
    let m = &matrix.m;
    let a = m[M_SCALE_X];
    let b = m[M_SKEW_X];
    let c = m[M_SKEW_Y];
    let d = m[M_SCALE_Y];

    // Check if 2x2 is degenerate
    let perp_dot = a * d - b * c;
    if scalar::nearly_zero(perp_dot, None) {
        return None;
    }

    // Polar decomposition: M = Q * S
    let (mut cos_q, mut sin_q, sa, sb, sd);

    if scalar::nearly_equal(b, c, None) {
        // Already symmetric
        cos_q = 1.0;
        sin_q = 0.0;
        sa = a;
        sb = b;
        sd = d;
    } else {
        cos_q = a + d;
        sin_q = c - b;
        let len = ((cos_q * cos_q + sin_q * sin_q) as f64).sqrt() as Scalar;
        if len == 0.0 {
            return None;
        }
        let reciplen = 1.0 / len;
        cos_q *= reciplen;
        sin_q *= reciplen;

        sa = a * cos_q + c * sin_q;
        sb = b * cos_q + d * sin_q;
        sd = -b * sin_q + d * cos_q;
    }

    // Compute eigenvalues (scales) and eigenvectors (rotations)
    let (w1, w2, mut cos1, mut sin1, cos2, sin2);

    if scalar::nearly_zero(sb, None) {
        // Already diagonalized
        w1 = sa;
        w2 = sd;
        cos1 = 1.0;
        sin1 = 0.0;
        cos2 = cos_q;
        sin2 = sin_q;
    } else {
        let diff = sa - sd;
        let discriminant = ((diff * diff + 4.0 * sb * sb) as f64).sqrt() as Scalar;
        let trace = sa + sd;

        if diff > 0.0 {
            w1 = 0.5 * (trace + discriminant);
            w2 = 0.5 * (trace - discriminant);
        } else {
            w1 = 0.5 * (trace - discriminant);
            w2 = 0.5 * (trace + discriminant);
        }

        cos1 = sb;
        sin1 = w1 - sa;
        let len1 = ((cos1 * cos1 + sin1 * sin1) as f64).sqrt() as Scalar;
        if len1 == 0.0 {
            return None;
        }
        let reciplen = 1.0 / len1;
        cos1 *= reciplen;
        sin1 *= reciplen;

        cos2 = cos1 * cos_q - sin1 * sin_q;
        sin2 = sin1 * cos_q + cos1 * sin_q;
        sin1 = -sin1; // Rotation 1 is U^T
    }

    let scale = Point::new(pk_double_to_scalar(w1 as f64), pk_double_to_scalar(w2 as f64));
    let rotation1 = Point::new(cos1, sin1);
    let rotation2 = Point::new(cos2, sin2);

    Some((rotation1, scale, rotation2))
}

/// Helper to convert f64 to Scalar
fn pk_double_to_scalar(v: f64) -> Scalar {
    v as Scalar
}

fn snap_to_zero(v: Scalar) -> Scalar {
    if scalar::nearly_zero(v, Some(scalar::NEARLY_ZERO)) {
        0.0
    } else {
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_maps_points_unchanged() {
        let m = Matrix::identity();
        assert_eq!(m.map_xy(3.0, 4.0), Point::new(3.0, 4.0));
    }

    #[test]
    fn scale_maps_points() {
        let m = Matrix::scale(2.0, 3.0);
        assert_eq!(m.map_xy(1.0, 1.0), Point::new(2.0, 3.0));
    }

    #[test]
    fn translate_maps_points() {
        let m = Matrix::translate(5.0, -2.0);
        assert_eq!(m.map_xy(1.0, 1.0), Point::new(6.0, -1.0));
    }

    #[test]
    fn rotate_90_degrees() {
        let m = Matrix::rotate_deg(90.0);
        let p = m.map_xy(1.0, 0.0);
        assert!((p.x - 0.0).abs() < 1e-6);
        assert!((p.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn concat_applies_right_operand_first() {
        let mut m = Matrix::identity();
        let scale = Matrix::scale(2.0, 2.0);
        let translate = Matrix::translate(1.0, 0.0);
        m.set_concat(&scale, &translate);
        // scale * translate: translate first, then scale.
        assert_eq!(m.map_xy(0.0, 0.0), Point::new(2.0, 0.0));
    }

    #[test]
    fn invert_scale_and_translate() {
        let mut m = Matrix::scale(2.0, 4.0);
        m.post_translate(10.0, 20.0);
        let inv = m.invert().expect("invertible");
        let p = m.map_xy(3.0, 5.0);
        let back = inv.map_xy(p.x, p.y);
        assert!((back.x - 3.0).abs() < 1e-4);
        assert!((back.y - 5.0).abs() < 1e-4);
    }

    #[test]
    fn invert_singular_returns_none() {
        let m = Matrix::scale(0.0, 1.0);
        assert!(m.invert().is_none());
    }

    #[test]
    fn map_rect_under_rotation_covers_bounds() {
        let m = Matrix::rotate_deg(45.0);
        let r = Rect::from_ltrb(-1.0, -1.0, 1.0, 1.0);
        let mapped = m.map_rect(&r);
        assert!(mapped.width() > r.width());
    }

    #[test]
    fn rect_to_rect_fill_maps_corners() {
        let src = Rect::from_ltrb(0.0, 0.0, 10.0, 10.0);
        let dst = Rect::from_ltrb(0.0, 0.0, 5.0, 20.0);
        let m = Matrix::rect_to_rect(&src, &dst, ScaleToFit::Fill).unwrap();
        let mapped = m.map_rect(&src);
        assert!((mapped.right - dst.right).abs() < 1e-4);
        assert!((mapped.bottom - dst.bottom).abs() < 1e-4);
    }

    #[test]
    fn rect_to_rect_empty_src_is_none() {
        assert!(
            Matrix::rect_to_rect(&Rect::empty(), &Rect::from_wh(1.0, 1.0), ScaleToFit::Fill)
                .is_none()
        );
    }

    #[test]
    fn test_set9() {
        let mut m = Matrix::identity();
        let src = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        m.set9(&src);
        assert_eq!(m.scale_x(), 1.0);
        assert_eq!(m.get(1), 2.0);
        assert_eq!(m.get(8), 9.0);
    }

    #[test]
    fn test_pre_translate_optimized() {
        let mut m = Matrix::translate(1.0, 2.0);
        m.pre_translate_optimized(5.0, 10.0);
        let p = m.map_xy(0.0, 0.0);
        assert!((p.x - 6.0).abs() < 1e-6);
        assert!((p.y - 12.0).abs() < 1e-6);
    }

    #[test]
    fn test_post_translate_optimized() {
        let mut m = Matrix::translate(1.0, 2.0);
        m.post_translate_optimized(5.0, 10.0);
        let p = m.map_xy(0.0, 0.0);
        assert!((p.x - 6.0).abs() < 1e-6);
        assert!((p.y - 12.0).abs() < 1e-6);
    }

    #[test]
    fn test_pre_scale_optimized() {
        let mut m = Matrix::scale(2.0, 3.0);
        m.pre_scale_optimized(5.0, 7.0);
        let p = m.map_xy(1.0, 1.0);
        assert!((p.x - 10.0).abs() < 1e-6);
        assert!((p.y - 21.0).abs() < 1e-6);
    }

    #[test]
    fn test_post_idiv() {
        let mut m = Matrix::scale(10.0, 20.0);
        let result = m.post_idiv(2, 4);
        assert!(result);
        assert!((m.scale_x() - 5.0).abs() < 1e-6);
        assert!((m.scale_y() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_post_idiv_zero() {
        let mut m = Matrix::scale(10.0, 20.0);
        let result = m.post_idiv(0, 4);
        assert!(!result);
    }

    #[test]
    fn test_pre_skew() {
        let mut m = Matrix::identity();
        m.pre_skew(0.5, 0.3);
        let p = m.map_xy(1.0, 0.0);
        assert!((p.x - 1.0).abs() < 1e-6);
        assert!((p.y - 0.3).abs() < 1e-6);
    }

    #[test]
    fn test_post_skew() {
        let mut m = Matrix::identity();
        m.post_skew(0.5, 0.3);
        let p = m.map_xy(1.0, 0.0);
        assert!((p.x - 1.0).abs() < 1e-6);
        assert!((p.y - 0.3).abs() < 1e-6);
    }

    #[test]
    fn test_map_points_fast() {
        let m = Matrix::translate(10.0, 20.0);
        let src = vec![Point::new(1.0, 2.0), Point::new(3.0, 4.0)];
        let mut dst = vec![Point::default(); 2];
        m.map_points_fast(&mut dst, &src);
        assert!((dst[0].x - 11.0).abs() < 1e-6);
        assert!((dst[0].y - 22.0).abs() < 1e-6);
    }

    #[test]
    fn test_map_xy_fast() {
        let m = Matrix::translate(5.0, 10.0);
        let p = m.map_xy_fast(1.0, 2.0);
        assert!((p.x - 6.0).abs() < 1e-6);
        assert!((p.y - 12.0).abs() < 1e-6);
    }

    #[test]
    fn test_map_homogeneous_point() {
        let m = Matrix::identity();
        let p = Point3::new(1.0, 2.0, 1.0);
        let result = m.map_homogeneous_point(p);
        assert!((result.x - 1.0).abs() < 1e-6);
        assert!((result.y - 2.0).abs() < 1e-6);
        assert!((result.z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_get_min_scale() {
        let m = Matrix::scale(2.0, 5.0);
        assert!((m.get_min_scale() - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_get_max_scale() {
        let m = Matrix::scale(2.0, 5.0);
        assert!((m.get_max_scale() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_get_min_max_scales() {
        let m = Matrix::scale(2.0, 5.0);
        let (min, max) = m.get_min_max_scales().unwrap();
        assert!((min - 2.0).abs() < 1e-6);
        assert!((max - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_get_min_max_scales_perspective() {
        let mut m = Matrix::scale(2.0, 5.0);
        m.m[M_PERSP_0] = 0.1;
        assert!(m.get_min_max_scales().is_none());
    }

    #[test]
    fn test_decompose_rotation() {
        let m = Matrix::rotate_deg(90.0);
        let result = decompose_upper_2x2(&m);
        assert!(result.is_some());
        let (_, s, _) = result.unwrap();
        assert!((s.x - 1.0).abs() < 1e-4);
        assert!((s.y - 1.0).abs() < 1e-4);
    }

    #[test]
    fn test_decompose_scaling() {
        let m = Matrix::scale(2.0, 4.0);
        let result = decompose_upper_2x2(&m);
        assert!(result.is_some());
        let (_, s, _) = result.unwrap();
        assert!((s.x - 2.0).abs() < 1e-4);
        assert!((s.y - 4.0).abs() < 1e-4);
    }

    #[test]
    fn test_decompose_degenerate() {
        let m = Matrix::scale(0.0, 1.0);
        let result = decompose_upper_2x2(&m);
        assert!(result.is_none());
    }
}
