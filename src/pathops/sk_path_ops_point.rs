//! Double-precision point and vector types for path operations.
//!
//! Port of Skia's `SkPathOpsPoint.h`. These are the coordinate types the
//! curve/intersection math in this module is built on: `f64` precision is
//! needed because the pathops engine accumulates many chained subdivisions
//! and root-finding steps, where `f32`'s precision would compound error.

use crate::core::Point;
use super::sk_path_ops_types::{
    almost_dequal_ulps, almost_pequal_ulps, approximately_equal, roughly_equal_ulps,
};

/// A double-precision 2D vector (displacement, not position).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SkDVector {
    /// The x component.
    pub f_x: f64,
    /// The y component.
    pub f_y: f64,
}

impl SkDVector {
    /// Creates a new vector.
    pub fn new(x: f64, y: f64) -> Self {
        Self { f_x: x, f_y: y }
    }

    /// The 2D cross product `self x a`.
    pub fn cross(&self, a: SkDVector) -> f64 {
        self.f_x * a.f_y - self.f_y * a.f_x
    }

    /// Like [`cross`](Self::cross), but treats a near-zero result (within
    /// 16 ULPs) as exactly zero, since near-parallel vectors otherwise
    /// produce a cross product that is nonzero only due to rounding.
    pub fn cross_check(&self, a: SkDVector) -> f64 {
        let xy = self.f_x * a.f_y;
        let yx = self.f_y * a.f_x;
        if almost_dequal_ulps(xy as f32, yx as f32) {
            0.0
        } else {
            xy - yx
        }
    }

    /// The dot product `self . a`.
    pub fn dot(&self, a: SkDVector) -> f64 {
        self.f_x * a.f_x + self.f_y * a.f_y
    }

    /// The vector's squared length.
    pub fn length_squared(&self) -> f64 {
        self.f_x * self.f_x + self.f_y * self.f_y
    }

    /// The vector's length.
    pub fn length(&self) -> f64 {
        self.length_squared().sqrt()
    }

    /// Returns true if both components are finite.
    pub fn is_finite(&self) -> bool {
        self.f_x.is_finite() && self.f_y.is_finite()
    }
}

impl std::ops::Add for SkDVector {
    type Output = SkDVector;
    fn add(self, rhs: SkDVector) -> SkDVector {
        SkDVector::new(self.f_x + rhs.f_x, self.f_y + rhs.f_y)
    }
}

impl std::ops::Sub for SkDVector {
    type Output = SkDVector;
    fn sub(self, rhs: SkDVector) -> SkDVector {
        SkDVector::new(self.f_x - rhs.f_x, self.f_y - rhs.f_y)
    }
}

impl std::ops::Mul<f64> for SkDVector {
    type Output = SkDVector;
    fn mul(self, s: f64) -> SkDVector {
        SkDVector::new(self.f_x * s, self.f_y * s)
    }
}

/// A double-precision 2D point.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SkDPoint {
    /// The x coordinate.
    pub f_x: f64,
    /// The y coordinate.
    pub f_y: f64,
}

impl SkDPoint {
    /// Creates a new point.
    pub fn new(x: f64, y: f64) -> Self {
        Self { f_x: x, f_y: y }
    }

    /// Converts from the crate's single-precision [`Point`].
    pub fn from_point(pt: Point) -> Self {
        Self {
            f_x: pt.x as f64,
            f_y: pt.y as f64,
        }
    }

    /// Converts to the crate's single-precision [`Point`].
    pub fn to_point(&self) -> Point {
        Point::new(self.f_x as f32, self.f_y as f32)
    }

    /// The midpoint of `a` and `b`.
    pub fn mid(a: SkDPoint, b: SkDPoint) -> SkDPoint {
        SkDPoint::new((a.f_x + b.f_x) / 2.0, (a.f_y + b.f_y) / 2.0)
    }

    /// The Euclidean distance to `a`.
    pub fn distance(&self, a: SkDPoint) -> f64 {
        (*self - a).length()
    }

    /// The squared Euclidean distance to `a` (cheaper than [`distance`](Self::distance)
    /// when only comparing magnitudes).
    pub fn distance_squared(&self, a: SkDPoint) -> f64 {
        (*self - a).length_squared()
    }

    /// True if both coordinates are within `FLT_EPSILON` of `a`'s (a cheap,
    /// scale-sensitive check — use [`approximately_equal`](Self::approximately_equal)
    /// for points whose magnitude may make that too strict).
    pub fn approximately_zero_or_equal(&self, a: SkDPoint) -> bool {
        approximately_equal(self.f_x, a.f_x) && approximately_equal(self.f_y, a.f_y)
    }

    /// True if this point and `a` are the same point up to floating-point
    /// tolerance, scaling the tolerance to the points' magnitude.
    pub fn approximately_equal(&self, a: SkDPoint) -> bool {
        if self.approximately_zero_or_equal(a) {
            return true;
        }
        if !roughly_equal_ulps(self.f_x as f32, a.f_x as f32)
            || !roughly_equal_ulps(self.f_y as f32, a.f_y as f32)
        {
            return false;
        }
        let dist = self.distance(a);
        let tiniest = self.f_x.min(a.f_x).min(self.f_y).min(a.f_y);
        let largest = self.f_x.max(a.f_x).max(self.f_y).max(a.f_y).max(-tiniest);
        almost_pequal_ulps(largest as f32, (largest + dist) as f32)
    }
}

impl std::ops::Sub for SkDPoint {
    type Output = SkDVector;
    fn sub(self, rhs: SkDPoint) -> SkDVector {
        SkDVector::new(self.f_x - rhs.f_x, self.f_y - rhs.f_y)
    }
}

impl std::ops::Add<SkDVector> for SkDPoint {
    type Output = SkDPoint;
    fn add(self, rhs: SkDVector) -> SkDPoint {
        SkDPoint::new(self.f_x + rhs.f_x, self.f_y + rhs.f_y)
    }
}

impl std::ops::Sub<SkDVector> for SkDPoint {
    type Output = SkDPoint;
    fn sub(self, rhs: SkDVector) -> SkDPoint {
        SkDPoint::new(self.f_x - rhs.f_x, self.f_y - rhs.f_y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector_cross_and_dot() {
        let a = SkDVector::new(1.0, 0.0);
        let b = SkDVector::new(0.0, 1.0);
        assert_eq!(a.cross(b), 1.0);
        assert_eq!(a.dot(b), 0.0);
        assert_eq!(a.dot(a), 1.0);
    }

    #[test]
    fn vector_length() {
        let v = SkDVector::new(3.0, 4.0);
        assert!((v.length() - 5.0).abs() < 1e-9);
        assert!((v.length_squared() - 25.0).abs() < 1e-9);
    }

    #[test]
    fn point_sub_yields_vector() {
        let a = SkDPoint::new(5.0, 5.0);
        let b = SkDPoint::new(1.0, 2.0);
        let v = a - b;
        assert_eq!(v.f_x, 4.0);
        assert_eq!(v.f_y, 3.0);
    }

    #[test]
    fn point_mid() {
        let a = SkDPoint::new(0.0, 0.0);
        let b = SkDPoint::new(10.0, 20.0);
        let m = SkDPoint::mid(a, b);
        assert_eq!(m, SkDPoint::new(5.0, 10.0));
    }

    #[test]
    fn point_distance() {
        let a = SkDPoint::new(0.0, 0.0);
        let b = SkDPoint::new(3.0, 4.0);
        assert!((a.distance(b) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn point_approximately_equal() {
        let a = SkDPoint::new(1.0, 1.0);
        let b = SkDPoint::new(1.0 + 1e-8, 1.0);
        assert!(a.approximately_equal(b));
        let c = SkDPoint::new(2.0, 2.0);
        assert!(!a.approximately_equal(c));
    }

    #[test]
    fn point_from_to_point() {
        let p = Point::new(1.5, -2.5);
        let d = SkDPoint::from_point(p);
        assert_eq!(d.f_x, 1.5);
        assert_eq!(d.f_y, -2.5);
        assert_eq!(d.to_point(), p);
    }

    #[test]
    fn vector_add_sub_mul() {
        let a = SkDVector::new(1.0, 2.0);
        let b = SkDVector::new(3.0, 4.0);
        assert_eq!(a + b, SkDVector::new(4.0, 6.0));
        assert_eq!(b - a, SkDVector::new(2.0, 2.0));
        assert_eq!(a * 2.0, SkDVector::new(2.0, 4.0));
    }
}
