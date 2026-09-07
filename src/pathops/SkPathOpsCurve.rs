//! Double-precision curve types and operations for path operations.
//!
//! Port of Skia's `SkPathOpsCurve.{h,cpp}`.
//!
//! This module provides double-precision variants of points, vectors, lines,
//! quadratics, conics, and cubics used in curve intersection and bounding box
//! computations.

use crate::core::{Path, Point, Scalar};

/// Approximate equality comparison using ULPs (units in last place).
fn almost_equal_ulps(a: Scalar, b: Scalar) -> bool {
    (a - b).abs() <= f32::EPSILON * a.abs().max(b.abs()).max(1.0)
}

/// Check if a value is approximately zero relative to a reference value.
fn roughly_zero_when_compared_to(val: Scalar, compared_to: Scalar) -> bool {
    if compared_to.abs() < 1e-6 {
        val.abs() < 1e-6
    } else {
        val.abs() / compared_to.abs() < 1e-4
    }
}

/// Check if a value is between min and max (inclusive), with ULP tolerance.
fn almost_between_ulps(min: Scalar, val: Scalar, max: Scalar) -> bool {
    min <= val && val <= max
}

/// Double-precision vector (2D).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SkDVector {
    pub fX: Scalar,
    pub fY: Scalar,
}

impl SkDVector {
    pub const fn new(x: Scalar, y: Scalar) -> Self {
        SkDVector { fX: x, fY: y }
    }

    pub const fn zero() -> Self {
        SkDVector { fX: 0.0, fY: 0.0 }
    }

    pub fn set(&mut self, pt: Point) {
        self.fX = pt.x;
        self.fY = pt.y;
    }

    pub fn as_sk_vector(&self) -> Point {
        Point::new(self.fX, self.fY)
    }

    /// Cross product (z-component) with another vector.
    pub fn cross(&self, a: SkDVector) -> Scalar {
        self.fX * a.fY - self.fY * a.fX
    }

    /// Cross product with nearly coincident check (returns 0 if nearly zero).
    pub fn cross_check(&self, a: SkDVector) -> Scalar {
        let xy = self.fX * a.fY;
        let yx = self.fY * a.fX;
        if almost_equal_ulps(xy, yx) {
            0.0
        } else {
            xy - yx
        }
    }

    /// Dot product with another vector.
    pub fn dot(&self, a: SkDVector) -> Scalar {
        self.fX * a.fX + self.fY * a.fY
    }

    /// Length (magnitude) of the vector.
    pub fn length(&self) -> Scalar {
        self.length_squared().sqrt()
    }

    /// Squared length of the vector.
    pub fn length_squared(&self) -> Scalar {
        self.fX * self.fX + self.fY * self.fY
    }

    /// Normalize the vector to unit length.
    pub fn normalize(&mut self) {
        let inv_len = 1.0 / self.length();
        self.fX *= inv_len;
        self.fY *= inv_len;
    }

    /// Check if the vector has finite coordinates.
    pub fn is_finite(&self) -> bool {
        self.fX.is_finite() && self.fY.is_finite()
    }
}

impl std::ops::Add for SkDVector {
    type Output = SkDVector;
    fn add(self, rhs: SkDVector) -> SkDVector {
        SkDVector {
            fX: self.fX + rhs.fX,
            fY: self.fY + rhs.fY,
        }
    }
}

impl std::ops::Sub for SkDVector {
    type Output = SkDVector;
    fn sub(self, rhs: SkDVector) -> SkDVector {
        SkDVector {
            fX: self.fX - rhs.fX,
            fY: self.fY - rhs.fY,
        }
    }
}

impl std::ops::Mul<Scalar> for SkDVector {
    type Output = SkDVector;
    fn mul(self, rhs: Scalar) -> SkDVector {
        SkDVector {
            fX: self.fX * rhs,
            fY: self.fY * rhs,
        }
    }
}

/// Double-precision point (2D).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SkDPoint {
    pub fX: Scalar,
    pub fY: Scalar,
}

impl SkDPoint {
    pub const fn new(x: Scalar, y: Scalar) -> Self {
        SkDPoint { fX: x, fY: y }
    }

    pub const fn zero() -> Self {
        SkDPoint { fX: 0.0, fY: 0.0 }
    }

    pub fn set(&mut self, pt: Point) {
        self.fX = pt.x;
        self.fY = pt.y;
    }

    /// Convert to Skia's Point (single precision).
    pub fn as_sk_point(&self) -> Point {
        Point::new(self.fX, self.fY)
    }

    /// Euclidean distance to another point.
    pub fn distance(&self, a: SkDPoint) -> Scalar {
        let diff = *self - a;
        diff.length()
    }

    /// Squared distance to another point.
    pub fn distance_squared(&self, a: SkDPoint) -> Scalar {
        let diff = *self - a;
        diff.length_squared()
    }

    /// Midpoint between two points.
    pub fn mid(a: SkDPoint, b: SkDPoint) -> SkDPoint {
        SkDPoint {
            fX: (a.fX + b.fX) / 2.0,
            fY: (a.fY + b.fY) / 2.0,
        }
    }

    /// Check approximate equality using double precision with ULP tolerance.
    pub fn approximately_d_equal(&self, a: SkDPoint) -> bool {
        if self.fX == a.fX && self.fY == a.fY {
            return true;
        }
        let dist = self.distance(a);
        let tiniest = self.fX.min(a.fX).min(self.fY).min(a.fY);
        let largest = self.fX.max(a.fX).max(self.fY).max(a.fY).max(-tiniest);
        almost_equal_ulps(largest, largest + dist)
    }

    /// Check approximate equality with single precision Point.
    pub fn approximately_equal(&self, a: Point) -> bool {
        let d_a = SkDPoint { fX: a.x, fY: a.y };
        self.approximately_d_equal(d_a)
    }

    /// Check if approximately zero.
    pub fn approximately_zero(&self) -> bool {
        self.fX.abs() < 1e-6 && self.fY.abs() < 1e-6
    }
}

impl std::ops::Sub for SkDPoint {
    type Output = SkDVector;
    fn sub(self, rhs: SkDPoint) -> SkDVector {
        SkDVector {
            fX: self.fX - rhs.fX,
            fY: self.fY - rhs.fY,
        }
    }
}

impl std::ops::Add<SkDVector> for SkDPoint {
    type Output = SkDPoint;
    fn add(self, rhs: SkDVector) -> SkDPoint {
        SkDPoint {
            fX: self.fX + rhs.fX,
            fY: self.fY + rhs.fY,
        }
    }
}

impl std::ops::Sub<SkDVector> for SkDPoint {
    type Output = SkDPoint;
    fn sub(self, rhs: SkDVector) -> SkDPoint {
        SkDPoint {
            fX: self.fX - rhs.fX,
            fY: self.fY - rhs.fY,
        }
    }
}

impl std::ops::Mul<Scalar> for SkDPoint {
    type Output = SkDPoint;
    fn mul(self, rhs: Scalar) -> SkDPoint {
        SkDPoint {
            fX: self.fX * rhs,
            fY: self.fY * rhs,
        }
    }
}

impl std::ops::Div<Scalar> for SkDPoint {
    type Output = SkDPoint;
    fn div(self, rhs: Scalar) -> SkDPoint {
        SkDPoint {
            fX: self.fX / rhs,
            fY: self.fY / rhs,
        }
    }
}

/// Double-precision line segment.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDLine {
    pub fPts: [SkDPoint; 2],
}

impl SkDLine {
    pub fn new() -> Self {
        SkDLine {
            fPts: [SkDPoint::zero(), SkDPoint::zero()],
        }
    }

    pub fn from_points(p0: SkDPoint, p1: SkDPoint) -> Self {
        SkDLine {
            fPts: [p0, p1],
        }
    }

    pub fn set(&mut self, pts: [Point; 2]) {
        self.fPts[0].set(pts[0]);
        self.fPts[1].set(pts[1]);
    }

    pub fn point(&self, index: usize) -> SkDPoint {
        self.fPts[index]
    }

    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        &mut self.fPts[index]
    }

    /// Get point at parameter t (0..1).
    pub fn pt_at_t(&self, t: Scalar) -> SkDPoint {
        let p0 = self.fPts[0];
        let p1 = self.fPts[1];
        SkDPoint {
            fX: p0.fX + t * (p1.fX - p0.fX),
            fY: p0.fY + t * (p1.fY - p0.fY),
        }
    }

    /// Check if point is approximately on the line.
    pub fn near_point(&self, xy: SkDPoint, unequal: &mut bool) -> Scalar {
        let dist = xy.distance(self.fPts[0]).min(xy.distance(self.fPts[1]));
        *unequal = dist > 1e-6;
        dist
    }

    /// Check if a perpendicular ray intersects the line.
    pub fn near_ray(&self, xy: SkDPoint) -> bool {
        let v = self.fPts[1] - self.fPts[0];
        let w = xy - self.fPts[0];
        let c1 = w.dot(v);
        if c1 <= 0.0 {
            return false;
        }
        let c2 = v.dot(v);
        if c2 <= c1 {
            return false;
        }
        let b = c1 / c2;
        let pb = self.fPts[0] + v * b;
        xy.approximately_d_equal(pb)
    }
}

/// Double-precision quadratic Bezier curve.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDQuad {
    pub fPts: [SkDPoint; 3],
}

impl SkDQuad {
    pub const K_POINT_COUNT: usize = 3;
    pub const K_POINT_LAST: usize = Self::K_POINT_COUNT - 1;
    pub const K_MAX_INTERSECTIONS: usize = 4;

    pub fn new() -> Self {
        SkDQuad {
            fPts: [SkDPoint::zero(); 3],
        }
    }

    pub fn from_points(pts: [SkDPoint; 3]) -> Self {
        SkDQuad { fPts: pts }
    }

    pub fn set(&mut self, pts: [Point; 3]) {
        for i in 0..3 {
            self.fPts[i].set(pts[i]);
        }
    }

    pub fn point(&self, index: usize) -> SkDPoint {
        self.fPts[index]
    }

    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        &mut self.fPts[index]
    }

    /// Check if all points are approximately the same (collapsed).
    pub fn collapsed(&self) -> bool {
        self.fPts[0].approximately_d_equal(self.fPts[1])
            && self.fPts[0].approximately_d_equal(self.fPts[2])
    }

    /// Check if control point is inside the span.
    pub fn controls_inside(&self) -> bool {
        let v01 = self.fPts[0] - self.fPts[1];
        let v02 = self.fPts[0] - self.fPts[2];
        let v12 = self.fPts[1] - self.fPts[2];
        v02.dot(v01) > 0.0 && v02.dot(v12) > 0.0
    }

    /// Flip the curve (reverse direction).
    pub fn flip(&self) -> SkDQuad {
        SkDQuad {
            fPts: [self.fPts[2], self.fPts[1], self.fPts[0]],
        }
    }

    /// Get point at parameter t (0..1).
    pub fn pt_at_t(&self, t: Scalar) -> SkDPoint {
        let t_inv = 1.0 - t;
        let p0 = self.fPts[0];
        let p1 = self.fPts[1];
        let p2 = self.fPts[2];
        SkDPoint {
            fX: t_inv * t_inv * p0.fX + 2.0 * t_inv * t * p1.fX + t * t * p2.fX,
            fY: t_inv * t_inv * p0.fY + 2.0 * t_inv * t * p1.fY + t * t * p2.fY,
        }
    }

    /// Get derivative (tangent vector) at parameter t.
    pub fn dxdy_at_t(&self, t: Scalar) -> SkDVector {
        let p0 = self.fPts[0];
        let p1 = self.fPts[1];
        let p2 = self.fPts[2];
        let two_t = 2.0 * t;
        SkDVector {
            fX: two_t * (p1.fX - p0.fX) + (1.0 - two_t) * (p2.fX - p1.fX),
            fY: two_t * (p1.fY - p0.fY) + (1.0 - two_t) * (p2.fY - p1.fY),
        }
    }

    /// Subdivide the curve at t1 and t2.
    pub fn sub_divide(&self, t1: Scalar, t2: Scalar) -> SkDQuad {
        let p0 = self.fPts[0];
        let p1 = self.fPts[1];
        let p2 = self.fPts[2];
        let q0 = p0;
        let q1 = p0 + (p1 - p0) * t1;
        let q2 = p1 + (p2 - p1) * t1;
        let r0 = q0 + (q1 - q0) * ((t2 - t1) / (1.0 - t1));
        let r1 = q1 + (q2 - q1) * ((t2 - t1) / (1.0 - t1));
        let r2 = q2 + (r1 - r0) * ((t2 - t1) / (1.0 - t1));
        SkDQuad::from_points([r0, r1, r2])
    }

    /// Check if curve is monotonic in x.
    pub fn monotonic_in_x(&self) -> bool {
        if self.fPts[0].fX < self.fPts[1].fX {
            self.fPts[1].fX <= self.fPts[2].fX
        } else {
            self.fPts[1].fX >= self.fPts[2].fX
        }
    }

    /// Check if curve is monotonic in y.
    pub fn monotonic_in_y(&self) -> bool {
        if self.fPts[0].fY < self.fPts[1].fY {
            self.fPts[1].fY <= self.fPts[2].fY
        } else {
            self.fPts[1].fY >= self.fPts[2].fY
        }
    }
}

/// Double-precision conic (rational quadratic) curve.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDConic {
    pub fPts: SkDQuad,
    pub fWeight: Scalar,
}

impl SkDConic {
    pub const K_POINT_COUNT: usize = 3;
    pub const K_POINT_LAST: usize = Self::K_POINT_COUNT - 1;
    pub const K_MAX_INTERSECTIONS: usize = 4;

    pub fn new() -> Self {
        SkDConic {
            fPts: SkDQuad::new(),
            fWeight: 1.0,
        }
    }

    pub fn from_quad(quad: SkDQuad, weight: Scalar) -> Self {
        SkDConic {
            fPts: quad,
            fWeight: weight,
        }
    }

    pub fn set(&mut self, pts: [Point; 3], weight: Scalar) {
        self.fPts.set(pts);
        self.fWeight = weight;
    }

    pub fn point(&self, index: usize) -> SkDPoint {
        self.fPts.point(index)
    }

    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        self.fPts.point_mut(index)
    }

    pub fn collapsed(&self) -> bool {
        self.fPts.collapsed()
    }

    pub fn controls_inside(&self) -> bool {
        self.fPts.controls_inside()
    }

    pub fn flip(&self) -> SkDConic {
        SkDConic {
            fPts: self.fPts.flip(),
            fWeight: self.fWeight,
        }
    }

    /// Get point at parameter t (0..1) using rational quadratic evaluation.
    pub fn pt_at_t(&self, t: Scalar) -> SkDPoint {
        let t_inv = 1.0 - t;
        let p0 = self.fPts.point(0);
        let p1 = self.fPts.point(1);
        let p2 = self.fPts.point(2);
        let w = self.fWeight;
        let denom = t_inv * t_inv + 2.0 * w * t_inv * t + t * t;
        SkDPoint {
            fX: (t_inv * t_inv * p0.fX + 2.0 * w * t_inv * t * p1.fX + t * t * p2.fX) / denom,
            fY: (t_inv * t_inv * p0.fY + 2.0 * w * t_inv * t * p1.fY + t * t * p2.fY) / denom,
        }
    }

    /// Get derivative at parameter t.
    pub fn dxdy_at_t(&self, t: Scalar) -> SkDVector {
        let p0 = self.fPts.point(0);
        let p1 = self.fPts.point(1);
        let p2 = self.fPts.point(2);
        let w = self.fWeight;
        let t_inv = 1.0 - t;
        let denom = t_inv * t_inv + 2.0 * w * t_inv * t + t * t;
        let d = denom * denom;
        let dx = 2.0 * t_inv * (p1.fX - p0.fX) * denom
            + 2.0 * t * (p2.fX - p1.fX) * denom
            - 2.0 * (t_inv * t_inv + t * t) * (p0.fX * t_inv + p1.fX * w * t + p2.fX * t);
        let dy = 2.0 * t_inv * (p1.fY - p0.fY) * denom
            + 2.0 * t * (p2.fY - p1.fY) * denom
            - 2.0 * (t_inv * t_inv + t * t) * (p0.fY * t_inv + p1.fY * w * t + p2.fY * t);
        SkDVector {
            fX: dx / d,
            fY: dy / d,
        }
    }

    /// Subdivide the conic at t1 and t2.
    pub fn sub_divide(&self, t1: Scalar, t2: Scalar) -> SkDConic {
        // Use de Casteljau's algorithm for conics
        let p0 = self.fPts.point(0);
        let p1 = self.fPts.point(1);
        let p2 = self.fPts.point(2);
        let w = self.fWeight;

        // First level
        let q0 = p0;
        let q1 = p0 + (p1 - p0) * t1;
        let q2 = p1 + (p2 - p1) * t1;
        let w0 = 1.0;
        let w1 = (w + t1) / 2.0;
        let w2 = (1.0 + t1) / 2.0;

        // Second level
        let r0 = q0 + (q1 - q0) * ((t2 - t1) / (1.0 - t1));
        let r1 = q1 + (q2 - q1) * ((t2 - t1) / (1.0 - t1));
        let w0_new = w0;
        let w1_new = (w0 + w1) / 2.0;
        let w2_new = (w1 + w2) / 2.0;

        SkDConic {
            fPts: SkDQuad::from_points([r0, r1, r1]),
            fWeight: w1_new,
        }
    }
}

/// Double-precision cubic Bezier curve.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDCubic {
    pub fPts: [SkDPoint; 4],
}

impl SkDCubic {
    pub const K_POINT_COUNT: usize = 4;
    pub const K_POINT_LAST: usize = Self::K_POINT_COUNT - 1;
    pub const K_MAX_INTERSECTIONS: usize = 9;

    pub fn new() -> Self {
        SkDCubic {
            fPts: [SkDPoint::zero(); 4],
        }
    }

    pub fn from_points(pts: [SkDPoint; 4]) -> Self {
        SkDCubic { fPts: pts }
    }

    pub fn set(&mut self, pts: [Point; 4]) {
        for i in 0..4 {
            self.fPts[i].set(pts[i]);
        }
    }

    pub fn point(&self, index: usize) -> SkDPoint {
        self.fPts[index]
    }

    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        &mut self.fPts[index]
    }

    /// Check if all points are approximately the same (collapsed).
    pub fn collapsed(&self) -> bool {
        self.fPts[0].approximately_d_equal(self.fPts[1])
            && self.fPts[0].approximately_d_equal(self.fPts[2])
            && self.fPts[0].approximately_d_equal(self.fPts[3])
    }

    /// Check if control points are inside the span.
    pub fn controls_inside(&self) -> bool {
        let v01 = self.fPts[0] - self.fPts[1];
        let v02 = self.fPts[0] - self.fPts[2];
        let v03 = self.fPts[0] - self.fPts[3];
        let v13 = self.fPts[1] - self.fPts[3];
        let v23 = self.fPts[2] - self.fPts[3];
        v03.dot(v01) > 0.0
            && v03.dot(v02) > 0.0
            && v03.dot(v13) > 0.0
            && v03.dot(v23) > 0.0
    }

    /// Get point at parameter t (0..1).
    pub fn pt_at_t(&self, t: Scalar) -> SkDPoint {
        let t_inv = 1.0 - t;
        let p0 = self.fPts[0];
        let p1 = self.fPts[1];
        let p2 = self.fPts[2];
        let p3 = self.fPts[3];
        SkDPoint {
            fX: t_inv * t_inv * t_inv * p0.fX
                + 3.0 * t_inv * t_inv * t * p1.fX
                + 3.0 * t_inv * t * t * p2.fX
                + t * t * t * p3.fX,
            fY: t_inv * t_inv * t_inv * p0.fY
                + 3.0 * t_inv * t_inv * t * p1.fY
                + 3.0 * t_inv * t * t * p2.fY
                + t * t * t * p3.fY,
        }
    }

    /// Get derivative (tangent vector) at parameter t.
    pub fn dxdy_at_t(&self, t: Scalar) -> SkDVector {
        let p0 = self.fPts[0];
        let p1 = self.fPts[1];
        let p2 = self.fPts[2];
        let p3 = self.fPts[3];
        let t_inv = 1.0 - t;
        let two_t = 2.0 * t;
        SkDVector {
            fX: 3.0 * t_inv * t_inv * (p1.fX - p0.fX)
                + 6.0 * t_inv * t * (p2.fX - p1.fX)
                + 3.0 * t * t * (p3.fX - p2.fX),
            fY: 3.0 * t_inv * t_inv * (p1.fY - p0.fY)
                + 6.0 * t_inv * t * (p2.fY - p1.fY)
                + 3.0 * t * t * (p3.fY - p2.fY),
        }
    }

    /// Subdivide the curve at t1 and t2.
    pub fn sub_divide(&self, t1: Scalar, t2: Scalar) -> SkDCubic {
        let p0 = self.fPts[0];
        let p1 = self.fPts[1];
        let p2 = self.fPts[2];
        let p3 = self.fPts[3];

        // First subdivision at t1
        let q0 = p0;
        let q1 = p0 + (p1 - p0) * t1;
        let q2 = p1 + (p2 - p1) * t1;
        let q3 = p2 + (p3 - p2) * t1;
        let r0 = q0;
        let r1 = q0 + (q1 - q0) * t1;
        let r2 = q1 + (q2 - q1) * t1;
        let s0 = r0;
        let s1 = r0 + (r1 - r0) * t1;

        // Second subdivision at t2
        let a = s0 + (s1 - s0) * t2;
        let b = s1 + ((q2 + (q3 - q2) * t1) - s1) * t2;
        let c = (q2 + (q3 - q2) * t1) + ((p3 + (p2 - p3) * (1.0 - t2)) - (q2 + (q3 - q2) * t1)) * t2;
        let d = p3 + (p2 - p3) * (1.0 - t2);

        SkDCubic::from_points([a, b, c, d])
    }

    /// Find extrema in a given coordinate axis.
    pub fn find_extrema(&self, axis: usize) -> Vec<Scalar> {
        let mut result = Vec::new();
        let points = if axis == 0 {
            [self.fPts[0].fX, self.fPts[1].fX, self.fPts[2].fX, self.fPts[3].fX]
        } else {
            [self.fPts[0].fY, self.fPts[1].fY, self.fPts[2].fY, self.fPts[3].fY]
        };

        // Solve derivative = 0 for extrema
        let a = points[3] - 3.0 * points[2] + 3.0 * points[1] - points[0];
        let b = 3.0 * points[2] - 6.0 * points[1] + 3.0 * points[0];
        let c = 3.0 * points[1] - 3.0 * points[0];

        let det = b * b - 4.0 * a * c;
        if det > 0.0 && a.abs() > 1e-10 {
            let t1 = (-b - det.sqrt()) / (2.0 * a);
            let t2 = (-b + det.sqrt()) / (2.0 * a);
            if (0.0..=1.0).contains(&t1) {
                result.push(t1);
            }
            if (0.0..=1.0).contains(&t2) {
                result.push(t2);
            }
        }

        result
    }

    /// Check if monotonic in x.
    pub fn monotonic_in_x(&self) -> bool {
        self.find_extrema(0).is_empty()
    }

    /// Check if monotonic in y.
    pub fn monotonic_in_y(&self) -> bool {
        self.find_extrema(1).is_empty()
    }

    /// Convert to quad approximation.
    pub fn to_quad(&self) -> SkDQuad {
        // Use a best-fit quadratic approximation
        let p0 = self.fPts[0];
        let p1 = self.fPts[1];
        let p2 = self.fPts[2];
        let p3 = self.fPts[3];

        let mid = SkDPoint::mid(p0, p3);
        let ctrl = p1 + (p2 - p1) * 0.5;

        SkDQuad::from_points([p0, ctrl, p3])
    }
}

/// Enum to represent different curve types (unified interface).
#[derive(Debug, Clone, Copy)]
pub enum SkDCurve {
    Line(SkDLine),
    Quad(SkDQuad),
    Conic(SkDConic),
    Cubic(SkDCubic),
}

impl Default for SkDCurve {
    fn default() -> Self {
        SkDCurve::Line(SkDLine::new())
    }
}

impl SkDCurve {
    /// Get point at index.
    pub fn point(&self, index: usize) -> SkDPoint {
        match self {
            SkDCurve::Line(l) => l.point(index),
            SkDCurve::Quad(q) => q.point(index),
            SkDCurve::Conic(c) => c.point(index),
            SkDCurve::Cubic(c) => c.point(index),
        }
    }

    /// Get mutable reference to point at index.
    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        match self {
            SkDCurve::Line(l) => l.point_mut(index),
            SkDCurve::Quad(q) => q.point_mut(index),
            SkDCurve::Conic(c) => c.point_mut(index),
            SkDCurve::Cubic(c) => c.point_mut(index),
        }
    }

    /// Get the curve type point count.
    pub fn point_count(&self) -> usize {
        match self {
            SkDCurve::Line(_) => 2,
            SkDCurve::Quad(_) => 3,
            SkDCurve::Conic(_) => 3,
            SkDCurve::Cubic(_) => 4,
        }
    }

    /// Offset the curve by a vector.
    pub fn offset(&mut self, off: SkDVector) {
        for i in 0..self.point_count() {
            *self.point_mut(i) += off;
        }
    }

    /// Get point at parameter t.
    pub fn pt_at_t(&self, t: Scalar) -> SkDPoint {
        match self {
            SkDCurve::Line(l) => l.pt_at_t(t),
            SkDCurve::Quad(q) => q.pt_at_t(t),
            SkDCurve::Conic(c) => c.pt_at_t(t),
            SkDCurve::Cubic(c) => c.pt_at_t(t),
        }
    }

    /// Get derivative at parameter t.
    pub fn dxdy_at_t(&self, t: Scalar) -> SkDVector {
        match self {
            SkDCurve::Line(_) => SkDVector::zero(),
            SkDCurve::Quad(q) => q.dxdy_at_t(t),
            SkDCurve::Conic(c) => c.dxdy_at_t(t),
            SkDCurve::Cubic(c) => c.dxdy_at_t(t),
        }
    }

    /// Check if curve is monotonic in x.
    pub fn monotonic_in_x(&self) -> bool {
        match self {
            SkDCurve::Line(_) => true,
            SkDCurve::Quad(q) => q.monotonic_in_x(),
            SkDCurve::Conic(_) => true,
            SkDCurve::Cubic(c) => c.monotonic_in_x(),
        }
    }

    /// Check if curve is monotonic in y.
    pub fn monotonic_in_y(&self) -> bool {
        match self {
            SkDCurve::Line(_) => true,
            SkDCurve::Quad(q) => q.monotonic_in_y(),
            SkDCurve::Conic(_) => true,
            SkDCurve::Cubic(c) => c.monotonic_in_y(),
        }
    }
}

/// Represents a curve's convex hull sweep for intersection testing.
#[derive(Debug, Clone)]
pub struct SkDCurveSweep {
    pub fCurve: SkDCurve,
    pub fSweep: [SkDVector; 2],
    pub fIsCurve: bool,
    pub fOrdered: bool,
}

impl Default for SkDCurveSweep {
    fn default() -> Self {
        SkDCurveSweep {
            fCurve: SkDCurve::default(),
            fSweep: [SkDVector::zero(); 2],
            fIsCurve: false,
            fOrdered: true,
        }
    }
}

impl SkDCurveSweep {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_curve(&self) -> bool {
        self.fIsCurve
    }

    pub fn is_ordered(&self) -> bool {
        self.fOrdered
    }

    /// Set up the sweep based on the curve type.
    pub fn set_curve_hull_sweep(&mut self, verb: Verb) {
        self.fOrdered = true;

        let point_count = match verb {
            Verb::Line => 2,
            Verb::Quad | Verb::Conic => 3,
            Verb::Cubic => 4,
            _ => 0,
        };

        if point_count < 2 {
            return;
        }

        self.fSweep[0] = self.fCurve.point(1) - self.fCurve.point(0);

        match verb {
            Verb::Line => {
                self.fSweep[1] = self.fSweep[0];
                self.fIsCurve = false;
            }
            Verb::Quad | Verb::Conic => {
                self.fSweep[1] = self.fCurve.point(2) - self.fCurve.point(0);

                let max_val = (0..point_count)
                    .map(|i| {
                        self.fCurve.point(i).fX.abs().max(self.fCurve.point(i).fY.abs())
                    })
                    .fold(0.0, f32::max);

                if roughly_zero_when_compared_to(self.fSweep[0].fX, max_val)
                    && roughly_zero_when_compared_to(self.fSweep[0].fY, max_val)
                {
                    self.fSweep[0] = self.fSweep[1];
                }
                self.fIsCurve = self.fSweep[0].cross_check(self.fSweep[1]) != 0.0;
            }
            Verb::Cubic => {
                self.fSweep[1] = self.fCurve.point(2) - self.fCurve.point(0);

                let max_val = (0..point_count)
                    .map(|i| {
                        self.fCurve.point(i).fX.abs().max(self.fCurve.point(i).fY.abs())
                    })
                    .fold(0.0, f32::max);

                if self.fSweep[0].fX == 0.0 && self.fSweep[0].fY == 0.0 {
                    self.fSweep[0] = self.fSweep[1];
                    self.fSweep[1] = self.fCurve.point(3) - self.fCurve.point(0);

                    if roughly_zero_when_compared_to(self.fSweep[0].fX, max_val)
                        && roughly_zero_when_compared_to(self.fSweep[0].fY, max_val)
                    {
                        self.fSweep[0] = self.fSweep[1];
                    }
                } else {
                    let third_sweep = self.fCurve.point(3) - self.fCurve.point(0);

                    let s1x3 = self.fSweep[0].cross_check(third_sweep);
                    let s3x2 = third_sweep.cross_check(self.fSweep[1]);

                    if s1x3 * s3x2 >= 0.0 {
                        self.fIsCurve = self.fSweep[0].cross_check(self.fSweep[1]) != 0.0;
                        return;
                    }

                    let s2x1 = self.fSweep[1].cross_check(self.fSweep[0]);

                    if s3x2 * s2x1 < 0.0 {
                        self.fSweep[0] = self.fSweep[1];
                        self.fSweep[1] = third_sweep;
                        self.fOrdered = false;
                    } else {
                        self.fSweep[1] = third_sweep;
                    }
                }

                self.fIsCurve = self.fSweep[0].cross_check(self.fSweep[1]) != 0.0;
            }
            _ => {}
        }
    }
}

/// Verb type for path segments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verb {
    Move,
    Line,
    Quad,
    Conic,
    Cubic,
    Close,
}

impl Verb {
    /// Get point count for a verb.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_skdvector_operations() {
        let a = SkDVector::new(3.0, 4.0);
        let b = SkDVector::new(1.0, 2.0);

        assert!((a.length() - 5.0).abs() < 1e-6);
        assert!((a.cross(b) - 2.0).abs() < 1e-6);
        assert!((a.dot(b) - 11.0).abs() < 1e-6);
    }

    #[test]
    fn test_skdpoint_operations() {
        let a = SkDPoint::new(0.0, 0.0);
        let b = SkDPoint::new(3.0, 4.0);

        assert!((a.distance(b) - 5.0).abs() < 1e-6);
        assert!((SkDPoint::mid(a, b).fX - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_skdline() {
        let line = SkDLine::from_points(
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(2.0, 2.0),
        );

        let pt = line.pt_at_t(0.5);
        assert!((pt.fX - 1.0).abs() < 1e-6);
        assert!((pt.fY - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_skdquad() {
        let quad = SkDQuad::from_points([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(1.0, 0.0),
            SkDPoint::new(1.0, 1.0),
        ]);

        let pt = quad.pt_at_t(0.5);
        assert!((pt.fY - 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_skdcubic() {
        let cubic = SkDCubic::from_points([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(0.0, 1.0),
            SkDPoint::new(1.0, 1.0),
            SkDPoint::new(1.0, 0.0),
        ]);

        let pt = cubic.pt_at_t(0.5);
        assert!((pt.fY - 0.5).abs() < 1e-6);

        let extrema = cubic.find_extrema(1);
        assert_eq!(extrema.len(), 1);
        assert!((extrema[0] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_skdcurve_enum() {
        let mut curve = SkDCurve::Line(SkDLine::from_points(
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(1.0, 1.0),
        ));

        assert_eq!(curve.point_count(), 2);
        assert!((curve.pt_at_t(0.5).fX - 0.5).abs() < 1e-6);

        let off = SkDVector::new(10.0, 10.0);
        curve.offset(off);
        assert!((curve.point(0).fX - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_skdcurve_sweep() {
        let mut sweep = SkDCurveSweep::new();
        sweep.fCurve = SkDCurve::Quad(SkDQuad::from_points([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(1.0, 0.0),
            SkDPoint::new(1.0, 1.0),
        ]));

        sweep.set_curve_hull_sweep(Verb::Quad);

        assert!(sweep.is_curve());
        assert!(sweep.is_ordered());
    }

    #[test]
    fn test_verb_point_counts() {
        assert_eq!(Verb::Line.point_count(), 2);
        assert_eq!(Verb::Quad.point_count(), 3);
        assert_eq!(Verb::Conic.point_count(), 3);
        assert_eq!(Verb::Cubic.point_count(), 4);
    }
}
