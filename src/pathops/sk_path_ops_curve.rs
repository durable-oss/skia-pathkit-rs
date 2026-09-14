//! Double-precision curve types and operations for path operations.
//!
//! Port of Skia's `SkPathOpsCurve.{h,cpp}`.
//!
//! This module provides double-precision variants of points, vectors, lines,
//! quadratics, conics, and cubics used in curve intersection and bounding box
//! computations.

use crate::core::{Point, Scalar};

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
    pub f_x: Scalar,
    pub f_y: Scalar,
}

impl SkDVector {
    pub const fn new(x: Scalar, y: Scalar) -> Self {
        SkDVector { f_x: x, f_y: y }
    }

    pub const fn zero() -> Self {
        SkDVector { f_x: 0.0, f_y: 0.0 }
    }

    pub fn set(&mut self, pt: Point) {
        self.f_x = pt.x;
        self.f_y = pt.y;
    }

    pub fn as_sk_vector(&self) -> Point {
        Point::new(self.f_x, self.f_y)
    }

    /// Cross product (z-component) with another vector.
    pub fn cross(&self, a: SkDVector) -> Scalar {
        self.f_x * a.f_y - self.f_y * a.f_x
    }

    /// Cross product with nearly coincident check (returns 0 if nearly zero).
    pub fn cross_check(&self, a: SkDVector) -> Scalar {
        let xy = self.f_x * a.f_y;
        let yx = self.f_y * a.f_x;
        if almost_equal_ulps(xy, yx) {
            0.0
        } else {
            xy - yx
        }
    }

    /// Dot product with another vector.
    pub fn dot(&self, a: SkDVector) -> Scalar {
        self.f_x * a.f_x + self.f_y * a.f_y
    }

    /// Length (magnitude) of the vector.
    pub fn length(&self) -> Scalar {
        self.length_squared().sqrt()
    }

    /// Squared length of the vector.
    pub fn length_squared(&self) -> Scalar {
        self.f_x * self.f_x + self.f_y * self.f_y
    }

    /// Normalize the vector to unit length.
    pub fn normalize(&mut self) {
        let inv_len = 1.0 / self.length();
        self.f_x *= inv_len;
        self.f_y *= inv_len;
    }

    /// Check if the vector has finite coordinates.
    pub fn is_finite(&self) -> bool {
        self.f_x.is_finite() && self.f_y.is_finite()
    }
}

impl std::ops::Add for SkDVector {
    type Output = SkDVector;
    fn add(self, rhs: SkDVector) -> SkDVector {
        SkDVector {
            f_x: self.f_x + rhs.f_x,
            f_y: self.f_y + rhs.f_y,
        }
    }
}

impl std::ops::Sub for SkDVector {
    type Output = SkDVector;
    fn sub(self, rhs: SkDVector) -> SkDVector {
        SkDVector {
            f_x: self.f_x - rhs.f_x,
            f_y: self.f_y - rhs.f_y,
        }
    }
}

impl std::ops::Mul<Scalar> for SkDVector {
    type Output = SkDVector;
    fn mul(self, rhs: Scalar) -> SkDVector {
        SkDVector {
            f_x: self.f_x * rhs,
            f_y: self.f_y * rhs,
        }
    }
}

/// Double-precision point (2D).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SkDPoint {
    pub f_x: Scalar,
    pub f_y: Scalar,
}

impl SkDPoint {
    pub const fn new(x: Scalar, y: Scalar) -> Self {
        SkDPoint { f_x: x, f_y: y }
    }

    pub const fn zero() -> Self {
        SkDPoint { f_x: 0.0, f_y: 0.0 }
    }

    pub fn set(&mut self, pt: Point) {
        self.f_x = pt.x;
        self.f_y = pt.y;
    }

    /// Convert to Skia's Point (single precision).
    pub fn as_sk_point(&self) -> Point {
        Point::new(self.f_x, self.f_y)
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
            f_x: (a.f_x + b.f_x) / 2.0,
            f_y: (a.f_y + b.f_y) / 2.0,
        }
    }

    /// Check approximate equality using double precision with ULP tolerance.
    pub fn approximately_d_equal(&self, a: SkDPoint) -> bool {
        if self.f_x == a.f_x && self.f_y == a.f_y {
            return true;
        }
        let dist = self.distance(a);
        let tiniest = self.f_x.min(a.f_x).min(self.f_y).min(a.f_y);
        let largest = self.f_x.max(a.f_x).max(self.f_y).max(a.f_y).max(-tiniest);
        almost_equal_ulps(largest, largest + dist)
    }

    /// Check approximate equality with single precision Point.
    pub fn approximately_equal(&self, a: Point) -> bool {
        let d_a = SkDPoint { f_x: a.x, f_y: a.y };
        self.approximately_d_equal(d_a)
    }

    /// Check if approximately zero.
    pub fn approximately_zero(&self) -> bool {
        self.f_x.abs() < 1e-6 && self.f_y.abs() < 1e-6
    }
}

impl std::ops::Sub for SkDPoint {
    type Output = SkDVector;
    fn sub(self, rhs: SkDPoint) -> SkDVector {
        SkDVector {
            f_x: self.f_x - rhs.f_x,
            f_y: self.f_y - rhs.f_y,
        }
    }
}

impl std::ops::Add<SkDVector> for SkDPoint {
    type Output = SkDPoint;
    fn add(self, rhs: SkDVector) -> SkDPoint {
        SkDPoint {
            f_x: self.f_x + rhs.f_x,
            f_y: self.f_y + rhs.f_y,
        }
    }
}

impl std::ops::AddAssign<SkDVector> for SkDPoint {
    fn add_assign(&mut self, rhs: SkDVector) {
        self.f_x += rhs.f_x;
        self.f_y += rhs.f_y;
    }
}

impl std::ops::Sub<SkDVector> for SkDPoint {
    type Output = SkDPoint;
    fn sub(self, rhs: SkDVector) -> SkDPoint {
        SkDPoint {
            f_x: self.f_x - rhs.f_x,
            f_y: self.f_y - rhs.f_y,
        }
    }
}

impl std::ops::Mul<Scalar> for SkDPoint {
    type Output = SkDPoint;
    fn mul(self, rhs: Scalar) -> SkDPoint {
        SkDPoint {
            f_x: self.f_x * rhs,
            f_y: self.f_y * rhs,
        }
    }
}

impl std::ops::Div<Scalar> for SkDPoint {
    type Output = SkDPoint;
    fn div(self, rhs: Scalar) -> SkDPoint {
        SkDPoint {
            f_x: self.f_x / rhs,
            f_y: self.f_y / rhs,
        }
    }
}

/// Double-precision line segment.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDLine {
    pub f_pts: [SkDPoint; 2],
}

impl SkDLine {
    pub fn new() -> Self {
        SkDLine {
            f_pts: [SkDPoint::zero(), SkDPoint::zero()],
        }
    }

    pub fn from_points(p0: SkDPoint, p1: SkDPoint) -> Self {
        SkDLine {
            f_pts: [p0, p1],
        }
    }

    pub fn set(&mut self, pts: [Point; 2]) {
        self.f_pts[0].set(pts[0]);
        self.f_pts[1].set(pts[1]);
    }

    pub fn point(&self, index: usize) -> SkDPoint {
        self.f_pts[index]
    }

    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        &mut self.f_pts[index]
    }

    /// Get point at parameter t (0..1).
    pub fn pt_at_t(&self, t: Scalar) -> SkDPoint {
        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        SkDPoint {
            f_x: p0.f_x + t * (p1.f_x - p0.f_x),
            f_y: p0.f_y + t * (p1.f_y - p0.f_y),
        }
    }

    /// Check if point is approximately on the line.
    pub fn near_point(&self, xy: SkDPoint, unequal: &mut bool) -> Scalar {
        let dist = xy.distance(self.f_pts[0]).min(xy.distance(self.f_pts[1]));
        *unequal = dist > 1e-6;
        dist
    }

    /// Check if a perpendicular ray intersects the line.
    pub fn near_ray(&self, xy: SkDPoint) -> bool {
        let v = self.f_pts[1] - self.f_pts[0];
        let w = xy - self.f_pts[0];
        let c1 = w.dot(v);
        if c1 <= 0.0 {
            return false;
        }
        let c2 = v.dot(v);
        if c2 <= c1 {
            return false;
        }
        let b = c1 / c2;
        let pb = self.f_pts[0] + v * b;
        xy.approximately_d_equal(pb)
    }
}

/// Double-precision quadratic Bezier curve.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDQuad {
    pub f_pts: [SkDPoint; 3],
}

impl SkDQuad {
    pub const K_POINT_COUNT: usize = 3;
    pub const K_POINT_LAST: usize = Self::K_POINT_COUNT - 1;
    pub const K_MAX_INTERSECTIONS: usize = 4;

    pub fn new() -> Self {
        SkDQuad {
            f_pts: [SkDPoint::zero(); 3],
        }
    }

    pub fn from_points(pts: [SkDPoint; 3]) -> Self {
        SkDQuad { f_pts: pts }
    }

    pub fn set(&mut self, pts: [Point; 3]) {
        for i in 0..3 {
            self.f_pts[i].set(pts[i]);
        }
    }

    pub fn point(&self, index: usize) -> SkDPoint {
        self.f_pts[index]
    }

    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        &mut self.f_pts[index]
    }

    /// Check if all points are approximately the same (collapsed).
    pub fn collapsed(&self) -> bool {
        self.f_pts[0].approximately_d_equal(self.f_pts[1])
            && self.f_pts[0].approximately_d_equal(self.f_pts[2])
    }

    /// Check if control point is inside the span.
    pub fn controls_inside(&self) -> bool {
        let v01 = self.f_pts[0] - self.f_pts[1];
        let v02 = self.f_pts[0] - self.f_pts[2];
        let v12 = self.f_pts[1] - self.f_pts[2];
        v02.dot(v01) > 0.0 && v02.dot(v12) > 0.0
    }

    /// Flip the curve (reverse direction).
    pub fn flip(&self) -> SkDQuad {
        SkDQuad {
            f_pts: [self.f_pts[2], self.f_pts[1], self.f_pts[0]],
        }
    }

    /// Get point at parameter t (0..1).
    pub fn pt_at_t(&self, t: Scalar) -> SkDPoint {
        let t_inv = 1.0 - t;
        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        let p2 = self.f_pts[2];
        SkDPoint {
            f_x: t_inv * t_inv * p0.f_x + 2.0 * t_inv * t * p1.f_x + t * t * p2.f_x,
            f_y: t_inv * t_inv * p0.f_y + 2.0 * t_inv * t * p1.f_y + t * t * p2.f_y,
        }
    }

    /// Get derivative (tangent vector) at parameter t.
    pub fn dxdy_at_t(&self, t: Scalar) -> SkDVector {
        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        let p2 = self.f_pts[2];
        let two_t = 2.0 * t;
        SkDVector {
            f_x: two_t * (p1.f_x - p0.f_x) + (1.0 - two_t) * (p2.f_x - p1.f_x),
            f_y: two_t * (p1.f_y - p0.f_y) + (1.0 - two_t) * (p2.f_y - p1.f_y),
        }
    }

    /// Subdivide the curve at t1 and t2.
    pub fn sub_divide(&self, t1: Scalar, t2: Scalar) -> SkDQuad {
        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        let p2 = self.f_pts[2];
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
        if self.f_pts[0].f_x < self.f_pts[1].f_x {
            self.f_pts[1].f_x <= self.f_pts[2].f_x
        } else {
            self.f_pts[1].f_x >= self.f_pts[2].f_x
        }
    }

    /// Check if curve is monotonic in y.
    pub fn monotonic_in_y(&self) -> bool {
        if self.f_pts[0].f_y < self.f_pts[1].f_y {
            self.f_pts[1].f_y <= self.f_pts[2].f_y
        } else {
            self.f_pts[1].f_y >= self.f_pts[2].f_y
        }
    }
}

/// Double-precision conic (rational quadratic) curve.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDConic {
    pub f_pts: SkDQuad,
    pub f_weight: Scalar,
}

/// Numerator of a conic's coordinate at `t`, in homogeneous form.
///
/// Port of `conic_eval_numerator`. `src` holds one coordinate of the three
/// control points.
fn conic_eval_numerator(src: &[Scalar; 3], w: Scalar, t: Scalar) -> Scalar {
    debug_assert!((0.0..=1.0).contains(&t));
    let src1w = src[1] * w;
    let c = src[0];
    let a = src[2] - 2.0 * src1w + c;
    let b = 2.0 * (src1w - c);
    (a * t + b) * t + c
}

/// Denominator of a conic at `t`, in homogeneous form.
///
/// Port of `conic_eval_denominator`.
fn conic_eval_denominator(w: Scalar, t: Scalar) -> Scalar {
    let b = 2.0 * (w - 1.0);
    let c = 1.0;
    let a = -b;
    (a * t + b) * t + c
}

impl SkDConic {
    pub const K_POINT_COUNT: usize = 3;
    pub const K_POINT_LAST: usize = Self::K_POINT_COUNT - 1;
    pub const K_MAX_INTERSECTIONS: usize = 4;

    pub fn new() -> Self {
        SkDConic {
            f_pts: SkDQuad::new(),
            f_weight: 1.0,
        }
    }

    pub fn from_quad(quad: SkDQuad, weight: Scalar) -> Self {
        SkDConic {
            f_pts: quad,
            f_weight: weight,
        }
    }

    pub fn set(&mut self, pts: [Point; 3], weight: Scalar) {
        self.f_pts.set(pts);
        self.f_weight = weight;
    }

    pub fn point(&self, index: usize) -> SkDPoint {
        self.f_pts.point(index)
    }

    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        self.f_pts.point_mut(index)
    }

    pub fn collapsed(&self) -> bool {
        self.f_pts.collapsed()
    }

    pub fn controls_inside(&self) -> bool {
        self.f_pts.controls_inside()
    }

    pub fn flip(&self) -> SkDConic {
        SkDConic {
            f_pts: self.f_pts.flip(),
            f_weight: self.f_weight,
        }
    }

    /// Get point at parameter t (0..1) using rational quadratic evaluation.
    pub fn pt_at_t(&self, t: Scalar) -> SkDPoint {
        let t_inv = 1.0 - t;
        let p0 = self.f_pts.point(0);
        let p1 = self.f_pts.point(1);
        let p2 = self.f_pts.point(2);
        let w = self.f_weight;
        let denom = t_inv * t_inv + 2.0 * w * t_inv * t + t * t;
        SkDPoint {
            f_x: (t_inv * t_inv * p0.f_x + 2.0 * w * t_inv * t * p1.f_x + t * t * p2.f_x) / denom,
            f_y: (t_inv * t_inv * p0.f_y + 2.0 * w * t_inv * t * p1.f_y + t * t * p2.f_y) / denom,
        }
    }

    /// Get derivative at parameter t.
    pub fn dxdy_at_t(&self, t: Scalar) -> SkDVector {
        let p0 = self.f_pts.point(0);
        let p1 = self.f_pts.point(1);
        let p2 = self.f_pts.point(2);
        let w = self.f_weight;
        let t_inv = 1.0 - t;
        let denom = t_inv * t_inv + 2.0 * w * t_inv * t + t * t;
        let d = denom * denom;
        let dx = 2.0 * t_inv * (p1.f_x - p0.f_x) * denom
            + 2.0 * t * (p2.f_x - p1.f_x) * denom
            - 2.0 * (t_inv * t_inv + t * t) * (p0.f_x * t_inv + p1.f_x * w * t + p2.f_x * t);
        let dy = 2.0 * t_inv * (p1.f_y - p0.f_y) * denom
            + 2.0 * t * (p2.f_y - p1.f_y) * denom
            - 2.0 * (t_inv * t_inv + t * t) * (p0.f_y * t_inv + p1.f_y * w * t + p2.f_y * t);
        SkDVector {
            f_x: dx / d,
            f_y: dy / d,
        }
    }

    /// Returns the piece of this conic between `t1` and `t2`.
    ///
    /// Port of `SkDConic::subDivide(double, double)`. The endpoints are
    /// evaluated in homogeneous form and the control point is recovered from
    /// the midpoint, which keeps the result on the original curve. Subdividing
    /// with plain de Casteljau on the projected points does not: the weight
    /// has to travel with the coordinates.
    pub fn sub_divide(&self, t1: Scalar, t2: Scalar) -> SkDConic {
        let xs = [self.f_pts.point(0).f_x, self.f_pts.point(1).f_x, self.f_pts.point(2).f_x];
        let ys = [self.f_pts.point(0).f_y, self.f_pts.point(1).f_y, self.f_pts.point(2).f_y];
        let w = self.f_weight;

        let (ax, ay, az) = if t1 == 0.0 {
            (xs[0], ys[0], 1.0)
        } else if t1 != 1.0 {
            (
                conic_eval_numerator(&xs, w, t1),
                conic_eval_numerator(&ys, w, t1),
                conic_eval_denominator(w, t1),
            )
        } else {
            (xs[2], ys[2], 1.0)
        };

        let mid_t = (t1 + t2) / 2.0;
        let dx = conic_eval_numerator(&xs, w, mid_t);
        let dy = conic_eval_numerator(&ys, w, mid_t);
        let dz = conic_eval_denominator(w, mid_t);

        let (cx, cy, cz) = if t2 == 1.0 {
            (xs[2], ys[2], 1.0)
        } else if t2 != 0.0 {
            (
                conic_eval_numerator(&xs, w, t2),
                conic_eval_numerator(&ys, w, t2),
                conic_eval_denominator(w, t2),
            )
        } else {
            (xs[0], ys[0], 1.0)
        };

        let bx = 2.0 * dx - (ax + cx) / 2.0;
        let by = 2.0 * dy - (ay + cy) / 2.0;
        let mut bz = 2.0 * dz - (az + cz) / 2.0;
        if bz == 0.0 {
            // Weight is 0, so the control point has no effect: any value does.
            bz = 1.0;
        }

        SkDConic {
            f_pts: SkDQuad::from_points([
                SkDPoint::new(ax / az, ay / az),
                SkDPoint::new(bx / bz, by / bz),
                SkDPoint::new(cx / cz, cy / cz),
            ]),
            f_weight: bz / (az * cz).sqrt(),
        }
    }
}

/// Double-precision cubic Bezier curve.
#[derive(Debug, Clone, Copy, Default)]
pub struct SkDCubic {
    pub f_pts: [SkDPoint; 4],
}

impl SkDCubic {
    pub const K_POINT_COUNT: usize = 4;
    pub const K_POINT_LAST: usize = Self::K_POINT_COUNT - 1;
    pub const K_MAX_INTERSECTIONS: usize = 9;

    pub fn new() -> Self {
        SkDCubic {
            f_pts: [SkDPoint::zero(); 4],
        }
    }

    pub fn from_points(pts: [SkDPoint; 4]) -> Self {
        SkDCubic { f_pts: pts }
    }

    pub fn set(&mut self, pts: [Point; 4]) {
        for i in 0..4 {
            self.f_pts[i].set(pts[i]);
        }
    }

    pub fn point(&self, index: usize) -> SkDPoint {
        self.f_pts[index]
    }

    pub fn point_mut(&mut self, index: usize) -> &mut SkDPoint {
        &mut self.f_pts[index]
    }

    /// Check if all points are approximately the same (collapsed).
    pub fn collapsed(&self) -> bool {
        self.f_pts[0].approximately_d_equal(self.f_pts[1])
            && self.f_pts[0].approximately_d_equal(self.f_pts[2])
            && self.f_pts[0].approximately_d_equal(self.f_pts[3])
    }

    /// Check if control points are inside the span.
    pub fn controls_inside(&self) -> bool {
        let v01 = self.f_pts[0] - self.f_pts[1];
        let v02 = self.f_pts[0] - self.f_pts[2];
        let v03 = self.f_pts[0] - self.f_pts[3];
        let v13 = self.f_pts[1] - self.f_pts[3];
        let v23 = self.f_pts[2] - self.f_pts[3];
        v03.dot(v01) > 0.0
            && v03.dot(v02) > 0.0
            && v03.dot(v13) > 0.0
            && v03.dot(v23) > 0.0
    }

    /// Get point at parameter t (0..1).
    pub fn pt_at_t(&self, t: Scalar) -> SkDPoint {
        let t_inv = 1.0 - t;
        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        let p2 = self.f_pts[2];
        let p3 = self.f_pts[3];
        SkDPoint {
            f_x: t_inv * t_inv * t_inv * p0.f_x
                + 3.0 * t_inv * t_inv * t * p1.f_x
                + 3.0 * t_inv * t * t * p2.f_x
                + t * t * t * p3.f_x,
            f_y: t_inv * t_inv * t_inv * p0.f_y
                + 3.0 * t_inv * t_inv * t * p1.f_y
                + 3.0 * t_inv * t * t * p2.f_y
                + t * t * t * p3.f_y,
        }
    }

    /// Get derivative (tangent vector) at parameter t.
    pub fn dxdy_at_t(&self, t: Scalar) -> SkDVector {
        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        let p2 = self.f_pts[2];
        let p3 = self.f_pts[3];
        let t_inv = 1.0 - t;
        SkDVector {
            f_x: 3.0 * t_inv * t_inv * (p1.f_x - p0.f_x)
                + 6.0 * t_inv * t * (p2.f_x - p1.f_x)
                + 3.0 * t * t * (p3.f_x - p2.f_x),
            f_y: 3.0 * t_inv * t_inv * (p1.f_y - p0.f_y)
                + 6.0 * t_inv * t * (p2.f_y - p1.f_y)
                + 3.0 * t * t * (p3.f_y - p2.f_y),
        }
    }

    /// Subdivide the curve at t1 and t2.
    pub fn sub_divide(&self, t1: Scalar, t2: Scalar) -> SkDCubic {
        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        let p2 = self.f_pts[2];
        let p3 = self.f_pts[3];

        // First subdivision at t1
        let q0 = p0;
        let q1 = p0 + (p1 - p0) * t1;
        let q2 = p1 + (p2 - p1) * t1;
        let q3 = p2 + (p3 - p2) * t1;
        let r0 = q0;
        let r1 = q0 + (q1 - q0) * t1;
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
            [self.f_pts[0].f_x, self.f_pts[1].f_x, self.f_pts[2].f_x, self.f_pts[3].f_x]
        } else {
            [self.f_pts[0].f_y, self.f_pts[1].f_y, self.f_pts[2].f_y, self.f_pts[3].f_y]
        };

        // B'(t)/3 as a quadratic in t. Matches SkDCubic::FindExtrema:
        //   A = d - a + 3(b - c),  B = 2(a - 2b + c),  C = b - a
        // The three must share a scale factor, or the roots come out wrong.
        let a = points[3] - points[0] + 3.0 * (points[1] - points[2]);
        let b = 2.0 * (points[0] - 2.0 * points[1] + points[2]);
        let c = points[1] - points[0];

        if a.abs() <= 1e-10 {
            // Degenerates to a line: b*t + c == 0.
            if b.abs() > 1e-10 {
                let t = -c / b;
                if (0.0..=1.0).contains(&t) {
                    result.push(t);
                }
            }
            return result;
        }

        let det = b * b - 4.0 * a * c;
        if det >= 0.0 {
            let root = det.sqrt();
            let t1 = (-b - root) / (2.0 * a);
            let t2 = (-b + root) / (2.0 * a);
            if (0.0..=1.0).contains(&t1) {
                result.push(t1);
            }
            // A repeated root must not be reported twice.
            if (0.0..=1.0).contains(&t2) && (t2 - t1).abs() > 1e-12 {
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
        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        let p2 = self.f_pts[2];
        let p3 = self.f_pts[3];

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
    pub f_curve: SkDCurve,
    pub f_sweep: [SkDVector; 2],
    pub f_is_curve: bool,
    pub f_ordered: bool,
}

impl Default for SkDCurveSweep {
    fn default() -> Self {
        SkDCurveSweep {
            f_curve: SkDCurve::default(),
            f_sweep: [SkDVector::zero(); 2],
            f_is_curve: false,
            f_ordered: true,
        }
    }
}

impl SkDCurveSweep {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_curve(&self) -> bool {
        self.f_is_curve
    }

    pub fn is_ordered(&self) -> bool {
        self.f_ordered
    }

    /// Set up the sweep based on the curve type.
    pub fn set_curve_hull_sweep(&mut self, verb: Verb) {
        self.f_ordered = true;

        let point_count = match verb {
            Verb::Line => 2,
            Verb::Quad | Verb::Conic => 3,
            Verb::Cubic => 4,
            _ => 0,
        };

        if point_count < 2 {
            return;
        }

        self.f_sweep[0] = self.f_curve.point(1) - self.f_curve.point(0);

        match verb {
            Verb::Line => {
                self.f_sweep[1] = self.f_sweep[0];
                self.f_is_curve = false;
            }
            Verb::Quad | Verb::Conic => {
                self.f_sweep[1] = self.f_curve.point(2) - self.f_curve.point(0);

                let max_val = (0..point_count)
                    .map(|i| {
                        self.f_curve.point(i).f_x.abs().max(self.f_curve.point(i).f_y.abs())
                    })
                    .fold(0.0, f32::max);

                if roughly_zero_when_compared_to(self.f_sweep[0].f_x, max_val)
                    && roughly_zero_when_compared_to(self.f_sweep[0].f_y, max_val)
                {
                    self.f_sweep[0] = self.f_sweep[1];
                }
                self.f_is_curve = self.f_sweep[0].cross_check(self.f_sweep[1]) != 0.0;
            }
            Verb::Cubic => {
                self.f_sweep[1] = self.f_curve.point(2) - self.f_curve.point(0);

                let max_val = (0..point_count)
                    .map(|i| {
                        self.f_curve.point(i).f_x.abs().max(self.f_curve.point(i).f_y.abs())
                    })
                    .fold(0.0, f32::max);

                if self.f_sweep[0].f_x == 0.0 && self.f_sweep[0].f_y == 0.0 {
                    self.f_sweep[0] = self.f_sweep[1];
                    self.f_sweep[1] = self.f_curve.point(3) - self.f_curve.point(0);

                    if roughly_zero_when_compared_to(self.f_sweep[0].f_x, max_val)
                        && roughly_zero_when_compared_to(self.f_sweep[0].f_y, max_val)
                    {
                        self.f_sweep[0] = self.f_sweep[1];
                    }
                } else {
                    let third_sweep = self.f_curve.point(3) - self.f_curve.point(0);

                    let s1x3 = self.f_sweep[0].cross_check(third_sweep);
                    let s3x2 = third_sweep.cross_check(self.f_sweep[1]);

                    if s1x3 * s3x2 >= 0.0 {
                        self.f_is_curve = self.f_sweep[0].cross_check(self.f_sweep[1]) != 0.0;
                        return;
                    }

                    let s2x1 = self.f_sweep[1].cross_check(self.f_sweep[0]);

                    if s3x2 * s2x1 < 0.0 {
                        self.f_sweep[0] = self.f_sweep[1];
                        self.f_sweep[1] = third_sweep;
                        self.f_ordered = false;
                    } else {
                        self.f_sweep[1] = third_sweep;
                    }
                }

                self.f_is_curve = self.f_sweep[0].cross_check(self.f_sweep[1]) != 0.0;
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
        assert!((SkDPoint::mid(a, b).f_x - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_skdline() {
        let line = SkDLine::from_points(
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(2.0, 2.0),
        );

        let pt = line.pt_at_t(0.5);
        assert!((pt.f_x - 1.0).abs() < 1e-6);
        assert!((pt.f_y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_skdquad() {
        let quad = SkDQuad::from_points([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(1.0, 0.0),
            SkDPoint::new(1.0, 1.0),
        ]);

        let pt = quad.pt_at_t(0.5);
        assert!((pt.f_y - 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_skdcubic() {
        let cubic = SkDCubic::from_points([
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(0.0, 1.0),
            SkDPoint::new(1.0, 1.0),
            SkDPoint::new(1.0, 0.0),
        ]);

        // B(1/2) = (p0 + 3p1 + 3p2 + p3)/8, so y = (0 + 3 + 3 + 0)/8.
        let pt = cubic.pt_at_t(0.5);
        assert!((pt.f_y - 0.75).abs() < 1e-6);
        assert!((pt.f_x - 0.5).abs() < 1e-6);

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
        assert!((curve.pt_at_t(0.5).f_x - 0.5).abs() < 1e-6);

        let off = SkDVector::new(10.0, 10.0);
        curve.offset(off);
        assert!((curve.point(0).f_x - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_skdcurve_sweep() {
        let mut sweep = SkDCurveSweep::new();
        sweep.f_curve = SkDCurve::Quad(SkDQuad::from_points([
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

    #[test]
    fn conic_sub_divide_stays_on_the_curve() {
        // A quarter circle arc: the weight matters, so a subdivision that
        // ignores it drifts off the curve.
        let w = std::f32::consts::FRAC_1_SQRT_2;
        let conic = SkDConic {
            f_pts: SkDQuad::from_points([
                SkDPoint::new(0.0, 0.0),
                SkDPoint::new(100.0, 0.0),
                SkDPoint::new(100.0, 100.0),
            ]),
            f_weight: w,
        };

        // Every point of the sub-conic must lie on the original arc, at the
        // t value the subdivision maps to.
        let (t1, t2) = (0.25, 0.75);
        let piece = conic.sub_divide(t1, t2);
        for i in 0..=10 {
            let s = i as f32 / 10.0;
            let got = piece.pt_at_t(s);
            let want = conic.pt_at_t(t1 + (t2 - t1) * s);
            assert!(
                (got.f_x - want.f_x).abs() < 0.05 && (got.f_y - want.f_y).abs() < 0.05,
                "at s={s}: got ({}, {}), want ({}, {})",
                got.f_x,
                got.f_y,
                want.f_x,
                want.f_y
            );
        }
    }

    #[test]
    fn conic_sub_divide_over_the_whole_range_is_the_original() {
        let w = std::f32::consts::FRAC_1_SQRT_2;
        let conic = SkDConic {
            f_pts: SkDQuad::from_points([
                SkDPoint::new(0.0, 0.0),
                SkDPoint::new(100.0, 0.0),
                SkDPoint::new(100.0, 100.0),
            ]),
            f_weight: w,
        };
        let whole = conic.sub_divide(0.0, 1.0);
        assert!((whole.f_weight - w).abs() < 1e-5);
        for i in 0..3 {
            let a = whole.f_pts.point(i);
            let b = conic.f_pts.point(i);
            assert!((a.f_x - b.f_x).abs() < 1e-3, "point {i} x");
            assert!((a.f_y - b.f_y).abs() < 1e-3, "point {i} y");
        }
    }

    #[test]
    fn conic_eval_helpers_match_the_rational_form() {
        // At t = 0 and t = 1 the numerator is the first and last coordinate,
        // and the denominator is 1 at both ends.
        let xs = [0.0, 100.0, 100.0];
        let w = 0.5;
        assert!((conic_eval_numerator(&xs, w, 0.0) - 0.0).abs() < 1e-6);
        assert!((conic_eval_numerator(&xs, w, 1.0) - 100.0).abs() < 1e-6);
        assert!((conic_eval_denominator(w, 0.0) - 1.0).abs() < 1e-6);
        assert!((conic_eval_denominator(w, 1.0) - 1.0).abs() < 1e-6);
        // A weight of 1 makes the denominator 1 everywhere: a plain quad.
        for i in 0..=10 {
            let t = i as f32 / 10.0;
            assert!((conic_eval_denominator(1.0, t) - 1.0).abs() < 1e-6);
        }
    }
}
