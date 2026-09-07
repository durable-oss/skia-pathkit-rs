//! 2D points and vectors, in both integer and scalar flavors.
//!
//! Ported from `include/core/SkPoint.h`.

use std::ops::{Add, AddAssign, Neg, Sub, SubAssign};

use super::scalar::{self, Scalar};

/// An integer-coordinate 2D point. Also used as a displacement vector
/// (`IVector` is an alias for `IPoint`, matching Skia).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct IPoint {
    /// X-axis value.
    pub x: i32,
    /// Y-axis value.
    pub y: i32,
}

/// An integer displacement. Alias for [`IPoint`]; interchangeable with it.
pub type IVector = IPoint;

impl IPoint {
    /// Constructs an [`IPoint`] from `(x, y)`.
    #[must_use]
    pub const fn new(x: i32, y: i32) -> Self {
        IPoint { x, y }
    }

    /// Returns `true` if both coordinates are zero.
    #[must_use]
    pub const fn is_zero(self) -> bool {
        self.x == 0 && self.y == 0
    }
}

impl Neg for IPoint {
    type Output = IPoint;
    fn neg(self) -> IPoint {
        IPoint::new(-self.x, -self.y)
    }
}

impl Add<IVector> for IPoint {
    type Output = IPoint;
    fn add(self, rhs: IVector) -> IPoint {
        IPoint::new(self.x.saturating_add(rhs.x), self.y.saturating_add(rhs.y))
    }
}

impl AddAssign<IVector> for IPoint {
    fn add_assign(&mut self, rhs: IVector) {
        *self = *self + rhs;
    }
}

impl Sub for IPoint {
    type Output = IVector;
    fn sub(self, rhs: IPoint) -> IVector {
        IPoint::new(self.x.saturating_sub(rhs.x), self.y.saturating_sub(rhs.y))
    }
}

impl SubAssign<IVector> for IPoint {
    fn sub_assign(&mut self, rhs: IVector) {
        *self = *self - rhs;
    }
}

/// A scalar-coordinate 2D point. Also used as a displacement vector
/// (`Vector` is an alias for `Point`, matching Skia).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
    /// X-axis value.
    pub x: Scalar,
    /// Y-axis value.
    pub y: Scalar,
}

/// A scalar displacement. Alias for [`Point`]; interchangeable with it.
pub type Vector = Point;

/// A 3D homogeneous point (x, y, w).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point3 {
    /// X-axis value.
    pub x: Scalar,
    /// Y-axis value.
    pub y: Scalar,
    /// Homogeneous w component.
    pub z: Scalar,
}

impl Point3 {
    /// Constructs a [`Point3`] from `(x, y, z)`.
    #[must_use]
    pub const fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Point3 { x, y, z }
    }
}

impl Point {
    /// Constructs a [`Point`] from `(x, y)`.
    #[must_use]
    pub const fn new(x: Scalar, y: Scalar) -> Self {
        Point { x, y }
    }

    /// Constructs a [`Point`] from integer coordinates.
    #[must_use]
    pub fn from_ipoint(p: IPoint) -> Self {
        Point::new(p.x as Scalar, p.y as Scalar)
    }

    /// Returns `true` if both coordinates are zero.
    #[must_use]
    pub fn is_zero(self) -> bool {
        self.x == 0.0 && self.y == 0.0
    }

    /// Returns `self` offset by `(dx, dy)`.
    #[must_use]
    pub fn offset(self, dx: Scalar, dy: Scalar) -> Point {
        Point::new(self.x + dx, self.y + dy)
    }

    /// Euclidean distance from the origin.
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::core::Point;
    /// assert_eq!(Point::new(3.0, 4.0).length(), 5.0);
    /// ```
    #[must_use]
    pub fn length(self) -> Scalar {
        Point::distance_to_origin(self.x, self.y)
    }

    /// Returns a unit vector in the same direction as `self`, or `None` if
    /// `self`'s length is zero or nearly zero.
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::core::Point;
    /// let n = Point::new(3.0, 4.0).normalized().unwrap();
    /// assert!((n.length() - 1.0).abs() < 1e-6);
    /// ```
    #[must_use]
    pub fn normalized(self) -> Option<Point> {
        Point::new(self.x, self.y).with_length(self.x, self.y, scalar::SCALAR_1)
    }

    /// Returns `self` scaled to the given `length`, or `None` if `self`'s
    /// current length is zero or nearly zero.
    #[must_use]
    pub fn scaled_to_length(self, length: Scalar) -> Option<Point> {
        self.with_length(self.x, self.y, length)
    }

    /// Returns the vector `(x, y)` scaled to `length`, or `None` if `(x, y)`'s
    /// length is zero or nearly zero.
    #[must_use]
    pub fn with_length(self, x: Scalar, y: Scalar, length: Scalar) -> Option<Point> {
        let mag = Point::distance_to_origin(x, y);
        if scalar::nearly_zero(mag, None) {
            None
        } else {
            let scale = length / mag;
            Some(Point::new(x * scale, y * scale))
        }
    }

    /// Returns `self` multiplied by `scale`.
    #[must_use]
    pub fn scale(self, scale: Scalar) -> Point {
        Point::new(self.x * scale, self.y * scale)
    }

    /// Returns `true` if neither coordinate is NaN or infinite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        scalar::are_finite(self.x, self.y)
    }

    /// Euclidean distance of `(x, y)` from the origin.
    ///
    /// Uses [`f32::hypot`] rather than Skia's `sqrt(x*x + y*y)` to avoid
    /// intermediate overflow for very large coordinates.
    #[must_use]
    pub fn distance_to_origin(x: Scalar, y: Scalar) -> Scalar {
        x.hypot(y)
    }

    /// Euclidean distance between `a` and `b`.
    #[must_use]
    pub fn distance(a: Point, b: Point) -> Scalar {
        Point::distance_to_origin(a.x - b.x, a.y - b.y)
    }

    /// Dot product of vectors `a` and `b`.
    #[must_use]
    pub fn dot_product(a: Vector, b: Vector) -> Scalar {
        a.x * b.x + a.y * b.y
    }

    /// Cross product (z-component) of vectors `a` and `b`.
    #[must_use]
    pub fn cross_product(a: Vector, b: Vector) -> Scalar {
        a.x * b.y - a.y * b.x
    }

    /// Cross product (z-component) of `self` and `vec`.
    #[must_use]
    pub fn cross(self, vec: Vector) -> Scalar {
        Point::cross_product(self, vec)
    }

    /// Dot product of `self` and `vec`.
    #[must_use]
    pub fn dot(self, vec: Vector) -> Scalar {
        Point::dot_product(self, vec)
    }
}

impl Neg for Point {
    type Output = Point;
    fn neg(self) -> Point {
        Point::new(-self.x, -self.y)
    }
}

impl Add<Vector> for Point {
    type Output = Point;
    fn add(self, rhs: Vector) -> Point {
        Point::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign<Vector> for Point {
    fn add_assign(&mut self, rhs: Vector) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Point {
    type Output = Vector;
    fn sub(self, rhs: Point) -> Vector {
        Point::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign<Vector> for Point {
    fn sub_assign(&mut self, rhs: Vector) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl std::ops::Mul<Scalar> for Point {
    type Output = Point;
    fn mul(self, rhs: Scalar) -> Point {
        Point::new(self.x * rhs, self.y * rhs)
    }
}

impl std::ops::MulAssign<Scalar> for Point {
    fn mul_assign(&mut self, rhs: Scalar) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ipoint_arithmetic() {
        let a = IPoint::new(1, 2);
        let b = IPoint::new(3, 4);
        assert_eq!(a + b, IPoint::new(4, 6));
        assert_eq!(b - a, IPoint::new(2, 2));
        assert_eq!(-a, IPoint::new(-1, -2));
    }

    #[test]
    fn ipoint_saturates() {
        let a = IPoint::new(i32::MAX, i32::MIN);
        let b = IPoint::new(1, -1);
        assert_eq!(a + b, IPoint::new(i32::MAX, i32::MIN));
    }

    #[test]
    fn point_length_and_distance() {
        assert_eq!(Point::new(3.0, 4.0).length(), 5.0);
        assert_eq!(
            Point::distance(Point::new(0.0, 0.0), Point::new(3.0, 4.0)),
            5.0
        );
    }

    #[test]
    fn point_normalize() {
        let n = Point::new(3.0, 4.0).normalized().unwrap();
        assert!((n.x - 0.6).abs() < 1e-6);
        assert!((n.y - 0.8).abs() < 1e-6);
        assert!(Point::new(0.0, 0.0).normalized().is_none());
    }

    #[test]
    fn point_dot_cross() {
        let a = Point::new(1.0, 0.0);
        let b = Point::new(0.0, 1.0);
        assert_eq!(Point::dot_product(a, b), 0.0);
        assert_eq!(Point::cross_product(a, b), 1.0);
    }

    #[test]
    fn point_scale_and_arithmetic() {
        let a = Point::new(1.0, 2.0);
        assert_eq!(a.scale(2.0), Point::new(2.0, 4.0));
        assert_eq!(a * 2.0, Point::new(2.0, 4.0));
        assert_eq!(a + Point::new(1.0, 1.0), Point::new(2.0, 3.0));
        assert_eq!(a - Point::new(1.0, 1.0), Point::new(0.0, 1.0));
        assert_eq!(-a, Point::new(-1.0, -2.0));
    }

    #[test]
    fn point_is_finite() {
        assert!(Point::new(1.0, 2.0).is_finite());
        assert!(!Point::new(f32::NAN, 0.0).is_finite());
        assert!(!Point::new(f32::INFINITY, 0.0).is_finite());
    }
}
