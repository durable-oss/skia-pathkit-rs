//! Point operations, including distance computations and normalization.
//!
//! Ported from `src/core/SkPoint.cpp`.

use super::point::{Point, Vector};
use super::scalar::{self, Scalar};
use super::sk_math::sk_ieee_float_divide;

/// Side of a line: left, right, or on the line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    OnLine = 0,
    Left = 1,
    Right = 2,
}

/// Scale a point by a scalar factor.
pub fn scale_point(pt: &Point, scale: Scalar, dst: &mut Point) {
    dst.x = pt.x * scale;
    dst.y = pt.y * scale;
}

/// Normalize `pt` to unit length. Returns `true` if successful.
pub fn normalize(pt: &mut Point) -> bool {
    set_length_to(pt, pt.x, pt.y, scalar::SCALAR_1)
}

/// Set point to normalized version of `(x, y)`. Returns `true` if successful.
pub fn set_normalize(x: Scalar, y: Scalar, pt: &mut Point) -> bool {
    set_length_to(pt, x, y, scalar::SCALAR_1)
}

/// Set `pt` to the given length. Returns `true` if successful.
pub fn set_length(pt: &mut Point, length: Scalar) -> bool {
    set_length_to(pt, pt.x, pt.y, length)
}

/// Compute the length (magnitude) of a vector `(dx, dy)`.
pub fn length(dx: Scalar, dy: Scalar) -> Scalar {
    let mag2 = dx * dx + dy * dy;
    if scalar::is_finite(mag2) {
        mag2.sqrt()
    } else {
        // Fall back to double-precision for overflow protection
        let xx = dx as f64;
        let yy = dy as f64;
        (xx * xx + yy * yy).sqrt() as Scalar
    }
}

/// Set point to `(x, y)` scaled to the given length. Returns `true` if successful.
pub fn set_length_to(pt: &mut Point, x: Scalar, y: Scalar, length: Scalar) -> bool {
    let mag2 = x * x + y * y;

    // Check for overflow to infinity
    if !scalar::is_finite(mag2) {
        // Fall back to double-precision computation
        let xx = x as f64;
        let yy = y as f64;
        let dmag = (xx * xx + yy * yy).sqrt();
        let dscale = sk_ieee_float_divide(length as f64, dmag);

        let new_x = (xx * dscale) as Scalar;
        let new_y = (yy * dscale) as Scalar;

        pt.x = new_x;
        pt.y = new_y;

        // Check for validity
        if !scalar::is_finite(pt.x) || !scalar::is_finite(pt.y) || (pt.x == 0.0 && pt.y == 0.0) {
            pt.x = 0.0;
            pt.y = 0.0;
            return false;
        }
        return true;
    }

    let mag = mag2.sqrt();

    if scalar::nearly_zero(mag, None) {
        pt.x = 0.0;
        pt.y = 0.0;
        return false;
    }

    let scale = length / mag;
    pt.x = x * scale;
    pt.y = y * scale;

    true
}

/// Squared distance from point to the infinite line through a and b.
/// Sets `side` to indicate which side of the line the point is on.
pub fn distance_to_line_between_sqd(
    pt: &Point,
    a: &Point,
    b: &Point,
    side: Option<&mut Side>,
) -> Scalar {
    let u = Vector::new(b.x - a.x, b.y - a.y);
    let v = Vector::new(pt.x - a.x, pt.y - a.y);

    let u_length_sqd = Vector::length_squared(u);
    let det = u.cross(v);

    if let Some(s) = side {
        *s = if det > 0.0 {
            Side::Left
        } else if det < 0.0 {
            Side::Right
        } else {
            Side::OnLine
        };
    }

    let temp = sk_ieee_float_divide(det as f64, u_length_sqd as f64) as Scalar * det;

    if !scalar::is_finite(temp) {
        Vector::length_squared(v)
    } else {
        temp
    }
}

/// Squared distance from point to the line segment from a to b.
pub fn distance_to_line_segment_between_sqd(pt: &Point, a: &Point, b: &Point) -> Scalar {
    let u = Vector::new(b.x - a.x, b.y - a.y);
    let v = Vector::new(pt.x - a.x, pt.y - a.y);

    let u_length_sqd = Vector::length_squared(u);
    let u_dot_v = u.dot(v);

    // Closest point is A
    if u_dot_v <= 0.0 {
        Vector::length_squared(v)
    // Closest point is B
    } else if u_dot_v > u_length_sqd {
        Vector::distance_squared(pt, b)
    // Closest point is on the segment
    } else {
        let det = u.cross(v);
        let temp = sk_ieee_float_divide(det as f64, u_length_sqd as f64) as Scalar * det;

        if !scalar::is_finite(temp) {
            Vector::length_squared(v)
        } else {
            temp
        }
    }
}

/// Extension methods for Point operations.
pub trait PointExt {
    fn scale_to(&self, scale: Scalar, dst: &mut Point);
    fn normalize(&mut self) -> bool;
    fn set_length(&mut self, length: Scalar) -> bool;
    fn length(&self) -> Scalar;
    fn scaled_to_length(&self, length: Scalar) -> Option<Point>;
    fn is_finite(&self) -> bool;
}

impl PointExt for Point {
    fn scale_to(&self, scale: Scalar, dst: &mut Point) {
        scale_point(self, scale, dst);
    }

    fn normalize(&mut self) -> bool {
        set_length_to(self, self.x, self.y, scalar::SCALAR_1)
    }

    fn set_length(&mut self, length: Scalar) -> bool {
        set_length_to(self, self.x, self.y, length)
    }

    fn length(&self) -> Scalar {
        Point::distance_to_origin(self.x, self.y)
    }

    fn scaled_to_length(&self, length: Scalar) -> Option<Point> {
        if set_length_to(&mut Point::new(self.x, self.y), self.x, self.y, length) {
            Some(Point::new(self.x, self.y))
        } else {
            None
        }
    }

    fn is_finite(&self) -> bool {
        scalar::are_finite(self.x, self.y)
    }
}

impl Vector {
    /// Squared length of the vector.
    pub fn length_squared(self) -> Scalar {
        self.x * self.x + self.y * self.y
    }

    /// Squared distance between two points.
    pub fn distance_squared(a: &Point, b: &Point) -> Scalar {
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        dx * dx + dy * dy
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scale() {
        let pt = Point::new(3.0, 4.0);
        let mut dst = Point::default();
        scale_point(&pt, 2.0, &mut dst);
        assert_eq!(dst.x, 6.0);
        assert_eq!(dst.y, 8.0);
    }

    #[test]
    fn test_normalize() {
        let mut pt = Point::new(3.0, 4.0);
        assert!(pt.normalize());
        assert!((pt.x - 0.6).abs() < 1e-6);
        assert!((pt.y - 0.8).abs() < 1e-6);

        let mut zero = Point::new(0.0, 0.0);
        assert!(!zero.normalize());
    }

    #[test]
    fn test_length() {
        assert_eq!(length(3.0, 4.0), 5.0);
        assert_eq!(length(0.0, 0.0), 0.0);
    }

    #[test]
    fn test_set_length_to() {
        let mut pt = Point::new(3.0, 4.0);
        assert!(set_length_to(&mut pt, 3.0, 4.0, 10.0));
        assert_eq!(pt.length(), 10.0);

        let mut zero = Point::new(0.0, 0.0);
        assert!(!set_length_to(&mut zero, 0.0, 0.0, 5.0));
    }

    #[test]
    fn test_distance_to_line_between_sqd() {
        let pt = Point::new(0.0, 1.0);
        let a = Point::new(0.0, 0.0);
        let b = Point::new(1.0, 0.0);

        // Point is 1 unit from the line y=0
        let dist_sqd = distance_to_line_between_sqd(&pt, &a, &b, None);
        assert!((dist_sqd - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_distance_to_line_segment_sqd() {
        // Point projects to interior of segment
        let pt = Point::new(0.5, 1.0);
        let a = Point::new(0.0, 0.0);
        let b = Point::new(1.0, 0.0);
        let dist_sqd = distance_to_line_segment_between_sqd(&pt, &a, &b);
        assert!((dist_sqd - 1.0).abs() < 1e-6);

        // Point projects beyond B
        let pt = Point::new(2.0, 1.0);
        let dist_sqd = distance_to_line_segment_between_sqd(&pt, &a, &b);
        // Distance to B is sqrt(1^2 + 1^2) = sqrt(2), squared = 2
        assert!((dist_sqd - 2.0).abs() < 1e-6);

        // Point projects before A
        let pt = Point::new(-1.0, 1.0);
        let dist_sqd = distance_to_line_segment_between_sqd(&pt, &a, &b);
        // Distance to A is sqrt(1^2 + 1^2) = sqrt(2), squared = 2
        assert!((dist_sqd - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_side() {
        let pt_left = Point::new(0.0, 1.0);
        let pt_right = Point::new(0.0, -1.0);
        let pt_on = Point::new(0.5, 0.0);
        let a = Point::new(0.0, 0.0);
        let b = Point::new(1.0, 0.0);

        let mut side = Side::OnLine;
        distance_to_line_between_sqd(&pt_left, &a, &b, Some(&mut side));
        assert_eq!(side, Side::Left);

        distance_to_line_between_sqd(&pt_right, &a, &b, Some(&mut side));
        assert_eq!(side, Side::Right);

        distance_to_line_between_sqd(&pt_on, &a, &b, Some(&mut side));
        assert_eq!(side, Side::OnLine);
    }

    #[test]
    fn test_vector_length_squared() {
        let v = Vector::new(3.0, 4.0);
        assert_eq!(v.length_squared(), 25.0);
    }

    #[test]
    fn test_distance_squared() {
        let a = Point::new(0.0, 0.0);
        let b = Point::new(3.0, 4.0);
        assert_eq!(Vector::distance_squared(&a, &b), 25.0);
    }
}
