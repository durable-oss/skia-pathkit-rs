//! Conversion from cubic to quadratic curves.
//!
//! Port of Skia's `SkDCubicToQuads.{h,cpp}`.
//!
//! This module provides utilities for approximating cubic Bézier curves
//! with quadratic Bézier curves using degree elevation.

/// A 2D point with double-precision (f64) coordinates.
///
/// Matches Skia's `SkDPoint`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DPoint {
    pub x: f64,
    pub y: f64,
}

impl DPoint {
    /// Constructs a new `DPoint`.
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Returns the midpoint of two points.
    pub const fn mid(a: Self, b: Self) -> Self {
        Self {
            x: (a.x + b.x) * 0.5,
            y: (a.y + b.y) * 0.5,
        }
    }
}

impl std::ops::Add for DPoint {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl std::ops::Sub for DPoint {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl std::ops::Neg for DPoint {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl std::ops::Mul<f64> for DPoint {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl std::ops::AddAssign for DPoint {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl std::ops::SubAssign for DPoint {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

/// A 2D vector (also used as a point displacement).
///
/// Matches Skia's `SkDVector`.
pub type DVector = DPoint;

impl DVector {
    /// Returns the dot product of two vectors.
    pub const fn dot(a: Self, b: Self) -> f64 {
        a.x * b.x + a.y * b.y
    }

    /// Returns the Euclidean length of the vector.
    pub fn length(self) -> f64 {
        self.x.hypot(self.y)
    }
}

/// A cubic Bézier curve defined by 4 control points.
///
/// Matches Skia's `SkDCubic`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DCubic {
    /// The four control points [P0, P1, P2, P3].
    pub pts: [DPoint; 4],
}

impl DCubic {
    /// Number of control points in a cubic.
    pub const POINT_COUNT: usize = 4;

    /// Index of the last point.
    pub const POINT_LAST: usize = Self::POINT_COUNT - 1;

    /// Creates a new cubic from four control points.
    pub const fn new(pts: [DPoint; 4]) -> Self {
        Self { pts }
    }

    /// Returns the point at index `n`.
    pub const fn at(&self, n: usize) -> DPoint {
        self.pts[n]
    }

    /// Returns the start point (P0).
    pub const fn start(&self) -> DPoint {
        self.pts[0]
    }

    /// Returns the end point (P3).
    pub const fn end(&self) -> DPoint {
        self.pts[3]
    }

    /// Converts this cubic to a quadratic using degree elevation.
    ///
    /// This is a degree-elevation technique that finds the best-fitting
    /// quadratic Bézier curve. The quadratic control points are computed as:
    /// - Q0 = C0 (start point)
    /// - Q1 = (3*C1 - C0)/2 averaged with (3*C2 - C3)/2
    /// - Q2 = C3 (end point)
    ///
    /// This matches Skia's `SkDCubic::toQuad()`.
    pub fn to_quad(&self) -> DQuad {
        let c0 = self.pts[0];
        let c1 = self.pts[1];
        let c2 = self.pts[2];
        let c3 = self.pts[3];

        // Compute P1 from the first equation: P1 = (3/2 * C1 - 1/2 * C0)
        let from_c1 = DPoint {
            x: (3.0 * c1.x - c0.x) * 0.5,
            y: (3.0 * c1.y - c0.y) * 0.5,
        };

        // Compute P1 from the second equation: P1 = (3/2 * C2 - 1/2 * C3)
        let from_c2 = DPoint {
            x: (3.0 * c2.x - c3.x) * 0.5,
            y: (3.0 * c2.y - c3.y) * 0.5,
        };

        // Average the two P1 values
        let q1 = DPoint::mid(from_c1, from_c2);

        DQuad::new([c0, q1, c3])
    }
}

/// A quadratic Bézier curve defined by 3 control points.
///
/// Matches Skia's `SkDQuad`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DQuad {
    /// The three control points [P0, P1, P2].
    pub pts: [DPoint; 3],
}

impl DQuad {
    /// Number of control points in a quadratic.
    pub const POINT_COUNT: usize = 3;

    /// Index of the last point.
    pub const POINT_LAST: usize = Self::POINT_COUNT - 1;

    /// Creates a new quadratic from three control points.
    pub const fn new(pts: [DPoint; 3]) -> Self {
        Self { pts }
    }

    /// Returns the point at index `n`.
    pub const fn at(&self, n: usize) -> DPoint {
        self.pts[n]
    }

    /// Returns the start point (P0).
    pub const fn start(&self) -> DPoint {
        self.pts[0]
    }

    /// Returns the control point (P1).
    pub const fn control(&self) -> DPoint {
        self.pts[1]
    }

    /// Returns the end point (P2).
    pub const fn end(&self) -> DPoint {
        self.pts[2]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dpoint_arithmetic() {
        let a = DPoint::new(1.0, 2.0);
        let b = DPoint::new(3.0, 4.0);
        assert_eq!(a + b, DPoint::new(4.0, 6.0));
        assert_eq!(a - b, DPoint::new(-2.0, -2.0));
        assert_eq!(-a, DPoint::new(-1.0, -2.0));
        assert_eq!(a * 2.0, DPoint::new(2.0, 4.0));
    }

    #[test]
    fn test_dpoint_mid() {
        let a = DPoint::new(0.0, 0.0);
        let b = DPoint::new(2.0, 4.0);
        assert_eq!(DPoint::mid(a, b), DPoint::new(1.0, 2.0));
    }

    #[test]
    fn test_dvector_dot() {
        let a = DVector::new(1.0, 2.0);
        let b = DVector::new(3.0, 4.0);
        assert_eq!(DVector::dot(a, b), 11.0);
    }

    #[test]
    fn test_dvector_length() {
        let v = DVector::new(3.0, 4.0);
        assert!((v.length() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_dubic_to_quad_simple() {
        // A simple cubic that should convert to a similar quadratic
        let cubic = DCubic::new([
            DPoint::new(0.0, 0.0),
            DPoint::new(1.0, 1.0),
            DPoint::new(2.0, 1.0),
            DPoint::new(3.0, 0.0),
        ]);

        let quad = cubic.to_quad();

        // Start and end points should be preserved
        assert_eq!(quad.start(), cubic.start());
        assert_eq!(quad.end(), cubic.end());

        // Control point should be computed
        let expected_ctrl = DPoint::new(1.5, 1.5);
        assert!((quad.control().x - expected_ctrl.x).abs() < 1e-10);
        assert!((quad.control().y - expected_ctrl.y).abs() < 1e-10);
    }

    #[test]
    fn test_dubic_to_quad_degree_elevated() {
        // Test with a cubic that was created by degree elevation from a quadratic
        // Original quadratic: P0=(0,0), P1=(2,1), P2=(4,0)
        // After degree elevation:
        //   C0 = P0 = (0,0)
        //   C1 = 1/3*P0 + 2/3*P1 = (4/3, 2/3)
        //   C2 = 2/3*P1 + 1/3*P2 = (8/3, 2/3)
        //   C3 = P2 = (4,0)
        let cubic = DCubic::new([
            DPoint::new(0.0, 0.0),
            DPoint::new(4.0 / 3.0, 2.0 / 3.0),
            DPoint::new(8.0 / 3.0, 2.0 / 3.0),
            DPoint::new(4.0, 0.0),
        ]);

        let quad = cubic.to_quad();

        // Should recover the original quadratic
        assert_eq!(quad.start(), DPoint::new(0.0, 0.0));
        assert_eq!(quad.end(), DPoint::new(4.0, 0.0));
        // Control point should be approximately (2, 1)
        assert!((quad.control().x - 2.0).abs() < 1e-10);
        assert!((quad.control().y - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_dubic_to_quad_linear() {
        // A cubic where all points are colinear (essentially a line)
        let cubic = DCubic::new([
            DPoint::new(0.0, 0.0),
            DPoint::new(1.0, 1.0),
            DPoint::new(2.0, 2.0),
            DPoint::new(3.0, 3.0),
        ]);

        let quad = cubic.to_quad();

        // Start and end should be preserved
        assert_eq!(quad.start(), DPoint::new(0.0, 0.0));
        assert_eq!(quad.end(), DPoint::new(3.0, 3.0));
        // Control point should be on the line
        assert!((quad.control().x - 1.5).abs() < 1e-10);
        assert!((quad.control().y - 1.5).abs() < 1e-10);
    }
}
