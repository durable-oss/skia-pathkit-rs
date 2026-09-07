//! Cubic-line intersection computation
//!
//! Port of Skia's SkDCubicLineIntersection.{h,cpp}

use crate::core::{Point, Scalar};

/// Maximum number of cubic roots
const MAX_CUBIC_ROOTS: usize = 3;

/// Maximum number of extrema points (cubic can have up to 2 extrema per axis)
const MAX_EXTREMA: usize = 6;

/// A cubic curve represented by 4 control points
#[derive(Debug, Clone, Copy)]
pub struct DCubic {
    pub p0: Point,
    pub p1: Point,
    pub p2: Point,
    pub p3: Point,
}

impl DCubic {
    pub fn new(p0: Point, p1: Point, p2: Point, p3: Point) -> Self {
        Self { p0, p1, p2, p3 }
    }

    /// Evaluate cubic at parameter t using Bernstein polynomials
    pub fn pt_at_t(&self, t: Scalar) -> Point {
        let one_minus_t = 1.0 - t;
        let one_minus_t2 = one_minus_t * one_minus_t;
        let one_minus_t3 = one_minus_t2 * one_minus_t;
        let t2 = t * t;
        let t3 = t2 * t;

        Point {
            x: one_minus_t3 * self.p0.x
                + 3.0 * one_minus_t2 * t * self.p1.x
                + 3.0 * one_minus_t * t2 * self.p2.x
                + t3 * self.p3.x,
            y: one_minus_t3 * self.p0.y
                + 3.0 * one_minus_t2 * t * self.p1.y
                + 3.0 * one_minus_t * t2 * self.p2.y
                + t3 * self.p3.y,
        }
    }

    /// Get control point at index
    pub fn point(&self, index: usize) -> Point {
        match index {
            0 => self.p0,
            1 => self.p1,
            2 => self.p2,
            3 => self.p3,
            _ => self.p3,
        }
    }

    /// Compute coefficients for polynomial: A*t^3 + B*t^2 + C*t + D
    pub fn coefficients(points: &[Point; 4]) -> (Scalar, Scalar, Scalar, Scalar) {
        let a = points[3].x - 3.0 * points[2].x + 3.0 * points[1].x - points[0].x;
        let b = 3.0 * (points[2].x - 2.0 * points[1].x + points[0].x);
        let c = 3.0 * (points[1].x - points[0].x);
        let d = points[0].x;
        (a, b, c, d)
    }

    /// Find extrema (where derivative = 0)
    pub fn find_extrema(points: &[Point; 4], extrema_ts: &mut [Scalar; MAX_EXTREMA]) -> usize {
        let a = points[3].x - 3.0 * points[2].x + 3.0 * points[1].x - points[0].x;
        let b = 2.0 * (points[2].x - points[1].x) - (points[1].x - points[0].x);
        let c = points[1].x - points[0].x;
        
        let discriminant = b * b - 4.0 * a * c;
        
        if a.abs() < 1e-10 {
            if c.abs() > 1e-10 {
                let t = -c / b;
                if (0.0..=1.0).contains(&t) {
                    extrema_ts[0] = t;
                    1
                } else { 0 }
            } else { 0 }
        } else if discriminant < 0.0 {
            0
        } else {
            let sqrt_disc = discriminant.sqrt();
            let t1 = (-b + sqrt_disc) / (2.0 * a);
            let t2 = (-b - sqrt_disc) / (2.0 * a);
            
            let mut count = 0;
            if (0.0..=1.0).contains(&t1) {
                extrema_ts[count] = t1;
                count += 1;
            }
            if (0.0..=1.0).contains(&t2) && (t1 - t2).abs() > 1e-10 {
                extrema_ts[count] = t2;
                count += 1;
            }
            count
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Axis {
    X,
    Y,
}

/// A line segment
#[derive(Debug, Clone, Copy)]
pub struct DLine {
    pub p0: Point,
    pub p1: Point,
}

impl DLine {
    pub fn new(p0: Point, p1: Point) -> Self {
        Self { p0, p1 }
    }

    pub fn point(&self, index: usize) -> Point {
        match index {
            0 => self.p0,
            _ => self.p1,
        }
    }

    /// Evaluate line at parameter t
    pub fn pt_at_t(&self, t: Scalar) -> Point {
        Point {
            x: self.p0.x + t * (self.p1.x - self.p0.x),
            y: self.p0.y + t * (self.p1.y - self.p0.y),
        }
    }

    /// Find t for point near line (returns -1.0 if outside segment)
    pub fn near_point(pt: Point, line: &DLine) -> Scalar {
        let dx = line.p1.x - line.p0.x;
        let dy = line.p1.y - line.p0.y;
        
        if dx.abs() > dy.abs() {
            let t = (pt.x - line.p0.x) / dx;
            if (0.0..=1.0).contains(&t) { t } else { -1.0 }
        } else {
            let t = (pt.y - line.p0.y) / dy;
            if (0.0..=1.0).contains(&t) { t } else { -1.0 }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cubic_eval_linear() {
        let cubic = DCubic::new(
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(3.0, 0.0),
        );
        
        let pt0 = cubic.pt_at_t(0.0);
        let pt1 = cubic.pt_at_t(0.5);
        let pt2 = cubic.pt_at_t(1.0);
        
        assert!((pt0.x - 0.0).abs() < 1e-10);
        assert!((pt1.x - 1.5).abs() < 1e-10);
        assert!((pt2.x - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_cubic_extrema() {
        let cubic = DCubic::new(
            Point::new(0.0, 0.0),
            Point::new(1.0, 3.0),
            Point::new(2.0, 3.0),
            Point::new(3.0, 0.0),
        );
        
        let mut extrema_ts = [0.0; 6];
        let count = DCubic::find_extrema(
            &[cubic.p0, cubic.p1, cubic.p2, cubic.p3],
            &mut extrema_ts,
        );
        
        assert!(count > 0, "Should find extrema");
        assert!(count <= 2, "Cubic can have at most 2 extrema");
        
        for i in 0..count {
            assert!((0.0..=1.0).contains(&extrema_ts[i]), "Extrema t in [0,1]");
        }
    }

    #[test]
    fn test_line_projection() {
        let line = DLine::new(
            Point::new(0.0, 0.0),
            Point::new(2.0, 0.0),
        );
        
        let pt = Point::new(1.0, 0.0);
        let t = DLine::near_point(pt, &line);
        
        assert!((t - 0.5).abs() < 1e-6, "t should be 0.5");
    }

    #[test]
    fn test_cubic_coefficients() {
        let points = [
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(3.0, 0.0),
        ];
        
        let (a, b, c, d) = DCubic::coefficients(&points);
        
        assert!((a).abs() < 1e-10);
        assert!((b).abs() < 1e-10);
        assert!((c - 3.0).abs() < 1e-10);
        assert!((d).abs() < 1e-10);
    }
}
