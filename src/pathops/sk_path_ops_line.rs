//! SkDLine - Line segment operations for path operations
//!
//! Port of Skia's SkPathOpsLine.{h,cpp}
//!
//! This module provides line segment geometry operations including
//! point-on-line tests, interpolation, and near-point calculations.

use super::sk_path_ops_types::{almost_between_ulps, almost_equal_ulps, roughly_equal_ulps};
use crate::core::{Point, Scalar};

/// A line segment defined by two points
#[derive(Debug, Clone, Copy, Default)]
pub struct DLine {
    /// Line endpoints (p0 and p1)
    pub p: [Point; 2],
}

impl DLine {
    /// Creates a new line from two points
    #[must_use]
    pub fn new(p0: Point, p1: Point) -> Self {
        Self { p: [p0, p1] }
    }

    /// Returns a reference to the nth endpoint (0 or 1)
    /// # Panics
    /// Panics if n is not 0 or 1
    #[must_use]
    pub fn get(&self, n: usize) -> &Point {
        assert!(n < 2, "Index must be 0 or 1");
        &self.p[n]
    }

    /// Returns a mutable reference to the nth endpoint
    /// # Panics
    /// Panics if n is not 0 or 1
    pub fn get_mut(&mut self, n: usize) -> &mut Point {
        assert!(n < 2, "Index must be 0 or 1");
        &mut self.p[n]
    }

    /// Interpolates a point on the line at parameter t
    ///
    /// t=0 returns p0, t=1 returns p1, intermediate values return interpolated points
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::pathops::sk_path_ops_line::DLine;
    /// use pathkit::core::Point;
    ///
    /// let line = DLine::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0));
    /// assert_eq!(line.pt_at_t(0.0), Point::new(0.0, 0.0));
    /// assert_eq!(line.pt_at_t(0.5), Point::new(5.0, 0.0));
    /// assert_eq!(line.pt_at_t(1.0), Point::new(10.0, 0.0));
    /// ```
    #[must_use]
    pub fn pt_at_t(&self, t: Scalar) -> Point {
        if t == 0.0 {
            return self.p[0];
        }
        if t == 1.0 {
            return self.p[1];
        }
        let one_t = 1.0 - t;
        Point::new(
            one_t * self.p[0].x + t * self.p[1].x,
            one_t * self.p[0].y + t * self.p[1].y,
        )
    }

    /// Checks if a point is exactly on the line
    ///
    /// Returns 0 if point equals p0, 1 if point equals p1, -1 otherwise
    #[must_use]
    pub fn exact_point(&self, xy: Point) -> Scalar {
        if xy == self.p[0] {
            return 0.0;
        }
        if xy == self.p[1] {
            return 1.0;
        }
        -1.0
    }

    /// Checks if a point is near the line within ULPS tolerance
    ///
    /// Returns t parameter if point is near the line, -1 otherwise
    /// Optionally sets unequal flag if distance differs at ULPS precision
    pub fn near_point(&self, xy: Point, unequal: Option<&mut bool>) -> Scalar {
        // Check if point is within x and y bounds of the line
        if !almost_between_ulps(self.p[0].x, xy.x, self.p[1].x)
            || !almost_between_ulps(self.p[0].y, xy.y, self.p[1].y)
        {
            return -1.0;
        }

        // Project a perpendicular ray from the point to the line; find the T on the line
        let len = Point::new(self.p[1].x - self.p[0].x, self.p[1].y - self.p[0].y);
        let denom = len.x * len.x + len.y * len.y;
        let ab0 = Point::new(xy.x - self.p[0].x, xy.y - self.p[0].y);
        let numer = len.x * ab0.x + len.y * ab0.y;

        if !between(0.0, numer, denom) {
            return -1.0;
        }

        if denom == 0.0 {
            return 0.0;
        }

        let mut t = numer / denom;
        let real_pt = self.pt_at_t(t);
        let dist = Point::distance(real_pt, xy);

        // Find the ordinal in the original line with the largest unsigned exponent
        let tiniest = min4(self.p[0].x, self.p[0].y, self.p[1].x, self.p[1].y);
        let largest = max4(self.p[0].x, self.p[0].y, self.p[1].x, self.p[1].y);
        let largest = largest.max(-tiniest);

        if !almost_equal_ulps(largest, largest + dist) {
            return -1.0;
        }

        if let Some(unequal_ref) = unequal {
            *unequal_ref = (largest as f32) != ((largest + dist) as f32);
        }

        t = pin_t(t);
        debug_assert!(between(0.0, t, 1.0));
        t
    }

    /// Checks if a point is near the line ray (unbounded in one direction)
    ///
    /// Returns true if point is within ULPS tolerance of the line
    #[must_use]
    pub fn near_ray(&self, xy: Point) -> bool {
        // Project a perpendicular ray from the point to the line; find the T on the line
        let len = Point::new(self.p[1].x - self.p[0].x, self.p[1].y - self.p[0].y);
        let denom = len.x * len.x + len.y * len.y;
        let ab0 = Point::new(xy.x - self.p[0].x, xy.y - self.p[0].y);
        let numer = len.x * ab0.x + len.y * ab0.y;
        let t = numer / denom;
        let real_pt = self.pt_at_t(t);
        let dist = Point::distance(real_pt, xy);

        // Find the ordinal in the original line with the largest unsigned exponent
        let tiniest = min4(self.p[0].x, self.p[0].y, self.p[1].x, self.p[1].y);
        let largest = max4(self.p[0].x, self.p[0].y, self.p[1].x, self.p[1].y);
        let largest = largest.max(-tiniest);

        roughly_equal_ulps(largest, largest + dist)
    }

    /// Static function to check if a point is exactly on a horizontal line segment
    ///
    /// Returns 0 if point equals left endpoint, 1 if equals right, -1 otherwise
    #[must_use]
    pub fn exact_point_h(xy: Point, left: Scalar, right: Scalar, y: Scalar) -> Scalar {
        if xy.y == y {
            if xy.x == left {
                return 0.0;
            }
            if xy.x == right {
                return 1.0;
            }
        }
        -1.0
    }

    /// Static function to check if a point is near a horizontal line segment
    ///
    /// Returns t parameter if point is near the segment, -1 otherwise
    #[must_use]
    pub fn near_point_h(xy: Point, left: Scalar, right: Scalar, y: Scalar) -> Scalar {
        if !almost_equal_ulps(xy.y, y) {
            return -1.0;
        }
        if !almost_between_ulps(left, xy.x, right) {
            return -1.0;
        }
        let mut t = (xy.x - left) / (right - left);
        t = pin_t(t);
        debug_assert!(between(0.0, t, 1.0));
        let real_pt_x = (1.0 - t) * left + t * right;
        let dist_sq = (xy.y - y) * (xy.y - y) + (xy.x - real_pt_x) * (xy.x - real_pt_x);
        let dist = dist_sq.sqrt();

        let tiniest = min3(y, left, right);
        let largest = max3(y, left, right);
        let largest = largest.max(-tiniest);

        if !almost_equal_ulps(largest, largest + dist) {
            return -1.0;
        }
        t
    }

    /// Static function to check if a point is exactly on a vertical line segment
    ///
    /// Returns 0 if point equals top endpoint, 1 if equals bottom, -1 otherwise
    #[must_use]
    pub fn exact_point_v(xy: Point, top: Scalar, bottom: Scalar, x: Scalar) -> Scalar {
        if xy.x == x {
            if xy.y == top {
                return 0.0;
            }
            if xy.y == bottom {
                return 1.0;
            }
        }
        -1.0
    }

    /// Static function to check if a point is near a vertical line segment
    ///
    /// Returns t parameter if point is near the segment, -1 otherwise
    #[must_use]
    pub fn near_point_v(xy: Point, top: Scalar, bottom: Scalar, x: Scalar) -> Scalar {
        if !almost_equal_ulps(xy.x, x) {
            return -1.0;
        }
        if !almost_between_ulps(top, xy.y, bottom) {
            return -1.0;
        }
        let mut t = (xy.y - top) / (bottom - top);
        t = pin_t(t);
        debug_assert!(between(0.0, t, 1.0));
        let real_pt_y = (1.0 - t) * top + t * bottom;
        let dist_sq = (xy.x - x) * (xy.x - x) + (xy.y - real_pt_y) * (xy.y - real_pt_y);
        let dist = dist_sq.sqrt();

        let tiniest = min3(x, top, bottom);
        let largest = max3(x, top, bottom);
        let largest = largest.max(-tiniest);

        if !almost_equal_ulps(largest, largest + dist) {
            return -1.0;
        }
        t
    }
}

/// Checks if a < b < c (inclusive)
fn between(a: Scalar, b: Scalar, c: Scalar) -> bool {
    b >= a && b <= c
}

/// Pins t to valid range with looser tolerance
pub fn pin_t(t: Scalar) -> Scalar {
    t.clamp(0.0, 1.0)
}

/// Returns minimum of 3 values
#[inline]
fn min3(a: Scalar, b: Scalar, c: Scalar) -> Scalar {
    a.min(b).min(c)
}

/// Returns minimum of 4 values
#[inline]
fn min4(a: Scalar, b: Scalar, c: Scalar, d: Scalar) -> Scalar {
    a.min(b).min(c).min(d)
}

/// Returns maximum of 3 values
#[inline]
fn max3(a: Scalar, b: Scalar, c: Scalar) -> Scalar {
    a.max(b).max(c)
}

/// Returns maximum of 4 values
#[inline]
fn max4(a: Scalar, b: Scalar, c: Scalar, d: Scalar) -> Scalar {
    a.max(b).max(c).max(d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pt_at_t() {
        let line = DLine::new(Point::new(0.0, 0.0), Point::new(10.0, 10.0));

        assert_eq!(line.pt_at_t(0.0), Point::new(0.0, 0.0));
        assert_eq!(line.pt_at_t(1.0), Point::new(10.0, 10.0));
        assert_eq!(line.pt_at_t(0.5), Point::new(5.0, 5.0));
        assert_eq!(line.pt_at_t(0.25), Point::new(2.5, 2.5));
        assert_eq!(line.pt_at_t(0.75), Point::new(7.5, 7.5));
    }

    #[test]
    fn test_exact_point() {
        let line = DLine::new(Point::new(0.0, 0.0), Point::new(10.0, 10.0));

        assert_eq!(line.exact_point(Point::new(0.0, 0.0)), 0.0);
        assert_eq!(line.exact_point(Point::new(10.0, 10.0)), 1.0);
        assert_eq!(line.exact_point(Point::new(5.0, 5.0)), -1.0);
        assert_eq!(line.exact_point(Point::new(1.0, 2.0)), -1.0);
    }

    #[test]
    fn test_near_point() {
        let line = DLine::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0));

        // Exact endpoints
        assert_eq!(line.near_point(Point::new(0.0, 0.0), None), 0.0);
        assert_eq!(line.near_point(Point::new(10.0, 0.0), None), 1.0);

        // Midpoint
        let t = line.near_point(Point::new(5.0, 0.0), None);
        assert!((t - 0.5).abs() < 1e-10);

        // Point slightly off line (within tolerance)
        let t = line.near_point(Point::new(5.0, 0.00000001), None);
        assert!(t >= 0.0);

        // Point far from line
        assert_eq!(line.near_point(Point::new(5.0, 10.0), None), -1.0);

        // Point outside line bounds
        assert_eq!(line.near_point(Point::new(-1.0, 0.0), None), -1.0);
        assert_eq!(line.near_point(Point::new(11.0, 0.0), None), -1.0);
    }

    #[test]
    fn test_near_point_unequal() {
        let line = DLine::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0));
        let mut unequal = false;

        // Point on line should not set unequal
        let t = line.near_point(Point::new(5.0, 0.0), Some(&mut unequal));
        assert!(t >= 0.0);

        // Check that unequal is only set when distance differs at ULPS precision
        let _ = line.near_point(Point::new(5.0, 0.0001), Some(&mut unequal));
    }

    #[test]
    fn test_near_ray() {
        let line = DLine::new(Point::new(0.0, 0.0), Point::new(10.0, 0.0));

        // Points on line should return true
        assert!(line.near_ray(Point::new(5.0, 0.0)));
        assert!(line.near_ray(Point::new(0.0, 0.0)));

        // Point slightly off line (within tolerance)
        assert!(line.near_ray(Point::new(5.0, 0.00000001)));

        // Point far from line should return false
        assert!(!line.near_ray(Point::new(5.0, 10.0)));
    }

    #[test]
    fn test_exact_point_h() {
        // Horizontal line from (0, 5) to (10, 5)
        let t = DLine::exact_point_h(Point::new(0.0, 5.0), 0.0, 10.0, 5.0);
        assert_eq!(t, 0.0);

        let t = DLine::exact_point_h(Point::new(10.0, 5.0), 0.0, 10.0, 5.0);
        assert_eq!(t, 1.0);

        let t = DLine::exact_point_h(Point::new(5.0, 5.0), 0.0, 10.0, 5.0);
        assert_eq!(t, -1.0); // Not exactly at endpoint

        let t = DLine::exact_point_h(Point::new(5.0, 6.0), 0.0, 10.0, 5.0);
        assert_eq!(t, -1.0); // Wrong y coordinate
    }

    #[test]
    fn test_near_point_h() {
        // Horizontal line from (0, 5) to (10, 5)
        let t = DLine::near_point_h(Point::new(0.0, 5.0), 0.0, 10.0, 5.0);
        assert!((t - 0.0).abs() < 1e-10);

        let t = DLine::near_point_h(Point::new(10.0, 5.0), 0.0, 10.0, 5.0);
        assert!((t - 1.0).abs() < 1e-10);

        let t = DLine::near_point_h(Point::new(5.0, 5.0), 0.0, 10.0, 5.0);
        assert!((t - 0.5).abs() < 1e-10);

        // "Near" means ULP-close, not loosely close: a whole unit off in y
        // is well outside tolerance and must be rejected.
        let t = DLine::near_point_h(Point::new(5.0, 6.0), 0.0, 10.0, 5.0);
        assert_eq!(t, -1.0);
    }

    #[test]
    fn test_exact_point_v() {
        // Vertical line from (5, 0) to (5, 10)
        let t = DLine::exact_point_v(Point::new(5.0, 0.0), 0.0, 10.0, 5.0);
        assert_eq!(t, 0.0);

        let t = DLine::exact_point_v(Point::new(5.0, 10.0), 0.0, 10.0, 5.0);
        assert_eq!(t, 1.0);

        let t = DLine::exact_point_v(Point::new(5.0, 5.0), 0.0, 10.0, 5.0);
        assert_eq!(t, -1.0); // Not exactly at endpoint

        let t = DLine::exact_point_v(Point::new(6.0, 5.0), 0.0, 10.0, 5.0);
        assert_eq!(t, -1.0); // Wrong x coordinate
    }

    #[test]
    fn test_near_point_v() {
        // Vertical line from (5, 0) to (5, 10)
        let t = DLine::near_point_v(Point::new(5.0, 0.0), 0.0, 10.0, 5.0);
        assert!((t - 0.0).abs() < 1e-10);

        let t = DLine::near_point_v(Point::new(5.0, 10.0), 0.0, 10.0, 5.0);
        assert!((t - 1.0).abs() < 1e-10);

        let t = DLine::near_point_v(Point::new(5.0, 5.0), 0.0, 10.0, 5.0);
        assert!((t - 0.5).abs() < 1e-10);

        // "Near" means ULP-close, not loosely close: a whole unit off in x
        // is well outside tolerance and must be rejected.
        let t = DLine::near_point_v(Point::new(6.0, 5.0), 0.0, 10.0, 5.0);
        assert_eq!(t, -1.0);
    }

    #[test]
    fn test_almost_between_ulps() {
        assert!(almost_between_ulps(0.0, 5.0, 10.0));
        assert!(almost_between_ulps(0.0, 0.0, 10.0));
        assert!(almost_between_ulps(0.0, 10.0, 10.0));
        assert!(almost_between_ulps(0.0, -0.00000001, 10.0));
        assert!(almost_between_ulps(0.0, 10.00000001, 10.0));
        assert!(!almost_between_ulps(0.0, 11.0, 10.0));
        assert!(!almost_between_ulps(0.0, -1.0, 10.0));
    }

    #[test]
    fn test_between() {
        assert!(between(0.0, 5.0, 10.0));
        assert!(between(0.0, 0.0, 10.0));
        assert!(between(0.0, 10.0, 10.0));
        assert!(!between(0.0, 11.0, 10.0));
        assert!(!between(0.0, -1.0, 10.0));
    }

    #[test]
    fn test_almost_equal_ulps() {
        assert!(almost_equal_ulps(5.0, 5.0));
        assert!(almost_equal_ulps(5.0, 5.000000001));
        assert!(!almost_equal_ulps(5.0, 5.1));
    }

    #[test]
    fn test_pin_t() {
        assert!((pin_t(0.5) - 0.5).abs() < 1e-10);
        assert!((pin_t(0.0) - 0.0).abs() < 1e-10);
        assert!((pin_t(1.0) - 1.0).abs() < 1e-10);
        assert!((pin_t(-0.5) - 0.0).abs() < 1e-10);
        assert!((pin_t(1.5) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_min_max() {
        assert_eq!(min3(3.0, 1.0, 2.0), 1.0);
        assert_eq!(min4(4.0, 2.0, 1.0, 3.0), 1.0);
        assert_eq!(max3(1.0, 3.0, 2.0), 3.0);
        assert_eq!(max4(1.0, 4.0, 2.0, 3.0), 4.0);
    }

    /// The `tests` table from Skia's `tests/PathOpsDLineTest.cpp`, covering the
    /// degenerate (zero-length), axis-aligned, and diagonal cases together.
    const UPSTREAM_LINES: [[(Scalar, Scalar); 2]; 6] = [
        [(2.0, 1.0), (2.0, 1.0)],
        [(2.0, 1.0), (1.0, 1.0)],
        [(2.0, 1.0), (2.0, 2.0)],
        [(1.0, 1.0), (2.0, 2.0)],
        [(3.0, 0.0), (2.0, 1.0)],
        [(3.0, 2.0), (1.0, 1.0)],
    ];

    #[test]
    fn upstream_line_utilities_midpoint() {
        // DEF_TEST(PathOpsLineUtilities): ptAtT(.5) is the average of the
        // endpoints, degenerate lines included.
        for (index, pts) in UPSTREAM_LINES.iter().enumerate() {
            let line = DLine::new(Point::new(pts[0].0, pts[0].1), Point::new(pts[1].0, pts[1].1));
            let mid = line.pt_at_t(0.5);
            assert!(
                (mid.x - (pts[0].0 + pts[1].0) / 2.0).abs() < 1e-6,
                "tests[{index}] x"
            );
            assert!(
                (mid.y - (pts[0].1 + pts[1].1) / 2.0).abs() < 1e-6,
                "tests[{index}] y"
            );
        }
    }

    #[test]
    fn upstream_line_utilities_round_trip_through_points() {
        // Upstream rebuilds each line from its two SkPoints and requires the
        // endpoints to survive. `DLine` already stores `core::Point`, so this
        // checks that construction preserves them and that t=0/t=1 return them
        // exactly rather than through the interpolation path.
        for (index, pts) in UPSTREAM_LINES.iter().enumerate() {
            let p0 = Point::new(pts[0].0, pts[0].1);
            let p1 = Point::new(pts[1].0, pts[1].1);
            let line = DLine::new(p0, p1);
            assert_eq!(*line.get(0), p0, "tests[{index}] p0");
            assert_eq!(*line.get(1), p1, "tests[{index}] p1");
            assert_eq!(line.pt_at_t(0.0), p0, "tests[{index}] t=0");
            assert_eq!(line.pt_at_t(1.0), p1, "tests[{index}] t=1");
        }
    }

    #[test]
    fn upstream_line_exact_point_matches_endpoints() {
        // `exact_point` is upstream's cheapest endpoint test: 0 for p0, 1 for
        // p1, -1 for anything else. The degenerate first case returns 0 for
        // both, since p0 == p1.
        for (index, pts) in UPSTREAM_LINES.iter().enumerate() {
            let p0 = Point::new(pts[0].0, pts[0].1);
            let p1 = Point::new(pts[1].0, pts[1].1);
            let line = DLine::new(p0, p1);
            assert_eq!(line.exact_point(p0), 0.0, "tests[{index}] p0");
            let expected_p1 = if p0 == p1 { 0.0 } else { 1.0 };
            assert_eq!(line.exact_point(p1), expected_p1, "tests[{index}] p1");
            assert_eq!(
                line.exact_point(Point::new(100.0, 100.0)),
                -1.0,
                "tests[{index}] off-line"
            );
        }
    }
}
