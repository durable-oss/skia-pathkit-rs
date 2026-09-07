//! Cubic curve clipping utilities.
//!
//! Ported from `src/core/SkCubicClipper.cpp`.

use super::point::Point;
use super::scalar::{self, Scalar};

/// Cubic curve clipping utilities.
///
/// This struct provides tools for working with cubic curves in clipping operations.
/// Currently supports finding the parameter t where a monotonic Y cubic crosses a given Y value.
pub struct SkCubicClipper;

impl SkCubicClipper {
    /// Finds the parameter `t` where a monotonic-in-Y cubic curve crosses the given `y` value.
    ///
    /// The cubic curve is defined by 4 points (start, 2 control points, end). The Y-coordinates
    /// of these points must be monotonic (either non-decreasing or non-increasing).
    ///
    /// Returns `true` and sets `t` to the parameter value where the curve crosses `y`,
    /// or `false` if the curve does not cross `y` (endpoints are on the same side of `y`).
    ///
    /// # Arguments
    ///
    /// * `pts` - Array of 4 points defining the cubic curve
    /// * `y` - The Y-coordinate to find the crossing parameter for
    /// * `t` - Output parameter that will be set to the crossing parameter if found
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::core::SkCubicClipper;
    /// use pathkit::core::Point;
    ///
    /// // Cubic from (0,0) to (1,1) with monotonic Y
    /// let pts = [
    ///     Point::new(0.0, 0.0),
    ///     Point::new(0.33, 0.33),
    ///     Point::new(0.66, 0.66),
    ///     Point::new(1.0, 1.0),
    /// ];
    ///
    /// let mut t = 0.0;
    /// let found = SkCubicClipper::chop_mono_at_y(&pts, 0.5, &mut t);
    /// assert!(found);
    /// // The curve is geometrically a straight line, but its t
    /// // parameterization is not constant-speed, so y(0.5) != 0.5; the
    /// // true root is ~0.503759.
    /// assert!((t - 0.503_759).abs() < 1e-3);
    /// ```
    #[must_use]
    pub fn chop_mono_at_y(pts: &[Point; 4], y: Scalar, t: &mut Scalar) -> bool {
        // Compute y values relative to the target y
        let ycrv = [
            pts[0].y - y,
            pts[1].y - y,
            pts[2].y - y,
            pts[3].y - y,
        ];

        // Check that the endpoints straddle zero
        let (t_neg, t_pos) = if ycrv[0] < 0.0 {
            if ycrv[3] < 0.0 {
                return false;
            }
            (0.0, scalar::SCALAR_1)
        } else if ycrv[0] > 0.0 {
            if ycrv[3] > 0.0 {
                return false;
            }
            (scalar::SCALAR_1, 0.0)
        } else {
            *t = 0.0;
            return true;
        };

        let mut t_neg = t_neg;
        let mut t_pos = t_pos;
        let mut iters = 0;
        let max_iters = 100;

        // Bisection to find the root
        while iters < max_iters {
            let t_mid = (t_pos + t_neg) * scalar::SCALAR_HALF;
            
            // Evaluate the cubic at t_mid using De Casteljau's algorithm
            let y01 = scalar::interp(ycrv[0], ycrv[1], t_mid);
            let y12 = scalar::interp(ycrv[1], ycrv[2], t_mid);
            let y23 = scalar::interp(ycrv[2], ycrv[3], t_mid);
            let y012 = scalar::interp(y01, y12, t_mid);
            let y123 = scalar::interp(y12, y23, t_mid);
            let y0123 = scalar::interp(y012, y123, t_mid);

            if scalar::nearly_zero(y0123, None) {
                *t = t_mid;
                return true;
            }

            if y0123 < 0.0 {
                t_neg = t_mid;
            } else {
                t_pos = t_mid;
            }

            if scalar::nearly_zero(t_pos - t_neg, None) {
                break;
            }

            iters += 1;
        }

        *t = (t_neg + t_pos) * scalar::SCALAR_HALF;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_monotonic_increasing_y() {
        // Cubic from (0,0) to (1,1) with monotonic increasing Y
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(0.33, 0.33),
            Point::new(0.66, 0.66),
            Point::new(1.0, 1.0),
        ];

        let mut t = 0.0;
        let found = SkCubicClipper::chop_mono_at_y(&pts, 0.5, &mut t);
        assert!(found);
        // Geometrically a straight line, but De Casteljau evaluation of a
        // cubic isn't constant-speed for these control points, so y(0.5)
        // != 0.5. The true root is ~0.503759 (checked via independent
        // bisection). chop_mono_at_y stops refining once y is within
        // scalar::NEARLY_ZERO (1/4096) of the target, so its `t` is only
        // accurate to roughly that scale, not machine precision.
        assert!((t - 0.503_759).abs() < 1e-3);
    }

    #[test]
    fn test_monotonic_decreasing_y() {
        // Cubic from (0,1) to (1,0) with monotonic decreasing Y
        let pts = [
            Point::new(0.0, 1.0),
            Point::new(0.33, 0.67),
            Point::new(0.66, 0.33),
            Point::new(1.0, 0.0),
        ];

        let mut t = 0.0;
        let found = SkCubicClipper::chop_mono_at_y(&pts, 0.5, &mut t);
        assert!(found);
        assert!((t - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_no_crossing_same_side() {
        // Cubic with both endpoints above y=0.5
        let pts = [
            Point::new(0.0, 0.6),
            Point::new(0.33, 0.7),
            Point::new(0.66, 0.65),
            Point::new(1.0, 0.8),
        ];

        let mut t = 0.0;
        let found = SkCubicClipper::chop_mono_at_y(&pts, 0.5, &mut t);
        assert!(!found);
    }

    #[test]
    fn test_endpoint_at_target() {
        // Cubic starting exactly at target y
        let pts = [
            Point::new(0.0, 0.5),
            Point::new(0.33, 0.6),
            Point::new(0.66, 0.7),
            Point::new(1.0, 0.8),
        ];

        let mut t = 0.0;
        let found = SkCubicClipper::chop_mono_at_y(&pts, 0.5, &mut t);
        assert!(found);
        assert_eq!(t, 0.0);
    }

    #[test]
    fn test_endpoints_at_target() {
        // Cubic with both endpoints at target y (all at same y)
        let pts = [
            Point::new(0.0, 0.5),
            Point::new(0.33, 0.5),
            Point::new(0.66, 0.5),
            Point::new(1.0, 0.5),
        ];

        let mut t = 0.0;
        let found = SkCubicClipper::chop_mono_at_y(&pts, 0.5, &mut t);
        assert!(found);
    }

    #[test]
    fn test_quadratic_bezier() {
        // Quadratic bezier (degenerate cubic) from (0,0) to (1,1)
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(0.5, 0.5),
            Point::new(0.5, 0.5),
            Point::new(1.0, 1.0),
        ];

        let mut t = 0.0;
        let found = SkCubicClipper::chop_mono_at_y(&pts, 0.5, &mut t);
        assert!(found);
        assert!((t - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_extreme_values() {
        // Cubic with larger coordinate values
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(500.0, 500.0),
            Point::new(1000.0, 1000.0),
            Point::new(1500.0, 1500.0),
        ];

        let mut t = 0.0;
        let found = SkCubicClipper::chop_mono_at_y(&pts, 750.0, &mut t);
        assert!(found);
        assert!((t - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_narrow_range() {
        // Cubic with very narrow y-range
        let pts = [
            Point::new(0.0, 0.4),
            Point::new(0.33, 0.45),
            Point::new(0.66, 0.55),
            Point::new(1.0, 0.6),
        ];

        let mut t = 0.0;
        let found = SkCubicClipper::chop_mono_at_y(&pts, 0.5, &mut t);
        assert!(found);
    }
}
