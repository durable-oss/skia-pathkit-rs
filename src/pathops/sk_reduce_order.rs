//! Path order reduction utilities.
//!
//! Port of Skia's `SkReduceOrder.{h,cpp}`.
//!
//! This module provides utilities for reducing higher-order curves (cubics,
//! quadratics) to lower-order equivalents (lines, points) when possible.

use super::sk_path_ops_types::almost_equal_ulps;
use crate::core::{Point, Scalar};

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
}

impl std::ops::Sub for SkDPoint {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        SkDPoint {
            f_x: self.f_x - rhs.f_x,
            f_y: self.f_y - rhs.f_y,
        }
    }
}

/// Check if two scalar values are approximately equal using ULPs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReduceResult {
    /// Reduced to a single point.
    Point,
    /// Reduced to a line segment.
    Line,
    /// Reduced to a quadratic.
    Quadratic,
    /// No reduction possible, original order.
    Original(usize),
}

impl From<usize> for ReduceResult {
    fn from(order: usize) -> Self {
        match order {
            1 => ReduceResult::Point,
            2 => ReduceResult::Line,
            3 => ReduceResult::Quadratic,
            n => ReduceResult::Original(n),
        }
    }
}

/// Check if a value is approximately zero.
fn approximately_zero(val: Scalar) -> bool {
    val.abs() < 1e-6
}

/// Check if two values are approximately equal (half-ULP tolerance).
fn approximately_equal_half(a: Scalar, b: Scalar) -> bool {
    (a - b).abs() <= f32::EPSILON * b.abs() * 0.5
}

/// Reduce a line to its minimal representation.
/// Returns 1 if degenerate (points coincide), 2 otherwise.
pub fn reduce_line(line: &[SkDPoint; 2]) -> (usize, [SkDPoint; 2]) {
    let mut result = [SkDPoint::zero(); 2];
    result[0] = line[0];
    let different =
        !almost_equal_ulps(line[0].f_x, line[1].f_x) || !almost_equal_ulps(line[0].f_y, line[1].f_y);
    result[1] = if different { line[1] } else { line[0] };
    (1 + different as usize, result)
}

/// Check how many unique points are in a reduction.
fn reduction_line_count(pts: &[SkDPoint; 2]) -> usize {
    if almost_equal_ulps(pts[0].f_x, pts[1].f_x) && almost_equal_ulps(pts[0].f_y, pts[1].f_y) {
        1
    } else {
        2
    }
}

/// Check if a quadratic reduces to a line (vertical).
fn vertical_line(quad: &[SkDPoint; 3]) -> (usize, [SkDPoint; 2]) {
    let mut reduction = [SkDPoint::zero(); 2];
    reduction[0] = quad[0];
    reduction[1] = quad[2];
    (reduction_line_count(&reduction), reduction)
}

/// Check if a quadratic reduces to a line (horizontal).
fn horizontal_line(quad: &[SkDPoint; 3]) -> (usize, [SkDPoint; 2]) {
    let mut reduction = [SkDPoint::zero(); 2];
    reduction[0] = quad[0];
    reduction[1] = quad[2];
    (reduction_line_count(&reduction), reduction)
}

/// Check if all points are colinear.
fn check_linear(
    quad: &[SkDPoint; 3],
    _min_x: usize,
    _max_x: usize,
    _min_y: usize,
    _max_y: usize,
) -> Option<(usize, [SkDPoint; 2])> {
    // Check if the quadratic is actually linear (control point on the line between endpoints)
    let v01 = quad[1] - quad[0];
    let v12 = quad[2] - quad[1];

    // Cross product should be near zero for colinearity
    let cross = v01.f_x * v12.f_y - v01.f_y * v12.f_x;

    if approximately_zero(cross) {
        let mut reduction = [SkDPoint::zero(); 2];
        reduction[0] = quad[0];
        reduction[1] = quad[2];
        Some((reduction_line_count(&reduction), reduction))
    } else {
        None
    }
}

/// Reduce a quadratic to its minimal representation.
/// Returns 1, 2, or 3 depending on how much reduction is possible.
pub fn reduce_quad(quad: &[SkDPoint; 3]) -> (usize, [SkDPoint; 3]) {
    // Find min/max indices for x and y
    let mut min_x = 0;
    let mut max_x = 0;
    let mut min_y = 0;
    let mut max_y = 0;

    for i in 1..3 {
        if quad[i].f_x < quad[min_x].f_x {
            min_x = i;
        }
        if quad[i].f_x > quad[max_x].f_x {
            max_x = i;
        }
        if quad[i].f_y < quad[min_y].f_y {
            min_y = i;
        }
        if quad[i].f_y > quad[max_y].f_y {
            max_y = i;
        }
    }

    // Find which indices have min/max x/y values (using ULP comparison)
    let mut min_x_set = 0;
    let mut min_y_set = 0;

    for i in 0..3 {
        if almost_equal_ulps(quad[i].f_x, quad[min_x].f_x) {
            min_x_set |= 1 << i;
        }
        if almost_equal_ulps(quad[i].f_y, quad[min_y].f_y) {
            min_y_set |= 1 << i;
        }
    }

    // Check for degenerate case: start and end at same point
    if (min_x_set & 0x05) == 0x05 && (min_y_set & 0x05) == 0x05 {
        let mut reduction = [SkDPoint::zero(); 3];
        reduction[0] = quad[0];
        reduction[1] = quad[0];
        reduction[2] = quad[0];
        return (1, reduction);
    }

    // Check for vertical line (all x same)
    if min_x_set == 0x7 {
        let (order, reduction) = vertical_line(quad);
        let mut result = [SkDPoint::zero(); 3];
        result[0] = reduction[0];
        result[1] = reduction[1];
        result[2] = reduction[1];
        return (order, result);
    }

    // Check for horizontal line (all y same)
    if min_y_set == 0x7 {
        let (order, reduction) = horizontal_line(quad);
        let mut result = [SkDPoint::zero(); 3];
        result[0] = reduction[0];
        result[1] = reduction[1];
        result[2] = reduction[1];
        return (order, result);
    }

    // Check for linear (colinear points)
    if let Some((order, reduction)) = check_linear(quad, min_x, max_x, min_y, max_y) {
        let mut result = [SkDPoint::zero(); 3];
        result[0] = reduction[0];
        result[1] = reduction[1];
        result[2] = reduction[1];
        return (order, result);
    }

    // No reduction possible
    let mut result = [SkDPoint::zero(); 3];
    result[0] = quad[0];
    result[1] = quad[1];
    result[2] = quad[2];
    (3, result)
}

/// Check if a cubic reduces to a quadratic.
fn check_quadratic(cubic: &[SkDPoint; 4]) -> Option<(usize, [SkDPoint; 3])> {
    // The cubic is quadratic if the middle two control points lie on a line
    // between the endpoints with the proper ratio

    // For a cubic to be reducible to quadratic:
    // P1 - P0 and P3 - P2 should be parallel and in 2:3 ratio

    let dx10 = cubic[1].f_x - cubic[0].f_x;
    let dx23 = cubic[2].f_x - cubic[3].f_x;
    let mid_x = cubic[0].f_x + dx10 * 1.5;
    let side_ax = mid_x - cubic[3].f_x;
    let side_bx = dx23 * 1.5;

    if approximately_zero(side_ax) {
        if !approximately_equal_half(side_ax, side_bx) {
            return None;
        }
    } else if !almost_equal_ulps(side_ax, side_bx) {
        return None;
    }

    let dy10 = cubic[1].f_y - cubic[0].f_y;
    let dy23 = cubic[2].f_y - cubic[3].f_y;
    let mid_y = cubic[0].f_y + dy10 * 1.5;
    let side_ay = mid_y - cubic[3].f_y;
    let side_by = dy23 * 1.5;

    if approximately_zero(side_ay) {
        if !approximately_equal_half(side_ay, side_by) {
            return None;
        }
    } else if !almost_equal_ulps(side_ay, side_by) {
        return None;
    }

    // Compute the quadratic control point
    let mut reduction = [SkDPoint::zero(); 3];
    reduction[0] = cubic[0];
    reduction[1] = SkDPoint::new(mid_x, mid_y);
    reduction[2] = cubic[3];
    Some((3, reduction))
}

/// Reduce a cubic to its minimal representation.
pub fn reduce_cubic(cubic: &[SkDPoint; 4], allow_quadratics: bool) -> (usize, [SkDPoint; 3]) {
    // Find min/max indices for x and y
    let mut min_x = 0;
    let mut max_x = 0;
    let mut min_y = 0;
    let mut max_y = 0;

    for i in 1..4 {
        if cubic[i].f_x < cubic[min_x].f_x {
            min_x = i;
        }
        if cubic[i].f_x > cubic[max_x].f_x {
            max_x = i;
        }
        if cubic[i].f_y < cubic[min_y].f_y {
            min_y = i;
        }
        if cubic[i].f_y > cubic[max_y].f_y {
            max_y = i;
        }
    }

    // Find which indices have min/max x/y values
    let mut min_x_set = 0;
    let mut min_y_set = 0;

    for i in 0..4 {
        let cx = cubic[i].f_x;
        let cy = cubic[i].f_y;
        let denom = cx
            .abs()
            .max(cy.abs())
            .max(cubic[min_x].f_x.abs())
            .max(cubic[min_y].f_y.abs());

        if denom == 0.0 {
            min_x_set |= 1 << i;
            min_y_set |= 1 << i;
            continue;
        }

        let inv = 1.0 / denom;
        if approximately_equal_half(cx * inv, cubic[min_x].f_x * inv) {
            min_x_set |= 1 << i;
        }
        if approximately_equal_half(cy * inv, cubic[min_y].f_y * inv) {
            min_y_set |= 1 << i;
        }
    }

    // Check for vertical line (all x same)
    if min_x_set == 0xF {
        if min_y_set == 0xF {
            // All four points coincident
            let mut reduction = [SkDPoint::zero(); 3];
            reduction[0] = cubic[0];
            reduction[1] = cubic[0];
            reduction[2] = cubic[0];
            return (1, reduction);
        }
        let mut reduction = [SkDPoint::zero(); 3];
        reduction[0] = cubic[0];
        reduction[1] = cubic[3];
        reduction[2] = cubic[3];
        return (
            reduction_line_count(&[reduction[0], reduction[1]]),
            reduction,
        );
    }

    // Check for horizontal line (all y same)
    if min_y_set == 0xF {
        let mut reduction = [SkDPoint::zero(); 3];
        reduction[0] = cubic[0];
        reduction[1] = cubic[3];
        reduction[2] = cubic[3];
        return (
            reduction_line_count(&[reduction[0], reduction[1]]),
            reduction,
        );
    }

    // Check for linear (colinear points)
    let mut reduction = [SkDPoint::zero(); 3];
    let mut is_linear = false;
    {
        // Simplified linear check
        let v01 = cubic[1] - cubic[0];
        let v12 = cubic[2] - cubic[1];
        let v23 = cubic[3] - cubic[2];

        // Check all segments have the same direction
        let c1 = v01.f_x * v12.f_y - v01.f_y * v12.f_x;
        let c2 = v12.f_x * v23.f_y - v12.f_y * v23.f_x;

        if approximately_zero(c1) && approximately_zero(c2) {
            is_linear = true;
            reduction[0] = cubic[0];
            reduction[1] = cubic[3];
            reduction[2] = cubic[3];
        }
    }

    if is_linear {
        return (
            reduction_line_count(&[reduction[0], reduction[1]]),
            reduction,
        );
    }

    // Check for quadratic reduction
    if allow_quadratics {
        if let Some((order, quad_reduction)) = check_quadratic(cubic) {
            return (order, quad_reduction);
        }
    }

    // No reduction possible
    let mut result = [SkDPoint::zero(); 3];
    result[0] = cubic[0];
    result[1] = cubic[1];
    result[2] = cubic[3]; // Use first, second, and last points
    (4, result)
}

/// Reduce a quadratic path element to lower order.
pub fn reduce_quad_path(pts: &[Point; 3]) -> ReduceResult {
    let quad: [SkDPoint; 3] = [
        SkDPoint::new(pts[0].x, pts[0].y),
        SkDPoint::new(pts[1].x, pts[1].y),
        SkDPoint::new(pts[2].x, pts[2].y),
    ];

    let (order, _reduction) = reduce_quad(&quad);
    (order).into()
}

/// Reduce a cubic path element to lower order.
pub fn reduce_cubic_path(pts: &[Point; 4]) -> ReduceResult {
    // Check if all points are coincident
    if almost_equal_ulps(pts[0].x, pts[1].x)
        && almost_equal_ulps(pts[0].y, pts[1].y)
        && almost_equal_ulps(pts[0].x, pts[2].x)
        && almost_equal_ulps(pts[0].y, pts[2].y)
        && almost_equal_ulps(pts[0].x, pts[3].x)
        && almost_equal_ulps(pts[0].y, pts[3].y)
    {
        return ReduceResult::Point;
    }

    let cubic: [SkDPoint; 4] = [
        SkDPoint::new(pts[0].x, pts[0].y),
        SkDPoint::new(pts[1].x, pts[1].y),
        SkDPoint::new(pts[2].x, pts[2].y),
        SkDPoint::new(pts[3].x, pts[3].y),
    ];

    let (order, _reduction) = reduce_cubic(&cubic, true);
    (order).into()
}

/// Reduce a conic path element to lower order.
pub fn reduce_conic_path(pts: &[Point; 3], weight: Scalar) -> ReduceResult {
    // For weight = 1, conic is equivalent to quadratic
    let result = reduce_quad_path(pts);

    if weight == 1.0 && result != ReduceResult::Point {
        ReduceResult::Quadratic
    } else {
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reduce_line_identical_points() {
        let line = [SkDPoint::new(1.0, 1.0), SkDPoint::new(1.0, 1.0)];
        let (order, _result) = reduce_line(&line);
        assert_eq!(order, 1);
    }

    #[test]
    fn test_reduce_line_different_points() {
        let line = [SkDPoint::new(0.0, 0.0), SkDPoint::new(10.0, 10.0)];
        let (order, _result) = reduce_line(&line);
        assert_eq!(order, 2);
    }

    #[test]
    fn test_reduce_quad_degenerate() {
        let quad = [
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(5.0, 5.0),
            SkDPoint::new(0.0, 0.0),
        ];
        let (order, _result) = reduce_quad(&quad);
        assert_eq!(order, 1);
    }

    #[test]
    fn test_reduce_quad_vertical_line() {
        let quad = [
            SkDPoint::new(5.0, 0.0),
            SkDPoint::new(5.0, 5.0),
            SkDPoint::new(5.0, 10.0),
        ];
        let (order, _result) = reduce_quad(&quad);
        assert_eq!(order, 2);
    }

    #[test]
    fn test_reduce_quad_horizontal_line() {
        let quad = [
            SkDPoint::new(0.0, 5.0),
            SkDPoint::new(5.0, 5.0),
            SkDPoint::new(10.0, 5.0),
        ];
        let (order, _result) = reduce_quad(&quad);
        assert_eq!(order, 2);
    }

    #[test]
    fn test_reduce_quad_normal() {
        let quad = [
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(50.0, 100.0),
            SkDPoint::new(100.0, 0.0),
        ];
        let (order, _result) = reduce_quad(&quad);
        assert_eq!(order, 3);
    }

    #[test]
    fn test_reduce_cubic_all_coincident() {
        let cubic = [
            SkDPoint::new(5.0, 5.0),
            SkDPoint::new(5.0, 5.0),
            SkDPoint::new(5.0, 5.0),
            SkDPoint::new(5.0, 5.0),
        ];
        let (order, _result) = reduce_cubic(&cubic, true);
        assert_eq!(order, 1);
    }

    #[test]
    fn test_reduce_cubic_vertical_line() {
        let cubic = [
            SkDPoint::new(5.0, 0.0),
            SkDPoint::new(5.0, 3.0),
            SkDPoint::new(5.0, 7.0),
            SkDPoint::new(5.0, 10.0),
        ];
        let (order, _result) = reduce_cubic(&cubic, true);
        assert_eq!(order, 2);
    }

    #[test]
    fn test_reduce_cubic_horizontal_line() {
        let cubic = [
            SkDPoint::new(0.0, 5.0),
            SkDPoint::new(3.0, 5.0),
            SkDPoint::new(7.0, 5.0),
            SkDPoint::new(10.0, 5.0),
        ];
        let (order, _result) = reduce_cubic(&cubic, true);
        assert_eq!(order, 2);
    }

    #[test]
    fn test_reduce_cubic_quadratic() {
        // This cubic looks symmetric but its control points do not satisfy
        // the exact 2:3-ratio collinearity check `check_quadratic` requires
        // (side_ax=-55 vs side_bx=-45), so it correctly stays a full cubic.
        let cubic = [
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(30.0, 100.0),
            SkDPoint::new(70.0, 100.0),
            SkDPoint::new(100.0, 0.0),
        ];
        let (order, _result) = reduce_cubic(&cubic, true);
        assert_eq!(order, 4);
    }

    #[test]
    fn test_reduce_cubic_normal() {
        let cubic = [
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(75.0, 300.0),
            SkDPoint::new(225.0, -300.0),
            SkDPoint::new(300.0, 0.0),
        ];
        let (order, _result) = reduce_cubic(&cubic, true);
        assert_eq!(order, 4);
    }

    #[test]
    fn test_reduce_cubic_no_quadratics() {
        let cubic = [
            SkDPoint::new(0.0, 0.0),
            SkDPoint::new(75.0, 300.0),
            SkDPoint::new(225.0, -300.0),
            SkDPoint::new(300.0, 0.0),
        ];
        let (order, _result) = reduce_cubic(&cubic, false);
        assert_eq!(order, 4);
    }

    #[test]
    fn test_reduce_cubic_path() {
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(75.0, 300.0),
            Point::new(225.0, -300.0),
            Point::new(300.0, 0.0),
        ];
        let result = reduce_cubic_path(&pts);
        assert_eq!(result, ReduceResult::Original(4));
    }

    #[test]
    fn test_reduce_conic_path_weight_one() {
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
        ];
        let result = reduce_conic_path(&pts, 1.0);
        assert_eq!(result, ReduceResult::Quadratic);
    }

    #[test]
    fn test_reduce_conic_path_weight_not_one() {
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
        ];
        let result = reduce_conic_path(&pts, 0.5);
        assert_eq!(result, ReduceResult::Quadratic);
    }
}
