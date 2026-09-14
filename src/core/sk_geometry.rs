//! Geometry utilities for Bezier curves (quadratics, cubics, conics).
//!
//! Ported from `src/core/SkGeometry.cpp`.

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

use super::{point::Point, point::Vector, scalar, scalar::Scalar};
use crate::core::point::Point3;

/// Result of classifying a cubic curve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CubicType {
    /// Straight line or single point.
    LineOrPoint,
    /// Quadratic bezier (one control point is collinear with endpoints).
    Quadratic,
    /// Local cusp (derivative goes to zero).
    LocalCusp,
    /// Cusp at infinity.
    CuspAtInfinity,
    /// Loop (curve intersects itself).
    Loop,
    /// Serpentine (has inflection points but no loop or cusp).
    Serpentine,
}

/// Finds roots of the quadratic equation A*x² + B*x + C = 0 that lie in [0, 1).
///
/// Returns the number of valid roots found (0, 1, or 2).
/// Roots are sorted ascending and duplicates are removed.
pub fn find_unit_quad_roots(a: Scalar, b: Scalar, c: Scalar, roots: &mut [Scalar; 2]) -> usize {
    if scalar::nearly_zero(a, None) {
        return valid_unit_divide(-c, b, roots) as usize;
    }

    // Use doubles for better precision
    let dr = b as f64 * b as f64 - 4.0 * a as f64 * c as f64;
    if dr < 0.0 {
        return 0;
    }
    let r = scalar::sqrt(dr as Scalar);
    if !scalar::is_finite(r) {
        return 0;
    }

    let q = if b < 0.0 {
        -(b - r) / 2.0
    } else {
        -(b + r) / 2.0
    };

    let mut count = 0;
    count += valid_unit_divide(q, a, &mut roots[count..]) as usize;
    count += valid_unit_divide(c, q, &mut roots[count..]) as usize;

    if count == 2 {
        if roots[0] > roots[1] {
            roots.swap(0, 1);
        } else if scalar::nearly_equal(roots[0], roots[1], None) {
            count = 1;
        }
    }

    count
}

/// Helper: check if division produces a valid unit interval result.
fn valid_unit_divide(mut numer: Scalar, mut denom: Scalar, ratio: &mut [Scalar]) -> usize {
    // Negating both keeps the ratio unchanged while letting the comparisons
    // below assume a non-negative numerator.
    if numer < 0.0 {
        numer = -numer;
        denom = -denom;
    }

    if denom == 0.0 || numer == 0.0 || numer >= denom {
        return 0;
    }

    let r = numer / denom;
    if r.is_nan() {
        return 0;
    }
    // Catch underflow when numer is vastly smaller than denom.
    if r == 0.0 {
        return 0;
    }

    if !ratio.is_empty() {
        ratio[0] = r;
    }
    1
}

/// Evaluate a quadratic Bezier curve at parameter t.
pub fn eval_quad_at(src: &[Point; 3], t: Scalar) -> Point {
    let p0 = src[0];
    let p1 = src[1];
    let p2 = src[2];

    // Quadratic: (1-t)²P0 + 2t(1-t)P1 + t²P2
    let one_minus_t = 1.0 - t;
    let t2 = t * t;
    let one_minus_t2 = one_minus_t * one_minus_t;
    let two_t_one_minus_t = 2.0 * t * one_minus_t;

    Point::new(
        one_minus_t2 * p0.x + two_t_one_minus_t * p1.x + t2 * p2.x,
        one_minus_t2 * p0.y + two_t_one_minus_t * p1.y + t2 * p2.y,
    )
}

/// Get the tangent vector of a quadratic Bezier at parameter t.
pub fn eval_quad_tangent_at(src: &[Point; 3], t: Scalar) -> Vector {
    let p0 = src[0];
    let p1 = src[1];
    let p2 = src[2];

    // Handle degenerate cases where control point equals endpoints
    if (scalar::nearly_zero(t, None)
        && scalar::nearly_equal(p0.x, p1.x, None)
        && scalar::nearly_equal(p0.y, p1.y, None))
        || (scalar::nearly_zero(t - 1.0, None)
            && scalar::nearly_equal(p1.x, p2.x, None)
            && scalar::nearly_equal(p1.y, p2.y, None))
    {
        return p2 - p0;
    }

    // Derivative: 2 * (B + A*t) where A = P2 - 2*P1 + P0, B = P1 - P0
    let a = p2 - Vector::new(2.0 * p1.x, 2.0 * p1.y) + p0;
    let b = p1 - p0;

    Vector::new(2.0 * (b.x + a.x * t), 2.0 * (b.y + a.y * t))
}

/// Chop a quadratic Bezier at parameter t into two quadratics.
/// dst receives 5 points (3 for first quad, 3 for second, with shared middle point).
pub fn chop_quad_at(src: &[Point; 3], dst: &mut [Point; 5], t: Scalar) {
    let one_minus_t = 1.0 - t;

    dst[0] = src[0];
    dst[4] = src[2];

    // Linear interpolations
    let ab_x = src[0].x * one_minus_t + src[1].x * t;
    let ab_y = src[0].y * one_minus_t + src[1].y * t;
    dst[1] = Point::new(ab_x, ab_y);

    let bc_x = src[1].x * one_minus_t + src[2].x * t;
    let bc_y = src[1].y * one_minus_t + src[2].y * t;
    dst[3] = Point::new(bc_x, bc_y);

    // Middle point
    dst[2] = Point::new(ab_x * one_minus_t + bc_x * t, ab_y * one_minus_t + bc_y * t);
}

/// Chop a quadratic Bezier at t=0.5.
pub fn chop_quad_at_half(src: &[Point; 3], dst: &mut [Point; 5]) {
    chop_quad_at(src, dst, 0.5);
}

/// Find parameter for quadratic extremum (max/min) in Y direction.
/// Returns true if an extremum exists in (0, 1).
pub fn find_quad_extrema(a: Scalar, b: Scalar, c: Scalar, t_value: &mut Scalar) -> bool {
    // Solve: At + B = 0, where A = a - 2b + c, B = b - a
    // t = -B / A = (b - a) / (a - 2b + c)
    let mut root = [0.0; 1];
    if valid_unit_divide(a - b, a - b - b + c, &mut root) == 0 {
        return false;
    }
    *t_value = root[0];
    true
}

/// Chop quadratic at Y extremum. Returns 1 if chopped, 0 if already monotonic.
pub fn chop_quad_at_y_extrema(src: &[Point; 3], dst: &mut [Point; 5]) -> usize {
    let a = src[0].y;
    let mut b = src[1].y;
    let c = src[2].y;

    if is_not_monotonic(a, b, c) {
        let mut t_value = 0.0;
        if find_quad_extrema(a, b, c, &mut t_value) {
            chop_quad_at(src, dst, t_value);
            // Snap both inner control points to the extremum's y so each
            // half is exactly monotonic (SkGeometry's
            // flatten_double_quad_extrema).
            dst[1].y = dst[2].y;
            dst[3].y = dst[2].y;
            return 1;
        }
        // Force monotonic if we couldn't compute t
        b = if (a - b).abs() < (b - c).abs() { a } else { c };
    }

    dst[0] = src[0];
    dst[1] = Point::new(src[1].x, b);
    dst[2] = src[2];
    0
}

/// Check if three values are not monotonic.
///
/// The sequence turns around when the two successive differences have
/// opposing signs, so `bc` is negated when `ab` is negative to reduce the
/// comparison to a single sign test.
fn is_not_monotonic(a: Scalar, b: Scalar, c: Scalar) -> bool {
    let ab = a - b;
    let mut bc = b - c;
    if ab < 0.0 {
        bc = -bc;
    }
    ab == 0.0 || bc < 0.0
}

/// Find parameter of maximum curvature for quadratic.
pub fn find_quad_max_curvature(src: &[Point; 3]) -> Scalar {
    // Ax = P1.x - P0.x, Ay = P1.y - P0.y
    let ax = src[1].x - src[0].x;
    let ay = src[1].y - src[0].y;
    // Bx = P0.x - 2*P1.x + P2.x, By = P0.y - 2*P1.y + P2.y
    let bx = src[0].x - 2.0 * src[1].x + src[2].x;
    let by = src[0].y - 2.0 * src[1].y + src[2].y;

    let numer = -(ax * bx + ay * by);
    let denom = bx * bx + by * by;

    if denom <= 0.0 || numer <= 0.0 {
        return 0.0;
    }
    if numer >= denom {
        return 1.0;
    }
    numer / denom
}

/// Chop quadratic at max curvature. Returns number of quads (1 or 2).
pub fn chop_quad_at_max_curvature(src: &[Point; 3], dst: &mut [Point; 5]) -> usize {
    let t = find_quad_max_curvature(src);
    if t > 0.0 && t < 1.0 {
        chop_quad_at(src, dst, t);
        2
    } else {
        dst[0..3].copy_from_slice(src);
        dst[3] = dst[2];
        dst[4] = dst[2];
        1
    }
}

/// Evaluate a cubic Bezier at parameter t.
pub fn eval_cubic_at(src: &[Point; 4], t: Scalar) -> Point {
    let p0 = src[0];
    let p1 = src[1];
    let p2 = src[2];
    let p3 = src[3];

    // Cubic: (1-t)³P0 + 3t(1-t)²P1 + 3t²(1-t)P2 + t³P3
    let one_minus_t = 1.0 - t;
    let t2 = t * t;
    let t3 = t2 * t;
    let one_minus_t2 = one_minus_t * one_minus_t;
    let one_minus_t3 = one_minus_t2 * one_minus_t;
    let three_t_one_minus_t2 = 3.0 * t * one_minus_t2;
    let three_t2_one_minus_t = 3.0 * t2 * one_minus_t;

    Point::new(
        one_minus_t3 * p0.x + three_t_one_minus_t2 * p1.x + three_t2_one_minus_t * p2.x + t3 * p3.x,
        one_minus_t3 * p0.y + three_t_one_minus_t2 * p1.y + three_t2_one_minus_t * p2.y + t3 * p3.y,
    )
}

/// Get tangent vector of cubic Bezier at parameter t.
pub fn eval_cubic_tangent_at(src: &[Point; 4], t: Scalar) -> Vector {
    let p0 = src[0];
    let p1 = src[1];
    let p2 = src[2];
    let p3 = src[3];

    // Handle degenerate cases
    if scalar::nearly_zero(t, None)
        && scalar::nearly_equal(p0.x, p1.x, None)
        && scalar::nearly_equal(p0.y, p1.y, None)
    {
        return p2 - p0;
    }
    if scalar::nearly_zero(t - 1.0, None)
        && scalar::nearly_equal(p2.x, p3.x, None)
        && scalar::nearly_equal(p2.y, p3.y, None)
    {
        return p3 - p1;
    }

    // Derivative: 3 * (C + B*t + A*t²) where
    // C = P1 - P0
    // B = P2 - 2*P1 + P0
    // A = P3 - 3*P2 + 3*P1 - P0
    let c = p1 - p0;
    let b = p2 - Vector::new(2.0 * p1.x, 2.0 * p1.y) + p0;
    let a = p3 - Vector::new(3.0 * p2.x, 3.0 * p2.y) + Vector::new(3.0 * p1.x, 3.0 * p1.y) - p0;

    let t2 = t * t;
    Vector::new(
        3.0 * (c.x + b.x * t + a.x * t2),
        3.0 * (c.y + b.y * t + a.y * t2),
    )
}

/// Get second derivative of cubic Bezier at parameter t.
pub fn eval_cubic_2nd_derivative(src: &[Point; 4], t: Scalar) -> Vector {
    let p0 = src[0];
    let p1 = src[1];
    let p2 = src[2];
    let p3 = src[3];

    // Second derivative: 6 * (B + A*t) where
    // A = P3 - 3*P2 + 3*P1 - P0
    // B = P2 - 2*P1 + P0
    let a = p3 - Vector::new(3.0 * p2.x, 3.0 * p2.y) + Vector::new(3.0 * p1.x, 3.0 * p1.y) - p0;
    let b = p2 - Vector::new(2.0 * p1.x, 2.0 * p1.y) + p0;

    Vector::new(6.0 * (b.x + a.x * t), 6.0 * (b.y + a.y * t))
}

/// Find extrema parameters for a cubic curve in one dimension.
pub fn find_cubic_extrema(
    a: Scalar,
    b: Scalar,
    c: Scalar,
    d: Scalar,
    t_values: &mut [Scalar; 2],
) -> usize {
    // Coefficients of derivative (divided by 3):
    // A = d - a + 3*(b - c)
    // B = 2*(a - 2b + c)
    // C = b - a
    let a_coef = d - a + 3.0 * (b - c);
    let b_coef = 2.0 * (a - b - b + c);
    let c_coef = b - a;

    find_unit_quad_roots(a_coef, b_coef, c_coef, t_values)
}

/// Chop cubic at parameter t.
pub fn chop_cubic_at(src: &[Point], dst: &mut [Point], t: Scalar) {
    if src.len() < 4 || dst.len() < 7 {
        return;
    }

    if scalar::nearly_zero(t - 1.0, None) {
        dst[0..4].copy_from_slice(&src[0..4]);
        dst[4] = src[3];
        dst[5] = src[3];
        dst[6] = src[3];
        return;
    }

    let one_minus_t = 1.0 - t;

    // First level of de Casteljau
    let ab = Point::new(
        src[0].x * one_minus_t + src[1].x * t,
        src[0].y * one_minus_t + src[1].y * t,
    );
    let bc = Point::new(
        src[1].x * one_minus_t + src[2].x * t,
        src[1].y * one_minus_t + src[2].y * t,
    );
    let cd = Point::new(
        src[2].x * one_minus_t + src[3].x * t,
        src[2].y * one_minus_t + src[3].y * t,
    );

    // Second level
    let abc = Point::new(ab.x * one_minus_t + bc.x * t, ab.y * one_minus_t + bc.y * t);
    let bcd = Point::new(bc.x * one_minus_t + cd.x * t, bc.y * one_minus_t + cd.y * t);

    // Third level (the split point)
    let abcd = Point::new(
        abc.x * one_minus_t + bcd.x * t,
        abc.y * one_minus_t + bcd.y * t,
    );

    dst[0] = src[0];
    dst[1] = ab;
    dst[2] = abc;
    dst[3] = abcd;
    dst[4] = bcd;
    dst[5] = cd;
    dst[6] = src[3];
}

/// Chop cubic at t=0.5.
pub fn chop_cubic_at_half(src: &[Point], dst: &mut [Point]) {
    chop_cubic_at(src, dst, 0.5);
}

/// Find inflection points of a cubic.
pub fn find_cubic_inflections(src: &[Point; 4], t_values: &mut [Scalar; 2]) -> usize {
    let ax = src[1].x - src[0].x;
    let ay = src[1].y - src[0].y;
    let bx = src[2].x - 2.0 * src[1].x + src[0].x;
    let by = src[2].y - 2.0 * src[1].y + src[0].y;
    let cx = src[3].x + 3.0 * (src[1].x - src[2].x) - src[0].x;
    let cy = src[3].y + 3.0 * (src[1].y - src[2].y) - src[0].y;

    // Solve (Bx*Cy - By*Cx)t² + (Ax*Cy - Ay*Cx)t + (Ax*By - Ay*Bx) = 0
    let a_coef = bx * cy - by * cx;
    let b_coef = ax * cy - ay * cx;
    let c_coef = ax * by - ay * bx;

    find_unit_quad_roots(a_coef, b_coef, c_coef, t_values)
}

/// Chop cubic at Y extrema. Returns number of cubics (1, 2, or 3).
pub fn chop_cubic_at_y_extrema(src: &[Point; 4], dst: &mut [Point; 10]) -> usize {
    let mut t_values = [0.0; 2];
    let roots = find_cubic_extrema(src[0].y, src[1].y, src[2].y, src[3].y, &mut t_values);

    if roots == 0 {
        dst[0..4].copy_from_slice(src);
        return 0;
    }

    chop_cubic_at(src, &mut dst[0..7], t_values[0]);
    if roots == 2 {
        // Use split_at_mut to get two mutable non-overlapping sub-slices
        let (first, second) = dst.split_at_mut(7);
        // src is first[3..] (4 points)
        // dst for second chop is second[0..] (3 points needed, but second has 3 elements from index 7-9)
        chop_cubic_at(&first[3..7], second, t_values[1]);
        // Ensure flat extrema
        dst[3].y = (dst[2].y + dst[4].y) / 2.0;
        dst[6].y = (dst[5].y + dst[7].y) / 2.0;
    }
    // Ensure flat first extremum
    dst[3].y = (dst[2].y + dst[4].y) / 2.0;

    roots
}

/// Compute maximum curvature parameters for a cubic.
pub fn find_cubic_max_curvature(src: &[Point; 4], t_values: &mut [Scalar; 4]) -> usize {
    // Formulate F' · F'' = 0 for both X and Y, then combine
    let mut coeff = [0.0; 4];
    formulate_f_dot_f2(&src[0].x, &src[1].x, &src[2].x, &src[3].x, &mut coeff);
    let mut coeff_y = [0.0; 4];
    formulate_f_dot_f2(&src[0].y, &src[1].y, &src[2].y, &src[3].y, &mut coeff_y);

    for i in 0..4 {
        t_values[i] += coeff_y[i];
    }

    solve_cubic_poly(t_values)
}

/// Helper for max curvature calculation.
fn formulate_f_dot_f2(a: &Scalar, b: &Scalar, c: &Scalar, d: &Scalar, coeff: &mut [Scalar; 4]) {
    let a_val = *c - *a;
    let b_val = *d - 2.0 * *c + *a;
    let c_val = *d - 3.0 * *c + 3.0 * *b - *a;

    coeff[0] = c_val * c_val;
    coeff[1] = 3.0 * b_val * c_val;
    coeff[2] = 2.0 * b_val * b_val + c_val * a_val;
    coeff[3] = a_val * b_val;
}

/// Solve cubic polynomial for roots in [0, 1).
fn solve_cubic_poly(coeff: &mut [Scalar; 4]) -> usize {
    if scalar::nearly_zero(coeff[0], None) {
        let mut roots = [0.0; 2];
        let count = find_unit_quad_roots(coeff[1], coeff[2], coeff[3], &mut roots);
        if count > 0 {
            coeff[0] = roots[0];
        }
        if count > 1 {
            coeff[1] = roots[1];
        }
        return count;
    }

    // Normalize to monic form
    let inv_a = 1.0 / coeff[0];
    let a = coeff[1] * inv_a;
    let b = coeff[2] * inv_a;
    let c = coeff[3] * inv_a;

    let q = (a * a - 3.0 * b) / 9.0;
    let r = (2.0 * a * a * a - 9.0 * a * b + 27.0 * c) / 54.0;
    let q3 = q * q * q;
    let r2_minus_q3 = r * r - q3;
    let a_div3 = a / 3.0;

    let mut roots = [0.0; 3];
    let mut count;

    if r2_minus_q3 < 0.0 {
        // Three real roots
        let theta = (r / scalar::sqrt(q3)).clamp(-1.0, 1.0).acos();
        let neg2_root_q = -2.0 * scalar::sqrt(q);

        roots[0] = (neg2_root_q * (theta / 3.0).cos() - a_div3).clamp(0.0, 1.0);
        roots[1] = (neg2_root_q * ((theta + 2.0 * scalar::SCALAR_PI) / 3.0).cos() - a_div3)
            .clamp(0.0, 1.0);
        roots[2] = (neg2_root_q * ((theta - 2.0 * scalar::SCALAR_PI) / 3.0).cos() - a_div3)
            .clamp(0.0, 1.0);

        // Sort roots
        roots.sort_by(|a, b| a.partial_cmp(b).unwrap());

        // Remove duplicates
        count = 1;
        for i in 1..3 {
            if !scalar::nearly_equal(roots[i], roots[count - 1], None) {
                roots[count] = roots[i];
                count += 1;
            }
        }
    } else {
        // One real root
        let mut a_val = r.abs() + scalar::sqrt(r2_minus_q3);
        a_val = scalar::cbrt(a_val);
        if r > 0.0 {
            a_val = -a_val;
        }
        if !scalar::nearly_zero(a_val, None) {
            a_val += q / a_val;
        }
        roots[0] = (a_val - a_div3).clamp(0.0, 1.0);
        count = 1;
    }

    // Copy back
    for i in 0..count {
        coeff[i] = roots[i];
    }
    count
}

/// Find cusp location for a cubic. Returns parameter t or -1 if no cusp.
pub fn find_cubic_cusp(src: &[Point; 4]) -> Scalar {
    // Skip if endpoint equals adjacent control point
    if (scalar::nearly_equal(src[0].x, src[1].x, None)
        && scalar::nearly_equal(src[0].y, src[1].y, None))
        || (scalar::nearly_equal(src[2].x, src[3].x, None)
            && scalar::nearly_equal(src[2].y, src[3].y, None))
    {
        return -1.0;
    }

    // Check if line segments cross (necessary for cusp)
    if on_same_side(src, 0, 2) || on_same_side(src, 2, 0) {
        return -1.0;
    }

    // Find max curvature points
    let mut max_curvature = [0.0; 4];
    let roots = find_cubic_max_curvature(src, &mut max_curvature);

    for i in 0..roots {
        let test_t = max_curvature[i];
        if test_t <= 0.0 || test_t >= 1.0 {
            continue;
        }

        // Check if derivative magnitude is near zero at this point
        let derivative = eval_cubic_tangent_at(src, test_t);
        let precision = calc_cubic_precision(src);
        if derivative.x * derivative.x + derivative.y * derivative.y < precision {
            return test_t;
        }
    }

    -1.0
}

/// Check if two points are on the same side of a line segment.
fn on_same_side(src: &[Point; 4], test_index: usize, line_index: usize) -> bool {
    let origin = src[line_index];
    let line = src[line_index + 1] - origin;

    let crosses: Vec<Scalar> = (0..2)
        .map(|index| {
            let test_line = src[test_index + index] - origin;
            line.cross(test_line)
        })
        .collect();

    crosses[0] * crosses[1] >= 0.0
}

/// Calculate precision threshold for cubic.
fn calc_cubic_precision(src: &[Point; 4]) -> Scalar {
    let d0 = (src[1].x - src[0].x).powi(2) + (src[1].y - src[0].y).powi(2);
    let d1 = (src[2].x - src[1].x).powi(2) + (src[2].y - src[1].y).powi(2);
    let d2 = (src[3].x - src[2].x).powi(2) + (src[3].y - src[2].y).powi(2);
    (d0 + d1 + d2) * 1e-8
}

/// Classify a cubic curve.
pub fn classify_cubic(src: &[Point; 4]) -> CubicType {
    // Calculate inflection function coefficients
    let a1 = calc_dot_cross(src[0], src[3], src[2]);
    let a2 = calc_dot_cross(src[1], src[0], src[3]);
    let a3 = calc_dot_cross(src[2], src[1], src[0]);

    let d3 = 3.0 * a3;
    let d2 = d3 - a2;
    let d1 = d2 - a2 + a1;

    if !scalar::nearly_zero(d1, None) {
        let discr = 3.0 * d2 * d2 - 4.0 * d1 * d3;
        if discr > 0.0 {
            CubicType::Serpentine
        } else if discr < 0.0 {
            CubicType::Loop
        } else {
            CubicType::LocalCusp
        }
    } else if !scalar::nearly_zero(d2, None) {
        CubicType::CuspAtInfinity
    } else if !scalar::nearly_zero(d3, None) {
        CubicType::Quadratic
    } else {
        CubicType::LineOrPoint
    }
}

/// Calculate triple product p0 · (p1 × p2).
fn calc_dot_cross(p0: Point, p1: Point, p2: Point) -> Scalar {
    let x_comp = p0.x * (p1.y - p2.y);
    let y_comp = p0.y * (p2.x - p1.x);
    let w_comp = p1.x * p2.y - p1.y * p2.x;
    x_comp + y_comp + w_comp
}

/// Conic (rational quadratic) representation.
#[derive(Debug, Clone, Copy, Default)]
pub struct Conic {
    /// Start point, control point, and end point, in that order.
    pub pts: [Point; 3],
    /// Weight of the control point. 1.0 is an ordinary quadratic; below 1
    /// the curve flattens toward the chord, above 1 it bows toward the
    /// control point.
    pub w: Scalar,
}

impl Conic {
    /// Create a new conic.
    pub fn new(pts: [Point; 3], w: Scalar) -> Self {
        Conic { pts, w }
    }

    /// Evaluate conic at parameter t.
    pub fn eval_at(&self, t: Scalar) -> Point {
        let one_minus_t = 1.0 - t;
        let t2 = t * t;
        let one_minus_t2 = one_minus_t * one_minus_t;
        let two_t_one_minus_t = 2.0 * t * one_minus_t;

        // Evaluate numerator and denominator separately
        let denom = one_minus_t2 + t2 + two_t_one_minus_t * self.w;
        let x = (one_minus_t2 * self.pts[0].x
            + two_t_one_minus_t * self.w * self.pts[1].x
            + t2 * self.pts[2].x)
            / denom;
        let y = (one_minus_t2 * self.pts[0].y
            + two_t_one_minus_t * self.w * self.pts[1].y
            + t2 * self.pts[2].y)
            / denom;
        Point::new(x, y)
    }

    /// Get tangent at parameter t.
    pub fn eval_tangent_at(&self, t: Scalar) -> Vector {
        let one_minus_t = 1.0 - t;
        let w = self.w;

        // Derivative of rational quadratic
        let p0 = self.pts[0];
        let p1 = self.pts[1];
        let p2 = self.pts[2];

        // Handle degenerate cases
        if (scalar::nearly_zero(t, None)
            && scalar::nearly_equal(p0.x, p1.x, None)
            && scalar::nearly_equal(p0.y, p1.y, None))
            || (scalar::nearly_zero(t - 1.0, None)
                && scalar::nearly_equal(p1.x, p2.x, None)
                && scalar::nearly_equal(p1.y, p2.y, None))
        {
            return p2 - p0;
        }

        // Calculate derivative using quotient rule on rational form
        let num_x =
            2.0 * ((p2.x - p0.x) * w - (p1.x - p0.x) * w * 2.0 + (p1.x - p0.x)) * t * one_minus_t
                + (p2.x - p0.x) * (1.0 - 2.0 * t) * (w - 1.0);
        let num_y =
            2.0 * ((p2.y - p0.y) * w - (p1.y - p0.y) * w * 2.0 + (p1.y - p0.y)) * t * one_minus_t
                + (p2.y - p0.y) * (1.0 - 2.0 * t) * (w - 1.0);

        Vector::new(num_x, num_y)
    }

    /// Chop conic at parameter t. Returns true if successful.
    pub fn chop_at(&self, t: Scalar, dst: &mut [Conic; 2]) -> bool {
        // Map to 3D, interpolate, then project back
        let tmp = ratquad_map_to_3d(&self.pts, self.w);

        let mut tmp2 = [Point3::default(); 3];
        let xs = [tmp[0].x, tmp[1].x, tmp[2].x];
        let ys = [tmp[0].y, tmp[1].y, tmp[2].y];
        let zs = [tmp[0].z, tmp[1].z, tmp[2].z];
        let mut xs_out = [0.0; 3];
        let mut ys_out = [0.0; 3];
        let mut zs_out = [0.0; 3];
        p3d_interp(&xs, &mut xs_out, t);
        p3d_interp(&ys, &mut ys_out, t);
        p3d_interp(&zs, &mut zs_out, t);
        for i in 0..3 {
            tmp2[i] = Point3::new(xs_out[i], ys_out[i], zs_out[i]);
        }

        let root = scalar::sqrt(tmp2[1].z);
        dst[0] = Conic::new(
            [self.pts[0], project_down(tmp2[0]), project_down(tmp2[1])],
            tmp2[0].z / root,
        );
        dst[1] = Conic::new(
            [dst[0].pts[2], project_down(tmp2[2]), self.pts[2]],
            tmp2[2].z / root,
        );

        dst[0].is_finite() && dst[1].is_finite()
    }

    /// Check if conic is finite.
    pub fn is_finite(&self) -> bool {
        scalar::are_finite(self.pts[0].x, self.pts[0].y)
            && scalar::are_finite(self.pts[1].x, self.pts[1].y)
            && scalar::are_finite(self.pts[2].x, self.pts[2].y)
    }

    /// Chop conic in half.
    pub fn chop(&self, dst: &mut [Conic; 2]) {
        self.chop_at(0.5, dst);
    }

    /// Find Y extremum parameter.
    pub fn find_y_extrema(&self, t: &mut Scalar) -> bool {
        conic_find_extrema(&[self.pts[0].y, self.pts[1].y, self.pts[2].y], self.w, t)
    }

    /// Find X extremum parameter.
    pub fn find_x_extrema(&self, t: &mut Scalar) -> bool {
        conic_find_extrema(&[self.pts[0].x, self.pts[1].x, self.pts[2].x], self.w, t)
    }

    /// Chop at Y extremum.
    pub fn chop_at_y_extrema(&self, dst: &mut [Conic; 2]) -> bool {
        let mut t = 0.0;
        if self.find_y_extrema(&mut t) {
            if self.chop_at(t, dst) {
                // t sits exactly at a y-extremum, so snap the control points
                // around the split to the shared endpoint's y. Each half is
                // then exactly monotonic in y.
                let value = dst[0].pts[2].y;
                dst[0].pts[1].y = value;
                dst[1].pts[0].y = value;
                dst[1].pts[1].y = value;
                return true;
            }
        }
        false
    }

    /// Chop at X extremum.
    pub fn chop_at_x_extrema(&self, dst: &mut [Conic; 2]) -> bool {
        let mut t = 0.0;
        if self.find_x_extrema(&mut t) {
            if self.chop_at(t, dst) {
                // Mirror of chop_at_y_extrema for the x axis.
                let value = dst[0].pts[2].x;
                dst[0].pts[1].x = value;
                dst[1].pts[0].x = value;
                dst[1].pts[1].x = value;
                return true;
            }
        }
        false
    }

    /// Compute tight bounds.
    pub fn compute_tight_bounds(&self) -> (Point, Point) {
        let mut min_pt = self.pts[0];
        let mut max_pt = self.pts[2];

        let mut t = 0.0;
        if self.find_x_extrema(&mut t) {
            let pt = self.eval_at(t);
            min_pt.x = min_pt.x.min(pt.x);
            max_pt.x = max_pt.x.max(pt.x);
        }
        if self.find_y_extrema(&mut t) {
            let pt = self.eval_at(t);
            min_pt.y = min_pt.y.min(pt.y);
            max_pt.y = max_pt.y.max(pt.y);
        }
        (min_pt, max_pt)
    }

    /// Compute fast bounds (just hull).
    pub fn compute_fast_bounds(&self) -> (Point, Point) {
        let mut min_pt = self.pts[0];
        let mut max_pt = self.pts[0];

        for pt in &self.pts {
            min_pt.x = min_pt.x.min(pt.x);
            min_pt.y = min_pt.y.min(pt.y);
            max_pt.x = max_pt.x.max(pt.x);
            max_pt.y = max_pt.y.max(pt.y);
        }
        (min_pt, max_pt)
    }

    /// Mid-tangent parameter (for splitting at curvature).
    pub fn find_mid_tangent(&self) -> Scalar {
        let tan0 = self.pts[1] - self.pts[0];
        let tan1 = self.pts[2] - self.pts[1];
        let bisector = find_bisector(tan0, -tan1);

        // Solve quadratic: bisector · (A + B*t + C*t²) = 0
        let a = (self.pts[2] - self.pts[0]) * (self.w - 1.0);
        let b = (self.pts[2] - self.pts[0]) - (self.pts[1] - self.pts[0]) * (self.w * 2.0);
        let c = (self.pts[1] - self.pts[0]) * self.w;

        let a_coef = bisector.dot(a);
        let b_coef = bisector.dot(b);
        let c_coef = bisector.dot(c);

        solve_quadratic_equation_for_midtangent(a_coef, b_coef, c_coef)
    }
}

/// Helper for conic derivative extrema.
pub fn conic_find_extrema(src: &[Scalar; 3], w: Scalar, t: &mut Scalar) -> bool {
    let mut coeff = [0.0; 3];
    conic_deriv_coeff(src, w, &mut coeff);

    let mut t_values = [0.0; 2];
    let roots = find_unit_quad_roots(coeff[0], coeff[1], coeff[2], &mut t_values);

    if roots == 1 {
        *t = t_values[0];
        true
    } else {
        false
    }
}

/// Compute conic derivative coefficients for one axis.
///
/// `src` holds that axis's coordinate for the three control points. The C++
/// takes a strided pointer into the point array and reads indices 0, 2 and 4;
/// this takes the three values directly.
fn conic_deriv_coeff(src: &[Scalar; 3], w: Scalar, coeff: &mut [Scalar; 3]) {
    let p20 = src[2] - src[0];
    let p10 = src[1] - src[0];
    let w_p10 = w * p10;
    coeff[0] = w * p20 - p20;
    coeff[1] = p20 - 2.0 * w_p10;
    coeff[2] = w_p10;
}

/// Helper: find bisector of two vectors.
pub fn find_bisector(a: Vector, b: Vector) -> Vector {
    if a.dot(b) >= 0.0 {
        // Within 90 degrees: normalize(a) + normalize(b), not
        // normalize(a + b) — these differ whenever a and b have
        // different lengths.
        if let (Some(n0), Some(n1)) = (a.normalized(), b.normalized()) {
            n0 + n1
        } else {
            a
        }
    } else if a.cross(b) >= 0.0 {
        // > 90 degrees - use interior normals
        let v0 = Vector::new(-a.y, a.x);
        let v1 = Vector::new(b.y, -b.x);
        if let (Some(n0), Some(n1)) = (v0.normalized(), v1.normalized()) {
            n0 + n1
        } else {
            a
        }
    } else {
        // < -90 degrees - use interior normals
        let v0 = Vector::new(a.y, -a.x);
        let v1 = Vector::new(-b.y, b.x);
        if let (Some(n0), Some(n1)) = (v0.normalized(), v1.normalized()) {
            n0 + n1
        } else {
            a
        }
    }
}

/// Helper: solve quadratic for mid-tangent.
fn solve_quadratic_equation_for_midtangent(a: Scalar, b: Scalar, c: Scalar) -> Scalar {
    let discr = b * b - 4.0 * a * c;
    if discr < 0.0 {
        return 0.5;
    }

    let q = -0.5 * (b + b.signum() * discr.sqrt());
    let _5qa = -0.5 * q / a;
    let t = if (q * q + _5qa).abs() < (a * c + _5qa).abs() {
        q / a
    } else {
        c / q
    };

    if t > 0.0 && t < 1.0 {
        t
    } else {
        0.5
    }
}

/// Map conic to 3D for rational evaluation.
fn ratquad_map_to_3d(pts: &[Point; 3], w: Scalar) -> [Point3; 3] {
    [
        Point3::new(pts[0].x, pts[0].y, 1.0),
        Point3::new(pts[1].x * w, pts[1].y * w, w),
        Point3::new(pts[2].x, pts[2].y, 1.0),
    ]
}

/// De Casteljau step for one coordinate across 3 collinear-in-parameter
/// points: `dst[0]` is the first half's control point, `dst[2]` the
/// second half's, and `dst[1]` their shared midpoint at `t`.
fn p3d_interp(src: &[Scalar; 3], dst: &mut [Scalar; 3], t: Scalar) {
    let ab = scalar::interp(src[0], src[1], t);
    let bc = scalar::interp(src[1], src[2], t);
    dst[0] = ab;
    dst[1] = scalar::interp(ab, bc, t);
    dst[2] = bc;
}

/// Project 3D point back to 2D.
fn project_down(p: Point3) -> Point {
    Point::new(p.x / p.z, p.y / p.z)
}

/// Build unit arc conics.
pub fn build_unit_arc(
    start: Vector,
    stop: Vector,
    direction: RotationDirection,
    dst: &mut [Conic; 4],
) -> usize {
    let x = Vector::dot_product(start, stop);
    let y = Vector::cross_product(start, stop);

    let abs_y = y.abs();

    // Check for coincident vectors
    if abs_y <= scalar::NEARLY_ZERO && x > 0.0 {
        if (y >= 0.0 && direction == RotationDirection::Cw)
            || (y <= 0.0 && direction == RotationDirection::Ccw)
        {
            return 0;
        }
    }

    let mut y = y;
    if direction == RotationDirection::Ccw {
        y = -y;
    }

    // Determine quadrant
    let quadrant = if y == 0.0 {
        2
    } else if x == 0.0 {
        if y > 0.0 {
            1
        } else {
            3
        }
    } else {
        let mut q = 0;
        if y < 0.0 {
            q += 2;
        }
        if (x < 0.0) != (y < 0.0) {
            q += 1;
        }
        q
    };

    let quad_pts = [
        Point::new(1.0, 0.0),
        Point::new(1.0, 1.0),
        Point::new(0.0, 1.0),
        Point::new(-1.0, 1.0),
        Point::new(-1.0, 0.0),
        Point::new(-1.0, -1.0),
        Point::new(0.0, -1.0),
        Point::new(1.0, -1.0),
    ];

    let quad_weight = scalar::SCALAR_ROOT_2_OVER_2;

    let mut conic_count = quadrant;
    for i in 0..quadrant {
        dst[i] = Conic::new(
            [quad_pts[i * 2], quad_pts[i * 2 + 1], quad_pts[(i + 1) * 2]],
            quad_weight,
        );
    }

    // Final partial arc if needed
    let final_p = Point::new(x, y);
    let last_q = quad_pts[quadrant * 2];
    let dot = Vector::dot_product(
        last_q - Vector::new(0.0, 0.0),
        final_p - Vector::new(0.0, 0.0),
    );

    if dot < 1.0 {
        let off_curve = last_q + final_p;
        let cos_theta_over_2 = scalar::sqrt((1.0 + dot) / 2.0);
        let off_curve = off_curve
            .scaled_to_length(1.0 / cos_theta_over_2)
            .unwrap_or(off_curve);

        if !(scalar::nearly_equal(last_q.x, off_curve.x, None)
            && scalar::nearly_equal(last_q.y, off_curve.y, None))
        {
            dst[conic_count] = Conic::new([last_q, off_curve, final_p], cos_theta_over_2);
            conic_count += 1;
        }
    }

    conic_count
}

/// Rotation direction for arc building.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RotationDirection {
    Cw,
    Ccw,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eval_quad_at() {
        let quad = [
            Point::new(0.0, 0.0),
            Point::new(0.5, 1.0),
            Point::new(1.0, 0.0),
        ];
        let t = 0.5;
        let pt = eval_quad_at(&quad, t);
        // Quadratic Bezier at t=0.5 is (P0 + 2*P1 + P2) / 4, not the
        // control point itself: y = (0 + 2*1.0 + 0) / 4 = 0.5.
        assert!((pt.x - 0.5).abs() < 1e-6);
        assert!((pt.y - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_chop_quad_at() {
        let quad = [
            Point::new(0.0, 0.0),
            Point::new(0.5, 1.0),
            Point::new(1.0, 0.0),
        ];
        let mut dst = [Point::default(); 5];
        chop_quad_at(&quad, &mut dst, 0.5);

        assert_eq!(dst[0], quad[0]);
        assert_eq!(dst[4], quad[2]);
        // All points should be finite
        assert!(dst.iter().all(|p| p.is_finite()));
    }

    #[test]
    fn test_cubic_classification() {
        // Line
        let line = [
            Point::new(0.0, 0.0),
            Point::new(0.5, 0.5),
            Point::new(0.5, 0.5),
            Point::new(1.0, 1.0),
        ];
        assert_eq!(classify_cubic(&line), CubicType::LineOrPoint);

        // This symmetric control polygon has discriminant 3*d2^2-4*d1*d3
        // == 0 exactly (verified independently), which is the boundary
        // case: a local cusp, not a loop or serpentine curve.
        let cubic = [
            Point::new(0.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
            Point::new(1.0, 0.0),
        ];
        let classification = classify_cubic(&cubic);
        assert_eq!(classification, CubicType::LocalCusp);
    }

    #[test]
    fn test_cubic_extrema() {
        let mut t_values = [0.0; 2];
        // Cubic with known extremum
        let count = find_cubic_extrema(0.0, 0.5, 1.0, 1.5, &mut t_values);
        assert!(count <= 2);
        // All roots should be in [0, 1]
        for i in 0..count {
            assert!(t_values[i] >= 0.0 && t_values[i] <= 1.0);
        }
    }

    #[test]
    fn test_conic_basic() {
        let conic = Conic::new(
            [
                Point::new(1.0, 0.0),
                Point::new(1.0, 1.0),
                Point::new(0.0, 1.0),
            ],
            scalar::SCALAR_ROOT_2_OVER_2,
        );

        let pt = conic.eval_at(0.5);
        assert!(pt.is_finite());

        let tangent = conic.eval_tangent_at(0.5);
        assert!(tangent.is_finite());
    }

    #[test]
    fn test_conic_chop() {
        let conic = Conic::new(
            [
                Point::new(1.0, 0.0),
                Point::new(1.0, 1.0),
                Point::new(0.0, 1.0),
            ],
            scalar::SCALAR_ROOT_2_OVER_2,
        );

        let mut dst = [Conic::default(); 2];
        assert!(conic.chop_at(0.5, &mut dst));
        assert!(dst[0].is_finite());
        assert!(dst[1].is_finite());
    }

    #[test]
    fn test_find_bisector() {
        let a = Vector::new(1.0, 0.0);
        let b = Vector::new(0.0, 1.0);
        let bisector = find_bisector(a, b);
        assert!((bisector.x - bisector.y).abs() < 1e-6);
        assert!((bisector.length() - 1.414).abs() < 0.001);
    }

    #[test]
    fn test_build_unit_arc() {
        let start = Vector::new(1.0, 0.0);
        let stop = Vector::new(0.0, 1.0);
        let mut dst = [Conic::default(); 4];

        let count = build_unit_arc(start, stop, RotationDirection::Cw, &mut dst);
        assert!(count > 0 && count <= 4);
        assert!(dst.iter().take(count).all(|c| c.is_finite()));
    }

    #[test]
    fn is_not_monotonic_detects_turnaround() {
        // Rising then falling, and falling then rising, both turn around.
        assert!(is_not_monotonic(0.0, 100.0, 0.0));
        assert!(is_not_monotonic(0.0, -100.0, 0.0));
        // Steadily rising or falling does not.
        assert!(!is_not_monotonic(0.0, 50.0, 100.0));
        assert!(!is_not_monotonic(100.0, 50.0, 0.0));
        // A flat leading difference counts as non-monotonic.
        assert!(is_not_monotonic(5.0, 5.0, 9.0));
    }

    #[test]
    fn valid_unit_divide_accepts_negative_numerator() {
        // Skia negates both terms rather than rejecting a negative numerator.
        let mut r = [0.0; 1];
        assert_eq!(valid_unit_divide(-100.0, -200.0, &mut r), 1);
        assert!((r[0] - 0.5).abs() < 1e-6);

        // Out-of-range and degenerate inputs are still rejected.
        assert_eq!(valid_unit_divide(200.0, 100.0, &mut r), 0);
        assert_eq!(valid_unit_divide(0.0, 100.0, &mut r), 0);
        assert_eq!(valid_unit_divide(100.0, 0.0, &mut r), 0);
    }

    #[test]
    fn find_quad_extrema_returns_t() {
        // A symmetric arch peaks at t = 0.5.
        let mut t = 0.0;
        assert!(find_quad_extrema(0.0, 100.0, 0.0, &mut t));
        assert!((t - 0.5).abs() < 1e-6, "t was {t}");
    }

    #[test]
    fn chop_quad_at_y_extrema_splits_arch() {
        // move(0,0) quad(50,100 -> 100,0) peaks at y = 50, so it must chop.
        let src = [
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
        ];
        let mut dst = [Point::default(); 5];
        assert_eq!(chop_quad_at_y_extrema(&src, &mut dst), 1);

        // The split point sits at the extremum.
        assert!((dst[2].y - 50.0).abs() < 1e-4, "split y {}", dst[2].y);
        // Both control points are snapped to it, making each half monotonic.
        assert!((dst[1].y - dst[2].y).abs() < 1e-6);
        assert!((dst[3].y - dst[2].y).abs() < 1e-6);
        // Endpoints are untouched.
        assert_eq!(dst[0], src[0]);
        assert_eq!(dst[4], src[2]);
    }

    #[test]
    fn conic_find_extrema_uses_all_three_points() {
        // An arch conic has an interior y-extremum; a monotonic one does not.
        let arch = Conic::new(
            [
                Point::new(0.0, 0.0),
                Point::new(50.0, 100.0),
                Point::new(100.0, 0.0),
            ],
            2.0,
        );
        let mut t = 0.0;
        assert!(arch.find_y_extrema(&mut t));
        assert!(t > 0.0 && t < 1.0, "t was {t}");

        let rising = Conic::new(
            [
                Point::new(0.0, 0.0),
                Point::new(50.0, 50.0),
                Point::new(100.0, 100.0),
            ],
            2.0,
        );
        let mut t2 = 0.0;
        assert!(!rising.find_y_extrema(&mut t2));
    }

    #[test]
    fn conic_chop_at_y_extrema_snaps_control_points() {
        let arch = Conic::new(
            [
                Point::new(0.0, 0.0),
                Point::new(50.0, 100.0),
                Point::new(100.0, 0.0),
            ],
            2.0,
        );
        let mut dst = [Conic::default(); 2];
        assert!(arch.chop_at_y_extrema(&mut dst));

        // The shared endpoint keeps the extremum's y, and the three control
        // points around the split are snapped to it.
        let value = dst[0].pts[2].y;
        assert!(value > 0.0, "extremum y {value}");
        assert!((dst[0].pts[1].y - value).abs() < 1e-6);
        assert!((dst[1].pts[0].y - value).abs() < 1e-6);
        assert!((dst[1].pts[1].y - value).abs() < 1e-6);
        // The outer endpoints are unchanged.
        assert!((dst[0].pts[0].y - 0.0).abs() < 1e-6);
        assert!((dst[1].pts[2].y - 0.0).abs() < 1e-6);
    }
}
