//! Utilities for evaluating paths.

/// When tessellating curved paths into linear segments, this defines the maximum distance in screen
/// space which a segment may deviate from the mathematically correct value. Above this value, the
/// segment will be subdivided.
/// This value was chosen to approximate the supersampling accuracy of the raster path (16 samples,
/// or one quarter pixel).
pub const DEFAULT_TOLERANCE: f32 = 0.25;

/// We guarantee that no quad or cubic will ever produce more than this many points
pub const MAX_POINTS_PER_CURVE: u32 = 1 << 10; // 1024

/// Helper function to convert tolerance to Wang's precision
fn tolerance_to_wangs_precision(src_tol: f32) -> f32 {
    // The GrPathUtil API defines tolerance as the max distance the linear segment can be from
    // the real curve. Wang's formula guarantees the linear segments will be within 1/precision
    // of the true curve, so precision = 1/srcTol
    1.0 / src_tol
}

/// Returns the maximum number of vertices required for a bezier curve given chop count.
fn max_bezier_vertices(chop_count: u32) -> u32 {
    const MAX_CHOPS_PER_CURVE: u32 = 10;
    1 << chop_count.min(MAX_CHOPS_PER_CURVE)
}

/// Returns the maximum number of vertices required when using a recursive chopping algorithm to
/// linearize the cubic Bezier to the given error tolerance.
/// This is a power of two and will not exceed MAX_POINTS_PER_CURVE.
pub fn cubic_point_count(points: &[[f32; 2]; 4], tol: f32) -> u32 {
    max_bezier_vertices(cubic_log2(tolerance_to_wangs_precision(tol), points))
}

/// Returns the log2 value of Wang's formula specialized for a cubic curve, rounded up to the next
/// int.
fn cubic_log2(precision: f32, points: &[[f32; 2]; 4]) -> u32 {
    // Wang's formula for cubics: sqrt(sqrt(max_length * precision^2 * 3^4 / 64))
    // We compute this in log2 space to avoid overflow and get the number of chops
    let (p0, p1, p2, p3) = (points[0], points[1], points[2], points[3]);
    
    // Compute the second difference: v = p0 - 2*p1 + p2 for each dimension
    // Then take the max of the squared lengths
    let v0 = p0[0] - 2.0 * p1[0] + p2[0];
    let v1 = p0[1] - 2.0 * p1[1] + p2[1];
    let v2 = p1[0] - 2.0 * p2[0] + p3[0];
    let v3 = p1[1] - 2.0 * p2[1] + p3[1];
    
    // maxLength = max(v0^2 + v1^2, v2^2 + v3^2)
    let length0 = v0 * v0 + v1 * v1;
    let length1 = v2 * v2 + v3 * v3;
    let max_length = length0.max(length1);
    
    // Wang's formula: sqrt(maxLength * precision * n*(n-1)/8) where n=3
    // For cubics: sqrt(maxLength * precision * 3*2/8) = sqrt(maxLength * precision * 0.75)
    // But we need log2 of this, and the C++ uses nextlog16 which is ceil(log2(x^(1/4)))
    
    // Actually, looking at the C++ code more carefully:
    // cubic_pow4 computes: maxLength * length_term_pow2<3>(precision)
    // where length_term_pow2<3>(precision) = (3*3 * 2*2 / 64) * precision^2 = (9*4/64)*precision^2 = 0.5625*precision^2
    // So cubic_pow4 = maxLength * 0.5625 * precision^2
    // Then nextlog16(cubic_pow4) = ceil(log2((cubic_pow4)^(1/4))) = ceil(log2(sqrt(sqrt(cubic_pow4))))
    
    let pow4 = max_length * 0.5625 * precision * precision;
    nextlog16(pow4)
}

/// Returns nextlog2(sqrt(sqrt(x))), which is ceil(log2(x^(1/4)))
fn nextlog16(x: f32) -> u32 {
    // nextlog16(x) == ceil(log2(sqrt(sqrt(x))))
    // This is equivalent to ceil(log16(x))
    if x <= 0.0 {
        return 0;
    }
    
    let log2_x = x.log2();
    // (log2_x + 3) / 4 with ceiling
    ((log2_x + 3.0).ceil() as i32 / 4) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tolerance_to_wangs_precision() {
        assert!((tolerance_to_wangs_precision(0.25) - 4.0).abs() < 1e-6);
        assert!((tolerance_to_wangs_precision(0.5) - 2.0).abs() < 1e-6);
        assert!((tolerance_to_wangs_precision(1.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_max_bezier_vertices() {
        assert_eq!(max_bezier_vertices(0), 1);
        assert_eq!(max_bezier_vertices(5), 32);
        assert_eq!(max_bezier_vertices(10), 1024);
        assert_eq!(max_bezier_vertices(15), 1024); // clamped to MAX_CHOPS_PER_CURVE
        assert_eq!(max_bezier_vertices(20), 1024);
    }

    #[test]
    fn test_cubic_point_count_straight_line() {
        // For a straight line (all control points collinear), the max length should be 0
        // which means we only need 1 segment (2 points)
        let points = [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0]];
        let count = cubic_point_count(&points, DEFAULT_TOLERANCE);
        assert_eq!(count, 1); // Should return 1 segment, so 1 vertex
    }

    #[test]
    fn test_cubic_point_count_curved() {
        // For a curved path, we should get more points
        let points = [[0.0, 0.0], [1.0, 2.0], [2.0, 2.0], [3.0, 0.0]];
        let count = cubic_point_count(&points, DEFAULT_TOLERANCE);
        assert!(count >= 1);
        assert!(count <= MAX_POINTS_PER_CURVE);
    }

    #[test]
    fn test_nextlog16() {
        assert_eq!(nextlog16(1.0), 0);
        assert_eq!(nextlog16(16.0), 1);
        assert_eq!(nextlog16(256.0), 2);
        assert_eq!(nextlog16(4096.0), 3);
    }

    #[test]
    fn test_cubic_log2_with_control_points() {
        // Test with a simple cubic Bezier
        let points = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]];
        let precision = 4.0; // from tolerance = 0.25
        let log2 = cubic_log2(precision, &points);
        assert!(log2 < MAX_POINTS_PER_CURVE);
    }

    #[test]
    fn test_constants() {
        assert_eq!(MAX_POINTS_PER_CURVE, 1024);
        assert!((DEFAULT_TOLERANCE - 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_cubic_point_count_with_various_tolerances() {
        let points = [[0.0, 0.0], [1.0, 1.0], [2.0, 1.0], [3.0, 0.0]];
        
        // Tighter tolerance should require more points
        let count_tight = cubic_point_count(&points, 0.1);
        let count_loose = cubic_point_count(&points, 0.5);
        assert!(count_tight >= count_loose);
    }
}
