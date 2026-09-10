//! Parameterized line used to measure which side of a line a point falls on.
//!
//! Port of Skia's `SkLineParameters.h`.
//!
//! Sources: computer-aided design, volume 22 number 9, november 1990,
//! pp 538-549; online at <http://cagd.cs.byu.edu/~tom/papers/bezclip.pdf>.
//!
//! A line segment is turned into a parameterized line of the form
//! `ax + by + c = 0`. When `a^2 + b^2 == 1` the line is normalized, and the
//! distance from `(x, y)` to the line is `d(x, y) = ax + by + c`.
//!
//! The distances computed here are not necessarily normalized. To get a true
//! distance, either call [`SkLineParameters::normalize`] after one of the
//! `*_end_points` methods, or divide the result by the square root of
//! [`SkLineParameters::normal_squared`].

use super::sk_path_ops_types::{approximately_zero, not_almost_equal_ulps, roughly_equal_ulps};

/// Coefficients of a line in the implicit form `ax + by + c = 0`.
///
/// Points are taken as `[x, y]` pairs in f64, matching the C++ original's use
/// of `double` throughout. Curves are passed as slices: 2 points for a line,
/// 3 for a quad or conic, 4 for a cubic.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SkLineParameters {
    /// The `a` coefficient.
    pub f_a: f64,
    /// The `b` coefficient.
    pub f_b: f64,
    /// The `c` coefficient.
    pub f_c: f64,
}

/// A point in the f64 space these computations use.
pub type LinePoint = [f64; 2];

const X: usize = 0;
const Y: usize = 1;

/// Returns true if `xy` is within ULPs tolerance of the ray through `line`.
///
/// Port of `SkDLine::nearRay` from `SkPathOpsLine.cpp`; lives here because
/// [`SkLineParameters::cubic_part`] is its only caller in this module.
fn near_ray(line: &[LinePoint], xy: LinePoint) -> bool {
    // project a perpendicular ray from the point to the line; find the T on the line
    let len_x = line[1][X] - line[0][X];
    let len_y = line[1][Y] - line[0][Y];
    let denom = len_x * len_x + len_y * len_y;
    let ab0_x = xy[X] - line[0][X];
    let ab0_y = xy[Y] - line[0][Y];
    let numer = len_x * ab0_x + ab0_y * len_y;
    let t = numer / denom;
    let real_x = line[0][X] + t * len_x;
    let real_y = line[0][Y] + t * len_y;
    let dist = ((real_x - xy[X]).powi(2) + (real_y - xy[Y]).powi(2)).sqrt();
    // find the ordinal in the original line with the largest unsigned exponent
    let tiniest = line[0][X].min(line[0][Y]).min(line[1][X]).min(line[1][Y]);
    let largest = line[0][X].max(line[0][Y]).max(line[1][X]).max(line[1][Y]);
    let largest = largest.max(-tiniest);
    // is the dist within ULPS tolerance?
    roughly_equal_ulps(largest as f32, (largest + dist) as f32)
}

impl SkLineParameters {
    /// Returns coefficients that are all zero.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            f_a: 0.0,
            f_b: 0.0,
            f_c: 0.0,
        }
    }

    /// Sets the coefficients from the cubic's endpoints, choosing a control
    /// point that yields a usable tangent.
    ///
    /// Returns false when the cubic degenerates to a line.
    pub fn cubic_end_points(&mut self, pts: &[LinePoint]) -> bool {
        let mut end_index = 1;
        self.cubic_end_points_at(pts, 0, end_index);
        if self.dy() != 0.0 {
            return true;
        }
        if self.dx() == 0.0 {
            end_index += 1;
            self.cubic_end_points_at(pts, 0, end_index);
            debug_assert_eq!(end_index, 2);
            if self.dy() != 0.0 {
                return true;
            }
            if self.dx() == 0.0 {
                end_index += 1;
                self.cubic_end_points_at(pts, 0, end_index); // line
                debug_assert_eq!(end_index, 3);
                return false;
            }
        }
        // FIXME: after switching to round sort, remove bumping fA
        if self.dx() < 0.0 {
            // only worry about y bias when breaking cw/ccw tie
            return true;
        }
        // if cubic tangent is on x axis, look at next control point to break tie
        // control point may be approximate, so it must move significantly to account for error
        end_index += 1;
        if not_almost_equal_ulps(pts[0][Y] as f32, pts[end_index][Y] as f32) {
            if pts[0][Y] > pts[end_index][Y] {
                // push it from 0 to slightly negative (dy() returns -a)
                self.f_a = f64::EPSILON;
            }
            return true;
        }
        if end_index == 3 {
            return true;
        }
        debug_assert_eq!(end_index, 2);
        if pts[0][Y] > pts[3][Y] {
            // push it from 0 to slightly negative (dy() returns -a)
            self.f_a = f64::EPSILON;
        }
        true
    }

    /// Sets the coefficients from the line through `pts[s]` and `pts[e]`.
    pub fn cubic_end_points_at(&mut self, pts: &[LinePoint], s: usize, e: usize) {
        self.f_a = pts[s][Y] - pts[e][Y];
        self.f_b = pts[e][X] - pts[s][X];
        self.f_c = pts[s][X] * pts[e][Y] - pts[e][X] * pts[s][Y];
    }

    /// Sets the coefficients from a cubic and returns the distance from the
    /// line to whichever control point is furthest off it.
    pub fn cubic_part(&mut self, part: &[LinePoint]) -> f64 {
        self.cubic_end_points(part);
        if part[0] == part[1] || near_ray(&part[0..2], part[2]) {
            return self.point_distance(part[3]);
        }
        self.point_distance(part[2])
    }

    /// Sets the coefficients from a line's two endpoints.
    pub fn line_end_points(&mut self, pts: &[LinePoint]) {
        self.f_a = pts[0][Y] - pts[1][Y];
        self.f_b = pts[1][X] - pts[0][X];
        self.f_c = pts[0][X] * pts[1][Y] - pts[1][X] * pts[0][Y];
    }

    /// Sets the coefficients from the quad's endpoints.
    ///
    /// Returns false when the quad degenerates to a line.
    pub fn quad_end_points(&mut self, pts: &[LinePoint]) -> bool {
        self.quad_end_points_at(pts, 0, 1);
        if self.dy() != 0.0 {
            return true;
        }
        if self.dx() == 0.0 {
            self.quad_end_points_at(pts, 0, 2);
            return false;
        }
        if self.dx() < 0.0 {
            // only worry about y bias when breaking cw/ccw tie
            return true;
        }
        // FIXME: after switching to round sort, remove this
        if pts[0][Y] > pts[2][Y] {
            self.f_a = f64::EPSILON;
        }
        true
    }

    /// Sets the coefficients from the line through `pts[s]` and `pts[e]`.
    pub fn quad_end_points_at(&mut self, pts: &[LinePoint], s: usize, e: usize) {
        self.f_a = pts[s][Y] - pts[e][Y];
        self.f_b = pts[e][X] - pts[s][X];
        self.f_c = pts[s][X] * pts[e][Y] - pts[e][X] * pts[s][Y];
    }

    /// Sets the coefficients from a quad and returns the distance from the
    /// line to its control point.
    pub fn quad_part(&mut self, part: &[LinePoint]) -> f64 {
        self.quad_end_points(part);
        self.point_distance(part[2])
    }

    /// Returns `a^2 + b^2`, the squared length of the line's normal.
    #[must_use]
    pub fn normal_squared(&self) -> f64 {
        self.f_a * self.f_a + self.f_b * self.f_b
    }

    /// Scales the coefficients so the normal has unit length.
    ///
    /// Returns false and zeroes the coefficients if the normal is degenerate.
    pub fn normalize(&mut self) -> bool {
        let normal = self.normal_squared().sqrt();
        if approximately_zero(normal) {
            self.f_a = 0.0;
            self.f_b = 0.0;
            self.f_c = 0.0;
            return false;
        }
        let reciprocal = 1.0 / normal;
        self.f_a *= reciprocal;
        self.f_b *= reciprocal;
        self.f_c *= reciprocal;
        true
    }

    /// Writes each cubic point's distance from the line into `distance` as a
    /// curve parameterized by `x = index / 3`.
    pub fn cubic_distance_y(&self, pts: &[LinePoint], distance: &mut [LinePoint]) {
        const ONE_THIRD: f64 = 1.0 / 3.0;
        for index in 0..4 {
            distance[index][X] = index as f64 * ONE_THIRD;
            distance[index][Y] = self.f_a * pts[index][X] + self.f_b * pts[index][Y] + self.f_c;
        }
    }

    /// Writes each quad point's distance from the line into `distance` as a
    /// curve parameterized by `x = index / 2`.
    pub fn quad_distance_y(&self, pts: &[LinePoint], distance: &mut [LinePoint]) {
        const ONE_HALF: f64 = 1.0 / 2.0;
        for index in 0..3 {
            distance[index][X] = index as f64 * ONE_HALF;
            distance[index][Y] = self.f_a * pts[index][X] + self.f_b * pts[index][Y] + self.f_c;
        }
    }

    /// Returns the distance from the line to cubic control point `index`,
    /// which must be 1 or 2.
    #[must_use]
    pub fn control_pt_distance_cubic(&self, pts: &[LinePoint], index: usize) -> f64 {
        debug_assert!(index == 1 || index == 2);
        self.f_a * pts[index][X] + self.f_b * pts[index][Y] + self.f_c
    }

    /// Returns the distance from the line to the quad's control point.
    #[must_use]
    pub fn control_pt_distance_quad(&self, pts: &[LinePoint]) -> f64 {
        self.f_a * pts[1][X] + self.f_b * pts[1][Y] + self.f_c
    }

    /// Returns the signed distance from the line to `pt`.
    #[must_use]
    pub fn point_distance(&self, pt: LinePoint) -> f64 {
        self.f_a * pt[X] + self.f_b * pt[Y] + self.f_c
    }

    /// Returns the line direction's x component.
    #[must_use]
    pub fn dx(&self) -> f64 {
        self.f_b
    }

    /// Returns the line direction's y component.
    #[must_use]
    pub fn dy(&self) -> f64 {
        -self.f_a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_end_points_gives_implicit_form() {
        // The line y = 0 from (0,0) to (1,0): a=0, b=1, c=0.
        let mut lp = SkLineParameters::new();
        lp.line_end_points(&[[0.0, 0.0], [1.0, 0.0]]);
        assert_eq!(lp.f_a, 0.0);
        assert_eq!(lp.f_b, 1.0);
        assert_eq!(lp.f_c, 0.0);
        // Distance is then just the y coordinate.
        assert_eq!(lp.point_distance([5.0, 3.0]), 3.0);
        assert_eq!(lp.point_distance([5.0, -3.0]), -3.0);
    }

    #[test]
    fn point_distance_sign_marks_the_side() {
        // Diagonal from (0,0) to (1,1).
        let mut lp = SkLineParameters::new();
        lp.line_end_points(&[[0.0, 0.0], [1.0, 1.0]]);
        // (0,1) is above the line, (1,0) below; signs must differ.
        let above = lp.point_distance([0.0, 1.0]);
        let below = lp.point_distance([1.0, 0.0]);
        assert!(above * below < 0.0);
        // A point on the line has zero distance.
        assert_eq!(lp.point_distance([0.5, 0.5]), 0.0);
    }

    #[test]
    fn dx_dy_recover_the_direction() {
        let mut lp = SkLineParameters::new();
        lp.line_end_points(&[[1.0, 2.0], [4.0, 6.0]]);
        assert_eq!(lp.dx(), 3.0);
        assert_eq!(lp.dy(), 4.0);
    }

    #[test]
    fn normalize_scales_distance_to_true_distance() {
        // Line from (0,0) to (0,3) is x = 0; unnormalized distances scale by 3.
        let mut lp = SkLineParameters::new();
        lp.line_end_points(&[[0.0, 0.0], [0.0, 3.0]]);
        assert_eq!(lp.point_distance([2.0, 0.0]), -6.0);
        assert_eq!(lp.normal_squared(), 9.0);
        assert!(lp.normalize());
        assert!((lp.point_distance([2.0, 0.0]) + 2.0).abs() < 1e-12);
        assert!((lp.normal_squared() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn normalize_fails_on_degenerate_line() {
        let mut lp = SkLineParameters::new();
        lp.line_end_points(&[[1.0, 1.0], [1.0, 1.0]]);
        assert!(!lp.normalize());
        assert_eq!(lp.f_a, 0.0);
        assert_eq!(lp.f_b, 0.0);
        assert_eq!(lp.f_c, 0.0);
    }

    #[test]
    fn quad_end_points_uses_first_control_point() {
        // Tangent leaves (0,0) toward (1,1), so dy != 0 and the first pair wins.
        let mut lp = SkLineParameters::new();
        let pts = [[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]];
        assert!(lp.quad_end_points(&pts));
        assert_eq!(lp.dx(), 1.0);
        assert_eq!(lp.dy(), 1.0);
    }

    #[test]
    fn quad_end_points_reports_line_when_degenerate() {
        // First control point coincides with the start: no tangent from it.
        let mut lp = SkLineParameters::new();
        let pts = [[0.0, 0.0], [0.0, 0.0], [2.0, 0.0]];
        assert!(!lp.quad_end_points(&pts));
        // Falls back to the 0..2 chord.
        assert_eq!(lp.dx(), 2.0);
        assert_eq!(lp.dy(), 0.0);
    }

    #[test]
    fn quad_end_points_biases_y_to_break_cw_ccw_tie() {
        // Horizontal tangent going +x with the end point below the start:
        // fA is bumped off zero so the sort has a direction to work with.
        let mut lp = SkLineParameters::new();
        let pts = [[0.0, 5.0], [1.0, 5.0], [2.0, 0.0]];
        assert!(lp.quad_end_points(&pts));
        assert_eq!(lp.f_a, f64::EPSILON);
        assert!(lp.dy() < 0.0);
    }

    #[test]
    fn quad_part_measures_the_control_point() {
        let mut lp = SkLineParameters::new();
        // Chord along y=0; control point sits at height 4.
        let pts = [[0.0, 0.0], [2.0, 0.0], [4.0, 4.0]];
        let dist = lp.quad_part(&pts);
        assert!(dist != 0.0);
        assert_eq!(dist, lp.point_distance(pts[2]));
    }

    #[test]
    fn cubic_end_points_uses_first_control_point() {
        let mut lp = SkLineParameters::new();
        let pts = [[0.0, 0.0], [1.0, 2.0], [2.0, 2.0], [3.0, 0.0]];
        assert!(lp.cubic_end_points(&pts));
        assert_eq!(lp.dx(), 1.0);
        assert_eq!(lp.dy(), 2.0);
    }

    #[test]
    fn cubic_end_points_walks_forward_past_coincident_controls() {
        // pts[1] equals pts[0], so the tangent comes from pts[2].
        let mut lp = SkLineParameters::new();
        let pts = [[0.0, 0.0], [0.0, 0.0], [2.0, 3.0], [4.0, 0.0]];
        assert!(lp.cubic_end_points(&pts));
        assert_eq!(lp.dx(), 2.0);
        assert_eq!(lp.dy(), 3.0);
    }

    #[test]
    fn cubic_end_points_reports_line_when_fully_degenerate() {
        // First three points identical: the cubic is a line to pts[3].
        let mut lp = SkLineParameters::new();
        let pts = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [4.0, 0.0]];
        assert!(!lp.cubic_end_points(&pts));
        assert_eq!(lp.dx(), 4.0);
        assert_eq!(lp.dy(), 0.0);
    }

    #[test]
    fn cubic_part_falls_through_to_last_point_on_a_near_ray() {
        // pts[2] lies on the ray through pts[0]..pts[1], so pts[3] is measured.
        let mut lp = SkLineParameters::new();
        let pts = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 5.0]];
        let dist = lp.cubic_part(&pts);
        assert_eq!(dist, lp.point_distance(pts[3]));
        assert!(dist != 0.0);
    }

    #[test]
    fn cubic_distance_y_parameterizes_by_thirds() {
        let mut lp = SkLineParameters::new();
        lp.line_end_points(&[[0.0, 0.0], [1.0, 0.0]]);
        let pts = [[0.0, 1.0], [1.0, 2.0], [2.0, 3.0], [3.0, 4.0]];
        let mut dist = [[0.0; 2]; 4];
        lp.cubic_distance_y(&pts, &mut dist);
        assert_eq!(dist[0], [0.0, 1.0]);
        assert!((dist[1][X] - 1.0 / 3.0).abs() < 1e-12);
        assert_eq!(dist[3][X], 1.0);
        // With a=0,b=1,c=0 the distance is the y coordinate.
        assert_eq!(dist[3][Y], 4.0);
    }

    #[test]
    fn quad_distance_y_parameterizes_by_halves() {
        let mut lp = SkLineParameters::new();
        lp.line_end_points(&[[0.0, 0.0], [1.0, 0.0]]);
        let pts = [[0.0, 1.0], [1.0, 2.0], [2.0, 3.0]];
        let mut dist = [[0.0; 2]; 3];
        lp.quad_distance_y(&pts, &mut dist);
        assert_eq!(dist[0], [0.0, 1.0]);
        assert_eq!(dist[1], [0.5, 2.0]);
        assert_eq!(dist[2], [1.0, 3.0]);
    }

    #[test]
    fn control_pt_distance_matches_point_distance() {
        let mut lp = SkLineParameters::new();
        lp.line_end_points(&[[0.0, 0.0], [1.0, 1.0]]);
        let cubic = [[0.0, 0.0], [1.0, 2.0], [2.0, 1.0], [3.0, 3.0]];
        assert_eq!(
            lp.control_pt_distance_cubic(&cubic, 1),
            lp.point_distance(cubic[1])
        );
        assert_eq!(
            lp.control_pt_distance_cubic(&cubic, 2),
            lp.point_distance(cubic[2])
        );
        let quad = [[0.0, 0.0], [1.0, 2.0], [2.0, 1.0]];
        assert_eq!(
            lp.control_pt_distance_quad(&quad),
            lp.point_distance(quad[1])
        );
    }
}
