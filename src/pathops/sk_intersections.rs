//! Intersection detection between path segments
//!
//! Port of Skia's SkIntersections.{h,cpp}

use crate::core::{Point, Scalar};

use super::sk_path_ops_line::{pin_t, DLine};
use super::sk_path_ops_types::{
    almost_equal_ulps, approximately_zero, not_almost_dequal_ulps, not_almost_equal_ulps_pin,
    precisely_between,
};

/// Maximum number of intersection points per curve pair
const MAX_INTERSECTIONS: usize = 13;

/// Stores intersection points between two curve segments
#[derive(Debug, Clone)]
pub struct SkIntersections {
    /// Intersection points
    f_pt: [Point; MAX_INTERSECTIONS],
    /// Secondary intersection points (for nearly coincident cases)
    f_pt2: [Point; MAX_INTERSECTIONS],
    /// T values for first curve
    f_t: [[Scalar; MAX_INTERSECTIONS]; 2],
    /// Coincidence flags (bit set for each curve's coincident T)
    f_is_coincident: [u16; 2],
    /// True if end points nearly match
    f_nearly_same: [bool; 2],
    /// Number of intersection points
    f_used: u8,
    /// Maximum allowed intersections
    f_max: u8,
    /// Allow nearly coincident
    f_allow_near: bool,
    /// Swap flag for curve order
    f_swap: bool,
}

#[allow(clippy::needless_range_loop)] // the index doubles as an endpoint's t
impl SkIntersections {
    pub fn new() -> Self {
        Self {
            f_pt: [Point::new(0.0, 0.0); MAX_INTERSECTIONS],
            f_pt2: [Point::new(0.0, 0.0); MAX_INTERSECTIONS],
            f_t: [[0.0; MAX_INTERSECTIONS]; 2],
            f_is_coincident: [0; 2],
            f_nearly_same: [false; 2],
            f_used: 0,
            f_max: 0,
            f_allow_near: true,
            f_swap: false,
        }
    }

    pub fn used(&self) -> usize {
        self.f_used as usize
    }

    pub fn set_max(&mut self, max: usize) {
        self.f_max = max as u8;
    }

    pub fn swap(&mut self) {
        self.f_swap = !self.f_swap;
    }

    pub fn swapped(&self) -> bool {
        self.f_swap
    }

    pub fn reset(&mut self) {
        self.f_allow_near = true;
        self.f_used = 0;
        self.f_is_coincident = [0; 2];
    }

    pub fn t(&self, curve: usize, index: usize) -> Scalar {
        self.f_t[curve][index]
    }

    pub fn pt(&self, index: usize) -> Point {
        self.f_pt[index]
    }

    pub fn is_coincident(&self, index: usize) -> bool {
        (self.f_is_coincident[0] & (1 << index)) != 0
    }

    pub fn set_coincident(&mut self, index: usize) {
        let bit: u16 = 1 << index;
        self.f_is_coincident[0] |= bit;
        self.f_is_coincident[1] |= bit;
    }

    pub fn pt_mut(&mut self, index: usize) -> &mut Point {
        &mut self.f_pt[index]
    }

    pub fn t_mut(&mut self, curve: usize, index: usize) -> &mut Scalar {
        &mut self.f_t[curve][index]
    }

    pub fn insert(&mut self, one: Scalar, two: Scalar, pt: Point) -> i32 {
        if self.f_is_coincident[0] == 3
            && between(self.f_t[0][0], one, self.f_t[0][1])
        {
            // For now, don't allow a mix of coincident and non-coincident
            return -1;
        }

        // Check for duplicate
        for index in 0..self.f_used as usize {
            if self.f_t[0][index] == one && self.f_t[1][index] == two {
                return -1;
            }

            if roughly_equal(self.f_t[0][index], one)
                && roughly_equal(self.f_t[1][index], two)
            {
                // Remove and reinsert if needed
                let remaining = self.f_used as usize - index - 1;
                if remaining > 0 {
                    for i in 0..remaining {
                        self.f_pt[index] = self.f_pt[index + 1 + i];
                        self.f_t[0][index] = self.f_t[0][index + 1 + i];
                        self.f_t[1][index] = self.f_t[1][index + 1 + i];
                    }
                }
                self.f_used -= 1;
                break;
            }
        }

        // Find insertion point (sorted by t)
        let mut index = 0;
        while index < self.f_used as usize {
            if self.f_t[0][index] > one {
                break;
            }
            index += 1;
        }

        if self.f_used as usize >= self.f_max as usize {
            return -1;
        }

        // Shift to make room
        let remaining = self.f_used as usize - index;
        if remaining > 0 {
            for i in (0..remaining).rev() {
                self.f_pt[index + 1 + i] = self.f_pt[index + i];
                self.f_t[0][index + 1 + i] = self.f_t[0][index + i];
                self.f_t[1][index + 1 + i] = self.f_t[1][index + i];
            }
        }

        self.f_pt[index] = pt;
        self.f_t[0][index] = one;
        self.f_t[1][index] = two;
        self.f_used += 1;

        index as i32
    }

    /// Intersects two lines treated as infinite rays.
    ///
    /// Port of `SkIntersections::intersectRay`. Unlike [`Self::intersect`] the
    /// t values are not clipped to the segments, so a caller gets the crossing
    /// of the underlying lines even when it lies outside both.
    ///
    /// Coincident rays report both ends rather than nothing, since there is no
    /// single meaningful crossing point.
    pub fn intersect_ray(&mut self, a: &DLine, b: &DLine) -> u8 {
        self.f_max = 2;
        let a_len = Point::new(a.p[1].x - a.p[0].x, a.p[1].y - a.p[0].y);
        let b_len = Point::new(b.p[1].x - b.p[0].x, b.p[1].y - b.p[0].y);
        // Slopes match exactly when this denominator goes to zero.
        let denom = b_len.y * a_len.x - a_len.y * b_len.x;
        #[allow(clippy::needless_late_init)] // mirrors the C++ branch structure
        let used;
        if !approximately_zero(denom as f64) {
            let ab0 = Point::new(a.p[0].x - b.p[0].x, a.p[0].y - b.p[0].y);
            let numer_a = ab0.y * b_len.x - b_len.y * ab0.x;
            let numer_b = ab0.y * a_len.x - a_len.y * ab0.x;
            self.f_t[0][0] = numer_a / denom;
            self.f_t[1][0] = numer_b / denom;
            used = 1;
        } else {
            // Parallel. They only coincide if their axis intercepts match too.
            if !almost_equal_ulps(
                a_len.x * a.p[0].y - a_len.y * a.p[0].x,
                a_len.x * b.p[0].y - a_len.y * b.p[0].x,
            ) {
                self.f_used = 0;
                return 0;
            }
            // No good answer for coincident rays; report the whole span.
            self.f_t[0][0] = 0.0;
            self.f_t[1][0] = 0.0;
            self.f_t[0][1] = 1.0;
            self.f_t[1][1] = 1.0;
            used = 2;
        }
        self.compute_points(a, used);
        self.f_used
    }

    /// Intersects two line segments.
    ///
    /// Port of `SkIntersections::intersect`. Endpoints that land exactly on the
    /// other line are recorded first, so a vertex shared by two adjacent
    /// segments registers once rather than as two near-misses, and collinear
    /// overlapping runs report both ends of the overlap.
    ///
    /// Only valid when neither line is horizontal or vertical; use
    /// [`Self::horizontal`] and [`Self::vertical`] for those.
    pub fn intersect(&mut self, a: &DLine, b: &DLine) -> u8 {
        // Three, so a third can be inserted before cleanup trims back to two.
        self.f_max = 3;
        for i_a in 0..2 {
            let t = b.exact_point(a.p[i_a]);
            if t >= 0.0 {
                let _ = self.insert(i_a as Scalar, t, a.p[i_a]);
            }
        }
        for i_b in 0..2 {
            let t = a.exact_point(b.p[i_b]);
            if t >= 0.0 {
                let _ = self.insert(t, i_b as Scalar, b.p[i_b]);
            }
        }
        let ax_len = a.p[1].x - a.p[0].x;
        let ay_len = a.p[1].y - a.p[0].y;
        let bx_len = b.p[1].x - b.p[0].x;
        let by_len = b.p[1].y - b.p[0].y;
        let ax_by_len = ax_len * by_len;
        let ay_bx_len = ay_len * bx_len;
        // Parallel is detected the same way here and in SkOpAngle's ordering,
        // so that "not parallel" also means "sortable".
        let unparallel = if self.f_allow_near {
            not_almost_equal_ulps_pin(ax_by_len, ay_bx_len)
        } else {
            not_almost_dequal_ulps(ax_by_len, ay_bx_len)
        };
        if unparallel && self.f_used == 0 {
            let ab0y = a.p[0].y - b.p[0].y;
            let ab0x = a.p[0].x - b.p[0].x;
            let numer_a = ab0y * bx_len - by_len * ab0x;
            let numer_b = ab0y * ax_len - ay_len * ab0x;
            let denom = ax_by_len - ay_bx_len;
            if between(0.0, numer_a, denom) && between(0.0, numer_b, denom) {
                self.f_t[0][0] = numer_a / denom;
                self.f_t[1][0] = numer_b / denom;
                self.compute_points(a, 1);
            }
        }
        // Track that both sets of end points are near each other - the lines
        // are entirely coincident - even when the end points are not exactly
        // equal. Either end is then free to mate with the next set of lines
        // without the pair folding back over itself.
        if self.f_allow_near || !unparallel {
            let mut a_near_b = [0.0f32; 2];
            let mut b_near_a = [0.0f32; 2];
            let mut a_not_b = [false; 2];
            let mut b_not_a = [false; 2];
            let mut near_count = 0;
            for index in 0..2 {
                let mut flag = false;
                a_near_b[index] = b.near_point(a.p[index], Some(&mut flag));
                a_not_b[index] = flag;
                near_count += i32::from(a_near_b[index] >= 0.0);
                let mut flag = false;
                b_near_a[index] = a.near_point(b.p[index], Some(&mut flag));
                b_not_a[index] = flag;
                near_count += i32::from(b_near_a[index] >= 0.0);
            }
            if near_count > 0 {
                // Skip when each segment contributes just one end point.
                if near_count != 2 || a_not_b[0] == a_not_b[1] {
                    for i_a in 0..2 {
                        if !a_not_b[i_a] {
                            continue;
                        }
                        let nearer = usize::from(a_near_b[i_a] > 0.5);
                        if !b_not_a[nearer] {
                            continue;
                        }
                        let _ = self.insert_near(
                            i_a as Scalar,
                            nearer as Scalar,
                            a.p[i_a],
                            b.p[nearer],
                        );
                        a_near_b[i_a] = -1.0;
                        b_near_a[nearer] = -1.0;
                        near_count -= 2;
                    }
                }
                if near_count > 0 {
                    for i_a in 0..2 {
                        if a_near_b[i_a] >= 0.0 {
                            let _ = self.insert(i_a as Scalar, a_near_b[i_a], a.p[i_a]);
                        }
                    }
                    for i_b in 0..2 {
                        if b_near_a[i_b] >= 0.0 {
                            let _ = self.insert(b_near_a[i_b], i_b as Scalar, b.p[i_b]);
                        }
                    }
                }
            }
        }
        self.clean_up_parallel_lines(!unparallel);
        debug_assert!(self.f_used <= 2);
        self.f_used
    }

    /// Returns the t at which `line` crosses the horizontal `y`.
    ///
    /// Port of `SkIntersections::HorizontalIntercept`.
    #[must_use]
    pub fn horizontal_intercept(line: &DLine, y: Scalar) -> Scalar {
        debug_assert!(line.p[1].y != line.p[0].y);
        pin_t((y - line.p[0].y) / (line.p[1].y - line.p[0].y))
    }

    /// Intersects `line` with the horizontal span from `left` to `right` at `y`.
    ///
    /// Port of `SkIntersections::horizontal`. `flipped` reverses the span's
    /// parameterization, for callers that walk it right to left.
    pub fn horizontal(
        &mut self,
        line: &DLine,
        left: Scalar,
        right: Scalar,
        y: Scalar,
        flipped: bool,
    ) -> u8 {
        // Cleaning up parallel lines at the end limits the result to 2.
        self.f_max = 3;
        let left_pt = Point::new(left, y);
        let t = line.exact_point(left_pt);
        if t >= 0.0 {
            let _ = self.insert(t, Scalar::from(u8::from(flipped)), left_pt);
        }
        if left != right {
            let right_pt = Point::new(right, y);
            let t = line.exact_point(right_pt);
            if t >= 0.0 {
                let _ = self.insert(t, Scalar::from(u8::from(!flipped)), right_pt);
            }
            for index in 0..2 {
                let t = DLine::exact_point_h(line.p[index], left, right, y);
                if t >= 0.0 {
                    let ty = if flipped { 1.0 - t } else { t };
                    let _ = self.insert(index as Scalar, ty, line.p[index]);
                }
            }
        }
        let result = horizontal_coincident(line, y);
        if result == 1 && self.f_used == 0 {
            self.f_t[0][0] = Self::horizontal_intercept(line, y);
            let x_intercept = line.p[0].x + self.f_t[0][0] * (line.p[1].x - line.p[0].x);
            if between(left, x_intercept, right) {
                self.f_t[1][0] = (x_intercept - left) / (right - left);
                if flipped {
                    for index in 0..result as usize {
                        self.f_t[1][index] = 1.0 - self.f_t[1][index];
                    }
                }
                self.f_pt[0] = Point::new(x_intercept, y);
                self.f_used = 1;
            }
        }
        if self.f_allow_near || result == 2 {
            let t = line.near_point(left_pt, None);
            if t >= 0.0 {
                let _ = self.insert(t, Scalar::from(u8::from(flipped)), left_pt);
            }
            if left != right {
                let right_pt = Point::new(right, y);
                let t = line.near_point(right_pt, None);
                if t >= 0.0 {
                    let _ = self.insert(t, Scalar::from(u8::from(!flipped)), right_pt);
                }
                for index in 0..2 {
                    let t = DLine::near_point_h(line.p[index], left, right, y);
                    if t >= 0.0 {
                        let ty = if flipped { 1.0 - t } else { t };
                        let _ = self.insert(index as Scalar, ty, line.p[index]);
                    }
                }
            }
        }
        self.clean_up_parallel_lines(result == 2);
        self.f_used
    }

    /// Returns the t at which `line` crosses the vertical `x`.
    ///
    /// Port of `SkIntersections::VerticalIntercept`.
    #[must_use]
    pub fn vertical_intercept(line: &DLine, x: Scalar) -> Scalar {
        debug_assert!(line.p[1].x != line.p[0].x);
        pin_t((x - line.p[0].x) / (line.p[1].x - line.p[0].x))
    }

    /// Intersects `line` with the vertical span from `top` to `bottom` at `x`.
    ///
    /// Port of `SkIntersections::vertical`.
    pub fn vertical(
        &mut self,
        line: &DLine,
        top: Scalar,
        bottom: Scalar,
        x: Scalar,
        flipped: bool,
    ) -> u8 {
        self.f_max = 3;
        let top_pt = Point::new(x, top);
        let t = line.exact_point(top_pt);
        if t >= 0.0 {
            let _ = self.insert(t, Scalar::from(u8::from(flipped)), top_pt);
        }
        if top != bottom {
            let bottom_pt = Point::new(x, bottom);
            let t = line.exact_point(bottom_pt);
            if t >= 0.0 {
                let _ = self.insert(t, Scalar::from(u8::from(!flipped)), bottom_pt);
            }
            for index in 0..2 {
                let t = DLine::exact_point_v(line.p[index], top, bottom, x);
                if t >= 0.0 {
                    let ty = if flipped { 1.0 - t } else { t };
                    let _ = self.insert(index as Scalar, ty, line.p[index]);
                }
            }
        }
        let result = vertical_coincident(line, x);
        if result == 1 && self.f_used == 0 {
            self.f_t[0][0] = Self::vertical_intercept(line, x);
            let y_intercept = line.p[0].y + self.f_t[0][0] * (line.p[1].y - line.p[0].y);
            if between(top, y_intercept, bottom) {
                self.f_t[1][0] = (y_intercept - top) / (bottom - top);
                if flipped {
                    for index in 0..result as usize {
                        self.f_t[1][index] = 1.0 - self.f_t[1][index];
                    }
                }
                self.f_pt[0] = Point::new(x, y_intercept);
                self.f_used = 1;
            }
        }
        if self.f_allow_near || result == 2 {
            let t = line.near_point(top_pt, None);
            if t >= 0.0 {
                let _ = self.insert(t, Scalar::from(u8::from(flipped)), top_pt);
            }
            if top != bottom {
                let bottom_pt = Point::new(x, bottom);
                let t = line.near_point(bottom_pt, None);
                if t >= 0.0 {
                    let _ = self.insert(t, Scalar::from(u8::from(!flipped)), bottom_pt);
                }
                for index in 0..2 {
                    let t = DLine::near_point_v(line.p[index], top, bottom, x);
                    if t >= 0.0 {
                        let ty = if flipped { 1.0 - t } else { t };
                        let _ = self.insert(index as Scalar, ty, line.p[index]);
                    }
                }
            }
        }
        self.clean_up_parallel_lines(result == 2);
        debug_assert!(self.f_used <= 2);
        self.f_used
    }

    /// Trims a line/line result down to at most two intersections.
    ///
    /// Port of `SkIntersections::cleanUpParallelLines`. Two surviving entries
    /// are marked coincident, which is how a collinear overlap is reported.
    fn clean_up_parallel_lines(&mut self, parallel: bool) {
        while self.f_used > 2 {
            self.remove_one(1);
        }
        if self.f_used == 2 && !parallel {
            let start_match = self.f_t[0][0] == 0.0 || zero_or_one(self.f_t[1][0]);
            let end_match = self.f_t[0][1] == 1.0 || zero_or_one(self.f_t[1][1]);
            if (!start_match && !end_match)
                || (self.f_t[0][0] - self.f_t[0][1]).abs() < f32::EPSILON
            {
                if start_match
                    && end_match
                    && (self.f_t[0][0] != 0.0 || !zero_or_one(self.f_t[1][0]))
                    && self.f_t[0][1] == 1.0
                    && zero_or_one(self.f_t[1][1])
                {
                    self.remove_one(0);
                } else {
                    self.remove_one(usize::from(end_match));
                }
            }
        }
        if self.f_used == 2 {
            self.f_is_coincident[0] = 0x03;
            self.f_is_coincident[1] = 0x03;
        }
    }

    /// Fills in the intersection points from the t values on `line`.
    ///
    /// Port of `SkIntersections::computePoints`.
    fn compute_points(&mut self, line: &DLine, used: u8) {
        self.f_pt[0] = line.pt_at_t(self.f_t[0][0]);
        self.f_used = used;
        if used == 2 {
            self.f_pt[1] = line.pt_at_t(self.f_t[0][1]);
        }
    }

    /// Intersects two lines given as raw point pairs.
    ///
    /// Convenience wrapper over [`Self::intersect`].
    pub fn line_line(&mut self, a: &[Point; 2], b: &[Point; 2]) -> u8 {
        self.reset();
        self.intersect(&DLine::new(a[0], a[1]), &DLine::new(b[0], b[1]))
    }

    /// Intersects a line with a horizontal span.
    ///
    /// Convenience wrapper over [`Self::horizontal`].
    pub fn line_horizontal(
        &mut self,
        pts: &[Point; 2],
        left: Scalar,
        right: Scalar,
        y: Scalar,
        flipped: bool,
    ) -> u8 {
        self.reset();
        self.horizontal(&DLine::new(pts[0], pts[1]), left, right, y, flipped)
    }

    /// Intersects a line with a vertical span.
    ///
    /// Convenience wrapper over [`Self::vertical`].
    pub fn line_vertical(
        &mut self,
        pts: &[Point; 2],
        top: Scalar,
        bottom: Scalar,
        x: Scalar,
        flipped: bool,
    ) -> u8 {
        self.reset();
        self.vertical(&DLine::new(pts[0], pts[1]), top, bottom, x, flipped)
    }

    pub fn closest_to(&self, range_start: Scalar, range_end: Scalar, test_pt: Point, closest_dist: &mut Scalar) -> i32 {
        let mut closest = -1;
        *closest_dist = Scalar::MAX;
        for index in 0..self.f_used as usize {
            if !between(range_start, self.f_t[0][index], range_end) {
                continue;
            }
            let dx = self.f_pt[index].x - test_pt.x;
            let dy = self.f_pt[index].y - test_pt.y;
            let dist = dx * dx + dy * dy;
            if *closest_dist > dist {
                *closest_dist = dist;
                closest = index as i32;
            }
        }
        closest
    }

    pub fn flip(&mut self) {
        for index in 0..self.f_used as usize {
            self.f_t[1][index] = 1.0 - self.f_t[1][index];
        }
    }

    pub fn insert_near(&mut self, one: Scalar, two: Scalar, pt1: Point, pt2: Point) -> i32 {
        self.f_nearly_same[if one == 1.0 { 1 } else { 0 }] = true;
        let index = self.insert(one, two, pt1);
        if index >= 0 {
            let idx = index as usize;
            self.f_pt2[idx] = pt2;
        }
        index
    }

    pub fn insert_coincident(&mut self, one: Scalar, two: Scalar, pt: Point) -> i32 {
        let index = self.insert(one, two, pt);
        if index >= 0 {
            self.set_coincident(index as usize);
        }
        index
    }

    pub fn merge(&mut self, a: &SkIntersections, a_index: usize, b: &SkIntersections, b_index: usize) {
        self.reset();
        self.f_t[0][0] = a.f_t[0][a_index];
        self.f_t[1][0] = b.f_t[0][b_index];
        self.f_pt[0] = a.f_pt[a_index];
        self.f_pt2[0] = b.f_pt[b_index];
        self.f_used = 1;
    }

    pub fn most_outside(&self, range_start: Scalar, range_end: Scalar, origin: Point) -> i32 {
        let mut result = -1;
        for index in 0..self.f_used as usize {
            if !between(range_start, self.f_t[0][index], range_end) {
                continue;
            }
            if result < 0 {
                result = index as i32;
                continue;
            }
            let best = self.f_pt[result as usize] - origin;
            let test = self.f_pt[index] - origin;
            if test.cross(best) < 0.0 {
                result = index as i32;
            }
        }
        result
    }

    /// Removes the intersection at `index`, shifting the rest down.
    ///
    /// Port of `SkIntersections::removeOne`. Note that the count drops even
    /// when the removed entry was the last one and nothing needs shifting -
    /// returning early without decrementing leaves the caller looping over an
    /// entry it just asked to have deleted.
    pub fn remove_one(&mut self, index: usize) {
        self.f_used -= 1;
        let remaining = self.f_used as usize - index;
        if remaining == 0 {
            return;
        }
        for i in 0..remaining {
            self.f_pt[index + i] = self.f_pt[index + i + 1];
            self.f_t[0][index + i] = self.f_t[0][index + i + 1];
            self.f_t[1][index + i] = self.f_t[1][index + i + 1];
        }
        // Shift the coincidence bits above `index` down by one, dropping the
        // bit that belonged to the removed entry.
        let keep_mask = !((1u16 << index) - 1);
        for side in 0..2 {
            let co_bit = self.f_is_coincident[side] & (1 << index);
            self.f_is_coincident[side] = self.f_is_coincident[side]
                .wrapping_sub(((self.f_is_coincident[side] >> 1) & keep_mask) + co_bit);
        }
    }
}


/// Returns 0 when `line` misses the horizontal at `y`, 2 when it lies along
/// it, and 1 otherwise.
///
/// Port of `horizontal_coincident`.
fn horizontal_coincident(line: &DLine, y: Scalar) -> u8 {
    let mut min = line.p[0].y;
    let mut max = line.p[1].y;
    if min > max {
        std::mem::swap(&mut min, &mut max);
    }
    if min > y || max < y {
        return 0;
    }
    if almost_equal_ulps(min, max) && max - min < (line.p[0].x - line.p[1].x).abs() {
        return 2;
    }
    1
}

/// Returns 0 when `line` misses the vertical at `x`, 2 when it lies along it,
/// and 1 otherwise.
///
/// Port of `vertical_coincident`.
fn vertical_coincident(line: &DLine, x: Scalar) -> u8 {
    let mut min = line.p[0].x;
    let mut max = line.p[1].x;
    if min > max {
        std::mem::swap(&mut min, &mut max);
    }
    if !precisely_between(min as f64, x as f64, max as f64) {
        return 0;
    }
    if almost_equal_ulps(min, max) {
        return 2;
    }
    1
}

impl Default for SkIntersections {
    fn default() -> Self {
        Self::new()
    }
}

/// Helper to check if a value is between two others
pub fn between(a: Scalar, b: Scalar, c: Scalar) -> bool {
    (a - b) * (c - b) <= 0.0
}

/// Check if two values are roughly equal
fn roughly_equal(a: Scalar, b: Scalar) -> bool {
    (a - b).abs() < 1e-6
}

/// Check if a scalar is zero or one
pub fn zero_or_one(t: Scalar) -> bool {
    t == 0.0 || t == 1.0
}

/// Linearly interpolate between a and b
pub fn interp(a: Scalar, b: Scalar, t: Scalar) -> Scalar {
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line_line_intersection() {
        let mut ts = SkIntersections::new();
        let a = [Point::new(0.0, 0.0), Point::new(2.0, 2.0)];
        let b = [Point::new(0.0, 2.0), Point::new(2.0, 0.0)];

        let pts = ts.line_line(&a, &b);
        assert_eq!(pts, 1);
        assert!((ts.pt(0).x - 1.0).abs() < 1e-6);
        assert!((ts.pt(0).y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_parallel_lines() {
        let mut ts = SkIntersections::new();
        let a = [Point::new(0.0, 0.0), Point::new(2.0, 0.0)];
        let b = [Point::new(0.0, 1.0), Point::new(2.0, 1.0)];

        let pts = ts.line_line(&a, &b);
        assert_eq!(pts, 0);
    }

    #[test]
    fn test_insert_and_used() {
        let mut ts = SkIntersections::new();
        ts.set_max(13);

        let _ = ts.insert(0.5, 0.5, Point::new(1.0, 1.0));
        assert_eq!(ts.used(), 1);

        let _ = ts.insert(0.25, 0.25, Point::new(0.5, 0.5));
        assert_eq!(ts.used(), 2);

        // Should be sorted by t value
        assert!(ts.t(0, 0) < ts.t(0, 1));
    }

    #[test]
    fn test_coincident_insert() {
        let mut ts = SkIntersections::new();
        ts.set_max(13);

        let idx = ts.insert_coincident(0.5, 0.5, Point::new(1.0, 1.0));
        assert!(idx >= 0);
        assert!(ts.is_coincident(idx as usize));
    }

    #[test]
    fn test_flip() {
        let mut ts = SkIntersections::new();
        ts.set_max(13);

        let _ = ts.insert(0.5, 0.25, Point::new(1.0, 1.0));
        ts.flip();

        assert!((ts.t(1, 0) - 0.75).abs() < 1e-6);
    }

    #[test]
    fn test_closest_to() {
        let mut ts = SkIntersections::new();
        ts.set_max(13);

        let _ = ts.insert(0.5, 0.5, Point::new(1.0, 1.0));
        let _ = ts.insert(0.75, 0.75, Point::new(2.0, 2.0));

        let test_pt = Point::new(1.1, 1.1);
        let mut closest_dist = 0.0;
        let closest = ts.closest_to(0.0, 1.0, test_pt, &mut closest_dist);

        assert_eq!(closest, 0);
        assert!(closest_dist < 1.0);
    }

    #[test]
    fn test_merg() {
        let mut a = SkIntersections::new();
        a.set_max(13);
        let _ = a.insert(0.5, 0.5, Point::new(1.0, 1.0));

        let mut b = SkIntersections::new();
        b.set_max(13);
        let _ = b.insert(0.5, 0.75, Point::new(1.0, 1.5));

        let mut ts = SkIntersections::new();
        ts.merge(&a, 0, &b, 0);

        // merge pairs the *first* curve's t from each side: a's t[0] becomes
        // the result's t[0], and b's t[0] becomes the result's t[1]. It does
        // not carry b's second t across.
        assert_eq!(ts.used(), 1);
        assert!((ts.t(0, 0) - 0.5).abs() < 1e-6);
        assert!((ts.t(1, 0) - 0.5).abs() < 1e-6);
        // The two points are kept separately rather than merged.
        assert_eq!(ts.pt(0), Point::new(1.0, 1.0));
    }

    #[test]
    fn test_remove_one() {
        let mut ts = SkIntersections::new();
        ts.set_max(13);

        let _ = ts.insert(0.25, 0.25, Point::new(0.5, 0.5));
        let _ = ts.insert(0.5, 0.5, Point::new(1.0, 1.0));
        let _ = ts.insert(0.75, 0.75, Point::new(1.5, 1.5));

        assert_eq!(ts.used(), 3);
        ts.remove_one(1);

        assert_eq!(ts.used(), 2);
        assert!((ts.t(0, 1) - 0.75).abs() < 1e-6);
    }

    #[test]
    fn remove_one_drops_the_last_entry() {
        // Removing the final entry shifts nothing, but the count must still
        // fall. Returning early here leaves the caller looking at an entry it
        // just deleted, which is what hung check_coincident.
        let mut ts = SkIntersections::new();
        ts.set_max(13);
        let _ = ts.insert(0.25, 0.25, Point::new(0.5, 0.5));
        let _ = ts.insert(0.75, 0.75, Point::new(1.5, 1.5));
        assert_eq!(ts.used(), 2);

        ts.remove_one(1);
        assert_eq!(ts.used(), 1);
        assert!((ts.t(0, 0) - 0.25).abs() < 1e-6);

        ts.remove_one(0);
        assert_eq!(ts.used(), 0);
    }

    #[test]
    fn remove_one_shifts_the_coincident_bits_down() {
        let mut ts = SkIntersections::new();
        ts.set_max(13);
        let _ = ts.insert(0.25, 0.25, Point::new(0.5, 0.5));
        let _ = ts.insert(0.5, 0.5, Point::new(1.0, 1.0));
        let _ = ts.insert(0.75, 0.75, Point::new(1.5, 1.5));
        // Mark the last one coincident, then drop the first.
        ts.set_coincident(2);
        assert!(ts.is_coincident(2));

        ts.remove_one(0);
        assert_eq!(ts.used(), 2);
        // The flag follows its entry down to index 1.
        assert!(ts.is_coincident(1));
        assert!(!ts.is_coincident(0));
    }

    #[test]
    fn test_line_horizontal() {
        let mut ts = SkIntersections::new();
        let pts = [Point::new(0.0, 0.0), Point::new(2.0, 2.0)];

        let count = ts.line_horizontal(&pts, 0.0, 2.0, 1.0, false);
        assert_eq!(count, 1);
        assert!((ts.pt(0).x - 1.0).abs() < 1e-6);
        assert!((ts.pt(0).y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_line_vertical() {
        let mut ts = SkIntersections::new();
        let pts = [Point::new(0.0, 0.0), Point::new(2.0, 2.0)];

        let count = ts.line_vertical(&pts, 0.0, 2.0, 1.0, false);
        assert_eq!(count, 1);
        assert!((ts.pt(0).x - 1.0).abs() < 1e-6);
        assert!((ts.pt(0).y - 1.0).abs() < 1e-6);
    }

    // --- item 12: line/line intersection ---------------------------------

    fn dline(x0: f32, y0: f32, x1: f32, y1: f32) -> DLine {
        DLine::new(Point::new(x0, y0), Point::new(x1, y1))
    }

    #[test]
    fn crossing_lines_meet_once_in_the_middle() {
        let mut ts = SkIntersections::new();
        let a = dline(0.0, 0.0, 10.0, 10.0);
        let b = dline(0.0, 10.0, 10.0, 0.0);
        assert_eq!(ts.intersect(&a, &b), 1);
        assert!((ts.t(0, 0) - 0.5).abs() < 1e-5);
        assert!((ts.t(1, 0) - 0.5).abs() < 1e-5);
        assert!((ts.pt(0).x - 5.0).abs() < 1e-4);
        assert!((ts.pt(0).y - 5.0).abs() < 1e-4);
    }

    #[test]
    fn parallel_lines_never_meet() {
        let mut ts = SkIntersections::new();
        let a = dline(0.0, 0.0, 10.0, 10.0);
        let b = dline(0.0, 5.0, 10.0, 15.0);
        assert_eq!(ts.intersect(&a, &b), 0);
    }

    #[test]
    fn collinear_overlapping_lines_report_both_ends() {
        // b starts inside a and runs past its end.
        let mut ts = SkIntersections::new();
        let a = dline(0.0, 0.0, 10.0, 10.0);
        let b = dline(5.0, 5.0, 15.0, 15.0);
        let count = ts.intersect(&a, &b);
        assert_eq!(count, 2, "an overlapping run has two ends");
        // The overlap is marked coincident, which is how callers tell it apart
        // from two ordinary crossings.
        assert!(ts.is_coincident(0));
        assert!(ts.is_coincident(1));
    }

    #[test]
    fn collinear_disjoint_lines_do_not_meet() {
        let mut ts = SkIntersections::new();
        let a = dline(0.0, 0.0, 4.0, 4.0);
        let b = dline(10.0, 10.0, 14.0, 14.0);
        assert_eq!(ts.intersect(&a, &b), 0);
    }

    #[test]
    fn a_shared_endpoint_registers_once() {
        // Two adjacent segments of a contour meeting at a vertex. This is the
        // case the exact-endpoint handling exists for: without it the shared
        // corner reads as two near-misses instead of one meeting.
        let mut ts = SkIntersections::new();
        let a = dline(0.0, 0.0, 10.0, 0.0);
        let b = dline(10.0, 0.0, 10.0, 10.0);
        let count = ts.intersect(&a, &b);
        assert_eq!(count, 1);
        assert!((ts.t(0, 0) - 1.0).abs() < 1e-5, "end of a");
        assert!((ts.t(1, 0) - 0.0).abs() < 1e-5, "start of b");
    }

    #[test]
    fn intersect_ray_finds_crossings_outside_the_segments() {
        // The segments stop short of each other, but the rays through them
        // cross. intersect() would find nothing here.
        let mut ts = SkIntersections::new();
        let a = dline(0.0, 0.0, 1.0, 1.0);
        let b = dline(10.0, 0.0, 9.0, 1.0);
        assert_eq!(ts.intersect_ray(&a, &b), 1);
        assert!(ts.t(0, 0) > 1.0, "crossing lies past the end of a");

        let mut seg = SkIntersections::new();
        assert_eq!(seg.intersect(&a, &b), 0);
    }

    #[test]
    fn intersect_ray_reports_the_span_for_coincident_rays() {
        let mut ts = SkIntersections::new();
        let a = dline(0.0, 0.0, 10.0, 10.0);
        let b = dline(2.0, 2.0, 5.0, 5.0);
        assert_eq!(ts.intersect_ray(&a, &b), 2);
        assert!((ts.t(0, 0) - 0.0).abs() < 1e-6);
        assert!((ts.t(0, 1) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn intersect_ray_misses_parallel_rays_that_do_not_coincide() {
        let mut ts = SkIntersections::new();
        let a = dline(0.0, 0.0, 10.0, 10.0);
        let b = dline(0.0, 5.0, 10.0, 15.0);
        assert_eq!(ts.intersect_ray(&a, &b), 0);
    }

    #[test]
    fn horizontal_span_crosses_a_line_once() {
        let mut ts = SkIntersections::new();
        // A vertical-ish line crossing y = 5 between x = 0 and x = 10.
        let line = dline(5.0, 0.0, 5.0, 10.0);
        assert_eq!(ts.horizontal(&line, 0.0, 10.0, 5.0, false), 1);
        assert!((ts.t(0, 0) - 0.5).abs() < 1e-5);
        assert!((ts.pt(0).x - 5.0).abs() < 1e-4);
        assert!((ts.pt(0).y - 5.0).abs() < 1e-4);
    }

    #[test]
    fn horizontal_flipped_reverses_the_span_parameter() {
        let mut plain = SkIntersections::new();
        let line = dline(2.5, 0.0, 2.5, 10.0);
        assert_eq!(plain.horizontal(&line, 0.0, 10.0, 5.0, false), 1);
        let straight = plain.t(1, 0);

        let mut flipped = SkIntersections::new();
        assert_eq!(flipped.horizontal(&line, 0.0, 10.0, 5.0, true), 1);
        assert!((flipped.t(1, 0) - (1.0 - straight)).abs() < 1e-5);
    }

    #[test]
    fn horizontal_span_misses_a_line_outside_it() {
        let mut ts = SkIntersections::new();
        let line = dline(50.0, 0.0, 50.0, 10.0);
        assert_eq!(ts.horizontal(&line, 0.0, 10.0, 5.0, false), 0);
    }

    #[test]
    fn vertical_span_crosses_a_line_once() {
        let mut ts = SkIntersections::new();
        let line = dline(0.0, 5.0, 10.0, 5.0);
        assert_eq!(ts.vertical(&line, 0.0, 10.0, 5.0, false), 1);
        assert!((ts.t(0, 0) - 0.5).abs() < 1e-5);
        assert!((ts.pt(0).x - 5.0).abs() < 1e-4);
        assert!((ts.pt(0).y - 5.0).abs() < 1e-4);
    }

    #[test]
    fn vertical_flipped_reverses_the_span_parameter() {
        let mut plain = SkIntersections::new();
        let line = dline(0.0, 2.5, 10.0, 2.5);
        assert_eq!(plain.vertical(&line, 0.0, 10.0, 5.0, false), 1);
        let straight = plain.t(1, 0);

        let mut flipped = SkIntersections::new();
        assert_eq!(flipped.vertical(&line, 0.0, 10.0, 5.0, true), 1);
        assert!((flipped.t(1, 0) - (1.0 - straight)).abs() < 1e-5);
    }

    #[test]
    fn intercept_helpers_agree_with_the_full_routines() {
        let line = dline(0.0, 0.0, 10.0, 20.0);
        // Crossing y = 10 happens halfway along.
        assert!((SkIntersections::horizontal_intercept(&line, 10.0) - 0.5).abs() < 1e-6);
        // Crossing x = 5 also happens halfway.
        assert!((SkIntersections::vertical_intercept(&line, 5.0) - 0.5).abs() < 1e-6);
        // Out of range values pin to the ends rather than running off.
        assert_eq!(SkIntersections::horizontal_intercept(&line, -100.0), 0.0);
        assert_eq!(SkIntersections::horizontal_intercept(&line, 100.0), 1.0);
    }
}
