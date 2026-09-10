//! Intersection detection between path segments
//!
//! Port of Skia's SkIntersections.{h,cpp}

use crate::core::{Point, Scalar};

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

    pub fn line_line(&mut self, a: &[Point; 2], b: &[Point; 2]) -> u8 {
        self.f_max = 2;
        self.reset();

        let dx1 = a[1].x - a[0].x;
        let dy1 = a[1].y - a[0].y;
        let dx2 = b[1].x - b[0].x;
        let dy2 = b[1].y - b[0].y;

        let det = dx1 * dy2 - dy1 * dx2;

        if det.abs() < 1e-10 {
            // Parallel lines
            return 0;
        }

        let t1 = ((b[0].x - a[0].x) * dy2 - (b[0].y - a[0].y) * dx2) / det;
        let t2 = ((b[0].x - a[0].x) * dy1 - (b[0].y - a[0].y) * dx1) / det;

        if (0.0..=1.0).contains(&t1) && (0.0..=1.0).contains(&t2) {
            let pt = Point::new(
                a[0].x + t1 * dx1,
                a[0].y + t1 * dy1,
            );
            let _ = self.insert(t1, t2, pt);
            1
        } else {
            0
        }
    }

    pub fn line_horizontal(&mut self, pts: &[Point; 2], left: Scalar, right: Scalar, y: Scalar, flipped: bool) -> u8 {
        self.f_max = 2;
        self.reset();

        // Find intersection of line with horizontal line y
        // Line from pts[0] to pts[1]
        let dx = pts[1].x - pts[0].x;
        let dy = pts[1].y - pts[0].y;

        if dy.abs() < 1e-10 {
            // Horizontal line - check for overlap
            return 0;
        }

        // t for y coordinate
        let t = (y - pts[0].y) / dy;

        if (0.0..=1.0).contains(&t) {
            let x = pts[0].x + t * dx;
            if (left..=right).contains(&x) {
                let pt = Point::new(x, y);
                let t_insert = if flipped { 1.0 - t } else { t };
                let _ = self.insert(t_insert, t, pt);
                return 1;
            }
        }

        0
    }

    pub fn line_vertical(&mut self, pts: &[Point; 2], top: Scalar, bottom: Scalar, x: Scalar, flipped: bool) -> u8 {
        self.f_max = 2;
        self.reset();

        let dx = pts[1].x - pts[0].x;
        let dy = pts[1].y - pts[0].y;

        if dx.abs() < 1e-10 {
            // Vertical line - check for overlap
            return 0;
        }

        let t = (x - pts[0].x) / dx;

        if (0.0..=1.0).contains(&t) {
            let y = pts[0].y + t * dy;
            if (top..=bottom).contains(&y) {
                let pt = Point::new(x, y);
                let t_insert = if flipped { 1.0 - t } else { t };
                let _ = self.insert(t_insert, t, pt);
                return 1;
            }
        }

        0
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
}
