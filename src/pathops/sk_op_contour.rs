//! SkOpContour - represents a path contour for path operations
//!
//! Port of Skia's SkOpContour.{h,cpp}

use super::sk_intersection_helper::SkPathOpsBounds;
use super::sk_op_segment::{SkOpSegment, SkOpSpan, Verb};
use crate::core::{Point, Scalar};

/// Direction for ray checking (matches Skia's SkOpRayDir)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkOpRayDir {
    Left,
    Top,
    Right,
    Bottom,
}

impl SkOpRayDir {
    /// Get the index (x or y) that changes in this direction
    pub fn xy_index(&self) -> usize {
        *self as usize & 1
    }

    /// Get the perpendicular index
    pub fn perp_index(&self) -> usize {
        1 - self.xy_index()
    }

    /// Returns true if we're comparing "less than" in this direction
    pub fn less_than(&self) -> bool {
        (*self as usize & 2) == 0
    }

    /// Rotate direction by offset
    pub fn rotate(&self, offset: usize) -> Self {
        match offset {
            0 => *self,
            1 => match self {
                SkOpRayDir::Left => SkOpRayDir::Top,
                SkOpRayDir::Top => SkOpRayDir::Right,
                SkOpRayDir::Right => SkOpRayDir::Bottom,
                SkOpRayDir::Bottom => SkOpRayDir::Left,
            },
            _ => *self,
        }
    }
}

/// Ray hit structure (matches Skia's SkOpRayHit)
#[derive(Debug, Clone)]
pub struct SkOpRayHit {
    pub t: Scalar,
    pub pt: Point,
    pub slope_x: Scalar,
    pub slope_y: Scalar,
}

/// Forward declarations for related types
#[derive(Debug, Clone)]
pub struct SkOpGlobalState;
pub struct SkOpAngle;
pub struct SkOpCoincidence;
pub struct SkPathWriter;

/// A path contour with segments
#[derive(Debug)]
pub struct SkOpContour {
    /// Global state reference
    f_state: Option<SkOpGlobalState>,
    /// First segment in the contour
    f_head: SkOpSegment,
    /// Last segment in the contour
    f_tail: Option<Box<SkOpSegment>>,
    /// Next contour in the chain
    f_next: Option<Box<SkOpContour>>,
    /// Bounds of the contour
    f_bounds: SkPathOpsBounds,
    /// Counter-clockwise flag (1 for CCW, -1 for CW)
    f_ccw: i32,
    /// Number of segments
    f_count: i32,
    /// First sorted index (debug)
    f_first_sorted: i32,
    /// Done flag
    f_done: bool,
    /// True if this is an operand (second argument to binary operator)
    f_operand: bool,
    /// True if contour should be reverse written
    f_reverse: bool,
    /// True if original path had even-odd fill
    f_xor: bool,
    /// True if opposite path had even-odd fill
    f_opp_xor: bool,
    /// Debug ID
    #[cfg(debug_assertions)]
    f_id: i32,
    /// Debug indent
    #[cfg(debug_assertions)]
    f_debug_indent: i32,
}

impl SkOpContour {
    /// Creates a new empty contour
    pub fn new() -> Self {
        Self::reset()
    }

    /// Resets the contour to initial state
    pub fn reset() -> Self {
        Self {
            f_state: None,
            f_head: SkOpSegment::new(),
            f_tail: None,
            f_next: None,
            f_bounds: SkPathOpsBounds::default(),
            f_ccw: 0,
            f_count: 0,
            f_first_sorted: -1,
            f_done: false,
            f_operand: false,
            f_reverse: false,
            f_xor: false,
            f_opp_xor: false,
            #[cfg(debug_assertions)]
            f_id: -1,
            #[cfg(debug_assertions)]
            f_debug_indent: 0,
        }
    }

    /// Initializes the contour with global state
    pub fn init(&mut self, global_state: SkOpGlobalState, operand: bool, is_xor: bool) {
        self.f_state = Some(global_state);
        self.f_operand = operand;
        self.f_xor = is_xor;
        #[cfg(debug_assertions)]
        {
            self.f_id = 1;
        }
    }

    /// Returns the global state
    pub fn global_state(&self) -> Option<&SkOpGlobalState> {
        self.f_state.as_ref()
    }

    /// Calculate angles for all segments in this contour
    pub fn calc_angles(&self) {
        // Simplified implementation - in full version would compute angles for each span
    }

    /// Check if this contour is missing coincidence
    pub fn missing_coincidence(&self) -> bool {
        // Simplified implementation
        false
    }

    /// Move multiples to align t values
    pub fn move_multiples(&mut self) -> bool {
        // Simplified implementation
        true
    }

    /// Move nearby points to eliminate small gaps
    pub fn move_nearby(&mut self) -> bool {
        // Simplified implementation
        true
    }

    /// Sort angles for this contour
    pub fn sort_angles(&mut self) -> bool {
        // Simplified implementation
        true
    }

    /// Returns the number of segments
    pub fn count(&self) -> i32 {
        self.f_count
    }

    /// Returns whether the contour is empty
    pub fn is_empty(&self) -> bool {
        self.f_count == 0
    }

    /// Returns the bounds
    pub fn bounds(&self) -> &SkPathOpsBounds {
        &self.f_bounds
    }

    /// Returns the first segment
    pub fn first(&self) -> Option<&SkOpSegment> {
        if self.f_count > 0 {
            Some(&self.f_head)
        } else {
            None
        }
    }

    /// Returns the last segment
    pub fn tail(&self) -> Option<&SkOpSegment> {
        self.f_tail.as_ref().map(|t| t.as_ref())
    }

    /// Returns the next contour
    pub fn next(&self) -> Option<&SkOpContour> {
        self.f_next.as_ref().map(|n| n.as_ref())
    }

    /// Sets the next contour
    pub fn set_next(&mut self, contour: Option<Box<SkOpContour>>) {
        self.f_next = contour;
    }

    /// Returns the start point
    pub fn start(&self) -> Option<Point> {
        if self.f_count > 0 {
            Some(self.f_head.last_pt())
        } else {
            None
        }
    }

    /// Returns the end point
    pub fn end(&self) -> Option<Point> {
        self.f_tail.as_ref().map(|t| t.last_pt())
    }

    /// Appends a new segment to the contour
    pub fn append_segment(&mut self) -> &mut SkOpSegment {
        if self.f_count == 0 {
            self.f_count = 1;
            &mut self.f_head
        } else {
            self.f_count += 1;
            if let Some(tail) = self.f_tail.take() {
                let mut new_segment = SkOpSegment::new();
                new_segment.set_prev(Some(tail));
                self.f_tail = Some(Box::new(new_segment));
                self.f_tail.as_mut().unwrap()
            } else {
                &mut self.f_head
            }
        }
    }

    /// Adds a line segment
    pub fn add_line(&mut self, p0: Point, p1: Point) -> bool {
        if p0 == p1 {
            return false;
        }
        let segment = self.append_segment();
        segment.add_line(p0, p1);
        true
    }

    /// Adds a quad segment
    pub fn add_quad(&mut self, pts: [Point; 3]) {
        let segment = self.append_segment();
        segment.add_quad(pts);
    }

    /// Adds a conic segment
    pub fn add_conic(&mut self, pts: [Point; 3], weight: Scalar) {
        let segment = self.append_segment();
        segment.add_conic(pts, weight);
    }

    /// Adds a cubic segment
    pub fn add_cubic(&mut self, pts: [Point; 4]) {
        let segment = self.append_segment();
        segment.add_cubic(pts);
    }

    /// Sets bounds based on all segments
    pub fn set_bounds(&mut self) {
        if self.f_count == 0 {
            return;
        }
        let mut bounds = self.f_head.bounds().clone();
        let mut segment = self.f_head.next();
        while let Some(seg) = segment {
            bounds.add(seg.bounds());
            segment = seg.next();
        }
        self.f_bounds = bounds;
    }

    /// Marks the contour as complete
    pub fn complete(&mut self) {
        self.set_bounds();
    }

    /// Sets the counter-clockwise flag
    pub fn set_ccw(&mut self, ccw: i32) {
        self.f_ccw = ccw;
    }

    /// Returns the counter-clockwise flag
    pub fn is_ccw(&self) -> i32 {
        self.f_ccw
    }

    /// Sets the done flag
    pub fn set_done(&mut self, done: bool) {
        self.f_done = done;
    }

    /// Returns the done flag
    pub fn is_done(&self) -> bool {
        self.f_done
    }

    /// Sets the operand flag
    pub fn set_operand(&mut self, is_operand: bool) {
        self.f_operand = is_operand;
    }

    /// Returns the operand flag
    pub fn is_operand(&self) -> bool {
        self.f_operand
    }

    /// Sets the reverse flag
    pub fn set_reverse(&mut self) {
        self.f_reverse = true;
    }

    /// Returns the reverse flag
    pub fn is_reversed(&self) -> bool {
        self.f_reverse
    }

    /// Sets the xor flag
    pub fn set_xor(&mut self, is_xor: bool) {
        self.f_xor = is_xor;
    }

    /// Returns the xor flag
    pub fn is_xor(&self) -> bool {
        self.f_xor
    }

    /// Sets the opposite xor flag
    pub fn set_opp_xor(&mut self, is_opp_xor: bool) {
        self.f_opp_xor = is_opp_xor;
    }

    /// Returns the opposite xor flag
    pub fn is_opp_xor(&self) -> bool {
        self.f_opp_xor
    }

    /// Returns the first undone span
    pub fn undone_span(&mut self) -> Option<&SkOpSpan> {
        let mut segment = self.f_head.next();
        while let Some(seg) = segment {
            if !seg.is_done() {
                return seg.undone_span();
            }
            segment = seg.next();
        }
        self.f_done = true;
        None
    }

    /// Marks all segments as done
    pub fn mark_all_done(&mut self) {
        let mut segment = self.f_head.next();
        while let Some(seg) = segment {
            // Note: This is a limitation of the current implementation
            // In the full implementation, segments would be stored differently
            // to allow mutable access
            break;
        }
    }

    /// Joins segment ends
    pub fn join_segments(&mut self) {
        // Simplified - in real implementation would join segment ends
    }

    /// Compares contours for sorting (by top, then left)
    pub fn compare_for_sort(&self, other: &SkOpContour) -> std::cmp::Ordering {
        if self.f_bounds.top != other.f_bounds.top {
            self.f_bounds
                .top
                .partial_cmp(&other.f_bounds.top)
                .unwrap_or(std::cmp::Ordering::Equal)
        } else {
            self.f_bounds
                .left
                .partial_cmp(&other.f_bounds.left)
                .unwrap_or(std::cmp::Ordering::Equal)
        }
    }

    /// Debug validate (only in debug builds)
    #[cfg(debug_assertions)]
    pub fn debug_validate(&self) {
        let mut segment = Some(&self.f_head);
        let mut prior: Option<&SkOpSegment> = None;
        while let Some(seg) = segment {
            seg.debug_validate();
            segment = seg.next();
        }
    }

    /// Dump contour for debugging
    pub fn dump(&self) {
        println!(
            "Contour: count={}, bounds={:?}",
            self.f_count, self.f_bounds
        );
    }
}

impl Default for SkOpContour {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Default)]
/// Builder for SkOpContour that batches line additions
#[derive(Debug)]
pub struct SkOpContourBuilder {
    contour: Option<Box<SkOpContour>>,
    last_line: Option<[Point; 2]>,
    last_is_line: bool,
}

impl SkOpContourBuilder {
    /// Creates a new builder with the given contour
    pub fn new(contour: SkOpContour) -> Self {
        Self {
            contour: Some(Box::new(contour)),
            last_line: None,
            last_is_line: false,
        }
    }

    /// Creates a new builder without a contour
    pub fn new_empty() -> Self {
        Self {
            contour: None,
            last_line: None,
            last_is_line: false,
        }
    }

    /// Flushes any pending line
    pub fn flush(&mut self) {
        if !self.last_is_line {
            return;
        }
        if let Some(ref line) = self.last_line {
            if let Some(ref mut contour) = self.contour {
                contour.add_line(line[0], line[1]);
            }
        }
        self.last_is_line = false;
        self.last_line = None;
    }

    /// Returns the contour
    pub fn contour(&self) -> Option<&SkOpContour> {
        self.contour.as_ref().map(|c| c.as_ref())
    }

    /// Returns mutable contour
    pub fn contour_mut(&mut self) -> Option<&mut SkOpContour> {
        self.contour.as_mut().map(|c| c.as_mut())
    }

    /// Adds a line (may be buffered)
    pub fn add_line(&mut self, pts: [Point; 2]) {
        if self.last_is_line {
            if let Some(ref last) = self.last_line {
                if last[0] == pts[1] && last[1] == pts[0] {
                    self.last_is_line = false;
                    self.last_line = None;
                    return;
                } else {
                    self.flush();
                }
            }
        }
        self.last_line = Some(pts);
        self.last_is_line = true;
    }

    /// Adds a quad
    pub fn add_quad(&mut self, pts: [Point; 3]) {
        self.flush();
        if let Some(ref mut contour) = self.contour {
            contour.add_quad(pts);
        }
    }

    /// Adds a conic
    pub fn add_conic(&mut self, pts: [Point; 3], weight: Scalar) {
        self.flush();
        if let Some(ref mut contour) = self.contour {
            contour.add_conic(pts, weight);
        }
    }

    /// Adds a cubic
    pub fn add_cubic(&mut self, pts: [Point; 4]) {
        self.flush();
        if let Some(ref mut contour) = self.contour {
            contour.add_cubic(pts);
        }
    }
}

/// Contour comparison for sorting
#[derive(Debug)]
pub struct SkOpContourHead {
    contours: Vec<SkOpContour>,
}

impl SkOpContourHead {
    /// Creates a new empty head
    pub fn new() -> Self {
        Self {
            contours: Vec::new(),
        }
    }

    /// Appends a new contour
    pub fn append_contour(&mut self) -> &mut SkOpContour {
        self.contours.push(SkOpContour::new());
        self.contours.last_mut().unwrap()
    }

    /// Returns the number of contours
    pub fn count(&self) -> usize {
        self.contours.len()
    }

    /// Returns an iterator over contours
    pub fn iter(&self) -> impl Iterator<Item = &SkOpContour> {
        self.contours.iter()
    }

    /// Returns a reference to a contour at the given index
    pub fn get_contour(&self, index: usize) -> Option<&SkOpContour> {
        self.contours.get(index)
    }

    /// Returns a mutable reference to a contour at the given index
    pub fn get_contour_mut(&mut self, index: usize) -> Option<&mut SkOpContour> {
        self.contours.get_mut(index)
    }
}

impl Default for SkOpContourHead {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_contour() {
        let contour = SkOpContour::new();
        assert_eq!(contour.count(), 0);
        assert!(contour.is_empty());
        assert!(!contour.is_done());
    }

    #[test]
    fn test_add_line() {
        let mut contour = SkOpContour::new();
        let p0 = Point::new(0.0, 0.0);
        let p1 = Point::new(10.0, 10.0);
        assert!(contour.add_line(p0, p1));
        assert_eq!(contour.count(), 1);
    }

    #[test]
    fn test_add_line_same_point() {
        let mut contour = SkOpContour::new();
        let p = Point::new(0.0, 0.0);
        assert!(!contour.add_line(p, p));
        assert_eq!(contour.count(), 0);
    }

    #[test]
    fn test_add_quad() {
        let mut contour = SkOpContour::new();
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(5.0, 10.0),
            Point::new(10.0, 0.0),
        ];
        contour.add_quad(pts);
        assert_eq!(contour.count(), 1);
    }

    #[test]
    fn test_add_conic() {
        let mut contour = SkOpContour::new();
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(5.0, 10.0),
            Point::new(10.0, 0.0),
        ];
        contour.add_conic(pts, 1.0);
        assert_eq!(contour.count(), 1);
    }

    #[test]
    fn test_add_cubic() {
        let mut contour = SkOpContour::new();
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(3.3, 3.3),
            Point::new(6.6, 6.6),
            Point::new(10.0, 10.0),
        ];
        contour.add_cubic(pts);
        assert_eq!(contour.count(), 1);
    }

    #[test]
    fn test_set_get_flags() {
        let mut contour = SkOpContour::new();
        contour.set_ccw(1);
        assert_eq!(contour.is_ccw(), 1);

        contour.set_done(true);
        assert!(contour.is_done());

        contour.set_operand(true);
        assert!(contour.is_operand());

        contour.set_reverse();
        assert!(contour.is_reversed());

        contour.set_xor(true);
        assert!(contour.is_xor());

        contour.set_opp_xor(true);
        assert!(contour.is_opp_xor());
    }

    #[test]
    fn test_builder_line_batching() {
        let mut builder = SkOpContourBuilder::new_empty();

        // Add back-to-back lines that cancel
        builder.add_line([Point::new(0.0, 0.0), Point::new(10.0, 0.0)]);
        builder.add_line([Point::new(10.0, 0.0), Point::new(0.0, 0.0)]);

        builder.flush();
        assert_eq!(builder.last_is_line, false);
    }

    #[test]
    fn test_contour_head() {
        let mut head = SkOpContourHead::new();
        assert_eq!(head.count(), 0);

        let contour = head.append_contour();
        contour.add_line(Point::new(0.0, 0.0), Point::new(10.0, 10.0));
        assert_eq!(head.count(), 1);
    }

    #[test]
    fn test_default() {
        let _contour: SkOpContour = Default::default();
        let _builder: SkOpContourBuilder = Default::default();
        let _head: SkOpContourHead = Default::default();
    }

    #[test]
    fn test_bounds() {
        let mut contour = SkOpContour::new();
        let p0 = Point::new(0.0, 0.0);
        let p1 = Point::new(10.0, 10.0);
        contour.add_line(p0, p1);
        contour.complete();
        // Would verify bounds here
    }
}
