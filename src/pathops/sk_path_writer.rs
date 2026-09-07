//! SkPathWriter - manages path writing with deferred line optimization
//!
//! Port of Skia's SkPathWriter.{h,cpp}

use crate::core::{Path, Point};

/// Wraps a Path to keep track of whether the contour is initialized and non-empty
pub struct SkPathWriter<'a> {
    path_ptr: &'a mut Path,
    current: Path,
    first_pt: Option<Point>,
    defer: [Option<Point>; 2],
    end_pts: Vec<Point>,
    partials: Vec<Path>,
}

impl<'a> SkPathWriter<'a> {
    /// Creates a new SkPathWriter wrapping the given path
    pub fn new(path: &'a mut Path) -> Self {
        Self {
            path_ptr: path,
            current: Path::new(),
            first_pt: None,
            defer: [None, None],
            end_pts: Vec::new(),
            partials: Vec::new(),
        }
    }

    /// Closes the current contour and adds it to the wrapped path
    pub fn close(&mut self) {
        if self.current.is_empty() {
            return;
        }
        self.current.close();
        // Copy current to a temp path before rewinding
        let temp = self.current.clone();
        self.current.rewind();
        self.init();
        // Copy temp to path_ptr
        for i in 0..temp.count_verbs() {
            if let Some(verb) = temp.verb(i) {
                self.append_verb(verb, &temp, i);
            }
        }
    }

    /// Adds a conic segment
    pub fn conic_to(&mut self, pt1: Point, pt2: Point, weight: f32) {
        self.current.conic_to(pt1.x, pt1.y, pt2.x, pt2.y, weight);
    }

    /// Adds a cubic segment
    pub fn cubic_to(&mut self, pt1: Point, pt2: Point, pt3: Point) {
        self.current.cubic_to(pt1.x, pt1.y, pt2.x, pt2.y, pt3.x, pt3.y);
    }

    /// Defers a line point, returns false if the point is degenerate
    pub fn deferred_line(&mut self, pt: Point) -> bool {
        if self.defer[0].is_none() {
            return true;
        }

        if self.defer[0] == Some(pt) {
            // Degenerate line - caller should have preflighted
            return true;
        }

        if self.pt_contains(self.defer[0].unwrap(), pt) {
            // Degenerate line
            return true;
        }

        if self.matched_last(pt) {
            return false;
        }

        if self.defer[1].is_some() && self.changed_slopes(pt) {
            self.line_to();
            self.defer[0] = self.defer[1];
        }
        self.defer[1] = Some(pt);
        true
    }

    /// Defers a move point
    pub fn deferred_move(&mut self, pt: Point) {
        if self.defer[1].is_none() {
            self.first_pt = Some(pt);
            self.defer[0] = Some(pt);
            return;
        }

        if !self.matched_last(pt) {
            self.finish_contour();
            self.first_pt = Some(pt);
            self.defer[0] = Some(pt);
        }
    }

    /// Finishes the current contour, potentially storing it as a partial
    pub fn finish_contour(&mut self) {
        if !self.matched_last(self.defer[0].unwrap_or_default()) {
            if self.defer[1].is_none() {
                return;
            }
            self.line_to();
        }

        if self.current.is_empty() {
            return;
        }

        if self.is_closed() {
            self.close();
        } else {
            self.end_pts.push(self.first_pt.unwrap());
            self.end_pts.push(self.defer[1].unwrap());
            self.partials.push(self.current.clone());
            self.init();
        }
    }

    /// Initializes the writer state
    fn init(&mut self) {
        self.current.rewind();
        self.first_pt = None;
        self.defer[0] = None;
        self.defer[1] = None;
    }

    /// Returns true if the last point matches the first point
    pub fn is_closed(&self) -> bool {
        self.first_pt.map_or(false, |first| {
            self.defer[1].map_or(false, |last| self.points_equal(first, last))
        })
    }

    /// Adds a line segment
    fn line_to(&mut self) {
        if self.current.is_empty() {
            self.move_to();
        }

        if let Some(pt) = self.defer[1] {
            self.current.line_to(pt.x, pt.y);
        }
    }

    /// Returns true if the test point matches the deferred point
    fn matched_last(&self, test: Point) -> bool {
        if Some(test) == self.defer[1] {
            return true;
        }

        if test == Point::new(0.0, 0.0) || self.defer[1].is_none() {
            return false;
        }

        self.defer[1].map_or(false, |dp| self.pt_contains(dp, test))
    }

    /// Adds a move to the current path
    fn move_to(&mut self) {
        if let Some(pt) = self.first_pt {
            self.current.move_to(pt.x, pt.y);
        }
    }

    /// Adds a quad segment
    pub fn quad_to(&mut self, pt1: Point, pt2: Point) {
        self.current.quad_to(pt1.x, pt1.y, pt2.x, pt2.y);
    }

    /// Updates the current point, handling deferred lines and closing
    pub fn update(&mut self, pt: Point) -> Point {
        if self.defer[1].is_none() {
            self.move_to();
        } else if !self.matched_last(self.defer[0].unwrap_or_default()) {
            self.line_to();
        }

        let mut result = pt;

        if let Some(first) = self.first_pt {
            if !self.points_equal(result, first) && self.pt_contains(first, pt) {
                result = first;
            }
        }

        self.defer[0] = Some(pt);
        self.defer[1] = Some(pt);
        result
    }

    /// Returns true if there are partial contours that need assembly
    pub fn some_assembly_required(&mut self) -> bool {
        self.finish_contour();
        !self.end_pts.is_empty()
    }

    /// Returns true if the slope changes at this point
    fn changed_slopes(&self, pt: Point) -> bool {
        if self.matched_last(self.defer[0].unwrap_or_default()) {
            return false;
        }

        let defer = self.defer[0].unwrap();
        let defer_pt = self.defer[1].unwrap();
        
        let defer_dx = defer_pt.x - defer.x;
        let defer_dy = defer_pt.y - defer.y;
        let line_dx = pt.x - defer_pt.x;
        let line_dy = pt.y - defer_pt.y;

        defer_dx * line_dy != defer_dy * line_dx
    }

    /// Returns true if p1 contains p2 (roughly equal)
    fn pt_contains(&self, p1: Point, p2: Point) -> bool {
        self.points_equal(p1, p2)
    }

    /// Returns true if two points are equal
    fn points_equal(&self, p1: Point, p2: Point) -> bool {
        (p1.x - p2.x).abs() <= f32::EPSILON && (p1.y - p2.y).abs() <= f32::EPSILON
    }

    /// Assembles partial contours by connecting closest endpoints
    pub fn assemble(&mut self) {
        if !self.some_assembly_required() {
            return;
        }

        let end_count = self.end_pts.len();
        if end_count == 0 {
            return;
        }

        // Extend partial contours adjacent to simple segments
        for _p_idx in 0..end_count {
            // Simplified: in the full implementation, this would extend contours
            // based on segment simplicity and t values
        }

        // Build distance matrix between endpoints
        let link_count = end_count / 2;
        let mut s_link: Vec<Option<usize>> = vec![None; link_count];
        let mut e_link: Vec<Option<usize>> = vec![None; link_count];

        let entries = end_count * (end_count - 1) / 2;
        let mut distances: Vec<f32> = Vec::with_capacity(entries);
        let mut sorted_dist: Vec<usize> = Vec::with_capacity(entries);

        let mut d_idx = 0;
        for r_idx in 0..end_count - 1 {
            let p1 = self.end_pts[r_idx];
            for i_idx in (r_idx + 1)..end_count {
                let p2 = self.end_pts[i_idx];
                let dx = p2.x - p1.x;
                let dy = p2.y - p1.y;
                let dist = dx * dx + dy * dy;
                distances.push(dist);
                sorted_dist.push(d_idx);
                d_idx += 1;
            }
        }

        // Sort by distance
        sorted_dist.sort_by(|a, b| {
            distances[*a]
                .partial_cmp(&distances[*b])
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Connect closest endpoints
        let mut remaining = link_count;
        for pair in sorted_dist {
            let row = pair / end_count;
            let col = pair - row * end_count;
            let ndx_one = row / 2;
            let end_one = row % 2 == 1;
            let ndx_two = col / 2;
            let end_two = col % 2 == 1;

            if s_link[ndx_one].is_some() || e_link[ndx_one].is_some() {
                continue;
            }
            if s_link[ndx_two].is_some() || e_link[ndx_two].is_some() {
                continue;
            }

            let flip = end_one == end_two;
            if end_one {
                e_link[ndx_one] = Some(if flip { !ndx_two } else { ndx_two });
            } else {
                s_link[ndx_one] = Some(if flip { !ndx_two } else { ndx_two });
            }

            if end_two {
                e_link[ndx_two] = Some(if flip { !ndx_one } else { ndx_one });
            } else {
                s_link[ndx_two] = Some(if flip { !ndx_one } else { ndx_one });
            }

            remaining -= 1;
            if remaining == 0 {
                break;
            }
        }

            // Build final path from linked contours
            let mut r_idx = 0;
            while r_idx < link_count {
                let forward = true;
                let mut first = true;

            let s_idx = match s_link[r_idx].take() {
                Some(v) => v,
                None => break,
            };

            let e_idx = if s_idx >= 0 {
                e_link[s_idx].take()
            } else {
                s_link[!s_idx].take()
            };

            if let Some(e_idx) = e_idx {
                while r_idx < link_count {
                    let contour = self.partials[r_idx].clone();

                    if !first {
                        // Connect gap if needed
                        if let Some(_last_pt) = self.path_ptr.last_point() {
                            // In a full implementation, we'd connect via segments
                            // rather than introducing a diagonal
                        }
                    }

                    if forward {
                        for i in 0..contour.count_verbs() {
                            if let Some(verb) = contour.verb(i) {
                                self.append_verb(verb, &contour, i);
                            }
                        }
                    } else {
                        // Add reversed contour
                        for i in (0..contour.count_verbs()).rev() {
                            if let Some(verb) = contour.verb(i) {
                                self.append_verb(verb, &contour, i);
                            }
                        }
                    }

                    if first {
                        first = false;
                    }

                    let close_now = s_idx == r_idx || s_idx == (r_idx + link_count) || e_idx == r_idx || e_idx == (r_idx + link_count);
                    if close_now {
                        self.path_ptr.close();
                        break;
                    }

                    // Update link for next iteration
                    if forward {
                        if e_idx >= 0 && e_idx < link_count {
                            s_link[e_idx] = None;
                        }
                    } else {
                        if r_idx < link_count {
                            s_link[r_idx] = None;
                        }
                    }

                    r_idx += 1;
                }
            }

            // Find next unprocessed contour
            r_idx = 0;
            while r_idx < link_count && (s_link[r_idx].is_some() || e_link[r_idx].is_some()) {
                r_idx += 1;
            }
        }
    }

    /// Returns the partial paths
    pub fn partials(&self) -> &[Path] {
        &self.partials
    }

    /// Appends a verb from a contour to the path pointer
    fn append_verb(&mut self, verb: crate::core::Verb, contour: &Path, verb_idx: usize) {
        match verb {
            crate::core::Verb::Move => {
                if let Some(pt) = contour.point(verb_idx as usize) {
                    self.path_ptr.move_to(pt.x, pt.y);
                }
            }
            crate::core::Verb::Line => {
                if let Some(pt) = contour.point(verb_idx as usize) {
                    self.path_ptr.line_to(pt.x, pt.y);
                }
            }
            crate::core::Verb::Quad => {
                if let Some(pt) = contour.point(verb_idx as usize) {
                    self.path_ptr.quad_to(
                        pt.x, pt.y,
                        contour.point(verb_idx as usize + 1).unwrap_or(Point::new(0.0, 0.0)).x,
                        contour.point(verb_idx as usize + 1).unwrap_or(Point::new(0.0, 0.0)).y,
                    );
                }
            }
            crate::core::Verb::Conic => {
                if let Some(pt) = contour.point(verb_idx as usize) {
                    let pt2 = contour.point(verb_idx as usize + 1).unwrap_or(Point::new(0.0, 0.0));
                    let weight = contour.conic_weights().get(0).copied().unwrap_or(1.0);
                    self.path_ptr.conic_to(pt.x, pt.y, pt2.x, pt2.y, weight);
                }
            }
            crate::core::Verb::Cubic => {
                if let Some(pt) = contour.point(verb_idx as usize) {
                    let pt2 = contour.point(verb_idx as usize + 1).unwrap_or(Point::new(0.0, 0.0));
                    let pt3 = contour.point(verb_idx as usize + 2).unwrap_or(Point::new(0.0, 0.0));
                    self.path_ptr.cubic_to(pt.x, pt.y, pt2.x, pt2.y, pt3.x, pt3.y);
                }
            }
            crate::core::Verb::Close => {
                self.path_ptr.close();
            }
        }
    }
}

impl<'a> Default for SkPathWriter<'a> {
    fn default() -> Self {
        // Create a static Path as default (note: this creates a static reference)
        // In practice, callers should use new() with an actual Path
        unimplemented!("Use SkPathWriter::new() with an actual path reference")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_writer() {
        let mut path = Path::new();
        let writer = SkPathWriter::new(&mut path);
        assert!(writer.partials().is_empty());
    }

    #[test]
    fn test_simple_move_and_line() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        // Start a contour
        writer.first_pt = Some(Point::new(0.0, 0.0));
        writer.defer[0] = Some(Point::new(0.0, 0.0));
        writer.current.move_to(0.0, 0.0);
        writer.defer[1] = Some(Point::new(10.0, 10.0));
        writer.line_to();

        assert!(!writer.current.is_empty());
        assert_eq!(writer.current.count_verbs(), 2); // Move and Line
    }

    #[test]
    fn test_is_closed() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        // Not closed
        writer.first_pt = Some(Point::new(0.0, 0.0));
        writer.defer[1] = Some(Point::new(10.0, 10.0));
        assert!(!writer.is_closed());

        // Closed (end matches start)
        writer.defer[1] = Some(Point::new(0.0, 0.0));
        assert!(writer.is_closed());
    }

    #[test]
    fn test_update_with_first_point() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        writer.first_pt = Some(Point::new(0.0, 0.0));
        writer.defer[0] = Some(Point::new(0.0, 0.0));
        writer.defer[1] = Some(Point::new(5.0, 5.0));

        // Update should connect line and return the point
        let result = writer.update(Point::new(10.0, 10.0));
        assert_eq!(result.x, 10.0);
        assert_eq!(result.y, 10.0);
    }

    #[test]
    fn test_changed_slopes() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        // Setup: 0,0 -> 5,5 -> 10,10 (same slope)
        writer.defer[0] = Some(Point::new(0.0, 0.0));
        writer.defer[1] = Some(Point::new(5.0, 5.0));
        assert!(!writer.changed_slopes(Point::new(10.0, 10.0)));

        // Setup: 0,0 -> 5,5 -> 10,0 (different slope)
        assert!(writer.changed_slopes(Point::new(10.0, 0.0)));
    }

    #[test]
    fn test_conic_and_cubic() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        // conic_to on an empty path auto-injects a moveTo(0, 0) first,
        // same as line_to/quad_to/cubic_to.
        writer.current.conic_to(5.0, 5.0, 10.0, 10.0, 0.5);
        assert_eq!(writer.current.count_verbs(), 2);
        assert_eq!(writer.current.verb(0), Some(crate::core::Verb::Move));
        assert_eq!(writer.current.verb(1), Some(crate::core::Verb::Conic));

        writer.current.cubic_to(5.0, 5.0, 10.0, 10.0, 15.0, 15.0);
        assert_eq!(writer.current.count_verbs(), 3);
        assert_eq!(writer.current.verb(2), Some(crate::core::Verb::Cubic));
    }

    #[test]
    fn test_close_contour() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        writer.current.move_to(0.0, 0.0);
        writer.current.line_to(10.0, 0.0);
        writer.current.line_to(10.0, 10.0);

        writer.close();

        assert!(writer.current.is_empty());
        assert_eq!(writer.path_ptr.count_verbs(), 4); // Move, Line, Line, Close
    }

    #[test]
    fn test_assembly_required() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        // Add partial contour data
        writer.end_pts.push(Point::new(0.0, 0.0));
        writer.end_pts.push(Point::new(10.0, 10.0));
        writer.partials.push(Path::new());

        assert!(writer.some_assembly_required());
    }

    #[test]
    fn test_deferred_move() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        writer.deferred_move(Point::new(0.0, 0.0));
        assert_eq!(writer.first_pt, Some(Point::new(0.0, 0.0)));

        // Second point
        writer.deferred_move(Point::new(10.0, 10.0));
        assert_eq!(writer.defer[0], Some(Point::new(10.0, 10.0)));
    }

    #[test]
    fn test_deferred_line() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        writer.defer[0] = Some(Point::new(0.0, 0.0));

        // Same point - should return true (degenerate)
        assert!(writer.deferred_line(Point::new(0.0, 0.0)));

        // Different point - should return true (valid line)
        assert!(writer.deferred_line(Point::new(10.0, 10.0)));
    }

    #[test]
    fn test_matched_last() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        writer.defer[1] = Some(Point::new(5.0, 5.0));
        assert!(writer.matched_last(Point::new(5.0, 5.0)));
        assert!(!writer.matched_last(Point::new(10.0, 10.0)));
    }

    #[test]
    fn test_points_equal() {
        let mut path = Path::new();
        let writer = SkPathWriter::new(&mut path);

        assert!(writer.points_equal(Point::new(0.0, 0.0), Point::new(0.0, 0.0)));
        assert!(writer.points_equal(Point::new(1.0, 1.0), Point::new(1.0 + f32::EPSILON, 1.0)));
        assert!(!writer.points_equal(Point::new(0.0, 0.0), Point::new(10.0, 10.0)));
    }

    #[test]
    fn test_init() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        writer.first_pt = Some(Point::new(0.0, 0.0));
        writer.defer[0] = Some(Point::new(5.0, 5.0));

        writer.init();
        assert!(writer.first_pt.is_none());
        assert!(writer.defer[0].is_none());
        assert!(writer.defer[1].is_none());
        assert!(writer.current.is_empty());
    }

    #[test]
    fn test_assemble_empty() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        // Should not panic on empty assembly
        writer.assemble();
    }

    #[test]
    fn test_deferred_line_with_slope_change() {
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);

        // Setup: 0,0 -> 5,5
        writer.defer[0] = Some(Point::new(0.0, 0.0));
        writer.defer[1] = Some(Point::new(5.0, 5.0));
        
        // Next point changes slope - should trigger lineTo
        let result = writer.deferred_line(Point::new(10.0, 0.0));
        assert!(result);
        assert_eq!(writer.defer[0], Some(Point::new(5.0, 5.0)));
        assert_eq!(writer.defer[1], Some(Point::new(10.0, 0.0)));
    }
}
