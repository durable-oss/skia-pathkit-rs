//! Incremental path construction.
//!
//! Ported from `include/core/SkPathBuilder.h` / `src/core/SkPathBuilder.cpp`.

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

use super::path::Path;
use super::point::Point;
use super::rect::Rect;
use super::types::{Direction, FillType, Verb};
use crate::error::PathKitError;

/// Bitmasks for segment types present in the path.
const SEGMENT_MASK_LINE: u8 = 1 << 0;
const SEGMENT_MASK_QUAD: u8 = 1 << 1;
const SEGMENT_MASK_CONIC: u8 = 1 << 2;
const SEGMENT_MASK_CUBIC: u8 = 1 << 3;

/// For optimization: tracks if the path is effectively just an oval or rrect.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum IsA {
    #[default]
    None,
    Oval,
    RRect,
}

/// Builds a [`Path`] incrementally via move/line/quad/conic/cubic verbs and
/// shape helpers.
///
/// Mirrors `SkPathBuilder`. Methods take and return `&mut Self` (rather than
/// consuming `self` as Skia's `operator&` chaining does) so callers can
/// choose between fluent chaining and a plain sequence of statements.
#[derive(Debug, Clone)]
pub struct PathBuilder {
    points: Vec<Point>,
    verbs: Vec<Verb>,
    conic_weights: Vec<f32>,
    fill_type: FillType,
    segment_mask: u8,
    last_move_index: i32,
    needs_move: bool,
    is_a: IsA,
}

impl Default for PathBuilder {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            verbs: Vec::new(),
            conic_weights: Vec::new(),
            fill_type: FillType::Winding,
            segment_mask: 0,
            last_move_index: -1,
            needs_move: true,
            is_a: IsA::None,
        }
    }
}

impl PathBuilder {
    /// Constructs an empty builder with [`FillType::Winding`].
    #[must_use]
    pub fn new() -> Self {
        PathBuilder::default()
    }

    /// Returns the builder's current [`FillType`].
    #[must_use]
    pub fn fill_type(&self) -> FillType {
        self.fill_type
    }

    /// Sets the builder's [`FillType`].
    pub fn set_fill_type(&mut self, ft: FillType) -> &mut Self {
        self.fill_type = ft;
        self
    }

    /// Returns a [`Path`] built from the builder's current state, leaving
    /// the builder unchanged.
    #[must_use]
    pub fn snapshot(&self) -> Path {
        self.build_path(false)
    }

    /// Returns a [`Path`] built from the builder's current state and resets
    /// the builder to empty.
    pub fn detach(&mut self) -> Path {
        let path = self.build_path(true);
        self.reset();
        path
    }

    fn build_path(&self, consume: bool) -> Path {
        let mut path = Path::new();
        path.fill_type = self.fill_type;
        path.last_move_to_index = self.last_move_index;

        if consume {
            path.verbs = self.verbs.clone();
            path.points = self.points.clone();
            path.conic_weights = self.conic_weights.clone();
        } else {
            path.verbs = self.verbs.clone();
            path.points = self.points.clone();
            path.conic_weights = self.conic_weights.clone();
        }
        path
    }

    /// Clears all verbs, points, and weights, and resets the fill type.
    pub fn reset(&mut self) -> &mut Self {
        self.points.clear();
        self.verbs.clear();
        self.conic_weights.clear();
        self.fill_type = FillType::Winding;
        self.segment_mask = 0;
        self.last_move_index = -1;
        self.needs_move = true;
        self.is_a = IsA::None;
        self
    }

    fn ensure_move(&mut self) {
        if self.needs_move {
            self.needs_move = false;
            self.verbs.push(Verb::Move);
            self.points.push(Point::default());
            self.last_move_index = self.points.len() as i32;
        }
    }

    /// Starts a new contour at `pt`.
    pub fn move_to(&mut self, pt: Point) -> &mut Self {
        self.needs_move = false;
        self.last_move_index = self.points.len() as i32;
        self.points.push(pt);
        self.verbs.push(Verb::Move);
        self
    }

    /// Appends a line from the current point to `pt`.
    pub fn line_to(&mut self, pt: Point) -> &mut Self {
        self.ensure_move();
        self.points.push(pt);
        self.verbs.push(Verb::Line);
        self.segment_mask |= SEGMENT_MASK_LINE;
        self
    }

    /// Appends a quadratic Bezier from the current point through control
    /// point `p1` to endpoint `p2`.
    pub fn quad_to(&mut self, p1: Point, p2: Point) -> &mut Self {
        self.ensure_move();
        self.points.push(p1);
        self.points.push(p2);
        self.verbs.push(Verb::Quad);
        self.segment_mask |= SEGMENT_MASK_QUAD;
        self
    }

    /// Appends a conic (rational quadratic) from the current point through
    /// control point `p1` to endpoint `p2`, with weight `w`.
    pub fn conic_to(&mut self, p1: Point, p2: Point, w: f32) -> &mut Self {
        self.ensure_move();
        self.points.push(p1);
        self.points.push(p2);
        self.verbs.push(Verb::Conic);
        self.conic_weights.push(w);
        self.segment_mask |= SEGMENT_MASK_CONIC;
        self
    }

    /// Appends a cubic Bezier from the current point through control points
    /// `p1`, `p2` to endpoint `p3`.
    pub fn cubic_to(&mut self, p1: Point, p2: Point, p3: Point) -> &mut Self {
        self.ensure_move();
        self.points.push(p1);
        self.points.push(p2);
        self.points.push(p3);
        self.verbs.push(Verb::Cubic);
        self.segment_mask |= SEGMENT_MASK_CUBIC;
        self
    }

    /// Closes the current contour with a line back to its start point.
    pub fn close(&mut self) -> &mut Self {
        if !self.verbs.is_empty() {
            self.ensure_move();
            self.verbs.push(Verb::Close);
            self.needs_move = true;
        }
        self
    }

    /// Appends a series of `line_to` calls through `pts`.
    pub fn polyline_to(&mut self, pts: &[Point]) -> Result<&mut Self, PathKitError> {
        if !pts.is_empty() {
            self.ensure_move();
            self.points.extend_from_slice(pts);
            for _ in pts {
                self.verbs.push(Verb::Line);
            }
            self.segment_mask |= SEGMENT_MASK_LINE;
        }
        Ok(self)
    }

    /// Adds a rectangle as a new contour.
    pub fn add_rect(&mut self, rect: Rect, dir: Direction, _start: u32) -> &mut Self {
        let corners = rect.to_quad();
        let order = match dir {
            Direction::Cw => [0, 1, 2, 3],
            Direction::Ccw => [0, 3, 2, 1],
        };
        self.move_to(corners[order[0]]);
        self.line_to(corners[order[1]]);
        self.line_to(corners[order[2]]);
        self.line_to(corners[order[3]]);
        self.close()
    }

    /// Adds an oval inscribed in `rect` as a new contour.
    pub fn add_oval(&mut self, rect: Rect, dir: Direction, _start: u32) -> &mut Self {
        let center_x = rect.left + (rect.right - rect.left) / 2.0;
        let center_y = rect.top + (rect.bottom - rect.top) / 2.0;

        let oval_pts = [
            Point::new(center_x, rect.top),
            Point::new(rect.right, center_y),
            Point::new(center_x, rect.bottom),
            Point::new(rect.left, center_y),
        ];

        let rect_pts = [
            Point::new(rect.left, rect.top),
            Point::new(rect.right, rect.top),
            Point::new(rect.right, rect.bottom),
            Point::new(rect.left, rect.bottom),
        ];

        let w = 0.7071067811865476f32; // sqrt(2)/2

        self.move_to(oval_pts[0]);
        for i in 0..4 {
            let rect_idx = (i + match dir {
                Direction::Cw => 0,
                Direction::Ccw => 1,
            }) % 4;
            let oval_idx = (i + 1) % 4;
            self.conic_to(rect_pts[rect_idx], oval_pts[oval_idx], w);
        }
        self.close()
    }

    /// Adds a circle centered at `(center_x, center_y)` with `radius`.
    pub fn add_circle(
        &mut self,
        center_x: f32,
        center_y: f32,
        radius: f32,
        dir: Direction,
    ) -> &mut Self {
        if radius >= 0.0 {
            let rect = Rect::from_ltrb(
                center_x - radius,
                center_y - radius,
                center_x + radius,
                center_y + radius,
            );
            self.add_oval(rect, dir, 0);
        }
        self
    }

    /// Adds `pts` as a new contour, closing it if `is_closed` is `true`.
    pub fn add_polygon(&mut self, pts: &[Point], is_closed: bool) -> &mut Self {
        if !pts.is_empty() {
            self.move_to(pts[0]);
            for &pt in &pts[1..] {
                self.line_to(pt);
            }
            if is_closed {
                self.close();
            }
        }
        self
    }

    /// Appends the contours of `path` to this builder.
    pub fn add_path(&mut self, path: &Path) -> &mut Self {
        for (i, &verb) in path.verbs().iter().enumerate() {
            let start_idx = path.points().len().saturating_sub(
                path.verbs()
                    .iter()
                    .skip(i + 1)
                    .map(|v| v.point_count())
                    .sum(),
            );
            match verb {
                Verb::Move => {
                    if !path.points().is_empty() {
                        self.move_to(path.points()[start_idx]);
                    }
                }
                Verb::Line => {
                    if start_idx + 1 < path.points().len() {
                        self.line_to(path.points()[start_idx + 1]);
                    }
                }
                Verb::Quad => {
                    if start_idx + 2 < path.points().len() {
                        self.quad_to(path.points()[start_idx + 1], path.points()[start_idx + 2]);
                    }
                }
                Verb::Conic => {
                    if start_idx + 2 < path.points().len() {
                        let w = path.conic_weights().get(0).copied().unwrap_or(1.0);
                        self.conic_to(
                            path.points()[start_idx + 1],
                            path.points()[start_idx + 2],
                            w,
                        );
                    }
                }
                Verb::Cubic => {
                    if start_idx + 3 < path.points().len() {
                        self.cubic_to(
                            path.points()[start_idx + 1],
                            path.points()[start_idx + 2],
                            path.points()[start_idx + 3],
                        );
                    }
                }
                Verb::Close => {
                    self.close();
                }
            }
        }
        self
    }

    /// Offsets every point added so far by `(dx, dy)`.
    pub fn offset(&mut self, dx: f32, dy: f32) -> &mut Self {
        for p in &mut self.points {
            p.x += dx;
            p.y += dy;
        }
        self
    }

    /// Replaces the fill type with its inverse.
    pub fn toggle_inverse_fill_type(&mut self) -> &mut Self {
        self.fill_type = match self.fill_type {
            FillType::Winding => FillType::InverseWinding,
            FillType::InverseWinding => FillType::Winding,
            FillType::EvenOdd => FillType::InverseEvenOdd,
            FillType::InverseEvenOdd => FillType::EvenOdd,
        };
        self
    }

    /// Returns `true` if this builder has no verbs.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.verbs.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_builder_is_empty() {
        let b = PathBuilder::new();
        assert!(b.is_empty());
        assert_eq!(b.fill_type(), FillType::Winding);
    }

    #[test]
    fn reset_clears_everything() {
        let mut b = PathBuilder::new();
        b.move_to(Point::new(1.0, 2.0));
        b.line_to(Point::new(3.0, 4.0));
        b.reset();
        assert!(b.is_empty());
        assert_eq!(b.fill_type(), FillType::Winding);
    }

    #[test]
    fn move_to_line_to() {
        let mut b = PathBuilder::new();
        b.move_to(Point::new(0.0, 0.0));
        b.line_to(Point::new(10.0, 10.0));
        assert_eq!(b.points.len(), 2);
        assert_eq!(b.verbs.len(), 2);
        assert_eq!(b.verbs[0], Verb::Move);
        assert_eq!(b.verbs[1], Verb::Line);
    }

    #[test]
    fn line_to_auto_moves() {
        let mut b = PathBuilder::new();
        b.line_to(Point::new(5.0, 5.0));
        assert_eq!(b.verbs.len(), 2);
        assert_eq!(b.verbs[0], Verb::Move);
        assert_eq!(b.points[0], Point::default());
    }

    #[test]
    fn quad_to() {
        let mut b = PathBuilder::new();
        b.move_to(Point::new(0.0, 0.0));
        b.quad_to(Point::new(50.0, 100.0), Point::new(100.0, 0.0));
        assert_eq!(b.verbs[1], Verb::Quad);
        assert_eq!(b.points.len(), 3);
    }

    #[test]
    fn conic_to() {
        let mut b = PathBuilder::new();
        b.move_to(Point::new(0.0, 0.0));
        b.conic_to(Point::new(50.0, 50.0), Point::new(100.0, 0.0), 0.5);
        assert_eq!(b.verbs[1], Verb::Conic);
        assert_eq!(b.conic_weights.len(), 1);
        assert!((b.conic_weights[0] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn cubic_to() {
        let mut b = PathBuilder::new();
        b.move_to(Point::new(0.0, 0.0));
        b.cubic_to(
            Point::new(10.0, 20.0),
            Point::new(30.0, 40.0),
            Point::new(50.0, 60.0),
        );
        assert_eq!(b.verbs[1], Verb::Cubic);
        assert_eq!(b.points.len(), 4);
    }

    #[test]
    fn close() {
        let mut b = PathBuilder::new();
        b.move_to(Point::new(0.0, 0.0));
        b.line_to(Point::new(10.0, 0.0));
        b.close();
        assert_eq!(b.verbs[2], Verb::Close);
    }

    #[test]
    fn add_rect() {
        let mut b = PathBuilder::new();
        let rect = Rect::from_ltrb(10.0, 20.0, 30.0, 40.0);
        b.add_rect(rect, Direction::Cw, 0);
        assert_eq!(b.verbs.len(), 5);
        assert_eq!(b.verbs[0], Verb::Move);
        assert_eq!(b.verbs[4], Verb::Close);
    }

    #[test]
    fn add_circle() {
        let mut b = PathBuilder::new();
        b.add_circle(50.0, 50.0, 10.0, Direction::Cw);
        assert!(!b.is_empty());
    }

    #[test]
    fn snapshot_and_detach() {
        let mut b = PathBuilder::new();
        b.move_to(Point::new(0.0, 0.0));
        b.line_to(Point::new(10.0, 10.0));
        b.close();

        let snap = b.snapshot();
        assert!(!snap.is_empty());
        assert!(!b.is_empty());

        let mut b2 = PathBuilder::new();
        b2.move_to(Point::new(0.0, 0.0));
        b2.line_to(Point::new(10.0, 10.0));
        let det = b2.detach();
        assert!(!det.is_empty());
        assert!(b2.is_empty());
    }

    #[test]
    fn polyline_to() {
        let mut b = PathBuilder::new();
        b.move_to(Point::new(0.0, 0.0));
        let pts = [Point::new(1.0, 1.0), Point::new(2.0, 2.0)];
        b.polyline_to(&pts).unwrap();
        assert_eq!(b.points.len(), 3);
        assert_eq!(b.verbs.len(), 3);
        assert_eq!(b.verbs[1], Verb::Line);
        assert_eq!(b.verbs[2], Verb::Line);
    }

    #[test]
    fn offset() {
        let mut b = PathBuilder::new();
        b.move_to(Point::new(0.0, 0.0));
        b.line_to(Point::new(10.0, 20.0));
        b.offset(5.0, 10.0);
        assert_eq!(b.points[0], Point::new(5.0, 10.0));
        assert_eq!(b.points[1], Point::new(15.0, 30.0));
    }

    #[test]
    fn add_oval_centers_on_a_rect_away_from_the_origin() {
        // The four on-curve points of an oval are the midpoints of the
        // bounding rect's sides, so each must sit on the rect's centre lines.
        let mut b = PathBuilder::new();
        b.add_oval(Rect::from_ltrb(20.0, 40.0, 120.0, 140.0), Direction::Cw, 0);

        let cx = 70.0;
        let cy = 90.0;
        assert_eq!(b.points[0], Point::new(cx, 40.0));
        assert!(b.points.contains(&Point::new(120.0, cy)));
        assert!(b.points.contains(&Point::new(cx, 140.0)));
        assert!(b.points.contains(&Point::new(20.0, cy)));
    }

    #[test]
    fn add_polygon() {
        let mut b = PathBuilder::new();
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(5.0, 10.0),
        ];
        b.add_polygon(&pts, true);
        assert_eq!(b.verbs.len(), 4);
        assert_eq!(b.verbs[3], Verb::Close);
    }
}
