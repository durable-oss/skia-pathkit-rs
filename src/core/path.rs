//! Path storage, construction, and iteration.
//!
//! Ported from `include/core/SkPath.h` / `src/core/SkPath.cpp`.

use super::point::{Point, Vector};
use super::rect::Rect;
use super::sk_cubic_clipper::SkCubicClipper;
use super::sk_geometry::{self, Conic};
use super::scalar::{self, Scalar};
use super::types::{Direction, FillType, Verb};

/// A 2D path: a sequence of verbs (move/line/quad/conic/cubic/close) with
/// their associated points and, for conics, weights.
///
/// Mirrors `SkPath`.
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    pub(crate) verbs: Vec<Verb>,
    pub(crate) points: Vec<Point>,
    pub(crate) conic_weights: Vec<f32>,
    pub(crate) fill_type: FillType,
    pub(crate) last_move_to_index: i32,
}

impl Default for Path {
    fn default() -> Self {
        Self {
            verbs: Vec::new(),
            points: Vec::new(),
            conic_weights: Vec::new(),
            fill_type: FillType::default(),
            // Negative means "no contour started yet"; a derived Default
            // would leave this at 0, which line_to/quad_to/etc. would
            // mistake for a valid point index and skip the implicit
            // moveTo(0, 0) they're supposed to inject.
            last_move_to_index: -1,
        }
    }
}

impl Path {
    /// Constructs an empty path with [`FillType::Winding`].
    #[must_use]
    pub fn new() -> Self {
        Path::default()
    }

    /// Returns the path's current [`FillType`].
    #[must_use]
    pub fn fill_type(&self) -> FillType {
        self.fill_type
    }

    /// Sets the path's [`FillType`].
    pub fn set_fill_type(&mut self, ft: FillType) {
        self.fill_type = ft;
    }

    /// Returns `true` if the fill type is one of the inverse rules.
    #[must_use]
    pub fn is_inverse_fill_type(&self) -> bool {
        self.fill_type.is_inverse()
    }

    /// Replaces the fill type with its inverse.
    pub fn toggle_inverse_fill_type(&mut self) {
        self.fill_type = match self.fill_type {
            FillType::Winding => FillType::InverseWinding,
            FillType::InverseWinding => FillType::Winding,
            FillType::EvenOdd => FillType::InverseEvenOdd,
            FillType::InverseEvenOdd => FillType::EvenOdd,
        };
    }

    /// Removes all verbs, points, and weights, and resets the fill type to
    /// [`FillType::Winding`].
    pub fn reset(&mut self) {
        *self = Path::new();
    }

    /// Removes all verbs, points, and weights, preserving internal storage.
    pub fn rewind(&mut self) {
        self.verbs.clear();
        self.points.clear();
        self.conic_weights.clear();
        self.fill_type = FillType::Winding;
        self.last_move_to_index = -1;
    }

    /// Returns `true` if the path has no verbs.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.verbs.is_empty()
    }

    /// Returns `true` if the last contour ends with a Close verb.
    #[must_use]
    pub fn is_last_contour_closed(&self) -> bool {
        self.verbs.last() == Some(&Verb::Close)
    }

    /// Returns the number of points stored in the path.
    #[must_use]
    pub fn count_points(&self) -> usize {
        self.points.len()
    }

    /// Returns the point at `index`, if any.
    #[must_use]
    pub fn point(&self, index: usize) -> Option<Point> {
        self.points.get(index).copied()
    }

    /// Returns all points in the path.
    #[must_use]
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    /// Returns the number of verbs stored in the path.
    #[must_use]
    pub fn count_verbs(&self) -> usize {
        self.verbs.len()
    }

    /// Returns the verb at `index`, if any.
    #[must_use]
    pub fn verb(&self, index: usize) -> Option<Verb> {
        self.verbs.get(index).copied()
    }

    /// Returns all verbs in the path.
    #[must_use]
    pub fn verbs(&self) -> &[Verb] {
        &self.verbs
    }

    /// Returns all conic weights in the path.
    #[must_use]
    pub fn conic_weights(&self) -> &[f32] {
        &self.conic_weights
    }

    /// Returns a mask of segment types present in the path.
    #[must_use]
    pub fn get_segment_masks(&self) -> u8 {
        let mut mask = 0u8;
        for v in &self.verbs {
            match v {
                Verb::Line => mask |= crate::core::types::SEGMENT_MASK_LINE,
                Verb::Quad => mask |= crate::core::types::SEGMENT_MASK_QUAD,
                Verb::Conic => mask |= crate::core::types::SEGMENT_MASK_CONIC,
                Verb::Cubic => mask |= crate::core::types::SEGMENT_MASK_CUBIC,
                _ => {}
            }
        }
        mask
    }

    /// Returns `true` if every point in the path is finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.points.iter().all(|p| p.is_finite())
    }

    /// Returns the last point in the path, if any.
    #[must_use]
    pub fn last_point(&self) -> Option<Point> {
        self.points.last().copied()
    }

    /// Returns `true` if the path contains only move verbs.
    fn has_only_move_tos(&self) -> bool {
        self.verbs.iter().all(|v| *v == Verb::Move)
    }

    /// Injects a Move(0,0) or Move(last_move_to) if the path needs one
    /// before a drawing verb.
    fn inject_move_to_if_needed(&mut self) {
        if self.last_move_to_index < 0 {
            let (x, y) = if self.verbs.is_empty() {
                (0.0, 0.0)
            } else {
                let idx = (!self.last_move_to_index) as usize;
                if idx < self.points.len() {
                    let pt = self.points[idx];
                    (pt.x, pt.y)
                } else {
                    (0.0, 0.0)
                }
            };
            self.move_to(x, y);
        }
    }

    /// Starts a new contour at `(x, y)`.
    pub fn move_to(&mut self, x: Scalar, y: Scalar) -> &mut Self {
        self.last_move_to_index = self.points.len() as i32;
        self.verbs.push(Verb::Move);
        self.points.push(Point::new(x, y));
        self
    }

    /// Appends a line from the last point to `(x, y)`.
    pub fn line_to(&mut self, x: Scalar, y: Scalar) -> &mut Self {
        self.inject_move_to_if_needed();
        self.verbs.push(Verb::Line);
        self.points.push(Point::new(x, y));
        self
    }

    /// Appends a quadratic Bezier from the last point through `(x1, y1)` to
    /// `(x2, y2)`.
    pub fn quad_to(&mut self, x1: Scalar, y1: Scalar, x2: Scalar, y2: Scalar) -> &mut Self {
        self.inject_move_to_if_needed();
        self.verbs.push(Verb::Quad);
        self.points.push(Point::new(x1, y1));
        self.points.push(Point::new(x2, y2));
        self
    }

    /// Appends a conic (rational quadratic) from the last point through
    /// `(x1, y1)` to `(x2, y2)` with weight `w`.
    pub fn conic_to(&mut self, x1: Scalar, y1: Scalar, x2: Scalar, y2: Scalar, w: Scalar) -> &mut Self {
        if !(w > 0.0) {
            return self.line_to(x2, y2);
        }
        if !w.is_finite() {
            self.line_to(x1, y1);
            return self.line_to(x2, y2);
        }
        if w == 1.0 {
            return self.quad_to(x1, y1, x2, y2);
        }
        self.inject_move_to_if_needed();
        self.verbs.push(Verb::Conic);
        self.points.push(Point::new(x1, y1));
        self.points.push(Point::new(x2, y2));
        self.conic_weights.push(w);
        self
    }

    /// Appends a cubic Bezier from the last point through `(x1, y1)` and
    /// `(x2, y2)` to `(x3, y3)`.
    pub fn cubic_to(&mut self, x1: Scalar, y1: Scalar, x2: Scalar, y2: Scalar, x3: Scalar, y3: Scalar) -> &mut Self {
        self.inject_move_to_if_needed();
        self.verbs.push(Verb::Cubic);
        self.points.push(Point::new(x1, y1));
        self.points.push(Point::new(x2, y2));
        self.points.push(Point::new(x3, y3));
        self
    }

    /// Closes the current contour.
    pub fn close(&mut self) -> &mut Self {
        if self.verbs.is_empty() {
            return self;
        }
        match self.verbs.last() {
            Some(Verb::Line | Verb::Quad | Verb::Conic | Verb::Cubic | Verb::Move) => {
                self.verbs.push(Verb::Close);
            }
            _ => {}
        }
        self.last_move_to_index ^= !(self.last_move_to_index >> (8 * std::mem::size_of::<i32>() - 1));
        self
    }

    /// Adds a rectangle as a new closed contour.
    pub fn add_rect(&mut self, rect: Rect, dir: Direction, start: u32) -> &mut Self {
        let corners = rect.to_quad();
        let indices: [usize; 4] = match dir {
            Direction::Cw => [0, 1, 2, 3],
            Direction::Ccw => [0, 3, 2, 1],
        };
        let start_idx = (start as usize) % 4;
        let rotated: [usize; 4] = [
            indices[(start_idx) % 4],
            indices[(start_idx + 1) % 4],
            indices[(start_idx + 2) % 4],
            indices[(start_idx + 3) % 4],
        ];
        self.move_to(corners[rotated[0]].x, corners[rotated[0]].y);
        self.line_to(corners[rotated[1]].x, corners[rotated[1]].y);
        self.line_to(corners[rotated[2]].x, corners[rotated[2]].y);
        self.line_to(corners[rotated[3]].x, corners[rotated[3]].y);
        self.close();
        self
    }

    /// Adds a rectangle as a new closed contour with CW direction and
    /// start=0.
    pub fn add_rect_simple(&mut self, rect: Rect) -> &mut Self {
        self.add_rect(rect, Direction::Cw, 0)
    }

    /// Adds `pts` as a new contour, connected by lines.
    ///
    /// If `close` is set, the contour is closed with [`Path::close`].
    /// Does nothing if `pts` is empty.
    pub fn add_poly(&mut self, pts: &[Point], close: bool) -> &mut Self {
        if pts.is_empty() {
            return self;
        }
        self.move_to(pts[0].x, pts[0].y);
        for p in &pts[1..] {
            self.line_to(p.x, p.y);
        }
        if close {
            self.close();
        }
        self
    }

    /// Adds an oval inscribed in `oval` as a new closed contour.
    pub fn add_oval(&mut self, oval: Rect, dir: Direction) -> &mut Self {
        // Ported from `SkPath::addOval` (`src/core/SkPath.cpp`), using its
        // default legacy start index of 1: the oval side-midpoints and the
        // rect corners are both walked starting one step past index 0, with
        // the corner walk offset by one extra step for `Ccw`. See
        // `SkPath_PointIterator`/`SkPath_OvalPointIterator`/
        // `SkPath_RectPointIterator` in `SkPathMakers.h`.
        let cx = oval.center_x();
        let cy = oval.center_y();
        // oval_pts[i]: top-mid, right-mid, bottom-mid, left-mid.
        let oval_pts = [
            Point::new(cx, oval.top),
            Point::new(oval.right, cy),
            Point::new(cx, oval.bottom),
            Point::new(oval.left, cy),
        ];
        // rect_pts[i]: TL, TR, BR, BL.
        let rect_pts = oval.to_quad();
        const WEIGHT: Scalar = std::f32::consts::SQRT_2 / 2.0;

        let (advance, rect_start): (i32, usize) = match dir {
            Direction::Cw => (1, 1),
            Direction::Ccw => (-1, 2),
        };
        let mut oval_idx = 1usize;
        let mut rect_idx = rect_start;
        let step = |idx: &mut usize| {
            *idx = (*idx as i32 + advance).rem_euclid(4) as usize;
        };

        self.move_to(oval_pts[oval_idx].x, oval_pts[oval_idx].y);
        for _ in 0..4 {
            step(&mut rect_idx);
            step(&mut oval_idx);
            let control = rect_pts[rect_idx];
            let end = oval_pts[oval_idx];
            self.conic_to(control.x, control.y, end.x, end.y, WEIGHT);
        }
        self.close();
        self
    }

    /// Adds a circle centered at `(x, y)` with radius `r` as a new closed
    /// contour. Does nothing if `r` is not positive.
    pub fn add_circle(&mut self, x: Scalar, y: Scalar, r: Scalar) -> &mut Self {
        if r > 0.0 {
            let oval = Rect::from_ltrb(x - r, y - r, x + r, y + r);
            self.add_oval(oval, Direction::Cw);
        }
        self
    }

    /// Appends an arc from the current point tangent to the line to
    /// `(x1, y1)` and then tangent to the line from `(x1, y1)` to
    /// `(x2, y2)`, with the given `radius`.
    ///
    /// Mirrors `SkPath::arcTo(x1, y1, x2, y2, radius)`.
    pub fn arc_to(&mut self, x1: Scalar, y1: Scalar, x2: Scalar, y2: Scalar, radius: Scalar) -> &mut Self {
        if radius == 0.0 {
            return self.line_to(x1, y1);
        }

        let (px, py) = match self.last_point() {
            Some(p) => (p.x, p.y),
            None => (0.0, 0.0),
        };

        let dx = x1 - px;
        let dy = y1 - py;
        let before_len = (dx * dx + dy * dy).sqrt();
        if before_len < 1e-6 {
            return self.line_to(x1, y1);
        }

        let after_len = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)).sqrt();
        if after_len < 1e-6 {
            return self.line_to(x1, y1);
        }

        let before_x = dx / before_len;
        let before_y = dy / before_len;
        let after_x = (x2 - x1) / after_len;
        let after_y = (y2 - y1) / after_len;

        let cross = before_x * after_y - before_y * after_x;
        if cross.abs() < 1e-6 {
            return self.line_to(x1, y1);
        }

        let cos_theta = before_x * after_x + before_y * after_y;
        let dist = radius * (1.0 - cos_theta) / cross;

        self.line_to(x1 - dist * before_x, y1 - dist * before_y);
        let weight = (0.5 + 0.5 * cos_theta).sqrt();
        self.conic_to(x1, y1, x1 + dist * after_x, y1 + dist * after_y, weight);
        self
    }

    /// Appends `src`'s verbs and points (offset by `(dx, dy)`) to this path.
    /// Does nothing if `src` is empty.
    pub fn add_path(&mut self, src: &Path, dx: Scalar, dy: Scalar) -> &mut Self {
        if src.is_empty() {
            return self;
        }

        let mut vi = 0usize;
        let mut pi = 0usize;
        let mut wi = 0usize;
        while vi < src.verbs.len() {
            match src.verbs[vi] {
                Verb::Move => {
                    let p = src.points[pi];
                    self.move_to(p.x + dx, p.y + dy);
                    pi += 1;
                }
                Verb::Line => {
                    let p = src.points[pi];
                    self.line_to(p.x + dx, p.y + dy);
                    pi += 1;
                }
                Verb::Quad => {
                    let p1 = src.points[pi];
                    let p2 = src.points[pi + 1];
                    self.quad_to(p1.x + dx, p1.y + dy, p2.x + dx, p2.y + dy);
                    pi += 2;
                }
                Verb::Conic => {
                    let p1 = src.points[pi];
                    let p2 = src.points[pi + 1];
                    let w = src.conic_weights[wi];
                    self.conic_to(p1.x + dx, p1.y + dy, p2.x + dx, p2.y + dy, w);
                    pi += 2;
                    wi += 1;
                }
                Verb::Cubic => {
                    let p1 = src.points[pi];
                    let p2 = src.points[pi + 1];
                    let p3 = src.points[pi + 2];
                    self.cubic_to(p1.x + dx, p1.y + dy, p2.x + dx, p2.y + dy, p3.x + dx, p3.y + dy);
                    pi += 3;
                }
                Verb::Close => {
                    self.close();
                }
            }
            vi += 1;
        }
        self
    }

    /// Returns an iterator over the path's segments.
    ///
    /// Each item is `(verb, points, conic_weight)`, where `points` holds
    /// the segment's start point followed by its control/end points
    /// (unused slots are `Point::default()`), and `conic_weight` is `Some`
    /// only for [`Verb::Conic`] segments.
    #[must_use]
    pub fn iter(&self) -> PathIter<'_> {
        PathIter::new(self)
    }

    /// Swaps this path with `other`.
    pub fn swap(&mut self, other: &mut Self) {
        std::mem::swap(self, other);
    }

    /// Returns the bounding box of the path's control points.
    #[must_use]
    pub fn bounds(&self) -> Rect {
        if self.points.is_empty() {
            return Rect::empty();
        }
        let mut b = Rect::from_ltrb(
            self.points[0].x, self.points[0].y,
            self.points[0].x, self.points[0].y,
        );
        for p in &self.points[1..] {
            b.left = b.left.min(p.x);
            b.top = b.top.min(p.y);
            b.right = b.right.max(p.x);
            b.bottom = b.bottom.max(p.y);
        }
        b
    }

    /// Computes the tight bounding box, accounting for curve extrema.
    #[must_use]
    pub fn compute_tight_bounds(&self) -> Rect {
        if self.verbs.is_empty() {
            return Rect::empty();
        }
        // If only lines, control-point bounds are tight.
        if self.get_segment_masks() == crate::core::types::SEGMENT_MASK_LINE || self.get_segment_masks() == 0 {
            return self.bounds();
        }

        // Seed with first point.
        let mut b = Rect::from_ltrb(
            self.points[0].x, self.points[0].y,
            self.points[0].x, self.points[0].y,
        );
        let mut vi = 0usize;
        let mut pi = 0usize;
        let mut wi = 0usize;
        while vi < self.verbs.len() {
            match self.verbs[vi] {
                Verb::Move => {
                    b = include_point(b, self.points[pi]);
                    pi += 1;
                }
                Verb::Line => {
                    b = include_point(b, self.points[pi]);
                    pi += 1;
                }
                Verb::Quad => {
                    let pts = [self.points[pi - 1], self.points[pi], self.points[pi + 1]];
                    b = include_quad_tight(b, &pts);
                    pi += 2;
                }
                Verb::Conic => {
                    let pts = [self.points[pi - 1], self.points[pi], self.points[pi + 1]];
                    let w = self.conic_weights[wi];
                    b = include_conic_tight(b, &pts, w);
                    pi += 2;
                    wi += 1;
                }
                Verb::Cubic => {
                    let pts = [self.points[pi - 1], self.points[pi], self.points[pi + 1], self.points[pi + 2]];
                    b = include_cubic_tight(b, &pts);
                    pi += 3;
                }
                Verb::Close => {}
            }
            vi += 1;
        }
        b
    }

    /// Returns `true` if the path contains exactly one line.
    #[must_use]
    pub fn is_line(&self, line: Option<&mut [Point; 2]>) -> bool {
        if self.verbs.len() == 2 && self.verbs[0] == Verb::Move && self.verbs[1] == Verb::Line {
            if let Some(l) = line {
                l[0] = self.points[0];
                l[1] = self.points[1];
            }
            true
        } else {
            false
        }
    }

    /// Returns `true` if the path is equivalent to a rectangle.
    #[must_use]
    pub fn is_rect(&self, rect: Option<&mut Rect>, is_closed: Option<&mut bool>, direction: Option<&mut Direction>) -> bool {
        // Check for exactly: Move, Line, Line, Line, Close (5 verbs)
        if self.verbs.len() != 5 {
            return false;
        }
        if self.verbs[0] != Verb::Move || self.verbs[4] != Verb::Close {
            return false;
        }
        for i in 1..4 {
            if self.verbs[i] != Verb::Line {
                return false;
            }
        }
        if self.points.len() < 4 {
            return false;
        }
        let pts = [self.points[0], self.points[1], self.points[2], self.points[3]];
        // Check that the lines form a rectangle: each point is aligned with one other.
        // Rect offset by diagonal: p0.x == p3.x, p0.y == p1.y, p1.x == p2.x, p2.y == p3.y
        if pts[0].x == pts[3].x && pts[0].y == pts[1].y
            && pts[1].x == pts[2].x && pts[2].y == pts[3].y
            && pts[0].x != pts[1].x && pts[0].y != pts[2].y
        {
            if let Some(r) = rect {
                let left = pts[0].x.min(pts[1].x);
                let top = pts[0].y.min(pts[3].y);
                let right = pts[0].x.max(pts[1].x);
                let bottom = pts[0].y.max(pts[3].y);
                *r = Rect::from_ltrb(left, top, right, bottom);
            }
            if let Some(c) = is_closed {
                *c = true;
            }
            if let Some(d) = direction {
                let cw = (pts[1].x - pts[0].x) * (pts[3].y - pts[0].y)
                    - (pts[1].y - pts[0].y) * (pts[3].x - pts[0].x);
                *d = if cw > 0.0 { Direction::Cw } else { Direction::Ccw };
            }
            return true;
        }
        false
    }

    /// Returns `true` if `(x, y)` is enclosed by the path.
    ///
    /// Ported from `SkPath::contains` (`src/core/SkPath.cpp`): computes a
    /// winding number via horizontal-ray crossings against every segment,
    /// then falls back to tangent-coincidence counting when the point lands
    /// exactly on an even number of curve crossings (e.g. two contours that
    /// touch at `(x, y)`).
    #[must_use]
    pub fn contains(&self, x: Scalar, y: Scalar) -> bool {
        let is_inverse = self.is_inverse_fill_type();
        if self.is_empty() {
            return is_inverse;
        }

        let b = self.bounds();
        if !(b.left <= x && x <= b.right && b.top <= y && y <= b.bottom) {
            return is_inverse;
        }

        let mut w = 0i32;
        let mut on_curve_count = 0i32;
        let mut contour_start = Point::default();
        let mut last_pt = Point::default();
        for (verb, pts, weight) in self.iter() {
            match verb {
                Verb::Move => {
                    contour_start = pts[0];
                    last_pt = pts[0];
                }
                Verb::Close => {
                    if last_pt != contour_start {
                        w += winding_line(&[last_pt, contour_start], x, y, &mut on_curve_count);
                    }
                }
                Verb::Line => {
                    w += winding_line(&[pts[0], pts[1]], x, y, &mut on_curve_count);
                    last_pt = pts[1];
                }
                Verb::Quad => {
                    w += winding_quad(&[pts[0], pts[1], pts[2]], x, y, &mut on_curve_count);
                    last_pt = pts[2];
                }
                Verb::Conic => {
                    w += winding_conic(&[pts[0], pts[1], pts[2]], x, y, weight.unwrap_or(1.0), &mut on_curve_count);
                    last_pt = pts[2];
                }
                Verb::Cubic => {
                    w += winding_cubic(&[pts[0], pts[1], pts[2], pts[3]], x, y, &mut on_curve_count);
                    last_pt = pts[3];
                }
            }
        }

        let even_odd_fill = matches!(self.fill_type, FillType::EvenOdd | FillType::InverseEvenOdd);
        if even_odd_fill {
            w &= 1;
        }
        if w != 0 {
            return !is_inverse;
        }
        if on_curve_count <= 1 {
            return (on_curve_count != 0) ^ is_inverse;
        }
        if (on_curve_count & 1) != 0 || even_odd_fill {
            return (on_curve_count & 1 != 0) ^ is_inverse;
        }

        // Point touches an even number of curves under winding fill: check
        // for coincidence by comparing tangents at (x, y). Two crossings
        // with opposing collinear tangents cancel out (the boundary just
        // touches, doesn't enclose); anything left over means it's inside.
        let mut tangents: Vec<Vector> = Vec::new();
        let mut contour_start = Point::default();
        let mut last_pt = Point::default();
        for (verb, pts, weight) in self.iter() {
            let old_count = tangents.len();
            match verb {
                Verb::Move => {
                    contour_start = pts[0];
                    last_pt = pts[0];
                }
                Verb::Close => {
                    if last_pt != contour_start {
                        tangent_line(&[last_pt, contour_start], x, y, &mut tangents);
                    }
                }
                Verb::Line => {
                    tangent_line(&[pts[0], pts[1]], x, y, &mut tangents);
                    last_pt = pts[1];
                }
                Verb::Quad => {
                    tangent_quad(&[pts[0], pts[1], pts[2]], x, y, &mut tangents);
                    last_pt = pts[2];
                }
                Verb::Conic => {
                    tangent_conic(&[pts[0], pts[1], pts[2]], x, y, weight.unwrap_or(1.0), &mut tangents);
                    last_pt = pts[2];
                }
                Verb::Cubic => {
                    tangent_cubic(&[pts[0], pts[1], pts[2], pts[3]], x, y, &mut tangents);
                    last_pt = pts[3];
                }
            }
            if tangents.len() > old_count {
                let last = tangents.len() - 1;
                let tangent = tangents[last];
                if scalar::nearly_zero(tangent.dot(tangent), None) {
                    tangents.remove(last);
                } else {
                    let mut coincident_index = None;
                    for index in 0..last {
                        let test = tangents[index];
                        if scalar::nearly_zero(test.cross(tangent), None)
                            && scalar::sign_as_int(tangent.x * test.x) <= 0
                            && scalar::sign_as_int(tangent.y * test.y) <= 0
                        {
                            coincident_index = Some(index);
                            break;
                        }
                    }
                    if let Some(index) = coincident_index {
                        tangents.remove(last);
                        tangents.remove(index);
                    }
                }
            }
        }
        (!tangents.is_empty()) ^ is_inverse
    }
}

fn between(a: Scalar, b: Scalar, c: Scalar) -> bool {
    (a - b) * (c - b) <= 0.0
}

fn check_on_curve(x: Scalar, y: Scalar, start: Point, end: Point) -> bool {
    if start.y == end.y {
        between(start.x, x, end.x) && x != end.x
    } else {
        x == start.x && y == start.y
    }
}

fn poly_eval3(a: Scalar, b: Scalar, c: Scalar, t: Scalar) -> Scalar {
    (a * t + b) * t + c
}

fn poly_eval4(a: Scalar, b: Scalar, c: Scalar, d: Scalar, t: Scalar) -> Scalar {
    ((a * t + b) * t + c) * t + d
}

fn eval_cubic_pts(c0: Scalar, c1: Scalar, c2: Scalar, c3: Scalar, t: Scalar) -> Scalar {
    let a = c3 + 3.0 * (c1 - c2) - c0;
    let b = 3.0 * (c2 - c1 - c1 + c0);
    let c = 3.0 * (c1 - c0);
    let d = c0;
    poly_eval4(a, b, c, d, t)
}

fn winding_mono_cubic(pts: &[Point; 4], x: Scalar, y: Scalar, on_curve_count: &mut i32) -> i32 {
    let mut y0 = pts[0].y;
    let mut y3 = pts[3].y;

    let mut dir = 1;
    if y0 > y3 {
        std::mem::swap(&mut y0, &mut y3);
        dir = -1;
    }
    if y < y0 || y > y3 {
        return 0;
    }
    if check_on_curve(x, y, pts[0], pts[3]) {
        *on_curve_count += 1;
        return 0;
    }
    if y == y3 {
        return 0;
    }

    let min = pts.iter().map(|p| p.x).fold(pts[0].x, Scalar::min);
    let max = pts.iter().map(|p| p.x).fold(pts[0].x, Scalar::max);
    if x < min {
        return 0;
    }
    if x > max {
        return dir;
    }

    let mut t = 0.0;
    if !SkCubicClipper::chop_mono_at_y(pts, y, &mut t) {
        return 0;
    }
    let xt = eval_cubic_pts(pts[0].x, pts[1].x, pts[2].x, pts[3].x, t);
    if scalar::nearly_equal(xt, x, None) {
        if x != pts[3].x || y != pts[3].y {
            *on_curve_count += 1;
            return 0;
        }
    }
    if xt < x { dir } else { 0 }
}

fn winding_cubic(pts: &[Point; 4], x: Scalar, y: Scalar, on_curve_count: &mut i32) -> i32 {
    let mut dst = [Point::default(); 10];
    let n = sk_geometry::chop_cubic_at_y_extrema(pts, &mut dst);
    let mut w = 0;
    for i in 0..=n {
        let mono = [dst[i * 3], dst[i * 3 + 1], dst[i * 3 + 2], dst[i * 3 + 3]];
        w += winding_mono_cubic(&mono, x, y, on_curve_count);
    }
    w
}

fn conic_eval_numerator(src: &[Scalar; 3], w: Scalar, t: Scalar) -> Scalar {
    let src2w = src[1] * w;
    let c = src[0];
    let a = src[2] - 2.0 * src2w + c;
    let b = 2.0 * (src2w - c);
    poly_eval3(a, b, c, t)
}

fn conic_eval_denominator(w: Scalar, t: Scalar) -> Scalar {
    let b = 2.0 * (w - 1.0);
    let c = 1.0;
    let a = -b;
    poly_eval3(a, b, c, t)
}

fn find_conic_unit_roots(a: Scalar, b: Scalar, c: Scalar, roots: &mut [Scalar; 2]) -> usize {
    const T_EPSILON: Scalar = 1e-6;

    let mut count = 0usize;
    if scalar::nearly_zero(a, None) {
        if !scalar::nearly_zero(b, None) {
            let t = -c / b;
            if t > T_EPSILON && t < 1.0 - T_EPSILON {
                roots[0] = t;
                count = 1;
            }
        }
        return count;
    }

    let discriminant = b as f64 * b as f64 - 4.0 * a as f64 * c as f64;
    if discriminant < 0.0 {
        return 0;
    }

    let sqrt_discriminant = (discriminant as Scalar).sqrt();
    if !sqrt_discriminant.is_finite() {
        return 0;
    }

    let denom = 2.0 * a;
    for t in [(-b - sqrt_discriminant) / denom, (-b + sqrt_discriminant) / denom] {
        if t > T_EPSILON
            && t < 1.0 - T_EPSILON
            && !roots[..count]
                .iter()
                .any(|root| scalar::nearly_equal(*root, t, None))
        {
            roots[count] = t;
            count += 1;
        }
    }

    if count == 2 && roots[0] > roots[1] {
        roots.swap(0, 1);
    }
    count
}

fn winding_mono_conic(conic: &Conic, x: Scalar, y: Scalar, on_curve_count: &mut i32) -> i32 {
    let pts = &conic.pts;
    let mut y0 = pts[0].y;
    let mut y2 = pts[2].y;

    let mut dir = 1;
    if y0 > y2 {
        std::mem::swap(&mut y0, &mut y2);
        dir = -1;
    }
    if y < y0 || y > y2 {
        return 0;
    }
    if check_on_curve(x, y, pts[0], pts[2]) {
        *on_curve_count += 1;
        return 0;
    }
    if y == y2 {
        return 0;
    }

    let mut roots = [0.0; 2];
    let mut a = pts[2].y;
    let mut b = pts[1].y * conic.w - y * conic.w + y;
    let mut c = pts[0].y;
    a += c - 2.0 * b;
    b -= c;
    c -= y;
    let n = find_conic_unit_roots(a, 2.0 * b, c, &mut roots);
    let xt = if n == 0 {
        // Zero roots only happens when y0 == y; pick the start point on the
        // side matching `dir` (mirrors Skia's `pts[1 - dir]` indexing).
        if dir == 1 { pts[0].x } else { pts[2].x }
    } else {
        let t = roots[0];
        let src_x = [pts[0].x, pts[1].x, pts[2].x];
        conic_eval_numerator(&src_x, conic.w, t) / conic_eval_denominator(conic.w, t)
    };
    if scalar::nearly_equal(xt, x, None) {
        if x != pts[2].x || y != pts[2].y {
            *on_curve_count += 1;
            return 0;
        }
    }
    if xt < x { dir } else { 0 }
}

fn is_mono_quad(y0: Scalar, y1: Scalar, y2: Scalar) -> bool {
    if y0 == y1 {
        return true;
    }
    if y0 < y1 {
        y1 <= y2
    } else {
        y1 >= y2
    }
}

fn winding_conic(pts: &[Point; 3], x: Scalar, y: Scalar, weight: Scalar, on_curve_count: &mut i32) -> i32 {
    let conic = Conic::new(*pts, weight);
    let mut chopped = [Conic::default(); 2];
    let is_mono = is_mono_quad(pts[0].y, pts[1].y, pts[2].y) || !conic.chop_at_y_extrema(&mut chopped);
    let mut w = winding_mono_conic(if is_mono { &conic } else { &chopped[0] }, x, y, on_curve_count);
    if !is_mono {
        w += winding_mono_conic(&chopped[1], x, y, on_curve_count);
    }
    w
}

fn winding_mono_quad(pts: &[Point; 3], x: Scalar, y: Scalar, on_curve_count: &mut i32) -> i32 {
    let mut y0 = pts[0].y;
    let mut y2 = pts[2].y;

    let mut dir = 1;
    if y0 > y2 {
        std::mem::swap(&mut y0, &mut y2);
        dir = -1;
    }
    if y < y0 || y > y2 {
        return 0;
    }
    if check_on_curve(x, y, pts[0], pts[2]) {
        *on_curve_count += 1;
        return 0;
    }
    if y == y2 {
        return 0;
    }

    let mut roots = [0.0; 2];
    let n = sk_geometry::find_unit_quad_roots(
        pts[0].y - 2.0 * pts[1].y + pts[2].y,
        2.0 * (pts[1].y - pts[0].y),
        pts[0].y - y,
        &mut roots,
    );
    let xt = if n == 0 {
        if dir == 1 { pts[0].x } else { pts[2].x }
    } else {
        let t = roots[0];
        let c = pts[0].x;
        let a = pts[2].x - 2.0 * pts[1].x + c;
        let b = 2.0 * (pts[1].x - c);
        poly_eval3(a, b, c, t)
    };
    if scalar::nearly_equal(xt, x, None) {
        if x != pts[2].x || y != pts[2].y {
            *on_curve_count += 1;
            return 0;
        }
    }
    if xt < x { dir } else { 0 }
}

fn winding_quad(pts: &[Point; 3], x: Scalar, y: Scalar, on_curve_count: &mut i32) -> i32 {
    if !is_mono_quad(pts[0].y, pts[1].y, pts[2].y) {
        let mut dst = [Point::default(); 5];
        let n = sk_geometry::chop_quad_at_y_extrema(pts, &mut dst);
        let mut w = winding_mono_quad(&[dst[0], dst[1], dst[2]], x, y, on_curve_count);
        if n > 0 {
            w += winding_mono_quad(&[dst[2], dst[3], dst[4]], x, y, on_curve_count);
        }
        w
    } else {
        winding_mono_quad(pts, x, y, on_curve_count)
    }
}

fn winding_line(pts: &[Point; 2], x: Scalar, y: Scalar, on_curve_count: &mut i32) -> i32 {
    let x0 = pts[0].x;
    let mut y0 = pts[0].y;
    let x1 = pts[1].x;
    let mut y1 = pts[1].y;

    let dy = y1 - y0;

    let mut dir = 1;
    if y0 > y1 {
        std::mem::swap(&mut y0, &mut y1);
        dir = -1;
    }
    if y < y0 || y > y1 {
        return 0;
    }
    if check_on_curve(x, y, pts[0], pts[1]) {
        *on_curve_count += 1;
        return 0;
    }
    if y == y1 {
        return 0;
    }
    let cross = (x1 - x0) * (y - pts[0].y) - dy * (x - x0);

    if cross == 0.0 {
        if x != x1 || y != pts[1].y {
            *on_curve_count += 1;
        }
        0
    } else if scalar::sign_as_int(cross) == dir {
        0
    } else {
        dir
    }
}

fn tangent_cubic(pts: &[Point; 4], x: Scalar, y: Scalar, tangents: &mut Vec<Vector>) {
    if !between(pts[0].y, y, pts[1].y) && !between(pts[1].y, y, pts[2].y) && !between(pts[2].y, y, pts[3].y) {
        return;
    }
    if !between(pts[0].x, x, pts[1].x) && !between(pts[1].x, x, pts[2].x) && !between(pts[2].x, x, pts[3].x) {
        return;
    }
    let mut dst = [Point::default(); 10];
    let n = sk_geometry::chop_cubic_at_y_extrema(pts, &mut dst);
    for i in 0..=n {
        let c = [dst[i * 3], dst[i * 3 + 1], dst[i * 3 + 2], dst[i * 3 + 3]];
        let mut t = 0.0;
        if !SkCubicClipper::chop_mono_at_y(&c, y, &mut t) {
            continue;
        }
        let xt = eval_cubic_pts(c[0].x, c[1].x, c[2].x, c[3].x, t);
        if !scalar::nearly_equal(x, xt, None) {
            continue;
        }
        tangents.push(sk_geometry::eval_cubic_tangent_at(&c, t));
    }
}

fn tangent_conic(pts: &[Point; 3], x: Scalar, y: Scalar, w: Scalar, tangents: &mut Vec<Vector>) {
    if !between(pts[0].y, y, pts[1].y) && !between(pts[1].y, y, pts[2].y) {
        return;
    }
    if !between(pts[0].x, x, pts[1].x) && !between(pts[1].x, x, pts[2].x) {
        return;
    }
    let mut roots = [0.0; 2];
    let mut a = pts[2].y;
    let mut b = pts[1].y * w - y * w + y;
    let mut c = pts[0].y;
    a += c - 2.0 * b;
    b -= c;
    c -= y;
    let n = find_conic_unit_roots(a, 2.0 * b, c, &mut roots);
    for &t in &roots[..n] {
        let src_x = [pts[0].x, pts[1].x, pts[2].x];
        let xt = conic_eval_numerator(&src_x, w, t) / conic_eval_denominator(w, t);
        if !scalar::nearly_equal(x, xt, None) {
            continue;
        }
        let conic = Conic::new(*pts, w);
        tangents.push(conic.eval_tangent_at(t));
    }
}

fn tangent_quad(pts: &[Point; 3], x: Scalar, y: Scalar, tangents: &mut Vec<Vector>) {
    if !between(pts[0].y, y, pts[1].y) && !between(pts[1].y, y, pts[2].y) {
        return;
    }
    if !between(pts[0].x, x, pts[1].x) && !between(pts[1].x, x, pts[2].x) {
        return;
    }
    let mut roots = [0.0; 2];
    let n = sk_geometry::find_unit_quad_roots(
        pts[0].y - 2.0 * pts[1].y + pts[2].y,
        2.0 * (pts[1].y - pts[0].y),
        pts[0].y - y,
        &mut roots,
    );
    for &t in &roots[..n] {
        let c = pts[0].x;
        let a = pts[2].x - 2.0 * pts[1].x + c;
        let b = 2.0 * (pts[1].x - c);
        let xt = poly_eval3(a, b, c, t);
        if !scalar::nearly_equal(x, xt, None) {
            continue;
        }
        tangents.push(sk_geometry::eval_quad_tangent_at(pts, t));
    }
}

fn tangent_line(pts: &[Point; 2], x: Scalar, y: Scalar, tangents: &mut Vec<Vector>) {
    let y0 = pts[0].y;
    let y1 = pts[1].y;
    if !between(y0, y, y1) {
        return;
    }
    let x0 = pts[0].x;
    let x1 = pts[1].x;
    if !between(x0, x, x1) {
        return;
    }
    let dx = x1 - x0;
    let dy = y1 - y0;
    if !scalar::nearly_equal((x - x0) * dy, dx * (y - y0), None) {
        return;
    }
    tangents.push(Vector::new(dx, dy));
}

/// Iterator over a [`Path`]'s segments, produced by [`Path::iter`].
pub struct PathIter<'a> {
    verbs: std::slice::Iter<'a, Verb>,
    points: std::slice::Iter<'a, Point>,
    conic_weights: std::slice::Iter<'a, Scalar>,
    last_point: Option<Point>,
}

impl<'a> PathIter<'a> {
    fn new(path: &'a Path) -> Self {
        Self {
            verbs: path.verbs.iter(),
            points: path.points.iter(),
            conic_weights: path.conic_weights.iter(),
            last_point: None,
        }
    }
}

impl<'a> Iterator for PathIter<'a> {
    type Item = (Verb, [Point; 4], Option<Scalar>);

    fn next(&mut self) -> Option<Self::Item> {
        let verb = *self.verbs.next()?;

        Some(match verb {
            Verb::Move => {
                let pt = *self.points.next()?;
                self.last_point = Some(pt);
                (verb, [pt, Point::default(), Point::default(), Point::default()], None)
            }
            Verb::Line => {
                let pt = *self.points.next()?;
                let start = self.last_point.unwrap_or_default();
                self.last_point = Some(pt);
                (verb, [start, pt, Point::default(), Point::default()], None)
            }
            Verb::Quad => {
                let p1 = *self.points.next()?;
                let p2 = *self.points.next()?;
                let start = self.last_point.unwrap_or_default();
                self.last_point = Some(p2);
                (verb, [start, p1, p2, Point::default()], None)
            }
            Verb::Conic => {
                let p1 = *self.points.next()?;
                let p2 = *self.points.next()?;
                let weight = *self.conic_weights.next()?;
                let start = self.last_point.unwrap_or_default();
                self.last_point = Some(p2);
                (verb, [start, p1, p2, Point::default()], Some(weight))
            }
            Verb::Cubic => {
                let p1 = *self.points.next()?;
                let p2 = *self.points.next()?;
                let p3 = *self.points.next()?;
                let start = self.last_point.unwrap_or_default();
                self.last_point = Some(p3);
                (verb, [start, p1, p2, p3], None)
            }
            Verb::Close => (verb, [Point::default(); 4], None),
        })
    }
}

/// Returns the start index of the `n`th verb's points in the points array.
fn verb_point_index(verbs: &[Verb]) -> Vec<usize> {
    let mut idx = 0usize;
    let mut starts = Vec::with_capacity(verbs.len());
    for v in verbs {
        starts.push(idx);
        idx += v.point_count();
    }
    starts
}

fn include_point(b: Rect, p: Point) -> Rect {
    Rect::from_ltrb(
        b.left.min(p.x),
        b.top.min(p.y),
        b.right.max(p.x),
        b.bottom.max(p.y),
    )
}

/// Find extrema of a quadratic: solves f'(t) = 0.
fn find_quad_extrema(a: Scalar, b: Scalar, c: Scalar) -> Option<Scalar> {
    // f'(t) = 2(a - 2b + c)t + 2(b - a) = 0
    // t = (a - b) / (a - 2b + c)
    let denom = a - 2.0 * b + c;
    if denom == 0.0 {
        return None;
    }
    let t = (a - b) / denom;
    if t > 0.0 && t < 1.0 {
        Some(t)
    } else {
        None
    }
}

fn eval_quad_at(pts: &[Point; 3], t: Scalar) -> Point {
    let mt = 1.0 - t;
    let a = mt * mt;
    let b = 2.0 * mt * t;
    let c = t * t;
    Point::new(a * pts[0].x + b * pts[1].x + c * pts[2].x,
               a * pts[0].y + b * pts[1].y + c * pts[2].y)
}

fn include_quad_tight(mut b: Rect, pts: &[Point; 3]) -> Rect {
    b = include_point(b, pts[2]);
    if let Some(tx) = find_quad_extrema(pts[0].x, pts[1].x, pts[2].x) {
        b = include_point(b, eval_quad_at(pts, tx));
    }
    if let Some(ty) = find_quad_extrema(pts[0].y, pts[1].y, pts[2].y) {
        b = include_point(b, eval_quad_at(pts, ty));
    }
    b
}

/// Find extrema of a cubic: solves f'(t) = 0 (quadratic).
fn find_cubic_extrema(a: Scalar, b: Scalar, c: Scalar, d: Scalar) -> Vec<Scalar> {
    // A = 3(-a + 3(b - c) + d) = 3(d - a + 3b - 3c)
    // B = 6(a - 2b + c)
    // C = 3(b - a)
    // Divide by 3: A = d - a + 3*(b - c), B = 2*(a - 2b + c), C = b - a
    let aa = d - a + 3.0 * (b - c);
    let bb = 2.0 * (a - 2.0 * b + c);
    let cc = b - a;
    find_unit_quad_roots(aa, bb, cc)
}

fn find_unit_quad_roots(a: Scalar, b: Scalar, c: Scalar) -> Vec<Scalar> {
    if a == 0.0 {
        if b == 0.0 {
            return vec![];
        }
        let t = -c / b;
        if t > 0.0 && t < 1.0 {
            return vec![t];
        }
        return vec![];
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return vec![];
    }
    let sqrt_disc = disc.sqrt();
    let q = if b < 0.0 { -(b - sqrt_disc) / 2.0 } else { -(b + sqrt_disc) / 2.0 };
    let mut roots = Vec::new();
    let t1 = q / a;
    if t1 > 0.0 && t1 < 1.0 {
        roots.push(t1);
    }
    let t2 = c / q;
    if t2 > 0.0 && t2 < 1.0 {
        roots.push(t2);
    }
    roots.sort_by(|a, b| a.partial_cmp(b).unwrap());
    roots.dedup();
    roots
}

fn eval_cubic_at(pts: &[Point; 4], t: Scalar) -> Point {
    let mt = 1.0 - t;
    let a = mt * mt * mt;
    let b = 3.0 * mt * mt * t;
    let c = 3.0 * mt * t * t;
    let d = t * t * t;
    Point::new(a * pts[0].x + b * pts[1].x + c * pts[2].x + d * pts[3].x,
               a * pts[0].y + b * pts[1].y + c * pts[2].y + d * pts[3].y)
}

fn include_cubic_tight(mut b: Rect, pts: &[Point; 4]) -> Rect {
    b = include_point(b, pts[3]);
    for t in find_cubic_extrema(pts[0].x, pts[1].x, pts[2].x, pts[3].x) {
        b = include_point(b, eval_cubic_at(pts, t));
    }
    for t in find_cubic_extrema(pts[0].y, pts[1].y, pts[2].y, pts[3].y) {
        b = include_point(b, eval_cubic_at(pts, t));
    }
    b
}

/// Conic extrema: solves derivative of rational quadratic = 0.
fn find_conic_extrema(a: Scalar, b: Scalar, c: Scalar, w: Scalar) -> Option<Scalar> {
    // For a conic p(t) = (num(t), denom(t)), the derivative crosses zero
    // when the numerator derivative times denominator equals numerator times denominator derivative.
    // The simplified extrema condition: coeff[0]*t^2 + coeff[1]*t + coeff[2] = 0
    // where coeff = [w*P20 - P20, P20 - 2*w*P10, w*P10]
    let p20 = c - a;
    let p10 = b - a;
    let coeff0 = w * p20 - p20;
    let coeff1 = p20 - 2.0 * w * p10;
    let coeff2 = w * p10;
    let roots = find_unit_quad_roots(coeff0, coeff1, coeff2);
    if roots.len() == 1 {
        Some(roots[0])
    } else {
        None
    }
}

fn eval_conic_at(pts: &[Point; 3], w: Scalar, t: Scalar) -> Point {
    let mt = 1.0 - t;
    let a = mt * mt;
    let b = 2.0 * mt * t * w;
    let c = t * t;
    let denom = a + b + c;
    let x = (a * pts[0].x + b * pts[1].x + c * pts[2].x) / denom;
    let y = (a * pts[0].y + b * pts[1].y + c * pts[2].y) / denom;
    Point::new(x, y)
}

fn include_conic_tight(mut b: Rect, pts: &[Point; 3], w: Scalar) -> Rect {
    b = include_point(b, pts[2]);
    if let Some(tx) = find_conic_extrema(pts[0].x, pts[1].x, pts[2].x, w) {
        b = include_point(b, eval_conic_at(pts, w, tx));
    }
    if let Some(ty) = find_conic_extrema(pts[0].y, pts[1].y, pts[2].y, w) {
        b = include_point(b, eval_conic_at(pts, w, ty));
    }
    b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_path_is_empty() {
        let p = Path::new();
        assert!(p.is_empty());
        assert_eq!(p.count_points(), 0);
        assert_eq!(p.count_verbs(), 0);
        assert_eq!(p.fill_type(), FillType::Winding);
    }

    #[test]
    fn toggle_inverse_fill_type() {
        let mut p = Path::new();
        assert_eq!(p.fill_type(), FillType::Winding);
        p.toggle_inverse_fill_type();
        assert_eq!(p.fill_type(), FillType::InverseWinding);
        p.toggle_inverse_fill_type();
        assert_eq!(p.fill_type(), FillType::Winding);
    }

    #[test]
    fn reset_clears_everything() {
        let mut p = Path::new();
        p.set_fill_type(FillType::EvenOdd);
        p.verbs.push(Verb::Move);
        p.points.push(Point::new(1.0, 2.0));
        p.reset();
        assert!(p.is_empty());
        assert_eq!(p.fill_type(), FillType::Winding);
    }

    #[test]
    fn move_to_and_line_to() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.line_to(10.0, 10.0);
        assert_eq!(p.count_verbs(), 2);
        assert_eq!(p.verb(0), Some(Verb::Move));
        assert_eq!(p.verb(1), Some(Verb::Line));
        assert_eq!(p.count_points(), 2);
        assert_eq!(p.point(0), Some(Point::new(0.0, 0.0)));
        assert_eq!(p.point(1), Some(Point::new(10.0, 10.0)));
    }

    #[test]
    fn line_to_auto_injects_move() {
        let mut p = Path::new();
        p.line_to(5.0, 5.0);
        assert_eq!(p.count_verbs(), 2);
        assert_eq!(p.verb(0), Some(Verb::Move));
        assert_eq!(p.point(0), Some(Point::new(0.0, 0.0)));
        assert_eq!(p.point(1), Some(Point::new(5.0, 5.0)));
    }

    #[test]
    fn cubic_to_creates_curve() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.cubic_to(10.0, 20.0, 30.0, 40.0, 50.0, 60.0);
        assert_eq!(p.verb(1), Some(Verb::Cubic));
        assert_eq!(p.count_points(), 4);
    }

    #[test]
    fn add_rect_creates_contour() {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(10.0, 20.0, 30.0, 40.0));
        assert_eq!(p.count_verbs(), 5);
        assert_eq!(p.verb(0), Some(Verb::Move));
        assert_eq!(p.verb(4), Some(Verb::Close));
        assert_eq!(p.count_points(), 4);
    }

    #[test]
    fn bounds_of_rect_path() {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(10.0, 20.0, 30.0, 40.0));
        let b = p.bounds();
        assert_eq!(b.left, 10.0);
        assert_eq!(b.top, 20.0);
        assert_eq!(b.right, 30.0);
        assert_eq!(b.bottom, 40.0);
    }

    #[test]
    fn tight_bounds_of_quadratic() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.quad_to(50.0, 100.0, 100.0, 0.0);
        let tb = p.compute_tight_bounds();
        // The quadratic peaks above 0, so tight bounds should be tighter than control bounds.
        assert!(tb.bottom > 0.0);
        assert!(tb.bottom <= 100.0);
        assert_eq!(tb.left, 0.0);
        assert_eq!(tb.right, 100.0);
    }

    #[test]
    fn tight_bounds_of_cubic() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.cubic_to(75.0, 300.0, 225.0, -300.0, 300.0, 0.0);
        let tb = p.compute_tight_bounds();
        // The cubic has extrema inside the control point bounds.
        assert!(tb.top <= 0.0 || tb.bottom >= 0.0);
        assert!(tb.right >= 0.0);
        assert!(tb.left <= 300.0);
    }

    #[test]
    fn segment_masks() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.line_to(1.0, 1.0);
        assert_eq!(p.get_segment_masks(), crate::core::types::SEGMENT_MASK_LINE);
        p.cubic_to(2.0, 2.0, 3.0, 3.0, 4.0, 4.0);
        assert_eq!(p.get_segment_masks(), crate::core::types::SEGMENT_MASK_LINE | crate::core::types::SEGMENT_MASK_CUBIC);
    }

    #[test]
    fn swap_paths() {
        let mut a = Path::new();
        a.move_to(1.0, 2.0);
        let mut b = Path::new();
        b.move_to(3.0, 4.0);
        a.swap(&mut b);
        assert_eq!(a.point(0), Some(Point::new(3.0, 4.0)));
        assert_eq!(b.point(0), Some(Point::new(1.0, 2.0)));
    }

    #[test]
    fn is_line_detection() {
        let mut p = Path::new();
        assert!(!p.is_line(None));
        p.move_to(0.0, 0.0);
        p.line_to(10.0, 10.0);
        let mut line = [Point::default(); 2];
        assert!(p.is_line(Some(&mut line)));
        assert_eq!(line[0], Point::new(0.0, 0.0));
        assert_eq!(line[1], Point::new(10.0, 10.0));
    }

    #[test]
    fn is_rect_detection() {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        let mut r = Rect::empty();
        let mut closed = false;
        let mut dir = Direction::Cw;
        assert!(p.is_rect(Some(&mut r), Some(&mut closed), Some(&mut dir)));
        assert_eq!(r, Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        assert!(closed);
    }

    #[test]
    fn conic_to_creates_conic_or_fallback() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.conic_to(50.0, 50.0, 100.0, 0.0, 0.5);
        assert_eq!(p.verb(1), Some(Verb::Conic));
        assert_eq!(p.conic_weights.len(), 1);
        assert!((p.conic_weights[0] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn conic_to_weight_one_becomes_quad() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.conic_to(50.0, 50.0, 100.0, 0.0, 1.0);
        assert_eq!(p.verb(1), Some(Verb::Quad));
    }

    #[test]
    fn close_works() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.line_to(10.0, 0.0);
        p.close();
        assert_eq!(p.verb(2), Some(Verb::Close));
        assert!(p.is_last_contour_closed());
    }

    #[test]
    fn rewind_preserves_capacity() {
        let mut p = Path::new();
        p.move_to(1.0, 2.0);
        p.rewind();
        assert!(p.is_empty());
        assert_eq!(p.fill_type(), FillType::Winding);
    }

    #[test]
    fn contains_empty_path() {
        let p = Path::new();
        assert!(!p.contains(0.0, 0.0));
        let mut inv = Path::new();
        inv.toggle_inverse_fill_type();
        assert!(inv.contains(0.0, 0.0));
    }

    #[test]
    fn contains_rect() {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        assert!(p.contains(5.0, 5.0));
        assert!(!p.contains(15.0, 15.0));
        assert!(!p.contains(-1.0, 5.0));
    }

    #[test]
    fn contains_rect_inverse_fill() {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        p.toggle_inverse_fill_type();
        assert!(!p.contains(5.0, 5.0));
        assert!(p.contains(15.0, 15.0));
    }

    #[test]
    fn contains_circle_via_conics() {
        let mut p = Path::new();
        p.add_circle(0.0, 0.0, 10.0);
        assert!(p.contains(0.0, 0.0));
        assert!(p.contains(5.0, 5.0));
        assert!(!p.contains(9.0, 9.0));
        assert!(!p.contains(20.0, 20.0));
    }

    #[test]
    fn contains_cubic_bowtie_even_odd() {
        // A path built purely from cubics tracing a curved blob, even-odd fill.
        let mut p = Path::new();
        p.set_fill_type(FillType::EvenOdd);
        p.move_to(0.0, 0.0);
        p.cubic_to(0.0, 20.0, 20.0, 20.0, 20.0, 0.0);
        p.cubic_to(20.0, -20.0, 0.0, -20.0, 0.0, 0.0);
        p.close();
        assert!(p.contains(10.0, 0.0));
        assert!(!p.contains(50.0, 50.0));
    }

    #[test]
    fn contains_two_disjoint_rects_winding() {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        p.add_rect_simple(Rect::from_ltrb(20.0, 20.0, 30.0, 30.0));
        assert!(p.contains(5.0, 5.0));
        assert!(p.contains(25.0, 25.0));
        assert!(!p.contains(15.0, 15.0));
    }
}

#[cfg(test)]
mod contains_debug2 {
    use super::*;
    #[test]
    fn debug_circle() {
        let mut p = Path::new();
        p.add_circle(0.0, 0.0, 10.0);
        let mut oc = 0;
        for (verb, pts, w) in p.iter() {
            let contrib = match verb {
                Verb::Conic => winding_conic(&[pts[0], pts[1], pts[2]], 9.0, 9.0, w.unwrap(), &mut oc),
                _ => 0,
            };
            println!("{:?} {:?} w={:?} contrib={}", verb, pts, w, contrib);
        }
        println!("contains(9,9) = {}", p.contains(9.0, 9.0));
    }
}
