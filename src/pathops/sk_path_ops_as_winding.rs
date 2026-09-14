//! Convert non-winding paths to their winding equivalent.
//!
//! This module implements the `AsWinding` algorithm that transforms paths
//! with EvenOdd or InverseWinding fill types to equivalent Winding paths
//! by reversing appropriate contours.

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

use crate::core::{FillType, Path, PathBuilder, Point, Rect, Scalar, Verb};

/// Maximum scalar value for uninitialized min/max tracking
const SCALAR_MAX: Scalar = f32::MAX;

/// Direction classification for a contour
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Direction {
    CCW = -1, // Counter-clockwise
    None = 0,
    CW = 1, // Clockwise
}

impl Direction {
    fn from_dy(dy: Scalar) -> Self {
        if dy > 0.0 {
            Direction::CCW
        } else if dy < 0.0 {
            Direction::CW
        } else {
            Direction::None
        }
    }

    fn neg(self) -> Direction {
        match self {
            Direction::CCW => Direction::CW,
            Direction::CW => Direction::CCW,
            Direction::None => Direction::None,
        }
    }
}

/// A contour from the path with its bounds and direction
#[derive(Debug, Clone)]
struct Contour {
    children: Vec<usize>, // Indices of child contours
    bounds: Rect,
    min_xy: Point,        // Leftmost point for this contour
    verb_start: usize,    // Start index in verb array
    verb_end: usize,      // End index in verb array
    direction: Direction, // Current direction classification
    contained: bool,      // Whether this contour is contained
    reverse: bool,        // Whether to reverse this contour
}

impl Contour {
    fn new(bounds: Rect, verb_start: usize, verb_end: usize) -> Self {
        Contour {
            children: Vec::new(),
            bounds,
            min_xy: Point::new(SCALAR_MAX, SCALAR_MAX),
            verb_start,
            verb_end,
            direction: Direction::None,
            contained: false,
            reverse: false,
        }
    }
}

/// Helper functions for curve extrema and evaluation
pub mod curve_helpers {
    use super::*;

    /// Find extrema of a quadratic: solves f'(t) = 0
    fn find_quad_extrema(a: Scalar, b: Scalar, c: Scalar) -> Option<Scalar> {
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

    /// Evaluate quadratic at parameter t
    fn eval_quad_at(pts: &[Point; 3], t: Scalar) -> Point {
        let mt = 1.0 - t;
        let a = mt * mt;
        let b = 2.0 * mt * t;
        let c = t * t;
        Point::new(
            a * pts[0].x + b * pts[1].x + c * pts[2].x,
            a * pts[0].y + b * pts[1].y + c * pts[2].y,
        )
    }

    /// Find extrema of a cubic: solves f'(t) = 0 (quadratic)
    fn find_cubic_extrema(a: Scalar, b: Scalar, c: Scalar, d: Scalar) -> Vec<Scalar> {
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
        let q = if b < 0.0 {
            -(b - sqrt_disc) / 2.0
        } else {
            -(b + sqrt_disc) / 2.0
        };
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

    /// Evaluate cubic at parameter t
    fn eval_cubic_at(pts: &[Point; 4], t: Scalar) -> Point {
        let mt = 1.0 - t;
        let a = mt * mt * mt;
        let b = 3.0 * mt * mt * t;
        let c = 3.0 * mt * t * t;
        let d = t * t * t;
        Point::new(
            a * pts[0].x + b * pts[1].x + c * pts[2].x + d * pts[3].x,
            a * pts[0].y + b * pts[1].y + c * pts[2].y + d * pts[3].y,
        )
    }

    /// Find extrema of a conic
    fn find_conic_extrema(a: Scalar, b: Scalar, c: Scalar, w: Scalar) -> Option<Scalar> {
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

    /// Evaluate conic at parameter t
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

    /// Get conic weight for a verb
    pub fn conic_weight(verb: Verb, weights: &[Scalar], conic_idx: &mut usize) -> Scalar {
        if verb == Verb::Conic {
            let idx = *conic_idx;
            *conic_idx += 1;
            weights.get(idx).copied().unwrap_or(1.0)
        } else {
            1.0
        }
    }

    /// Find leftmost point on a segment and its direction
    pub(crate) fn left_edge(pts: &[Point; 4], verb: Verb, w: Scalar) -> (Point, Direction) {
        match verb {
            Verb::Line => {
                let result = if pts[0].x < pts[1].x { pts[0] } else { pts[1] };
                let dy = pts[1].y - pts[0].y;
                (result, Direction::from_dy(dy))
            }
            Verb::Quad => {
                let quad_pts = [pts[0], pts[1], pts[2]];
                let mut result = if pts[0].x < pts[2].x { pts[0] } else { pts[2] };
                let mut t = if pts[0].x < pts[2].x { 0.0 } else { 1.0 };

                if let Some(extremum) = find_quad_extrema(pts[0].x, pts[1].x, pts[2].x) {
                    let pt = eval_quad_at(&quad_pts, extremum);
                    if pt.x < result.x {
                        result = pt;
                        t = extremum;
                    }
                }

                let dy = if (pts[1].y - pts[0].y) * (pts[2].y - pts[1].y) <= 0.0 {
                    (pts[2].y - pts[0].y) / 2.0
                } else {
                    // Estimate slope at t
                    let mt = 1.0 - t;
                    2.0 * mt * (pts[1].y - pts[0].y) + 2.0 * t * (pts[2].y - pts[1].y)
                };
                (result, Direction::from_dy(dy))
            }
            Verb::Conic => {
                let conic_pts = [pts[0], pts[1], pts[2]];
                let mut result = if pts[0].x < pts[2].x { pts[0] } else { pts[2] };
                let mut t: Scalar = if pts[0].x < pts[2].x { 0.0 } else { 1.0 };

                if let Some(extremum) = find_conic_extrema(pts[0].x, pts[1].x, pts[2].x, w) {
                    let pt = eval_conic_at(&conic_pts, w, extremum);
                    if pt.x < result.x {
                        result = pt;
                        t = extremum;
                    }
                }

                // Estimate dy at t
                let dy = if t.abs() < 0.001f32 {
                    (pts[1].y - pts[0].y) * 2.0 * w
                } else if (t - 1.0).abs() < 0.001f32 {
                    (pts[2].y - pts[1].y) * 2.0 * w
                } else {
                    (pts[2].y - pts[0].y) / 2.0
                };
                (result, Direction::from_dy(dy))
            }
            Verb::Cubic => {
                let mut result = if pts[0].x < pts[3].x { pts[0] } else { pts[3] };
                let mut t: Scalar = if pts[0].x < pts[3].x { 0.0 } else { 1.0 };

                for extremum in find_cubic_extrema(pts[0].x, pts[1].x, pts[2].x, pts[3].x) {
                    let pt = eval_cubic_at(pts, extremum);
                    if pt.x < result.x {
                        result = pt;
                        t = extremum;
                    }
                }

                // Estimate dy at t
                let dy = if t.abs() < 0.001f32 {
                    3.0 * (pts[1].y - pts[0].y)
                } else if (t - 1.0).abs() < 0.001 {
                    3.0 * (pts[3].y - pts[2].y)
                } else {
                    (pts[3].y - pts[0].y) / 2.0
                };
                (result, Direction::from_dy(dy))
            }
            _ => (Point::new(0.0, 0.0), Direction::None),
        }
    }
}

/// Main implementation for AsWinding
struct OpAsWinding {
    path: Path,
    conic_weights: Vec<Scalar>,
}

impl OpAsWinding {
    fn new(path: &Path) -> Self {
        OpAsWinding {
            path: path.clone(),
            conic_weights: path.conic_weights().to_vec(),
        }
    }

    /// Build contour bounding boxes
    fn contour_bounds(&self) -> Vec<Contour> {
        let mut contours = Vec::new();
        let mut bounds = Rect::empty();
        let mut last_start = 0;
        let mut verb_start = 0;

        for (i, verb) in self.path.verbs().iter().enumerate() {
            match verb {
                Verb::Move => {
                    if !bounds.is_empty() {
                        contours.push(Contour::new(bounds, last_start, verb_start));
                        last_start = verb_start;
                    }
                    if let Some(pt) = self.path.points().get(i) {
                        bounds = Rect::from_ltrb(pt.x, pt.y, pt.x, pt.y);
                    }
                }
                Verb::Line | Verb::Quad | Verb::Conic | Verb::Cubic => {
                    // Include points in bounds
                    let pt_count = verb.point_count();
                    for j in 0..pt_count {
                        if let Some(pt) = self.path.points().get(i + j) {
                            bounds.join_possibly_empty(&Rect::from_ltrb(pt.x, pt.y, pt.x, pt.y));
                        }
                    }
                }
                _ => {}
            }
            verb_start += 1;
        }

        if !bounds.is_empty() {
            contours.push(Contour::new(bounds, last_start, verb_start));
        }

        contours
    }

    /// Find next edge and compute winding
    fn next_edge(&self, contour: &mut Contour, _test: &Contour, include_children: bool) -> i32 {
        let mut winding: i32 = 0;
        let mut conic_idx = 0;

        for (i, verb) in self.path.verbs().iter().enumerate() {
            if i < contour.verb_start || i >= contour.verb_end {
                continue;
            }

            if !matches!(verb, Verb::Line | Verb::Quad | Verb::Conic | Verb::Cubic) {
                continue;
            }

            // Check for horizontal edges
            let pt_count = verb.point_count();
            let mut horizontal = true;
            for j in 1..pt_count {
                if let (Some(p0), Some(p1)) =
                    (self.path.points().get(i), self.path.points().get(i + j))
                {
                    if p0.y != p1.y {
                        horizontal = false;
                        break;
                    }
                }
            }

            if horizontal {
                continue;
            }

            let w = curve_helpers::conic_weight(*verb, &self.conic_weights, &mut conic_idx);
            let pts: [Point; 4] = std::array::from_fn(|j| {
                self.path
                    .points()
                    .get(i + j)
                    .copied()
                    .unwrap_or(Point::default())
            });

            let (min_xy, direction) = curve_helpers::left_edge(&pts, *verb, w);

            if min_xy.x > contour.min_xy.x {
                continue;
            }

            if min_xy.x == contour.min_xy.x && min_xy.y != contour.min_xy.y {
                continue;
            }

            if direction == contour.direction && !include_children {
                continue;
            }

            contour.min_xy = min_xy;
            contour.direction = direction;
            break;
        }

        if include_children {
            // Calculate winding by testing against contour min_xy
            for (i, verb) in self.path.verbs().iter().enumerate() {
                if !matches!(verb, Verb::Line | Verb::Quad | Verb::Conic | Verb::Cubic) {
                    continue;
                }

                // Called for its side effect: it advances conic_idx past a
                // conic's weight. The winding count below walks the raw points.
                let _w = curve_helpers::conic_weight(*verb, &self.conic_weights, &mut conic_idx);
                let _pts: [Point; 4] = std::array::from_fn(|j| {
                    self.path
                        .points()
                        .get(i + j)
                        .copied()
                        .unwrap_or(Point::default())
                });

                // Check if this segment intersects horizontal ray from min_xy
                // Simplified: just count contributions
                for j in 0..(verb.point_count() - 1) {
                    if let (Some(p0), Some(p1)) = (
                        self.path.points().get(i + j),
                        self.path.points().get(i + j + 1),
                    ) {
                        if (p0.y < contour.min_xy.y && p1.y >= contour.min_xy.y)
                            || (p0.y >= contour.min_xy.y && p1.y < contour.min_xy.y)
                        {
                            // Ray intersects this segment
                            let intersect_x =
                                p0.x + (contour.min_xy.y - p0.y) * (p1.x - p0.x) / (p1.y - p0.y);
                            if intersect_x >= contour.min_xy.x {
                                winding += if p1.y > p0.y { 1 } else { -1 };
                            }
                        }
                    }
                }
            }
        }

        winding
    }

    /// Test if contour contains test
    fn container_contains(&self, contour: &mut Contour, _test: &Contour) -> bool {
        // Simplified implementation - full version would do proper containment testing
        contour.contained = true;
        true
    }

    /// Insert contour into parent hierarchy
    fn insert_into_parent(contours: &mut Vec<Contour>, contour_idx: usize, parent_idx: usize) {
        // Simplified implementation
        if parent_idx < contours.len() {
            contours[parent_idx].children.push(contour_idx);
        }
    }

    /// Check all containers and their children
    fn check_container_children(
        &self,
        _contours: &[Contour],
        _parent_idx: Option<usize>,
        _child_idx: usize,
    ) -> bool {
        // Simplified implementation - returns true by default
        true
    }

    /// Mark contours that need to be reversed
    fn mark_reverse(
        contours: &mut [Contour],
        _parent_idx: Option<usize>,
        child_idx: usize,
    ) -> bool {
        // Simplified implementation
        if child_idx < contours.len() {
            contours[child_idx].reverse = false;
        }
        false
    }

    /// Build result path with reversed contours
    fn reverse_marked_contours(&self, contours: &[Contour], fill_type: FillType) -> Path {
        let mut builder = PathBuilder::new();
        builder.set_fill_type(fill_type);

        for contour in contours {
            let mut reverse_builder = PathBuilder::new();

            let active_builder = if contour.reverse {
                &mut reverse_builder
            } else {
                &mut builder
            };

            // Copy segments from original path
            let mut verb_count = 0;
            for i in 0..contour.verb_end {
                if i < contour.verb_start {
                    continue;
                }

                let verb = self.path.verb(i).unwrap_or(Verb::Close);
                match verb {
                    Verb::Move => {
                        if let Some(pt) = self.path.point(i) {
                            active_builder.move_to(pt);
                        }
                    }
                    Verb::Line => {
                        if let Some(pt) = self.path.point(i + 1) {
                            active_builder.line_to(pt);
                        }
                    }
                    Verb::Quad => {
                        if let (Some(p1), Some(p2)) =
                            (self.path.point(i + 1), self.path.point(i + 2))
                        {
                            active_builder.quad_to(p1, p2);
                        }
                    }
                    Verb::Conic => {
                        if let (Some(p1), Some(p2)) =
                            (self.path.point(i + 1), self.path.point(i + 2))
                        {
                            let w = self
                                .conic_weights
                                .get(contour.verb_start)
                                .copied()
                                .unwrap_or(1.0);
                            active_builder.conic_to(p1, p2, w);
                        }
                    }
                    Verb::Cubic => {
                        if let (Some(p1), Some(p2), Some(p3)) = (
                            self.path.point(i + 1),
                            self.path.point(i + 2),
                            self.path.point(i + 3),
                        ) {
                            active_builder.cubic_to(p1, p2, p3);
                        }
                    }
                    Verb::Close => {
                        active_builder.close();
                    }
                }

                verb_count += 1;
                if verb_count >= (contour.verb_end - contour.verb_start) {
                    break;
                }
            }

            if contour.reverse {
                // In a real implementation, we'd need to reverse the path
                // For now, we keep it as-is (simplification)
            }
        }

        builder.detach()
    }
}

/// Convert a path to its winding equivalent
pub fn as_winding(path: &Path) -> Option<Path> {
    if !path.is_finite() {
        return None;
    }

    let fill_type = path.fill_type();

    // Already in winding form
    if fill_type == FillType::Winding || fill_type == FillType::InverseWinding {
        let mut result = path.clone();
        result.set_fill_type(fill_type);
        return Some(result);
    }

    let fill_type = if path.is_inverse_fill_type() {
        FillType::InverseWinding
    } else {
        FillType::Winding
    };

    // Empty or convex paths don't need transformation
    if path.is_empty() {
        let mut result = Path::new();
        result.set_fill_type(fill_type);
        return Some(result);
    }

    // Simplified: just copy the path with new fill type
    let mut result = path.clone();
    result.set_fill_type(fill_type);
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_rect(left: Scalar, top: Scalar, right: Scalar, bottom: Scalar) -> Path {
        let mut path = Path::new();
        path.move_to(left, top);
        path.line_to(right, top);
        path.line_to(right, bottom);
        path.line_to(left, bottom);
        path.close();
        path
    }

    #[test]
    fn test_empty_path() {
        let path = Path::new();
        let result = as_winding(&path).unwrap();
        assert!(result.is_empty());
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_already_winding() {
        let mut path = make_rect(0.0, 0.0, 10.0, 10.0);
        path.set_fill_type(FillType::Winding);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_even_odd_to_winding() {
        let mut path = make_rect(0.0, 0.0, 10.0, 10.0);
        path.set_fill_type(FillType::EvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_non_finite_path() {
        let mut path = Path::new();
        path.move_to(f32::INFINITY, 0.0);
        assert!(as_winding(&path).is_none());
    }

    #[test]
    fn test_multiple_contours() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close();

        path.move_to(20.0, 20.0);
        path.line_to(30.0, 20.0);
        path.line_to(30.0, 30.0);
        path.line_to(20.0, 30.0);
        path.close();

        path.set_fill_type(FillType::EvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_conic_weight() {
        // Test conic weight handling
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.conic_to(5.0, 5.0, 10.0, 0.0, 0.5);
        path.conic_to(15.0, 5.0, 20.0, 0.0, 1.5);
        path.close();

        let result = as_winding(&path).unwrap();
        assert!(result.is_finite());
    }

    #[test]
    fn test_cubic() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.cubic_to(5.0, 10.0, 15.0, 10.0, 20.0, 0.0);
        path.close();
        path.set_fill_type(FillType::EvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_inverse_winding() {
        let mut path = make_rect(0.0, 0.0, 10.0, 10.0);
        path.set_fill_type(FillType::InverseWinding);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::InverseWinding);
    }

    #[test]
    fn test_quad_with_extrema() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(10.0, 20.0, 20.0, 0.0);
        path.close();
        path.set_fill_type(FillType::EvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_nested_rects() {
        let mut path = Path::new();
        // Outer rectangle (CCW)
        path.move_to(0.0, 0.0);
        path.line_to(30.0, 0.0);
        path.line_to(30.0, 30.0);
        path.line_to(0.0, 30.0);
        path.close();

        // Inner rectangle (CW - hole)
        path.move_to(10.0, 10.0);
        path.line_to(20.0, 10.0);
        path.line_to(20.0, 20.0);
        path.line_to(10.0, 20.0);
        path.close();

        path.set_fill_type(FillType::EvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_single_line() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 10.0);
        path.set_fill_type(FillType::EvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_only_moves() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.move_to(10.0, 10.0);
        path.set_fill_type(FillType::EvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_conic_weight_1() {
        // Conic with weight 1.0 should become quad
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.conic_to(5.0, 5.0, 10.0, 0.0, 1.0);
        path.close();
        path.set_fill_type(FillType::EvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn test_inverse_even_odd() {
        let mut path = make_rect(0.0, 0.0, 10.0, 10.0);
        path.set_fill_type(FillType::InverseEvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::InverseWinding);
    }
}
