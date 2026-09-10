//! Accumulates a series of path operations, optimized for unioning many paths.
//!
//! Port of Skia's `SkOpBuilder.cpp`.
//!
//! The point of this type is [`OpBuilder::resolve`]. Replaying `op(result,
//! next, Union)` N-1 times runs the whole boolean engine N-1 times, and each
//! pass reintroduces its own numerical error into the input of the next. When
//! every operation is a union and the paths are either convex or mutually
//! disjoint, the same answer falls out of concatenating them all and calling
//! [`simplify`](super::simplify) **once**.
//!
//! Concatenation only works if the contours wind the same way. Two convex
//! contours wound oppositely cancel under the nonzero rule, so
//! [`fix_winding`] reverses the ones that would cancel before they are added.

use crate::core::{FillType, Path, Point, Rect, Verb};
use crate::error::PathKitError;

use super::sk_path_ops_simplify::is_convex;
use super::{op, simplify, PathOp};

/// Which way a contour winds.
///
/// Port of `SkPathFirstDirection`. `Unknown` covers a path with no area to
/// measure, or one whose contours disagree.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirstDirection {
    /// Clockwise in Skia's y-down coordinate system.
    Cw,
    /// Counterclockwise in Skia's y-down coordinate system.
    Ccw,
    /// No direction could be determined.
    Unknown,
}

/// Returns true when `path` has at most one contour.
///
/// Port of `one_contour`: any `Move` after the first starts another.
#[must_use]
pub fn one_contour(path: &Path) -> bool {
    path.iter()
        .skip(1)
        .all(|(verb, _, _)| verb != Verb::Move)
}

/// Returns the vertices of `path`, flattening curves.
///
/// Unlike the convexity helper's collector this evaluates conics at their
/// weight rather than pushing the raw control point, since the signed area
/// below has to match the region the path actually fills.
fn flattened_points(path: &Path) -> Vec<Point> {
    /// Samples per curve. Enough for a direction test, which only needs the
    /// sign of the total area.
    const STEPS: usize = 16;
    let mut pts: Vec<Point> = Vec::new();
    let push = |p: Point, pts: &mut Vec<Point>| {
        if pts.last().map_or(true, |l| Point::distance(*l, p) > 0.0) {
            pts.push(p);
        }
    };
    for (verb, p, w) in path.iter() {
        match verb {
            Verb::Move => push(p[0], &mut pts),
            Verb::Line => push(p[1], &mut pts),
            Verb::Quad => {
                for i in 1..=STEPS {
                    let t = i as f32 / STEPS as f32;
                    let u = 1.0 - t;
                    let pt = Point::new(
                        u * u * p[0].x + 2.0 * u * t * p[1].x + t * t * p[2].x,
                        u * u * p[0].y + 2.0 * u * t * p[1].y + t * t * p[2].y,
                    );
                    push(pt, &mut pts);
                }
            }
            Verb::Conic => {
                let weight = w.unwrap_or(1.0);
                for i in 1..=STEPS {
                    let t = i as f32 / STEPS as f32;
                    let u = 1.0 - t;
                    let cross = 2.0 * u * t * weight;
                    let denom = u * u + cross + t * t;
                    let pt = Point::new(
                        (u * u * p[0].x + cross * p[1].x + t * t * p[2].x) / denom,
                        (u * u * p[0].y + cross * p[1].y + t * t * p[2].y) / denom,
                    );
                    push(pt, &mut pts);
                }
            }
            Verb::Cubic => {
                for i in 1..=STEPS {
                    let t = i as f32 / STEPS as f32;
                    let u = 1.0 - t;
                    let pt = Point::new(
                        u * u * u * p[0].x
                            + 3.0 * u * u * t * p[1].x
                            + 3.0 * u * t * t * p[2].x
                            + t * t * t * p[3].x,
                        u * u * u * p[0].y
                            + 3.0 * u * u * t * p[1].y
                            + 3.0 * u * t * t * p[2].y
                            + t * t * t * p[3].y,
                    );
                    push(pt, &mut pts);
                }
            }
            Verb::Close => {}
        }
    }
    pts
}

/// Returns twice the signed area enclosed by `pts`, via the shoelace formula.
fn double_signed_area(pts: &[Point]) -> f32 {
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        sum += a.x * b.y - b.x * a.y;
    }
    sum
}

/// Returns which way `path`'s first contour winds.
///
/// Port of `SkPathPriv::ComputeFirstDirection`. In Skia's y-down space a
/// positive shoelace area is a clockwise contour.
#[must_use]
pub fn compute_first_direction(path: &Path) -> FirstDirection {
    let pts = flattened_points(path);
    let area = double_signed_area(&pts);
    if area > 0.0 {
        FirstDirection::Cw
    } else if area < 0.0 {
        FirstDirection::Ccw
    } else {
        FirstDirection::Unknown
    }
}

/// One verb of a contour: its type, its points, and a conic's weight.
type Segment = (Verb, Vec<Point>, Option<f32>);

/// Reverses the direction `path` is traced in, keeping the same region.
///
/// Port of `SkOpBuilder::ReversePath`. Curves are reversed by swapping their
/// endpoints and the order of their control points, so the geometry is
/// unchanged and only the winding flips.
pub fn reverse_path(path: &Path) -> Path {
    // Collect each contour as its verbs, then replay them backwards.
    let mut out = Path::new();
    out.set_fill_type(path.fill_type());

    let mut contours: Vec<Vec<Segment>> = Vec::new();
    for (verb, p, w) in path.iter() {
        match verb {
            Verb::Move => contours.push(vec![(verb, vec![p[0]], w)]),
            Verb::Line => {
                if let Some(c) = contours.last_mut() {
                    c.push((verb, vec![p[0], p[1]], w));
                }
            }
            Verb::Quad | Verb::Conic => {
                if let Some(c) = contours.last_mut() {
                    c.push((verb, vec![p[0], p[1], p[2]], w));
                }
            }
            Verb::Cubic => {
                if let Some(c) = contours.last_mut() {
                    c.push((verb, vec![p[0], p[1], p[2], p[3]], w));
                }
            }
            Verb::Close => {
                if let Some(c) = contours.last_mut() {
                    c.push((verb, Vec::new(), w));
                }
            }
        }
    }

    for contour in &contours {
        let segments: Vec<&Segment> = contour
            .iter()
            .filter(|(v, _, _)| *v != Verb::Move && *v != Verb::Close)
            .collect();
        let closed = contour.iter().any(|(v, _, _)| *v == Verb::Close);

        // Start where the contour ended.
        let start = match segments.last() {
            Some((_, pts, _)) => *pts.last().expect("segment has points"),
            None => match contour.first() {
                Some((_, pts, _)) if !pts.is_empty() => pts[0],
                _ => continue,
            },
        };
        out.move_to(start.x, start.y);

        for (verb, pts, w) in segments.iter().rev() {
            match verb {
                Verb::Line => {
                    out.line_to(pts[0].x, pts[0].y);
                }
                Verb::Quad => {
                    out.quad_to(pts[1].x, pts[1].y, pts[0].x, pts[0].y);
                }
                Verb::Conic => {
                    out.conic_to(pts[1].x, pts[1].y, pts[0].x, pts[0].y, w.unwrap_or(1.0));
                }
                Verb::Cubic => {
                    // Control points swap as well as the endpoints.
                    out.cubic_to(
                        pts[2].x, pts[2].y, pts[1].x, pts[1].y, pts[0].x, pts[0].y,
                    );
                }
                _ => {}
            }
        }
        if closed {
            out.close();
        }
    }
    out
}

/// Rewrites `path` so its contours wind consistently under the nonzero rule.
///
/// Port of `SkOpBuilder::FixWinding`. An even-odd fill becomes the equivalent
/// winding fill, and a single contour running clockwise is reversed so that
/// concatenating it with others does not cancel them.
///
/// The C++ general case walks the contour graph with `FindSortableTop` to
/// order nested contours; that machinery is not ported yet, so a path with
/// several contours is returned with only its fill type corrected. That is
/// safe for [`OpBuilder::resolve`], which only reaches here for paths that are
/// convex (hence one contour) or disjoint from every other path.
pub fn fix_winding(path: &Path) -> Path {
    let fill_type = match path.fill_type() {
        FillType::InverseEvenOdd => FillType::InverseWinding,
        FillType::EvenOdd => FillType::Winding,
        other => other,
    };
    if one_contour(path) {
        let dir = compute_first_direction(path);
        if dir != FirstDirection::Unknown {
            let mut out = if dir == FirstDirection::Cw {
                reverse_path(path)
            } else {
                path.clone()
            };
            out.set_fill_type(fill_type);
            return out;
        }
    }
    let mut out = path.clone();
    out.set_fill_type(fill_type);
    out
}

/// Returns true when the two paths' bounds overlap.
///
/// Port of `SkOpBuilder::Intersects`, which is a bounds test only.
#[must_use]
pub fn intersects(one: &Path, two: &Path) -> bool {
    rects_intersect(one.bounds(), two.bounds())
}

/// Returns true when two rectangles overlap.
fn rects_intersect(a: Rect, b: Rect) -> bool {
    a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom
}

/// Accumulates a series of path operations.
///
/// Port of `SkOpBuilder`. See the module docs for why [`Self::resolve`] is
/// worth more than a loop of [`op`] calls.
#[derive(Debug, Default)]
pub struct OpBuilder {
    paths: Vec<Path>,
    ops: Vec<PathOp>,
}

impl OpBuilder {
    /// Returns an empty builder.
    #[must_use]
    pub fn new() -> Self {
        Self {
            paths: Vec::new(),
            ops: Vec::new(),
        }
    }

    /// Adds `path` to the accumulated result via `operator`.
    ///
    /// Port of `SkOpBuilder::add`. A leading non-union operand needs something
    /// to operate on, so an empty path is unioned in ahead of it.
    pub fn add(&mut self, path: Path, operator: PathOp) {
        if self.ops.is_empty() && operator != PathOp::Union {
            self.paths.push(Path::new());
            self.ops.push(PathOp::Union);
        }
        self.paths.push(path);
        self.ops.push(operator);
    }

    /// Discards everything accumulated so far.
    ///
    /// Port of `SkOpBuilder::reset`.
    pub fn reset(&mut self) {
        self.paths.clear();
        self.ops.clear();
    }

    /// Computes the accumulated result and resets the builder.
    ///
    /// Port of `SkOpBuilder::resolve`. When every operation is a union and
    /// each path is either convex or bounds-disjoint from the ones before it,
    /// the paths are simplified, wound consistently, concatenated, and
    /// simplified once more. Otherwise the operations are replayed pairwise.
    ///
    /// # Errors
    ///
    /// Returns [`PathKitError::OperationFailed`] if any underlying operation
    /// fails. The builder is reset either way.
    pub fn resolve(&mut self) -> Result<Path, PathKitError> {
        if self.paths.is_empty() {
            self.reset();
            return Ok(Path::new());
        }

        let all_union = self.can_take_the_union_path();

        if !all_union {
            let mut result = self.paths[0].clone();
            for index in 1..self.paths.len() {
                result = match op(&result, &self.paths[index], self.ops[index]) {
                    Ok(p) => p,
                    Err(e) => {
                        self.reset();
                        return Err(e);
                    }
                };
            }
            self.reset();
            return Ok(result);
        }

        let mut sum = Path::new();
        for path in &self.paths {
            let simplified = match simplify(path) {
                Ok(p) => p,
                Err(e) => {
                    self.reset();
                    return Err(e);
                }
            };
            if simplified.is_empty() {
                continue;
            }
            // simplify returns an even-odd path; convert back to winding form
            // before accumulating, or the contours cancel each other.
            let wound = fix_winding(&simplified);
            sum.add_path(&wound, 0.0, 0.0);
        }
        self.reset();
        // One simplify over the whole sum, instead of N-1 boolean ops.
        simplify(&sum)
    }

    /// Returns true when `resolve` can use the concatenate-and-simplify path.
    ///
    /// Mirrors the loop at the top of `SkOpBuilder::resolve`: every operation
    /// must be a union over a non-inverse path, and each path must be either
    /// convex or bounds-disjoint from all the paths before it.
    fn can_take_the_union_path(&self) -> bool {
        for (index, path) in self.paths.iter().enumerate() {
            if self.ops[index] != PathOp::Union || path.is_inverse_fill_type() {
                return false;
            }
            if is_convex(path) {
                if compute_first_direction(path) == FirstDirection::Unknown {
                    return false;
                }
                continue;
            }
            // Not convex, so it is only safe if it touches nothing before it.
            let bounds = path.bounds();
            for inner in &self.paths[..index] {
                if rects_intersect(inner.bounds(), bounds) {
                    return false;
                }
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Rect;

    fn rect_path(l: f32, t: f32, r: f32, b: f32) -> Path {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(l, t, r, b));
        p
    }

    fn contour_count(p: &Path) -> usize {
        p.iter().filter(|(v, _, _)| *v == Verb::Move).count()
    }

    #[test]
    fn one_contour_counts_moves() {
        assert!(one_contour(&rect_path(0.0, 0.0, 10.0, 10.0)));
        let mut two = rect_path(0.0, 0.0, 10.0, 10.0);
        two.add_rect_simple(Rect::from_ltrb(20.0, 20.0, 30.0, 30.0));
        assert!(!one_contour(&two));
        assert!(one_contour(&Path::new()));
    }

    #[test]
    fn compute_first_direction_reads_the_winding() {
        // add_rect_simple traces one way; reversing it must flip the answer.
        let cw = rect_path(0.0, 0.0, 10.0, 10.0);
        let dir = compute_first_direction(&cw);
        assert_ne!(dir, FirstDirection::Unknown);
        let flipped = compute_first_direction(&reverse_path(&cw));
        assert_ne!(flipped, FirstDirection::Unknown);
        assert_ne!(dir, flipped, "reversing must flip the direction");
    }

    #[test]
    fn compute_first_direction_is_unknown_without_area() {
        let mut line = Path::new();
        line.move_to(0.0, 0.0);
        line.line_to(10.0, 0.0);
        line.close();
        assert_eq!(compute_first_direction(&line), FirstDirection::Unknown);
        assert_eq!(compute_first_direction(&Path::new()), FirstDirection::Unknown);
    }

    #[test]
    fn reverse_path_keeps_the_region() {
        let original = rect_path(0.0, 0.0, 10.0, 10.0);
        let reversed = reverse_path(&original);
        // Same area covered, traced the other way.
        assert!(reversed.contains(5.0, 5.0));
        assert!(!reversed.contains(15.0, 5.0));
        let a = original.bounds();
        let b = reversed.bounds();
        assert!((a.left - b.left).abs() < 1e-4);
        assert!((a.right - b.right).abs() < 1e-4);
        assert!((a.top - b.top).abs() < 1e-4);
        assert!((a.bottom - b.bottom).abs() < 1e-4);
    }

    #[test]
    fn reverse_path_handles_curves() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.quad_to(50.0, 100.0, 100.0, 0.0);
        p.cubic_to(120.0, -40.0, 140.0, -40.0, 160.0, 0.0);
        p.close();
        let r = reverse_path(&p);
        // The curve verbs survive, and the bounds are unchanged.
        let quads = r.iter().filter(|(v, _, _)| *v == Verb::Quad).count();
        let cubics = r.iter().filter(|(v, _, _)| *v == Verb::Cubic).count();
        assert_eq!(quads, 1);
        assert_eq!(cubics, 1);
        let a = p.bounds();
        let b = r.bounds();
        assert!((a.left - b.left).abs() < 1e-3);
        assert!((a.right - b.right).abs() < 1e-3);
    }

    #[test]
    fn reverse_path_preserves_conic_weight() {
        let mut p = Path::new();
        p.add_circle(0.0, 0.0, 50.0);
        let r = reverse_path(&p);
        let mut weights: Vec<f32> = r
            .iter()
            .filter_map(|(v, _, w)| if v == Verb::Conic { w } else { None })
            .collect();
        assert!(!weights.is_empty(), "a circle is built from conics");
        weights.sort_by(|a, b| a.partial_cmp(b).unwrap());
        // Every weight is the quarter-arc weight, not 1.
        for w in &weights {
            assert!((w - std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-4);
        }
    }

    #[test]
    fn fix_winding_converts_even_odd_to_winding() {
        let mut p = rect_path(0.0, 0.0, 10.0, 10.0);
        p.set_fill_type(FillType::EvenOdd);
        assert_eq!(fix_winding(&p).fill_type(), FillType::Winding);

        p.set_fill_type(FillType::InverseEvenOdd);
        assert_eq!(fix_winding(&p).fill_type(), FillType::InverseWinding);
    }

    #[test]
    fn fix_winding_makes_single_contours_agree() {
        // Two rects traced opposite ways must come out of fix_winding with
        // the same direction, or concatenating them cancels the overlap.
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = reverse_path(&rect_path(5.0, 5.0, 15.0, 15.0));
        assert_ne!(
            compute_first_direction(&a),
            compute_first_direction(&b),
            "the fixture needs them disagreeing to start with"
        );
        let fa = fix_winding(&a);
        let fb = fix_winding(&b);
        assert_eq!(compute_first_direction(&fa), compute_first_direction(&fb));
    }

    #[test]
    fn nested_same_direction_contours_do_not_cancel() {
        // A small square inside a large one, both wound the same way, fills
        // solid under the nonzero rule. If fix_winding flipped one of them the
        // middle would punch out.
        let outer = fix_winding(&rect_path(0.0, 0.0, 100.0, 100.0));
        let inner = fix_winding(&rect_path(40.0, 40.0, 60.0, 60.0));
        let mut sum = Path::new();
        sum.add_path(&outer, 0.0, 0.0);
        sum.add_path(&inner, 0.0, 0.0);
        sum.set_fill_type(FillType::Winding);
        assert!(sum.contains(50.0, 50.0), "the nested region must stay filled");
        assert!(sum.contains(10.0, 10.0));
    }

    #[test]
    fn intersects_is_a_bounds_test() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        let c = rect_path(50.0, 50.0, 60.0, 60.0);
        assert!(intersects(&a, &b));
        assert!(!intersects(&a, &c));
    }

    #[test]
    fn empty_builder_resolves_to_an_empty_path() {
        let mut builder = OpBuilder::new();
        assert!(builder.resolve().unwrap().is_empty());
    }

    #[test]
    fn add_prepends_an_empty_union_for_a_leading_non_union() {
        let mut builder = OpBuilder::new();
        builder.add(rect_path(0.0, 0.0, 10.0, 10.0), PathOp::Difference);
        // Differencing against nothing leaves nothing.
        let result = builder.resolve().unwrap();
        assert!(result.is_empty() || !result.contains(5.0, 5.0));
    }

    #[test]
    fn reset_discards_everything() {
        let mut builder = OpBuilder::new();
        builder.add(rect_path(0.0, 0.0, 10.0, 10.0), PathOp::Union);
        builder.reset();
        assert!(builder.resolve().unwrap().is_empty());
    }

    #[test]
    fn union_of_five_overlapping_rects_matches_sequential_ops() {
        // The acceptance case: the fast path and the pairwise path must agree
        // on the filled region.
        let rects: Vec<Path> = (0..5)
            .map(|i| {
                let x = i as f32 * 8.0;
                rect_path(x, 0.0, x + 20.0, 20.0)
            })
            .collect();

        let mut builder = OpBuilder::new();
        for r in &rects {
            builder.add(r.clone(), PathOp::Union);
        }
        let fast = builder.resolve().unwrap();

        let mut sequential = rects[0].clone();
        for r in &rects[1..] {
            sequential = op(&sequential, r, PathOp::Union).unwrap();
        }

        assert!(!fast.is_empty());
        // Sample the region; both must agree everywhere.
        for i in 0..=60 {
            for j in 0..=20 {
                let (x, y) = (i as f32, j as f32);
                // Skip points within a hair of an edge, where either result
                // may legitimately land on the other side.
                if (0..=60).any(|k| (x - k as f32 * 8.0).abs() < 0.5)
                    || (x - 52.0).abs() < 0.5
                    || y < 0.5
                    || (y - 20.0).abs() < 0.5
                {
                    continue;
                }
                assert_eq!(
                    fast.contains(x, y),
                    sequential.contains(x, y),
                    "disagreement at ({x}, {y})"
                );
            }
        }
    }

    #[test]
    fn union_of_disjoint_rects_keeps_every_one() {
        let mut builder = OpBuilder::new();
        for i in 0..4 {
            let x = i as f32 * 50.0;
            builder.add(rect_path(x, 0.0, x + 10.0, 10.0), PathOp::Union);
        }
        let result = builder.resolve().unwrap();
        assert_eq!(contour_count(&result), 4);
        for i in 0..4 {
            let x = i as f32 * 50.0 + 5.0;
            assert!(result.contains(x, 5.0), "rect {i} went missing");
        }
    }

    #[test]
    fn a_mixed_operator_sequence_falls_back_to_pairwise() {
        let mut builder = OpBuilder::new();
        builder.add(rect_path(0.0, 0.0, 20.0, 20.0), PathOp::Union);
        builder.add(rect_path(10.0, 0.0, 30.0, 20.0), PathOp::Difference);
        let result = builder.resolve().unwrap();
        assert!(result.contains(5.0, 10.0), "left of the cut stays");
        assert!(!result.contains(15.0, 10.0), "the cut is taken out");
    }

    #[test]
    fn resolve_leaves_the_builder_empty() {
        let mut builder = OpBuilder::new();
        builder.add(rect_path(0.0, 0.0, 10.0, 10.0), PathOp::Union);
        let _ = builder.resolve().unwrap();
        assert!(builder.resolve().unwrap().is_empty());
    }
}
