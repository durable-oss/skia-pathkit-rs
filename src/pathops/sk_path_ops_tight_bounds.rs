//! SkPathOpsTightBounds - computes curve-aware tight bounding boxes for paths
//!
//! Port of Skia's SkPathOpsTightBounds.cpp

use crate::core::{Path, Rect, Scalar, Verb};
use super::sk_reduce_order::{reduce_conic_path, reduce_cubic_path, reduce_quad_path, ReduceResult};

/// Threshold for floating point comparisons
const SMALL_TOLERANCE: Scalar = 1e-10;

/// Check if a value is between two others (inclusive)
fn between(a: Scalar, b: Scalar, c: Scalar) -> bool {
    b >= a.min(c) && b <= a.max(c)
}

/// Check if a path is well-behaved (no inflection points in curves)
/// Returns true if quads, conics, and cubics have monotonic coordinates
fn is_well_behaved(path: &Path) -> bool {
    let mut pt_idx = 0;

    for verb in path.verbs() {
        match verb {
            Verb::Quad => {
                if pt_idx + 2 >= path.points().len() {
                    break;
                }
                let pts = path.points();
                if !between(pts[pt_idx].x, pts[pt_idx + 1].x, pts[pt_idx + 2].x) {
                    return false;
                }
                if !between(pts[pt_idx].y, pts[pt_idx + 1].y, pts[pt_idx + 2].y) {
                    return false;
                }
                pt_idx += 2;
            }
            Verb::Conic => {
                if pt_idx + 2 >= path.points().len() {
                    break;
                }
                let pts = path.points();
                if !between(pts[pt_idx].x, pts[pt_idx + 1].x, pts[pt_idx + 2].x) {
                    return false;
                }
                if !between(pts[pt_idx].y, pts[pt_idx + 1].y, pts[pt_idx + 2].y) {
                    return false;
                }
                pt_idx += 2;
            }
            Verb::Cubic => {
                if pt_idx + 3 >= path.points().len() {
                    break;
                }
                let pts = path.points();
                if !between(pts[pt_idx].x, pts[pt_idx + 1].x, pts[pt_idx + 3].x) {
                    return false;
                }
                if !between(pts[pt_idx].y, pts[pt_idx + 1].y, pts[pt_idx + 3].y) {
                    return false;
                }
                if !between(pts[pt_idx].x, pts[pt_idx + 2].x, pts[pt_idx + 3].x) {
                    return false;
                }
                if !between(pts[pt_idx].y, pts[pt_idx + 2].y, pts[pt_idx + 3].y) {
                    return false;
                }
                pt_idx += 3;
            }
            Verb::Move | Verb::Line | Verb::Close => {}
        }
    }
    true
}

/// Compute a loose bounds by iterating over points only
fn compute_move_bounds(path: &Path) -> Rect {
    let mut bounds = Rect {
        left: Scalar::MAX,
        top: Scalar::MAX,
        right: Scalar::MIN,
        bottom: Scalar::MIN,
    };

    for pt in path.points() {
        bounds.left = bounds.left.min(pt.x);
        bounds.top = bounds.top.min(pt.y);
        bounds.right = bounds.right.max(pt.x);
        bounds.bottom = bounds.bottom.max(pt.y);
    }

    if bounds.left > bounds.right || bounds.top > bounds.bottom {
        Rect::empty()
    } else {
        bounds
    }
}

/// Compute tight bounds using the pathops engine.
///
/// Upstream reaches this through `SkOpEdgeBuilder`, which runs every curve
/// through `SkReduceOrder` on the way to becoming a segment. A curve that
/// reduces to a point or a line contributes only the reduced points, so its
/// control-point excursion never reaches the bounds. `Path::compute_tight_bounds`
/// has no such step and measures each curve's true extrema, which for a
/// degenerate curve means reporting an apex that upstream discards. Do the
/// reduction here, then measure whatever survives it.
fn compute_tight_bounds_full(path: &Path, _move_bounds: Rect) -> Option<Rect> {
    if path.is_empty() {
        return Some(Rect::empty());
    }

    let reduced = reduce_degenerate_curves(path);
    Some(reduced.as_ref().unwrap_or(path).compute_tight_bounds())
}

/// Rebuilds `path` with every degenerate curve replaced by its reduction.
///
/// Returns `None` when nothing reduced, so the common case keeps using the
/// original path rather than a copy of it. A curve reducing to a point becomes
/// a line to its own endpoint; one reducing to a line becomes that line. Both
/// keep the contour connected, which matters because the caller measures
/// points, not segments.
fn reduce_degenerate_curves(path: &Path) -> Option<Path> {
    let pts = path.points();
    let weights = path.conic_weights();
    let mut out = Path::new();
    let mut reduced_any = false;
    let mut pi = 0usize;
    let mut wi = 0usize;

    for verb in path.verbs() {
        match verb {
            Verb::Move => {
                out.move_to(pts[pi].x, pts[pi].y);
                pi += 1;
            }
            Verb::Line => {
                out.line_to(pts[pi].x, pts[pi].y);
                pi += 1;
            }
            Verb::Quad => {
                let hull = [pts[pi - 1], pts[pi], pts[pi + 1]];
                match reduce_quad_path(&hull) {
                    ReduceResult::Point | ReduceResult::Line => {
                        reduced_any = true;
                        out.line_to(hull[2].x, hull[2].y);
                    }
                    _ => {
                        out.quad_to(hull[1].x, hull[1].y, hull[2].x, hull[2].y);
                    }
                }
                pi += 2;
            }
            Verb::Conic => {
                let hull = [pts[pi - 1], pts[pi], pts[pi + 1]];
                let w = weights[wi];
                match reduce_conic_path(&hull, w) {
                    ReduceResult::Point | ReduceResult::Line => {
                        reduced_any = true;
                        out.line_to(hull[2].x, hull[2].y);
                    }
                    _ => {
                        out.conic_to(hull[1].x, hull[1].y, hull[2].x, hull[2].y, w);
                    }
                }
                pi += 2;
                wi += 1;
            }
            Verb::Cubic => {
                let hull = [pts[pi - 1], pts[pi], pts[pi + 1], pts[pi + 2]];
                match reduce_cubic_path(&hull) {
                    ReduceResult::Point | ReduceResult::Line => {
                        reduced_any = true;
                        out.line_to(hull[3].x, hull[3].y);
                    }
                    _ => {
                        out.cubic_to(
                            hull[1].x, hull[1].y, hull[2].x, hull[2].y, hull[3].x, hull[3].y,
                        );
                    }
                }
                pi += 3;
            }
            Verb::Close => {
                out.close();
            }
        }
    }

    reduced_any.then_some(out)
}

/// Compute the tight bounding box of a path, accounting for curve extrema.
///
/// This function attempts to use the path's native bounds() if the path is
/// well-behaved (no inflection points), otherwise it uses the full pathops
/// engine to compute the actual tight bounds.
///
/// Returns `Some(Rect)` on success, `None` if the path cannot be processed.
pub fn tight_bounds(path: &Path) -> Option<Rect> {
    if path.is_empty() {
        return Some(Rect::empty());
    }

    let move_bounds = compute_move_bounds(path);

    // Try fast path for well-behaved paths
    if is_well_behaved(path) {
        // Path is well-behaved, use native bounds
        Some(path.bounds())
    } else {
        // Use full pathops engine for complex paths
        compute_tight_bounds_full(path, move_bounds)
    }
}

/// Alternative implementation that always uses pathops engine
/// This matches the C++ behavior more closely but is more expensive
pub fn tight_bounds_full(path: &Path) -> Option<Rect> {
    if path.is_empty() {
        return Some(Rect::empty());
    }

    let move_bounds = compute_move_bounds(path);
    compute_tight_bounds_full(path, move_bounds)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_between() {
        assert!(between(0.0, 5.0, 10.0));
        assert!(between(10.0, 5.0, 0.0));
        assert!(!between(0.0, 15.0, 10.0));
        assert!(between(0.0, 0.0, 10.0));
        assert!(between(0.0, 10.0, 10.0));
    }

    #[test]
    fn test_compute_move_bounds_empty() {
        let path = Path::new();
        let bounds = compute_move_bounds(&path);
        // Empty path should return initialized bounds
        // Note: bounds may vary based on implementation
        let _ = bounds;
    }

    #[test]
    fn test_compute_move_bounds_single_point() {
        let mut path = Path::new();
        path.move_to(5.0, 10.0);
        let bounds = compute_move_bounds(&path);
        assert_eq!(bounds.left, 5.0);
        assert_eq!(bounds.right, 5.0);
        assert_eq!(bounds.top, 10.0);
        assert_eq!(bounds.bottom, 10.0);
    }

    #[test]
    fn test_compute_move_bounds_multiple_points() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 5.0);
        path.line_to(5.0, 10.0);
        let bounds = compute_move_bounds(&path);
        assert_eq!(bounds.left, 0.0);
        assert_eq!(bounds.right, 10.0);
        assert_eq!(bounds.top, 0.0);
        assert_eq!(bounds.bottom, 10.0);
    }

    #[test]
    fn test_is_well_behaved_line_only() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 10.0);
        assert!(is_well_behaved(&path));
    }

    #[test]
    fn test_is_well_behaved_monotonic_quad() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(5.0, 5.0, 10.0, 10.0); // Monotonic
        assert!(is_well_behaved(&path));
    }

    #[test]
    fn test_is_well_behaved_non_monotonic_quad() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(50.0, 100.0, 100.0, 0.0); // Has inflection in Y
        assert!(!is_well_behaved(&path));
    }

    #[test]
    fn test_is_well_behaved_monotonic_cubic() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.cubic_to(25.0, 25.0, 75.0, 75.0, 100.0, 100.0); // Monotonic
        assert!(is_well_behaved(&path));
    }

    #[test]
    fn test_is_well_behaved_non_monotonic_cubic() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.cubic_to(75.0, 300.0, 225.0, -300.0, 300.0, 0.0); // Has inflections
        assert!(!is_well_behaved(&path));
    }

    #[test]
    fn test_tight_bounds_empty() {
        let path = Path::new();
        let bounds = tight_bounds(&path);
        assert!(bounds.is_some());
    }

    #[test]
    fn test_tight_bounds_rect() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close();

        let bounds = tight_bounds(&path).unwrap();
        assert!((bounds.left - 0.0).abs() < SMALL_TOLERANCE);
        assert!((bounds.right - 10.0).abs() < SMALL_TOLERANCE);
        assert!((bounds.top - 0.0).abs() < SMALL_TOLERANCE);
        assert!((bounds.bottom - 10.0).abs() < SMALL_TOLERANCE);
    }

    #[test]
    fn test_tight_bounds_quad_bezier() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(50.0, 100.0, 100.0, 0.0);

        let bounds = tight_bounds(&path).unwrap();
        assert_eq!(bounds.left, 0.0);
        assert_eq!(bounds.right, 100.0);
        assert!(bounds.top >= 0.0);
        assert!(bounds.bottom > 0.0);
    }

    #[test]
    fn test_tight_bounds_cubic_bezier() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.cubic_to(75.0, 300.0, 225.0, -300.0, 300.0, 0.0);

        let bounds = tight_bounds(&path).unwrap();
        assert_eq!(bounds.left, 0.0);
        assert_eq!(bounds.right, 300.0);
        // The cubic goes above and below the line, so top < 0 and bottom > 0
    }

    #[test]
    fn test_tight_bounds_full_vs_tight() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(50.0, 100.0, 100.0, 0.0);

        let bounds = tight_bounds(&path).unwrap();
        let bounds_full = tight_bounds_full(&path).unwrap();
        // Both should produce the same bounds
        assert!((bounds.left - bounds_full.left).abs() < SMALL_TOLERANCE);
        assert!((bounds.right - bounds_full.right).abs() < SMALL_TOLERANCE);
    }

    #[test]
    fn test_tight_bounds_with_move_only() {
        let mut path = Path::new();
        path.move_to(5.0, 5.0);
        path.move_to(10.0, 10.0);
        path.move_to(3.0, 8.0);

        let bounds = tight_bounds(&path).unwrap();
        assert!((bounds.left - 3.0).abs() < SMALL_TOLERANCE);
        assert!((bounds.right - 10.0).abs() < SMALL_TOLERANCE);
        assert!((bounds.top - 5.0).abs() < SMALL_TOLERANCE);
        assert!((bounds.bottom - 10.0).abs() < SMALL_TOLERANCE);
    }

    #[test]
    fn test_pathops_integration() {
        // Integration test using pathops module
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close();

        // This would use the pathops module if it were exposed
        let bounds = tight_bounds(&path).unwrap();
        assert!((bounds.left - 0.0).abs() < SMALL_TOLERANCE);
    }

    // The cases below are ported from Skia's `tests/PathOpsTightBoundsTest.cpp`.
    // The upstream thread-runner tests (random lines/quads compared against a
    // rasterization) are not portable here; these are its deterministic
    // `DEF_TEST` one-offs. Upstream asserts `ComputeTightBounds` succeeds and
    // then compares against `path.getBounds()`, which is what each case does.
    //
    // `PathOpsTightBoundsTiny` is deliberately absent: it diverges. See
    // `TODO/2026-09-11-bug-tight-bounds-tiny-quad-not-collapsed.md`.

    #[test]
    fn upstream_tight_bounds_move() {
        // PathOpsTightBoundsMove: degenerate contours; tight == loose.
        let mut path = Path::new();
        path.move_to(10.0, 10.0);
        path.close();
        path.move_to(20.0, 20.0);
        path.line_to(20.0, 20.0);
        path.close();
        path.move_to(15.0, 15.0);
        path.line_to(15.0, 15.0);
        path.close();

        assert_eq!(tight_bounds(&path).unwrap(), path.bounds());
    }

    #[test]
    fn upstream_tight_bounds_move_one() {
        // PathOpsTightBoundsMoveOne: a lone move.
        let mut path = Path::new();
        path.move_to(20.0, 20.0);

        assert_eq!(tight_bounds(&path).unwrap(), path.bounds());
    }

    #[test]
    fn upstream_tight_bounds_move_two() {
        // PathOpsTightBoundsMoveTwo: two lone moves.
        let mut path = Path::new();
        path.move_to(20.0, 20.0);
        path.move_to(40.0, 40.0);

        assert_eq!(tight_bounds(&path).unwrap(), path.bounds());
    }

    #[test]
    fn upstream_tight_bounds_well_behaved() {
        // PathOpsTightBoundsWellBehaved: a monotonic quad has no extrema
        // inside (0, 1), so the control-point hull is already tight.
        let mut path = Path::new();
        path.move_to(1.0, 1.0);
        path.quad_to(2.0, 3.0, 4.0, 5.0);

        assert_eq!(tight_bounds(&path).unwrap(), path.bounds());
    }

    #[test]
    fn upstream_tight_bounds_ill_behaved() {
        // PathOpsTightBoundsIllBehaved: the control point lies outside the
        // curve's own range, so the loose bounds overshoot and the tight
        // bounds must be strictly smaller.
        let mut path = Path::new();
        path.move_to(1.0, 1.0);
        path.quad_to(4.0, 3.0, 2.0, 2.0);

        let tight = tight_bounds(&path).unwrap();
        let loose = path.bounds();
        assert_ne!(tight, loose);
        assert!(tight.right < loose.right);
        assert!(tight.bottom < loose.bottom);
    }

    #[test]
    fn upstream_tight_bounds_tiny() {
        // PathOpsTightBoundsTiny: a quad that starts and ends at the same
        // point with the control one ULP away. The curve is degenerate, so
        // the bounds collapse to the point and differ from the loose bounds,
        // which still carry the control point's excursion.
        let mut path = Path::new();
        path.move_to(1.0, 1.0);
        path.quad_to(1.000001, 1.0, 1.0, 1.0);

        let tight = tight_bounds(&path).unwrap();
        assert_eq!(tight, Rect::from_ltrb(1.0, 1.0, 1.0, 1.0));
        assert_ne!(tight, path.bounds());
    }

    #[test]
    fn a_curved_quad_is_still_measured_after_the_reduction_pass() {
        // The reduction must not swallow curves that genuinely bend: this
        // quad's apex lies outside its endpoints and has to survive.
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(1.0, 2.0, 2.0, 0.0);

        let tight = tight_bounds(&path).unwrap();
        // Apex of B(1/2) in y is 1.0, not the control point's 2.0.
        assert!((tight.bottom - 1.0).abs() < 1e-5, "got {tight:?}");
        assert!((tight.right - 2.0).abs() < 1e-5, "got {tight:?}");
    }

    #[test]
    fn upstream_tight_bounds_ill_behaved_scaled() {
        // PathOpsTightBoundsIllBehavedScaled: same shape at a scale where the
        // curve ends exactly on its maximum. Upstream pins the two extremes
        // rather than the whole rect.
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(1048578.0, 1048577.0, 1048576.0, 1048576.0);

        let tight = tight_bounds(&path).unwrap();
        assert_ne!(tight, path.bounds());
        assert_eq!(tight.right, 1048576.0);
        assert_eq!(tight.bottom, 1048576.0);
    }
}
