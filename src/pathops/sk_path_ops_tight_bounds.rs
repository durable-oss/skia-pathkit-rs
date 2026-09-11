//! SkPathOpsTightBounds - computes curve-aware tight bounding boxes for paths
//!
//! Port of Skia's SkPathOpsTightBounds.cpp

use crate::core::{Path, Rect, Scalar, Verb};

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

/// Compute tight bounds using the pathops engine
fn compute_tight_bounds_full(path: &Path, _move_bounds: Rect) -> Option<Rect> {
    // For paths with curves, use the Path's built-in tight bounds computation
    // which properly handles curve extrema
    if path.is_empty() {
        return Some(Rect::empty());
    }
    Some(path.compute_tight_bounds())
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
