//! Boolean path operations: union, intersect, difference, xor.
//!
//! [`op`] handles empty/identical paths directly, then runs the ported Skia
//! engine in [`sk_op_engine`]. Inputs that engine declines — curve/curve
//! coincidence is the case that still reaches it — fall back to the
//! flatten-split-classify boolean in `boolean`, which returns a polyline but
//! returns the right region.
//!
//! Source: `old/pathkit/include/pathops/SkPathOps.h`.

use crate::core::{Path, Rect};
use crate::error::PathKitError;

mod boolean;
pub mod sk_curve_intersect_ray;
pub mod sk_d_conic_line_intersection;
pub mod sk_d_cubic_line_intersection;
pub mod sk_d_cubic_to_quads;
pub mod sk_d_quad_line_intersection;
pub mod sk_intersections;
pub mod sk_line_parameters;
pub mod sk_op_angle;
pub mod sk_op_angle_order;
pub mod sk_op_arena;
pub mod sk_op_builder;
pub mod sk_op_coincidence;
pub mod sk_op_common;
pub mod sk_op_engine;
pub mod sk_op_sortable_top;
pub mod sk_op_span;
pub mod sk_op_walker;
pub mod sk_path_ops_as_winding;
pub mod sk_path_ops_conic;
pub mod sk_path_ops_curve;
pub mod sk_path_ops_cubic;
pub mod sk_path_ops_debug;
pub mod sk_path_ops_line;
pub mod sk_path_ops_point;
pub mod sk_path_ops_quad;
pub mod sk_path_ops_rect;
pub mod sk_path_ops_simplify;
pub mod sk_path_ops_tight_bounds;
pub mod sk_path_ops_tsect;
pub mod sk_path_ops_types;
pub mod sk_path_ops_winding;
pub mod sk_path_writer;
pub mod sk_reduce_order;

/// A boolean operation to perform between two paths via [`op`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathOp {
    /// Subtract the second path from the first.
    Difference,
    /// Keep only the area common to both paths.
    Intersect,
    /// Keep the area covered by either path.
    Union,
    /// Keep the area covered by exactly one path.
    Xor,
    /// Subtract the first path from the second.
    ReverseDifference,
}

/// Combines `one` and `two` with `op`, returning the resulting path.
///
/// Empty and identical paths are handled directly. Everything else goes
/// through the ported engine in [`sk_op_engine`], which walks the segment
/// graph and keeps the inputs' curve verbs: a contour the operation never
/// touches comes back with its cubics intact.
///
/// The engine reports failure rather than guessing when it cannot resolve an
/// input — two rays disagreeing about a span's winding, or a coincidence it
/// cannot classify. Those fall back to the flattening boolean, which gives a
/// polyline with the right filled region. Curve/curve coincidence is the
/// gap that still lands there; see `TODO/09-bridge-winding-xor.md`.
///
/// # Errors
///
/// Returns [`PathKitError::OperationFailed`] if the inputs are non-finite.
pub fn op(one: &Path, two: &Path, op: PathOp) -> Result<Path, PathKitError> {
    if !one.is_finite() || !two.is_finite() {
        return Err(PathKitError::OperationFailed);
    }
    if one.is_empty() {
        return Ok(match op {
            PathOp::Union | PathOp::Xor | PathOp::ReverseDifference => two.clone(),
            PathOp::Intersect | PathOp::Difference => Path::new(),
        });
    }
    if two.is_empty() {
        return Ok(match op {
            PathOp::Union | PathOp::Xor | PathOp::Difference => one.clone(),
            PathOp::Intersect | PathOp::ReverseDifference => Path::new(),
        });
    }
    if one == two {
        return Ok(match op {
            PathOp::Union | PathOp::Intersect => one.clone(),
            PathOp::Difference | PathOp::Xor | PathOp::ReverseDifference => Path::new(),
        });
    }
    if let Some(result) = sk_op_engine::op_with_engine(one, two, op) {
        return Ok(result);
    }
    boolean::path_op(one, two, op)
}

/// Reduces `path` to an equivalent path built from non-overlapping
/// contours.
///
/// Goes through the same ported engine as [`op`], keeping curve verbs;
/// the substitute in [`sk_path_ops_simplify`] is the fallback for inputs the
/// engine declines.
pub fn simplify(path: &Path) -> Result<Path, PathKitError> {
    if let Some(result) = sk_op_engine::simplify_with_engine(path) {
        return Ok(result);
    }
    crate::pathops::sk_path_ops_simplify::simplify(path).map_err(|_| PathKitError::OperationFailed)
}

/// Computes the exact (curve-aware) bounding box of `path`.
///
/// Uses the pathops engine to compute tight bounds that account for curve
/// extrema. For well-behaved paths (no inflection points), this falls back
/// to the native bounds() method for better performance.
pub fn tight_bounds(path: &Path) -> Result<Rect, PathKitError> {
    sk_path_ops_tight_bounds::tight_bounds(path).ok_or(PathKitError::OperationFailed)
}

/// Returns a path equivalent in filled area to `path`, but with
/// [`FillType::Winding`] fill.
pub fn as_winding(path: &Path) -> Result<Path, PathKitError> {
    crate::pathops::sk_path_ops_as_winding::as_winding(path).ok_or(PathKitError::OperationFailed)
}

pub use sk_op_builder::OpBuilder;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::FillType;

    #[test]
    fn union_identical_paths() {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        let result = op(&p, &p, PathOp::Union).unwrap();
        assert_eq!(result, p);
    }

    #[test]
    fn union_two_rects() {
        let mut a = Path::new();
        a.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        let mut b = Path::new();
        b.add_rect_simple(Rect::from_ltrb(5.0, 5.0, 15.0, 15.0));
        let result = op(&a, &b, PathOp::Union).unwrap();
        assert!(result.contains(1.0, 1.0));
        assert!(result.contains(14.0, 14.0));
        assert!(result.contains(7.0, 7.0));
        assert!(!result.contains(1.0, 14.0));
        assert!(!result.contains(14.0, 1.0));
    }

    #[test]
    fn intersect_overlapping_rects() {
        let mut a = Path::new();
        a.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        let mut b = Path::new();
        b.add_rect_simple(Rect::from_ltrb(5.0, 5.0, 15.0, 15.0));
        let result = op(&a, &b, PathOp::Intersect).unwrap();
        assert!(result.contains(7.0, 7.0));
        assert!(!result.contains(2.0, 2.0));
        assert!(!result.contains(12.0, 12.0));
        assert!(!result.contains(2.0, 12.0));
    }

    #[test]
    fn intersect_disjoint_rects_is_empty() {
        let mut a = Path::new();
        a.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 1.0, 1.0));
        let mut b = Path::new();
        b.add_rect_simple(Rect::from_ltrb(10.0, 10.0, 11.0, 11.0));
        let result = op(&a, &b, PathOp::Intersect).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn op_builder_basic() {
        let mut builder = OpBuilder::new();
        let mut a = Path::new();
        a.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 5.0, 5.0));
        let mut b = Path::new();
        b.add_rect_simple(Rect::from_ltrb(3.0, 3.0, 10.0, 10.0));
        builder.add(a, PathOp::Union);
        builder.add(b, PathOp::Union);
        let result = builder.resolve().unwrap();
        assert!(result.contains(1.0, 1.0));
        assert!(result.contains(9.0, 9.0));
        assert!(result.contains(4.0, 4.0));
        assert!(!result.contains(1.0, 9.0));
    }

    #[test]
    fn tight_bounds_uses_path_method() {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(1.0, 2.0, 3.0, 4.0));
        let tb = tight_bounds(&p).unwrap();
        assert!((tb.left - 1.0).abs() < 1e-6);
        assert!((tb.right - 3.0).abs() < 1e-6);
    }

    #[test]
    fn tight_bounds_with_quadratic_bezier() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.quad_to(50.0, 100.0, 100.0, 0.0);
        let tb = tight_bounds(&p).unwrap();
        assert!((tb.left - 0.0).abs() < 1e-6);
        assert!((tb.right - 100.0).abs() < 1e-6);
        // Quadratic should have positive bottom due to curve extrema
        assert!(tb.bottom > 0.0);
    }

    #[test]
    fn tight_bounds_with_cubic_bezier() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.cubic_to(75.0, 300.0, 225.0, -300.0, 300.0, 0.0);
        let tb = tight_bounds(&p).unwrap();
        assert!((tb.left - 0.0).abs() < 1e-6);
        assert!((tb.right - 300.0).abs() < 1e-6);
    }

    #[test]
    fn as_winding_empty_path() {
        let path = Path::new();
        let result = as_winding(&path).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn as_winding_even_odd_to_winding() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close();
        path.set_fill_type(FillType::EvenOdd);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::Winding);
    }

    #[test]
    fn as_winding_invert_fill_type() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close();
        path.set_fill_type(FillType::InverseWinding);
        let result = as_winding(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::InverseWinding);
    }

    #[test]
    fn simplify_keeps_an_even_odd_hole() {
        // Concentric squares under even-odd fill: an annulus with a square
        // hole. Pins `bridgeXor`, which is what keeps the hole open instead
        // of filling it solid and rewriting the fill type to winding.
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 100.0, 100.0));
        p.add_rect_simple(Rect::from_ltrb(25.0, 25.0, 75.0, 75.0));
        p.set_fill_type(FillType::EvenOdd);
        assert!(!p.contains(50.0, 50.0), "the input really has a hole");

        let got = simplify(&p).expect("simplifies");
        assert!(!got.contains(50.0, 50.0), "the hole survives");
        assert!(got.contains(10.0, 50.0), "and the ring around it is filled");
    }

    #[test]
    fn the_engine_keeps_an_even_odd_hole_too() {
        // simplify() falls back to the substitute engine on a decline; this
        // pins the real engine's own answer directly.
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 100.0, 100.0));
        p.add_rect_simple(Rect::from_ltrb(25.0, 25.0, 75.0, 75.0));
        p.set_fill_type(FillType::EvenOdd);

        let got = sk_op_engine::simplify_with_engine(&p).expect("the engine answers");
        assert!(!got.contains(50.0, 50.0), "the hole survives");
        assert!(got.contains(10.0, 50.0), "and the ring around it is filled");
    }
}
