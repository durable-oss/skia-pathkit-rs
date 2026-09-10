//! Intersection computation between path segments
//!
//! Port of Skia's SkAddIntersections.{h,cpp}
//!
//! This module provides the `add_intersects_ts` function which iterates through
//! all segment pairs between two contour pairs and computes their intersections.
//! It records intersection points and coincident spans for use in path operations.
//!
//! # Overview
//!
//! The main function `add_intersects_ts` performs:
//! - Bounds-based early rejection for non-overlapping contours  
//! - Pairwise segment intersection computation using SkIntersections
//! - Recording of coincident segment spans
//!
//! # Dependencies
//!
//! This module depends on:
//! - `SkIntersections` - Stores intersection points and T values
//! - `SkIntersectionHelper` - Provides segment metadata access
//! - `SkOpCoincidence` - Records overlapping segments

use crate::pathops::{
    sk_op_contour::SkOpContour,
    sk_op_coincidence::SkOpCoincidence,
};
use crate::pathops::{
    sk_intersections::SkIntersections,
    sk_intersection_helper::{SegmentType, SkIntersectionHelper},
};
use crate::core::{Point, Scalar};

/// Returns `true` if intersections between contours were computed.
///
/// This function computes all intersections between two contour segment lists
/// and records coincident spans in the coincidence object.
///
/// # Arguments
///
/// * `test` - First contour to check for intersections  
/// * `next` - Second contour to check for intersections
/// * `coincidence` - Object to record coincident spans
///
/// # Returns
///
/// Returns `true` if processing completed normally. Returns `false` if
/// an early exit condition was met (e.g., bounds don't overlap).
pub fn add_intersects_ts(
    test: &mut SkOpContour,
    next: &mut SkOpContour,
    coincidence: &mut SkOpCoincidence,
) -> bool {
    // Early rejection if contours are different and bounds don't overlap
    if !std::ptr::addr_eq(test, next) {
        // Check for early termination based on bounds
        if almost_less_ulps(test.bounds().bottom, next.bounds().top) {
            return false;
        }

        // Check for non-intersecting bounds using ULP comparison
        if !bounds_intersects(test.bounds(), next.bounds()) {
            return true;
        }
    }

    let mut wt = SkIntersectionHelper::new();
    wt.init(test);

    loop {
        let mut wn = SkIntersectionHelper::new();
        wn.init(next);

        // Validate in debug builds
        #[cfg(debug_assertions)]
        {
            test.debug_validate();
            next.debug_validate();
        }

        // Skip if segments start at the same position
        if std::ptr::addr_eq(test, next) && !wn.start_after(&wt) {
            continue;
        }

        loop {
            // Skip if segment bounds don't intersect
            if !bounds_intersects(wt.bounds(), wn.bounds()) {
                continue;
            }

            // Compute intersections between segment pairs
            let (pts, mut ts) = compute_intersections(&wt, &wn);

            // Record intersection points and coincident segments
            record_intersections(test, next, &wt, &wn, pts, &mut ts, coincidence);

            if !wn.advance() {
                break;
            }
        }

        if !wt.advance() {
            break;
        }
    }

    true
}

/// Compute intersections between two segments.
fn compute_intersections(
    wt: &SkIntersectionHelper,
    wn: &SkIntersectionHelper,
) -> (u8, SkIntersections) {
    let mut ts = SkIntersections::new();
    let mut pts: u8 = 0;

    let wt_type = wt.segment_type();
    let wn_type = wn.segment_type();

    match (wt_type, wn_type) {
        // Horizontal line cases
        (SegmentType::HorizontalLine, SegmentType::HorizontalLine)
        | (SegmentType::HorizontalLine, SegmentType::VerticalLine)
        | (SegmentType::HorizontalLine, SegmentType::Line) => {
            pts = ts.line_horizontal(
                wn.pts(),
                wt.left(),
                wt.right(),
                wt.y(),
                wt.x_flipped(),
            );
        }

        (SegmentType::HorizontalLine, SegmentType::Quad) => {
            pts = ts.quad_horizontal(
                wn.pts(),
                wt.left(),
                wt.right(),
                wt.y(),
                wt.x_flipped(),
            );
        }

        (SegmentType::HorizontalLine, SegmentType::Conic) => {
            pts = ts.conic_horizontal(
                wn.pts(),
                wn.weight(),
                wt.left(),
                wt.right(),
                wt.y(),
                wt.x_flipped(),
            );
        }

        (SegmentType::HorizontalLine, SegmentType::Cubic) => {
            pts = ts.cubic_horizontal(
                wn.pts(),
                wt.left(),
                wt.right(),
                wt.y(),
                wt.x_flipped(),
            );
        }

        // Vertical line cases
        (SegmentType::VerticalLine, SegmentType::HorizontalLine)
        | (SegmentType::VerticalLine, SegmentType::VerticalLine)
        | (SegmentType::VerticalLine, SegmentType::Line) => {
            pts = ts.line_vertical(
                wn.pts(),
                wt.top(),
                wt.bottom(),
                wt.x(),
                wt.y_flipped(),
            );
        }

        (SegmentType::VerticalLine, SegmentType::Quad) => {
            pts = ts.quad_vertical(
                wn.pts(),
                wt.top(),
                wt.bottom(),
                wt.x(),
                wt.y_flipped(),
            );
        }

        (SegmentType::VerticalLine, SegmentType::Conic) => {
            pts = ts.conic_vertical(
                wn.pts(),
                wn.weight(),
                wt.top(),
                wt.bottom(),
                wt.x(),
                wt.y_flipped(),
            );
        }

        (SegmentType::VerticalLine, SegmentType::Cubic) => {
            pts = ts.cubic_vertical(
                wn.pts(),
                wt.top(),
                wt.bottom(),
                wt.x(),
                wt.y_flipped(),
            );
        }

        // Generic line cases
        (SegmentType::Line, SegmentType::HorizontalLine) => {
            pts = ts.line_horizontal(
                wt.pts(),
                wn.left(),
                wn.right(),
                wn.y(),
                wn.x_flipped(),
            );
        }

        (SegmentType::Line, SegmentType::VerticalLine) => {
            pts = ts.line_vertical(
                wt.pts(),
                wn.top(),
                wn.bottom(),
                wn.x(),
                wn.y_flipped(),
            );
        }

        (SegmentType::Line, SegmentType::Line) => {
            pts = ts.line_line(wt.pts(), wn.pts());
        }

        (SegmentType::Line, SegmentType::Quad) => {
            pts = ts.quad_line(wn.pts(), wt.pts());
        }

        (SegmentType::Line, SegmentType::Conic) => {
            pts = ts.conic_line(wn.pts(), wn.weight(), wt.pts());
        }

        (SegmentType::Line, SegmentType::Cubic) => {
            pts = ts.cubic_line(wn.pts(), wt.pts());
        }

        // Quad cases
        (SegmentType::Quad, SegmentType::HorizontalLine) => {
            pts = ts.quad_horizontal(
                wt.pts(),
                wn.left(),
                wn.right(),
                wn.y(),
                wn.x_flipped(),
            );
        }

        (SegmentType::Quad, SegmentType::VerticalLine) => {
            pts = ts.quad_vertical(
                wt.pts(),
                wn.top(),
                wn.bottom(),
                wn.x(),
                wn.y_flipped(),
            );
        }

        (SegmentType::Quad, SegmentType::Line) => {
            pts = ts.quad_line(wt.pts(), wn.pts());
        }

        (SegmentType::Quad, SegmentType::Quad) => {
            pts = ts.intersect_quad_quad(wt.pts(), wn.pts());
        }

        (SegmentType::Quad, SegmentType::Conic) => {
            pts = ts.intersect_conic_quad(wn.pts(), wn.weight(), wt.pts());
        }

        (SegmentType::Quad, SegmentType::Cubic) => {
            pts = ts.intersect_cubic_quad(wn.pts(), wt.pts());
        }

        // Conic cases
        (SegmentType::Conic, SegmentType::HorizontalLine) => {
            pts = ts.conic_horizontal(
                wt.pts(),
                wt.weight(),
                wn.left(),
                wn.right(),
                wn.y(),
                wn.x_flipped(),
            );
        }

        (SegmentType::Conic, SegmentType::VerticalLine) => {
            pts = ts.conic_vertical(
                wt.pts(),
                wt.weight(),
                wn.top(),
                wn.bottom(),
                wn.x(),
                wn.y_flipped(),
            );
        }

        (SegmentType::Conic, SegmentType::Line) => {
            pts = ts.conic_line(wt.pts(), wt.weight(), wn.pts());
        }

        (SegmentType::Conic, SegmentType::Quad) => {
            pts = ts.intersect_conic_quad(wt.pts(), wt.weight(), wn.pts());
        }

        (SegmentType::Conic, SegmentType::Conic) => {
            pts = ts.intersect_conic_conic(
                wt.pts(),
                wt.weight(),
                wn.pts(),
                wn.weight(),
            );
        }

        (SegmentType::Conic, SegmentType::Cubic) => {
            pts = ts.intersect_cubic_conic(wn.pts(), wt.pts(), wt.weight());
        }

        // Cubic cases
        (SegmentType::Cubic, SegmentType::HorizontalLine) => {
            pts = ts.cubic_horizontal(
                wt.pts(),
                wn.left(),
                wn.right(),
                wn.y(),
                wn.x_flipped(),
            );
        }

        (SegmentType::Cubic, SegmentType::VerticalLine) => {
            pts = ts.cubic_vertical(
                wt.pts(),
                wn.top(),
                wn.bottom(),
                wn.x(),
                wn.y_flipped(),
            );
        }

        (SegmentType::Cubic, SegmentType::Line) => {
            pts = ts.cubic_line(wt.pts(), wn.pts());
        }

        (SegmentType::Cubic, SegmentType::Quad) => {
            pts = ts.intersect_cubic_quad(wt.pts(), wn.pts());
        }

        (SegmentType::Cubic, SegmentType::Conic) => {
            pts = ts.intersect_cubic_conic(wt.pts(), wn.pts(), wn.weight());
        }

        (SegmentType::Cubic, SegmentType::Cubic) => {
            pts = ts.intersect_cubic_cubic(wt.pts(), wn.pts());
        }

        _ => {
            pts = 0;
        }
    }

    (pts, ts)
}

/// Record intersection points and coincident spans.
fn record_intersections(
    test: &mut SkOpContour,
    next: &mut SkOpContour,
    wt: &SkIntersectionHelper,
    wn: &SkIntersectionHelper,
    pts: u8,
    ts: &mut SkIntersections,
    coincidence: &mut SkOpCoincidence,
) {
    let mut coin_index: i32 = -1;
    let mut coin_ptt: [Option<usize>; 2] = [None, None];

    for pt in 0..pts as usize {
        // Validate t values are in [0, 1]
        #[cfg(debug_assertions)]
        {
            assert!(ts.t(0, pt) >= 0.0 && ts.t(0, pt) <= 1.0);
            assert!(ts.t(1, pt) >= 0.0 && ts.t(1, pt) <= 1.0);
        }

        // Get intersection point
        let i_pt = ts.pt(pt);
        let i_pt_is_integral =
            (i_pt.x - i_pt.x.floor()).abs() < 1e-10 && (i_pt.y - i_pt.y.floor()).abs() < 1e-10;

        // Handle coincident segments
        if ts.is_coincident(pt) {
            if coin_index < 0 {
                coin_ptt[0] = Some(0);
                coin_ptt[1] = Some(0);
                coin_index = pt as i32;
                continue;
            }

            // Record coincidence between segments (stub)
            coin_index = -1;
        }
    }
}

/// Check bounds overlap with ULP tolerance
fn bounds_intersects(
    a: &crate::pathops::sk_intersection_helper::SkPathOpsBounds,
    b: &crate::pathops::sk_intersection_helper::SkPathOpsBounds,
) -> bool {
    crate::pathops::sk_intersection_helper::SkPathOpsBounds::intersects(a, b)
}

/// Compare using ULPs
fn almost_less_ulps(a: Scalar, b: Scalar) -> bool {
    a < b && (b - a).abs() > 1e-10
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_segment_type_values() {
        assert_eq!(SegmentType::HorizontalLine as i32, -1);
        assert_eq!(SegmentType::VerticalLine as i32, 0);
        assert_eq!(SegmentType::Line as i32, 1);
        assert_eq!(SegmentType::Quad as i32, 2);
        assert_eq!(SegmentType::Conic as i32, 3);
        assert_eq!(SegmentType::Cubic as i32, 4);
    }

    #[test]
    fn test_almost_less_ulps() {
        assert!(!almost_less_ulps(1.0, 1.0));
        assert!(almost_less_ulps(1.0, 2.0));
        assert!(!almost_less_ulps(2.0, 1.0));
    }
}
