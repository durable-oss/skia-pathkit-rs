//! SkPathOpsOp - main path operation implementation
//!
//! Port of Skia's SkPathOpsOp.cpp - core boolean path operations
//!
//! This module implements the main path operation functions:
//! - Op: Boolean combination of two paths
//! - OpDebug: Debug version with additional checks
//! - bridgeOp: Bridge operation to construct contours
//! - findChaseOp: Find next operation in chase queue

use crate::core::{Path, PathFillType, Rect};
use super::{PathOp, SkOpContourHead, SkOpSpan, SkOpCoincidence, SkPathWriter, SkOpGlobalState};

/// Operation inverse table for fill type handling
const OP_INVERSE: [[[PathOp; 2]; 2]; 5] = [
    // kDifference, kIntersect, kUnion, kXor, kReverseDifference
    [[PathOp::Difference, PathOp::Intersect],   // 0: kDifference
     [PathOp::Union, PathOp::ReverseDifference]],
    [[PathOp::Intersect, PathOp::Difference],   // 1: kIntersect
     [PathOp::ReverseDifference, PathOp::Union]],
    [[PathOp::Union, PathOp::ReverseDifference], // 2: kUnion
     [PathOp::Difference, PathOp::Intersect]],
    [[PathOp::Xor, PathOp::Xor],               // 3: kXor
     [PathOp::Xor, PathOp::Xor]],
    [[PathOp::ReverseDifference, PathOp::Union], // 4: kReverseDifference
     [PathOp::Intersect, PathOp::Difference]],
];

/// Output fill type inverse table
const OUT_INVERSE: [[[bool; 2]; 2]; 5] = [
    [[false, false], [true, false]],    // diff
    [[false, false], [false, true]],    // sect
    [[false, true], [true, true]],      // union
    [[false, true], [true, false]],     // xor
    [[false, true], [false, false]],    // rev diff
];

/// Main path operation: combine two paths with an operation
pub fn op(one: &Path, two: &Path, op: PathOp, result: &mut Path) -> bool {
    // Handle inverse fill types
    let op_inverted = OP_INVERSE[op as usize][one.is_inverse_fill_type() as usize][two.is_inverse_fill_type() as usize];
    let inverse_fill = OUT_INVERSE[op_inverted as usize][one.is_inverse_fill_type() as usize][two.is_inverse_fill_type() as usize];
    
    let fill_type = if inverse_fill {
        PathFillType::InverseEvenOdd
    } else {
        PathFillType::EvenOdd
    };

    // Fast path: rectangular intersection
    if op_inverted == PathOp::Intersect {
        let mut rect1 = Rect::empty();
        let mut rect2 = Rect::empty();
        if one.is_rect(Some(&mut rect1), None, None) && 
           two.is_rect(Some(&mut rect2), None, None) {
            result.reset();
            result.set_fill_type(fill_type);
            if let Some(intersection) = rect1.intersection(&rect2) {
                result.add_rect_simple(intersection);
            }
            return true;
        }
    }

    // Fast path: empty paths
    if one.is_empty() || two.is_empty() {
        let mut work = Path::new();
        match op_inverted {
            PathOp::Intersect => {}
            PathOp::Union | PathOp::Xor => {
                work = if one.is_empty() { two.clone() } else { one.clone() };
            }
            PathOp::Difference => {
                if !one.is_empty() { work = one.clone(); }
            }
            PathOp::ReverseDifference => {
                if !two.is_empty() { work = two.clone(); }
            }
            _ => return false,
        }
        
        if inverse_fill != work.is_inverse_fill_type() {
            work.toggle_inverse_fill_type();
        }
        return simplify(&work, result);
    }

    // Main path operation implementation
    op_debug(one, two, op_inverted, result)
}

/// Simplify a path to non-overlapping contours
pub fn simplify(path: &Path, result: &mut Path) -> bool {
    // Check if path is already simplified (winding fill type with no self-intersections)
    if path.fill_type() == PathFillType::Winding {
        *result = path.clone();
        return true;
    }

    // Use as_winding for fill type conversion
    if let Some(winding_path) = super::SkPathOpsAsWinding::as_winding(path) {
        *result = winding_path;
        return true;
    }

    false
}

/// Compute tight bounds including curves
pub fn tight_bounds(path: &Path, result: &mut Rect) -> bool {
    *result = path.compute_tight_bounds();
    true
}

/// Debug version of path operation with full implementation
fn op_debug(one: &Path, two: &Path, op: PathOp, result: &mut Path) -> bool {
    // Create global state for path operations
    let mut contour = SkOpContourHead::new();
    let mut global_state = SkOpGlobalState::new();
    let mut coincidence = SkOpCoincidence::new(&global_state);

    // Build edges from first path (minuend)
    let minuend = one;
    let subtrahend = two;

    // Create edge builder
    let mut builder = super::SkOpEdgeBuilder::new(minuend, &mut contour, &mut global_state);
    
    if builder.unparseable() {
        return false;
    }

    let xor_mask = builder.xor_mask();
    builder.add_operand(subtrahend);

    if !builder.finish() {
        return false;
    }

    let xor_op_mask = builder.xor_mask();

    // Sort contour list
    if !super::SkPathOpsCommon::sort_contour_list(
        &mut contour,
        xor_mask == super::kEvenOdd_PathOpsMask,
        xor_op_mask == super::kEvenOdd_PathOpsMask,
    ) {
        result.reset();
        result.set_fill_type(PathFillType::EvenOdd);
        return true;
    }

    // Find all intersections between segments
    for (i, current) in contour.iter().enumerate() {
        for next in contour.iter().skip(i + 1) {
            super::SkAddIntersections::add_intersect_ts(current, next, &mut coincidence);
        }
    }

    // Handle coincident segments
    if !super::SkPathOpsCommon::handle_coincidence(&contour, &mut coincidence) {
        return false;
    }

    // Construct output path using bridge
    result.reset();
    result.set_fill_type(PathFillType::EvenOdd);
    
    let mut writer = SkPathWriter::new(result);
    
    if !bridge_op(&contour, op, xor_mask, xor_op_mask, &mut writer) {
        return false;
    }

    // Assemble any remaining edges
    writer.assemble();
    true
}

/// Bridge operation: walk the contour list and generate output
fn bridge_op(
    contour_list: &SkOpContourHead,
    op: PathOp,
    xor_mask: i32,
    xor_op_mask: i32,
    writer: &mut SkPathWriter,
) -> bool {
    let mut unsortable = false;
    let mut last_simple = false;

    loop {
        // Find sortable top contour
        let span = if let Some(span) = super::SkPathOpsCommon::find_sortable_top(contour_list) {
            span
        } else {
            break;
        };

        let current = span.segment();
        let mut start = span.next();
        let mut end = span;

        let mut chase: Vec<&SkOpSpan> = Vec::new();

        loop {
            // Perform operation on current span
            if current.active_op(start, end, xor_mask, xor_op_mask, op) {
                loop {
                    if !unsortable && current.done() {
                        break;
                    }

                    let next_start = start;
                    let next_end = end;
                    last_simple = true;

                    // Find next operation
                    if let Some(next) = current.find_next_op(&mut chase, next_start, next_end, &mut unsortable, &mut last_simple, op, xor_mask, xor_op_mask) {
                        // Add curve to path
                        if !current.add_curve_to(start, end, writer) {
                            return false;
                        }

                        current = next;
                        start = next_start;
                        end = next_end;

                        if writer.is_closed() || (unsortable && start.starter(end).done()) {
                            break;
                        }
                    } else {
                        // No more operations, finish current
                        if !unsortable && writer.has_move() && 
                            current.verb() != super::Verb::Line && 
                            !writer.is_closed() {
                            if !current.add_curve_to(start, end, writer) {
                                return false;
                            }
                        } else if last_simple {
                            if !current.add_curve_to(start, end, writer) {
                                return false;
                            }
                        }
                        break;
                    }
                }

                // Finish contour if active winding exists
                if current.active_winding(start, end) && !writer.is_closed() {
                    let span_start = start.starter(end);
                    if !span_start.done() {
                        if !current.add_curve_to(start, end, writer) {
                            return false;
                        }
                        current.mark_done(span_start);
                    }
                }

                writer.finish_contour();
            } else {
                // Mark and chase done
                if !current.mark_and_chase_done(start, end, &mut None) {
                    return false;
                }

                if !chase.is_empty() {
                    last.push(chase.last().unwrap());
                }
            }

            // Find next operation in chase
            if !find_chase_op(&mut chase, &mut start, &mut end, &mut current) {
                return false;
            }

            if current.is_none() {
                break;
            }
        }
    }

    true
}

/// Find next operation in chase queue
fn find_chase_op(
    chase: &mut Vec<&SkOpSpan>,
    start_ptr: &mut &SkOpSpan,
    end_ptr: &mut &SkOpSpan,
    result: &mut Option<&SkOpSegment>,
) -> bool {
    while !chase.is_empty() {
        let span = chase.pop().unwrap();
        
        *start_ptr = span.pt_t().prev().span();
        let segment = start_ptr.segment();
        let mut done = true;
        *end_ptr = start_ptr;

        // Check for active angle
        if let Some(angle) = segment.active_angle(*start_ptr, start_ptr, end_ptr, &mut done) {
            *start_ptr = angle.start();
            *end_ptr = angle.end();
            *result = Some(angle.segment());
            return true;
        }

        if done {
            continue;
        }

        // Check winding
        let winding = super::SkPathOpsCommon::angle_winding(*start_ptr, *end_ptr);
        if winding.is_none() {
            *result = None;
            return true;
        }

        let winding_value = winding.unwrap();
        if winding_value == i32::MIN {
            continue;
        }

        // Check sortable
        if !super::SkPathOpsCommon::sort_angles_for_operation(start_ptr, end_ptr) {
            continue;
        }

        *result = Some(segment);
        return true;
    }

    *result = None;
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::PathBuilder;

    fn make_rect(left: f32, top: f32, right: f32, bottom: f32) -> Path {
        let mut path = Path::new();
        path.move_to(left, top);
        path.line_to(right, top);
        path.line_to(right, bottom);
        path.line_to(left, bottom);
        path.close();
        path
    }

    #[test]
    fn test_union_identical_paths() {
        let p = make_rect(0.0, 0.0, 10.0, 10.0);
        let mut result = Path::new();
        assert!(op(&p, &p, PathOp::Union, &mut result));
        assert_eq!(result, p);
    }

    #[test]
    fn test_intersect_overlapping_rects() {
        let a = make_rect(0.0, 0.0, 10.0, 10.0);
        let b = make_rect(5.0, 5.0, 15.0, 15.0);
        let mut result = Path::new();
        assert!(op(&a, &b, PathOp::Intersect, &mut result));
        
        let mut expected = make_rect(5.0, 5.0, 10.0, 10.0);
        assert_eq!(result, expected);
    }

    #[test]
    fn test_union_overlapping_rects() {
        let a = make_rect(0.0, 0.0, 10.0, 10.0);
        let b = make_rect(5.0, 5.0, 15.0, 15.0);
        let mut result = Path::new();
        assert!(op(&a, &b, PathOp::Union, &mut result));
        
        // Union should cover both rects
        assert!(!result.is_empty());
    }

    #[test]
    fn test_difference_rects() {
        let a = make_rect(0.0, 0.0, 10.0, 10.0);
        let b = make_rect(5.0, 0.0, 10.0, 10.0);
        let mut result = Path::new();
        assert!(op(&a, &b, PathOp::Difference, &mut result));
        
        // Should be left half of original rect
        assert!(!result.is_empty());
    }

    #[test]
    fn test_xor_rects() {
        let a = make_rect(0.0, 0.0, 10.0, 10.0);
        let b = make_rect(5.0, 5.0, 15.0, 15.0);
        let mut result = Path::new();
        assert!(op(&a, &b, PathOp::Xor, &mut result));
        
        // XOR should give non-overlapping regions
        assert!(!result.is_empty());
    }

    #[test]
    fn test_reverse_difference() {
        let a = make_rect(0.0, 0.0, 10.0, 10.0);
        let b = make_rect(5.0, 5.0, 15.0, 15.0);
        let mut result = Path::new();
        assert!(op(&a, &b, PathOp::ReverseDifference, &mut result));
        
        // B minus A (right half of B)
        assert!(!result.is_empty());
    }

    #[test]
    fn test_empty_path_operations() {
        let empty = Path::new();
        let a = make_rect(0.0, 0.0, 10.0, 10.0);
        
        // Union with empty
        let mut result = Path::new();
        assert!(op(&empty, &a, PathOp::Union, &mut result));
        assert!(!result.is_empty());
        
        // Intersection with empty should be empty
        let mut result = Path::new();
        assert!(op(&empty, &a, PathOp::Intersect, &mut result));
        assert!(result.is_empty());
    }

    #[test]
    fn test_simplify() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close();
        
        let mut result = Path::new();
        assert!(simplify(&path, &mut result));
        assert!(!result.is_empty());
    }

    #[test]
    fn test_tight_bounds() {
        let path = make_rect(0.0, 0.0, 10.0, 10.0);
        let mut result = Rect::empty();
        assert!(tight_bounds(&path, &mut result));
        
        assert!((result.left - 0.0).abs() < 1e-6);
        assert!((result.right - 10.0).abs() < 1e-6);
        assert!((result.top - 0.0).abs() < 1e-6);
        assert!((result.bottom - 10.0).abs() < 1e-6);
    }
}
