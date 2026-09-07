//! SkPathOpsCommon - common utilities for path operations
//!
//! Port of Skia's SkPathOpsCommon.{h,cpp}
//!
//! This module provides common functions used across path operations including
//! winding computation, contour sorting, and coincidence handling.

use super::sk_op_contour::SkOpContour;

/// Tolerance for approximate comparisons
const APPROX_EPSILON: f64 = 1e-10;

/// Check if two scalars are approximately equal
pub fn approximately_equal(a: f64, b: f64) -> bool {
    (a - b).abs() < APPROX_EPSILON
}

/// Check if a scalar is approximately zero
pub fn approximately_zero(a: f64) -> bool {
    a.abs() < APPROX_EPSILON
}

/// Check if b is between a and c (inclusive)
pub fn between(a: f64, b: f64, c: f64) -> bool {
    b >= a && b <= c
}

/// Check if b is approximately between a and c
pub fn approximately_between(a: f64, b: f64, c: f64) -> bool {
    b >= a - APPROX_EPSILON && b <= c + APPROX_EPSILON
}

/// Sort contour list by position
pub fn sort_contour_list(
    contour_list: &mut Vec<SkOpContour>,
    _even_odd: bool,
    _opp_even_odd: bool,
) -> bool {
    if contour_list.is_empty() {
        return false;
    }

    // Sort by position (top, then left)
    contour_list.sort_by(|a, b| {
        let a_top = a.bounds().top;
        let b_top = b.bounds().top;
        if (a_top - b_top).abs() > 1e-10 {
            a_top
                .partial_cmp(&b_top)
                .unwrap_or(std::cmp::Ordering::Equal)
        } else {
            let a_left = a.bounds().left;
            let b_left = b.bounds().left;
            a_left
                .partial_cmp(&b_left)
                .unwrap_or(std::cmp::Ordering::Equal)
        }
    });

    true
}

/// Calculate angles for all contours
pub fn calc_angles(contour_list: &mut [SkOpContour]) {
    for contour in contour_list.iter_mut() {
        contour.calc_angles();
    }
}

/// Check for missing coincidence
pub fn missing_coincidence(contour_list: &[SkOpContour]) -> bool {
    contour_list.iter().any(|c| c.missing_coincidence())
}

/// Move multiples
pub fn move_multiples(contour_list: &mut [SkOpContour]) -> bool {
    contour_list.iter_mut().all(|c| c.move_multiples())
}

/// Move nearby points
pub fn move_nearby(contour_list: &mut [SkOpContour]) -> bool {
    contour_list.iter_mut().all(|c| c.move_nearby())
}

/// Sort angles
pub fn sort_angles(contour_list: &mut [SkOpContour]) -> bool {
    contour_list.iter_mut().all(|c| c.sort_angles())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sort_contour_list_empty() {
        let mut contours: Vec<SkOpContour> = vec![];
        assert!(!sort_contour_list(&mut contours, false, false));
    }

    #[test]
    fn test_sort_contour_list_single() {
        let mut contours = vec![SkOpContour::new()];
        assert!(sort_contour_list(&mut contours, false, false));
        assert_eq!(contours.len(), 1);
    }
}
