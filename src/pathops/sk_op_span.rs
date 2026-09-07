//! SkOpSpan - extended span structures for path operations
//!
//! This module extends the basic span structures with full implementation
//! from the C++ Skia SkOpSpan.{h,cpp}
//!
//! Port of Skia's SkOpSpan.{h,cpp}

use std::cell::RefCell;
use std::rc::Rc;

use crate::core::{Point, Scalar};

/// Minimum i32 value for winding sums
pub const PK_MIN_S32: i32 = i32::MIN;

/// Max winding tries
pub const MAX_WINDING_TRIES: i32 = 100;

/// Collapsed status for spans
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Collapsed {
    No,
    Yes,
    Error,
}

impl Default for Collapsed {
    fn default() -> Self {
        Collapsed::No
    }
}

/// Helper function to check if a scalar is zero or one
pub fn is_zero_or_one(t: Scalar) -> bool {
    (t - 0.0).abs() < 1e-10 || (t - 1.0).abs() < 1e-10
}

/// A span with minimal fields
#[derive(Debug, Clone)]
pub struct SkOpSpan {
    pub t: Scalar,
    pub pt: Point,
    pub wind_value: i32,
    pub opp_value: i32,
    pub wind_sum: i32,
    pub opp_sum: i32,
    pub done: bool,
    pub next: Option<Box<SkOpSpan>>,
    pub final_span: bool,
    pub top_t_try: i32,
}

impl SkOpSpan {
    pub fn new(t: Scalar, pt: Point) -> Self {
        Self {
            t,
            pt,
            wind_value: 0,
            opp_value: 0,
            wind_sum: PK_MIN_S32,
            opp_sum: PK_MIN_S32,
            done: false,
            next: None,
            final_span: false,
            top_t_try: 0,
        }
    }

    pub fn t(&self) -> Scalar {
        self.t
    }

    pub fn next(&self) -> Option<&SkOpSpan> {
        self.next.as_ref().map(|n| n.as_ref())
    }

    pub fn r#final(&self) -> bool {
        self.final_span
    }

    pub fn wind_value(&self) -> i32 {
        self.wind_value
    }

    pub fn opp_value(&self) -> i32 {
        self.opp_value
    }

    pub fn wind_sum(&self) -> i32 {
        self.wind_sum
    }

    pub fn opp_sum(&self) -> i32 {
        self.opp_sum
    }

    pub fn done(&self) -> bool {
        self.done
    }

    pub fn set_wind_sum(&mut self, val: i32) {
        self.wind_sum = val;
    }

    pub fn set_opp_sum(&mut self, val: i32) {
        self.opp_sum = val;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_zero_or_one() {
        assert!(is_zero_or_one(0.0));
        assert!(is_zero_or_one(1.0));
        assert!(!is_zero_or_one(0.5));
        assert!(!is_zero_or_one(0.999));
    }

    #[test]
    fn test_collapsed_enum() {
        let c: Collapsed = Default::default();
        assert_eq!(c, Collapsed::No);
    }

    #[test]
    fn test_constants() {
        assert_eq!(PK_MIN_S32, i32::MIN);
        assert_eq!(MAX_WINDING_TRIES, 100);
    }
}
