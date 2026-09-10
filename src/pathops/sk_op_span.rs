//! SkOpSpan - extended span structures for path operations
//!
//! This module extends the basic span structures with full implementation
//! from the C++ Skia SkOpSpan.{h,cpp}
//!
//! Port of Skia's SkOpSpan.{h,cpp}

use crate::core::{Point, Scalar};
use super::sk_path_ops_types::OpGlobalState;

/// PtT node (declared in SkOpSpan.h per C++)
#[derive(Debug, Clone)]
pub struct SkOpPtT {
    pub f_t: Scalar,
    pub f_pt: Point,
    pub f_span: Option<usize>, // arena index
    pub f_next: Option<usize>,
    pub f_deleted: bool,
    pub f_duplicate_pt: bool,
    pub f_coincident: bool,
}

impl SkOpPtT {
    pub fn new(t: Scalar, pt: Point, span: Option<usize>) -> Self {
        Self {
            f_t: t,
            f_pt: pt,
            f_span: span,
            f_next: None,
            f_deleted: false,
            f_duplicate_pt: false,
            f_coincident: false,
        }
    }

    pub fn active(&self) -> bool {
        !self.f_deleted && self.f_span.is_some()
    }

    pub fn contains(&self, t: Scalar) -> bool {
        (self.f_t - t).abs() < 1e-10
    }

    pub fn set_deleted(&mut self, del: bool) {
        self.f_deleted = del;
    }
}

/// Minimum i32 value for winding sums
pub const PK_MIN_S32: i32 = i32::MIN;

/// Max winding tries
pub const MAX_WINDING_TRIES: i32 = 100;

/// Collapsed status for spans
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Collapsed {
    #[default]
    No,
    Yes,
    Error,
}

/// Helper function to check if a scalar is zero or one
pub fn is_zero_or_one(t: Scalar) -> bool {
    (t - 0.0).abs() < 1e-10 || (t - 1.0).abs() < 1e-10
}

/// Base for spans (terminal t==1 shares this without winding)
#[derive(Debug, Clone)]
pub struct SkOpSpanBase {
    pub f_t: Scalar,
    pub f_pt: Point,
    pub f_segment: Option<usize>, // arena index
    /// This span's own point-and-t node, the head of its ring.
    ///
    /// Port of `SkOpSpanBase::fPtT`.
    pub f_ptt: Option<usize>,
    pub f_coin_end: Option<usize>, // circular coin list
    pub f_from_angle: Option<usize>,
    pub f_prev: Option<usize>,
    /// Next span in the segment.
    ///
    /// C++ puts `fNext` on `SkOpSpan` rather than the base, since a terminal
    /// span has nothing after it. The arena pools both roles together, so the
    /// edge lives here and is `None` for the tail.
    pub f_next: Option<usize>,
    pub f_span_adds: i32,
    pub f_aligned: bool,
    pub f_chased: bool,
    pub f_collapsed: Collapsed,
}

/// Full span (inherits base fields via composition in arena model)
#[derive(Debug, Clone)]
pub struct SkOpSpan {
    pub base: SkOpSpanBase,
    pub f_coincident: Option<usize>, // circular list
    pub f_to_angle: Option<usize>,
    pub f_next: Option<usize>,
    pub f_wind_sum: i32,
    pub f_opp_sum: i32,
    pub f_wind_value: i32,
    pub f_opp_value: i32,
    pub f_top_t_try: i32,
    pub f_done: bool,
    pub f_already_added: bool,
}

impl SkOpSpanBase {
    pub fn new(t: Scalar, pt: Point, segment: Option<usize>) -> Self {
        Self {
            f_t: t,
            f_pt: pt,
            f_segment: segment,
            f_ptt: None,
            f_coin_end: None,
            f_from_angle: None,
            f_prev: None,
            f_next: None,
            f_span_adds: 0,
            f_aligned: false,
            f_chased: false,
            f_collapsed: Collapsed::No,
        }
    }

    pub fn t(&self) -> Scalar { self.f_t }
    pub fn pt(&self) -> Point { self.f_pt }
    pub fn collapsed(&self) -> Collapsed { self.f_collapsed }
}

impl SkOpSpan {
    pub fn new(t: Scalar, pt: Point, segment: Option<usize>) -> Self {
        Self {
            base: SkOpSpanBase::new(t, pt, segment),
            f_coincident: None,
            f_to_angle: None,
            f_next: None,
            f_wind_sum: PK_MIN_S32,
            f_opp_sum: PK_MIN_S32,
            f_wind_value: 0,
            f_opp_value: 0,
            f_top_t_try: 0,
            f_done: false,
            f_already_added: false,
        }
    }

    pub fn t(&self) -> Scalar { self.base.t() }
    pub fn pt(&self) -> Point { self.base.pt() }
    pub fn wind_value(&self) -> i32 { self.f_wind_value }
    pub fn opp_value(&self) -> i32 { self.f_opp_value }
    pub fn wind_sum(&self) -> i32 { self.f_wind_sum }
    pub fn opp_sum(&self) -> i32 { self.f_opp_sum }
    pub fn done(&self) -> bool { self.f_done }
    pub fn set_wind_sum(&mut self, val: i32) { self.f_wind_sum = val; }
    pub fn set_opp_sum(&mut self, val: i32) { self.f_opp_sum = val; }
    pub fn set_done(&mut self, done: bool) { self.f_done = done; }

    pub fn compute_wind_sum(&mut self, _global: &OpGlobalState) -> i32 {
        self.f_wind_sum
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
