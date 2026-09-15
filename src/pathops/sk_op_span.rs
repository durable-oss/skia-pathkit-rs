//! SkOpSpan - extended span structures for path operations
//!
//! This module extends the basic span structures with full implementation
//! from the C++ Skia SkOpSpan.{h,cpp}
//!
//! Port of Skia's SkOpSpan.{h,cpp}

use crate::core::{Point, Scalar};

/// PtT node (declared in SkOpSpan.h per C++)
#[derive(Debug, Clone)]
pub struct SkOpPtT {
    /// Parametric position along the owning segment, in [0, 1].
    pub f_t: Scalar,
    /// The point the segment reaches at `f_t`.
    pub f_pt: Point,
    /// Arena index of the span this node belongs to.
    pub f_span: Option<usize>, // arena index
    /// Next node in the ring of coincident points. The ring links every
    /// segment that passes through this same location.
    pub f_next: Option<usize>,
    /// True once the node has been removed from its ring.
    pub f_deleted: bool,
    /// True when another node in the ring sits at the same point.
    pub f_duplicate_pt: bool,
    /// True when this node participates in a coincident run of spans, rather
    /// than a single crossing.
    pub f_coincident: bool,
}

impl SkOpPtT {
    /// Constructs a node at `t` on `span`, sitting at `pt`, not yet linked
    /// into a ring.
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

    /// True if the node is still linked to a span and has not been deleted.
    pub fn active(&self) -> bool {
        !self.f_deleted && self.f_span.is_some()
    }

    /// True if this node sits at `t`, within a fixed tolerance.
    pub fn contains(&self, t: Scalar) -> bool {
        (self.f_t - t).abs() < 1e-10
    }

    /// Marks the node deleted or restores it.
    pub fn set_deleted(&mut self, del: bool) {
        self.f_deleted = del;
    }
}

/// Minimum i32 value for winding sums
pub const PK_MIN_S32: i32 = i32::MIN;

/// Collapsed status for spans
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Collapsed {
    /// The span has measurable extent.
    #[default]
    No,
    /// The span's ends coincide, so it contributes no length.
    Yes,
    /// The collapse test could not reach an answer.
    Error,
}

/// Helper function to check if a scalar is zero or one
pub fn is_zero_or_one(t: Scalar) -> bool {
    (t - 0.0).abs() < 1e-10 || (t - 1.0).abs() < 1e-10
}

/// Base for spans (terminal t==1 shares this without winding)
#[derive(Debug, Clone)]
pub struct SkOpSpanBase {
    /// Parametric position of this span's start along its segment.
    pub f_t: Scalar,
    /// The point the segment reaches at `f_t`.
    pub f_pt: Point,
    /// Arena index of the segment that owns this span.
    pub f_segment: Option<usize>, // arena index
    /// This span's own point-and-t node, the head of its ring.
    ///
    /// Port of `SkOpSpanBase::fPtT`.
    pub f_ptt: Option<usize>,
    /// Next span in the circular list of coincident span ends.
    pub f_coin_end: Option<usize>, // circular coin list
    /// Angle arriving at this span, used to order spans around a crossing.
    pub f_from_angle: Option<usize>,
    /// Previous span in the segment, or `None` at the head.
    pub f_prev: Option<usize>,
    /// Next span in the segment.
    ///
    /// C++ puts `fNext` on `SkOpSpan` rather than the base, since a terminal
    /// span has nothing after it. The arena pools both roles together, so the
    /// edge lives here and is `None` for the tail.
    pub f_next: Option<usize>,
    /// How many times this span has been added to an output contour.
    pub f_span_adds: i32,
    /// True once this span's point has been snapped to its ring's location.
    pub f_aligned: bool,
    /// True once the chaining walk has visited this span.
    pub f_chased: bool,
    /// Whether the span's ends coincide.
    pub f_collapsed: Collapsed,

    // The fields below are C++'s `SkOpSpan`, the derived type. The arena
    // pools both roles in one `Vec`, so they live here and go unread on the
    // terminal span - which is exactly what the C++ inheritance means.
    /// Ring of spans coincident with this one.
    pub f_coincident: Option<usize>,
    /// Angle leaving this span.
    pub f_to_angle: Option<usize>,
    /// Accumulated winding, or [`PK_MIN_S32`] when not yet computed.
    pub f_wind_sum: i32,
    /// Accumulated opposite-operand winding.
    pub f_opp_sum: i32,
    /// Winding contribution of this span.
    pub f_wind_value: i32,
    /// Opposite-operand winding contribution.
    pub f_opp_value: i32,
    /// How many times a top has been sought from here.
    pub f_top_t_try: i32,
    /// True once this span has been resolved.
    pub f_done: bool,
    /// True once this span has been emitted.
    pub f_already_added: bool,
}

/// A span carrying winding data.
///
/// C++ has `SkOpSpan` derive from `SkOpSpanBase`, the extra fields being the
/// winding state a terminal span does not need. The arena keeps both in one
/// pool, so this is an alias: whether a span is terminal is answered by
/// [`OpArena::span_is_final`](super::sk_op_arena::OpArena::span_is_final)
/// rather than by its type.
pub type SkOpSpan = SkOpSpanBase;

impl SkOpSpanBase {
    /// Constructs a span at `t` on `segment`, sitting at `pt`, with no links
    /// and no winding computed yet.
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
            f_coincident: None,
            f_to_angle: None,
            f_wind_sum: PK_MIN_S32,
            f_opp_sum: PK_MIN_S32,
            f_wind_value: 0,
            f_opp_value: 0,
            f_top_t_try: 0,
            f_done: false,
            f_already_added: false,
        }
    }

    /// Returns this span's parametric position along its segment.
    pub fn t(&self) -> Scalar { self.f_t }
    /// Returns the point the segment reaches at this span's t.
    pub fn pt(&self) -> Point { self.f_pt }
    /// Returns whether this span's ends coincide.
    pub fn collapsed(&self) -> Collapsed { self.f_collapsed }

    /// Returns this span's winding contribution.
    pub fn wind_value(&self) -> i32 { self.f_wind_value }
    /// Returns this span's opposite-operand contribution.
    pub fn opp_value(&self) -> i32 { self.f_opp_value }
    /// Returns the accumulated winding, or [`PK_MIN_S32`] if not yet computed.
    pub fn wind_sum(&self) -> i32 { self.f_wind_sum }
    /// Returns the accumulated opposite-operand winding.
    pub fn opp_sum(&self) -> i32 { self.f_opp_sum }
    /// Returns true once this span has been resolved.
    pub fn done(&self) -> bool { self.f_done }
    /// Sets the accumulated winding.
    pub fn set_wind_sum(&mut self, val: i32) { self.f_wind_sum = val; }
    /// Sets the accumulated opposite-operand winding.
    pub fn set_opp_sum(&mut self, val: i32) { self.f_opp_sum = val; }
    /// Marks this span resolved.
    pub fn set_done(&mut self, done: bool) { self.f_done = done; }

    /// Returns true when this span contributes nothing to either operand.
    ///
    /// Port of `SkOpSpan::isCanceled`.
    #[must_use]
    pub fn is_canceled(&self) -> bool {
        self.f_wind_value == 0 && self.f_opp_value == 0
    }

    /// Returns true when this span has already been emitted.
    ///
    /// Port of `SkOpSpan::alreadyAdded`.
    #[must_use]
    pub fn already_added(&self) -> bool {
        self.f_already_added
    }

    /// Marks this span emitted.
    ///
    /// Port of `SkOpSpan::markAdded`.
    pub fn mark_added(&mut self) {
        self.f_already_added = true;
    }

    /// Returns how many rays have already been tried from this span.
    ///
    /// Port of `SkOpSpan::fTopTTry`. Each failed ray bumps it, so the next
    /// attempt picks a different t and a different direction rather than
    /// repeating one that did not work.
    #[must_use]
    pub fn top_t_try(&self) -> i32 {
        self.f_top_t_try
    }

    /// Records that another ray has been tried from this span.
    pub fn bump_top_t_try(&mut self) {
        self.f_top_t_try += 1;
    }

    /// Counts another span added through this one.
    ///
    /// Port of `SkOpSpanBase::bumpSpanAdds`.
    pub fn bump_span_adds(&mut self) {
        self.f_span_adds += 1;
    }

    /// Returns true when this span has been walked during chasing.
    ///
    /// Port of `SkOpSpanBase::chased`.
    #[must_use]
    pub fn chased(&self) -> bool {
        self.f_chased
    }

    /// Records whether this span has been walked during chasing.
    ///
    /// Port of `SkOpSpanBase::setChased`.
    pub fn set_chased(&mut self, chased: bool) {
        self.f_chased = chased;
    }

    /// Sets this span's winding contribution.
    ///
    /// Port of `SkOpSpan::setWindValue`.
    pub fn set_wind_value(&mut self, wind_value: i32) {
        debug_assert!(wind_value >= 0);
        debug_assert_eq!(self.f_wind_sum, PK_MIN_S32);
        self.f_wind_value = wind_value;
    }

    /// Sets this span's opposite-operand contribution.
    ///
    /// Port of `SkOpSpan::setOppValue`.
    pub fn set_opp_value(&mut self, opp_value: i32) {
        debug_assert_eq!(self.f_opp_sum, PK_MIN_S32);
        self.f_opp_value = opp_value;
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
    }
}
