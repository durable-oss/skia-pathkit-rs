//! The arena that owns the pathops object graph, and the global state beside it.
//!
//! Port of `SkOpGlobalState` (`SkPathOpsTypes.h`) plus the `SkArenaAlloc` the
//! pathops engine allocates every node from.
//!
//! # Why indices instead of pointers
//!
//! `SkOpSpan`, `SkOpSpanBase`, `SkOpPtT`, `SkOpSegment`, `SkOpAngle` and
//! `SkCoincidentSpans` form one cyclic, mutable graph. In C++ each node is
//! arena-allocated and holds raw pointers in both directions, and the whole
//! arena is dropped at the end rather than nodes being freed individually.
//!
//! `Rc<RefCell<_>>` would model that badly: every traversal step would borrow,
//! the cycles would leak, and a walk that holds one node while reaching for its
//! neighbour would panic at runtime rather than fail to compile. Index handles
//! into `Vec` pools give the same aliasing freedom the C++ has, with the
//! lifetime of the whole graph tied to one owner.
//!
//! The cost is that traversal needs the arena in hand: `state.span(id).next()`
//! rather than `span->next()`. Every method in the engine that walks the graph
//! therefore takes `&OpArena` or `&mut OpArena`.
//!
//! # Handles
//!
//! The five handle types are distinct newtypes. A bare `usize` for all of them
//! would let a span id index the segment pool with nothing to catch it; here
//! that does not compile.
//!
//! # Deletion
//!
//! Nothing is ever removed from a pool. C++ marks nodes `fDeleted` and lets the
//! arena drop wholesale; compacting a pool here would invalidate every live
//! handle. [`SkOpPtT::f_deleted`](super::sk_op_span::SkOpPtT::f_deleted) stays
//! the way a node is retired.
//!
//! # Not ported
//!
//! The `DEBUG_COIN` and `DEBUG_T_SECT_LOOP_COUNT` dictionaries
//! (`fCoinDict`, `fDebugLoopCount`) are omitted. They exist to log coincidence
//! decisions and t-section iteration counts under debug builds and have no
//! effect on results.

use super::sk_op_angle::SkOpAngle;
use super::sk_op_span::{SkOpPtT, SkOpSpanBase};
use super::sk_path_ops_types::OpPhase;

/// Maximum number of times winding computation is retried before giving up.
///
/// Port of `SkOpGlobalState::kMaxWindingTries`.
pub const MAX_WINDING_TRIES: i32 = 10;

/// Handle to a span in [`OpArena::spans`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SpanId(pub u32);

/// Handle to a point-and-t node in [`OpArena::pt_ts`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PtTId(pub u32);

/// Handle to a segment in [`OpArena::segments`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SegmentId(pub u32);

/// Handle to an angle in [`OpArena::angles`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AngleId(pub u32);

/// Handle to a coincident-span record in [`OpArena::coins`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CoinId(pub u32);

/// Generates the index/handle plumbing shared by every handle type.
macro_rules! impl_handle {
    ($name:ident) => {
        impl $name {
            /// Returns the handle for pool position `index`.
            #[must_use]
            pub const fn new(index: usize) -> Self {
                Self(index as u32)
            }

            /// Returns the pool position this handle refers to.
            #[must_use]
            pub const fn index(self) -> usize {
                self.0 as usize
            }
        }
    };
}

impl_handle!(SpanId);
impl_handle!(PtTId);
impl_handle!(SegmentId);
impl_handle!(AngleId);
impl_handle!(CoinId);

/// One record of two segments running along the same path.
///
/// Port of `SkCoincidentSpans`. Kept here rather than in
/// [`super::sk_op_coincidence`] so the arena owns every node type in one place.
#[derive(Debug, Clone, Default)]
pub struct SkCoincidentSpans {
    /// Next record in the coincidence list.
    pub f_next: Option<CoinId>,
    /// Start of the run on the first segment.
    pub f_coin_ptt_start: Option<PtTId>,
    /// End of the run on the first segment.
    pub f_coin_ptt_end: Option<PtTId>,
    /// Start of the run on the second segment.
    pub f_opp_ptt_start: Option<PtTId>,
    /// End of the run on the second segment.
    pub f_opp_ptt_end: Option<PtTId>,
    /// True when the two runs are traced in opposite directions.
    pub f_flipped: bool,
    /// Debug id.
    pub f_id: i32,
}

/// A segment as the arena stores it.
///
/// Only the graph edges live here; the geometry stays on
/// [`super::sk_op_segment::SkOpSegment`], which item 05 threads through this
/// arena. Splitting it this way keeps item 02 from having to rewrite the
/// geometry code before the graph exists to hold it.
#[derive(Debug, Clone, Default)]
pub struct ArenaSegment {
    /// First span of the segment.
    pub f_head: Option<SpanId>,
    /// Terminal span, at t == 1.
    pub f_tail: Option<SpanId>,
    /// Next segment in the contour.
    pub f_next: Option<SegmentId>,
    /// Previous segment in the contour.
    pub f_prev: Option<SegmentId>,
    /// Number of spans in the segment.
    pub f_count: i32,
    /// Number of spans already resolved.
    pub f_done_count: i32,
    /// True once every span has been walked.
    pub f_done: bool,
    /// Debug id.
    pub f_id: i32,
}

/// Owns every node in the pathops graph, and the state carried alongside it.
///
/// Port of `SkOpGlobalState` together with its `SkArenaAlloc`. The
/// `OpGlobalState` in [`super::sk_path_ops_types`] was a bag of three
/// accessors; it is folded in here rather than kept as a second state object.
#[derive(Debug)]
pub struct OpArena {
    /// Span pool. `SkOpSpan` and `SkOpSpanBase` share it: a base is a span
    /// whose winding fields go unread, exactly as the C++ inheritance means.
    spans: Vec<SkOpSpanBase>,
    /// Point-and-t pool.
    pt_ts: Vec<SkOpPtT>,
    /// Segment graph-edge pool.
    segments: Vec<ArenaSegment>,
    /// Angle pool.
    angles: Vec<SkOpAngle>,
    /// Coincident-run pool.
    coins: Vec<SkCoincidentSpans>,

    /// First segment of the contour list.
    contour_head: Option<SegmentId>,
    /// Head of the coincidence list.
    coincidence: Option<CoinId>,

    /// How deeply nested the contour being wound is.
    nested: i32,
    /// True once a span has been allocated in the current pass.
    allocated_op_span: bool,
    /// True when winding could not be computed.
    winding_failed: bool,
    /// Which stage of the operation is running.
    phase: OpPhase,

    /// Debug id counters, handed out by the `next_*_id` methods.
    next_angle_id: i32,
    next_coin_id: i32,
    next_contour_id: i32,
    next_ptt_id: i32,
    next_segment_id: i32,
    next_span_id: i32,
}

impl Default for OpArena {
    fn default() -> Self {
        Self::new()
    }
}

impl OpArena {
    /// Returns an empty arena.
    #[must_use]
    pub fn new() -> Self {
        Self {
            spans: Vec::new(),
            pt_ts: Vec::new(),
            segments: Vec::new(),
            angles: Vec::new(),
            coins: Vec::new(),
            contour_head: None,
            coincidence: None,
            nested: 0,
            allocated_op_span: false,
            winding_failed: false,
            phase: OpPhase::Intersecting,
            next_angle_id: 0,
            next_coin_id: 0,
            next_contour_id: 0,
            next_ptt_id: 0,
            next_segment_id: 0,
            next_span_id: 0,
        }
    }

    // --- allocation ------------------------------------------------------

    /// Adds `span` to the pool and returns its handle.
    ///
    /// Also sets `allocated_op_span`, matching
    /// `SkOpGlobalState::setAllocatedOpSpan`.
    pub fn alloc_span(&mut self, span: SkOpSpanBase) -> SpanId {
        self.allocated_op_span = true;
        self.next_span_id += 1;
        self.spans.push(span);
        SpanId::new(self.spans.len() - 1)
    }

    /// Adds `pt_t` to the pool and returns its handle.
    pub fn alloc_ptt(&mut self, pt_t: SkOpPtT) -> PtTId {
        self.next_ptt_id += 1;
        self.pt_ts.push(pt_t);
        PtTId::new(self.pt_ts.len() - 1)
    }

    /// Adds `segment` to the pool and returns its handle.
    pub fn alloc_segment(&mut self, mut segment: ArenaSegment) -> SegmentId {
        segment.f_id = self.next_segment_id;
        self.next_segment_id += 1;
        self.segments.push(segment);
        SegmentId::new(self.segments.len() - 1)
    }

    /// Adds `angle` to the pool and returns its handle.
    pub fn alloc_angle(&mut self, mut angle: SkOpAngle) -> AngleId {
        angle.f_id = self.next_angle_id;
        self.next_angle_id += 1;
        self.angles.push(angle);
        AngleId::new(self.angles.len() - 1)
    }

    /// Adds `coin` to the pool and returns its handle.
    pub fn alloc_coin(&mut self, mut coin: SkCoincidentSpans) -> CoinId {
        coin.f_id = self.next_coin_id;
        self.next_coin_id += 1;
        self.coins.push(coin);
        CoinId::new(self.coins.len() - 1)
    }

    // --- access ----------------------------------------------------------

    /// Returns the span `id` refers to.
    #[must_use]
    pub fn span(&self, id: SpanId) -> &SkOpSpanBase {
        &self.spans[id.index()]
    }

    /// Returns the span `id` refers to, mutably.
    pub fn span_mut(&mut self, id: SpanId) -> &mut SkOpSpanBase {
        &mut self.spans[id.index()]
    }

    /// Returns the point-and-t node `id` refers to.
    #[must_use]
    pub fn ptt(&self, id: PtTId) -> &SkOpPtT {
        &self.pt_ts[id.index()]
    }

    /// Returns the point-and-t node `id` refers to, mutably.
    pub fn ptt_mut(&mut self, id: PtTId) -> &mut SkOpPtT {
        &mut self.pt_ts[id.index()]
    }

    /// Returns the segment `id` refers to.
    #[must_use]
    pub fn segment(&self, id: SegmentId) -> &ArenaSegment {
        &self.segments[id.index()]
    }

    /// Returns the segment `id` refers to, mutably.
    pub fn segment_mut(&mut self, id: SegmentId) -> &mut ArenaSegment {
        &mut self.segments[id.index()]
    }

    /// Returns the angle `id` refers to.
    #[must_use]
    pub fn angle(&self, id: AngleId) -> &SkOpAngle {
        &self.angles[id.index()]
    }

    /// Returns the angle `id` refers to, mutably.
    pub fn angle_mut(&mut self, id: AngleId) -> &mut SkOpAngle {
        &mut self.angles[id.index()]
    }

    /// Returns the coincident run `id` refers to.
    #[must_use]
    pub fn coin(&self, id: CoinId) -> &SkCoincidentSpans {
        &self.coins[id.index()]
    }

    /// Returns the coincident run `id` refers to, mutably.
    pub fn coin_mut(&mut self, id: CoinId) -> &mut SkCoincidentSpans {
        &mut self.coins[id.index()]
    }

    /// Returns how many spans have been allocated.
    #[must_use]
    pub fn span_count(&self) -> usize {
        self.spans.len()
    }

    /// Returns how many point-and-t nodes have been allocated.
    #[must_use]
    pub fn ptt_count(&self) -> usize {
        self.pt_ts.len()
    }

    /// Returns how many segments have been allocated.
    #[must_use]
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    /// Returns how many angles have been allocated.
    #[must_use]
    pub fn angle_count(&self) -> usize {
        self.angles.len()
    }

    /// Returns how many coincident runs have been allocated.
    #[must_use]
    pub fn coin_count(&self) -> usize {
        self.coins.len()
    }

    // --- graph roots -----------------------------------------------------

    /// Returns the first segment of the contour list.
    ///
    /// Port of `SkOpGlobalState::contourHead`.
    #[must_use]
    pub fn contour_head(&self) -> Option<SegmentId> {
        self.contour_head
    }

    /// Sets the first segment of the contour list.
    pub fn set_contour_head(&mut self, head: Option<SegmentId>) {
        self.contour_head = head;
    }

    /// Returns the head of the coincidence list.
    ///
    /// Port of `SkOpGlobalState::coincidence`.
    #[must_use]
    pub fn coincidence(&self) -> Option<CoinId> {
        self.coincidence
    }

    /// Sets the head of the coincidence list.
    pub fn set_coincidence(&mut self, head: Option<CoinId>) {
        self.coincidence = head;
    }

    // --- state -----------------------------------------------------------

    /// Returns true if a span was allocated since the last reset.
    ///
    /// Port of `allocatedOpSpan`.
    #[must_use]
    pub fn allocated_op_span(&self) -> bool {
        self.allocated_op_span
    }

    /// Clears the allocated-span flag.
    ///
    /// Port of `resetAllocatedOpSpan`.
    pub fn reset_allocated_op_span(&mut self) {
        self.allocated_op_span = false;
    }

    /// Increases the nesting depth by one.
    ///
    /// Port of `bumpNested`.
    pub fn bump_nested(&mut self) {
        self.nested += 1;
    }

    /// Resets the nesting depth to zero.
    ///
    /// Port of `clearNested`.
    pub fn clear_nested(&mut self) {
        self.nested = 0;
    }

    /// Returns the current nesting depth.
    #[must_use]
    pub fn nested(&self) -> i32 {
        self.nested
    }

    /// Returns which stage of the operation is running.
    #[must_use]
    pub fn phase(&self) -> OpPhase {
        self.phase
    }

    /// Moves to a different stage of the operation.
    ///
    /// The C++ asserts the phase actually changes, which catches a stage being
    /// entered twice; the same check is kept as a debug assertion.
    pub fn set_phase(&mut self, phase: OpPhase) {
        debug_assert_ne!(self.phase, phase, "phase set to what it already was");
        self.phase = phase;
    }

    /// Returns true when winding computation gave up.
    #[must_use]
    pub fn winding_failed(&self) -> bool {
        self.winding_failed
    }

    /// Records that winding computation gave up.
    ///
    /// Port of `setWindingFailed`.
    pub fn set_winding_failed(&mut self) {
        self.winding_failed = true;
    }

    // --- debug ids -------------------------------------------------------

    /// Returns the next angle debug id.
    pub fn next_angle_id(&mut self) -> i32 {
        let id = self.next_angle_id;
        self.next_angle_id += 1;
        id
    }

    /// Returns the next contour debug id.
    pub fn next_contour_id(&mut self) -> i32 {
        let id = self.next_contour_id;
        self.next_contour_id += 1;
        id
    }

    /// Returns the next span debug id.
    pub fn next_span_id(&mut self) -> i32 {
        let id = self.next_span_id;
        self.next_span_id += 1;
        id
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point;

    /// Builds a segment with `count` spans linked head to tail, and links each
    /// span back to the segment. Returns the segment and its span handles.
    fn linked_segment(arena: &mut OpArena, count: usize) -> (SegmentId, Vec<SpanId>) {
        let seg = arena.alloc_segment(ArenaSegment::default());
        let mut ids = Vec::new();
        for i in 0..count {
            let t = i as f32 / (count - 1) as f32;
            let span = SkOpSpanBase::new(t, Point::new(t * 10.0, 0.0), Some(seg.index()));
            ids.push(arena.alloc_span(span));
        }
        // Link next/prev through the chain.
        for i in 0..count {
            if i + 1 < count {
                arena.span_mut(ids[i]).f_next = Some(ids[i + 1].index());
            }
            if i > 0 {
                arena.span_mut(ids[i]).f_prev = Some(ids[i - 1].index());
            }
        }
        arena.segment_mut(seg).f_head = Some(ids[0]);
        arena.segment_mut(seg).f_tail = Some(ids[count - 1]);
        arena.segment_mut(seg).f_count = count as i32;
        (seg, ids)
    }

    #[test]
    fn handles_index_their_own_pool() {
        let a = SpanId::new(3);
        assert_eq!(a.index(), 3);
        let b = SegmentId::new(3);
        assert_eq!(b.index(), 3);
        // Same position, different pools, and not interchangeable. That a
        // SpanId cannot be passed where a SegmentId is wanted is checked by
        // the compiler, not here; see the doc comment on the handle types.
        assert_eq!(a.0, b.0);
    }

    #[test]
    fn allocation_hands_out_distinct_handles() {
        let mut arena = OpArena::new();
        let s0 = arena.alloc_span(SkOpSpanBase::new(0.0, Point::default(), None));
        let s1 = arena.alloc_span(SkOpSpanBase::new(1.0, Point::default(), None));
        assert_ne!(s0, s1);
        assert_eq!(arena.span_count(), 2);
        assert!((arena.span(s0).t() - 0.0).abs() < 1e-9);
        assert!((arena.span(s1).t() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn every_pool_allocates_independently() {
        let mut arena = OpArena::new();
        let _ = arena.alloc_span(SkOpSpanBase::new(0.0, Point::default(), None));
        let p = arena.alloc_ptt(SkOpPtT::new(0.5, Point::default(), None));
        let g = arena.alloc_segment(ArenaSegment::default());
        let a = arena.alloc_angle(SkOpAngle::new());
        let c = arena.alloc_coin(SkCoincidentSpans::default());
        // Each pool starts at zero regardless of what the others hold.
        assert_eq!(p.index(), 0);
        assert_eq!(g.index(), 0);
        assert_eq!(a.index(), 0);
        assert_eq!(c.index(), 0);
        assert_eq!(arena.span_count(), 1);
        assert_eq!(arena.ptt_count(), 1);
        assert_eq!(arena.segment_count(), 1);
        assert_eq!(arena.angle_count(), 1);
        assert_eq!(arena.coin_count(), 1);
    }

    #[test]
    fn a_span_chain_walks_both_ways_and_returns() {
        // The acceptance case: two segments, spans linked, traversed forwards
        // and backwards, ending where the walk began.
        let mut arena = OpArena::new();
        let (seg_a, spans_a) = linked_segment(&mut arena, 4);
        let (seg_b, spans_b) = linked_segment(&mut arena, 3);

        // The two segments are distinct and each knows its own spans.
        assert_ne!(seg_a, seg_b);
        assert_eq!(arena.segment(seg_a).f_count, 4);
        assert_eq!(arena.segment(seg_b).f_count, 3);

        // Walk forward from head to tail.
        let mut cur = arena.segment(seg_a).f_head.expect("head");
        let mut forward = vec![cur];
        while let Some(next) = arena.span(cur).f_next {
            cur = SpanId::new(next);
            forward.push(cur);
        }
        assert_eq!(forward, spans_a);
        assert_eq!(Some(cur), arena.segment(seg_a).f_tail);

        // And back again, arriving at the head we started from.
        let mut backward = vec![cur];
        while let Some(prev) = arena.span(cur).f_prev {
            cur = SpanId::new(prev);
            backward.push(cur);
        }
        backward.reverse();
        assert_eq!(backward, spans_a);
        assert_eq!(Some(cur), arena.segment(seg_a).f_head);
    }

    #[test]
    fn a_span_reaches_the_segment_that_owns_it() {
        let mut arena = OpArena::new();
        let (seg, spans) = linked_segment(&mut arena, 3);
        for id in &spans {
            let owner = arena.span(*id).f_segment.expect("span knows its segment");
            assert_eq!(SegmentId::new(owner), seg);
        }
    }

    #[test]
    fn segments_link_into_a_contour_list() {
        let mut arena = OpArena::new();
        let a = arena.alloc_segment(ArenaSegment::default());
        let b = arena.alloc_segment(ArenaSegment::default());
        let c = arena.alloc_segment(ArenaSegment::default());
        arena.segment_mut(a).f_next = Some(b);
        arena.segment_mut(b).f_prev = Some(a);
        arena.segment_mut(b).f_next = Some(c);
        arena.segment_mut(c).f_prev = Some(b);
        arena.set_contour_head(Some(a));

        let mut walked = Vec::new();
        let mut cur = arena.contour_head();
        while let Some(id) = cur {
            walked.push(id);
            cur = arena.segment(id).f_next;
        }
        assert_eq!(walked, vec![a, b, c]);
    }

    #[test]
    fn ptt_nodes_form_a_ring() {
        // A t value shared between two segments links their PtT nodes into a
        // loop, which is how the engine finds every segment through a point.
        let mut arena = OpArena::new();
        let p0 = arena.alloc_ptt(SkOpPtT::new(0.25, Point::new(1.0, 1.0), Some(0)));
        let p1 = arena.alloc_ptt(SkOpPtT::new(0.75, Point::new(1.0, 1.0), Some(1)));
        arena.ptt_mut(p0).f_next = Some(p1.index());
        arena.ptt_mut(p1).f_next = Some(p0.index());

        let mut cur = p0;
        let mut seen = Vec::new();
        loop {
            seen.push(cur);
            cur = PtTId::new(arena.ptt(cur).f_next.expect("ring is closed"));
            if cur == p0 {
                break;
            }
        }
        assert_eq!(seen, vec![p0, p1]);
    }

    #[test]
    fn coincident_runs_link_into_a_list() {
        let mut arena = OpArena::new();
        let first = arena.alloc_coin(SkCoincidentSpans::default());
        let second = arena.alloc_coin(SkCoincidentSpans::default());
        arena.coin_mut(first).f_next = Some(second);
        arena.set_coincidence(Some(first));

        let mut walked = Vec::new();
        let mut cur = arena.coincidence();
        while let Some(id) = cur {
            walked.push(id);
            cur = arena.coin(id).f_next;
        }
        assert_eq!(walked, vec![first, second]);
        // Records carry sequential debug ids.
        assert_eq!(arena.coin(first).f_id, 0);
        assert_eq!(arena.coin(second).f_id, 1);
    }

    #[test]
    fn allocating_a_span_marks_the_flag() {
        let mut arena = OpArena::new();
        assert!(!arena.allocated_op_span());
        let _ = arena.alloc_span(SkOpSpanBase::new(0.0, Point::default(), None));
        assert!(arena.allocated_op_span());
        arena.reset_allocated_op_span();
        assert!(!arena.allocated_op_span());
    }

    #[test]
    fn nesting_bumps_and_clears() {
        let mut arena = OpArena::new();
        assert_eq!(arena.nested(), 0);
        arena.bump_nested();
        arena.bump_nested();
        assert_eq!(arena.nested(), 2);
        arena.clear_nested();
        assert_eq!(arena.nested(), 0);
    }

    #[test]
    fn phase_and_winding_failure_are_tracked() {
        let mut arena = OpArena::new();
        assert_eq!(arena.phase(), OpPhase::Intersecting);
        arena.set_phase(OpPhase::Winding);
        assert_eq!(arena.phase(), OpPhase::Winding);
        assert!(!arena.winding_failed());
        arena.set_winding_failed();
        assert!(arena.winding_failed());
    }

    #[test]
    fn debug_ids_are_handed_out_in_order() {
        let mut arena = OpArena::new();
        assert_eq!(arena.next_angle_id(), 0);
        assert_eq!(arena.next_angle_id(), 1);
        assert_eq!(arena.next_contour_id(), 0);
        // Angles allocated through the pool get their ids from the same run.
        let a = arena.alloc_angle(SkOpAngle::new());
        assert_eq!(arena.angle(a).f_id, 2);
    }

    #[test]
    fn deleted_nodes_keep_their_slot() {
        // Retiring a node must not move the ones after it, or every live
        // handle would dangle.
        let mut arena = OpArena::new();
        let a = arena.alloc_ptt(SkOpPtT::new(0.0, Point::default(), None));
        let b = arena.alloc_ptt(SkOpPtT::new(0.5, Point::default(), None));
        let c = arena.alloc_ptt(SkOpPtT::new(1.0, Point::default(), None));
        arena.ptt_mut(b).set_deleted(true);

        assert_eq!(arena.ptt_count(), 3);
        assert!(arena.ptt(b).f_deleted);
        assert!(!arena.ptt(b).active());
        // a and c are untouched and still where they were.
        assert!((arena.ptt(a).f_t - 0.0).abs() < 1e-9);
        assert!((arena.ptt(c).f_t - 1.0).abs() < 1e-9);
    }
}
