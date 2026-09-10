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
use super::sk_op_span::{is_zero_or_one, SkOpPtT, SkOpSpanBase, PK_MIN_S32};
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


    // --- SkOpPtT circular list -------------------------------------------
    //
    // A point shared by several segments links one SkOpPtT per segment into a
    // ring. Walking the ring is how the engine finds every segment through a
    // point, so these are the first graph operations item 03 needs.

    /// Returns the next node in `id`'s ring.
    ///
    /// A node not yet inserted into a ring points at itself, as after
    /// `SkOpPtT::init`; this returns `id` in that case rather than `None`.
    #[must_use]
    pub fn ptt_next(&self, id: PtTId) -> PtTId {
        match self.ptt(id).f_next {
            Some(n) => PtTId::new(n),
            None => id,
        }
    }

    /// Closes `id` into a ring of one, as `SkOpPtT::init` leaves it.
    pub fn ptt_init_ring(&mut self, id: PtTId) {
        self.ptt_mut(id).f_next = Some(id.index());
    }

    /// Inserts `node` directly after `id` in the ring.
    ///
    /// Port of `SkOpPtT::insert`.
    pub fn ptt_insert(&mut self, id: PtTId, node: PtTId) {
        debug_assert_ne!(id, node, "a node cannot be inserted after itself");
        let after = self.ptt(id).f_next;
        self.ptt_mut(node).f_next = after.or(Some(id.index()));
        self.ptt_mut(id).f_next = Some(node.index());
    }

    /// Returns every node in `id`'s ring, starting at `id`.
    ///
    /// Stops early rather than spinning if the links are corrupt.
    #[must_use]
    pub fn ptt_ring(&self, id: PtTId) -> Vec<PtTId> {
        let mut out = vec![id];
        let mut cur = self.ptt_next(id);
        while cur != id {
            out.push(cur);
            if out.len() > self.pt_ts.len() {
                break;
            }
            cur = self.ptt_next(cur);
        }
        out
    }

    /// Returns the node before `id` in its ring.
    ///
    /// Port of `SkOpPtT::prev`, which walks forward because the ring is singly
    /// linked.
    #[must_use]
    pub fn ptt_prev(&self, id: PtTId) -> PtTId {
        let mut result = id;
        let mut next = self.ptt_next(id);
        while next != id {
            result = next;
            next = self.ptt_next(next);
        }
        result
    }

    /// Returns true when `check` is somewhere in `id`'s ring, other than `id`.
    ///
    /// Port of `SkOpPtT::contains(const SkOpPtT*)`.
    #[must_use]
    pub fn ptt_contains(&self, id: PtTId, check: PtTId) -> bool {
        let mut cur = self.ptt_next(id);
        let mut guard = 0;
        while cur != id {
            if cur == check {
                return true;
            }
            guard += 1;
            if guard > self.pt_ts.len() {
                break;
            }
            cur = self.ptt_next(cur);
        }
        false
    }

    /// Returns the first node in `id`'s ring on `segment` that is not deleted.
    ///
    /// Port of `SkOpPtT::find`. Unlike [`Self::ptt_contains_segment`] the walk
    /// includes `id` itself.
    #[must_use]
    pub fn ptt_find(&self, id: PtTId, segment: SegmentId) -> Option<PtTId> {
        self.ptt_ring(id)
            .into_iter()
            .find(|&node| !self.ptt(node).f_deleted && self.ptt_segment(node) == Some(segment))
    }

    /// Returns a node on `check` elsewhere in `id`'s ring, if there is one.
    ///
    /// Port of `SkOpPtT::contains(const SkOpSegment*)`.
    #[must_use]
    pub fn ptt_contains_segment(&self, id: PtTId, check: SegmentId) -> Option<PtTId> {
        let mut cur = self.ptt_next(id);
        let mut guard = 0;
        while cur != id {
            if !self.ptt(cur).f_deleted && self.ptt_segment(cur) == Some(check) {
                return Some(cur);
            }
            guard += 1;
            if guard > self.pt_ts.len() {
                break;
            }
            cur = self.ptt_next(cur);
        }
        None
    }

    /// Returns true when `id`'s ring holds a node on `segment` at `t`.
    ///
    /// Port of `SkOpPtT::contains(const SkOpSegment*, double)`.
    #[must_use]
    pub fn ptt_contains_t(&self, id: PtTId, segment: SegmentId, t: f32) -> bool {
        let mut cur = self.ptt_next(id);
        let mut guard = 0;
        while cur != id {
            if self.ptt(cur).f_t == t && self.ptt_segment(cur) == Some(segment) {
                return true;
            }
            guard += 1;
            if guard > self.pt_ts.len() {
                break;
            }
            cur = self.ptt_next(cur);
        }
        false
    }

    /// Returns the span `id` belongs to.
    ///
    /// Port of `SkOpPtT::span`.
    #[must_use]
    pub fn ptt_span(&self, id: PtTId) -> Option<SpanId> {
        self.ptt(id).f_span.map(SpanId::new)
    }

    /// Returns the segment `id` sits on, by way of its span.
    ///
    /// Port of `SkOpPtT::segment`.
    #[must_use]
    pub fn ptt_segment(&self, id: PtTId) -> Option<SegmentId> {
        let span = self.ptt_span(id)?;
        self.span(span).f_segment.map(SegmentId::new)
    }

    /// Returns a live node for `id`, which may be `id` itself.
    ///
    /// Port of `SkOpPtT::active`. A deleted node yields another node on the
    /// same span that is not deleted, or `None` when there is none — which the
    /// caller must treat as a failure rather than carrying on.
    #[must_use]
    pub fn ptt_active(&self, id: PtTId) -> Option<PtTId> {
        if !self.ptt(id).f_deleted {
            return Some(id);
        }
        let span = self.ptt(id).f_span;
        let mut cur = self.ptt_next(id);
        let mut guard = 0;
        while cur != id {
            let n = self.ptt(cur);
            if n.f_span == span && !n.f_deleted {
                return Some(cur);
            }
            guard += 1;
            if guard > self.pt_ts.len() {
                break;
            }
            cur = self.ptt_next(cur);
        }
        None
    }

    /// Returns the node whose `f_next` points at `opp`, or `None` when `id` is
    /// already inside `opp`'s ring.
    ///
    /// Port of `SkOpPtT::oppPrev`. Returning `None` is how [`Self::ptt_add_opp`]
    /// learns that the two rings are already joined.
    #[must_use]
    pub fn ptt_opp_prev(&self, id: PtTId, opp: PtTId) -> Option<PtTId> {
        let mut opp_prev = self.ptt_next(opp);
        if opp_prev == id {
            return None;
        }
        while self.ptt_next(opp_prev) != opp {
            opp_prev = self.ptt_next(opp_prev);
            if opp_prev == id {
                return None;
            }
        }
        Some(opp_prev)
    }

    /// Splices `opp`'s ring into `id`'s.
    ///
    /// Port of `SkOpPtT::addOpp`. Returns false when the two are already one
    /// ring, which is not an error: the point is already shared.
    pub fn ptt_add_opp(&mut self, id: PtTId, opp: PtTId) -> bool {
        let Some(opp_prev) = self.ptt_opp_prev(id, opp) else {
            return false;
        };
        let old_next = self.ptt_next(id);
        self.ptt_mut(id).f_next = Some(opp.index());
        self.ptt_mut(opp_prev).f_next = Some(old_next.index());
        true
    }

    /// Returns true when `id` is not the node its own span points at.
    ///
    /// Port of `SkOpPtT::alias`.
    #[must_use]
    pub fn ptt_is_alias(&self, id: PtTId) -> bool {
        match self.ptt_span(id) {
            Some(span) => self.span(span).f_ptt != Some(id.index()),
            None => false,
        }
    }

    /// Returns true when `id` is the node at one end of its segment.
    ///
    /// Port of `SkOpPtT::onEnd`.
    #[must_use]
    pub fn ptt_on_end(&self, id: PtTId) -> bool {
        let Some(span) = self.ptt_span(id) else {
            return false;
        };
        if self.span(span).f_ptt != Some(id.index()) {
            return false;
        }
        let Some(seg) = self.ptt_segment(id) else {
            return false;
        };
        let s = self.segment(seg);
        s.f_head == Some(span) || s.f_tail == Some(span)
    }


    // --- SkOpSpanBase chain (item 03, part 2) -----------------------------
    //
    // Spans run head to tail along a segment in increasing t. The terminal
    // span sits at t == 1 and carries no winding, which is what C++ expresses
    // by making it an SkOpSpanBase rather than an SkOpSpan.

    /// Returns the next span along the segment.
    ///
    /// Port of `SkOpSpan::next`. `None` at the tail.
    #[must_use]
    pub fn span_next(&self, id: SpanId) -> Option<SpanId> {
        self.span(id).f_next.map(SpanId::new)
    }

    /// Returns the previous span along the segment.
    ///
    /// Port of `SkOpSpanBase::prev`. `None` at the head.
    #[must_use]
    pub fn span_prev(&self, id: SpanId) -> Option<SpanId> {
        self.span(id).f_prev.map(SpanId::new)
    }

    /// Links `next` after `id`.
    ///
    /// Port of `SkOpSpan::setNext`, with the matching back link so the chain
    /// stays walkable in both directions.
    pub fn span_set_next(&mut self, id: SpanId, next: Option<SpanId>) {
        self.span_mut(id).f_next = next.map(SpanId::index);
        if let Some(n) = next {
            self.span_mut(n).f_prev = Some(id.index());
        }
    }

    /// Links `prev` before `id`.
    ///
    /// Port of `SkOpSpanBase::setPrev`.
    pub fn span_set_prev(&mut self, id: SpanId, prev: Option<SpanId>) {
        self.span_mut(id).f_prev = prev.map(SpanId::index);
        if let Some(p) = prev {
            self.span_mut(p).f_next = Some(id.index());
        }
    }

    /// Returns true when `id` is the terminal span, at t == 1.
    ///
    /// Port of `SkOpSpanBase::final`.
    #[must_use]
    pub fn span_is_final(&self, id: SpanId) -> bool {
        self.span(id).f_t == 1.0
    }

    /// Returns `id` as a winding-carrying span, or `None` if it is terminal.
    ///
    /// Port of `SkOpSpanBase::upCastable`. C++ downcasts the base pointer;
    /// the arena keeps both roles in one pool, so the distinction is made
    /// here instead of by the type. `upCast` itself, which asserts rather than
    /// returning null, has no separate form: a caller that knows the span is
    /// not terminal can `expect` on this.
    #[must_use]
    pub fn span_upcastable(&self, id: SpanId) -> Option<SpanId> {
        if self.span_is_final(id) {
            None
        } else {
            Some(id)
        }
    }

    /// Returns the segment `id` belongs to.
    ///
    /// Port of `SkOpSpanBase::segment`.
    #[must_use]
    pub fn span_segment(&self, id: SpanId) -> Option<SegmentId> {
        self.span(id).f_segment.map(SegmentId::new)
    }

    /// Returns the span's own point-and-t node.
    ///
    /// Port of `SkOpSpanBase::ptT`.
    #[must_use]
    pub fn span_ptt(&self, id: SpanId) -> Option<PtTId> {
        self.span(id).f_ptt.map(PtTId::new)
    }

    /// Returns whichever of `id` and `end` has the lesser t, as a
    /// winding-carrying span.
    ///
    /// Port of `SkOpSpanBase::starter`. The two must be on the same segment.
    /// The lesser of the pair is never the terminal span unless both are, so
    /// the upcast succeeds in every case the engine asks about.
    #[must_use]
    pub fn span_starter(&self, id: SpanId, end: SpanId) -> Option<SpanId> {
        debug_assert_eq!(
            self.span_segment(id),
            self.span_segment(end),
            "starter compares spans of one segment"
        );
        let result = if self.span(id).f_t < self.span(end).f_t {
            id
        } else {
            end
        };
        self.span_upcastable(result)
    }

    /// Returns 1 when `end` lies after `id` along the segment, -1 otherwise.
    ///
    /// Port of `SkOpSpanBase::step`.
    #[must_use]
    pub fn span_step(&self, id: SpanId, end: SpanId) -> i32 {
        if self.span(id).f_t < self.span(end).f_t {
            1
        } else {
            -1
        }
    }

    /// Returns every span of `segment`, head to tail.
    ///
    /// Stops early rather than spinning if the links are corrupt.
    #[must_use]
    pub fn segment_spans(&self, segment: SegmentId) -> Vec<SpanId> {
        let mut out = Vec::new();
        let mut cur = self.segment(segment).f_head;
        while let Some(id) = cur {
            out.push(id);
            if out.len() > self.spans.len() {
                break;
            }
            cur = self.span_next(id);
        }
        out
    }

    /// Returns the node on `id`'s ring that sits on `segment`.
    ///
    /// Port of `SkOpSpanBase::contains(const SkOpSegment*)`.
    #[must_use]
    pub fn span_contains_segment(&self, id: SpanId, segment: SegmentId) -> Option<PtTId> {
        let start = self.span_ptt(id)?;
        self.ptt_ring(start)
            .into_iter()
            .find(|&node| !self.ptt(node).f_deleted && self.ptt_segment(node) == Some(segment))
    }


    // --- coincidence membership (item 03, part 3) -------------------------
    //
    // Two rings, both self-referential when empty: `f_coincident` links spans
    // that start a coincident run together, `f_coin_end` links the spans that
    // end one. C++ initialises each to point at its own span, so "not
    // coincident" is "points at itself"; `None` means the same here.

    /// Returns the next span in `id`'s coincident-start ring.
    #[must_use]
    pub fn span_coincident(&self, id: SpanId) -> SpanId {
        match self.span(id).f_coincident {
            Some(n) => SpanId::new(n),
            None => id,
        }
    }

    /// Returns the next span in `id`'s coincident-end ring.
    ///
    /// Port of `SkOpSpanBase::coinEnd`.
    #[must_use]
    pub fn span_coin_end(&self, id: SpanId) -> SpanId {
        match self.span(id).f_coin_end {
            Some(n) => SpanId::new(n),
            None => id,
        }
    }

    /// Returns true when `id` starts a coincident run.
    ///
    /// Port of `SkOpSpan::isCoincident`, which is "the ring is not just me".
    #[must_use]
    pub fn span_is_coincident(&self, id: SpanId) -> bool {
        self.span_coincident(id) != id
    }

    /// Returns true when `coin` is in `id`'s coincident-start ring.
    ///
    /// Port of `SkOpSpan::containsCoincidence(const SkOpSpan*)`.
    #[must_use]
    pub fn span_contains_coincidence(&self, id: SpanId, coin: SpanId) -> bool {
        let mut next = self.span_coincident(id);
        let mut guard = 0;
        while next != id {
            if next == coin {
                return true;
            }
            guard += 1;
            if guard > self.spans.len() {
                break;
            }
            next = self.span_coincident(next);
        }
        false
    }

    /// Returns true when `coin` is in `id`'s coincident-end ring.
    ///
    /// Port of `SkOpSpanBase::containsCoinEnd`.
    #[must_use]
    pub fn span_contains_coin_end(&self, id: SpanId, coin: SpanId) -> bool {
        let mut next = self.span_coin_end(id);
        let mut guard = 0;
        while next != id {
            if next == coin {
                return true;
            }
            guard += 1;
            if guard > self.spans.len() {
                break;
            }
            next = self.span_coin_end(next);
        }
        false
    }

    /// Splices `coin` into `id`'s coincident-start ring.
    ///
    /// Port of `SkOpSpan::insertCoincidence`. A span already in the ring is
    /// left alone, so this is idempotent.
    pub fn span_insert_coincidence(&mut self, id: SpanId, coin: SpanId) {
        debug_assert_ne!(id, coin);
        if self.span_contains_coincidence(id, coin) {
            return;
        }
        let coin_next = self.span_coincident(coin);
        let this_next = self.span_coincident(id);
        self.span_mut(coin).f_coincident = Some(this_next.index());
        self.span_mut(id).f_coincident = Some(coin_next.index());
    }

    /// Splices `coin` into `id`'s coincident-end ring.
    ///
    /// Port of `SkOpSpanBase::insertCoinEnd`.
    pub fn span_insert_coin_end(&mut self, id: SpanId, coin: SpanId) {
        debug_assert_ne!(id, coin);
        if self.span_contains_coin_end(id, coin) {
            return;
        }
        let coin_next = self.span_coin_end(coin);
        let this_next = self.span_coin_end(id);
        self.span_mut(coin).f_coin_end = Some(this_next.index());
        self.span_mut(id).f_coin_end = Some(coin_next.index());
    }

    /// Drops `id` out of its coincident-start ring.
    ///
    /// Port of `SkOpSpan::clearCoincident`. Returns false when there was
    /// nothing to clear.
    pub fn span_clear_coincident(&mut self, id: SpanId) -> bool {
        debug_assert!(!self.span_is_final(id));
        if !self.span_is_coincident(id) {
            return false;
        }
        self.span_mut(id).f_coincident = Some(id.index());
        true
    }

    /// Returns true when `id`'s coincident-start ring reaches `segment`.
    ///
    /// Port of `SkOpSpan::containsCoincidence(const SkOpSegment*)`.
    #[must_use]
    pub fn span_coincidence_reaches(&self, id: SpanId, segment: SegmentId) -> bool {
        let mut next = self.span_coincident(id);
        let mut guard = 0;
        loop {
            if self.span_segment(next) == Some(segment) {
                return true;
            }
            next = self.span_coincident(next);
            guard += 1;
            if next == self.span_coincident(id) || guard > self.spans.len() {
                break;
            }
        }
        false
    }

    // --- angle attachment (item 03, part 4) -------------------------------

    /// Returns the angle leaving `id`.
    ///
    /// Port of `SkOpSpan::toAngle`.
    #[must_use]
    pub fn span_to_angle(&self, id: SpanId) -> Option<AngleId> {
        self.span(id).f_to_angle.map(AngleId::new)
    }

    /// Returns the angle arriving at `id`.
    ///
    /// Port of `SkOpSpanBase::fromAngle`.
    #[must_use]
    pub fn span_from_angle(&self, id: SpanId) -> Option<AngleId> {
        self.span(id).f_from_angle.map(AngleId::new)
    }

    /// Attaches the angle leaving `id`.
    ///
    /// Port of `SkOpSpan::setToAngle`, which asserts the span is not terminal:
    /// nothing leaves the end of a segment.
    pub fn span_set_to_angle(&mut self, id: SpanId, angle: Option<AngleId>) {
        debug_assert!(
            !self.span_is_final(id),
            "the terminal span has no angle leaving it"
        );
        self.span_mut(id).f_to_angle = angle.map(AngleId::index);
    }

    /// Attaches the angle arriving at `id`.
    ///
    /// Port of `SkOpSpanBase::setFromAngle`.
    pub fn span_set_from_angle(&mut self, id: SpanId, angle: Option<AngleId>) {
        self.span_mut(id).f_from_angle = angle.map(AngleId::index);
    }

    // --- winding state (item 03, part 5) ----------------------------------

    /// Computes the winding sum for `id`, or returns the stored one.
    ///
    /// Port of `SkOpSpan::computeWindSum`, which retries `sortableTop` until
    /// it succeeds or [`MAX_WINDING_TRIES`] passes go by.
    ///
    /// `sortable_top` is passed in because `FindSortableTop` is item 07 and
    /// does not exist yet; when it lands it becomes the caller. Passing a
    /// closure that always fails reproduces the old behaviour of returning the
    /// stored field, but now that is the caller's choice rather than a silent
    /// stub.
    pub fn span_compute_wind_sum<F>(&mut self, id: SpanId, mut sortable_top: F) -> i32
    where
        F: FnMut(&mut OpArena, SpanId) -> bool,
    {
        let mut tries = 0;
        while !sortable_top(self, id) {
            tries += 1;
            if tries >= MAX_WINDING_TRIES {
                break;
            }
        }
        self.span(id).f_wind_sum
    }


    // --- merge and release (item 03, part 6) ------------------------------
    //
    // These mutate the graph while it is being walked, which is why C++ keeps
    // them apart from the plain accessors and why they land last.

    /// Returns true when `span`'s PtT node is in `id`'s ring.
    ///
    /// Port of `SkOpSpanBase::contains(const SkOpSpanBase*)`.
    #[must_use]
    pub fn span_contains_span(&self, id: SpanId, span: SpanId) -> bool {
        let (Some(start), Some(check)) = (self.span_ptt(id), self.span_ptt(span)) else {
            return false;
        };
        self.ptt_contains(start, check)
    }

    /// Sets the accumulated winding, flagging failure if it disagrees.
    ///
    /// Port of `SkOpSpan::setWindSum`. A second, different value means the
    /// walk reached the same span two ways and disagreed, which is not
    /// recoverable — C++ records it and returns rather than overwriting.
    pub fn span_set_wind_sum(&mut self, id: SpanId, wind_sum: i32) {
        debug_assert!(!self.span_is_final(id));
        let current = self.span(id).f_wind_sum;
        if current != PK_MIN_S32 && current != wind_sum {
            self.set_winding_failed();
            return;
        }
        self.span_mut(id).f_wind_sum = wind_sum;
    }

    /// Sets the accumulated opposite-operand winding, flagging disagreement.
    ///
    /// Port of `SkOpSpan::setOppSum`.
    pub fn span_set_opp_sum(&mut self, id: SpanId, opp_sum: i32) {
        debug_assert!(!self.span_is_final(id));
        let current = self.span(id).f_opp_sum;
        if current != PK_MIN_S32 && current != opp_sum {
            self.set_winding_failed();
            return;
        }
        self.span_mut(id).f_opp_sum = opp_sum;
    }

    /// Unlinks `id` from its segment, handing its PtT nodes to `kept`.
    ///
    /// Port of `SkOpSpan::release`. The span comes out of the chain, its own
    /// PtT node is marked deleted, and every node in the ring that still
    /// pointed at this span is repointed at `kept`'s span — otherwise those
    /// nodes would refer to a span no longer in any chain.
    ///
    /// The `SkOpCoincidence::fixUp` call C++ makes here is not ported; that is
    /// item 06. Coincident runs touching a released span are left stale.
    pub fn span_release(&mut self, id: SpanId, kept: PtTId) {
        debug_assert!(!self.span_is_final(id));
        let prev = self.span_prev(id);
        let next = self.span_next(id);
        // Close the chain over the gap.
        match (prev, next) {
            (Some(p), Some(n)) => {
                self.span_mut(p).f_next = Some(n.index());
                self.span_mut(n).f_prev = Some(p.index());
            }
            (Some(p), None) => self.span_mut(p).f_next = None,
            (None, Some(n)) => self.span_mut(n).f_prev = None,
            (None, None) => {}
        }
        // Keep the segment's endpoints honest.
        if let Some(seg) = self.span_segment(id) {
            if self.segment(seg).f_head == Some(id) {
                self.segment_mut(seg).f_head = next;
            }
            if self.segment(seg).f_tail == Some(id) {
                self.segment_mut(seg).f_tail = prev;
            }
            self.segment_mut(seg).f_count -= 1;
        }

        let Some(own) = self.span_ptt(id) else {
            return;
        };
        self.ptt_mut(own).set_deleted(true);
        // Anything still pointing at this span now points at kept's.
        let kept_span = self.ptt(kept).f_span;
        for node in self.ptt_ring(own) {
            if self.ptt(node).f_span == Some(id.index()) {
                self.ptt_mut(node).f_span = kept_span;
            }
        }
    }

    /// Folds `span`'s PtT ring into `id`'s, releasing `span`.
    ///
    /// Port of `SkOpSpanBase::merge`. The two spans share a t value or a
    /// point; this moves every node into one ring without trying to decide
    /// which t or point is the better one.
    ///
    /// Returns false when the two were already in one ring, which C++ treats
    /// as a case that should have been caught earlier.
    pub fn span_merge(&mut self, id: SpanId, span: SpanId) -> bool {
        let (Some(own), Some(span_ptt)) = (self.span_ptt(id), self.span_ptt(span)) else {
            return false;
        };
        debug_assert_ne!(self.span(id).f_t, self.ptt(span_ptt).f_t);
        self.span_release(span, own);
        if self.span_contains_span(id, span) {
            return false;
        }
        let remainder_start = self.ptt_next(span_ptt);
        self.ptt_insert(own, span_ptt);

        // Move the rest of span's old ring over, skipping any node that
        // duplicates one already there on the same span at the same t.
        let mut remainder = remainder_start;
        let mut guard = 0;
        while remainder != span_ptt {
            let next = self.ptt_next(remainder);
            let mut duplicate = false;
            let mut compare = self.ptt_next(span_ptt);
            let mut inner_guard = 0;
            while compare != span_ptt {
                let next_c = self.ptt_next(compare);
                if self.ptt(next_c).f_span == self.ptt(remainder).f_span
                    && self.ptt(next_c).f_t == self.ptt(remainder).f_t
                {
                    duplicate = true;
                    break;
                }
                compare = next_c;
                inner_guard += 1;
                if inner_guard > self.pt_ts.len() {
                    break;
                }
            }
            if !duplicate {
                self.ptt_insert(span_ptt, remainder);
            }
            remainder = next;
            guard += 1;
            if guard > self.pt_ts.len() {
                break;
            }
        }
        let adds = self.span(span).f_span_adds;
        self.span_mut(id).f_span_adds += adds;
        true
    }

    /// Releases spans of `opp` that duplicate spans of `id` on one segment.
    ///
    /// Port of `SkOpSpanBase::mergeMatches`. Where both rings hold a node on
    /// the same segment, the one at an interior t is released in favour of the
    /// one at an end; when both sit at an end the segment has collapsed to a
    /// point and both nodes are deleted.
    ///
    /// `mark_all_done` reports a collapsed segment, since marking it is
    /// `SkOpSegment`'s job and that is item 05.
    ///
    /// Returns false if the walk ran away, matching the C++ safety hatch.
    pub fn span_merge_matches<F>(
        &mut self,
        id: SpanId,
        opp: SpanId,
        mut mark_all_done: F,
    ) -> bool
    where
        F: FnMut(&mut OpArena, SegmentId),
    {
        let (Some(head), Some(opp_head)) = (self.span_ptt(id), self.span_ptt(opp)) else {
            return false;
        };
        let mut safety_hatch = 1_000_000;
        let mut test = head;
        loop {
            safety_hatch -= 1;
            if safety_hatch == 0 {
                return false;
            }
            let test_next = self.ptt_next(test);
            if !self.ptt(test).f_deleted {
                if let Some(segment) = self.ptt_segment(test) {
                    let test_base = self.ptt(test).f_span.map(SpanId::new);
                    for inner in self.ptt_ring(opp_head) {
                        if self.ptt_segment(inner) != Some(segment)
                            || self.ptt(inner).f_deleted
                        {
                            continue;
                        }
                        let Some(inner_base) = self.ptt(inner).f_span.map(SpanId::new) else {
                            continue;
                        };
                        let inner_t = self.ptt(inner).f_t;
                        let test_t = self.ptt(test).f_t;
                        if !is_zero_or_one(inner_t) {
                            self.span_release(inner_base, test);
                        } else if !is_zero_or_one(test_t) {
                            if let Some(tb) = test_base {
                                self.span_release(tb, inner);
                            }
                        } else {
                            // Both ends: the segment has collapsed.
                            mark_all_done(self, segment);
                            self.ptt_mut(test).set_deleted(true);
                            self.ptt_mut(inner).set_deleted(true);
                        }
                    }
                }
            }
            test = test_next;
            if test == head {
                break;
            }
        }
        true
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
    use crate::pathops::sk_op_span::PK_MIN_S32;

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

    // --- SkOpPtT ring (item 03, part 1) ----------------------------------

    /// Allocates a span on `seg` with its own PtT node, and returns both.
    fn span_with_ptt(arena: &mut OpArena, seg: SegmentId, t: f32) -> (SpanId, PtTId) {
        let span = arena.alloc_span(SkOpSpanBase::new(
            t,
            Point::new(t * 10.0, 0.0),
            Some(seg.index()),
        ));
        let ptt = arena.alloc_ptt(SkOpPtT::new(t, Point::new(t * 10.0, 0.0), Some(span.index())));
        arena.ptt_init_ring(ptt);
        arena.span_mut(span).f_ptt = Some(ptt.index());
        (span, ptt)
    }

    #[test]
    fn a_fresh_ptt_is_a_ring_of_one() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (_, p) = span_with_ptt(&mut arena, seg, 0.5);
        assert_eq!(arena.ptt_next(p), p);
        assert_eq!(arena.ptt_prev(p), p);
        assert_eq!(arena.ptt_ring(p), vec![p]);
    }

    #[test]
    fn a_ptt_ring_of_three_round_trips() {
        // The acceptance case for this part.
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (_, a) = span_with_ptt(&mut arena, seg, 0.0);
        let (_, b) = span_with_ptt(&mut arena, seg, 0.5);
        let (_, c) = span_with_ptt(&mut arena, seg, 1.0);
        arena.ptt_insert(a, b);
        arena.ptt_insert(b, c);

        assert_eq!(arena.ptt_ring(a), vec![a, b, c]);
        // Walking next three times returns to the start.
        let mut cur = a;
        for _ in 0..3 {
            cur = arena.ptt_next(cur);
        }
        assert_eq!(cur, a);
        // And prev is the node whose next is this one.
        assert_eq!(arena.ptt_prev(a), c);
        assert_eq!(arena.ptt_prev(b), a);
        assert_eq!(arena.ptt_prev(c), b);
    }

    #[test]
    fn ptt_contains_finds_ring_members_but_not_itself() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (_, a) = span_with_ptt(&mut arena, seg, 0.0);
        let (_, b) = span_with_ptt(&mut arena, seg, 0.5);
        let (_, outside) = span_with_ptt(&mut arena, seg, 1.0);
        arena.ptt_insert(a, b);

        assert!(arena.ptt_contains(a, b));
        assert!(arena.ptt_contains(b, a));
        assert!(!arena.ptt_contains(a, a), "the walk excludes the start");
        assert!(!arena.ptt_contains(a, outside));
    }

    #[test]
    fn a_ptt_reaches_its_span_and_segment() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (span, ptt) = span_with_ptt(&mut arena, seg, 0.25);
        assert_eq!(arena.ptt_span(ptt), Some(span));
        assert_eq!(arena.ptt_segment(ptt), Some(seg));
    }

    #[test]
    fn ptt_find_locates_a_node_on_a_given_segment() {
        let mut arena = OpArena::new();
        let seg_a = arena.alloc_segment(ArenaSegment::default());
        let seg_b = arena.alloc_segment(ArenaSegment::default());
        let (_, a) = span_with_ptt(&mut arena, seg_a, 0.4);
        let (_, b) = span_with_ptt(&mut arena, seg_b, 0.6);
        arena.ptt_insert(a, b);

        assert_eq!(arena.ptt_find(a, seg_a), Some(a));
        assert_eq!(arena.ptt_find(a, seg_b), Some(b));
        // contains_segment skips the start, so it will not report seg_a here.
        assert_eq!(arena.ptt_contains_segment(a, seg_b), Some(b));
        assert_eq!(arena.ptt_contains_segment(a, seg_a), None);
    }

    #[test]
    fn ptt_find_skips_deleted_nodes() {
        let mut arena = OpArena::new();
        let seg_a = arena.alloc_segment(ArenaSegment::default());
        let seg_b = arena.alloc_segment(ArenaSegment::default());
        let (_, a) = span_with_ptt(&mut arena, seg_a, 0.4);
        let (_, dead) = span_with_ptt(&mut arena, seg_b, 0.6);
        let (_, live) = span_with_ptt(&mut arena, seg_b, 0.7);
        arena.ptt_insert(a, dead);
        arena.ptt_insert(dead, live);
        arena.ptt_mut(dead).set_deleted(true);

        assert_eq!(arena.ptt_find(a, seg_b), Some(live));
        assert_eq!(arena.ptt_contains_segment(a, seg_b), Some(live));
    }

    #[test]
    fn ptt_contains_t_matches_segment_and_parameter() {
        let mut arena = OpArena::new();
        let seg_a = arena.alloc_segment(ArenaSegment::default());
        let seg_b = arena.alloc_segment(ArenaSegment::default());
        let (_, a) = span_with_ptt(&mut arena, seg_a, 0.4);
        let (_, b) = span_with_ptt(&mut arena, seg_b, 0.6);
        arena.ptt_insert(a, b);

        assert!(arena.ptt_contains_t(a, seg_b, 0.6));
        assert!(!arena.ptt_contains_t(a, seg_b, 0.5), "wrong t");
        assert!(!arena.ptt_contains_t(a, seg_a, 0.6), "wrong segment");
    }

    #[test]
    fn ptt_active_yields_a_live_node_on_the_same_span() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (span, dead) = span_with_ptt(&mut arena, seg, 0.5);
        // A second node on the same span, alive.
        let live = arena.alloc_ptt(SkOpPtT::new(0.5, Point::new(5.0, 0.0), Some(span.index())));
        arena.ptt_init_ring(live);
        arena.ptt_insert(dead, live);
        arena.ptt_mut(dead).set_deleted(true);

        assert_eq!(arena.ptt_active(live), Some(live), "a live node is its own");
        assert_eq!(arena.ptt_active(dead), Some(live));
    }

    #[test]
    fn ptt_active_gives_up_when_every_node_on_the_span_is_gone() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (_, only) = span_with_ptt(&mut arena, seg, 0.5);
        arena.ptt_mut(only).set_deleted(true);
        assert_eq!(arena.ptt_active(only), None);
    }

    #[test]
    fn add_opp_joins_two_rings() {
        // Two segments crossing at a point: each has its own ring, and
        // add_opp splices them so one walk reaches both.
        let mut arena = OpArena::new();
        let seg_a = arena.alloc_segment(ArenaSegment::default());
        let seg_b = arena.alloc_segment(ArenaSegment::default());
        let (_, a0) = span_with_ptt(&mut arena, seg_a, 0.3);
        let (_, a1) = span_with_ptt(&mut arena, seg_a, 0.4);
        arena.ptt_insert(a0, a1);
        let (_, b0) = span_with_ptt(&mut arena, seg_b, 0.7);
        let (_, b1) = span_with_ptt(&mut arena, seg_b, 0.8);
        arena.ptt_insert(b0, b1);

        assert_eq!(arena.ptt_ring(a0).len(), 2);
        assert_eq!(arena.ptt_ring(b0).len(), 2);

        assert!(arena.ptt_add_opp(a0, b0));
        let joined = arena.ptt_ring(a0);
        assert_eq!(joined.len(), 4, "all four nodes are now one ring: {joined:?}");
        for id in [a0, a1, b0, b1] {
            assert!(joined.contains(&id), "{id:?} missing from the joined ring");
        }
    }

    #[test]
    fn add_opp_reports_rings_that_are_already_joined() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (_, a) = span_with_ptt(&mut arena, seg, 0.0);
        let (_, b) = span_with_ptt(&mut arena, seg, 0.5);
        arena.ptt_insert(a, b);
        // Already one ring, so there is nothing to splice.
        assert!(!arena.ptt_add_opp(a, b));
        assert_eq!(arena.ptt_ring(a).len(), 2);
    }

    #[test]
    fn opp_prev_finds_the_node_pointing_at_the_target() {
        let mut arena = OpArena::new();
        let seg_a = arena.alloc_segment(ArenaSegment::default());
        let seg_b = arena.alloc_segment(ArenaSegment::default());
        let (_, a) = span_with_ptt(&mut arena, seg_a, 0.3);
        let (_, b0) = span_with_ptt(&mut arena, seg_b, 0.7);
        let (_, b1) = span_with_ptt(&mut arena, seg_b, 0.8);
        arena.ptt_insert(b0, b1);

        // b1 is the node whose next is b0.
        assert_eq!(arena.ptt_opp_prev(a, b0), Some(b1));
        // Once joined, a is inside b's ring and there is no opp prev.
        assert!(arena.ptt_add_opp(a, b0));
        assert_eq!(arena.ptt_opp_prev(a, b0), None);
    }

    #[test]
    fn a_span_knows_which_ptt_is_its_own() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (span, own) = span_with_ptt(&mut arena, seg, 0.5);
        // A second node on the same span is an alias, not the span's own.
        let alias = arena.alloc_ptt(SkOpPtT::new(0.5, Point::new(5.0, 0.0), Some(span.index())));
        arena.ptt_init_ring(alias);
        arena.ptt_insert(own, alias);

        assert!(!arena.ptt_is_alias(own));
        assert!(arena.ptt_is_alias(alias));
    }

    #[test]
    fn on_end_is_true_only_at_the_segment_ends() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (head, head_ptt) = span_with_ptt(&mut arena, seg, 0.0);
        let (mid, mid_ptt) = span_with_ptt(&mut arena, seg, 0.5);
        let (tail, tail_ptt) = span_with_ptt(&mut arena, seg, 1.0);
        arena.segment_mut(seg).f_head = Some(head);
        arena.segment_mut(seg).f_tail = Some(tail);
        let _ = mid;

        assert!(arena.ptt_on_end(head_ptt));
        assert!(arena.ptt_on_end(tail_ptt));
        assert!(!arena.ptt_on_end(mid_ptt));
    }

    // --- SkOpSpanBase chain (item 03, part 2) ----------------------------

    /// Builds a segment whose spans sit at the given t values, linked in order
    /// and each with its own PtT node.
    fn segment_at_ts(arena: &mut OpArena, ts: &[f32]) -> (SegmentId, Vec<SpanId>) {
        let seg = arena.alloc_segment(ArenaSegment::default());
        let mut ids = Vec::new();
        for &t in ts {
            let (span, _) = span_with_ptt(arena, seg, t);
            ids.push(span);
        }
        for i in 0..ids.len().saturating_sub(1) {
            arena.span_set_next(ids[i], Some(ids[i + 1]));
        }
        arena.segment_mut(seg).f_head = ids.first().copied();
        arena.segment_mut(seg).f_tail = ids.last().copied();
        arena.segment_mut(seg).f_count = ids.len() as i32;
        (seg, ids)
    }

    #[test]
    fn set_next_links_both_directions() {
        let mut arena = OpArena::new();
        let (seg, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        assert_eq!(arena.segment_spans(seg), spans);
        for i in 0..spans.len() {
            if i + 1 < spans.len() {
                assert_eq!(arena.span_next(spans[i]), Some(spans[i + 1]));
                assert_eq!(arena.span_prev(spans[i + 1]), Some(spans[i]));
            }
        }
        assert_eq!(arena.span_prev(spans[0]), None, "head has no previous");
        assert_eq!(arena.span_next(spans[2]), None, "tail has no next");
    }

    #[test]
    fn set_prev_links_both_directions_too() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (a, _) = span_with_ptt(&mut arena, seg, 0.0);
        let (b, _) = span_with_ptt(&mut arena, seg, 1.0);
        arena.span_set_prev(b, Some(a));
        assert_eq!(arena.span_next(a), Some(b));
        assert_eq!(arena.span_prev(b), Some(a));
    }

    #[test]
    fn only_the_span_at_one_is_final() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        assert!(!arena.span_is_final(spans[0]));
        assert!(!arena.span_is_final(spans[1]));
        assert!(arena.span_is_final(spans[2]));
    }

    #[test]
    fn upcastable_refuses_the_terminal_span() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        assert_eq!(arena.span_upcastable(spans[0]), Some(spans[0]));
        assert_eq!(arena.span_upcastable(spans[1]), Some(spans[1]));
        assert_eq!(
            arena.span_upcastable(spans[2]),
            None,
            "the terminal span carries no winding"
        );
    }

    #[test]
    fn starter_returns_the_lesser_t_either_way_round() {
        // The acceptance case: both orderings must give the same answer.
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.25, 0.75, 1.0]);
        let (lo, hi) = (spans[1], spans[2]);
        assert_eq!(arena.span_starter(lo, hi), Some(lo));
        assert_eq!(arena.span_starter(hi, lo), Some(lo));

        // Against the head, the head wins from either side.
        assert_eq!(arena.span_starter(spans[0], hi), Some(spans[0]));
        assert_eq!(arena.span_starter(hi, spans[0]), Some(spans[0]));
    }

    #[test]
    fn step_reports_which_way_the_walk_runs() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        assert_eq!(arena.span_step(spans[0], spans[2]), 1, "forwards");
        assert_eq!(arena.span_step(spans[2], spans[0]), -1, "backwards");
    }

    #[test]
    fn a_span_reaches_its_ptt_and_segment() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment(ArenaSegment::default());
        let (span, ptt) = span_with_ptt(&mut arena, seg, 0.5);
        assert_eq!(arena.span_ptt(span), Some(ptt));
        assert_eq!(arena.span_segment(span), Some(seg));
    }

    #[test]
    fn span_contains_segment_walks_the_ptt_ring() {
        // Two segments crossing: from a span on one, the ring reaches the
        // other, which is how the engine hops between segments at a crossing.
        let mut arena = OpArena::new();
        let (seg_a, spans_a) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (seg_b, spans_b) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let a_mid = arena.span_ptt(spans_a[1]).expect("ptt");
        let b_mid = arena.span_ptt(spans_b[1]).expect("ptt");
        assert!(arena.ptt_add_opp(a_mid, b_mid));

        assert_eq!(arena.span_contains_segment(spans_a[1], seg_b), Some(b_mid));
        assert_eq!(arena.span_contains_segment(spans_b[1], seg_a), Some(a_mid));
        // A span not part of the crossing reaches nothing on the other segment.
        assert_eq!(arena.span_contains_segment(spans_a[0], seg_b), None);
    }

    #[test]
    fn two_segments_are_walked_independently() {
        let mut arena = OpArena::new();
        let (seg_a, spans_a) = segment_at_ts(&mut arena, &[0.0, 0.3, 1.0]);
        let (seg_b, spans_b) = segment_at_ts(&mut arena, &[0.0, 0.6, 0.9, 1.0]);
        assert_eq!(arena.segment_spans(seg_a), spans_a);
        assert_eq!(arena.segment_spans(seg_b), spans_b);
        for id in &spans_a {
            assert_eq!(arena.span_segment(*id), Some(seg_a));
        }
        for id in &spans_b {
            assert_eq!(arena.span_segment(*id), Some(seg_b));
        }
    }

    // --- coincidence membership (item 03, part 3) ------------------------

    #[test]
    fn a_lone_span_is_not_coincident() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        assert!(!arena.span_is_coincident(spans[0]));
        assert_eq!(arena.span_coincident(spans[0]), spans[0]);
        assert_eq!(arena.span_coin_end(spans[0]), spans[0]);
    }

    #[test]
    fn inserting_coincidence_joins_two_spans() {
        let mut arena = OpArena::new();
        let (_, a) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (_, b) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        arena.span_insert_coincidence(a[0], b[0]);

        assert!(arena.span_is_coincident(a[0]));
        assert!(arena.span_is_coincident(b[0]));
        assert!(arena.span_contains_coincidence(a[0], b[0]));
        assert!(arena.span_contains_coincidence(b[0], a[0]));
        // A span outside the ring is not reported.
        assert!(!arena.span_contains_coincidence(a[0], a[1]));
    }

    #[test]
    fn inserting_the_same_coincidence_twice_changes_nothing() {
        let mut arena = OpArena::new();
        let (_, a) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (_, b) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        arena.span_insert_coincidence(a[0], b[0]);
        let after_first = arena.span_coincident(a[0]);
        arena.span_insert_coincidence(a[0], b[0]);
        assert_eq!(arena.span_coincident(a[0]), after_first);
    }

    #[test]
    fn clearing_coincidence_takes_a_span_back_out() {
        let mut arena = OpArena::new();
        let (_, a) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (_, b) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        arena.span_insert_coincidence(a[0], b[0]);
        assert!(arena.span_clear_coincident(a[0]));
        assert!(!arena.span_is_coincident(a[0]));
        // Clearing again reports there was nothing to do.
        assert!(!arena.span_clear_coincident(a[0]));
    }

    #[test]
    fn coin_end_is_a_separate_ring_from_coincident() {
        let mut arena = OpArena::new();
        let (_, a) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (_, b) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        arena.span_insert_coin_end(a[2], b[2]);

        assert!(arena.span_contains_coin_end(a[2], b[2]));
        assert!(arena.span_contains_coin_end(b[2], a[2]));
        // The start ring is untouched by an end-ring insertion.
        assert!(!arena.span_is_coincident(a[2]));
        assert!(!arena.span_contains_coincidence(a[2], b[2]));
    }

    #[test]
    fn coincidence_reaches_the_other_segment() {
        let mut arena = OpArena::new();
        let (seg_a, a) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (seg_b, b) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        arena.span_insert_coincidence(a[0], b[0]);
        assert!(arena.span_coincidence_reaches(a[0], seg_b));
        assert!(arena.span_coincidence_reaches(b[0], seg_a));
    }

    #[test]
    fn is_canceled_when_neither_operand_contributes() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        assert!(arena.span(spans[0]).is_canceled());
        arena.span_mut(spans[0]).set_wind_value(1);
        assert!(!arena.span(spans[0]).is_canceled());
    }

    // --- angle attachment (item 03, part 4) ------------------------------

    #[test]
    fn angles_attach_to_and_from_a_span() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let to = arena.alloc_angle(SkOpAngle::new());
        let from = arena.alloc_angle(SkOpAngle::new());

        assert_eq!(arena.span_to_angle(spans[0]), None);
        assert_eq!(arena.span_from_angle(spans[0]), None);
        arena.span_set_to_angle(spans[0], Some(to));
        arena.span_set_from_angle(spans[0], Some(from));
        assert_eq!(arena.span_to_angle(spans[0]), Some(to));
        assert_eq!(arena.span_from_angle(spans[0]), Some(from));
    }

    #[test]
    fn the_terminal_span_still_takes_an_arriving_angle() {
        // Nothing leaves the end of a segment, but something arrives at it.
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 1.0]);
        let from = arena.alloc_angle(SkOpAngle::new());
        arena.span_set_from_angle(spans[1], Some(from));
        assert_eq!(arena.span_from_angle(spans[1]), Some(from));
    }

    // --- winding state (item 03, part 5) ---------------------------------

    #[test]
    fn wind_and_opp_values_are_set_and_read() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let s = spans[0];
        assert_eq!(arena.span(s).wind_value(), 0);
        arena.span_mut(s).set_wind_value(2);
        arena.span_mut(s).set_opp_value(3);
        assert_eq!(arena.span(s).wind_value(), 2);
        assert_eq!(arena.span(s).opp_value(), 3);
        // Sums start unset.
        assert_eq!(arena.span(s).wind_sum(), PK_MIN_S32);
        arena.span_mut(s).set_wind_sum(5);
        assert_eq!(arena.span(s).wind_sum(), 5);
    }

    #[test]
    fn compute_wind_sum_runs_the_search_until_it_succeeds() {
        // The acceptance case: it computes rather than returning the field.
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let s = spans[0];
        assert_eq!(arena.span(s).wind_sum(), PK_MIN_S32);

        // A search that fails twice, then writes a sum and succeeds.
        let mut calls = 0;
        let sum = arena.span_compute_wind_sum(s, |arena, id| {
            calls += 1;
            if calls < 3 {
                return false;
            }
            arena.span_mut(id).set_wind_sum(7);
            true
        });
        assert_eq!(calls, 3, "it retried until the search succeeded");
        assert_eq!(sum, 7);
        assert_eq!(arena.span(s).wind_sum(), 7);
    }

    #[test]
    fn compute_wind_sum_gives_up_after_max_tries() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let mut calls = 0;
        let sum = arena.span_compute_wind_sum(spans[0], |_, _| {
            calls += 1;
            false
        });
        assert_eq!(calls, MAX_WINDING_TRIES, "bounded, not an infinite loop");
        assert_eq!(sum, PK_MIN_S32, "nothing was computed");
    }

    #[test]
    fn marking_added_and_bumping_adds() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let s = spans[0];
        assert!(!arena.span(s).already_added());
        arena.span_mut(s).mark_added();
        assert!(arena.span(s).already_added());

        assert_eq!(arena.span(s).f_span_adds, 0);
        arena.span_mut(s).bump_span_adds();
        arena.span_mut(s).bump_span_adds();
        assert_eq!(arena.span(s).f_span_adds, 2);
    }

    #[test]
    fn chased_is_recorded() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        assert!(!arena.span(spans[0]).chased());
        arena.span_mut(spans[0]).set_chased(true);
        assert!(arena.span(spans[0]).chased());
    }

    // --- merge and release (item 03, part 6) -----------------------------

    #[test]
    fn releasing_a_span_closes_the_chain_over_it() {
        let mut arena = OpArena::new();
        let (seg, spans) = segment_at_ts(&mut arena, &[0.0, 0.25, 0.5, 1.0]);
        let keep = arena.span_ptt(spans[0]).expect("ptt");
        arena.span_release(spans[1], keep);

        // The chain skips the released span.
        assert_eq!(arena.segment_spans(seg), vec![spans[0], spans[2], spans[3]]);
        assert_eq!(arena.span_next(spans[0]), Some(spans[2]));
        assert_eq!(arena.span_prev(spans[2]), Some(spans[0]));
        assert_eq!(arena.segment(seg).f_count, 3);
    }

    #[test]
    fn releasing_the_head_moves_the_head() {
        let mut arena = OpArena::new();
        let (seg, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let keep = arena.span_ptt(spans[1]).expect("ptt");
        arena.span_release(spans[0], keep);
        assert_eq!(arena.segment(seg).f_head, Some(spans[1]));
        assert_eq!(arena.span_prev(spans[1]), None);
        assert_eq!(arena.segment_spans(seg), vec![spans[1], spans[2]]);
    }

    #[test]
    fn releasing_marks_the_ptt_deleted_and_repoints_the_ring() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let doomed = spans[1];
        let own = arena.span_ptt(doomed).expect("ptt");
        // A second node in the ring still pointing at the doomed span.
        let alias = arena.alloc_ptt(SkOpPtT::new(0.5, Point::new(5.0, 0.0), Some(doomed.index())));
        arena.ptt_init_ring(alias);
        arena.ptt_insert(own, alias);

        let keep = arena.span_ptt(spans[0]).expect("ptt");
        arena.span_release(doomed, keep);

        assert!(arena.ptt(own).f_deleted, "the span's own node is retired");
        // The alias now points at the kept span rather than a span that is no
        // longer in any chain.
        assert_eq!(arena.ptt(alias).f_span, Some(spans[0].index()));
    }

    #[test]
    fn set_wind_sum_flags_a_disagreement() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let s = spans[0];
        arena.span_set_wind_sum(s, 3);
        assert_eq!(arena.span(s).wind_sum(), 3);
        assert!(!arena.winding_failed());

        // Setting the same value again is fine.
        arena.span_set_wind_sum(s, 3);
        assert!(!arena.winding_failed());

        // A different value means the walk reached here two ways and
        // disagreed; the original is kept and failure is recorded.
        arena.span_set_wind_sum(s, 5);
        assert!(arena.winding_failed());
        assert_eq!(arena.span(s).wind_sum(), 3, "the first value stands");
    }

    #[test]
    fn set_opp_sum_flags_a_disagreement_too() {
        let mut arena = OpArena::new();
        let (_, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let s = spans[0];
        arena.span_set_opp_sum(s, 1);
        assert!(!arena.winding_failed());
        arena.span_set_opp_sum(s, 2);
        assert!(arena.winding_failed());
        assert_eq!(arena.span(s).opp_sum(), 1);
    }

    #[test]
    fn span_contains_span_looks_through_the_ptt_ring() {
        let mut arena = OpArena::new();
        let (_, a) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (_, b) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        assert!(!arena.span_contains_span(a[1], b[1]));

        let pa = arena.span_ptt(a[1]).expect("ptt");
        let pb = arena.span_ptt(b[1]).expect("ptt");
        assert!(arena.ptt_add_opp(pa, pb));
        assert!(arena.span_contains_span(a[1], b[1]));
        assert!(arena.span_contains_span(b[1], a[1]));
    }

    #[test]
    fn merging_two_spans_gathers_their_ptt_nodes() {
        // Two segments crossing near the same place. Merging folds the second
        // span's ring into the first and takes the second out of its chain.
        let mut arena = OpArena::new();
        let (seg_a, a) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (seg_b, b) = segment_at_ts(&mut arena, &[0.0, 0.25, 1.0]);
        let _ = seg_a;

        let before = arena.ptt_ring(arena.span_ptt(a[1]).expect("ptt")).len();
        assert!(arena.span_merge(a[1], b[1]));

        let after = arena.ptt_ring(arena.span_ptt(a[1]).expect("ptt")).len();
        assert!(after > before, "the merged ring grew: {before} -> {after}");
        // The merged-away span is out of its own chain.
        assert_eq!(arena.segment_spans(seg_b), vec![b[0], b[2]]);
    }

    #[test]
    fn merging_carries_the_span_add_count_across() {
        let mut arena = OpArena::new();
        let (_, a) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (_, b) = segment_at_ts(&mut arena, &[0.0, 0.25, 1.0]);
        arena.span_mut(a[1]).bump_span_adds();
        arena.span_mut(b[1]).bump_span_adds();
        arena.span_mut(b[1]).bump_span_adds();

        assert!(arena.span_merge(a[1], b[1]));
        assert_eq!(arena.span(a[1]).f_span_adds, 3);
    }

    #[test]
    fn merge_matches_releases_the_interior_duplicate() {
        // Both rings hold a node on the same segment; the one at an interior
        // t gives way to the one at an end.
        let mut arena = OpArena::new();
        let (seg, spans) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);
        let (_, other) = segment_at_ts(&mut arena, &[0.0, 0.5, 1.0]);

        // Put an end node and an interior node of `seg` into two rings.
        let end_ptt = arena.span_ptt(spans[0]).expect("ptt");
        let mid_ptt = arena.span_ptt(spans[1]).expect("ptt");
        let opp_ptt = arena.span_ptt(other[1]).expect("ptt");
        assert!(arena.ptt_add_opp(opp_ptt, mid_ptt));

        let mut collapsed = Vec::new();
        assert!(arena.span_merge_matches(spans[0], other[1], |_, s| collapsed.push(s)));
        let _ = end_ptt;

        // The interior span was released, so the chain skips it.
        assert_eq!(arena.segment_spans(seg), vec![spans[0], spans[2]]);
        assert!(collapsed.is_empty(), "nothing collapsed here");
    }

    #[test]
    fn merge_matches_reports_a_collapsed_segment() {
        // Both nodes sit at ends of the same segment: it has no length left.
        let mut arena = OpArena::new();
        let (seg, spans) = segment_at_ts(&mut arena, &[0.0, 1.0]);
        let (_, other) = segment_at_ts(&mut arena, &[0.0, 1.0]);

        let head_ptt = arena.span_ptt(spans[0]).expect("ptt");
        let opp_ptt = arena.span_ptt(other[0]).expect("ptt");
        // Point the opposite node at this segment's tail, also an end.
        let tail_ptt = arena.span_ptt(spans[1]).expect("ptt");
        assert!(arena.ptt_add_opp(opp_ptt, tail_ptt));

        let mut collapsed = Vec::new();
        assert!(arena.span_merge_matches(spans[0], other[0], |_, s| collapsed.push(s)));
        let _ = head_ptt;
        assert_eq!(collapsed, vec![seg], "the segment collapsed to a point");
    }
}
