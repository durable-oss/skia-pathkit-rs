# 02 — SkOpGlobalState and the arena

**Blocks 03, 04, 05, 06, 07, 08.** Nothing else in the chain can start until
this exists. Design item as much as coding item — get it reviewed before
building on it.

## Problem

`SkOpSpan`, `SkOpSpanBase`, `SkOpPtT`, `SkOpSegment`, `SkOpAngle` and
`SkOpCoincidence` form one cyclic, mutable object graph. In C++ every node is
arena-allocated and holds raw pointers in both directions:

```cpp
SkOpSpanBase* next = span->next();
SkOpSegment*  seg  = span->segment();
SkOpPtT*      ptT  = span->ptT()->next();
```

The Rust structs already declare these edges as `Option<usize>` with
`// arena index` comments (`sk_op_span.rs:16,79`, `sk_op_coincidence.rs:22`) —
but **no arena exists and nothing ever assigns them**. Verified: no `f_next =`,
no `f_prev =`, no `f_to_angle =` anywhere in `src/pathops/`. The fields are
decoration.

`compute_wind_sum` (`sk_op_span.rs:154`) returns `self.f_wind_sum` unchanged
instead of computing it, because it has no way to reach neighbouring spans.

## Task

Build the arena the existing field types already assume. `Option<usize>` index
handles into `Vec` pools is the right call for a cyclic graph in Rust —
`Rc<RefCell<>>` would fight the borrow checker at every traversal and leak on
the cycles.

```rust
pub struct OpGlobalState {
    spans:    Vec<SkOpSpanBase>,
    pt_ts:    Vec<SkOpPtT>,
    segments: Vec<SkOpSegment>,
    angles:   Vec<SkOpAngle>,
    coins:    Vec<SkCoincidentSpans>,
    contour_head: Option<SegmentId>,
    coincidence:  Option<CoinId>,
    nested: i32,
    allocated_op_span: bool,
    winding_failed: bool,
    phase: OpPhase,
    // debug id counters: next_angle_id, next_span_id, next_segment_id, ...
}
```

Decisions to make and write down:

- **Newtype the handles.** `SpanId(u32)`, `PtTId(u32)`, `AngleId(u32)`,
  `SegmentId(u32)`. A bare `usize` for all five invites indexing the wrong
  pool, and the compiler will not catch it.
- **Accessor shape.** Traversal is `state.span(id).next()`, so every method
  that walks the graph takes `&OpGlobalState` (or `&mut`). This is the
  invasive part: it changes the signature of nearly every method in items
  03-07. Settle it now.
- **Deletion.** C++ never frees (arena drops wholesale at the end) but does
  mark `fDeleted`. Keep the `f_deleted` flag; do not compact the pools, or
  every live handle is invalidated.
- `SkOpGlobalState` in `sk_path_ops_types.rs` currently has `phase`,
  `nesting`, `winding_failed` as trivial accessors — fold that struct into
  this one rather than having two.

Not needed: the `DEBUG_COIN` / `DEBUG_T_SECT_LOOP_COUNT` dictionaries. Skip
them, note the omission.

## Acceptance

- The graph can be built and walked in a test: allocate two segments, link
  their spans, traverse `next`/`prev` in both directions and get back where
  you started.
- `sk_op_coincidence.rs:29`'s `global_state: Some(0), // placeholder arena
  index` is gone.
- Handles are newtypes; passing a `SpanId` where a `PtTId` is expected does
  not compile.
- No behaviour change yet — nothing consumes the arena until item 03.

---

## Resolution (2026-09-10)

Done, in `src/pathops/sk_op_arena.rs`.

### Decisions, as asked

**Handles are newtypes.** `SpanId`, `PtTId`, `SegmentId`, `AngleId`, `CoinId`,
each wrapping a `u32`. Verified against the compiler, not just by reading:
`arena.segment(span_id)` fails with

```
error[E0308]: mismatched types
    |     let _ = a.segment(s);
    |               ------- ^ expected `SegmentId`, found `SpanId`
```

**Accessor shape is `arena.span(id)`**, with `&OpArena` / `&mut OpArena` passed
down. This is the invasive decision the item flagged, and it is now settled:
every graph-walking method in items 03-07 takes the arena as a parameter. The
alternative — methods on the node reaching for neighbours — cannot work, since
a node has no way back to the pool.

**Nothing is ever removed from a pool.** `SkOpPtT::f_deleted` retires a node in
place; `deleted_nodes_keep_their_slot` pins that down.

**One state object.** The arena carries `nested`, `allocated_op_span`,
`winding_failed`, `phase`, the two graph roots and the debug id counters, so
`sk_path_ops_types::OpGlobalState` is redundant. It is not deleted yet only
because `sk_op_span::compute_wind_sum` still takes one; that goes with item 03.

**Skipped, as agreed:** the `DEBUG_COIN` / `DEBUG_T_SECT_LOOP_COUNT`
dictionaries.

### Two things the item did not anticipate

- **`SkOpSpanBase` had no `f_next`.** C++ puts `fNext` on the derived
  `SkOpSpan`, since a terminal span has nothing after it. The arena pools both
  roles in one `Vec`, so the edge has to live on the base and is `None` for the
  tail. Added, with a comment saying why it differs from C++.
- **`ArenaSegment` holds only the graph edges**, not the geometry.
  `SkOpSegment` in `sk_op_segment.rs` keeps its points, verb and bounds. This
  keeps item 02 from having to rewrite the geometry code before there is a
  graph to hold it; item 05 joins the two.

### Acceptance

- `a_span_chain_walks_both_ways_and_returns`: two segments allocated, spans
  linked, walked head to tail and back, arriving at the head it started from.
- `sk_op_coincidence.rs`'s `global_state: Some(0), // placeholder arena index`
  is gone. `SkOpCoincidence` now keeps a real list head and allocates records
  through `arena.alloc_coin`, with `add_run` / `records` / `count` tested. Its
  detection passes (`add_missing`, `expand`, `mark_collapsed`, `fix_up`) are
  still stubs and now say so; that is item 06.
- Handles are newtypes and the compiler enforces it.
- No behaviour change: 758 tests pass, unchanged in outcome from before.
