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
