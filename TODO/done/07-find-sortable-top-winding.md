# 07 — SkPathOpsWinding: FindSortableTop and ray-cast winding

**Depends on 02, 03, 04, 05.** Blocks 08. One of the pieces originally
requested.

## Problem

`FindSortableTop` is not in `SkPathOpsCommon.cpp` where you would expect it —
it is in `SkPathOpsWinding.cpp:412`, along with the ray-casting winding
computation it relies on.

`src/pathops/sk_path_ops_winding.rs` is 215 lines against 426 lines of C++,
and the only methods on it are four accessors on a direction helper:

```rust
pub fn xy_index(&self) -> usize
pub fn perp_index(&self) -> usize
pub fn less_than(&self) -> bool
pub fn rotate(&self, offset: usize) -> Self
```

Absent: the entire `SkOpRayHit` / ray-cast apparatus, `SkOpSegment::
sortableTop`, `SkOpContour::findSortableTop`, `FindSortableTop`,
`SkOpSpan::computeWindSum`'s real body, and the horizontal/vertical ray
intersection helpers.

## What it does

`FindSortableTop` picks the span the contour walk should start from: an
undone span whose winding can be determined unambiguously, found by casting a
ray from a candidate point and counting what it crosses. It retries up to
`kMaxWindingTries` (10) times across all contours before giving up.

`bridgeWinding` (item 08) calls it once per output contour. Without it there
is no starting point and no walk.

## Task

1. `SkOpRayHit` struct + the ray-cast: `SkOpSegment::rayCheck`, the
   horizontal and vertical intersect helpers, hit sorting.
2. `SkOpSpan::sortableTop` — cast rays from the span, decide if the winding
   is unambiguous, set `fWindSum` when it is.
3. `SkOpSegment::findSortableTop`, `SkOpContour::findSortableTop` — the two
   traversal wrappers.
4. `FindSortableTop` — the `kMaxWindingTries` retry loop.

The direction-rotation helper already in the file (`xy_index`, `perp_index`,
`rotate`) is the C++ `SkOpRayDir` support and looks correct; build on it
rather than replacing it.

## Acceptance

- On a two-overlapping-rectangles contour list (built via the real
  `SkOpEdgeBuilder` + `AddIntersectTs`), `FindSortableTop` returns a span
  with a resolved `windSum`, not `PK_MIN_S32`.
- The retry loop terminates on input it cannot resolve, returning `None`
  rather than looping.
- `compute_wind_sum` in `sk_op_span.rs` no longer returns the stored field
  unchanged (this is the same defect from item 03; whichever lands second
  should find it already fixed).

---

## Closed (2026-09-14)

Landed in `src/pathops/sk_op_sortable_top.rs`: `SkOpRayHit`,
`SkOpSegment::rayCheck`, `SkOpSpan::sortableTop`, both `findSortableTop`
wrappers, `FindSortableTop`, plus `windingSpanAtT` and the `get_t_guess`
subdivision.

The direction helpers already in `sk_path_ops_winding.rs` were built on
rather than replaced, as the item asked.

### Three things it refuses rather than guesses at

Each would otherwise produce a plausible wrong count:

- A hit landing exactly on a span boundary belongs to two spans equally, so
  `winding_span_at_t` returns nothing for it.
- A crossing whose tangent runs nearly along the ray is marked invalid;
  which side it leaves on would be decided by rounding.
- Two crossings at the same point cannot be ordered, so the whole cast is
  abandoned and retried.

### On the acceptance criteria

- `FindSortableTop` returns a span with a resolved `windSum`, tested on a
  rectangle and on two rectangles sharing collinear edges.
- The retry loop terminates and returns `None` rather than looping, tested.
- `compute_wind_sum` takes the `sortable_top` closure and runs it under
  `MAX_WINDING_TRIES` — it no longer returns the stored field unchanged.

### A note for whoever picks up item 09

`a_ray_that_runs_along_an_edge_is_retried_in_another_direction` records the
case that matters: when two rectangles share a y range, every horizontal edge
lies along another one, the first ray resolves nothing, and only the retry
gets there. A **terminal span is never resolvable at all** — `sortableTop`
casts from inside the span's own interval and a terminal span has none.
Anything that asks a terminal span for its winding will wait out ten tries
and get `PK_MinS32`.
