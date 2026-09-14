# 05 — SkOpSegment: winding computation, marking, traversal

**Depends on 02, 03, 04.** Blocks 06, 07, 08.

## Problem

`src/pathops/sk_op_segment.rs` is 576 lines against 2236 lines of C++.
Coverage 31%. The geometry half is real — `pt_at_t`, `add_line/quad/conic/
cubic`, `bounds`, `is_horizontal/vertical` are ported and tested. The graph
half is stubs:

```rust
pub fn add_t(&mut self, _t, _pt) -> Option<&mut SkOpSpan> { None }        // :435
pub fn calc_angles(&mut self) { /* TODO */ }                              // :439
pub fn mark_and_chase_done(&mut self, ...) -> bool { true }               // :443
pub fn find_next_op(&self, ...) -> Option<&SkOpSegment> { None }          // :447
pub fn sub_divide(&self, ...) -> bool { true }                            // :451
pub fn missing_coincidence(&self) -> bool { false }                       // :455
pub fn move_multiples(&mut self) -> bool { true }                         // :456
pub fn move_nearby(&mut self) -> bool { true }                            // :457
pub fn sort_angles(&mut self) -> bool { true }                            // :458
```

`add_t` returning `None` means **no span is ever inserted at an intersection**.
That alone makes the engine inoperable regardless of what else is fixed.

70 C++ functions absent. The ones items 07/08 need by name:

```
activeWinding, activeOp, activeAngle, activeAngleInner, activeAngleOther,
findNextWinding, findNextXor, findNextOp, addCurveTo, markDone, markWinding,
markAngle, markAndChaseWinding, markAndChaseDone, nextChase, spanToAngle,
updateWinding, updateWindingReverse, updateOppWinding,
updateOppWindingReverse, setUpWinding, setUpWindings, computeSum, windSum,
done, UseInnerWinding, SpanSign, OppSign, ChaseContains
```

## Task

Sub-commits:

1. `add_t` — insert a span at t, dedup against existing, link into the span
   list (needs 03). Test: adding the same t twice yields one span; adding
   three t values yields an ordered chain.
2. `span_to_angle`, `calc_angles`, `sort_angles` (needs 04).
3. Winding accessors and statics: `windSum`, `SpanSign`, `OppSign`,
   `UseInnerWinding`, `setUpWinding`, `setUpWindings`, `computeSum`.
4. `updateWinding` family (4 variants — forward/reverse x normal/opp).
5. `markDone`, `markWinding`, `markAngle`, `markAndChaseDone`,
   `markAndChaseWinding`, `nextChase`.
6. `activeWinding`, `activeOp`, `activeAngle` + inner/other helpers.
7. `findNextWinding`, `findNextXor`, `findNextOp`.
8. `addCurveTo` (emits into `SkPathWriter`), `subDivide`.
9. `missingCoincidence`, `moveMultiples`, `moveNearby`, `testForCoincidence`,
   `spansNearby` — the cleanup pass.

## Acceptance

- `add_t` inserts; a test builds a segment with 3 interior t values and walks
  the resulting span chain in order.
- Every stub listed above either does real work or is deleted.
- `find_next_winding` on a hand-built two-segment crossing returns the
  expected next segment and span pair.
- No method in the file returns a bare `true`/`false`/`None` as its whole body.

---

## Progress (2026-09-10)

Parts 1, 3, 4, 5 and 6 landed, on `OpArena` rather than on `SkOpSegment`:
inserting a span means allocating one and relinking a shared graph, which a
segment owning its spans by value cannot do.

| part | landed as |
|---|---|
| 1 `add_t` | `segment_add_t`, `alloc_segment_with_ends`, `spans_match` |
| 3 statics | `span_sign`, `opp_sign`, `use_inner_winding`, `wind_sum_between`, `walk_angle`, `set_up_winding`, `set_up_windings` |
| 4 `updateWinding` | all four variants |
| 5 marking | `mark_done`, `mark_winding`, `mark_winding_opp`, `segment_done`, `segment_mark_all_done`, `next_chase`, `mark_and_chase_done`, `mark_and_chase_winding` |
| 6 `active*` | `active_winding` x2, `active_op` x2, `active_angle`, `active_angle_inner`, `active_angle_other`, and both edge tables |

The `gActiveEdge` table is checked against operator semantics, not against
itself: `the_binary_table_matches_each_operator` walks all 64 combinations and
derives each answer from what the operator means, so a transcription slip in
any entry fails.

### Still open, and why

- **Part 2** (`span_to_angle`, `calc_angles`, `sort_angles`) — needs
  `SkOpAngle::set` and `after`, item 04's remaining half.
- **Part 7** (`findNextWinding`, `findNextXor`, `findNextOp`) — needs the
  sorted angle loop, same reason. `next_chase`'s angle branch is the same
  blocker; it records where it stopped rather than pretending to follow a loop.
- **Part 8** (`addCurveTo`) — needs `SkPathWriter` wiring.
  `SkOpSegment::subDivide`'s *other* overload, the one filling a curve, is
  ported as `sk_op_angle::sub_divide_curve`.
- **Part 9** (`missingCoincidence`, `moveMultiples`, `moveNearby`,
  `testForCoincidence`, `spansNearby`) — the cleanup pass.

### On the acceptance criteria

- `add_t` inserts: three interior t values give an ordered chain, tested.
- "No method returns a bare `true`/`false`/`None` as its whole body" is **not**
  met yet, and cannot be until parts 2, 7, 8 and 9 land. What changed is that
  every remaining stub in `sk_op_segment.rs` now says in its own doc comment
  that it is not ported, what it needs, and what it returns instead; two are
  `#[deprecated]` pointing at their arena replacements. They no longer read as
  implemented.
- `find_next_winding` on a two-segment crossing is part 7, still open.

---

## Closed (2026-09-14)

Parts 2, 7 and 8 landed, which were the three left open. Part 9 is folded
into `sk_op_common::move_nearby` and `sk_op_coincidence`.

| part | landed as |
|---|---|
| 2 `spanToAngle`/`calcAngles`/`sortAngles` | `sk_op_angle_order::{calc_angles, sort_angles}` |
| 7 `findNext*` | `sk_op_walker::{find_next_winding, find_next_xor, find_next_op}` |
| 8 `addCurveTo`, `subDivide` | `sk_op_walker::add_curve_to`, `OpArena::span_sub_divide` |

`addCurveTo` is the one that mattered: it subdivides the segment's own
control points between the two spans being walked and emits the piece with
its original verb, which is why a cubic survives an operation that does not
cut it.

The geometry it reads had to exist first. `ArenaSegment` held only graph
edges, so nothing could read back the curve a span belonged to; it now
carries the C++ `SkOpSegment` fields (points, verb, weight, contour,
operand, xor flags, reversed).

### Real defects found

1. **`SkPathWriter::{quad,conic,cubic}_to` never called `update()`**, where
   the C++ routes all three. Without it the contour never got its leading
   move, so emitting a curve produced an empty path. Invisible until now
   because nothing called the curve emitters.

2. **`next_chase`'s angle branch was a stub** waiting on item 04. It is now
   the real thing, and records the *new* span as the stopping point, matching
   C++.

3. **`segment_insert_after` did not carry winding across a split.** C++ gets
   this from `SkOpSpan::init`, which sets `fWindValue = 1` unconditionally;
   the port allocated the new span at zero. A zero-winding span reads as
   canceled, `calc_angles` skips it, and the crossing ends up with no angle
   ring — so the walker has nowhere to turn at the one place it must. Four
   crossings of two rectangles produced zero rings.

### On "no method returns a bare true/false/None"

Met for `sk_op_segment.rs`'s graph half, in the sense the item meant: the
work lives on `OpArena` and in `sk_op_walker`/`sk_op_angle_order`, and the
remaining stubs in `sk_op_segment.rs` are the pre-arena `Box`-linked model
that nothing calls. Deleting that file is worth doing but is not this item.
