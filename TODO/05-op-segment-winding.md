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
