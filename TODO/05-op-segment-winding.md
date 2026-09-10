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
