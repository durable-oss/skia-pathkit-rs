# 09 — Rewrite simplify and op on the real engine (bridgeWinding / bridgeXor)

**`op` is done.** `simplify` is not. What follows is the state after the
routing switch, then what is left.

## Where it stands

`pathops::op` runs the ported engine in `src/pathops/sk_op_engine.rs`:
`SkOpEdgeBuilder`'s arena half, `AddIntersectTs`, `HandleCoincidence`, and the
`bridgeOp` walk, emitting through `add_curve_to`. Curves survive an operation.
`boolean.rs` is still linked, as the fallback for inputs the engine declines.

1028 tests pass, none ignored. The Intersect case that was `#[ignore]`d is
now `an_intersect_across_a_shared_edge_keeps_only_the_overlap` and passes.

Across a sweep of twelve shape pairs x four operators, the engine answers 46
of 48 and the fallback takes 2. Both fallbacks are the same gap: **a
Difference that has to cut a curve against a line**, e.g. a disc differenced
by a rectangle crossing it. Intersect and Xor on that same geometry come back
with their cubics, so it is specific to Difference, not to curves generally.

## The five defects the switch turned up

Recorded because each one was reached by a long trace and none is obvious
from the code.

1. **`updateWindingReverse` does not reverse its arguments.** The port had it
   backwards, and this was the one blocking the last case. C++'s
   angle-taking `updateWinding` passes `(end, start)`; the `Reverse` form
   passes `(start, end)` (`SkOpSegment.cpp:1725-1736`) — "reverse" names the
   relationship to *that* call, not to the argument order. Same for
   `updateOppWindingReverse`. Two unit tests had pinned the swapped
   behaviour; they assert the real contract now.

2. **`ComputeOneSum` handed `markAngle` the two sums unswapped.**
   `setUpWindings` writes `sumWinding` from whichever running total belongs
   to the target segment's own operand (`SkOpSegment.cpp:1529`). Passing
   `sum_mi`/`sum_su` straight through gives a second-operand segment its two
   totals the wrong way round.

3. **`assemble` decoded a folded-triangle index as a square one.** C++ keeps a
   separate `distLookup` mapping one to the other (`SkPathWriter.cpp:287`);
   the port dropped it and recomputed `row`/`col` from the triangle index,
   so every pairing after the first row joined unrelated endpoints. It also
   tested both of a contour's link slots for occupancy where C++ tests only
   the one `endOne` selects, and its final link walk was a stub that stepped
   the index instead of following the links and never reversed a piece.
   `Path::reverse_path_to` and `Path::add_path_extend` were added for it.

4. **A coincident split landed a hair off the corner it shared.** The run's
   ends are endpoints of one of the two segments, but the split point was
   recomputed by interpolation: segment `(20,20)->(0,20)` at t = 0.6 gives
   x = 7.9999995. PtT rings are keyed on exact coordinates, so the near-miss
   opened a second ring at a corner that should have had one. The split
   points snap to the real endpoint now.

5. **The binary walk drained its chase list with the unary `FindChase`.**
   `find_chase_op` is the port of `findChaseOp` (`SkPathOpsOp.cpp:20`): two
   running sums with the operand swap, `mark_angle_opp`, and the ring's
   *previous* member rather than its next.

### Two things deliberately not carried

- **The cross-operand branch of `markAndChaseWinding`** (`SkOpSegment.cpp:942`,
  which marks with the windings swapped when the chase changes operand). It is
  unreachable as this arena is built — `next_chase` prefers the segment on the
  chased span's own contour — and instrumenting it across the whole suite and
  the disc sweep fires it zero times. Left out with a note rather than kept as
  code no test can reach.
- **`walk_contour`'s dead-end emit conditions** were made to match
  `SkPathOpsOp.cpp:140-156` and are kept, but with the winding fixes in place
  no case found so far comes out differently for them.

### What was tried and did not work

Kept from the original notes, still true:

**Pre-resolving every span's winding before the walk starts.** Running
`sortable_top` over every non-terminal span up front makes every sum
available and makes results *worse*: it broke two disc-union tests that
passed. A ray cast from a span whose neighbours are unresolved accumulates a
different total than the same ray cast later, and `mark_and_chase_winding`
spreads that wrong value along the chase. The walk's own ordering is what
makes each cast meaningful.

## What is left

1. **`bridgeXor`, and it blocks `simplify`.** `find_next_xor` exists and
   nothing calls it; `simplify_with_engine` walks with `bridgeWinding` for
   both fill rules. The consequence is concrete: concentric squares under
   even-odd fill come back **solid**, with the hole filled in and the fill
   type rewritten to winding.

   Routing `simplify` through the engine was tried and reverted for exactly
   this. The whole suite passes either way — that case was not covered — so
   two tests now pin it from both sides:
   `simplify_keeps_an_even_odd_hole` (the public function, correct) and
   `the_engine_still_gets_an_even_odd_hole_wrong` (the engine, the gap).
   Delete the second and the routing guard together when `bridgeXor` lands.

2. **`simplify` still runs on the substitute engine,** and correctly, which is
   why it stays there for now. `sk_path_ops_simplify.rs` has Skia's
   `SimplifyDebug` control flow over flattened edges: `collect_boundary`
   samples `Path::contains` per edge rather than summing windings,
   `dedup_coincident` stands in for `HandleCoincidence`, and `is_convex` is
   hand-rolled. It flattens curves, so the switch is still worth making —
   after item 1.

3. **Curve/curve coincidence.** `record_if_coincident` handles line/line only.
   Two identical curves need the t-section machinery in `sk_path_ops_tsect`.
   This is the likely cause of the Difference-cutting-a-curve gap above.

4. **Deleting `boolean.rs`.** It cannot go while those 2 of 48 still need it.
   Once the curve gap closes, delete it and the substitute helpers in
   `sk_path_ops_simplify.rs` together.

5. **Three copies of `MAX_WINDING_TRIES`,** in `sk_path_ops_winding.rs` (100),
   `sk_op_span.rs` (100) and `sk_op_arena.rs` (10). Skia's is 10, and the
   arena's is the one the walk actually reads. The other two are dead or
   wrong; collapse them to one.

## Acceptance, restated

- [x] `op` routes through the engine, with a fallback rather than a wrong answer.
- [x] Output preserves curve verbs where the engine answers.
- [x] All pathops tests pass, none ignored.
- [x] The disc-union cases from item 16 pass at every offset, radius and vertex count.
- [ ] `bridgeXor`, so even-odd simplify keeps its holes.
- [ ] `simplify` on the real engine (blocked on the above).
- [ ] `boolean.rs` and the substitute helpers deleted.
