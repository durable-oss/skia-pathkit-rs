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

### Update (2026-09-15): that gap is closed; a narrower one remains

The curve/line Difference gap above does not reproduce against a clean
repro (disc minus a rectangle crossing it at a generic angle, avoiding the
circle's own quadrant points) — `op_with_engine` answers it directly, cubics
intact. A dense sweep of 512 shape pairs (disc/disc across 3 radii x 3 radii
x 8 offsets, disc/rect across 5 offsets x 4 rotations, and 9-gon-through-
20-gon pairs across 4 phases, all four operators) declines **zero**. Whatever
produced the original 2-of-48 was either fixed by an intervening commit or
was this session's own degenerate test geometry (arc endpoints landing
exactly on the other shape's corners) rather than a real coincidence gap;
either way it is not reproducible now.

A different, narrower gap was found instead: **`empty_is_the_answer` could
not tell a Difference apart when one shape's bounding box contains the
other's**, since box containment does not imply shape containment. Two
things came out of chasing it:

- When the two operands' boundaries provably never cross (`build` split no
  segment — `graph_operands_do_not_cross`), a single interior sample point on
  each settles the question outright, for every operator, not just
  Intersect. This closes the swallowed-shape case: a disc entirely inside a
  bigger disc, a rect entirely inside a bigger rect with no shared boundary,
  disjoint shapes, touching-but-zero-area boxes. See `nesting` and
  `interior_point` in `sk_op_engine.rs`; sampling had to move off the
  boundary itself; sampling `one`'s first move-to point directly votes
  either way arbitrarily whenever it sits exactly on `two`'s boundary, which
  is exactly the touching case this was meant to cover.
- **What is still open:** nested shapes whose boundaries touch — share an
  edge, a corner, or more — go through `record_if_coincident` first, which
  splits the touching segments (`f_count` goes above 2) before the walk
  looks at nesting. `graph_operands_do_not_cross` correctly reads that as
  "cannot assume nesting from zero splits" and declines, even though the
  contact is a coincident run, not an interior crossing, and nesting still
  holds outside it. A sweep of one rect nested in another, varying how many
  edges/corners they share, still declines on Difference in every case that
  shares at least one edge. Distinguishing "split from a coincident run" from
  "split from a real crossing" would need `segment_add_t` calls tagged by
  cause, which does not exist today and is a bigger change than this gap
  is worth reaching for on its own. `boolean.rs` still answers this case
  correctly (verified end to end via `pathops::op`), so it is a fallback
  case, not a wrong answer.

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

3. **Curve/curve coincidence** — `record_if_coincident` still handles
   line/line only, and two identical curves would still need the t-section
   machinery in `sk_path_ops_tsect` to detect. Downgraded from "likely cause
   of the Difference-cutting-a-curve gap" — that gap did not reproduce (see
   the 2026-09-15 update above) — to "no known failing case, but no coverage
   either." Worth a dedicated repro sweep before claiming it is fine, since
   absence of a failure in an unrelated sweep is not the same as testing it.

4. **Nested shapes that share part of their boundary, under Difference** —
   the gap the 2026-09-15 update above found. `boolean.rs` still answers it
   correctly; closing it in the engine needs `segment_add_t` splits tagged by
   whether they came from a real crossing or from `record_if_coincident`.

5. **Deleting `boolean.rs`.** Blocked on item 4, now the only known case
   where the engine still declines an input the fallback answers correctly.
   Once it closes, delete `boolean.rs` and the substitute helpers in
   `sk_path_ops_simplify.rs` together — but re-run a broad sweep first
   (a few hundred varied shape pairs across all four operators) rather than
   trusting the last known gap was the only one; that is how the stale "2 of
   48, curve/curve coincidence" framing above happened in the first place.

6. **Three copies of `MAX_WINDING_TRIES`,** in `sk_path_ops_winding.rs` (100),
   `sk_op_span.rs` (100) and `sk_op_arena.rs` (10). Skia's is 10, and the
   arena's is the one the walk actually reads. The other two are dead or
   wrong; collapse them to one.

## Acceptance, restated

- [x] `op` routes through the engine, with a fallback rather than a wrong answer.
- [x] Output preserves curve verbs where the engine answers.
- [x] All pathops tests pass, none ignored.
- [x] The disc-union cases from item 16 pass at every offset, radius and vertex count.
- [x] The swallowed-shape empty-Difference gap closes when the operands'
      boundaries provably do not cross (`nesting`/`interior_point`).
- [ ] `bridgeXor`, so even-odd simplify keeps its holes.
- [ ] `simplify` on the real engine (blocked on the above).
- [ ] The nested-shared-boundary Difference gap (item 4 above).
- [ ] `boolean.rs` and the substitute helpers deleted (blocked on the above).
