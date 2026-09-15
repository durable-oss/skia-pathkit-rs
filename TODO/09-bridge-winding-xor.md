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

### Update (2026-09-15): `bridgeXor` lands; `simplify` now routes through the engine

`bridge_xor` (port of `bridgeXor`, `SkPathOpsSimplify.cpp`) is written in
`sk_op_engine.rs`, and `find_next_xor` — which existed but was never
called — was actually a wrong port of `findNextXor`'s ring walk (it
reused the winding/op `pick_next` helper, which does chase-list
bookkeeping and an active-edge filter that C++'s `findNextXor` does not
have). Rewritten as a standalone loop matching the C++ body directly.

`simplify_with_engine` also had a second, unrelated bug on the way to
this: it never set the result path's fill type, so it defaulted to
`Winding` and every even-odd result read back solid regardless of what
the walk produced. Fixed to always set even-odd (or inverse-even-odd),
matching `SimplifyDebug`'s `result->setFillType(fillType)`.

`simplify()` now routes through `sk_op_engine::simplify_with_engine`
first, falling back to the substitute engine the same way `op` does. The
two pinning tests (`simplify_keeps_an_even_odd_hole`, and the former
`the_engine_still_gets_an_even_odd_hole_wrong`) are collapsed into one
correct pin on each side (`simplify_keeps_an_even_odd_hole` and
`the_engine_keeps_an_even_odd_hole_too`).

**However:** verified against a cached real-Skia build (`skia-pathops`
Python package) as an oracle, `bridge_xor`'s *topology* is correct — same
contours, same point cycle — but curved inputs whose boundary gets cut at
two or more points on the same original arc (e.g. two overlapping
circles) come back with corrupted curve geometry from a separate,
pre-existing bug in curve subdivision. It is not new and not specific to
xor — `op_with_engine`'s `Union` shows the identical corruption on the
same geometry, untouched by this change. Filed as
`2026-09-15-curve-subdivision-corrupts-multi-intersection-arcs.md`; it is
a known, accepted gap for the `simplify` routing decision above, not a
blocker on it, since straight-edge and single-intersection-per-arc inputs
(most of them) work correctly either way.

1. ~~**`bridgeXor`, and it blocks `simplify`.**~~ Closed above.

2. ~~**`simplify` still runs on the substitute engine.**~~ Closed above.

3. **Curve/curve coincidence** — `record_if_coincident` still handles
   line/line only, and two identical curves would still need the t-section
   machinery in `sk_path_ops_tsect` to detect. Downgraded from "likely cause
   of the Difference-cutting-a-curve gap" — that gap did not reproduce (see
   the 2026-09-15 update above) — to "no known failing case, but no coverage
   either." Worth a dedicated repro sweep before claiming it is fine, since
   absence of a failure in an unrelated sweep is not the same as testing it.

4. ~~**Nested shapes that share part of their boundary, under Difference**~~
   Closed 2026-09-15: `ArenaSegment` gained `f_coincident_splits`, a count of
   how many of a segment's spans came from `record_if_coincident` or
   coincidence expansion (`SkOpCoincidence::addExpanded`'s port) rather than
   a real crossing, via a new `segment_add_t_coincident` that tags the split
   at the two call sites that split for coincidence. `graph_operands_do_not_cross`
   now treats `f_count - f_coincident_splits == 2` as "does not cross,"
   so a segment with only coincident splits still lets `nesting` settle the
   pair, while a segment carrying even one real-crossing split still
   correctly declines. Regression tests: a rect nested in another sharing
   one edge, two edges, or only a corner (all settle now), plus a case
   sharing an edge *and* poking through with a real crossing (still
   declines, confirming the tagging does not overcorrect).

5. **Deleting `boolean.rs`.** Was blocked on item 4 alone; that closed, but
   the broad sweep item 5 itself asked for (a few hundred shape pairs across
   all four operators, checked against interior-point containment, not
   against `boolean.rs`'s own output — see below) found **two more
   pre-existing gaps**, unrelated to nesting and confirmed to predate this
   session's item-4 change:
   - Two overlapping, differently-sized circles whose crossing
     `find_crossings` never finds at all, which then makes `nesting`'s
     single-sample shortcut misclassify a real partial overlap as full
     containment.
   - A disc/rect pair whose crossings *are* found and split correctly, but
     the walk still produces a wrong answer downstream of a correctly-built
     graph — a winding/chase bug in the family of the five defects this file
     already found, not a graph-construction bug.

   Filed as `2026-09-15-broad-sweep-found-two-more-op-with-engine-gaps.md`.
   `boolean.rs` stays until both close; deleting it now would leave these two
   gaps with no fallback. This is exactly the scenario the broad-sweep
   instruction above was written to catch, and it did: the "2 of 48,
   curve/curve coincidence" framing from earlier in this file undercounted,
   and a bigger sweep found real gaps a narrower one had missed. Re-run the
   sweep again once the new file's two gaps close, rather than assuming
   they are the last ones either.

   Update 2026-09-15: both of those closed (see
   `TODO/done/2026-09-15-broad-sweep-found-two-more-op-with-engine-gaps.md`),
   and widening the sweep as its own due diligence step surfaced a third,
   unrelated gap — filed as
   `2026-09-15-union-drops-the-far-side-of-a-cubic-and-conic-pair.md`. That
   one traced to a genuine engine bug (exact tangential contact between two
   curves misleads the angle-ring sort in `find_next_op`/`pick_next` into
   picking the wrong boundary edge — a real wrong answer, not a decline),
   confirmed narrow (a generic, non-tangent offset of the same shapes
   answers correctly) but **not fixed** — the fix needs curvature-based
   angle disambiguation on the scale of Skia's own `SkOpAngle.cpp`, out of
   scope for a bounded bug fix. Combined with item 3 above (curve/curve
   coincidence: no known failure, but also no coverage), `boolean.rs` stays
   blocked on two fronts: one confirmed wrong-answer gap and one unaudited
   one. It is not being deleted.

6. ~~**Three copies of `MAX_WINDING_TRIES`,**~~ Closed 2026-09-15: the two
   dead copies (`sk_path_ops_winding.rs`, `sk_op_span.rs`, both 100 and
   unread anywhere) are deleted along with their pinning tests. The
   arena's copy (10, matching Skia, and the one the walk actually reads)
   is untouched.

## Acceptance, restated

- [x] `op` routes through the engine, with a fallback rather than a wrong answer.
- [x] Output preserves curve verbs where the engine answers.
- [x] All pathops tests pass, none ignored.
- [x] The disc-union cases from item 16 pass at every offset, radius and vertex count.
- [x] The swallowed-shape empty-Difference gap closes when the operands'
      boundaries provably do not cross (`nesting`/`interior_point`).
- [x] `bridgeXor`, so even-odd simplify keeps its holes.
- [x] `simplify` on the real engine (falls back to the substitute engine
      the same way `op` does; curved multi-intersection inputs have a
      known, separately-filed correctness gap, not a decline).
- [x] The nested-shared-boundary Difference gap (item 4 above).
- [ ] `boolean.rs` and the substitute helpers deleted. Both gaps in
      `2026-09-15-broad-sweep-found-two-more-op-with-engine-gaps.md` are
      now fixed, but widening that file's sweep as its own due diligence
      found a third, unrelated gap first — a real wrong-answer bug at
      exact tangential contact between two curves, traced to the
      angle-ring sort, not fixed (needs curvature-based disambiguation on
      the scale of Skia's own `SkOpAngle.cpp`) — filed as
      `2026-09-15-union-drops-the-far-side-of-a-cubic-and-conic-pair.md`.
      Combined with item 3's unaudited curve/curve-coincidence gap, two
      fronts are still open; `boolean.rs` is not being deleted.
      The curve-subdivision gap that used to block this is fixed — see
      `TODO/done/2026-09-15-curve-subdivision-corrupts-multi-intersection-arcs.md`.
