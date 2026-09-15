# Broad sweep found two more op_with_engine gaps

Found while doing the due-diligence sweep item 5 of `TODO/09-bridge-winding-xor.md`
asked for before deleting `boolean.rs`: a few hundred shape pairs across all
four operators, checked by interior-point containment against each operand's
own `contains` (not against `boolean.rs`'s own output — see below). Both
gaps below are confirmed to predate this session's item-4 fix (reproduced
against the commit before it, `7359944`), so neither is a regression from
that work.

`boolean.rs` is kept for now because of these two; item 4 itself (nested
shapes sharing a boundary) is closed, and its own regression tests pass.

## Gap 1: two overlapping circles whose crossing is never found

Two circles, radius 30 at the origin and radius 50 centred at `(40, 0)`.
`d = 40`, `ra + rb = 80`, `|ra - rb| = 20` — a textbook partial overlap,
neither swallowing the other.

`op_with_engine` gets every operator wrong on this pair:

- `Union` comes back as **circle B alone** (points span exactly B's bounding
  box, `x` from -10 to 90).
- `Intersect` comes back as **circle A alone**.
- `Difference` comes back **empty**.
- `Xor` comes back as **both circles, unmodified** (two separate contours).

These are exactly the answers for "A is entirely inside B," which is false
here — A's leftmost point (`x = -30`) sits outside B (B's leftmost point is
`x = -10`).

Root cause, traced directly: `build()` never finds the two circles'
intersection at all. Every segment on both circles comes back with
`f_count == 2` (no interior spans added), confirmed with a probe on
`graph_operands_do_not_cross` and the per-segment counts. Since nothing
crossed, `graph_operands_do_not_cross` correctly reports `true` given what
it sees, and `nesting()` then samples one interior point of A, finds it
inside B (true for that one sample, since the sample lands in the lens of
overlap), and concludes "A entirely inside B" — which a single sample cannot
tell apart from "A partially overlaps B" without knowing the boundaries
never cross in the first place.

So this is not a nesting-classification bug (item 4's kind) — nesting's
logic is sound *given* `graph_operands_do_not_cross`. The actual defect is
upstream: `find_crossings`/`intersect_pair` (`sk_op_engine.rs`) fails to
find a real intersection between two circles built from four conics each,
at least for this radius/offset combination. Two identical-radius circles
at the same combination of offsets did not reproduce this in earlier
sweeps (see `TODO/09-bridge-winding-xor.md`'s 2026-09-15 update, "512 shape
pairs... declines zero"), so it looks specific to differently-sized circles,
or to this particular offset, not to disc/disc crossings generally.

### Task

1. Reproduce minimally (radius 30 at origin, radius 50 at offset 40 is a
   known repro) and trace `find_crossings` for these two conic arcs — likely
   the four-conic circle representation and where each circle's crossing
   conic segment actually is, versus where the search samples.
2. Fix `find_crossings` to find the real intersection, not `nesting` or
   `graph_operands_do_not_cross` — the bug is that no split ever happens at
   all, not that a split existed and was misclassified.
3. Regression test: this exact disc pair, all four operators, checked by
   interior-point containment.
4. Re-run the broad sweep in `sk_op_engine.rs`
   (`a_broad_sweep_of_shape_pairs_never_declines_and_matches_the_fallback`)
   and lower its mismatch tolerance once this gap's mismatches are gone.

## Gap 2: a disc/rect pair with a correctly-built graph still walks wrong

Circle radius 30 at the origin, rect from `(0, -40)` to `(20, 40)`.

Unlike gap 1, the graph here is built correctly: `graph_operands_do_not_cross`
reports `false`, and segment counts show real splits (`f_count` of 3 and 4 on
some segments, from real crossings — confirmed by probing `build()`
directly). So this is a walk/winding bug downstream of correct input, not a
missing-crossing bug.

`op_with_engine` mismatches `Union`, `Intersect`, `Difference` and `Xor` at a
probe point that is inside the circle and outside the rect (e.g.
`(-16.9, 2.7)` relative to this pair, though the exact probe coordinates are
an artifact of the sweep's grid, not special to the geometry).

### Task

1. Reproduce minimally (circle radius 30 at origin, rect `(0,-40)-(20,40)`
   is a known repro) and trace the walk (`bridge`/`find_next_op` or the
   winding sums at the crossing spans near the mismatching probe point) —
   this is a `mark_and_chase_winding`/`ComputeOneSum`-class bug, in the
   family of the five defects `TODO/09-bridge-winding-xor.md` already found
   and fixed, not a graph-construction bug.
2. Fix the winding/walk logic, not the containment check.
3. Regression test: this exact disc/rect pair, all four operators, checked
   by interior-point containment.
4. Re-run the broad sweep and lower its mismatch tolerance once this gap's
   mismatches are gone too.

## Why the sweep checks against `contains`, not `boolean.rs`

The obvious oracle — compare `op_with_engine`'s answer to `boolean::path_op`'s
— turned out to be unusable directly: `boolean.rs` has its own containment
bug, found while chasing gap 1. A union of two plain circles (radius 30 at
origin, radius 50 at offset 40, no pathops involved beyond the union itself)
answers `contains()` as `false` at a point verified to be inside circle A
alone (`a.contains(...)` is `true` on the untouched circle). That is a
`boolean.rs`/flattening bug, not an engine bug, and it is out of scope here —
filed only as this note, not a separate TODO, since it was not chased beyond
confirming it exists and it does not block anything (the engine still
correctly answers `op_with_engine` when its own two gaps above are not in
play). If someone picks this file up and needs a reliable oracle beyond raw
`contains` on the operands, do not reach for `boolean.rs` without re-checking
it first.

## Acceptance

- [x] Gap 1 (missed circle/circle crossing) fixed, with a regression test.
      Root cause: `find_crossings` discarded any touch landing exactly on
      either curve's own endpoint outright, on the assumption that this
      only ever meant "adjacent sides of one contour meeting at their
      shared corner." Two circles whose real crossing happens to land on
      one circle's own quadrant point hit the same code path with a
      genuine crossing, and were silently dropped. Fixed by
      `contour_crosses_at_endpoint` in `sk_op_engine.rs`, which decides per
      touch whether the contour actually passes through to the far side.
      See `overlapping_circles_whose_crossing_lands_on_a_quadrant_point`.
- [x] Gap 2 (wrong walk on a correctly-built disc/rect graph) fixed, with a
      regression test. Turned out to share gap 1's root cause rather than
      being a separate winding bug — fixing only `find_crossings` brought
      the broad sweep's mismatch count to zero, so no separate walk fix
      was needed. See `disc_and_rect_pair_from_the_broad_sweep_gap_two`.
- [x] The broad sweep in `sk_op_engine.rs` tightened from "at most 30
      mismatches" back to zero, and widened with four held-out 3-4-5
      disc/disc pairs at different scales to check the fix generalizes.
- [ ] `boolean.rs` still not deleted: the widened sweep found a third,
      unrelated gap (a `Union`-only bug on a cubic/conic pair) before
      landing — see `TODO/2026-09-15-union-drops-the-far-side-of-a-cubic-and-conic-pair.md`.
      Revisit deleting `boolean.rs` once that one closes too, and re-run
      the broad sweep once more before doing it, per
      `TODO/09-bridge-winding-xor.md` item 5's own warning about trusting a
      sweep that turned out not to be broad enough — now three times this
      session.
