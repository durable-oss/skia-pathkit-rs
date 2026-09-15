# Union drops the far side of a cubic/conic pair

Found while widening the broad sweep in `sk_op_engine.rs`
(`a_broad_sweep_of_shape_pairs_never_declines_and_matches_the_fallback`) as
due diligence before deleting `boolean.rs`, per item 5 of
`TODO/09-bridge-winding-xor.md`. Not the same bug as the two gaps fixed
alongside that widening (see `TODO/done/2026-09-15-broad-sweep-found-two-more-op-with-engine-gaps.md`)
— confirmed by tracing directly, not assumed: `find_crossings` finds the
right crossings here (`graph_operands_do_not_cross` correctly reports
`false`), and `Intersect`, `Difference` and `Xor` on the same pair all
answer correctly at every probed point. Only `Union` is wrong.

## Symptom

A disc (`add_circle`, so conics) unioned with a hand-rolled rounded square
(straight edges plus four cubic corners) at a moderate offset:

```
let mut disc = Path::new();
disc.add_circle(0.0, 0.0, 30.0);
// half-width 30, corner radius 8, cubic corners (k = r * 0.5522847498)
let square = rounded_square_cubics(/* center */ 15.0, 0.0, /* half */ 30.0, /* corner r */ 8.0);
let union = op_with_engine(&disc, &square, PathOp::Union).unwrap();
```

`union` comes back non-empty but wrong: every point checked comes back
`false` for `x` past roughly `0.007` — the entire right-hand two-thirds of
what should be a large union region is missing. The emitted contour's
points never exceed `x ≈ 0.007` (both circle and square extend well past
`x = 40`), and the verb sequence traces only the disc's own left arc plus
a sliver, not a merged boundary. `Intersect`, `Difference` and `Xor` on the
identical pair all agree with `contains()` on both operands at the same
probe points — this is not a graph-construction problem, and not the
quadrant-point crossing gap just closed (the offset here, 15, was chosen
arbitrarily for sweep coverage and has no special radius relationship to
30).

## What was ruled out

- **The test shape itself.** `rounded_square_cubics`'s corners use the
  standard circular-arc cubic-Bezier approximation constant
  (`k = r * 0.5522847498`), and at offset `0.0` (concentric, same
  center) the same construction unions correctly with the disc — so the
  shape is not self-intersecting or malformed; the bug needs the
  off-center case specifically.
- **Crossing detection.** `graph_operands_do_not_cross` reports `false`
  for this pair (real crossings were found and split), unlike gap 1 of
  the sibling TODO.
- **A blanket walk bug.** `Intersect`/`Difference`/`Xor` on the exact same
  built graph answer correctly, so whatever is wrong is specific to
  `Union`'s branch of the walk (`find_next_op`'s active-edge gate, or
  `bridge`'s handling of the `Union` case), not the graph or the crossing
  set feeding it.

## Likely area

Not confirmed — this file exists to record the repro and scope, not a
traced root cause. Given `Union` alone is wrong on an otherwise-correctly-
built graph, the next place to look is the active-edge test `find_next_op`
uses to decide whether a given winding state means "this edge is on the
`Union` boundary," or `bridge`'s outer contour-restart loop for `Union`
specifically (an active starter for the second contour segment might not
be found, matching the "only the near side gets emitted" shape of the
mismatch).

## Task

1. Reproduce with the exact geometry above (disc radius 30 at origin,
   rounded square center `(15, 0)` half-width 30 corner-radius 8) and trace
   `bridge`'s outer loop and `find_next_op`'s active-edge decision at the
   crossing spans on the far (positive-`x`) side of the union, comparing
   against what `Intersect`'s call to the same `bridge` machinery does
   differently.
2. Fix at the root cause in the walk, not by special-casing this shape
   pair.
3. Regression test: this exact disc/rounded-square pair, `Union` only
   (the other three operators already pass), checked by interior-point
   containment at several probe points spanning the missing region.
4. Once fixed, re-add the held-out cubic/conic case to the broad sweep in
   `sk_op_engine.rs` (a disc against `rounded_square_cubics` at a few
   offsets) — it was pulled back out when this bug surfaced rather than
   landed as a new decline/mismatch source; see the sweep's own comments
   for where it was removed.
5. Re-run the broad sweep once more before revisiting `boolean.rs`
   deletion, per `TODO/09-bridge-winding-xor.md` item 5's warning about
   trusting a sweep that turned out not to be broad enough, twice already
   this session.

## Acceptance

- [ ] Root cause identified in the `Union` walk path, not worked around.
- [ ] The disc/rounded-square pair above resolves correctly for `Union`,
      checked by interior-point containment.
- [ ] A regression test pins it.
- [ ] The cubic/conic pair is back in the broad sweep, passing.
