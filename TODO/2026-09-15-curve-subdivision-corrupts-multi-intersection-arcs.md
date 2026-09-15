# Curve subdivision corrupts arcs cut at 2+ intersection points

Found while closing `09-bridge-winding-xor.md`. Not caused by that work —
reproduces on `op_with_engine`'s `Union`, which nothing in this session
touched — but it makes `bridgeXor`'s new curved-input cases unreliable, so
it's filed separately rather than folded in.

## Symptom

Two overlapping circles (`Path::add_circle`, conics; a hand-rolled 4-cubic
circle reproduces it too) through `op_with_engine`/`simplify_with_engine`
come back with corrupted output. Two concrete cases:

- `op(circleA, circleB, Union)`: comes back **empty** (just `Move`+`Close`,
  no curves at all), for two circles centered 10 apart with radius 20 each
  — a case with a large, unambiguous overlap.
- `simplify` of the two circles combined into one even-odd path: resolves
  (does not decline) with a plausible-looking two-contour result, but
  `contains()` disagrees with the flattening fallback (`boolean.rs`/
  `sk_path_ops_simplify.rs`) at multiple interior points, e.g. a point
  inside only one of the two circles comes back excluded when it should be
  included.

The corrupted curves are visible directly in the output's control points:
adjacent `Cubic`/`Conic` verbs repeat a point as both an endpoint and the
*next* piece's control point, e.g.

```
Cubic [(50,30), (50,39.3192), (50,30), (50,20.68)]
```

— the second control point collapses back onto the piece's own start
point, which is not a valid subdivision of a circular arc. The same
pattern shows up in the conic case (`Path::add_circle`'s representation):
`Conic [(30,50), (30,50), (10,50), _]`.

## What was ruled out

- **Topology.** For the even-odd case, the *sequence of points and which
  segments contribute to each output contour* matches real Skia's answer
  exactly, verified against the cached `skia-pathops` Python package (a
  real Skia build) as an oracle — same two contours, same point cycle
  (one reversed relative to the other, which is immaterial under even-odd
  fill). So `bridgeXor`/`find_next_xor` walk the graph correctly; this bug
  is downstream of that, in how a chosen span pair gets turned into an
  emitted curve.
- **The subdivision math itself.** `sub_divide_curve`,
  `cubic_sub_divide_controls`, and `conic_sub_divide_control`
  (`sk_op_angle.rs`) all have direct unit tests confirming they reproduce
  the original curve's points at `t1..t2` correctly given a `t1 < t2` pair.
  Not a formula bug.
- **Straight-line geometry.** Two crossing rectangles under the same
  `simplify`/even-odd path (`bridge_xor`) come back correct at every
  probed point. The bug needs curves.
- **Not `bridgeXor`-specific.** `op_with_engine`'s `Union` (untouched by
  this session, exercised by `bridge`/`active_op`, a different walk
  entirely) shows the identical corrupted-control-point pattern on the
  same disc pair. Both walks call the same `add_curve_to` →
  `span_sub_divide` → `sub_divide_curve` pipeline, so the shared suspect is
  that pipeline's inputs, not either walk.

## Likely area

Not confirmed, but the working hypothesis given the above: the t-value (or
`PtT`) recorded on a span is wrong for at least one of the two spans
`add_curve_to` is asked to subdivide between, specifically when a single
original curve gets **two or more** new intersection points added to it
(as circle A's arc does here: one arc facing circle B picks up two new
`t`s from `AddIntersectTs`, not one). `segment_add_t`'s sorted insertion
is the next place to look — check whether inserting a second `t` on an
arc that already has one shifts or corrupts the first one's recorded
value, or whether `span_sub_divide`'s `PtT` lookup is reading the wrong
entry after such an insert.

## Why `op`/`simplify` don't already catch this

`op_with_engine`'s empty-Union case should be indistinguishable from "the
walk found nothing," which `empty_is_the_answer` is supposed to gate —
worth checking why it did not, since two large overlapping circles
unioning to nothing should never be treated as a real answer.
`simplify_with_engine`'s case is worse: it resolves without complaint, so
nothing currently steers a caller to the flattening fallback for it. See
the discussion on `09-bridge-winding-xor.md` (2026-09-15) about routing
`simplify` through the engine regardless — that decision was made with
this gap known and accepted; this file is where the fix belongs.

## Task

1. Reproduce with a minimal case (two circles, or two cubics/conics with a
   shared arc that crosses another curve at two points) and trace
   `segment_add_t`/`span_sub_divide` for the corrupted span to find where
   the wrong t/PtT enters.
2. Fix it there, not by adding a workaround in `add_curve_to`.
3. Regression test: the disc-pair geometry from this file, checked by
   `contains()` at several interior points against the flattening
   fallback, for both `op(Union)` and `simplify` under even-odd fill.
4. Once fixed, re-run the disc-union sweep in `sk_op_engine.rs`
   (`discs_union_to_one_contour_across_the_offset_sweep`) with point
   containment checks added, not just contour count — that sweep would
   not have caught this, since it only asserts one contour, not that the
   contour is the right shape.
5. Re-check `empty_is_the_answer`'s gate for the Union-comes-back-empty
   case above; either it has its own bug, or the corrupted curve output
   is what's making the result look non-empty enough to slip past it in
   other cases while returning truly empty in this one.

## Acceptance

- [ ] Root cause identified in `segment_add_t` or `span_sub_divide`, not
      worked around.
- [ ] The disc-pair case above resolves correctly for `op(Union)` and for
      `simplify` under even-odd fill, checked by interior-point
      containment against the flattening fallback.
- [ ] A regression test pins it.
