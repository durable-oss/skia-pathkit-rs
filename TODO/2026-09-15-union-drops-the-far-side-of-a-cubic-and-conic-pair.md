# Union drops the far side of a cubic/conic pair — traced to exact tangency

Found while widening the broad sweep in `sk_op_engine.rs`
(`a_broad_sweep_of_shape_pairs_never_declines_and_matches_the_fallback`) as
due diligence before deleting `boolean.rs`, per item 5 of
`TODO/09-bridge-winding-xor.md`. Not the same bug as the two gaps fixed
alongside that widening (see
`TODO/done/2026-09-15-broad-sweep-found-two-more-op-with-engine-gaps.md`).

## Root cause found: the original repro was a degenerate tangency, not a generic bug

The disc (`add_circle`, radius 30 at the origin) and the rounded square in
the original repro (center `(15, 0)`, half-width 30, corner radius 8) share
one more coincidence than the file first assumed: half-width 30 puts the
square's straight top and bottom edges at exactly `y = ±30` — the same `y`
as the circle's own topmost and bottommost points. The two shapes don't
just cross there, they touch tangentially, with near-parallel tangent
directions at the contact.

Confirmed by direct test: the identical construction at a **generic**
offset/half-width (`rounded_square_cubics(15.0, 3.0, 22.0, 8.0)`, chosen so
neither straight edge lines up with either circle extremum) unions
correctly — `contains()` agrees with `disc.contains(p) || square.contains(p)`
at every probed point, including points on the far (positive-`x`) side that
the tangent case dropped. This is now pinned as
`union_of_a_disc_and_a_rounded_square_matches_the_operands` in
`sk_op_engine.rs`.

Traced the tangent case's walk directly (temporary instrumentation on
`bridge`/`walk_contour`/`find_next_op`/`active_op_with`, since none existed
before): at the junction where the walk should cross from the circle's
boundary onto the square's own far edge, three ring candidates are sorted,
and the wrong one is picked as active — the square's continuing edge comes
back `false` when it should be `true`, and a tiny near-degenerate fragment
comes back `true` instead. The winding math (`set_up_windings`,
`active_op_with`, the `ACTIVE_EDGE`/`gActiveEdge` table) was checked
character-for-character against the C++ source
(`old/pathkit/src/pathops/SkOpSegment.cpp`) and matches exactly — the bug is
not there. It is in how the angle ring orders (or fails to distinguish)
candidates whose tangent directions coincide at a degenerate contact point.

**That is a different, and much bigger, class of problem than this file was
scoped for.** Real Skia's `SkOpAngle` has substantial machinery
(`orderable`/`unorderable` fallback logic, curvature comparison via
`SkOpAngle::CalcAngle`/`convexHullOverlaps`, and further heuristics in
`SkOpAngle.cpp`) specifically for disambiguating candidates whose linear
sort is ambiguous, i.e. exactly this tangent-contact case. It is not
`unorderable` here — the ring does sort successfully, just to the wrong
answer — so it is not caught by the `unorderable()` fallback either; the
bug is in the sort's tie-breaking for a genuinely hard geometric case, not
a missing guard. Reproducing that machinery is a project on the scale of
the angle-sorting work already in `sk_op_angle.rs`/`sk_op_angle_order.rs`,
not a bounded bug fix, and out of scope for this file.

## Disposition

- The generic (non-tangent) case, which is what a rounded-square/disc union
  actually looks like in ordinary use, works correctly and is now covered
  by a regression test.
- The tangent case is real but narrow: it needs two curves whose tangent
  lines coincide (not merely cross) at an intersection point, which is a
  measure-zero geometric coincidence for arbitrary inputs. It is not
  fixed here.
- It also means `op_with_engine` gives a **wrong, non-declining answer**
  for this class of input, not a decline — the walk picks a candidate and
  emits a result, it just emits the wrong one. That is exactly the
  situation `boolean.rs` exists to be a safety net for, and there is
  currently no gate that detects "the engine answered, but this was a
  tangent-contact case it cannot be trusted on" the way `empty_is_the_answer`
  gates the empty-result case. So this gap is not caught by anything
  upstream of the wrong answer reaching a caller.

## boolean.rs: not deleted

Item 5 of `09-bridge-winding-xor.md` also asked, once the sweep came back
clean, to delete `boolean.rs` and the substitute helpers in
`sk_path_ops_simplify.rs`. Re-ran the full broad sweep first, per that
item's own warning about trusting a sweep that turned out not broad enough
twice already this session:
`a_broad_sweep_of_shape_pairs_never_declines_and_matches_the_fallback`
passes at zero mismatches, and a disc/rounded-square case at a few
non-tangent offsets was added and passes too.

**But the sweep coming back clean does not mean this file's own gap is
closed** — the tangent-contact bug above is a real wrong answer, found by
hand-picked geometry precisely because a broad *random* sweep is unlikely
to land exactly on a tangency (that's why the sweep didn't catch it in the
first place). "The sweep is clean" and "the engine has no known
wrong-answer gaps" are different claims, and item 5's own text conflates
them. Given that, and that item 3 of `09-bridge-winding-xor.md` is *also*
still open ("curve/curve coincidence... no known failing case, but no
coverage either" — an honest admission of untested territory, not a clean
bill of health), deleting the only fallback now would remove the safety
net for at least one confirmed wrong-answer case and at least one
unaudited one. `boolean.rs` and the substitute helpers in
`sk_path_ops_simplify.rs` are kept.

If the tangent case gets fixed later, the fix belongs in the angle-ordering
code (`sk_op_angle.rs`/`sk_op_angle_order.rs`), specifically wherever the
ring sort picks between two candidates whose direction vectors are equal or
opposite at the junction — it should probably start from Skia's own
`SkOpAngle` tie-breaking logic (`SkOpAngle.cpp`'s curvature-comparison
fallback) as a reference rather than from scratch. Until then, `boolean.rs`'s
deletion stays blocked on this file plus item 3.

**The fix itself is tracked in its own file, broken into small
independently-testable pieces:**
`TODO/2026-09-15-tangent-contact-angle-ordering.md`. This file stays as the
investigation record (root cause, what was ruled out); further work belongs
in the split-out file, not here.

## Acceptance

- [x] Root cause identified: exact tangency between the two operands'
      boundaries, not a general Union-walk bug. The winding tables and
      windings-computation code were checked against C++ and are correct.
- [x] The disc/rounded-square pair resolves correctly for `Union` at a
      generic (non-tangent) offset, checked by interior-point containment.
- [x] A regression test pins the generic case
      (`union_of_a_disc_and_a_rounded_square_matches_the_operands`).
- [ ] The exact-tangency case itself is not fixed. It is a real
      wrong-answer gap (not a decline), which is why `boolean.rs` stays —
      see "boolean.rs: not deleted" above. A fix belongs in the
      angle-ordering machinery, not in this walk-level file.
