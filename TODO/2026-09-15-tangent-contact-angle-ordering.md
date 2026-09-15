# Fix angle-ring mis-ordering at exact tangential contact

Split out of `TODO/2026-09-15-union-drops-the-far-side-of-a-cubic-and-conic-pair.md`,
which stays as the record of how the bug was found and root-caused. This
file tracks fixing it, broken into pieces smaller than "port the rest of
`SkOpAngle`'s tie-breaking."

## Recap of the root cause (see the source file for the full trace)

A disc union a rounded square whose straight edge lands at exactly the
circle's own extremum (generic offsets don't trigger this — only exact
tangency does). At the junction, three ring candidates sort successfully
(so `unorderable()` does not catch it) but pick the wrong one as active.
The winding math was checked character-for-character against C++ and is
not the problem — this is purely a sort/tie-break bug in `orderable`
(`sk_op_angle_order.rs:792`) or `convex_hull_overlaps`
(`sk_op_angle.rs:818`) for candidates whose tangent directions coincide.

The three curve/curve branch (`orderable`'s `else` arm, line 830) delegates
to `convex_hull_overlaps`, which is the most likely place the tie-break is
wrong, since the line/line and line/curve branches already have explicit
"exactly opposite" and `approximately_zero` handling that the curve/curve
branch lacks — it falls straight through to `ends_intersect` with no
tangent-coincidence check at all.

## Independently testable pieces

1. **Minimal repro inside `sk_op_angle_order.rs`'s own test module**,
   bypassing the full union walk. Construct the two (or three) `OpAngle`s
   at the exact tangent junction directly (not via `op_with_engine`) and
   assert what `orderable`/`convex_hull_overlaps` return today, pinned as a
   failing/ignored test. This isolates the bug from the walk, the winding
   code, and the rest of the engine, and gives every later piece a fast
   test to run.
2. **Instrument and confirm which branch mis-fires.** Given piece 1's
   repro, confirm whether the wrong answer comes from `convex_hull_overlaps`
   itself (the hull test genuinely can't distinguish the two candidates) or
   from `orderable` calling it when it should have taken a different path
   (e.g. a missed exact-tangent check before falling through). Record the
   answer in this file or piece 1's test comment — don't assume it's the
   hull test without checking.
3. **Add the missing exact-tangent-coincidence check**, modeled on the
   line/line branch's `x_ry == rx_y` handling (lines 805-809) — the
   curve/curve branch needs an analogous "tangent directions are equal"
   short-circuit before delegating to the hull test, using curvature (which
   direction each curve bends away from the shared tangent line) to break
   the tie the way C++'s `SkOpAngle::CalcAngle` does. This is the actual
   fix; keep it scoped to the tangent-equal case piece 2 identifies, not a
   rewrite of `convex_hull_overlaps`.
4. **Re-run the disc/rounded-square tangent case** (the original repro from
   the source file) through `op_with_engine` end to end and confirm it now
   matches `disc.contains(p) || square.contains(p)` at the probe points
   that were wrong before, plus a regression test alongside
   `union_of_a_disc_and_a_rounded_square_matches_the_operands`.
5. **Sweep for other tangent-contact shapes** beyond the one repro (e.g.
   two circles tangent at a single point, a line tangent to a circle) to
   check the fix generalizes rather than overfitting to the one case in
   piece 4. Depends on 3 and 4 landing first.

Do pieces 1-2 before touching any fix code — they're pure diagnosis and
de-risk piece 3 by confirming exactly what's wrong before changing it.

## Acceptance

- [ ] Piece 1: standalone failing repro in `sk_op_angle_order.rs` tests.
- [ ] Piece 2: root cause narrowed to a specific branch/function, recorded.
- [ ] Piece 3: exact-tangent tie-break implemented, piece 1's repro passes.
- [ ] Piece 4: original disc/rounded-square tangent case fixed end to end,
      regression test added.
- [ ] Piece 5: fix confirmed to generalize across other tangent-contact
      shapes, or documented as narrower than hoped with what's still open.
