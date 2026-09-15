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

## Update (2026-09-15, later): pieces 1-2 done — root cause is narrower than the recap above assumed

Built the repro directly (via `op_with_engine`, not a hand-built standalone
`OpAngle` pair — see "why not a standalone OpAngle repro" below) and traced
it with temporary `eprintln!` instrumentation in `orderable`,
`convex_hull_overlaps`, and `ends_intersect` (added, used, then reverted —
not kept in the tree). Permanent pin:
`tangent_contact_disc_and_rounded_square_still_drops_the_far_side` in
`sk_op_engine.rs`, which asserts the current wrong answer so it fails loudly
once this is actually fixed.

**The recap's guess was wrong**: the curve/curve branch's fall-through to
`convex_hull_overlaps` is not where the bug is, and `convex_hull_overlaps`
is not missing an exact-tangent short-circuit for this case. At the actual
junction (`(-13.35, +26.87)` and its mirror `(-13.35, -26.87)`, where the
square's rounded-corner cubic meets the circle — *not* the `y = +-30`
extrema originally assumed by the source file's root-cause writeup),
neither `s0xt0` nor `s1xt0` (the cross products `convex_hull_overlaps` uses
for its "sweeps are exactly equal" and "same half-plane" shortcuts) come out
as exactly zero. The tangent directions are close but not coincident to the
degree those checks test for.

What actually happens: `convex_hull_overlaps` correctly declines with `-1`
via its `t_between_s` branch (traced: `t_between_s=true`) — this means one
candidate's hull genuinely wraps around the other's from the angle-sort's
point of view, which is a legitimate "cannot decide from the hull alone"
case, not a bug. `orderable` then falls through to `ends_intersect`
(exactly as designed — this is its documented job, "the main ordering path
for curves whose hulls overlap"), and **`ends_intersect`'s chord-ray
sampling is what returns the wrong answer** for this geometry. Traced its
internals (`small_ts`, `use_intersect`, `s_cept`/`sept_dir`) for both
mirror-image junctions (top: `lh=17(square) rh=5(circle)`, bottom:
`lh=6(circle) rh=18(square)`): both resolve to `sept_dir` values that are
near-exact negatives of each other (`0.0635` vs. `-0.0635`) sampled from the
same curve (the square's rounded-corner cubic) via `s_index`, and both
calls return `true`. That symmetry-under-mirroring is suspicious but not
yet proven to be *the* defect — it's as far as the trace was taken before
handing off to piece 3, since going further means reading `ends_intersect`
line-by-line against Skia's C++ (`SkOpAngle::endsIntersect`,
`SkOpAngle.cpp`) to find the actual discrepancy, which is fix-scoped work,
not diagnosis.

**Why not a standalone `OpAngle` repro (piece 1 as originally scoped)**:
`sk_op_angle.rs`'s test module already has `line_angle`/`quad_angle`
builders for two-angle cases, but reproducing *this* bug needs the angles
as they exist mid-walk — with `f_start`/`f_end`/`f_computed_end` set to
real span references into a real arena, since `ends_intersect` reads those
to find `t_start`/`t_end` for its ray-intersection search. Hand-constructing
that arena state without going through `add_intersect_ts`/`bridge` would
risk building an unfaithful repro. Going through `op_with_engine` end to end
and instrumenting is what was actually done; it's slightly heavier than a
pure `sk_op_angle_order.rs`-local test but traces the real code path
exactly, which matters more here than isolation given how state-heavy
`OpAngle` turned out to be.

## Independently testable pieces

1. ~~**Minimal repro**~~ Closed above (via `op_with_engine` +
   instrumentation instead of a standalone `OpAngle` pair; see why-not
   above). Permanent pin: `tangent_contact_disc_and_rounded_square_still_
   drops_the_far_side`.
2. ~~**Instrument and confirm which branch mis-fires.**~~ Closed above:
   it's `ends_intersect`, not `convex_hull_overlaps`'s tangent-equality
   shortcuts (those correctly decline; `t_between_s` is a legitimate hull
   wrap here, not a missed exact-tangent case). The recap's original guess
   ("the curve/curve branch... falls straight through to `ends_intersect`
   with no tangent-coincidence check at all") was about which function is
   called, which was right, but the *fix* target is inside `ends_intersect`
   itself, not a missing pre-check before calling it.
3. **Find and fix the actual defect**, now further narrowed by a
   character-for-character comparison of `ends_intersect`
   (`sk_op_angle_order.rs:574`) against `SkOpAngle::endsIntersect`
   (`SkOpAngle.cpp:502`), done in this update — see below for what was
   checked and ruled out. The port matches C++ at every point checked; the
   defect (if it is in this function at all) is not a transcription error
   found so far. **Reconsider whether the bug is actually downstream**, in
   `pick_next`/`find_next_op`/the ring walk (`sk_op_walker.rs`) rather than
   in the pairwise `orderable`/`ends_intersect` comparison itself — see
   "what's still not narrowed" below.

   **What was checked and matches C++ exactly** (line-by-line, this
   update): the ray construction (507-508 C++ / 585-587 Rust); the
   `fEnd->contains(rh->fEnd)` / `ends_share_point` short-circuit; the
   swapped-index `lPts`/`rPts` check for "already found by ordinary
   intersection" (654/666, `index==1` correctly reads the *other* curve's
   point count in both ports); the unswapped `ptCount` for `curve_extent`
   (579-580/690-695, each curve uses its own point count); the
   `fOriginalCurvePart[0] != fPart.fCurve.fLine[0]` gate (601/708-709,
   always keyed on `lh`/`this` in both ports, not `index`-dependent); and
   the final `sRayLonger ^ (sIndex == 0) ^ (septDir < 0)` combination
   (630/771, identical).

   **One real but inconsequential discrepancy found**: the marginal-delta
   band is `delta < 4e-3 && delta > 1e-3` in C++ (strict on both ends) vs.
   `(1e-3..4e-3).contains(&delta)` in Rust (inclusive at `1e-3`). Confirmed
   via instrumentation that this does **not** explain the traced bug —
   `delta` at both mirror junctions was `~6.17`, nowhere near either bound,
   so the marginal-band branch is correctly skipped either way. Worth
   fixing for exactness (`delta > 1e-3` should be strict, matching C++) if
   anyone is in this function anyway, but it is not on the causal path for
   this bug. Not fixed here since it isn't the culprit and this update's
   scope was diagnosis.

   **Also checked and ruled out**: whether `f_part.f_verb` could diverge
   from `f_original_curve_part.f_verb` (which would make the Rust port's
   `line_on_one_side_of` calls at 721-722/728-729 use different verbs than
   C++'s single `test->segment()->verb()`, since C++'s two calls share one
   verb by construction). Traced `align_to` (`sk_op_angle_order.rs:1003`,
   called on `lh`/`rh`/`angle` before every `orderable` call from `after()`)
   and confirmed it always re-derives `f_part.f_verb` from
   `f_original_curve_part.f_verb` (line 1008), so the two stay in sync by
   the time `ends_intersect` reads them. Not the bug.

   **What's still not narrowed**: with `ends_intersect` itself checked this
   closely and no discrepancy found, the defect may not be a local bug in
   the pairwise comparison at all — it could be in how `pick_next`
   (`sk_op_walker.rs:532`) or `find_next_op` assembles the full angle ring
   and picks from it, using `after()`'s three-way comparisons
   (`sk_op_angle_order.rs:882`) rather than a single `orderable` call in
   isolation. A hand geometric check at the top junction (tangent angle
   of `lh` ≈ -127.5°, `rh` ≈ -153.6° in screen coordinates, roughly 26°
   apart, cross product -13.9) confirms the two curves are genuinely
   distinguishable directionally — this is not a degenerate near-parallel
   case that any pairwise test should reasonably decline on. Whether
   `orderable(lh, rh) == true` is the *correct* answer for these two specific
   angles requires knowing which one the walk should pick given the current
   winding direction/operand — state that lives in the ring/walk, not in
   `ends_intersect` alone. That context-dependence is exactly why this
   trace stalled: verifying "is `true` correct here" needs the full ring,
   not just the pair.
4. **Re-run the disc/rounded-square tangent case** end to end once 3 lands,
   and tighten `tangent_contact_disc_and_rounded_square_still_drops_the_
   far_side` from "pins the wrong answer" to "asserts the correct one" (the
   test's own failure message says to do this).
5. **Sweep for other tangent-contact shapes** beyond the one repro (e.g.
   two circles tangent at a single point, a line tangent to a circle) to
   check the fix generalizes rather than overfitting to the one case in
   piece 4. Depends on 3 and 4 landing first.

## Acceptance

- [x] Piece 1: repro built and pinned (via `op_with_engine`, not a
      standalone `sk_op_angle_order.rs` test — see why-not above).
- [x] Piece 2: root cause narrowed to `ends_intersect`'s chord-ray sampling,
      not `convex_hull_overlaps`'s tangent-equality shortcuts as the recap
      guessed. Recorded above with the actual traced values.
- [ ] Piece 3: not fixed. `ends_intersect` was checked character-for-
      character against `SkOpAngle::endsIntersect` and matches (one
      inconsequential strict-vs-inclusive bound discrepancy found and ruled
      out as the cause). The defect either needs deeper numeric tracing
      inside a function that otherwise matches its reference, or — more
      likely per the update above — is not in this pairwise comparison at
      all and lives in how `pick_next`/`after()` assemble and read the full
      angle ring. Next step for whoever picks this up: trace `pick_next`
      (`sk_op_walker.rs:532`) for this same repro, not `ends_intersect`
      again.
- [ ] Piece 4: original disc/rounded-square tangent case fixed end to end;
      regression test tightened from pinning the bug to asserting
      correctness.
- [ ] Piece 5: fix confirmed to generalize across other tangent-contact
      shapes, or documented as narrower than hoped with what's still open.
