# Detect curve/curve coincidence in `record_if_coincident`

Split out of `TODO/09-bridge-winding-xor.md` item 3, which is now closed as
"the crash it caused is fixed; the detection gap itself is not." This file
tracks the detection gap on its own, independent of the tangent-contact bug
(`TODO/2026-09-15-union-drops-the-far-side-of-a-cubic-and-conic-pair.md`) and
the polygon-union-noise investigation
(`TODO/2026-09-15-union-of-many-adjacent-line-polygons-adds-boundary-noise.md`),
which are unrelated root causes that happen to share the same downstream
blocker (`boolean.rs` deletion).

## Where it stands

`record_if_coincident` (`sk_op_engine.rs:256`) only recognizes line/line
overlap. Two curves that are identical, or that share part of one arc
exactly, get no coincidence record at all; the ray-cast winding computation
then treats the shared boundary as a real, unresolved crossing.

A prior fix (`TODO/09-bridge-winding-xor.md` item 3) made the resulting
winding-sum overflow saturate instead of panic
(`set_up_windings`/`sk_op_arena.rs`), which is enough for `Union` and
`Intersect` of two identical cubics to come back *correct* by accident (the
saturated sum happens to still pick the right winding). `Difference` of the
same pair declines rather than crashing or answering wrong.

That means: no confirmed wrong-answer case exists today, but there is also
no real detection — the correct answers so far are the saturation happening
to land right, not the engine understanding the coincidence. That is a gap
in coverage, not a clean bill of health.

## Update (2026-09-15, later): pieces 1, 2, 4 closed — a real wrong-answer case existed and is fixed

Piece 1's search (a scratch, `#[ignore]`d sweep in `sk_op_engine.rs`'s test
module, run explicitly and then removed) found the saturation-luck boundary
on the first real attempt: **identical quads and identical conics**, unlike
identical cubics, came back **wrong** under both `Union` and `Intersect` —
`op_with_engine(p, p, Union)` and `Intersect(p, p)` both returned an empty
result for a simple closed quad/conic contour unioned or intersected with
itself, confirmed wrong against `p.contains()` at several interior probe
points (not boundary points — checked separately that `p.contains()` itself
was sane before trusting the mismatch). So the earlier "no confirmed
wrong-answer case" framing undercounted, the same way `09-bridge-winding-
xor.md`'s "2 of 48" framing undercounted before its own broader sweep.
Cubics happening to saturate to the right answer was luck specific to
cubics, not a property of the winding-saturation fix in general.

Root cause: `record_if_coincident` (`sk_op_engine.rs:256`) only recognized
line/line overlap, exactly as documented above, so a quad or conic paired
with an identical copy of itself fell through to `intersect_pair`'s
numerical crossing search, which finds no usable isolated crossing for two
coincident curves — no coincidence record is created, and the ray-cast
winding computation treats the doubled boundary as an unresolved crossing.

**Fix (piece 2, closes the gap for this case):** `record_if_coincident` now
checks, before its line/line-only gate, whether `a` and `b` have the same
verb, the same weight (within tolerance), and the same control points in
order — forward or exactly reversed. When they do, the pair is recorded
coincident along their whole span (`t` in `[0, 1]` on each), via a new
`record_whole_curve_coincidence` that reuses the same `segment_add_t_
coincident` / `ArenaCoincidence::add_or_extend` machinery the line/line path
already used, just without the projection/overlap-extent math line/line
needs (the whole curve overlaps, so the extent is always `[0, 1]`). `t = 0`
and `t = 1` already have spans at a segment's own endpoints, so this adds no
spurious splits.

This fixes the found bug (identical quads/conics under `Union`/`Intersect`,
now correct) and, as a side effect, also handles identical cubics without
depending on saturation luck, and the reversed-orientation case (two cubics
sharing an arc traversed in opposite directions — regression test
`two_cubics_sharing_a_reversed_arc_union_correctly`) that was not covered
before.

**Piece 4 fell out for free**: `Difference` of two identical cubics
(`two_identical_cubics_intersect_correctly_but_difference_still_declines`,
which used to assert the decline) now answers directly instead of falling
back, and answers correctly — tightened into
`two_identical_cubics_intersect_and_difference_both_answer_correctly`,
which checks the difference result contains nothing at the same probe
points, rather than just checking it declines.

**What's still not covered**, found by the same search and left as-is
(the fix is scoped to *exact* identity, not partial overlap):
- `Xor` of identical quads/conics still declines (matches identical-cubic
  behavior already documented; `boolean.rs` answers it as the fallback).
- Two cubics that are a **partial sub-arc** of each other (one is a
  De Casteljau subdivision of the other, sharing the first half of the
  curve exactly but diverging after) still decline on `Union`, `Difference`
  and `Xor` — expected, since this is exactly the general case piece 3
  exists for, not a regression from the fix.
- `Intersect` of the reversed-shared-arc case above still declines.

None of these are new gaps; they're the general partial-overlap problem
piece 3 was already scoped to solve, now with slightly better data on its
edges (whole-identity handles more than expected — reversed orientation,
all curve verbs — but partial/sub-arc overlap needs the real t-section
work regardless).

Full test suite: 1021 passed, 0 failed, 0 ignored (was 1019 before this
session's two new permanent regression tests).

## Independently testable pieces

Each of these can be picked up and closed on its own; they do not depend on
each other or on the other two open TODO files.

1. ~~**Characterize the saturation-luck boundary.**~~ Closed above: a real
   wrong-answer case was found (identical quads/conics under Union/
   Intersect) and fixed.
2. ~~**Extend `record_if_coincident` to detect two curves that are exactly
   identical**~~ Closed above (`record_whole_curve_coincidence`).
3. **Curve/curve coincidence via `sk_path_ops_tsect`** — the general case
   (two curves that partially overlap along a shared sub-arc without being
   identical). This is the piece that actually needs the t-section
   machinery (`sk_path_ops_tsect.rs`) ported and wired into
   `record_if_coincident`. Largest and least well-scoped piece; still not
   started — the "sub-arc" and "reversed Intersect" gaps found in the
   update above are two concrete test cases ready for whoever picks this
   up (`half_subdivided_shared_subarc`-shaped input, and the reversed-arc
   pair under `Intersect` specifically).
4. ~~**Make `Difference` of two identical curves answer instead of decline**~~
   Closed above, fell out of piece 2 as expected.

## Acceptance

- [x] Piece 1: saturation-luck boundary characterized — a real wrong-answer
      case was found (identical quads/conics, Union and Intersect) and
      fixed rather than merely documented.
- [x] Piece 2: exact-identical-curve coincidence detected without t-section
      machinery (`record_whole_curve_coincidence`, same-verb/same-points/
      same-weight, forward or reversed).
- [ ] Piece 3: general curve/curve coincidence via `sk_path_ops_tsect`. Not
      started; two concrete failing shapes identified above for whoever
      picks it up (partial sub-arc overlap; reversed-arc pair under
      `Intersect`).
- [x] Piece 4: `Difference` of two identical curves answers correctly
      (fell out of piece 2, confirmed with a tightened regression test
      rather than assumed).
