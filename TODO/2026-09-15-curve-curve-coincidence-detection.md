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

## Independently testable pieces

Each of these can be picked up and closed on its own; they do not depend on
each other or on the other two open TODO files.

1. **Characterize the saturation-luck boundary.** Find or construct a
   curve/curve coincidence case where `Union`/`Intersect` come back *wrong*
   (not just `Difference` declining) — i.e. where saturating the winding sum
   does not accidentally land on the right answer. If none is found after a
   real search (not just the two cases already tried in
   `sk_op_engine.rs`: `two_identical_cubics_union_to_one_of_them`,
   `two_overlapping_cubics_sharing_an_arc_union_correctly`), record that as
   a negative result the way the polygon-union file records its negative
   sweeps, rather than leaving it implicit.
2. **Extend `record_if_coincident` to detect two curves that are exactly
   identical** (same control points, same order) as a special case that
   needs no t-section machinery — a pure equality check on the verb list
   before falling through to line/line. Narrowest possible slice of the
   general problem.
3. **Curve/curve coincidence via `sk_path_ops_tsect`** — the general case
   (two curves that partially overlap along a shared sub-arc without being
   identical). This is the piece that actually needs the t-section
   machinery (`sk_path_ops_tsect.rs`) ported and wired into
   `record_if_coincident`. Largest and least well-scoped piece; do not
   start here — do 1 and 2 first, since either may narrow what 3 actually
   needs to cover.
4. **Make `Difference` of two identical curves answer instead of decline**,
   once 2 is in place — should fall out of 2 rather than needing separate
   work, but confirm with a regression test
   (`two_identical_cubics_difference_is_empty` or similar) rather than
   assuming.

## Acceptance

- [ ] Piece 1: saturation-luck boundary characterized, with either a found
      wrong-answer case or a documented negative sweep.
- [ ] Piece 2: exact-identical-curve coincidence detected without t-section
      machinery.
- [ ] Piece 3: general curve/curve coincidence via `sk_path_ops_tsect`.
- [ ] Piece 4: `Difference` of two identical curves answers correctly.
