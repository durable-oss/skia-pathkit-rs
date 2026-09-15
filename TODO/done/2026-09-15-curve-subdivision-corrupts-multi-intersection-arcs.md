# Curve subdivision corrupts arcs cut at 2+ intersection points

Found while closing `09-bridge-winding-xor.md`. Not caused by that work —
reproduces on `op_with_engine`'s `Union`, which nothing in that session
touched — but it made `bridgeXor`'s new curved-input cases unreliable, so
it was filed separately rather than folded in.

## Root cause (not what the title suggests)

Despite the name, this had nothing to do with `segment_add_t`, `PtT`, or a
segment picking up a second intersection t value. `span_sub_divide` and
`sub_divide_curve` were traced end to end with debug prints and computed the
right control points every time, at every call site.

The corruption was in `SkPathWriter::close` (`sk_path_writer.rs`), in how a
finished contour gets copied from the writer's scratch path (`self.current`)
into the caller's output path. The old code:

```rust
for i in 0..temp.count_verbs() {
    if let Some(verb) = temp.verb(i) {
        self.append_verb(verb, &temp, i);
    }
}
```

and `append_verb` read `contour.point(verb_idx)` — indexing the path's flat
point array by the verb's *position in the verb list*, not by how many
points had actually been consumed so far. That only lines up while every
verb before the current one contributed exactly one new point (Move, Line).
The moment a Quad/Conic (2 new points) or Cubic (3 new points) appears
before another verb, every later verb reads from the wrong offset — short by
however many extra points the curve verbs ahead of it contributed. A conic
weight had the same bug: `conic_weights().get(0)` always read the *first*
conic weight in the whole contour, never the one belonging to the conic
being copied.

Two circles are each four conics, so the second one onward already reads
from the wrong offset — hence every conic in the dump except the first
showed a corrupted control point (typically the previous verb's start point,
since the read landed one point short).

Fixed by rewriting `close()` to walk `temp.iter()` instead, which already
resolves each verb's own start/control/end points and, for a conic, its own
weight, matching the RawIter convention the rest of the codebase already
uses (see `PathIter` in `core/path.rs`). This deleted `append_verb` entirely
— nothing else called it.

This explains why the bug needed "2+ intersections on one arc": that's what
it takes to get more than one curve verb into a contour that also goes
through partial-contour assembly (a walk that has to close via `finish_contour`
rather than the direct `is_closed` path). It also explains why it was not
`bridgeXor`-specific and not new — `SkPathWriter::close` is shared by every
walk in the engine.

## What was ruled out (superseded by the above, kept for the record)

- **Topology.** Verified correct against the cached `skia-pathops` Python
  package as an oracle before the fix, and still is: the point cycle was
  never wrong, only the emitted curve verbs downstream of it.
- **The subdivision math (`sub_divide_curve`, `cubic_sub_divide_controls`,
  `conic_sub_divide_control`).** Confirmed correct again during this fix, by
  instrumenting `span_sub_divide` directly: it always returned the right
  `f_curve` before `add_curve_to` handed it to the path writer.
- **`segment_add_t`'s sorted insertion.** Read in full; it inserts by t order
  using real `SpanId`/`PtTId` links, not indices into a vector that a later
  insert could shift. The original hypothesis in this file's first draft
  assumed an index-based structure that this arena does not use.

## Fix

`src/pathops/sk_path_writer.rs`: `SkPathWriter::close` now copies a finished
contour verb-by-verb via `Path::iter()`, and the dead `append_verb` helper
(the buggy verb-index-as-point-index reader) is deleted.

## Regression tests

Added to `src/pathops/sk_op_engine.rs`:

- `two_overlapping_circles_union_keeps_correct_curve_geometry` — the disc
  pair from this file's original repro, through `op_with_engine(Union)`,
  checked by `contains()` at five points against the two circles' own
  geometry.
- `two_overlapping_circles_simplify_keeps_correct_curve_geometry` — the same
  geometry as one even-odd path through `simplify_with_engine`, same
  containment check.
- `discs_union_to_one_contour_across_the_offset_sweep` extended with
  interior-point containment checks (previously only asserted contour
  count, which a corrupted-but-single-contour result would still pass).

All three pass. Full suite: 1004 lib tests + 18 doctests, 0 failed, 0
ignored — no regressions from the rewrite.

## empty_is_the_answer (task item 5)

Checked, not changed. For `Union` with non-nested operands,
`empty_is_the_answer` falls through to its box-overlap fallback, which
always answers `false` (not-empty) for `Union` unconditionally — it never
actually inspected the corrupted result to decide. The original
empty-Union symptom was a downstream effect of the corrupted curve geometry
happening to produce a degenerate walk, not a bug in this gate. With the
writer fix in place, the two-large-overlapping-circles case no longer comes
back empty, and `empty_is_the_answer` did not need to change.

## Acceptance

- [x] Root cause identified — in `SkPathWriter::close`'s point-array
      indexing, not in `segment_add_t` or `span_sub_divide` as originally
      hypothesized (see "Root cause" above for why the original hypothesis
      was reasonable but wrong).
- [x] The disc-pair case above resolves correctly for `op(Union)` and for
      `simplify` under even-odd fill, checked by interior-point
      containment against known circle geometry.
- [x] A regression test pins it (two dedicated tests, plus the existing
      offset sweep extended with containment checks).
</content>
</invoke>
