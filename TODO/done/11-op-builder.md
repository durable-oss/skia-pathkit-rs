# 11 — SkOpBuilder has no Rust file

**Independent.** Small (263 lines of C++).

## Problem

`old/pathkit/src/pathops/SkOpBuilder.cpp` has no counterpart under `src/`.
Coverage 0%. Unported: `add`, `resolve`, `reset`, `Intersects`, `ReversePath`,
`FixWinding`, `ComputeFirstDirection`, `one_contour`.

`pathops::mod.rs` defines its own `OpBuilder` that just replays
`op(result, next, op)` in a loop. C++ `SkOpBuilder::resolve` is smarter: for
an all-union sequence it concatenates every path into one, fixes the winding
so nested contours do not cancel, and calls `Simplify` **once** instead of
running N-1 pairwise ops. That is both faster and more numerically robust,
which is the entire reason the class exists.

## Task

Port to `src/pathops/sk_op_builder.rs`:

- `ComputeFirstDirection` / `FixWinding` — reverse contours whose direction
  would cancel their neighbours under winding fill.
- `Intersects`, `ReversePath`, `one_contour`.
- `resolve` — the all-union fast path, falling back to pairwise for mixed
  operator sequences.

Then decide whether `pathops::OpBuilder` delegates to it or is replaced by it.
Do not leave two builders.

Note `FixWinding` is declared in `SkPathOpsCommon.h` but defined in
`SkOpBuilder.cpp` — port it here, not in item 08.

## Acceptance

- Unioning 5 overlapping rects via the builder gives the same filled region
  as 4 sequential `op` calls, in one `Simplify` pass.
- Nested same-direction contours do not cancel.
- One `OpBuilder` type in the crate, not two.

---

## Resolution (2026-09-10)

Done, in `src/pathops/sk_op_builder.rs`. Ported `add`, `reset`, `resolve`,
`Intersects`, `ReversePath`, `ComputeFirstDirection`, `one_contour` and
`FixWinding` (which, as the note said, lives in `SkOpBuilder.cpp` rather than
with its declaration).

`pathops::mod.rs`'s own `OpBuilder` is gone; `mod.rs` re-exports this one, so
there is a single builder type in the crate. Its existing `op_builder_basic`
test now exercises the real implementation and still passes.

`resolve` takes the all-union fast path when every operation is a union over a
non-inverse path and each path is either convex (with a determinable direction,
reversing to agree with the first) or bounds-disjoint from everything before
it. It then simplifies each path, fixes its winding, concatenates, and
simplifies once. Anything else replays pairwise, as in C++.

### Departures, both deliberate

- **`FixWinding`'s general case is not ported.** The C++ multi-contour path
  runs `FindSortableTop` over the contour graph to order nested contours; that
  is items 02-07 and does not exist yet. A multi-contour path here gets its
  fill type corrected and nothing else. This is sound for `resolve`, which only
  calls it for paths that are convex (hence single-contour) or disjoint — but
  it is a real gap for a caller invoking `fix_winding` directly on a nested
  path, and should be finished when `FindSortableTop` lands.
- **`ComputeFirstDirection` is a shoelace test**, not Skia's cross-product
  walk. `Path` exposes no convexity or direction API to build on. Curves are
  evaluated (conics at their weight, not as quads) so the sign matches the
  region actually filled.

### Acceptance

- `union_of_five_overlapping_rects_matches_sequential_ops` samples the region
  and asserts the fast path and four sequential `op` calls agree everywhere,
  skipping only points within half a unit of an edge.
- `nested_same_direction_contours_do_not_cancel` puts a small square inside a
  large one, both through `fix_winding`, and checks the middle stays filled.
- One `OpBuilder` in the crate.
