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
