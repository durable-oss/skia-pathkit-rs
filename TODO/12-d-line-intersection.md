# 12 — SkDLineIntersection has no Rust file

**Independent.** 344 lines of C++. Needed by `AddIntersectTs` (item 08's
pipeline), so land it before 09 if possible.

## Problem

`old/pathkit/src/pathops/SkDLineIntersection.cpp` has no counterpart.
Coverage 0%. Unported: `intersect`, `intersectRay`, `horizontal`, `vertical`,
`horizontal_coincident`, `vertical_coincident`, `cleanUpParallelLines`,
`computePoints`, `HorizontalIntercept`, `VerticalIntercept`, `ExactPointH/V`,
`NearPointH/V`.

Line-line intersection is the base case of the whole intersection system —
every curve-curve routine bottoms out in it. `sk_path_ops_line.rs` exists
(467 lines) and has the `SkDLine` type and geometry, but not the intersection
methods, which live in this separate translation unit.

## Task

Port into `src/pathops/sk_path_ops_line.rs` (matching how Skia splits the
type from its intersection code, but keeping one Rust module) or a new
`sk_d_line_intersection.rs` — pick one and note it.

The exact/near endpoint handling (`ExactPointH`, `NearPointH`, …) is not
optional detail; it is what makes shared endpoints between adjacent segments
register as one intersection rather than two near-misses.

## Acceptance

- Crossing, parallel, collinear-overlapping, collinear-disjoint, and
  shared-endpoint line pairs each produce the correct intersection count and
  t values.
- Coincident runs report both endpoints, matching C++.
