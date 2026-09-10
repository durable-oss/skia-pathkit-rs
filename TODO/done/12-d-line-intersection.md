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

---

## Resolution (2026-09-10)

Done. Ported into `src/pathops/sk_intersections.rs` rather than a new file, and
not into `sk_path_ops_line.rs`: in C++ these are methods on `SkIntersections`,
not on `SkDLine`, and they need direct access to `fT`, `fPt`, `fUsed` and
`fIsCoincident`. `sk_path_ops_line.rs` already held the `SkDLine` geometry and
the endpoint helpers (`exact_point`, `near_point`, `exact_point_h/v`,
`near_point_h/v`, `near_ray`), which this calls into. `pin_t` was made `pub` for
the intercept helpers.

Ported: `intersectRay`, `intersect`, `horizontal`, `vertical`,
`horizontal_coincident`, `vertical_coincident`, `HorizontalIntercept`,
`VerticalIntercept`, `cleanUpParallelLines`, `computePoints`.

What was there before was not a port. `line_line` returned 0 for any parallel
pair, so collinear overlap was invisible; none of the three did exact- or
near-endpoint matching, so a vertex shared by two adjacent segments never
registered; and nothing set the coincident flags. The old names are kept as
thin wrappers taking `&[Point; 2]`, since `sk_add_intersections` calls them.

Tests (13, in `sk_intersections.rs`): crossing lines meet once at the middle;
parallel never meet; collinear-overlapping report both ends *and* set both
coincident bits; collinear-disjoint report nothing; a shared endpoint registers
once with t = 1 on one side and t = 0 on the other; `intersect_ray` finds
crossings beyond the segment ends where `intersect` finds none, reports the
whole span for coincident rays, and misses non-coincident parallel rays;
horizontal and vertical spans each cross once, respect `flipped` by mirroring
the span parameter, and miss a line outside the span; the intercept helpers
agree with the full routines and pin out-of-range values to the ends.

`ExactPointH/V` and `NearPointH/V` were already present and are now exercised
through `horizontal` / `vertical`.
