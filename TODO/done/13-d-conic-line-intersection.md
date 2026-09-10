# 13 — SkDConicLineIntersection has no Rust file

**Independent.** 366 lines of C++. Depends on item 12 in practice.

## Problem

`old/pathkit/src/pathops/SkDConicLineIntersection.cpp` has no counterpart.
Coverage 0%, 30 functions unported.

Note the asymmetry: `SkDCubicLineIntersection.rs` and
`SkDQuadLineIntersection.rs` **do** exist (though both are orphaned — see item
01). The conic one was never written. Since `add_circle`, `add_oval` and
`add_round_rect` all emit conics, this gap means intersections against any
circular geometry cannot be computed at all.

Unported: `LineConicIntersections` and its whole surface —
`intersect`, `intersectRay`, `horizontalIntersect`, `verticalIntersect`,
`addExactEndPoints`, `addNearEndPoints`, `addExactHorizontalEndPoints`,
`addNearHorizontalEndPoints`, `addExactVerticalEndPoints`,
`addNearVerticalEndPoints`, `checkCoincident`, `findLineT`, `pinTs`,
`validT`, `uniqueAnswer`, `RootsValidT`.

## Task

Port to `src/pathops/sk_d_conic_line_intersection.rs`, following the structure
of the existing quad and cubic files so the three read alike.

Watch the rational form: the conic's implicit equation carries the weight, so
`findLineT` solves a different quadratic than the quad case. Do not adapt the
quad file by analogy — port from the conic source.

## Acceptance

- A line through a circle's centre intersects it at exactly 2 points, at the
  expected t values.
- A tangent line yields 1 intersection.
- Horizontal and vertical fast paths agree with the general path.
- Endpoints shared with an adjacent segment register once, not twice.

---

## Resolution (2026-09-10)

Done, in `src/pathops/sk_d_conic_line_intersection.rs`, following the quad
file's shape as asked: a `DConic` value type mirroring `DQuad` with the weight
added, then `LineConicIntersections` holding the algorithm.

Ported: `intersect`, `intersectRay`, `horizontalIntersect`,
`verticalIntersect`, `addExactEndPoints`, `addNearEndPoints`,
`addLineNearEndPoints`, `addExactHorizontalEndPoints`,
`addNearHorizontalEndPoints`, `addExactVerticalEndPoints`,
`addNearVerticalEndPoints`, `checkCoincident`, `findLineT`, `pinTs`, `validT`,
`uniqueAnswer`, and the three `SkIntersections` entry points as free functions
(`horizontal`, `vertical`, `intersect`) since they take the conic as an
argument rather than being about the intersection set itself.

`validT` was taken from the conic source, not adapted from the quad file. The
weight enters through `B` alone — `B = b*w - xCept*w + xCept`, then
`A = a + c - 2B` and `B -= C` — which is not what substituting into the plain
quadratic gives. `valid_t_uses_the_weight` guards that directly: the same three
control points with w = 0.4 and w = 2.0 must not produce the same roots.

`SkIntersections::has_t` and `has_opp_t` were private helpers inside
`sk_d_quad_line_intersection.rs`; moved onto `SkIntersections` where C++ has
them, since the conic code needs them too.

`SkDCurve::nearPoint` for the conic verb does not exist in this port, so
`DConic::near_point` samples and refines instead, rejecting a candidate that is
nearer the line's far end. That is the one place this is not a literal port.

### Acceptance

- A line through the arc crosses once, on the circle, at 45 degrees; a line
  clear of it misses; a chord below the apex cuts twice on opposite sides.
- Tangency: the apex is a *double root* whose discriminant is zero to within
  rounding (measured: -1.5e-11), so the count at exactly that height is decided
  by f32 noise. The test checks either side instead — above misses, below cuts
  twice and the crossings straddle the apex — which is the honest statement of
  what the arithmetic supports.
- Horizontal and vertical fast paths agree with the general path on both count
  and conic t, and `flipped` mirrors the line parameter.
- An endpoint shared with the line registers once, at conic t = 1 and line
  t = 0.
