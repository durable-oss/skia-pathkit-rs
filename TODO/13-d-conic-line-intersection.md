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
