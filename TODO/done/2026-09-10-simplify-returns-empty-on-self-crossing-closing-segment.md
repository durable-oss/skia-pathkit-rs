# `pathops::simplify` returns an empty path when the closing segment crosses the body

Found: 2026-09-10, against `18c17a6` ("feat(pathops): port SkPathOpsSimplify to Rust").

## Summary

`pathops::simplify` returns `Ok` with an **empty** path for a closed polyline
whose implicit closing segment crosses the rest of the contour. The input is a
perfectly ordinary self-intersecting path with a large filled area; the correct
output is one or more non-overlapping contours covering that area, and instead
the whole shape disappears.

It returns `Ok(empty)` rather than `Err`, so a caller that only checks for an
error sees a successful simplification of a glyph into nothing.

## Repro

`examples/simplify_empty_repro.rs` in this repo — `cargo run --example
simplify_empty_repro`. It panics on the assert at the end.

```
full path:          in=34 verbs, out=0 verbs, empty=true    <- bug
last point removed: in=33 verbs, out=30 verbs, empty=false  <- fine
```

The path is a 33-point polyline reduced from a font glyph's swept stroke (the
waist of a `3`, where the bridge passes under the upper bowl). Removing only
the final point — the one whose closing segment crosses the body — makes
`simplify` behave correctly on the remaining 32.

## What is and is not the trigger

Each of these was tested separately against the same point set:

| Variation | Result |
|---|---|
| As-is | **empty** |
| Duplicate consecutive points removed (3 pairs) | **empty** |
| Coordinates scaled 10x | **empty** |
| Final (crossing) point removed | 30 verbs, correct |
| Deduplicated *and* final point removed | 30 verbs, correct |
| Last 6 points alone | 6 verbs, correct |

So it is not the duplicate points, not float resolution at that magnitude, and
not the crossing geometry in isolation — a simple bowtie at the same coordinate
range (`(200,300) (400,500) (400,300) (200,500)`) simplifies correctly into two
triangles with the crossing vertex inserted. It takes a contour of this length
*plus* a self-crossing closing segment.

## Why it matters here

This is the blocking issue for resolving self-intersecting swept strokes in
`david-fonty-lispy`. A pen stroke is one closed path traced up one side of its
spine and back down the other; wherever the spine doubles back far enough for
the stroke to touch its own body, the path crosses itself, and under the
nonzero rule the doubly-enclosed region winds to zero and drops out. The glyph
renders with a bite taken out of it exactly where it should be thickest.

`dfl::overlap::self_union` calls `pathops::simplify` to repair that. It
currently has a fallback that returns the contour unchanged when simplify comes
back empty, which is why the bite is still visible rather than the stroke
vanishing — but the repair never happens. `neofujara`'s `three` has 12
self-intersections and is the real-world case.

## Related

`pathops::op(.., PathOp::Union)` looks like it may share a root cause, though I
have not confirmed they are the same defect:

- Two copies of a disc offset 1 unit apart (≈99% overlapping) return **2**
  contours instead of 1. Rectangles union correctly at every offset; curved
  input fails across a band of offsets that scales with the radius (roughly
  0.5–20 units at r=120, 0.5–5 at r=40, correct at both extremes).
- At exact tangency, two shapes return **1** contour that is only the *first*
  input — the second is dropped entirely rather than merged.

Failing tests for those live in `david-fonty-lispy/tests/overlap.rs`
(`a_disc_offset_from_itself_stays_a_disc`,
`an_ellipse_offset_along_its_short_axis`,
`a_stem_offset_from_itself_grows_by_the_offset`), with rendered SVG/PNG for
each case in `david-fonty-lispy/scratch/overlap-cases/` — the contour count is
in each filename.

## Suggested acceptance

- `simplify` never returns an empty path for a non-empty input with area.
- The repro example's assert passes.
- A contour whose closing segment crosses its body simplifies into
  non-overlapping contours covering the same filled area (compare rasterized
  ink before and after; it should be unchanged or larger, never zero).
