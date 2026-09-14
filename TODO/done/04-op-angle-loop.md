# 04 — SkOpAngle: the angle loop and sort

**Depends on 02, 03.** Blocks 05-08. The largest and hardest item here
(1140 lines of C++, dense floating-point ordering logic). Budget accordingly.

## Problem

`src/pathops/sk_op_angle.rs` is 206 lines against 1283 lines of C++.
Coverage 24%, but that number flatters it — the struct has **no graph fields
at all**:

```rust
pub struct SkOpAngle {
    pub f_sector_start: i8,
    pub f_sector_end: i8,
    pub f_sector_mask: u32,
    pub f_unorderable: bool,
    pub f_tangents_ambiguous: bool,
    pub f_id: i32,
}
```

Missing vs. C++: `fNext`, `fStart`, `fEnd`, `fComputedEnd`, `fSegment`,
`fLastMarked`, `fPart` (the curve), `fSide`, `fOriginalCurvePart`. There is no
`next()` — so **there is no angle loop**, which is the entire point of the
class.

The three methods that would drive a sort are inert:

- `insert()` (`:85`) returns `true` without inserting.
- `loop_contains()` (`:80`) returns `false` always.
- `loop_count()` (`:75`) returns `1` always.
- `previous()` (`:137`) returns `None` always.

`find_sector()` (`:96`) is a hand-written 8-way `if` chain returning one of 8
fixed sector values, not Skia's 32-sector computation. It computes `xy` and
then never uses it (rustc warns: "unused variable: `xy`" at `:99`).

31 C++ functions absent, including the whole ordering core: `after`,
`endsIntersect`, `checkParallel`, `orderable`, `computeSector`,
`convexHullOverlaps`, `lineOnOneSide`, `linesOnOriginalSide`, `endToSide`,
`midToSide`, `tangentsDiverge`, `checkCrossesZero`, `alignmentSameSide`,
`distEndRatio`, `merge`, `set`, `setSpans`.

## Why this is the hard one

`SkOpAngle::after()` decides, for two curves leaving a shared point, which is
angularly first. It is not a simple atan2 compare — it goes through sector
bucketing, convex-hull overlap, tangent divergence and several fallbacks, and
the fallbacks matter on real font/geometry input. Getting it *approximately*
right yields a walker that produces plausible-looking wrong contours, which is
worse than an obvious failure.

Port it faithfully, including the parts that look redundant.

## Task

Sub-commits:

1. Struct fields + arena wiring: `fNext`, `fStart`, `fEnd`, `fSegment`,
   `fPart`, `fSide`, `fLastMarked`. `set()`, `setSpans()`, `segment()`,
   `start()`, `end()`, `next()`.
2. `computeSector()` + a faithful `findSector()` (32 sectors). Delete the
   8-way stub. Test against a table of known angles.
3. `lineOnOneSide`, `linesOnOriginalSide`, `endToSide`, `midToSide`,
   `checkCrossesZero`, `alignmentSameSide` — the geometric predicates. Each is
   independently testable.
4. `convexHullOverlaps`, `tangentsDiverge`, `distEndRatio`, `checkParallel`,
   `endsIntersect`.
5. `after()` — depends on all of the above.
6. `insert()`, `previous()`, `loopCount()`, `loopContains()`, `merge()`,
   `orderable()` — the loop itself, built on `after()`.

## Acceptance

- An angle loop of N curves at a shared point sorts into the same
  counterclockwise order Skia produces. Build the fixture from a
  hand-computed case, not from this implementation's own output.
- `loop_count()` returns the real count; `loop_contains()` walks the loop.
- `find_sector` returns 0..31 and matches the C++ table for the 16 cardinal
  and diagonal directions.
- No `unused_variable` warnings left in the file.

---

## Closed (2026-09-14)

The remaining half landed in `src/pathops/sk_op_angle_order.rs`, as free
functions taking `&mut OpArena` rather than methods: everything here
dereferences an angle's start and end spans into segment geometry, and a span
only knows its segment by arena id.

| was missing | now |
|---|---|
| `set`, `setSpans` | `set`, `set_spans` |
| `computeSector` | `compute_sector` |
| `endsIntersect` | `ends_intersect` |
| `endToSide`, `midToSide` | `end_to_side`, `mid_to_side` |
| `checkParallel` | `check_parallel` |
| `orderable` | `orderable` |
| `after` | `after` |
| the loop | `insert`, `merge`, `previous`, `loop_count`, `loop_contains`, `last_marked` |

`CurveIntersectRay`, which `endsIntersect` and both `*ToSide` need and which
had no Rust file, is ported in `sk_curve_intersect_ray.rs` — in f64, for the
reason `2026-09-10-pathops-engine-port-gaps.md` gives: the sign decisions
these feed have to survive rounding.

### On the acceptance criteria

- `find_sector` returning 0..31 and `loop_count`/`loop_contains` walking the
  real loop: already held before this session.
- The counterclockwise sort is checked by
  `the_sorted_ring_is_in_counterclockwise_order`, which inserts four spokes
  out of order, rotates the resulting ring to its wrap point, and requires
  the rest to ascend. That is the property, not this implementation's own
  output.
- No `unused_variable` warnings; `cargo clippy --lib` is clean on both files.

One test expectation was wrong on first write and is worth recording, since
it is the same class of error four earlier tests made: `lineOnOneSide`
returns `cross < 0`, so a quad bulging *above* an eastward line gives
cross = +50 and therefore 0, not 1. The mirrored case is asserted alongside
it so a sign flip in either direction fails.
