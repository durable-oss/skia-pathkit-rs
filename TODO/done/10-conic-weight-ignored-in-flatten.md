# 10 — Conic weight ignored when flattening; every boolean op on curves returns empty

**Independent, small, live bug.** Highest value-per-line item in this set.

## Problem

`boolean.rs` and `sk_path_ops_simplify.rs` both flatten a conic by discarding
its weight and treating the three points as a quadratic:

- `src/pathops/boolean.rs:62`
- `src/pathops/sk_path_ops_simplify.rs:302` (and `:250` in `contour_points`)

```rust
Verb::Quad | Verb::Conic => {
    flatten_quad(pts[0], pts[1], pts[2], MAX_FLAT_DEPTH, &mut segs);
    last = pts[2];
}
```

A conic with w != 1 traces a different curve than the quadratic through the
same three points. `Path::contains` evaluates the conic correctly, so the
flattened boundary and the fill test disagree. The perpendicular sample points
in `classify_edge` / `collect_boundary` then land on the wrong side, every edge
is rejected as "same on both sides", and the result is empty.

`add_circle` emits conics with w = sqrt(2)/2, so this hits every circle, oval
and rounded rect.

## Repro

```rust
let mut a = Path::new(); a.add_circle(200.0, 200.0, 40.0);
let mut b = Path::new(); b.add_circle(205.0, 200.0, 40.0);
assert!(!op(&a, &b, PathOp::Union).unwrap().is_empty());  // fails: empty
```

Measured 2026-09-10 at `3f6aa61`, all offsets and both radii:

| shape | offset | contours |
|---|---|---|
| rect ∪ rect | any | 1 — correct |
| disc ∪ disc | 0 | 1 (only because `mod.rs` short-circuits identical paths) |
| disc ∪ disc | 0.5, 1, 5, 20, 60 | **0 — empty** |
| disc ∩ disc | 5 | **0 — empty** |
| disc ∪ rect | — | **0 — empty** |

This is more severe than the older TODO
(`2026-09-10-simplify-returns-empty-on-self-crossing-closing-segment.md`)
recorded — that one reported 2 contours instead of 1 for near-coincident discs.
It now returns nothing at all for any curved boolean.

## Fix

Add a weight-aware `flatten_conic`. Subdivide with de Casteljau on the rational
form, or evaluate at fixed t and emit chords — `Conic::chop_at` in
`src/core/sk_geometry.rs` already does the rational split correctly and is
tested. Route `Verb::Conic` to it and pass `weight` through; `Path::iter()`
already yields the weight as the third tuple element, both call sites currently
bind it and drop it.

`contour_points` at `sk_path_ops_simplify.rs:250` pushes the raw control point
for the convexity test. That is defensible for a hull-based convexity check but
should be revisited once the flattener is correct.

## Acceptance

- Union, intersect and difference of two overlapping discs each return one
  non-empty contour with the expected filled area.
- Disc ∪ rect returns a single contour.
- A test asserts flattened-conic points lie within tolerance of
  `Conic::eval_at`, so a quad-vs-conic regression is caught directly rather
  than as a mysterious empty result.
- The existing 617 tests still pass.

---

## Resolution (2026-09-10)

Fixed. `flatten_conic` in both `boolean.rs` and `sk_path_ops_simplify.rs`
subdivides with `Conic::chop`, which splits in the rational form, so both
halves stay on the true curve. `Verb::Conic` no longer shares the `Verb::Quad`
arm, and the weight from `Path::iter()` is passed through instead of dropped.
`Conic` is now re-exported from `core`.

Tests in `boolean.rs`: `flatten_conic_stays_on_the_true_curve` (every emitted
point within tolerance of `Conic::eval_at`, which is the regression guard this
file asked for), `flatten_conic_differs_from_flatten_quad_when_weighted`,
`flatten_conic_with_unit_weight_matches_a_quad`,
`boolean_ops_on_discs_are_not_empty`, `union_of_a_disc_and_a_rect_is_not_empty`.

Disc ∪ rect and all three disc booleans return non-empty. Note that fixing this
alone left discs empty at small offsets; that turned out to be item 15, and
both had to land before curved booleans worked across the range.
