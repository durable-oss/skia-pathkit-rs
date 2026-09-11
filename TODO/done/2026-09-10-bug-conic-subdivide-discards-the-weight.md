# `SkDConic::sub_divide` is not a subdivision: weight discarded, control point duplicated

Found: 2026-09-10, by clippy once the orphaned modules compiled (item 01).
Fixed: `f149261`.

## Symptom

None visible, which is the worrying part. `sub_divide` returned a `SkDConic`
that was neither the requested piece of the curve nor a valid conic, and nothing
checked it. Clippy pointed at two unused locals (`w0_new`, `w2_new`), which is
what led to reading the function.

## Cause

`src/pathops/sk_path_ops_curve.rs`. The port ran de Casteljau on the *projected*
points, ignoring the weight, and then emitted the control point twice:

```rust
let w0 = 1.0;
let w1 = (w + t1) / 2.0;
let w2 = (1.0 + t1) / 2.0;
let r0 = q0 + (q1 - q0) * ((t2 - t1) / (1.0 - t1));
let r1 = q1 + (q2 - q1) * ((t2 - t1) / (1.0 - t1));
let w0_new = w0;                          // unused
let w1_new = (w0 + w1) / 2.0;
let w2_new = (w1 + w2) / 2.0;             // unused
SkDConic {
    fPts: SkDQuad::from_points([r0, r1, r1]),   // r1 as both control AND end
    fWeight: w1_new,
}
```

Three things wrong at once:

- A conic is a **rational** quadratic. Interpolating the projected points drops
  the weight and gives a curve that is not the original arc.
- `[r0, r1, r1]` uses the control point as the end point, so the piece does not
  even reach where it should.
- Two of the three computed weights are thrown away.

`SkDConic::subDivide` (`SkPathOpsConic.cpp:125`) evaluates both ends in
homogeneous form and recovers the control point from the midpoint:

```cpp
double bx = 2 * dx - (ax + cx) / 2;
...
PkDoubleToScalar(bz / sqrt(az * cz))      // the new weight
```

## Why it went unnoticed

`SkPathOpsCurve.rs` was orphaned — present under `src/pathops/` but absent from
`mod.rs`, so never compiled and never tested. See `01-wire-orphaned-modules.md`.

## Fix

Replaced with the real port, plus the two helpers it needs
(`conic_eval_numerator`, `conic_eval_denominator`).

Note this is a *different* conic-weight bug from
`done/10-conic-weight-ignored-in-flatten.md`, which was about flattening for the
substitute boolean engine. Same root misconception — that a conic can be treated
as a quad — in two unrelated places.

## Regression cover

In `sk_path_ops_curve.rs`:

- `conic_sub_divide_stays_on_the_curve` — samples the sub-conic at eleven points
  and asserts each lies on the original quarter arc, which the old code could
  not satisfy.
- `conic_sub_divide_over_the_whole_range_is_the_original` — subdividing `0..1`
  returns the input, control point and weight included.
- `conic_eval_helpers_match_the_rational_form` — the numerator is the first and
  last coordinate at the ends, and a weight of 1 makes the denominator 1
  everywhere.
