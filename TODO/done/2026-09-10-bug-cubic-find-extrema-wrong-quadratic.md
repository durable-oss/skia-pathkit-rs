# Both cubic `find_extrema` implementations solve the wrong quadratic

Found: 2026-09-10, while wiring the orphaned modules (item 01).
Fixed: `f3fd3c9`.

## Symptom

Extrema were silently missed. A cubic whose x genuinely doubles back reported
zero extrema, because the discriminant came out negative.

Concretely, for the cubic `(0,0) (3,1) (-1,2) (2,3)` — x runs 0 → 3 → -1 → 2, so
`dx/dt` changes sign twice:

| | A | B | C | discriminant |
|---|---|---|---|---|
| as written | 14 | -11 | 3 | **-47** → no roots |
| correct | 14 | -14 | 3 | 28 → roots at 0.311 and 0.689 |

## Cause

Two separate implementations, each wrong in its own way.

**`sk_d_cubic_line_intersection.rs`** had the wrong `B`:

```rust
let b = 2.0 * (points[2].x - points[1].x) - (points[1].x - points[0].x);
```

`SkDCubic::FindExtrema` (`SkPathOpsCubic.cpp:544`) is `B = 2(a - 2b + c)`. The
port is missing the factor of 2 on the second term, giving `2p2 - 3p1 + p0`
instead of `2p0 - 4p1 + 2p2`.

**`sk_path_ops_curve.rs`** had `a` missing the factor of 3 that `b` and `c`
carried:

```rust
let a = points[3] - 3.0 * points[2] + 3.0 * points[1] - points[0];
let b = 3.0 * points[2] - 6.0 * points[1] + 3.0 * points[0];
let c = 3.0 * points[1] - 3.0 * points[0];
```

The three coefficients must share a scale factor or the quadratic is not the
derivative of anything. It also required `det > 0.0 && a.abs() > 1e-10`, which
drops two real cases: a repeated root (tangency), and the linear case where
`a == 0` because the control points make the quadratic degenerate.

## Blast radius

`monotonic_in_x` and `monotonic_in_y` in `sk_path_ops_curve.rs` are built on the
second one, so they answered incorrectly for any cubic whose extrema the broken
formula missed.

Checked and **not** affected: `SkDQuad::find_extrema`
(`sk_path_ops_quad.rs:300`) and `SkDCubic::find_extrema`
(`sk_path_ops_cubic.rs:374`) are separate implementations and both match the C++.
`sk_path_ops_rect.rs`, the main live caller, goes through the quad one.

## Why it went unnoticed

Both files were orphaned — present under `src/pathops/` but absent from
`mod.rs`, so neither compiled. See `01-wire-orphaned-modules.md`.

The existing test made it worse rather than catching it:
`test_cubic_extrema` asserted `count > 0` for an arch whose x is *monotonic*, so
the correct answer is 0. It was written expecting y-extrema from an x-only
function. Fixing the formula made that test fail, which is how the test itself
turned out to be wrong.

## Fix

Both now use the C++ coefficients:

```
A = d - a + 3(b - c)
B = 2(a - 2b + c)
C = b - a
```

and `sk_path_ops_curve`'s version handles the linear branch (`a == 0`) and a
repeated root explicitly rather than dropping them.

## Regression cover

- `test_cubic_extrema_none_when_x_is_monotonic` — the arch, asserting 0, with a
  comment saying why.
- `test_cubic_extrema_finds_the_x_turn` — the doubling-back cubic above,
  asserting it finds an extremum in range.
- `test_skdcubic` — checks `B(1/2) = (p0 + 3p1 + 3p2 + p3)/8` and that the
  y-extremum of a symmetric arch is at t = 0.5, which the old code returned 0
  results for.
