# Four tests asserted the wrong answer; in every case the test was wrong, not the code

Found: 2026-09-10, on first running the orphaned modules' tests (item 01).
Fixed: `f3fd3c9`.

## Why this is worth its own file

These are not code defects — each implementation matched the C++. They are
recorded because a wrong test is worse than a missing one: it will be trusted,
and the next person to touch the code will "fix" working code to satisfy it.

All four had never executed, because their files were absent from `mod.rs`. See
`01-wire-orphaned-modules.md`.

## The four

### 1. `sk_intersections::test_merg` — expected `t(1,0) == 0.75`

```rust
b.insert(0.5, 0.75, Point::new(1.0, 1.5));
ts.merge(&a, 0, &b, 0);
assert!((ts.t(1, 0) - 0.75).abs() < 1e-6);   // wrong
```

`merge` copies `b.fT[0][bIndex]` into `fT[1][0]` — a line-for-line match with
`SkIntersections.cpp:132`. It pairs the *first* curve's t from each side and does
not carry `b`'s second t across. `b.insert(0.5, 0.75, ..)` stores `f_t[0] = 0.5`,
so the answer is **0.5**.

Corrected, with a comment stating the pairing rule so the number is not mistaken
for arbitrary.

### 2. `sk_d_cubic_line_intersection::test_cubic_extrema` — expected `count > 0`

The fixture is an arch, `(0,0) (1,3) (2,3) (3,0)`. Its **x** runs 0 → 3
monotonically, and `find_extrema` is x-only, so the correct answer is 0. The test
was written expecting y-extrema.

Split into two tests: one asserting 0 for the monotonic case with a comment
saying why, one using a cubic whose x actually doubles back. Note this test was
actively harmful — it *passed* against the broken coefficients documented in
`2026-09-10-bug-cubic-find-extrema-wrong-quadratic.md`, and failed once they were
fixed.

### 3. `sk_d_quad_line_intersection::test_quad_eval` — expected `y = 0.75` at `t = 0.5`

For `(0,0) (1,1) (2,0)`: `B(1/2) = p0/4 + p1/2 + p2/4 = 0 + 1/2 + 0` = **0.5**.

### 4. `sk_path_ops_curve::test_skdcubic` — expected `y = 0.5` at `t = 0.5`

For control y values `(0, 1, 1, 0)`: `B(1/2) = (p0 + 3p1 + 3p2 + p3)/8 =
(0 + 3 + 3 + 0)/8` = **0.75**.

The same test's extremum assertion (`t = 0.5`) was correct, and was the one
failing legitimately against the broken `find_extrema`.

## Method

Each expected value was recomputed from the Bernstein form or checked against the
C++ source before changing the assertion, not adjusted to match whatever the
implementation printed. The corrected tests carry the arithmetic in a comment so
the next reader can check it without redoing the algebra.
