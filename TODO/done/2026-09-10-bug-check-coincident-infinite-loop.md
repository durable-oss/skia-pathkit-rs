# `check_coincident` never terminates; a horizontal line crossing a quad hangs

Found: 2026-09-10, while wiring the orphaned modules (item 01).
Fixed: `f3fd3c9`.

## Symptom

`cargo test` hung. The test runner reported:

```
test pathops::sk_d_quad_line_intersection::tests::test_quad_line_intersect
    has been running for over 60 seconds
```

and never got further. The test is unremarkable — a horizontal line through a
quad, which crosses it twice:

```rust
let quad = DQuad::new(Point::new(0.0, 0.0), Point::new(1.0, 1.0), Point::new(2.0, 0.0));
let line = DLine::new(Point::new(0.0, 0.5), Point::new(2.0, 0.5));
intersect(&quad, &line, true);   // hangs
```

## Cause

`LineQuadraticIntersections::check_coincident` in
`src/pathops/sk_d_quad_line_intersection.rs`. Two departures from
`SkDQuadLineIntersection.cpp:117`, which together make the loop unable to end:

```rust
let last = self.intersections.used();          // C++: used() - 1
let mut index = 0;
while index < last {
    ...
    if self.intersections.is_coincident(index) {
        self.intersections.remove_one(index);  // C++ also does --last
    } else if ... {
        self.intersections.remove_one(index + 1);
    } else {
        self.intersections.set_coincident(index);
        index += 1;                            // only this branch advances
    }
}
```

- `last` was `used()` rather than `used() - 1`. The body reads `index + 1`, so
  the walk also overruns by one.
- C++ decrements `last` after each `removeOne`; the port did not.

On the removal branches `index` does not advance and `last` does not shrink, so
once the walk reaches a coincident pair it revisits the same index forever.

The removal branch needs an actual removal to be entered, which is why this only
showed up on a line that genuinely crosses twice.

## Why it went unnoticed

`SkDQuadLineIntersection.rs` was one of seven files that existed under
`src/pathops/` but were absent from `mod.rs`. `rustc` never compiled it and its
tests never ran. See `01-wire-orphaned-modules.md`.

## Fix

`last` is `used() - 1`, held as `isize`, and decremented on each removal —
matching the C++ line for line.

This bug also has a second half: `SkIntersections::remove_one` did not always
decrement its count, so the removal branch sometimes removed nothing at all.
That is
`2026-09-10-bug-remove-one-skips-the-last-entry.md`, and both had to be fixed
before the loop terminated.

## Regression cover

`pathops::sk_d_quad_line_intersection::tests::test_quad_line_intersect` now
passes rather than hanging. A hang is not a great regression test — it fails by
timing out rather than by asserting — but the `remove_one` tests listed in the
companion file cover the underlying mechanism directly.
