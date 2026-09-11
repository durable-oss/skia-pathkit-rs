# `tight_bounds` does not collapse a quad whose curve is a single point

Found: 2026-09-11, while porting Skia's `tests/PathOpsTightBoundsTest.cpp`.

Low severity. Every other case in that file ports and passes; this one is a
sub-ULP difference, recorded so the gap is known rather than rediscovered.

## Symptom

`DEF_TEST(PathOpsTightBoundsTiny)` builds a quad that starts and ends at the
same point, with a control point one ULP away:

```cpp
SkPath path = SkPathBuilder().moveTo(1, 1).quadTo(1.000001f, 1, 1, 1).detach();
```

Upstream asserts the tight bounds collapse to the degenerate rect `{1, 1, 1, 1}`
and differ from the loose bounds. The port returns `right = 1.0000005`:

| | left | top | right | bottom |
|---|---|---|---|---|
| `path.bounds()` | 1.0 | 1.0 | 1.000001 | 1.0 |
| `tight_bounds` (port) | 1.0 | 1.0 | **1.0000005** | 1.0 |
| upstream expectation | 1.0 | 1.0 | **1.0** | 1.0 |

1.0000005 is the curve's true maximum in x — the quad's apex at t=0.5, halfway
to the control point — so the port is computing the extremum correctly. Upstream
gets 1.0 because it discards the curve as degenerate before measuring it.

## Cause

`tight_bounds` (`src/pathops/sk_path_ops_tight_bounds.rs:113`) has no
degeneracy check. It routes on `is_well_behaved` and otherwise measures extrema:

```rust
if is_well_behaved(path) {
    Some(path.bounds())
} else {
    compute_tight_bounds_full(path, move_bounds)
}
```

For this quad, x runs 1 → 1.000001 → 1, so `between` fails and it takes the
extrema path, which faithfully reports the apex.

Upstream reaches `{1,1,1,1}` through the pathops engine: the edge builder
reduces a curve whose endpoints coincide to a point, and the contour
contributes only that point. `SkReduceOrder` is what performs that collapse, and
`sk_reduce_order::reduce_quad` already exists in this port — it simply is not
consulted here.

## Fix

Run each curve through `sk_reduce_order::reduce_quad` (and the cubic/conic
equivalents) before measuring it, and contribute only the reduced points when
the result is a point or a line. That matches how upstream arrives at the
degenerate rect, and is the same machinery the rest of the engine uses.

A narrower alternative — special-casing "endpoints equal" in
`compute_tight_bounds_full` — would close this case but leave the general
reduction gap open.

## Blast radius

Limited to curves that are geometrically degenerate but not point-identical in
their control hull. Any such curve currently reports a bounding box up to the
control point's excursion rather than collapsing. In practice these arise from
simplification and stroking of near-zero-length segments.

## Acceptance

- The upstream `Tiny` case yields exactly `{1, 1, 1, 1}` and differs from
  `path.bounds()`.
- The other `PathOpsTightBoundsTest.cpp` one-offs keep passing — they are
  covered in `sk_path_ops_tight_bounds.rs` as `upstream_tight_bounds_*`, and
  `Tiny` joins them when this is fixed.
