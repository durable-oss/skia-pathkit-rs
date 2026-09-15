# Boolean ops return polylines: every curve in the result is destroyed

**Blocked on `01-wire-orphaned-modules.md`** (which owns `SkPathOpsOp.rs`),
but filed separately: 01 is scoped as a compilation/hygiene task and does not
record what the missing wiring costs callers. This is that cost.

## Problem

`pathops::op` never returns a curve. Not a cubic, not a quad, not a conic.
Every `Union`/`Intersect`/`Difference`/`Xor` on non-trivial input comes back
as a polyline.

`pathops::op` (`src/pathops/mod.rs:70`) dispatches to `boolean::path_op`,
whose own header says what it is:

```
//! Boolean path operations via flatten, split, and inside tests.
//!
//! The full Skia pathops engine is not yet wired. This module implements
//! union/intersect/difference/xor for closed paths by flattening curves,
//! splitting edges at intersections, and keeping pieces that sit on the
//! result boundary according to [`Path::contains`].
```

The stopgap cannot preserve curves even in principle. Its only output type is

```rust
struct Edge {
    from: Point,
    to: Point,
}
```

and `assemble(&edges)` builds the result from those. Curves enter `flatten()`
at `FLAT_TOL = 0.1` and never come back.

Real Skia pathops is curve-preserving: it computes curve/curve intersections,
subdivides, and emits curves. The port of that engine exists in-tree as
`SkPathOpsOp.rs` — but it is not declared in `mod.rs`, nothing references it,
and no call path reaches it. Confirmed:

```
$ grep -rn "SkPathOpsOp\|sk_path_ops_op" src/ --include="*.rs" \
    | grep -v "^src/pathops/SkPathOpsOp.rs"
(no output)
```

So the curve-aware engine is orphaned, and the flattening stopgap is the only
implementation any caller gets.

## Measurement

From `david-fonty-lispy`, subtracting a small rectangle from a glyph built of
one stem (4 lines) plus a ring (8 curves):

```
before: 4 lines, 8 curves
after : 261 lines, 0 curves
```

The rectangle touches only the stem. The ring is flattened anyway — every
contour handed to `path_op` is flattened, whether or not the operation
reaches it.

`4 + 8 = 12` segments in, `261` out. That is a 20x growth in point count
alongside total loss of curve fidelity.

## Why it matters downstream

`david-fonty-lispy` uses `pathops::op` for two things a font pipeline cannot
do without:

- `overlap::fuse` — unioning contours meant to touch (a `u`'s stem crossing
  its bowl, a `y`'s diagonals).
- `overlap::subtract` — applying `(subtract ...)` cuts, which is how ink traps
  are drawn.

Both currently trade every bezier in the affected glyph for a polyline. In a
font that is not a cosmetic issue:

- 20x point counts inflate `glyf` and slow rasterization.
- Hinting and autohinting work against curve structure that no longer exists.
- A polyline at 261 segments is visibly faceted at display sizes.
- It makes the ink-trap feature unusable in practice, which is how this was
  found: the traps were disabled rather than shipped.

`fontmake` avoids the problem entirely by calling real `skia-pathops`
(`--overlaps-backend pathops`), which is curve-preserving. Any DFL-native
path is strictly worse until this lands.

## Task

Wire the ported engine and route `pathops::op` through it:

1. Land `01-wire-orphaned-modules.md` for `SkPathOpsOp.rs` specifically —
   rename to `sk_path_ops_op.rs`, declare it, fix what breaks.
2. Point `pathops::mod::op` at it instead of `boolean::path_op`.
3. Keep `boolean.rs` reachable as a fallback if the engine returns an error,
   so a failure degrades to a flattened result rather than an empty path.
   (`overlap::subtract` already treats a failed op as "return ink unchanged",
   but `fuse` callers may not.)
4. Update the `pathops::op` doc comment — it currently documents the
   flattening behavior as though it were the design.

Note that several open TODOs are prerequisites for the engine actually
working once wired: `06-op-coincidence.md`, `05-op-segment-winding.md`,
`09-bridge-winding-xor.md`, and `13-d-conic-line-intersection.md` (no Rust
file; conic/line intersection was never ported). See
`2026-09-10-PORT-STATUS-pathops-engine.md`.

## Acceptance

- `pathops::op` returns curves when its inputs have curves and the operation
  does not cut through them.
- A regression test asserting curve survival, e.g. the shape above:
  difference of a rect against stem+ring returns a result containing
  `Verb::Cubic` or `Verb::Quad`, and fewer than 40 total segments.
- A contour the operation does not touch comes back with its input segments
  unchanged.

---

## Progress (2026-09-14)

**The engine exists and its acceptance criteria pass.** `pathops::op` does
not yet route through it.

Items 01 and 04-08 are closed (see `TODO/done/`). The curve-preserving engine
is `src/pathops/sk_op_engine.rs`; `op_with_engine` is the entry point.

Measured against the shape this file describes — a stem of four lines plus a
ring of eight cubics, differenced against a rectangle touching only the stem:

| | before | engine |
|---|---|---|
| curves in result | 0 | present (`Verb::Cubic`) |
| segments | 261 | under 40 |

All three acceptance criteria hold, as tests in `sk_op_engine.rs`:

- `a_contour_the_operation_never_touches_keeps_its_curves`
- `the_result_does_not_explode_into_a_polyline`
- `a_cut_curve_is_subdivided_rather_than_flattened`

### Update (2026-09-15): closed

The winding bug this file was blocked on is fixed (see `09-bridge-winding-
xor.md`, "The five defects the switch turned up"). `pathops::mod::op` and
`pathops::mod::simplify` both route through `sk_op_engine` now, each with
the flattening engine kept as the fallback for inputs the real one
declines. All four acceptance criteria at the top of this file hold.

A separate, pre-existing curve-subdivision bug was found while closing
`09` (corrupts curves cut at 2+ intersection points on the same arc — see
`2026-09-15-curve-subdivision-corrupts-multi-intersection-arcs.md`). It
does not block this file: the criteria here are about curves surviving an
operation at all, which they do; that gap is about a further class of
input where the surviving curve's own geometry can still come out wrong.

### Task list status

1. ~~Land `01-wire-orphaned-modules.md` for `SkPathOpsOp.rs`~~ — done, and
   the file turned out to be superseded rather than needed. Its inverse-fill
   tables are ported into `sk_op_engine::resolve_inverse`.
2. ~~Point `pathops::mod::op` at the engine~~ — done.
3. Keep `boolean.rs` as a fallback — that is the intended shape;
   `op_with_engine` returns `None` rather than an empty path when it cannot
   resolve an input, so the caller degrades to flattening. `simplify` now
   does the same via `simplify_with_engine`.
4. ~~Update the `pathops::op` doc comment~~ — done; it now says the result is
   a polyline and why the engine is not wired in.

The prerequisites this file listed are all closed: `06-op-coincidence.md`,
`05-op-segment-winding.md`, `09`'s dependencies, and
`13-d-conic-line-intersection.md` (which was already done).
