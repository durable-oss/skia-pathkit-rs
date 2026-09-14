# 01 — Seven pathops files never compile

**Independent.** Do before anything else; it changes what the compiler can
check for every later item.

## Problem

These exist under `src/pathops/` but are not declared in `src/pathops/mod.rs`,
so `rustc` never sees them. Nothing in them is type-checked, and the
`cargo clippy` / `missing_docs` sweep skips them entirely:

```
SkAddIntersections.rs        SkDCubicToQuads.rs        SkPathOpsCurve.rs
SkDCubicLineIntersection.rs  SkDQuadLineIntersection.rs SkPathOpsOp.rs
SkIntersections.rs
```

`mod.rs` declares only the snake_case modules. These CamelCase files were
early ports superseded by a rename that never finished.

## Task

For each file, decide and act:

1. **Supersededom** — a snake_case module already covers it. Delete the file.
2. **Still needed** — rename to snake_case, add `pub mod` to `mod.rs`, fix
   whatever breaks.

Do them one at a time; each is its own commit. Expect real breakage on the
first `pub mod` — these have not been compiled since they were written.

Known overlaps to check first:
- `SkPathOpsCurve.rs` (1057 lines) vs `sk_path_ops_{line,quad,conic,cubic}.rs`
- `SkIntersections.rs` vs `sk_intersection_helper.rs`
- `SkPathOpsOp.rs` vs `pathops/mod.rs::op` + `boolean.rs`

## Acceptance

- No file under `src/pathops/` is absent from `mod.rs`.
- `cargo build` and `cargo test` pass.
- This check is empty:
  `for f in src/pathops/*.rs; do n=$(basename $f .rs); [ "$n" = mod ] || grep -q "mod $n;" src/pathops/mod.rs || echo "ORPHAN $n"; done`

---

## Progress (2026-09-10)

Five of the seven are wired in, renamed to snake_case:

| was | now | notes |
|---|---|---|
| `SkIntersections.rs` | `sk_intersections.rs` | compiles as-is |
| `SkPathOpsCurve.rs` | `sk_path_ops_curve.rs` | needed `AddAssign<SkDVector> for SkDPoint` |
| `SkDQuadLineIntersection.rs` | `sk_d_quad_line_intersection.rs` | had a stale `use SkIntersections::…` and a local `impl SkIntersections` re-declaring `remove_one`/`flip`; removed the duplicates, kept `has_t`/`has_opposite_t` |
| `SkDCubicLineIntersection.rs` | `sk_d_cubic_line_intersection.rs` | compiles as-is |
| `SkDCubicToQuads.rs` | `sk_d_cubic_to_quads.rs` | compiles as-is |

Compiling them exposed four wrong test expectations, which had never run.
All four were the tests, not the code — each implementation matches the C++:

- `sk_intersections::test_merg` expected `t(1,0) == 0.75`. `merge` copies
  `b.fT[0][bIndex]` into `fT[1][0]`, a line-for-line match with
  `SkIntersections.cpp:132`, so the value is 0.5.
- `sk_d_cubic_line_intersection::test_cubic_extrema` asserted `count > 0` for
  an arch whose **x** runs 0→3 monotonically. `find_extrema` is x-only, so 0 is
  right. Split into two tests, one for each case.
- `sk_d_quad_line_intersection::test_quad_eval` expected y = 0.75 at t = 0.5.
  B(1/2) = p0/4 + p1/2 + p2/4 = 0.5.
- `sk_path_ops_curve::test_skdcubic` expected y = 0.5 at t = 0.5.
  B(1/2) = (p0 + 3p1 + 3p2 + p3)/8 = 0.75.

### Still orphaned, and why

- **`sk_add_intersections.rs`** — renamed and its stale CamelCase paths and
  duplicate `SegmentType` fixed, but it calls `SkIntersections::quad_horizontal`
  / `conic_horizontal` / `cubic_horizontal`, which do not exist. Those are item
  08's pipeline. Wire it in when they land; the file is otherwise ready.
- **`SkPathOpsOp.rs`** — this is the *real* engine's `Op`, item 09's target. It
  needs `SkOpContourHead`, `SkPathWriter`, `SkOpGlobalState` re-exported from
  `pathops`, which the 02-08 chain provides. Not a duplicate of
  `mod.rs::op`; leave it until 09.

So the acceptance check is not yet empty. It will be after items 08 and 09.

### Real bugs found by compiling these

Three defects in the code itself, not just in tests. None could have been
caught before, because none of it was compiled:

1. **Infinite loop** in `sk_d_quad_line_intersection::check_coincident`. The
   C++ takes `last = used() - 1` and decrements it after each `removeOne`; the
   port took `used()` and never decremented. On the removal branch `index` does
   not advance either, so a horizontal line crossing a quad twice hung forever.
   `test_quad_line_intersect` sat there until killed.

2. **Wrong cubic extrema coefficients** in
   `sk_d_cubic_line_intersection::find_extrema`. It had
   `b = 2(p2 - p1) - (p1 - p0)`; `SkDCubic::FindExtrema`
   (`SkPathOpsCubic.cpp:544`) has `B = 2(a - 2b + c)`. The discriminant came out
   negative for a cubic that genuinely turns, so extrema were silently missed.

3. **Inconsistently scaled coefficients** in
   `sk_path_ops_curve::SkDCubic::find_extrema`: `a` was missing the factor of 3
   that `b` and `c` carried, so the quadratic was not the derivative of
   anything. It also required `det > 0` and `a != 0`, dropping both the
   repeated-root case and the linear case where the cubic's control points make
   the quadratic degenerate. Rewritten against the same C++ formula, with the
   linear branch handled.

`monotonic_in_x` / `monotonic_in_y` are built on (3), so they were answering
incorrectly for any cubic whose extrema the broken formula missed.

Two more, found by clippy once the files compiled:

4. **`SkIntersections::remove_one` did not always decrement.** C++ is
   `int remaining = --fUsed - index;` — the count drops first, then the early
   return. The port computed `used() - index - 1` and returned before
   decrementing, so removing the *last* entry did nothing at all. This is the
   direct cause of bug (1): `check_coincident` asked for a removal, got none,
   and looped. It also never shifted the `fIsCoincident` bitmask, so coincidence
   flags stayed attached to the wrong entries after any removal.

5. **`SkDConic::sub_divide` was not a subdivision.** It ran de Casteljau on the
   projected points, ignoring the weight, and emitted `[r0, r1, r1]` with the
   control point duplicated as the endpoint. `SkDConic::subDivide`
   (`SkPathOpsConic.cpp:125`) evaluates both ends in homogeneous form and
   recovers the control point from the midpoint. Replaced with the real port,
   plus `conic_eval_numerator` / `conic_eval_denominator`. Tests check that the
   sub-conic lies on the original arc and that subdividing over `0..1` returns
   the original.

---

## Closed (2026-09-14)

The acceptance check is empty:

```
$ for f in src/pathops/*.rs; do n=$(basename $f .rs); [ "$n" = mod ] || \
    grep -q "mod $n;" src/pathops/mod.rs || echo "ORPHAN $n"; done
(no output)
```

Both remaining orphans turned out to be superseded rather than needed:

- **`SkPathOpsOp.rs`** — its `Op`/`bridgeOp` is now `sk_op_engine.rs`, built
  on the arena. The one thing worth keeping was `gOpInverse`/`gOutInverse`,
  the inverse-fill tables; those are ported into `sk_op_engine::
  resolve_inverse`. File deleted.
- **`sk_add_intersections.rs`** — the intersection pass on the old contour
  model, replaced by `sk_op_engine::add_intersect_ts` on the arena. It was
  blocked on `SkIntersections::{quad,conic,cubic}_horizontal`, which never
  needed to exist: the arena pass finds crossings by bracketing sign changes
  and refining, and detects collinear coincidence separately. File deleted.
