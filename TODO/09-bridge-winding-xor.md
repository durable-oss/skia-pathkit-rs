# 09 — Rewrite simplify and op on the real engine (bridgeWinding / bridgeXor)

**Depends on 02-08.** The payoff item — everything above exists to make this
possible.

## Current state

`src/pathops/sk_path_ops_simplify.rs` implements Skia's `SimplifyDebug`
control flow (convex fast path, build, intersect, classify, bridge, assemble)
but on a **flattened-edge substitute engine**, because the op-segment engine
did not exist when it was written. The module docs say so explicitly.

Consequences of the substitute:

- Curves are flattened to line segments, so output is always polylines. Skia
  preserves the original quad/conic/cubic verbs via `addCurveTo`.
- `collect_boundary` decides membership by sampling `Path::contains` at a
  point offset perpendicular to each edge. That is a point-in-polygon test per
  edge, not winding-sum bookkeeping — it is O(edges x path size) and gets the
  wrong answer where the offset lands across a nearby edge.
- `dedup_coincident` is a hand-rolled stand-in for `HandleCoincidence`.
- `is_convex` is hand-rolled because `Path::is_convex()` does not exist.

`src/pathops/boolean.rs` is the same substitute engine for `op`.

## Task

Once 02-08 land, replace both with faithful ports:

1. `bridgeWinding` and `bridgeXor` from `SkPathOpsSimplify.cpp` — the real
   ones, walking the segment graph via `findNextWinding`/`findNextXor` and
   emitting through `addCurveTo`.
2. `SimplifyDebug` proper: `SkOpEdgeBuilder` -> `SortContourList` ->
   `AddIntersectTs` -> `HandleCoincidence` -> bridge -> `assemble`.
3. `OpDebug` from `SkPathOpsOp.cpp` for `op`, replacing `boolean.rs`.
4. Delete the substitute engine and its helpers once the tests pass on the
   real one.

Keep the existing 19 simplify tests — they encode correct behaviour and
should pass unchanged against the real engine. If one fails, decide which is
wrong before changing it.

## Acceptance

- Output preserves curve verbs: simplifying a path containing a cubic yields
  a path containing a cubic, not 200 line segments.
- All existing pathops tests pass.
- `examples/simplify_empty_repro.rs` passes.
- The disc-union cases from item 10 pass.
- `boolean.rs` and the substitute helpers in `sk_path_ops_simplify.rs` are
  gone.

---

## Progress (2026-09-14): the engine exists and keeps curves, but is not wired in

`src/pathops/sk_op_engine.rs` is the real thing: the arena edge builder,
`AddIntersectTs`, collinear coincidence detection, and the `bridgeOp` /
`bridgeWinding` walks. `op_with_engine` and `simplify_with_engine` are its
entry points.

**The curve-preservation acceptance criteria pass**, against the shape from
`2026-09-14-boolean-ops-destroy-all-curves.md` (a stem of four lines plus a
ring of eight cubics, differenced against a rectangle that touches only the
stem):

- `a_contour_the_operation_never_touches_keeps_its_curves` — the result
  contains `Verb::Cubic`.
- `the_result_does_not_explode_into_a_polyline` — under 40 segments, against
  the flattening engine's 261.
- `a_cut_curve_is_subdivided_rather_than_flattened` — a disc cut by a
  rectangle comes back with cubics.
- `the_engine_unions_two_overlapping_rectangles` — one closed contour,
  correct inside/outside at four sample points.

`pathops::op` still routes to `boolean.rs`.

## What is left, precisely

Routing `op` through the engine passes 1005 of 1007 tests. The two failures
are both in `sk_op_builder`, and both reduce to one case: **two rectangles
whose top and bottom edges are collinear and overlapping**.

Traced as far as this:

1. Coincidence *is* detected there now (`record_if_coincident`), and
   `apply` folds the shared run correctly — segment 0's middle span comes out
   with `wind_value = 2`.
2. The walk still emits that span. `is_active` calls `active_op`, which calls
   `update_winding(end, start, sortable_top)`, and that returns **0** rather
   than the 2 the span carries.
3. So the interior bottom edge reads as "outside on the far side", which for
   `Union` puts it on the boundary. The result is rectangle A's outline plus
   a stray `(20,0)->(8,0)` fragment, and B is never walked at all.

The suspect is the ray-cast winding, not the gate: a ray fired from a point
on the bottom edge exits downward immediately and reads 0, which is right for
*that operand* but never picks up B's contribution into `oppSum`.
`sk_op_sortable_top::accumulate` swaps `wind`/`opp_wind` per hit on
`segment_operand`, which is the C++ shape, so the defect is probably in which
spans the ray finds rather than in the accumulation.

Start there: `find_sortable_top` on segment 0's t = 0.4 span, and check what
`accumulate` sees. `a_ray_that_runs_along_an_edge_is_retried_in_another_direction`
in `sk_op_sortable_top.rs` sets up exactly this geometry.

## Still not done from the original task list

- `bridgeXor` proper — `find_next_xor` exists, but nothing calls it;
  `simplify_with_engine` uses `bridgeWinding` for both fill rules.
- Deleting `boolean.rs` and the substitute helpers in
  `sk_path_ops_simplify.rs`. They stay until the switch lands.
- Curve/curve coincidence. `record_if_coincident` handles line/line only,
  which is the case that breaks a union of boxes; two identical curves need
  the t-section machinery.
