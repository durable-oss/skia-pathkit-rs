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
