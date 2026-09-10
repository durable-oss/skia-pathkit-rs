# Port status: the pathops engine (index)

Audited 2026-09-10 against `3f6aa61`. This is the index for the pathops port;
each numbered item below has its own TODO file with the detail.

## How this was measured

- `cargo`-level: 7 files under `src/pathops/` are not declared in `mod.rs` and
  therefore never compile (see item 01).
- Function coverage: C++ function names per translation unit vs. Rust `fn`
  names in the counterpart file, snake_case-normalized. This over-counts (a
  name can match while the body is a stub), so treat the percentages as an
  upper bound.
- Stub detection: functions whose entire body is a single trivial expression
  (`true`, `false`, `None`, `0`, a bare field read). 79 found; most are
  legitimate accessors, the rest are listed per-item.

| C++ unit | Rust file | fn coverage (upper bound) | state |
|---|---|---|---|
| SkOpCoincidence | `sk_op_coincidence.rs` | 10% | skeleton, 5 stub methods |
| SkOpAngle | `sk_op_angle.rs` | 24% | no angle loop at all |
| SkOpSpan | `sk_op_span.rs` | 19% | fields only, no linkage |
| SkOpSegment | `sk_op_segment.rs` | 31% | geometry ok, graph absent |
| SkPathOpsDebug | `sk_path_ops_debug.rs` | 8% | mostly absent |
| SkPathOpsCommon | `sk_path_ops_common.rs` | 55% | 4 key fns absent |
| SkOpEdgeBuilder | `sk_op_edge_builder.rs` | 41% | partial |
| SkOpContour | `sk_op_contour.rs` | 57% | partial |
| SkIntersections | `SkIntersections.rs` | 44% | orphaned |
| SkAddIntersections | `SkAddIntersections.rs` | 0% | orphaned |
| SkPathWriter | `sk_path_writer.rs` | 74% | closest to done |
| SkOpBuilder | — | 0% | no Rust file |
| SkDLineIntersection | — | 0% | no Rust file |
| SkDConicLineIntersection | — | 0% | no Rust file |
| SkOpCubicHull | — | 0% | no Rust file |
| SkLineParameters | — | 0% | no Rust file |

## The dependency that orders everything

`SkOpSpan`, `SkOpSegment`, `SkOpAngle` and `SkOpCoincidence` are a cyclic
object graph in C++ (raw pointers both ways, arena-allocated, freely mutated
during the walk). The Rust structs already declare the fields for this
(`f_next`, `f_prev`, `f_coincident`, `f_to_angle`, `f_span`, …) typed as
`Option<usize>` and commented `// arena index` — but **no arena exists**, and
nothing anywhere assigns those fields. They are inert.

So the ordering is forced:

```
02 arena  ->  03 span linkage  ->  04 angle loop  ->  05 segment winding
          ->  06 coincidence   ->  07 common fns  ->  08 bridge/simplify
```

Items 02-08 are sequential. Items 01, and 09-14 are independent and can be
done in any order or in parallel.

## Items

- `01-wire-orphaned-modules.md` — 7 files that never compile. Independent, do first.
- `02-op-global-state-arena.md` — the arena. Blocks 03-08.
- `03-op-span-linkage.md` — span/PtT lists and traversal.
- `04-op-angle-loop.md` — angle sort; the largest single item.
- `05-op-segment-winding.md` — winding computation and marking.
- `06-op-coincidence.md` — the requested `SkOpCoincidence` port.
- `07-path-ops-common-fns.md` — the requested `FindSortableTop`/`FindChase`/`HandleCoincidence`.
- `08-bridge-winding-xor.md` — rewrite simplify/op on the real engine.
- `09-bridge-winding-xor.md` — rewrite simplify/op on the real engine. The payoff.

Independent, can start immediately:

- `10-conic-weight-ignored-in-flatten.md` — **live bug**, small, high value.
  Every boolean op involving a conic returns empty.
- `15-union-assemble-drops-contours.md` — **live bug**, separate from 10.
  Confirmed with pure polygons: `assemble` discards whole contours when edges
  are short, so union of two 64-gons returns empty.
- `11-op-builder.md` — no Rust file (263 lines C++).
- `12-d-line-intersection.md` — no Rust file (344 lines C++). Wanted by 09.
- `13-d-conic-line-intersection.md` — no Rust file (366 lines C++). The quad
  and cubic equivalents exist; the conic one was never written.

## Closed during this audit (in `done/`)

- `14-op-cubic-hull.md` — **already ported**, found in
  `sk_path_ops_cubic.rs` rather than a file of its own. No work needed.
- `15-line-parameters.md` — landed while the audit was running
  (`sk_line_parameters.rs`, 449 lines with tests).
- `2026-09-10-simplify-returns-empty...` — the reported repro no longer
  fails; fixed by `3f6aa61`'s `Path::contains` curve repairs. The union bug
  in its "Related" section is *not* fixed and is split into items 10 and 15.

## Caveat on the coverage numbers

0% for a module means "no Rust file of that name", **not** necessarily "not
ported" — item 14 was already complete inside a sibling module. Check whether
the code landed elsewhere before starting items 11, 12 or 13.
