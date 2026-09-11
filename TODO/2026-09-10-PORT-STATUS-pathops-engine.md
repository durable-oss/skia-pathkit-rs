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


---

# Progress, 2026-09-10 (second pass)

Tests: **811** lib tests, from 581 at the start. `cargo clippy --lib` is clean
on every file touched. Nothing is stubbed silently: where a dependency does not
exist yet it is a closure parameter or a documented `Not ported`, not a
hard-coded `true`.

## Closed

| item | what |
|---|---|
| 02 | the arena — `sk_op_arena.rs` |
| 03 | span/PtT linkage, all six sub-parts |
| 10 | conic weight in flatten (**was a live bug**) |
| 11 | `SkOpBuilder`, and the duplicate `OpBuilder` removed |
| 12 | `SkDLineIntersection` |
| 13 | `SkDConicLineIntersection` |
| 15 | `assemble` dropping contours (**was a live bug**) |

Also landed outside the numbered items: `SkLineParameters` (item 15 in `done/`),
`SkOpAngle`'s geometric core, and `sub_divide_curve` — the
`SkOpSegment::subDivide(…, SkDCurve*)` overload that `setSpans` needs, which
did not exist in either form.

## Bug index

Every defect found this pass has its own file. Fixed ones are in `done/`.

| bug | file | state |
|---|---|---|
| `check_coincident` never terminates | `done/2026-09-10-bug-check-coincident-infinite-loop.md` | fixed |
| `remove_one` skips the last entry, loses coincidence flags | `done/2026-09-10-bug-remove-one-skips-the-last-entry.md` | fixed |
| `SkDConic::sub_divide` discards the weight | `done/2026-09-10-bug-conic-subdivide-discards-the-weight.md` | fixed |
| both cubic `find_extrema` solve the wrong quadratic | `done/2026-09-10-bug-cubic-find-extrema-wrong-quadratic.md` | fixed |
| four tests asserted the wrong answer | `done/2026-09-10-bug-four-wrong-test-expectations.md` | fixed |
| conic weight ignored when flattening | `done/10-conic-weight-ignored-in-flatten.md` | fixed |
| `assemble` drops contours | `done/15-union-assemble-drops-contours.md` | fixed |
| quad/line `exact_point` is not an endpoint test | `done/2026-09-11-bug-quad-line-exact-point-is-not-an-endpoint-test.md` | fixed |
| near-coincident discs fragment | `16-union-of-near-coincident-discs-fragments.md` | **open** |
| `tight_bounds` does not collapse a degenerate quad | `2026-09-11-bug-tight-bounds-tiny-quad-not-collapsed.md` | **open** |

The first five were invisible because their files were absent from `mod.rs` and
so never compiled — see `01-wire-orphaned-modules.md`, which stays open for the
two orphans that remain (`sk_add_intersections`, `SkPathOpsOp`; both blocked on
items 08 and 09).

## Where item 04 stands

Most of it landed. Present and tested: the 32-sector `find_sector` and
`set_sector` with compass-point bumping and mask, `check_crosses_zero`,
`opposite_planes`, both `line_on_one_side` forms, `lines_on_original_side`,
`alignment_same_side`, `convex_hull_overlaps`, `tangents_diverge`,
`dist_end_ratio`, the hull sweep, and the loop itself (`insert`, `merge`,
`loop_count`, `loop_contains`, `previous`, `validate_next`) in an `AngleList`
arena.

Still absent, and all for one reason — they need the span graph attached to
real segment geometry, which is item 05:

`set`, `setSpans`, `computeSector`, `endsIntersect`, `endToSide`, `midToSide`,
`checkParallel`, `orderable`, `after`.

`AngleList::insert` and `merge` take the comparator as a closure precisely so
`after` drops in without touching the splice logic. **Note the contract**:
`after(angle, test)` must mean "angle lies in the ccw arc from `test` to
`test.next`", not "angle's sector is greater". A naive `>` is not an ordering on
a ring, and under `merge` it silently *drops* angles — confirmed against the
C++ algorithm directly, so it is a property of the algorithm, not of this port.

## Remaining order

```
05 segment winding  ->  04 (finish)  ->  06 coincidence
                    ->  07 sortable top  ->  08 common fns  ->  09 bridge
```

Item 05 moved to the front: it unblocks the rest of 04, and `SkOpSegment` is
still on the pre-arena `Box`-linked model with two competing `SkOpSpan`
definitions, which everything downstream has to reconcile anyway.
