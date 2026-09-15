# pathops engine: what is still stubbed, and what SkOpAngle is waiting on

Found: 2026-09-10, while porting `SkOpAngle` on top of `3f6aa61`.

## Summary

`src/pathops/sk_op_angle.rs` and `src/pathops/sk_line_parameters.rs` are now
ported for everything that does not read the span graph. The rest of the
pathops engine is still largely stubs that return hard-coded `true` / `false` /
`None`, so the boolean engine has no working span insertion, coincidence
handling, or winding computation. This file records what a later session should
pick up, roughly in dependency order.

Nothing here is a regression — it is the port's remaining surface. The one
active user-visible defect is tracked separately in
`2026-09-10-simplify-returns-empty-on-self-crossing-closing-segment.md`.

## 1. The two competing SkOpSpan definitions

There are two unrelated `SkOpSpan` / `SkOpSpanBase` types:

| Location | Model | Used by |
|---|---|---|
| `src/pathops/sk_op_span.rs` | arena indices (`Option<usize>`) | `sk_op_coincidence.rs` |
| `src/pathops/sk_op_segment.rs:31-99` | `Box`-linked, no winding fields | `SkOpSegment` itself |

The segment-local copy has no `f_coincident`, `f_to_angle`, `f_wind_sum`, or
`f_opp_sum`, so it cannot carry winding. **These must be unified before angles
can be attached to spans**, and the arena model in `sk_op_span.rs` is the one to
keep — it already matches how `SkOpAngle`'s `AngleList` links its members, and
Skia's own pointer graph is cyclic in a way `Box` cannot express.

Blocks: everything below.

## 2. SkOpAngle: the parts deliberately not ported

The geometric core is done and tested (sector assignment, the loop algorithms,
convex-hull and tangent predicates). What is missing is exactly the set of
methods that dereference `fStart` / `fEnd` into the span graph. From
`old/pathkit/src/pathops/SkOpAngle.cpp`:

| C++ method | Needs | Note |
|---|---|---|
| `setSpans` | `SkOpSegment::subDivide` | fills `fPart` / `fOriginalCurvePart` and `fSide`; the cubic branch also needs `SkDCubic::FindInflections` |
| `computeSector` | span `next`/`prev`/`final` walking | lengthens an angle too short to have a sector; `f_compute_sector` is already set for it |
| `endsIntersect` | `CurveIntersectRay`, `SkIntersections` | the main ordering path for curves |
| `endToSide`, `midToSide` | `CurveIntersectRay`, `closestTo`, `mostOutside` | fallbacks used by `checkParallel` |
| `checkParallel` | the two above plus `dPtAtT` | partially expressible today; left out to keep it honest |
| `orderable` | all of the above | the three-way comparator |
| `after` | `orderable` | drives `insert` |

`AngleList::insert` and `merge` take the comparator as a closure
(`FnMut(&AngleList, usize, usize) -> bool`) precisely so `after` can be dropped
in once these land, without touching the splice logic.

**Note on the comparator contract.** `after(angle, test)` must mean "angle falls
in the counterclockwise arc from `test` to `test.next`", not "angle's sector is
greater". A naive `>` comparison is not a valid ordering on a circular list, and
under `merge` it silently *drops* angles: `merge` calls `insert`, which can
re-enter `merge`, and an angle that compares as belonging nowhere gets unlinked
without being relinked. This was confirmed against the C++ algorithm directly —
it is a property of the algorithm, not of the Rust port. The test helper
`by_sector` in `sk_op_angle.rs` shows the shape a correct comparator must have.

## 3. SkOpSegment — the biggest gap

`src/pathops/sk_op_segment.rs` returns constants from most of its interesting
methods:

```rust
pub fn add_t(..) -> Option<&mut SkOpSpan>   { None }   // must insert a span at t
pub fn calc_angles(&mut self)               { }       // must build SkOpAngles
pub fn sort_angles(&mut self) -> bool       { true }  // must sort the angle loop
pub fn sub_divide(..) -> bool               { true }   // must emit the curve start..end
pub fn mark_and_chase_done(..) -> bool      { true }
pub fn find_next_op(..) -> Option<..>       { None }
pub fn missing_coincidence(&self) -> bool   { false }
pub fn move_multiples(&mut self) -> bool    { true }
pub fn move_nearby(&mut self) -> bool       { true }
```

`sub_divide` is the one `SkOpAngle::setSpans` needs first, and its signature is
also wrong for that use: it currently takes a `SkPathWriter` sink, where the
angle code needs it to fill a curve (C++ has both overloads — `subDivide(start,
end, SkDCurve*)` is the one to add).

C++ `SkOpSegment` exposes roughly 90 methods; see PORTPLAN.txt §2 for the full
list.

## 4. SkOpCoincidence — every method is a no-op

`src/pathops/sk_op_coincidence.rs` is 57 lines against a 1900-line original.
`add`, `add_missing`, `expand`, `mark_collapsed`, and `fix_up` all return `true`
without doing anything; `release_deleted` is empty. Coincident-edge handling is
therefore entirely absent, which is a plausible contributor to the union defects
noted in the simplify TODO (two near-identical discs returning 2 contours,
tangent shapes dropping an input).

## 5. Orphaned files never compiled

Seven files under `src/pathops/` are not declared in `mod.rs`, so they are dead
weight that nothing type-checks:

```
SkAddIntersections.rs   SkIntersections.rs        SkDCubicLineIntersection.rs
SkDCubicToQuads.rs      SkDQuadLineIntersection.rs SkPathOpsOp.rs
SkPathOpsCurve.rs
```

Two of them hold things the angle code will want — `SkIntersections::closest_to`
/ `most_outside` (needed by `endToSide` / `midToSide`) and `SkDCurveSweep` /
`SkDCurve` in `SkPathOpsCurve.rs`. Both are written in **f32**, where the C++
uses `double` throughout.

**Decision made for `SkOpAngle`:** it carries its own f64 `AngleVector` and
`CurveSweep` rather than depending on these. Angle sorting is numerically
delicate — the whole `cross_check` / sector scheme exists to make sign decisions
survive rounding — and f32 is not a safe substitute. Whoever wires these files
in should plan to convert them to f64 rather than making `sk_op_angle` narrow to
meet them. `sk_line_parameters.rs` is f64 for the same reason.

Either wire these up (converted) or delete them; leaving them uncompiled means
they rot.

## 6. Smaller items

- `sk_op_contour.rs` still declares its own empty `pub struct SkOpCoincidence;`
  and `pub struct SkPathWriter;` forward declarations that shadow the real
  types. The `SkOpAngle` one was replaced with a re-export of the real type in
  this session; the other two should get the same treatment once those modules
  are real.
- `SkOpSpan::compute_wind_sum` returns `self.f_wind_sum` unchanged, ignoring the
  global state it is handed.
- `sk_path_ops_types.rs` has ULPs helpers for f32 only. The angle code needs
  them on f64 and narrows to f32 at the call site, matching how the C++ `double`
  overloads of `AlmostEqualUlps` are defined (`SkPathOpsTypes.h:237`). If f64
  ULPs helpers are added later, keep that narrowing — widening the tolerance
  would change sort results.
- `sk_path_ops_tsect.rs` uses the deprecated `std::f32::INFINITY` constant.

## Suggested order

1. Unify the two `SkOpSpan` definitions onto the arena model (§1).
2. Port `SkOpSegment::sub_divide` into a curve, plus span insertion `add_t` (§3).
3. Port `SkOpAngle::setSpans` / `computeSector` on top of those (§2).
4. Convert and wire in `SkIntersections` + `SkPathOpsCurve` as f64 (§5).
5. Port `endsIntersect` / `endToSide` / `midToSide` / `orderable` / `after`, and
   hand `after` to `AngleList::insert` (§2).
6. Port `SkOpCoincidence` for real (§4).

---

## Closed (2026-09-15)

Every item here is resolved or has moved to a file of its own.

| § | item | outcome |
|---|---|---|
| 1 | two competing `SkOpSpan` definitions | The arena's won, as this file recommended. The `Box`-linked one in `sk_op_segment.rs` is unreachable dead code; deleting it is `17-retire-the-pre-arena-segment-model.md`. |
| 2 | `SkOpAngle`'s unported half | Done — `sk_op_angle_order.rs`. See `done/04-op-angle-loop.md`. |
| 3 | `SkOpSegment`'s stubs | Done on the arena. See `done/05-op-segment-winding.md`. |
| 4 | `SkOpCoincidence` all no-ops | Done. See `done/06-op-coincidence.md`. |
| 5 | seven orphaned files | Done. See `done/01-wire-orphaned-modules.md`. Two turned out superseded and were deleted; the rest were wired in. |
| 6 | `sk_op_contour`'s shadow types | `SkOpCoincidence` and `SkPathWriter` are now re-exports of the real types. `SkOpGlobalState` stays until item 17 retires the model that uses it. |
| 6 | `SkOpSpan::compute_wind_sum` returning its field | Fixed — it runs the `sortable_top` closure under `MAX_WINDING_TRIES`. |
| 6 | f32-only ULPs helpers | Unchanged, and deliberately: this file's own advice was to keep the narrowing at the call site, matching how the C++ `double` overloads are defined. |
| 6 | `sk_path_ops_tsect.rs` deprecated `std::f32::INFINITY` | Already gone. |

The "suggested order" at the bottom was followed almost exactly, with one
change worth recording: step 4 said to convert `SkIntersections` and
`SkPathOpsCurve` to f64. Instead `sk_curve_intersect_ray.rs` was written
fresh in f64 for what the angle code actually needs — `intersectRay` is a
much smaller thing than the full segment intersection those files implement,
and converting them would have meant touching every existing caller.
