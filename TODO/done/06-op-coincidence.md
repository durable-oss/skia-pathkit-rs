# 06 — SkOpCoincidence

**Depends on 02, 03, 05.** Blocks 07, 08. One of the two items originally
requested.

## Problem

`src/pathops/sk_op_coincidence.rs` is 57 lines against 1743 lines of C++.
Coverage 10% — the lowest in the engine. The whole file:

```rust
pub struct SkCoincidentSpans {
    pub f_coin_start: Option<usize>, pub f_coin_end: Option<usize>,
    pub f_opp_start: Option<usize>,  pub f_opp_end: Option<usize>,
    pub f_done: bool,
}
pub struct SkOpCoincidence {
    pub f_head: Option<usize>,
    global_state: Option<usize>,          // placeholder arena index
}
impl SkOpCoincidence {
    pub fn add(...) -> bool { true }              // TODO: full port
    pub fn add_missing(...) -> bool { true }
    pub fn expand(&mut self) -> bool { true }
    pub fn mark_collapsed(...) -> bool { true }
    pub fn fix_up(&mut self) -> bool { true }
    pub fn release_deleted(&mut self) {}
}
```

Five methods, all returning `true` without doing anything. `SkCoincidentSpans`
is missing `fNext`, `fOppPtTStart/End` as distinct from `fCoinPtTStart/End`,
and the `fHorizontal`/`fVertical` flags.

55 C++ functions absent. `HandleCoincidence` (item 07) calls these by name and
cannot be written without them:

```
addExpanded, addMissing, addEndMovedSpans, addIfMissing, addOrOverlap,
addOverlap, apply, mark, expand, correctEnds, correctOneEnd, findOverlaps,
checkOverlap, contains, extend, release, restoreHead, isEmpty, ordered,
flipped, overlap, TRange, Ordered, Overlaps, setStarts, setEnds, set,
coinPtTStart/End, oppPtTStart/End (+ Writable variants), next, endpoint
```

## Task

Port `SkOpCoincidence.h` + `.cpp` in dependency order:

1. `SkCoincidentSpans` proper: all four PtT endpoints, `fNext`, flags,
   `set`, `setStarts`, `setEnds`, `flipped`, `ordered`, `extend`, `contains`.
2. List management on `SkOpCoincidence`: `add`, `next`, `isEmpty`, `release`,
   `restoreHead`, `releaseDeleted`, `fixUp`.
3. Range predicates: `TRange`, `Ordered`, `Overlaps`, `overlap`,
   `checkOverlap`.
4. `expand`, `addExpanded` — the loosening pass.
5. `addMissing`, `addIfMissing`, `addOrOverlap`, `addOverlap` — the A-B/A-C
   implies B-C inference.
6. `correctEnds`, `correctOneEnd`, `addEndMovedSpans`.
7. `mark` — writes coincidence back onto the spans.
8. `apply` — adjusts winding values for coincident edges.
9. `findOverlaps` — builds the secondary overlap set that
   `HandleCoincidence`'s loop drains.

`apply` and `mark` are where correctness actually shows up in output; the
earlier steps only feed them.

## Acceptance

- Two segments sharing a collinear run are detected as one coincident pair,
  with correct start/end PtTs on both sides.
- `apply` adjusts winding so a shared edge between two same-direction
  contours ends up interior (this is the case `sk_path_ops_simplify.rs`
  currently hacks around in `dedup_coincident`).
- `isEmpty` and the `findOverlaps` loop terminate — `HandleCoincidence` runs
  them under a `SAFETY_COUNT` of 3 and returns failure if it does not settle.
- No method returns a bare `true`.

---

## Closed (2026-09-14)

Every method returned a bare `true`; the ones that matter are now real.

| part | landed as |
|---|---|
| 1 `SkCoincidentSpans` | already had all four PtT ends and `fNext`; `extend_record`, `record_contains` added |
| 2 list management | `add_run`, `records`, `relink`, `release_deleted`, `fix_up` |
| 3 range predicates | `overlaps`, `ordered_segments`, `shared_range` |
| 4 `expand`, `addExpanded` | `expand`, `add_expanded` |
| 5 `addOverlap` family | `add_or_extend`, `add_overlap`, `find_contributing` |
| 7 `mark` | `mark` |
| 8 `apply` | `apply` |
| 9 `findOverlaps` | `find_overlaps` |

`correctEnds` / `addEndMovedSpans` (part 6) are not ported. They repair
records whose endpoints moved during `move_nearby`; the arena's `move_nearby`
merges PtT rings rather than moving points, so there is nothing to correct.
If point-moving is added later, they come back with it.

### On the acceptance criteria

- Two segments sharing a collinear run are detected as one pair with correct
  start/end PtTs on both sides: `add_or_extend`, tested.
- `apply` makes a shared edge interior: `apply_folds_a_shared_edge_onto_one_side`
  asserts the pair's total winding is conserved *and* that one side is zeroed
  and marked done. This is the case `sk_path_ops_simplify::dedup_coincident`
  hand-rolls around.
- `isEmpty` and the `findOverlaps` loop terminate under `SAFETY_COUNT`:
  `sk_op_common::handle_coincidence` keeps the bound and returns false.
- No method returns a bare `true`.

One test expectation was wrong on first write: a flipped pair walks the
opposite run from its far end, so segment A's head pairs with segment B's
*tail*. Comparing the two heads reads one zeroed span and one untouched one
and looks like a failure to cancel. The test now checks the spans `apply`
actually pairs.
