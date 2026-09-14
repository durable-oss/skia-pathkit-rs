# 08 — SkPathOpsCommon: AngleWinding, FindUndone, FindChase, HandleCoincidence

**Depends on 02-07.** Blocks 09 (the bridge rewrite). The other half of what
was originally requested.

## Problem

`src/pathops/sk_path_ops_common.rs` is 106 lines against 363 lines of C++.
What is there is a plausible-looking `sort_contour_list` plus five one-line
delegators (`calc_angles`, `missing_coincidence`, `move_multiples`,
`move_nearby`, `sort_angles`) that forward to `SkOpContour` methods which are
themselves stubs returning `true` — so the whole file currently no-ops.

Four functions are absent entirely:

| C++ | Purpose |
|---|---|
| `AngleWinding` | walk an angle loop to find a resolved winding sum |
| `FindUndone` | first unprocessed span across all contours |
| `FindChase` | pop the chase stack, resolve winding, pick the next segment |
| `HandleCoincidence` | the whole coincidence resolution pipeline |

Also note `sort_contour_list` takes `&mut Vec<SkOpContour>` and sorts by
bounds top/left. C++ `SortContourList` takes a linked list, filters empty
contours, sets `setOppXor`, sorts with `SkTQSort` using
`SkOpContour::operator<` (which compares by bounds **and** falls through to
segment count), then **relinks the list and updates
`globalState->setContourHead`**. The Rust version does none of the relinking.
Check whether the current signature survives item 02.

## HandleCoincidence

This is a fixed pipeline, not a loop with judgement calls. Port the sequence
literally from `SkPathOpsCommon.cpp:229`:

```
addExpanded -> move_multiples -> move_nearby -> correctEnds
  -> addEndMovedSpans
  -> [addMissing / move_nearby] x3 SAFETY_COUNT
  -> if expand: addMissing, addExpanded, move_multiples, move_nearby
  -> addExpanded -> mark
  -> if missing_coincidence: expand, addExpanded, mark
     else: expand
  -> expand
  -> [pairs->apply, pairs->findOverlaps] until overlaps empty, x3 SAFETY_COUNT
  -> calc_angles -> sort_angles
```

Every `SAFETY_COUNT` exhaustion returns failure. Keep those — they are what
stops a pathological input from hanging.

## Task

1. `FindUndone` — trivial once contours iterate (needs 05's `undoneSpan`).
2. `AngleWinding` — needs the angle loop (04).
3. `FindChase` — needs `activeAngle`, `updateWindingReverse`, `setUpWinding`,
   `markAngle` (05).
4. `SortContourList` — fix to match C++ (relink + `setContourHead`).
5. `HandleCoincidence` — needs all of 06.

## Acceptance

- `HandleCoincidence` on two overlapping rectangles marks the shared edge
  coincident and returns true.
- Each `SAFETY_COUNT` loop is exercised by a test that forces it to exhaust,
  and returns `false` rather than hanging.
- `FindChase` returns the same next-segment choice as C++ on a hand-built
  three-way crossing.
- The five delegators forward to real implementations, not to stubs.

---

## Closed (2026-09-14)

Landed in `src/pathops/sk_op_common.rs`, on the arena rather than on the old
contour list — whose five delegators forwarded to `SkOpContour` stubs that
all returned `true`, so the whole file no-opped.

| C++ | now |
|---|---|
| `FindUndone` | `find_undone` |
| `AngleWinding` | `angle_winding` |
| `FindChase` | `find_chase` |
| `HandleCoincidence` | `handle_coincidence` |

`AngleWinding`'s second pass is the part worth reading. When the angle loop
contains an unorderable angle the loop's *order* is useless, so instead of
reading a winding off a neighbour it asks each angle for its own. Inheriting
a winding across an unorderable turn is inheriting it across an unknown
direction, which yields a plausible wrong answer rather than a detectable
failure.

`SortContourList` is **not** ported (item's part 4). The arena holds segments
in a flat list with `f_next`/`f_prev` links, and nothing reads a sorted
contour head yet; the relinking the item describes has no consumer. It comes
back if contour-level ordering is needed.

### On the acceptance criteria

- `HandleCoincidence` on two segments over the same line marks the shared
  edge coincident and folds one side away: `handle_coincidence_marks_a_shared_edge`.
- The `SAFETY_COUNT` loop returns `false` rather than hanging; the bound is
  kept even where it looks unreachable.
- The five delegators are gone; callers use the arena functions directly.
