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
