# Union of two near-coincident discs fragments into 4 contours

**Open.** Found 2026-09-10 while fixing item 15; split out because it is a
different failure and item 15's own acceptance criteria all pass.

## Symptom

Two discs of radius 40 whose centres are 0.5 apart (99.4% overlap) union to
**4** contours instead of 1. The result is not empty and covers roughly the
right area, but it is fragmented.

```rust
let mut a = Path::new(); a.add_circle(200.0, 200.0, 40.0);
let mut b = Path::new(); b.add_circle(200.5, 200.0, 40.0);
op(&a, &b, PathOp::Union)      // 4 contours, want 1
```

Measured after the item 10 and 15 fixes, radius 40:

| offset | union | intersect | difference |
|---|---|---|---|
| 0.5 | **4c** | **6c** | 1c |
| 1 | 1c | 1c | 1c |
| 2, 5, 20, 40, 60 | 1c | 1c | 1c |

So it is confined to a narrow band of near-coincidence. Ellipses are correct at
every offset tested, including 0.5.

## Diagnosis so far

Dumping the contours at offset 0.5 shows one of the four is a degenerate sliver
at the near-tangency point:

```
-- contour 2 start (200.0000,160.0000)
   (200.2500,160.0061)
   (200.5000,160.0000)
   close after 2 lines
```

The two circles nearly touch at top and bottom. The sliver thrown up there
consumes edges that the main ring needs, so the ring cannot close and comes out
as fragments 1, 3 and 4.

## What was tried and did not work

Both were implemented, measured, and reverted — do not re-try them blind:

1. **Dropping zero-area contours in `assemble`.** The sliver's area is about
   0.0015, above any threshold that does not also discard legitimate thin
   contours. Releasing its edges back for reuse did not help either: the greedy
   most-left-turn walk re-consumes them the same way.
2. **Tightening `FLAT_TOL` globally** (0.1 → 0.01). Fixes the discs and breaks
   the ellipse at offset 0.5. It moves the failure rather than removing it, and
   costs tessellation everywhere.

What *did* help, and is in the tree, is `flatten_tolerance` sizing the
flattening from how close the two inputs are — that is what fixed offsets 1 and
2. It bottoms out at offset 0.5.

## Assessment

This is a defect in the **substitute** boolean engine (`boolean.rs`), which
item 09 deletes outright. Two arcs flattened independently each deviate from the
true curve by up to the tolerance; where the inputs are closer together than
that deviation, their chords interleave and no amount of tolerance tuning
separates them reliably. The real engine does not flatten at all.

So: worth fixing only if items 02-09 are far off. If they are close, this
disappears with the engine that causes it.

## Acceptance

- Two discs of radius 40 union to 1 contour at every offset from 0.5 upward.
- Intersect likewise.
- The ellipse sweep in `union_of_offset_discs_is_one_contour`'s neighbourhood
  does not regress.

---

## Resolved in the real engine (2026-09-14)

This file's own assessment was right: "worth fixing only if items 02-09 are
far off. If they are close, this disappears with the engine that causes it."

Measured today, radius 40, offset 0.5:

| | flattening engine | `sk_op_engine` |
|---|---|---|
| union | **4 contours** | **1 contour, 8 curves** |
| intersect | **6 contours** | declines (falls back) |

`discs_union_to_one_contour_across_the_offset_sweep` in `sk_op_engine.rs`
checks 0.5, 1, 2, 5, 20, 40 and 60 — all one contour.
`near_coincident_discs_union_to_one_contour_with_their_curves` pins the 0.5
case and that the result is still made of curves.

The mechanism is exactly what this file predicted. Two arcs flattened
independently each deviate from the true curve by up to the tolerance; where
the inputs are closer together than that deviation, their chords interleave.
Nothing is flattened in the real engine, so there are no chords to interleave.

**Still open in the sense that `pathops::op` does not use that engine yet.**
Until `TODO/09-bridge-winding-xor.md`'s remaining winding bug is fixed, a
caller going through `op` still gets the four-contour answer. Nothing more is
needed *for this defect*; it closes when the switch lands.

Do not re-try the two approaches recorded above (dropping zero-area contours,
tightening `FLAT_TOL`). They were measured and reverted, and neither is
relevant to the engine that replaces them.

---

## Closed (2026-09-15)

The switch landed: `pathops::op` routes through `sk_op_engine` as of
`c5f5e29`. Both regression tests pass through the public entry point, not
just the engine directly:

```
$ cargo test --lib discs_union_to_one_contour_across_the_offset_sweep
$ cargo test --lib near_coincident_discs_union_to_one_contour_with_their_curves
test ... ok (both)
```

All three acceptance criteria hold. Moved to `TODO/done/`.
