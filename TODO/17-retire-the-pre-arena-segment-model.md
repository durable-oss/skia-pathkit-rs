# 17 — Retire the pre-arena segment model

**Independent of item 09.** Pure deletion plus one doc fix; it does not touch
the working engine and can land before or after the `op` switch.

## Problem

Five modules under `src/pathops/` are the segment model the port used before
the arena existed. They compile, they are tested, and **nothing in the
working engine calls them**:

| module | lines |
|---|---|
| `sk_op_segment.rs` | 655 |
| `sk_op_contour.rs` | 704 |
| `sk_op_edge_builder.rs` | 313 |
| `sk_intersection_helper.rs` | 232 |
| `sk_path_ops_common.rs` | 106 |
| **total** | **2006** |

They reference each other and nothing else. Confirmed:

```
$ for m in sk_op_contour sk_op_segment sk_op_edge_builder \
           sk_intersection_helper sk_path_ops_common; do
    grep -rln "$m::" src/ --include="*.rs" \
      | grep -v "src/pathops/$m.rs" \
      | grep -vE "sk_op_contour|sk_op_segment|sk_op_edge_builder|sk_intersection_helper|sk_path_ops_common"
  done
(one hit: a doc-comment reference in sk_op_arena.rs:268)
```

The arena replaced each of them:

| pre-arena | replacement |
|---|---|
| `SkOpSegment` (Box-linked spans, own `Verb`, own `SkOpSpan`) | `ArenaSegment` + `OpArena` |
| `SkOpContour` | the contour links on `ArenaSegment` (`f_next`/`f_prev`/`f_contour`) |
| `SkOpEdgeBuilder` | `sk_op_engine::add_path` |
| `SkIntersectionHelper` | `sk_op_engine::add_intersect_ts` |
| `sk_path_ops_common.rs` | `sk_op_common.rs` |

## Why it is worth doing rather than leaving

Three concrete costs, not tidiness:

1. **`sk_op_segment.rs` declares a second `SkOpSpan`.** This was §1 of
   `2026-09-10-pathops-engine-port-gaps.md` — two unrelated span types, one
   arena-indexed and one `Box`-linked. The arena one won; the other is still
   there to be imported by mistake.

2. **`sk_op_segment.rs` declares a third `Verb`.** `core::Verb` is what the
   engine speaks. A fourth lives in `sk_path_ops_curve.rs`. Picking the wrong
   one is a compile error today only because the types differ; it is a real
   trap for the next person.

3. **`sk_intersection_helper.rs` holds a `&'static mut SkOpSegment`.** That
   is a lifetime laundering that exists only because the pre-arena model had
   no other way to express the graph. It should not outlive the model.

## Task

1. Check each module's tests for anything that tests *geometry* rather than
   the dead graph — `sk_op_segment.rs`'s `pt_at_t`, `bounds`,
   `is_horizontal`/`is_vertical` are ported and tested, and the arena's
   equivalents should cover the same cases before the originals go. Move any
   test that is not already duplicated.
2. Delete the five files and their `pub mod` lines.
3. Fix the doc-comment reference at `sk_op_arena.rs:268`.
4. Check `sk_path_ops_debug.rs` and `sk_op_builder.rs`, which were not in the
   grep above but may name these types in prose.

## Acceptance

- The five files are gone and `cargo build --lib` passes.
- `cargo test --lib` passes with no fewer tests than before, once geometry
  tests are moved rather than dropped.
- `grep -rn "SkOpContour\|SkIntersectionHelper" src/` is empty.
- Only one `SkOpSpan` and one pathops-facing `Verb` remain reachable.
