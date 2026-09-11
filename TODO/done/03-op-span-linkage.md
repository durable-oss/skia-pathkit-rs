# 03 — SkOpSpan / SkOpSpanBase / SkOpPtT linkage

**Depends on 02.** Blocks 04-08.

## Problem

`src/pathops/sk_op_span.rs` is 182 lines against 1062 lines of C++
(`SkOpSpan.h` + `.cpp`). Coverage 19%, and what exists is fields plus trivial
accessors. 62 C++ functions are absent, including every one that makes a span
part of a graph:

```
next, prev, setNext, setPrev, upCast, upCastable, starter, ptT, segment,
contour, globalState, insert, merge, mergeMatches, release, final, deleted,
coincident, insertCoincidence, containsCoincidence, clearCoincident,
insertCoinEnd, containsCoinEnd, coinEnd, addOpp, isCoincident, isCanceled,
setToAngle, setFromAngle, toAngle, fromAngle, chased, setChased, step,
setWindValue, setOppValue, bumpSpanAdds, alreadyAdded, markAdded, ...
```

`SkOpPtT` (the point-at-t node, `sk_op_span.rs:13`) has `f_next` but no
`next()`, no `span()`, no `oppPrev()`, no `contains()` over the loop — it is
a circular list in C++ and a disconnected struct here.

## Task

Port `SkOpSpan.h` + `SkOpSpan.cpp` onto the arena from item 02. Suggested
sub-commits, each independently testable:

1. `SkOpPtT` circular list: `next`, `insert`, `contains`, `oppPrev`, `active`,
   `find`, `span`, `segment`.
2. `SkOpSpanBase` chain: `next`/`prev`/`setNext`/`setPrev`, `upCast`,
   `upCastable`, `final`, `starter`, `ptT`, `segment`, `contour`.
3. Coincidence membership: `insertCoincidence`, `containsCoincidence`,
   `clearCoincident`, `insertCoinEnd`, `containsCoinEnd`, `isCoincident`,
   `isCanceled`, `addOpp`.
4. Angle attachment: `setToAngle`, `setFromAngle`, `toAngle`, `fromAngle`.
5. Winding state: `setWindValue`, `setOppValue`, `computeWindSum` (a real
   computation — see `SkOpSpan::computeWindSum` in the .cpp), `markAdded`,
   `bumpSpanAdds`, `alreadyAdded`.
6. `merge` / `mergeMatches` / `release` — the mutation-during-walk paths.
   Do these last; they are the subtlest.

`upCast` is worth calling out: C++ downcasts `SkOpSpanBase*` to `SkOpSpan*`
when the span is not final. In Rust make that explicit — either two pools
(base vs. full) or a `is_final` discriminant with `upcast() -> Option<SpanId>`.
Pick one in item 02 and stay with it.

## Acceptance

- Each sub-commit lands with unit tests over a hand-built two-segment graph.
- `computeWindSum` computes rather than returning the stored field.
- `starter(end)` returns the lesser of the two spans, matching C++ semantics
  (test both orderings).
- A PtT loop of 3+ nodes round-trips via `next()`.

---

## Resolution (2026-09-10)

Done, all six sub-parts, each its own commit with tests over a hand-built
two-segment graph. The operations live on `OpArena` rather than on the node
structs, because a walk needs the pool: a node has no way back to it.

| part | what landed |
|---|---|
| 1 | `ptt_next/prev/insert/ring/contains/find/contains_segment/contains_t/span/segment/active/opp_prev/add_opp/is_alias/on_end` |
| 2 | `span_next/prev/set_next/set_prev/is_final/upcastable/segment/ptt/starter/step`, `segment_spans`, `span_contains_segment` |
| 3 | `span_coincident/coin_end/is_coincident/contains_coincidence/contains_coin_end/insert_coincidence/insert_coin_end/clear_coincident/coincidence_reaches`, `is_canceled` |
| 4 | `span_to_angle/from_angle/set_to_angle/set_from_angle` |
| 5 | `set_wind_value/set_opp_value/already_added/mark_added/bump_span_adds/chased/set_chased`, `span_compute_wind_sum` |
| 6 | `span_contains_span`, `span_set_wind_sum/set_opp_sum`, `span_release`, `span_merge`, `span_merge_matches` |

### `upCast`, as the item asked

`span_upcastable(id) -> Option<SpanId>`, `None` at t == 1. The arena keeps
`SkOpSpan` and `SkOpSpanBase` in **one pool** — the choice made in item 02 —
so the distinction cannot live in the type. `SkOpSpan` is now an alias for
`SkOpSpanBase` and the winding fields moved onto it; whether a span is terminal
is answered by `span_is_final`.

### Things worth knowing

- **`computeWindSum` computes.** It was returning the stored field. It now runs
  the search bounded by `MAX_WINDING_TRIES`. `FindSortableTop` is item 07, so
  the search is a closure parameter — an explicit dependency rather than a stub
  that silently succeeds. Same pattern for `mark_all_done` in
  `span_merge_matches`, which is `SkOpSegment`'s job (item 05).
- **`setWindSum` / `setOppSum` flag disagreement.** If the walk reaches a span
  two ways with different sums, C++ records `setWindingFailed` and keeps the
  first value. The old setters silently overwrote.
- **`find` includes the starting node; `contains` does not.** Both are ported
  as C++ has them, and the tests pin the difference down.
- **`SkOpCoincidence::fixUp` is not called from `span_release`.** That is item
  06; a coincident run touching a released span is left stale until then.

### Acceptance

- Each part landed with its own tests; 60 in `sk_op_arena` overall.
- `compute_wind_sum` computes, and stops after `MAX_WINDING_TRIES` rather than
  spinning.
- `starter(end)` tested in both argument orders.
- A PtT ring of three round-trips through `next`, and two rings become one
  four-node ring after `add_opp`.
