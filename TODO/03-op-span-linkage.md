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
