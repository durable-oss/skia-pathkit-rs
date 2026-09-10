# 14 — SkOpCubicHull — ALREADY PORTED (closed 2026-09-10, no work needed)

Closed during the 2026-09-10 audit without changes. Recorded so nobody
re-ports it.

All three C++ functions in `old/pathkit/src/pathops/SkOpCubicHull.cpp` are
present in `src/pathops/sk_path_ops_cubic.rs`, faithfully:

| C++ | Rust |
|---|---|
| `SkDCubic::convexHull` | `sk_path_ops_cubic.rs:469` `convex_hull` |
| `static bool rotate` | `rotate` |
| `static int side` | `side` |

(`other_two` is present too.) Tests exist:
`convex_hull_quadrilateral` (`:912`), `convex_hull_degenerate_point` (`:920`).

## Why the audit initially flagged it

The coverage scan compared each C++ translation unit against the Rust file of
the same name. There is no `sk_op_cubic_hull.rs` — the code was folded into
`sk_path_ops_cubic.rs`, matching how Skia declares these as `SkDCubic` methods
even though it defines them in a separate .cpp. That is the better layout;
leave it.

**Lesson for the remaining items:** a 0% coverage number means "no file of
that name", not necessarily "not ported". Check whether the code landed in a
sibling module before starting. Applies to items 11, 12, 13.
