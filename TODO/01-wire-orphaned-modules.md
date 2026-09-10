# 01 — Seven pathops files never compile

**Independent.** Do before anything else; it changes what the compiler can
check for every later item.

## Problem

These exist under `src/pathops/` but are not declared in `src/pathops/mod.rs`,
so `rustc` never sees them. Nothing in them is type-checked, and the
`cargo clippy` / `missing_docs` sweep skips them entirely:

```
SkAddIntersections.rs        SkDCubicToQuads.rs        SkPathOpsCurve.rs
SkDCubicLineIntersection.rs  SkDQuadLineIntersection.rs SkPathOpsOp.rs
SkIntersections.rs
```

`mod.rs` declares only the snake_case modules. These CamelCase files were
early ports superseded by a rename that never finished.

## Task

For each file, decide and act:

1. **Supersededom** — a snake_case module already covers it. Delete the file.
2. **Still needed** — rename to snake_case, add `pub mod` to `mod.rs`, fix
   whatever breaks.

Do them one at a time; each is its own commit. Expect real breakage on the
first `pub mod` — these have not been compiled since they were written.

Known overlaps to check first:
- `SkPathOpsCurve.rs` (1057 lines) vs `sk_path_ops_{line,quad,conic,cubic}.rs`
- `SkIntersections.rs` vs `sk_intersection_helper.rs`
- `SkPathOpsOp.rs` vs `pathops/mod.rs::op` + `boolean.rs`

## Acceptance

- No file under `src/pathops/` is absent from `mod.rs`.
- `cargo build` and `cargo test` pass.
- This check is empty:
  `for f in src/pathops/*.rs; do n=$(basename $f .rs); [ "$n" = mod ] || grep -q "mod $n;" src/pathops/mod.rs || echo "ORPHAN $n"; done`
