# 15 — SkLineParameters — PORTED (closed 2026-09-10)

Listed as 0% coverage / "no Rust file" at the start of the 2026-09-10 audit.
`src/pathops/sk_line_parameters.rs` (449 lines) landed while the audit was
running, and `pub mod sk_line_parameters;` is declared in
`src/pathops/mod.rs`. It compiles.

Ported surface includes `control_pt_distance_{quad,cubic}`, `cubic_distance_y`,
`quad_distance_y`, `cubic_end_points`, `cubic_end_points_at`, `cubic_part`,
`quad_end_points`, `quad_part`, `line_end_points`, `dx`, `dy`, `normalize`,
`normal_squared`, `point_distance`, `near_ray`, with unit tests for the
degenerate cubic cases.

No further work identified. Reopen if `SkOpAngle` (item 04) needs something
from it that turns out to be missing — `convexHullOverlaps` and
`lineOnOneSide` are the consumers.
