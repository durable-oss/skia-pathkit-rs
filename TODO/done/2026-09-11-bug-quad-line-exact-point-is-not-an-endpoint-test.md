# Quad/line intersection always reports 2 hits; `exact_point` is not an endpoint test

Found: 2026-09-11, while porting Skia's `tests/PathOpsQuadLineIntersectionTest.cpp`.

## Symptom

`sk_d_quad_line_intersection::intersect` returns 2 for every input, including
quad/line pairs that do not touch at all.

Upstream's `lineQuadTests` table, run against the port:

| # | quad | line | expected | got |
|---|---|---|---|---|
| 0 | (1,1) (2,1) (0,2) | (0,0)-(1,1) | 1 | **2** |
| 1 | (0,0) (1,1) (3,1) | (0,0)-(3,1) | 2 | 2 |
| 2 | (2,0) (1,1) (2,2) | (0,0)-(0,2) | 0 | **2** |
| 3 | (4,0) (0,1) (4,2) | (3,1)-(4,1) | 0 | **2** |
| 4 | (0,0) (0,1) (1,1) | (0,1)-(1,0) | 1 | **2** |

Cases 2 and 3 are disjoint — the line misses the quad entirely — and still
report two intersections.

## Cause

`DLine::exact_point` in `src/pathops/sk_d_quad_line_intersection.rs:159` is a
different function from the one it is named after.

Upstream (`SkPathOpsLine.cpp:22`) is an exact-equality endpoint test:

```cpp
double SkDLine::exactPoint(const SkDPoint& xy) const {
    if (xy == fPts[0]) { return 0; }   // do cheapest test first
    if (xy == fPts[1]) { return 1; }
    return -1;
}
```

The port instead solves for `t` along the line and accepts any result in range:

```rust
pub fn exact_point(pt: Point, line: &DLine) -> Scalar {
    let dx = line.p1.x - line.p0.x;
    let dy = line.p1.y - line.p0.y;

    if dx.abs() > 1e-10 {
        let t = (pt.x - line.p0.x) / dx;
        if approximately_one_or_less_double(t) && approximately_zero_or_more_double(t) {
            return t;
        }
    }
    // ... same for dy
    -1.0
}
```

Two problems compound:

1. It answers "where along the line is this point's x?" rather than "is this
   point an endpoint?". Any point whose x-projection lands in `[0, 1]` gets a
   non-negative `t`.
2. It tests **one coordinate at a time** and returns on the first hit, so the
   other coordinate is never checked. A point nowhere near the line passes as
   long as its x falls within the line's x-range.

`add_exact_end_points` (`:398`) consults it for both quad endpoints:

```rust
for q_index in [0, 2] {
    let line_t = DLine::exact_point(self.quad.point(q_index), self.line);
    if line_t >= 0.0 {
        let quad_t = (q_index / 2) as Scalar;
        let _ = self.intersections.insert(quad_t, line_t, self.quad.point(q_index));
    }
}
```

Both endpoints are therefore inserted unconditionally in the common case,
which is the 2 that comes back. The root solve that follows is fine; its
results are simply added to two bogus endpoint hits, and `unique_answer`
rejects genuine roots that coincide with them.

Note this is a *module-local* `DLine`, distinct from
`sk_path_ops_line::DLine`, whose own `exact_point` (`:76`) is correct. The two
types share a name and differ in contract.

## Also wrong in the same file

`quad_near_point` (`:437`) subtracts its argument from itself:

```rust
fn quad_near_point(pt: Point, line_dir: Point) -> Scalar {
    let dx = line_dir.x;
    let dy = line_dir.y;
    if dx.abs() > dy.abs() {
        let t = (pt.x - line_dir.x) / dx;   // line_dir used as both origin and direction
```

Its caller in `add_near_end_points` passes `self.line.point(l_index)` and
`self.line.point(!l_index)`, so `line_dir` is an endpoint, not a direction. The
function does not project onto the quad at all — despite the name, no quad is
in scope. Upstream's equivalent walks the quad looking for a near point.

`check_coincident` (`:451`) was compared against upstream and matches, trailing
`setCoincident` included. It is not implicated.

## Repro

```rust
use pathkit::core::Point;
use pathkit::pathops::sk_d_quad_line_intersection::{intersect, DLine, DQuad};

// Disjoint: the vertical line x=0 never reaches the quad, which spans x >= 1.
let quad = DQuad::new(Point::new(2.0, 0.0), Point::new(1.0, 1.0), Point::new(2.0, 2.0));
let line = DLine::new(Point::new(0.0, 0.0), Point::new(0.0, 2.0));
assert_eq!(intersect(&quad, &line, true), 0);  // fails: returns 2
```

## Fix

Make `exact_point` an equality test against the two endpoints, matching
upstream, and let `near_point` keep the tolerance-based work. Then rewrite
`quad_near_point` to take the quad and search it, or drop it in favour of the
existing `sk_path_ops_line::DLine::near_point`.

Worth considering whether this module's `DLine`/`DQuad` should exist at all:
`sk_path_ops_line::DLine` already has correct `exact_point`/`near_point`, and
duplicating the type is what allowed the contracts to drift apart.

## Acceptance

- All five `lineQuadTests` rows from `tests/PathOpsQuadLineIntersectionTest.cpp`
  return their expected counts (1, 2, 0, 0, 1).
- For every reported hit, `quad.pt_at_t(quad_t)` and `line.pt_at_t(line_t)`
  agree to tolerance — upstream's own check.
- The six `oneOffs` rows from the same file produce no hit whose two points
  disagree.
- `exact_point` returns -1 for a point that shares an x with the line but lies
  well off it.

---

## Fixed, 2026-09-11

`DLine::exact_point` is now the endpoint identity test upstream defines, and
`DLine::near_point` is the real `SkDLine::nearPoint` — both coordinates have to
fall in the line's range before a perpendicular projection runs, and the
resulting distance is checked against a ULPS tolerance scaled to the line's
own magnitude. The single-axis early return that let a point pass on its x
alone is gone.

`quad_near_point` was replaced rather than patched. It is now a port of
`SkDCurve::nearPoint` for the quad verb: reject against the control hull's
bounding box, cast a ray through the point perpendicular to the line, take the
nearest place the quad crosses it, and require that distance to be within ULPS.
Its caller also passed `self.line.point(!l_index)` — a bitwise NOT on a
`usize`, which only reached the right endpoint because `DLine::point` clamps
out-of-range indices to `p1`. That is now `1 - l_index`.

The module-local `DLine` still shadows `sk_path_ops_line::DLine`, which is what
let the two contracts drift apart in the first place. Unifying them is a larger
change that touches `horizontal`, `vertical` and `intersect_ray`, so it is left
alone here; the note in the "Fix" section above still stands.

### Acceptance

- `upstream_line_quad_tests_report_the_expected_counts` — all five rows return
  1, 2, 0, 0, 1.
- `every_reported_hit_has_agreeing_points` — upstream's point-agreement check
  over the same table.
- `exact_point_is_an_endpoint_test_not_a_projection` and
  `near_point_checks_both_coordinates` cover the two specific defects.
- The `oneOffs` bullet is **not** covered: `old/pathkit/` vendors `src/` only,
  with no `tests/` anywhere in the repo, so that table is not available to port
  from. Worth revisiting if the upstream tests are ever vendored.
