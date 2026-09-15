# Reproduce the ribbon-union slash artifact inside pathkit directly

Split out of
`TODO/2026-09-15-union-of-many-adjacent-line-polygons-adds-boundary-noise.md`,
which stays as the full investigation history (ruled-out hypotheses,
synthetic sweeps that didn't reproduce, the font-vectorizer-side dedupe fix
that didn't explain it either). This file is just the next concrete step
that source file's own disposition already called for, broken into pieces
small enough to close independently.

## Where it stands

Real ribbon coordinates from a failing `font-vectorizer` run (`bowl/full.png`,
17 edges, Y/multi-junction topology, post-dedupe) are pasted in the source
file. The artifact is a self-intersection-shaped defect inside a single
292-point unioned contour (not a disjoint extra piece, not a failure to
merge two regions) — narrowed but not yet reproduced inside pathkit's own
test suite with real geometry. Every synthetic hand-built shape tried so far
(rectangles, T-junctions, Y-junctions, chains, off-tangent crossings) came
back clean.

## Independently testable pieces

1. **Build the 17 ribbons from the pasted coordinates** as `Path`s in a
   `sk_op_engine.rs` test (or a scratch binary first, promoted to a test
   only once it reproduces) and fold them pairwise with `op_with_engine(_,
   _, PathOp::Union)` in edge order, exactly as `expand_skeleton` does. This
   alone — does it reproduce the slash — is the highest-value single data
   point available right now.
2. **If it reproduces:** dump the verb/point list of the result directly
   (no `font-vectorizer` involved) and confirm the same signature reported
   there — one contour with an out-of-place bounding box and a
   self-intersection-shaped defect, not a disjoint piece. This rules in/out
   whether the bug is in `op_with_engine` itself versus something specific
   to how `font-vectorizer` reads the result back (`SkPath::iter()` /
   `sk_path_to_contours`).
3. **If it does not reproduce:** that shifts the search to the boundary
   between pathkit and font-vectorizer (the routing between `op_with_engine`
   and the `boolean.rs` fallback in `pathops::op`, or something in
   `font-vectorizer`'s own result-walking code) — a separate investigation,
   likely to live on the font-vectorizer side, not this repo. Record the
   negative result here either way; don't leave it unresolved silently.
4. **Once reproduced inside pathkit**, bisect which subset of the 17 edges
   is minimal to trigger it (binary-search style: try the full set, then
   halves, etc.) — 17 ribbons is too much geometry to trace by hand, and a
   minimal failing subset is what actually makes tracing `build`/
   `record_if_coincident`/the walk (per the source file's step 3) tractable.
5. **Trace the minimal repro** through `build`/`record_if_coincident`/the
   walk the way the sibling angle-ordering and broad-sweep gaps in this
   directory were traced, once piece 4 has something small enough to trace.
   This is where a real root cause and fix would land; don't start here.

Piece 1 is the only one that should start immediately. Pieces 2-3 branch on
its result. Pieces 4-5 are blocked on reproducing at all.

## Acceptance

- [ ] Piece 1: real 17-edge geometry run through `op_with_engine` directly,
      reproduces or doesn't — either way, recorded.
- [ ] Piece 2 or 3 (whichever branch applies): confirms whether this is a
      pathkit-side bug or a font-vectorizer-side integration issue.
- [ ] Piece 4: minimal failing edge subset found (only if reproduced).
- [ ] Piece 5: root cause traced and fixed (only if reproduced).
