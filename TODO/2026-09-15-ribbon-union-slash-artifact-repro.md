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

## Update (2026-09-15, later): piece 1 done — a similar-shaped defect appears, but it is not the same signature as the reported slash

Built the 17 ribbons from the pasted coordinates directly in a
`sk_op_engine.rs` test (`scratch_ribbon_union_slash_repro`, `#[ignore]`d
and kept in the tree — real production geometry, worth re-running rather
than a one-off throwaway) and folded them pairwise with
`op_with_engine(_, _, PathOp::Union)` in edge order, exactly as
`expand_skeleton` does.

**Result**: no decline anywhere in the 16-step pairwise fold — every step
resolved directly through the engine, none fell back to `boolean.rs`. The
final result has 10 contours, 440 verbs total:

- **Contour 0**: 392 points, bounding box `(44.3, 5.0)-(783.6, 684.1)` —
  spans the whole glyph, matching the "bounding box spanning the entire
  glyph" part of the reported signature. (Different point count than the
  292 originally reported — expected, since font-vectorizer's own pipeline
  runs additional dedup/smoothing before this stage that this repro
  bypasses by using the pasted post-dedupe coordinates directly.)
- **Contours 1-9**: nine small fragment contours, 2 to 6 points each,
  scattered around the glyph (e.g. `(586.8,529.5)-(602.0,546.9)`,
  `(639.3,490.5)-(648.2,493.4)`). Contour 9 is degenerate: two points at
  the *same* coordinate, i.e. zero area.

**But**: a naive O(n²) segment-crossing check
(`count_self_intersections`, in the same test) found **zero
self-intersections** in the 392-point dominant contour. That means this
does *not* reproduce the specific "self-intersection-shaped defect inside
a single contour" signature the source file's real-coordinate update
described — no slash cutting across the ring was found in this repro,
despite the bounding-box match.

**Reading this as two related-but-distinct findings, not one:**
1. The nine small fragment contours (one of them literally zero-area) are
   real, reproducible boundary noise from this exact pairwise-fold
   pipeline on real geometry — a legitimate defect on its own, independent
   of whether it's the reported slash.
2. The reported self-intersecting slash itself did **not** reproduce here.
   Per this file's own piece 3, that shifts the search: either
   font-vectorizer's own pre-processing (smoothing/additional dedup
   between skeleton extraction and `ribbon_boundary`) or its result-reading
   code (`sk_path_to_contours`) does something this direct repro bypasses,
   or the real pipeline's coordinates differ subtly from the pasted
   snapshot in a way that matters. Not established which.

## Independently testable pieces

1. ~~**Build the 17 ribbons... fold them pairwise**~~ Closed above. Result:
   a similar-shaped-but-distinct defect (fragment-contour noise, bounding
   box matches) reproduces; the specific self-intersecting slash does not.
2. ~~**If it reproduces:** dump the verb/point list...~~ Partially
   applicable — done (see breakdown above), but since the *exact* slash
   signature didn't reproduce, this doesn't yet rule in `op_with_engine`
   as the slash's cause. It does rule `op_with_engine` in as the source of
   the fragment-contour noise, which is real and unexplained.
3. **If it does not reproduce** (the slash specifically, per above): the
   search for the slash itself shifts to the boundary between pathkit and
   font-vectorizer — worth checking `font-vectorizer`'s own
   `sk_path_to_contours` result-walking code, and whether its full
   pipeline's coordinates differ from the pasted post-dedupe snapshot used
   here. Not started.
4. **Separately, trace the fragment-contour noise found here** — nine small
   contours (one zero-area) from a clean 17-edge real-geometry fold that
   should plausibly resolve to many fewer contours. This is a new, more
   tractable target than the original slash: real geometry, already
   reproducing, no decline involved. Bisecting which edge pair(s) produce
   which fragment (piece 4's bisection approach, retargeted at this
   finding instead of the slash) is the natural next step.
5. **Trace the minimal repro** through `build`/`record_if_coincident`/the
   walk, once a minimal subset is found for either the fragment-noise
   target (4) or the slash (if 3 relocates it back into pathkit).

## Acceptance

- [x] Piece 1: real 17-edge geometry run through `op_with_engine` directly.
      Reproduces a related defect (bounding-box-spanning contour + 9 small
      fragment contours) but not the exact reported self-intersection
      signature — recorded above with the distinction made explicit.
- [ ] Piece 2 or 3 (whichever branch applies): not resolved for the slash
      specifically — the fragment-contour noise is confirmed pathkit-side
      (reproduces directly, no font-vectorizer involved), but whether the
      *slash* is pathkit-side or font-vectorizer-side is still open.
- [ ] Piece 4: minimal failing edge subset — not found yet for either the
      original slash or the newly-found fragment-contour noise.
- [ ] Piece 5: root cause traced and fixed — not started.
