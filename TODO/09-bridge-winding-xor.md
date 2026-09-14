# 09 — Rewrite simplify and op on the real engine (bridgeWinding / bridgeXor)

**Depends on 02-08.** The payoff item — everything above exists to make this
possible.

## Current state

`src/pathops/sk_path_ops_simplify.rs` implements Skia's `SimplifyDebug`
control flow (convex fast path, build, intersect, classify, bridge, assemble)
but on a **flattened-edge substitute engine**, because the op-segment engine
did not exist when it was written. The module docs say so explicitly.

Consequences of the substitute:

- Curves are flattened to line segments, so output is always polylines. Skia
  preserves the original quad/conic/cubic verbs via `addCurveTo`.
- `collect_boundary` decides membership by sampling `Path::contains` at a
  point offset perpendicular to each edge. That is a point-in-polygon test per
  edge, not winding-sum bookkeeping — it is O(edges x path size) and gets the
  wrong answer where the offset lands across a nearby edge.
- `dedup_coincident` is a hand-rolled stand-in for `HandleCoincidence`.
- `is_convex` is hand-rolled because `Path::is_convex()` does not exist.

`src/pathops/boolean.rs` is the same substitute engine for `op`.

## Task

Once 02-08 land, replace both with faithful ports:

1. `bridgeWinding` and `bridgeXor` from `SkPathOpsSimplify.cpp` — the real
   ones, walking the segment graph via `findNextWinding`/`findNextXor` and
   emitting through `addCurveTo`.
2. `SimplifyDebug` proper: `SkOpEdgeBuilder` -> `SortContourList` ->
   `AddIntersectTs` -> `HandleCoincidence` -> bridge -> `assemble`.
3. `OpDebug` from `SkPathOpsOp.cpp` for `op`, replacing `boolean.rs`.
4. Delete the substitute engine and its helpers once the tests pass on the
   real one.

Keep the existing 19 simplify tests — they encode correct behaviour and
should pass unchanged against the real engine. If one fails, decide which is
wrong before changing it.

## Acceptance

- Output preserves curve verbs: simplifying a path containing a cubic yields
  a path containing a cubic, not 200 line segments.
- All existing pathops tests pass.
- `examples/simplify_empty_repro.rs` passes.
- The disc-union cases from item 10 pass.
- `boolean.rs` and the substitute helpers in `sk_path_ops_simplify.rs` are
  gone.

---

## Progress (2026-09-14): the engine exists and keeps curves, but is not wired in

`src/pathops/sk_op_engine.rs` is the real thing: the arena edge builder,
`AddIntersectTs`, collinear coincidence detection, and the `bridgeOp` /
`bridgeWinding` walks. `op_with_engine` and `simplify_with_engine` are its
entry points.

**The curve-preservation acceptance criteria pass**, against the shape from
`2026-09-14-boolean-ops-destroy-all-curves.md` (a stem of four lines plus a
ring of eight cubics, differenced against a rectangle that touches only the
stem):

- `a_contour_the_operation_never_touches_keeps_its_curves` — the result
  contains `Verb::Cubic`.
- `the_result_does_not_explode_into_a_polyline` — under 40 segments, against
  the flattening engine's 261.
- `a_cut_curve_is_subdivided_rather_than_flattened` — a disc cut by a
  rectangle comes back with cubics.
- `the_engine_unions_two_overlapping_rectangles` — one closed contour,
  correct inside/outside at four sample points.

`pathops::op` still routes to `boolean.rs`.

## What is left, precisely

Routing `op` through the engine passes 1005 of 1007 tests. The two failures
are both in `sk_op_builder`, and both reduce to one case: **two rectangles
whose top and bottom edges are collinear and overlapping**.

Traced as far as this:

1. Coincidence *is* detected there now (`record_if_coincident`), and
   `apply` folds the shared run correctly — segment 0's middle span comes out
   with `wind_value = 2`.
2. The walk still emits that span. `is_active` calls `active_op`, which calls
   `update_winding(end, start, sortable_top)`, and that returns **0** rather
   than the 2 the span carries.
3. So the interior bottom edge reads as "outside on the far side", which for
   `Union` puts it on the boundary. The result is rectangle A's outline plus
   a stray `(20,0)->(8,0)` fragment, and B is never walked at all.

The suspect is the ray-cast winding, not the gate: a ray fired from a point
on the bottom edge exits downward immediately and reads 0, which is right for
*that operand* but never picks up B's contribution into `oppSum`.
`sk_op_sortable_top::accumulate` swaps `wind`/`opp_wind` per hit on
`segment_operand`, which is the C++ shape, so the defect is probably in which
spans the ray finds rather than in the accumulation.

Start there: `find_sortable_top` on segment 0's t = 0.4 span, and check what
`accumulate` sees. `a_ray_that_runs_along_an_edge_is_retried_in_another_direction`
in `sk_op_sortable_top.rs` sets up exactly this geometry.

## Still not done from the original task list

- `bridgeXor` proper — `find_next_xor` exists, but nothing calls it;
  `simplify_with_engine` uses `bridgeWinding` for both fill rules.
- Deleting `boolean.rs` and the substitute helpers in
  `sk_path_ops_simplify.rs`. They stay until the switch lands.
- Curve/curve coincidence. `record_if_coincident` handles line/line only,
  which is the case that breaks a union of boxes; two identical curves need
  the t-section machinery.

### What was tried and did not work

Recorded so it is not re-tried blind, in the style of `TODO/16`:

**Pre-resolving every span's winding before the walk starts.** The reasoning
was that `find_sortable_top` only resolves the spans it happens to visit on
its way to a starting point, so a gate reached later reads an unset value.
Running `sortable_top` over every non-terminal span up front, each with the
full `MAX_WINDING_TRIES` budget, does make every sum available — and makes
things *worse*: it broke the two disc-union tests that previously passed,
taking the failure count from 2 to 5.

The order matters. A ray cast from a span whose neighbours have not been
resolved yet accumulates a different total than the same ray cast later, and
`mark_and_chase_winding` then spreads that wrong value along the chase. The
walk resolves spans in an order that makes each cast meaningful; resolving
them in segment order does not.

So the fix is not "resolve more, earlier". It is to find why one particular
span's cast reads 0.

### Where the data actually stands

Dumping every span after `build` on the two-collinear-rectangles case shows
the windings are **all correct and all resolvable**:

```
seg 0 (0,0)->(20,0)   opnd=false t=0    wv=1 ov=0  ws=-1 os=0
seg 0 (0,0)->(20,0)   opnd=false t=0.4  wv=2 ov=0  ws=-2 os=0   <- interior
seg 1 (20,0)->(20,20) opnd=false t=0    wv=1 ov=0  ws=-2 os=0
...
seg 4 (8,0)->(28,0)   opnd=true  t=0    wv=0 ov=0  (zeroed by coincidence)
seg 4 (8,0)->(28,0)   opnd=true  t=0.6  wv=0 ov=1  ws=0  os=-1
```

`ws=-2` on segment 0's t = 0.4 span is right: covered on both sides,
therefore interior, therefore not on a union's boundary. The gate still lets
it through, and `active_op` was observed getting `sum_mi = 0` for it. So the
defect is between `update_winding` and `active_op_with`, on a span whose
stored sum is already correct — not in the ray cast that produced it.

---

## Update (2026-09-14, later): three more defects fixed, two cases left

Routing `op` through the engine now passes **1012 of 1013**, down from 1005
of 1007 when this was first written. Three real defects came out of chasing
that number down:

1. **`set_up_windings` did not swap on the operand.** The binary form branches
   on `operand()`: a second-operand segment takes its own delta off
   `sumSuWinding` and reads `sumMiWinding` as its opposite
   (`SkOpSegment.cpp:1655`). The port always took it off the first sum, so
   every second-operand edge was measured against the wrong running total.
   Fixing this took the five-rectangle union and both disc-union cases from
   failing to passing.

2. **`update_winding` guarded on the span sign rather than the winding.**
   C++ is `if (winding && ...)` (`:1701`). Note `updateOppWinding` (`:1719`)
   really does guard on `oppSpanWinding` — the asymmetry is in the original,
   and both are now commented so neither gets "fixed" to match the other.

3. **The edge builder put the second operand's 1 in `oppValue`.** C++ sets
   `windValue = 1` on every span whichever operand it belongs to
   (`SkOpSpan::init`); the distinction lives in `operand()`. With the 1 in
   the wrong field, coincidence's `apply` folded a shared edge into one
   operand's winding with nothing left in the other, so the result thought
   the second input covered nothing. This is what made a Difference across a
   shared edge keep the part it should have cut.

The four `update_winding` call sites in the walker were also passing a
closure that never resolved anything; they now use the real ray cast, with
the segment list parked on the arena the way C++ reaches the contour list off
the global state.

### Two more fixed, one case left

Two further defects landed after the above:

4. **`expand` called two records duplicates on matching starts alone.**
   `reach_out` widens the coincident side without touching the opposite one,
   so two records can briefly agree on where they begin while describing
   different runs. On two rectangles sharing *both* horizontal edges, both
   records were created and one was discarded before `handle_coincidence`
   ran; the second rectangle's whole top edge kept a winding it should have
   given up, and `calc_angles` found nothing to sort at the corner the walk
   needed to turn at. All four ends are compared now.

5. **`close_open_contour` emitted at most one edge, and gated it wrong.** A
   contour can be missing several edges when the walk ran out of *active*
   continuations partway round. It loops now, and uses the same active-edge
   gate the walk does - with the right operator and a real resolver. It had
   been calling `active_winding`, the one-operand form, with a closure that
   never resolved anything.

Removing the gate entirely while testing the loop is worth recording: it
closes every contour, and a Difference then gets back the piece it just cut.
The gate is load-bearing, not belt-and-braces.

### What is left: one case

**Two rectangles sharing both horizontal edges still union to two contours.**
Unioning `(0,0,20,20)` with `(8,0,28,20)` gives A's outline plus a fragment.
The closing step finds the right edge - segment 1, A's right side at x = 20,
running (20,20) to (20,0) - and the gate then refuses it, because that span's
winding reads as interior when it is not.

So the remaining defect is in the winding on a segment that is *not* part of
any coincident run but sits between two that are. Every other segment in that
graph resolves correctly; segment 1 does not.

Intersect on the same geometry fails the same way and for what looks like the
same reason: `an_intersect_across_a_shared_edge_keeps_only_the_overlap` is
`#[ignore]`d rather than weakened, so `cargo test -- --ignored` shows it.
Difference on those two rectangles is now correct, which is the useful
contrast - the graph is the same, so the difference is in how the two sums
are read, not in how they were built.

### Correction: the winding is right, the walk is short

Dumping segment 1 after `handle_coincidence` shows `ws = -1, os = -1`, and
`active_op` answers **false** for both directions along it. That is *correct*:
x = 20 is A's right edge, B spans 8..28, so B covers it, and an edge with
fill on both sides is not on a union's boundary. The gate is right to refuse
it.

The real symptom is narrower than described above. Unioning `(0,0,20,20)`
with `(8,0,28,20)` returns A's outline alone — `(25, 10)`, which is inside B,
reads as outside. Segments 5 and 6 (B's right side and the part of its top
beyond x = 20) are never walked at all, and their windings are correct and
non-zero:

```
SEG 4 (8,0)->(28,0)   [(0.0, 0, 0), (0.6, 1, 0), (1.0, 1, 0)]
SEG 5 (28,0)->(28,20) [(0.0, 1, 0), (1.0, 1, 0)]
SEG 6 (28,20)->(8,20) [(0.0, 1, 0), (0.4, 0, 0), (1.0, 1, 0)]
```

Segment 4's t = 0 span and segment 6's t = 0.4 span are the coincident runs,
correctly zeroed. Everything else carries winding.

### Traced one step further

The outer loop *is* reached; `find_sortable_top` returns `None` on the second
pass. Before the walk starts, no segment is done:

```
PRE SegmentId(5) count=2 done_count=0 done=false
PRE SegmentId(6) count=3 done_count=1 done=false
```

so the walk itself retires segments 5 and 6 without ever emitting them. That
is `pick_next`: it calls `mark_and_chase_done` on every angle it does not
take, and `mark_done(starter)` on the edge it leaves. C++ does the same —
and then drains the chase list, which is where those retired spans are
supposed to come back.

**The chase list is empty when the first contour finishes**, and the middle
loop therefore runs exactly once. That is the whole failure: `pick_next`
retires segments 5 and 6 but nothing records where to come back to, so
neither `find_chase` nor the next `find_sortable_top` can reach them.

Two things were checked and are *not* it:

- `pick_next`'s inactive branch does push now — it takes the span
  `mark_and_chase_done` stopped at, since `lastMarked` is only set when
  `computeSum` actually transferred a winding and here it never does. That
  change is in the tree and correct on its own terms, but the list is still
  empty, so `pick_next` is not seeing an inactive angle on this input.
- `bridge`'s own inactive branch never fires either: the gate passes on the
  first edge, `walk_contour` runs, and the loop exits on the empty chase.

### The end of the trace: a winding sum that is too large by one

Dumping `pick_next`'s ring walk answers it. At (20, 20) the two candidates
are:

```
(20,20)->(28,20)  active=false  ws=-2  os=-2  wv=1
(20,20)->(20,0)   active=false  ws=-3  os=-1  wv=1
```

Both inactive, so the walk stops there and the chase list stays empty —
which is the empty-chase symptom above, and the reason segments 5 and 6 are
never reached.

`(20,20)->(28,20)` is B's top edge beyond the overlap. It **is** on the
union's boundary: above it is empty. Its winding should have a zero on one
side, and reads `ws = -2, os = -2` instead — fill on both sides, therefore
interior, therefore refused.

So the whole chain ends at the ray-cast winding counting a crossing that is
not there. The suspect is `sk_op_sortable_top::accumulate` and the spans
coincidence zeroed: it skips a hit whose span has `wind_value == 0 &&
opp_value == 0`, which is right, but the *segments* those spans belong to are
still crossed by the ray and still bound a region. Check what `accumulate`
sees for a ray fired upward from the middle of segment 6's t = 0 span, and
whether the zeroed coincident spans on segments 0 and 4 are being skipped in
a way that leaves the running total one too high.

`accumulate` itself was compared line by line against
`SkOpSpan::sortableTop`'s second half and matches, including the
zero-winding `continue` and where `last` is assigned. So the suspect moves
one step earlier, to **`ray_check`: which spans it reports a hit against.**

### Correction again, and the real remaining question

Casting a ray directly from segment 6's head gives `ws = -1, os = 0` — the
right answer, one crossing below and nothing above. The `-2 / -2` the walk
sees is therefore on a *different span* of that segment, not a bad cast of
the same one.

`span_set_wind_sum` and `span_set_opp_sum` both refuse to overwrite a
differing sum (they set `winding_failed` instead), and `mark_winding_opp`
and the chase match C++ line for line, so nothing is clobbering a good value
with a bad one. Which leaves: **the ring member at (20, 20) is not the span
the probe cast from.**

Segment 6 is `(28,20)->(8,20)` with spans at t = 0, 0.4, 1. The walk's
candidate `(20,20)->(28,20)` runs *backwards* along it, so its starter is the
t = 0 span — which is what the probe used. But the dump earlier showed
segment 6's t = 0.4 span zeroed by coincidence, and `span_starter` picks by
lesser t, so a candidate spanning t = 0..0.4 starts at t = 0 and one spanning
0.4..1 starts at 0.4.

### Answered, and one more fix

The span ids match. The `-2` arrives *first*, chased in across a corner from
the other input, and the setters then correctly refuse to overwrite it with
the right value.

`next_chase`'s no-angle branch was stepping to `ptt_next` — one arbitrary
ring neighbour. In C++ a plain corner is a two-element ring so that is
unambiguous; here a corner that is also a crossing holds members from both
inputs. It now prefers a member on the segment this one is linked to along
its own contour. Fixed and tested
(`next_chase_prefers_the_segment_on_its_own_contour`).

### Where it stands

Routing `op` through the engine passes **1016 of 1017** with one ignored.
The single remaining failure is
`sk_op_builder::union_of_five_overlapping_rects_matches_sequential_ops`, and
Intersect across a shared edge is still wrong (the `#[ignore]`d test).

Both are the same geometry family: rectangles that share a full collinear
edge. Everything else in the suite - unions and differences of overlapping
boxes, near-coincident discs at every offset, curve preservation through a
cut - passes through the engine.

`winding_failed` is set in `span_set_wind_sum` and `span_set_opp_sum` and
read nowhere. It is the signal that two casts disagreed about a span, which
is exactly the symptom above; wiring it into `bridge` so a disagreement fails
the op rather than silently keeping the first answer would at least turn a
wrong result into a fallback.
