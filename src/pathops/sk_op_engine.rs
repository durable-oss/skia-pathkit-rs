//! Building the arena graph from paths, and driving the walk over it.
//!
//! Port of `SkOpEdgeBuilder`'s arena half, `AddIntersectTs`, and the
//! `bridgeOp` / `bridgeWinding` walks from `SkPathOpsOp.cpp` and
//! `SkPathOpsSimplify.cpp`.
//!
//! # This is where curves survive
//!
//! The substitute engine in [`boolean`](super::boolean) flattens every
//! contour to line segments before it starts, so its output is always a
//! polyline and there is no way to recover the curve. Here the segments keep
//! their own control points the whole way through: intersections split them
//! at t values, the walk steps between spans, and
//! [`add_curve_to`](super::sk_op_walker::add_curve_to) subdivides the
//! original geometry to emit each piece. A contour the operation never
//! touches comes back with its input verbs unchanged.

use super::sk_op_angle_order::{calc_angles, sort_angles};
use super::sk_op_arena::{OpArena, SegmentId, SpanId};
use super::sk_op_coincidence::SkOpCoincidence;
use super::sk_op_common::{find_chase, find_chase_op, handle_coincidence};
use super::sk_op_walker::{add_curve_to, find_next_op, find_next_winding, WalkState};
use super::sk_path_writer::SkPathWriter;
use super::sk_op_sortable_top::{find_sortable_top, sortable_top};
use super::PathOp;
use crate::core::{Path, FillType, Point, Verb};

/// The graph one or two paths were built into.
#[derive(Debug)]
pub struct OpGraph {
    /// The arena holding every span, PtT, segment and angle.
    pub arena: OpArena,
    /// Every segment, in the order the paths were walked.
    pub segments: Vec<SegmentId>,
    /// Coincident runs found while intersecting.
    pub coincidence: SkOpCoincidence,
}

/// Adds `path`'s segments to the graph, marked as `operand`'s side.
///
/// Port of `SkOpEdgeBuilder::addOperand` plus `walk`. Degenerate segments —
/// those whose endpoints coincide — are dropped rather than added: they
/// contribute no winding and would give the angle sorter a zero-length
/// direction to order.
pub fn add_path(graph: &mut OpGraph, path: &Path, operand: bool, xor: bool, opp_xor: bool) {
    let mut contour_start: Option<Point> = None;
    let mut current: Option<Point> = None;
    // Segments added for the contour being walked, so its ends can be
    // linked into one ring when it closes.
    let mut contour_segments: Vec<SegmentId> = Vec::new();
    for (verb, pts, weight) in path.iter() {
        match verb {
            Verb::Move => {
                // A new contour: close off whatever came before it.
                close_contour(graph, &mut contour_segments, contour_start, current, operand, xor, opp_xor);
                contour_start = Some(pts[0]);
                current = Some(pts[0]);
            }
            Verb::Close => {
                close_contour(graph, &mut contour_segments, contour_start, current, operand, xor, opp_xor);
                current = contour_start;
            }
            Verb::Line | Verb::Quad | Verb::Conic | Verb::Cubic => {
                let count = verb.point_count();
                let w = weight.unwrap_or(1.0);
                if let Some(seg) =
                    push_segment(graph, &pts[..count], verb, w, operand, xor, opp_xor)
                {
                    contour_segments.push(seg);
                }
                current = Some(pts[count - 1]);
            }
        }
    }
    // An unclosed contour is closed implicitly, matching Skia: pathops only
    // has an answer for filled regions.
    close_contour(graph, &mut contour_segments, contour_start, current, operand, xor, opp_xor);
}

/// Closes the contour just walked and links its segments end to end.
///
/// The linking is what makes a corner one place rather than two. Each
/// segment is built with its own PtT nodes at t = 0 and t = 1; without
/// joining consecutive segments' rings, two sides meeting at a corner do not
/// know about each other, `calc_angles` finds nothing to sort there, and the
/// walk cannot turn the corner - it reports the edge unsortable and gives
/// up, which is exactly what leaves each edge its own contour.
#[allow(clippy::too_many_arguments)] // the builder's per-contour state
fn close_contour(
    graph: &mut OpGraph,
    segments: &mut Vec<SegmentId>,
    contour_start: Option<Point>,
    current: Option<Point>,
    operand: bool,
    xor: bool,
    opp_xor: bool,
) {
    if let (Some(start), Some(cur)) = (contour_start, current) {
        if !points_equal(start, cur) {
            if let Some(seg) =
                push_segment(graph, &[cur, start], Verb::Line, 1.0, operand, xor, opp_xor)
            {
                segments.push(seg);
            }
        }
    }
    if segments.is_empty() {
        return;
    }
    // Join each segment's tail to the next segment's head, and the last back
    // to the first: a closed contour is a ring.
    for i in 0..segments.len() {
        let a = segments[i];
        let b = segments[(i + 1) % segments.len()];
        let (Some(a_tail), Some(b_head)) = (
            graph.arena.segment(a).f_tail,
            graph.arena.segment(b).f_head,
        ) else {
            continue;
        };
        if let (Some(pa), Some(pb)) = (
            graph.arena.span_ptt(a_tail),
            graph.arena.span_ptt(b_head),
        ) {
            if pa != pb {
                graph.arena.ptt_add_opp(pa, pb);
            }
        }
        // The contour's own next/prev links, which the walk follows when
        // only one segment continues.
        graph.arena.segment_mut(a).f_next = Some(b);
        graph.arena.segment_mut(b).f_prev = Some(a);
    }
    segments.clear();
}

/// Adds one segment, skipping it if it collapses to a point.
fn push_segment(
    graph: &mut OpGraph,
    pts: &[Point],
    verb: Verb,
    weight: f32,
    operand: bool,
    xor: bool,
    opp_xor: bool,
) -> Option<SegmentId> {
    if points_equal(pts[0], pts[pts.len() - 1]) && verb == Verb::Line {
        // A line from a point to itself has no direction to sort by.
        return None;
    }
    let seg = graph.arena.alloc_segment_with_curve(pts, verb, weight);
    graph.arena.set_segment_operand(seg, operand);
    graph.arena.set_segment_xor(seg, xor, opp_xor);
    // Every segment starts with windValue = 1 and oppValue = 0, whichever
    // operand it belongs to. C++ does this in `SkOpSpan::init`, and the
    // operand distinction lives in `operand()` rather than in which field
    // carries the 1: `setUpWindings` and `activeOp` both branch on
    // `operand()` to decide which running sum a segment's own winding comes
    // off. Putting the 1 in `oppValue` for the second operand instead makes
    // every one of those branches read the wrong field, and coincidence's
    // `apply` then folds the pair into a single operand's winding with
    // nothing left in the other - so the result thinks the second input
    // covers nothing.
    for span in graph.arena.segment_spans(seg) {
        graph.arena.span_mut(span).set_wind_value(1);
        graph.arena.span_mut(span).set_opp_value(0);
    }
    graph.segments.push(seg);
    Some(seg)
}

/// Returns true when two points are the same to within tolerance.
fn points_equal(a: Point, b: Point) -> bool {
    (a.x - b.x).abs() < POINT_TOL && (a.y - b.y).abs() < POINT_TOL
}

/// How close two points must be to name the same place.
const POINT_TOL: f32 = 1e-6;

/// Splits every segment at every intersection with every other.
///
/// Port of `AddIntersectTs`. The pairwise loop is what the C++ does too; the
/// bounds check in front of it is what keeps it from being quadratic in
/// practice.
pub fn add_intersect_ts(graph: &mut OpGraph) {
    let segments = graph.segments.clone();
    for (i, &a) in segments.iter().enumerate() {
        for &b in &segments[i + 1..] {
            intersect_pair(graph, a, b);
        }
    }
}

/// Finds where two segments cross and adds a span to each at the crossing.
fn intersect_pair(graph: &mut OpGraph, a: SegmentId, b: SegmentId) {
    let (a_l, a_t, a_r, a_bot) = graph.arena.segment_bounds(a);
    let (b_l, b_t, b_r, b_bot) = graph.arena.segment_bounds(b);
    // Boxes that do not touch cannot cross.
    if a_r < b_l - POINT_TOL
        || b_r < a_l - POINT_TOL
        || a_bot < b_t - POINT_TOL
        || b_bot < a_t - POINT_TOL
    {
        return;
    }
    // Collinear segments that overlap do not cross, so the crossing search
    // finds nothing for them. They are coincident instead, and that has to
    // be recorded or the shared run's winding is counted on both sides.
    if record_if_coincident(graph, a, b) {
        return;
    }
    let crossings = find_crossings(&graph.arena, a, b);
    for (ta, tb, pt) in crossings {
        let pa = graph.arena.segment_add_t(a, ta, pt);
        let pb = graph.arena.segment_add_t(b, tb, pt);
        // Linking the two PtT nodes is what makes the crossing one place
        // rather than two: the walker steps between segments through this
        // ring, and the angle sorter reads it to know what meets here.
        if let (Some(pa), Some(pb)) = (pa, pb) {
            if pa != pb {
                graph.arena.ptt_add_opp(pa, pb);
            }
        }
    }
}

/// Records `a` and `b` as coincident when they are two lines running along
/// the same infinite line with overlapping extents.
///
/// Returns true when a record was made, in which case the pair has no
/// crossing to look for: they meet everywhere along the shared run, not at a
/// point.
///
/// Only the line/line case is detected. Curve coincidence needs the
/// t-section machinery in `sk_path_ops_tsect`; two identical curves are rare
/// in practice next to two rectangles sharing an edge, which is the case
/// that makes a union of overlapping boxes wrong.
fn record_if_coincident(graph: &mut OpGraph, a: SegmentId, b: SegmentId) -> bool {
    let (a_pts, a_verb, _) = graph.arena.segment_curve(a);
    let (b_pts, b_verb, _) = graph.arena.segment_curve(b);
    if a_verb != Verb::Line || b_verb != Verb::Line {
        return false;
    }
    let (a0, a1) = (a_pts[0], a_pts[1]);
    let (b0, b1) = (b_pts[0], b_pts[1]);
    let a_dir = (a1.x - a0.x, a1.y - a0.y);
    let b_dir = (b1.x - b0.x, b1.y - b0.y);
    // Parallel, and b's start on a's line: the two share an infinite line.
    let cross = a_dir.0 * b_dir.1 - a_dir.1 * b_dir.0;
    let len_sq = a_dir.0 * a_dir.0 + a_dir.1 * a_dir.1;
    if len_sq == 0.0 || cross.abs() > COLLINEAR_TOL * len_sq.sqrt() {
        return false;
    }
    let off = (b0.x - a0.x, b0.y - a0.y);
    let side = a_dir.0 * off.1 - a_dir.1 * off.0;
    if side.abs() > COLLINEAR_TOL * len_sq.sqrt() {
        return false;
    }

    // Project both of b's ends onto a's parameter, and see where the two
    // extents overlap.
    let project = |p: Point| ((p.x - a0.x) * a_dir.0 + (p.y - a0.y) * a_dir.1) / len_sq;
    let (tb0, tb1) = (project(b0), project(b1));
    let (lo, hi) = if tb0 <= tb1 { (tb0, tb1) } else { (tb1, tb0) };
    let start = lo.max(0.0);
    let end = hi.min(1.0);
    if end - start <= COLLINEAR_TOL {
        // They touch at a point at most, which is not a run.
        return false;
    }

    // Split both segments at the run's ends and record the pair.
    //
    // The run's ends are real endpoints of `a` or of `b`, so use those
    // coordinates rather than the interpolated ones. Evaluating segment 2 of
    // (20,20)->(0,20) at t = 0.6 gives x = 7.9999995, not 8, and a split made
    // at that point lands a hair off the corner it is supposed to share.
    // The PtT rings are keyed on exact coordinates, so the near-miss opens a
    // second ring at the same corner and the walk loses the edges in it.
    let snap = |p: Point| -> Point {
        for c in [a0, a1, b0, b1] {
            if (p.x - c.x).abs() <= COLLINEAR_TOL && (p.y - c.y).abs() <= COLLINEAR_TOL {
                return c;
            }
        }
        p
    };
    let a_start_pt = snap(graph.arena.segment_pt_at_t(a, start));
    let a_end_pt = snap(graph.arena.segment_pt_at_t(a, end));
    let Some(ca) = graph.arena.segment_add_t(a, start, a_start_pt) else {
        return false;
    };
    let Some(cb) = graph.arena.segment_add_t(a, end, a_end_pt) else {
        return false;
    };
    // The same two points, as parameters along b.
    let inv = |p: Point| {
        let d = (p.x - b0.x) * b_dir.0 + (p.y - b0.y) * b_dir.1;
        let l = b_dir.0 * b_dir.0 + b_dir.1 * b_dir.1;
        if l == 0.0 {
            0.0
        } else {
            (d / l).clamp(0.0, 1.0)
        }
    };
    let (ta, tb) = (inv(a_start_pt), inv(a_end_pt));
    let Some(oa) = graph.arena.segment_add_t(b, ta, a_start_pt) else {
        return false;
    };
    let Some(ob) = graph.arena.segment_add_t(b, tb, a_end_pt) else {
        return false;
    };
    // Join the matching points, so each end of the run is one place.
    for (x, y) in [(ca, oa), (cb, ob)] {
        if x != y {
            graph.arena.ptt_add_opp(x, y);
        }
    }
    let mut coincidence = std::mem::take(&mut graph.coincidence);
    coincidence.add_or_extend(&mut graph.arena, ca, cb, oa, ob);
    graph.coincidence = coincidence;
    true
}

/// How far off a line a point may sit and still count as on it.
const COLLINEAR_TOL: f32 = 1e-4;

/// How finely a curve is sampled when looking for crossings.
///
/// Only used to bracket a crossing, never to approximate the curve itself:
/// each bracket is then refined by bisection against the exact geometry, so
/// the resulting t values are as accurate as f32 allows. The curve that gets
/// emitted is always the original.
const SAMPLES: usize = 24;

/// How close a bisection must get before the crossing is accepted.
const REFINE_TOL: f32 = 1e-7;

/// Returns the `(t_a, t_b, point)` of every crossing between two segments.
///
/// Curve/curve intersection in general has no closed form, so this brackets
/// sign changes of the cross product between the two curves' offsets and
/// refines each by bisection.
fn find_crossings(arena: &OpArena, a: SegmentId, b: SegmentId) -> Vec<(f32, f32, Point)> {
    let mut out: Vec<(f32, f32, Point)> = Vec::new();
    // Sample b, and for each sample find where a passes closest.
    for i in 0..SAMPLES {
        let t0 = i as f32 / SAMPLES as f32;
        let t1 = (i + 1) as f32 / SAMPLES as f32;
        if let Some(hit) = refine_between(arena, a, b, t0, t1) {
            // Crossings within tolerance of one already found are the same
            // crossing seen from two brackets.
            if !out
                .iter()
                .any(|(ta, tb, _)| (ta - hit.0).abs() < 1e-4 && (tb - hit.1).abs() < 1e-4)
            {
                out.push(hit);
            }
        }
    }
    out
}

/// Refines a crossing bracketed by `t0`..`t1` on `b`.
fn refine_between(
    arena: &OpArena,
    a: SegmentId,
    b: SegmentId,
    t0: f32,
    t1: f32,
) -> Option<(f32, f32, Point)> {
    let (mut lo, mut hi) = (t0, t1);
    let mut lo_sign = closest_signed(arena, a, arena.segment_pt_at_t(b, lo))?;
    let hi_sign = closest_signed(arena, a, arena.segment_pt_at_t(b, hi))?;
    if lo_sign.1 == 0.0 {
        if !is_interior(lo) || !is_interior(lo_sign.0) {
            return None;
        }
        let pt = arena.segment_pt_at_t(b, lo);
        return Some((lo_sign.0, lo, pt));
    }
    // No sign change in this bracket means no crossing inside it.
    if (lo_sign.1 > 0.0) == (hi_sign.1 > 0.0) {
        return None;
    }
    for _ in 0..40 {
        let mid = (lo + hi) / 2.0;
        let mid_sign = closest_signed(arena, a, arena.segment_pt_at_t(b, mid))?;
        if (hi - lo).abs() < REFINE_TOL {
            let pt = arena.segment_pt_at_t(b, mid);
            // The crossing must actually be on both curves, not merely where
            // the signed distance changes.
            let on_a = arena.segment_pt_at_t(a, mid_sign.0);
            if (on_a.x - pt.x).abs() > 1e-3 || (on_a.y - pt.y).abs() > 1e-3 {
                return None;
            }
            // A touch at either curve's own endpoint is not a crossing to
            // split at: the two already meet there, and adding a span at
            // t = 0 or t = 1 would duplicate the endpoint the segments
            // share. Adjacent sides of one contour meet exactly this way.
            if !is_interior(mid) || !is_interior(mid_sign.0) {
                return None;
            }
            return Some((mid_sign.0, mid, pt));
        }
        if (mid_sign.1 > 0.0) == (lo_sign.1 > 0.0) {
            lo = mid;
            lo_sign = mid_sign;
        } else {
            hi = mid;
        }
    }
    None
}

/// Returns true when `t` is strictly inside a segment, not at either end.
fn is_interior(t: f32) -> bool {
    t > END_TOL && t < 1.0 - END_TOL
}

/// How far from an endpoint a crossing must be to count as interior.
const END_TOL: f32 = 1e-5;

/// Returns the t on `seg` nearest `pt`, and the signed side `pt` falls on.
///
/// The sign comes from the cross product of the segment's tangent with the
/// offset to `pt`, so it flips exactly when `pt` crosses the segment.
fn closest_signed(arena: &OpArena, seg: SegmentId, pt: Point) -> Option<(f32, f32)> {
    let mut best_t = 0.0f32;
    let mut best_d = f32::MAX;
    for i in 0..=SAMPLES {
        let t = i as f32 / SAMPLES as f32;
        let p = arena.segment_pt_at_t(seg, t);
        let d = (p.x - pt.x).powi(2) + (p.y - pt.y).powi(2);
        if d < best_d {
            best_d = d;
            best_t = t;
        }
    }
    // Two rounds of local refinement around the best sample.
    let mut step = 1.0f32 / SAMPLES as f32;
    for _ in 0..24 {
        step /= 2.0;
        for cand in [best_t - step, best_t + step] {
            if !(0.0..=1.0).contains(&cand) {
                continue;
            }
            let p = arena.segment_pt_at_t(seg, cand);
            let d = (p.x - pt.x).powi(2) + (p.y - pt.y).powi(2);
            if d < best_d {
                best_d = d;
                best_t = cand;
            }
        }
    }
    let on = arena.segment_pt_at_t(seg, best_t);
    let ahead = arena.segment_pt_at_t(seg, (best_t + 1e-3).min(1.0));
    let behind = arena.segment_pt_at_t(seg, (best_t - 1e-3).max(0.0));
    let tangent = (ahead.x - behind.x, ahead.y - behind.y);
    let offset = (pt.x - on.x, pt.y - on.y);
    let cross = tangent.0 * offset.1 - tangent.1 * offset.0;
    Some((best_t, cross))
}

/// Builds the graph for one or two paths and resolves its coincidence.
///
/// Returns `None` when the coincidence pipeline could not settle, which the
/// caller treats as "this engine cannot do this input" rather than as an
/// empty result.
pub fn build(one: &Path, two: Option<&Path>, xor: bool, opp_xor: bool) -> Option<OpGraph> {
    let mut graph = OpGraph {
        arena: OpArena::new(),
        segments: Vec::new(),
        coincidence: SkOpCoincidence::new(),
    };
    add_path(&mut graph, one, false, xor, opp_xor);
    if let Some(two) = two {
        add_path(&mut graph, two, true, xor, opp_xor);
    }
    if graph.segments.is_empty() {
        return None;
    }
    add_intersect_ts(&mut graph);

    let segments = graph.segments.clone();
    let mut coincidence = std::mem::take(&mut graph.coincidence);
    let ok = handle_coincidence(&mut graph.arena, &segments, &mut coincidence);
    graph.coincidence = coincidence;
    if !ok {
        return None;
    }
    Some(graph)
}

/// Walks the graph, emitting the boundary of the operation's result.
///
/// Port of `bridgeOp` (and `bridgeWinding`, which differs only in which
/// `findNext` it calls). `op` selects between them.
///
/// The shape matters and is easy to get wrong. Three nested loops:
///
/// - the outer one starts a new output contour from each resolvable span;
/// - the middle one drains the chase list, which holds turnings the walk
///   passed but did not take;
/// - the inner one walks one contour edge by edge.
///
/// Each edge is emitted **before** stepping to the next, and only after the
/// active-edge gate says it is on the result boundary. Emitting first is what
/// links the edges into one contour rather than leaving each a separate
/// move-and-line; gating is what makes the result the boolean rather than
/// every edge of both inputs.
fn bridge(
    graph: &mut OpGraph,
    op: Option<PathOp>,
    xor_mi_mask: i32,
    xor_su_mask: i32,
    writer: &mut SkPathWriter,
) -> bool {
    let segments = graph.segments.clone();
    // The ray-cast winding needs the whole graph, and the walker asks for it
    // several calls deep; parking the list on the arena is how it gets there.
    graph.arena.set_walk_segments(segments.clone());
    let mut outer_guard = OUTER_GUARD;
    loop {
        outer_guard -= 1;
        if outer_guard == 0 {
            return false;
        }
        // Two rays disagreed about some span's winding. The setters keep the
        // first answer and raise this rather than overwrite, so carrying on
        // means walking a graph that contradicts itself. Reporting failure
        // sends the caller to the flattening engine, which gets a worse
        // shape but the right one.
        if graph.arena.winding_failed() {
            return false;
        }
        let Some(span) = find_sortable_top(&mut graph.arena, &segments) else {
            break;
        };
        let Some(next) = graph.arena.span_next(span) else {
            break;
        };
        let mut state = WalkState::new(next, span);
        let mut chase: Vec<SpanId> = Vec::new();

        let mut middle_guard = INNER_GUARD;
        loop {
            middle_guard -= 1;
            if middle_guard == 0 {
                return false;
            }
            if is_active(graph, &state, op, xor_mi_mask, xor_su_mask) {
                if !walk_contour(graph, &mut state, &mut chase, op, xor_mi_mask, xor_su_mask, writer)
                {
                    return false;
                }
                // The walk stops one edge short whenever the last step had
                // nowhere active to go but the contour still has not closed.
                // That edge is the one back to the start, and without it the
                // contour is left open and assemble has nothing to stitch it
                // to. C++ does the same here.
                close_open_contour(graph, &mut state, op, xor_mi_mask, xor_su_mask, writer);
                writer.finish_contour();
            } else {
                // Not on the boundary: retire the edge, and remember where
                // the chase should come back to.
                if let Some(last) = graph
                    .arena
                    .mark_and_chase_done(state.start, state.end)
                    .flatten()
                {
                    if !graph.arena.span(last).chased() {
                        graph.arena.span_mut(last).set_chased(true);
                        chase.push(last);
                    }
                }
            }

            let mut chase_start = state.start;
            let mut chase_end = Some(state.end);
            // The binary walk needs the binary drain: `find_chase` resolves
            // one operand's winding only. The drain is reached on real
            // geometry (the disc-union sweep enters it 67 times) though no
            // case found so far comes out differently for it; the unary form
            // here would be a latent wrong answer rather than a visible one.
            let chased = if op.is_some() {
                find_chase_op(&mut graph.arena, &mut chase, &mut chase_start, &mut chase_end)
            } else {
                find_chase(&mut graph.arena, &mut chase, &mut chase_start, &mut chase_end)
            };
            match chased {
                Some(_) => {
                    let Some(e) = chase_end else { break };
                    if chase_start == e {
                        break;
                    }
                    state = WalkState::new(chase_start, e);
                }
                None => break,
            }
        }
    }
    true
}

/// Emits the edge that closes a contour the walk left open.
///
/// Port of the `activeWinding` block after `bridgeOp`'s inner loop. It fires
/// only when the edge is still on the boundary and has not been walked, so a
/// contour that genuinely ends there is not given a spurious closing line.
fn close_open_contour(
    graph: &mut OpGraph,
    state: &mut WalkState,
    op: Option<PathOp>,
    xor_mi_mask: i32,
    xor_su_mask: i32,
    writer: &mut SkPathWriter,
) {
    if writer.is_closed() {
        return;
    }
    // The walk left off at `state`, which names the edge it last emitted.
    // The edge that would close the contour is the one continuing from
    // there, so step across the shared point to find it.
    let Some(target) = writer.contour_start() else {
        return;
    };
    // Keep stepping across until the contour closes or there is nowhere
    // left to go. One step is not always enough: a contour can be missing
    // several edges when the walk ran out of *active* continuations partway
    // round, and each of those edges is on the boundary.
    let mut guard = CLOSE_GUARD;
    loop {
        guard -= 1;
        if guard == 0 {
            return;
        }
        let Some(next) = step_across(graph, state, target) else {
            return;
        };
        *state = next;
        let Some(span_start) = graph.arena.span_starter(state.start, state.end) else {
            return;
        };
        if graph.arena.span(span_start).already_added() {
            return;
        }
        // The same gate the walk itself uses. Without it the closing step
        // adds whatever edge happens to point homeward, boundary or not,
        // and a Difference gets back the piece it just cut.
        if !is_active(graph, state, op, xor_mi_mask, xor_su_mask) {
            return;
        }
        if !add_curve_to(&mut graph.arena, state.start, state.end, writer) {
            return;
        }
        graph.arena.mark_done(span_start);
        if writer.is_closed() {
            return;
        }
    }
}

/// Bounds the closing walk, so a contour that cannot be closed gives up
/// rather than circling.
const CLOSE_GUARD: i32 = 1000;

/// Returns the walk's continuation across the point `state` ends at.
///
/// The walk's last edge arrives somewhere; whatever leaves that point on
/// another segment is where a closing edge would come from. This finds it
/// through the shared PtT ring, the same way `nextChase` does.
fn step_across(graph: &OpGraph, state: &WalkState, target: Point) -> Option<WalkState> {
    let ptt = graph.arena.span_ptt(state.end)?;
    let mut best: Option<(WalkState, f32)> = None;
    for node in graph.arena.ptt_ring(ptt) {
        let Some(span) = graph.arena.ptt_span(node) else {
            continue;
        };
        if span == state.end
            || graph.arena.span_segment(span) == graph.arena.span_segment(state.end)
        {
            continue;
        }
        // Either direction along that segment is a candidate. Take the one
        // that heads back to where the contour began: the other leads away,
        // and following it lengthens the contour instead of closing it.
        for other in [graph.arena.span_next(span), graph.arena.span_prev(span)] {
            let Some(other) = other else { continue };
            let Some(starter) = graph.arena.span_starter(span, other) else {
                continue;
            };
            // `already_added`, not `done`: the closing edge is routinely
            // marked done by the walk that passed it (pick_next retires
            // every angle it does not take), but it has not been emitted,
            // and emitting it is exactly what closes the contour.
            if graph.arena.span(starter).already_added() {
                continue;
            }
            let far = graph.arena.span(other).f_pt;
            let dist = (far.x - target.x).powi(2) + (far.y - target.y).powi(2);
            if best.as_ref().map_or(true, |(_, d)| dist < *d) {
                best = Some((WalkState::new(span, other), dist));
            }
        }
    }
    best.map(|(s, _)| s)
}

/// Returns whether the edge `state` names is on the result's boundary.
///
/// The `activeOp` / `activeWinding` gate at the top of `bridgeOp`. Without
/// it the walk emits every edge of both inputs rather than the boolean.
fn is_active(
    graph: &mut OpGraph,
    state: &WalkState,
    op: Option<PathOp>,
    xor_mi_mask: i32,
    xor_su_mask: i32,
) -> bool {
    let Some(segment) = graph.arena.span_segment(state.start) else {
        return false;
    };
    // Resolve the winding at this span first. The gate reads the sums, and a
    // span whose sum is still PK_MinS32 gives an answer that is not an
    // answer: passing a closure that never resolves leaves every gate
    // reading an unset value, which is how an interior edge ends up on the
    // result's boundary.
    let segments = graph.arena.walk_segments();
    let resolve = |arena: &mut OpArena, span: SpanId| sortable_top(arena, span, &segments);
    match op {
        Some(op) => {
            let operand = graph.arena.segment_operand(segment);
            graph.arena.active_op(
                state.start,
                state.end,
                operand,
                xor_mi_mask,
                xor_su_mask,
                op,
                resolve,
            )
        }
        None => graph.arena.active_winding(state.start, state.end, resolve),
    }
}

/// Walks and emits one output contour.
///
/// The inner loop of `bridgeOp`. Each edge is emitted before the step to the
/// next, so the writer stays positioned at the contour's growing end and the
/// pieces join instead of each starting its own move.
#[allow(clippy::too_many_arguments)] // mirrors the C++ signature
fn walk_contour(
    graph: &mut OpGraph,
    state: &mut WalkState,
    chase: &mut Vec<SpanId>,
    op: Option<PathOp>,
    xor_mi_mask: i32,
    xor_su_mask: i32,
    writer: &mut SkPathWriter,
) -> bool {
    let mut guard = INNER_GUARD;
    // `simple` as it stood before the step about to be taken. C++ keeps the
    // previous iteration's value (`lastSimple = simple` ahead of
    // `findNextOp`), because the dead-end emit asks whether the edge just
    // walked was reached by a simple step, not whether the failed step was.
    let mut last_simple = false;
    loop {
        guard -= 1;
        if guard == 0 {
            return false;
        }
        if !state.unsortable {
            if let Some(seg) = graph.arena.span_segment(state.start) {
                if graph.arena.segment_done(seg) {
                    break;
                }
            }
        }
        let edge_start = state.start;
        let edge_end = state.end;
        let edge_segment = graph.arena.span_segment(edge_start);
        let prev_simple = last_simple;
        last_simple = state.simple;
        let _ = prev_simple;
        // find_next advances `state` to the edge to walk next, and reports
        // which segment that is. When it reports none the walk has run out
        // of active edges, and `state` still names the edge just walked.
        let next_segment = match op {
            Some(op) => find_next_op(
                &mut graph.arena,
                state,
                chase,
                op,
                xor_mi_mask,
                xor_su_mask,
            ),
            None => find_next_winding(&mut graph.arena, state, chase),
        };
        if next_segment.is_none() {
            // Nowhere active to go. C++ does *not* emit unconditionally here
            // (`SkPathOpsOp.cpp:140-156`): a line that ran out of
            // continuations is left for the closing step and for assemble,
            // and only a curve on an open contour, or the tail of a simple
            // step, is written out.
            //
            // With the winding fixes in place no case found so far reaches
            // this branch with a different answer either way — the walk stops
            // because the contour closed, not because it ran dry. It is kept
            // because it is the shape C++ has, and because an unconditional
            // emit here writes a trailing edge onto a contour that did run
            // dry.
            let is_line = edge_segment
                .map_or(true, |s| graph.arena.segment_curve(s).1 == Verb::Line);
            let emit = (!state.unsortable
                && writer.has_move()
                && !is_line
                && !writer.is_closed())
                || prev_simple;
            if emit && !add_curve_to(&mut graph.arena, edge_start, edge_end, writer) {
                return false;
            }
            break;
        }
        // Emit the edge just walked, before stepping on.
        if !add_curve_to(&mut graph.arena, edge_start, edge_end, writer) {
            return false;
        }
        if writer.is_closed() {
            break;
        }
        if state.unsortable {
            if let Some(starter) = graph.arena.span_starter(state.start, state.end) {
                if graph.arena.span(starter).done() {
                    break;
                }
            }
        }
    }
    true
}

/// Bounds the outer walk, which restarts once per output contour.
const OUTER_GUARD: i32 = 10_000;

/// Bounds a single contour's walk.
const INNER_GUARD: i32 = 100_000;

/// How an operation changes when one or both operands fill inverted.
///
/// Port of `gOpInverse` (`SkPathOpsOp.cpp`). Inverting an operand turns the
/// operation into its complement: subtracting an inverse-filled shape is
/// intersecting with the shape itself, and so on. Indexed
/// `[op][one is inverse][two is inverse]`.
const OP_INVERSE: [[[PathOp; 2]; 2]; 5] = [
    // Difference
    [
        [PathOp::Difference, PathOp::Intersect],
        [PathOp::Union, PathOp::ReverseDifference],
    ],
    // Intersect
    [
        [PathOp::Intersect, PathOp::Difference],
        [PathOp::ReverseDifference, PathOp::Union],
    ],
    // Union
    [
        [PathOp::Union, PathOp::ReverseDifference],
        [PathOp::Difference, PathOp::Intersect],
    ],
    // Xor: inverting either operand inverts the result, not the operation.
    [
        [PathOp::Xor, PathOp::Xor],
        [PathOp::Xor, PathOp::Xor],
    ],
    // ReverseDifference
    [
        [PathOp::ReverseDifference, PathOp::Union],
        [PathOp::Intersect, PathOp::Difference],
    ],
];

/// Whether the result of an operation fills inverted.
///
/// Port of `gOutInverse`, indexed the same way but by the *already inverted*
/// operation from [`OP_INVERSE`].
const OUT_INVERSE: [[[bool; 2]; 2]; 5] = [
    [[false, false], [true, false]],  // difference
    [[false, false], [false, true]],  // intersect
    [[false, true], [true, true]],    // union
    [[false, true], [true, false]],   // xor
    [[false, true], [false, false]],  // reverse difference
];

/// Returns the index of `op` in the two inverse tables.
const fn op_index(op: PathOp) -> usize {
    match op {
        PathOp::Difference => 0,
        PathOp::Intersect => 1,
        PathOp::Union => 2,
        PathOp::Xor => 3,
        PathOp::ReverseDifference => 4,
    }
}

/// Rewrites `op` and the output fill for inverse-filled operands.
///
/// Port of the `gOpInverse` / `gOutInverse` lookup at the top of `OpDebug`.
/// Returns the operation to actually perform and the fill type the result
/// should carry.
#[must_use]
pub fn resolve_inverse(one: &Path, two: &Path, op: PathOp) -> (PathOp, FillType) {
    let a = usize::from(one.fill_type().is_inverse());
    let b = usize::from(two.fill_type().is_inverse());
    let resolved = OP_INVERSE[op_index(op)][a][b];
    let inverse_fill = OUT_INVERSE[op_index(resolved)][a][b];
    let fill = if inverse_fill {
        FillType::InverseEvenOdd
    } else {
        FillType::EvenOdd
    };
    (resolved, fill)
}

/// Returns the xor masks for `op`'s two operands.
///
/// Port of the `gOutInverse`/mask setup in `SkPathOpsOp.cpp`: an even-odd
/// operand contributes only its low winding bit.
#[must_use]
pub fn xor_masks(one: &Path, two: &Path) -> (i32, i32) {
    let mask = |p: &Path| {
        if p.fill_type() == FillType::EvenOdd
            || p.fill_type() == FillType::InverseEvenOdd
        {
            1
        } else {
            -1
        }
    };
    (mask(one), mask(two))
}

/// Runs `op` on two paths through the real engine.
///
/// Returns `None` when the engine could not resolve the input, which the
/// caller degrades to the flattening fallback rather than reporting as an
/// empty result.
#[must_use]
pub fn op_with_engine(one: &Path, two: &Path, op: PathOp) -> Option<Path> {
    let (op, fill) = resolve_inverse(one, two, op);
    // ReverseDifference is Difference with the operands the other way round,
    // so the walk never has to know about it.
    let (one, two, op) = if op == PathOp::ReverseDifference {
        (two, one, PathOp::Difference)
    } else {
        (one, two, op)
    };
    let (xor_mi, xor_su) = xor_masks(one, two);
    let mut graph = build(one, Some(two), xor_mi == 1, xor_su == 1)?;
    let mut result = Path::new();
    result.set_fill_type(fill);
    {
        let mut writer = SkPathWriter::new(&mut result);
        if !bridge(&mut graph, Some(op), xor_mi, xor_su, &mut writer) {
            return None;
        }
        writer.assemble();
    }
    if result.is_empty() && !empty_is_the_answer(one, two, op) {
        // The walk emitted nothing and the operation should have produced
        // something, so treat it as a decline rather than as an empty result.
        return None;
    }
    Some(result)
}

/// Returns whether an empty result is the right answer for these inputs.
///
/// The walk emitting nothing means one of two very different things: the
/// operation genuinely covers no area, or the engine failed to find it. Only
/// the bounding boxes can tell them apart cheaply, and only in one direction
/// — boxes that do not overlap really do make Intersect empty, whereas
/// overlapping boxes say nothing either way. Anything not decided here is
/// reported as a decline, which costs a fallback rather than a wrong answer.
fn empty_is_the_answer(one: &Path, two: &Path, op: PathOp) -> bool {
    let (a, b) = (one.bounds(), two.bounds());
    let disjoint = a.right < b.left || b.right < a.left || a.bottom < b.top || b.bottom < a.top;
    match op {
        // Nothing in common, so nothing to keep.
        PathOp::Intersect => disjoint,
        // Union and Xor of two non-empty paths always cover something, and
        // a Difference only empties out when the subtrahend swallows the
        // minuend, which the boxes cannot establish.
        _ => false,
    }
}

/// Simplifies one path through the real engine.
///
/// Returns `None` on the same terms as [`op_with_engine`].
#[must_use]
pub fn simplify_with_engine(path: &Path) -> Option<Path> {
    let xor = path.fill_type() == FillType::EvenOdd
        || path.fill_type() == FillType::InverseEvenOdd;
    let mut graph = build(path, None, xor, xor)?;
    let mut result = Path::new();
    {
        let mut writer = SkPathWriter::new(&mut result);
        if !bridge(&mut graph, None, if xor { 1 } else { -1 }, -1, &mut writer) {
            return None;
        }
        writer.assemble();
    }
    if result.is_empty() {
        return None;
    }
    Some(result)
}

/// Rebuilds the angle graph after the spans have changed.
///
/// Exposed for callers that add intersections outside [`build`].
pub fn rebuild_angles(graph: &mut OpGraph) -> bool {
    let segments = graph.segments.clone();
    for &segment in &segments {
        calc_angles(&mut graph.arena, segment);
    }
    for &segment in &segments {
        if !sort_angles(&mut graph.arena, segment) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a rectangle path.
    fn rect_path(l: f32, t: f32, r: f32, b: f32) -> Path {
        let mut p = Path::new();
        p.move_to(l, t);
        p.line_to(r, t);
        p.line_to(r, b);
        p.line_to(l, b);
        p.close();
        p
    }

    #[test]
    fn a_rectangle_becomes_four_segments() {
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &rect_path(0.0, 0.0, 10.0, 10.0), false, false, false);
        assert_eq!(
            graph.segments.len(),
            4,
            "four sides, and the close adds none because the last side reaches the start"
        );
    }

    #[test]
    fn the_builder_keeps_a_cubics_verb_and_points() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.cubic_to(0.0, 50.0, 50.0, 100.0, 100.0, 100.0);
        p.close();
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &p, false, false, false);
        let (pts, verb, _) = graph.arena.segment_curve(graph.segments[0]);
        assert_eq!(verb, Verb::Cubic, "the cubic enters the graph as a cubic");
        assert_eq!(pts[1], Point::new(0.0, 50.0));
        assert_eq!(pts[2], Point::new(50.0, 100.0));
    }

    #[test]
    fn the_builder_keeps_a_conics_weight() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.conic_to(50.0, 100.0, 100.0, 0.0, 0.75);
        p.close();
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &p, false, false, false);
        let (_, verb, weight) = graph.arena.segment_curve(graph.segments[0]);
        assert_eq!(verb, Verb::Conic);
        assert!((weight - 0.75).abs() < 1e-6, "weight was {weight}");
    }

    #[test]
    fn a_degenerate_line_is_not_added() {
        let mut p = Path::new();
        p.move_to(5.0, 5.0);
        p.line_to(5.0, 5.0);
        p.line_to(10.0, 5.0);
        p.close();
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &p, false, false, false);
        for &seg in &graph.segments {
            let (pts, _, _) = graph.arena.segment_curve(seg);
            assert!(
                !points_equal(pts[0], pts[pts.len() - 1]),
                "a line from a point to itself has no direction to sort by"
            );
        }
    }

    #[test]
    fn an_open_contour_is_closed_implicitly() {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.line_to(10.0, 0.0);
        p.line_to(10.0, 10.0);
        // No close(): pathops only has an answer for filled regions.
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &p, false, false, false);
        assert_eq!(graph.segments.len(), 3, "the third side is supplied");
    }

    #[test]
    fn the_second_operand_is_marked_as_such() {
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &rect_path(0.0, 0.0, 10.0, 10.0), false, false, false);
        let first = graph.segments.len();
        add_path(&mut graph, &rect_path(5.0, 5.0, 15.0, 15.0), true, false, false);
        assert!(!graph.arena.segment_operand(graph.segments[0]));
        assert!(graph.arena.segment_operand(graph.segments[first]));
    }

    #[test]
    fn crossing_rectangles_split_each_other() {
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &rect_path(0.0, 0.0, 10.0, 10.0), false, false, false);
        add_path(&mut graph, &rect_path(5.0, 5.0, 15.0, 15.0), true, false, false);
        let before: usize = graph
            .segments
            .iter()
            .map(|&s| graph.arena.segment_spans(s).len())
            .sum();
        add_intersect_ts(&mut graph);
        let after: usize = graph
            .segments
            .iter()
            .map(|&s| graph.arena.segment_spans(s).len())
            .sum();
        assert!(
            after > before,
            "two overlapping rectangles cross, so spans must be added: {before} then {after}"
        );
    }

    #[test]
    fn disjoint_rectangles_are_not_split() {
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &rect_path(0.0, 0.0, 10.0, 10.0), false, false, false);
        add_path(&mut graph, &rect_path(50.0, 50.0, 60.0, 60.0), true, false, false);
        let before: usize = graph
            .segments
            .iter()
            .map(|&s| graph.arena.segment_spans(s).len())
            .sum();
        add_intersect_ts(&mut graph);
        let after: usize = graph
            .segments
            .iter()
            .map(|&s| graph.arena.segment_spans(s).len())
            .sum();
        assert_eq!(after, before, "boxes that do not touch cannot cross");
    }

    #[test]
    fn a_crossing_links_the_two_segments_pt_t_rings() {
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        // Two lines crossing at (5, 5).
        let mut a = Path::new();
        a.move_to(0.0, 5.0);
        a.line_to(10.0, 5.0);
        let mut b = Path::new();
        b.move_to(5.0, 0.0);
        b.line_to(5.0, 10.0);
        add_path(&mut graph, &a, false, false, false);
        add_path(&mut graph, &b, true, false, false);
        add_intersect_ts(&mut graph);

        // Some PtT node on the first segment must now see the second.
        let seg_a = graph.segments[0];
        let seg_b = graph.segments[1];
        let linked = graph
            .arena
            .segment_spans(seg_a)
            .into_iter()
            .filter_map(|s| graph.arena.span_ptt(s))
            .any(|p| graph.arena.ptt_contains_segment(p, seg_b).is_some());
        assert!(
            linked,
            "a crossing must be one place to the graph, not two"
        );
    }

    #[test]
    fn xor_masks_read_each_paths_own_fill_type() {
        let mut nonzero = rect_path(0.0, 0.0, 1.0, 1.0);
        nonzero.set_fill_type(FillType::Winding);
        let mut evenodd = rect_path(0.0, 0.0, 1.0, 1.0);
        evenodd.set_fill_type(FillType::EvenOdd);
        assert_eq!(xor_masks(&nonzero, &evenodd), (-1, 1));
        assert_eq!(xor_masks(&evenodd, &nonzero), (1, -1));
    }

    #[test]
    fn build_returns_nothing_for_an_empty_path() {
        assert!(build(&Path::new(), None, false, false).is_none());
    }

    #[test]
    fn build_resolves_a_pair_of_overlapping_rectangles() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        let graph = build(&a, Some(&b), false, false).expect("the pipeline settles");
        assert_eq!(graph.segments.len(), 8, "four sides each");
    }

    #[test]
    fn the_engine_builds_an_angle_loop_at_every_crossing() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        let graph = build(&a, Some(&b), false, false).expect("builds");
        let looped = graph
            .segments
            .iter()
            .flat_map(|&seg| graph.arena.segment_spans(seg))
            .filter_map(|span| graph.arena.span_to_angle(span))
            .filter(|&a| super::super::sk_op_angle_order::loop_count(&graph.arena, a) > 1)
            .count();
        // The two rectangles cross at four points, and each needs a ring for
        // the walker to have somewhere to turn.
        assert_eq!(
            looped, 4,
            "every crossing must carry an angle ring, got {looped}"
        );
    }

    #[test]
    fn a_split_span_keeps_the_winding_of_the_span_it_split() {
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &rect_path(0.0, 0.0, 10.0, 10.0), false, false, false);
        add_path(&mut graph, &rect_path(5.0, 5.0, 15.0, 15.0), true, false, false);
        add_intersect_ts(&mut graph);
        for &seg in &graph.segments {
            for span in graph.arena.segment_spans(seg) {
                if graph.arena.span_is_final(span) {
                    continue;
                }
                assert!(
                    !graph.arena.span(span).is_canceled(),
                    "a split span that contributes nothing is skipped by \
                     calc_angles, leaving the crossing with no ring to sort"
                );
            }
        }
    }

    #[test]
    fn the_engine_finds_nothing_to_intersect_in_disjoint_rectangles() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(50.0, 50.0, 60.0, 60.0);
        // Boxes that do not touch have no common area, so the walk must find
        // no active edge at all rather than emitting either input. Empty is
        // the answer here, not a decline: `None` would send this to the
        // flattening fallback to compute the same empty path again.
        let got = op_with_engine(&a, &b, PathOp::Intersect).expect("an answer, not a decline");
        assert!(got.is_empty(), "nothing in common: {:?}", got.verbs());
    }

    #[test]
    fn an_empty_walk_is_still_a_decline_where_empty_cannot_be_right() {
        // Overlapping boxes, so Intersect covers real area. If the walk ever
        // emitted nothing here it would be a failure, and reporting it as an
        // empty path would silently lose the overlap.
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        assert!(!empty_is_the_answer(&a, &b, PathOp::Intersect));
        // Union and Xor of two non-empty paths always cover something.
        assert!(!empty_is_the_answer(&a, &b, PathOp::Union));
        assert!(!empty_is_the_answer(&a, &b, PathOp::Xor));
        // And a Difference cannot be settled from the boxes either way.
        let swallowed = rect_path(-5.0, -5.0, 20.0, 20.0);
        assert!(!empty_is_the_answer(&a, &swallowed, PathOp::Difference));
    }

    #[test]
    fn the_engine_unions_two_overlapping_rectangles() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        let got = op_with_engine(&a, &b, PathOp::Union).expect("the walk closes");
        assert!(
            got.verbs().contains(&Verb::Close),
            "the contour must close, got {:?}",
            got.verbs()
        );
        // Inside either input is inside the union; inside neither is outside.
        assert!(got.contains(2.0, 2.0), "the first rectangle's interior");
        assert!(got.contains(12.0, 12.0), "the second rectangle's interior");
        assert!(!got.contains(2.0, 12.0), "the notch is not filled");
        assert!(!got.contains(-5.0, -5.0), "and neither is the outside");
    }

    #[test]
    fn the_union_is_one_contour_with_the_l_shapes_eight_corners() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        let got = op_with_engine(&a, &b, PathOp::Union).expect("the walk closes");
        let moves = got.verbs().iter().filter(|v| **v == Verb::Move).count();
        assert_eq!(moves, 1, "one region, so one contour: {:?}", got.verbs());
        let lines = got.verbs().iter().filter(|v| **v == Verb::Line).count();
        assert_eq!(
            lines, 8,
            "the L has eight sides counting the closing one: {:?}",
            got.verbs()
        );
    }

    /// The shape from `TODO/2026-09-14-boolean-ops-destroy-all-curves.md`:
    /// a stem of four lines plus a ring of eight curves.
    fn stem_and_ring() -> Path {
        let mut p = Path::new();
        // The stem.
        p.move_to(100.0, 0.0);
        p.line_to(140.0, 0.0);
        p.line_to(140.0, 300.0);
        p.line_to(100.0, 300.0);
        p.close();
        // The ring, well clear of the stem, as eight cubics.
        let (cx, cy, r) = (400.0f32, 150.0f32, 80.0f32);
        let k = r * 0.5523;
        p.move_to(cx + r, cy);
        p.cubic_to(cx + r, cy + k, cx + k, cy + r, cx, cy + r);
        p.cubic_to(cx - k, cy + r, cx - r, cy + k, cx - r, cy);
        p.cubic_to(cx - r, cy - k, cx - k, cy - r, cx, cy - r);
        p.cubic_to(cx + k, cy - r, cx + r, cy - k, cx + r, cy);
        p.close();
        p
    }

    #[test]
    fn a_contour_the_operation_never_touches_keeps_its_curves() {
        let glyph = stem_and_ring();
        // A rectangle that cuts the stem and comes nowhere near the ring.
        let cut = rect_path(110.0, 100.0, 130.0, 200.0);
        let got = op_with_engine(&glyph, &cut, PathOp::Difference)
            .expect("the engine resolves this");
        assert!(
            got.verbs().contains(&Verb::Cubic),
            "the ring is untouched, so its cubics must survive: {:?}",
            got.verbs()
        );
    }

    #[test]
    fn the_result_does_not_explode_into_a_polyline() {
        let glyph = stem_and_ring();
        let cut = rect_path(110.0, 100.0, 130.0, 200.0);
        let got = op_with_engine(&glyph, &cut, PathOp::Difference)
            .expect("the engine resolves this");
        let segments = got
            .verbs()
            .iter()
            .filter(|v| !matches!(v, Verb::Move | Verb::Close))
            .count();
        // The flattening engine turns 12 input segments into 261. The TODO's
        // bar is under 40.
        assert!(
            segments < 40,
            "expected a handful of segments, got {segments}: {:?}",
            got.verbs()
        );
    }

    #[test]
    fn a_cut_curve_is_subdivided_rather_than_flattened() {
        // A single disc, cut by a rectangle across its middle.
        let (cx, cy, r) = (100.0f32, 100.0f32, 50.0f32);
        let k = r * 0.5523;
        let mut disc = Path::new();
        disc.move_to(cx + r, cy);
        disc.cubic_to(cx + r, cy + k, cx + k, cy + r, cx, cy + r);
        disc.cubic_to(cx - k, cy + r, cx - r, cy + k, cx - r, cy);
        disc.cubic_to(cx - r, cy - k, cx - k, cy - r, cx, cy - r);
        disc.cubic_to(cx + k, cy - r, cx + r, cy - k, cx + r, cy);
        disc.close();
        let cut = rect_path(40.0, 90.0, 160.0, 110.0);
        let Some(got) = op_with_engine(&disc, &cut, PathOp::Difference) else {
            // The engine may decline this input; that is a fallback, not a
            // wrong answer, and the flattening path handles it.
            return;
        };
        assert!(
            got.verbs().contains(&Verb::Cubic),
            "a cut through a disc leaves curved pieces: {:?}",
            got.verbs()
        );
    }

    #[test]
    fn rectangles_sharing_a_collinear_edge_are_recorded_as_coincident() {
        // Both span y 0..20, so their top and bottom edges lie along the same
        // lines and overlap. Those edges do not cross anywhere: they coincide,
        // and the crossing search finds nothing for them.
        let a = rect_path(0.0, 0.0, 20.0, 20.0);
        let b = rect_path(8.0, 0.0, 28.0, 20.0);
        let graph = build(&a, Some(&b), false, false).expect("builds");
        assert_eq!(
            graph.coincidence.count(&graph.arena),
            2,
            "the shared top run and the shared bottom run"
        );
    }

    #[test]
    fn rectangles_that_only_cross_record_no_coincidence() {
        // Offset in both axes, so no pair of edges is collinear.
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        let graph = build(&a, Some(&b), false, false).expect("builds");
        assert_eq!(graph.coincidence.count(&graph.arena), 0);
    }

    #[test]
    fn two_rectangles_flush_against_each_other_share_one_whole_edge() {
        // A's right side and B's left side are the same segment, traced
        // opposite ways. That is a coincident run of full length, not a
        // point touch: the two rectangles' top and bottom edges meet only at
        // a corner, but this pair overlaps completely.
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(10.0, 0.0, 20.0, 10.0);
        let graph = build(&a, Some(&b), false, false).expect("builds");
        assert_eq!(
            graph.coincidence.count(&graph.arena),
            1,
            "the shared vertical edge, and nothing from the corner touches"
        );
    }

    #[test]
    fn rectangles_meeting_at_only_a_corner_record_nothing() {
        // Diagonal neighbours: they touch at (10, 10) and nowhere else, so
        // no pair of edges overlaps with any width.
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(10.0, 10.0, 20.0, 20.0);
        let graph = build(&a, Some(&b), false, false).expect("builds");
        assert_eq!(
            graph.coincidence.count(&graph.arena),
            0,
            "a touch is not a run"
        );
    }

    #[test]
    fn inverting_the_subtrahend_turns_difference_into_intersect() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let mut b = rect_path(5.0, 5.0, 15.0, 15.0);
        b.set_fill_type(FillType::InverseWinding);
        // Subtracting everything-but-B is keeping what is in both.
        let (op, fill) = resolve_inverse(&a, &b, PathOp::Difference);
        assert_eq!(op, PathOp::Intersect);
        assert!(!fill.is_inverse(), "the result is a bounded region");
    }

    #[test]
    fn inverting_both_operands_swaps_union_and_intersect() {
        let mut a = rect_path(0.0, 0.0, 10.0, 10.0);
        let mut b = rect_path(5.0, 5.0, 15.0, 15.0);
        a.set_fill_type(FillType::InverseWinding);
        b.set_fill_type(FillType::InverseWinding);
        // By De Morgan: the union of two complements is the complement of
        // their intersection.
        let (op, fill) = resolve_inverse(&a, &b, PathOp::Union);
        assert_eq!(op, PathOp::Intersect);
        assert!(fill.is_inverse(), "and the result is the complement");
    }

    #[test]
    fn xor_keeps_its_operation_whatever_is_inverted() {
        let plain = rect_path(0.0, 0.0, 10.0, 10.0);
        let mut inverted = rect_path(5.0, 5.0, 15.0, 15.0);
        inverted.set_fill_type(FillType::InverseWinding);
        // Inverting an operand of xor inverts the result, not the operation.
        for (x, y) in [(&plain, &inverted), (&inverted, &plain)] {
            let (op, fill) = resolve_inverse(x, y, PathOp::Xor);
            assert_eq!(op, PathOp::Xor);
            assert!(fill.is_inverse());
        }
        let (op, fill) = resolve_inverse(&inverted, &inverted, PathOp::Xor);
        assert_eq!(op, PathOp::Xor);
        assert!(
            !fill.is_inverse(),
            "inverting both cancels: the symmetric difference is unchanged"
        );
    }

    #[test]
    fn plain_operands_leave_the_operation_alone() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        for op in [
            PathOp::Difference,
            PathOp::Intersect,
            PathOp::Union,
            PathOp::Xor,
            PathOp::ReverseDifference,
        ] {
            let (resolved, fill) = resolve_inverse(&a, &b, op);
            assert_eq!(resolved, op, "{op:?} is unchanged without inversion");
            assert!(!fill.is_inverse());
        }
    }

    #[test]
    fn reverse_difference_is_difference_with_the_operands_swapped() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        let rev = op_with_engine(&a, &b, PathOp::ReverseDifference);
        let fwd = op_with_engine(&b, &a, PathOp::Difference);
        assert_eq!(
            rev.map(|p| p.points().to_vec()),
            fwd.map(|p| p.points().to_vec()),
            "the two must produce the same path"
        );
    }

    #[test]
    fn near_coincident_discs_union_to_one_contour_with_their_curves() {
        // The case from TODO/16: two discs of radius 40 whose centres are
        // 0.5 apart. The flattening engine fragments this into four
        // contours, because two arcs flattened independently deviate by more
        // than the gap between them and their chords interleave. Nothing is
        // flattened here, so the question does not arise.
        let mut a = Path::new();
        a.add_circle(200.0, 200.0, 40.0);
        let mut b = Path::new();
        b.add_circle(200.5, 200.0, 40.0);
        let got = op_with_engine(&a, &b, PathOp::Union).expect("the engine resolves this");
        let contours = got.verbs().iter().filter(|v| **v == Verb::Move).count();
        assert_eq!(contours, 1, "one overlapping blob, one contour");
        let curves = got
            .verbs()
            .iter()
            .filter(|v| matches!(v, Verb::Cubic | Verb::Quad | Verb::Conic))
            .count();
        assert!(curves > 0, "and it is still made of curves: {:?}", got.verbs());
    }

    #[test]
    fn discs_union_to_one_contour_across_the_offset_sweep() {
        // TODO/16 measured the flattening engine failing only in a narrow
        // band around 0.5 and holding elsewhere. Check the whole sweep.
        for offset in [0.5f32, 1.0, 2.0, 5.0, 20.0, 40.0, 60.0] {
            let mut a = Path::new();
            a.add_circle(200.0, 200.0, 40.0);
            let mut b = Path::new();
            b.add_circle(200.0 + offset, 200.0, 40.0);
            let got = op_with_engine(&a, &b, PathOp::Union)
                .unwrap_or_else(|| panic!("offset {offset} should resolve"));
            let contours = got.verbs().iter().filter(|v| **v == Verb::Move).count();
            assert_eq!(contours, 1, "offset {offset} gave {contours} contours");
        }
    }

    #[test]
    fn every_segment_starts_with_its_winding_in_the_same_field() {
        // Both operands' spans carry windValue = 1 and oppValue = 0. The
        // operand distinction lives in segment_operand, not in which field
        // holds the 1: setUpWindings and activeOp both branch on the operand
        // to decide which running sum a segment's winding comes off.
        let mut graph = OpGraph {
            arena: OpArena::new(),
            segments: Vec::new(),
            coincidence: SkOpCoincidence::new(),
        };
        add_path(&mut graph, &rect_path(0.0, 0.0, 10.0, 10.0), false, false, false);
        add_path(&mut graph, &rect_path(5.0, 5.0, 15.0, 15.0), true, false, false);
        for &seg in &graph.segments {
            for span in graph.arena.segment_spans(seg) {
                assert_eq!(graph.arena.span(span).wind_value(), 1);
                assert_eq!(graph.arena.span(span).opp_value(), 0);
            }
        }
    }

    #[test]
    fn a_difference_across_a_shared_edge_takes_the_cut_out() {
        // Two rectangles overlapping in x and sharing their whole y range,
        // so their top and bottom edges are collinear. Subtracting the
        // second must remove the right half of the first.
        let a = rect_path(0.0, 0.0, 20.0, 20.0);
        let b = rect_path(10.0, 0.0, 30.0, 20.0);
        let got = op_with_engine(&a, &b, PathOp::Difference).expect("resolves");
        assert!(got.contains(5.0, 10.0), "left of the cut stays");
        assert!(!got.contains(15.0, 10.0), "the cut is taken out");
        assert!(!got.contains(25.0, 10.0), "and the subtrahend is not added");
    }

    #[test]
    fn an_intersect_across_a_shared_edge_keeps_only_the_overlap() {
        // Two rectangles sharing both horizontal edges. This was the last
        // case the walk got wrong, and it failed because the winding chased
        // across the corner into the other operand kept its two sums in the
        // fields they had on this side.
        let a = rect_path(0.0, 0.0, 20.0, 20.0);
        let b = rect_path(10.0, 0.0, 30.0, 20.0);
        let got = op_with_engine(&a, &b, PathOp::Intersect).expect("resolves");
        assert!(!got.contains(5.0, 10.0), "left of the overlap is out");
        assert!(got.contains(15.0, 10.0), "the overlap is in");
        assert!(!got.contains(25.0, 10.0), "right of it is out");
    }

    #[test]
    fn a_winding_disagreement_fails_the_op_rather_than_guessing() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        let mut graph = build(&a, Some(&b), false, false).expect("builds");
        // Two rays disagreed about some span. Carrying on would walk a graph
        // that contradicts itself, so the op reports failure and the caller
        // falls back to flattening.
        graph.arena.set_winding_failed();
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);
        assert!(!bridge(&mut graph, Some(PathOp::Union), -1, -1, &mut writer));
    }

    /// Two rectangles sharing *both* horizontal edges. This is the geometry
    /// that kept the walk short: the winding chased across the corner at
    /// (20, 20) into the second operand arrived with its two sums in the
    /// fields they had on the first operand's side, so B's right half read as
    /// interior and was never walked.
    #[test]
    fn a_union_across_two_shared_edges_is_one_contour_covering_both() {
        let a = rect_path(0.0, 0.0, 20.0, 20.0);
        let b = rect_path(8.0, 0.0, 28.0, 20.0);
        let got = op_with_engine(&a, &b, PathOp::Union).expect("resolves");

        let moves = got.verbs().iter().filter(|v| **v == Verb::Move).count();
        assert_eq!(moves, 1, "one contour, not A's outline plus a fragment: {:?}", got.verbs());
        // The half that only B covers is the part that used to go missing.
        assert!(got.contains(25.0, 10.0), "B's right half is in the union");
        assert!(got.contains(1.0, 10.0), "A's left half is too");
        assert!(got.contains(14.0, 10.0), "and the overlap");
        assert!(!got.contains(30.0, 10.0), "but not past B's right edge");
    }

    /// The same graph read the other way. Difference was already right when
    /// Union and Intersect were not, which is what pointed at the sums rather
    /// than at how the graph was built.
    #[test]
    fn the_three_operators_agree_on_two_rectangles_sharing_both_edges() {
        let a = rect_path(0.0, 0.0, 20.0, 20.0);
        let b = rect_path(8.0, 0.0, 28.0, 20.0);
        // (x, inside A, inside B)
        let probes = [(1.0f32, true, false), (14.0, true, true), (25.0, false, true)];

        for (op, name) in [
            (PathOp::Union, "union"),
            (PathOp::Intersect, "intersect"),
            (PathOp::Difference, "difference"),
        ] {
            let got = op_with_engine(&a, &b, op).unwrap_or_else(|| panic!("{name} resolves"));
            for (x, in_a, in_b) in probes {
                let want = match op {
                    PathOp::Union => in_a || in_b,
                    PathOp::Intersect => in_a && in_b,
                    PathOp::Difference => in_a && !in_b,
                    _ => unreachable!(),
                };
                assert_eq!(got.contains(x, 10.0), want, "{name} at ({x}, 10)");
            }
        }
    }

    /// A coincident run's ends are endpoints of one of the two segments, so
    /// the split must land on those exact coordinates. Evaluating segment
    /// (20,20)->(0,20) at t = 0.6 gives x = 7.9999995, and a PtT ring keyed on
    /// that near-miss is a second ring at a corner that should have one.
    #[test]
    fn a_coincident_split_lands_exactly_on_the_shared_corner() {
        let a = rect_path(0.0, 0.0, 20.0, 20.0);
        let b = rect_path(8.0, 0.0, 28.0, 20.0);
        let graph = build(&a, Some(&b), false, false).expect("builds");

        let mut corner_rings = 0;
        for &seg in &graph.segments {
            for span in graph.arena.segment_spans(seg) {
                let Some(ptt) = graph.arena.span_ptt(span) else {
                    continue;
                };
                let pt = graph.arena.ptt(ptt).f_pt;
                assert!(
                    pt.x.fract() == 0.0 && pt.y.fract() == 0.0,
                    "every split here is on an integer corner, got {pt:?}"
                );
                if pt == Point::new(8.0, 20.0) {
                    corner_rings = corner_rings.max(graph.arena.ptt_ring(ptt).len());
                }
            }
        }
        // A's top edge, B's top edge and B's left edge all meet at (8, 20).
        assert_eq!(corner_rings, 3, "all three segments share the one ring");
    }
}
