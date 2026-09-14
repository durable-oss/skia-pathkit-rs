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
use super::sk_op_common::{find_chase, handle_coincidence};
use super::sk_op_walker::{add_curve_to, find_next_op, find_next_winding, WalkState};
use super::sk_path_writer::SkPathWriter;
use super::sk_op_sortable_top::find_sortable_top;
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
    for (verb, pts, weight) in path.iter() {
        match verb {
            Verb::Move => {
                contour_start = Some(pts[0]);
                current = Some(pts[0]);
            }
            Verb::Close => {
                // Close the contour with the line back to its start, unless
                // the contour already ends there.
                if let (Some(start), Some(cur)) = (contour_start, current) {
                    if !points_equal(start, cur) {
                        push_segment(graph, &[cur, start], Verb::Line, 1.0, operand, xor, opp_xor);
                    }
                }
                current = contour_start;
            }
            Verb::Line | Verb::Quad | Verb::Conic | Verb::Cubic => {
                let count = verb.point_count();
                let w = weight.unwrap_or(1.0);
                push_segment(graph, &pts[..count], verb, w, operand, xor, opp_xor);
                current = Some(pts[count - 1]);
            }
        }
    }
    // An unclosed contour is closed implicitly, matching Skia: pathops only
    // has an answer for filled regions.
    if let (Some(start), Some(cur)) = (contour_start, current) {
        if !points_equal(start, cur) {
            push_segment(graph, &[cur, start], Verb::Line, 1.0, operand, xor, opp_xor);
        }
    }
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
) {
    if points_equal(pts[0], pts[pts.len() - 1]) && verb == Verb::Line {
        // A line from a point to itself has no direction to sort by.
        return;
    }
    let seg = graph.arena.alloc_segment_with_curve(pts, verb, weight);
    graph.arena.set_segment_operand(seg, operand);
    graph.arena.set_segment_xor(seg, xor, opp_xor);
    // Each segment starts contributing one to its own operand's winding.
    for span in graph.arena.segment_spans(seg) {
        if operand {
            graph.arena.span_mut(span).set_opp_value(1);
        } else {
            graph.arena.span_mut(span).set_wind_value(1);
        }
    }
    graph.segments.push(seg);
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
    let mut outer_guard = OUTER_GUARD;
    loop {
        outer_guard -= 1;
        if outer_guard == 0 {
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
            match find_chase(&mut graph.arena, &mut chase, &mut chase_start, &mut chase_end) {
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
                |_, _| false,
            )
        }
        None => graph
            .arena
            .active_winding(state.start, state.end, |_, _| false),
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
        // Emit the edge just walked. Doing this before the `next_segment`
        // check is what keeps a contour whose walk ends here from losing its
        // last edge.
        if !add_curve_to(&mut graph.arena, edge_start, edge_end, writer) {
            break;
        }
        if next_segment.is_none() || writer.is_closed() {
            break;
        }
    }
    true
}

/// Bounds the outer walk, which restarts once per output contour.
const OUTER_GUARD: i32 = 10_000;

/// Bounds a single contour's walk.
const INNER_GUARD: i32 = 100_000;

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
    let (xor_mi, xor_su) = xor_masks(one, two);
    let mut graph = build(one, Some(two), xor_mi == 1, xor_su == 1)?;
    let mut result = Path::new();
    {
        let mut writer = SkPathWriter::new(&mut result);
        if !bridge(&mut graph, Some(op), xor_mi, xor_su, &mut writer) {
            return None;
        }
        writer.assemble();
    }
    if result.is_empty() {
        return None;
    }
    Some(result)
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
        // no active edge at all rather than emitting either input.
        assert!(op_with_engine(&a, &b, PathOp::Intersect).is_none());
    }
}
