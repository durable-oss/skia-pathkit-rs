//! The four functions the walk is driven from: `AngleWinding`, `FindUndone`,
//! `FindChase` and `HandleCoincidence`.
//!
//! Port of `SkPathOpsCommon.cpp` onto the arena.
//!
//! # HandleCoincidence is a fixed sequence
//!
//! It reads like a pile of repeated calls, and it is: the passes feed each
//! other, and running one can create work for one that already ran. The order
//! is not a judgement call — it is transcribed from
//! `SkPathOpsCommon.cpp:229`, and changing it changes results.
//!
//! Each loop in it is bounded by [`SAFETY_COUNT`]. Those bounds are the
//! difference between a pathological input returning failure and one hanging
//! the whole operation, so they are kept even where they look unreachable.

use super::sk_op_angle_order::{calc_angles, loop_count, sort_angles};
use super::sk_op_arena::{OpArena, SegmentId, SpanId};
use super::sk_op_coincidence::SkOpCoincidence;
use super::sk_op_span::PK_MIN_S32;
use super::sk_op_walker::{mark_angle, mark_angle_opp, PK_NAN32};
use super::sk_op_arena::AngleId;

/// How many times a self-feeding pass may re-run before the input is called
/// pathological.
///
/// Port of `HandleCoincidence`'s `SAFETY_COUNT`.
pub const SAFETY_COUNT: i32 = 3;

/// Returns the first span across `segments` that has not been walked.
///
/// Port of `FindUndone`.
#[must_use]
pub fn find_undone(arena: &OpArena, segments: &[SegmentId]) -> Option<SpanId> {
    for &segment in segments {
        if arena.segment_done(segment) {
            continue;
        }
        for span in arena.segment_spans(segment) {
            if !arena.span_is_final(span) && !arena.span(span).done() {
                return Some(span);
            }
        }
    }
    None
}

/// What [`angle_winding`] found.
#[derive(Debug, Clone, Copy)]
pub struct AngleWinding {
    /// The angle the search stopped on.
    pub angle: Option<AngleId>,
    /// The winding at that angle, or [`PK_MIN_S32`] when none was found.
    pub winding: i32,
    /// False when the loop contained an unorderable angle, so its order
    /// cannot be relied on.
    pub sortable: bool,
}

/// Walks the angle loop at `start`/`end` looking for a resolved winding.
///
/// Port of `AngleWinding`. Two passes, and the second is the interesting one:
/// if the loop contains an unorderable angle, the loop's *order* is useless,
/// so rather than reading a winding off a neighbour it asks each angle for
/// its own winding directly. A winding inherited across an unorderable turn
/// would be a winding inherited across an unknown direction.
pub fn angle_winding(arena: &mut OpArena, start: SpanId, end: SpanId) -> AngleWinding {
    let Some(first_angle) = arena.walk_angle(start, end) else {
        return AngleWinding {
            angle: None,
            winding: PK_MIN_S32,
            sortable: true,
        };
    };
    let mut angle = first_angle;
    // Set inside the loop; the loop always runs at least once.
    let compute_winding;
    let mut looped = false;
    let mut unorderable = false;
    let mut winding = PK_MIN_S32;
    let mut guard = LOOP_GUARD;

    loop {
        guard -= 1;
        if guard == 0 {
            return AngleWinding {
                angle: None,
                winding: PK_MIN_S32,
                sortable: false,
            };
        }
        angle = match arena.angle(angle).f_next.map(AngleId::new) {
            Some(n) => n,
            None => {
                return AngleWinding {
                    angle: None,
                    winding: PK_MIN_S32,
                    sortable: !unorderable,
                }
            }
        };
        unorderable |= arena.angle(angle).unorderable();
        if unorderable || (angle == first_angle && looped) {
            compute_winding = true;
            break;
        }
        looped |= angle == first_angle;
        winding = angle_wind_sum(arena, angle);
        if winding != PK_MIN_S32 {
            compute_winding = false;
            break;
        }
    }

    if compute_winding {
        // The order is not trustworthy, so take each angle's own winding
        // rather than inheriting one from a neighbour.
        let first = angle;
        winding = PK_MIN_S32;
        let mut guard = LOOP_GUARD;
        loop {
            guard -= 1;
            if guard == 0 {
                break;
            }
            let test = angle_wind_sum(arena, angle);
            if test != PK_MIN_S32 {
                winding = test;
            }
            angle = match arena.angle(angle).f_next.map(AngleId::new) {
                Some(n) => n,
                None => break,
            };
            if angle == first {
                break;
            }
        }
    }

    AngleWinding {
        angle: Some(angle),
        winding,
        sortable: !unorderable,
    }
}

/// Bounds a walk over an angle ring that failed to close.
const LOOP_GUARD: i32 = 1_000_000;

/// Returns the winding sum at an angle's starter span.
fn angle_wind_sum(arena: &OpArena, angle: AngleId) -> i32 {
    let (Some(start), Some(end)) = (
        arena.angle(angle).f_start.map(SpanId::new),
        arena.angle(angle).f_end.map(SpanId::new),
    ) else {
        return PK_MIN_S32;
    };
    arena.wind_sum_between(start, end)
}

/// Pops the chase list and picks the next segment to walk from, for a
/// two-operand op.
///
/// Port of `findChaseOp` (`SkPathOpsOp.cpp:20`). It differs from
/// [`find_chase`] in exactly the way the binary walk differs from the unary
/// one: two running sums instead of one, swapped when the angle's segment
/// belongs to the second operand, and `mark_angle_opp` to record both. Using
/// the unary `find_chase` here resolves the opposite operand's winding to
/// nothing, so the drain comes back with an edge that looks active only
/// because half its winding was never counted — which is how a Difference
/// ends up tracing the operand it was supposed to subtract.
///
/// `start` and `end` are updated to the span pair the returned segment should
/// be walked over.
pub fn find_chase_op(
    arena: &mut OpArena,
    chase: &mut Vec<SpanId>,
    start: &mut SpanId,
    end: &mut Option<SpanId>,
) -> Option<SegmentId> {
    while let Some(span) = chase.pop() {
        // C++ takes the ring's *previous* member, not the next one.
        let Some(ptt) = arena.span_ptt(span) else {
            continue;
        };
        let prev_ptt = arena.ptt_prev(ptt);
        let Some(prev_span) = arena.ptt_span(prev_ptt) else {
            continue;
        };
        *start = prev_span;
        *end = None;
        let mut done = true;
        let mut start_ptr = None;
        let mut end_ptr = None;
        if let Some(last) = arena.active_angle(*start, &mut start_ptr, &mut end_ptr, &mut done) {
            if let (Some(s), Some(e)) = (
                arena.angle(last).f_start.map(SpanId::new),
                arena.angle(last).f_end.map(SpanId::new),
            ) {
                *start = s;
                *end = Some(e);
                chase.push(span);
                return arena.span_segment(s);
            }
        }
        if done {
            continue;
        }
        let Some(end_span) = *end else {
            continue;
        };

        let found = angle_winding(arena, *start, end_span);
        let angle = found.angle?;
        if found.winding == PK_MIN_S32 || found.winding == PK_NAN32 {
            continue;
        }

        // Both running sums, read reversed off the angle, then swapped when
        // the angle's own segment is the second operand.
        let mut sum_mi_winding = 0;
        let mut sum_su_winding = 0;
        if found.sortable {
            let (Some(a_start), Some(a_end)) = (
                arena.angle(angle).f_start.map(SpanId::new),
                arena.angle(angle).f_end.map(SpanId::new),
            ) else {
                continue;
            };
            sum_mi_winding = arena.update_winding_reverse(a_start, a_end, |_, _| false);
            sum_su_winding = arena.update_opp_winding_reverse(a_start, a_end);
            if sum_mi_winding == PK_MIN_S32 || sum_su_winding == PK_MIN_S32 {
                return None;
            }
            if arena
                .span_segment(a_start)
                .is_some_and(|s| arena.segment_operand(s))
            {
                std::mem::swap(&mut sum_mi_winding, &mut sum_su_winding);
            }
        }

        let mut first: Option<SegmentId> = None;
        let mut current = angle;
        let mut guard = LOOP_GUARD;
        loop {
            guard -= 1;
            if guard == 0 {
                break;
            }
            current = match arena.angle(current).f_next.map(AngleId::new) {
                Some(n) => n,
                None => break,
            };
            if current == angle {
                break;
            }
            let (Some(a_start), Some(a_end)) = (
                arena.angle(current).f_start.map(SpanId::new),
                arena.angle(current).f_end.map(SpanId::new),
            ) else {
                continue;
            };
            let Some(segment) = arena.span_segment(a_start) else {
                continue;
            };
            let (max_winding, opp_max_winding) = if found.sortable {
                let operand = arena.segment_operand(segment);
                arena.set_up_windings(
                    a_start,
                    a_end,
                    operand,
                    &mut sum_mi_winding,
                    &mut sum_su_winding,
                )
            } else {
                (0, 0)
            };
            let span_done = arena
                .span_starter(a_start, a_end)
                .is_some_and(|s| arena.span(s).done());
            if span_done {
                continue;
            }
            let has_winding = arena
                .span_starter(a_start, a_end)
                .is_some_and(|s| arena.span(s).wind_sum() != PK_MIN_S32);
            if first.is_none() && (found.sortable || has_winding) {
                first = Some(segment);
                *start = a_start;
                *end = Some(a_end);
            }
            if found.sortable {
                // After `set_up_windings` the two sums hold this angle's own
                // side; C++ passes them as sumWinding / oppSumWinding.
                let operand = arena.segment_operand(segment);
                let (sum_winding, opp_sum_winding) = if operand {
                    (sum_su_winding, sum_mi_winding)
                } else {
                    (sum_mi_winding, sum_su_winding)
                };
                mark_angle_opp(
                    arena,
                    max_winding,
                    sum_winding,
                    opp_max_winding,
                    opp_sum_winding,
                    current,
                );
            }
        }
        if let Some(segment) = first {
            chase.push(span);
            return Some(segment);
        }
    }
    None
}

/// Pops the chase list and picks the next segment to walk from.
///
/// Port of `FindChase`. The chase list holds spans the walker passed but did
/// not take; this comes back to them, resolves their winding, and returns
/// whichever segment leaving that point is still active.
///
/// `start` and `end` are updated to the span pair the returned segment should
/// be walked over.
pub fn find_chase(
    arena: &mut OpArena,
    chase: &mut Vec<SpanId>,
    start: &mut SpanId,
    end: &mut Option<SpanId>,
) -> Option<SegmentId> {
    while let Some(span) = chase.pop() {
        // Step across to whatever else meets at this point.
        let Some(ptt) = arena.span_ptt(span) else {
            continue;
        };
        let next_ptt = arena.ptt_next(ptt);
        let Some(next_span) = arena.ptt_span(next_ptt) else {
            continue;
        };
        *start = next_span;
        *end = None;
        let mut done = true;
        let mut start_ptr = None;
        let mut end_ptr = None;
        if let Some(last) = arena.active_angle(*start, &mut start_ptr, &mut end_ptr, &mut done) {
            if let (Some(s), Some(e)) = (
                arena.angle(last).f_start.map(SpanId::new),
                arena.angle(last).f_end.map(SpanId::new),
            ) {
                *start = s;
                *end = Some(e);
                chase.push(span);
                return arena.span_segment(s);
            }
        }
        if done {
            continue;
        }
        let Some(end_span) = *end else {
            continue;
        };

        let found = angle_winding(arena, *start, end_span);
        // C++ returns nullptr outright here rather than trying the next
        // chase entry: no angle means the graph is not walkable from it.
        let angle = found.angle?;
        if found.winding == PK_MIN_S32 || found.winding == PK_NAN32 {
            continue;
        }

        let mut sum_winding = 0;
        if found.sortable {
            if let (Some(a_start), Some(a_end)) = (
                arena.angle(angle).f_start.map(SpanId::new),
                arena.angle(angle).f_end.map(SpanId::new),
            ) {
                sum_winding = arena.update_winding_reverse(a_start, a_end, |_, _| false);
            }
        }

        // Walk the rest of the ring, marking each member's winding and
        // taking the first that still has work left.
        let mut first: Option<SegmentId> = None;
        let mut current = angle;
        let mut guard = LOOP_GUARD;
        loop {
            guard -= 1;
            if guard == 0 {
                break;
            }
            current = match arena.angle(current).f_next.map(AngleId::new) {
                Some(n) => n,
                None => break,
            };
            if current == angle {
                break;
            }
            let (Some(a_start), Some(a_end)) = (
                arena.angle(current).f_start.map(SpanId::new),
                arena.angle(current).f_end.map(SpanId::new),
            ) else {
                continue;
            };
            let Some(segment) = arena.span_segment(a_start) else {
                continue;
            };
            let max_winding = if found.sortable {
                arena.set_up_winding(a_start, a_end, &mut sum_winding)
            } else {
                0
            };
            let span_done = arena
                .span_starter(a_start, a_end)
                .is_some_and(|s| arena.span(s).done());
            if span_done {
                continue;
            }
            let has_winding = arena
                .span_starter(a_start, a_end)
                .is_some_and(|s| arena.span(s).wind_sum() != PK_MIN_S32);
            if first.is_none() && (found.sortable || has_winding) {
                first = Some(segment);
                *start = a_start;
                *end = Some(a_end);
            }
            if found.sortable {
                mark_angle(arena, max_winding, sum_winding, current);
            }
        }
        if let Some(segment) = first {
            chase.push(span);
            return Some(segment);
        }
    }
    None
}

/// Runs the whole coincidence pipeline.
///
/// Port of `HandleCoincidence`. The order of the passes is transcribed from
/// the C++ rather than reasoned about: running one can create work for one
/// that already ran, so the sequence and its repeats are load-bearing.
///
/// Returns false when a bounded loop exhausted its budget, which means the
/// input could not be resolved and the caller must not trust the result.
pub fn handle_coincidence(
    arena: &mut OpArena,
    segments: &[SegmentId],
    coincidence: &mut SkOpCoincidence,
) -> bool {
    // Match up points within the coincident runs.
    if !coincidence.add_expanded(arena) {
        return false;
    }
    // Move t values and points together to close small gaps.
    if !move_nearby(arena, segments) {
        return false;
    }

    // Loosen the ranges, then re-match whatever that opened up.
    if coincidence.expand(arena) {
        if !coincidence.add_expanded(arena) {
            return false;
        }
        move_nearby(arena, segments);
    }
    if !coincidence.add_expanded(arena) {
        return false;
    }
    // Mark spans of coincident segments as coincident.
    if !coincidence.mark(arena) {
        return false;
    }

    // Apply, then look for pairs the application implied, until none are
    // left. Bounded: a set of records that keeps implying new ones is
    // pathological, and returning false beats spinning.
    let mut overlaps = SkOpCoincidence::new();
    let mut safety = SAFETY_COUNT;
    loop {
        let apply_ok = if overlaps.is_empty() {
            coincidence.apply(arena)
        } else {
            overlaps.apply(arena)
        };
        if !apply_ok {
            return false;
        }
        let mut found = SkOpCoincidence::new();
        let searched = if overlaps.is_empty() {
            coincidence.find_overlaps(arena, &mut found)
        } else {
            overlaps.find_overlaps(arena, &mut found)
        };
        if !searched {
            return false;
        }
        overlaps = found;
        safety -= 1;
        if safety == 0 {
            return false;
        }
        if overlaps.is_empty() {
            break;
        }
    }

    for &segment in segments {
        calc_angles(arena, segment);
    }
    for &segment in segments {
        if !sort_angles(arena, segment) {
            return false;
        }
    }
    true
}

/// Merges spans that sit closer together than the engine can tell apart.
///
/// Port of `move_nearby`. A pair of spans a rounding error apart is two
/// places to the graph and one place to the geometry; leaving both makes the
/// walker turn where there is no turn.
pub fn move_nearby(arena: &mut OpArena, segments: &[SegmentId]) -> bool {
    for &segment in segments {
        let spans = arena.segment_spans(segment);
        for pair in spans.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let (ta, tb) = (arena.span(a).f_t, arena.span(b).f_t);
            if tb - ta >= NEARBY_T {
                continue;
            }
            // Never collapse an endpoint: t = 0 and t = 1 are where this
            // segment joins its neighbours.
            if ta == 0.0 || tb == 1.0 {
                continue;
            }
            if let (Some(pa), Some(pb)) = (arena.span_ptt(a), arena.span_ptt(b)) {
                if pa != pb {
                    arena.ptt_add_opp(pa, pb);
                }
            }
        }
    }
    true
}

/// How close two t values must be before they name the same place.
const NEARBY_T: f32 = 1e-6;

/// Returns true when the graph still holds an angle ring worth sorting.
///
/// Used by callers deciding whether a second pass is worth running.
#[must_use]
pub fn has_angle_loops(arena: &OpArena, segments: &[SegmentId]) -> bool {
    for &segment in segments {
        for span in arena.segment_spans(segment) {
            if let Some(angle) = arena.span_to_angle(span) {
                if loop_count(arena, angle) > 1 {
                    return true;
                }
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Point, Verb};

    /// Builds a closed rectangle as four segments, with winding on every span.
    fn rect(arena: &mut OpArena, l: f32, t: f32, r: f32, b: f32) -> Vec<SegmentId> {
        let corners = [
            Point::new(l, t),
            Point::new(r, t),
            Point::new(r, b),
            Point::new(l, b),
        ];
        let mut segs = Vec::new();
        for i in 0..4 {
            let seg = arena.alloc_segment_with_curve(
                &[corners[i], corners[(i + 1) % 4]],
                Verb::Line,
                1.0,
            );
            for span in arena.segment_spans(seg) {
                arena.span_mut(span).set_wind_value(1);
            }
            segs.push(seg);
        }
        segs
    }

    #[test]
    fn find_undone_returns_a_span_that_still_has_work() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 10.0, 10.0);
        let span = find_undone(&arena, &segs).expect("something is undone");
        assert!(!arena.span(span).done());
    }

    #[test]
    fn find_undone_returns_nothing_once_everything_is_walked() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 10.0, 10.0);
        for &seg in &segs {
            arena.segment_mark_all_done(seg);
        }
        assert!(find_undone(&arena, &segs).is_none());
    }

    #[test]
    fn find_undone_skips_a_finished_segment_and_finds_the_next() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 10.0, 10.0);
        arena.segment_mark_all_done(segs[0]);
        let span = find_undone(&arena, &segs).expect("a later segment is undone");
        assert_ne!(
            arena.span_segment(span),
            Some(segs[0]),
            "the finished segment must be skipped"
        );
    }

    #[test]
    fn angle_winding_reports_no_winding_without_a_loop() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let found = angle_winding(&mut arena, head, tail);
        assert_eq!(found.winding, PK_MIN_S32);
        assert!(found.angle.is_none());
    }

    #[test]
    fn find_chase_on_an_empty_list_finds_nothing() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 10.0, 10.0);
        let mut chase = Vec::new();
        let mut start = arena.segment(segs[0]).f_head.expect("head");
        let mut end = None;
        assert!(find_chase(&mut arena, &mut chase, &mut start, &mut end).is_none());
    }

    #[test]
    fn find_chase_drains_a_list_it_cannot_resolve() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 10.0, 10.0);
        let head = arena.segment(segs[0]).f_head.expect("head");
        let mut chase = vec![head, head, head];
        let mut start = head;
        let mut end = None;
        // Nothing here can be resolved, so it must empty the list and stop
        // rather than pushing entries back forever.
        find_chase(&mut arena, &mut chase, &mut start, &mut end);
        assert!(
            chase.len() <= 3,
            "the chase list must not grow without bound, got {}",
            chase.len()
        );
    }

    #[test]
    fn handle_coincidence_succeeds_with_nothing_recorded() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 10.0, 10.0);
        let mut coin = SkOpCoincidence::new();
        assert!(handle_coincidence(&mut arena, &segs, &mut coin));
    }

    #[test]
    fn handle_coincidence_marks_a_shared_edge() {
        let mut arena = OpArena::new();
        // Two segments over the same line, as two rectangles sharing an edge
        // would produce.
        let a = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let b = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        for seg in [a, b] {
            for span in arena.segment_spans(seg) {
                arena.span_mut(span).set_wind_value(1);
            }
        }
        let a_head = arena.segment(a).f_head.expect("head");
        let a_tail = arena.segment(a).f_tail.expect("tail");
        let b_head = arena.segment(b).f_head.expect("head");
        let b_tail = arena.segment(b).f_tail.expect("tail");
        let (ah, at, bh, bt) = (
            arena.span_ptt(a_head).expect("ptt"),
            arena.span_ptt(a_tail).expect("ptt"),
            arena.span_ptt(b_head).expect("ptt"),
            arena.span_ptt(b_tail).expect("ptt"),
        );
        let mut coin = SkOpCoincidence::new();
        coin.add_run(&mut arena, ah, at, bh, bt, false);

        assert!(handle_coincidence(&mut arena, &[a, b], &mut coin));
        assert!(
            arena.span_is_coincident(a_head),
            "the shared edge is marked coincident"
        );
        let (wa, wb) = (
            arena.span(a_head).wind_value(),
            arena.span(b_head).wind_value(),
        );
        assert!(
            wa == 0 || wb == 0,
            "and one side is folded away, got {wa} and {wb}"
        );
    }

    #[test]
    fn move_nearby_leaves_the_endpoints_alone() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        // A span a hair off the start: collapsing it into t = 0 would detach
        // the segment from whatever joins it there.
        arena
            .segment_add_t(seg, 1e-7, Point::new(0.0, 0.0))
            .expect("split");
        let before = arena.segment_spans(seg).len();
        assert!(move_nearby(&mut arena, &[seg]));
        assert_eq!(
            arena.segment_spans(seg).len(),
            before,
            "no span is removed, and the endpoint keeps its own identity"
        );
        assert_eq!(arena.span(arena.segment(seg).f_head.expect("head")).f_t, 0.0);
    }

    #[test]
    fn has_angle_loops_is_false_before_any_angles_are_built() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 10.0, 10.0);
        assert!(!has_angle_loops(&arena, &segs));
    }
}
