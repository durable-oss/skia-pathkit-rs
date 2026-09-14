//! Walking the sorted segment graph: winding transfer, the three `findNext`
//! variants, and emitting a walked edge as a curve.
//!
//! Port of the second half of `SkOpSegment.cpp` — `computeSum`,
//! `ComputeOneSum`, `markAngle`, `findNextWinding`, `findNextXor`,
//! `findNextOp` and `addCurveTo`.
//!
//! # Where the curves survive
//!
//! [`add_curve_to`] is the reason the engine preserves curves at all. It
//! subdivides the segment's own geometry between the two spans being walked
//! and emits the piece with its original verb, so a cubic that the operation
//! merely traverses comes out a cubic. The flattening engine in
//! [`boolean`](super::boolean) has no equivalent: its only output type is a
//! line.
//!
//! # The winding transfer
//!
//! [`compute_sum`] is the part that is easy to get subtly wrong. It walks the
//! angle ring looking for a member whose winding is already known and carries
//! that value to its neighbours, in both directions. Only *adjacent orderable*
//! angles may transfer: a value carried across an unorderable angle is a value
//! carried across an unknown turn.

use super::sk_op_angle::IncludeType;
use super::sk_op_angle_order::{last_marked, previous};
use super::sk_op_arena::{
    use_inner_winding, AngleId, ChaseState, OpArena, SegmentId, SpanId,
};
use super::sk_op_span::PK_MIN_S32;
use super::sk_path_writer::SkPathWriter;
use super::PathOp;
use crate::core::{Point, Verb};

/// Stands in for C++'s `PK_NaN32`, the "no winding could be computed" marker
/// `computeSum` returns.
pub const PK_NAN32: i32 = i32::MIN;

/// How far the chase list may grow before a walk is called pathological.
const CHASE_LIMIT: usize = 100_000;

/// Emits the walk from `start` to `end` into `path`, keeping its verb.
///
/// Port of `SkOpSegment::addCurveTo`. The piece between the two spans is
/// subdivided out of the segment's own control points, so a cubic emerges a
/// cubic; only a curve whose control points turn out collinear degrades to a
/// line, which is what `isCurve` decides.
///
/// Returns false when the span was already emitted, which is how a walk that
/// doubles back on itself is caught.
pub fn add_curve_to(
    arena: &mut OpArena,
    start: SpanId,
    end: SpanId,
    path: &mut SkPathWriter,
) -> bool {
    let Some(span_start) = arena.span_starter(start, end) else {
        return false;
    };
    if arena.span(span_start).already_added() {
        return false;
    }
    arena.span_mut(span_start).mark_added();

    let Some(segment) = arena.span_segment(start) else {
        return false;
    };
    let (_, verb, _) = arena.segment_curve(segment);
    let mut curve_part = super::sk_op_angle::CurveSweep::new();
    arena.span_sub_divide(start, end, &mut curve_part);
    curve_part.f_verb = verb;
    curve_part.set_curve_hull_sweep();

    let start_pt = span_point(arena, start);
    let end_pt = span_point(arena, end);
    path.deferred_move(start_pt);

    // A curve whose hull collapsed is emitted as the line it actually is.
    let emit = if curve_part.is_curve() { verb } else { Verb::Line };
    match emit {
        Verb::Quad => path.quad_to(to_point(curve_part.f_curve[1]), end_pt),
        Verb::Conic => path.conic_to(
            to_point(curve_part.f_curve[1]),
            end_pt,
            curve_part.f_weight as f32,
        ),
        Verb::Cubic => path.cubic_to(
            to_point(curve_part.f_curve[1]),
            to_point(curve_part.f_curve[2]),
            end_pt,
        ),
        // Line, and anything degenerate.
        _ => {
            if !path.deferred_line(end_pt) {
                return false;
            }
        }
    }
    true
}

/// Converts an f64 control point back to the path's f32 point type.
fn to_point(p: [f64; 2]) -> Point {
    Point::new(p[0] as f32, p[1] as f32)
}

/// Returns a span's point, preferring its PtT node's cached value.
fn span_point(arena: &OpArena, span: SpanId) -> Point {
    match arena.span(span).f_ptt {
        Some(id) => arena.ptt(super::sk_op_arena::PtTId::new(id)).f_pt,
        None => arena.span(span).f_pt,
    }
}

/// Returns the angle for walking `end` back to `start`, if one exists.
fn span_to_angle(arena: &OpArena, end: SpanId, start: SpanId) -> Option<AngleId> {
    arena.walk_angle(end, start)
}

/// Marks the angle's span pair with a winding value and chases it along.
///
/// Port of the unary `SkOpSegment::markAngle`. Returns the last span marked,
/// which the caller records on the angle so the chase walk can pick it up.
pub fn mark_angle(
    arena: &mut OpArena,
    max_winding: i32,
    sum_winding: i32,
    angle: AngleId,
) -> Option<Option<SpanId>> {
    let winding = if use_inner_winding(max_winding, sum_winding) {
        sum_winding
    } else {
        max_winding
    };
    let start = SpanId::new(arena.angle(angle).f_start?);
    let end = SpanId::new(arena.angle(angle).f_end?);
    arena
        .mark_and_chase_winding(start, end, winding)
        .map(|(_, last)| last)
}

/// Marks the angle's span pair with both operands' winding values.
///
/// Port of the binary `SkOpSegment::markAngle`. The opposite winding is only
/// narrowed when the two values actually differ, matching C++: narrowing an
/// equal pair would be a no-op that still costs the comparison.
pub fn mark_angle_opp(
    arena: &mut OpArena,
    max_winding: i32,
    sum_winding: i32,
    opp_max_winding: i32,
    opp_sum_winding: i32,
    angle: AngleId,
) -> Option<Option<SpanId>> {
    let winding = if use_inner_winding(max_winding, sum_winding) {
        sum_winding
    } else {
        max_winding
    };
    let opp_winding = if opp_max_winding != opp_sum_winding
        && use_inner_winding(opp_max_winding, opp_sum_winding)
    {
        opp_sum_winding
    } else {
        opp_max_winding
    };
    let start = SpanId::new(arena.angle(angle).f_start?);
    let end = SpanId::new(arena.angle(angle).f_end?);
    arena
        .mark_and_chase_winding_opp(start, end, winding, opp_winding)
        .map(|(_, last)| last)
}

/// Carries a known winding from `base_angle` forward onto `next_angle`.
///
/// Port of `SkOpSegment::ComputeOneSum`. The base's winding is read
/// *reversed*, because the transfer runs against the base angle's own
/// direction of travel.
fn compute_one_sum(
    arena: &mut OpArena,
    base_angle: AngleId,
    next_angle: AngleId,
    include_type: IncludeType,
) -> bool {
    let binary = include_type >= IncludeType::BinarySingle;
    let (Some(base_start), Some(base_end)) = (
        arena.angle(base_angle).f_start.map(SpanId::new),
        arena.angle(base_angle).f_end.map(SpanId::new),
    ) else {
        return false;
    };
    let mut sum_mi = arena.update_winding_reverse(base_start, base_end, |_, _| false);
    let mut sum_su = PK_MIN_S32;
    if binary {
        sum_su = arena.update_opp_winding_reverse(base_start, base_end);
        if segment_operand(arena, base_angle) {
            std::mem::swap(&mut sum_mi, &mut sum_su);
        }
    }
    let (Some(next_start), Some(next_end)) = (
        arena.angle(next_angle).f_start.map(SpanId::new),
        arena.angle(next_angle).f_end.map(SpanId::new),
    ) else {
        return false;
    };
    transfer(
        arena, next_angle, next_start, next_end, binary, sum_mi, sum_su,
    )
}

/// Carries a known winding backward onto `next_angle`.
///
/// Port of `SkOpSegment::ComputeOneSumReverse`. Two things flip against
/// [`compute_one_sum`]: the base's winding is read forward, and the target's
/// span pair is set up end-to-start.
fn compute_one_sum_reverse(
    arena: &mut OpArena,
    base_angle: AngleId,
    next_angle: AngleId,
    include_type: IncludeType,
) -> bool {
    let binary = include_type >= IncludeType::BinarySingle;
    let (Some(base_start), Some(base_end)) = (
        arena.angle(base_angle).f_start.map(SpanId::new),
        arena.angle(base_angle).f_end.map(SpanId::new),
    ) else {
        return false;
    };
    let mut sum_mi = arena.update_winding(base_start, base_end, |_, _| false);
    let mut sum_su = PK_MIN_S32;
    if binary {
        sum_su = arena.update_opp_winding(base_start, base_end);
        if segment_operand(arena, base_angle) {
            std::mem::swap(&mut sum_mi, &mut sum_su);
        }
    }
    let (Some(next_start), Some(next_end)) = (
        arena.angle(next_angle).f_start.map(SpanId::new),
        arena.angle(next_angle).f_end.map(SpanId::new),
    ) else {
        return false;
    };
    // Note the swapped order: the reverse transfer sets up end-to-start.
    transfer(
        arena, next_angle, next_end, next_start, binary, sum_mi, sum_su,
    )
}

/// Sets up the target's windings and marks it, shared by both transfers.
fn transfer(
    arena: &mut OpArena,
    next_angle: AngleId,
    start: SpanId,
    end: SpanId,
    binary: bool,
    mut sum_mi: i32,
    mut sum_su: i32,
) -> bool {
    let last = if binary {
        // C++ calls nextSegment->setUpWindings, so the operand is the
        // segment being marked, not the one the winding came from.
        let operand = arena
            .span_segment(start)
            .is_some_and(|seg| arena.segment_operand(seg));
        let (max_winding, opp_max_winding) =
            arena.set_up_windings(start, end, operand, &mut sum_mi, &mut sum_su);
        match mark_angle_opp(
            arena,
            max_winding,
            sum_mi,
            opp_max_winding,
            sum_su,
            next_angle,
        ) {
            Some(last) => last,
            None => return false,
        }
    } else {
        let max_winding = arena.set_up_winding(start, end, &mut sum_mi);
        match mark_angle(arena, max_winding, sum_mi, next_angle) {
            Some(last) => last,
            None => return false,
        }
    };
    arena
        .angle_mut(next_angle)
        .set_last_marked(last.map(SpanId::index));
    true
}

/// Returns whether the angle's segment belongs to the second operand.
fn segment_operand(arena: &OpArena, angle: AngleId) -> bool {
    let Some(start) = arena.angle(angle).f_start.map(SpanId::new) else {
        return false;
    };
    let Some(segment) = arena.span_segment(start) else {
        return false;
    };
    arena.segment_operand(segment)
}

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

/// Spreads a known winding around the angle ring at `end`/`start`.
///
/// Port of `SkOpSegment::computeSum`. Returns [`PK_NAN32`] when no winding
/// could be resolved for the span pair.
///
/// The rule the two passes encode: a value may only be carried between two
/// *adjacent* angles when they and their neighbours are all orderable.
/// Carrying a winding across an unorderable angle would be carrying it across
/// a turn whose direction is unknown, which produces a plausible-looking
/// wrong answer rather than a detectable failure.
pub fn compute_sum(
    arena: &mut OpArena,
    start: SpanId,
    end: SpanId,
    include_type: IncludeType,
) -> i32 {
    debug_assert_ne!(include_type, IncludeType::UnaryXor);
    let Some(first_angle) = span_to_angle(arena, end, start) else {
        return PK_NAN32;
    };
    if arena.angle(first_angle).f_next.is_none() {
        return PK_NAN32;
    }

    let mut base_angle: Option<AngleId> = None;
    let mut try_reverse = false;

    // Counterclockwise pass.
    let Some(mut angle) = previous(arena, first_angle) else {
        return PK_NAN32;
    };
    let Some(mut next) = arena.angle(angle).f_next.map(AngleId::new) else {
        return PK_NAN32;
    };
    let mut first_angle = next;
    loop {
        let prior = angle;
        angle = next;
        next = match arena.angle(angle).f_next.map(AngleId::new) {
            Some(n) => n,
            None => return PK_NAN32,
        };
        if arena.angle(prior).unorderable()
            || arena.angle(angle).unorderable()
            || arena.angle(next).unorderable()
        {
            base_angle = None;
        } else if angle_wind_sum(arena, angle) != PK_MIN_S32 {
            base_angle = Some(angle);
            try_reverse = true;
        } else if let Some(base) = base_angle {
            compute_one_sum(arena, base, angle, include_type);
            base_angle = if angle_wind_sum(arena, angle) != PK_MIN_S32 {
                Some(angle)
            } else {
                None
            };
        }
        if next == first_angle {
            break;
        }
    }

    if let Some(base) = base_angle {
        if angle_wind_sum(arena, first_angle) == PK_MIN_S32 {
            first_angle = base;
            try_reverse = true;
        }
    }

    // Clockwise pass, from wherever the first one left a known value.
    if try_reverse {
        base_angle = None;
        let mut prior = first_angle;
        loop {
            let angle = prior;
            prior = match previous(arena, angle) {
                Some(p) => p,
                None => break,
            };
            let next = match arena.angle(angle).f_next.map(AngleId::new) {
                Some(n) => n,
                None => break,
            };
            if arena.angle(prior).unorderable()
                || arena.angle(angle).unorderable()
                || arena.angle(next).unorderable()
            {
                base_angle = None;
            } else if angle_wind_sum(arena, angle) != PK_MIN_S32 {
                base_angle = Some(angle);
            } else if let Some(base) = base_angle {
                compute_one_sum_reverse(arena, base, angle, include_type);
                base_angle = if angle_wind_sum(arena, angle) != PK_MIN_S32 {
                    Some(angle)
                } else {
                    None
                };
            }
            if prior == first_angle {
                break;
            }
        }
    }

    match arena.span_starter(start, end) {
        Some(starter) => arena.span(starter).wind_sum(),
        None => PK_NAN32,
    }
}

/// Where a `findNext` walk currently stands, and what it found.
#[derive(Debug, Clone, Copy)]
pub struct WalkState {
    /// Span the next edge starts at.
    pub start: SpanId,
    /// Span the next edge ends at.
    pub end: SpanId,
    /// Set when the walk could not decide which way to go.
    pub unsortable: bool,
    /// Set when only one segment continued, so no sort was needed.
    pub simple: bool,
}

impl WalkState {
    /// Returns a walk about to step from `start` to `end`.
    #[must_use]
    pub fn new(start: SpanId, end: SpanId) -> Self {
        Self {
            start,
            end,
            unsortable: false,
            simple: false,
        }
    }
}

/// Takes the single onward step when only one segment continues.
///
/// Port of the `isSimple` branch shared by all three `findNext` variants.
/// Returns `Some(Some(segment))` when it stepped, `Some(None)` when the walk
/// is finished, and `None` when the caller must sort the angles instead.
fn try_simple_step(arena: &mut OpArena, state: &mut WalkState) -> Option<Option<SegmentId>> {
    let step = arena.span_step(state.start, state.end);
    let mut chase = ChaseState {
        start: state.start,
        step,
        min: None,
        last: None,
    };
    // The span to retire is the one this walk is leaving, so it is read from
    // the pair as it stands now - before next_chase moves the start onto the
    // other segment. Reading it after would ask starter() to compare two
    // spans of different segments, which it has no answer for.
    let Some(start_span) = arena.span_starter(state.start, state.end) else {
        return Some(None);
    };
    let other = arena.next_chase(&mut chase)?;
    state.start = chase.start;
    if arena.span(start_span).done() {
        return Some(None);
    }
    arena.mark_done(start_span);
    let next_end = if chase.step > 0 {
        arena.span_next(state.start)
    } else {
        arena.span_prev(state.start)
    };
    match next_end {
        Some(e) => {
            state.end = e;
            state.simple = true;
            Some(Some(other))
        }
        None => Some(None),
    }
}

/// Gives up on the current edge, marking it done.
fn give_up(arena: &mut OpArena, state: &mut WalkState, orig_start: SpanId, orig_end: SpanId) {
    state.unsortable = true;
    if let Some(starter) = arena.span_starter(orig_start, orig_end) {
        arena.mark_done(starter);
    }
}

/// Walks the angle ring, marking each member and picking the one to follow.
///
/// The body shared by all three `findNext` variants: they differ only in how
/// each candidate's "is this edge active" question is answered, which is
/// `is_active`.
fn pick_next<F>(
    arena: &mut OpArena,
    state: &mut WalkState,
    angle: AngleId,
    chase: &mut Vec<SpanId>,
    mut is_active: F,
) -> Option<SegmentId>
where
    F: FnMut(&mut OpArena, SpanId, SpanId, SegmentId) -> bool,
{
    let orig_start = state.start;
    let orig_end = state.end;
    let mut next_angle = AngleId::new(arena.angle(angle).f_next?);
    let mut found_angle: Option<AngleId> = None;
    let mut found_done = false;
    let mut active_count = 0;

    // A do-while over the ring: the exit is `next_angle == angle` at the
    // bottom, so the first member is always visited. A `while let` would
    // move that test to the top and skip it.
    #[allow(clippy::while_let_loop)]
    loop {
        let Some(next_start) = arena.angle(next_angle).f_start.map(SpanId::new) else {
            break;
        };
        let Some(next_end) = arena.angle(next_angle).f_end.map(SpanId::new) else {
            break;
        };
        let Some(next_segment) = arena.span_segment(next_start) else {
            break;
        };
        let active = is_active(arena, next_start, next_end, next_segment);
        if active {
            active_count += 1;
            // The first active candidate wins, unless it was already walked
            // and an odd number have come since: that pairing is what keeps
            // the walk from re-entering an edge it just left.
            if found_angle.is_none() || (found_done && (active_count & 1) == 1) {
                found_angle = Some(next_angle);
                found_done = arena
                    .span_starter(next_start, next_end)
                    .is_some_and(|s| arena.span(s).done());
            }
        }
        if !arena.segment_done(next_segment) {
            if !active {
                arena.mark_and_chase_done(next_start, next_end);
            }
            if let Some(last) = last_marked(arena, next_angle) {
                if chase.len() < CHASE_LIMIT {
                    chase.push(last);
                }
            }
        }
        next_angle = match arena.angle(next_angle).f_next.map(AngleId::new) {
            Some(n) => n,
            None => break,
        };
        if next_angle == angle {
            break;
        }
    }

    if let Some(starter) = arena.span_starter(orig_start, orig_end) {
        arena.mark_done(starter);
    }
    let found = found_angle?;
    state.start = SpanId::new(arena.angle(found).f_start?);
    state.end = SpanId::new(arena.angle(found).f_end?);
    arena.span_segment(state.start)
}

/// Picks the next segment for a simplify walk.
///
/// Port of `SkOpSegment::findNextWinding`.
pub fn find_next_winding(
    arena: &mut OpArena,
    state: &mut WalkState,
    chase: &mut Vec<SpanId>,
) -> Option<SegmentId> {
    debug_assert_ne!(state.start, state.end);
    let orig_start = state.start;
    let orig_end = state.end;
    if let Some(result) = try_simple_step(arena, state) {
        return result;
    }
    if compute_sum(arena, orig_start, orig_end, IncludeType::UnaryWinding) == PK_NAN32 {
        give_up(arena, state, orig_start, orig_end);
        return None;
    }
    let Some(angle) = span_to_angle(arena, orig_end, orig_start) else {
        give_up(arena, state, orig_start, orig_end);
        return None;
    };
    if arena.angle(angle).unorderable() {
        give_up(arena, state, orig_start, orig_end);
        return None;
    }
    let mut sum_winding = arena.update_winding(orig_end, orig_start, |_, _| false);
    pick_next(arena, state, angle, chase, |arena, s, e, _| {
        arena.active_winding_with(s, e, &mut sum_winding)
    })
}

/// Picks the next segment for an xor-fill walk.
///
/// Port of `SkOpSegment::findNextXor`. An xor walk needs no winding at all:
/// every edge that has not been walked is active, so there is nothing to
/// transfer and nothing to fail on.
pub fn find_next_xor(
    arena: &mut OpArena,
    state: &mut WalkState,
    chase: &mut Vec<SpanId>,
) -> Option<SegmentId> {
    debug_assert_ne!(state.start, state.end);
    let orig_start = state.start;
    let orig_end = state.end;
    if let Some(result) = try_simple_step(arena, state) {
        return result;
    }
    let Some(angle) = span_to_angle(arena, orig_end, orig_start) else {
        give_up(arena, state, orig_start, orig_end);
        return None;
    };
    if arena.angle(angle).unorderable() {
        give_up(arena, state, orig_start, orig_end);
        return None;
    }
    pick_next(arena, state, angle, chase, |arena, s, e, _| {
        arena
            .span_starter(s, e)
            .is_some_and(|span| !arena.span(span).done())
    })
}

/// Picks the next segment for a boolean-op walk.
///
/// Port of `SkOpSegment::findNextOp`. Unlike the winding walk this carries
/// two sums, one per operand, and swaps them when stepping onto a segment
/// from the other operand.
pub fn find_next_op(
    arena: &mut OpArena,
    state: &mut WalkState,
    chase: &mut Vec<SpanId>,
    op: PathOp,
    xor_mi_mask: i32,
    xor_su_mask: i32,
) -> Option<SegmentId> {
    debug_assert_ne!(state.start, state.end);
    let orig_start = state.start;
    let orig_end = state.end;
    if let Some(result) = try_simple_step(arena, state) {
        return result;
    }
    if compute_sum(arena, orig_start, orig_end, IncludeType::BinaryOpp) == PK_NAN32 {
        give_up(arena, state, orig_start, orig_end);
        return None;
    }
    let Some(angle) = span_to_angle(arena, orig_end, orig_start) else {
        give_up(arena, state, orig_start, orig_end);
        return None;
    };
    if arena.angle(angle).unorderable() {
        give_up(arena, state, orig_start, orig_end);
        return None;
    }
    let mut sum_mi = arena.update_winding(orig_end, orig_start, |_, _| false);
    if sum_mi == PK_MIN_S32 {
        give_up(arena, state, orig_start, orig_end);
        return None;
    }
    let mut sum_su = arena.update_opp_winding(orig_end, orig_start);
    let Some(own_segment) = arena.span_segment(orig_start) else {
        give_up(arena, state, orig_start, orig_end);
        return None;
    };
    if arena.segment_operand(own_segment) {
        std::mem::swap(&mut sum_mi, &mut sum_su);
    }
    pick_next(arena, state, angle, chase, |arena, s, e, segment| {
        let operand = arena.segment_operand(segment);
        arena.active_op_with(
            s,
            e,
            operand,
            xor_mi_mask,
            xor_su_mask,
            op,
            &mut sum_mi,
            &mut sum_su,
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Path;

    /// Builds a segment and returns it with its head and tail spans.
    fn line(arena: &mut OpArena, a: Point, b: Point) -> (SegmentId, SpanId, SpanId) {
        let seg = arena.alloc_segment_with_curve(&[a, b], Verb::Line, 1.0);
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        (seg, head, tail)
    }

    #[test]
    fn add_curve_to_emits_a_cubic_as_a_cubic() {
        let mut arena = OpArena::new();
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(0.0, 50.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 100.0),
        ];
        let seg = arena.alloc_segment_with_curve(&pts, Verb::Cubic, 1.0);
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let mut path = Path::new();
        {
            let mut writer = SkPathWriter::new(&mut path);
            assert!(add_curve_to(&mut arena, head, tail, &mut writer));
            // An open contour is held back as a partial; closing the loop is
            // what the walker itself does, and what puts it in the path.
            writer.deferred_line(pts[0]);
            writer.finish_contour();
        }
        assert!(
            path.verbs().contains(&Verb::Cubic),
            "the cubic must survive the walk, got {:?}",
            path.verbs()
        );
    }

    #[test]
    fn add_curve_to_emits_a_quad_as_a_quad() {
        let mut arena = OpArena::new();
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
        ];
        let seg = arena.alloc_segment_with_curve(&pts, Verb::Quad, 1.0);
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let mut path = Path::new();
        {
            let mut writer = SkPathWriter::new(&mut path);
            assert!(add_curve_to(&mut arena, head, tail, &mut writer));
            writer.deferred_line(pts[0]);
            writer.finish_contour();
        }
        assert!(path.verbs().contains(&Verb::Quad), "{:?}", path.verbs());
    }

    #[test]
    fn add_curve_to_emits_a_conic_with_its_weight() {
        let mut arena = OpArena::new();
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
        ];
        let seg = arena.alloc_segment_with_curve(&pts, Verb::Conic, 0.75);
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let mut path = Path::new();
        {
            let mut writer = SkPathWriter::new(&mut path);
            assert!(add_curve_to(&mut arena, head, tail, &mut writer));
            writer.deferred_line(pts[0]);
            writer.finish_contour();
        }
        assert!(path.verbs().contains(&Verb::Conic), "{:?}", path.verbs());
        let weights = path.conic_weights();
        assert!(!weights.is_empty(), "the conic kept a weight");
        assert!(
            (weights[0] - 0.75).abs() < 1e-5,
            "weight was {}",
            weights[0]
        );
    }

    #[test]
    fn a_collinear_cubic_degrades_to_a_line() {
        let mut arena = OpArena::new();
        // Every control point on the same straight line.
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(20.0, 0.0),
            Point::new(30.0, 0.0),
        ];
        let seg = arena.alloc_segment_with_curve(&pts, Verb::Cubic, 1.0);
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let mut path = Path::new();
        {
            let mut writer = SkPathWriter::new(&mut path);
            assert!(add_curve_to(&mut arena, head, tail, &mut writer));
            writer.deferred_line(Point::new(15.0, 10.0));
            writer.deferred_line(pts[0]);
            writer.finish_contour();
        }
        assert!(
            !path.verbs().contains(&Verb::Cubic),
            "a cubic with no bend is a line, got {:?}",
            path.verbs()
        );
    }

    #[test]
    fn add_curve_to_refuses_the_same_span_twice() {
        let mut arena = OpArena::new();
        let (_, head, tail) = line(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        let mut path = Path::new();
        let mut writer = SkPathWriter::new(&mut path);
        assert!(add_curve_to(&mut arena, head, tail, &mut writer));
        assert!(
            !add_curve_to(&mut arena, head, tail, &mut writer),
            "a walk that doubles back must be caught"
        );
    }

    #[test]
    fn add_curve_to_subdivides_an_interior_walk() {
        let mut arena = OpArena::new();
        let pts = [
            Point::new(0.0, 0.0),
            Point::new(0.0, 100.0),
            Point::new(100.0, 100.0),
            Point::new(100.0, 0.0),
        ];
        let seg = arena.alloc_segment_with_curve(&pts, Verb::Cubic, 1.0);
        let mid_pt = arena.segment_pt_at_t(seg, 0.5);
        arena.segment_add_t(seg, 0.5, mid_pt).expect("split");
        let head = arena.segment(seg).f_head.expect("head");
        let mid = arena.span_next(head).expect("mid");
        let mut path = Path::new();
        {
            let mut writer = SkPathWriter::new(&mut path);
            assert!(add_curve_to(&mut arena, head, mid, &mut writer));
            writer.deferred_line(pts[0]);
            writer.finish_contour();
        }
        assert!(path.verbs().contains(&Verb::Cubic));
        // The cubic's own endpoint must be the split point, not t = 1. It is
        // the fourth point of the contour: move, then three for the cubic.
        let cubic_end = path.points()[3];
        assert!(
            (cubic_end.x - mid_pt.x).abs() < 1e-3 && (cubic_end.y - mid_pt.y).abs() < 1e-3,
            "the cubic ended at {cubic_end:?}, wanted the split at {mid_pt:?}"
        );
        assert!(
            (cubic_end.x - 100.0).abs() > 1.0,
            "and not at the segment's own end"
        );
    }

    #[test]
    fn compute_sum_reports_nan_without_an_angle_loop() {
        let mut arena = OpArena::new();
        let (_, head, tail) = line(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        assert_eq!(
            compute_sum(&mut arena, head, tail, IncludeType::UnaryWinding),
            PK_NAN32,
            "no angles means nothing to spread"
        );
    }

    #[test]
    fn mark_angle_narrows_to_the_inner_winding() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        arena.span_mut(head).set_wind_value(1);
        let angle = arena.alloc_angle(super::super::sk_op_angle::SkOpAngle::new());
        super::super::sk_op_angle_order::set(&mut arena, angle, head, tail);
        // use_inner_winding(outer, inner) is "is the OUTER the smaller one".
        // With outer 3 and inner 1 it is not, so max_winding stands.
        assert!(mark_angle(&mut arena, 3, 1, angle).is_some());
        assert_eq!(arena.span(head).wind_sum(), 3);

        // Reversed, the outer is now the smaller, so the inner is taken.
        let seg2 = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 5.0), Point::new(10.0, 5.0)],
            Verb::Line,
            1.0,
        );
        let head2 = arena.segment(seg2).f_head.expect("head");
        let tail2 = arena.segment(seg2).f_tail.expect("tail");
        arena.span_mut(head2).set_wind_value(1);
        let angle2 = arena.alloc_angle(super::super::sk_op_angle::SkOpAngle::new());
        super::super::sk_op_angle_order::set(&mut arena, angle2, head2, tail2);
        assert!(mark_angle(&mut arena, 1, 3, angle2).is_some());
        assert_eq!(
            arena.span(head2).wind_sum(),
            3,
            "the inner value wins when the outer has the smaller magnitude"
        );
    }
}
