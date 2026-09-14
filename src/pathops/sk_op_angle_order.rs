//! The half of `SkOpAngle` that reads the span graph: `set`, `setSpans`,
//! `computeSector`, `endsIntersect`, `orderable` and `after`.
//!
//! # Why these are not on `SkOpAngle`
//!
//! Everything here dereferences an angle's start and end spans into segment
//! geometry, and a span only knows its segment by arena id. In C++ those are
//! raw pointer hops inside the class; here the arena has to be in hand, so
//! they are free functions taking `&OpArena` rather than methods.
//!
//! The geometric predicates they are built on — sector assignment, convex
//! hull overlap, tangent divergence, the line-side tests — stay on
//! [`SkOpAngle`](super::sk_op_angle::SkOpAngle), where they need no graph.
//!
//! # The comparator contract
//!
//! [`after`] means "`angle` lies in the counterclockwise arc from `test` to
//! `test.next`", not "`angle`'s sector is greater". A naive `>` is not an
//! ordering on a ring: under `merge` an angle that compares as belonging
//! nowhere gets unlinked without being relinked, so it is silently dropped.

use super::sk_curve_intersect_ray::{
    closest_to, curve_d_slope_at_t, curve_extent, curve_intersect_ray, curve_pt_at_t, most_outside,
};
use super::sk_line_parameters::{LinePoint, SkLineParameters};
use super::sk_op_angle::{sub_divide_curve, verb_to_points, AngleVector, SkOpAngle};
use super::sk_op_arena::{AngleId, OpArena, SegmentId, SpanId};
use super::sk_path_ops_cubic::SkDCubic;
use super::sk_path_ops_types::{approximately_equal, approximately_zero};
use crate::core::Verb;

/// Returns the segment geometry an angle's start span belongs to, in f64.
///
/// The angle code works in f64 throughout, so the conversion happens once
/// here rather than at every use.
fn angle_curve(arena: &OpArena, angle: AngleId) -> Option<(Vec<LinePoint>, Verb, f64)> {
    let start = SpanId::new(arena.angle(angle).f_start?);
    let segment = arena.span_segment(start)?;
    Some(segment_curve_f64(arena, segment))
}

/// Returns a segment's control points, verb and weight in f64.
fn segment_curve_f64(arena: &OpArena, segment: SegmentId) -> (Vec<LinePoint>, Verb, f64) {
    let (pts, verb, weight) = arena.segment_curve(segment);
    let dpts = pts
        .iter()
        .map(|p| [f64::from(p.x), f64::from(p.y)])
        .collect();
    (dpts, verb, f64::from(weight))
}

/// Returns the t of a span.
fn span_t(arena: &OpArena, span: SpanId) -> f64 {
    f64::from(arena.span(span).f_t)
}

/// Returns the point of a span, preferring its PtT node's cached point.
fn span_pt(arena: &OpArena, span: SpanId) -> LinePoint {
    let pt = match arena.span(span).f_ptt {
        Some(id) => arena.ptt(super::sk_op_arena::PtTId::new(id)).f_pt,
        None => arena.span(span).f_pt,
    };
    [f64::from(pt.x), f64::from(pt.y)]
}

/// Points `angle` at the walk from `start` to `end` and computes its curve
/// and sector.
///
/// Port of `SkOpAngle::set`.
pub fn set(arena: &mut OpArena, angle: AngleId, start: SpanId, end: SpanId) {
    debug_assert_ne!(start, end);
    {
        let a = arena.angle_mut(angle);
        a.f_start = Some(start.index());
        a.f_end = Some(end.index());
        a.f_computed_end = Some(end.index());
        a.f_next = None;
        a.f_compute_sector = false;
        a.f_computed_sector = false;
        a.f_check_coincidence = false;
        a.f_tangents_ambiguous = false;
    }
    set_spans(arena, angle);
    arena.angle_mut(angle).set_sector();
}

/// Fills the angle's curve part and the side its bulk falls on.
///
/// Port of `SkOpAngle::setSpans`. The `fSide` it computes is not normalized;
/// only its sign is ever read.
pub fn set_spans(arena: &mut OpArena, angle: AngleId) {
    {
        let a = arena.angle_mut(angle);
        a.f_unorderable = false;
        a.f_last_marked = None;
    }
    let (Some(start), Some(end)) = (
        arena.angle(angle).f_start.map(SpanId::new),
        arena.angle(angle).f_end.map(SpanId::new),
    ) else {
        arena.angle_mut(angle).f_unorderable = true;
        return;
    };
    let Some(segment) = arena.span_segment(start) else {
        arena.angle_mut(angle).f_unorderable = true;
        return;
    };
    let (pts, verb, weight) = segment_curve_f64(arena, segment);

    let mut part = arena.angle(angle).f_part;
    part.f_verb = verb;
    part.f_weight = weight;
    sub_divide_curve(
        &pts,
        verb,
        weight,
        span_pt(arena, start),
        span_t(arena, start),
        span_pt(arena, end),
        span_t(arena, end),
        &mut part,
    );
    let mut original = part;
    part.set_curve_hull_sweep();

    // A curve whose control points are collinear with its ends sorts as a
    // line; collapse it to one so the line paths below apply.
    if verb != Verb::Line && !part.is_curve() {
        part.f_curve[1] = part.f_curve[verb_to_points(verb)];
        original.f_curve[1] = part.f_curve[1];
        let half = [part.f_curve[0], part.f_curve[1]];
        let mut tangent = SkLineParameters::new();
        tangent.line_end_points(&half);
        let a = arena.angle_mut(angle);
        a.f_part = part;
        a.f_original_curve_part = original;
        a.f_tangent_half = tangent;
        a.f_side = 0.0;
        return;
    }

    let side;
    let mut tangent_half = arena.angle(angle).f_tangent_half;
    match verb {
        Verb::Line => {
            // The far end of the line as the segment stores it, which is the
            // control point in the direction of travel.
            let c_p1 = pts[usize::from(span_t(arena, start) < span_t(arena, end))];
            let half = [span_pt(arena, start), c_p1];
            tangent_half.line_end_points(&half);
            side = 0.0;
        }
        Verb::Quad | Verb::Conic => {
            let mut tangent_part = SkLineParameters::new();
            tangent_part.quad_end_points(&part.f_curve[..3]);
            side = -tangent_part.point_distance(part.f_curve[2]);
        }
        Verb::Cubic => {
            let mut tangent_part = SkLineParameters::new();
            tangent_part.cubic_part(&part.f_curve[..4]);
            side = cubic_best_side(
                arena,
                &pts,
                weight,
                &part.f_curve[..4],
                span_t(arena, start),
                span_t(arena, end),
            );
            // The line parameters above are recomputed inside the sweep; the
            // distance to the far endpoint is the fallback when no inflection
            // lies in range.
            let _ = tangent_part.point_distance(part.f_curve[3]);
        }
        Verb::Move | Verb::Close => {
            arena.angle_mut(angle).f_unorderable = true;
            return;
        }
    }
    let a = arena.angle_mut(angle);
    a.f_part = part;
    a.f_original_curve_part = original;
    a.f_tangent_half = tangent_half;
    a.f_side = side;
}

/// Returns the signed side of a cubic's bulk relative to its own chord.
///
/// Port of the cubic branch of `SkOpAngle::setSpans`. A cubic can bulge both
/// ways inside one span, so the sign is taken from whichever sample sits
/// furthest off the chord, sampling at the inflections and the midpoints
/// between them rather than at a single t.
fn cubic_best_side(
    _arena: &OpArena,
    pts: &[LinePoint],
    weight: f64,
    part: &[LinePoint],
    start_t: f64,
    end_t: f64,
) -> f64 {
    let cubic = SkDCubic::new([
        to_dpoint(pts[0]),
        to_dpoint(pts[1]),
        to_dpoint(pts[2]),
        to_dpoint(pts[3]),
    ]);
    let (inflections, inflection_count) = cubic.find_inflections();

    // Collect the sample ts: the inflections that fall inside the span, plus
    // both ends.
    let mut test_ts: Vec<f64> = Vec::with_capacity(inflection_count + 2);
    let (lo, hi) = if start_t <= end_t {
        (start_t, end_t)
    } else {
        (end_t, start_t)
    };
    for &t in inflections.iter().take(inflection_count) {
        if t > lo && t < hi {
            test_ts.push(t);
        }
    }
    test_ts.push(start_t);
    test_ts.push(end_t);
    test_ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let mut test_part = SkLineParameters::new();
    test_part.cubic_end_points(part);

    let mut best_side = 0.0f64;
    // Sample at each t and at the midpoint of each adjacent pair, matching
    // the C++ walk over `(testCount << 1) - 1` cases.
    for index in 0..test_ts.len() {
        for &t in &[
            Some(test_ts[index]),
            test_ts.get(index + 1).map(|next| (test_ts[index] + next) / 2.0),
        ] {
            let Some(t) = t else { continue };
            let pt = curve_pt_at_t(pts, Verb::Cubic, weight, t);
            let test_side = test_part.point_distance(pt);
            if best_side.abs() < test_side.abs() {
                best_side = test_side;
            }
        }
    }
    -best_side
}

/// Converts an f64 pair to the cubic module's point type.
fn to_dpoint(p: LinePoint) -> super::sk_path_ops_point::SkDPoint {
    super::sk_path_ops_point::SkDPoint { f_x: p[0], f_y: p[1] }
}

/// Lengthens an angle too short to have a sector and recomputes it.
///
/// Port of `SkOpAngle::computeSector`. Returns false when the angle cannot be
/// lengthened without running into an adjacent angle, which leaves it
/// unorderable.
pub fn compute_sector(arena: &mut OpArena, angle: AngleId) -> bool {
    if arena.angle(angle).f_computed_sector {
        return !arena.angle(angle).f_unorderable;
    }
    arena.angle_mut(angle).f_computed_sector = true;
    let (Some(start), Some(end)) = (
        arena.angle(angle).f_start.map(SpanId::new),
        arena.angle(angle).f_end.map(SpanId::new),
    ) else {
        arena.angle_mut(angle).f_unorderable = true;
        return false;
    };
    let step_up = span_t(arena, start) < span_t(arena, end);
    if arena.span_is_final(end) && step_up {
        arena.angle_mut(angle).f_unorderable = true;
        return false;
    }
    let Some(own_segment) = arena.span_segment(start) else {
        arena.angle_mut(angle).f_unorderable = true;
        return false;
    };

    // Walk outward from the end until a span on our own segment is found at
    // the same t, which is where lengthening would collide.
    let mut check_end = Some(end);
    while let Some(current) = check_end {
        if shares_t_with_own_segment(arena, current, own_segment) {
            break;
        }
        check_end = if step_up {
            if arena.span_is_final(current) {
                None
            } else {
                arena.span_next(current)
            }
        } else {
            arena.span_prev(current)
        };
    }

    let computed_end = match check_end {
        Some(c) => {
            if step_up {
                arena.span_prev(c)
            } else {
                arena.span_next(c)
            }
        }
        None => {
            let seg = arena.span_segment(end).unwrap_or(own_segment);
            if step_up {
                arena.segment(seg).f_head
            } else {
                arena.segment(seg).f_tail
            }
        }
    };
    let Some(computed_end) = computed_end else {
        arena.angle_mut(angle).f_unorderable = true;
        return false;
    };
    if check_end == Some(end) || computed_end == end || computed_end == start {
        arena.angle_mut(angle).f_unorderable = true;
        return false;
    }
    if step_up != (span_t(arena, start) < span_t(arena, computed_end)) {
        arena.angle_mut(angle).f_unorderable = true;
        return false;
    }

    // Recompute against the lengthened end, then put the real end back: the
    // sector is what changes, not what the angle spans.
    let save_end = end;
    arena.angle_mut(angle).f_end = Some(computed_end.index());
    arena.angle_mut(angle).f_computed_end = Some(computed_end.index());
    set_spans(arena, angle);
    arena.angle_mut(angle).set_sector();
    arena.angle_mut(angle).f_end = Some(save_end.index());
    !arena.angle(angle).f_unorderable
}

/// Returns true when some span at `check_end`'s t sits on `own_segment`.
///
/// The inner loop of `computeSector`: lengthening past a point another span
/// of our own segment already occupies would make the angle cross itself.
fn shares_t_with_own_segment(arena: &OpArena, check_end: SpanId, own_segment: SegmentId) -> bool {
    let Some(other) = arena.span_segment(check_end) else {
        return false;
    };
    let check_t = arena.span(check_end).f_t;
    let mut span = arena.segment(other).f_head;
    while let Some(o_span) = span {
        if arena.span_segment(o_span) == Some(own_segment)
            && o_span != check_end
            && approximately_equal(f64::from(arena.span(o_span).f_t), f64::from(check_t))
        {
            return true;
        }
        if arena.span_is_final(o_span) {
            break;
        }
        span = arena.span_next(o_span);
    }
    false
}

/// Returns the t halfway along the angle's span.
fn mid_t(arena: &OpArena, angle: AngleId) -> f64 {
    let (Some(start), Some(end)) = (
        arena.angle(angle).f_start.map(SpanId::new),
        arena.angle(angle).f_end.map(SpanId::new),
    ) else {
        return 0.5;
    };
    (span_t(arena, start) + span_t(arena, end)) / 2.0
}

/// Returns true when the two angles' end spans name the same point.
fn ends_share_point(arena: &OpArena, lh: AngleId, rh: AngleId) -> bool {
    let (Some(l_end), Some(r_end)) = (
        arena.angle(lh).f_end.map(SpanId::new),
        arena.angle(rh).f_end.map(SpanId::new),
    ) else {
        return false;
    };
    let (Some(l_ptt), Some(r_ptt)) = (arena.span_ptt(l_end), arena.span_ptt(r_end)) else {
        return false;
    };
    arena.ptt_contains(l_ptt, r_ptt)
}

/// Decides order when the two curves leave the shared point in near-parallel
/// directions.
///
/// Port of `SkOpAngle::checkParallel`. Each step is a fallback for the one
/// before: the tangent cross product, then perpendiculars cast from the ends
/// and the middles, then the cross product of the two mid-t vectors.
pub fn check_parallel(arena: &mut OpArena, lh: AngleId, rh: AngleId) -> bool {
    let sweep = if arena.angle(lh).f_part.is_ordered() {
        arena.angle(lh).f_part.f_sweep[0]
    } else {
        arena.angle(lh).f_part.pt(1) - arena.angle(lh).f_part.pt(0)
    };
    let tweep = if arena.angle(rh).f_part.is_ordered() {
        arena.angle(rh).f_part.f_sweep[0]
    } else {
        arena.angle(rh).f_part.pt(1) - arena.angle(rh).f_part.pt(0)
    };
    let s0xt0 = sweep.cross_check(tweep);

    let (l_pts, l_verb, _) = match angle_curve(arena, lh) {
        Some(v) => v,
        None => return true,
    };
    let (r_pts, r_verb, r_weight) = match angle_curve(arena, rh) {
        Some(v) => v,
        None => return true,
    };
    let rh_angle = arena.angle(rh).clone();
    if arena
        .angle_mut(lh)
        .tangents_diverge(&rh_angle, s0xt0, &l_pts, l_verb, &r_pts, r_verb)
    {
        return s0xt0 < 0.0;
    }

    // Cast a perpendicular from each end and each middle onto the other
    // curve, and see which side the crossing lands on.
    if !ends_share_point(arena, lh, rh) {
        if let Some(inside) = end_to_side(arena, lh, rh) {
            return inside;
        }
        if let Some(inside) = end_to_side(arena, rh, lh) {
            return !inside;
        }
    }
    if let Some(inside) = mid_to_side(arena, lh, rh) {
        return inside;
    }
    if let Some(inside) = mid_to_side(arena, rh, lh) {
        return !inside;
    }

    // Last resort: compare the vectors to each curve's own mid-t point.
    let (l_weight, l_mid_t, r_mid_t) = (
        angle_curve(arena, lh).map_or(1.0, |c| c.2),
        mid_t(arena, lh),
        mid_t(arena, rh),
    );
    let l_origin = arena.angle(lh).f_part.f_curve[0];
    let r_origin = arena.angle(rh).f_part.f_curve[0];
    let l_pt = curve_pt_at_t(&l_pts, l_verb, l_weight, l_mid_t);
    let r_pt = curve_pt_at_t(&r_pts, r_verb, r_weight, r_mid_t);
    let m0 = AngleVector::new(l_pt[0] - l_origin[0], l_pt[1] - l_origin[1]);
    let m1 = AngleVector::new(r_pt[0] - r_origin[0], r_pt[1] - r_origin[1]);
    let m0xm1 = m0.cross_check(m1);
    if m0xm1 == 0.0 {
        arena.angle_mut(lh).f_unorderable = true;
        arena.angle_mut(rh).f_unorderable = true;
        return true;
    }
    m0xm1 < 0.0
}

/// Casts a perpendicular from `lh`'s end onto `rh` and reports which side it
/// lands on.
///
/// Port of `SkOpAngle::endToSide`. Returns `None` when the crossing is too
/// close to call, which is what sends `checkParallel` to its next fallback.
fn end_to_side(arena: &OpArena, lh: AngleId, rh: AngleId) -> Option<bool> {
    let l_start = SpanId::new(arena.angle(lh).f_start?);
    let l_end = SpanId::new(arena.angle(lh).f_end?);
    let l_segment = arena.span_segment(l_start)?;
    let (l_pts, l_verb, l_weight) = segment_curve_f64(arena, l_segment);

    // The ray is the perpendicular to the curve's own slope at its end.
    let end_pt = span_pt(arena, l_end);
    let slope = curve_d_slope_at_t(&l_pts, l_verb, l_weight, span_t(arena, l_end));
    let ray = [end_pt, [end_pt[0] + slope[1], end_pt[1] - slope[0]]];

    let r_start = SpanId::new(arena.angle(rh).f_start?);
    let r_end = SpanId::new(arena.angle(rh).f_end?);
    let r_segment = arena.span_segment(r_start)?;
    let (r_pts, r_verb, r_weight) = segment_curve_f64(arena, r_segment);
    let hits = curve_intersect_ray(&r_pts, r_verb, r_weight, &ray);
    let (closest, mut end_dist) = closest_to(
        &hits,
        span_t(arena, r_start),
        span_t(arena, r_end),
        ray[0],
    )?;
    if end_dist == 0.0 {
        return None;
    }

    // Normalize by the opposite curve's size, so the cutoff below means the
    // same thing at any scale.
    let max_width = curve_extent(&arena.angle(rh).f_part.f_curve, r_verb);
    end_dist /= max_width;
    // Written as a negated `>=` rather than `<` so a NaN also fails,
    // matching the C++ comment at this line.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(end_dist >= 5e-12) {
        return None;
    }

    let start = span_pt(arena, l_start);
    let opp_pt = hits.pt(closest);
    let v_left = AngleVector::new(ray[0][0] - start[0], ray[0][1] - start[1]);
    let v_right = AngleVector::new(opp_pt[0] - start[0], opp_pt[1] - start[1]);
    let dir = v_left.cross_no_normal_check(v_right);
    if dir == 0.0 {
        return None;
    }
    Some(dir < 0.0)
}

/// Casts a perpendicular from the middle of `lh`'s chord onto both curves and
/// compares which side each crossing falls on.
///
/// Port of `SkOpAngle::midToSide`.
fn mid_to_side(arena: &OpArena, lh: AngleId, rh: AngleId) -> Option<bool> {
    let l_start = SpanId::new(arena.angle(lh).f_start?);
    let l_end = SpanId::new(arena.angle(lh).f_end?);
    let l_segment = arena.span_segment(l_start)?;
    let (l_pts, l_verb, l_weight) = segment_curve_f64(arena, l_segment);

    let start_pt = span_pt(arena, l_start);
    let end_pt = span_pt(arena, l_end);
    let mid = [
        (start_pt[0] + end_pt[0]) / 2.0,
        (start_pt[1] + end_pt[1]) / 2.0,
    ];
    let ray = [
        mid,
        [
            mid[0] + (end_pt[1] - start_pt[1]),
            mid[1] - (end_pt[0] - start_pt[0]),
        ],
    ];

    let own_hits = curve_intersect_ray(&l_pts, l_verb, l_weight, &ray);
    let i_outside = most_outside(
        &own_hits,
        span_t(arena, l_start),
        span_t(arena, l_end),
        start_pt,
    )?;

    let r_start = SpanId::new(arena.angle(rh).f_start?);
    let r_end = SpanId::new(arena.angle(rh).f_end?);
    let r_segment = arena.span_segment(r_start)?;
    let (r_pts, r_verb, r_weight) = segment_curve_f64(arena, r_segment);
    let opp_hits = curve_intersect_ray(&r_pts, r_verb, r_weight, &ray);
    let opp_outside = most_outside(
        &opp_hits,
        span_t(arena, r_start),
        span_t(arena, r_end),
        start_pt,
    )?;

    let i_pt = own_hits.pt(i_outside);
    let o_pt = opp_hits.pt(opp_outside);
    let i_side = AngleVector::new(i_pt[0] - start_pt[0], i_pt[1] - start_pt[1]);
    let opp_side = AngleVector::new(o_pt[0] - start_pt[0], o_pt[1] - start_pt[1]);
    let dir = i_side.cross_check(opp_side);
    if dir == 0.0 {
        return None;
    }
    Some(dir < 0.0)
}

/// Decides order by where each curve's extension crosses the other.
///
/// Port of `SkOpAngle::endsIntersect`. This is the main ordering path for
/// curves whose hulls overlap: it casts each curve's chord as a ray at the
/// other, and takes the order from whichever crossing is more decisive.
pub fn ends_intersect(arena: &mut OpArena, lh: AngleId, rh: AngleId) -> bool {
    let Some((l_pts, l_verb, l_weight)) = angle_curve(arena, lh) else {
        return true;
    };
    let Some((r_pts, r_verb, r_weight)) = angle_curve(arena, rh) else {
        return true;
    };
    let l_points = verb_to_points(l_verb);
    let r_points = verb_to_points(r_verb);
    let origin = arena.angle(lh).f_part.f_curve[0];
    // Ray 0 runs at rh's far end; ray 1 runs at lh's own.
    let rays = [
        [origin, arena.angle(rh).f_part.f_curve[r_points]],
        [origin, arena.angle(lh).f_part.f_curve[l_points]],
    ];

    if ends_share_point(arena, lh, rh) {
        return check_parallel(arena, lh, rh);
    }

    // For each curve, the t furthest along it that its own ray reaches.
    let mut small_ts = [-1.0f64; 2];
    let mut limited = [false; 2];
    for index in 0..2 {
        let (pts, verb, weight) = if index == 1 {
            (&r_pts, r_verb, r_weight)
        } else {
            (&l_pts, l_verb, l_weight)
        };
        if verb == Verb::Line {
            // A line meets a ray only where they cross, which ordinary
            // intersection has already found.
            continue;
        }
        let angle = if index == 1 { rh } else { lh };
        let Some(start) = arena.angle(angle).f_start.map(SpanId::new) else {
            continue;
        };
        let computed_end = arena
            .angle(angle)
            .f_computed_end
            .or(arena.angle(angle).f_end)
            .map(SpanId::new);
        let Some(computed_end) = computed_end else {
            continue;
        };
        let t_start = span_t(arena, start);
        let t_end = span_t(arena, computed_end);
        let ascends = t_start < t_end;
        let mut t: f64 = if ascends { 0.0 } else { 1.0 };
        let hits = curve_intersect_ray(pts, verb, weight, &rays[index]);
        for idx2 in 0..hits.used() {
            let test_t = hits.t(idx2);
            if !between_orderable(t_start, test_t, t_end) {
                continue;
            }
            if approximately_equal(t_start, test_t) {
                continue;
            }
            t = if ascends {
                t.max(test_t)
            } else {
                t.min(test_t)
            };
            small_ts[index] = t;
            limited[index] = approximately_equal(t, t_end);
        }
    }

    // Pick whichever crossing is far enough from the chord end to be
    // meaningful, preferring one that reaches past its own ray.
    let mut s_ray_longer = false;
    let mut s_cept = AngleVector::zero();
    let mut s_cept_t = -1.0f64;
    let mut s_index = usize::MAX;
    let mut use_intersect = false;
    for index in 0..2 {
        if small_ts[index] < 0.0 {
            continue;
        }
        let (pts, verb, weight) = if index == 1 {
            (&r_pts, r_verb, r_weight)
        } else {
            (&l_pts, l_verb, l_weight)
        };
        let d_pt = curve_pt_at_t(pts, verb, weight, small_ts[index]);
        let cept = AngleVector::new(
            d_pt[0] - rays[index][0][0],
            d_pt[1] - rays[index][0][1],
        );
        // For a ray aimed at a line, a crossing nearer the start than the end
        // would already have been found by ordinary intersection.
        let other_points = if index == 1 { l_points } else { r_points };
        let end = AngleVector::new(
            rays[index][1][0] - rays[index][0][0],
            rays[index][1][1] - rays[index][0][1],
        );
        if other_points == 1 && cept.length_squared() * 2.0 < end.length_squared() {
            continue;
        }
        if cept.f_x * end.f_x < 0.0 || cept.f_y * end.f_y < 0.0 {
            // The crossing is behind the ray's origin.
            continue;
        }
        let ray_dist = cept.length();
        let end_dist = end.length();
        let ray_longer = ray_dist > end_dist;
        if limited[0] && limited[1] && ray_longer {
            use_intersect = true;
            s_ray_longer = ray_longer;
            s_cept = cept;
            s_cept_t = small_ts[index];
            s_index = index;
            break;
        }
        let mut delta = (ray_dist - end_dist).abs();
        let curve = if index == 1 {
            &arena.angle(rh).f_part.f_curve
        } else {
            &arena.angle(lh).f_part.f_curve
        };
        let max_width = curve_extent(curve, verb);
        delta = if max_width == 0.0 {
            0.0
        } else {
            delta / max_width
        };
        // A crossing this marginal can flip with a translation. Check whether
        // moving the curves to a common origin changed which side of rh's
        // chord lh falls on; if it did, the two are effectively parallel.
        if (1e-3..4e-3).contains(&delta)
            && !use_intersect
            && arena.angle(lh).f_part.is_curve()
            && arena.angle(rh).f_part.is_curve()
            && arena.angle(lh).f_original_curve_part.f_curve[0]
                != arena.angle(lh).f_part.f_curve[0]
        {
            let r_origin = arena.angle(rh).f_original_curve_part.f_curve[0];
            let count = verb_to_points(r_verb);
            let line = AngleVector::new(
                arena.angle(rh).f_original_curve_part.f_curve[count][0] - r_origin[0],
                arena.angle(rh).f_original_curve_part.f_curve[count][1] - r_origin[1],
            );
            let lh_angle = arena.angle(lh).clone();
            let original_side = SkOpAngle::line_on_one_side_of(
                r_origin,
                line,
                &lh_angle.f_original_curve_part.f_curve,
                lh_angle.f_original_curve_part.f_verb,
            );
            if original_side >= 0 {
                let translated_side = SkOpAngle::line_on_one_side_of(
                    r_origin,
                    line,
                    &lh_angle.f_part.f_curve,
                    lh_angle.f_part.f_verb,
                );
                if original_side != translated_side {
                    continue;
                }
            }
        }
        if delta > 1e-3 {
            use_intersect = !use_intersect;
            if use_intersect {
                s_ray_longer = ray_longer;
                s_cept = cept;
                s_cept_t = small_ts[index];
                s_index = index;
            }
        }
    }

    if !use_intersect || s_index == usize::MAX {
        return check_parallel(arena, lh, rh);
    }

    let (pts, verb, weight) = if s_index == 1 {
        (&r_pts, r_verb, r_weight)
    } else {
        (&l_pts, l_verb, l_weight)
    };
    let angle = if s_index == 1 { rh } else { lh };
    let curve_origin = arena.angle(angle).f_part.f_curve[0];
    let Some(start) = arena.angle(angle).f_start.map(SpanId::new) else {
        return check_parallel(arena, lh, rh);
    };
    let t_start = span_t(arena, start);
    let mid_pt = curve_pt_at_t(pts, verb, weight, t_start + (s_cept_t - t_start) / 2.0);
    let mid = AngleVector::new(
        mid_pt[0] - curve_origin[0],
        mid_pt[1] - curve_origin[1],
    );
    let sept_dir = mid.cross_check(s_cept);
    if sept_dir == 0.0 {
        return check_parallel(arena, lh, rh);
    }
    s_ray_longer ^ (s_index == 0) ^ (sept_dir < 0.0)
}

/// Returns true when `t` lies between `start` and `end`, in either order.
///
/// Port of `approximately_between_orderable`, which unlike `between` does not
/// require the bounds to be given low-to-high.
fn between_orderable(start: f64, t: f64, end: f64) -> bool {
    if start <= end {
        start <= t && t <= end
    } else {
        end <= t && t <= start
    }
}

/// Returns 1 when `lh` sorts before `rh`, 0 when after, -1 when neither can
/// be decided.
///
/// Port of `SkOpAngle::orderable`. The four branches are line/line,
/// line/curve, curve/line and curve/curve, each falling through to
/// [`ends_intersect`] when its own test is inconclusive.
pub fn orderable(arena: &mut OpArena, lh: AngleId, rh: AngleId) -> i32 {
    let lh_is_curve = arena.angle(lh).f_part.is_curve();
    let rh_is_curve = arena.angle(rh).f_part.is_curve();

    if !lh_is_curve {
        if !rh_is_curve {
            // Two lines: compare the tangent directions directly.
            let left_x = arena.angle(lh).f_tangent_half.dx();
            let left_y = arena.angle(lh).f_tangent_half.dy();
            let right_x = arena.angle(rh).f_tangent_half.dx();
            let right_y = arena.angle(rh).f_tangent_half.dy();
            let x_ry = left_x * right_y;
            let rx_y = right_x * left_y;
            if x_ry == rx_y {
                if left_x * right_x < 0.0 || left_y * right_y < 0.0 {
                    return 1; // exactly 180 degrees apart
                }
                return mark_unorderable(arena, lh, rh);
            }
            return i32::from(x_ry < rx_y);
        }
        let rh_angle = arena.angle(rh).clone();
        let result = arena.angle_mut(lh).line_on_one_side(&rh_angle, false);
        if result >= 0 {
            return result;
        }
        if arena.angle(lh).f_unorderable || approximately_zero(arena.angle(rh).f_side) {
            return mark_unorderable(arena, lh, rh);
        }
    } else if !rh_is_curve {
        let lh_angle = arena.angle(lh).clone();
        let result = arena.angle_mut(rh).line_on_one_side(&lh_angle, false);
        if result >= 0 {
            return i32::from(result == 0);
        }
        if arena.angle(rh).f_unorderable || approximately_zero(arena.angle(lh).f_side) {
            return mark_unorderable(arena, lh, rh);
        }
    } else {
        let result = convex_hull_overlaps(arena, lh, rh);
        if result >= 0 {
            return result;
        }
    }
    i32::from(ends_intersect(arena, lh, rh))
}

/// Marks both angles unorderable and reports that no order was found.
fn mark_unorderable(arena: &mut OpArena, lh: AngleId, rh: AngleId) -> i32 {
    arena.angle_mut(lh).f_unorderable = true;
    arena.angle_mut(rh).f_unorderable = true;
    -1
}

/// Runs the hull-overlap test with the mid-t vectors it needs.
fn convex_hull_overlaps(arena: &mut OpArena, lh: AngleId, rh: AngleId) -> i32 {
    let Some((l_pts, l_verb, l_weight)) = angle_curve(arena, lh) else {
        return -1;
    };
    let Some((r_pts, r_verb, r_weight)) = angle_curve(arena, rh) else {
        return -1;
    };
    let l_origin = arena.angle(lh).f_part.f_curve[0];
    let r_origin = arena.angle(rh).f_part.f_curve[0];
    let l_mid_pt = curve_pt_at_t(&l_pts, l_verb, l_weight, mid_t(arena, lh));
    let r_mid_pt = curve_pt_at_t(&r_pts, r_verb, r_weight, mid_t(arena, rh));
    let l_mid = AngleVector::new(l_mid_pt[0] - l_origin[0], l_mid_pt[1] - l_origin[1]);
    let r_mid = AngleVector::new(r_mid_pt[0] - r_origin[0], r_mid_pt[1] - r_origin[1]);
    let rh_angle = arena.angle(rh).clone();
    arena.angle_mut(lh).convex_hull_overlaps(
        &rh_angle,
        l_mid,
        r_mid,
        &l_pts,
        l_verb,
        &r_pts,
        r_verb,
    )
}

/// Returns true when `angle` lies in the counterclockwise arc from `test` to
/// `test.next`.
///
/// Port of `SkOpAngle::after`, the comparator the angle loop sorts with.
///
/// This is deliberately not "`angle`'s sector is greater". A ring has no
/// least element, so a plain `>` is not an ordering on one, and under `merge`
/// an angle that compares as belonging nowhere is unlinked without being
/// relinked — dropped silently. The three-way test below is what makes the
/// comparison well defined.
pub fn after(arena: &mut OpArena, angle: AngleId, test: AngleId) -> bool {
    let lh = test;
    let Some(rh) = arena.angle(lh).f_next.map(AngleId::new) else {
        return true;
    };
    debug_assert_ne!(lh, rh);

    // Move all three curves to a common origin so their directions can be
    // compared; this is what `fOriginalCurvePart` exists to undo.
    align_to(arena, angle, angle);
    align_to(arena, lh, angle);
    align_to(arena, rh, angle);

    if arena.angle(lh).f_compute_sector && !compute_sector(arena, lh) {
        return true;
    }
    if arena.angle(angle).f_compute_sector && !compute_sector(arena, angle) {
        return true;
    }
    if arena.angle(rh).f_compute_sector && !compute_sector(arena, rh) {
        return true;
    }

    let (a_mask, l_mask, r_mask) = (
        arena.angle(angle).f_sector_mask,
        arena.angle(lh).f_sector_mask,
        arena.angle(rh).f_sector_mask,
    );
    let (a_start, l_start, r_start) = (
        i32::from(arena.angle(angle).f_sector_start),
        i32::from(arena.angle(lh).f_sector_start),
        i32::from(arena.angle(rh).f_sector_start),
    );
    let l_end = i32::from(arena.angle(lh).f_sector_end);

    let ltr_overlap = ((l_mask | r_mask) & a_mask) != 0;
    let lr_overlap = (l_mask & r_mask) != 0;

    let lr_order: i32;
    if !lr_overlap {
        if !ltr_overlap {
            // None of the three share a sector, so the sector numbers alone
            // settle it.
            return (l_end > r_start) ^ (a_start > l_end) ^ (a_start > r_start);
        }
        lr_order = gap_order(r_start - l_start);
    } else {
        lr_order = orderable(arena, lh, rh);
        if !ltr_overlap && lr_order >= 0 {
            return lr_order == 0;
        }
    }

    let mut lt_order = if (l_mask & a_mask) != 0 {
        orderable(arena, lh, angle)
    } else {
        gap_order(a_start - l_start)
    };
    let mut tr_order = if (r_mask & a_mask) != 0 {
        orderable(arena, angle, rh)
    } else {
        gap_order(r_start - a_start)
    };

    let this_angle = arena.angle(angle).clone();
    this_angle.alignment_same_side(arena.angle(lh), &mut lt_order);
    this_angle.alignment_same_side(arena.angle(rh), &mut tr_order);

    if lr_order >= 0 && lt_order >= 0 && tr_order >= 0 {
        return if lr_order != 0 {
            (lt_order & tr_order) != 0
        } else {
            (lt_order | tr_order) != 0
        };
    }

    // Not enough information to sort outright. Where one pair is already
    // known to be in opposite half-planes, the remaining pair decides.
    if lt_order == 0 && lr_order == 0 {
        return arena.angle(lh).opposite_planes(arena.angle(angle));
    }
    if lt_order == 1 && tr_order == 0 {
        return arena.angle(angle).opposite_planes(arena.angle(rh));
    }
    if lr_order == 1 && tr_order == 1 {
        return arena.angle(lh).opposite_planes(arena.angle(rh));
    }

    // Last resort for lines: if exactly one pair shares a start point, the
    // third's position relative to that pair can still decide.
    if let Some(result) = original_side_order(arena, angle, lh, rh) {
        return result;
    }

    if lr_order < 0 {
        if lt_order < 0 {
            return tr_order != 0;
        }
        return lt_order != 0;
    }
    lr_order == 0
}

/// Returns the order implied by a gap of `raw` sectors, or -1 when the gap is
/// in the band where a tiny change could flip it.
///
/// A start can move by up to 4 sectors under rounding, so gaps of 12..20 in
/// either direction are not safe to read an order from.
fn gap_order(raw: i32) -> i32 {
    let gap = (raw + 32) & 0x1f;
    if gap > 20 {
        0
    } else if gap > 11 {
        -1
    } else {
        1
    }
}

/// Restores an angle's curve from its original and re-offsets it to sit at
/// `to`'s start point.
fn align_to(arena: &mut OpArena, angle: AngleId, to: AngleId) {
    let target = arena.angle(to).f_original_curve_part.f_curve[0];
    let original = arena.angle(angle).f_original_curve_part;
    let a = arena.angle_mut(angle);
    a.f_part.f_curve = original.f_curve;
    a.f_part.f_verb = original.f_verb;
    a.f_part.f_weight = original.f_weight;
    let origin = a.f_part.f_curve[0];
    a.f_part.offset(target[0] - origin[0], target[1] - origin[1]);
}

/// Tries to order three lines from their untranslated positions.
///
/// Port of the `fUnorderable` fallback at the end of `SkOpAngle::after`. It
/// applies only when all three are lines and exactly one pair shares a start
/// point, so the third touches neither of that pair.
fn original_side_order(
    arena: &mut OpArena,
    angle: AngleId,
    lh: AngleId,
    rh: AngleId,
) -> Option<bool> {
    if !arena.angle(angle).f_unorderable
        && !arena.angle(lh).f_unorderable
        && !arena.angle(rh).f_unorderable
    {
        return None;
    }
    // Restricted to lines: the curve cases are not known to need it.
    if arena.angle(angle).f_part.is_curve()
        || arena.angle(lh).f_part.is_curve()
        || arena.angle(rh).f_part.is_curve()
    {
        return None;
    }
    let a_origin = arena.angle(angle).f_original_curve_part.f_curve[0];
    let l_origin = arena.angle(lh).f_original_curve_part.f_curve[0];
    let r_origin = arena.angle(rh).f_original_curve_part.f_curve[0];
    let lt_share = i32::from(l_origin == a_origin);
    let lr_share = i32::from(l_origin == r_origin);
    let tr_share = i32::from(a_origin == r_origin);
    if lt_share + lr_share + tr_share != 1 {
        return None;
    }
    if lr_share == 1 {
        let this_angle = arena.angle(angle).clone();
        let lt = arena.angle_mut(lh).lines_on_original_side(&this_angle);
        let rt = arena.angle_mut(rh).lines_on_original_side(&this_angle);
        if (rt ^ lt) == 1 {
            return Some(lt != 0);
        }
    } else if tr_share == 1 {
        let lh_angle = arena.angle(lh).clone();
        let tl = arena.angle_mut(angle).lines_on_original_side(&lh_angle);
        let rl = arena.angle_mut(rh).lines_on_original_side(&lh_angle);
        if (tl ^ rl) == 1 {
            return Some(rl != 0);
        }
    } else {
        let this_angle = arena.angle(angle).clone();
        let tr = arena.angle_mut(rh).lines_on_original_side(&this_angle);
        let rh_angle = arena.angle(rh).clone();
        let lr = arena.angle_mut(lh).lines_on_original_side(&rh_angle);
        if (lr ^ tr) == 1 {
            return Some(tr != 0);
        }
    }
    None
}

// --- the angle loop ------------------------------------------------------

/// Returns the number of angles in `angle`'s loop.
///
/// Port of `SkOpAngle::loopCount`. An angle not yet in a loop counts 1.
#[must_use]
pub fn loop_count(arena: &OpArena, angle: AngleId) -> i32 {
    let mut count = 0;
    let mut next = Some(angle);
    loop {
        next = next.and_then(|n| arena.angle(n).f_next.map(AngleId::new));
        count += 1;
        match next {
            Some(n) if n != angle => {}
            _ => return count,
        }
    }
}

/// Returns true when `angle`'s reverse already sits in `loop_head`'s loop.
///
/// Port of `SkOpAngle::loopContains`. "Reverse" is the point: it looks for a
/// member on the same segment whose span pair runs the opposite way, which is
/// what marks the two as the same edge walked from either end.
#[must_use]
pub fn loop_contains(arena: &OpArena, loop_head: AngleId, angle: AngleId) -> bool {
    if arena.angle(loop_head).f_next.is_none() {
        return false;
    }
    let (Some(t_start_span), Some(t_end_span)) = (
        arena.angle(angle).f_start.map(SpanId::new),
        arena.angle(angle).f_end.map(SpanId::new),
    ) else {
        return false;
    };
    let Some(t_segment) = arena.span_segment(t_start_span) else {
        return false;
    };
    let t_start = span_t(arena, t_start_span);
    let t_end = span_t(arena, t_end_span);

    let mut current = loop_head;
    loop {
        if let (Some(l_start), Some(l_end)) = (
            arena.angle(current).f_start.map(SpanId::new),
            arena.angle(current).f_end.map(SpanId::new),
        ) {
            if arena.span_segment(l_start) == Some(t_segment)
                && span_t(arena, l_start) == t_end
                && span_t(arena, l_end) == t_start
            {
                return true;
            }
        }
        match arena.angle(current).f_next.map(AngleId::new) {
            Some(next) if next != loop_head => current = next,
            _ => return false,
        }
    }
}

/// Splices `angle` into `head`'s sorted loop.
///
/// Port of `SkOpAngle::insert`. Returns false only when the loop could not be
/// closed, which the caller treats as a failed sort.
pub fn insert(arena: &mut OpArena, head: AngleId, angle: AngleId) -> bool {
    if arena.angle(angle).f_next.is_some() {
        // Both are already loops; fold the smaller into the larger.
        if loop_count(arena, head) >= loop_count(arena, angle) {
            if !merge(arena, head, angle) {
                return true;
            }
        } else if arena.angle(head).f_next.is_some() {
            if !merge(arena, angle, head) {
                return true;
            }
        } else {
            insert(arena, angle, head);
        }
        return true;
    }
    let singleton = arena.angle(head).f_next.is_none();
    if singleton {
        arena.angle_mut(head).f_next = Some(head.index());
    }
    let next = AngleId::new(arena.angle(head).f_next.expect("loop closed above"));
    if arena.angle(next).f_next == Some(head.index()) {
        // A one- or two-element loop: there is only one place to go.
        if singleton || after(arena, angle, head) {
            arena.angle_mut(head).f_next = Some(angle.index());
            arena.angle_mut(angle).f_next = Some(next.index());
        } else {
            arena.angle_mut(next).f_next = Some(angle.index());
            arena.angle_mut(angle).f_next = Some(head.index());
        }
        return true;
    }

    let mut last = head;
    let mut next = next;
    let mut flip_ambiguity = false;
    loop {
        debug_assert_eq!(arena.angle(last).f_next, Some(next.index()));
        let ambiguous = arena.angle(angle).tangents_ambiguous() && flip_ambiguity;
        if after(arena, angle, last) ^ ambiguous {
            arena.angle_mut(last).f_next = Some(angle.index());
            arena.angle_mut(angle).f_next = Some(next.index());
            return true;
        }
        last = next;
        if last == head {
            if flip_ambiguity {
                return false;
            }
            // All the way round with no home. If a comparison was ambiguous,
            // flip it so the next pass terminates.
            flip_ambiguity = true;
        }
        next = match arena.angle(next).f_next.map(AngleId::new) {
            Some(n) => n,
            None => return false,
        };
    }
}

/// Folds every angle in `angle`'s loop into `head`'s loop.
///
/// Port of `SkOpAngle::merge`. Returns false when the two are already the
/// same loop, which is not an error: there is nothing to do.
pub fn merge(arena: &mut OpArena, head: AngleId, angle: AngleId) -> bool {
    debug_assert!(arena.angle(head).f_next.is_some());
    debug_assert!(arena.angle(angle).f_next.is_some());
    let mut working = angle;
    loop {
        if head == working {
            return false;
        }
        working = match arena.angle(working).f_next.map(AngleId::new) {
            Some(n) => n,
            None => return false,
        };
        if working == angle {
            break;
        }
    }
    let mut working = angle;
    loop {
        let next = arena.angle(working).f_next.map(AngleId::new);
        // Unlink before inserting: insert takes the "not yet in a loop" path
        // only for an angle whose next is clear.
        arena.angle_mut(working).f_next = None;
        insert(arena, head, working);
        working = match next {
            Some(n) => n,
            None => break,
        };
        if working == angle {
            break;
        }
    }
    true
}

/// Returns the angle before `angle` in its loop.
///
/// Port of `SkOpAngle::previous`, which walks the ring rather than keeping a
/// back pointer; it is called rarely enough not to warrant one.
#[must_use]
pub fn previous(arena: &OpArena, angle: AngleId) -> Option<AngleId> {
    let mut last = AngleId::new(arena.angle(angle).f_next?);
    loop {
        let next = AngleId::new(arena.angle(last).f_next?);
        if next == angle {
            return Some(last);
        }
        last = next;
    }
}

/// Returns the angle's last marked span, claiming it so it is returned once.
///
/// Port of `SkOpAngle::lastMarked`. The chase walk uses this to avoid
/// revisiting a span another branch already took.
pub fn last_marked(arena: &mut OpArena, angle: AngleId) -> Option<SpanId> {
    let marked = SpanId::new(arena.angle(angle).f_last_marked?);
    if arena.span(marked).chased() {
        return None;
    }
    arena.span_mut(marked).set_chased(true);
    Some(marked)
}

/// Builds the angles for every span of `segment`.
///
/// Port of `SkOpSegment::calcAngles`. Each interior span gets an angle
/// leaving it and one arriving at it; a cancelled span gets neither, since
/// nothing walks through it.
pub fn calc_angles(arena: &mut OpArena, segment: SegmentId) {
    let Some(head) = arena.segment(segment).f_head else {
        return;
    };
    let tail = arena.segment(segment).f_tail;
    let mut active_prior = !arena.span(head).is_canceled();
    if active_prior && !span_is_simple(arena, head) {
        add_start_span(arena, segment);
    }
    let mut prior = head;
    let mut span_base = arena.span_next(head);
    while let Some(current) = span_base {
        if Some(current) == tail {
            break;
        }
        if active_prior {
            let prior_angle = arena.alloc_angle(SkOpAngle::new());
            set(arena, prior_angle, current, prior);
            arena.span_set_from_angle(current, Some(prior_angle));
        }
        let active = !arena.span(current).is_canceled();
        let next = arena.span_next(current);
        if active {
            if let Some(next) = next {
                let angle = arena.alloc_angle(SkOpAngle::new());
                set(arena, angle, current, next);
                arena.span_set_to_angle(current, Some(angle));
            }
        }
        active_prior = active;
        prior = current;
        span_base = next;
    }
    if let Some(tail) = tail {
        if active_prior && !span_is_simple(arena, tail) {
            add_end_span(arena, segment);
        }
    }
}

/// Returns true when the span's PtT ring holds only itself.
///
/// Port of `SkOpSpanBase::simple`. A simple span is not shared with any other
/// segment, so it needs no angle: nothing can turn there.
fn span_is_simple(arena: &OpArena, span: SpanId) -> bool {
    let Some(ptt) = arena.span_ptt(span) else {
        return true;
    };
    arena.ptt_next(arena.ptt_next(ptt)) == ptt
}

/// Gives the segment's head span an angle leaving it.
///
/// Port of `SkOpSegment::addStartSpan`.
fn add_start_span(arena: &mut OpArena, segment: SegmentId) -> Option<AngleId> {
    let head = arena.segment(segment).f_head?;
    let next = arena.span_next(head)?;
    let angle = arena.alloc_angle(SkOpAngle::new());
    set(arena, angle, head, next);
    arena.span_set_to_angle(head, Some(angle));
    Some(angle)
}

/// Gives the segment's tail span an angle arriving at it.
///
/// Port of `SkOpSegment::addEndSpan`.
fn add_end_span(arena: &mut OpArena, segment: SegmentId) -> Option<AngleId> {
    let tail = arena.segment(segment).f_tail?;
    let prev = arena.span_prev(tail)?;
    let angle = arena.alloc_angle(SkOpAngle::new());
    set(arena, angle, tail, prev);
    arena.span_set_from_angle(tail, Some(angle));
    Some(angle)
}

/// Guards against a malformed PtT ring spinning forever.
///
/// The C++ carries the same counter; a ring that does not close is a bug
/// elsewhere, and the sort reports failure rather than hanging.
const SORT_SAFETY_NET: i32 = 1_000_000;

/// Sorts every angle loop on `segment` into counterclockwise order.
///
/// Port of `SkOpSegment::sortAngles`. For each span it joins the angle
/// arriving and the angle leaving into one loop, then folds in the angles of
/// every other segment meeting at the same point. Returns false when a loop
/// could not be closed.
pub fn sort_angles(arena: &mut OpArena, segment: SegmentId) -> bool {
    let mut span = arena.segment(segment).f_head;
    while let Some(current) = span {
        if !sort_span_angles(arena, current) {
            return false;
        }
        if arena.span_is_final(current) {
            break;
        }
        span = arena.span_next(current);
    }
    true
}

/// Sorts the angles meeting at one span.
fn sort_span_angles(arena: &mut OpArena, span: SpanId) -> bool {
    let from_angle = arena.span_from_angle(span);
    let to_angle = if arena.span_is_final(span) {
        None
    } else {
        arena.span_to_angle(span)
    };
    let (Some(base_angle), _) = (from_angle.or(to_angle), ()) else {
        return true;
    };
    if let (Some(from), Some(to)) = (from_angle, to_angle) {
        if !insert(arena, from, to) {
            return false;
        }
    }

    // Walk the PtT ring: every other span at this point contributes its own
    // angles to the same loop.
    let Some(stop_ptt) = arena.span_ptt(span) else {
        return true;
    };
    let mut ptt = stop_ptt;
    let mut safety = SORT_SAFETY_NET;
    loop {
        safety -= 1;
        if safety == 0 {
            return false;
        }
        if let Some(o_span) = arena.ptt_span(ptt) {
            if o_span != span {
                if let Some(o_angle) = arena.span_from_angle(o_span) {
                    if !loop_contains(arena, o_angle, base_angle) {
                        insert(arena, base_angle, o_angle);
                    }
                }
                if !arena.span_is_final(o_span) {
                    if let Some(o_angle) = arena.span_to_angle(o_span) {
                        if !loop_contains(arena, o_angle, base_angle) {
                            insert(arena, base_angle, o_angle);
                        }
                    }
                }
            }
        }
        ptt = arena.ptt_next(ptt);
        if ptt == stop_ptt {
            break;
        }
    }

    // A loop of one means nothing actually meets here, so drop the angles
    // again rather than leaving a degenerate loop for the walker to find.
    if loop_count(arena, base_angle) == 1 {
        arena.span_set_from_angle(span, None);
        if to_angle.is_some() {
            arena.span_set_to_angle(span, None);
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point;

    /// Builds a segment leaving the origin in the given direction, and the
    /// angle for walking it from its head.
    fn spoke(arena: &mut OpArena, dx: f32, dy: f32) -> AngleId {
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(dx, dy)],
            Verb::Line,
            1.0,
        );
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let angle = arena.alloc_angle(SkOpAngle::new());
        set(arena, angle, head, tail);
        angle
    }

    #[test]
    fn set_records_the_span_pair_and_its_curve() {
        let mut arena = OpArena::new();
        let angle = spoke(&mut arena, 10.0, 0.0);
        assert!(arena.angle(angle).f_start.is_some());
        assert!(arena.angle(angle).f_end.is_some());
        assert_eq!(arena.angle(angle).f_part.f_curve[0], [0.0, 0.0]);
        assert_eq!(arena.angle(angle).f_part.f_curve[1], [10.0, 0.0]);
    }

    #[test]
    fn set_assigns_a_sector_to_each_compass_direction() {
        let mut arena = OpArena::new();
        // A sector is 1/32 of a circle, so four cardinal directions must land
        // on four different ones.
        let mut seen = Vec::new();
        for (dx, dy) in [(10.0, 0.0), (0.0, 10.0), (-10.0, 0.0), (0.0, -10.0)] {
            let angle = spoke(&mut arena, dx, dy);
            let sector = arena.angle(angle).f_sector_start;
            assert!((0..32).contains(&i32::from(sector)), "sector {sector}");
            seen.push(sector);
        }
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), 4, "four directions, four sectors");
    }

    #[test]
    fn a_line_is_not_reported_as_a_curve() {
        let mut arena = OpArena::new();
        let angle = spoke(&mut arena, 3.0, 4.0);
        assert!(!arena.angle(angle).f_part.is_curve());
        assert_eq!(arena.angle(angle).f_side, 0.0);
    }

    #[test]
    fn a_bulging_quad_is_reported_as_a_curve_with_a_side() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[
                Point::new(0.0, 0.0),
                Point::new(5.0, 10.0),
                Point::new(10.0, 0.0),
            ],
            Verb::Quad,
            1.0,
        );
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let angle = arena.alloc_angle(SkOpAngle::new());
        set(&mut arena, angle, head, tail);
        assert!(arena.angle(angle).f_part.is_curve());
        assert_ne!(
            arena.angle(angle).f_side,
            0.0,
            "a quad that bulges has a side"
        );
    }

    #[test]
    fn a_collinear_quad_collapses_to_a_line() {
        let mut arena = OpArena::new();
        // The control point sits exactly on the chord.
        let seg = arena.alloc_segment_with_curve(
            &[
                Point::new(0.0, 0.0),
                Point::new(5.0, 0.0),
                Point::new(10.0, 0.0),
            ],
            Verb::Quad,
            1.0,
        );
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let angle = arena.alloc_angle(SkOpAngle::new());
        set(&mut arena, angle, head, tail);
        assert!(
            !arena.angle(angle).f_part.is_curve(),
            "a flat quad sorts as a line"
        );
        assert_eq!(arena.angle(angle).f_side, 0.0);
    }

    #[test]
    fn two_perpendicular_lines_are_orderable_both_ways() {
        let mut arena = OpArena::new();
        let east = spoke(&mut arena, 10.0, 0.0);
        let north = spoke(&mut arena, 0.0, 10.0);
        let forward = orderable(&mut arena, east, north);
        let backward = orderable(&mut arena, north, east);
        assert!(forward >= 0, "a right angle is decidable");
        assert!(backward >= 0);
        assert_ne!(
            forward, backward,
            "the order must reverse when the operands swap"
        );
    }

    #[test]
    fn opposite_lines_are_180_degrees_apart() {
        let mut arena = OpArena::new();
        let east = spoke(&mut arena, 10.0, 0.0);
        let west = spoke(&mut arena, -10.0, 0.0);
        // The C++ reports 1 for exactly antiparallel rather than giving up.
        assert_eq!(orderable(&mut arena, east, west), 1);
    }

    #[test]
    fn a_line_and_a_quad_leaving_together_are_orderable() {
        let mut arena = OpArena::new();
        let line = spoke(&mut arena, 10.0, 0.0);
        let seg = arena.alloc_segment_with_curve(
            &[
                Point::new(0.0, 0.0),
                Point::new(5.0, 5.0),
                Point::new(10.0, 0.0),
            ],
            Verb::Quad,
            1.0,
        );
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let quad = arena.alloc_angle(SkOpAngle::new());
        set(&mut arena, quad, head, tail);
        let order = orderable(&mut arena, line, quad);
        assert!(order >= 0, "a line against a bulging quad is decidable");
        // lineOnOneSide returns `cross < 0`. The line runs east, so
        // line = (10, 0); the quad's control point (5, 5) gives
        // cross = 10*5 - 0*5 = 50, which is positive, so the answer is 0.
        // The quad lies counterclockwise of the line.
        assert_eq!(order, 0);

        // Swapping the operands must swap the answer.
        assert_eq!(orderable(&mut arena, quad, line), 1);
    }

    #[test]
    fn a_quad_bulging_the_other_way_reverses_the_order() {
        let mut arena = OpArena::new();
        let line = spoke(&mut arena, 10.0, 0.0);
        // The mirror of the previous test: the control point is below.
        let seg = arena.alloc_segment_with_curve(
            &[
                Point::new(0.0, 0.0),
                Point::new(5.0, -5.0),
                Point::new(10.0, 0.0),
            ],
            Verb::Quad,
            1.0,
        );
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let quad = arena.alloc_angle(SkOpAngle::new());
        set(&mut arena, quad, head, tail);
        assert_eq!(
            orderable(&mut arena, line, quad),
            1,
            "mirroring the quad across the line must flip the order"
        );
    }

    #[test]
    fn gap_order_refuses_the_band_where_rounding_could_flip_it() {
        // A start can move +/- 4 sectors, so 12..20 either way is unsafe.
        assert_eq!(gap_order(1), 1);
        assert_eq!(gap_order(11), 1);
        assert_eq!(gap_order(12), -1);
        assert_eq!(gap_order(20), -1);
        assert_eq!(gap_order(21), 0);
        assert_eq!(gap_order(31), 0);
        assert_eq!(gap_order(-1), 0, "-1 wraps to 31");
        assert_eq!(gap_order(-12), -1);
    }

    #[test]
    fn between_orderable_accepts_a_reversed_range() {
        assert!(between_orderable(0.0, 0.5, 1.0));
        assert!(between_orderable(1.0, 0.5, 0.0));
        assert!(!between_orderable(0.0, 1.5, 1.0));
        assert!(!between_orderable(1.0, 1.5, 0.0));
    }

    #[test]
    fn after_puts_a_spoke_in_the_arc_it_belongs_to() {
        let mut arena = OpArena::new();
        // A two-angle loop: east, then north.
        let east = spoke(&mut arena, 10.0, 0.0);
        let north = spoke(&mut arena, 0.0, 10.0);
        arena.angle_mut(east).f_next = Some(north.index());
        arena.angle_mut(north).f_next = Some(east.index());
        // A spoke between them and one outside must not answer the same way.
        let north_east = spoke(&mut arena, 10.0, 10.0);
        let south = spoke(&mut arena, 0.0, -10.0);
        let inside = after(&mut arena, north_east, east);
        let outside = after(&mut arena, south, east);
        assert_ne!(
            inside, outside,
            "a spoke inside the east-to-north arc must sort differently \
             from one outside it"
        );
    }

    #[test]
    fn compute_sector_on_a_lone_segment_reports_unorderable() {
        let mut arena = OpArena::new();
        // Nothing to lengthen into: the segment's end is final.
        let angle = spoke(&mut arena, 10.0, 0.0);
        assert!(!compute_sector(&mut arena, angle));
        assert!(arena.angle(angle).f_unorderable);
    }

    #[test]
    fn compute_sector_is_computed_only_once() {
        let mut arena = OpArena::new();
        let angle = spoke(&mut arena, 10.0, 0.0);
        let first = compute_sector(&mut arena, angle);
        // The second call must short-circuit on f_computed_sector rather than
        // redo the walk, and give the same answer.
        assert!(arena.angle(angle).f_computed_sector);
        assert_eq!(compute_sector(&mut arena, angle), first);
    }

    #[test]
    fn loop_count_of_a_lone_angle_is_one() {
        let mut arena = OpArena::new();
        let angle = spoke(&mut arena, 10.0, 0.0);
        assert_eq!(loop_count(&arena, angle), 1);
    }

    #[test]
    fn insert_builds_a_closed_ring_of_every_spoke() {
        let mut arena = OpArena::new();
        let head = spoke(&mut arena, 10.0, 0.0);
        let others: Vec<AngleId> = [(0.0, 10.0), (-10.0, 0.0), (0.0, -10.0)]
            .iter()
            .map(|&(dx, dy)| spoke(&mut arena, dx, dy))
            .collect();
        for &angle in &others {
            assert!(insert(&mut arena, head, angle));
        }
        assert_eq!(loop_count(&arena, head), 4, "all four spokes are in");

        // Walking next four times must come back to the start, and must have
        // visited each angle exactly once.
        let mut seen = Vec::new();
        let mut current = head;
        for _ in 0..4 {
            seen.push(current);
            current = AngleId::new(arena.angle(current).f_next.expect("ring is closed"));
        }
        assert_eq!(current, head, "the ring closes");
        seen.sort_unstable_by_key(|a| a.index());
        seen.dedup();
        assert_eq!(seen.len(), 4, "no angle appears twice");
    }

    #[test]
    fn the_sorted_ring_is_in_counterclockwise_order() {
        let mut arena = OpArena::new();
        // Insert four spokes out of order and check the ring comes out
        // monotone by sector, which is what a correct sort means here.
        let head = spoke(&mut arena, 10.0, 0.0);
        for (dx, dy) in [(-10.0, 0.0), (0.0, 10.0), (0.0, -10.0)] {
            let a = spoke(&mut arena, dx, dy);
            insert(&mut arena, head, a);
        }
        let mut sectors = Vec::new();
        let mut current = head;
        for _ in 0..4 {
            sectors.push(i32::from(arena.angle(current).f_sector_start));
            current = AngleId::new(arena.angle(current).f_next.expect("closed"));
        }
        // Rotate so the smallest sector leads, then the rest must ascend: a
        // ring sorted counterclockwise has exactly one wrap point.
        let min_at = sectors
            .iter()
            .enumerate()
            .min_by_key(|(_, &s)| s)
            .map(|(i, _)| i)
            .expect("four sectors");
        sectors.rotate_left(min_at);
        assert!(
            sectors.windows(2).all(|w| w[0] < w[1]),
            "sectors are not counterclockwise: {sectors:?}"
        );
    }

    #[test]
    fn previous_walks_back_around_the_ring() {
        let mut arena = OpArena::new();
        let head = spoke(&mut arena, 10.0, 0.0);
        let second = spoke(&mut arena, 0.0, 10.0);
        let third = spoke(&mut arena, -10.0, 0.0);
        insert(&mut arena, head, second);
        insert(&mut arena, head, third);
        // previous(next(x)) is x for every member.
        let mut current = head;
        for _ in 0..3 {
            let next = AngleId::new(arena.angle(current).f_next.expect("closed"));
            assert_eq!(previous(&arena, next), Some(current));
            current = next;
        }
    }

    #[test]
    fn merge_folds_two_rings_into_one() {
        let mut arena = OpArena::new();
        let a1 = spoke(&mut arena, 10.0, 0.0);
        let a2 = spoke(&mut arena, 1.0, 10.0);
        insert(&mut arena, a1, a2);
        let b1 = spoke(&mut arena, -10.0, 1.0);
        let b2 = spoke(&mut arena, -1.0, -10.0);
        insert(&mut arena, b1, b2);
        assert_eq!(loop_count(&arena, a1), 2);
        assert_eq!(loop_count(&arena, b1), 2);

        assert!(merge(&mut arena, a1, b1));
        assert_eq!(loop_count(&arena, a1), 4, "all four ended up in one ring");
    }

    #[test]
    fn merging_a_ring_with_itself_is_a_no_op() {
        let mut arena = OpArena::new();
        let a1 = spoke(&mut arena, 10.0, 0.0);
        let a2 = spoke(&mut arena, 0.0, 10.0);
        insert(&mut arena, a1, a2);
        assert!(
            !merge(&mut arena, a1, a2),
            "already the same loop, so nothing to fold"
        );
        assert_eq!(loop_count(&arena, a1), 2, "and nothing was lost");
    }

    #[test]
    fn last_marked_is_handed_out_only_once() {
        let mut arena = OpArena::new();
        let angle = spoke(&mut arena, 10.0, 0.0);
        let span = SpanId::new(arena.angle(angle).f_start.expect("start"));
        arena.angle_mut(angle).f_last_marked = Some(span.index());
        assert_eq!(last_marked(&mut arena, angle), Some(span));
        assert_eq!(
            last_marked(&mut arena, angle),
            None,
            "the second ask finds it already chased"
        );
    }

    #[test]
    fn calc_angles_gives_an_interior_span_both_directions() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        arena
            .segment_add_t(seg, 0.5, Point::new(5.0, 0.0))
            .expect("split");
        // A span needs winding to count as active.
        for span in arena.segment_spans(seg) {
            arena.span_mut(span).set_wind_value(1);
        }
        calc_angles(&mut arena, seg);
        let head = arena.segment(seg).f_head.expect("head");
        let mid = arena.span_next(head).expect("mid");
        assert!(
            arena.span_to_angle(mid).is_some(),
            "the interior span has an angle leaving it"
        );
        assert!(
            arena.span_from_angle(mid).is_some(),
            "and one arriving at it"
        );
    }

    #[test]
    fn calc_angles_skips_a_cancelled_span() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        arena
            .segment_add_t(seg, 0.5, Point::new(5.0, 0.0))
            .expect("split");
        // Leaving wind values at zero cancels every span.
        calc_angles(&mut arena, seg);
        let head = arena.segment(seg).f_head.expect("head");
        let mid = arena.span_next(head).expect("mid");
        assert!(
            arena.span_to_angle(mid).is_none(),
            "nothing walks through a cancelled span, so it needs no angle"
        );
    }

    #[test]
    fn sort_angles_drops_a_loop_of_one() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        arena
            .segment_add_t(seg, 0.5, Point::new(5.0, 0.0))
            .expect("split");
        for span in arena.segment_spans(seg) {
            arena.span_mut(span).set_wind_value(1);
        }
        calc_angles(&mut arena, seg);
        assert!(sort_angles(&mut arena, seg));
        let head = arena.segment(seg).f_head.expect("head");
        let mid = arena.span_next(head).expect("mid");
        // Nothing else meets at the midpoint, so the two angles there form a
        // loop of two; the sort keeps it. What it must not leave behind is a
        // loop of one.
        if let Some(a) = arena.span_from_angle(mid) {
            assert!(loop_count(&arena, a) > 1, "a kept loop has real members");
        }
    }

    #[test]
    fn loop_contains_finds_the_reverse_walk_of_the_same_edge() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let forward = arena.alloc_angle(SkOpAngle::new());
        set(&mut arena, forward, head, tail);
        let backward = arena.alloc_angle(SkOpAngle::new());
        set(&mut arena, backward, tail, head);
        // Put forward in a loop of its own so loop_contains has one to walk.
        arena.angle_mut(forward).f_next = Some(forward.index());
        assert!(
            loop_contains(&arena, forward, backward),
            "the same edge walked the other way is already present"
        );
    }
}
