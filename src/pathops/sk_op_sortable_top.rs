//! Finding a span whose winding can be resolved, by casting rays at it.
//!
//! Port of `SkPathOpsWinding.cpp`: `SkOpRayHit`, `SkOpSegment::rayCheck`,
//! `SkOpSpan::sortableTop`, the two `findSortableTop` wrappers, and
//! `FindSortableTop` itself.
//!
//! # What it is for
//!
//! The walker has to start somewhere, and it can only start from a span
//! whose winding is known. `FindSortableTop` picks that span: it fires a ray
//! from a candidate point, counts what the ray crosses on the way out, and
//! reads the winding off the running total. A span on the outside of
//! everything has winding zero, and every crossing on the way in changes it.
//!
//! # Why it retries
//!
//! A ray is only usable if it crosses cleanly. A ray that grazes a curve
//! tangentially, or passes through a point two segments share, gives a count
//! that could go either way. Rather than guess, `sortableTop` reports failure
//! and the caller tries again with a different t and a different direction —
//! up to [`MAX_WINDING_TRIES`](super::sk_op_arena::MAX_WINDING_TRIES) times
//! across every span, and then gives up rather than looping.

use super::sk_curve_intersect_ray::{curve_d_slope_at_t, curve_intercept_h, curve_intercept_v};
use super::sk_op_arena::{use_inner_winding, OpArena, SegmentId, SpanId, MAX_WINDING_TRIES};
use super::sk_op_span::PK_MIN_S32;
use super::sk_path_ops_winding::SkOpRayDir;
use crate::core::{Point, Verb};

/// Where a ray crossed a segment, and whether the crossing is usable.
#[derive(Debug, Clone, Copy)]
pub struct SkOpRayHit {
    /// The span whose interval the crossing falls in.
    pub span: Option<SpanId>,
    /// The point of the crossing.
    pub pt: Point,
    /// Parameter along the crossed segment.
    pub t: f64,
    /// The segment's tangent at the crossing.
    pub slope: [f64; 2],
    /// False when the crossing is too close to a tangent or an endpoint to
    /// read a winding from.
    pub valid: bool,
}

/// Returns the coordinate of `pt` along the ray's own axis.
fn pt_xy(pt: Point, dir: SkOpRayDir) -> f32 {
    if dir.xy_index() == 0 {
        pt.x
    } else {
        pt.y
    }
}

/// Returns the coordinate of `pt` across the ray's axis.
fn pt_yx(pt: Point, dir: SkOpRayDir) -> f32 {
    if dir.xy_index() == 0 {
        pt.y
    } else {
        pt.x
    }
}

/// Returns the component of `v` along the ray's axis.
fn pt_dxdy(v: [f64; 2], dir: SkOpRayDir) -> f64 {
    v[dir.xy_index()]
}

/// Returns the component of `v` across the ray's axis.
fn pt_dydx(v: [f64; 2], dir: SkOpRayDir) -> f64 {
    v[dir.perp_index()]
}

/// Returns the side of `bounds` the ray points at.
///
/// Port of `rect_side`, which indexes the rect by the direction's own
/// discriminant: left, top, right, bottom in that order.
fn rect_side(bounds: (f32, f32, f32, f32), dir: SkOpRayDir) -> f32 {
    match dir {
        SkOpRayDir::Left => bounds.0,
        SkOpRayDir::Top => bounds.1,
        SkOpRayDir::Right => bounds.2,
        SkOpRayDir::Bottom => bounds.3,
    }
}

/// Returns true when the bounds straddle the ray's line.
///
/// Port of `sideways_overlap`. A segment whose box does not reach across the
/// ray cannot be crossed by it, whatever its shape.
fn sideways_overlap(bounds: (f32, f32, f32, f32), pt: Point, dir: SkOpRayDir) -> bool {
    let (lo, hi, v) = if dir.perp_index() == 0 {
        (bounds.0, bounds.2, pt.x)
    } else {
        (bounds.1, bounds.3, pt.y)
    };
    lo <= v && v <= hi
}

/// Returns true when the crossing runs counterclockwise relative to the ray.
///
/// Port of `ccw_dxdy`. The sign of the winding contribution: a segment
/// crossing the ray one way adds, the other way subtracts.
fn ccw_dxdy(v: [f64; 2], dir: SkOpRayDir) -> bool {
    let v_part_pos = pt_dydx(v, dir) > 0.0;
    let left_bottom = ((dir as i32 + 1) & 2) != 0;
    v_part_pos == left_bottom
}

/// Returns true when `a` and `b` agree to within an f32-scale tolerance.
fn approx_equal(a: f32, b: f32) -> bool {
    (a - b).abs() < 1e-5
}

/// Returns true when the two points are the same to within tolerance.
fn points_approx_equal(a: Point, b: Point) -> bool {
    approx_equal(a.x, b.x) && approx_equal(a.y, b.y)
}

/// Returns the sequence of `(t, direction offset)` guesses to try.
///
/// Port of `get_t_guess`. The t values walk a binary subdivision of the span
/// — 1/2, then 1/4 and 3/4, then the eighths — so successive tries sample
/// genuinely different places rather than creeping along from one end. The
/// low bit of the try count alternates the ray direction.
#[must_use]
pub fn t_guess(t_try: i32) -> (f64, usize) {
    let dir_offset = (t_try & 1) as usize;
    let mut t = 0.5;
    let t_base = t_try >> 1;
    let mut t_bits = 0;
    let mut shifting = t_try;
    while {
        shifting >>= 1;
        shifting != 0
    } {
        t /= 2.0;
        t_bits += 1;
    }
    if t_bits > 0 {
        let t_index = (t_base - 1) & ((1 << t_bits) - 1);
        t += t * 2.0 * f64::from(t_index);
    }
    (t, dir_offset)
}

/// Returns the span whose interval contains `t_hit`, or `None` when the hit
/// lands exactly on a span boundary.
///
/// Port of `SkOpSegment::windingSpanAtT`. A hit on a boundary is refused on
/// purpose: it belongs to two spans equally, so neither can claim its
/// winding contribution.
#[must_use]
pub fn winding_span_at_t(arena: &OpArena, segment: SegmentId, t_hit: f64) -> Option<SpanId> {
    let mut span = arena.segment(segment).f_head?;
    loop {
        let next = arena.span_next(span)?;
        let next_t = f64::from(arena.span(next).f_t);
        if (t_hit - next_t).abs() < 1e-5 {
            return None;
        }
        if t_hit < next_t {
            return Some(span);
        }
        if arena.span_is_final(next) {
            return None;
        }
        span = next;
    }
}

/// Collects every crossing of the ray from `base` in `dir` against `segment`.
///
/// Port of `SkOpSegment::rayCheck`.
fn ray_check(
    arena: &OpArena,
    base: &SkOpRayHit,
    base_segment: Option<SegmentId>,
    dir: SkOpRayDir,
    segment: SegmentId,
    hits: &mut Vec<SkOpRayHit>,
) {
    let bounds = arena.segment_bounds(segment);
    if !sideways_overlap(bounds, base.pt, dir) {
        return;
    }
    let base_xy = pt_xy(base.pt, dir);
    let bounds_xy = rect_side(bounds, dir);
    let check_less_than = dir.less_than();
    // The whole segment lies behind the ray's origin.
    if !approx_equal(base_xy, bounds_xy) && (base_xy < bounds_xy) == check_less_than {
        return;
    }

    let (pts, verb, weight) = arena.segment_curve(segment);
    let dpts: Vec<[f64; 2]> = pts
        .iter()
        .map(|p| [f64::from(p.x), f64::from(p.y)])
        .collect();
    let base_yx = f64::from(pt_yx(base.pt, dir));
    let crossings = if dir.xy_index() == 0 {
        curve_intercept_h(&dpts, verb, f64::from(weight), base_yx)
    } else {
        curve_intercept_v(&dpts, verb, f64::from(weight), base_yx)
    };

    let last_point = pts[pts.len() - 1];
    for index in 0..crossings.used() {
        let t = crossings.t(index);
        // The ray's own origin is not a crossing of the segment it came from.
        if base_segment == Some(segment) && (base.t - t).abs() < 1e-5 {
            continue;
        }
        let mut slope = [0.0f64, 0.0];
        let mut valid = false;
        let pt;
        if t.abs() < 1e-5 {
            pt = pts[0];
        } else if (t - 1.0).abs() < 1e-5 {
            pt = last_point;
        } else {
            pt = arena.segment_pt_at_t(segment, t as f32);
            if points_approx_equal(pt, base.pt) {
                if base_segment == Some(segment) {
                    continue;
                }
            } else {
                let pt_xy_v = pt_xy(pt, dir);
                // Behind the ray's origin.
                if !approx_equal(base_xy, pt_xy_v) && (base_xy < pt_xy_v) == check_less_than {
                    continue;
                }
                slope = curve_d_slope_at_t(&dpts, verb, f64::from(weight), t);
                // A crossing whose tangent is nearly along the ray is not a
                // crossing that can be counted: which side it leaves on is
                // decided by rounding. The 10000 is Skia's.
                if (pt_dydx(slope, dir) * 10000.0).abs() > pt_dxdy(slope, dir).abs() {
                    valid = true;
                }
            }
        }
        let span = winding_span_at_t(arena, segment, t);
        match span {
            None => valid = false,
            Some(s) => {
                // A span contributing nothing is not a crossing at all.
                if arena.span(s).wind_value() == 0 && arena.span(s).opp_value() == 0 {
                    continue;
                }
            }
        }
        hits.push(SkOpRayHit {
            span,
            pt,
            t,
            slope,
            valid,
        });
    }
}

/// Tries to resolve `span`'s winding by casting one ray at the whole graph.
///
/// Port of `SkOpSpan::sortableTop`. Returns false when the ray it chose was
/// not usable, which is a signal to try again with a different one, not an
/// error.
pub fn sortable_top(arena: &mut OpArena, span: SpanId, segments: &[SegmentId]) -> bool {
    let t_try = arena.span(span).top_t_try();
    arena.span_mut(span).bump_top_t_try();
    let (t, dir_offset) = t_guess(t_try);

    let Some(segment) = arena.span_segment(span) else {
        return false;
    };
    let Some(next) = arena.span_next(span) else {
        return false;
    };
    // The base point is t of the way along this span's own interval.
    let base_t = f64::from(arena.span(span).f_t) * (1.0 - t)
        + f64::from(arena.span(next).f_t) * t;
    let (pts, verb, weight) = arena.segment_curve(segment);
    let dpts: Vec<[f64; 2]> = pts
        .iter()
        .map(|p| [f64::from(p.x), f64::from(p.y)])
        .collect();
    let slope = curve_d_slope_at_t(&dpts, verb, f64::from(weight), base_t);
    if slope[0] == 0.0 && slope[1] == 0.0 {
        return false;
    }
    // Fire across the steeper axis, so the ray meets the curve squarely.
    let mut dir = if slope[0].abs() < slope[1].abs() {
        SkOpRayDir::Left
    } else {
        SkOpRayDir::Top
    };
    dir = dir.rotate(dir_offset);

    let base = SkOpRayHit {
        span: Some(span),
        pt: arena.segment_pt_at_t(segment, base_t as f32),
        t: base_t,
        slope,
        valid: true,
    };
    // A curve whose tangent runs along the ray gives no usable crossing.
    if verb != Verb::Line && pt_dydx(base.slope, dir) == 0.0 {
        return false;
    }

    let mut hits = vec![base];
    for &test in segments {
        ray_check(arena, &base, Some(segment), dir, test, &mut hits);
    }

    // Sort along the ray, so the running winding accumulates in the order the
    // ray actually meets the crossings.
    let xy = dir.xy_index();
    let ascending = dir.less_than();
    hits.sort_by(|a, b| {
        let (av, bv) = if xy == 0 {
            (a.pt.x, b.pt.x)
        } else {
            (a.pt.y, b.pt.y)
        };
        let ord = av.partial_cmp(&bv).unwrap_or(std::cmp::Ordering::Equal);
        if ascending {
            ord
        } else {
            ord.reverse()
        }
    });

    accumulate(arena, &hits, dir)
}

/// Walks the sorted hits, accumulating winding and writing it onto each span.
///
/// The second half of `SkOpSpan::sortableTop`. Returns false as soon as a hit
/// turns out unusable, leaving whatever was written in place: the caller
/// retries with a different ray rather than trusting a partial count.
fn accumulate(arena: &mut OpArena, hits: &[SkOpRayHit], dir: SkOpRayDir) -> bool {
    let mut last: Option<Point> = None;
    let mut wind = 0;
    let mut opp_wind = 0;
    for (index, hit) in hits.iter().enumerate() {
        if !hit.valid {
            return false;
        }
        let Some(span) = hit.span else {
            return false;
        };
        if arena.span(span).wind_value() == 0 && arena.span(span).opp_value() == 0 {
            continue;
        }
        // Two crossings at the same point cannot be ordered, so the count
        // through them is not trustworthy.
        if last.is_some_and(|l| points_approx_equal(l, hit.pt)) {
            return false;
        }
        if let Some(next) = hits.get(index + 1) {
            if points_approx_equal(next.pt, hit.pt) {
                return false;
            }
        }
        let Some(hit_segment) = arena.span_segment(span) else {
            return false;
        };
        let operand = arena.segment_operand(hit_segment);
        if operand {
            std::mem::swap(&mut wind, &mut opp_wind);
        }
        let last_wind = wind;
        let last_opp = opp_wind;
        let ccw = ccw_dxdy(hit.slope, dir);
        let wind_value = if ccw {
            -arena.span(span).wind_value()
        } else {
            arena.span(span).wind_value()
        };
        let opp_value = if ccw {
            -arena.span(span).opp_value()
        } else {
            arena.span(span).opp_value()
        };
        wind += wind_value;
        opp_wind += opp_value;

        let wind_sum = if use_inner_winding(last_wind, wind) {
            wind
        } else {
            last_wind
        };
        let opp_sum = if use_inner_winding(last_opp, opp_wind) {
            opp_wind
        } else {
            last_opp
        };
        let mut sum_set = false;
        if arena.span(span).wind_sum() == PK_MIN_S32 {
            arena.span_set_wind_sum(span, wind_sum);
            sum_set = true;
        }
        if arena.span(span).opp_sum() == PK_MIN_S32 {
            arena.span_set_opp_sum(span, opp_sum);
        }
        if sum_set {
            // Spread the newly known winding along the segments that follow,
            // in both directions from this span.
            if let Some(next) = arena.span_next(span) {
                arena.mark_and_chase_winding_opp(span, next, wind_sum, opp_sum);
                arena.mark_and_chase_winding_opp(next, span, wind_sum, opp_sum);
            }
            arena.bump_nested();
        }
        if operand {
            std::mem::swap(&mut wind, &mut opp_wind);
        }
        last = Some(hit.pt);
    }
    true
}

/// Returns the first span of `segment` whose winding could be resolved.
///
/// Port of `SkOpSegment::findSortableTop`.
pub fn segment_find_sortable_top(
    arena: &mut OpArena,
    segment: SegmentId,
    segments: &[SegmentId],
) -> Option<SpanId> {
    let mut span = arena.segment(segment).f_head?;
    loop {
        let next = arena.span_next(span);
        if !arena.span(span).done() {
            if arena.span(span).wind_sum() != PK_MIN_S32 {
                return Some(span);
            }
            if sortable_top(arena, span, segments) {
                return Some(span);
            }
        }
        match next {
            Some(n) if !arena.span_is_final(n) => span = n,
            _ => return None,
        }
    }
}

/// Returns a span whose winding is resolved, retrying across every segment.
///
/// Port of `FindSortableTop`. The retry loop is bounded: on input it cannot
/// resolve it returns `None` rather than spinning, which is what keeps a
/// pathological path from hanging the whole operation.
pub fn find_sortable_top(arena: &mut OpArena, segments: &[SegmentId]) -> Option<SpanId> {
    for _ in 0..MAX_WINDING_TRIES {
        for &segment in segments {
            if arena.segment_done(segment) {
                continue;
            }
            if let Some(span) = segment_find_sortable_top(arena, segment, segments) {
                return Some(span);
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn t_guess_walks_a_binary_subdivision() {
        // The first guess is the middle; later ones must land elsewhere, so
        // successive tries sample genuinely different places.
        assert_eq!(t_guess(0).0, 0.5);
        let ts: Vec<f64> = (0..8).map(|i| t_guess(i).0).collect();
        for t in &ts {
            assert!(
                (0.0..=1.0).contains(t),
                "every guess stays inside the span: {ts:?}"
            );
        }
        let mut distinct = ts.clone();
        distinct.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        distinct.dedup();
        assert!(
            distinct.len() >= 4,
            "the guesses must spread out, got {ts:?}"
        );
    }

    #[test]
    fn t_guess_alternates_the_ray_direction() {
        assert_eq!(t_guess(0).1, 0);
        assert_eq!(t_guess(1).1, 1);
        assert_eq!(t_guess(2).1, 0);
        assert_eq!(t_guess(3).1, 1);
    }

    #[test]
    fn winding_span_at_t_refuses_a_hit_on_a_boundary() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        arena
            .segment_add_t(seg, 0.5, Point::new(5.0, 0.0))
            .expect("split");
        // Inside the first interval.
        assert!(winding_span_at_t(&arena, seg, 0.25).is_some());
        // Exactly on the split: it belongs to neither span alone.
        assert!(winding_span_at_t(&arena, seg, 0.5).is_none());
    }

    #[test]
    fn winding_span_at_t_picks_the_interval_the_hit_falls_in() {
        let mut arena = OpArena::new();
        let seg = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        arena
            .segment_add_t(seg, 0.5, Point::new(5.0, 0.0))
            .expect("split");
        let head = arena.segment(seg).f_head.expect("head");
        let mid = arena.span_next(head).expect("mid");
        assert_eq!(winding_span_at_t(&arena, seg, 0.25), Some(head));
        assert_eq!(winding_span_at_t(&arena, seg, 0.75), Some(mid));
    }

    #[test]
    fn sideways_overlap_rejects_a_segment_the_ray_misses() {
        let bounds = (0.0, 0.0, 10.0, 10.0);
        // A horizontal ray at y = 5 crosses this box.
        assert!(sideways_overlap(
            bounds,
            Point::new(-50.0, 5.0),
            SkOpRayDir::Right
        ));
        // At y = 50 it passes clear above it.
        assert!(!sideways_overlap(
            bounds,
            Point::new(-50.0, 50.0),
            SkOpRayDir::Right
        ));
    }

    #[test]
    fn ccw_dxdy_gives_opposite_signs_to_opposite_crossings() {
        // The same ray, met by two segments travelling opposite ways.
        let up = [0.0, 1.0];
        let down = [0.0, -1.0];
        assert_ne!(
            ccw_dxdy(up, SkOpRayDir::Right),
            ccw_dxdy(down, SkOpRayDir::Right)
        );
        assert_ne!(
            ccw_dxdy(up, SkOpRayDir::Left),
            ccw_dxdy(down, SkOpRayDir::Left)
        );
    }

    #[test]
    fn rect_side_names_the_side_the_ray_points_at() {
        let b = (1.0, 2.0, 3.0, 4.0);
        assert_eq!(rect_side(b, SkOpRayDir::Left), 1.0);
        assert_eq!(rect_side(b, SkOpRayDir::Top), 2.0);
        assert_eq!(rect_side(b, SkOpRayDir::Right), 3.0);
        assert_eq!(rect_side(b, SkOpRayDir::Bottom), 4.0);
    }

    #[test]
    fn find_sortable_top_resolves_a_winding_on_a_rectangle() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 100.0, 100.0);
        let span = find_sortable_top(&mut arena, &segs).expect("a resolvable span");
        assert_ne!(
            arena.span(span).wind_sum(),
            PK_MIN_S32,
            "the span it returned must have a real winding"
        );
    }

    #[test]
    fn find_sortable_top_gives_up_rather_than_looping() {
        let mut arena = OpArena::new();
        // Every span already walked, so there is nothing left to resolve.
        let segs = rect(&mut arena, 0.0, 0.0, 100.0, 100.0);
        for &seg in &segs {
            arena.segment_mark_all_done(seg);
        }
        assert!(
            find_sortable_top(&mut arena, &segs).is_none(),
            "it must return rather than retry forever"
        );
    }

    #[test]
    fn find_sortable_top_returns_a_span_that_already_knows_its_winding() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 100.0, 100.0);
        let head = arena.segment(segs[0]).f_head.expect("head");
        arena.span_set_wind_sum(head, 7);
        let span = find_sortable_top(&mut arena, &segs).expect("a span");
        assert_eq!(span, head, "a known winding is taken without a ray");
        assert_eq!(arena.span(span).wind_sum(), 7);
    }

    #[test]
    fn a_ray_cast_at_a_rectangle_finds_its_far_side() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 100.0, 100.0);
        // Fire right, from outside the left edge at mid height.
        let base = SkOpRayHit {
            span: None,
            pt: Point::new(-10.0, 50.0),
            t: 0.0,
            slope: [0.0, 1.0],
            valid: true,
        };
        let mut hits = Vec::new();
        for &seg in &segs {
            ray_check(&arena, &base, None, SkOpRayDir::Right, seg, &mut hits);
        }
        assert_eq!(
            hits.len(),
            2,
            "a ray through a rectangle crosses two sides, got {hits:?}"
        );
    }

    #[test]
    fn a_ray_aimed_away_from_a_rectangle_finds_nothing() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 100.0, 100.0);
        // Same point, fired left: the rectangle is entirely behind it.
        let base = SkOpRayHit {
            span: None,
            pt: Point::new(-10.0, 50.0),
            t: 0.0,
            slope: [0.0, 1.0],
            valid: true,
        };
        let mut hits = Vec::new();
        for &seg in &segs {
            ray_check(&arena, &base, None, SkOpRayDir::Left, seg, &mut hits);
        }
        assert!(hits.is_empty(), "nothing lies that way, got {hits:?}");
    }

    #[test]
    fn a_span_with_no_winding_is_not_a_crossing() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 100.0, 100.0);
        // Zero every span's contribution: the ray now meets nothing that
        // counts, even though the geometry is unchanged.
        for &seg in &segs {
            for span in arena.segment_spans(seg) {
                arena.span_mut(span).set_wind_value(0);
                arena.span_mut(span).set_opp_value(0);
            }
        }
        let base = SkOpRayHit {
            span: None,
            pt: Point::new(-10.0, 50.0),
            t: 0.0,
            slope: [0.0, 1.0],
            valid: true,
        };
        let mut hits = Vec::new();
        for &seg in &segs {
            ray_check(&arena, &base, None, SkOpRayDir::Right, seg, &mut hits);
        }
        assert!(hits.is_empty(), "got {hits:?}");
    }

    #[test]
    fn hits_are_sorted_along_the_ray_not_by_discovery() {
        let mut arena = OpArena::new();
        let segs = rect(&mut arena, 0.0, 0.0, 100.0, 100.0);
        let base = SkOpRayHit {
            span: None,
            pt: Point::new(-10.0, 50.0),
            t: 0.0,
            slope: [0.0, 1.0],
            valid: true,
        };
        let mut hits = Vec::new();
        // Walk the segments in reverse, so the right edge is found first.
        for &seg in segs.iter().rev() {
            ray_check(&arena, &base, None, SkOpRayDir::Right, seg, &mut hits);
        }
        hits.sort_by(|a, b| a.pt.x.partial_cmp(&b.pt.x).expect("finite"));
        assert_eq!(hits.len(), 2);
        assert!(
            hits[0].pt.x < hits[1].pt.x,
            "the nearer crossing comes first: {:?}",
            hits.iter().map(|h| h.pt.x).collect::<Vec<_>>()
        );
    }
}
