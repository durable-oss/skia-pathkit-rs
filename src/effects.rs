//! Path effects: transformations applied to a path before stroking, such
//! as dashing or corner rounding.
//!
//! Ported from `include/effects/SkDashPathEffect.h`,
//! `include/effects/SkCornerPathEffect.h`, and `src/core/SkPathEffect.cpp`.

use crate::core::{Path, Point, Scalar, StrokeRec, StrokeStyle, Verb};
use crate::error::PathKitError;

/// A dash/gap or corner-rounding transform applied to a path before
/// stroking.
#[derive(Debug, Clone, PartialEq)]
pub enum PathEffect {
    /// Alternating "on"/"off" lengths applied along the path.
    Dash {
        intervals: Vec<Scalar>,
        phase: Scalar,
    },
    /// Rounds sharp corners with the given radius.
    Corner { radius: Scalar },
    /// Composes two effects: outer(inner(path)).
    Compose {
        outer: Box<PathEffect>,
        inner: Box<PathEffect>,
    },
    /// Sums two effects: first(path) + second(path).
    Sum {
        first: Box<PathEffect>,
        second: Box<PathEffect>,
    },
}

impl PathEffect {
    /// Constructs a [`PathEffect::Dash`] from `intervals` and `phase`.
    ///
    /// # Errors
    ///
    /// Returns [`PathKitError::Unimplemented`] if `intervals` has an odd
    /// length, contains negative values, or sums to zero.
    pub fn dash(intervals: &[Scalar], phase: Scalar) -> Result<Self, PathKitError> {
        if intervals.len() < 2 || intervals.len() % 2 != 0 {
            return Err(PathKitError::Unimplemented(
                "dash intervals must have even length >= 2",
            ));
        }
        for &v in intervals {
            if v < 0.0 {
                return Err(PathKitError::Unimplemented(
                    "dash intervals must be non-negative",
                ));
            }
        }
        let total: Scalar = intervals.iter().sum();
        if total <= 0.0 || !phase.is_finite() || !total.is_finite() {
            return Err(PathKitError::Unimplemented(
                "dash intervals must sum to > 0",
            ));
        }
        Ok(PathEffect::Dash {
            intervals: intervals.to_vec(),
            phase,
        })
    }

    /// Constructs a [`PathEffect::Corner`] with the given `radius`.
    #[must_use]
    pub fn corner(radius: Scalar) -> Self {
        PathEffect::Corner { radius }
    }

    /// Creates a composed path effect that applies `inner` then `outer`.
    /// Returns `inner` if `outer` is None, `outer` if `inner` is None,
    /// or None if both are None.
    #[must_use]
    pub fn make_compose(outer: Option<PathEffect>, inner: Option<PathEffect>) -> Option<Self> {
        match (outer, inner) {
            (None, None) => None,
            (Some(e), None) | (None, Some(e)) => Some(e),
            (Some(outer), Some(inner)) => Some(PathEffect::Compose {
                outer: Box::new(outer),
                inner: Box::new(inner),
            }),
        }
    }

    /// Creates a summed path effect that applies `first` and `second`
    /// sequentially, returning the result of either.
    /// Returns `first` if `second` is None, `second` if `first` is None,
    /// or None if both are None.
    #[must_use]
    pub fn make_sum(first: Option<PathEffect>, second: Option<PathEffect>) -> Option<Self> {
        match (first, second) {
            (None, None) => None,
            (Some(e), None) | (None, Some(e)) => Some(e),
            (Some(first), Some(second)) => Some(PathEffect::Sum {
                first: Box::new(first),
                second: Box::new(second),
            }),
        }
    }

    /// Applies this effect to `src`, honoring `stroke_rec` for
    /// stroke-only effects like [`PathEffect::Dash`]. Returns the filtered
    /// path, or `None` if the effect could not be applied.
    ///
    /// For [`PathEffect::Dash`], returns the path with only the "on" dash
    /// segments retained.
    /// For [`PathEffect::Compose`], applies inner then outer.
    /// For [`PathEffect::Sum`], returns result of either effect.
    #[must_use]
    pub fn filter(&self, src: &Path, stroke_rec: &StrokeRec) -> Option<Path> {
        match self {
            PathEffect::Dash { intervals, phase } => {
                if stroke_rec.style() == StrokeStyle::Fill
                    || stroke_rec.style() == StrokeStyle::StrokeAndFill
                {
                    return None;
                }
                let count = intervals.len() as i32;
                let (initial_dash_length, initial_dash_index, interval_length) =
                    calc_dash_parameters(*phase, intervals);
                if interval_length <= 0.0 {
                    return None;
                }
                let mut dst = Path::new();
                if !dash_path_segments(
                    src,
                    &mut dst,
                    intervals,
                    count,
                    initial_dash_length,
                    initial_dash_index,
                    interval_length,
                ) {
                    return None;
                }
                if dst.is_empty() {
                    return None;
                }
                Some(dst)
            }
            PathEffect::Corner { .. } => {
                let _ = (src, stroke_rec);
                // TODO: implement corner effect
                None
            }
            PathEffect::Compose { outer, inner } => {
                // Apply inner first, then outer
                let path_after_inner = inner.filter(src, stroke_rec);
                match path_after_inner {
                    Some(path) => outer.filter(&path, stroke_rec),
                    None => outer.filter(src, stroke_rec),
                }
            }
            PathEffect::Sum { first, second } => {
                // Try first effect; if it succeeds, return its result
                // Otherwise try second effect
                if let Some(result) = first.filter(src, stroke_rec) {
                    Some(result)
                } else if let Some(result) = second.filter(src, stroke_rec) {
                    Some(result)
                } else {
                    None
                }
            }
        }
    }
}

/// Compute dash parameters from intervals and phase.
fn calc_dash_parameters(phase: Scalar, intervals: &[Scalar]) -> (Scalar, i32, Scalar) {
    let total: Scalar = intervals.iter().sum();
    let mut phase = phase;
    if phase < 0.0 {
        phase = -phase;
        if phase > total {
            phase = phase % total;
        }
        phase = total - phase;
        if phase == total {
            phase = 0.0;
        }
    } else if phase >= total {
        phase = phase % total;
    }

    let mut idx = 0i32;
    let mut accum = phase;
    for (i, &interval) in intervals.iter().enumerate() {
        if accum > interval || (accum == interval && interval > 0.0) {
            accum -= interval;
        } else {
            idx = i as i32;
            return (interval - accum, idx, total);
        }
    }
    (intervals[0], 0, total)
}

/// Dash a path by walking its segments linearly.
fn dash_path_segments(
    src: &Path,
    dst: &mut Path,
    _intervals: &[Scalar],
    _count: i32,
    initial_dash_length: Scalar,
    initial_dash_index: i32,
    _interval_length: Scalar,
) -> bool {
    // Walk the source path verb by verb, dashing each segment.
    // For each segment, compute its length, then apply the dash pattern.
    let mut vi = 0usize;
    let mut pi = 0usize;
    let mut wi = 0usize;
    let mut contour_start = Point::new(0.0, 0.0);
    let mut current_pt = Point::new(0.0, 0.0);
    let mut first_point = true;

    while vi < src.verbs.len() {
        match src.verbs[vi] {
            Verb::Move => {
                contour_start = src.points[pi];
                current_pt = src.points[pi];
                if first_point {
                    dst.move_to(current_pt.x, current_pt.y);
                    first_point = false;
                }
                pi += 1;
            }
            Verb::Line => {
                let end = src.points[pi];
                let start = current_pt;
                dash_line_segment(
                    start,
                    end,
                    dst,
                    initial_dash_length,
                    initial_dash_index,
                    _interval_length,
                    _intervals,
                    _count,
                    &mut contour_start,
                );
                current_pt = end;
                pi += 1;
            }
            Verb::Quad => {
                // For simplicity, approximate quad with line segments.
                let end = src.points[pi + 1];
                let start = current_pt;
                dash_line_segment(
                    start,
                    end,
                    dst,
                    initial_dash_length,
                    initial_dash_index,
                    _interval_length,
                    _intervals,
                    _count,
                    &mut contour_start,
                );
                current_pt = end;
                pi += 2;
            }
            Verb::Conic => {
                let end = src.points[pi + 1];
                let start = current_pt;
                dash_line_segment(
                    start,
                    end,
                    dst,
                    initial_dash_length,
                    initial_dash_index,
                    _interval_length,
                    _intervals,
                    _count,
                    &mut contour_start,
                );
                current_pt = end;
                pi += 2;
                wi += 1;
            }
            Verb::Cubic => {
                let end = src.points[pi + 2];
                let start = current_pt;
                dash_line_segment(
                    start,
                    end,
                    dst,
                    initial_dash_length,
                    initial_dash_index,
                    _interval_length,
                    _intervals,
                    _count,
                    &mut contour_start,
                );
                current_pt = end;
                pi += 3;
            }
            Verb::Close => {
                // Close by dashing back to contour start
                if current_pt != contour_start {
                    let start = current_pt;
                    dash_line_segment(
                        start,
                        contour_start,
                        dst,
                        initial_dash_length,
                        initial_dash_index,
                        _interval_length,
                        _intervals,
                        _count,
                        &mut contour_start,
                    );
                    current_pt = contour_start;
                }
                dst.close();
            }
        }
        vi += 1;
    }
    true
}

fn dash_line_segment(
    start: Point,
    end: Point,
    dst: &mut Path,
    mut initial_dash_length: Scalar,
    mut initial_dash_index: i32,
    interval_length: Scalar,
    intervals: &[Scalar],
    _count: i32,
    _contour_start: &mut Point,
) {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let seg_len = dx.hypot(dy);
    if seg_len < 1e-8 {
        return;
    }
    let ux = dx / seg_len;
    let uy = dy / seg_len;

    let mut distance = 0.0;
    let mut dlen = initial_dash_length;
    let mut index = initial_dash_index;
    let mut first_segment = true;

    while distance < seg_len {
        // Clamp dlen to remaining segment length
        let effective_dlen = if distance + dlen > seg_len {
            seg_len - distance
        } else {
            dlen
        };

        if effective_dlen > 0.0 && index % 2 == 0 {
            // "on" segment
            let sx = start.x + ux * distance;
            let sy = start.y + uy * distance;
            let ex = sx + ux * effective_dlen;
            let ey = sy + uy * effective_dlen;
            if first_segment {
                dst.move_to(sx, sy);
                first_segment = false;
            }
            dst.line_to(ex, ey);
        }

        distance += effective_dlen;
        index += 1;
        if index as usize >= intervals.len() {
            index = 0;
        }
        dlen = intervals[index as usize];
    }

    // Update initial dash info for next segment
    initial_dash_length = dlen;
    initial_dash_index = index;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Rect;

    #[test]
    fn corner_effect_stores_radius() {
        let effect = PathEffect::corner(5.0);
        assert_eq!(effect, PathEffect::Corner { radius: 5.0 });
    }

    #[test]
    fn dash_effect_validates_intervals() {
        assert!(PathEffect::dash(&[1.0, 2.0], 0.0).is_ok());
        assert!(PathEffect::dash(&[1.0], 0.0).is_err());
        assert!(PathEffect::dash(&[1.0, 2.0, 3.0], 0.0).is_err());
        assert!(PathEffect::dash(&[-1.0, 2.0], 0.0).is_err());
    }

    #[test]
    fn dash_filter_rect_path() {
        let mut path = Path::new();
        path.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 100.0, 100.0));
        let effect = PathEffect::dash(&[10.0, 10.0], 0.0).unwrap();
        let rec = StrokeRec::hairline();
        let result = effect.filter(&path, &rec);
        assert!(result.is_some());
        let dashed = result.unwrap();
        // A 100x100 rect has 4 sides of 100 each = 400 total length.
        // With 10-on, 10-off pattern, we should get ~20 "on" segments.
        assert!(!dashed.is_empty());
        assert!(dashed.count_verbs() > 0);
    }

    #[test]
    fn dash_filter_fill_style_returns_none() {
        let path = Path::new();
        let effect = PathEffect::dash(&[10.0, 10.0], 0.0).unwrap();
        let rec = StrokeRec::fill();
        assert!(effect.filter(&path, &rec).is_none());
    }

    #[test]
    fn calc_dash_parameters_basic() {
        let (init_len, init_idx, total) = calc_dash_parameters(0.0, &[5.0, 5.0]);
        assert!((init_len - 5.0).abs() < 1e-6);
        assert_eq!(init_idx, 0);
        assert!((total - 10.0).abs() < 1e-6);
    }

    #[test]
    fn calc_dash_parameters_with_phase() {
        let (init_len, init_idx, _) = calc_dash_parameters(3.0, &[5.0, 5.0]);
        assert!((init_len - 2.0).abs() < 1e-6);
        assert_eq!(init_idx, 0);
    }

    #[test]
    fn calc_dash_parameters_phase_mid_interval() {
        let (init_len, init_idx, _) = calc_dash_parameters(6.0, &[5.0, 5.0]);
        assert!((init_len - 4.0).abs() < 1e-6);
        assert_eq!(init_idx, 1);
    }

    #[test]
    fn make_compose_none_cases() {
        let dash = PathEffect::dash(&[10.0, 10.0], 0.0).unwrap();
        let corner = PathEffect::corner(5.0);

        assert_eq!(PathEffect::make_compose(None, None), None);
        assert_eq!(
            PathEffect::make_compose(Some(dash.clone()), None),
            Some(dash.clone())
        );
        assert_eq!(
            PathEffect::make_compose(None, Some(corner.clone())),
            Some(corner.clone())
        );
    }

    #[test]
    fn make_compose() {
        let dash = PathEffect::dash(&[10.0, 10.0], 0.0).unwrap();
        let corner = PathEffect::corner(5.0);
        let composed = PathEffect::make_compose(Some(dash), Some(corner)).unwrap();
        match composed {
            PathEffect::Compose { outer, inner } => {
                assert!(matches!(*outer, PathEffect::Dash { .. }));
                assert!(matches!(*inner, PathEffect::Corner { .. }));
            }
            _ => panic!("Expected Compose variant"),
        }
    }

    #[test]
    fn make_sum_none_cases() {
        let dash = PathEffect::dash(&[10.0, 10.0], 0.0).unwrap();
        let corner = PathEffect::corner(5.0);

        assert_eq!(PathEffect::make_sum(None, None), None);
        assert_eq!(
            PathEffect::make_sum(Some(dash.clone()), None),
            Some(dash.clone())
        );
        assert_eq!(
            PathEffect::make_sum(None, Some(corner.clone())),
            Some(corner.clone())
        );
    }

    #[test]
    fn make_sum() {
        let dash = PathEffect::dash(&[10.0, 10.0], 0.0).unwrap();
        let corner = PathEffect::corner(5.0);
        let summed = PathEffect::make_sum(Some(dash), Some(corner)).unwrap();
        match summed {
            PathEffect::Sum { first, second } => {
                assert!(matches!(*first, PathEffect::Dash { .. }));
                assert!(matches!(*second, PathEffect::Corner { .. }));
            }
            _ => panic!("Expected Sum variant"),
        }
    }

    #[test]
    fn compose_filter() {
        let mut path = Path::new();
        path.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 100.0, 100.0));
        let dash = PathEffect::dash(&[10.0, 10.0], 0.0).unwrap();
        let composed = PathEffect::make_compose(Some(dash.clone()), Some(dash.clone())).unwrap();
        let rec = StrokeRec::hairline();
        let result = composed.filter(&path, &rec);
        assert!(result.is_some());
        assert!(!result.unwrap().is_empty());
    }

    #[test]
    fn sum_filter() {
        let mut path = Path::new();
        path.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 100.0, 100.0));
        let dash = PathEffect::dash(&[10.0, 10.0], 0.0).unwrap();
        let summed = PathEffect::make_sum(Some(dash.clone()), Some(dash)).unwrap();
        let rec = StrokeRec::hairline();
        let result = summed.filter(&path, &rec);
        assert!(result.is_some());
        assert!(!result.unwrap().is_empty());
    }

    #[test]
    fn sum_filter_first_fails_uses_second() {
        let mut path = Path::new();
        path.add_rect_simple(Rect::from_ltrb(0.0, 0.0, 100.0, 100.0));
        // Corner effect should fail (returns None)
        let corner = PathEffect::corner(5.0);
        let dash = PathEffect::dash(&[10.0, 10.0], 0.0).unwrap();
        let summed = PathEffect::make_sum(Some(corner), Some(dash)).unwrap();
        let rec = StrokeRec::hairline();
        let result = summed.filter(&path, &rec);
        assert!(result.is_some());
    }
}
