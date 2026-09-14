use super::path::Path;
use super::point::Point;
use super::scalar::{self, Scalar};
use super::types::Verb;

const MAX_T_VALUE: i32 = 0x3FFFFFFF;
const MAX_T_VALUE_F: Scalar = MAX_T_VALUE as Scalar;

fn t_value_to_scalar(t: i32) -> Scalar {
    t as Scalar * (1.0 / MAX_T_VALUE_F)
}

fn tspan_big_enough(tspan: i32) -> bool {
    (tspan >> 10) != 0
}

const CHEAP_DIST_LIMIT: Scalar = 0.5;

fn quad_too_curvy(pts: &[Point; 3], tolerance: Scalar) -> bool {
    let dx = pts[1].x * 0.5 - (pts[0].x + pts[2].x) * 0.25;
    let dy = pts[1].y * 0.5 - (pts[0].y + pts[2].y) * 0.25;
    let dist = dx.abs().max(dy.abs());
    dist > tolerance
}

fn conic_too_curvy(first_pt: &Point, mid_t_pt: &Point, last_pt: &Point, tolerance: Scalar) -> bool {
    let mid_ends = Point::new(
        (first_pt.x + last_pt.x) * 0.5,
        (first_pt.y + last_pt.y) * 0.5,
    );
    let dxy = *mid_t_pt - mid_ends;
    let dist = dxy.x.abs().max(dxy.y.abs());
    dist > tolerance
}

fn cheap_dist_exceeds_limit(pt: &Point, x: Scalar, y: Scalar, tolerance: Scalar) -> bool {
    let dist = (x - pt.x).abs().max((y - pt.y).abs());
    dist > tolerance
}

fn cubic_too_curvy(pts: &[Point; 4], tolerance: Scalar) -> bool {
    cheap_dist_exceeds_limit(
        &pts[1],
        scalar::interp(pts[0].x, pts[3].x, 1.0 / 3.0),
        scalar::interp(pts[0].y, pts[3].y, 1.0 / 3.0),
        tolerance,
    ) || cheap_dist_exceeds_limit(
        &pts[2],
        scalar::interp(pts[0].x, pts[3].x, 2.0 / 3.0),
        scalar::interp(pts[0].y, pts[3].y, 2.0 / 3.0),
        tolerance,
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SegType {
    Line = 0,
    Quad = 1,
    Cubic = 2,
    Conic = 3,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Segment {
    pub distance: Scalar,
    pub pt_index: u32,
    pub t_value: u32,
    pub seg_type: SegType,
}

impl Segment {
    pub fn get_scalar_t(&self) -> Scalar {
        t_value_to_scalar(self.t_value as i32)
    }

    pub fn next<'a>(segments: &'a [Segment], seg: &Segment) -> &'a Segment {
        let pt_index = seg.pt_index;
        segments
            .iter()
            .skip_while(|s| s.pt_index == pt_index)
            .next()
            .unwrap_or_else(|| &segments[segments.len() - 1])
    }
}

/// Measures length, position, and tangent along a single contour of a path.
///
/// Obtained from [`ContourMeasureIter`], which walks a [`Path`] contour by
/// contour.
#[derive(Debug, Clone)]
pub struct ContourMeasure {
    segments: Vec<Segment>,
    pts: Vec<Point>,
    length: Scalar,
    is_closed: bool,
}

impl ContourMeasure {
    /// Total length of this contour.
    pub fn length(&self) -> Scalar {
        self.length
    }

    /// Whether this contour is closed (either explicitly via
    /// [`Path::close`] or via `force_closed` on the iterator).
    pub fn is_closed(&self) -> bool {
        self.is_closed
    }

    /// Computes the position and/or tangent at `distance` along the
    /// contour. `distance` is clamped to `[0, length()]`.
    ///
    /// Returns `false` if `distance` is NaN.
    pub fn get_pos_tan(
        &self,
        distance: Scalar,
        pos: Option<&mut Point>,
        tangent: Option<&mut Point>,
    ) -> bool {
        if distance.is_nan() {
            return false;
        }

        let length = self.length;
        let distance = distance.clamp(0.0, length);

        let mut t: Scalar = 0.0;
        let seg = self.distance_to_segment(distance, &mut t);
        if t.is_nan() {
            return false;
        }

        compute_pos_tan(
            &self.pts[seg.pt_index as usize..],
            seg.seg_type,
            t,
            pos,
            tangent,
        );
        true
    }

    /// Computes a transformation matrix at `distance` along the contour,
    /// based on `flags`.
    ///
    /// Returns `false` if `distance` is invalid.
    pub fn get_matrix(
        &self,
        distance: Scalar,
        matrix: &mut super::matrix::Matrix,
        flags: MatrixFlags,
    ) -> bool {
        let mut position = Point::default();
        let mut tangent = Point::default();

        if self.get_pos_tan(distance, Some(&mut position), Some(&mut tangent)) {
            if flags.contains(MatrixFlags::GET_TANGENT) {
                matrix.set_sin_cos_pivot(tangent.y, tangent.x, 0.0, 0.0);
            } else {
                matrix.reset();
            }
            if flags.contains(MatrixFlags::GET_POSITION) {
                matrix.post_translate(position.x, position.y);
            }
            true
        } else {
            false
        }
    }

    /// Extracts the sub-path between `start_d` and `stop_d` into `dst`.
    ///
    /// Returns `false` if the range is invalid or empty.
    pub fn get_segment(
        &self,
        start_d: Scalar,
        stop_d: Scalar,
        dst: &mut Path,
        start_with_move_to: bool,
    ) -> bool {
        let length = self.length;
        let start_d = start_d.max(0.0);
        let stop_d = stop_d.min(length);

        if !(start_d <= stop_d) {
            return false;
        }
        if self.segments.is_empty() {
            return false;
        }

        let mut start_t: Scalar = 0.0;
        let seg = self.distance_to_segment(start_d, &mut start_t);
        if !start_t.is_finite() {
            return false;
        }

        let mut stop_t: Scalar = 0.0;
        let stop_seg = self.distance_to_segment(stop_d, &mut stop_t);
        if !stop_t.is_finite() {
            return false;
        }

        if start_with_move_to {
            let mut p = Point::default();
            compute_pos_tan(
                &self.pts[seg.pt_index as usize..],
                seg.seg_type,
                start_t,
                Some(&mut p),
                None,
            );
            dst.move_to(p.x, p.y);
        }

        if seg.pt_index == stop_seg.pt_index {
            contour_measure_seg_to(
                &self.pts[seg.pt_index as usize..],
                seg.seg_type,
                start_t,
                stop_t,
                dst,
            );
        } else {
            let mut current_seg = seg;
            loop {
                contour_measure_seg_to(
                    &self.pts[current_seg.pt_index as usize..],
                    current_seg.seg_type,
                    start_t,
                    1.0,
                    dst,
                );
                let next_seg = Segment::next(&self.segments, current_seg);
                if next_seg.pt_index >= stop_seg.pt_index {
                    break;
                }
                current_seg = next_seg;
                start_t = 0.0;
            }
            contour_measure_seg_to(
                &self.pts[current_seg.pt_index as usize..],
                current_seg.seg_type,
                0.0,
                stop_t,
                dst,
            );
        }

        true
    }

    fn distance_to_segment(&self, distance: Scalar, t: &mut Scalar) -> &Segment {
        let segs = &self.segments;
        let mut lo = 0usize;
        let mut hi = segs.len() - 1;

        while lo < hi {
            let mid = (hi + lo) >> 1;
            if segs[mid].distance < distance {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }

        let mut index = hi;
        if segs[index].distance < distance {
            index = index.wrapping_add(1);
            index = !index;
        } else if distance < segs[index].distance {
            index = !index;
        }

        let index = if (index as isize) < 0 {
            (!index) as usize
        } else {
            index
        };

        let seg = &segs[index];

        let mut start_t = 0.0;
        let mut start_d = 0.0;
        if index > 0 {
            start_d = segs[index - 1].distance;
            if segs[index - 1].pt_index == seg.pt_index {
                start_t = segs[index - 1].get_scalar_t();
            }
        }

        *t = start_t
            + (seg.get_scalar_t() - start_t) * (distance - start_d) / (seg.distance - start_d);
        seg
    }
}

/// Flags controlling which components [`ContourMeasure::get_matrix`]
/// computes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MatrixFlags(u8);

impl MatrixFlags {
    /// Compute the position component.
    pub const GET_POSITION: Self = MatrixFlags(0x01);
    /// Compute the tangent (rotation) component.
    pub const GET_TANGENT: Self = MatrixFlags(0x02);
    /// Compute both position and tangent.
    pub const GET_POS_AND_TAN: Self = MatrixFlags(0x03);

    /// Returns `true` if `self` includes all bits set in `other`.
    pub fn contains(&self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }
}

/// Iterates over the contours of a [`Path`], yielding a
/// [`ContourMeasure`] for each one.
#[derive(Debug)]
pub struct ContourMeasureIter {
    path: Path,
    impl_: Option<ContourMeasureIterImpl>,
}

impl ContourMeasureIter {
    /// Creates an empty iterator with no path set.
    pub fn new() -> Self {
        ContourMeasureIter {
            path: Path::new(),
            impl_: None,
        }
    }

    /// Creates an iterator over `path`'s contours.
    ///
    /// If `force_closed` is set, every contour is treated as closed even
    /// without an explicit [`Path::close`]. `res_scale` controls the
    /// tessellation tolerance for curved segments.
    pub fn from_path(path: &Path, force_closed: bool, res_scale: Scalar) -> Self {
        let mut iter = ContourMeasureIter {
            path: path.clone(),
            impl_: None,
        };
        iter.reset(path, force_closed, res_scale);
        iter
    }

    /// Resets the iterator to walk a new path from the start.
    pub fn reset(&mut self, path: &Path, force_closed: bool, res_scale: Scalar) {
        self.path = path.clone();
        if path.is_finite() {
            self.impl_ = Some(ContourMeasureIterImpl::new(
                &self.path,
                force_closed,
                res_scale,
            ));
        } else {
            self.impl_ = None;
        }
    }

    /// Returns the next contour, or `None` once all contours have been
    /// consumed.
    pub fn next(&mut self) -> Option<ContourMeasure> {
        let impl_ = self.impl_.as_mut()?;
        loop {
            if !impl_.has_next_segments() {
                return None;
            }
            if let Some(cm) = impl_.build_segments() {
                return Some(cm);
            }
        }
    }
}

impl Default for ContourMeasureIter {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
struct ContourMeasureIterImpl {
    path: Path,
    verb_index: usize,
    point_index: usize,
    weight_index: usize,
    tolerance: Scalar,
    force_closed: bool,
    segments: Vec<Segment>,
    pts: Vec<Point>,
}

impl ContourMeasureIterImpl {
    fn new(path: &Path, force_closed: bool, res_scale: Scalar) -> Self {
        ContourMeasureIterImpl {
            path: path.clone(),
            verb_index: 0,
            point_index: 0,
            weight_index: 0,
            tolerance: CHEAP_DIST_LIMIT * res_scale.recip(),
            force_closed,
            segments: Vec::new(),
            pts: Vec::new(),
        }
    }

    fn has_next_segments(&self) -> bool {
        self.verb_index < self.path.verbs().len()
    }

    fn build_segments(&mut self) -> Option<ContourMeasure> {
        let mut pt_index: i32 = -1;
        let mut distance: Scalar = 0.0;
        let mut have_seen_close = self.force_closed;
        let mut have_seen_move_to = false;

        self.segments.clear();
        self.pts.clear();

        let verbs = self.path.verbs();
        let points = self.path.points();
        let weights = self.path.conic_weights();

        let mut vi = self.verb_index;
        let mut pi = self.point_index;
        let mut wi = self.weight_index;

        loop {
            if vi >= verbs.len() {
                break;
            }
            let verb = verbs[vi];
            if have_seen_move_to && verb == Verb::Move {
                break;
            }

            match verb {
                Verb::Move => {
                    pt_index += 1;
                    self.pts.push(points[pi]);
                    pi += 1;
                    have_seen_move_to = true;
                }
                Verb::Line => {
                    let prev_d = distance;
                    distance = compute_line_seg(
                        points[pi - 1],
                        points[pi],
                        distance,
                        pt_index as u32,
                        &mut self.segments,
                    );
                    if distance > prev_d {
                        self.pts.push(points[pi]);
                        pt_index += 1;
                    }
                    pi += 1;
                }
                Verb::Quad => {
                    let prev_d = distance;
                    let quad_pts = [points[pi - 1], points[pi], points[pi + 1]];
                    distance = compute_quad_segs(
                        &quad_pts,
                        distance,
                        0,
                        MAX_T_VALUE,
                        pt_index as u32,
                        &mut self.segments,
                        self.tolerance,
                    );
                    if distance > prev_d {
                        self.pts.push(points[pi]);
                        self.pts.push(points[pi + 1]);
                        pt_index += 2;
                    }
                    pi += 2;
                }
                Verb::Conic => {
                    let prev_d = distance;
                    let conic = Conic::new(points[pi - 1], points[pi], points[pi + 1], weights[wi]);
                    distance = compute_conic_segs(
                        &conic,
                        distance,
                        0,
                        &conic.f_pts[0],
                        MAX_T_VALUE,
                        &conic.f_pts[2],
                        pt_index as u32,
                        &mut self.segments,
                        self.tolerance,
                    );
                    if distance > prev_d {
                        self.pts.push(Point::new(conic.f_w, 0.0));
                        self.pts.push(points[pi]);
                        self.pts.push(points[pi + 1]);
                        pt_index += 3;
                    }
                    pi += 2;
                    wi += 1;
                }
                Verb::Cubic => {
                    let prev_d = distance;
                    let cubic_pts = [points[pi - 1], points[pi], points[pi + 1], points[pi + 2]];
                    distance = compute_cubic_segs(
                        &cubic_pts,
                        distance,
                        0,
                        MAX_T_VALUE,
                        pt_index as u32,
                        &mut self.segments,
                        self.tolerance,
                    );
                    if distance > prev_d {
                        self.pts.push(points[pi]);
                        self.pts.push(points[pi + 1]);
                        self.pts.push(points[pi + 2]);
                        pt_index += 3;
                    }
                    pi += 3;
                }
                Verb::Close => {
                    have_seen_close = true;
                }
            }

            vi += 1;
        }

        self.verb_index = vi;
        self.point_index = pi;
        self.weight_index = wi;

        if !distance.is_finite() {
            return None;
        }
        if self.segments.is_empty() {
            return None;
        }

        if have_seen_close {
            let prev_d = distance;
            let first_pt = self.pts[0];
            distance = compute_line_seg(
                self.pts[pt_index as usize],
                first_pt,
                distance,
                pt_index as u32,
                &mut self.segments,
            );
            if distance > prev_d {
                self.pts.push(first_pt);
            }
        }

        Some(ContourMeasure {
            segments: std::mem::take(&mut self.segments),
            pts: std::mem::take(&mut self.pts),
            length: distance,
            is_closed: have_seen_close,
        })
    }
}

fn compute_line_seg(
    p0: Point,
    p1: Point,
    distance: Scalar,
    pt_index: u32,
    segments: &mut Vec<Segment>,
) -> Scalar {
    let d = Point::distance(p0, p1);
    let prev_d = distance;
    let distance = distance + d;
    if distance > prev_d {
        segments.push(Segment {
            distance,
            pt_index,
            t_value: MAX_T_VALUE as u32,
            seg_type: SegType::Line,
        });
    }
    distance
}

fn compute_quad_segs(
    pts: &[Point; 3],
    distance: Scalar,
    mint: i32,
    maxt: i32,
    pt_index: u32,
    segments: &mut Vec<Segment>,
    tolerance: Scalar,
) -> Scalar {
    if tspan_big_enough(maxt - mint) && quad_too_curvy(pts, tolerance) {
        let halft = (mint + maxt) >> 1;
        let mut tmp = [Point::default(); 5];
        chop_quad_at_half(pts, &mut tmp);
        let distance = compute_quad_segs(
            &[tmp[0], tmp[1], tmp[2]],
            distance,
            mint,
            halft,
            pt_index,
            segments,
            tolerance,
        );
        compute_quad_segs(
            &[tmp[2], tmp[3], tmp[4]],
            distance,
            halft,
            maxt,
            pt_index,
            segments,
            tolerance,
        )
    } else {
        let d = Point::distance(pts[0], pts[2]);
        let prev_d = distance;
        let distance = distance + d;
        if distance > prev_d {
            segments.push(Segment {
                distance,
                pt_index,
                t_value: maxt as u32,
                seg_type: SegType::Quad,
            });
        }
        distance
    }
}

fn compute_conic_segs(
    conic: &Conic,
    distance: Scalar,
    mint: i32,
    min_pt: &Point,
    maxt: i32,
    max_pt: &Point,
    pt_index: u32,
    segments: &mut Vec<Segment>,
    tolerance: Scalar,
) -> Scalar {
    let halft = (mint + maxt) >> 1;
    let half_pt = conic.eval_at_point(t_value_to_scalar(halft));
    if !half_pt.is_finite() {
        return distance;
    }
    if tspan_big_enough(maxt - mint) && conic_too_curvy(min_pt, &half_pt, max_pt, tolerance) {
        let distance = compute_conic_segs(
            conic, distance, mint, min_pt, halft, &half_pt, pt_index, segments, tolerance,
        );
        compute_conic_segs(
            conic, distance, halft, &half_pt, maxt, max_pt, pt_index, segments, tolerance,
        )
    } else {
        let d = Point::distance(*min_pt, *max_pt);
        let prev_d = distance;
        let distance = distance + d;
        if distance > prev_d {
            segments.push(Segment {
                distance,
                pt_index,
                t_value: maxt as u32,
                seg_type: SegType::Conic,
            });
        }
        distance
    }
}

fn compute_cubic_segs(
    pts: &[Point; 4],
    distance: Scalar,
    mint: i32,
    maxt: i32,
    pt_index: u32,
    segments: &mut Vec<Segment>,
    tolerance: Scalar,
) -> Scalar {
    if tspan_big_enough(maxt - mint) && cubic_too_curvy(pts, tolerance) {
        let halft = (mint + maxt) >> 1;
        let mut tmp = [Point::default(); 7];
        chop_cubic_at_half(pts, &mut tmp);
        let distance = compute_cubic_segs(
            &[tmp[0], tmp[1], tmp[2], tmp[3]],
            distance,
            mint,
            halft,
            pt_index,
            segments,
            tolerance,
        );
        compute_cubic_segs(
            &[tmp[3], tmp[4], tmp[5], tmp[6]],
            distance,
            halft,
            maxt,
            pt_index,
            segments,
            tolerance,
        )
    } else {
        let d = Point::distance(pts[0], pts[3]);
        let prev_d = distance;
        let distance = distance + d;
        if distance > prev_d {
            segments.push(Segment {
                distance,
                pt_index,
                t_value: maxt as u32,
                seg_type: SegType::Cubic,
            });
        }
        distance
    }
}

fn contour_measure_seg_to(
    pts: &[Point],
    seg_type: SegType,
    start_t: Scalar,
    stop_t: Scalar,
    dst: &mut Path,
) {
    if start_t == stop_t {
        if !dst.is_empty() {
            if let Some(last_pt) = dst.last_point() {
                dst.line_to(last_pt.x, last_pt.y);
            }
        }
        return;
    }

    match seg_type {
        SegType::Line => {
            if stop_t == 1.0 {
                dst.line_to(pts[1].x, pts[1].y);
            } else {
                dst.line_to(
                    scalar::interp(pts[0].x, pts[1].x, stop_t),
                    scalar::interp(pts[0].y, pts[1].y, stop_t),
                );
            }
        }
        SegType::Quad => {
            if start_t == 0.0 {
                if stop_t == 1.0 {
                    dst.quad_to(pts[1].x, pts[1].y, pts[2].x, pts[2].y);
                } else {
                    let mut tmp = [Point::default(); 5];
                    chop_quad_at(pts, &mut tmp, stop_t);
                    dst.quad_to(tmp[1].x, tmp[1].y, tmp[2].x, tmp[2].y);
                }
            } else {
                let mut tmp = [Point::default(); 5];
                chop_quad_at(pts, &mut tmp, start_t);
                if stop_t == 1.0 {
                    dst.quad_to(tmp[3].x, tmp[3].y, tmp[4].x, tmp[4].y);
                } else {
                    let mut tmp2 = [Point::default(); 5];
                    let t = (stop_t - start_t) / (1.0 - start_t);
                    chop_quad_at(&[tmp[2], tmp[3], tmp[4]], &mut tmp2, t);
                    dst.quad_to(tmp2[1].x, tmp2[1].y, tmp2[2].x, tmp2[2].y);
                }
            }
        }
        SegType::Conic => {
            let conic = Conic::new(pts[0], pts[2], pts[3], pts[1].x);
            if start_t == 0.0 {
                if stop_t == 1.0 {
                    dst.conic_to(
                        conic.f_pts[1].x,
                        conic.f_pts[1].y,
                        conic.f_pts[2].x,
                        conic.f_pts[2].y,
                        conic.f_w,
                    );
                } else {
                    let mut tmp = [Conic::default(); 2];
                    if conic.chop_at(stop_t, &mut tmp) {
                        dst.conic_to(
                            tmp[0].f_pts[1].x,
                            tmp[0].f_pts[1].y,
                            tmp[0].f_pts[2].x,
                            tmp[0].f_pts[2].y,
                            tmp[0].f_w,
                        );
                    }
                }
            } else {
                if stop_t == 1.0 {
                    let mut tmp = [Conic::default(); 2];
                    if conic.chop_at(start_t, &mut tmp) {
                        dst.conic_to(
                            tmp[1].f_pts[1].x,
                            tmp[1].f_pts[1].y,
                            tmp[1].f_pts[2].x,
                            tmp[1].f_pts[2].y,
                            tmp[1].f_w,
                        );
                    }
                } else {
                    let mut tmp = Conic::default();
                    conic.chop_at_range(start_t, stop_t, &mut tmp);
                    dst.conic_to(
                        tmp.f_pts[1].x,
                        tmp.f_pts[1].y,
                        tmp.f_pts[2].x,
                        tmp.f_pts[2].y,
                        tmp.f_w,
                    );
                }
            }
        }
        SegType::Cubic => {
            if start_t == 0.0 {
                if stop_t == 1.0 {
                    dst.cubic_to(pts[1].x, pts[1].y, pts[2].x, pts[2].y, pts[3].x, pts[3].y);
                } else {
                    let mut tmp = [Point::default(); 7];
                    chop_cubic_at(pts, &mut tmp, stop_t);
                    dst.cubic_to(tmp[1].x, tmp[1].y, tmp[2].x, tmp[2].y, tmp[3].x, tmp[3].y);
                }
            } else {
                let mut tmp = [Point::default(); 7];
                chop_cubic_at(pts, &mut tmp, start_t);
                if stop_t == 1.0 {
                    dst.cubic_to(tmp[4].x, tmp[4].y, tmp[5].x, tmp[5].y, tmp[6].x, tmp[6].y);
                } else {
                    let mut tmp2 = [Point::default(); 7];
                    let t = (stop_t - start_t) / (1.0 - start_t);
                    chop_cubic_at(&[tmp[3], tmp[4], tmp[5], tmp[6]], &mut tmp2, t);
                    dst.cubic_to(
                        tmp2[1].x, tmp2[1].y, tmp2[2].x, tmp2[2].y, tmp2[3].x, tmp2[3].y,
                    );
                }
            }
        }
    }
}

fn compute_pos_tan(
    pts: &[Point],
    seg_type: SegType,
    t: Scalar,
    pos: Option<&mut Point>,
    tangent: Option<&mut Point>,
) {
    match seg_type {
        SegType::Line => {
            if let Some(pos) = pos {
                pos.x = scalar::interp(pts[0].x, pts[1].x, t);
                pos.y = scalar::interp(pts[0].y, pts[1].y, t);
            }
            if let Some(tangent) = tangent {
                let dx = pts[1].x - pts[0].x;
                let dy = pts[1].y - pts[0].y;
                let mag = Point::distance_to_origin(dx, dy);
                if mag > 0.0 {
                    tangent.x = dx / mag;
                    tangent.y = dy / mag;
                }
            }
        }
        SegType::Quad => {
            compute_quad_pos_tan(pts, t, pos, tangent);
        }
        SegType::Conic => {
            compute_conic_pos_tan(pts, t, pos, tangent);
        }
        SegType::Cubic => {
            compute_cubic_pos_tan(pts, t, pos, tangent);
        }
    }
}

fn compute_quad_pos_tan(
    pts: &[Point],
    t: Scalar,
    pos: Option<&mut Point>,
    tangent: Option<&mut Point>,
) {
    let mt = 1.0 - t;
    if let Some(pos) = pos {
        pos.x = mt * mt * pts[0].x + 2.0 * mt * t * pts[1].x + t * t * pts[2].x;
        pos.y = mt * mt * pts[0].y + 2.0 * mt * t * pts[1].y + t * t * pts[2].y;
    }
    if let Some(tangent) = tangent {
        if t == 0.0 && pts[0] == pts[1] {
            tangent.x = pts[2].x - pts[0].x;
            tangent.y = pts[2].y - pts[0].y;
        } else if t == 1.0 && pts[1] == pts[2] {
            tangent.x = pts[2].x - pts[0].x;
            tangent.y = pts[2].y - pts[0].y;
        } else {
            let b = Point::new(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
            let a = Point::new(pts[2].x - pts[1].x - b.x, pts[2].y - pts[1].y - b.y);
            tangent.x = 2.0 * (a.x * t + b.x);
            tangent.y = 2.0 * (a.y * t + b.y);
        }
        let mag = Point::distance_to_origin(tangent.x, tangent.y);
        if mag > 0.0 {
            tangent.x /= mag;
            tangent.y /= mag;
        }
    }
}

fn compute_conic_pos_tan(
    pts: &[Point],
    t: Scalar,
    pos: Option<&mut Point>,
    tangent: Option<&mut Point>,
) {
    let conic = Conic::new(pts[0], pts[2], pts[3], pts[1].x);
    if let Some(pos) = pos {
        let p = conic.eval_at_point(t);
        *pos = p;
    }
    if let Some(tangent) = tangent {
        if t == 0.0 && conic.f_pts[0] == conic.f_pts[1] {
            tangent.x = conic.f_pts[2].x - conic.f_pts[0].x;
            tangent.y = conic.f_pts[2].y - conic.f_pts[0].y;
        } else if t == 1.0 && conic.f_pts[1] == conic.f_pts[2] {
            tangent.x = conic.f_pts[2].x - conic.f_pts[0].x;
            tangent.y = conic.f_pts[2].y - conic.f_pts[0].y;
        } else {
            let p20 = Point::new(
                conic.f_pts[2].x - conic.f_pts[0].x,
                conic.f_pts[2].y - conic.f_pts[0].y,
            );
            let p10 = Point::new(
                conic.f_pts[1].x - conic.f_pts[0].x,
                conic.f_pts[1].y - conic.f_pts[0].y,
            );
            let c = p10.scale(conic.f_w);
            let a = p20.scale(conic.f_w) - p20;
            let b = p20 - c - c;
            tangent.x = (a.x * t + b.x) * t + c.x;
            tangent.y = (a.y * t + b.y) * t + c.y;
        }
        let mag = Point::distance_to_origin(tangent.x, tangent.y);
        if mag > 0.0 {
            tangent.x /= mag;
            tangent.y /= mag;
        }
    }
}

fn compute_cubic_pos_tan(
    pts: &[Point],
    t: Scalar,
    pos: Option<&mut Point>,
    tangent: Option<&mut Point>,
) {
    let mt = 1.0 - t;
    if let Some(pos) = pos {
        let a = mt * mt * mt;
        let b = 3.0 * mt * mt * t;
        let c = 3.0 * mt * t * t;
        let d = t * t * t;
        pos.x = a * pts[0].x + b * pts[1].x + c * pts[2].x + d * pts[3].x;
        pos.y = a * pts[0].y + b * pts[1].y + c * pts[2].y + d * pts[3].y;
    }
    if let Some(tangent) = tangent {
        if t == 0.0 && pts[0] == pts[1] {
            tangent.x = pts[2].x - pts[0].x;
            tangent.y = pts[2].y - pts[0].y;
        } else if t == 1.0 && pts[2] == pts[3] {
            tangent.x = pts[3].x - pts[1].x;
            tangent.y = pts[3].y - pts[1].y;
        } else {
            let a_pt = (pts[3] - pts[0]) + (pts[1] - pts[2]).scale(3.0);
            let b_pt = (pts[2] - pts[1].scale(2.0) + pts[0]).scale(2.0);
            let c_pt = pts[1] - pts[0];
            tangent.x = (a_pt.x * t + b_pt.x) * t + c_pt.x;
            tangent.y = (a_pt.y * t + b_pt.y) * t + c_pt.y;
        }
        let mag = Point::distance_to_origin(tangent.x, tangent.y);
        if mag > 0.0 {
            tangent.x /= mag;
            tangent.y /= mag;
        }
    }
}

// ---- Geometry helpers ----

#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct Conic {
    pub f_pts: [Point; 3],
    pub f_w: Scalar,
}

impl Conic {
    pub fn new(p0: Point, p1: Point, p2: Point, w: Scalar) -> Self {
        Conic {
            f_pts: [p0, p1, p2],
            f_w: w,
        }
    }

    pub fn eval_at(&self, t: Scalar, pos: Option<&mut Point>, tangent: Option<&mut Point>) {
        let mt = 1.0 - t;
        let tt = t;
        let w = self.f_w;

        let x = mt * mt * self.f_pts[0].x
            + 2.0 * mt * tt * w * self.f_pts[1].x
            + tt * tt * self.f_pts[2].x;
        let y = mt * mt * self.f_pts[0].y
            + 2.0 * mt * tt * w * self.f_pts[1].y
            + tt * tt * self.f_pts[2].y;
        let z = mt * mt + 2.0 * mt * tt * w + tt * tt;

        if let Some(pos) = pos {
            pos.x = x / z;
            pos.y = y / z;
        }
        if let Some(tangent) = tangent {
            if t == 0.0 && self.f_pts[0] == self.f_pts[1] {
                let dx = self.f_pts[2].x - self.f_pts[0].x;
                let dy = self.f_pts[2].y - self.f_pts[0].y;
                tangent.x = dx;
                tangent.y = dy;
            } else if t == 1.0 && self.f_pts[1] == self.f_pts[2] {
                let dx = self.f_pts[2].x - self.f_pts[0].x;
                let dy = self.f_pts[2].y - self.f_pts[0].y;
                tangent.x = dx;
                tangent.y = dy;
            } else {
                let p20 = Point::new(
                    self.f_pts[2].x - self.f_pts[0].x,
                    self.f_pts[2].y - self.f_pts[0].y,
                );
                let p10 = Point::new(
                    self.f_pts[1].x - self.f_pts[0].x,
                    self.f_pts[1].y - self.f_pts[0].y,
                );
                let c = p10.scale(w);
                let a = p20.scale(w) - p20;
                let b = p20 - c - c;
                tangent.x = (a.x * t + b.x) * t + c.x;
                tangent.y = (a.y * t + b.y) * t + c.y;
            }
        }
    }

    pub fn eval_at_point(&self, t: Scalar) -> Point {
        let mut pos = Point::default();
        self.eval_at(t, Some(&mut pos), None);
        pos
    }

    pub fn chop_at(&self, t: Scalar, dst: &mut [Conic; 2]) -> bool {
        if t <= 0.0 || t >= 1.0 {
            return false;
        }

        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        let p2 = self.f_pts[2];
        let w = self.f_w;

        let tmp0 = [p0.x, p0.y, 1.0];
        let tmp1 = [p1.x * w, p1.y * w, w];
        let tmp2 = [p2.x, p2.y, 1.0];

        let ab_x = scalar::interp(tmp0[0], tmp1[0], t);
        let ab_y = scalar::interp(tmp0[1], tmp1[1], t);
        let ab_z = scalar::interp(tmp0[2], tmp1[2], t);

        let bc_x = scalar::interp(tmp1[0], tmp2[0], t);
        let bc_y = scalar::interp(tmp1[1], tmp2[1], t);
        let bc_z = scalar::interp(tmp1[2], tmp2[2], t);

        let abc_x = scalar::interp(ab_x, bc_x, t);
        let abc_y = scalar::interp(ab_y, bc_y, t);
        let abc_z = scalar::interp(ab_z, bc_z, t);

        let root = abc_z.sqrt();

        dst[0] = Conic {
            f_pts: [
                p0,
                project_down(ab_x, ab_y, ab_z),
                project_down(abc_x, abc_y, abc_z),
            ],
            f_w: ab_z / root,
        };
        dst[1] = Conic {
            f_pts: [
                project_down(abc_x, abc_y, abc_z),
                project_down(bc_x, bc_y, bc_z),
                p2,
            ],
            f_w: bc_z / root,
        };

        true
    }

    pub fn chop_at_range(&self, t1: Scalar, t2: Scalar, dst: &mut Conic) {
        if t1 == 0.0 && t2 == 1.0 {
            *dst = *self;
            return;
        }
        if t1 == 0.0 || t2 == 1.0 {
            let mut pair = [Conic::default(); 2];
            let t = if t1 != 0.0 { t1 } else { t2 };
            if self.chop_at(t, &mut pair) {
                *dst = if t1 != 0.0 { pair[1] } else { pair[0] };
                return;
            }
        }

        let p0 = self.f_pts[0];
        let p1 = self.f_pts[1];
        let p2 = self.f_pts[2];
        let w = self.f_w;

        let a = conic_eval_numer_denom(p0, p1, p2, w, t1);
        let mid = conic_eval_numer_denom(p0, p1, p2, w, (t1 + t2) * 0.5);
        let c = conic_eval_numer_denom(p0, p1, p2, w, t2);

        let b_x = 2.0 * mid.x - (a.x + c.x) * 0.5;
        let b_y = 2.0 * mid.y - (a.y + c.y) * 0.5;
        let b_z = 2.0 * mid.z - (a.z + c.z) * 0.5;

        dst.f_pts[0] = Point::new(a.x / a.z, a.y / a.z);
        dst.f_pts[1] = Point::new(b_x / b_z, b_y / b_z);
        dst.f_pts[2] = Point::new(c.x / c.z, c.y / c.z);
        dst.f_w = b_z / (a.z * c.z).sqrt();
    }
}

fn project_down(x: Scalar, y: Scalar, z: Scalar) -> Point {
    Point::new(x / z, y / z)
}

fn conic_eval_numer_denom(p0: Point, p1: Point, p2: Point, w: Scalar, t: Scalar) -> NumerDenom {
    conic_eval_numer_denom_v(p0, p1, p2, w, t)
}

struct NumerDenom {
    x: Scalar,
    y: Scalar,
    z: Scalar,
}

fn conic_eval_numer_denom_v(p0: Point, p1: Point, p2: Point, w: Scalar, t: Scalar) -> NumerDenom {
    let mt = 1.0 - t;
    let x = mt * mt * p0.x + 2.0 * mt * t * w * p1.x + t * t * p2.x;
    let y = mt * mt * p0.y + 2.0 * mt * t * w * p1.y + t * t * p2.y;
    let z = mt * mt + 2.0 * mt * t * w + t * t;
    NumerDenom { x, y, z }
}

fn chop_quad_at(src: &[Point], dst: &mut [Point; 5], t: Scalar) {
    let p0 = src[0];
    let p1 = src[1];
    let p2 = src[2];

    let p01 = Point::new(scalar::interp(p0.x, p1.x, t), scalar::interp(p0.y, p1.y, t));
    let p12 = Point::new(scalar::interp(p1.x, p2.x, t), scalar::interp(p1.y, p2.y, t));
    let p012 = Point::new(
        scalar::interp(p01.x, p12.x, t),
        scalar::interp(p01.y, p12.y, t),
    );

    dst[0] = p0;
    dst[1] = p01;
    dst[2] = p012;
    dst[3] = p12;
    dst[4] = p2;
}

fn chop_quad_at_half(src: &[Point; 3], dst: &mut [Point; 5]) {
    chop_quad_at(src, dst, 0.5);
}

fn chop_cubic_at(src: &[Point], dst: &mut [Point; 7], t: Scalar) {
    let p0 = src[0];
    let p1 = src[1];
    let p2 = src[2];
    let p3 = src[3];

    let ab = Point::new(scalar::interp(p0.x, p1.x, t), scalar::interp(p0.y, p1.y, t));
    let bc = Point::new(scalar::interp(p1.x, p2.x, t), scalar::interp(p1.y, p2.y, t));
    let cd = Point::new(scalar::interp(p2.x, p3.x, t), scalar::interp(p2.y, p3.y, t));
    let abc = Point::new(scalar::interp(ab.x, bc.x, t), scalar::interp(ab.y, bc.y, t));
    let bcd = Point::new(scalar::interp(bc.x, cd.x, t), scalar::interp(bc.y, cd.y, t));
    let abcd = Point::new(
        scalar::interp(abc.x, bcd.x, t),
        scalar::interp(abc.y, bcd.y, t),
    );

    dst[0] = p0;
    dst[1] = ab;
    dst[2] = abc;
    dst[3] = abcd;
    dst[4] = bcd;
    dst[5] = cd;
    dst[6] = p3;
}

fn chop_cubic_at_half(src: &[Point; 4], dst: &mut [Point; 7]) {
    chop_cubic_at(src, dst, 0.5);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_path() -> Path {
        let mut p = Path::new();
        p.move_to(0.0, 0.0);
        p.line_to(10.0, 0.0);
        p.line_to(10.0, 10.0);
        p.line_to(0.0, 10.0);
        p.close();
        p
    }

    #[test]
    fn test_contour_measure_length() {
        let path = make_path();
        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");
        assert!((cm.length() - 40.0).abs() < 1e-4);
        assert!(cm.is_closed());
    }

    #[test]
    fn test_contour_measure_two_contours() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.move_to(0.0, 10.0);
        path.line_to(10.0, 10.0);

        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm1 = iter.next().expect("first contour");
        assert!((cm1.length() - 10.0).abs() < 1e-4);
        let cm2 = iter.next().expect("second contour");
        assert!((cm2.length() - 10.0).abs() < 1e-4);
        assert!(iter.next().is_none());
    }

    #[test]
    fn test_contour_measure_get_pos_tan() {
        let path = make_path();
        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");

        let mut pos = Point::default();
        let mut tan = Point::default();
        let result = cm.get_pos_tan(0.0, Some(&mut pos), Some(&mut tan));
        assert!(result);
        assert!((pos.x - 0.0).abs() < 1e-4);
        assert!((pos.y - 0.0).abs() < 1e-4);
        assert!((tan.x - 1.0).abs() < 1e-4);
        assert!((tan.y - 0.0).abs() < 1e-4);

        let result = cm.get_pos_tan(10.0, Some(&mut pos), Some(&mut tan));
        assert!(result);
        assert!((pos.x - 10.0).abs() < 1e-4);
        assert!((pos.y - 0.0).abs() < 1e-4);
        assert!((tan.x - 1.0).abs() < 1e-4);
        assert!((tan.y - 0.0).abs() < 1e-4);

        let result = cm.get_pos_tan(15.0, Some(&mut pos), Some(&mut tan));
        assert!(result);
        assert!((pos.x - 10.0).abs() < 1e-4);
        assert!((pos.y - 5.0).abs() < 1e-4);
    }

    #[test]
    fn test_contour_measure_get_segment() {
        let path = make_path();
        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");

        let mut dst = Path::new();
        let result = cm.get_segment(0.0, 10.0, &mut dst, true);
        assert!(result);
        assert_eq!(dst.verbs().len(), 2);
        assert_eq!(dst.verbs()[0], Verb::Move);
        assert_eq!(dst.verbs()[1], Verb::Line);
    }

    #[test]
    fn test_contour_measure_get_segment_out_of_range() {
        let path = make_path();
        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");

        let mut dst = Path::new();
        let result = cm.get_segment(5.0, 3.0, &mut dst, true);
        assert!(!result);
    }

    #[test]
    fn test_contour_measure_nan_distance() {
        let path = make_path();
        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");

        let result = cm.get_pos_tan(Scalar::NAN, None, None);
        assert!(!result);
    }

    #[test]
    fn test_contour_measure_empty_path() {
        let path = Path::new();
        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        assert!(iter.next().is_none());
    }

    #[test]
    fn test_contour_measure_quad_path() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(50.0, 100.0, 100.0, 0.0);

        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");
        assert!(cm.length() > 100.0);

        let mut pos = Point::default();
        let mut tan = Point::default();
        let result = cm.get_pos_tan(cm.length() * 0.5, Some(&mut pos), Some(&mut tan));
        assert!(result);
        assert!(pos.x > 0.0 && pos.x < 100.0);
        assert!(pos.y > 0.0);
    }

    #[test]
    fn test_contour_measure_cubic_path() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.cubic_to(50.0, 100.0, 150.0, -100.0, 200.0, 0.0);

        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");
        assert!(cm.length() > 200.0);
    }

    #[test]
    fn test_contour_measure_force_closed() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);

        let mut iter = ContourMeasureIter::from_path(&path, true, 1.0);
        let cm = iter.next().expect("should have a contour");
        assert!(cm.is_closed());
        let diag = (200.0_f32).sqrt();
        assert!((cm.length() - (20.0 + diag)).abs() < 1e-2);
    }

    #[test]
    fn test_contour_measure_not_closed() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);

        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");
        assert!(!cm.is_closed());
        assert!((cm.length() - 20.0).abs() < 1e-4);
    }

    #[test]
    fn test_segment_next() {
        let segments = vec![
            Segment {
                distance: 10.0,
                pt_index: 0,
                t_value: MAX_T_VALUE as u32,
                seg_type: SegType::Line,
            },
            Segment {
                distance: 20.0,
                pt_index: 0,
                t_value: MAX_T_VALUE as u32,
                seg_type: SegType::Line,
            },
            Segment {
                distance: 30.0,
                pt_index: 1,
                t_value: MAX_T_VALUE as u32,
                seg_type: SegType::Line,
            },
        ];
        let next = Segment::next(&segments, &segments[0]);
        assert_eq!(next.pt_index, 1);
    }

    #[test]
    fn test_conic_eval() {
        let conic = Conic::new(
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
            0.5,
        );
        let pt = conic.eval_at_point(0.0);
        assert!((pt.x - 0.0).abs() < 1e-4);
        assert!((pt.y - 0.0).abs() < 1e-4);

        let pt = conic.eval_at_point(1.0);
        assert!((pt.x - 100.0).abs() < 1e-4);
        assert!((pt.y - 0.0).abs() < 1e-4);
    }

    #[test]
    fn test_conic_chop() {
        let conic = Conic::new(
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
            0.5,
        );
        let mut dst = [Conic::default(); 2];
        let result = conic.chop_at(0.5, &mut dst);
        assert!(result);

        let orig_mid = conic.eval_at_point(0.5);
        let left_end = dst[0].eval_at_point(1.0);
        let right_start = dst[1].eval_at_point(0.0);
        assert!((left_end.x - orig_mid.x).abs() < 1e-4);
        assert!((left_end.y - orig_mid.y).abs() < 1e-4);
        assert!((right_start.x - orig_mid.x).abs() < 1e-4);
        assert!((right_start.y - orig_mid.y).abs() < 1e-4);

        let left_start = dst[0].eval_at_point(0.0);
        let orig_start = conic.eval_at_point(0.0);
        let right_end = dst[1].eval_at_point(1.0);
        let orig_end = conic.eval_at_point(1.0);
        assert!((left_start.x - orig_start.x).abs() < 1e-4);
        assert!((left_start.y - orig_start.y).abs() < 1e-4);
        assert!((right_end.x - orig_end.x).abs() < 1e-4);
        assert!((right_end.y - orig_end.y).abs() < 1e-4);
    }

    #[test]
    fn test_conic_chop_arbitrary_t() {
        let conic = Conic::new(
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
            0.5,
        );
        for t in [0.3, 0.7, 0.9] {
            let mut dst = [Conic::default(); 2];
            assert!(conic.chop_at(t, &mut dst));
            let orig_mid = conic.eval_at_point(t);
            let left_end = dst[0].eval_at_point(1.0);
            let right_start = dst[1].eval_at_point(0.0);
            assert!((left_end.x - orig_mid.x).abs() < 1e-3);
            assert!((left_end.y - orig_mid.y).abs() < 1e-3);
            assert!((right_start.x - orig_mid.x).abs() < 1e-3);
            assert!((right_start.y - orig_mid.y).abs() < 1e-3);
        }
    }

    #[test]
    fn test_chop_quad_at_half_exactness() {
        let quad = [
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
        ];
        let mut dst = [Point::default(); 5];
        chop_quad_at(&quad, &mut dst, 0.5);
        assert!((dst[0].x - quad[0].x).abs() < 1e-5);
        assert!((dst[2].x - quad[0].x * 0.25 - quad[1].x * 0.5 - quad[2].x * 0.25).abs() < 1e-5);
        assert!((dst[4].x - quad[2].x).abs() < 1e-5);
    }

    #[test]
    fn test_chop_cubic_at_half_exactness() {
        let cubic = [
            Point::new(0.0, 0.0),
            Point::new(30.0, 60.0),
            Point::new(70.0, -60.0),
            Point::new(100.0, 0.0),
        ];
        let mut dst = [Point::default(); 7];
        chop_cubic_at(&cubic, &mut dst, 0.5);
        assert!((dst[3].x - dst[3].x).abs() < 1e-5);
        assert!((dst[0].x - cubic[0].x).abs() < 1e-5);
        assert!((dst[6].x - cubic[3].x).abs() < 1e-5);
        let eval = |pts: &[Point], t: Scalar| {
            let mt = 1.0 - t;
            let a = mt * mt * mt;
            let b = 3.0 * mt * mt * t;
            let c = 3.0 * mt * t * t;
            let d = t * t * t;
            Point::new(
                a * pts[0].x + b * pts[1].x + c * pts[2].x + d * pts[3].x,
                a * pts[0].y + b * pts[1].y + c * pts[2].y + d * pts[3].y,
            )
        };
        let mid = eval(&cubic, 0.5);
        assert!((dst[3].x - mid.x).abs() < 1e-5);
        assert!((dst[3].y - mid.y).abs() < 1e-5);
    }

    #[test]
    fn test_t_value_conversion() {
        assert_eq!(t_value_to_scalar(0), 0.0);
        assert!((t_value_to_scalar(MAX_T_VALUE) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_curviness_helpers() {
        assert!(quad_too_curvy(
            &[
                Point::new(0.0, 0.0),
                Point::new(50.0, 100.0),
                Point::new(100.0, 0.0),
            ],
            0.1
        ));
        assert!(!quad_too_curvy(
            &[
                Point::new(0.0, 0.0),
                Point::new(50.0, 0.0),
                Point::new(100.0, 0.0),
            ],
            0.1
        ));
        assert!(cubic_too_curvy(
            &[
                Point::new(0.0, 0.0),
                Point::new(30.0, 60.0),
                Point::new(70.0, -60.0),
                Point::new(100.0, 0.0),
            ],
            0.1
        ));
        assert!(!cubic_too_curvy(
            &[
                Point::new(0.0, 0.0),
                Point::new(100.0 / 3.0, 0.0),
                Point::new(200.0 / 3.0, 0.0),
                Point::new(100.0, 0.0),
            ],
            0.1
        ));
    }

    #[test]
    fn test_conic_curviness() {
        let first = Point::new(0.0, 0.0);
        let mid = Point::new(50.0, 50.0);
        let last = Point::new(100.0, 0.0);
        assert!(conic_too_curvy(&first, &mid, &last, 0.1));
        let straight_mid = Point::new(50.0, 0.0);
        assert!(!conic_too_curvy(&first, &straight_mid, &last, 0.1));
    }

    #[test]
    fn test_contour_measure_reset() {
        let path = make_path();
        let mut iter = ContourMeasureIter::new();
        iter.reset(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");
        assert!((cm.length() - 40.0).abs() < 1e-4);
    }

    #[test]
    fn test_contour_measure_bezier_arc_length() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(0.0, 100.0, 100.0, 100.0);

        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        let cm = iter.next().expect("should have a contour");
        assert!(cm.length() > 100.0);
        assert!(cm.length() < 200.0);
    }

    #[test]
    fn test_contour_measure_high_precision() {
        let path = make_path();
        let mut iter = ContourMeasureIter::from_path(&path, false, 4.0);
        let cm = iter.next().expect("should have a contour");
        assert!((cm.length() - 40.0).abs() < 1e-4);
    }
}
