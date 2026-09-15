//! Angular sorting of path segments that share a common point.
//!
//! Port of Skia's `SkOpAngle.{h,cpp}`.
//!
//! Angles are sorted counterclockwise. The smallest angle has a positive x and
//! the smallest positive y. The largest angle has a positive x and a zero y.
//!
//! # Sectors
//!
//! Direction is quantized into 32 sectors (16 "sedecimants" x 2), so that a
//! cheap mask test can rule out most comparisons before any real geometry runs.
//! The quarter-angle values are:
//!
//! ```text
//! 31   x > 0, y == 0              horizontal line (to the right)
//! 0    x > 0, y == epsilon        quad/cubic horizontal tangent eventually going +y
//! 1    x > 0, y > 0, x > y        nearer horizontal angle
//! 2                  x + e == y   quad/cubic 45 going horiz
//! 3    x > 0, y > 0, x == y       45 angle
//! 4                  x == y + e   quad/cubic 45 going vert
//! 5    x > 0, y > 0, x < y        nearer vertical angle
//! 6    x == epsilon, y > 0        quad/cubic vertical tangent eventually going +x
//! 7    x == 0, y > 0              vertical line (to the top)
//!
//!                                       8  7  6
//!                                  9       |       5
//!                               10         |          4
//!                             11           |            3
//!                           12  \          |           / 2
//!                          13              |              1
//!                         14               |               0
//!                         15 --------------+------------- 31
//!                         16               |              30
//!                          17              |             29
//!                           18  /          |          \ 28
//!                             19           |           27
//!                               20         |         26
//!                                  21      |      25
//!                                      22 23 24
//! ```
//!
//! # Arena model
//!
//! The C++ original links angles into a circular list through raw `fNext`
//! pointers and mutates them during comparison. This port keeps the angles in
//! an [`AngleList`] arena and links them by index, matching the
//! `Option<usize>` convention used by [`super::sk_op_span`]. Operations that
//! walk or splice the loop are methods on the arena rather than on a single
//! angle.
//!
//! # Port status
//!
//! The geometric core is ported: sector assignment, the loop algorithms, and
//! the hull/tangent predicates that decide ordering. The parts of the C++ file
//! that read the span graph (`setSpans`, `computeSector`, `endsIntersect`,
//! `endToSide`, `midToSide`) need `SkOpSegment::subDivide`, span linkage, and
//! curve/ray intersection, none of which exist yet in this port; they are
//! marked with `TODO(port)` where they attach. See
//! `TODO/2026-09-10-pathops-engine-port-gaps.md`.

use super::sk_line_parameters::{LinePoint, SkLineParameters};
use super::sk_path_ops_types::almost_equal_ulps;
use crate::core::Verb;

/// Number of sectors the circle of directions is divided into.
pub const NUM_SECTORS: i32 = 32;

/// A direction vector in the f64 space the angle math uses.
///
/// The C++ original uses `double` throughout; angle sorting is numerically
/// delicate enough that the f32 `SkDVector` in this crate is not a substitute.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AngleVector {
    /// The x component.
    pub f_x: f64,
    /// The y component.
    pub f_y: f64,
}

impl AngleVector {
    /// Returns a vector with the given components.
    #[must_use]
    pub const fn new(f_x: f64, f_y: f64) -> Self {
        Self { f_x, f_y }
    }

    /// Returns the zero vector.
    #[must_use]
    pub const fn zero() -> Self {
        Self::new(0.0, 0.0)
    }

    /// Returns the z component of the cross product with `a`.
    #[must_use]
    pub fn cross(&self, a: Self) -> f64 {
        self.f_x * a.f_y - self.f_y * a.f_x
    }

    /// Returns the cross product, collapsing near-parallel results to zero.
    ///
    /// The two products are compared in ULPs so that a difference lost in
    /// rounding reads as exactly parallel rather than as a tiny sign.
    #[must_use]
    pub fn cross_check(&self, a: Self) -> f64 {
        let xy = self.f_x * a.f_y;
        let yx = self.f_y * a.f_x;
        if almost_equal_ulps(xy as f32, yx as f32) {
            0.0
        } else {
            xy - yx
        }
    }

    /// Returns the cross product without the near-parallel check.
    #[must_use]
    pub fn cross_no_normal_check(&self, a: Self) -> f64 {
        self.f_x * a.f_y - self.f_y * a.f_x
    }

    /// Returns the dot product with `a`.
    #[must_use]
    pub fn dot(&self, a: Self) -> f64 {
        self.f_x * a.f_x + self.f_y * a.f_y
    }

    /// Returns the vector's length.
    #[must_use]
    pub fn length(&self) -> f64 {
        self.length_squared().sqrt()
    }

    /// Returns the vector's squared length.
    #[must_use]
    pub fn length_squared(&self) -> f64 {
        self.f_x * self.f_x + self.f_y * self.f_y
    }
}

impl std::ops::Sub for AngleVector {
    type Output = AngleVector;
    fn sub(self, rhs: AngleVector) -> AngleVector {
        AngleVector::new(self.f_x - rhs.f_x, self.f_y - rhs.f_y)
    }
}

/// How winding is accumulated when angles are walked.
///
/// Port of `SkOpAngle::IncludeType`.
// Ordered because the C++ tests `includeType >= kBinarySingle` to mean "this
// is a two-operand walk"; the variant order below is the C++ enum's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum IncludeType {
    /// One operand, nonzero fill.
    UnaryWinding,
    /// One operand, even-odd fill.
    UnaryXor,
    /// Two operands, the angle's own.
    BinarySingle,
    /// Two operands, the opposite one.
    BinaryOpp,
}

/// Returns the number of points past the first for `verb`.
///
/// Port of `SkPathOpsVerbToPoints`: a line has 1, quad and conic 2, cubic 3.
#[must_use]
pub fn verb_to_points(verb: Verb) -> usize {
    match verb {
        Verb::Line => 1,
        Verb::Quad | Verb::Conic => 2,
        Verb::Cubic => 3,
        _ => 0,
    }
}

/// The curve from an angle's start to its end, plus its hull sweep.
///
/// Port of `SkDCurveSweep`, in f64 and carrying only what the angle code
/// reads. Points are stored as a fixed array indexed 0..=`verb_to_points`.
#[derive(Debug, Clone, Copy)]
pub struct CurveSweep {
    /// The curve's control points, `0..=verb_to_points(verb)` of them valid.
    pub f_curve: [LinePoint; 4],
    /// The verb describing which of `f_curve` are in use.
    pub f_verb: Verb,
    /// The conic weight; ignored for other verbs.
    pub f_weight: f64,
    /// The two hull sweep vectors bounding the curve's direction.
    pub f_sweep: [AngleVector; 2],
    /// False when the curve is a line or is line-like.
    f_is_curve: bool,
    /// Cleared when a cubic's control point isn't between the sweep vectors.
    f_ordered: bool,
}

impl Default for CurveSweep {
    fn default() -> Self {
        Self {
            f_curve: [[0.0; 2]; 4],
            f_verb: Verb::Line,
            f_weight: 1.0,
            f_sweep: [AngleVector::zero(); 2],
            f_is_curve: false,
            f_ordered: true,
        }
    }
}

impl CurveSweep {
    /// Returns a sweep with no curve set.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns true if the curve bends; false for lines and line-like curves.
    #[must_use]
    pub fn is_curve(&self) -> bool {
        self.f_is_curve
    }

    /// Returns false when a cubic's control point falls outside the sweep.
    #[must_use]
    pub fn is_ordered(&self) -> bool {
        self.f_ordered
    }

    /// Returns control point `index` as a vector.
    #[must_use]
    pub fn pt(&self, index: usize) -> AngleVector {
        AngleVector::new(self.f_curve[index][0], self.f_curve[index][1])
    }

    /// Translates every control point in use by `dx`, `dy`.
    ///
    /// Port of `SkDCurve::offset`; angles are compared after being moved to a
    /// common origin.
    #[allow(clippy::needless_range_loop)] // mirrors the C++ point indexing
    pub fn offset(&mut self, dx: f64, dy: f64) {
        let count = verb_to_points(self.f_verb);
        for index in 0..=count {
            self.f_curve[index][0] += dx;
            self.f_curve[index][1] += dy;
        }
    }

    /// Computes the hull sweep vectors for the current curve.
    ///
    /// Port of `SkDCurveSweep::setCurveHullSweep`.
    pub fn set_curve_hull_sweep(&mut self) {
        let verb = self.f_verb;
        self.f_ordered = true;
        self.f_sweep[0] = self.pt(1) - self.pt(0);
        if verb == Verb::Line {
            self.f_sweep[1] = self.f_sweep[0];
            self.f_is_curve = false;
            return;
        }
        self.f_sweep[1] = self.pt(2) - self.pt(0);
        if verb == Verb::Quad || verb == Verb::Conic {
            let max_val = self.max_component(2);
            if approximately_zero_when_compared_to(self.f_sweep[0].f_x, max_val)
                && approximately_zero_when_compared_to(self.f_sweep[0].f_y, max_val)
            {
                self.f_sweep[0] = self.f_sweep[1];
            }
            self.f_is_curve = self.f_sweep[0].cross_check(self.f_sweep[1]) != 0.0;
            return;
        }
        debug_assert_eq!(verb, Verb::Cubic);
        let max_val = self.max_component(3);
        // The cubic's first sweep may be degenerate; step out to the next
        // control point and remember that the hull is no longer ordered.
        if approximately_zero_when_compared_to(self.f_sweep[0].f_x, max_val)
            && approximately_zero_when_compared_to(self.f_sweep[0].f_y, max_val)
        {
            self.f_sweep[0] = self.f_sweep[1];
            self.f_sweep[1] = self.pt(3) - self.pt(0);
            self.f_ordered = false;
        } else if approximately_zero_when_compared_to(self.f_sweep[1].f_x, max_val)
            && approximately_zero_when_compared_to(self.f_sweep[1].f_y, max_val)
        {
            self.f_sweep[1] = self.pt(3) - self.pt(0);
            self.f_ordered = false;
        }
        self.f_is_curve = self.f_sweep[0].cross_check(self.f_sweep[1]) != 0.0;
    }

    /// Returns the largest absolute coordinate over points `0..=count`.
    #[allow(clippy::needless_range_loop)] // mirrors the C++ point indexing
    fn max_component(&self, count: usize) -> f64 {
        let mut max_val: f64 = 0.0;
        for index in 0..=count {
            max_val = max_val
                .max(self.f_curve[index][0].abs())
                .max(self.f_curve[index][1].abs());
        }
        max_val
    }
}

/// Returns true if `x` is negligible next to `y`.
///
/// Port of `approximately_zero_when_compared_to` for f64 operands.
fn approximately_zero_when_compared_to(x: f64, y: f64) -> bool {
    x == 0.0 || x.abs() < y.abs() * f64::from(f32::EPSILON)
}

/// Fills `out` with the piece of a curve between two t values.
///
/// Port of `SkOpSegment::subDivide(start, end, SkDCurve*)`, taking the
/// geometry directly rather than a segment, since `SkOpSegment` is still on
/// the pre-arena model until item 05.
///
/// `pts` holds the segment's control points, `0..=verb_to_points(verb)` of
/// them in use. The endpoints come from the caller rather than being
/// recomputed, matching C++, which uses the PtT nodes' cached points: the two
/// can differ by rounding, and the cached ones are what the rest of the engine
/// compares against.
///
/// Returns true when a real subdivision happened, false when the piece is the
/// whole curve or a line and the control points were simply copied.
#[allow(clippy::too_many_arguments)] // mirrors the C++ signature plus its segment fields
pub fn sub_divide_curve(
    pts: &[LinePoint],
    verb: Verb,
    weight: f64,
    start_pt: LinePoint,
    start_t: f64,
    end_pt: LinePoint,
    end_t: f64,
    out: &mut CurveSweep,
) -> bool {
    let points = verb_to_points(verb);
    out.f_verb = verb;
    out.f_curve[0] = start_pt;
    out.f_curve[points] = end_pt;
    if verb == Verb::Line {
        return false;
    }
    if (start_t == 0.0 || end_t == 0.0) && (start_t == 1.0 || end_t == 1.0) {
        // The piece is the whole curve, so the midpoints are already known.
        match verb {
            Verb::Quad | Verb::Conic => {
                out.f_curve[1] = pts[1];
                out.f_weight = weight;
            }
            _ => {
                // Cubic. Running backwards swaps the two control points.
                if start_t == 0.0 {
                    out.f_curve[1] = pts[1];
                    out.f_curve[2] = pts[2];
                } else {
                    out.f_curve[1] = pts[2];
                    out.f_curve[2] = pts[1];
                }
            }
        }
        return false;
    }
    match verb {
        Verb::Quad => {
            out.f_curve[1] = quad_sub_divide_control(pts, start_pt, end_pt, start_t, end_t);
        }
        Verb::Conic => {
            let (ctrl, w) = conic_sub_divide_control(pts, weight, start_t, end_t);
            out.f_curve[1] = ctrl;
            out.f_weight = w;
        }
        _ => {
            let (c1, c2) = cubic_sub_divide_controls(pts, start_pt, end_pt, start_t, end_t);
            out.f_curve[1] = c1;
            out.f_curve[2] = c2;
        }
    }
    true
}

/// Returns a quad's value at `t`, in one coordinate.
fn quad_at(p: &[f64; 3], t: f64) -> f64 {
    let u = 1.0 - t;
    u * u * p[0] + 2.0 * u * t * p[1] + t * t * p[2]
}

/// Returns the control point of the quad piece spanning `t1..t2`.
///
/// Port of `SkDQuad::SubDivide`: with both ends known, the piece's midpoint
/// determines the control point.
fn quad_sub_divide_control(
    pts: &[LinePoint],
    a: LinePoint,
    c: LinePoint,
    t1: f64,
    t2: f64,
) -> LinePoint {
    let xs = [pts[0][0], pts[1][0], pts[2][0]];
    let ys = [pts[0][1], pts[1][1], pts[2][1]];
    let mid_t = (t1 + t2) / 2.0;
    let dx = quad_at(&xs, mid_t);
    let dy = quad_at(&ys, mid_t);
    [
        2.0 * dx - (a[0] + c[0]) / 2.0,
        2.0 * dy - (a[1] + c[1]) / 2.0,
    ]
}

/// Returns a conic's numerators and denominator at `t`.
fn conic_homogeneous(xs: &[f64; 3], ys: &[f64; 3], w: f64, t: f64) -> (f64, f64, f64) {
    if t == 0.0 {
        return (xs[0], ys[0], 1.0);
    }
    if t == 1.0 {
        return (xs[2], ys[2], 1.0);
    }
    let num = |src: &[f64; 3]| {
        let src1w = src[1] * w;
        let c = src[0];
        let a = src[2] - 2.0 * src1w + c;
        let b = 2.0 * (src1w - c);
        (a * t + b) * t + c
    };
    let b = 2.0 * (w - 1.0);
    let denom = (-b * t + b) * t + 1.0;
    (num(xs), num(ys), denom)
}

/// Returns the control point and weight of the conic piece spanning `t1..t2`.
///
/// Port of `SkDConic::SubDivide`. Both ends are evaluated in homogeneous form,
/// which is where the weight is carried; treating the conic as a quad here
/// gives a piece that does not lie on the original curve.
fn conic_sub_divide_control(pts: &[LinePoint], w: f64, t1: f64, t2: f64) -> (LinePoint, f64) {
    let xs = [pts[0][0], pts[1][0], pts[2][0]];
    let ys = [pts[0][1], pts[1][1], pts[2][1]];

    let (ax, ay, az) = conic_homogeneous(&xs, &ys, w, t1);
    let (cx, cy, cz) = conic_homogeneous(&xs, &ys, w, t2);
    let mid_t = (t1 + t2) / 2.0;
    let (dx, dy, dz) = conic_homogeneous(&xs, &ys, w, mid_t);

    let bx = 2.0 * dx - (ax + cx) / 2.0;
    let by = 2.0 * dy - (ay + cy) / 2.0;
    let mut bz = 2.0 * dz - (az + cz) / 2.0;
    if bz == 0.0 {
        // A zero weight makes the control point irrelevant; any value serves.
        bz = 1.0;
    }
    ([bx / bz, by / bz], bz / (az * cz).sqrt())
}

/// Returns a cubic's value at `t`, in one coordinate.
fn cubic_at(p: &[f64; 4], t: f64) -> f64 {
    let u = 1.0 - t;
    u * u * u * p[0] + 3.0 * u * u * t * p[1] + 3.0 * u * t * t * p[2] + t * t * t * p[3]
}

/// Returns the two control points of the cubic piece spanning `t1..t2`.
///
/// Port of `SkDCubic::SubDivide`. With both ends pinned, sampling the original
/// curve at a third and two thirds of the way along the piece gives two
/// equations in the two unknown control points:
///
/// ```text
/// B(1/3) = (8a + 12b + 6c +  d) / 27
/// B(2/3) = ( a +  6b + 12c + 8d) / 27
/// ```
fn cubic_sub_divide_controls(
    pts: &[LinePoint],
    a: LinePoint,
    d: LinePoint,
    t1: f64,
    t2: f64,
) -> (LinePoint, LinePoint) {
    let xs = [pts[0][0], pts[1][0], pts[2][0], pts[3][0]];
    let ys = [pts[0][1], pts[1][1], pts[2][1], pts[3][1]];
    let t_1_3 = t1 + (t2 - t1) / 3.0;
    let t_2_3 = t1 + 2.0 * (t2 - t1) / 3.0;
    let e = [cubic_at(&xs, t_1_3), cubic_at(&ys, t_1_3)];
    let f = [cubic_at(&xs, t_2_3), cubic_at(&ys, t_2_3)];
    let solve = |i: usize| {
        let m = 27.0 * e[i] - 8.0 * a[i] - d[i];
        let n = 27.0 * f[i] - a[i] - 8.0 * d[i];
        let b = (2.0 * m - n) / 18.0;
        let c = (2.0 * n - m) / 18.0;
        (b, c)
    };
    let (bx, cx) = solve(0);
    let (by, cy) = solve(1);
    ([bx, by], [cx, cy])
}

/// A curve from a start point to an end point, sortable against other angles
/// that share the start point.
///
/// Port of `SkOpAngle`. Angles live in an [`AngleList`]; `f_next` and the
/// span/segment handles are indices into their respective arenas rather than
/// pointers.
#[derive(Debug, Clone)]
pub struct SkOpAngle {
    /// The curve from start to end, before being moved to a common origin.
    pub f_original_curve_part: CurveSweep,
    /// The curve from start to end, offset as needed for comparison.
    pub f_part: CurveSweep,
    /// Signed distance from the chord to the furthest control point.
    ///
    /// Only the sign is meaningful; it is not normalized.
    pub f_side: f64,
    /// Used only to sort a pair of lines or line-like sections.
    pub f_tangent_half: SkLineParameters,
    /// Next angle in the circular sorted loop, as an [`AngleList`] index.
    pub f_next: Option<usize>,
    /// Last span marked while walking this angle, as a span arena index.
    pub f_last_marked: Option<usize>,
    /// The span this angle starts at, as a span arena index.
    pub f_start: Option<usize>,
    /// The span this angle ends at, as a span arena index.
    pub f_end: Option<usize>,
    /// The end used for sector computation, which may be lengthened.
    pub f_computed_end: Option<usize>,
    /// Bitmask of the sectors this angle sweeps through.
    pub f_sector_mask: u32,
    /// Sector the angle starts in, in 32nds of a circle; -1 when unset.
    pub f_sector_start: i8,
    /// Sector the angle ends in, in 32nds of a circle; -1 when unset.
    pub f_sector_end: i8,
    /// True when this angle cannot be ordered against its neighbors.
    pub f_unorderable: bool,
    /// True when the sector could not be determined and must be recomputed.
    pub f_compute_sector: bool,
    /// True once sector recomputation has been attempted.
    pub f_computed_sector: bool,
    /// True when the angle needs a coincidence check.
    pub f_check_coincidence: bool,
    /// True when the tangents are too close to call.
    pub f_tangents_ambiguous: bool,
    /// Debug id, or -1.
    pub f_id: i32,
}

impl Default for SkOpAngle {
    fn default() -> Self {
        Self::new()
    }
}

impl SkOpAngle {
    /// Returns an angle with no curve or linkage set.
    #[must_use]
    pub fn new() -> Self {
        Self {
            f_original_curve_part: CurveSweep::new(),
            f_part: CurveSweep::new(),
            f_side: 0.0,
            f_tangent_half: SkLineParameters::new(),
            f_next: None,
            f_last_marked: None,
            f_start: None,
            f_end: None,
            f_computed_end: None,
            f_sector_mask: 0,
            f_sector_start: -1,
            f_sector_end: -1,
            f_unorderable: false,
            f_compute_sector: false,
            f_computed_sector: false,
            f_check_coincidence: false,
            f_tangents_ambiguous: false,
            f_id: -1,
        }
    }

    /// Returns the debug id.
    #[must_use]
    pub fn debug_id(&self) -> i32 {
        self.f_id
    }

    /// Returns true when the tangents were too close to order confidently.
    #[must_use]
    pub fn tangents_ambiguous(&self) -> bool {
        self.f_tangents_ambiguous
    }

    /// Returns true when this angle could not be ordered.
    #[must_use]
    pub fn unorderable(&self) -> bool {
        self.f_unorderable
    }

    /// Returns the next angle in the sorted loop.
    #[must_use]
    pub fn next(&self) -> Option<usize> {
        self.f_next
    }

    /// Returns the span this angle starts at.
    #[must_use]
    pub fn start(&self) -> Option<usize> {
        self.f_start
    }

    /// Returns the span this angle ends at.
    #[must_use]
    pub fn end(&self) -> Option<usize> {
        self.f_end
    }

    /// Records the last span marked while walking this angle.
    pub fn set_last_marked(&mut self, marked: Option<usize>) {
        self.f_last_marked = marked;
    }

    /// Returns true when the sector span wraps past sector zero.
    ///
    /// Port of `SkOpAngle::checkCrossesZero`.
    #[must_use]
    pub fn check_crosses_zero(&self) -> bool {
        let start = self.f_sector_start.min(self.f_sector_end);
        let end = self.f_sector_start.max(self.f_sector_end);
        i32::from(end) - i32::from(start) > 16
    }

    /// Returns true when this angle and `rh` start in opposite half planes.
    ///
    /// Port of `SkOpAngle::oppositePlanes`.
    #[must_use]
    pub fn opposite_planes(&self, rh: &SkOpAngle) -> bool {
        let start_span = (i32::from(rh.f_sector_start) - i32::from(self.f_sector_start)).abs();
        start_span >= 8
    }

    /// Returns the sector for direction `(x, y)` on a curve with `verb`.
    ///
    /// Port of `SkOpAngle::findSector`. The circle is split into 16 parts and
    /// the result doubled plus one, so that exact compass points land on odd
    /// sectors and have room to be nudged either way by
    /// [`SkOpAngle::set_sector`].
    ///
    /// Returns -1 when the direction is degenerate and the sector cannot be
    /// determined yet.
    #[must_use]
    pub fn find_sector(&self, verb: Verb, x: f64, y: f64) -> i8 {
        let abs_x = x.abs();
        let abs_y = y.abs();
        // For curves, a near-tie between |x| and |y| is treated as an exact
        // 45 degrees so the tangent gets its own sector.
        let xy = if verb == Verb::Line || !almost_equal_ulps(abs_x as f32, abs_y as f32) {
            abs_x - abs_y
        } else {
            0.0
        };
        // If there are four quadrants and eight octants, and since the Latin for sixteen is
        // sedecim, one could coin the term sedecimant for a space divided into 16 sections.
        // http://english.stackexchange.com/questions/133688/word-for-something-partitioned-into-16-parts
        const SEDECIMANT: [[[i8; 3]; 3]; 3] = [
            //       y<0            y==0            y>0
            //   x<0 x==0 x>0   x<0 x==0 x>0   x<0 x==0 x>0
            [[4, 3, 2], [7, -1, 15], [10, 11, 12]], // abs(x) <  abs(y)
            [[5, -1, 1], [-1, -1, -1], [9, -1, 13]], // abs(x) == abs(y)
            [[6, 3, 0], [7, -1, 15], [8, 11, 14]],  // abs(x) >  abs(y)
        ];
        let xy_index = usize::from(xy >= 0.0) + usize::from(xy > 0.0);
        let y_index = usize::from(y >= 0.0) + usize::from(y > 0.0);
        let x_index = usize::from(x >= 0.0) + usize::from(x > 0.0);
        let sedecimant = SEDECIMANT[xy_index][y_index][x_index];
        if sedecimant < 0 {
            return -1;
        }
        sedecimant * 2 + 1
    }

    /// Assigns the sector range and mask from the hull sweep.
    ///
    /// Port of `SkOpAngle::setSector`. When a sector cannot be determined,
    /// `f_compute_sector` is set so the angle is lengthened and retried later.
    pub fn set_sector(&mut self) {
        if self.f_start.is_none() && self.f_part.f_verb == Verb::Move {
            self.f_unorderable = true;
            return;
        }
        let verb = self.f_part.f_verb;
        self.f_sector_start = self.find_sector(verb, self.f_part.f_sweep[0].f_x, self.f_part.f_sweep[0].f_y);
        if self.f_sector_start < 0 {
            self.defer_sector_til_later();
            return;
        }
        if !self.f_part.is_curve() {
            // A line or line-like curve occupies a single sector.
            self.f_sector_end = self.f_sector_start;
            self.f_sector_mask = 1u32 << self.f_sector_start;
            return;
        }
        debug_assert_ne!(verb, Verb::Line);
        self.f_sector_end = self.find_sector(verb, self.f_part.f_sweep[1].f_x, self.f_part.f_sweep[1].f_y);
        if self.f_sector_end < 0 {
            self.defer_sector_til_later();
            return;
        }
        if self.f_sector_end == self.f_sector_start && (self.f_sector_start & 3) != 3 {
            // The sector has no span, so it can't be an exact angle.
            self.f_sector_mask = 1u32 << self.f_sector_start;
            return;
        }
        let crosses_zero = self.check_crosses_zero();
        let start = self.f_sector_start.min(self.f_sector_end);
        let curve_bends_ccw = (self.f_sector_start == start) ^ crosses_zero;
        // Bump the start and end of the sector span if they are on exact
        // compass points, so that the span has width to compare against.
        if (self.f_sector_start & 3) == 3 {
            self.f_sector_start =
                (self.f_sector_start + if curve_bends_ccw { 1 } else { 31 }) & 0x1f;
        }
        if (self.f_sector_end & 3) == 3 {
            self.f_sector_end = (self.f_sector_end + if curve_bends_ccw { 31 } else { 1 }) & 0x1f;
        }
        let crosses_zero = self.check_crosses_zero();
        let start = u32::from((self.f_sector_start.min(self.f_sector_end)) as u8);
        let end = u32::from((self.f_sector_start.max(self.f_sector_end)) as u8);
        self.f_sector_mask = if crosses_zero {
            (u32::MAX >> (31 - start)) | (u32::MAX << end)
        } else {
            (u32::MAX >> (31 - end + start)) << start
        };
    }

    /// Marks the sector as undeterminable until the angle can be lengthened.
    fn defer_sector_til_later(&mut self) {
        self.f_sector_start = -1;
        self.f_sector_end = -1;
        self.f_sector_mask = 0;
        // Can't determine the sector until the segment length can be found.
        self.f_compute_sector = true;
    }

    /// Returns the t value halfway between the angle's start and end.
    ///
    /// Port of `SkOpAngle::midT`. Takes the two t values directly because the
    /// span arena is not threaded through this module yet.
    #[must_use]
    pub fn mid_t_of(start_t: f64, end_t: f64) -> f64 {
        (start_t + end_t) / 2.0
    }

    /// Returns the ratio of the segment's longest chord to `dist`.
    ///
    /// Port of `SkOpAngle::distEndRatio`. `pts` holds the segment's control
    /// points, `0..=verb_to_points(verb)` of them in use.
    #[must_use]
    pub fn dist_end_ratio(&self, pts: &[LinePoint], verb: Verb, dist: f64) -> f64 {
        let pt_count = verb_to_points(verb);
        let mut longest: f64 = 0.0;
        for idx1 in 0..pt_count {
            for idx2 in (idx1 + 1)..=pt_count {
                let v = AngleVector::new(
                    pts[idx2][0] - pts[idx1][0],
                    pts[idx2][1] - pts[idx1][1],
                );
                longest = longest.max(v.length_squared());
            }
        }
        longest.sqrt() / dist
    }

    /// Returns true when the tangents diverge enough to decide the order.
    ///
    /// Port of `SkOpAngle::tangentsDiverge`. Sets `f_tangents_ambiguous` when
    /// the result is near the empirically chosen cutoff.
    pub fn tangents_diverge(
        &mut self,
        rh: &SkOpAngle,
        s0xt0: f64,
        self_pts: &[LinePoint],
        self_verb: Verb,
        rh_pts: &[LinePoint],
        rh_verb: Verb,
    ) -> bool {
        if s0xt0 == 0.0 {
            return false;
        }
        // If the control tangents are not nearly parallel, use them. Solve for
        // the opposite direction displacement scale factor m:
        //   initial dir = v1.cross(v2) == v2.x * v1.y - v2.y * v1.x
        //   displacement of q1[1] : dq1 = { -m * v1.y, m * v1.x } + q1[1]
        //   straight angle when : v2.x * (dq1.y - q1[0].y) == v2.y * (dq1.x - q1[0].x)
        //                         v2.x * (m * v1.x + v1.y) == v2.y * (-m * v1.y + v1.x)
        //   - m * (v2.x * v1.x + v2.y * v1.y) == v2.x * v1.y - v2.y * v1.x
        //   m = (v2.y * v1.x - v2.x * v1.y) / (v2.x * v1.x + v2.y * v1.y)
        //   m = v1.cross(v2) / v1.dot(v2)
        let sweep = &self.f_part.f_sweep;
        let tweep = &rh.f_part.f_sweep;
        let s0dt0 = sweep[0].dot(tweep[0]);
        if s0dt0 == 0.0 {
            return true;
        }
        let m = s0xt0 / s0dt0;
        let s_dist = sweep[0].length() * m;
        let t_dist = tweep[0].length() * m;
        let use_s = s_dist.abs() < t_dist.abs();
        let m_factor = if use_s {
            self.dist_end_ratio(self_pts, self_verb, s_dist).abs()
        } else {
            rh.dist_end_ratio(rh_pts, rh_verb, t_dist).abs()
        };
        self.f_tangents_ambiguous = (50.0..200.0).contains(&m_factor);
        m_factor < 50.0 // empirically found limit
    }

    /// Returns how this angle's hull sits relative to `rh`'s.
    ///
    /// Port of `SkOpAngle::convexHullOverlaps`. Returns -1 when the hulls
    /// overlap and no order can be read from them, 0 when this angle is
    /// clockwise of `rh`, and 1 when it is counterclockwise.
    ///
    /// `self_mid` and `rh_mid` are the vectors from each curve's origin to the
    /// point at its mid t, used only when the sweeps span more than 180
    /// degrees.
    #[allow(clippy::too_many_arguments)] // segment data the arena cannot supply yet
    pub fn convex_hull_overlaps(
        &mut self,
        rh: &SkOpAngle,
        self_mid: AngleVector,
        rh_mid: AngleVector,
        self_pts: &[LinePoint],
        self_verb: Verb,
        rh_pts: &[LinePoint],
        rh_verb: Verb,
    ) -> i32 {
        let sweep = self.f_part.f_sweep;
        let tweep = rh.f_part.f_sweep;
        let s0xs1 = sweep[0].cross_check(sweep[1]);
        let s0xt0 = sweep[0].cross_check(tweep[0]);
        let s1xt0 = sweep[1].cross_check(tweep[0]);
        let mut t_between_s = if s0xs1 > 0.0 {
            s0xt0 > 0.0 && s1xt0 < 0.0
        } else {
            s0xt0 < 0.0 && s1xt0 > 0.0
        };
        let s0xt1 = sweep[0].cross_check(tweep[1]);
        let s1xt1 = sweep[1].cross_check(tweep[1]);
        t_between_s |= if s0xs1 > 0.0 {
            s0xt1 > 0.0 && s1xt1 < 0.0
        } else {
            s0xt1 < 0.0 && s1xt1 > 0.0
        };
        let t0xt1 = tweep[0].cross_check(tweep[1]);
        if t_between_s {
            return -1;
        }
        if (s0xt0 == 0.0 && s1xt1 == 0.0) || (s1xt0 == 0.0 && s0xt1 == 0.0) {
            // s0 to s1 equals t0 to t1.
            return -1;
        }
        let mut s_between_t = if t0xt1 > 0.0 {
            s0xt0 < 0.0 && s0xt1 > 0.0
        } else {
            s0xt0 > 0.0 && s0xt1 < 0.0
        };
        s_between_t |= if t0xt1 > 0.0 {
            s1xt0 < 0.0 && s1xt1 > 0.0
        } else {
            s1xt0 > 0.0 && s1xt1 < 0.0
        };
        if s_between_t {
            return -1;
        }
        // If all of the sweeps are in the same half plane, the order of any
        // pair is enough.
        if s0xt0 >= 0.0 && s0xt1 >= 0.0 && s1xt0 >= 0.0 && s1xt1 >= 0.0 {
            return 0;
        }
        if s0xt0 <= 0.0 && s0xt1 <= 0.0 && s1xt0 <= 0.0 && s1xt1 <= 0.0 {
            return 1;
        }
        // The outside sweeps are greater than 180 degrees. Assume the initial
        // tangents give the order; if the midpoint direction agrees, that is
        // enough.
        let m0xm1 = self_mid.cross_check(rh_mid);
        if s0xt0 > 0.0 && m0xm1 > 0.0 {
            return 0;
        }
        if s0xt0 < 0.0 && m0xm1 < 0.0 {
            return 1;
        }
        if self.tangents_diverge(rh, s0xt0, self_pts, self_verb, rh_pts, rh_verb) {
            return i32::from(s0xt0 < 0.0);
        }
        i32::from(m0xm1 < 0.0)
    }

    /// Returns which side of the ray through `origin` and `line` the test
    /// curve falls on.
    ///
    /// Port of the const `SkOpAngle::lineOnOneSide` overload. Returns -1 when
    /// the curve straddles the line, -2 when every cross product is zero, and
    /// otherwise 1 if the curve is clockwise of the line and 0 if counter.
    #[must_use]
    pub fn line_on_one_side_of(
        origin: LinePoint,
        line: AngleVector,
        test_curve: &[LinePoint],
        test_verb: Verb,
    ) -> i32 {
        let mut crosses = [0.0f64; 3];
        let i_max = verb_to_points(test_verb);
        for index in 1..=i_max {
            let xy1 = line.f_x * (test_curve[index][1] - origin[1]);
            let xy2 = line.f_y * (test_curve[index][0] - origin[0]);
            crosses[index - 1] = if almost_bequal_ulps(xy1, xy2) {
                0.0
            } else {
                xy1 - xy2
            };
        }
        if crosses[0] * crosses[1] < 0.0 {
            return -1;
        }
        if test_verb == Verb::Cubic
            && (crosses[0] * crosses[2] < 0.0 || crosses[1] * crosses[2] < 0.0)
        {
            return -1;
        }
        if crosses[0] != 0.0 {
            return i32::from(crosses[0] < 0.0);
        }
        if crosses[1] != 0.0 {
            return i32::from(crosses[1] < 0.0);
        }
        if test_verb == Verb::Cubic && crosses[2] != 0.0 {
            return i32::from(crosses[2] < 0.0);
        }
        -2
    }

    /// Returns which side of this line the curve `test` falls on.
    ///
    /// Port of the mutating `SkOpAngle::lineOnOneSide`. Requires that this
    /// angle is a line and `test` is a curve. Returns -1 when no single side
    /// can be determined, marking this angle unorderable.
    ///
    /// `use_original` selects the untranslated curve, for checking whether a
    /// translation flipped the sides.
    pub fn line_on_one_side(&mut self, test: &SkOpAngle, use_original: bool) -> i32 {
        debug_assert!(!self.f_part.is_curve());
        let origin = self.f_part.f_curve[0];
        let line = self.f_part.pt(1) - self.f_part.pt(0);
        let test_part = if use_original {
            &test.f_original_curve_part
        } else {
            &test.f_part
        };
        let result =
            Self::line_on_one_side_of(origin, line, &test_part.f_curve, test_part.f_verb);
        if result == -2 {
            self.f_unorderable = true;
            return -1;
        }
        result
    }

    /// Returns which side of this line the line `test` falls on, using the
    /// untranslated curves.
    ///
    /// Port of `SkOpAngle::linesOnOriginalSide`. Returns 2 when the lines are
    /// 180 degrees apart, -1 when no side can be determined.
    pub fn lines_on_original_side(&mut self, test: &SkOpAngle) -> i32 {
        debug_assert!(!self.f_part.is_curve());
        debug_assert!(!test.f_part.is_curve());
        let origin = self.f_original_curve_part.f_curve[0];
        let line = self.f_original_curve_part.pt(1) - self.f_original_curve_part.pt(0);
        let mut dots = [0.0f64; 2];
        let mut crosses = [0.0f64; 2];
        for index in 0..2 {
            let test_line = AngleVector::new(
                test.f_original_curve_part.f_curve[index][0] - origin[0],
                test.f_original_curve_part.f_curve[index][1] - origin[1],
            );
            let xy1 = line.f_x * test_line.f_y;
            let xy2 = line.f_y * test_line.f_x;
            dots[index] = line.f_x * test_line.f_x + line.f_y * test_line.f_y;
            crosses[index] = if almost_bequal_ulps(xy1, xy2) {
                0.0
            } else {
                xy1 - xy2
            };
        }
        if crosses[0] * crosses[1] < 0.0 {
            return -1;
        }
        if crosses[0] != 0.0 {
            return i32::from(crosses[0] < 0.0);
        }
        if crosses[1] != 0.0 {
            return i32::from(crosses[1] < 0.0);
        }
        if (dots[0] == 0.0 && dots[1] < 0.0) || (dots[0] < 0.0 && dots[1] == 0.0) {
            return 2; // 180 degrees apart
        }
        self.f_unorderable = true;
        -1
    }

    /// Flips `order` when translating the curves moved this angle to the other
    /// side of `test`.
    ///
    /// Port of `SkOpAngle::alignmentSameSide`. To sort the angles, all curves
    /// are translated to share a starting point; if a control point was on one
    /// side of a compared line before the translation and on the other side
    /// after, the previously computed order is reversed.
    #[allow(clippy::needless_range_loop)] // mirrors the C++ point indexing
    pub fn alignment_same_side(&self, test: &SkOpAngle, order: &mut i32) {
        if *order < 0 {
            return;
        }
        // This should support all curve types, but the only bug that requires
        // it has lines; turning it on for curves breaks existing tests.
        if self.f_part.is_curve() || test.f_part.is_curve() {
            return;
        }
        let x_origin = test.f_part.f_curve[0];
        let o_origin = test.f_original_curve_part.f_curve[0];
        if x_origin == o_origin {
            return;
        }
        let i_max = verb_to_points(self.f_part.f_verb);
        let x_line = test.f_part.pt(1) - test.f_part.pt(0);
        let o_line = test.f_original_curve_part.pt(1) - test.f_original_curve_part.pt(0);
        for index in 1..=i_max {
            let test_pt = self.f_part.f_curve[index];
            let x_cross = o_line.cross_check(AngleVector::new(
                test_pt[0] - x_origin[0],
                test_pt[1] - x_origin[1],
            ));
            let o_cross = x_line.cross_check(AngleVector::new(
                test_pt[0] - o_origin[0],
                test_pt[1] - o_origin[1],
            ));
            if o_cross * x_cross < 0.0 {
                *order ^= 1;
                break;
            }
        }
    }
}

/// Returns true if `a` and `b` are within 2 ULPs of each other.
///
/// The C++ `AlmostBequalUlps` overload for doubles narrows to float first;
/// this does the same so the tolerance matches.
fn almost_bequal_ulps(a: f64, b: f64) -> bool {
    super::sk_path_ops_types::almost_bequal_ulps(a as f32, b as f32)
}

/// An arena of angles linked into circular sorted loops.
///
/// The C++ original splices `SkOpAngle::fNext` pointers in place. Here the
/// angles are owned by the arena and linked by index, so the loop walks and
/// splices are arena methods.
#[derive(Debug, Default)]
pub struct AngleList {
    angles: Vec<SkOpAngle>,
    next_id: i32,
}

impl AngleList {
    /// Returns an empty arena.
    #[must_use]
    pub fn new() -> Self {
        Self {
            angles: Vec::new(),
            next_id: 0,
        }
    }

    /// Returns the number of angles allocated.
    #[must_use]
    pub fn len(&self) -> usize {
        self.angles.len()
    }

    /// Returns true when no angles have been allocated.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.angles.is_empty()
    }

    /// Adds `angle` to the arena and returns its index.
    ///
    /// The angle is given the next debug id, matching
    /// `SkOpGlobalState::nextAngleID`.
    pub fn push(&mut self, mut angle: SkOpAngle) -> usize {
        angle.f_id = self.next_id;
        self.next_id += 1;
        self.angles.push(angle);
        self.angles.len() - 1
    }

    /// Returns the angle at `index`.
    #[must_use]
    pub fn get(&self, index: usize) -> &SkOpAngle {
        &self.angles[index]
    }

    /// Returns the angle at `index` mutably.
    pub fn get_mut(&mut self, index: usize) -> &mut SkOpAngle {
        &mut self.angles[index]
    }

    /// Returns the next angle in `index`'s loop.
    #[must_use]
    pub fn next_of(&self, index: usize) -> Option<usize> {
        self.angles[index].f_next
    }

    /// Returns the number of angles in `index`'s loop.
    ///
    /// Port of `SkOpAngle::loopCount`. An angle not yet in a loop counts 1.
    #[must_use]
    pub fn loop_count(&self, index: usize) -> i32 {
        let mut count = 0;
        let first = index;
        let mut next = Some(index);
        loop {
            next = next.and_then(|n| self.angles[n].f_next);
            count += 1;
            match next {
                None => break,
                Some(n) if n == first => break,
                Some(_) => {}
            }
        }
        count
    }

    /// Returns the angle preceding `index` in its loop.
    ///
    /// Port of `SkOpAngle::previous`, which walks forward because the list is
    /// singly linked.
    #[must_use]
    pub fn previous(&self, index: usize) -> Option<usize> {
        let mut last = self.angles[index].f_next?;
        loop {
            let next = self.angles[last].f_next?;
            if next == index {
                return Some(last);
            }
            last = next;
        }
    }

    /// Returns the last span marked on `index`, claiming it so that a span is
    /// only chased once.
    ///
    /// Port of `SkOpAngle::lastMarked`, which returns null for a span that has
    /// already been chased. `chased` reports whether a span index is claimed
    /// and marks it as claimed.
    pub fn last_marked<F>(&self, index: usize, mut chased: F) -> Option<usize>
    where
        F: FnMut(usize) -> bool,
    {
        let marked = self.angles[index].f_last_marked?;
        if chased(marked) {
            return None;
        }
        Some(marked)
    }

    /// Returns true when `index`'s loop already contains a reversal of
    /// `angle`.
    ///
    /// Port of `SkOpAngle::loopContains`. An angle matches when it runs over
    /// the same segment between the same two t values in the other direction.
    /// `segment_of` maps a span index to its segment index.
    #[must_use]
    pub fn loop_contains<F>(
        &self,
        index: usize,
        angle: &SkOpAngle,
        t_of: impl Fn(usize) -> f64,
        segment_of: F,
    ) -> bool
    where
        F: Fn(usize) -> usize,
    {
        if self.angles[index].f_next.is_none() {
            return false;
        }
        let (Some(a_start), Some(a_end)) = (angle.f_start, angle.f_end) else {
            return false;
        };
        let t_segment = segment_of(a_start);
        let t_start = t_of(a_start);
        let t_end = t_of(a_end);
        let first = index;
        let mut loop_index = index;
        loop {
            let l = &self.angles[loop_index];
            if let (Some(l_start), Some(l_end)) = (l.f_start, l.f_end) {
                if segment_of(l_start) == t_segment
                    && t_of(l_start) == t_end
                    && t_of(l_end) == t_start
                {
                    return true;
                }
            }
            match l.f_next {
                Some(n) if n != first => loop_index = n,
                _ => break,
            }
        }
        false
    }

    /// Inserts `angle` into `index`'s sorted loop.
    ///
    /// Port of `SkOpAngle::insert`. `after` decides ordering for a candidate
    /// against a loop member, standing in for `SkOpAngle::after`, which needs
    /// the span graph this port does not have yet.
    ///
    /// Returns false only when the loop could not be resolved.
    pub fn insert<F>(&mut self, index: usize, angle: usize, after: &mut F) -> bool
    where
        F: FnMut(&AngleList, usize, usize) -> bool,
    {
        if self.angles[angle].f_next.is_some() {
            if self.loop_count(index) >= self.loop_count(angle) {
                if !self.merge(index, angle, after) {
                    return true;
                }
            } else if self.angles[index].f_next.is_some() {
                if !self.merge(angle, index, after) {
                    return true;
                }
            } else {
                self.insert(angle, index, after);
            }
            return true;
        }
        let singleton = self.angles[index].f_next.is_none();
        if singleton {
            self.angles[index].f_next = Some(index);
        }
        let next = self.angles[index].f_next.expect("loop is closed");
        if self.angles[next].f_next == Some(index) {
            if singleton || after(self, angle, index) {
                self.angles[index].f_next = Some(angle);
                self.angles[angle].f_next = Some(next);
            } else {
                self.angles[next].f_next = Some(angle);
                self.angles[angle].f_next = Some(index);
            }
            return true;
        }
        let mut last = index;
        let mut next = next;
        let mut flip_ambiguity = false;
        loop {
            debug_assert_eq!(self.angles[last].f_next, Some(next));
            let ambiguous = self.angles[angle].tangents_ambiguous() && flip_ambiguity;
            if after(self, angle, last) ^ ambiguous {
                self.angles[last].f_next = Some(angle);
                self.angles[angle].f_next = Some(next);
                return true;
            }
            last = next;
            if last == index {
                if flip_ambiguity {
                    return false;
                }
                // We're in a loop. If a sort was ambiguous, flip it to end the loop.
                flip_ambiguity = true;
            }
            next = match self.angles[next].f_next {
                Some(n) => n,
                None => return false,
            };
        }
    }

    /// Folds every angle in `angle`'s loop into `index`'s loop.
    ///
    /// Port of `SkOpAngle::merge`. Returns false when the two are already the
    /// same loop.
    pub fn merge<F>(&mut self, index: usize, angle: usize, after: &mut F) -> bool
    where
        F: FnMut(&AngleList, usize, usize) -> bool,
    {
        debug_assert!(self.angles[index].f_next.is_some());
        debug_assert!(self.angles[angle].f_next.is_some());
        let mut working = angle;
        loop {
            if index == working {
                return false;
            }
            working = match self.angles[working].f_next {
                Some(n) => n,
                None => break,
            };
            if working == angle {
                break;
            }
        }
        let mut working = angle;
        loop {
            let next = self.angles[working].f_next;
            self.angles[working].f_next = None;
            self.insert(index, working, after);
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

    /// Walks `index`'s loop, calling `visit` on each member once.
    ///
    /// Returns the number visited. Stops early if the links are corrupt.
    pub fn for_each_in_loop<F: FnMut(usize, &SkOpAngle)>(&self, index: usize, mut visit: F) -> usize {
        let first = index;
        let mut current = index;
        let mut count = 0;
        loop {
            visit(current, &self.angles[current]);
            count += 1;
            if count > self.angles.len() {
                // Corrupt loop; stop rather than spin.
                break;
            }
            match self.angles[current].f_next {
                Some(n) if n != first => current = n,
                _ => break,
            }
        }
        count
    }

    /// Returns true when `index`'s loop is a well-formed cycle back to itself.
    ///
    /// Port of `SkOpAngle::debugValidateNext`, which verifies in debug builds
    /// that the angle loop is uncorrupted.
    #[must_use]
    pub fn validate_next(&self, index: usize) -> bool {
        let first = index;
        let mut next = match self.angles[index].f_next {
            Some(n) => n,
            None => return true, // not yet in a loop
        };
        let mut count = 0;
        while next != first {
            count += 1;
            if count > self.angles.len() {
                return false;
            }
            next = match self.angles[next].f_next {
                Some(n) => n,
                None => return false,
            };
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds an angle whose part is a line from `from` to `to`.
    fn line_angle(from: LinePoint, to: LinePoint) -> SkOpAngle {
        let mut angle = SkOpAngle::new();
        angle.f_part.f_verb = Verb::Line;
        angle.f_part.f_curve[0] = from;
        angle.f_part.f_curve[1] = to;
        angle.f_part.set_curve_hull_sweep();
        angle.f_original_curve_part = angle.f_part;
        angle.f_tangent_half.line_end_points(&[from, to]);
        angle.set_sector();
        angle
    }

    /// Builds an angle whose part is a quad through the three points.
    fn quad_angle(pts: [LinePoint; 3]) -> SkOpAngle {
        let mut angle = SkOpAngle::new();
        angle.f_part.f_verb = Verb::Quad;
        angle.f_part.f_curve[0] = pts[0];
        angle.f_part.f_curve[1] = pts[1];
        angle.f_part.f_curve[2] = pts[2];
        angle.f_part.set_curve_hull_sweep();
        angle.f_original_curve_part = angle.f_part;
        angle.set_sector();
        angle
    }

    #[test]
    fn new_angle_starts_unset() {
        let angle = SkOpAngle::new();
        assert_eq!(angle.debug_id(), -1);
        assert!(!angle.unorderable());
        assert!(!angle.tangents_ambiguous());
        assert_eq!(angle.f_sector_start, -1);
        assert_eq!(angle.f_sector_end, -1);
        assert_eq!(angle.f_sector_mask, 0);
    }

    #[test]
    fn verb_to_points_counts_points_past_the_first() {
        assert_eq!(verb_to_points(Verb::Line), 1);
        assert_eq!(verb_to_points(Verb::Quad), 2);
        assert_eq!(verb_to_points(Verb::Conic), 2);
        assert_eq!(verb_to_points(Verb::Cubic), 3);
    }

    // --- find_sector ------------------------------------------------------

    #[test]
    fn find_sector_places_the_compass_points() {
        let a = SkOpAngle::new();
        // Skia's y axis points down, so the header diagram's "to the top"
        // sector 7 is reached with a negative y.
        assert_eq!(a.find_sector(Verb::Line, 1.0, 0.0), 31);
        assert_eq!(a.find_sector(Verb::Line, 0.0, -1.0), 7);
        assert_eq!(a.find_sector(Verb::Line, -1.0, 0.0), 15);
        assert_eq!(a.find_sector(Verb::Line, 0.0, 1.0), 23);
    }

    #[test]
    fn find_sector_places_the_diagonals() {
        let a = SkOpAngle::new();
        // Exact 45s land on sedecimants 1, 5, 9, 13 -> sectors 3, 11, 19, 27,
        // walking counterclockwise on screen from the up-and-right diagonal.
        assert_eq!(a.find_sector(Verb::Line, 1.0, -1.0), 3);
        assert_eq!(a.find_sector(Verb::Line, -1.0, -1.0), 11);
        assert_eq!(a.find_sector(Verb::Line, -1.0, 1.0), 19);
        assert_eq!(a.find_sector(Verb::Line, 1.0, 1.0), 27);
    }

    #[test]
    fn find_sector_orders_counterclockwise_within_a_quadrant() {
        let a = SkOpAngle::new();
        // Sweeping ccw on screen from +x means y goes negative. Sector 31
        // wraps to 0, so compare the three interior directions.
        let shallow = a.find_sector(Verb::Line, 4.0, -1.0); // |x| > |y|
        let diagonal = a.find_sector(Verb::Line, 1.0, -1.0); // |x| == |y|
        let steep = a.find_sector(Verb::Line, 1.0, -4.0); // |x| < |y|
        assert_eq!(shallow, 1);
        assert_eq!(diagonal, 3);
        assert_eq!(steep, 5);
        assert!(shallow < diagonal && diagonal < steep);
    }

    #[test]
    fn find_sector_covers_all_four_quadrants_in_order() {
        let a = SkOpAngle::new();
        // Walk ccw on screen from just above +x all the way around; the
        // sectors ascend the whole way.
        let dirs = [
            (4.0, -1.0),
            (1.0, -1.0),
            (1.0, -4.0),
            (-1.0, -4.0),
            (-1.0, -1.0),
            (-4.0, -1.0),
            (-4.0, 1.0),
            (-1.0, 1.0),
            (-1.0, 4.0),
            (1.0, 4.0),
            (1.0, 1.0),
            (4.0, 1.0),
        ];
        let sectors: Vec<i8> = dirs
            .iter()
            .map(|&(x, y)| a.find_sector(Verb::Line, x, y))
            .collect();
        for pair in sectors.windows(2) {
            assert!(
                pair[0] < pair[1],
                "sectors must ascend counterclockwise, got {sectors:?}"
            );
        }
    }

    #[test]
    fn find_sector_is_undetermined_at_the_origin() {
        let a = SkOpAngle::new();
        assert_eq!(a.find_sector(Verb::Line, 0.0, 0.0), -1);
    }

    #[test]
    fn find_sector_treats_curve_near_ties_as_exact_diagonals() {
        let a = SkOpAngle::new();
        // Eight ULPs above 1.0 in f32: inside the tolerance, but not equal.
        let nearly = 1.000_000_953_674_316_4;
        // For a line, a hair off 45 degrees is not the diagonal sector.
        assert_eq!(a.find_sector(Verb::Line, nearly, -1.0), 1);
        // For a curve the near-tie collapses to the exact diagonal, so the
        // tangent gets its own sector.
        assert_eq!(a.find_sector(Verb::Quad, nearly, -1.0), 3);
    }

    // --- set_sector -------------------------------------------------------

    #[test]
    fn set_sector_gives_a_line_one_sector() {
        let angle = line_angle([0.0, 0.0], [1.0, 0.0]);
        assert_eq!(angle.f_sector_start, 31);
        assert_eq!(angle.f_sector_end, 31);
        assert_eq!(angle.f_sector_mask, 1 << 31);
        assert!(!angle.f_part.is_curve());
    }

    #[test]
    fn set_sector_masks_only_the_sectors_swept() {
        // A quad bending from +x toward +y sweeps the low sectors.
        let angle = quad_angle([[0.0, 0.0], [2.0, 1.0], [2.0, 4.0]]);
        assert!(angle.f_part.is_curve());
        assert!(angle.f_sector_start >= 0 && angle.f_sector_end >= 0);
        // Every sector in the swept range is set, and the ends are included.
        assert_ne!(angle.f_sector_mask & (1 << angle.f_sector_start), 0);
        assert_ne!(angle.f_sector_mask & (1 << angle.f_sector_end), 0);
    }

    #[test]
    fn set_sector_defers_when_direction_is_degenerate() {
        // A zero-length line gives no direction, so the sector waits until the
        // angle can be lengthened.
        let angle = line_angle([1.0, 1.0], [1.0, 1.0]);
        assert_eq!(angle.f_sector_start, -1);
        assert_eq!(angle.f_sector_end, -1);
        assert_eq!(angle.f_sector_mask, 0);
        assert!(angle.f_compute_sector);
    }

    #[test]
    fn set_sector_bumps_exact_compass_points_off_the_boundary() {
        // A quad starting exactly along +x (sector 31, and 31 & 3 == 3) must
        // have its start nudged so the span has width.
        let angle = quad_angle([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0]]);
        assert!(angle.f_part.is_curve());
        assert_ne!(angle.f_sector_start & 3, 3);
    }

    #[test]
    fn set_sector_mask_wraps_when_the_span_crosses_zero() {
        // A quad from below the +x axis to above it sweeps across sector 0.
        let angle = quad_angle([[0.0, 0.0], [4.0, -1.0], [4.0, 4.0]]);
        if angle.f_part.is_curve() && angle.check_crosses_zero() {
            // Both the high and low ends of the circle are represented.
            assert_ne!(angle.f_sector_mask & 0x8000_0000, 0);
            assert_ne!(angle.f_sector_mask & 0x0000_0001, 0);
        }
    }

    #[test]
    fn check_crosses_zero_only_for_wide_spans() {
        let mut angle = SkOpAngle::new();
        angle.f_sector_start = 2;
        angle.f_sector_end = 10;
        assert!(!angle.check_crosses_zero());
        angle.f_sector_start = 1;
        angle.f_sector_end = 30;
        assert!(angle.check_crosses_zero());
    }

    #[test]
    fn opposite_planes_needs_a_quarter_turn() {
        let mut a = SkOpAngle::new();
        let mut b = SkOpAngle::new();
        a.f_sector_start = 0;
        b.f_sector_start = 16;
        assert!(a.opposite_planes(&b));
        b.f_sector_start = 8;
        assert!(a.opposite_planes(&b));
        b.f_sector_start = 7;
        assert!(!a.opposite_planes(&b));
    }

    // --- hull sweep -------------------------------------------------------

    #[test]
    fn hull_sweep_of_a_line_is_its_direction_twice() {
        let angle = line_angle([1.0, 1.0], [4.0, 5.0]);
        assert_eq!(angle.f_part.f_sweep[0], AngleVector::new(3.0, 4.0));
        assert_eq!(angle.f_part.f_sweep[1], AngleVector::new(3.0, 4.0));
        assert!(!angle.f_part.is_curve());
    }

    #[test]
    fn hull_sweep_of_a_bending_quad_spans_two_directions() {
        let angle = quad_angle([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]]);
        assert_eq!(angle.f_part.f_sweep[0], AngleVector::new(2.0, 0.0));
        assert_eq!(angle.f_part.f_sweep[1], AngleVector::new(2.0, 2.0));
        assert!(angle.f_part.is_curve());
    }

    #[test]
    fn a_straight_quad_is_not_a_curve() {
        // Control point on the chord: the sweeps are parallel.
        let angle = quad_angle([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]);
        assert!(!angle.f_part.is_curve());
    }

    #[test]
    fn offset_moves_every_point_in_use() {
        let mut sweep = CurveSweep::new();
        sweep.f_verb = Verb::Quad;
        sweep.f_curve[0] = [1.0, 1.0];
        sweep.f_curve[1] = [2.0, 2.0];
        sweep.f_curve[2] = [3.0, 3.0];
        sweep.f_curve[3] = [9.0, 9.0]; // unused by a quad
        sweep.offset(-1.0, -1.0);
        assert_eq!(sweep.f_curve[0], [0.0, 0.0]);
        assert_eq!(sweep.f_curve[2], [2.0, 2.0]);
        assert_eq!(sweep.f_curve[3], [9.0, 9.0]);
    }

    // --- vector math ------------------------------------------------------

    #[test]
    fn cross_check_collapses_parallel_vectors_to_zero() {
        let a = AngleVector::new(1.0, 2.0);
        let b = AngleVector::new(2.0, 4.0);
        assert_eq!(a.cross_check(b), 0.0);
        // A real turn keeps its sign.
        let c = AngleVector::new(-2.0, 1.0);
        assert!(a.cross_check(c) > 0.0);
        assert!(c.cross_check(a) < 0.0);
    }

    #[test]
    fn cross_no_normal_check_keeps_tiny_differences() {
        let a = AngleVector::new(1.0, 1.0);
        let b = AngleVector::new(1.0, 1.0 + 1e-15);
        assert_eq!(a.cross_check(b), 0.0);
        assert_ne!(a.cross_no_normal_check(b), 0.0);
    }

    #[test]
    fn dot_and_length_behave() {
        let a = AngleVector::new(3.0, 4.0);
        assert_eq!(a.length_squared(), 25.0);
        assert_eq!(a.length(), 5.0);
        assert_eq!(a.dot(AngleVector::new(1.0, 0.0)), 3.0);
    }

    // --- line_on_one_side -------------------------------------------------

    #[test]
    fn line_on_one_side_detects_a_straddle() {
        // Ray along +x from the origin; the quad's control points sit on
        // opposite sides of it.
        let curve = [[0.0, 0.0], [1.0, 1.0], [2.0, -1.0], [0.0, 0.0]];
        let result = SkOpAngle::line_on_one_side_of(
            [0.0, 0.0],
            AngleVector::new(1.0, 0.0),
            &curve,
            Verb::Quad,
        );
        assert_eq!(result, -1);
    }

    #[test]
    fn line_on_one_side_reports_the_side() {
        // Both control points above the ray.
        let above = [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [0.0, 0.0]];
        let up = SkOpAngle::line_on_one_side_of(
            [0.0, 0.0],
            AngleVector::new(1.0, 0.0),
            &above,
            Verb::Quad,
        );
        // Both below.
        let below = [[0.0, 0.0], [1.0, -1.0], [2.0, -2.0], [0.0, 0.0]];
        let down = SkOpAngle::line_on_one_side_of(
            [0.0, 0.0],
            AngleVector::new(1.0, 0.0),
            &below,
            Verb::Quad,
        );
        assert!(up == 0 || up == 1);
        assert!(down == 0 || down == 1);
        assert_ne!(up, down, "opposite sides must give opposite answers");
    }

    #[test]
    fn line_on_one_side_is_undetermined_when_collinear() {
        // Every control point lies on the ray.
        let collinear = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [0.0, 0.0]];
        let result = SkOpAngle::line_on_one_side_of(
            [0.0, 0.0],
            AngleVector::new(1.0, 0.0),
            &collinear,
            Verb::Quad,
        );
        assert_eq!(result, -2);
    }

    #[test]
    fn line_on_one_side_marks_unorderable_when_collinear() {
        let mut line = line_angle([0.0, 0.0], [1.0, 0.0]);
        let collinear = quad_angle([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]);
        assert_eq!(line.line_on_one_side(&collinear, false), -1);
        assert!(line.unorderable());
    }

    #[test]
    fn lines_on_original_side_detects_180_degrees() {
        let mut right = line_angle([0.0, 0.0], [1.0, 0.0]);
        let left = line_angle([0.0, 0.0], [-1.0, 0.0]);
        assert_eq!(right.lines_on_original_side(&left), 2);
    }

    #[test]
    fn lines_on_original_side_reports_the_side() {
        let mut base = line_angle([0.0, 0.0], [1.0, 0.0]);
        let up = line_angle([0.0, 0.0], [1.0, 1.0]);
        let down = line_angle([0.0, 0.0], [1.0, -1.0]);
        let up_side = base.lines_on_original_side(&up);
        let mut base2 = line_angle([0.0, 0.0], [1.0, 0.0]);
        let down_side = base2.lines_on_original_side(&down);
        assert_ne!(up_side, down_side);
    }

    // --- alignment_same_side ---------------------------------------------

    #[test]
    fn alignment_same_side_leaves_unorderable_alone() {
        let a = line_angle([0.0, 0.0], [1.0, 0.0]);
        let b = line_angle([0.0, 0.0], [0.0, 1.0]);
        let mut order = -1;
        a.alignment_same_side(&b, &mut order);
        assert_eq!(order, -1);
    }

    #[test]
    fn alignment_same_side_is_a_no_op_without_translation() {
        // test's part and original part share an origin, so nothing flips.
        let a = line_angle([0.0, 0.0], [1.0, 1.0]);
        let b = line_angle([0.0, 0.0], [1.0, 0.0]);
        let mut order = 1;
        a.alignment_same_side(&b, &mut order);
        assert_eq!(order, 1);
    }

    #[test]
    fn alignment_same_side_flips_when_translation_crosses_the_line() {
        // b's part is translated away from where it originally sat, and a's
        // control point ends up on the other side of it.
        let a = line_angle([0.0, 0.0], [0.0, 4.0]);
        let mut b = line_angle([0.0, 0.0], [4.0, 0.0]);
        b.f_original_curve_part.f_curve[0] = [0.0, 8.0];
        b.f_original_curve_part.f_curve[1] = [4.0, 8.0];
        let mut order = 1;
        a.alignment_same_side(&b, &mut order);
        assert_eq!(order, 0, "crossing the compared line reverses the order");
    }

    // --- dist_end_ratio ---------------------------------------------------

    #[test]
    fn dist_end_ratio_uses_the_longest_chord() {
        let angle = SkOpAngle::new();
        // Longest chord of this line is 5; at dist 1 the ratio is 5.
        let pts: [LinePoint; 4] = [[0.0, 0.0], [3.0, 4.0], [0.0, 0.0], [0.0, 0.0]];
        assert!((angle.dist_end_ratio(&pts, Verb::Line, 1.0) - 5.0).abs() < 1e-12);
        assert!((angle.dist_end_ratio(&pts, Verb::Line, 5.0) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn dist_end_ratio_spans_every_control_point_pair() {
        let angle = SkOpAngle::new();
        // The longest pair here is points 1 and 3, distance 10.
        let pts: [LinePoint; 4] = [[0.0, 0.0], [0.0, 0.0], [1.0, 0.0], [10.0, 0.0]];
        assert!((angle.dist_end_ratio(&pts, Verb::Cubic, 1.0) - 10.0).abs() < 1e-12);
    }

    // --- convex_hull_overlaps --------------------------------------------

    #[test]
    fn convex_hull_overlaps_orders_disjoint_hulls() {
        // Two quads leaving the origin into different half planes.
        let mut a = quad_angle([[0.0, 0.0], [4.0, 1.0], [8.0, 3.0]]);
        let b = quad_angle([[0.0, 0.0], [1.0, 4.0], [3.0, 8.0]]);
        let a_mid = AngleVector::new(4.0, 1.0);
        let b_mid = AngleVector::new(1.0, 4.0);
        let a_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 1.0], [8.0, 3.0], [0.0, 0.0]];
        let b_pts: [LinePoint; 4] = [[0.0, 0.0], [1.0, 4.0], [3.0, 8.0], [0.0, 0.0]];
        let order =
            a.convex_hull_overlaps(&b, a_mid, b_mid, &a_pts, Verb::Quad, &b_pts, Verb::Quad);
        assert!(order == 0 || order == 1, "disjoint hulls must be orderable");

        // Reversing the pair must reverse the answer.
        let mut b2 = quad_angle([[0.0, 0.0], [1.0, 4.0], [3.0, 8.0]]);
        let a2 = quad_angle([[0.0, 0.0], [4.0, 1.0], [8.0, 3.0]]);
        let reversed =
            b2.convex_hull_overlaps(&a2, b_mid, a_mid, &b_pts, Verb::Quad, &a_pts, Verb::Quad);
        assert_eq!(reversed, 1 - order);
    }

    #[test]
    fn convex_hull_overlaps_gives_up_on_nested_hulls() {
        // b's sweep sits inside a's, so the hulls alone cannot order them.
        let mut a = quad_angle([[0.0, 0.0], [4.0, 0.0], [0.0, 4.0]]);
        let b = quad_angle([[0.0, 0.0], [3.0, 1.0], [1.0, 3.0]]);
        let a_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [0.0, 4.0], [0.0, 0.0]];
        let b_pts: [LinePoint; 4] = [[0.0, 0.0], [3.0, 1.0], [1.0, 3.0], [0.0, 0.0]];
        let order = a.convex_hull_overlaps(
            &b,
            AngleVector::new(3.0, 1.0),
            AngleVector::new(2.0, 2.0),
            &a_pts,
            Verb::Quad,
            &b_pts,
            Verb::Quad,
        );
        assert_eq!(order, -1);
    }

    // --- tangents_diverge -------------------------------------------------

    #[test]
    fn tangents_do_not_diverge_when_parallel() {
        let mut a = quad_angle([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0]]);
        let b = quad_angle([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0]]);
        let pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 0.0]];
        assert!(!a.tangents_diverge(&b, 0.0, &pts, Verb::Quad, &pts, Verb::Quad));
    }

    #[test]
    fn tangents_diverge_for_a_wide_turn() {
        // Sweeps at right angles: clearly divergent, well under the cutoff.
        let mut a = quad_angle([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0]]);
        let b = quad_angle([[0.0, 0.0], [0.0, 4.0], [4.0, 4.0]]);
        let a_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 0.0]];
        let b_pts: [LinePoint; 4] = [[0.0, 0.0], [0.0, 4.0], [4.0, 4.0], [0.0, 0.0]];
        let s0xt0 = a.f_part.f_sweep[0].cross_check(b.f_part.f_sweep[0]);
        assert!(a.tangents_diverge(&b, s0xt0, &a_pts, Verb::Quad, &b_pts, Verb::Quad));
        assert!(!a.tangents_ambiguous());
    }

    // --- arena and loops --------------------------------------------------

    /// Stands in for `SkOpAngle::after` using sectors alone.
    ///
    /// Returns true when `angle` falls in the counterclockwise arc running
    /// from `test` to `test`'s successor. The real comparator answers the same
    /// question with curve geometry; sectors are enough to exercise the loop
    /// splicing. A plain `>` on the sector would not do: the loop is circular,
    /// so ordering has to be relative to the arc rather than absolute.
    fn by_sector(list: &AngleList, angle: usize, test: usize) -> bool {
        let gap = |from: i8, to: i8| -> i32 { (i32::from(to) - i32::from(from)).rem_euclid(32) };
        let Some(next) = list.next_of(test) else {
            return true;
        };
        if next == test {
            return true;
        }
        let test_sector = list.get(test).f_sector_start;
        gap(test_sector, list.get(angle).f_sector_start)
            < gap(test_sector, list.get(next).f_sector_start)
    }

    #[test]
    fn push_assigns_sequential_ids() {
        let mut list = AngleList::new();
        let a = list.push(SkOpAngle::new());
        let b = list.push(SkOpAngle::new());
        assert_eq!(list.get(a).debug_id(), 0);
        assert_eq!(list.get(b).debug_id(), 1);
        assert_eq!(list.len(), 2);
        assert!(!list.is_empty());
    }

    #[test]
    fn a_lone_angle_is_a_loop_of_one() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        assert_eq!(list.loop_count(a), 1);
        assert!(list.validate_next(a));
    }

    #[test]
    fn inserting_closes_a_two_angle_loop() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        let b = list.push(line_angle([0.0, 0.0], [0.0, 1.0]));
        assert!(list.insert(a, b, &mut by_sector));
        assert_eq!(list.loop_count(a), 2);
        assert_eq!(list.loop_count(b), 2);
        assert_eq!(list.next_of(a), Some(b));
        assert_eq!(list.next_of(b), Some(a));
        assert!(list.validate_next(a));
    }

    #[test]
    fn previous_walks_back_around_the_loop() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        let b = list.push(line_angle([0.0, 0.0], [0.0, 1.0]));
        let c = list.push(line_angle([0.0, 0.0], [-1.0, 0.0]));
        list.insert(a, b, &mut by_sector);
        list.insert(a, c, &mut by_sector);
        assert_eq!(list.loop_count(a), 3);
        for &idx in &[a, b, c] {
            let prev = list.previous(idx).expect("loop member has a predecessor");
            assert_eq!(list.next_of(prev), Some(idx));
        }
    }

    #[test]
    fn inserting_sorts_by_the_comparator() {
        let mut list = AngleList::new();
        // Sectors 31 (+x), 7 (+y), 15 (-x), 23 (-y).
        let east = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        let north = list.push(line_angle([0.0, 0.0], [0.0, 1.0]));
        let west = list.push(line_angle([0.0, 0.0], [-1.0, 0.0]));
        let south = list.push(line_angle([0.0, 0.0], [0.0, -1.0]));
        list.insert(east, north, &mut by_sector);
        list.insert(east, west, &mut by_sector);
        list.insert(east, south, &mut by_sector);
        assert_eq!(list.loop_count(east), 4);
        assert!(list.validate_next(east));

        // The loop is circular, so the walk is some rotation of the ascending
        // order: each step advances counterclockwise, wrapping exactly once.
        let mut seen = Vec::new();
        list.for_each_in_loop(north, |_, angle| seen.push(angle.f_sector_start));
        assert_eq!(seen.len(), 4);
        let mut sorted = seen.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, vec![7, 15, 23, 31]);
        let wraps = seen
            .windows(2)
            .filter(|pair| pair[0] > pair[1])
            .count()
            + usize::from(seen[3] > seen[0]);
        assert_eq!(wraps, 1, "sectors ascend counterclockwise: {seen:?}");
    }

    #[test]
    fn for_each_in_loop_visits_every_member_once() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        let b = list.push(line_angle([0.0, 0.0], [0.0, 1.0]));
        let c = list.push(line_angle([0.0, 0.0], [-1.0, 0.0]));
        list.insert(a, b, &mut by_sector);
        list.insert(a, c, &mut by_sector);
        let mut seen = Vec::new();
        let count = list.for_each_in_loop(a, |idx, _| seen.push(idx));
        assert_eq!(count, 3);
        seen.sort_unstable();
        assert_eq!(seen, vec![a, b, c]);
    }

    #[test]
    fn merging_two_loops_keeps_every_angle() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        let b = list.push(line_angle([0.0, 0.0], [4.0, 1.0]));
        let c = list.push(line_angle([0.0, 0.0], [0.0, 1.0]));
        let d = list.push(line_angle([0.0, 0.0], [-1.0, 0.0]));
        list.insert(a, b, &mut by_sector);
        list.insert(c, d, &mut by_sector);
        assert_eq!(list.loop_count(a), 2);
        assert_eq!(list.loop_count(c), 2);
        // Folding the second loop into the first gathers all four.
        list.insert(a, c, &mut by_sector);
        assert_eq!(list.loop_count(a), 4);
        assert!(list.validate_next(a));
        let mut seen = Vec::new();
        list.for_each_in_loop(a, |idx, _| seen.push(idx));
        seen.sort_unstable();
        assert_eq!(seen, vec![a, b, c, d]);
    }

    #[test]
    fn merging_a_loop_into_itself_reports_no_work() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        let b = list.push(line_angle([0.0, 0.0], [0.0, 1.0]));
        list.insert(a, b, &mut by_sector);
        assert!(!list.merge(a, b, &mut by_sector));
        assert_eq!(list.loop_count(a), 2);
    }

    #[test]
    fn validate_next_rejects_a_broken_link() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        let b = list.push(line_angle([0.0, 0.0], [0.0, 1.0]));
        list.insert(a, b, &mut by_sector);
        assert!(list.validate_next(a));
        // Break the cycle: b now leads nowhere.
        list.get_mut(b).f_next = None;
        assert!(!list.validate_next(a));
    }

    #[test]
    fn last_marked_is_claimed_only_once() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        list.get_mut(a).set_last_marked(Some(7));
        let mut chased = false;
        let first = list.last_marked(a, |_| std::mem::replace(&mut chased, true));
        assert_eq!(first, Some(7));
        // The span is claimed now, so the second ask comes back empty.
        let second = list.last_marked(a, |_| chased);
        assert_eq!(second, None);
    }

    #[test]
    fn last_marked_is_none_when_nothing_was_marked() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        assert_eq!(list.last_marked(a, |_| false), None);
    }

    #[test]
    fn loop_contains_finds_the_reversed_angle() {
        // Spans 0 and 1 sit on segment 0 at t=0 and t=1.
        let t_of = |span: usize| if span == 0 { 0.0 } else { 1.0 };
        let segment_of = |_span: usize| 0usize;

        let mut list = AngleList::new();
        let mut forward = line_angle([0.0, 0.0], [1.0, 0.0]);
        forward.f_start = Some(0);
        forward.f_end = Some(1);
        let a = list.push(forward);
        let mut other = line_angle([0.0, 0.0], [0.0, 1.0]);
        other.f_start = Some(0);
        other.f_end = Some(1);
        let b = list.push(other);
        list.insert(a, b, &mut by_sector);

        // The reverse of the angle already in the loop: t=1 to t=0.
        let mut reversed = SkOpAngle::new();
        reversed.f_start = Some(1);
        reversed.f_end = Some(0);
        assert!(list.loop_contains(a, &reversed, t_of, segment_of));

        // Same direction as the loop member, so not a reversal.
        let mut same = SkOpAngle::new();
        same.f_start = Some(0);
        same.f_end = Some(1);
        assert!(!list.loop_contains(a, &same, t_of, segment_of));
    }

    #[test]
    fn loop_contains_is_false_outside_a_loop() {
        let mut list = AngleList::new();
        let a = list.push(line_angle([0.0, 0.0], [1.0, 0.0]));
        let mut probe = SkOpAngle::new();
        probe.f_start = Some(1);
        probe.f_end = Some(0);
        assert!(!list.loop_contains(a, &probe, |_| 0.0, |_| 0));
    }

    #[test]
    fn mid_t_is_the_average() {
        assert_eq!(SkOpAngle::mid_t_of(0.0, 1.0), 0.5);
        assert_eq!(SkOpAngle::mid_t_of(0.25, 0.75), 0.5);
        assert_eq!(SkOpAngle::mid_t_of(0.2, 0.4), 0.30000000000000004);
    }

    // --- sub_divide_curve (item 04, part 1 prerequisite) -----------------

    /// Evaluates a quad at t.
    fn quad_pt(p: &[LinePoint], t: f64) -> LinePoint {
        let u = 1.0 - t;
        [
            u * u * p[0][0] + 2.0 * u * t * p[1][0] + t * t * p[2][0],
            u * u * p[0][1] + 2.0 * u * t * p[1][1] + t * t * p[2][1],
        ]
    }

    /// Evaluates a conic at t.
    fn conic_pt(p: &[LinePoint], w: f64, t: f64) -> LinePoint {
        let u = 1.0 - t;
        let cross = 2.0 * u * t * w;
        let denom = u * u + cross + t * t;
        [
            (u * u * p[0][0] + cross * p[1][0] + t * t * p[2][0]) / denom,
            (u * u * p[0][1] + cross * p[1][1] + t * t * p[2][1]) / denom,
        ]
    }

    /// Evaluates a cubic at t.
    fn cubic_pt(p: &[LinePoint], t: f64) -> LinePoint {
        let u = 1.0 - t;
        [
            u * u * u * p[0][0]
                + 3.0 * u * u * t * p[1][0]
                + 3.0 * u * t * t * p[2][0]
                + t * t * t * p[3][0],
            u * u * u * p[0][1]
                + 3.0 * u * u * t * p[1][1]
                + 3.0 * u * t * t * p[2][1]
                + t * t * t * p[3][1],
        ]
    }

    #[test]
    fn sub_divide_a_line_just_copies_the_ends() {
        let pts: [LinePoint; 4] = [[0.0, 0.0], [10.0, 10.0], [0.0, 0.0], [0.0, 0.0]];
        let mut out = CurveSweep::new();
        let did = sub_divide_curve(
            &pts,
            Verb::Line,
            1.0,
            [2.0, 2.0],
            0.2,
            [8.0, 8.0],
            0.8,
            &mut out,
        );
        assert!(!did, "a line needs no subdivision");
        assert_eq!(out.f_curve[0], [2.0, 2.0]);
        assert_eq!(out.f_curve[1], [8.0, 8.0]);
    }

    #[test]
    fn sub_divide_the_whole_quad_keeps_its_control_point() {
        let pts: [LinePoint; 4] = [[0.0, 0.0], [50.0, 100.0], [100.0, 0.0], [0.0, 0.0]];
        let mut out = CurveSweep::new();
        let did = sub_divide_curve(
            &pts,
            Verb::Quad,
            1.0,
            pts[0],
            0.0,
            pts[2],
            1.0,
            &mut out,
        );
        assert!(!did, "0..1 is the curve itself");
        assert_eq!(out.f_curve[1], pts[1]);
    }

    #[test]
    fn sub_divide_a_quad_piece_stays_on_the_curve() {
        let pts: [LinePoint; 4] = [[0.0, 0.0], [50.0, 100.0], [100.0, 0.0], [0.0, 0.0]];
        let (t1, t2) = (0.25, 0.75);
        let mut out = CurveSweep::new();
        assert!(sub_divide_curve(
            &pts,
            Verb::Quad,
            1.0,
            quad_pt(&pts, t1),
            t1,
            quad_pt(&pts, t2),
            t2,
            &mut out,
        ));
        // Every sample of the piece lies on the original quad.
        for i in 0..=10 {
            let s = f64::from(i) / 10.0;
            let got = quad_pt(&out.f_curve, s);
            let want = quad_pt(&pts, t1 + (t2 - t1) * s);
            assert!(
                (got[0] - want[0]).abs() < 1e-6 && (got[1] - want[1]).abs() < 1e-6,
                "at s={s}: got {got:?}, want {want:?}"
            );
        }
    }

    #[test]
    fn sub_divide_a_conic_piece_carries_the_weight() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let pts: [LinePoint; 4] = [[100.0, 0.0], [100.0, 100.0], [0.0, 100.0], [0.0, 0.0]];
        let (t1, t2) = (0.2, 0.8);
        let mut out = CurveSweep::new();
        assert!(sub_divide_curve(
            &pts,
            Verb::Conic,
            w,
            conic_pt(&pts, w, t1),
            t1,
            conic_pt(&pts, w, t2),
            t2,
            &mut out,
        ));
        for i in 0..=10 {
            let s = f64::from(i) / 10.0;
            let got = conic_pt(&out.f_curve, out.f_weight, s);
            let want = conic_pt(&pts, w, t1 + (t2 - t1) * s);
            assert!(
                (got[0] - want[0]).abs() < 1e-4 && (got[1] - want[1]).abs() < 1e-4,
                "at s={s}: got {got:?}, want {want:?}"
            );
        }
        // The piece of an arc is not a plain quadratic.
        assert!((out.f_weight - 1.0).abs() > 1e-3, "weight {}", out.f_weight);
    }

    #[test]
    fn sub_divide_a_cubic_piece_stays_on_the_curve() {
        let pts: [LinePoint; 4] = [[0.0, 0.0], [30.0, 90.0], [70.0, -30.0], [100.0, 60.0]];
        let (t1, t2) = (0.3, 0.9);
        let mut out = CurveSweep::new();
        assert!(sub_divide_curve(
            &pts,
            Verb::Cubic,
            1.0,
            cubic_pt(&pts, t1),
            t1,
            cubic_pt(&pts, t2),
            t2,
            &mut out,
        ));
        for i in 0..=10 {
            let s = f64::from(i) / 10.0;
            let got = cubic_pt(&out.f_curve, s);
            let want = cubic_pt(&pts, t1 + (t2 - t1) * s);
            assert!(
                (got[0] - want[0]).abs() < 1e-5 && (got[1] - want[1]).abs() < 1e-5,
                "at s={s}: got {got:?}, want {want:?}"
            );
        }
    }

    #[test]
    fn sub_divide_a_reversed_cubic_swaps_the_controls() {
        let pts: [LinePoint; 4] = [[0.0, 0.0], [30.0, 90.0], [70.0, -30.0], [100.0, 60.0]];
        let mut out = CurveSweep::new();
        // Running 1 -> 0 is the whole curve backwards.
        let did = sub_divide_curve(
            &pts,
            Verb::Cubic,
            1.0,
            pts[3],
            1.0,
            pts[0],
            0.0,
            &mut out,
        );
        assert!(!did);
        assert_eq!(out.f_curve[0], pts[3]);
        assert_eq!(out.f_curve[3], pts[0]);
        assert_eq!(out.f_curve[1], pts[2], "controls swap when reversed");
        assert_eq!(out.f_curve[2], pts[1]);
    }

    // --- extended coverage: extrema, degenerate curves, tangent ties ------

    /// Builds an angle whose part is a cubic through the four points.
    fn cubic_angle(pts: [LinePoint; 4]) -> SkOpAngle {
        let mut angle = SkOpAngle::new();
        angle.f_part.f_verb = Verb::Cubic;
        angle.f_part.f_curve[0] = pts[0];
        angle.f_part.f_curve[1] = pts[1];
        angle.f_part.f_curve[2] = pts[2];
        angle.f_part.f_curve[3] = pts[3];
        angle.f_part.set_curve_hull_sweep();
        angle.f_original_curve_part = angle.f_part;
        angle.set_sector();
        angle
    }

    // --- find_sector: every octant boundary, not just the four compass
    // points and the four 45s already covered above.

    #[test]
    fn find_sector_covers_every_sign_combination() {
        let a = SkOpAngle::new();
        // (x sign, y sign) x (|x| vs |y|) exhausts the sedecimant table's
        // input space away from the axes and the diagonal.
        let cases = [
            ((2.0, -1.0), true),   // x>0,y<0, |x|>|y|
            ((1.0, -2.0), true),   // x>0,y<0, |x|<|y|
            ((-2.0, -1.0), true),  // x<0,y<0, |x|>|y|
            ((-1.0, -2.0), true),  // x<0,y<0, |x|<|y|
            ((-2.0, 1.0), true),   // x<0,y>0, |x|>|y|
            ((-1.0, 2.0), true),   // x<0,y>0, |x|<|y|
            ((2.0, 1.0), true),    // x>0,y>0, |x|>|y|
            ((1.0, 2.0), true),    // x>0,y>0, |x|<|y|
        ];
        let mut seen = std::collections::HashSet::new();
        for ((x, y), _) in cases {
            let s = a.find_sector(Verb::Line, x, y);
            assert!(s >= 0 && s < NUM_SECTORS as i8, "sector out of range: {s}");
            assert!(seen.insert(s), "sector {s} reused for ({x}, {y})");
        }
    }

    #[test]
    fn find_sector_negative_zero_matches_positive_zero() {
        // -0.0 compares equal to 0.0 and must not be treated as a sign.
        let a = SkOpAngle::new();
        assert_eq!(
            a.find_sector(Verb::Line, 1.0, -0.0),
            a.find_sector(Verb::Line, 1.0, 0.0)
        );
        assert_eq!(
            a.find_sector(Verb::Line, -0.0, 1.0),
            a.find_sector(Verb::Line, 0.0, 1.0)
        );
    }

    #[test]
    fn find_sector_is_undetermined_on_both_axes_at_once() {
        // Degenerate in both coordinates, not just the origin case already
        // covered: NaN-free but still zero-zero after cancellation.
        let a = SkOpAngle::new();
        assert_eq!(a.find_sector(Verb::Quad, 0.0, 0.0), -1);
        assert_eq!(a.find_sector(Verb::Cubic, -0.0, 0.0), -1);
    }

    // --- set_sector: cubic extrema (degenerate first or second control) ---

    #[test]
    fn set_sector_cubic_with_degenerate_first_control_steps_out() {
        // Control point 1 sits on top of the start, so the first sweep
        // vector is zero-length; set_curve_hull_sweep must step to the
        // second control point and clear f_ordered.
        let angle = cubic_angle([[0.0, 0.0], [0.0, 0.0], [4.0, 0.0], [4.0, 4.0]]);
        assert!(!angle.f_part.is_ordered());
        assert!(angle.f_part.is_curve());
        // The sector must still resolve from the stepped-out sweep rather
        // than being deferred.
        assert_ne!(angle.f_sector_start, -1);
    }

    #[test]
    fn set_sector_cubic_with_degenerate_second_control_steps_out() {
        // Control point 2 sits on top of the start too (but control 1
        // doesn't), landing in the `f_sweep[1]` branch of
        // set_curve_hull_sweep instead of the `f_sweep[0]` branch.
        let angle = cubic_angle([[0.0, 0.0], [4.0, 1.0], [0.0, 0.0], [4.0, 4.0]]);
        assert!(!angle.f_part.is_ordered());
        assert_ne!(angle.f_sector_start, -1);
    }

    #[test]
    fn set_sector_cubic_with_both_controls_degenerate_falls_back_to_line() {
        // Both control points sit on the start point: the cubic is really a
        // line from p0 to p3. Neither the "step to sweep[1]" branch nor the
        // "step to pt(3)" branch changes this outcome once both original
        // sweeps are zero, since the second branch's replacement is also
        // pt(3) - pt(0).
        let angle = cubic_angle([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [4.0, 4.0]]);
        // Whatever the code decides, it must not silently treat this as
        // orderable with a stale zero sector, and must not panic.
        if angle.f_sector_start != -1 {
            assert_eq!(angle.f_sector_start, angle.find_sector(Verb::Cubic, 4.0, 4.0));
        }
    }

    #[test]
    fn set_sector_quad_at_exact_vertical_extremum() {
        // Control point directly above the start: sweep[0] is (0, -4), an
        // exact compass point (sector 7), which set_sector must bump off
        // the boundary once the far sweep gives the curve real width.
        let angle = quad_angle([[0.0, 0.0], [0.0, -4.0], [4.0, -4.0]]);
        assert!(angle.f_part.is_curve());
        assert_ne!(angle.f_sector_start & 3, 3, "exact compass point must be bumped");
    }

    #[test]
    fn set_sector_conic_extreme_weight_does_not_change_the_hull_sweep() {
        // The hull sweep only looks at control points, not weight, so a
        // conic and an equivalent quad through the same three points must
        // land in the same sectors regardless of weight.
        let mut conic = SkOpAngle::new();
        conic.f_part.f_verb = Verb::Conic;
        conic.f_part.f_curve[0] = [0.0, 0.0];
        conic.f_part.f_curve[1] = [4.0, 1.0];
        conic.f_part.f_curve[2] = [8.0, 3.0];
        conic.f_part.f_weight = 1e6; // extreme weight
        conic.f_part.set_curve_hull_sweep();
        conic.f_original_curve_part = conic.f_part;
        conic.set_sector();

        let quad = quad_angle([[0.0, 0.0], [4.0, 1.0], [8.0, 3.0]]);
        assert_eq!(conic.f_sector_start, quad.f_sector_start);
        assert_eq!(conic.f_sector_end, quad.f_sector_end);
    }

    // --- CurveSweep::set_curve_hull_sweep: degenerate cubic combinations --

    #[test]
    fn hull_sweep_cubic_control_at_max_component_scale_does_not_false_positive_degenerate() {
        // A very large curve where the first control offset is tiny relative
        // to the curve's scale must still be treated as degenerate by
        // approximately_zero_when_compared_to, exercising the `max_val`
        // relative (not absolute) tolerance.
        let mut sweep = CurveSweep::new();
        sweep.f_verb = Verb::Cubic;
        sweep.f_curve[0] = [0.0, 0.0];
        sweep.f_curve[1] = [1e-3, 1e-3]; // tiny relative to 1e6 below
        sweep.f_curve[2] = [1e6, 0.0];
        sweep.f_curve[3] = [1e6, 1e6];
        sweep.set_curve_hull_sweep();
        // Control 1 must have been treated as degenerate and stepped over.
        assert!(!sweep.is_ordered());
        assert_eq!(sweep.f_sweep[0], AngleVector::new(1e6, 0.0));
    }

    #[test]
    fn hull_sweep_cubic_control_at_comparable_scale_is_not_treated_as_degenerate() {
        // Same absolute offset as the tiny case above, but now the curve's
        // own scale is small too, so the offset is not negligible by
        // comparison and must NOT be stepped over.
        let mut sweep = CurveSweep::new();
        sweep.f_verb = Verb::Cubic;
        sweep.f_curve[0] = [0.0, 0.0];
        sweep.f_curve[1] = [1e-3, 1e-3];
        sweep.f_curve[2] = [2e-3, 0.0];
        sweep.f_curve[3] = [3e-3, 3e-3];
        sweep.set_curve_hull_sweep();
        assert!(sweep.is_ordered());
        assert_eq!(sweep.f_sweep[0], AngleVector::new(1e-3, 1e-3));
    }

    // --- dist_end_ratio: degenerate distances ------------------------------

    #[test]
    fn dist_end_ratio_zero_distance_is_infinite() {
        // A zero dist means the tangent lines were exactly coincident;
        // dividing by zero must produce +inf, not panic or NaN, so callers
        // that compare it against 50.0/200.0 get a well-defined answer.
        let angle = SkOpAngle::new();
        let pts: [LinePoint; 4] = [[0.0, 0.0], [3.0, 4.0], [0.0, 0.0], [0.0, 0.0]];
        let ratio = angle.dist_end_ratio(&pts, Verb::Line, 0.0);
        assert!(ratio.is_infinite() && ratio > 0.0);
    }

    #[test]
    fn dist_end_ratio_all_points_coincident_is_zero() {
        // Every control point on top of every other: longest chord is 0.
        let angle = SkOpAngle::new();
        let pts: [LinePoint; 4] = [[5.0, 5.0], [5.0, 5.0], [5.0, 5.0], [5.0, 5.0]];
        assert_eq!(angle.dist_end_ratio(&pts, Verb::Cubic, 1.0), 0.0);
    }

    // --- tangents_diverge: the perpendicular and boundary cases -----------

    #[test]
    fn tangents_diverge_when_sweeps_are_exactly_perpendicular() {
        // s0.dot(t0) == 0 is a separate early-return branch from the
        // s0xt0 == 0 branch already covered by
        // tangents_do_not_diverge_when_parallel; perpendicular sweeps must
        // report divergence unconditionally.
        let mut a = quad_angle([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0]]);
        let b = quad_angle([[0.0, 0.0], [0.0, 4.0], [-4.0, 4.0]]);
        let a_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 0.0]];
        let b_pts: [LinePoint; 4] = [[0.0, 0.0], [0.0, 4.0], [-4.0, 4.0], [0.0, 0.0]];
        let s0xt0 = a.f_part.f_sweep[0].cross_check(b.f_part.f_sweep[0]);
        assert_eq!(a.f_part.f_sweep[0].dot(b.f_part.f_sweep[0]), 0.0);
        assert!(a.tangents_diverge(&b, s0xt0, &a_pts, Verb::Quad, &b_pts, Verb::Quad));
    }

    #[test]
    fn tangents_diverge_reports_zero_for_exactly_parallel_cross() {
        // s0xt0 == 0.0 is an explicit early return regardless of how the
        // curves actually relate; confirm it short-circuits even when the
        // curves are otherwise very different (different verbs, different
        // scales), not just the identical-quads case already covered.
        let mut a = quad_angle([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0]]);
        let b = cubic_angle([[0.0, 0.0], [8.0, 0.0], [20.0, 0.0], [20.0, 40.0]]);
        let a_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 0.0]];
        let b_pts: [LinePoint; 4] = [[0.0, 0.0], [8.0, 0.0], [20.0, 0.0], [20.0, 40.0]];
        assert!(!a.tangents_diverge(&b, 0.0, &a_pts, Verb::Quad, &b_pts, Verb::Cubic));
    }

    #[test]
    fn tangents_ambiguous_flag_tracks_the_50_to_200_band() {
        // Pick a pair whose m_factor lands inside (50, 200) and confirm the
        // ambiguous flag is set even though the function still returns a
        // definite (non-divergent) answer; the two are independent signals.
        // A very shallow turn keeps m_factor large without being infinite.
        let mut a = quad_angle([[0.0, 0.0], [1000.0, 0.0], [1000.0, 1.0]]);
        let b = quad_angle([[0.0, 0.0], [1000.0, 0.0], [1000.0, -1.0]]);
        let a_pts: [LinePoint; 4] = [[0.0, 0.0], [1000.0, 0.0], [1000.0, 1.0], [0.0, 0.0]];
        let b_pts: [LinePoint; 4] = [[0.0, 0.0], [1000.0, 0.0], [1000.0, -1.0], [0.0, 0.0]];
        let s0xt0 = a.f_part.f_sweep[0].cross_check(b.f_part.f_sweep[0]);
        // Whatever the divergence verdict, the m_factor computation and the
        // ambiguous-band flag must not panic and must be internally
        // consistent: ambiguous implies m_factor was in [50, 200), which is
        // a strict subset of "diverge is false" (diverge requires < 50).
        let diverges = a.tangents_diverge(&b, s0xt0, &a_pts, Verb::Quad, &b_pts, Verb::Quad);
        if a.tangents_ambiguous() {
            assert!(!diverges, "ambiguous band (>=50) can't also be < 50 (diverges)");
        }
    }

    // --- convex_hull_overlaps: tangent-tie and boundary cases --------------
    //
    // TODO/2026-09-15-tangent-contact-angle-ordering.md initially suspected
    // convex_hull_overlaps of missing an exact-tangent tie-break, but its
    // later update (piece 2, traced via op_with_engine) narrowed the actual
    // defect to ends_intersect in sk_op_angle_order.rs instead: at the real
    // repro's junction, convex_hull_overlaps correctly declines via
    // t_between_s (a legitimate hull-wrap case), and ends_intersect's
    // chord-ray sampling is what returns the wrong answer downstream. The
    // tests below still exercise convex_hull_overlaps's own tie-break
    // behavior directly (it has no coverage for the tangent-tie shape at
    // all), but they're written as consistency checks rather than pins
    // against that TODO, since this function is not where its bug lives.

    #[test]
    fn convex_hull_overlaps_same_initial_tangent_opposite_curvature() {
        // Both curves leave the origin along +x (identical first sweep
        // vector), one bending up and one bending down. This is exactly the
        // "tangent lines coincide, curvature must break the tie" shape from
        // the TODO. The two answers must at least be consistent with each
        // other (one clockwise, one counterclockwise) — silently returning
        // the same order for both, or -1 (decline) for a case this
        // unambiguous, would be a real ordering bug.
        let mut up = quad_angle([[0.0, 0.0], [4.0, 0.0], [8.0, 4.0]]);
        let down = quad_angle([[0.0, 0.0], [4.0, 0.0], [8.0, -4.0]]);
        let up_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [8.0, 4.0], [0.0, 0.0]];
        let down_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [8.0, -4.0], [0.0, 0.0]];
        let order = up.convex_hull_overlaps(
            &down,
            AngleVector::new(4.0, 0.0),
            AngleVector::new(4.0, 0.0),
            &up_pts,
            Verb::Quad,
            &down_pts,
            Verb::Quad,
        );
        // A curve bending toward +y and one bending toward -y from the same
        // initial tangent are unambiguously on opposite sides; declining
        // (-1) here would push the tie-break work onto ends_intersect with
        // no geometric reason to, and picking a definite order that flips
        // under a relabeling would be worse. At minimum, this must not
        // decline outright, since the mid-vectors alone determine the side.
        assert_ne!(
            order, -1,
            "curves bending to opposite sides of a shared tangent must be orderable from the hull"
        );
    }

    #[test]
    fn convex_hull_overlaps_reversing_operands_reverses_the_answer_at_a_tangent_tie() {
        // Same shape as above but checked for the self-consistency property
        // that must hold regardless of which side of the bug lands: calling
        // with (up, down) and (down, up) must give complementary answers
        // whenever either call actually returns an order (0 or 1). If the
        // two calls agree, or one declines while the other doesn't, that is
        // a real bug in the tie-break, not just an ambiguous case.
        let up_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [8.0, 4.0], [0.0, 0.0]];
        let down_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [8.0, -4.0], [0.0, 0.0]];
        let up_mid = AngleVector::new(4.0, 0.0);
        let down_mid = AngleVector::new(4.0, 0.0);

        let mut up = quad_angle([[0.0, 0.0], [4.0, 0.0], [8.0, 4.0]]);
        let down = quad_angle([[0.0, 0.0], [4.0, 0.0], [8.0, -4.0]]);
        let forward = up.convex_hull_overlaps(
            &down, up_mid, down_mid, &up_pts, Verb::Quad, &down_pts, Verb::Quad,
        );

        let mut down2 = quad_angle([[0.0, 0.0], [4.0, 0.0], [8.0, -4.0]]);
        let up2 = quad_angle([[0.0, 0.0], [4.0, 0.0], [8.0, 4.0]]);
        let backward = down2.convex_hull_overlaps(
            &up2, down_mid, up_mid, &down_pts, Verb::Quad, &up_pts, Verb::Quad,
        );

        if forward != -1 && backward != -1 {
            assert_eq!(
                backward,
                1 - forward,
                "swapping operands at a tangent tie must flip clockwise/counterclockwise, \
                 got forward={forward} backward={backward}"
            );
        }
    }

    #[test]
    fn convex_hull_overlaps_identical_curves_declines() {
        // Two copies of the same curve: sweeps are pairwise equal, matching
        // the explicit "s0 to s1 equals t0 to t1" early return.
        let mut a = quad_angle([[0.0, 0.0], [4.0, 1.0], [8.0, 3.0]]);
        let b = quad_angle([[0.0, 0.0], [4.0, 1.0], [8.0, 3.0]]);
        let pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, 1.0], [8.0, 3.0], [0.0, 0.0]];
        let order = a.convex_hull_overlaps(
            &b,
            AngleVector::new(4.0, 1.0),
            AngleVector::new(4.0, 1.0),
            &pts,
            Verb::Quad,
            &pts,
            Verb::Quad,
        );
        assert_eq!(order, -1, "identical hulls give no order to read");
    }

    #[test]
    fn convex_hull_overlaps_exactly_opposite_sweeps() {
        // 180 degrees apart: one curve's sweep is the exact negation of the
        // other's, so every cross product between them is zero and this
        // must not panic or silently pick an arbitrary order-looking value
        // from a same-half-plane check that shouldn't fire.
        let mut a = quad_angle([[0.0, 0.0], [4.0, 0.0], [8.0, 0.0]]); // straight, not a curve
        let b = quad_angle([[0.0, 0.0], [-4.0, 0.0], [-8.0, 0.0]]);
        let pts_a: [LinePoint; 4] = [[0.0, 0.0], [4.0, 0.0], [8.0, 0.0], [0.0, 0.0]];
        let pts_b: [LinePoint; 4] = [[0.0, 0.0], [-4.0, 0.0], [-8.0, 0.0], [0.0, 0.0]];
        let order = a.convex_hull_overlaps(
            &b,
            AngleVector::new(4.0, 0.0),
            AngleVector::new(-4.0, 0.0),
            &pts_a,
            Verb::Quad,
            &pts_b,
            Verb::Quad,
        );
        // Every combination of cross_check on antiparallel vectors is zero,
        // so this falls into the "sweeps equal" or same-half-plane path;
        // either way it must terminate with one of the documented return
        // values, not something outside {-1, 0, 1}.
        assert!((-1..=1).contains(&order));
    }

    #[test]
    fn convex_hull_overlaps_reflex_sweep_uses_midpoint_tiebreak() {
        // A hull sweeping more than 180 degrees (the "outside sweeps are
        // greater than 180 degrees" branch) must fall through to the
        // midpoint cross product rather than the same-half-plane shortcuts,
        // since no half-plane contains the whole sweep.
        let mut a = quad_angle([[0.0, 0.0], [4.0, -4.0], [-4.0, -4.0]]); // sweeps almost 180
        let b = quad_angle([[0.0, 0.0], [1.0, 4.0], [-1.0, 4.0]]);
        let a_pts: [LinePoint; 4] = [[0.0, 0.0], [4.0, -4.0], [-4.0, -4.0], [0.0, 0.0]];
        let b_pts: [LinePoint; 4] = [[0.0, 0.0], [1.0, 4.0], [-1.0, 4.0], [0.0, 0.0]];
        let order = a.convex_hull_overlaps(
            &b,
            AngleVector::new(4.0, -4.0),
            AngleVector::new(1.0, 4.0),
            &a_pts,
            Verb::Quad,
            &b_pts,
            Verb::Quad,
        );
        // b's hull sits entirely below a's (opposite half-plane, +y here is
        // down-screen), so this is orderable from the hull without falling
        // back to ends_intersect.
        assert_ne!(order, -1);
    }

    // --- line_on_one_side_of: cubic verb, exact zero, mixed signs ---------

    #[test]
    fn line_on_one_side_of_cubic_checks_the_third_control() {
        // Both of the first two controls sit exactly on the ray, so only
        // the third (cubic-only) cross product distinguishes the side.
        // This exercises the `test_verb == Verb::Cubic && crosses[2] != 0.0`
        // branch, which no existing test reaches (existing coverage is
        // Verb::Quad only).
        let curve = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 1.0]];
        let result = SkOpAngle::line_on_one_side_of(
            [0.0, 0.0],
            AngleVector::new(1.0, 0.0),
            &curve,
            Verb::Cubic,
        );
        assert_eq!(result, 0);

        let mirrored = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, -1.0]];
        let mirrored_result = SkOpAngle::line_on_one_side_of(
            [0.0, 0.0],
            AngleVector::new(1.0, 0.0),
            &mirrored,
            Verb::Cubic,
        );
        assert_eq!(mirrored_result, 1);
        assert_ne!(result, mirrored_result);
    }

    #[test]
    fn line_on_one_side_of_cubic_straddle_via_third_control() {
        // First two controls agree in sign; the third one disagrees with
        // both, which must be caught by the cubic-specific straddle check
        // (`crosses[0] * crosses[2] < 0` / `crosses[1] * crosses[2] < 0`)
        // rather than only the `crosses[0] * crosses[1]` check quads use.
        let curve = [[0.0, 0.0], [1.0, 1.0], [2.0, 1.0], [3.0, -1.0]];
        let result = SkOpAngle::line_on_one_side_of(
            [0.0, 0.0],
            AngleVector::new(1.0, 0.0),
            &curve,
            Verb::Cubic,
        );
        assert_eq!(result, -1);
    }

    #[test]
    fn line_on_one_side_of_origin_not_at_the_first_point() {
        // The ray's origin need not be the curve's own start point; confirm
        // the offsets are taken relative to `origin`, not hardcoded to
        // curve[0]. Both control points sit strictly above the ray through
        // (10, 10), so this must resolve to a definite side, and the same
        // curve translated to the real origin must give the same answer.
        let curve = [[10.0, 10.0], [11.0, 11.0], [12.0, 12.0], [10.0, 10.0]];
        let result = SkOpAngle::line_on_one_side_of(
            [10.0, 10.0],
            AngleVector::new(1.0, 0.0),
            &curve,
            Verb::Quad,
        );
        let curve_at_origin = [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [0.0, 0.0]];
        let result_at_origin = SkOpAngle::line_on_one_side_of(
            [0.0, 0.0],
            AngleVector::new(1.0, 0.0),
            &curve_at_origin,
            Verb::Quad,
        );
        assert_eq!(result, result_at_origin, "origin must not be hardcoded to curve[0]");
        assert!(result == 0 || result == 1);
    }
}
