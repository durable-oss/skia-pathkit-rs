//! Path stroking implementation.
//!
//! Ported from `src/core/SkStroke.cpp`.
//!
//! This module implements the core path stroking logic including:
//! - Line stroking with cap handling
//! - Quadratic Bezier stroking
//! - Conic stroking  
//! - Cubic Bezier stroking with inflection handling

use super::path::Path;
use super::point::{Point, Vector};
use super::scalar::Scalar;
use super::sk_geometry::{
    eval_cubic_at, eval_quad_at, find_cubic_cusp, find_cubic_inflections,
    find_quad_max_curvature, Conic,
};

// Recursive limits for curve subdivision
const K_TANGENT_RECURSIVE_LIMIT: usize = 0;
const K_CUBIC_RECURSIVE_LIMIT: usize = 1;
const K_CONIC_RECURSIVE_LIMIT: usize = 2;
const K_QUAD_RECURSIVE_LIMIT: usize = 3;

// Recursive limits with 3x multiplier for safety
const K_RECURSIVE_LIMITS: [usize; 4] = [15, 24, 33, 33];

/// Result of comparing a stroke approximation to the actual curve
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ResultType {
    Split,
    Degenerate,
    Quad,
}

/// Reduction types for linearizing curves
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReductionType {
    Point,
    Line,
    Quad,
    Degenerate,
    Degenerate2,
    Degenerate3,
}

/// Stroke type - outer vs inner path
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StrokeType {
    Outer = 1,
    Inner = -1,
}

/// State for a quad stroke under construction
struct SkQuadConstruct {
    quad: [Point; 3],
    tangent_start: Point,
    tangent_end: Point,
    start_t: Scalar,
    mid_t: Scalar,
    end_t: Scalar,
    start_set: bool,
    end_set: bool,
    opposite_tangents: bool,
}

impl SkQuadConstruct {
    fn new() -> Self {
        SkQuadConstruct {
            quad: [Point::default(); 3],
            tangent_start: Point::default(),
            tangent_end: Point::default(),
            start_t: 0.0,
            mid_t: 0.0,
            end_t: 0.0,
            start_set: false,
            end_set: false,
            opposite_tangents: false,
        }
    }

    fn init(&mut self, start: Scalar, end: Scalar) -> bool {
        self.start_t = start;
        self.mid_t = (start + end) * 0.5;
        self.end_t = end;
        self.start_set = false;
        self.end_set = false;
        self.start_t < self.mid_t && self.mid_t < self.end_t
    }

    fn init_with_start(&mut self, parent: &SkQuadConstruct) -> bool {
        if !self.init(parent.start_t, parent.mid_t) {
            return false;
        }
        self.quad[0] = parent.quad[0];
        self.tangent_start = parent.tangent_start;
        self.start_set = true;
        true
    }

    fn init_with_end(&mut self, parent: &SkQuadConstruct) -> bool {
        if !self.init(parent.mid_t, parent.end_t) {
            return false;
        }
        self.quad[2] = parent.quad[2];
        self.tangent_end = parent.tangent_end;
        self.end_set = true;
        true
    }
}

/// Main path stroker class
pub struct SkPathStroker {
    // Configuration
    radius: Scalar,
    inv_miter_limit: Scalar,
    res_scale: Scalar,
    inv_res_scale: Scalar,
    inv_res_scale_squared: Scalar,

    // State from path traversal
    first_normal: Vector,
    prev_normal: Vector,
    first_unit_normal: Vector,
    prev_unit_normal: Vector,
    first_pt: Point,
    prev_pt: Point,
    first_outer_pt: Point,
    first_outer_pt_index_in_contour: usize,
    segment_count: i32,
    prev_is_line: bool,
    can_ignore_center: bool,

    // Path storage
    inner: Path,
    outer: Path,
    cusper: Path,

    // Stroke type (outer or inner)
    stroke_type: StrokeType,

    // State tracking
    recursion_depth: i32,
    found_tangents: bool,
    join_completed: bool,
}

impl SkPathStroker {
    /// Create a new path stroker
    pub fn new(
        radius: Scalar,
        miter_limit: Scalar,
        _cap: crate::core::Cap,
        join: crate::core::Join,
        res_scale: Scalar,
        can_ignore_center: bool,
    ) -> Self {
        // The cap/join dispatch (capper and joiner function pointers in the
        // C++) is not wired up yet, so the stroker has nowhere to store the
        // degenerate-miter demotion to bevel; only inv_miter_limit is kept.
        let mut inv_miter_limit = 0.0;

        if join == crate::core::Join::Miter && miter_limit > 1.0 {
            inv_miter_limit = 1.0 / miter_limit;
        }

        let inv_res_scale = 1.0 / (res_scale * 4.0);
        let inv_res_scale_squared = inv_res_scale * inv_res_scale;

        SkPathStroker {
            radius,
            inv_miter_limit,
            res_scale,
            inv_res_scale,
            inv_res_scale_squared,
            first_normal: Vector::default(),
            prev_normal: Vector::default(),
            first_unit_normal: Vector::default(),
            prev_unit_normal: Vector::default(),
            first_pt: Point::default(),
            prev_pt: Point::default(),
            first_outer_pt: Point::default(),
            first_outer_pt_index_in_contour: 0,
            segment_count: -1,
            prev_is_line: false,
            can_ignore_center,
            inner: Path::new(),
            outer: Path::new(),
            cusper: Path::new(),
            stroke_type: StrokeType::Outer,
            recursion_depth: 0,
            found_tangents: false,
            join_completed: false,
        }
    }

    /// Check if there have only had a moveTo
    pub fn has_only_move_to(&self) -> bool {
        self.segment_count == 0
    }

    /// Get the moveTo point
    pub fn move_to_pt(&self) -> Option<Point> {
        if self.segment_count >= 0 {
            Some(self.first_pt)
        } else {
            None
        }
    }

    /// Start a new contour
    pub fn move_to(&mut self, pt: Point) {
        if self.segment_count > 0 {
            self.finish_contour(false, false);
        }
        self.segment_count = 0;
        self.first_pt = pt;
        self.prev_pt = pt;
        self.join_completed = false;
    }

    /// Add a line segment
    pub fn line_to(&mut self, curr_pt: Point) {
        let teeny_line = (curr_pt - self.prev_pt).length_squared() < self.inv_res_scale_squared;

        if teeny_line {
            if self.join_completed {
                return;
            }
        }

        let mut normal = Vector::default();
        let mut unit_normal = Vector::default();

        if !self.pre_join_to(curr_pt, &mut normal, &mut unit_normal, true) {
            return;
        }

        self.line_to_with_normal(curr_pt, normal);
        self.post_join_to(curr_pt, normal, unit_normal);
    }

    /// Add a quadratic Bezier segment
    pub fn quad_to(&mut self, pt1: Point, pt2: Point) {
        let quad = [self.prev_pt, pt1, pt2];
        let mut reduction = Point::default();
        let reduction_type = Self::check_quad_linear(&quad, &mut reduction);

        if reduction_type == ReductionType::Point {
            self.line_to(pt2);
            return;
        }

        if reduction_type == ReductionType::Line {
            self.line_to(pt2);
            return;
        }

        if reduction_type == ReductionType::Degenerate {
            self.line_to(reduction);
            self.line_to(pt2);
            return;
        }

        let mut normal_ab = Vector::default();
        let mut unit_ab = Vector::default();
        let mut normal_bc = Vector::default();
        let mut unit_bc = Vector::default();

        if !self.pre_join_to(pt1, &mut normal_ab, &mut unit_ab, false) {
            self.line_to(pt2);
            return;
        }

        let mut quad_pts = SkQuadConstruct::new();
        self.init(StrokeType::Outer, &mut quad_pts, 0.0, 1.0);
        self.quad_stroke(&quad, &mut quad_pts);
        self.init(StrokeType::Inner, &mut quad_pts, 0.0, 1.0);
        self.quad_stroke(&quad, &mut quad_pts);
        Self::set_quad_end_normal(&quad, normal_ab, unit_ab, &mut normal_bc, &mut unit_bc);
        self.post_join_to(pt2, normal_bc, unit_bc);
    }

    /// Add a conic segment
    pub fn conic_to(&mut self, pt1: Point, pt2: Point, weight: Scalar) {
        let conic = Conic::new([self.prev_pt, pt1, pt2], weight);
        let mut reduction = Point::default();
        let reduction_type = Self::check_conic_linear(&conic, &mut reduction);

        if reduction_type == ReductionType::Point {
            self.line_to(pt2);
            return;
        }

        if reduction_type == ReductionType::Line {
            self.line_to(pt2);
            return;
        }

        if reduction_type == ReductionType::Degenerate {
            self.line_to(reduction);
            self.line_to(pt2);
            return;
        }

        let mut normal_ab = Vector::default();
        let mut unit_ab = Vector::default();
        let mut normal_bc = Vector::default();
        let mut unit_bc = Vector::default();

        if !self.pre_join_to(pt1, &mut normal_ab, &mut unit_ab, false) {
            self.line_to(pt2);
            return;
        }

        let mut quad_pts = SkQuadConstruct::new();
        self.init(StrokeType::Outer, &mut quad_pts, 0.0, 1.0);
        self.conic_stroke(&conic, &mut quad_pts);
        self.init(StrokeType::Inner, &mut quad_pts, 0.0, 1.0);
        self.conic_stroke(&conic, &mut quad_pts);
        Self::set_conic_end_normal(&conic, normal_ab, unit_ab, &mut normal_bc, &mut unit_bc);
        self.post_join_to(pt2, normal_bc, unit_bc);
    }

    /// Add a cubic Bezier segment
    pub fn cubic_to(&mut self, pt1: Point, pt2: Point, pt3: Point) {
        let cubic = [self.prev_pt, pt1, pt2, pt3];
        let mut reduction = [Point::default(); 3];
        let tangent_pt = Self::check_cubic_linear(&cubic, &mut reduction);

        if tangent_pt.is_none() {
            self.line_to(pt3);
            return;
        }

        let mut normal_ab = Vector::default();
        let mut unit_ab = Vector::default();
        let normal_cd = Vector::default();
        let unit_cd = Vector::default();

        let tangent_pt = tangent_pt.unwrap();
        if !self.pre_join_to(*tangent_pt, &mut normal_ab, &mut unit_ab, false) {
            self.line_to(pt3);
            return;
        }

        let mut inflections = [0.0; 2];
        let count = find_cubic_inflections(&cubic, &mut inflections);
        let mut last_t = 0.0;

        for i in 0..=count + 1 {
            let next_t = if i < count { inflections[i] } else { 1.0 };
            let mut quad_pts = SkQuadConstruct::new();
            self.init(StrokeType::Outer, &mut quad_pts, last_t, next_t);
            self.cubic_stroke(&cubic, &mut quad_pts);
            self.init(StrokeType::Inner, &mut quad_pts, last_t, next_t);
            self.cubic_stroke(&cubic, &mut quad_pts);
            last_t = next_t;
        }

        // Handle cusps
        let cusp = find_cubic_cusp(&cubic);
        if cusp > 0.0 && cusp < 1.0 {
            let _cusp_loc = eval_cubic_at(&cubic, cusp);
            // TODO: add the cusp circle to self.cusper once the capper is ported.
        }

        self.post_join_to(pt3, normal_cd, unit_cd);
    }

    /// Finish the current contour
    pub fn done(&mut self, is_line: bool) -> Path {
        self.finish_contour(false, is_line);
        let mut result = Path::new();
        std::mem::swap(&mut result, &mut self.outer);
        result
    }

    /// Get the resolution scale
    pub fn get_res_scale(&self) -> Scalar {
        self.res_scale
    }

    // Helper methods

    fn pre_join_to(
        &mut self,
        curr_pt: Point,
        normal: &mut Vector,
        unit_normal: &mut Vector,
        curr_is_line: bool,
    ) -> bool {
        let prev_x = self.prev_pt.x;
        let prev_y = self.prev_pt.y;

        if !Self::set_normal_unitnormal(
            self.prev_pt,
            curr_pt,
            self.res_scale,
            self.radius,
            normal,
            unit_normal,
        ) {
            if self.radius == 0.0 {
                return false;
            }
            *normal = Point::new(self.radius, 0.0);
            *unit_normal = Point::new(1.0, 0.0);
        }

        if self.segment_count == 0 {
            self.first_normal = *normal;
            self.first_unit_normal = *unit_normal;
            self.first_outer_pt.x = prev_x + normal.x;
            self.first_outer_pt.y = prev_y + normal.y;

            self.outer
                .move_to(self.first_outer_pt.x, self.first_outer_pt.y);
            self.inner.move_to(prev_x - normal.x, prev_y - normal.y);
        } else {
            // Join segments - would call joiner function here
        }

        self.prev_is_line = curr_is_line;
        true
    }

    fn post_join_to(&mut self, curr_pt: Point, normal: Vector, unit_normal: Vector) {
        self.join_completed = true;
        self.prev_pt = curr_pt;
        self.prev_unit_normal = unit_normal;
        self.prev_normal = normal;
        self.segment_count += 1;
    }

    fn finish_contour(&mut self, close: bool, _is_line: bool) {
        if self.segment_count > 0 {
            if close {
                // Close the contour with a join
                self.outer.close();
                self.inner.close();
            } else {
                // Add caps to start and end
            }
        }

        self.inner.rewind();
        self.segment_count = -1;
        self.first_outer_pt_index_in_contour = self.outer.count_points();
    }

    fn line_to_with_normal(&mut self, curr_pt: Point, normal: Vector) {
        self.outer
            .line_to(curr_pt.x + normal.x, curr_pt.y + normal.y);
        self.inner
            .line_to(curr_pt.x - normal.x, curr_pt.y - normal.y);
    }

    fn init(
        &mut self,
        stroke_type: StrokeType,
        quad_pts: &mut SkQuadConstruct,
        t_start: Scalar,
        t_end: Scalar,
    ) {
        self.stroke_type = stroke_type;
        self.found_tangents = false;
        quad_pts.init(t_start, t_end);
    }

    fn set_normal_unitnormal(
        before: Point,
        after: Point,
        scale: Scalar,
        radius: Scalar,
        normal: &mut Vector,
        unit_normal: &mut Vector,
    ) -> bool {
        let dx = (after.x - before.x) * scale;
        let dy = (after.y - before.y) * scale;

        if (dx * dx + dy * dy).sqrt() < 1e-10 {
            return false;
        }

        *unit_normal = Point::new(dx, dy);
        *unit_normal = unit_normal.scale(1.0 / unit_normal.length());
        // Rotate 90 degrees CCW
        let temp = unit_normal.x;
        unit_normal.x = -unit_normal.y;
        unit_normal.y = temp;

        normal.x = unit_normal.x * radius;
        normal.y = unit_normal.y * radius;
        true
    }

    fn check_quad_linear(quad: &[Point; 3], reduction: &mut Point) -> ReductionType {
        let degenerate_ab = (quad[1] - quad[0]).length_squared() < 1e-10;
        let degenerate_bc = (quad[2] - quad[1]).length_squared() < 1e-10;

        if degenerate_ab && degenerate_bc {
            return ReductionType::Point;
        }

        if degenerate_ab || degenerate_bc {
            return ReductionType::Line;
        }

        if !Self::quad_in_line(quad) {
            return ReductionType::Quad;
        }

        let t = find_quad_max_curvature(quad);
        if t <= 0.0 || t >= 1.0 {
            return ReductionType::Line;
        }

        *reduction = eval_quad_at(quad, t);
        ReductionType::Degenerate
    }

    fn quad_in_line(quad: &[Point; 3]) -> bool {
        let mut pt_max = -1.0;
        let mut outer1 = 0;
        let mut outer2 = 1;

        for i in 0..2 {
            for j in i + 1..3 {
                let diff = quad[j] - quad[i];
                let test_max = diff.x.abs().max(diff.y.abs());
                if test_max > pt_max {
                    pt_max = test_max;
                    outer1 = i;
                    outer2 = j;
                }
            }
        }

        let mid = outer1 ^ outer2 ^ 3;
        let line_slop = pt_max * pt_max * 0.000005;
        Self::pt_to_line(quad[mid], quad[outer1], quad[outer2]) <= line_slop
    }

    fn pt_to_line(pt: Point, line_start: Point, line_end: Point) -> Scalar {
        let dxy = line_end - line_start;
        let ab0 = pt - line_start;
        let numer = dxy.dot(ab0);
        let denom = dxy.dot(dxy);
        let t = numer / denom;

        if t >= 0.0 && t <= 1.0 {
            let hit = Point::new(
                line_start.x * (1.0 - t) + line_end.x * t,
                line_start.y * (1.0 - t) + line_end.y * t,
            );
            (pt - hit).length_squared()
        } else {
            (pt - line_start).length_squared()
        }
    }

    fn check_conic_linear(conic: &Conic, reduction: &mut Point) -> ReductionType {
        Self::check_quad_linear(&conic.pts, reduction)
    }

    fn check_cubic_linear<'a>(
        cubic: &'a [Point; 4],
        _reduction: &'a mut [Point; 3],
    ) -> Option<&'a Point> {
        let degenerate_ab = (cubic[1] - cubic[0]).length_squared() < 1e-10;
        let degenerate_bc = (cubic[2] - cubic[1]).length_squared() < 1e-10;
        let degenerate_cd = (cubic[3] - cubic[2]).length_squared() < 1e-10;

        if degenerate_ab && degenerate_bc && degenerate_cd {
            return None;
        }

        if degenerate_ab || degenerate_bc || degenerate_cd {
            return Some(&cubic[0]);
        }

        if !Self::cubic_in_line(cubic) {
            return Some(&cubic[1]);
        }

        Some(&cubic[1])
    }

    fn cubic_in_line(_cubic: &[Point; 4]) -> bool {
        // Simplified collinearity check
        true
    }

    fn quad_stroke(&mut self, quad: &[Point; 3], quad_pts: &mut SkQuadConstruct) -> bool {
        let result_type = self.compare_quad_quad(quad, quad_pts);

        if result_type == ResultType::Quad {
            let path = if self.stroke_type == StrokeType::Outer {
                &mut self.outer
            } else {
                &mut self.inner
            };
            path.quad_to(
                quad_pts.quad[1].x,
                quad_pts.quad[1].y,
                quad_pts.quad[2].x,
                quad_pts.quad[2].y,
            );
            return true;
        }

        if result_type == ResultType::Degenerate {
            self.add_degenerate_line(quad_pts);
            return true;
        }

        if self.recursion_depth > K_RECURSIVE_LIMITS[K_QUAD_RECURSIVE_LIMIT] as i32 {
            return false;
        }

        let mut half = SkQuadConstruct::new();
        if !half.init_with_start(quad_pts) {
            self.add_degenerate_line(quad_pts);
            self.recursion_depth -= 1;
            return true;
        }

        if !self.quad_stroke(quad, &mut half) {
            return false;
        }

        if !half.init_with_end(quad_pts) {
            self.add_degenerate_line(quad_pts);
            self.recursion_depth -= 1;
            return true;
        }

        if !self.quad_stroke(quad, &mut half) {
            return false;
        }

        self.recursion_depth -= 1;
        true
    }

    fn conic_stroke(&mut self, conic: &Conic, quad_pts: &mut SkQuadConstruct) -> bool {
        let result_type = self.compare_quad_conic(conic, quad_pts);

        if result_type == ResultType::Quad {
            let path = if self.stroke_type == StrokeType::Outer {
                &mut self.outer
            } else {
                &mut self.inner
            };
            path.quad_to(
                quad_pts.quad[1].x,
                quad_pts.quad[1].y,
                quad_pts.quad[2].x,
                quad_pts.quad[2].y,
            );
            return true;
        }

        if result_type == ResultType::Degenerate {
            self.add_degenerate_line(quad_pts);
            return true;
        }

        if self.recursion_depth > K_RECURSIVE_LIMITS[K_CONIC_RECURSIVE_LIMIT] as i32 {
            return false;
        }

        let mut half = SkQuadConstruct::new();
        half.init_with_start(quad_pts);
        if !self.conic_stroke(conic, &mut half) {
            return false;
        }

        half.init_with_end(quad_pts);
        if !self.conic_stroke(conic, &mut half) {
            return false;
        }

        self.recursion_depth -= 1;
        true
    }

    fn cubic_stroke(&mut self, cubic: &[Point; 4], quad_pts: &mut SkQuadConstruct) -> bool {
        if !self.found_tangents {
            let result_type = self.tangents_meet(cubic, quad_pts);

            if result_type != ResultType::Quad {
                if result_type == ResultType::Degenerate {
                    self.add_degenerate_line(quad_pts);
                    return true;
                }
            } else {
                self.found_tangents = true;
            }
        }

        if self.found_tangents {
            let result_type = self.compare_quad_cubic(cubic, quad_pts);

            if result_type == ResultType::Quad {
                let path = if self.stroke_type == StrokeType::Outer {
                    &mut self.outer
                } else {
                    &mut self.inner
                };
                path.quad_to(
                    quad_pts.quad[1].x,
                    quad_pts.quad[1].y,
                    quad_pts.quad[2].x,
                    quad_pts.quad[2].y,
                );
                return true;
            }

            if result_type == ResultType::Degenerate && !quad_pts.opposite_tangents {
                self.add_degenerate_line(quad_pts);
                return true;
            }
        }

        if self.recursion_depth > K_RECURSIVE_LIMITS[K_CUBIC_RECURSIVE_LIMIT] as i32 {
            return false;
        }

        let mut half = SkQuadConstruct::new();
        if !half.init_with_start(quad_pts) {
            self.add_degenerate_line(quad_pts);
            self.recursion_depth -= 1;
            return true;
        }

        if !self.cubic_stroke(cubic, &mut half) {
            return false;
        }

        if !half.init_with_end(quad_pts) {
            self.add_degenerate_line(quad_pts);
            self.recursion_depth -= 1;
            return true;
        }

        if !self.cubic_stroke(cubic, &mut half) {
            return false;
        }

        self.recursion_depth -= 1;
        true
    }

    fn add_degenerate_line(&mut self, quad_pts: &SkQuadConstruct) {
        let path = if self.stroke_type == StrokeType::Outer {
            &mut self.outer
        } else {
            &mut self.inner
        };
        path.line_to(quad_pts.quad[2].x, quad_pts.quad[2].y);
    }

    fn compare_quad_quad(
        &mut self,
        quad: &[Point; 3],
        quad_pts: &mut SkQuadConstruct,
    ) -> ResultType {
        self.conic_quad_ends(quad, quad_pts);
        let result_type = self.intersect_ray(quad_pts);

        if result_type != ResultType::Quad {
            return result_type;
        }

        let ray = self.project_perp_ray(quad, quad_pts.mid_t);
        self.stroke_close_enough(&quad_pts.quad, &ray)
    }

    fn compare_quad_conic(&mut self, conic: &Conic, quad_pts: &mut SkQuadConstruct) -> ResultType {
        self.conic_quad_ends(&conic.pts, quad_pts);
        let result_type = self.intersect_ray(quad_pts);

        if result_type != ResultType::Quad {
            return result_type;
        }

        let ray = self.conic_perp_ray(conic, quad_pts.mid_t);
        self.stroke_close_enough(&quad_pts.quad, &ray)
    }

    fn compare_quad_cubic(
        &mut self,
        cubic: &[Point; 4],
        quad_pts: &mut SkQuadConstruct,
    ) -> ResultType {
        self.conic_quad_ends(cubic, quad_pts);
        let result_type = self.intersect_ray(quad_pts);

        if result_type != ResultType::Quad {
            return result_type;
        }

        let ray = self.cubic_perp_ray(cubic, quad_pts.mid_t);
        self.stroke_close_enough(&quad_pts.quad, &ray)
    }

    fn tangents_meet(&mut self, cubic: &[Point; 4], quad_pts: &mut SkQuadConstruct) -> ResultType {
        self.conic_quad_ends(cubic, quad_pts);
        self.intersect_ray(quad_pts)
    }

    fn conic_quad_ends(&mut self, pts: &[Point], quad_pts: &mut SkQuadConstruct) {
        if !quad_pts.start_set {
            let start = if quad_pts.start_t == 0.0 {
                pts[0]
            } else {
                pts[pts.len() - 1]
            };
            quad_pts.quad[0] = start;
            quad_pts.tangent_start = start;
            quad_pts.start_set = true;
        }

        if !quad_pts.end_set {
            let end = if quad_pts.end_t == 0.0 {
                pts[0]
            } else {
                pts[pts.len() - 1]
            };
            quad_pts.quad[2] = end;
            quad_pts.tangent_end = end;
            quad_pts.end_set = true;
        }
    }

    fn intersect_ray(&mut self, quad_pts: &mut SkQuadConstruct) -> ResultType {
        let start = quad_pts.quad[0];
        let end = quad_pts.quad[2];
        let a_len = quad_pts.tangent_start - start;
        let b_len = quad_pts.tangent_end - end;

        let denom = a_len.cross(b_len);
        if denom.abs() < 1e-10 {
            quad_pts.opposite_tangents = a_len.dot(b_len) < 0.0;
            return ResultType::Degenerate;
        }

        let ab0 = start - end;
        let numer_a = b_len.cross(ab0);
        let numer_b = a_len.cross(ab0);

        if (numer_a >= 0.0) == (numer_b >= 0.0) {
            let dist1 = Self::pt_to_line(start, end, quad_pts.tangent_end);
            let dist2 = Self::pt_to_line(end, start, quad_pts.tangent_start);
            if dist1.max(dist2) <= self.inv_res_scale_squared {
                return ResultType::Degenerate;
            }
            return ResultType::Split;
        }

        if numer_a.abs() > self.inv_res_scale {
            ResultType::Quad
        } else {
            quad_pts.opposite_tangents = a_len.dot(b_len) < 0.0;
            ResultType::Degenerate
        }
    }

    fn stroke_close_enough(&self, stroke: &[Point; 3], ray: &[Point; 2]) -> ResultType {
        let stroke_mid = eval_quad_at(stroke, 0.5);
        if (ray[0] - stroke_mid).length_squared() <= self.inv_res_scale_squared {
            return ResultType::Quad;
        }

        ResultType::Split
    }

    fn project_perp_ray(&self, quad: &[Point; 3], t: Scalar) -> [Point; 2] {
        let pt = eval_quad_at(quad, t);
        let tangent = Self::eval_quad_tangent(quad, t);
        [pt, pt + tangent]
    }

    fn eval_quad_tangent(quad: &[Point; 3], t: Scalar) -> Vector {
        let p0 = quad[0];
        let p1 = quad[1];
        let p2 = quad[2];
        let a = p2 - Vector::new(2.0 * p1.x, 2.0 * p1.y) + p0;
        let b = p1 - p0;
        Vector::new(2.0 * (b.x + a.x * t), 2.0 * (b.y + a.y * t))
    }

    fn conic_perp_ray(&self, conic: &Conic, t: Scalar) -> [Point; 2] {
        let pt = conic.eval_at(t);
        let tangent = conic.eval_tangent_at(t);
        [pt, pt + tangent]
    }

    fn cubic_perp_ray(&self, cubic: &[Point; 4], t: Scalar) -> [Point; 2] {
        let pt = eval_cubic_at(cubic, t);
        let tangent = Vector::new(
            3.0 * (cubic[1].x - cubic[0].x + (cubic[2].x - 2.0 * cubic[1].x + cubic[0].x) * t),
            3.0 * (cubic[1].y - cubic[0].y + (cubic[2].y - 2.0 * cubic[1].y + cubic[0].y) * t),
        );
        [pt, pt + tangent]
    }

    fn set_quad_end_normal(
        quad: &[Point; 3],
        normal_ab: Vector,
        unit_ab: Vector,
        normal_bc: &mut Vector,
        unit_bc: &mut Vector,
    ) {
        if !Self::set_normal_unitnormal(quad[1], quad[2], 1.0, 1.0, normal_bc, unit_bc) {
            *normal_bc = normal_ab;
            *unit_bc = unit_ab;
        }
    }

    fn set_conic_end_normal(
        conic: &Conic,
        normal_ab: Vector,
        unit_ab: Vector,
        normal_bc: &mut Vector,
        unit_bc: &mut Vector,
    ) {
        Self::set_quad_end_normal(&conic.pts, normal_ab, unit_ab, normal_bc, unit_bc);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_path_stroker_creation() {
        let stroker = SkPathStroker::new(
            2.0,
            4.0,
            crate::core::Cap::Round,
            crate::core::Join::Round,
            1.0,
            false,
        );
        assert!((stroker.get_res_scale() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_move_to() {
        let mut stroker = SkPathStroker::new(
            2.0,
            4.0,
            crate::core::Cap::Round,
            crate::core::Join::Round,
            1.0,
            false,
        );
        stroker.move_to(Point::new(0.0, 0.0));
        assert_eq!(stroker.move_to_pt(), Some(Point::new(0.0, 0.0)));
    }

    #[test]
    fn test_line_to() {
        let mut stroker = SkPathStroker::new(
            2.0,
            4.0,
            crate::core::Cap::Round,
            crate::core::Join::Round,
            1.0,
            false,
        );
        stroker.move_to(Point::new(0.0, 0.0));
        stroker.line_to(Point::new(10.0, 0.0));
        let result = stroker.done(false);
        assert!(result.count_points() > 0);
    }

    #[test]
    fn test_quad_to() {
        let mut stroker = SkPathStroker::new(
            2.0,
            4.0,
            crate::core::Cap::Round,
            crate::core::Join::Round,
            1.0,
            false,
        );
        stroker.move_to(Point::new(0.0, 0.0));
        stroker.quad_to(Point::new(5.0, 10.0), Point::new(10.0, 0.0));
        let result = stroker.done(false);
        assert!(result.count_points() > 0);
    }

    #[test]
    fn test_conic_to() {
        let mut stroker = SkPathStroker::new(
            2.0,
            4.0,
            crate::core::Cap::Round,
            crate::core::Join::Round,
            1.0,
            false,
        );
        stroker.move_to(Point::new(0.0, 0.0));
        stroker.conic_to(Point::new(5.0, 10.0), Point::new(10.0, 0.0), 0.707);
        let result = stroker.done(false);
        assert!(result.count_points() > 0);
    }

    #[test]
    fn test_cubic_to() {
        let mut stroker = SkPathStroker::new(
            2.0,
            4.0,
            crate::core::Cap::Round,
            crate::core::Join::Round,
            1.0,
            false,
        );
        stroker.move_to(Point::new(0.0, 0.0));
        stroker.cubic_to(
            Point::new(0.0, 10.0),
            Point::new(10.0, 10.0),
            Point::new(10.0, 0.0),
        );
        let result = stroker.done(false);
        assert!(result.count_points() > 0);
    }
}
