//! SkTSect and SkTSpan - curve intersection utilities for path operations
//!
//! Port of Skia's SkPathOpsTSect.{h,cpp}
//!
//! This module provides the data structures and algorithms for finding
//! intersections between parametric curves (quadratic, cubic, conic).

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

use std::cell::RefCell;

use std::rc::Rc;

use crate::core::{Point, Scalar};

/// Maximum number of spans to prevent infinite loops
const MAX_SPAN_ITERATIONS: i32 = 10000;
/// Safety net for deletion iterations
const MAX_DELETION_ITERATIONS: i32 = 1000;
/// Coincident span count threshold
const COINCIDENT_SPAN_COUNT: i32 = 9;
/// Tolerance for point equality
const POINT_EPSILON: Scalar = 1e-9;
/// Tolerance for T value comparison.
///
/// Mirrors Skia's `ROUGH_EPSILON` (`FLT_EPSILON * 64`, ~7.6e-6): `Scalar`
/// is `f32`, so a `double`-scale `1e-9` tolerance is too tight to absorb
/// realistic float rounding in `t` values near 0 or 1.
const T_EPSILON: Scalar = f32::EPSILON * 64.0;

/// Check if a scalar is between two others (inclusive)
fn between(a: Scalar, b: Scalar, c: Scalar) -> bool {
    (a - b) * (c - b) <= 0.0
}

/// Check if `b` is roughly between `a` and `c` (in either order), within
/// `T_EPSILON` tolerance. Mirrors the middle-argument-is-the-value
/// convention of the C++ `between`/`precisely_between` helpers.
fn roughly_between(a: Scalar, b: Scalar, c: Scalar) -> bool {
    if a <= c {
        (a - b) < T_EPSILON && (b - c) < T_EPSILON
    } else {
        (b - a) < T_EPSILON && (c - b) < T_EPSILON
    }
}

/// Check if `b` is precisely between `a` and `c` (in either order).
fn precisely_between(a: Scalar, b: Scalar, c: Scalar) -> bool {
    if a <= c {
        a <= b && b <= c
    } else {
        c <= b && b <= a
    }
}

/// Check if a scalar is approximately zero
fn approximately_zero(x: Scalar) -> bool {
    x.abs() < 1e-6
}

/// Check if a scalar is precisely zero
fn precisely_zero(x: Scalar) -> bool {
    x == 0.0
}

/// Check if two values are zero when compared to a max value
fn precisely_zero_when_compared_to(x: Scalar, max_val: Scalar) -> bool {
    x.abs() <= 1e-10 * max_val.abs().max(1e-10)
}

fn approximately_zero_when_compared_to(x: Scalar, max_val: Scalar) -> bool {
    x.abs() <= 1e-6 * max_val.abs().max(1e-6)
}

/// Check if two points are approximately equal
fn points_approximately_equal(a: Point, b: Point) -> bool {
    (a.x - b.x).abs() < POINT_EPSILON && (a.y - b.y).abs() < POINT_EPSILON
}

/// Compute squared distance between two points
fn distance_squared(a: Point, b: Point) -> Scalar {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}

/// Compute curve point at parameter t
fn curve_pt_at_t(curve: &Curve, t: Scalar) -> Point {
    curve.pt_at_t(t)
}

/// Compute curve derivative at parameter t
fn curve_dxdy_at_t(curve: &Curve, t: Scalar) -> Point {
    curve.dxdy_at_t(t)
}

/// Simplified curve representation for intersection testing
struct Curve {
    points: [Point; 4],
}

impl Curve {
    fn new(points: [Point; 4]) -> Self {
        Self { points }
    }

    fn point_count(&self) -> usize {
        4 // Always 4 control points for cubic/quadratic
    }

    fn pt_at_t(&self, t: Scalar) -> Point {
        // Evaluate cubic Bezier at t
        let one_minus_t = 1.0 - t;
        let t2 = t * t;
        let t3 = t2 * t;
        let one_minus_t2 = one_minus_t * one_minus_t;
        let one_minus_t3 = one_minus_t2 * one_minus_t;

        Point {
            x: one_minus_t3 * self.points[0].x
                + 3.0 * one_minus_t2 * t * self.points[1].x
                + 3.0 * one_minus_t * t2 * self.points[2].x
                + t3 * self.points[3].x,
            y: one_minus_t3 * self.points[0].y
                + 3.0 * one_minus_t2 * t * self.points[1].y
                + 3.0 * one_minus_t * t2 * self.points[2].y
                + t3 * self.points[3].y,
        }
    }

    fn dxdy_at_t(&self, t: Scalar) -> Point {
        // Derivative of cubic Bezier at t
        let one_minus_t = 1.0 - t;
        let t2 = t * t;
        let one_minus_t2 = one_minus_t * one_minus_t;

        Point {
            x: 3.0 * one_minus_t2 * (self.points[1].x - self.points[0].x)
                + 6.0 * one_minus_t * t * (self.points[2].x - self.points[1].x)
                + 3.0 * t2 * (self.points[3].x - self.points[2].x),
            y: 3.0 * one_minus_t2 * (self.points[1].y - self.points[0].y)
                + 6.0 * one_minus_t * t * (self.points[2].y - self.points[1].y)
                + 3.0 * t2 * (self.points[3].y - self.points[2].y),
        }
    }

    fn collapsed(&self) -> bool {
        // Check if curve has collapsed to a point
        points_approximately_equal(self.points[0], self.points[3])
    }

    fn controls_inside(&self) -> bool {
        // Check if control points are within the start/end bounds
        let mut min_x = self.points[0].x.min(self.points[3].x);
        let mut max_x = self.points[0].x.max(self.points[3].x);
        let mut min_y = self.points[0].y.min(self.points[3].y);
        let mut max_y = self.points[0].y.max(self.points[3].y);

        for i in 1..4 {
            min_x = min_x.min(self.points[i].x);
            max_x = max_x.max(self.points[i].x);
            min_y = min_y.min(self.points[i].y);
            max_y = max_y.max(self.points[i].y);
        }

        true // Simplified - actual implementation is more complex
    }

    fn point0(&self) -> Point {
        self.points[0]
    }

    fn point3(&self) -> Point {
        self.points[3]
    }
}

/// Bounding box for a curve
struct DRect {
    left: Scalar,
    top: Scalar,
    right: Scalar,
    bottom: Scalar,
}

impl DRect {
    fn new(left: Scalar, top: Scalar, right: Scalar, bottom: Scalar) -> Self {
        Self {
            left,
            top,
            right,
            bottom,
        }
    }

    fn empty() -> Self {
        Self {
            left: 0.0,
            top: 0.0,
            right: 0.0,
            bottom: 0.0,
        }
    }

    fn valid(&self) -> bool {
        !self.left.is_nan() && !self.top.is_nan()
    }

    fn width(&self) -> Scalar {
        self.right - self.left
    }

    fn height(&self) -> Scalar {
        self.bottom - self.top
    }

    fn intersects(&self, other: &DRect) -> bool {
        self.left < other.right
            && other.left < self.right
            && self.top < other.bottom
            && other.top < self.bottom
    }

    fn set_bounds(&mut self, curve: &Curve) {
        self.left = curve.points[0].x;
        self.top = curve.points[0].y;
        self.right = self.left;
        self.bottom = self.top;

        for i in 1..4 {
            self.left = self.left.min(curve.points[i].x);
            self.top = self.top.min(curve.points[i].y);
            self.right = self.right.max(curve.points[i].x);
            self.bottom = self.bottom.max(curve.points[i].y);
        }
    }
}

/// Coincident intersection state for span endpoints
struct TCoincident {
    perp_pt: Point,
    perp_t: Scalar,
    match_flag: bool,
}

impl TCoincident {
    fn new() -> Self {
        Self {
            perp_pt: Point::new(Scalar::NAN, Scalar::NAN),
            perp_t: -1.0,
            match_flag: false,
        }
    }

    fn init(&mut self) {
        self.perp_t = -1.0;
        self.match_flag = false;
        self.perp_pt = Point::new(Scalar::NAN, Scalar::NAN);
    }

    fn is_match(&self) -> bool {
        self.match_flag
    }

    fn mark_coincident(&mut self) {
        if !self.match_flag {
            self.perp_t = -1.0;
        }
        self.match_flag = true;
    }

    fn set_perp(&mut self, c1: &Curve, t: Scalar, c_pt: Point, c2: &Curve) {
        // Compute perpendicular from curve point to opposite curve
        let dxdy = c1.dxdy_at_t(t);

        // Create perpendicular line through c_pt
        let perp_start = c_pt;
        let perp_end = Point {
            x: c_pt.x + dxdy.y,
            y: c_pt.y - dxdy.x,
        };

        // Find intersection with opposite curve
        let used = Self::intersect_ray(c2, perp_start, perp_end);

        if used == 0 || used == 3 {
            self.init();
            return;
        }

        self.perp_t = used as Scalar; // Simplified - actual uses intersection index

        let closest_pt = c2.pt_at_t(self.perp_t);
        self.perp_pt = closest_pt;

        // Check if we found a matching point
        self.match_flag = points_approximately_equal(c_pt, self.perp_pt);
    }

    fn intersect_ray(curve: &Curve, ray_start: Point, ray_end: Point) -> i32 {
        // Simplified ray-curve intersection
        // In the full implementation, this finds intersections using Newton's method

        // Check if endpoints approximately match
        if points_approximately_equal(ray_start, curve.point0()) {
            return 1;
        }
        if points_approximately_equal(ray_end, curve.point0()) {
            return 1;
        }
        if points_approximately_equal(ray_start, curve.point3()) {
            return 1;
        }
        if points_approximately_equal(ray_end, curve.point3()) {
            return 1;
        }

        // For now, return 1 if we found a match
        // In the full implementation this would return the actual count
        1
    }
}

/// A bounded span - reference to an intersecting span
struct TSpanBounded {
    bounded: Rc<RefCell<TSpan>>,
    next: Option<Rc<RefCell<TSpanBounded>>>,
}

impl TSpanBounded {
    fn new(bounded: Rc<RefCell<TSpan>>) -> Self {
        Self {
            bounded,
            next: None,
        }
    }
}

/// A span represents a segment of a curve parameterized by t in [0,1]
struct TSpan {
    // Curve parameters
    start_t: Scalar,
    end_t: Scalar,

    // Bounded bounds
    bounds: DRect,

    // Linked list for span ordering
    prev: Option<Rc<RefCell<TSpan>>>,
    next: Option<Rc<RefCell<TSpan>>>,

    // Intersection tracking - spans that intersect this one
    bounded: Option<Rc<RefCell<TSpanBounded>>>,

    // Coincidence tracking
    coin_start: TCoincident,
    coin_end: TCoincident,

    // Span state flags
    is_linear: bool,
    is_line: bool,
    collapsed: bool,
    has_perp: bool,
    deleted: bool,

    // Computed values
    bounds_max: Scalar,
}

impl TSpan {
    fn new(curve: &Curve) -> Self {
        let mut span = Self {
            start_t: 0.0,
            end_t: 1.0,
            bounds: DRect::empty(),
            prev: None,
            next: None,
            bounded: None,
            coin_start: TCoincident::new(),
            coin_end: TCoincident::new(),
            is_linear: false,
            is_line: false,
            collapsed: false,
            has_perp: false,
            deleted: false,
            bounds_max: 0.0,
        };
        span.init_bounds(curve);
        span
    }

    fn init(&mut self, curve: &Curve) {
        self.prev = None;
        self.next = None;
        self.start_t = 0.0;
        self.end_t = 1.0;
        self.bounded = None;
        self.reset_bounds(curve);
    }

    fn init_bounds(&mut self, curve: &Curve) -> bool {
        if self.start_t.is_nan() || self.end_t.is_nan() {
            return false;
        }

        // Subdivide curve to get bounding box
        let sub_curve = curve; // Simplified - actual subdivides by t range
        self.bounds.set_bounds(&sub_curve);
        self.coin_start.init();
        self.coin_end.init();
        self.bounds_max = self.bounds.width().max(self.bounds.height());
        self.collapsed = sub_curve.collapsed();
        self.has_perp = false;
        self.deleted = false;

        self.bounds.valid()
    }

    fn reset_bounds(&mut self, curve: &Curve) {
        self.is_linear = false;
        self.is_line = false;
        self.init_bounds(curve);
    }

    fn point_first(&self) -> Point {
        Point::new(self.start_t, 0.0) // Simplified - actual uses curve evaluation
    }

    fn point_last(&self) -> Point {
        Point::new(self.end_t, 0.0) // Simplified - actual uses curve evaluation
    }

    fn contains(&self, t: Scalar) -> bool {
        between(self.start_t, t, self.end_t)
    }

    fn add_bounded(&mut self, span: Rc<RefCell<TSpan>>, _heap: &ArenaAlloc) {
        // Add span to bounded list
        let bounded = TSpanBounded::new(span);
        let bounded_rc = Rc::new(RefCell::new(bounded));
        if let Some(existing) = self.bounded.take() {
            bounded_rc.borrow_mut().next = Some(existing);
        }
        self.bounded = Some(bounded_rc);
    }

    fn remove_bounded(&mut self, to_remove: &TSpan) -> bool {
        // Remove span from bounded list
        let mut result = false;

        if self.has_perp {
            // Check if we still have perp references
            let mut found_start = false;
            let mut found_end = false;
            let mut current = self.bounded.clone();

            while let Some(curr) = current {
                let span_ref = curr.borrow();
                if span_ref.bounded.borrow().start_t != to_remove.start_t {
                    found_start = found_start
                        || between(
                            span_ref.bounded.borrow().start_t,
                            self.coin_start.perp_t,
                            span_ref.bounded.borrow().end_t,
                        );
                    found_end = found_end
                        || between(
                            span_ref.bounded.borrow().start_t,
                            self.coin_end.perp_t,
                            span_ref.bounded.borrow().end_t,
                        );
                }
                current = span_ref.next.clone();
            }

            if !found_start || !found_end {
                self.has_perp = false;
                self.coin_start.init();
                self.coin_end.init();
            }
        }

        // Remove from linked list
        let mut prev: Option<Rc<RefCell<TSpanBounded>>> = None;
        let mut current = self.bounded.clone();

        while let Some(curr) = current {
            let next = curr.borrow().next.clone();
            if curr.borrow().bounded.borrow().start_t == to_remove.start_t {
                if let Some(p) = prev {
                    p.borrow_mut().next = next;
                    result = false;
                } else {
                    self.bounded = next;
                    result = self.bounded.is_none();
                }
                break;
            }
            prev = Some(curr);
            current = next;
        }

        result
    }

    fn hull_check(&self, _opp: &TSpan) -> i32 {
        // Check hull intersection - returns 0=no, 1=yes, 2=share endpoint, -1=needs more check
        if self.is_linear {
            return -1;
        }

        // Simplified - actual implementation checks convex hull intersection
        1
    }

    fn hulls_intersect(&self, opp: &TSpan) -> i32 {
        // Check if spans' bounding boxes intersect
        if !self.bounds.intersects(&opp.bounds) {
            return 0;
        }

        let hull_sect = self.hull_check(opp);
        if hull_sect >= 0 {
            return hull_sect;
        }

        let opp_hull_sect = opp.hull_check(self);
        if opp_hull_sect >= 0 {
            return opp_hull_sect;
        }

        -1
    }

    fn split_at(&mut self, work: &mut TSpan, t: Scalar) -> bool {
        if t == self.start_t || t == self.end_t {
            self.collapsed = true;
            return false;
        }

        if work.start_t == work.end_t {
            work.collapsed = true;
            return false;
        }

        // Split the span at t
        self.start_t = t;
        work.end_t = t;

        // Reset bounds
        self.bounds.set_bounds(&Curve::new([
            self.point_first(),
            self.point_first(),
            self.point_first(),
            self.point_first(),
        ]));

        true
    }

    fn closest_bounded_t(&self, pt: Point) -> Scalar {
        let mut result = -1.0;
        let mut closest = f32::INFINITY;

        let mut current = self.bounded.clone();
        while let Some(curr) = current {
            let span_ref = curr.borrow();
            let test = span_ref.bounded.borrow();

            let start_dist = distance_squared(test.point_first(), pt);
            if closest > start_dist {
                closest = start_dist;
                result = test.start_t;
            }

            let end_dist = distance_squared(test.point_last(), pt);
            if closest > end_dist {
                closest = end_dist;
                result = test.end_t;
            }

            current = span_ref.next.clone();
        }

        result
    }
}

/// Simple arena allocator for span objects
struct ArenaAlloc {
    data: RefCell<Vec<u8>>,
    offset: RefCell<usize>,
    block_size: usize,
}

impl ArenaAlloc {
    fn new(block_size: usize) -> Self {
        Self {
            data: RefCell::new(vec![0; block_size * 4]),
            offset: RefCell::new(0),
            block_size,
        }
    }

    fn alloc<T>(&self) -> *mut T {
        let mut offset = self.offset.borrow_mut();
        let ptr = self.data.borrow().as_ptr() as usize + *offset;
        *offset += std::mem::size_of::<T>();
        ptr as *mut T
    }

    fn reset(&self) {
        *self.offset.borrow_mut() = 0;
    }
}

/// Main intersection engine for curves
struct TSect {
    curve: Curve,
    heap: ArenaAlloc,
    head: Option<Rc<RefCell<TSpan>>>,
    coincident: Option<Rc<RefCell<TSpan>>>,
    deleted: Option<Rc<RefCell<TSpan>>>,
    active_count: i32,
    removed_start_t: bool,
    removed_end_t: bool,
    hung: bool,
}

impl TSect {
    fn new(curve: Curve) -> Self {
        let mut sect = Self {
            curve,
            heap: ArenaAlloc::new(1024),
            head: None,
            coincident: None,
            deleted: None,
            active_count: 0,
            removed_start_t: false,
            removed_end_t: false,
            hung: false,
        };
        sect.reset_removed_ends();
        sect.head = Some(sect.add_one());

        sect
    }

    fn reset_removed_ends(&mut self) {
        self.removed_start_t = false;
        self.removed_end_t = false;
    }

    fn add_one(&mut self) -> Rc<RefCell<TSpan>> {
        let span;

        if let Some(deleted) = self.deleted.take() {
            span = deleted;
        } else {
            let curve = &self.curve;
            span = Rc::new(RefCell::new(TSpan::new(curve)));
        }

        let mut span_mut = span.borrow_mut();
        span_mut.start_t = 0.0;
        span_mut.end_t = 1.0;
        span_mut.bounded = None;
        span_mut.has_perp = false;
        span_mut.deleted = false;
        drop(span_mut);

        self.active_count += 1;
        span
    }

    fn add_following(&mut self, prior: Option<Rc<RefCell<TSpan>>>) -> Rc<RefCell<TSpan>> {
        let result = self.add_one();
        result.borrow_mut().start_t = prior.as_ref().map_or(0.0, |p| p.borrow().end_t);

        let next = prior
            .as_ref()
            .and_then(|p| p.borrow().next.clone())
            .or_else(|| self.head.clone());

        result.borrow_mut().end_t = next.as_ref().map_or(1.0, |n| n.borrow().start_t);

        if let Some(ref prior) = prior {
            prior.borrow_mut().next = Some(Rc::clone(&result));
        } else {
            self.head = Some(Rc::clone(&result));
        }

        if let Some(ref next) = next {
            next.borrow_mut().prev = Some(Rc::clone(&result));
        }

        result.borrow_mut().prev = prior;
        result.borrow_mut().next = next;

        result.borrow_mut().reset_bounds(&self.curve);
        result
    }

    fn bounds_max(&mut self) -> Option<Rc<RefCell<TSpan>>> {
        let mut test = self.head.clone();
        let mut largest = self.head.clone();
        let mut safety_net = MAX_SPAN_ITERATIONS;

        while let Some(t) = test {
            if safety_net <= 0 {
                self.hung = true;
                return None;
            }
            safety_net -= 1;

            let t_collapsed = t.borrow().collapsed;
            if let Some(ref l) = largest {
                let l_collapsed = l.borrow().collapsed;
                if (l_collapsed && !t_collapsed)
                    || (l_collapsed == t_collapsed && l.borrow().bounds_max < t.borrow().bounds_max)
                {
                    largest = Some(Rc::clone(&t));
                }
            }

            test = t.borrow().next.clone();
        }

        largest
    }

    fn span_at_t(&self, t: Scalar) -> Option<Rc<RefCell<TSpan>>> {
        let mut test = self.head.clone();
        while let Some(span) = test {
            if span.borrow().end_t >= t {
                if span.borrow().start_t <= t {
                    return Some(span);
                }
                break;
            }
            test = span.borrow().next.clone();
        }
        None
    }

    fn tail(&self) -> Option<Rc<RefCell<TSpan>>> {
        let mut result = self.head.clone();
        let mut next = self.head.clone();
        let mut safety_net = MAX_SPAN_ITERATIONS;

        while let Some(n) = next {
            if safety_net <= 0 {
                return None;
            }
            safety_net -= 1;

            if let Some(ref r) = result {
                if n.borrow().end_t > r.borrow().end_t {
                    result = Some(Rc::clone(&n));
                }
            }
            next = n.borrow().next.clone();
        }

        result
    }

    fn remove_span(&mut self, span: Rc<RefCell<TSpan>>) -> bool {
        self.removed_end_check(&span.borrow());

        if !self.unlink_span(span.clone()) {
            return false;
        }

        self.mark_span_gone(span)
    }

    fn unlink_span(&mut self, span: Rc<RefCell<TSpan>>) -> bool {
        let prev = span.borrow().prev.clone();
        let next = span.borrow().next.clone();

        if let Some(ref p) = prev {
            p.borrow_mut().next = next.clone();
            if let Some(ref n) = next {
                n.borrow_mut().prev = prev.clone();
                if n.borrow().start_t > n.borrow().end_t {
                    return false;
                }
            }
        } else {
            self.head = next.clone();
            if let Some(ref n) = next {
                n.borrow_mut().prev = None;
            }
        }

        true
    }

    fn mark_span_gone(&mut self, span: Rc<RefCell<TSpan>>) -> bool {
        self.active_count -= 1;
        if self.active_count < 0 {
            return false;
        }

        span.borrow_mut().next = self.deleted.clone();
        self.deleted = Some(span);
        true
    }

    fn removed_end_check(&mut self, span: &TSpan) {
        if span.start_t == 0.0 {
            self.removed_start_t = true;
        }
        if span.end_t == 1.0 {
            self.removed_end_t = true;
        }
    }

    fn validate(&self) {
        // Debug validation
        #[cfg(debug_assertions)]
        {
            let mut count = 0;
            let mut last = 0.0;
            let mut test = self.head.clone();

            while let Some(t) = test {
                let t_ref = t.borrow();
                assert!(t_ref.start_t >= last);
                last = t_ref.end_t;
                count += 1;
                test = t_ref.next.clone();
            }

            assert_eq!(count, self.active_count as usize);
        }
    }
}

/// Binary search for coincident points between two curves
fn binary_search_coincident(
    sect1: &TSect,
    _sect2: &TSect,
    t_start: Scalar,
    mut t_step: Scalar,
) -> Option<(Scalar, Scalar)> {
    // Simplified implementation - actual uses iterative binary search

    let mut work_start_t = t_start;

    let mut last = sect1.curve.pt_at_t(t_start);
    let mut flip = false;
    let down = t_step < 0.0;

    for _ in 0..50 {
        let mut t_step_half = t_step * 0.5;
        work_start_t += t_step_half;

        if flip {
            t_step_half = -t_step_half;
        }
        let _ = t_step_half;

        // Simplified - actual would check bounds and coincidence
        if points_approximately_equal(last, sect1.curve.pt_at_t(work_start_t)) {
            break;
        }

        last = sect1.curve.pt_at_t(work_start_t);

        // Check condition - simplified for testing
        if down && work_start_t <= work_start_t || !down && work_start_t >= work_start_t {
            return None;
        }

        t_step = -t_step;
        flip = true;
    }

    Some((t_start, t_start))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_between() {
        assert!(between(0.0, 0.5, 1.0));
        assert!(between(0.0, 0.0, 1.0));
        assert!(between(0.0, 1.0, 1.0));
        assert!(!between(0.0, 1.5, 1.0));
    }

    #[test]
    fn test_points_approximately_equal() {
        let a = Point::new(1.0, 2.0);
        let b = Point::new(1.0 + 1e-10, 2.0 + 1e-10);
        let c = Point::new(1.0, 2.0);

        assert!(points_approximately_equal(a, c));
        assert!(points_approximately_equal(a, b));
        assert!(!points_approximately_equal(a, Point::new(2.0, 2.0)));
    }

    #[test]
    fn test_coincident_init() {
        let coin = TCoincident::new();
        assert!(!coin.is_match());
        assert_eq!(coin.perp_t, -1.0);
    }

    #[test]
    fn test_rect_intersects() {
        let r1 = DRect::new(0.0, 0.0, 10.0, 10.0);
        let r2 = DRect::new(5.0, 5.0, 15.0, 15.0);
        let r3 = DRect::new(20.0, 20.0, 30.0, 30.0);

        assert!(r1.intersects(&r2));
        assert!(!r1.intersects(&r3));
    }

    #[test]
    fn test_between_roughly() {
        assert!(roughly_between(0.0, 0.5, 1.0));
        assert!(roughly_between(-0.0000001, 0.0, 1.0));
        // 0.0 is nowhere near the endpoints 1.0000001/1.0 (off by ~1.0,
        // far past any rough tolerance), so this must be rejected.
        assert!(!roughly_between(1.0000001, 0.0, 1.0));
    }

    #[test]
    fn test_precisely_between() {
        assert!(precisely_between(0.0, 0.5, 1.0));
        assert!(precisely_between(0.0, 0.0, 1.0));
        assert!(!precisely_between(0.5, 0.0, 1.0));
    }

    #[test]
    fn test_approximately_zero() {
        assert!(approximately_zero(0.0));
        assert!(approximately_zero(1e-7));
        assert!(!approximately_zero(1e-4));
    }

    #[test]
    fn test_precisely_zero() {
        assert!(precisely_zero(0.0));
        assert!(!precisely_zero(1e-10));
    }

    #[test]
    fn test_distance_squared() {
        let a = Point::new(0.0, 0.0);
        let b = Point::new(3.0, 4.0);
        assert_eq!(distance_squared(a, b), 25.0);
    }

    #[test]
    fn test_curve_pt_at_t() {
        let points = [
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ];
        let curve = Curve::new(points);
        let pt = curve.pt_at_t(0.5);
        // Cubic Bezier at t=0.5 with this control polygon is the weighted
        // sum 0.125*P0 + 0.375*P1 + 0.375*P2 + 0.125*P3, not the polygon's
        // centroid: x = 0.75, y = 0.5.
        assert!((pt.x - 0.75).abs() < 1e-6);
        assert!((pt.y - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_drect_valid() {
        let r = DRect::new(0.0, 0.0, 10.0, 10.0);
        assert!(r.valid());
        assert_eq!(r.width(), 10.0);
        assert_eq!(r.height(), 10.0);
    }

    #[test]
    fn test_arena_alloc() {
        let alloc = ArenaAlloc::new(1024);
        // Just verify we can create one
        assert!(alloc.data.borrow().len() > 0);
    }
}
