//! Private stroking helpers.
//!
//! Ported from `src/core/SkStrokerPriv.cpp`.

use super::scalar::{self, Scalar};
use super::path::Path;
use super::point::{Point, Vector};

/// Callback used to draw line caps at the ends of a contour.
///
/// Corresponds to `SkStrokerPriv::CapProc` in C++.
pub type CapProc = fn(&mut Path, Point, Vector, Point, Option<&mut Path>);

/// Callback used to draw joins between segments.
///
/// Corresponds to `SkStrokerPriv::JoinProc` in C++.
pub type JoinProc =
    fn(&mut Path, &mut Path, Vector, Point, Vector, Scalar, Scalar, bool, bool);

/// Helper to determine if the turn from `before` to `after` is clockwise.
fn is_clockwise(before: Vector, after: Vector) -> bool {
    // cross > 0 means clockwise turn (z-component of 2D cross product)
    before.x * after.y > before.y * after.x
}

/// Classification of angle types for join handling.
#[derive(Debug, PartialEq)]
enum AngleType {
    Nearly180,
    Sharp,
    Shallow,
    NearlyLine,
}

fn dot2_angle_type(dot: Scalar) -> AngleType {
    if dot >= 0.0 {
        // shallow or line
        if scalar::nearly_zero(1.0 - dot, None) {
            AngleType::NearlyLine
        } else {
            AngleType::Shallow
        }
    } else {
        // sharp or 180
        if scalar::nearly_zero(1.0 + dot, None) {
            AngleType::Nearly180
        } else {
            AngleType::Sharp
        }
    }
}

/// Draws a butt cap: just line to the stop point.
fn butt_capper(path: &mut Path, _pivot: Point, _normal: Vector, stop: Point, _other: Option<&mut Path>) {
    path.line_to(stop.x, stop.y);
}

/// Draws a round cap: two conic arcs forming a semicircle.
fn round_capper(path: &mut Path, pivot: Point, normal: Vector, stop: Point, _other: Option<&mut Path>) {
    // Rotate normal 90 degrees clockwise to get the parallel vector
    let parallel = Vector::new(normal.y, -normal.x);
    let projected_center = pivot + parallel;

    path.conic_to(projected_center.x + normal.x, projected_center.y + normal.y,
                  projected_center.x, projected_center.y, scalar::SCALAR_ROOT_2_OVER_2);
    path.conic_to(projected_center.x - normal.x, projected_center.y - normal.y,
                  stop.x, stop.y, scalar::SCALAR_ROOT_2_OVER_2);
}

/// Draws a square cap: extends perpendicular to the path direction.
fn square_capper(path: &mut Path, pivot: Point, normal: Vector, stop: Point, other: Option<&mut Path>) {
    let parallel = Vector::new(normal.y, -normal.x);

    if let Some(other_path) = other {
        if !other_path.points().is_empty() {
            other_path.line_to(pivot.x + normal.x + parallel.x, pivot.y + normal.y + parallel.y);
            other_path.line_to(pivot.x - normal.x + parallel.x, pivot.y - normal.y + parallel.y);
        } else {
            other_path.move_to(pivot.x + normal.x + parallel.x, pivot.y + normal.y + parallel.y);
            other_path.line_to(pivot.x - normal.x + parallel.x, pivot.y - normal.y + parallel.y);
        }
    } else {
        path.line_to(pivot.x + normal.x + parallel.x, pivot.y + normal.y + parallel.y);
        path.line_to(pivot.x - normal.x + parallel.x, pivot.y - normal.y + parallel.y);
        path.line_to(stop.x, stop.y);
    }
}

/// Adds an inner join point to close the inner side of a stroke.
fn handle_inner_join(inner: &mut Path, pivot: Point, after: Vector) {
    inner.line_to(pivot.x, pivot.y);
    inner.line_to(pivot.x - after.x, pivot.y - after.y);
}

/// Blunt (bevel) joiner: connects the outer edges with a straight line.
fn blunt_joiner(outer: &mut Path, inner: &mut Path, before_unit_normal: Vector, pivot: Point, after_unit_normal: Vector,
                radius: Scalar, _inv_miter_limit: Scalar, _prev_is_line: bool, _curr_is_line: bool) {
    let after = after_unit_normal.scale(radius);

    let (outer, inner) = if is_clockwise(before_unit_normal, after_unit_normal) {
        (outer, inner)
    } else {
        (inner, outer)
    };

    outer.line_to(pivot.x + after.x, pivot.y + after.y);
    handle_inner_join(inner, pivot, after);
}

/// Round joiner: connects the outer edges with a circular arc.
fn round_joiner(outer: &mut Path, inner: &mut Path, before_unit_normal: Vector, pivot: Point, after_unit_normal: Vector,
                radius: Scalar, _inv_miter_limit: Scalar, _prev_is_line: bool, _curr_is_line: bool) {
    let dot = Point::dot_product(before_unit_normal, after_unit_normal);
    if dot2_angle_type(dot) == AngleType::NearlyLine {
        return;
    }

    let mut before = before_unit_normal;
    let mut after = after_unit_normal;
    let (outer, inner, dir) = if is_clockwise(before, after) {
        (outer, inner, super::sk_geometry::RotationDirection::Cw)
    } else {
        before = Vector::new(-before.x, -before.y);
        after = Vector::new(-after.x, -after.y);
        (inner, outer, super::sk_geometry::RotationDirection::Ccw)
    };

    let mut matrix = crate::core::Matrix::identity();
    matrix.set_scale(radius, radius);
    matrix.post_translate(pivot.x, pivot.y);

    let mut conics = [super::sk_geometry::Conic::default(); 4];
    let count = super::sk_geometry::build_unit_arc(before, after, dir, &mut conics);
    if count > 0 {
        for conic in &conics[..count] {
            let src = [conic.pts[1], conic.pts[2]];
            let mut pts = [Point::default(); 2];
            matrix.map_points(&mut pts, &src);
            outer.conic_to(pts[0].x, pts[0].y, pts[1].x, pts[1].y, conic.w);
        }
        let after = after.scale(radius);
        handle_inner_join(inner, pivot, after);
    }
}

const K_ONE_OVER_SQRT2: Scalar = 0.707106781;

/// Miter joiner: extends segments to meet at a miter point or falls back to blunt.
fn miter_joiner(outer: &mut Path, inner: &mut Path, before_unit_normal: Vector, pivot: Point, after_unit_normal: Vector,
                radius: Scalar, inv_miter_limit: Scalar, prev_is_line: bool, mut curr_is_line: bool) {
    let dot = Point::dot_product(before_unit_normal, after_unit_normal);
    let angle_type = dot2_angle_type(dot);

    if angle_type == AngleType::NearlyLine {
        return;
    }

    if angle_type == AngleType::Nearly180 {
        curr_is_line = false;
        blunt_joiner(outer, inner, before_unit_normal, pivot, after_unit_normal, radius, inv_miter_limit, prev_is_line, curr_is_line);
        return;
    }

    let ccw = !is_clockwise(before_unit_normal, after_unit_normal);
    let mut before = before_unit_normal;
    let mut after = after_unit_normal;
    let mut mid = Vector::new(0.0, 0.0);

    // For a counter-clockwise turn, "outer" and "inner" swap roles: the
    // path that receives the miter point is whichever one is on the
    // outside of the turn.
    let (outer, inner) = if ccw {
        before.x = -before.x;
        before.y = -before.y;
        after.x = -after.x;
        after.y = -after.y;
        (inner, outer)
    } else {
        (outer, inner)
    };

    // Fast path for right angles
    if dot == 0.0 && inv_miter_limit <= K_ONE_OVER_SQRT2 {
        mid.x = (before.x + after.x) * radius;
        mid.y = (before.y + after.y) * radius;
        outer.line_to(pivot.x + mid.x, pivot.y + mid.y);
        after = after.scale(radius);
        if !curr_is_line {
            outer.line_to(pivot.x + after.x, pivot.y + after.y);
        }
        handle_inner_join(inner, pivot, after);
        return;
    }

    let sin_half_angle = (0.5 * (1.0 + dot)).sqrt();
    if sin_half_angle < inv_miter_limit {
        curr_is_line = false;
        blunt_joiner(outer, inner, before_unit_normal, pivot, after_unit_normal, radius, inv_miter_limit, prev_is_line, curr_is_line);
        return;
    }

    // Form mid-vector
    if angle_type == AngleType::Sharp {
        mid.x = after.y - before.y;
        mid.y = before.x - after.x;
        if ccw {
            mid.x = -mid.x;
            mid.y = -mid.y;
        }
    } else {
        mid.x = before.x + after.x;
        mid.y = before.y + after.y;
    }

    let mid_len = Point::distance_to_origin(mid.x, mid.y);
    if mid_len > 1e-6 {
        let scale = radius / sin_half_angle / mid_len;
        mid.x *= scale;
        mid.y *= scale;
    }

    outer.line_to(pivot.x + mid.x, pivot.y + mid.y);
    after = after.scale(radius);
    if !curr_is_line {
        outer.line_to(pivot.x + after.x, pivot.y + after.y);
    }
    handle_inner_join(inner, pivot, after);
}

/// Returns the cap factory function for the given cap style.
pub fn cap_factory(cap: super::stroke::Cap) -> CapProc {
    match cap {
        super::stroke::Cap::Butt => butt_capper,
        super::stroke::Cap::Round => round_capper,
        super::stroke::Cap::Square => square_capper,
    }
}

/// Returns the join factory function for the given join style.
pub fn join_factory(join: super::stroke::Join) -> JoinProc {
    match join {
        super::stroke::Join::Miter => miter_joiner,
        super::stroke::Join::Round => round_joiner,
        super::stroke::Join::Bevel => blunt_joiner,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::stroke::Cap;
    use crate::core::stroke::Join;

    #[test]
    fn test_is_clockwise() {
        assert!(is_clockwise(Vector::new(1.0, 0.0), Vector::new(0.0, 1.0)));
        assert!(!is_clockwise(Vector::new(0.0, 1.0), Vector::new(1.0, 0.0)));
    }

    #[test]
    fn test_dot2_angle_type() {
        assert!(matches!(dot2_angle_type(0.9999), AngleType::NearlyLine));
        assert!(matches!(dot2_angle_type(0.5), AngleType::Shallow));
        assert!(matches!(dot2_angle_type(-0.5), AngleType::Sharp));
        assert!(matches!(dot2_angle_type(-0.9999), AngleType::Nearly180));
    }

    #[test]
    fn test_cap_factory() {
        let butt = cap_factory(Cap::Butt);
        let round = cap_factory(Cap::Round);
        let square = cap_factory(Cap::Square);
        assert!((butt as usize) != (round as usize));
        assert!((round as usize) != (square as usize));
    }

    #[test]
    fn test_join_factory() {
        let miter = join_factory(Join::Miter);
        let round = join_factory(Join::Round);
        let bevel = join_factory(Join::Bevel);
        assert!((miter as usize) != (round as usize));
        assert!((round as usize) != (bevel as usize));
    }

    #[test]
    fn test_butt_capper() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 10.0);

        let stop = Point::new(10.0, 10.0);
        let normal = Vector::new(0.0, 1.0);
        let pivot = Point::new(10.0, 10.0);

        butt_capper(&mut path, pivot, normal, stop, None);

        assert_eq!(path.points().len(), 3);
    }

    #[test]
    fn test_square_capper() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 10.0);

        let stop = Point::new(10.0, 10.0);
        let normal = Vector::new(0.0, 1.0);
        let pivot = Point::new(10.0, 10.0);

        square_capper(&mut path, pivot, normal, stop, None);

        assert_eq!(path.points().len(), 5);
    }

    #[test]
    fn test_butt_capper_with_other_path() {
        let mut path = Path::new();
        let mut other = Path::new();
        path.move_to(0.0, 0.0);

        let stop = Point::new(10.0, 10.0);
        let normal = Vector::new(0.0, 1.0);
        let pivot = Point::new(10.0, 10.0);

        butt_capper(&mut path, pivot, normal, stop, Some(&mut other));

        assert_eq!(path.points().len(), 2);
    }

    #[test]
    fn test_square_capper_with_other_path() {
        let mut path = Path::new();
        let mut other = Path::new();
        path.move_to(0.0, 0.0);

        let stop = Point::new(10.0, 10.0);
        let normal = Vector::new(0.0, 1.0);
        let pivot = Point::new(10.0, 10.0);

        square_capper(&mut path, pivot, normal, stop, Some(&mut other));

        assert_eq!(other.points().len(), 2);
        assert_eq!(path.points().len(), 1);
    }

    #[test]
    fn test_blunt_joiner() {
        let mut outer = Path::new();
        let mut inner = Path::new();

        let before = Vector::new(-1.0, 0.0);
        let after = Vector::new(0.0, -1.0);
        let pivot = Point::new(10.0, 10.0);

        blunt_joiner(&mut outer, &mut inner, before, pivot, after, 5.0, 4.0, true, true);

        assert!(outer.points().len() > 0);
        assert!(inner.points().len() > 0);
    }

    #[test]
    fn test_miter_joiner_basic() {
        let mut outer = Path::new();
        let mut inner = Path::new();

        let before = Vector::new(-1.0, 0.0);
        let after = Vector::new(0.0, -1.0);
        let pivot = Point::new(10.0, 10.0);

        miter_joiner(&mut outer, &mut inner, before, pivot, after, 5.0, 4.0, true, true);

        assert!(!outer.points().is_empty());
        assert!(!inner.points().is_empty());
    }
}
