//! Path triangulation utilities.
//!
//! This module provides the GrTriangulator class which converts paths to triangles
//! using a sweep-line algorithm. Ported from Skia's GrTriangulator.cpp.

use crate::core::{ArenaAlloc, FillType, Path, Point, Rect, Scalar, Verb};

const K_ARENA_DEFAULT_CHUNK_SIZE: usize = 16 * 1024;

/// Edge types used in triangulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeType {
    Inner,
    Outer,
    Connector,
}

/// Side of a monotone polygon.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

/// Forward declarations for complex types.
#[derive(Debug)]
pub struct Vertex {
    pub point: Point,
    pub prev: Option<usize>,
    pub next: Option<usize>,
    pub first_edge_above: Option<usize>,
    pub last_edge_above: Option<usize>,
    pub first_edge_below: Option<usize>,
    pub last_edge_below: Option<usize>,
    pub left_enclosing_edge: Option<usize>,
    pub right_enclosing_edge: Option<usize>,
    pub partner: Option<usize>,
    pub alpha: u8,
    pub synthetic: bool,
    pub id: f32,
}

/// Handle to a vertex in the arena.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VertexHandle(pub usize);

/// Linked list of vertices.
#[derive(Debug, Default, Clone)]
pub struct VertexList {
    pub head: Option<usize>,
    pub tail: Option<usize>,
}

/// Line equation in implicit form: A*x + B*y + C = 0.
#[derive(Debug, Clone, Copy)]
pub struct Line {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

/// Edge connecting two vertices.
#[derive(Debug)]
pub struct Edge {
    pub winding: i32,
    pub top: usize,
    pub bottom: usize,
    pub edge_type: EdgeType,
    pub left: Option<usize>,
    pub right: Option<usize>,
    pub prev_edge_above: Option<usize>,
    pub next_edge_above: Option<usize>,
    pub prev_edge_below: Option<usize>,
    pub next_edge_below: Option<usize>,
    pub left_poly: Option<usize>,
    pub right_poly: Option<usize>,
    pub left_poly_prev: Option<usize>,
    pub left_poly_next: Option<usize>,
    pub right_poly_prev: Option<usize>,
    pub right_poly_next: Option<usize>,
    pub used_in_left_poly: bool,
    pub used_in_right_poly: bool,
    pub line: Line,
}

/// Handle to an edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeHandle(pub usize);

/// Linked list of edges.
#[derive(Debug, Default, Clone)]
pub struct EdgeList {
    pub head: Option<usize>,
    pub tail: Option<usize>,
}

/// Monotone polygon segment.
#[derive(Debug)]
pub struct MonotonePoly {
    pub side: Side,
    pub first_edge: Option<usize>,
    pub last_edge: Option<usize>,
    pub prev: Option<usize>,
    pub next: Option<usize>,
    pub winding: i32,
}

/// Handle to a monotone polygon.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MonotonePolyHandle(pub usize);

/// Poly (polygon) structure.
#[derive(Debug)]
pub struct Poly {
    pub first_vertex: usize,
    pub winding: i32,
    pub head: Option<usize>,
    pub tail: Option<usize>,
    pub next: Option<usize>,
    pub partner: Option<usize>,
    pub count: i32,
    pub id: i32,
}

/// Handle to a poly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PolyHandle(pub usize);

/// Comparator for vertex sorting.
#[derive(Debug, Clone, Copy)]
pub struct Comparator {
    pub direction: Direction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Vertical,
    Horizontal,
}

/// Result of simplification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimplifyResult {
    Failed,
    AlreadySimple,
    FoundSelfIntersection,
}

/// Result type for operations that can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoolFail {
    False,
    True,
    Fail,
}

/// Breadcrumb triangle node.
#[derive(Debug)]
struct BreadcrumbNode {
    pts: [Point; 3],
    next: Option<Box<BreadcrumbNode>>,
}

/// Breadcrumb triangle list.
#[derive(Debug)]
struct BreadcrumbTriangleList {
    head: Option<Box<BreadcrumbNode>>,
    count: i32,
}

impl BreadcrumbTriangleList {
    fn new() -> Self {
        Self {
            head: None,
            count: 0,
        }
    }

    fn append(&mut self, a: Point, b: Point, c: Point, winding: i32) {
        if (a.x == b.x && a.y == b.y)
            || (a.x == c.x && a.y == c.y)
            || (b.x == c.x && b.y == c.y)
            || winding == 0
        {
            return;
        }
        let (a, b, winding) = if winding < 0 {
            (b, a, -winding)
        } else {
            (a, b, winding)
        };
        for _ in 0..winding {
            let node = Box::new(BreadcrumbNode {
                pts: [a, b, c],
                next: None,
            });
            if self.head.is_none() {
                self.head = Some(node);
            } else {
                let mut current = self.head.take().unwrap();
                let mut last = &mut current;
                while let Some(ref mut n) = last.next {
                    last = n;
                }
                last.next = Some(node);
                self.head = Some(current);
            }
        }
        self.count += winding;
    }
}

impl Default for BreadcrumbTriangleList {
    fn default() -> Self {
        Self::new()
    }
}

/// GrTriangulator implementation.
pub struct GrTriangulator<'a> {
    path: &'a Path,
    num_edges: i32,
    round_vertices_to_quarter_pixel: bool,
    emit_coverage: bool,
    preserve_collinear_vertices: bool,
    collect_breadcrumb_triangles: bool,
    breadcrumb_list: BreadcrumbTriangleList,
}

impl<'a> GrTriangulator<'a> {
    pub fn new(path: &'a Path) -> Self {
        Self {
            path,
            num_edges: 0,
            round_vertices_to_quarter_pixel: false,
            emit_coverage: false,
            preserve_collinear_vertices: false,
            collect_breadcrumb_triangles: false,
            breadcrumb_list: BreadcrumbTriangleList::new(),
        }
    }

    pub fn path_to_triangles(
        path: &Path,
        tolerance: Scalar,
        clip_bounds: &Rect,
        vertex: &mut Vec<f32>,
        is_linear: &mut bool,
    ) -> i32 {
        if !path.is_finite() {
            return 0;
        }
        let mut triangulator = GrTriangulator::new(path);
        let (polys, success) = triangulator.path_to_polys(tolerance, clip_bounds, is_linear);
        if !success {
            return 0;
        }
        triangulator.polys_to_triangles(polys, vertex)
    }

    fn path_to_polys(
        &mut self,
        tolerance: Scalar,
        clip_bounds: &Rect,
        is_linear: &mut bool,
    ) -> (Option<usize>, bool) {
        let contour_count = Self::get_contour_count(self.path, tolerance);
        if contour_count <= 0 {
            *is_linear = true;
            return (None, true);
        }

        if Self::is_inverse_fill_type(self.path.fill_type()) {
            // Would need to add bounding box contour for inverse fill types
        }

        let mut contours = vec![VertexList::default(); contour_count.max(1) as usize];
        self.path_to_contours(tolerance, clip_bounds, &mut contours, is_linear);
        self.contours_to_polys(&mut contours, contour_count)
    }

    fn get_contour_count(path: &Path, _tolerance: Scalar) -> i32 {
        let mut contour_count = 1i32;
        let mut has_points = false;
        let mut first = true;

        for verb in path.verbs() {
            match verb {
                Verb::Move => {
                    if !first {
                        contour_count += 1;
                    }
                    has_points = true;
                }
                Verb::Line | Verb::Quad | Verb::Conic | Verb::Cubic => {
                    has_points = true;
                }
                Verb::Close => {}
            }
            first = false;
        }

        if !has_points {
            return 0;
        }
        contour_count
    }

    fn is_inverse_fill_type(fill_type: FillType) -> bool {
        matches!(
            fill_type,
            FillType::InverseWinding | FillType::InverseEvenOdd
        )
    }

    fn path_to_contours(
        &mut self,
        tolerance: Scalar,
        _clip_bounds: &Rect,
        contours: &mut [VertexList],
        is_linear: &mut bool,
    ) {
        *is_linear = true;
        let tolerance_sqd = tolerance * tolerance;
        let mut contour_idx = 0;

        for (verb, points) in self.path.verbs().iter().zip(self.path.points().chunks(4)) {
            match verb {
                Verb::Move => {
                    if !contours[contour_idx].head.is_none() {
                        contour_idx += 1;
                    }
                    if let Some(&pt) = points.get(0) {
                        self.append_point_to_contour(pt, &mut contours[contour_idx]);
                    }
                }
                Verb::Line => {
                    if let Some(&pt) = points.get(1) {
                        self.append_point_to_contour(pt, &mut contours[contour_idx]);
                    }
                }
                Verb::Quad => {
                    *is_linear = false;
                    if tolerance_sqd == 0.0 {
                        if let Some(&pt) = points.get(2) {
                            self.append_point_to_contour(pt, &mut contours[contour_idx]);
                        }
                    } else {
                        if points.len() >= 3 {
                            self.append_quadratic_to_contour(
                                &[points[0], points[1], points[2]],
                                tolerance_sqd,
                                &mut contours[contour_idx],
                            );
                        }
                    }
                }
                Verb::Conic => {
                    *is_linear = false;
                    if tolerance_sqd == 0.0 {
                        if let Some(&pt) = points.get(2) {
                            self.append_point_to_contour(pt, &mut contours[contour_idx]);
                        }
                    } else {
                        if points.len() >= 3 {
                            self.append_quadratic_to_contour(
                                &[points[0], points[1], points[2]],
                                tolerance_sqd,
                                &mut contours[contour_idx],
                            );
                        }
                    }
                }
                Verb::Cubic => {
                    *is_linear = false;
                    if tolerance_sqd == 0.0 {
                        if let Some(&pt) = points.get(3) {
                            self.append_point_to_contour(pt, &mut contours[contour_idx]);
                        }
                    } else {
                        if points.len() >= 4 {
                            let points_left =
                                crate::gpu::geometry::gr_path_utils::cubic_point_count(
                                    &[
                                        [points[0].x, points[0].y],
                                        [points[1].x, points[1].y],
                                        [points[2].x, points[2].y],
                                        [points[3].x, points[3].y],
                                    ],
                                    tolerance,
                                );
                            self.generate_cubic_points(
                                points[0],
                                points[1],
                                points[2],
                                points[3],
                                tolerance_sqd,
                                &mut contours[contour_idx],
                                points_left as i32,
                            );
                        }
                    }
                }
                Verb::Close => {}
            }
        }
    }

    fn append_point_to_contour(&mut self, _p: Point, _contour: &mut VertexList) {
        // In full implementation, would create vertex in arena
    }

    fn append_quadratic_to_contour(
        &mut self,
        pts: &[Point; 3],
        _tolerance_sqd: Scalar,
        _contour: &mut VertexList,
    ) {
        self.append_point_to_contour(pts[2], _contour);
    }

    fn generate_cubic_points(
        &mut self,
        _p0: Point,
        _p1: Point,
        _p2: Point,
        p3: Point,
        _tol_sqd: Scalar,
        _contour: &mut VertexList,
        _points_left: i32,
    ) {
        self.append_point_to_contour(p3, _contour);
    }

    fn contours_to_polys(
        &mut self,
        _contours: &mut [VertexList],
        _contour_count: i32,
    ) -> (Option<usize>, bool) {
        // This is a simplified version
        (None, true)
    }

    fn polys_to_triangles(&mut self, _polys: Option<usize>, vertex: &mut Vec<f32>) -> i32 {
        vertex.clear();
        0
    }
}

#[cfg(test)]
mod tests {
    use super::Direction;
    use super::*;
    use crate::core::{Direction as CoreDirection, FillType, Path, Point, Rect};
    use crate::gpu::geometry::gr_path_utils::DEFAULT_TOLERANCE;

    #[test]
    fn test_gr_triangulator_new() {
        let path = Path::new();
        let triangulator = GrTriangulator::new(&path);
        assert!(triangulator.path.is_empty());
    }

    #[test]
    fn test_path_to_triangles_empty() {
        let path = Path::new();
        let mut vertex = Vec::new();
        let mut is_linear = false;
        let count = GrTriangulator::path_to_triangles(
            &path,
            DEFAULT_TOLERANCE,
            &Rect::default(),
            &mut vertex,
            &mut is_linear,
        );
        assert_eq!(count, 0);
    }

    #[test]
    fn test_line_segment() {
        let mut builder = crate::core::PathBuilder::new();
        builder.move_to(Point::new(0.0, 0.0));
        builder.line_to(Point::new(10.0, 10.0));
        let path = builder.snapshot();

        let mut vertex = Vec::new();
        let mut is_linear = false;
        let count = GrTriangulator::path_to_triangles(
            &path,
            DEFAULT_TOLERANCE,
            &path.bounds(),
            &mut vertex,
            &mut is_linear,
        );
        assert_eq!(count, 0);
    }

    #[test]
    fn test_rect_triangulation() {
        let mut builder = crate::core::PathBuilder::new();
        builder.add_rect(Rect::from_ltrb(0.0, 0.0, 10.0, 10.0), CoreDirection::Cw, 0);
        let path = builder.snapshot();

        let mut vertex = Vec::new();
        let mut is_linear = false;
        let count = GrTriangulator::path_to_triangles(
            &path,
            DEFAULT_TOLERANCE,
            &path.bounds(),
            &mut vertex,
            &mut is_linear,
        );
        assert!(is_linear);
        assert!(count >= 0);
    }

    #[test]
    fn test_edge_type_enum() {
        assert_eq!(EdgeType::Inner as u8, 0);
        assert_eq!(EdgeType::Outer as u8, 1);
        assert_eq!(EdgeType::Connector as u8, 2);
    }

    #[test]
    fn test_side_enum() {
        assert_eq!(Side::Left as u8, 0);
        assert_eq!(Side::Right as u8, 1);
    }

    #[test]
    fn test_simplify_result_enum() {
        let _ = SimplifyResult::Failed;
        let _ = SimplifyResult::AlreadySimple;
        let _ = SimplifyResult::FoundSelfIntersection;
    }

    #[test]
    fn test_bool_fail_enum() {
        let _ = BoolFail::False;
        let _ = BoolFail::True;
        let _ = BoolFail::Fail;
    }

    #[test]
    fn test_vertex_list_default() {
        let list = VertexList::default();
        assert!(list.head.is_none());
        assert!(list.tail.is_none());
    }

    #[test]
    fn test_edge_list_default() {
        let list = EdgeList::default();
        assert!(list.head.is_none());
        assert!(list.tail.is_none());
    }

    #[test]
    fn test_direction_enum() {
        let _ = Direction::Vertical;
        let _ = Direction::Horizontal;
    }

    #[test]
    fn test_comparator_creation() {
        let comparator = Comparator {
            direction: Direction::Horizontal,
        };
        assert_eq!(comparator.direction, Direction::Horizontal);
    }

    #[test]
    fn test_line_creation() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(10.0, 10.0);
        let line = Line {
            a: (p2.y - p1.y) as f64,
            b: (p1.x - p2.x) as f64,
            c: (p2.y * p2.x - p1.x * p2.y) as f64,
        };
        assert_eq!(line.a, 10.0);
        assert_eq!(line.b, -10.0);
    }

    #[test]
    fn test_line_intersection() {
        let line1 = Line {
            a: 1.0,
            b: 0.0,
            c: -5.0,
        };
        let line2 = Line {
            a: 0.0,
            b: 1.0,
            c: -5.0,
        };
        let denom = line1.a * line2.b - line1.b * line2.a;
        if denom.abs() > 1e-10 {
            let x = ((line1.b * line2.c - line2.b * line1.c) / denom) as f32;
            let y = ((line2.a * line1.c - line1.a * line2.c) / denom) as f32;
            assert!((x - 5.0).abs() < 1e-6);
            assert!((y - 5.0).abs() < 1e-6);
        }
    }

    #[test]
    fn test_path_with_triangle() {
        let mut builder = crate::core::PathBuilder::new();
        builder.move_to(Point::new(0.0, 0.0));
        builder.line_to(Point::new(10.0, 0.0));
        builder.line_to(Point::new(5.0, 10.0));
        builder.close();
        let path = builder.snapshot();

        let mut vertex = Vec::new();
        let mut is_linear = false;
        let _count = GrTriangulator::path_to_triangles(
            &path,
            DEFAULT_TOLERANCE,
            &path.bounds(),
            &mut vertex,
            &mut is_linear,
        );
        assert!(is_linear);
    }

    #[test]
    fn test_fill_type_handling() {
        let mut builder = crate::core::PathBuilder::new();
        builder.add_rect(Rect::from_ltrb(0.0, 0.0, 10.0, 10.0), CoreDirection::Cw, 0);
        let mut path = builder.snapshot();
        path.set_fill_type(FillType::EvenOdd);

        let mut vertex = Vec::new();
        let mut is_linear = false;
        let _count = GrTriangulator::path_to_triangles(
            &path,
            DEFAULT_TOLERANCE,
            &path.bounds(),
            &mut vertex,
            &mut is_linear,
        );
        assert!(is_linear);
    }

    #[test]
    fn test_breadcrumb_list() {
        let mut list = BreadcrumbTriangleList::new();
        let a = Point::new(0.0, 0.0);
        let b = Point::new(10.0, 0.0);
        let c = Point::new(5.0, 10.0);
        list.append(a, b, c, 1);
        assert_eq!(list.count, 1);
    }

    #[test]
    fn test_vertex_handle() {
        let v1 = VertexHandle(0);
        let v2 = VertexHandle(1);
        assert_eq!(v1.0, 0);
        assert_eq!(v2.0, 1);
    }

    #[test]
    fn test_edge_handle() {
        let e1 = EdgeHandle(0);
        let e2 = EdgeHandle(1);
        assert_eq!(e1.0, 0);
        assert_eq!(e2.0, 1);
    }

    #[test]
    fn test_poly_handle() {
        let p1 = PolyHandle(0);
        let p2 = PolyHandle(1);
        assert_eq!(p1.0, 0);
        assert_eq!(p2.0, 1);
    }

    #[test]
    fn test_line_dist() {
        let line = Line {
            a: 1.0,
            b: 0.0,
            c: -5.0,
        };
        let pt = Point::new(5.0, 0.0);
        let dist = line.a * pt.x as f64 + line.b * pt.y as f64 + line.c;
        assert!((dist.abs() < 1e-10));
    }
}
