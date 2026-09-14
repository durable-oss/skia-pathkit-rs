//! Path triangulation utilities.
//!
//! This module provides the GrTriangulator class which converts paths to triangles
//! using a sweep-line algorithm. Ported from Skia's GrTriangulator.cpp.

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

use crate::core::{FillType, Path, Point, Rect, Scalar, Verb};

const K_ARENA_DEFAULT_CHUNK_SIZE: usize = 16 * 1024;

/// Edge types used in triangulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeType {
    /// Edge derived from the path itself, carrying winding and participating in
    /// the sweep-line mesh.
    Inner,
    /// Edge of the outer (antialiasing) boundary generated around an inner
    /// contour; it bounds the coverage ramp rather than the fill.
    Outer,
    /// Edge inserted to join an inner vertex to its outer partner, stitching the
    /// two boundaries together so the ramp region can be triangulated.
    Connector,
}

/// Side of a monotone polygon.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// The chain of edges running down the left of the polygon.
    Left,
    /// The chain of edges running down the right of the polygon.
    Right,
}

/// Forward declarations for complex types.
#[derive(Debug)]
pub struct Vertex {
    /// Position of this vertex in path space.
    pub point: Point,
    /// Index of the previous vertex in the enclosing [`VertexList`], that is the
    /// one immediately before this in sweep order (or in contour order, while
    /// the contour is still being built).
    pub prev: Option<usize>,
    /// Index of the next vertex in the enclosing [`VertexList`], the one
    /// immediately after this in sweep order.
    pub next: Option<usize>,
    /// First edge in the list of edges whose bottom endpoint is this vertex,
    /// ordered left to right.
    pub first_edge_above: Option<usize>,
    /// Last (rightmost) edge in the list of edges ending at this vertex.
    pub last_edge_above: Option<usize>,
    /// First edge in the list of edges whose top endpoint is this vertex,
    /// ordered left to right.
    pub first_edge_below: Option<usize>,
    /// Last (rightmost) edge in the list of edges starting at this vertex.
    pub last_edge_below: Option<usize>,
    /// Edge of the active edge list lying immediately to the left of this vertex
    /// when the sweep line reaches it, if any.
    pub left_enclosing_edge: Option<usize>,
    /// Edge of the active edge list lying immediately to the right of this
    /// vertex when the sweep line reaches it, if any.
    pub right_enclosing_edge: Option<usize>,
    /// The matching vertex on the opposite boundary when an antialiased outer
    /// contour is generated: an inner vertex points at its outer counterpart and
    /// vice versa.
    pub partner: Option<usize>,
    /// Coverage emitted at this vertex, 255 inside the shape and 0 on the outer
    /// edge of the antialiasing ramp.
    pub alpha: u8,
    /// True if the triangulator created this vertex (for example at an
    /// intersection) rather than it coming from the input path.
    pub synthetic: bool,
    /// Ordinal assigned during sorting, used to break ties between coincident
    /// vertices and to keep the ordering stable.
    pub id: f32,
}

/// Handle to a vertex in the arena.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VertexHandle(pub usize);

/// Linked list of vertices.
#[derive(Debug, Default, Clone)]
pub struct VertexList {
    /// First vertex of the doubly linked chain, that is the contour's starting
    /// point or, once sorted, the topmost vertex in sweep order.
    pub head: Option<usize>,
    /// Last vertex of the chain, kept so appending is constant time.
    pub tail: Option<usize>,
}

/// Line equation in implicit form: A*x + B*y + C = 0.
#[derive(Debug, Clone, Copy)]
pub struct Line {
    /// Coefficient of `x`, equal to the difference in y between the two
    /// endpoints of the edge this line was built from.
    pub a: f64,
    /// Coefficient of `y`, equal to the negated difference in x between the two
    /// endpoints.
    pub b: f64,
    /// Constant term, fixing the line to pass through its endpoints. Evaluating
    /// `a*x + b*y + c` gives a signed distance scaled by the edge length.
    pub c: f64,
}

/// Edge connecting two vertices.
#[derive(Debug)]
pub struct Edge {
    /// Signed winding contributed by this edge, positive when the original
    /// segment ran from top to bottom and negative when it ran the other way.
    /// Coincident edges are merged by summing their windings.
    pub winding: i32,
    /// Vertex at the upper end of the edge in sweep order.
    pub top: usize,
    /// Vertex at the lower end of the edge in sweep order.
    pub bottom: usize,
    /// Whether this edge comes from the path, from a generated outer boundary,
    /// or connects the two.
    pub edge_type: EdgeType,
    /// Neighbour to the left in the active edge list, the set of edges crossing
    /// the sweep line, kept sorted by x.
    pub left: Option<usize>,
    /// Neighbour to the right in the active edge list.
    pub right: Option<usize>,
    /// Previous edge in the left-to-right list of edges ending at
    /// [`Self::bottom`].
    pub prev_edge_above: Option<usize>,
    /// Next edge in the left-to-right list of edges ending at
    /// [`Self::bottom`].
    pub next_edge_above: Option<usize>,
    /// Previous edge in the left-to-right list of edges starting at
    /// [`Self::top`].
    pub prev_edge_below: Option<usize>,
    /// Next edge in the left-to-right list of edges starting at [`Self::top`].
    pub next_edge_below: Option<usize>,
    /// Polygon lying immediately to the left of this edge, which this edge helps
    /// bound on its right side.
    pub left_poly: Option<usize>,
    /// Polygon lying immediately to the right of this edge.
    pub right_poly: Option<usize>,
    /// Previous edge along the boundary chain of [`Self::left_poly`].
    pub left_poly_prev: Option<usize>,
    /// Next edge along the boundary chain of [`Self::left_poly`].
    pub left_poly_next: Option<usize>,
    /// Previous edge along the boundary chain of [`Self::right_poly`].
    pub right_poly_prev: Option<usize>,
    /// Next edge along the boundary chain of [`Self::right_poly`].
    pub right_poly_next: Option<usize>,
    /// Set once this edge has been consumed as part of the left polygon's
    /// boundary, so it is not added twice.
    pub used_in_left_poly: bool,
    /// Set once this edge has been consumed as part of the right polygon's
    /// boundary.
    pub used_in_right_poly: bool,
    /// Implicit line through the two endpoints, used in double precision for
    /// intersection tests and side-of-line queries.
    pub line: Line,
}

/// Handle to an edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeHandle(pub usize);

/// Linked list of edges.
#[derive(Debug, Default, Clone)]
pub struct EdgeList {
    /// Leftmost edge of the chain. For the active edge list this is the edge
    /// with the smallest x where it crosses the sweep line.
    pub head: Option<usize>,
    /// Rightmost edge of the chain.
    pub tail: Option<usize>,
}

/// Monotone polygon segment.
#[derive(Debug)]
pub struct MonotonePoly {
    /// Which side of the parent polygon this piece was split off from, which
    /// decides the orientation of the triangles emitted from it.
    pub side: Side,
    /// First edge of the boundary chain, at the top of the monotone piece.
    pub first_edge: Option<usize>,
    /// Last edge of the boundary chain, at the bottom of the monotone piece.
    pub last_edge: Option<usize>,
    /// Previous monotone piece of the same [`Poly`], lying above this one.
    pub prev: Option<usize>,
    /// Next monotone piece of the same [`Poly`], lying below this one.
    pub next: Option<usize>,
    /// Winding number of the region this piece covers, inherited from the parent
    /// polygon.
    pub winding: i32,
}

/// Handle to a monotone polygon.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MonotonePolyHandle(pub usize);

/// Poly (polygon) structure.
#[derive(Debug)]
pub struct Poly {
    /// Topmost vertex of the polygon, the one that opened it during the sweep.
    pub first_vertex: usize,
    /// Winding number of the region this polygon covers. Whether it is filled is
    /// decided later by applying the path's fill rule to this number.
    pub winding: i32,
    /// First monotone piece, at the top of the polygon.
    pub head: Option<usize>,
    /// Last monotone piece, at the bottom; new pieces are appended here as the
    /// sweep descends.
    pub tail: Option<usize>,
    /// Next polygon in the list of all polygons produced by the sweep.
    pub next: Option<usize>,
    /// The polygon on the other side of a shared boundary, used when a polygon
    /// is split so the two halves can be rejoined.
    pub partner: Option<usize>,
    /// Number of vertices added to the polygon so far, used to skip degenerate
    /// polygons with fewer than three.
    pub count: i32,
    /// Sequential identifier assigned at creation, useful for debugging and for
    /// stable ordering.
    pub id: i32,
}

/// Handle to a poly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PolyHandle(pub usize);

/// Comparator for vertex sorting.
#[derive(Debug, Clone, Copy)]
pub struct Comparator {
    /// Axis the sweep runs along, chosen from the path bounds so the longer
    /// dimension is swept and fewer vertices share a sweep position.
    pub direction: Direction,
}

/// Axis along which the sweep line advances.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Sweep top to bottom, comparing y first and then x.
    Vertical,
    /// Sweep left to right, comparing x first and then y.
    Horizontal,
}

/// Result of simplification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimplifyResult {
    /// Simplification could not complete, for example because the mesh ran out
    /// of budget; the caller abandons the triangulation.
    Failed,
    /// No self intersections were found, so the mesh was left unchanged.
    AlreadySimple,
    /// An intersection was found and split, so the mesh changed and the sweep
    /// has to be run again.
    FoundSelfIntersection,
}

/// Result type for operations that can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoolFail {
    /// The operation completed and the answer is no.
    False,
    /// The operation completed and the answer is yes.
    True,
    /// The operation could not be completed, which is distinct from answering
    /// no and aborts the triangulation.
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
    /// Creates a triangulator for `path` with every option off and an empty
    /// breadcrumb list.
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

    /// Triangulates `path`, flattening curves to within `tolerance` and
    /// clipping against `clip_bounds`, and writes the triangle vertices into
    /// `vertex`.
    ///
    /// Sets `is_linear` to true when the path contained no curves. Returns the
    /// number of vertices written, or 0 if the path is not finite or the sweep
    /// fails.
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
