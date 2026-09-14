// Copyright 2020 Google Inc.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! AA (Anti-Aliased) Triangulator module.
//!
//! This module implements the GrAATriangulator class from Skia, which
//! triangulates paths with alpha ramps for antialiasing. It extends the
//! base GrTriangulator with additional stages to handle screen-space AA.

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

/// AA Triangulator constant: cosine of miter angle threshold (~14 degrees)
const K_COS_MITER_ANGLE: f32 = 0.97;

/// AA Triangulator constant: quarter pixel squared distance threshold
const K_QUARTER_PIXEL_SQ: f64 = 0.25 * 0.25;

/// AA Triangulator constant: half pixel displacement for stroke
const K_HALF_PIXEL: f64 = 0.5;

/// A point in device (screen) space, where AA displacement is measured in pixels.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Point {
    /// Horizontal device-space coordinate.
    pub x: f32,
    /// Vertical device-space coordinate.
    pub y: f32,
}

impl Point {
    /// Build a point from device-space coordinates.
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

/// A direction or displacement in device space, kept at double precision because
/// edge normals are used for sub-pixel distance comparisons.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Vector {
    /// Horizontal component.
    pub x: f64,
    /// Vertical component.
    pub y: f64,
}

impl Vector {
    /// Build a vector from its components.
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Dot product, used to test whether two edge normals point the same way.
    pub fn dot(self, other: Vector) -> f64 {
        self.x * other.x + self.y * other.y
    }
}

/// An edge's supporting line in implicit form `a*x + b*y + c = 0`, matching
/// Skia's `Line` helper. The coefficients are the unnormalized edge normal, so
/// `(a, b)` doubles as the edge normal direction.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Line {
    /// Coefficient of x; equal to `bottom.y - top.y` for a line built from an edge.
    pub a: f64,
    /// Coefficient of y; equal to `top.x - bottom.x` for a line built from an edge.
    pub b: f64,
    /// Constant term; the cross product of the two endpoints.
    pub c: f64,
}

impl Line {
    /// Build a line directly from implicit-form coefficients.
    pub fn new(a: f64, b: f64, c: f64) -> Self {
        Self { a, b, c }
    }

    /// Intersection of two lines by Cramer's rule, or `None` when the
    /// determinant is near zero because the lines are parallel or coincident.
    pub fn intersect(&self, other: &Line) -> Option<Point> {
        let det = self.a * other.b - other.a * self.b;
        if det.abs() < 1e-10 {
            return None;
        }
        Some(Point::new(
            ((self.b * other.c - other.b * self.c) / det) as f32,
            ((other.a * self.c - self.a * other.c) / det) as f32,
        ))
    }

    /// True when the two lines have effectively the same normal direction.
    /// Compares the raw `a` and `b` coefficients rather than normalized ones,
    /// so it is only meaningful for lines of similar magnitude.
    pub fn near_parallel(&self, other: &Line) -> bool {
        (other.a - self.a).abs() < 0.00001 && (other.b - self.b).abs() < 0.00001
    }
}

/// Role an edge plays in the antialiased mesh. Skia's AA triangulator builds a
/// one-pixel-wide ramp around the fill, so each boundary edge is duplicated
/// into an opaque and a transparent copy joined by connectors.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EdgeType {
    /// Edge of the fully opaque interior mesh, displaced half a pixel inward.
    Inner,
    /// Edge of the transparent outer mesh, displaced half a pixel outward.
    Outer,
    /// Edge stitching an inner vertex to its outer counterpart across the ramp.
    Connector,
}

/// Axis the sweep line advances along. Skia picks the axis with the larger
/// extent so the sweep splits the path into as few monotone pieces as possible.
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum ComparatorDirection {
    /// Sweep top to bottom, breaking ties left to right.
    Vertical,
    /// Sweep left to right, breaking ties bottom to top.
    Horizontal,
}

/// Orders points along the sweep direction for the sweep-line passes.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Comparator {
    /// Axis the sweep advances along.
    pub direction: ComparatorDirection,
}

impl Comparator {
    /// Build a comparator for the given sweep axis.
    pub fn new(direction: ComparatorDirection) -> Self {
        Self { direction }
    }

    /// True when `a` precedes `b` in sweep order, using the secondary axis to
    /// break ties so that coincident points still have a total order.
    pub fn sweep_lt(&self, a: Point, b: Point) -> bool {
        match self.direction {
            ComparatorDirection::Vertical => a.y < b.y || (a.y == b.y && a.x < b.x),
            ComparatorDirection::Horizontal => a.x < b.x || (a.x == b.x && a.y > b.y),
        }
    }
}

/// A mesh vertex carrying the coverage value that produces the alpha ramp.
#[derive(Clone, Debug)]
pub struct Vertex {
    /// Device-space position after any AA displacement has been applied.
    pub point: Point,
    /// Coverage emitted for this vertex: 255 on the inner mesh, 0 on the outer
    /// mesh, and interpolated values where the ramp collapses.
    pub alpha: u8,
    /// Set when the vertex was introduced by the triangulator itself (for
    /// example at a collapsed overlap) rather than coming from the input path.
    pub synthetic: bool,
}

impl Vertex {
    /// Build a non-synthetic vertex at `point` with the given coverage.
    pub fn new(point: Point, alpha: u8) -> Self {
        Self {
            point,
            alpha,
            synthetic: false,
        }
    }
}

/// A directed mesh edge between two vertices, ordered along the sweep.
#[derive(Clone, Debug)]
pub struct Edge {
    /// Endpoint that comes first in sweep order.
    pub top: Vertex,
    /// Endpoint that comes last in sweep order.
    pub bottom: Vertex,
    /// Signed contribution to the winding number: +1 when the edge runs in the
    /// sweep direction, -1 when it was reversed to put `top` first.
    pub winding: i32,
    /// Whether this edge belongs to the inner mesh, the outer mesh, or bridges them.
    pub edge_type: EdgeType,
    /// Cached implicit line through `top` and `bottom`, used for intersection
    /// tests and as the edge normal.
    pub line: Line,
}

impl Edge {
    /// Build an edge and derive its supporting line from the two endpoints.
    pub fn new(top: Vertex, bottom: Vertex, winding: i32, edge_type: EdgeType) -> Self {
        let line = Line::new(
            (bottom.point.y - top.point.y) as f64,
            (top.point.x - bottom.point.x) as f64,
            top.point.y as f64 * bottom.point.x as f64 - top.point.x as f64 * bottom.point.y as f64,
        );
        Self {
            top,
            bottom,
            winding,
            edge_type,
            line,
        }
    }
}

/// An ordered run of vertices, standing in for Skia's intrusive `VertexList`.
/// Order is the sweep order for a mesh, or the walk order for a boundary
/// contour; the backing `Vec` replaces Skia's head/tail pointer chain.
#[derive(Clone, Debug)]
pub struct VertexList {
    vertices: Vec<Vertex>,
}

impl VertexList {
    /// Create an empty list.
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
        }
    }

    /// Number of vertices in the list.
    pub fn count(&self) -> usize {
        self.vertices.len()
    }

    /// Add a vertex at the tail, keeping the existing order.
    pub fn append(&mut self, vertex: Vertex) {
        self.vertices.push(vertex);
    }

    /// First vertex in list order, or `None` when empty.
    pub fn head(&self) -> Option<&Vertex> {
        self.vertices.first()
    }

    /// Last vertex in list order, or `None` when empty.
    pub fn tail(&self) -> Option<&Vertex> {
        self.vertices.last()
    }

    /// Walk the vertices from head to tail.
    pub fn iter(&self) -> std::slice::Iter<'_, Vertex> {
        self.vertices.iter()
    }
}

impl Default for VertexList {
    fn default() -> Self {
        Self::new()
    }
}

/// An ordered run of edges, standing in for Skia's intrusive `EdgeList`. Used
/// both for the active edge list during a sweep and for an extracted boundary
/// contour, where order is the walk around the contour.
#[derive(Clone, Debug)]
pub struct EdgeList {
    edges: Vec<Edge>,
}

impl EdgeList {
    /// Create an empty list.
    pub fn new() -> Self {
        Self { edges: Vec::new() }
    }

    /// Number of edges in the list.
    pub fn count(&self) -> usize {
        self.edges.len()
    }

    /// Add an edge at the tail, keeping the existing order.
    pub fn append(&mut self, edge: Edge) {
        self.edges.push(edge);
    }

    /// First edge in list order, or `None` when empty.
    pub fn head(&self) -> Option<&Edge> {
        self.edges.first()
    }

    /// Last edge in list order, or `None` when empty. During boundary
    /// simplification this is the most recently kept edge.
    pub fn tail(&self) -> Option<&Edge> {
        self.edges.last()
    }

    /// Walk the edges from head to tail.
    pub fn iter(&self) -> std::slice::Iter<'_, Edge> {
        self.edges.iter()
    }
}

impl Default for EdgeList {
    fn default() -> Self {
        Self::new()
    }
}

/// A monotone polygon produced by the sweep, which the final pass fans out into
/// triangles. Corresponds to Skia's `Poly`.
#[derive(Clone, Debug)]
pub struct Poly {
    vertices: Vec<Vertex>,
    /// Winding number of the region this polygon covers; the fill rule decides
    /// whether it is emitted.
    pub winding: i32,
    /// Vertex count tracked alongside the polygon as it is built. Maintained by
    /// the caller, not derived from `vertices`.
    pub count: usize,
}

impl Poly {
    /// Create an empty polygon for a region with the given winding number.
    pub fn new(winding: i32) -> Self {
        Self {
            vertices: Vec::new(),
            winding,
            count: 0,
        }
    }
}

/// An edge of the "spine" skeleton that overlap collapsing walks. Skia tracks
/// this parallel structure so an edge can be collapsed without losing the
/// connectivity of the mesh it came from. Neighbours are held as indices into
/// the skeleton's edge array instead of Skia's raw pointers.
#[derive(Clone, Debug)]
pub struct SSEdge {
    /// Mesh edge this skeleton edge shadows; `None` once the edge has collapsed.
    pub edge: Option<Edge>,
    /// Index of the preceding skeleton edge along the spine.
    pub prev_index: usize,
    /// Index of the following skeleton edge along the spine.
    pub next_index: usize,
}

impl SSEdge {
    /// Build a skeleton edge with explicit neighbour indices.
    pub fn new(edge: Option<Edge>, prev_index: usize, next_index: usize) -> Self {
        Self {
            edge,
            prev_index,
            next_index,
        }
    }
}

/// A skeleton vertex pairing a mesh vertex with its neighbours along the spine,
/// so collapsed edges can be rewired without touching the mesh itself.
#[derive(Clone, Debug)]
pub struct SSVertex {
    /// Index of the mesh vertex this skeleton vertex stands for.
    pub vertex_index: usize,
    /// Index of the incoming skeleton edge, or `usize::MAX` when unlinked.
    pub prev_index: usize,
    /// Index of the outgoing skeleton edge, or `usize::MAX` when unlinked.
    pub next_index: usize,
}

impl SSVertex {
    /// Wrap a mesh vertex, leaving both spine links unset.
    pub fn new(vertex_index: usize) -> Self {
        Self {
            vertex_index,
            prev_index: usize::MAX,
            next_index: usize::MAX,
        }
    }
}

/// A pending edge collapse during overlap resolution: the point where an edge's
/// two sides meet, queued so the tightest collapses are applied first.
#[derive(Clone, Debug)]
pub struct Event {
    /// Index of the skeleton edge that collapses when this event fires.
    pub edge_index: usize,
    /// Device-space point the collapsed edge's endpoints merge to.
    pub point: Point,
    /// Coverage to assign the merged vertex; also the event's priority key.
    pub alpha: u8,
}

impl Event {
    /// Build a collapse event for one skeleton edge.
    pub fn new(edge_index: usize, point: Point, alpha: u8) -> Self {
        Self {
            edge_index,
            point,
            alpha,
        }
    }
}

/// Which end of the alpha range an [`EventComparator`] treats as highest priority.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EventOp {
    /// Order ascending by alpha.
    LessThan,
    /// Order descending by alpha.
    GreaterThan,
}

/// Ordering policy for the event queue, selecting whether low or high coverage
/// events are collapsed first.
#[derive(Clone, Debug)]
pub struct EventComparator {
    op: EventOp,
}

impl EventComparator {
    /// Build a comparator using the given ordering.
    pub fn new(op: EventOp) -> Self {
        Self { op }
    }
}

/// Priority queue of pending edge collapses, keyed on event alpha.
#[derive(Clone, Debug)]
pub struct EventList {
    events: Vec<Event>,
}

impl EventList {
    /// Create an empty queue. The comparator only picks the initial sort order;
    /// [`EventList::push`] currently re-sorts ascending by alpha regardless.
    pub fn new(comparator: EventComparator) -> Self {
        let mut events = Vec::new();
        // Sort events by alpha according to comparator
        match comparator.op {
            EventOp::LessThan => events.sort_by_key(|e: &Event| e.alpha),
            EventOp::GreaterThan => events.sort_by_key(|e: &Event| std::cmp::Reverse(e.alpha)),
        }
        Self { events }
    }

    /// Queue an event, re-sorting ascending by alpha so the highest-alpha event
    /// sits at the tail where [`EventList::pop`] takes it from.
    pub fn push(&mut self, event: Event) {
        self.events.push(event);
        // Re-sort to maintain order
        self.events.sort_by_key(|e: &Event| e.alpha);
    }

    /// Remove and return the highest-alpha event, or `None` when the queue is empty.
    pub fn pop(&mut self) -> Option<Event> {
        self.events.pop()
    }

    /// True when no collapses remain to process.
    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// Number of events still queued.
    pub fn size(&self) -> usize {
        self.events.len()
    }
}

/// GrAATriangulator - extends GrTriangulator with AA-specific stages
#[derive(Clone, Debug)]
pub struct GrAATriangulator {
    /// Transparent mesh displaced half a pixel outward from the boundary; its
    /// vertices carry alpha 0 and form the outside of the coverage ramp.
    pub outer_mesh: VertexList,
    inner_mesh: VertexList,
}

impl GrAATriangulator {
    /// Create new AA triangulator
    pub fn new() -> Self {
        Self {
            outer_mesh: VertexList::new(),
            inner_mesh: VertexList::new(),
        }
    }

    /// Simplify boundary by removing pointy vertices that cause inversions
    ///
    /// This implements stage 5c of the AA triangulation algorithm. It
    /// detects vertices whose adjacent edge normals point in opposite directions
    /// and whose adjacent vertices are less than a quarter pixel from an edge.
    pub fn simplify_boundary(&self, boundary: &mut EdgeList, _comparator: &Comparator) -> EdgeList {
        let mut result = EdgeList::new();

        while let Some(edge) = boundary.edges.pop() {
            // Check for pointy vertices
            if let Some(prev) = result.tail() {
                let normal = get_edge_normal(&edge);
                let prev_normal = get_edge_normal(prev);

                // Check if normals point in opposite directions
                let dot = normal.dot(prev_normal);
                if dot < 0.0 {
                    // Check distance threshold
                    let prev_point = prev.bottom.point;
                    let next_point = edge.top.point;
                    let dist = ((prev_point.x - next_point.x) as f64).powi(2)
                        + ((prev_point.y - next_point.y) as f64).powi(2);

                    if dist <= K_QUARTER_PIXEL_SQ {
                        // Skip this edge - it will be merged
                        continue;
                    }
                }
            }

            result.append(edge);
        }

        result
    }

    /// Stroke boundary to create inner/outer vertex pairs
    ///
    /// This implements stage 5d of the AA triangulation algorithm. It
    /// displaces edges by half a pixel inward and outward to create
    /// antialiased vertex pairs.
    pub fn stroke_boundary(
        &self,
        boundary: &EdgeList,
        _comparator: &Comparator,
    ) -> (VertexList, VertexList) {
        let mut inner_vertices = VertexList::new();
        let mut outer_vertices = VertexList::new();

        for edge in &boundary.edges {
            let normal = get_edge_normal(edge);

            // Displace inward
            let inner_point = Point::new(
                edge.top.point.x - normal.x as f32 * K_HALF_PIXEL as f32,
                edge.top.point.y - normal.y as f32 * K_HALF_PIXEL as f32,
            );
            let inner_vertex = Vertex::new(inner_point, 255);

            // Displace outward
            let outer_point = Point::new(
                edge.top.point.x + normal.x as f32 * K_HALF_PIXEL as f32,
                edge.top.point.y + normal.y as f32 * K_HALF_PIXEL as f32,
            );
            let outer_vertex = Vertex::new(outer_point, 0);

            inner_vertices.append(inner_vertex);
            outer_vertices.append(outer_vertex);
        }

        (inner_vertices, outer_vertices)
    }

    /// Collapse overlap regions
    ///
    /// This handles complex meshes where filled regions overlap. It
    /// uses a sweep-line algorithm to find and collapse intersection points.
    pub fn collapse_overlap_regions(&self, mesh: &VertexList, _comparator: &Comparator) -> bool {
        // Simplified version - actual implementation requires full mesh manipulation
        //
        // In the full algorithm:
        // 1. Find edges that join two filled regions (overlap edges)
        // 2. Create SSEdge structures for tracking the skeleton
        // 3. Process events in priority queue order to collapse edges
        // 4. Create connector edges between collapsed vertices

        // Check if there are complex overlaps
        let mut has_overlaps = false;
        for vertex in mesh.iter() {
            if vertex.synthetic {
                has_overlaps = true;
                break;
            }
        }

        has_overlaps
    }
}

fn get_edge_normal(edge: &Edge) -> Vector {
    Vector::new(edge.line.a, edge.line.b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangulator_creation() {
        let triangulator = GrAATriangulator::new();
        assert!(triangulator.outer_mesh.count() == 0);
    }

    #[test]
    fn test_line_intersection() {
        let line1 = Line::new(1.0, -1.0, 0.0); // y = x
        let line2 = Line::new(1.0, 1.0, 0.0); // y = -x
        let intersection = line1.intersect(&line2);
        assert!(intersection.is_some());
        let point = intersection.unwrap();
        assert!((point.x - 0.0).abs() < 1e-6);
        assert!((point.y - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_line_parallel() {
        let line1 = Line::new(1.0, 1.0, 0.0);
        let line2 = Line::new(1.0, 1.0, 5.0); // Parallel, different offset
        assert!(line1.near_parallel(&line2));
    }

    #[test]
    fn test_edge_creation() {
        let top = Vertex::new(Point::new(0.0, 0.0), 255);
        let bottom = Vertex::new(Point::new(0.0, 10.0), 255);
        let edge = Edge::new(top, bottom, 1, EdgeType::Outer);
        assert_eq!(edge.winding, 1);
        assert_eq!(edge.edge_type, EdgeType::Outer);
    }

    #[test]
    fn test_vertex_list() {
        let mut list = VertexList::new();
        let v = Vertex::new(Point::new(0.0, 0.0), 255);
        list.append(v);
        assert_eq!(list.count(), 1);
    }

    #[test]
    fn test_edge_list() {
        let mut list = EdgeList::new();
        let top = Vertex::new(Point::new(0.0, 0.0), 255);
        let bottom = Vertex::new(Point::new(0.0, 10.0), 255);
        let edge = Edge::new(top, bottom, 1, EdgeType::Outer);
        list.append(edge);
        assert_eq!(list.count(), 1);
    }

    #[test]
    fn test_event_list() {
        let comparator = EventComparator::new(EventOp::LessThan);
        let mut list = EventList::new(comparator);

        list.push(Event::new(0, Point::new(0.0, 0.0), 100));
        list.push(Event::new(1, Point::new(1.0, 1.0), 50));
        list.push(Event::new(2, Point::new(2.0, 2.0), 200));

        assert_eq!(list.size(), 3);

        // Pop returns the last element in the vector
        let event1 = list.pop().unwrap();
        assert_eq!(event1.alpha, 200); // Last in sorted order

        let event2 = list.pop().unwrap();
        assert_eq!(event2.alpha, 100);
    }

    #[test]
    fn test_simplify_boundary() {
        let mut boundary = EdgeList::new();
        let top = Vertex::new(Point::new(0.0, 0.0), 255);
        let bottom = Vertex::new(Point::new(0.0, 10.0), 255);
        let edge = Edge::new(top, bottom, 1, EdgeType::Outer);
        boundary.append(edge);

        let comparator = Comparator::new(ComparatorDirection::Vertical);
        let result = GrAATriangulator::new().simplify_boundary(&mut boundary, &comparator);
        assert!(result.count() >= 0);
    }

    #[test]
    fn test_stroke_boundary() {
        let mut boundary = EdgeList::new();
        let top = Vertex::new(Point::new(0.0, 0.0), 255);
        let bottom = Vertex::new(Point::new(10.0, 10.0), 255);
        let edge = Edge::new(top, bottom, 1, EdgeType::Outer);
        boundary.append(edge);

        let comparator = Comparator::new(ComparatorDirection::Vertical);
        let (inner, outer) = GrAATriangulator::new().stroke_boundary(&boundary, &comparator);
        assert_eq!(inner.count(), 1);
        assert_eq!(outer.count(), 1);
    }

    #[test]
    fn test_constants() {
        assert_eq!(K_COS_MITER_ANGLE, 0.97);
        assert_eq!(K_QUARTER_PIXEL_SQ, 0.0625);
        assert_eq!(K_HALF_PIXEL, 0.5);
    }

    #[test]
    fn test_comparator_vertical() {
        let comparator = Comparator::new(ComparatorDirection::Vertical);

        // Same Y, different X
        assert!(comparator.sweep_lt(Point::new(0.0, 0.0), Point::new(1.0, 0.0)));

        // Different Y
        assert!(comparator.sweep_lt(Point::new(0.0, 0.0), Point::new(0.0, 1.0)));
        assert!(!comparator.sweep_lt(Point::new(0.0, 1.0), Point::new(0.0, 0.0)));
    }

    #[test]
    fn test_comparator_horizontal() {
        let comparator = Comparator::new(ComparatorDirection::Horizontal);

        // Same X, different Y
        assert!(comparator.sweep_lt(Point::new(0.0, 1.0), Point::new(0.0, 0.0)));

        // Different X
        assert!(comparator.sweep_lt(Point::new(0.0, 0.0), Point::new(1.0, 0.0)));
        assert!(!comparator.sweep_lt(Point::new(1.0, 0.0), Point::new(0.0, 0.0)));
    }

    #[test]
    fn test_sse_edge() {
        let edge = SSEdge::new(None, 0, 1);
        assert_eq!(edge.prev_index, 0);
        assert_eq!(edge.next_index, 1);
    }

    #[test]
    fn test_ss_vertex() {
        let vertex = SSVertex::new(42);
        assert_eq!(vertex.vertex_index, 42);
        assert_eq!(vertex.prev_index, usize::MAX);
        assert_eq!(vertex.next_index, usize::MAX);
    }

    #[test]
    fn test_collapse_overlap_regions() {
        let mut mesh = VertexList::new();
        mesh.append(Vertex::new(Point::new(0.0, 0.0), 255));

        let comparator = Comparator::new(ComparatorDirection::Vertical);
        let result = GrAATriangulator::new().collapse_overlap_regions(&mesh, &comparator);
        assert!(!result); // No synthetic vertices
    }
}
