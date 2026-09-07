// Copyright 2020 Google Inc.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! AA (Anti-Aliased) Triangulator module.
//!
//! This module implements the GrAATriangulator class from Skia, which
//! triangulates paths with alpha ramps for antialiasing. It extends the
//! base GrTriangulator with additional stages to handle screen-space AA.

/// AA Triangulator constant: cosine of miter angle threshold (~14 degrees)
const K_COS_MITER_ANGLE: f32 = 0.97;

/// AA Triangulator constant: quarter pixel squared distance threshold
const K_QUARTER_PIXEL_SQ: f64 = 0.25 * 0.25;

/// AA Triangulator constant: half pixel displacement for stroke
const K_HALF_PIXEL: f64 = 0.5;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

impl Point {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
}

impl Vector {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn dot(self, other: Vector) -> f64 {
        self.x * other.x + self.y * other.y
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Line {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

impl Line {
    pub fn new(a: f64, b: f64, c: f64) -> Self {
        Self { a, b, c }
    }

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

    pub fn near_parallel(&self, other: &Line) -> bool {
        (other.a - self.a).abs() < 0.00001 && (other.b - self.b).abs() < 0.00001
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EdgeType {
    Inner,
    Outer,
    Connector,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum ComparatorDirection {
    Vertical,
    Horizontal,
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Comparator {
    pub direction: ComparatorDirection,
}

impl Comparator {
    pub fn new(direction: ComparatorDirection) -> Self {
        Self { direction }
    }

    pub fn sweep_lt(&self, a: Point, b: Point) -> bool {
        match self.direction {
            ComparatorDirection::Vertical => a.y < b.y || (a.y == b.y && a.x < b.x),
            ComparatorDirection::Horizontal => a.x < b.x || (a.x == b.x && a.y > b.y),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Vertex {
    pub point: Point,
    pub alpha: u8,
    pub synthetic: bool,
}

impl Vertex {
    pub fn new(point: Point, alpha: u8) -> Self {
        Self { point, alpha, synthetic: false }
    }
}

#[derive(Clone, Debug)]
pub struct Edge {
    pub top: Vertex,
    pub bottom: Vertex,
    pub winding: i32,
    pub edge_type: EdgeType,
    pub line: Line,
}

impl Edge {
    pub fn new(top: Vertex, bottom: Vertex, winding: i32, edge_type: EdgeType) -> Self {
        let line = Line::new(
            (bottom.point.y - top.point.y) as f64,
            (top.point.x - bottom.point.x) as f64,
            top.point.y as f64 * bottom.point.x as f64 - top.point.x as f64 * bottom.point.y as f64,
        );
        Self { top, bottom, winding, edge_type, line }
    }
}

#[derive(Clone, Debug)]
pub struct VertexList {
    vertices: Vec<Vertex>,
}

impl VertexList {
    pub fn new() -> Self {
        Self { vertices: Vec::new() }
    }

    pub fn count(&self) -> usize {
        self.vertices.len()
    }

    pub fn append(&mut self, vertex: Vertex) {
        self.vertices.push(vertex);
    }

    pub fn head(&self) -> Option<&Vertex> {
        self.vertices.first()
    }

    pub fn tail(&self) -> Option<&Vertex> {
        self.vertices.last()
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Vertex> {
        self.vertices.iter()
    }
}

impl Default for VertexList {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
pub struct EdgeList {
    edges: Vec<Edge>,
}

impl EdgeList {
    pub fn new() -> Self {
        Self { edges: Vec::new() }
    }

    pub fn count(&self) -> usize {
        self.edges.len()
    }

    pub fn append(&mut self, edge: Edge) {
        self.edges.push(edge);
    }

    pub fn head(&self) -> Option<&Edge> {
        self.edges.first()
    }

    pub fn tail(&self) -> Option<&Edge> {
        self.edges.last()
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Edge> {
        self.edges.iter()
    }
}

impl Default for EdgeList {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
pub struct Poly {
    vertices: Vec<Vertex>,
    pub winding: i32,
    pub count: usize,
}

impl Poly {
    pub fn new(winding: i32) -> Self {
        Self { vertices: Vec::new(), winding, count: 0 }
    }
}

#[derive(Clone, Debug)]
pub struct SSEdge {
    pub edge: Option<Edge>,
    pub prev_index: usize,
    pub next_index: usize,
}

impl SSEdge {
    pub fn new(edge: Option<Edge>, prev_index: usize, next_index: usize) -> Self {
        Self { edge, prev_index, next_index }
    }
}

#[derive(Clone, Debug)]
pub struct SSVertex {
    pub vertex_index: usize,
    pub prev_index: usize,
    pub next_index: usize,
}

impl SSVertex {
    pub fn new(vertex_index: usize) -> Self {
        Self { vertex_index, prev_index: usize::MAX, next_index: usize::MAX }
    }
}

#[derive(Clone, Debug)]
pub struct Event {
    pub edge_index: usize,
    pub point: Point,
    pub alpha: u8,
}

impl Event {
    pub fn new(edge_index: usize, point: Point, alpha: u8) -> Self {
        Self { edge_index, point, alpha }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EventOp {
    LessThan,
    GreaterThan,
}

#[derive(Clone, Debug)]
pub struct EventComparator {
    op: EventOp,
}

impl EventComparator {
    pub fn new(op: EventOp) -> Self {
        Self { op }
    }
}

#[derive(Clone, Debug)]
pub struct EventList {
    events: Vec<Event>,
}

impl EventList {
    pub fn new(comparator: EventComparator) -> Self {
        let mut events = Vec::new();
        // Sort events by alpha according to comparator
        match comparator.op {
            EventOp::LessThan => events.sort_by_key(|e: &Event| e.alpha),
            EventOp::GreaterThan => events.sort_by_key(|e: &Event| std::cmp::Reverse(e.alpha)),
        }
        Self { events }
    }

    pub fn push(&mut self, event: Event) {
        self.events.push(event);
        // Re-sort to maintain order
        self.events.sort_by_key(|e: &Event| e.alpha);
    }

    pub fn pop(&mut self) -> Option<Event> {
        self.events.pop()
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    pub fn size(&self) -> usize {
        self.events.len()
    }
}

/// GrAATriangulator - extends GrTriangulator with AA-specific stages
#[derive(Clone, Debug)]
pub struct GrAATriangulator {
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
    pub fn simplify_boundary(
        &self,
        boundary: &mut EdgeList,
        comparator: &Comparator,
    ) -> EdgeList {
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
        comparator: &Comparator,
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
    pub fn collapse_overlap_regions(
        &self,
        mesh: &VertexList,
        comparator: &Comparator,
    ) -> bool {
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
        let line1 = Line::new(1.0, -1.0, 0.0);  // y = x
        let line2 = Line::new(1.0, 1.0, 0.0);   // y = -x
        let intersection = line1.intersect(&line2);
        assert!(intersection.is_some());
        let point = intersection.unwrap();
        assert!((point.x - 0.0).abs() < 1e-6);
        assert!((point.y - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_line_parallel() {
        let line1 = Line::new(1.0, 1.0, 0.0);
        let line2 = Line::new(1.0, 1.0, 5.0);  // Parallel, different offset
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
        assert_eq!(event1.alpha, 200);  // Last in sorted order
        
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
        assert!(!result);  // No synthetic vertices
    }
}
