//! Boolean path operations via flatten, split, and inside tests.
//!
//! The full Skia pathops engine is not yet wired. This module implements
//! union/intersect/difference/xor for closed paths by flattening curves,
//! splitting edges at intersections, and keeping pieces that sit on the
//! result boundary according to [`Path::contains`].

use crate::core::{FillType, Path, Point, Verb};
use crate::error::PathKitError;

use super::PathOp;

const FLAT_TOL: f32 = 0.1;
const MAX_FLAT_DEPTH: u32 = 16;
const MIN_EDGE: f32 = 1e-4;
const OFFSET: f32 = 0.25;
const T_EPS: f32 = 1e-6;

pub(super) fn path_op(one: &Path, two: &Path, op: PathOp) -> Result<Path, PathKitError> {
    let mut segs = flatten(one);
    segs.extend(flatten(two));
    if segs.is_empty() {
        return Ok(Path::new());
    }

    let pieces = split_segments(&segs);
    let mut edges: Vec<Edge> = Vec::new();
    for piece in pieces {
        if let Some(edge) = classify_edge(piece, one, two, op) {
            edges.push(edge);
        }
    }
    if edges.is_empty() {
        return Ok(Path::new());
    }

    Ok(assemble(&edges))
}

struct Edge {
    from: Point,
    to: Point,
}

fn flatten(path: &Path) -> Vec<[Point; 2]> {
    let mut segs = Vec::new();
    let mut contour_start = Point::default();
    let mut last = Point::default();
    let mut started = false;

    for (verb, pts, _weight) in path.iter() {
        match verb {
            Verb::Move => {
                contour_start = pts[0];
                last = pts[0];
                started = true;
            }
            Verb::Line => {
                push_seg(&mut segs, pts[0], pts[1]);
                last = pts[1];
            }
            Verb::Quad | Verb::Conic => {
                flatten_quad(pts[0], pts[1], pts[2], MAX_FLAT_DEPTH, &mut segs);
                last = pts[2];
            }
            Verb::Cubic => {
                flatten_cubic(
                    pts[0],
                    pts[1],
                    pts[2],
                    pts[3],
                    MAX_FLAT_DEPTH,
                    &mut segs,
                );
                last = pts[3];
            }
            Verb::Close => {
                if started {
                    push_seg(&mut segs, last, contour_start);
                }
            }
        }
    }
    segs
}

fn push_seg(segs: &mut Vec<[Point; 2]>, a: Point, b: Point) {
    if Point::distance(a, b) >= MIN_EDGE {
        segs.push([a, b]);
    }
}

fn flatten_quad(p0: Point, p1: Point, p2: Point, depth: u32, out: &mut Vec<[Point; 2]>) {
    if depth == 0 || dist_to_line(p1, p0, p2) <= FLAT_TOL {
        push_seg(out, p0, p2);
        return;
    }
    let p01 = mid(p0, p1);
    let p12 = mid(p1, p2);
    let p012 = mid(p01, p12);
    flatten_quad(p0, p01, p012, depth - 1, out);
    flatten_quad(p012, p12, p2, depth - 1, out);
}

fn flatten_cubic(
    p0: Point,
    p1: Point,
    p2: Point,
    p3: Point,
    depth: u32,
    out: &mut Vec<[Point; 2]>,
) {
    if depth == 0
        || (dist_to_line(p1, p0, p3) <= FLAT_TOL && dist_to_line(p2, p0, p3) <= FLAT_TOL)
    {
        push_seg(out, p0, p3);
        return;
    }
    let p01 = mid(p0, p1);
    let p12 = mid(p1, p2);
    let p23 = mid(p2, p3);
    let p012 = mid(p01, p12);
    let p123 = mid(p12, p23);
    let p0123 = mid(p012, p123);
    flatten_cubic(p0, p01, p012, p0123, depth - 1, out);
    flatten_cubic(p0123, p123, p23, p3, depth - 1, out);
}

fn mid(a: Point, b: Point) -> Point {
    Point::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5)
}

fn dist_to_line(p: Point, a: Point, b: Point) -> f32 {
    let ab = b - a;
    let len = ab.length();
    if len < 1e-12 {
        return Point::distance(p, a);
    }
    ab.cross(p - a).abs() / len
}

fn split_segments(segs: &[[Point; 2]]) -> Vec<[Point; 2]> {
    let n = segs.len();
    let mut params: Vec<Vec<f32>> = vec![vec![0.0, 1.0]; n];
    for i in 0..n {
        for j in (i + 1)..n {
            let (ti, tj) = split_params(segs[i], segs[j]);
            params[i].extend(ti);
            params[j].extend(tj);
        }
    }

    let mut out = Vec::new();
    for (seg, mut ts) in segs.iter().zip(params) {
        ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        ts.dedup_by(|a, b| (*a - *b).abs() < T_EPS);
        for w in ts.windows(2) {
            let a = lerp(seg[0], seg[1], w[0]);
            let b = lerp(seg[0], seg[1], w[1]);
            push_seg(&mut out, a, b);
        }
    }
    out
}

fn split_params(a: [Point; 2], b: [Point; 2]) -> (Vec<f32>, Vec<f32>) {
    let mut ta = Vec::new();
    let mut tb = Vec::new();
    let da = a[1] - a[0];
    let db = b[1] - b[0];
    let denom = da.cross(db);
    let ab = b[0] - a[0];
    if denom.abs() > 1e-12 {
        let t = ab.cross(db) / denom;
        let u = ab.cross(da) / denom;
        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
            if t > T_EPS && t < 1.0 - T_EPS {
                ta.push(t);
            }
            if u > T_EPS && u < 1.0 - T_EPS {
                tb.push(u);
            }
        }
    } else if da.cross(ab).abs() <= 1e-4 * da.length().max(1.0) {
        if let Some(t) = project_t(a, b[0]) {
            if t > T_EPS && t < 1.0 - T_EPS {
                ta.push(t);
            }
        }
        if let Some(t) = project_t(a, b[1]) {
            if t > T_EPS && t < 1.0 - T_EPS {
                ta.push(t);
            }
        }
        if let Some(u) = project_t(b, a[0]) {
            if u > T_EPS && u < 1.0 - T_EPS {
                tb.push(u);
            }
        }
        if let Some(u) = project_t(b, a[1]) {
            if u > T_EPS && u < 1.0 - T_EPS {
                tb.push(u);
            }
        }
    }
    (ta, tb)
}

fn project_t(seg: [Point; 2], p: Point) -> Option<f32> {
    let d = seg[1] - seg[0];
    let len2 = d.dot(d);
    if len2 < 1e-16 {
        return None;
    }
    Some((p - seg[0]).dot(d) / len2)
}

fn lerp(a: Point, b: Point, t: f32) -> Point {
    Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

fn classify_edge(seg: [Point; 2], one: &Path, two: &Path, op: PathOp) -> Option<Edge> {
    let dir = seg[1] - seg[0];
    let len = dir.length();
    if len < MIN_EDGE {
        return None;
    }
    let step = OFFSET.min(len * 0.5);
    let n = Point::new(-dir.y / len * step, dir.x / len * step);
    let mid_pt = mid(seg[0], seg[1]);
    let left = mid_pt + n;
    let right = mid_pt - n;
    let in_left = in_result(left, one, two, op);
    let in_right = in_result(right, one, two, op);
    if in_left == in_right {
        return None;
    }
    if in_left {
        Some(Edge {
            from: seg[0],
            to: seg[1],
        })
    } else {
        Some(Edge {
            from: seg[1],
            to: seg[0],
        })
    }
}

fn in_result(p: Point, one: &Path, two: &Path, op: PathOp) -> bool {
    let a = one.contains(p.x, p.y);
    let b = two.contains(p.x, p.y);
    match op {
        PathOp::Difference => a && !b,
        PathOp::Intersect => a && b,
        PathOp::Union => a || b,
        PathOp::Xor => a != b,
        PathOp::ReverseDifference => b && !a,
    }
}

fn key(p: Point) -> (i32, i32) {
    (
        (p.x * 256.0).round() as i32,
        (p.y * 256.0).round() as i32,
    )
}

fn assemble(edges: &[Edge]) -> Path {
    let n = edges.len();
    let mut outgoing: std::collections::HashMap<(i32, i32), Vec<usize>> =
        std::collections::HashMap::new();
    for (i, e) in edges.iter().enumerate() {
        outgoing.entry(key(e.from)).or_default().push(i);
    }

    let mut used = vec![false; n];
    let mut path = Path::new();
    path.set_fill_type(FillType::Winding);

    for start in 0..n {
        if used[start] {
            continue;
        }
        let mut idx = start;
        let origin = edges[idx].from;
        let mut verts: Vec<Point> = vec![origin];
        let mut incoming = edges[idx].to - edges[idx].from;
        let mut closed = false;
        loop {
            used[idx] = true;
            let cur = edges[idx].to;
            if key(cur) == key(origin) && verts.len() >= 3 {
                closed = true;
                break;
            }
            verts.push(cur);
            let Some(cands) = outgoing.get(&key(cur)) else {
                break;
            };
            let mut best: Option<(usize, f32)> = None;
            for &j in cands {
                if used[j] {
                    continue;
                }
                let out = edges[j].to - edges[j].from;
                let cr = incoming.cross(out);
                let dt = incoming.dot(out);
                let ang = cr.atan2(dt);
                if best.map_or(true, |(_, a)| ang > a) {
                    best = Some((j, ang));
                }
            }
            let Some((next, _)) = best else {
                break;
            };
            incoming = edges[next].to - edges[next].from;
            idx = next;
            if verts.len() > n + 2 {
                break;
            }
        }
        if !closed || verts.len() < 3 {
            continue;
        }
        path.move_to(verts[0].x, verts[0].y);
        for v in verts.iter().skip(1) {
            path.line_to(v.x, v.y);
        }
        path.close();
    }

    path
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Rect;

    fn rect_path(l: f32, t: f32, r: f32, b: f32) -> Path {
        let mut p = Path::new();
        p.add_rect_simple(Rect::from_ltrb(l, t, r, b));
        p
    }

    #[test]
    fn union_l_shape_is_not_bounding_rect() {
        let a = rect_path(0.0, 0.0, 10.0, 30.0);
        let b = rect_path(0.0, 20.0, 40.0, 30.0);
        let result = path_op(&a, &b, PathOp::Union).unwrap();
        assert!(result.contains(5.0, 5.0));
        assert!(result.contains(30.0, 25.0));
        assert!(result.contains(5.0, 25.0));
        assert!(!result.contains(30.0, 5.0));
    }

    #[test]
    fn intersect_two_squares() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 5.0, 15.0, 15.0);
        let result = path_op(&a, &b, PathOp::Intersect).unwrap();
        assert!(result.contains(7.0, 7.0));
        assert!(!result.contains(2.0, 2.0));
        assert!(!result.contains(12.0, 12.0));
    }

    #[test]
    fn difference_cuts_right_half() {
        let a = rect_path(0.0, 0.0, 10.0, 10.0);
        let b = rect_path(5.0, 0.0, 10.0, 10.0);
        let result = path_op(&a, &b, PathOp::Difference).unwrap();
        assert!(result.contains(2.0, 5.0));
        assert!(!result.contains(7.0, 5.0));
    }
}
