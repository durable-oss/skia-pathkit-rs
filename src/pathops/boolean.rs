//! Boolean path operations via flatten, split, and inside tests.
//!
//! The full Skia pathops engine is not yet wired. This module implements
//! union/intersect/difference/xor for closed paths by flattening curves,
//! splitting edges at intersections, and keeping pieces that sit on the
//! result boundary according to [`Path::contains`].


use crate::core::{Conic, FillType, Path, Point, Scalar, Verb};
use crate::error::PathKitError;

use super::PathOp;

const FLAT_TOL: f32 = 0.1;
const MAX_FLAT_DEPTH: u32 = 16;
const MIN_EDGE: f32 = 1e-4;
const OFFSET: f32 = 0.25;
const T_EPS: f32 = 1e-6;

pub(super) fn path_op(one: &Path, two: &Path, op: PathOp) -> Result<Path, PathKitError> {
    // Flatten finely enough that the two boundaries stay distinguishable.
    // Two arcs flattened independently each deviate from the true curve by up
    // to the tolerance, so where the inputs run closer together than that,
    // their chords interleave and the crossings come out scrambled.
    let tol = flatten_tolerance(one, two);
    let mut segs = flatten(one, tol);
    segs.extend(flatten(two, tol));
    if segs.is_empty() {
        return Ok(Path::new());
    }

    let pieces = split_segments(&segs);
    let mut edges: Vec<Edge> = Vec::new();
    for (i, piece) in pieces.iter().enumerate() {
        let clearance = clearance_at(&pieces, i);
        if let Some(edge) = classify_edge(*piece, one, two, op, clearance) {
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

/// Returns a flattening tolerance fine enough to keep `one` and `two` apart.
///
/// Bounded below so that nearly-identical inputs do not tessellate without
/// limit, and above by [`FLAT_TOL`], which is accurate enough whenever the two
/// paths are comfortably separated.
fn flatten_tolerance(one: &Path, two: &Path) -> f32 {
    let (a, b) = (one.bounds(), two.bounds());
    // How far the two bounding boxes are from coinciding. For shapes that
    // nearly overlap this is small, and the tolerance follows it down.
    let sep = (a.left - b.left)
        .abs()
        .max((a.top - b.top).abs())
        .max((a.right - b.right).abs())
        .max((a.bottom - b.bottom).abs());
    let extent = (a.right - a.left)
        .abs()
        .max((a.bottom - a.top).abs())
        .max((b.right - b.left).abs())
        .max((b.bottom - b.top).abs());
    // Never finer than this fraction of the shapes themselves, or a big scene
    // would tessellate into an unbounded number of chords. On a large scene
    // the floor can exceed FLAT_TOL, so it wins rather than clamping to an
    // empty range.
    let floor = (extent * 1e-4).max(1e-4);
    if sep > 0.0 {
        (sep * 0.05).min(FLAT_TOL).max(floor)
    } else {
        FLAT_TOL
    }
}

fn flatten(path: &Path, tol: f32) -> Vec<[Point; 2]> {
    let mut segs = Vec::new();
    let mut contour_start = Point::default();
    let mut last = Point::default();
    let mut started = false;

    for (verb, pts, weight) in path.iter() {
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
            Verb::Quad => {
                flatten_quad(pts[0], pts[1], pts[2], tol, MAX_FLAT_DEPTH, &mut segs);
                last = pts[2];
            }
            Verb::Conic => {
                flatten_conic(
                    pts[0],
                    pts[1],
                    pts[2],
                    weight.unwrap_or(1.0),
                    tol,
                    MAX_FLAT_DEPTH,
                    &mut segs,
                );
                last = pts[2];
            }
            Verb::Cubic => {
                flatten_cubic(
                    pts[0],
                    pts[1],
                    pts[2],
                    pts[3],
                    tol,
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

fn flatten_quad(p0: Point, p1: Point, p2: Point, tol: f32, depth: u32, out: &mut Vec<[Point; 2]>) {
    if depth == 0 || dist_to_line(p1, p0, p2) <= tol {
        push_seg(out, p0, p2);
        return;
    }
    let p01 = mid(p0, p1);
    let p12 = mid(p1, p2);
    let p012 = mid(p01, p12);
    flatten_quad(p0, p01, p012, tol, depth - 1, out);
    flatten_quad(p012, p12, p2, tol, depth - 1, out);
}

/// Flattens a conic into chords, honouring its weight.
///
/// A conic with `w != 1` traces a different curve than the quadratic through
/// the same three points, so it cannot be flattened as a quad: the flattened
/// boundary would disagree with `Path::contains`, which evaluates the conic
/// correctly, and every edge would then be classified against the wrong side.
///
/// Subdivision uses [`Conic::chop`], which splits in the rational form and so
/// keeps both halves on the original curve.
fn flatten_conic(
    p0: Point,
    p1: Point,
    p2: Point,
    w: Scalar,
    tol: f32,
    depth: u32,
    out: &mut Vec<[Point; 2]>,
) {
    // A weight of 1 is exactly the quadratic, and the control point's distance
    // to the chord bounds the error of the straight-line approximation.
    if depth == 0 || dist_to_line(p1, p0, p2) <= tol {
        push_seg(out, p0, p2);
        return;
    }
    let conic = Conic::new([p0, p1, p2], w);
    let mut halves = [Conic::new([Point::default(); 3], 1.0); 2];
    conic.chop(&mut halves);
    if !halves[0].is_finite() || !halves[1].is_finite() {
        // Degenerate weight; fall back to the chord rather than recursing.
        push_seg(out, p0, p2);
        return;
    }
    flatten_conic(
        halves[0].pts[0],
        halves[0].pts[1],
        halves[0].pts[2],
        halves[0].w,
        tol,
        depth - 1,
        out,
    );
    flatten_conic(
        halves[1].pts[0],
        halves[1].pts[1],
        halves[1].pts[2],
        halves[1].w,
        tol,
        depth - 1,
        out,
    );
}

fn flatten_cubic(
    p0: Point,
    p1: Point,
    p2: Point,
    p3: Point,
    tol: f32,
    depth: u32,
    out: &mut Vec<[Point; 2]>,
) {
    if depth == 0 || (dist_to_line(p1, p0, p3) <= tol && dist_to_line(p2, p0, p3) <= tol) {
        push_seg(out, p0, p3);
        return;
    }
    let p01 = mid(p0, p1);
    let p12 = mid(p1, p2);
    let p23 = mid(p2, p3);
    let p012 = mid(p01, p12);
    let p123 = mid(p12, p23);
    let p0123 = mid(p012, p123);
    flatten_cubic(p0, p01, p012, p0123, tol, depth - 1, out);
    flatten_cubic(p0123, p123, p23, p3, tol, depth - 1, out);
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

/// Returns how far the midpoint of `pieces[i]` sits from every other piece.
///
/// The pieces have already been split at every crossing, so no other piece
/// passes through this one's interior. Half that distance is therefore a step
/// that is guaranteed to stay in the region on either side of this edge and
/// not spill across a neighbouring boundary.
fn clearance_at(pieces: &[[Point; 2]], i: usize) -> f32 {
    let mid_pt = mid(pieces[i][0], pieces[i][1]);
    let mut best = f32::INFINITY;
    for (j, other) in pieces.iter().enumerate() {
        if j == i {
            continue;
        }
        let d = dist_to_segment(mid_pt, other[0], other[1]);
        // A piece lying on top of this one is a coincident edge, where the two
        // input paths share a boundary. It says nothing about how far there is
        // to step, and taking it would drive the step to zero and lose the
        // edge entirely, breaking the contour that runs through it.
        if d <= MIN_EDGE {
            continue;
        }
        if d < best {
            best = d;
        }
    }
    if best.is_finite() {
        // Half, so the sample lands strictly between the two boundaries.
        best * 0.5
    } else {
        OFFSET
    }
}

/// Returns the distance from `p` to the segment `a`-`b`.
fn dist_to_segment(p: Point, a: Point, b: Point) -> f32 {
    let ab = b - a;
    let len2 = ab.dot(ab);
    if len2 < 1e-16 {
        return Point::distance(p, a);
    }
    let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
    Point::distance(p, Point::new(a.x + ab.x * t, a.y + ab.y * t))
}

fn classify_edge(
    seg: [Point; 2],
    one: &Path,
    two: &Path,
    op: PathOp,
    clearance: f32,
) -> Option<Edge> {
    let dir = seg[1] - seg[0];
    let len = dir.length();
    if len < MIN_EDGE {
        return None;
    }
    let mid_pt = mid(seg[0], seg[1]);
    // Step perpendicular far enough to leave this edge, but not so far as to
    // cross a different one. Where two boundaries run close together - a
    // near-tangent sliver - a fixed step lands beyond both and the edge is
    // misread as a boundary, which breaks the contour that should have been
    // assembled through it. `clearance` is the room actually available here.
    let mut step = OFFSET.min(len * 0.5).min(clearance);
    let unit = Point::new(-dir.y / len, dir.x / len);
    // Shrink until the two samples disagree; a sliver may need several halvings.
    let mut decided = None;
    for _ in 0..24 {
        if step < MIN_EDGE {
            break;
        }
        let n = Point::new(unit.x * step, unit.y * step);
        let in_left = in_result(mid_pt + n, one, two, op);
        let in_right = in_result(mid_pt - n, one, two, op);
        if in_left != in_right {
            decided = Some(in_left);
            break;
        }
        step *= 0.5;
    }
    let in_left = decided?;
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

/// Relative tolerance for welding two edge endpoints into one vertex.
///
/// Endpoints come from intersections computed in f32, so the two edges meeting
/// at a vertex rarely land on bit-identical coordinates. Anything closer than
/// this fraction of the scene's size is the same vertex.
const WELD_REL_TOL: f32 = 1e-5;

/// Groups edge endpoints that are the same vertex, so chains can be walked by
/// exact identity afterwards.
///
/// A fixed quantization grid cannot do this job: two endpoints straddling a
/// cell boundary stay apart no matter how fine the grid, and a coarse grid
/// welds points that are genuinely distinct. The tolerance here scales with
/// the input instead, which is what the failure demanded - the same shapes
/// assembled correctly once the scene was scaled up.
struct VertexWeld {
    /// Cluster representatives, indexed by cluster id.
    reps: Vec<Point>,
    /// Cell size used for the lookup grid; at least one cell per tolerance.
    cell: f32,
    /// Cluster id per occupied grid cell.
    cells: std::collections::HashMap<(i32, i32), Vec<usize>>,
}

impl VertexWeld {
    /// Builds a weld over every endpoint in `edges`.
    fn new(edges: &[Edge]) -> Self {
        // Size the tolerance from the extent of the geometry, not from a
        // constant, so that scaling the scene does not change the outcome.
        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;
        for e in edges {
            for p in [e.from, e.to] {
                min_x = min_x.min(p.x);
                min_y = min_y.min(p.y);
                max_x = max_x.max(p.x);
                max_y = max_y.max(p.y);
            }
        }
        let extent = (max_x - min_x).max(max_y - min_y);
        let tol = if extent.is_finite() && extent > 0.0 {
            extent * WELD_REL_TOL
        } else {
            WELD_REL_TOL
        };
        let mut weld = VertexWeld {
            reps: Vec::new(),
            // One cell per tolerance: a point's cluster is then always in its
            // own cell or one of the eight neighbours.
            cell: tol.max(f32::MIN_POSITIVE),
            cells: std::collections::HashMap::new(),
        };
        for e in edges {
            weld.intern(e.from);
            weld.intern(e.to);
        }
        weld
    }

    /// Returns the grid cell `p` falls in.
    fn cell_of(&self, p: Point) -> (i32, i32) {
        (
            (p.x / self.cell).floor() as i32,
            (p.y / self.cell).floor() as i32,
        )
    }

    /// Returns the cluster id for `p`, creating one if nothing is near enough.
    fn intern(&mut self, p: Point) -> usize {
        if let Some(existing) = self.find(p) {
            return existing;
        }
        let id = self.reps.len();
        self.reps.push(p);
        let (cx, cy) = self.cell_of(p);
        self.cells.entry((cx, cy)).or_default().push(id);
        id
    }

    /// Returns the cluster `p` belongs to, if there is one within tolerance.
    fn find(&self, p: Point) -> Option<usize> {
        let (cx, cy) = self.cell_of(p);
        let tol2 = self.cell * self.cell;
        let mut best: Option<(usize, f32)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                let Some(ids) = self.cells.get(&(cx + dx, cy + dy)) else {
                    continue;
                };
                for &id in ids {
                    let r = self.reps[id];
                    let d2 = (r.x - p.x).powi(2) + (r.y - p.y).powi(2);
                    if d2 <= tol2 && best.map_or(true, |(_, b)| d2 < b) {
                        best = Some((id, d2));
                    }
                }
            }
        }
        best.map(|(id, _)| id)
    }

    /// Returns the cluster id for a point known to have been interned.
    fn id_of(&self, p: Point) -> usize {
        self.find(p).expect("every endpoint was interned")
    }
}

fn assemble(edges: &[Edge]) -> Path {
    let n = edges.len();
    if n == 0 {
        return Path::new();
    }
    // Weld endpoints first so the chain walk can match on exact identity.
    let weld = VertexWeld::new(edges);
    let from_id: Vec<usize> = edges.iter().map(|e| weld.id_of(e.from)).collect();
    let to_id: Vec<usize> = edges.iter().map(|e| weld.id_of(e.to)).collect();

    let mut outgoing: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    for (i, &id) in from_id.iter().enumerate() {
        outgoing.entry(id).or_default().push(i);
    }

    let mut used = vec![false; n];
    let mut path = Path::new();
    path.set_fill_type(FillType::Winding);
    // Chains that never returned to their origin, kept as a fallback so a
    // single unmatched endpoint cannot silently delete the whole result.
    let mut partials: Vec<Vec<Point>> = Vec::new();

    for start in 0..n {
        if used[start] {
            continue;
        }
        let mut idx = start;
        let origin = from_id[start];
        let mut verts: Vec<Point> = vec![weld.reps[origin]];
        let mut incoming = edges[idx].to - edges[idx].from;
        let mut closed = false;
        loop {
            used[idx] = true;
            let cur = to_id[idx];
            if cur == origin && verts.len() >= 3 {
                closed = true;
                break;
            }
            verts.push(weld.reps[cur]);
            let Some(cands) = outgoing.get(&cur) else {
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
        if closed && verts.len() >= 3 {
            // A ring enclosing no area is a sliver thrown up where two
            // boundaries nearly touch; it contributes nothing to the fill.
            if signed_area(&verts).abs() > MIN_EDGE * MIN_EDGE {
                emit_contour(&mut path, &verts);
            }
        } else if verts.len() >= 3 {
            partials.push(verts);
        }
    }

    // Only fall back to partial chains if nothing closed. Closing them is a
    // visible approximation, which still beats returning nothing at all.
    if path.is_empty() {
        for verts in partials {
            emit_contour(&mut path, &verts);
        }
    }

    path
}

/// Returns twice the signed area enclosed by `verts` via the shoelace formula.
fn signed_area(verts: &[Point]) -> f32 {
    let mut sum = 0.0;
    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        sum += a.x * b.y - b.x * a.y;
    }
    sum * 0.5
}

/// Appends `verts` to `path` as one closed contour.
fn emit_contour(path: &mut Path, verts: &[Point]) {
    path.move_to(verts[0].x, verts[0].y);
    for v in verts.iter().skip(1) {
        path.line_to(v.x, v.y);
    }
    path.close();
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

    /// Returns the closest distance from `pt` to the conic, sampled densely.
    fn dist_to_conic(pt: Point, conic: &Conic) -> f32 {
        let mut best = f32::INFINITY;
        for i in 0..=2000 {
            let t = i as f32 / 2000.0;
            best = best.min(Point::distance(pt, conic.eval_at(t)));
        }
        best
    }

    #[test]
    fn flatten_conic_stays_on_the_true_curve() {
        // The weight a circle's quarter arc uses; a quad through the same
        // three points bulges noticeably away from it.
        let w = std::f32::consts::FRAC_1_SQRT_2;
        let p0 = Point::new(0.0, 0.0);
        let p1 = Point::new(100.0, 0.0);
        let p2 = Point::new(100.0, 100.0);
        let conic = Conic::new([p0, p1, p2], w);

        let mut segs = Vec::new();
        flatten_conic(p0, p1, p2, w, FLAT_TOL, MAX_FLAT_DEPTH, &mut segs);
        assert!(!segs.is_empty());

        for seg in &segs {
            for &pt in seg {
                assert!(
                    dist_to_conic(pt, &conic) <= FLAT_TOL,
                    "flattened point {pt:?} is {} off the conic",
                    dist_to_conic(pt, &conic)
                );
            }
        }
        // The chain runs end to end along the curve.
        assert_eq!(segs[0][0], p0);
        assert_eq!(segs[segs.len() - 1][1], p2);
    }

    #[test]
    fn flatten_conic_differs_from_flatten_quad_when_weighted() {
        let w = std::f32::consts::FRAC_1_SQRT_2;
        let p0 = Point::new(0.0, 0.0);
        let p1 = Point::new(100.0, 0.0);
        let p2 = Point::new(100.0, 100.0);

        let mut as_conic = Vec::new();
        flatten_conic(p0, p1, p2, w, FLAT_TOL, MAX_FLAT_DEPTH, &mut as_conic);
        let mut as_quad = Vec::new();
        flatten_quad(p0, p1, p2, FLAT_TOL, MAX_FLAT_DEPTH, &mut as_quad);

        // Treating the conic as a quad is what the old code did; the midpoints
        // of the two flattenings must not agree, or the bug would be invisible.
        let conic_mid = as_conic[as_conic.len() / 2][0];
        let quad_mid = as_quad[as_quad.len() / 2][0];
        assert!(
            Point::distance(conic_mid, quad_mid) > 1.0,
            "a weighted conic must not flatten like a quad"
        );
    }

    #[test]
    fn flatten_conic_with_unit_weight_matches_a_quad() {
        // w == 1 is exactly the quadratic, so the two must agree closely.
        let p0 = Point::new(0.0, 0.0);
        let p1 = Point::new(50.0, 100.0);
        let p2 = Point::new(100.0, 0.0);
        let quad = Conic::new([p0, p1, p2], 1.0);

        let mut segs = Vec::new();
        flatten_conic(p0, p1, p2, 1.0, FLAT_TOL, MAX_FLAT_DEPTH, &mut segs);
        for seg in &segs {
            for &pt in seg {
                assert!(dist_to_conic(pt, &quad) <= FLAT_TOL);
            }
        }
    }

    #[test]
    fn boolean_ops_on_discs_are_not_empty() {
        // Item 10's repro: every curved boolean returned empty because the
        // conic weight was dropped when flattening.
        let mut a = Path::new();
        a.add_circle(200.0, 200.0, 40.0);
        let mut b = Path::new();
        b.add_circle(230.0, 200.0, 40.0);
        for op in [PathOp::Union, PathOp::Intersect, PathOp::Difference] {
            let result = path_op(&a, &b, op).unwrap();
            assert!(!result.is_empty(), "{op:?} of two discs came back empty");
        }
    }

    #[test]
    fn union_of_a_disc_and_a_rect_is_not_empty() {
        let mut disc = Path::new();
        disc.add_circle(200.0, 200.0, 40.0);
        let mut rect = Path::new();
        rect.add_rect_simple(crate::core::Rect::from_ltrb(180.0, 180.0, 260.0, 260.0));
        let result = path_op(&disc, &rect, PathOp::Union).unwrap();
        assert!(!result.is_empty());
        assert!(result.contains(200.0, 200.0));
        assert!(result.contains(250.0, 250.0));
    }

    /// Builds a regular n-gon centred at `(cx, cy)`.
    fn ngon(cx: f32, cy: f32, r: f32, n: usize) -> Path {
        let mut p = Path::new();
        for i in 0..n {
            let a = i as f32 / n as f32 * std::f32::consts::TAU;
            let (x, y) = (cx + r * a.cos(), cy + r * a.sin());
            if i == 0 {
                p.move_to(x, y);
            } else {
                p.line_to(x, y);
            }
        }
        p.close();
        p
    }

    fn contour_count(p: &Path) -> usize {
        p.iter().filter(|(v, _, _)| *v == Verb::Move).count()
    }

    #[test]
    fn union_of_many_vertex_polygons_stays_one_contour() {
        // Item 15: assemble chained edges on a fixed 1/256 quantization grid,
        // so once edges got short enough their endpoints stopped agreeing and
        // whole contours were dropped, returning empty.
        for n in [4usize, 6, 8, 12, 16, 24, 32, 48, 64, 128] {
            let a = ngon(200.0, 200.0, 40.0, n);
            let b = ngon(205.0, 200.0, 40.0, n);
            let u = path_op(&a, &b, PathOp::Union).unwrap();
            assert!(!u.is_empty(), "union of two {n}-gons came back empty");
            assert_eq!(contour_count(&u), 1, "union of two {n}-gons split apart");
        }
    }

    #[test]
    fn union_of_polygons_is_scale_invariant() {
        // The bug vanished purely by scaling the scene up, which is what
        // pinned it on an absolute tolerance rather than on the geometry.
        for r in [10.0f32, 40.0, 100.0, 400.0, 1000.0] {
            let a = ngon(200.0, 200.0, r, 64);
            let b = ngon(205.0, 200.0, r, 64);
            let u = path_op(&a, &b, PathOp::Union).unwrap();
            assert_eq!(contour_count(&u), 1, "union at radius {r} split apart");
        }
    }

    #[test]
    fn union_of_polygons_holds_across_offsets() {
        // Offsets used to fail non-monotonically (8 and 10 worked, 15 did not),
        // the signature of a threshold being straddled.
        for off in [0.5f32, 1.0, 2.0, 5.0, 8.0, 10.0, 15.0, 20.0] {
            let a = ngon(200.0, 200.0, 40.0, 64);
            let b = ngon(200.0 + off, 200.0, 40.0, 64);
            let u = path_op(&a, &b, PathOp::Union).unwrap();
            assert_eq!(contour_count(&u), 1, "union at offset {off} split apart");
        }
    }

    #[test]
    fn union_keeps_coincident_shared_edges() {
        // Two hexagons offset along x share horizontal collinear edges. Those
        // edges have a coincident twin, and treating the twin as a neighbour
        // drives the sampling step to zero and loses the edge, which breaks
        // the ring that runs through it.
        let a = ngon(200.0, 200.0, 40.0, 6);
        let b = ngon(205.0, 200.0, 40.0, 6);
        let u = path_op(&a, &b, PathOp::Union).unwrap();
        assert_eq!(contour_count(&u), 1);
        assert!(u.contains(200.0, 200.0));
    }

    #[test]
    fn union_of_offset_discs_is_one_contour() {
        // Offset 0.5 is deliberately absent: it still fragments into 4
        // contours (6 for intersect). That is the substitute engine flattening
        // two arcs whose separation is below the flattening deviation, and it
        // goes away with the engine itself — see
        // `TODO/16-union-of-near-coincident-discs-fragments.md`.
        for off in [1.0f32, 2.0, 5.0, 20.0, 60.0] {
            let mut a = Path::new();
            a.add_circle(200.0, 200.0, 40.0);
            let mut b = Path::new();
            b.add_circle(200.0 + off, 200.0, 40.0);
            let u = path_op(&a, &b, PathOp::Union).unwrap();
            assert_eq!(contour_count(&u), 1, "disc union at offset {off} split apart");
        }
    }

    #[test]
    fn vertex_weld_groups_near_identical_endpoints() {
        // Two edges meeting at a vertex whose coordinates differ in the last
        // few bits must resolve to a single vertex id.
        let edges = vec![
            Edge {
                from: Point::new(0.0, 0.0),
                to: Point::new(100.0, 0.0),
            },
            Edge {
                from: Point::new(100.000_01, 0.0),
                to: Point::new(100.0, 100.0),
            },
        ];
        let weld = VertexWeld::new(&edges);
        assert_eq!(weld.id_of(edges[0].to), weld.id_of(edges[1].from));
        // Genuinely distinct endpoints stay distinct.
        assert_ne!(weld.id_of(edges[0].from), weld.id_of(edges[1].to));
    }

    #[test]
    fn signed_area_is_zero_for_a_degenerate_ring() {
        let flat = [
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0),
        ];
        assert!(signed_area(&flat).abs() < 1e-9);
        let square = [
            Point::new(0.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(2.0, 2.0),
            Point::new(0.0, 2.0),
        ];
        assert!((signed_area(&square).abs() - 4.0).abs() < 1e-6);
    }
}
