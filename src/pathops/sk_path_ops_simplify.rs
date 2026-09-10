//! Path simplification: rewrites a path so its contours no longer overlap or
//! self-intersect, while filling exactly the same region.
//!
//! Port of Skia's `SkPathOpsSimplify.cpp`.
//!
//! The result always uses an even-odd fill rule (inverse-even-odd for inverse
//! input), matching Skia. Because the output contours are disjoint and
//! non-self-intersecting, even-odd and winding agree on it anyway; even-odd is
//! what Skia promises, so that is what we set.
//!
//! # Relationship to the C++ original
//!
//! Skia's `SimplifyDebug` drives its op-segment engine: `SkOpEdgeBuilder`
//! converts the path to contours, `AddIntersectTs` finds every crossing,
//! `HandleCoincidence` resolves overlapping runs, and then `bridgeWinding` (for
//! winding-masked input) or `bridgeXor` (for even-odd input) walks the segment
//! graph emitting closed contours.
//!
//! That engine is not yet ported in this crate (see `sk_op_segment`,
//! `sk_op_coincidence`), so the same pipeline is realized here on the flattened
//! edge representation that [`super::boolean`] uses: curves are flattened,
//! every edge is split at all intersections, each resulting piece is kept only
//! if it lies on the boundary of the filled region, and the survivors are
//! walked into closed contours. The staging mirrors the original —
//! convex fast path, build, intersect, classify by fill mask, bridge, assemble
//! leftovers — and `bridge_winding` / `bridge_xor` correspond to the two
//! branches Skia selects on `builder.xorMask()`.


use crate::core::{Conic, FillType, Path, Point, Scalar, Verb};

/// Flatness tolerance, in path units, for subdividing curves into line
/// segments.
const FLAT_TOL: f32 = 0.1;

/// Maximum recursion depth when flattening a curve.
const MAX_FLAT_DEPTH: u32 = 16;

/// Edges shorter than this are dropped as degenerate.
const MIN_EDGE: f32 = 1e-4;

/// Distance to step perpendicular to an edge when sampling which side is
/// inside the filled region.
const OFFSET: f32 = 0.25;

/// Smallest perpendicular offset tried when sampling an edge's two sides.
///
/// Below this the two sample points are close enough to the edge that
/// `contains` can no longer tell them apart from a point on the boundary
/// itself, so shrinking further buys nothing.
const PROBE_MIN: f32 = 1e-3;

/// Factor by which the sampling offset shrinks on each retry.
const PROBE_SHRINK: f32 = 0.25;

/// Parametric tolerance for merging near-duplicate split points.
const T_EPS: f32 = 1e-6;

/// Quantization step for matching endpoints when assembling contours.
const WELD: f32 = 256.0;

/// Smallest area an unclosed run must enclose before it is closed up and kept
/// rather than discarded.
const MIN_AREA: f32 = 1e-3;

/// Simplifies `path`, returning a path that fills the same region with
/// non-overlapping, non-self-intersecting contours.
///
/// Corresponds to Skia's `Simplify(const SkPath&, SkPath*)`.
///
/// The returned path uses [`FillType::EvenOdd`], or
/// [`FillType::InverseEvenOdd`] when `path` has an inverse fill type. Convex
/// paths are returned unchanged apart from that fill type, since a convex
/// contour cannot self-intersect.
///
/// # Errors
///
/// Returns `Err` if `path` contains non-finite coordinates. Paths that the
/// engine cannot fully resolve do not error; they degrade to a partially
/// assembled result, as in Skia.
///
/// # Examples
///
/// ```
/// use pathkit::core::Path;
/// use pathkit::pathops::sk_path_ops_simplify::simplify;
///
/// // A bowtie: one contour that crosses itself.
/// let mut path = Path::new();
/// path.move_to(0.0, 0.0);
/// path.line_to(10.0, 10.0);
/// path.line_to(10.0, 0.0);
/// path.line_to(0.0, 10.0);
/// path.close();
///
/// let simplified = simplify(&path).unwrap();
/// // The two lobes lie left and right of the crossing at (5, 5).
/// assert!(simplified.contains(1.0, 5.0));
/// assert!(simplified.contains(9.0, 5.0));
/// assert!(!simplified.contains(5.0, 1.0));
/// ```
pub fn simplify(path: &Path) -> Result<Path, String> {
    let mut result = Path::new();
    if simplify_debug(path, &mut result, None) {
        Ok(result)
    } else {
        Err("simplify failed".to_string())
    }
}

/// Simplifies `path` into `result`, returning whether the operation succeeded.
///
/// Corresponds to Skia's `SimplifyDebug`. `test_name` is accepted for parity
/// with the C++ debug parameter and is used only in failure messages.
///
/// Unlike [`simplify`], this reports failure through its return value and
/// leaves a best-effort path in `result`.
pub fn simplify_debug(path: &Path, result: &mut Path, _test_name: Option<&str>) -> bool {
    // Returns even-odd for normal fills and inverse-even-odd for inverse
    // fills, regardless of whether the input was winding or even-odd.
    let fill_type = if path.is_inverse_fill_type() {
        FillType::InverseEvenOdd
    } else {
        FillType::EvenOdd
    };

    if !path.is_finite() {
        return false;
    }

    // A convex contour never crosses itself and never overlaps another, so
    // there is nothing to resolve. Skia takes the same shortcut.
    if is_convex(path) {
        *result = path.clone();
        result.set_fill_type(fill_type);
        return true;
    }

    // Turn the path into a list of edges. Stands in for SkOpEdgeBuilder.
    let edges = build_edges(path);
    if edges.is_empty() {
        result.reset();
        result.set_fill_type(fill_type);
        return true;
    }

    // Find all intersections between edges and split there, so that no two
    // pieces cross except at shared endpoints. Stands in for AddIntersectTs
    // plus HandleCoincidence.
    let pieces = split_at_intersections(&edges);
    if pieces.is_empty() {
        result.reset();
        result.set_fill_type(fill_type);
        return true;
    }

    // Construct closed contours. Which fill mask the input used decides how a
    // piece is judged to be on the boundary, exactly as Skia picks between
    // bridgeWinding and bridgeXor on the edge builder's xor mask.
    result.reset();
    result.set_fill_type(fill_type);

    let boundary = if path.fill_type().is_even_odd() {
        bridge_xor(&pieces, path)
    } else {
        bridge_winding(&pieces, path)
    };

    if boundary.is_empty() {
        return true;
    }

    *result = assemble(&boundary, fill_type);
    true
}

/// A directed boundary edge of the simplified result.
///
/// The direction is chosen so the filled region lies to the edge's left, which
/// is what lets [`assemble`] chain edges into consistently wound contours.
/// A boundary edge identified by its quantized endpoints, used to spot
/// coincident pieces.
type EdgeKey = ((i32, i32), (i32, i32));

#[derive(Clone, Copy, Debug)]
struct Edge {
    /// Start point.
    from: Point,
    /// End point.
    to: Point,
}

/// Returns true if every turn in `path` has the same sign and the path has a
/// single contour, i.e. the path is convex.
///
/// Stands in for Skia's `SkPath::isConvex()`, which this crate's `Path` does
/// not expose. Curves are flattened first so control-point turns count.
fn is_convex(path: &Path) -> bool {
    let mut contours = 0;
    for (verb, _, _) in path.iter() {
        if verb == Verb::Move {
            contours += 1;
            if contours > 1 {
                return false;
            }
        }
    }
    if contours == 0 {
        // An empty path is trivially convex; Skia reports the same.
        return true;
    }

    let pts = contour_points(path);
    if pts.len() < 3 {
        return true;
    }

    let n = pts.len();
    let mut sign = 0i32;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let c = pts[(i + 2) % n];
        let cross = (b - a).cross(c - b);
        if cross.abs() <= 1e-9 {
            continue;
        }
        let s = if cross > 0.0 { 1 } else { -1 };
        if sign == 0 {
            sign = s;
        } else if sign != s {
            return false;
        }
    }
    true
}

/// Collects the vertices of a single-contour path, flattening any curves.
fn contour_points(path: &Path) -> Vec<Point> {
    let mut pts: Vec<Point> = Vec::new();
    let mut push = |p: Point| {
        if pts.last().map_or(true, |last| Point::distance(*last, p) >= MIN_EDGE) {
            pts.push(p);
        }
    };

    for (verb, p, _) in path.iter() {
        match verb {
            Verb::Move => push(p[0]),
            Verb::Line => push(p[1]),
            Verb::Quad | Verb::Conic => {
                push(p[1]);
                push(p[2]);
            }
            Verb::Cubic => {
                push(p[1]);
                push(p[2]);
                push(p[3]);
            }
            Verb::Close => {}
        }
    }

    // Drop a trailing duplicate of the start point; the wrap-around in
    // is_convex covers the closing turn already.
    if pts.len() >= 2 {
        let first = pts[0];
        if Point::distance(*pts.last().unwrap(), first) < MIN_EDGE {
            pts.pop();
        }
    }
    pts
}

/// Converts `path` into a flat list of line segments, closing every contour.
///
/// Stands in for `SkOpEdgeBuilder::finish()`: open contours are implicitly
/// closed, because a fill region is defined by closed boundaries.
fn build_edges(path: &Path) -> Vec<[Point; 2]> {
    let mut segs = Vec::new();
    let mut contour_start = Point::default();
    let mut last = Point::default();
    let mut started = false;

    let close_contour = |segs: &mut Vec<[Point; 2]>, last: Point, start: Point| {
        push_seg(segs, last, start);
    };

    for (verb, pts, weight) in path.iter() {
        match verb {
            Verb::Move => {
                if started {
                    close_contour(&mut segs, last, contour_start);
                }
                contour_start = pts[0];
                last = pts[0];
                started = true;
            }
            Verb::Line => {
                push_seg(&mut segs, pts[0], pts[1]);
                last = pts[1];
            }
            Verb::Quad => {
                flatten_quad(pts[0], pts[1], pts[2], MAX_FLAT_DEPTH, &mut segs);
                last = pts[2];
            }
            Verb::Conic => {
                flatten_conic(
                    pts[0],
                    pts[1],
                    pts[2],
                    weight.unwrap_or(1.0),
                    MAX_FLAT_DEPTH,
                    &mut segs,
                );
                last = pts[2];
            }
            Verb::Cubic => {
                flatten_cubic(pts[0], pts[1], pts[2], pts[3], MAX_FLAT_DEPTH, &mut segs);
                last = pts[3];
            }
            Verb::Close => {
                if started {
                    close_contour(&mut segs, last, contour_start);
                    last = contour_start;
                }
            }
        }
    }
    if started {
        close_contour(&mut segs, last, contour_start);
    }
    segs
}

/// Appends `a`-`b` to `segs` unless it is degenerately short.
fn push_seg(segs: &mut Vec<[Point; 2]>, a: Point, b: Point) {
    if Point::distance(a, b) >= MIN_EDGE {
        segs.push([a, b]);
    }
}

/// Recursively subdivides a quadratic until it is flat enough to replace with
/// a chord.
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

/// Recursively subdivides a cubic until it is flat enough to replace with a
/// chord.
/// Flattens a conic into chords, honouring its weight.
///
/// A conic with `w != 1` traces a different curve than the quadratic through
/// the same three points, so it cannot be flattened as a quad: the flattened
/// boundary would disagree with `Path::contains`, which evaluates the conic
/// correctly, and every edge would then be classified against the wrong side.
///
/// Subdivision uses [`Conic::chop`], which splits in the rational form and so
/// keeps both halves on the original curve.
fn flatten_conic(p0: Point, p1: Point, p2: Point, w: Scalar, depth: u32, out: &mut Vec<[Point; 2]>) {
    // A weight of 1 is exactly the quadratic, and the control point's distance
    // to the chord bounds the error of the straight-line approximation.
    if depth == 0 || dist_to_line(p1, p0, p2) <= FLAT_TOL {
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
        depth - 1,
        out,
    );
    flatten_conic(
        halves[1].pts[0],
        halves[1].pts[1],
        halves[1].pts[2],
        halves[1].w,
        depth - 1,
        out,
    );
}

fn flatten_cubic(p0: Point, p1: Point, p2: Point, p3: Point, depth: u32, out: &mut Vec<[Point; 2]>) {
    if depth == 0 || (dist_to_line(p1, p0, p3) <= FLAT_TOL && dist_to_line(p2, p0, p3) <= FLAT_TOL) {
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

/// Returns the midpoint of `a` and `b`.
fn mid(a: Point, b: Point) -> Point {
    Point::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5)
}

/// Returns the perpendicular distance from `p` to the line through `a` and
/// `b`.
fn dist_to_line(p: Point, a: Point, b: Point) -> f32 {
    let ab = b - a;
    let len = ab.length();
    if len < 1e-12 {
        return Point::distance(p, a);
    }
    ab.cross(p - a).abs() / len
}

/// Splits every segment at each point where it meets another, so the returned
/// pieces intersect only at shared endpoints.
///
/// Stands in for `AddIntersectTs` followed by `HandleCoincidence`: collinear
/// overlaps are handled by projecting each segment's endpoints onto the other,
/// which splits coincident runs at their shared boundaries.
fn split_at_intersections(segs: &[[Point; 2]]) -> Vec<[Point; 2]> {
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

/// Returns the interior parametric split points that `a` and `b` induce on
/// each other.
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
        // Collinear: split each segment wherever the other one starts or ends.
        for &p in &[b[0], b[1]] {
            if let Some(t) = project_t(a, p) {
                if t > T_EPS && t < 1.0 - T_EPS {
                    ta.push(t);
                }
            }
        }
        for &p in &[a[0], a[1]] {
            if let Some(u) = project_t(b, p) {
                if u > T_EPS && u < 1.0 - T_EPS {
                    tb.push(u);
                }
            }
        }
    }
    (ta, tb)
}

/// Returns the parametric position of `p` projected onto `seg`.
fn project_t(seg: [Point; 2], p: Point) -> Option<f32> {
    let d = seg[1] - seg[0];
    let len2 = d.dot(d);
    if len2 < 1e-16 {
        return None;
    }
    Some((p - seg[0]).dot(d) / len2)
}

/// Linearly interpolates between `a` and `b`.
fn lerp(a: Point, b: Point, t: f32) -> Point {
    Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

/// Keeps the pieces that bound the region filled under the winding rule,
/// orienting each so the filled side is on its left.
///
/// Corresponds to Skia's `bridgeWinding`.
fn bridge_winding(pieces: &[[Point; 2]], path: &Path) -> Vec<Edge> {
    collect_boundary(pieces, path, false)
}

/// Keeps the pieces that bound the region filled under the even-odd rule,
/// orienting each so the filled side is on its left.
///
/// Corresponds to Skia's `bridgeXor`.
fn bridge_xor(pieces: &[[Point; 2]], path: &Path) -> Vec<Edge> {
    collect_boundary(pieces, path, true)
}

/// Selects the pieces whose two sides disagree about being inside the fill.
///
/// A piece with fill on both sides is interior to the region and a piece with
/// fill on neither is exterior; only pieces that separate the two are part of
/// the simplified outline. This replaces the winding-sum bookkeeping that
/// Skia's segment graph performs.
fn collect_boundary(pieces: &[[Point; 2]], path: &Path, even_odd: bool) -> Vec<Edge> {
    // Sample against a copy carrying the requested rule, with any inverse
    // stripped: the boundary is the same curve either way, and testing the
    // non-inverted region keeps the left-is-inside convention below correct.
    let mut probe = path.clone();
    probe.set_fill_type(if even_odd {
        FillType::EvenOdd
    } else {
        FillType::Winding
    });

    let mut edges = Vec::new();
    for &seg in pieces {
        let dir = seg[1] - seg[0];
        let len = dir.length();
        if len < MIN_EDGE {
            continue;
        }
        let mid_pt = mid(seg[0], seg[1]);
        // Step perpendicular to the piece, scaled down for very short pieces
        // so the sample points stay near this edge rather than landing across
        // a neighbouring one.
        //
        // A single fixed offset cannot classify a piece bounding a region
        // narrower than the step: both samples land outside and the piece is
        // discarded, which breaks the ring the pieces form and leaves
        // `assemble` with nothing it can close. Retry at successively finer
        // offsets so slivers are resolved at whatever width they actually
        // have.
        let unit = Point::new(-dir.y / len, dir.x / len);
        let mut step = OFFSET.min(len * 0.5);
        let mut sides = None;
        while step >= PROBE_MIN {
            let n = Point::new(unit.x * step, unit.y * step);
            let l = probe.contains((mid_pt + n).x, (mid_pt + n).y);
            let r = probe.contains((mid_pt - n).x, (mid_pt - n).y);
            if l != r {
                sides = Some(l);
                break;
            }
            step *= PROBE_SHRINK;
        }
        let Some(in_left) = sides else {
            continue;
        };
        if in_left {
            edges.push(Edge {
                from: seg[0],
                to: seg[1],
            });
        } else {
            edges.push(Edge {
                from: seg[1],
                to: seg[0],
            });
        }
    }
    dedup_coincident(edges)
}

/// Collapses coincident boundary edges down to one edge each.
///
/// Two contours that share an edge produce two pieces occupying the same
/// space; both pass the boundary test, and emitting both would yield
/// duplicated contours that cancel under the even-odd output rule. Skia folds
/// these together in `HandleCoincidence` before bridging; this does the
/// equivalent on the split pieces.
///
/// Edges that coincide but run in opposite directions are dropped entirely:
/// they are back-to-back boundaries of regions that both turned out to be
/// filled, so the shared edge is interior to the result.
fn dedup_coincident(edges: Vec<Edge>) -> Vec<Edge> {
    let mut seen: std::collections::HashMap<EdgeKey, usize> = std::collections::HashMap::new();
    let mut keep = vec![true; edges.len()];

    for (i, e) in edges.iter().enumerate() {
        let fwd = (key(e.from), key(e.to));
        let rev = (fwd.1, fwd.0);
        if let Some(j) = seen.remove(&rev) {
            // Opposing pair: neither edge bounds the result.
            keep[i] = false;
            keep[j] = false;
        } else if let std::collections::hash_map::Entry::Vacant(slot) = seen.entry(fwd) {
            slot.insert(i);
        } else {
            // Same-direction duplicate: keep only the first.
            keep[i] = false;
        }
    }

    edges
        .into_iter()
        .zip(keep)
        .filter_map(|(e, k)| k.then_some(e))
        .collect()
}

/// Twice the signed area enclosed by the closed polygon through `verts`.
///
/// Positive for counter-clockwise winding in a y-up frame. Used only for its
/// magnitude, to tell a run that bounds real fill from one that doubles back
/// on itself and encloses nothing.
fn signed_area(verts: &[Point]) -> f32 {
    let mut acc = 0.0;
    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        acc += a.cross(b);
    }
    acc * 0.5
}

/// Quantizes a point so endpoints that should coincide compare equal.
fn key(p: Point) -> (i32, i32) {
    ((p.x * WELD).round() as i32, (p.y * WELD).round() as i32)
}

/// Chains boundary edges into closed contours and writes them to a path.
///
/// At each vertex the most sharply left-turning unused edge is taken, which
/// walks the outline of the region rather than cutting across it.
///
/// A run that never returns to its origin is closed by joining its two ends,
/// provided it encloses real area. This mirrors Skia's
/// `SkPathWriter::assemble`, which links leftover partial contours to their
/// nearest free endpoints instead of discarding them: dropping such a run
/// would delete a filled region the input actually had, and a single dropped
/// piece anywhere on a ring would otherwise take the whole contour with it.
/// Runs that enclose no area are still dropped, since they contribute no fill.
fn assemble(edges: &[Edge], fill_type: FillType) -> Path {
    let n = edges.len();
    let mut outgoing: std::collections::HashMap<(i32, i32), Vec<usize>> =
        std::collections::HashMap::new();
    for (i, e) in edges.iter().enumerate() {
        outgoing.entry(key(e.from)).or_default().push(i);
    }

    let mut used = vec![false; n];
    let mut path = Path::new();
    path.set_fill_type(fill_type);

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
                let ang = incoming.cross(out).atan2(incoming.dot(out));
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
        if verts.len() < 3 {
            continue;
        }
        if !closed && signed_area(&verts).abs() < MIN_AREA {
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

    #[test]
    fn test_simplify_empty_path() {
        let path = Path::new();
        let result = simplify(&path).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn test_simplify_simple_rect() {
        let mut path = Path::new();
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));

        let result = simplify(&path).unwrap();
        assert!(!result.is_empty());
        assert!(result.contains(5.0, 5.0));
        assert!(!result.contains(15.0, 5.0));
    }

    #[test]
    fn test_simplify_convex_path() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(5.0, 10.0);
        path.close();

        let result = simplify(&path).unwrap();
        assert!(!result.is_empty());
    }

    #[test]
    fn test_simplify_overlapping_rects() {
        let mut path = Path::new();
        // Two overlapping rectangles
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        path.add_rect_simple(crate::core::Rect::from_ltrb(5.0, 5.0, 15.0, 15.0));

        let result = simplify(&path).unwrap();
        assert!(!result.is_empty());
        // The union region survives, including the overlap, which under the
        // input's default winding rule was filled.
        assert!(result.contains(2.0, 2.0));
        assert!(result.contains(7.0, 7.0));
        assert!(result.contains(12.0, 12.0));
        assert!(!result.contains(12.0, 2.0));
        assert!(!result.contains(2.0, 12.0));
    }

    #[test]
    fn test_simplify_with_inverse_fill() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close();
        path.set_fill_type(FillType::InverseWinding);

        let result = simplify(&path).unwrap();
        assert!(!result.is_empty());
        assert_eq!(result.fill_type(), FillType::InverseEvenOdd);
    }

    #[test]
    fn test_simplify_preserves_convex() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(5.0, 10.0);
        path.close();

        let result = simplify(&path).unwrap();
        // For convex paths, should preserve the shape: a triangle has 3
        // points (move_to + 2 line_to); close() doesn't add a 4th.
        assert_eq!(result.points().len(), 3);
    }

    #[test]
    fn test_simplify_even_odd_fill() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close();
        path.set_fill_type(FillType::EvenOdd);

        let result = simplify(&path).unwrap();
        assert!(!result.is_empty());
        assert_eq!(result.fill_type(), FillType::EvenOdd);
    }

    #[test]
    fn test_simplify_non_convex_basic() {
        let mut path = Path::new();
        // Non-convex shape (L-shape)
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 5.0);
        path.line_to(5.0, 5.0);
        path.line_to(5.0, 10.0);
        path.line_to(0.0, 10.0);
        path.close();

        let result = simplify(&path).unwrap();
        assert!(!result.is_empty());
        // The notch stays empty and the arms stay filled.
        assert!(result.contains(2.0, 2.0));
        assert!(result.contains(8.0, 2.0));
        assert!(result.contains(2.0, 8.0));
        assert!(!result.contains(8.0, 8.0));
    }

    #[test]
    fn test_simplify_debug() {
        let mut path = Path::new();
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));

        let mut result = Path::new();
        let success = simplify_debug(&path, &mut result, Some("test"));
        assert!(success);
        assert!(!result.is_empty());
    }

    #[test]
    fn simplify_result_is_even_odd() {
        let mut path = Path::new();
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        path.set_fill_type(FillType::Winding);
        let result = simplify(&path).unwrap();
        assert_eq!(result.fill_type(), FillType::EvenOdd);
    }

    #[test]
    fn simplify_self_intersecting_bowtie() {
        // A bowtie crosses itself at (5, 5). Simplified, it becomes two
        // triangles and the crossing point is no longer interior to an edge.
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 10.0);
        path.line_to(10.0, 0.0);
        path.line_to(0.0, 10.0);
        path.close();

        let result = simplify(&path).unwrap();
        assert!(!result.is_empty());
        // This winding order puts the two lobes left and right of the
        // crossing, so those are the filled regions.
        assert!(result.contains(1.0, 5.0));
        assert!(result.contains(9.0, 5.0));
        // Above and below the crossing is outside the bowtie.
        assert!(!result.contains(5.0, 1.5));
        assert!(!result.contains(5.0, 8.5));
    }

    #[test]
    fn simplify_nested_rects_winding_fills_hole() {
        // Same-direction nested rectangles: under winding both wind +1 and
        // +2, so the whole outer rect is filled and the inner ring vanishes.
        let mut path = Path::new();
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 30.0, 30.0));
        path.add_rect_simple(crate::core::Rect::from_ltrb(10.0, 10.0, 20.0, 20.0));
        path.set_fill_type(FillType::Winding);

        let result = simplify(&path).unwrap();
        assert!(result.contains(5.0, 5.0));
        assert!(result.contains(15.0, 15.0));
        assert!(!result.contains(35.0, 15.0));
    }

    #[test]
    fn simplify_nested_rects_even_odd_keeps_hole() {
        // The same geometry under even-odd is a frame with a hole, and
        // simplify must preserve the hole.
        let mut path = Path::new();
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 30.0, 30.0));
        path.add_rect_simple(crate::core::Rect::from_ltrb(10.0, 10.0, 20.0, 20.0));
        path.set_fill_type(FillType::EvenOdd);

        let result = simplify(&path).unwrap();
        assert!(result.contains(5.0, 5.0));
        assert!(!result.contains(15.0, 15.0));
        assert!(!result.contains(35.0, 15.0));
    }

    #[test]
    fn simplify_coincident_edges() {
        // Two rectangles sharing the edge x = 10 merge into one rectangle.
        let mut path = Path::new();
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        path.add_rect_simple(crate::core::Rect::from_ltrb(10.0, 0.0, 20.0, 10.0));

        let result = simplify(&path).unwrap();
        assert!(result.contains(5.0, 5.0));
        assert!(result.contains(15.0, 5.0));
        // The shared edge is interior now, so points just either side of it
        // are both filled.
        assert!(result.contains(9.5, 5.0));
        assert!(result.contains(10.5, 5.0));
        assert!(!result.contains(25.0, 5.0));
    }

    #[test]
    fn simplify_duplicate_contours_winding() {
        // The same rectangle twice: winding sums to 2, still filled.
        let mut path = Path::new();
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        path.set_fill_type(FillType::Winding);

        let result = simplify(&path).unwrap();
        assert!(result.contains(5.0, 5.0));
        assert!(!result.contains(15.0, 5.0));
    }

    #[test]
    fn simplify_rejects_non_finite() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(f32::NAN, 10.0);
        path.line_to(10.0, 10.0);
        path.close();

        assert!(simplify(&path).is_err());
    }

    #[test]
    fn simplify_curved_contour_is_preserved() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.quad_to(50.0, 100.0, 100.0, 0.0);
        path.close();

        let result = simplify(&path).unwrap();
        assert!(!result.is_empty());
        // The quad bulges toward +y, so the filled lobe sits below the chord
        // that closes the contour.
        assert!(result.contains(50.0, 25.0));
        assert!(result.contains(50.0, 45.0));
        assert!(!result.contains(50.0, -10.0));
        assert!(!result.contains(5.0, 40.0));
    }

    /// A contour ending in a near-degenerate spike used to vanish entirely.
    ///
    /// The spike's two flanks bound a region thinner than the fixed
    /// perpendicular offset `collect_boundary` sampled at, so both samples read
    /// "outside" and the piece was discarded. That broke the ring of boundary
    /// pieces, and `assemble`, which kept only runs that returned to their
    /// origin, then emitted nothing at all.
    #[test]
    fn simplify_keeps_contour_with_degenerate_spike() {
        // Reduced from a font glyph's swept stroke: a body with a hairline
        // spike at the tail.
        const PTS: &[(f32, f32)] = &[
            (265.2254, 646.6782),
            (293.8087, 654.6358),
            (324.1818, 659.5676),
            (355.7458, 661.2591),
            (387.9075, 659.524),
            (419.1481, 654.2556),
            (448.0477, 645.9188),
            (474.1282, 634.8705),
            (496.9165, 621.5131),
            (516.2436, 606.6158),
            (531.5778, 590.3043),
            (542.7201, 573.075),
            (549.6874, 555.2849),
            (552.4633, 537.5287),
            (551.3721, 520.1179),
            (546.4543, 502.7428),
            (537.4683, 485.4285),
            (524.5909, 468.1683),
            (507.2624, 451.7708),
            (485.5978, 436.8444),
            (459.8072, 423.953),
            (450.3936, 421.8325),
            (429.5548, 418.3125),
            (403.0803, 414.4648),
            (374.814, 410.7244),
            (348.3902, 407.4814),
            (327.4115, 405.1188),
            (315.6939, 404.0379),
            (322.0133, 356.9807),
            (321.526, 360.3026),
        ];

        let mut path = Path::new();
        path.move_to(PTS[0].0, PTS[0].1);
        for p in &PTS[1..] {
            path.line_to(p.0, p.1);
        }
        path.close();

        let simplified = simplify(&path).unwrap();
        assert!(
            !simplified.is_empty(),
            "a contour with a large filled area must not simplify to nothing"
        );

        // The fill must be preserved, not merely non-empty. Sample the
        // bounding box and require every filled point to survive.
        for i in 0..120 {
            for j in 0..120 {
                let x = 250.0 + 320.0 * (i as f32 + 0.5) / 120.0;
                let y = 340.0 + 340.0 * (j as f32 + 0.5) / 120.0;
                if path.contains(x, y) {
                    assert!(
                        simplified.contains(x, y),
                        "point ({x}, {y}) was filled before simplify and is not after"
                    );
                }
            }
        }
    }

    /// A wedge whose interior is far thinner than the default sampling offset.
    #[test]
    fn simplify_keeps_sliver_thinner_than_probe_offset() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        path.line_to(100.0, 0.01);
        path.line_to(0.0, 0.02);
        path.close();

        let simplified = simplify(&path).unwrap();
        assert!(!simplified.is_empty());
    }

    /// A closed contour whose final segment cuts back across the body.
    #[test]
    fn simplify_contour_whose_close_crosses_body() {
        let mut path = Path::new();
        path.move_to(100.0, 100.0);
        path.line_to(300.0, 100.0);
        path.line_to(300.0, 300.0);
        path.line_to(150.0, 300.0);
        path.line_to(150.0, 200.0);
        path.line_to(250.0, 200.0);
        path.line_to(250.0, 50.0);
        path.close();

        let simplified = simplify(&path).unwrap();
        assert!(!simplified.is_empty());

        // Every point the input filled is still filled.
        for i in 0..80 {
            for j in 0..80 {
                let x = 90.0 + 230.0 * (i as f32 + 0.5) / 80.0;
                let y = 40.0 + 280.0 * (j as f32 + 0.5) / 80.0;
                if path.contains(x, y) {
                    assert!(simplified.contains(x, y), "lost fill at ({x}, {y})");
                }
            }
        }
    }

    #[test]
    fn signed_area_matches_shoelace() {
        // Unit square, counter-clockwise in a y-up frame.
        let square = [
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ];
        assert!((signed_area(&square) - 1.0).abs() < 1e-6);

        // Reversing the winding flips the sign.
        let mut reversed = square;
        reversed.reverse();
        assert!((signed_area(&reversed) + 1.0).abs() < 1e-6);

        // A run doubling back on itself encloses nothing.
        let hair = [
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 0.0),
        ];
        assert!(signed_area(&hair).abs() < 1e-6);
    }

    #[test]
    fn simplify_is_idempotent() {
        let mut path = Path::new();
        path.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
        path.add_rect_simple(crate::core::Rect::from_ltrb(5.0, 5.0, 15.0, 15.0));

        let once = simplify(&path).unwrap();
        let twice = simplify(&once).unwrap();
        for &(x, y) in &[
            (2.0, 2.0),
            (7.0, 7.0),
            (12.0, 12.0),
            (12.0, 2.0),
            (2.0, 12.0),
        ] {
            assert_eq!(once.contains(x, y), twice.contains(x, y), "at ({x}, {y})");
        }
    }

    #[test]
    fn is_convex_detects_shapes() {
        let mut tri = Path::new();
        tri.move_to(0.0, 0.0);
        tri.line_to(10.0, 0.0);
        tri.line_to(5.0, 10.0);
        tri.close();
        assert!(is_convex(&tri));

        let mut ell = Path::new();
        ell.move_to(0.0, 0.0);
        ell.line_to(10.0, 0.0);
        ell.line_to(10.0, 5.0);
        ell.line_to(5.0, 5.0);
        ell.line_to(5.0, 10.0);
        ell.line_to(0.0, 10.0);
        ell.close();
        assert!(!is_convex(&ell));

        let mut two = Path::new();
        two.add_rect_simple(crate::core::Rect::from_ltrb(0.0, 0.0, 1.0, 1.0));
        two.add_rect_simple(crate::core::Rect::from_ltrb(5.0, 5.0, 6.0, 6.0));
        assert!(!is_convex(&two));
    }
}
