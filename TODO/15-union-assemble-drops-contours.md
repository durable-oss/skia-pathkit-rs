# 15 — `assemble` drops every contour on many-vertex input; union returns empty

**Independent of item 10.** Confirmed a *separate* bug from the conic-weight
one — this reproduces with pure polygons, no conics anywhere.

## Symptom

`pathops::op(.., Union)` returns an empty path for two overlapping regular
polygons once the vertex count gets high enough, while the same shapes at low
vertex counts work.

Measured 2026-09-10 at `3f6aa61`, two 40-radius n-gons offset 5 units:

| n | edge length | result |
|---|---|---|
| 4, 6, 8, 12, 16, 24, 32 | 15.3 – 7.8 | 1 contour — correct |
| 48, 64, 128 | 5.2 – 2.0 | **0 contours — empty** |

Scaling the whole scene up fixes it, which shows it is edge length in absolute
units that matters, not vertex count as such:

| radius (n=64) | edge length | result |
|---|---|---|
| 10, 40, 100 | 0.98, 3.9, 9.8 | **empty** |
| 400, 1000 | 39.3, 98.1 | 1 contour |

Offset also matters non-monotonically at n=64/r=40 — 8 and 10 work, 15 fails,
20 works — which is the signature of a threshold being straddled, not of a
geometric special case.

## Where it is NOT

Classification is fine. Instrumenting the pipeline stages on the failing
cases:

```
n=48 off=5   raw segs=96   kept=52  both-inside=44  neither=0  -> empty
n=64 off=5   raw segs=128  kept=68  both-inside=60  neither=0  -> empty
```

52 and 68 boundary edges survive `classify_edge` with correct
inside/outside decisions. The edges exist and are right. Everything is lost
after that, in `assemble`.

Two hypotheses were tested and **ruled out**:

1. *Perpendicular sample offset too large for short edges.* `classify_edge`
   stepped a fixed `OFFSET = 0.25` from the edge midpoint, unclamped. Clamping
   it to `len * 0.5` changes **nothing** — output is byte-identical, every
   number in the tables above unchanged.

   This has since been committed independently as `0ff3a2f` ("correct offset
   computation in classify_edge for short edges"), and the tables above were
   re-measured against it. The clamp is a correct change on its own terms; it
   does not fix this bug. Do not re-try it.
2. *Conic flattening (item 10).* These test paths are built from `line_to`
   only. No conics involved.

## Where it is

`assemble` (`boolean.rs:263` onward, and the near-identical copy at
`sk_path_ops_simplify.rs`):

```rust
fn key(p: Point) -> (i32, i32) {
    ((p.x * 256.0).round() as i32, (p.y * 256.0).round() as i32)
}
```

Edges are chained by exact equality of this quantized key. At each vertex it
looks up `outgoing[key(cur)]`, and if the lookup misses or every candidate is
used, it `break`s. A chain that fails to return to its origin is discarded
wholesale:

```rust
if !closed || verts.len() < 3 { continue; }
```

So a single unmatched endpoint anywhere in a ring silently deletes the entire
contour. With short edges, endpoints produced by `split_segments`
(intersection points computed in `f32`) land either side of a 1/256 boundary
and the keys stop agreeing, the ring breaks, and every edge is thrown away.

The 1/256 grid is also just coarse: at 0.0039 units it is larger than the
`T_EPS = 1e-6` used for deduplicating split parameters, so two points that
`split_segments` considers distinct can weld together, and two it considers
identical can quantize apart.

## Fix directions

Do not simply shrink the quantization — that trades false welds for false
splits. Options, roughly in order of preference:

1. **Snap endpoints before assembly.** Cluster all edge endpoints with a
   tolerance tied to the geometry scale (not a fixed constant), rewrite every
   edge to the cluster representative, then chain on exact identity.
2. **Tolerant lookup.** Replace the `HashMap<key, …>` with a spatial index
   and match the nearest endpoint within tolerance.
3. **Do not discard on failure.** An unclosed chain currently vanishes.
   `SkPathWriter::assemble` in Skia exists precisely to stitch partial
   contours; emitting the partial result, or falling back to it, would turn a
   silent empty return into a visible-but-imperfect one.

Whichever is chosen, the tolerance must scale with the input, since the bug
disappears purely by scaling the scene up 10x.

Note this is a defect in the **substitute engine**, which item 09 deletes
outright. If 02-08 are close to landing, fixing it may be wasted work — but
until then every downstream consumer sees empty unions on curved or
finely-tessellated input, so it is probably worth doing anyway.

## Acceptance

- Two 64-gons of radius 40 offset by 5 union to one non-empty contour.
- The radius sweep (10, 40, 100, 400, 1000) gives 1 contour at every radius.
- The offset sweep at n=64/r=40 gives 1 contour at every offset from 0.5 to
  20 — no non-monotonic gaps.
- A contour that cannot be closed does not silently produce an empty result.
- Existing tests still pass.
