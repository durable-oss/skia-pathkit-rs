# Repeated Union of many adjacent line-only polygons adds boundary noise not present in any input

Found from `font-vectorizer` (`../../fonts/font-vectorizer`), which unions
several closed line-only polygons ("ribbons": a stroke's spine offset left
and right by half its width) per glyph to build a filled outline from a
skeleton graph. Each ribbon is itself clean — see below — but the outline
that comes back from unioning them together is visibly jagged along
boundaries that should be smooth, with roughly double the line-segment
count of what the inputs justify.

## Repro

`font-vectorizer`'s `expand::expand_skeleton` (`src/expand.rs`) builds one
`pathkit::core::Path` per skeleton edge (`ribbon_path`, straight `Line`
verbs only, closed, `FillType::Winding`), then folds them together:

```rust
let mut union: Option<Path> = None;
for edge in &graph.edges {
    let ribbon = ribbon_path(edge)?;
    union = Some(match union {
        None => ribbon,
        Some(acc) => op(&acc, &ribbon, PathOp::Union)?,
    });
}
```

Traced glyph: `foundry/nounounou/input/strokes/vertical/I.png` (an art-nouveau
"I" stroke crop), run through `font-vectorizer`'s
`img2bez` → scanline width sampling → MST skeleton → `expand_skeleton`
pipeline. The resulting skeleton has several edges (a long main stem plus a
few short serif/junction edges).

**Isolated check**: expanding *only* the longest edge (stem) alone, with no
union against anything else, produces a clean contour — 71 `Line` segments,
points progressing smoothly and monotonically along the stroke (checked by
eye against the raw coordinate list; no back-and-forth zigzag).

**Full glyph**: expanding all edges and folding them together with repeated
pairwise `op(_, _, PathOp::Union)` as above produces 152 `Line` segments
across 7 contours for the same glyph — roughly double the single-edge count
for what should still be one dominant stroke shape plus a couple of small
serif additions, and the boundary visibly zigzags (small in/out notches)
where a smooth edge is expected. Rendered comparison screenshots exist only
in the originating chat session, not committed anywhere; re-running the
above pipeline against the same source image reproduces the segment-count
discrepancy.

## What's been ruled out on the `font-vectorizer` side

- Ribbon construction itself: confirmed clean via the single-edge isolation
  test above.
- Spine noise: `font-vectorizer` already smooths spine positions and
  widths (moving average / median over a window, angle-corrected for scan
  direction) before building each ribbon; widening that window 2x5 made no
  difference to the final unioned output, and the single-edge case is clean
  regardless — so the noise is not coming from unsmoothed input to `op`.
- Winding consistency: each ribbon is independently normalized to positive
  signed area before being passed to `op`, ruling out cancellation between
  oppositely-wound inputs (a separate bug that was found and fixed on the
  `font-vectorizer` side this session).

That leaves the repeated `op(_, _, PathOp::Union)` folding itself, or
something in how the engine (or its `boolean.rs` fallback — unclear which
path these particular inputs take) resolves a union of several
closed straight-line polygons that touch or nearly touch along shared
edges, as the remaining candidate.

## Suspected shape of the bug

Ribbons from adjacent skeleton edges meet at or very near a shared spine
point (a junction or a short serif-to-stem connection), so their polygon
boundaries run close to parallel and close to coincident over some span,
not just crossing at isolated points. That is exactly the kind of
near-degenerate input (near-tangency / near-coincidence rather than a clean
transversal crossing) that
`TODO/2026-09-15-union-drops-the-far-side-of-a-cubic-and-conic-pair.md`
found the angle-ordering machinery mishandles for curved inputs at *exact*
tangency. This may be a related failure mode for straight-line inputs at
*near*-coincidence (not exact), or it may be a distinct bug — not
established here.

**Checked**: whether repeated pairwise folding (union of the accumulator
against each new ribbon, one at a time) itself introduces error that a
single N-way union would not, since each pairwise `op` re-flattens/re-
intersects the accumulator — exactly the numerical-error-compounding
problem `OpBuilder` (`src/pathops/sk_op_builder.rs`) exists to avoid for
all-union cases. Swapped the naive pairwise fold in `expand_skeleton` for
`OpBuilder::add`/`resolve` (same repro, same "I" glyph) and re-ran: 155
`Line` segments, versus 152 from the pairwise fold — **no meaningful
difference**. This rules out pairwise-fold error compounding as the cause;
the noise is in how the union (however it's driven) resolves this
particular kind of input, not in the folding strategy. Reverted
`font-vectorizer` back to the plain pairwise fold, since `OpBuilder` bought
nothing here and constrains callers to a single op across all adds.

## Update (2026-09-15): synthetic repro attempted, did not reproduce

Built several synthetic line-only-polygon unions directly in
`sk_op_engine.rs`'s test module, per the disposition below, and swept them
against the "near-coincident edge" hypothesis this file started with:

- Two rectangles sharing a flush edge, and the same pair at gaps from
  `1e-6` down through `1e-2`. Clean at every gap.
- Two ribbons (thin quads, straight-line verbs) meeting at a T-junction
  near a shared point, at epsilon offsets from exact coincidence through
  `1e-1`. Clean at every epsilon.
- Three ribbons meeting at a Y-junction, folded pairwise the way
  `expand_skeleton` does (`union = fold(ribbons, |acc, r| op(acc, r,
  Union))`), at the same epsilon sweep. Clean at every epsilon.
- A chain of six ribbons end to end along a gently bending spine (a
  piecewise-linear stand-in for a smoothed stroke centerline), each
  sharing an exact endpoint with its neighbor and folded pairwise. Clean —
  the unioned verb count (26) came out lower than the input total (30), as
  expected from real edge merging, and every probed point agreed with the
  union of the individual ribbons.
- A plain rectangle crossed by a diagonal ribbon at an ordinary 70-degree
  angle (not tangent, not near-coincident) near one of the rectangle's own
  edges. Clean.

**An earlier pass through this same sweep found real-looking mismatches**
(up to 27 bad probe points on the three-ribbon case), but they turned out
to be a test-harness artifact, not an engine bug: the probe grid used
half-integer coordinates that landed exactly on an input polygon's own
straight edges, where `Path::contains` on a bare operand can legitimately
disagree with `Path::contains` on the unioned result right at the boundary
line itself — a boundary-inclusion quirk, not a containment error. Once
the grid was offset off every input's edges (`+ 0.13`/`+ 0.07`-style
jitter), every one of the above cases came back with zero mismatches. This
is a reminder for whoever sweeps `contains()` against a union result:
always jitter the probe grid off axis-aligned/round-number coordinates,
since those are exactly where straight-edged polygon inputs tend to have
their own vertices and edges.

None of the above reproduces the noise `font-vectorizer` reports. The two
permanent regression tests that came out of this
(`near_coincident_and_flush_rectangle_unions_agree_with_the_operands`,
`chain_of_slightly_bent_ribbons_folds_to_a_clean_union` in
`sk_op_engine.rs`) are kept as coverage for the shapes that were tried, not
as a fix — they pass because nothing was found wrong with these particular
constructions, not because a bug was found and closed.

**Not yet tried, and worth trying next:** the real pipeline has more
ribbons per glyph (7+ edges from a full skeleton graph, versus 6 in the
synthetic chain here) and a genuine Y- or X-junction where three or more
ribbons converge within a small region from *organically smoothed*,
non-radial spine geometry — the synthetic Y-junction tried here used a
clean, hand-picked 3-way radial split, which may not stress the same code
path as a graph node from a real MST skeleton with noisy vertex positions.
Reconstructing a repro closer to the real skeleton geometry (dumping the
actual ribbon vertex coordinates from a `font-vectorizer` run that shows
the bug, rather than constructing analogous shapes by hand) is the
strongest next step.

## Disposition

`OpBuilder` vs. pairwise fold is already ruled out (see above). The
"near-coincident edge" hypothesis did not reproduce in several synthetic
shapes (see update above), so it is downgraded from "suspected cause" to
"checked and not confirmed" — it may still be a contributing factor in
geometry more complex than what was tried, just not demonstrated here.
Whoever picks this up next should:

1. Get the actual ribbon coordinates out of a real `font-vectorizer` run on
   the traced glyph (`foundry/nounounou/input/strokes/vertical/I.png`)
   rather than constructing new analogous shapes by hand — the synthetic
   attempts above cover several plausible shapes of the bug and found
   nothing, so the next most useful data point is the real failing input,
   not another guess.
2. Feed those real coordinates through `op_with_engine` directly (bypassing
   `font-vectorizer`'s own pipeline) and compare against the same
   `contains()`-based check used here, with a jittered probe grid (see
   above — do not use round/half-integer coordinates for the probe grid
   when the inputs are axis-aligned or round-number polygons).
3. If that reproduces, trace `build`/`record_if_coincident`/the walk the
   way the sibling TODOs in this directory did for their gaps. If it still
   doesn't reproduce with real coordinates, the bug may be somewhere
   outside `sk_op_engine.rs` entirely (the routing between
   `op_with_engine` and the `boolean.rs` fallback in `pathops::op`, or
   something in how `font-vectorizer` itself constructs or interprets the
   result), which would need its own investigation on that side.

## Update (2026-09-15, later): real coordinates dumped; one real bug found and fixed on the font-vectorizer side, did not explain the failure

Followed this file's own "Disposition" step 1: added `font_vectorizer::ribbon_boundary` (public, `src/expand.rs`) to dump the exact per-edge polygon vertices `expand_skeleton` builds, before they reach `pathkit::pathops::op`, for a real failing glyph rather than a hand-built analogue.

**Real bug found first, on the `font-vectorizer` side, not pathkit's**:
dumping `foundry/nounounou/input/strokes/bowl/full.png`'s ribbons (a
"bowl"-stroke glyph whose union renders with a spurious diagonal slash cut
across it, a distinct visible defect from the "I" glyph's jaggedness this
file was originally opened for) turned up a ribbon with an exact duplicate
consecutive vertex: two different spine points a few font units apart, hit
by a sharp direction reversal in noisy scan data, both offsetting to the
identical coordinate `(167.3854, 102.5829)`. A closed polygon with a
zero-length edge is degenerate input for any boolean-ops engine regardless
of why it happened, so this needed fixing on the `font-vectorizer` side
either way: added `dedupe_consecutive` (collapses exact repeats, including
the wraparound seam) as the last step of `ribbon_boundary`, with a
regression test pinned to the exact real spine that produced it
(`ribbon_boundary_has_no_zero_length_edges_on_the_real_bowl_edge_that_produced_one`
in `expand.rs`).

**That fix did not change the slash artifact.** Re-ran the same glyph after
the fix: visually identical spurious diagonal cut through the ring. So the
duplicate-vertex bug was real and worth fixing, but it is not (or not
fully) the cause of this particular failure.

**Narrowed further**: the slash is not a stray extra ribbon surviving the
union unclipped. The unioned output has 11 contours; one of them (292
points) has a bounding box spanning the entire glyph and contains the
slash itself — a single contour with a self-intersection-shaped defect, not
a disjoint extra piece sitting on top of a correct main contour. That
argues for a walk/winding problem inside one union result, closer in shape
to this directory's other tangent/near-coincidence gaps than to "two
regions failed to merge."

**Real ribbon coordinates** (post-dedupe, 17 edges from this glyph's
skeleton graph — a Y/multi-junction topology, not a simple chain, unlike
every synthetic case tried in the update above) are pasted below for
whoever picks up this file's step 1 next, so a repro can be built from
real geometry instead of another hand-picked shape. Node kinds/positions
first (index refers to `edge.start`/`edge.end`), then each edge's `Vec<Point>` in the same left-forward-then-right-backward order `ribbon_boundary` returns (already positive-signed-area, no duplicate vertices):

```
# 17 edges, 18 nodes
# node 0: Endpoint (252.213, 85.012)
# node 1: Junction (199.543, 127.814)
# node 2: Endpoint (152.125, 197.530)
# node 3: Endpoint (196.639, 197.530)
# node 4: Junction (139.601, 267.247)
# node 5: Endpoint (734.339, 358.423)
# node 6: Endpoint (106.890, 385.150)
# node 7: Endpoint (645.378, 522.875)
# node 8: Endpoint (149.168, 559.540)
# node 9: Endpoint (614.966, 566.943)
# node 10: Endpoint (549.544, 609.612)
# node 11: Endpoint (74.219, 618.879)
# node 12: Endpoint (144.531, 124.665)
# node 13: Junction (156.250, 580.066)
# node 14: Endpoint (167.969, 252.139)
# node 15: Junction (636.719, 552.601)
# node 16: Junction (671.875, 115.489)
# node 17: Endpoint (753.906, 99.570)
# edge 0: start=1 end=0
edge_0 = [(149.5809, 86.7757), (177.3841, 67.9104), (167.3854, 78.3814), (167.3854, 102.5829), (167.3854, 82.4788), (188.6397, 63.0082), (211.5099, 32.2403), (288.4901, 121.8952), (260.3237, 134.9487), (275.3435, 122.6870), (275.3435, 102.5829), (275.3435, 126.7843), (257.1631, 144.5454), (249.5058, 168.8519)]
# edge 1: start=1 end=12
edge_1 = [(188.9744, 191.6000), (103.3522, 182.7073), (103.3522, 123.4541), (103.3522, 196.3443), (146.3616, 168.0368), (142.7009, 81.2937), (243.1102, 50.5638), (243.1102, 123.4541), (243.1102, 64.2008), (210.1123, 64.0277)]
# edge 2: start=1 end=3
edge_2 = [(261.4154, 146.5816), (183.2864, 200.9875), (150.2351, 162.6721), (184.7839, 124.1644), (215.5815, 193.3630), (177.6957, 201.6979), (193.1555, 201.1799), (227.7043, 162.6721), (194.6530, 124.3567), (137.6714, 109.0460)]
# edge 3: start=2 end=4
edge_3 = [(157.4358, 198.4311), (143.6659, 262.6425), (115.8524, 232.3888), (143.3715, 262.6163), (175.0111, 273.9643), (104.1904, 260.5299), (149.0549, 202.1613), (176.5740, 232.3888), (148.7605, 202.1350), (146.8146, 196.6298)]
# edge 4: start=4 end=6
edge_4 = [(175.0712, 273.6392), (168.4262, 302.0838), (166.7293, 308.5091), (164.2728, 313.1781), (163.0964, 318.1632), (160.6238, 322.8969), (157.1184, 332.9841), (155.6337, 340.3155), (153.5435, 345.7886), (152.7842, 349.2955), (150.2104, 353.9533), (141.9063, 387.5801), (90.3420, 379.3028), (93.2290, 343.2130), (93.1737, 336.2514), (94.9812, 328.1389), (96.9447, 323.0993), (97.4341, 320.8232), (98.1971, 309.7841), (98.1444, 303.4215), (99.5580, 297.4266), (99.8167, 291.2754), (100.9181, 287.1088), (104.1303, 260.8549)]
# edge 5: start=4 end=14
edge_5 = [(126.4709, 233.6819), (134.9569, 224.4388), (141.1929, 213.4093), (194.7446, 290.8692), (174.2561, 298.3157), (152.7306, 300.8123)]
# edge 6: start=5 end=15
edge_6 = [(751.8461, 362.3389), (747.5354, 398.3045), (746.1010, 405.6586), (744.1252, 412.0863), (742.3734, 418.6641), (740.0369, 424.4510), (736.2730, 434.6115), (732.8421, 445.7734), (728.6743, 457.1842), (723.9594, 468.0325), (718.9105, 478.2340), (713.9483, 488.0492), (707.5023, 498.4120), (700.4077, 508.7075), (694.4977, 518.6445), (693.1255, 523.9281), (691.1162, 527.7161), (688.5633, 533.1147), (683.1819, 539.9397), (664.7111, 581.1228), (608.7264, 524.0795), (648.2100, 508.3836), (650.0384, 506.1723), (653.2658, 502.0822), (657.0310, 498.1697), (661.3678, 495.0403), (668.9728, 486.8967), (673.9559, 478.4019), (678.0447, 469.6810), (681.9723, 460.6796), (686.0126, 451.8674), (688.7476, 442.8936), (691.7670, 433.1781), (694.1184, 423.2138), (696.0231, 412.1486), (697.0799, 406.3161), (698.2724, 401.2744), (698.8336, 396.0827), (699.5713, 391.8174), (701.1344, 358.0663)]
# edge 7: start=5 end=16
edge_7 = [(701.1567, 362.5886), (698.4757, 328.5842), (697.7159, 323.8601), (696.7494, 318.4226), (695.5198, 312.9907), (694.2727, 306.9914), (692.3226, 295.2643), (690.0620, 284.1597), (687.3394, 273.0202), (683.8896, 261.9429), (679.8521, 250.9389), (675.2005, 240.0456), (669.8698, 229.3249), (663.7109, 218.8932), (656.6587, 208.9189), (648.0097, 197.9249), (644.3484, 185.9163), (637.1291, 182.6808), (635.2817, 176.9552), (626.5479, 173.3942), (623.3412, 163.8418), (560.1898, 133.5878), (783.5602, 97.3902), (731.9025, 138.0620), (732.3700, 140.1543), (727.2757, 148.2323), (729.0556, 154.1419), (725.4324, 162.5386), (728.0268, 173.8972), (727.1267, 186.1421), (727.3496, 199.4066), (728.1480, 212.2138), (729.5067, 224.7319), (731.2723, 237.0775), (733.3472, 249.3124), (735.6551, 261.4740), (738.2825, 273.5733), (740.8769, 285.7076), (743.2212, 297.2194), (744.4672, 302.8396), (745.5641, 309.0271), (746.7442, 315.2090), (747.9386, 322.1043), (751.8238, 357.8166)]
# edge 8: start=6 end=8
edge_8 = [(142.2324, 382.9860), (142.1368, 416.5475), (142.6400, 420.7092), (143.3568, 425.8419), (144.5997, 430.9322), (145.7677, 436.9642), (147.3431, 448.6707), (150.1320, 459.1999), (153.7417, 469.7230), (158.2496, 480.2615), (163.5687, 491.2622), (167.9964, 496.1512), (169.4094, 501.2990), (174.8511, 505.2131), (176.5208, 510.3974), (215.5109, 535.3379), (92.0985, 580.1282), (105.7863, 535.3520), (103.4738, 528.9170), (105.1794, 521.2116), (103.1231, 514.7400), (104.3604, 508.0095), (102.9131, 495.7713), (101.1590, 483.0709), (99.2298, 470.3552), (97.3684, 457.6455), (95.2439, 446.1132), (94.0516, 440.5257), (93.2435, 433.9965), (92.2291, 427.5098), (91.3279, 420.0521), (90.0159, 383.8970)]
# edge 9: start=7 end=15
edge_9 = [(692.5291, 543.0822), (628.6738, 586.6896), (635.3608, 587.5847), (676.3582, 557.6781), (597.0793, 547.5242), (641.5554, 490.4572), (648.2423, 491.3523), (598.2265, 502.6673)]
# edge 10: start=8 end=13
edge_10 = [(218.2724, 545.3589), (221.0779, 562.0442), (252.3635, 577.6836), (60.1365, 582.4476), (90.8761, 576.0566), (89.3371, 570.1073)]
# edge 11: start=11 end=13
edge_11 = [(55.7859, 576.8769), (44.3375, 567.3666), (44.3375, 569.1917), (113.7920, 493.8055), (198.7080, 666.3257), (162.6937, 642.8522), (162.6937, 644.6773), (92.6516, 660.8812)]
# edge 12: start=13 end=10
edge_12 = [(209.9275, 500.3022), (210.9078, 542.1127), (212.8853, 542.8424), (213.6615, 547.5985), (216.9992, 548.6897), (220.8294, 553.9407), (230.5966, 560.5694), (238.5287, 563.7565), (246.1222, 569.3346), (255.3425, 572.4443), (263.8115, 577.4508), (273.4363, 581.9999), (283.4543, 586.0608), (293.7492, 589.7126), (304.2593, 592.9282), (314.9354, 595.7760), (325.7403, 598.2583), (336.6317, 600.3810), (347.5799, 602.1512), (358.5480, 603.5753), (369.5056, 604.6483), (380.4343, 605.3821), (391.3185, 605.7995), (402.1498, 605.9298), (412.9212, 605.8427), (423.6304, 605.5728), (434.2834, 605.1230), (444.8846, 604.4339), (455.4463, 603.5077), (465.9742, 602.3087), (476.4795, 600.7980), (488.4736, 598.9568), (494.6840, 597.8672), (499.9023, 597.3757), (505.0060, 596.2728), (509.0527, 595.8344), (542.5033, 582.6942), (566.8717, 647.3323), (530.0098, 660.6997), (522.3377, 663.4350), (515.7227, 665.2076), (509.2223, 667.3077), (503.7139, 668.5341), (492.2705, 672.3330), (479.3383, 675.5334), (466.4287, 678.1812), (453.5529, 680.3043), (440.7166, 681.9227), (427.9321, 683.0927), (415.2038, 683.8046), (402.5377, 684.1083), (389.9315, 684.0805), (377.3782, 683.8254), (364.8694, 683.4053), (352.3895, 682.8725), (339.9201, 682.2617), (327.4308, 681.5845), (314.8847, 680.8428), (302.2521, 680.0274), (289.4907, 679.1157), (276.5633, 678.0576), (263.4207, 676.8630), (250.0012, 675.4264), (236.1885, 673.7119), (221.2200, 671.5259), (209.2504, 666.5246), (195.5371, 664.0916), (183.9049, 657.8282), (174.4959, 655.3077), (167.8586, 655.9806), (161.0069, 652.1911), (151.9760, 651.7657), (144.6926, 647.0234), (102.5725, 659.8290)]
# edge 13: start=0 end=16
edge_13 = [(231.4919, 20.9569), (271.2047, 16.6059), (279.5112, 14.4383), (286.6511, 14.4705), (293.3721, 12.9656), (299.1079, 13.3444), (310.8934, 11.6812), (324.1550, 10.3551), (337.2731, 9.3028), (350.2833, 8.5145), (363.1912, 7.9588), (375.9680, 7.5491), (388.6527, 7.3355), (401.2657, 7.2469), (413.8209, 7.0342), (426.3482, 7.0861), (438.8550, 7.3923), (451.3465, 7.9472), (463.8403, 8.7477), (476.3615, 9.4969), (488.9162, 10.0001), (501.5232, 10.5216), (514.2597, 11.0740), (527.1423, 11.7026), (540.2111, 12.4089), (553.5147, 13.2480), (567.0626, 14.3608), (580.6618, 15.5931), (594.5973, 16.7501), (609.3096, 18.3102), (626.2260, 20.8898), (638.5239, 27.4229), (648.5701, 29.0077), (655.7196, 27.3511), (662.8304, 31.1597), (673.0684, 29.0251), (681.6841, 33.7175), (730.6283, 18.7978), (613.1217, 212.1802), (608.1476, 164.4977), (605.9249, 163.5690), (605.1290, 156.1407), (601.0688, 154.9911), (596.9478, 148.6837), (585.6872, 140.6739), (576.8990, 137.0227), (570.3779, 130.9575), (561.6527, 124.9543), (552.1507, 119.3984), (542.3124, 114.4673), (532.4228, 110.1514), (522.2889, 106.2146), (511.9202, 102.7397), (501.3653, 99.7106), (490.6643, 97.1008), (479.8338, 94.9048), (468.9510, 93.1108), (458.0347, 92.0024), (447.0910, 91.3915), (436.1450, 90.9670), (425.2143, 90.7324), (414.3041, 90.6897), (403.4218, 90.8425), (392.5973, 91.5975), (381.8445, 92.7349), (371.1838, 94.2082), (360.6542, 96.1140), (350.2269, 98.4147), (339.9075, 101.0887), (329.7316, 104.1760), (318.0796, 107.6486), (312.0966, 110.1760), (307.0989, 111.0373), (302.5200, 113.6666), (299.1078, 114.3371), (268.5081, 133.1787)]
# edge 14: start=10 end=9
edge_14 = [(532.5110, 588.5334), (561.7297, 569.1236), (549.4704, 572.5608), (549.4704, 588.8964), (549.4704, 572.0723), (566.6315, 558.0826), (581.5321, 528.8411), (645.0304, 600.2273), (613.5428, 612.2483), (621.8540, 605.7204), (621.8540, 588.8964), (621.8540, 605.2320), (601.7035, 615.6298), (576.8640, 641.4931)]
# edge 15: start=9 end=15
edge_15 = [(592.4105, 521.5642), (586.6275, 533.8159), (586.6275, 534.9031), (617.8259, 517.3859), (655.6116, 587.8165), (665.1803, 581.9035), (665.1803, 582.9907), (634.1520, 607.5042)]
# edge 16: start=16 end=17
edge_16 = [(647.6314, 4.9748), (712.7718, 51.1942), (749.0703, 67.9497), (758.7422, 131.1898), (732.5407, 157.5040), (696.1186, 226.0032)]
```

The full union (fold `op(_, _, PathOp::Union)` pairwise over `edge_0..edge_16` in order) is what should reproduce, if this is a pathkit-side gap and not something specific to how `font-vectorizer` walks `SkPath::iter()`'s result afterward (also worth checking, and easy to rule out: convert the unioned `Path` straight to an SVG `d` string via pathkit's own facilities, if any exist, or eyeball its verb/point list directly, without going through `font-vectorizer`'s `sk_path_to_contours`).

**The next step (feeding this data through `op_with_engine` and tracing
from there) is tracked in its own file, broken into small
independently-testable pieces:**
`TODO/2026-09-15-ribbon-union-slash-artifact-repro.md`. This file stays as
the full investigation history; further work belongs in the split-out file.

## Acceptance

- [x] Confirmed `OpBuilder` (all-at-once union) does *not* resolve the
      noise that pairwise-folded `op(_, _, PathOp::Union)` shows on this
      repro (155 vs. 152 segments — no meaningful difference). Rules out
      pairwise-fold error compounding as the cause.
- [x] Minimal synthetic repro attempted inside pathkit's own test suite,
      independent of `font-vectorizer` — see update above. Did not
      reproduce the noise in any of the five shapes tried (flush/near-gap
      rectangles, T-junction, Y-junction pairwise fold, six-ribbon chain,
      off-tangent crossing).
- [x] Real ribbon coordinates extracted from a failing `font-vectorizer`
      run (`bowl/full.png`, 17 edges) and pasted into this file, per the
      disposition's step 1 — see the second update above.
- [x] One real, distinct bug found and fixed on the `font-vectorizer` side
      while extracting that data (a ribbon with an exact-duplicate
      consecutive vertex, degenerate input for any boolean-ops engine) —
      but confirmed this does *not* explain the slash artifact it was
      found alongside; fixing it left that artifact unchanged.
- [ ] Root cause identified: still open. The near-coincident-edge
      hypothesis did not reproduce in synthetic shapes; no alternative root
      cause has been confirmed either. The real-coordinate repro above is
      narrowed further than prior updates: the artifact is a
      self-intersection-shaped defect inside a single unioned contour, not
      a stray extra piece or a failure to merge two regions.
- [ ] Relationship (or lack of one) to the tangent-contact gap in
      `2026-09-15-union-drops-the-far-side-of-a-cubic-and-conic-pair.md`
      not established — the synthetic shapes tried here don't share that
      bug's exact-tangency signature (the crossings tried were all at
      ordinary, non-tangent angles), so no evidence either way yet.
- [ ] Fixed: not yet. The real coordinates above are ready to feed through
      `op_with_engine` directly per step 2 of the disposition.
