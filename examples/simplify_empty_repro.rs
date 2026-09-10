use pathkit::{core::Path, pathops};

fn main() {
    // A 33-point polyline reduced from a font glyph's swept stroke. The
    // closing segment crosses the path's own body.
    const PTS: &[(f32, f32)] = &[
        (265.2254,646.6782),(293.8087,654.6358),(324.1818,659.5676),(355.7458,661.2591),
        (387.9075,659.5240),(387.9075,659.5240),(419.1481,654.2556),(448.0477,645.9188),
        (474.1282,634.8705),(496.9165,621.5131),(516.2436,606.6158),(531.5778,590.3043),
        (542.7201,573.0750),(549.6874,555.2849),(549.6874,555.2849),(552.4633,537.5287),
        (551.3721,520.1179),(546.4543,502.7428),(537.4683,485.4285),(524.5909,468.1683),
        (507.2624,451.7708),(485.5978,436.8444),(459.8072,423.9530),(459.8072,423.9530),
        (450.3936,421.8325),(429.5548,418.3125),(403.0803,414.4648),(374.8140,410.7244),
        (348.3902,407.4814),(327.4115,405.1188),(315.6939,404.0379),(322.0133,356.9807),
        (321.5260,360.3026),
    ];

    let build = |pts: &[(f32, f32)]| {
        let mut p = Path::new();
        p.move_to(pts[0].0, pts[0].1);
        for q in &pts[1..] { p.line_to(q.0, q.1); }
        p.close();
        p
    };

    let full = build(PTS);
    let s = pathops::simplify(&full).expect("simplify should not error");
    println!("full path:        in={} verbs, out={} verbs, empty={}",
             full.iter().count(), s.iter().count(), s.is_empty());

    let trimmed = build(&PTS[..PTS.len() - 1]);
    let t = pathops::simplify(&trimmed).expect("simplify should not error");
    println!("last point removed: in={} verbs, out={} verbs, empty={}",
             trimmed.iter().count(), t.iter().count(), t.is_empty());

    assert!(!s.is_empty(), "BUG: simplify returned an empty path for a non-empty input");
}
