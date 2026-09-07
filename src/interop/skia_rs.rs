//! Conversions between [`crate::core::Path`] and `skia-rs-path`.

use std::convert::TryFrom;

use crate::core::{FillType, Path, PathBuilder, Point, Verb};
use crate::error::PathKitError;

/// Converts a `pathkit` path into a `skia-rs-path` path.
///
/// The conversion preserves fill type, path verbs, coordinates, and conic
/// weights. Non-finite coordinates are passed through unchanged, matching
/// both crates' path-construction behavior.
pub fn to_skia_rs_path(path: &Path) -> Result<skia_rs_path::Path, PathKitError> {
    let mut builder =
        skia_rs_path::PathBuilder::with_fill_type(to_skia_fill_type(path.fill_type()));

    for (verb, points, conic_weight) in path.iter() {
        match verb {
            Verb::Move => {
                let p = to_skia_point(points[0]);
                builder.move_to(p.x, p.y);
            }
            Verb::Line => {
                let p = to_skia_point(points[1]);
                builder.line_to(p.x, p.y);
            }
            Verb::Quad => {
                let p1 = to_skia_point(points[1]);
                let p2 = to_skia_point(points[2]);
                builder.quad_to(p1.x, p1.y, p2.x, p2.y);
            }
            Verb::Conic => {
                let p1 = to_skia_point(points[1]);
                let p2 = to_skia_point(points[2]);
                let weight = conic_weight.ok_or_else(|| {
                    PathKitError::UnsupportedConversion(
                        "pathkit conic verb missing its conic weight".to_owned(),
                    )
                })?;
                builder.conic_to(p1.x, p1.y, p2.x, p2.y, weight);
            }
            Verb::Cubic => {
                let p1 = to_skia_point(points[1]);
                let p2 = to_skia_point(points[2]);
                let p3 = to_skia_point(points[3]);
                builder.cubic_to(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
            }
            Verb::Close => {
                builder.close();
            }
        }
    }

    Ok(builder.build())
}

/// Converts a `skia-rs-path` path into a `pathkit` path.
///
/// The conversion preserves fill type, path elements, coordinates, and conic
/// weights. Non-finite coordinates are passed through unchanged, matching
/// both crates' path-construction behavior.
pub fn from_skia_rs_path(path: &skia_rs_path::Path) -> Result<Path, PathKitError> {
    let mut builder = PathBuilder::new();
    builder.set_fill_type(from_skia_fill_type(path.fill_type()));

    for element in path {
        match element {
            skia_rs_path::PathElement::Move(p) => {
                builder.move_to(from_skia_point(p));
            }
            skia_rs_path::PathElement::Line(p) => {
                builder.line_to(from_skia_point(p));
            }
            skia_rs_path::PathElement::Quad(p1, p2) => {
                builder.quad_to(from_skia_point(p1), from_skia_point(p2));
            }
            skia_rs_path::PathElement::Conic(p1, p2, weight) => {
                builder.conic_to(from_skia_point(p1), from_skia_point(p2), weight);
            }
            skia_rs_path::PathElement::Cubic(p1, p2, p3) => {
                builder.cubic_to(
                    from_skia_point(p1),
                    from_skia_point(p2),
                    from_skia_point(p3),
                );
            }
            skia_rs_path::PathElement::Close => {
                builder.close();
            }
        }
    }

    Ok(builder.snapshot())
}

impl TryFrom<&skia_rs_path::Path> for Path {
    type Error = PathKitError;

    fn try_from(path: &skia_rs_path::Path) -> Result<Self, Self::Error> {
        from_skia_rs_path(path)
    }
}

impl TryFrom<&Path> for skia_rs_path::Path {
    type Error = PathKitError;

    fn try_from(path: &Path) -> Result<Self, Self::Error> {
        to_skia_rs_path(path)
    }
}

fn to_skia_point(point: Point) -> skia_rs_core::Point {
    skia_rs_core::Point::new(point.x, point.y)
}

fn from_skia_point(point: skia_rs_core::Point) -> Point {
    Point::new(point.x, point.y)
}

fn to_skia_fill_type(fill_type: FillType) -> skia_rs_path::FillType {
    match fill_type {
        FillType::Winding => skia_rs_path::FillType::Winding,
        FillType::EvenOdd => skia_rs_path::FillType::EvenOdd,
        FillType::InverseWinding => skia_rs_path::FillType::InverseWinding,
        FillType::InverseEvenOdd => skia_rs_path::FillType::InverseEvenOdd,
    }
}

fn from_skia_fill_type(fill_type: skia_rs_path::FillType) -> FillType {
    match fill_type {
        skia_rs_path::FillType::Winding => FillType::Winding,
        skia_rs_path::FillType::EvenOdd => FillType::EvenOdd,
        skia_rs_path::FillType::InverseWinding => FillType::InverseWinding,
        skia_rs_path::FillType::InverseEvenOdd => FillType::InverseEvenOdd,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pathops::{self, PathOp};

    #[test]
    fn empty_path_round_trips() {
        let path = Path::new();

        let skia = to_skia_rs_path(&path).unwrap();
        let converted = from_skia_rs_path(&skia).unwrap();

        assert_eq!(converted, path);
    }

    #[test]
    fn all_verbs_round_trip_losslessly() {
        let mut builder = PathBuilder::new();
        builder
            .set_fill_type(FillType::InverseEvenOdd)
            .move_to(Point::new(1.0, 2.0))
            .line_to(Point::new(3.0, 4.0))
            .quad_to(Point::new(5.0, 6.0), Point::new(7.0, 8.0))
            .conic_to(Point::new(9.0, 10.0), Point::new(11.0, 12.0), 0.75)
            .cubic_to(
                Point::new(13.0, 14.0),
                Point::new(15.0, 16.0),
                Point::new(17.0, 18.0),
            )
            .close();
        let path = builder.snapshot();

        let skia = skia_rs_path::Path::try_from(&path).unwrap();
        let converted = Path::try_from(&skia).unwrap();

        assert_eq!(converted, path);
    }

    #[test]
    fn pathop_result_can_convert_back_to_skia_rs() {
        let mut builder = PathBuilder::new();
        builder
            .move_to(Point::new(0.0, 0.0))
            .cubic_to(
                Point::new(20.0, 0.0),
                Point::new(20.0, 20.0),
                Point::new(0.0, 20.0),
            )
            .close();
        let path = builder.snapshot();
        let result = pathops::op(&path, &path, PathOp::Union).unwrap();

        let skia = to_skia_rs_path(&result).unwrap();

        assert!(skia
            .iter()
            .any(|element| matches!(element, skia_rs_path::PathElement::Cubic(_, _, _))));
    }
}
