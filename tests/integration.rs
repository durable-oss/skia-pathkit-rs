//! Integration tests against the public API surface that is implemented
//! so far (geometry primitives). Path construction and boolean path
//! operations are not yet implemented; see `PORTING.md`.

use pathkit::core::{FillType, Matrix, Path, Point, Rect};

#[test]
fn rect_and_point_interop() {
    let rect = Rect::from_xywh(0.0, 0.0, 10.0, 20.0);
    let corners = rect.to_quad();
    assert_eq!(corners[0], Point::new(0.0, 0.0));
    assert_eq!(corners[2], Point::new(10.0, 20.0));
    assert_eq!(rect.width(), 10.0);
    assert_eq!(rect.height(), 20.0);
}

#[test]
fn matrix_round_trips_through_inverse() {
    let m = Matrix::translate(3.0, -4.0);
    let inv = m.invert().expect("translation matrices are invertible");
    let p = Point::new(1.0, 1.0);
    let round_tripped = inv.map_point(m.map_point(p));
    assert!((round_tripped.x - p.x).abs() < 1e-4);
    assert!((round_tripped.y - p.y).abs() < 1e-4);
}

#[test]
fn matrix_maps_rect_bounds() {
    let m = Matrix::scale(2.0, 3.0);
    let r = Rect::from_ltrb(0.0, 0.0, 10.0, 10.0);
    let mapped = m.map_rect(&r);
    assert_eq!(mapped, Rect::from_ltrb(0.0, 0.0, 20.0, 30.0));
}

#[test]
fn empty_path_has_winding_fill_by_default() {
    let path = Path::new();
    assert!(path.is_empty());
    assert_eq!(path.fill_type(), FillType::Winding);
}
