//! Demonstrates the geometry primitives implemented so far: `Point`,
//! `Rect`, and `Matrix`. Path construction and boolean path operations are
//! not yet implemented; see `PORTING.md`.

use pathkit::core::{Matrix, Point, Rect};

fn main() {
    let rect = Rect::from_xywh(10.0, 10.0, 100.0, 50.0);
    println!(
        "rect: {:?} (center: {}, {})",
        rect,
        rect.center_x(),
        rect.center_y()
    );

    let corners = rect.to_quad();
    println!("corners: {corners:?}");

    let rotation = Matrix::rotate_deg(90.0);
    let mapped = rotation.map_point(Point::new(1.0, 0.0));
    println!("(1, 0) rotated 90 degrees clockwise: {mapped:?}");

    let mut m = Matrix::identity();
    m.pre_translate(5.0, 5.0);
    m.pre_scale(2.0, 2.0);
    let mapped_rect = m.map_rect(&rect);
    println!("rect after scale-then-translate: {mapped_rect:?}");
}
