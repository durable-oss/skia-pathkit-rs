# pathkit

A Rust port of [Google Skia's PathKit][pathkit]: 2D path construction,
affine/perspective transforms, and boolean path operations
(union/intersect/difference/xor/simplify).

[pathkit]: https://skia.org/docs/user/modules/pathkit/

## Status

This is an in-progress port. 

## Usage

```rust
use pathkit::core::{Matrix, Point, Rect};

let r = Rect::from_ltrb(0.0, 0.0, 100.0, 50.0);
assert_eq!((r.center_x(), r.center_y()), (50.0, 25.0));

let m = Matrix::translate(10.0, 20.0);
assert_eq!(m.map_point(Point::new(0.0, 0.0)), Point::new(10.0, 20.0));
```

## License

BSD-3-Clause, matching the original PathKit/Skia license. See
[`LICENSE`](LICENSE).
