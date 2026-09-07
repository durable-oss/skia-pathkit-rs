//! Core geometry and path types: points, rectangles, matrices, and the
//! [`Path`] type itself.
//!
//! Mirrors the directory layout of `include/core/` and `src/core/` in the
//! original C++.

mod builder;
mod contour_measure;
mod matrix;
mod paint;
mod paint_priv;
mod path;
mod point;
mod rect;
mod rrect;
pub mod scalar;
mod sk_arena_alloc;
mod sk_cubic_clipper;
mod sk_geometry;
mod sk_malloc;
mod sk_math;
mod sk_path_measure;
mod sk_path_ref;
mod sk_point;
mod sk_stroke;
mod sk_stroker_priv;
mod stroke;
mod types;

pub use builder::PathBuilder;
pub use contour_measure::{ContourMeasure, ContourMeasureIter};
pub use matrix::{Matrix, ScaleToFit};
pub use paint::{Paint, Style as PaintStyle};
pub use paint_priv::PaintPriv;
pub use path::{Path, PathIter};
pub use point::{IPoint, IVector, Point, Vector};
pub use rect::Rect;
pub use rrect::{Corner, RRect, Type as RRectType};
pub use scalar::Scalar;
pub use sk_arena_alloc::ArenaAlloc;
pub use sk_cubic_clipper::SkCubicClipper;
pub use sk_geometry::CubicType;
pub use sk_malloc::{sk_bzero, sk_careful_memcpy};
pub use sk_math::{sk_floats_are_unit, sk_ieee_float_divide, sk_sqrt_bits, SkSafeMath};
pub use sk_path_measure::{MatrixFlags, PathMeasure};
pub use sk_path_ref::SkPathRef;
pub use sk_point::{PointExt, Side};
pub use stroke::{Cap, Join, StrokeRec, Style as StrokeStyle};
pub use types::{
    Direction, FillType, Verb, SEGMENT_MASK_CONIC, SEGMENT_MASK_CUBIC, SEGMENT_MASK_LINE,
    SEGMENT_MASK_QUAD,
};
