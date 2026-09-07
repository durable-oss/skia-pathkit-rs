//! Rounded rectangles.
//!
//! Ported from `include/core/SkRRect.h` / `src/core/SkRRect.cpp`.
//!
//! # Port status
//!
//! [`Type`] and [`Corner`] are ported. [`RRect`]'s constructors and
//! queries (`set_rect_xy`, `set_nine_patch`, `set_rect_radii`, `width`,
//! `height`, `rect`, `radii`, `contains`, `is_valid`, `inset`/`outset`) are
//! stubbed pending port.

use super::point::Point;
use super::rect::Rect;
use super::scalar::Scalar;

/// The degree of specialization an [`RRect`] falls into. Larger values have
/// more degrees of freedom than smaller ones.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Type {
    /// Zero width or height.
    Empty,
    /// Non-zero width and height, zeroed radii.
    Rect,
    /// Non-zero width and height, filled with radii (an oval or circle).
    Oval,
    /// Non-zero width and height, all four corners share equal radii.
    Simple,
    /// Non-zero width and height, radii are axis-aligned (each pair of
    /// opposite corners shares a radius).
    NinePatch,
    /// Non-zero width and height, each corner has its own radii.
    Complex,
}

/// Indexes into an [`RRect`]'s per-corner radii, in top-left, top-right,
/// bottom-right, bottom-left order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Corner {
    /// Top-left corner.
    UpperLeft,
    /// Top-right corner.
    UpperRight,
    /// Bottom-right corner.
    LowerRight,
    /// Bottom-left corner.
    LowerLeft,
}

/// A rectangle with independently controllable per-corner radii.
///
/// Mirrors `SkRRect`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RRect {
    rect: Rect,
    radii: [Point; 4],
    rrect_type: Type,
}

impl RRect {
    /// Constructs an empty `RRect` at the origin.
    #[must_use]
    pub fn new() -> Self {
        RRect {
            rect: Rect::empty(),
            radii: [Point::new(0.0, 0.0); 4],
            rrect_type: Type::Empty,
        }
    }

    /// Returns the specialization type of this `RRect`.
    #[must_use]
    pub fn get_type(&self) -> Type {
        self.rrect_type
    }

    /// Returns the bounding rectangle.
    #[must_use]
    pub fn rect(&self) -> Rect {
        self.rect
    }

    /// Returns the per-corner radii.
    #[must_use]
    pub fn radii(&self) -> [Point; 4] {
        self.radii
    }

    /// Constructs a simple `RRect`: `rect` with every corner rounded by the
    /// same `(rx, ry)` radii.
    ///
    /// # Panics
    ///
    /// Always panics; not yet ported.
    #[must_use]
    pub fn from_rect_xy(rect: Rect, rx: Scalar, ry: Scalar) -> Self {
        let _ = (rect, rx, ry);
        todo!("port SkRRect::setRectXY (src/core/SkRRect.cpp)")
    }

    /// Constructs an `RRect` with independently specified per-corner radii.
    ///
    /// # Panics
    ///
    /// Always panics; not yet ported.
    #[must_use]
    pub fn from_rect_radii(rect: Rect, radii: [Point; 4]) -> Self {
        let _ = (rect, radii);
        todo!("port SkRRect::setRectRadii (src/core/SkRRect.cpp)")
    }

    /// Returns `true` if `(x, y)` is enclosed by this `RRect`.
    ///
    /// # Panics
    ///
    /// Always panics; not yet ported.
    #[must_use]
    pub fn contains(&self, x: Scalar, y: Scalar) -> bool {
        let _ = (x, y);
        todo!("port SkRRect::contains (src/core/SkRRect.cpp)")
    }

    /// Returns `true` if the radii and rect are internally consistent
    /// (non-negative, non-overlapping, and matching `rrect_type`).
    ///
    /// # Panics
    ///
    /// Always panics; not yet ported.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        todo!("port SkRRect::isValid (src/core/SkRRect.cpp)")
    }
}

impl Default for RRect {
    fn default() -> Self {
        RRect::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_rrect_is_empty() {
        let r = RRect::new();
        assert_eq!(r.get_type(), Type::Empty);
    }
}
