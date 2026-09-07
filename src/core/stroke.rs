//! Stroke parameters and application of stroking to a path.
//!
//! Ported from `include/core/SkStrokeRec.h` / `src/core/SkStrokeRec.cpp`.
//!
//! # Port status
//!
//! [`Cap`], [`Join`], and [`Style`] are ported. [`StrokeRec`]'s data layout
//! is ported; `apply_to_path`, `inflation_radius`, and the paint-derived
//! constructors are stubbed pending the stroking engine port
//! (`src/core/SkStroke.cpp`, `src/core/SkStrokerPriv.cpp`).

use super::path::Path;
use super::scalar::Scalar;

/// How the ends of an open contour are drawn when stroked.
///
/// Corresponds to `SkPaint::Cap`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Cap {
    /// No extension past the endpoint.
    #[default]
    Butt,
    /// A semicircle extension with radius equal to half the stroke width.
    Round,
    /// A square extension with side length equal to the stroke width.
    Square,
}

/// How the corners between stroked segments are drawn.
///
/// Corresponds to `SkPaint::Join`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Join {
    /// Extends both segments to a sharp point, up to the miter limit.
    #[default]
    Miter,
    /// A circular arc joining the segments.
    Round,
    /// A straight line connecting the outer edges of the segments.
    Bevel,
}

/// How a stroke operation combines with the path's fill.
///
/// Corresponds to `SkStrokeRec::Style`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Style {
    /// A one-pixel-wide stroke regardless of transform, with no fill.
    Hairline,
    /// Fill only; no stroke outline is generated.
    Fill,
    /// Stroke outline only.
    Stroke,
    /// Both the fill and the stroke outline.
    StrokeAndFill,
}

/// Stroke parameters: width, cap, join, miter limit, and whether stroking
/// combines with fill.
///
/// Mirrors `SkStrokeRec`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StrokeRec {
    pub(crate) res_scale: Scalar,
    pub(crate) width: Scalar,
    pub(crate) miter_limit: Scalar,
    pub(crate) cap: Cap,
    pub(crate) join: Join,
    pub(crate) stroke_and_fill: bool,
}

impl Default for StrokeRec {
    fn default() -> Self {
        Self::hairline()
    }
}

impl StrokeRec {
    /// Constructs a hairline `StrokeRec`: zero width, no fill.
    #[must_use]
    pub fn hairline() -> Self {
        StrokeRec {
            res_scale: 1.0,
            width: 0.0,
            miter_limit: 4.0,
            cap: Cap::Butt,
            join: Join::Miter,
            stroke_and_fill: false,
        }
    }

    /// Constructs a fill-only `StrokeRec`: no stroke outline is generated.
    #[must_use]
    pub fn fill() -> Self {
        StrokeRec {
            res_scale: 1.0,
            width: -1.0,
            miter_limit: 4.0,
            cap: Cap::Butt,
            join: Join::Miter,
            stroke_and_fill: false,
        }
    }

    /// Constructs a stroke `StrokeRec` with the given width.
    #[must_use]
    pub fn stroke(width: Scalar) -> Self {
        StrokeRec {
            res_scale: 1.0,
            width,
            miter_limit: 4.0,
            cap: Cap::Butt,
            join: Join::Miter,
            stroke_and_fill: false,
        }
    }

    /// Constructs a stroke-and-fill `StrokeRec` with the given width.
    #[must_use]
    pub fn stroke_and_fill(width: Scalar) -> Self {
        StrokeRec {
            res_scale: 1.0,
            width,
            miter_limit: 4.0,
            cap: Cap::Butt,
            join: Join::Miter,
            stroke_and_fill: true,
        }
    }

    /// Returns the effective [`Style`], derived from `width` and whether
    /// stroke-and-fill was requested.
    #[must_use]
    pub fn style(&self) -> Style {
        if self.width < 0.0 {
            Style::Fill
        } else if self.width == 0.0 {
            Style::Hairline
        } else if self.stroke_and_fill {
            Style::StrokeAndFill
        } else {
            Style::Stroke
        }
    }

    /// Sets the style to fill-only.
    pub fn set_fill_style(&mut self) {
        self.width = -1.0;
        self.stroke_and_fill = false;
    }

    /// Sets the style to hairline.
    pub fn set_hairline_style(&mut self) {
        self.width = 0.0;
        self.stroke_and_fill = false;
    }

    /// Sets the style to stroke with optional fill.
    pub fn set_stroke_style(&mut self, width: Scalar, stroke_and_fill: bool) {
        if stroke_and_fill && width == 0.0 {
            self.set_fill_style();
        } else {
            self.width = width;
            self.stroke_and_fill = stroke_and_fill;
        }
    }

    /// Returns the stroke width.
    #[must_use]
    pub fn width(&self) -> Scalar {
        self.width
    }

    /// Returns the miter limit.
    #[must_use]
    pub fn miter_limit(&self) -> Scalar {
        self.miter_limit
    }

    /// Sets the miter limit.
    pub fn set_miter_limit(&mut self, limit: Scalar) {
        self.miter_limit = limit;
    }

    /// Returns the line cap style.
    #[must_use]
    pub fn cap(&self) -> Cap {
        self.cap
    }

    /// Sets the cap style.
    pub fn set_cap(&mut self, cap: Cap) {
        self.cap = cap;
    }

    /// Returns the line join style.
    #[must_use]
    pub fn join(&self) -> Join {
        self.join
    }

    /// Sets the join style.
    pub fn set_join(&mut self, join: Join) {
        self.join = join;
    }

    /// Sets the cap, join, and miter limit used for stroking.
    pub fn set_stroke_params(&mut self, cap: Cap, join: Join, miter_limit: Scalar) {
        self.cap = cap;
        self.join = join;
        self.miter_limit = miter_limit;
    }

    /// Returns `true` if [`StrokeRec::style`] is [`Style::Hairline`].
    #[must_use]
    pub fn is_hairline_style(&self) -> bool {
        self.style() == Style::Hairline
    }

    /// Returns `true` if [`StrokeRec::style`] is [`Style::Fill`].
    #[must_use]
    pub fn is_fill_style(&self) -> bool {
        self.style() == Style::Fill
    }

    /// Returns `true` if this `StrokeRec` specifies stroking thick enough
    /// that [`StrokeRec::apply_to_path`] will change the path.
    #[must_use]
    pub fn needs_to_apply(&self) -> bool {
        matches!(self.style(), Style::Stroke | Style::StrokeAndFill)
    }

    /// Returns the resolution scale used to adjust stroking tolerance.
    #[must_use]
    pub fn res_scale(&self) -> Scalar {
        self.res_scale
    }

    /// Sets the resolution scale used to adjust stroking tolerance.
    pub fn set_res_scale(&mut self, res_scale: Scalar) {
        self.res_scale = res_scale;
    }

    /// Applies these stroke parameters to `src`, returning the stroked
    /// outline. Returns `None` if `style()` is `Hairline` or `Fill` (i.e.
    /// there is no outline to generate).
    ///
    /// # Panics
    ///
    /// Always panics when stroking is actually required; not yet ported.
    #[must_use]
    pub fn apply_to_path(&self, src: &Path) -> Option<Path> {
        if !self.needs_to_apply() {
            return None;
        }
        let _ = src;
        todo!("port SkStrokeRec::applyToPath (src/core/SkStroke.cpp)")
    }

    /// A conservative outset to apply to a shape's bounds to account for
    /// inflation from stroking with these parameters.
    #[must_use]
    pub fn inflation_radius(&self) -> Scalar {
        get_inflation_radius(self.join, self.miter_limit, self.cap, self.width)
    }
}

/// Computes the inflation radius for a stroke with given parameters.
fn get_inflation_radius(join: Join, miter_limit: Scalar, cap: Cap, stroke_width: Scalar) -> Scalar {
    if stroke_width < 0.0 {
        // fill
        0.0
    } else if stroke_width == 0.0 {
        // hairline - FIXME: needs matrixScale parameter for proper handling
        1.0
    } else {
        let mut multiplier: Scalar = 1.0;
        // Only apply miter limit multiplier for Miter join
        if join == Join::Miter && miter_limit > 1.0 {
            multiplier = multiplier.max(miter_limit);
        }
        if cap == Cap::Square {
            multiplier = multiplier.max(2.0_f32.sqrt());
        }
        stroke_width / 2.0 * multiplier
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hairline_style() {
        let rec = StrokeRec::hairline();
        assert_eq!(rec.style(), Style::Hairline);
        assert!(rec.is_hairline_style());
        assert!(!rec.needs_to_apply());
    }

    #[test]
    fn fill_style() {
        let rec = StrokeRec::fill();
        assert_eq!(rec.style(), Style::Fill);
        assert!(rec.is_fill_style());
        assert!(!rec.needs_to_apply());
    }

    #[test]
    fn stroke_style_from_width() {
        let rec = StrokeRec::stroke(2.0);
        assert_eq!(rec.style(), Style::Stroke);
        assert!(rec.needs_to_apply());
    }

    #[test]
    fn stroke_and_fill_style() {
        let rec = StrokeRec::stroke_and_fill(2.0);
        assert_eq!(rec.style(), Style::StrokeAndFill);
        assert!(rec.needs_to_apply());
    }

    #[test]
    fn set_fill_style() {
        let mut rec = StrokeRec::stroke(2.0);
        rec.set_fill_style();
        assert_eq!(rec.style(), Style::Fill);
        assert!(rec.is_fill_style());
        assert!(!rec.needs_to_apply());
    }

    #[test]
    fn set_hairline_style() {
        let mut rec = StrokeRec::fill();
        rec.set_hairline_style();
        assert_eq!(rec.style(), Style::Hairline);
        assert!(rec.is_hairline_style());
        assert!(!rec.needs_to_apply());
    }

    #[test]
    fn set_stroke_style() {
        let mut rec = StrokeRec::fill();
        rec.set_stroke_style(5.0, false);
        assert_eq!(rec.style(), Style::Stroke);
        assert_eq!(rec.width(), 5.0);
        assert!(rec.needs_to_apply());
    }

    #[test]
    fn set_stroke_and_fill_style() {
        let mut rec = StrokeRec::fill();
        rec.set_stroke_style(5.0, true);
        assert_eq!(rec.style(), Style::StrokeAndFill);
        assert_eq!(rec.width(), 5.0);
        assert!(rec.needs_to_apply());
    }

    #[test]
    fn set_stroke_and_fill_hairline_becomes_fill() {
        let mut rec = StrokeRec::fill();
        rec.set_stroke_style(0.0, true);
        assert_eq!(rec.style(), Style::Fill);
        assert!(!rec.needs_to_apply());
    }

    #[test]
    fn test_miter_limit() {
        let mut rec = StrokeRec::hairline();
        assert_eq!(rec.miter_limit(), 4.0);
        rec.set_miter_limit(10.0);
        assert_eq!(rec.miter_limit(), 10.0);
    }

    #[test]
    fn test_cap() {
        let mut rec = StrokeRec::hairline();
        assert_eq!(rec.cap(), Cap::Butt);
        rec.set_cap(Cap::Round);
        assert_eq!(rec.cap(), Cap::Round);
    }

    #[test]
    fn test_join() {
        let mut rec = StrokeRec::hairline();
        assert_eq!(rec.join(), Join::Miter);
        rec.set_join(Join::Round);
        assert_eq!(rec.join(), Join::Round);
    }

    #[test]
    fn test_res_scale() {
        let mut rec = StrokeRec::hairline();
        assert_eq!(rec.res_scale(), 1.0);
        rec.set_res_scale(2.0);
        assert_eq!(rec.res_scale(), 2.0);
    }

    #[test]
    fn test_default() {
        let rec = StrokeRec::default();
        assert_eq!(rec.style(), Style::Hairline);
    }

    #[test]
    fn inflation_radius_fill() {
        let rec = StrokeRec::fill();
        assert_eq!(rec.inflation_radius(), 0.0);
    }

    #[test]
    fn inflation_radius_hairline() {
        let rec = StrokeRec::hairline();
        assert_eq!(rec.inflation_radius(), 1.0);
    }

    #[test]
    fn inflation_radius_stroke() {
        // With default Miter join and miter_limit=4.0, multiplier=4.0
        // inflation = 10.0/2 * 4.0 = 20.0
        let rec = StrokeRec::stroke(10.0);
        assert_eq!(rec.inflation_radius(), 20.0);
    }

    #[test]
    fn inflation_radius_with_square_cap() {
        // With Miter join (miter_limit=4.0) and Square cap (sqrt(2)),
        // multiplier = max(4.0, sqrt(2)) = 4.0
        // inflation = 10.0/2 * 4.0 = 20.0
        let mut rec = StrokeRec::stroke(10.0);
        rec.set_cap(Cap::Square);
        assert_eq!(rec.inflation_radius(), 20.0);
    }

    #[test]
    fn inflation_radius_with_miter_join() {
        let mut rec = StrokeRec::stroke(10.0);
        rec.set_miter_limit(3.0);
        assert_eq!(rec.inflation_radius(), 5.0 * 3.0);
    }
}
