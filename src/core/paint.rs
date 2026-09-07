//! Paint settings controlling how paths are drawn.
//!
//! Ported from `old/pathkit/include/core/SkPaint.h` and
//! `old/pathkit/src/core/SkPaint.cpp`.

use super::paint_priv::PaintPriv;
use super::scalar::Scalar;
use super::stroke::{Cap, Join, StrokeRec};
use super::Matrix;
use super::Path;
use super::Rect;

/// Controls how a path is rendered (fill, stroke, or both).
///
/// Corresponds to `SkPaint::Style`. Unlike [`stroke::Style`](super::stroke::Style),
/// this has no `Hairline` variant — hairline-ness is a property of the
/// stroke width (zero), not a distinct paint style.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Style {
    /// Fill only.
    #[default]
    Fill,
    /// Stroke only.
    Stroke,
    /// Fill and stroke.
    StrokeAndFill,
}

impl Style {
    /// Total number of valid style values.
    pub const COUNT: usize = 3;

    /// Returns `true` if `style` is a valid style value.
    pub fn is_valid(style: u8) -> bool {
        style < Self::COUNT as u8
    }
}

/// Controls options applied when drawing.
///
/// Paint collects all options outside of the canvas clip and canvas
/// matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Paint {
    width: Scalar,
    miter_limit: Scalar,
    style: Style,
    cap: Cap,
    join: Join,
    anti_alias: bool,
    dither: bool,
}

impl Default for Paint {
    fn default() -> Self {
        Self::new()
    }
}

impl Paint {
    /// Default miter limit.
    pub const DEFAULT_MITER_LIMIT: Scalar = 4.0;

    /// Constructs a new Paint with default values.
    #[must_use]
    pub fn new() -> Self {
        Paint {
            width: 0.0,
            miter_limit: Self::DEFAULT_MITER_LIMIT,
            style: Style::default(),
            cap: Cap::default(),
            join: Join::default(),
            anti_alias: false,
            dither: false,
        }
    }

    /// Sets all contents to their initial values.
    pub fn reset(&mut self) {
        *self = Paint::new();
    }

    // --- Style ---

    /// Returns the current style.
    #[must_use]
    pub fn style(&self) -> Style {
        self.style
    }

    /// Sets the style. Has no effect if the value is invalid.
    pub fn set_style(&mut self, style: Style) {
        self.style = style;
    }

    /// Sets the style to stroke if true, or fill if false.
    pub fn set_stroke(&mut self, is_stroke: bool) {
        self.style = if is_stroke {
            Style::Stroke
        } else {
            Style::Fill
        };
    }

    // --- Width ---

    /// Returns the current stroke width.
    #[must_use]
    pub fn stroke_width(&self) -> Scalar {
        self.width
    }

    /// Sets the stroke width. Negative values are ignored.
    pub fn set_stroke_width(&mut self, width: Scalar) {
        if width >= 0.0 {
            self.width = width;
        }
    }

    // --- Miter limit ---

    /// Returns the current miter limit.
    #[must_use]
    pub fn stroke_miter(&self) -> Scalar {
        self.miter_limit
    }

    /// Sets the miter limit. Negative values are ignored.
    pub fn set_stroke_miter(&mut self, limit: Scalar) {
        if limit >= 0.0 {
            self.miter_limit = limit;
        }
    }

    // --- Cap ---

    /// Returns the current stroke cap.
    #[must_use]
    pub fn stroke_cap(&self) -> Cap {
        self.cap
    }

    /// Sets the stroke cap.
    pub fn set_stroke_cap(&mut self, cap: Cap) {
        self.cap = cap;
    }

    // --- Join ---

    /// Returns the current stroke join.
    #[must_use]
    pub fn stroke_join(&self) -> Join {
        self.join
    }

    /// Sets the stroke join.
    pub fn set_stroke_join(&mut self, join: Join) {
        self.join = join;
    }

    // --- Anti-alias & Dither ---

    /// Returns `true` if anti-aliasing is enabled.
    #[must_use]
    pub fn is_anti_alias(&self) -> bool {
        self.anti_alias
    }

    /// Enables or disables anti-aliasing.
    pub fn set_anti_alias(&mut self, enabled: bool) {
        self.anti_alias = enabled;
    }

    /// Returns `true` if dithering is enabled.
    #[must_use]
    pub fn is_dither(&self) -> bool {
        self.dither
    }

    /// Enables or disables dithering.
    pub fn set_dither(&mut self, enabled: bool) {
        self.dither = enabled;
    }

    // --- getFillPath ---

    /// Returns the filled equivalent of the stroked path.
    ///
    /// Returns `true` if the path represents a fill, or `false` if it represents a hairline.
    pub fn get_fill_path(
        &self,
        src: &Path,
        dst: &mut Path,
        cull_rect: Option<&Rect>,
        res_scale: Scalar,
    ) -> bool {
        self.get_fill_path_with_matrix(src, dst, cull_rect, &Matrix::identity(), res_scale)
    }

    /// Returns the filled equivalent of the stroked path with a custom transform.
    pub fn get_fill_path_with_matrix(
        &self,
        src: &Path,
        dst: &mut Path,
        _cull_rect: Option<&Rect>,
        ctm: &Matrix,
        res_scale: Scalar,
    ) -> bool {
        // Check if source path is finite
        if !src.is_finite() {
            dst.reset();
            return false;
        }

        // Compute resolution scale and create stroke record
        let res_scale = PaintPriv::compute_res_scale_for_stroking(ctm) * res_scale;
        let rec = StrokeRec {
            res_scale,
            width: self.width,
            miter_limit: self.miter_limit,
            cap: self.cap,
            join: self.join,
            stroke_and_fill: matches!(self.style, Style::StrokeAndFill),
        };

        // Apply to path
        match rec.apply_to_path(src) {
            Some(stroked) => *dst = stroked,
            None => *dst = src.clone(),
        }

        // Check if result is finite
        if !dst.is_finite() {
            dst.reset();
            return false;
        }

        // Return true if not hairline
        !rec.is_hairline_style()
    }

    /// Converts this Paint into a [`StrokeRec`].
    #[must_use]
    pub fn to_stroke_rec(&self, res_scale: Scalar) -> StrokeRec {
        StrokeRec {
            res_scale,
            width: self.width,
            miter_limit: self.miter_limit,
            cap: self.cap,
            join: self.join,
            stroke_and_fill: matches!(self.style, Style::StrokeAndFill),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_paint_defaults() {
        let paint = Paint::new();
        assert_eq!(paint.style(), Style::Fill);
        assert_eq!(paint.stroke_width(), 0.0);
        assert_eq!(paint.stroke_miter(), Paint::DEFAULT_MITER_LIMIT);
        assert_eq!(paint.stroke_cap(), Cap::Butt);
        assert_eq!(paint.stroke_join(), Join::Miter);
        assert!(!paint.is_anti_alias());
        assert!(!paint.is_dither());
    }

    #[test]
    fn test_style_methods() {
        let mut paint = Paint::new();
        paint.set_style(Style::Stroke);
        assert_eq!(paint.style(), Style::Stroke);

        paint.set_stroke(true);
        assert_eq!(paint.style(), Style::Stroke);

        paint.set_stroke(false);
        assert_eq!(paint.style(), Style::Fill);
    }

    #[test]
    fn test_stroke_width() {
        let mut paint = Paint::new();
        paint.set_stroke_width(5.0);
        assert_eq!(paint.stroke_width(), 5.0);

        // Negative values should be ignored
        paint.set_stroke_width(-1.0);
        assert_eq!(paint.stroke_width(), 5.0);
    }

    #[test]
    fn test_stroke_miter() {
        let mut paint = Paint::new();
        paint.set_stroke_miter(10.0);
        assert_eq!(paint.stroke_miter(), 10.0);

        // Negative values should be ignored
        paint.set_stroke_miter(-1.0);
        assert_eq!(paint.stroke_miter(), 10.0);
    }

    #[test]
    fn test_cap_and_join() {
        let mut paint = Paint::new();
        paint.set_stroke_cap(Cap::Round);
        assert_eq!(paint.stroke_cap(), Cap::Round);

        paint.set_stroke_join(Join::Bevel);
        assert_eq!(paint.stroke_join(), Join::Bevel);
    }

    #[test]
    fn test_anti_alias_and_dither() {
        let mut paint = Paint::new();
        paint.set_anti_alias(true);
        assert!(paint.is_anti_alias());

        paint.set_dither(true);
        assert!(paint.is_dither());

        paint.set_anti_alias(false);
        assert!(!paint.is_anti_alias());
    }

    #[test]
    fn test_reset() {
        let mut paint = Paint::new();
        paint.set_stroke_width(5.0);
        paint.set_anti_alias(true);
        paint.reset();
        assert_eq!(paint.stroke_width(), 0.0);
        assert!(!paint.is_anti_alias());
    }

    #[test]
    fn test_get_fill_path_empty() {
        let paint = Paint::new();
        let src = Path::new();
        let mut dst = Path::new();
        let result = paint.get_fill_path(&src, &mut dst, None, 1.0);
        // Empty path should return false (no valid fill path)
        assert!(!result);
    }

    #[test]
    fn test_style_validation() {
        assert!(Style::is_valid(0)); // Fill
        assert!(Style::is_valid(1)); // Stroke
        assert!(Style::is_valid(2)); // StrokeAndFill
        assert!(!Style::is_valid(3)); // Invalid
        assert!(!Style::is_valid(255)); // Invalid
    }
}
