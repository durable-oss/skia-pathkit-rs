//! Paint private utilities.
//!
//! Ported from `src/core/SkPaintPriv.h` / `src/core/SkPaintPriv.cpp`.

use super::{matrix, scalar, Matrix, Point};

/// Private helpers for [`super::paint::Paint`].
pub struct PaintPriv;

impl PaintPriv {
    /// Computes the resolution scale factor for stroking based on the matrix.
    ///
    /// This computes the maximum scale factor along the x and y axes by
    /// computing the length of the transformed basis vectors. If either
    /// component is not finite (NaN or infinity), the function returns 1.
    ///
    /// Note: Perspective transforms are not specially handled; the function
    /// treats them the same as affine transforms.
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::core::Matrix;
    /// use pathkit::core::PaintPriv;
    ///
    /// let m = Matrix::scale(2.0, 3.0);
    /// let scale = PaintPriv::compute_res_scale_for_stroking(&m);
    /// assert!((scale - 3.0).abs() < 1e-6);
    /// ```
    #[must_use]
    pub fn compute_res_scale_for_stroking(matrix: &Matrix) -> f32 {
        // Compute the length of the transformed x and y basis vectors
        let sx =
            Point::distance_to_origin(matrix.get(matrix::M_SCALE_X), matrix.get(matrix::M_SKEW_Y));
        let sy =
            Point::distance_to_origin(matrix.get(matrix::M_SKEW_X), matrix.get(matrix::M_SCALE_Y));

        if scalar::are_finite(sx, sy) {
            let scale = sx.max(sy);
            if scale > 0.0 {
                return scale;
            }
        }
        1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_matrix_returns_one() {
        let m = Matrix::identity();
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        assert_eq!(scale, 1.0);
    }

    #[test]
    fn uniform_scale_returns_that_scale() {
        let m = Matrix::scale(2.0, 2.0);
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        assert!((scale - 2.0).abs() < 1e-6);
    }

    #[test]
    fn non_uniform_scale_returns_max() {
        let m = Matrix::scale(2.0, 3.0);
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        assert!((scale - 3.0).abs() < 1e-6);
    }

    #[test]
    fn translation_matrix_returns_one() {
        let m = Matrix::translate(5.0, 10.0);
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        assert_eq!(scale, 1.0);
    }

    #[test]
    fn rotation_matrix_returns_one() {
        let m = Matrix::rotate_deg(45.0);
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        assert!((scale - 1.0).abs() < 1e-6);
    }

    #[test]
    fn zero_scale_returns_one() {
        let m = Matrix::scale(0.0, 0.0);
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        assert_eq!(scale, 1.0);
    }

    #[test]
    fn skew_matrix_computes_correct_scale() {
        let mut m = Matrix::identity();
        m.set(matrix::M_SKEW_X, 1.0);
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        let expected = Point::distance_to_origin(1.0, 1.0);
        assert!((scale - expected).abs() < 1e-6);
    }

    #[test]
    fn perspective_matrix_returns_computed_scale() {
        let m = Matrix::scale(2.0, 3.0);
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        assert!((scale - 3.0).abs() < 1e-6);
    }

    #[test]
    fn large_scale_returns_correct() {
        let m = Matrix::scale(100.0, 200.0);
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        assert!((scale - 200.0).abs() < 1e-6);
    }

    #[test]
    fn combined_transform() {
        let mut m = Matrix::scale(1.5, 2.5);
        m.post_translate(100.0, 200.0);
        let scale = PaintPriv::compute_res_scale_for_stroking(&m);
        assert!((scale - 2.5).abs() < 1e-6);
    }
}
