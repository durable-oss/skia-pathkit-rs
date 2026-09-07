//! Path measurement utilities for computing length, position, and tangent along paths.
//!
//! Ported from `include/core/SkPathMeasure.h` and `src/core/SkPathMeasure.cpp`.

use super::contour_measure::{ContourMeasure, ContourMeasureIter};
use super::matrix::Matrix;
use super::path::Path;
use super::point::{Point, Vector};
use super::scalar::Scalar;

/// Flags for getMatrix to control what transformation components to compute.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatrixFlags {
    /// Compute position only.
    GetPosition = 0x01,
    /// Compute tangent only.
    GetTangent = 0x02,
    /// Compute both position and tangent.
    GetPosAndTan = 0x03,
}

/// SkPathMeasure allows computing position and tangent at any point along a path,
/// extracting path segments, and measuring contour lengths.
pub struct PathMeasure {
    iter: ContourMeasureIter,
    contour: Option<ContourMeasure>,
}

impl PathMeasure {
    /// Create a new PathMeasure with default (empty) state.
    pub fn new() -> Self {
        Self {
            iter: ContourMeasureIter::new(),
            contour: None,
        }
    }

    /// Create a PathMeasure from a path with optional closing and resolution scale.
    pub fn from_path(path: &Path, force_closed: bool, res_scale: Scalar) -> Self {
        let mut iter = ContourMeasureIter::from_path(path, force_closed, res_scale);
        let contour = iter.next();
        Self { iter, contour }
    }

    /// Set the path to measure with optional closing flag.
    pub fn set_path(&mut self, path: &Path, force_closed: bool) {
        self.iter.reset(path, force_closed, 1.0);
        self.contour = self.iter.next();
    }

    /// Return the length of the current contour.
    pub fn length(&self) -> Scalar {
        self.contour.as_ref().map(|c| c.length()).unwrap_or(0.0)
    }

    /// Compute position and unit tangent at the given distance.
    /// Distance is pinned to [0, length]. Returns false if no contour or invalid distance.
    pub fn get_pos_tan(&self, distance: Scalar, pos: &mut Point, tangent: &mut Vector) -> bool {
        self.contour
            .as_ref()
            .map(|c| c.get_pos_tan(distance, Some(pos), Some(tangent)))
            .unwrap_or(false)
    }

    /// Compute transformation matrix at the given distance based on flags.
    /// Returns false if no contour or invalid distance.
    pub fn get_matrix(&self, distance: Scalar, flags: MatrixFlags, matrix: &mut Matrix) -> bool {
        self.contour
            .as_ref()
            .map(|c| {
                c.get_matrix(
                    distance,
                    matrix,
                    match flags {
                        MatrixFlags::GetPosition => {
                            super::contour_measure::MatrixFlags::GET_POSITION
                        }
                        MatrixFlags::GetTangent => super::contour_measure::MatrixFlags::GET_TANGENT,
                        MatrixFlags::GetPosAndTan => {
                            super::contour_measure::MatrixFlags::GET_POS_AND_TAN
                        }
                    },
                )
            })
            .unwrap_or(false)
    }

    /// Extract a path segment from startD to stopD.
    /// Returns false if startD > stopD or if the segment is zero-length.
    pub fn get_segment(
        &self,
        start_d: Scalar,
        stop_d: Scalar,
        start_with_move_to: bool,
    ) -> Option<Path> {
        let c = self.contour.as_ref()?;
        let mut dst = Path::new();
        if c.get_segment(start_d, stop_d, &mut dst, start_with_move_to) {
            Some(dst)
        } else {
            None
        }
    }

    /// Return true if the current contour is closed.
    pub fn is_closed(&self) -> bool {
        self.contour
            .as_ref()
            .map(|c| c.is_closed())
            .unwrap_or(false)
    }

    /// Advance to the next contour in the path.
    /// Returns false if no more contours remain.
    pub fn next_contour(&mut self) -> bool {
        self.contour = self.iter.next();
        self.contour.is_some()
    }
}

impl Default for PathMeasure {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_empty() {
        let pm = PathMeasure::new();
        assert_eq!(pm.length(), 0.0);
        assert!(!pm.is_closed());
    }

    #[test]
    fn test_from_path_line() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        assert!((pm.length() - 100.0).abs() < 1e-6);
        assert!(!pm.is_closed());
    }

    #[test]
    fn test_from_path_closed() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        path.line_to(100.0, 100.0);
        path.line_to(0.0, 100.0);
        path.close();
        let pm = PathMeasure::from_path(&path, false, 1.0);
        assert!((pm.length() - 400.0).abs() < 1e-6);
        assert!(pm.is_closed());
    }

    #[test]
    fn test_set_path() {
        let mut pm = PathMeasure::new();
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(50.0, 50.0);
        pm.set_path(&path, false);
        assert!((pm.length() - 70.710678).abs() < 1e-4);
    }

    #[test]
    fn test_get_pos_tan_line() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        let mut pos = Point::default();
        let mut tan = Vector::default();
        assert!(pm.get_pos_tan(50.0, &mut pos, &mut tan));
        assert!((pos.x - 50.0).abs() < 1e-6);
        assert!((pos.y - 0.0).abs() < 1e-6);
        assert!((tan.x - 1.0).abs() < 1e-6);
        assert!((tan.y - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_get_pos_tan_diagonal() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 100.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        let mut pos = Point::default();
        let mut tan = Vector::default();
        assert!(pm.get_pos_tan(50.0, &mut pos, &mut tan));
        assert!((pos.x - 35.355339).abs() < 1e-4);
        assert!((pos.y - 35.355339).abs() < 1e-4);
        let norm = (tan.x * tan.x + tan.y * tan.y).sqrt();
        assert!((norm - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_get_pos_tan_out_of_bounds() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        let mut pos = Point::default();
        let mut tan = Vector::default();
        assert!(pm.get_pos_tan(150.0, &mut pos, &mut tan));
        assert!((pos.x - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_get_matrix_position_only() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        let mut matrix = Matrix::identity();
        let result = pm.get_matrix(50.0, MatrixFlags::GetPosition, &mut matrix);
        assert!(result);
        assert!((matrix.get(2) - 50.0).abs() < 1e-6);
    }

    #[test]
    fn test_get_matrix_tangent_only() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 100.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        let mut matrix = Matrix::identity();
        let result = pm.get_matrix(50.0, MatrixFlags::GetTangent, &mut matrix);
        assert!(result);
        let expected = (0.5f32).sqrt();
        // set_sin_cos_pivot lays out a rotation matrix with skew_x = -sin
        // and scale_y = cos; for this 45-degree tangent sin == cos, but
        // the sign on the skew term still differs.
        assert!((matrix.get(1) - -expected).abs() < 1e-6);
        assert!((matrix.get(4) - expected).abs() < 1e-6);
    }

    #[test]
    fn test_get_matrix_both() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 100.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        let mut matrix = Matrix::identity();
        let result = pm.get_matrix(50.0, MatrixFlags::GetPosAndTan, &mut matrix);
        assert!(result);
        assert!((matrix.get(2) - 35.355339).abs() < 1e-4);
        assert!((matrix.get(5) - 35.355339).abs() < 1e-4);
    }

    #[test]
    fn test_get_segment() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        let segment = pm.get_segment(20.0, 80.0, true);
        assert!(segment.is_some());
        let segment = segment.unwrap();
        assert_eq!(segment.verbs().len(), 2);
    }

    #[test]
    fn test_get_segment_no_move_to() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        let segment = pm.get_segment(20.0, 80.0, false);
        assert!(segment.is_some());
    }

    #[test]
    fn test_get_segment_invalid_range() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        let pm = PathMeasure::from_path(&path, false, 1.0);
        let segment = pm.get_segment(80.0, 20.0, true);
        assert!(segment.is_none());
    }

    #[test]
    fn test_next_contour() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        path.move_to(0.0, 100.0);
        path.line_to(100.0, 100.0);
        let mut pm = PathMeasure::from_path(&path, false, 1.0);
        assert!((pm.length() - 100.0).abs() < 1e-6);
        assert!(pm.next_contour());
        assert!(!pm.next_contour());
    }

    #[test]
    fn test_iter_next() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        path.move_to(0.0, 100.0);
        path.line_to(100.0, 100.0);
        let mut iter = ContourMeasureIter::from_path(&path, false, 1.0);
        assert!(iter.next().is_some());
        assert!(iter.next().is_some());
        assert!(iter.next().is_none());
    }

    #[test]
    fn test_iter_reset() {
        let mut path1 = Path::new();
        path1.move_to(0.0, 0.0);
        path1.line_to(100.0, 0.0);
        let mut path2 = Path::new();
        path2.move_to(0.0, 0.0);
        path2.line_to(200.0, 0.0);
        let mut iter = ContourMeasureIter::from_path(&path1, false, 1.0);
        assert!(iter.next().is_some());
        iter.reset(&path2, false, 1.0);
        assert!(iter.next().is_some());
    }

    #[test]
    fn test_force_closed() {
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(100.0, 0.0);
        let pm = PathMeasure::from_path(&path, true, 1.0);
        assert!(pm.is_closed());
        assert!((pm.length() - 200.0).abs() < 1e-6);
    }

    #[test]
    fn test_empty_path() {
        let path = Path::new();
        let pm = PathMeasure::from_path(&path, false, 1.0);
        assert_eq!(pm.length(), 0.0);
    }
}
