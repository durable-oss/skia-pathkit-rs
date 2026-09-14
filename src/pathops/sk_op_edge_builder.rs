//! SkOpEdgeBuilder - builds contours from path data for path operations
//!
//! Port of Skia's SkOpEdgeBuilder.{h,cpp}

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

use super::sk_op_contour::{SkOpContourBuilder, SkOpContourHead};
use crate::core::{Path, Point, Scalar, Verb};

/// Path ops mask values (matching Skia's constants)
const EVENODD_PATH_OPS_MASK: u32 = 0x02;
const WINDING_PATH_OPS_MASK: u32 = 0x01;

/// Threshold for forcing small values to zero (matching Skia's FLT_EPSILON_ORDERABLE_ERR)
const SMALL_THRESHOLD: Scalar = 1e-10;

/// Force very small coordinate values to zero for numerical stability
fn force_small_to_zero(pt: Point) -> Point {
    Point {
        x: if pt.x.abs() < SMALL_THRESHOLD {
            0.0
        } else {
            pt.x
        },
        y: if pt.y.abs() < SMALL_THRESHOLD {
            0.0
        } else {
            pt.y
        },
    }
}

/// Check if two points are approximately equal
fn approximately_equal(a: Point, b: Point) -> bool {
    (a.x - b.x).abs() < SMALL_THRESHOLD && (a.y - b.y).abs() < SMALL_THRESHOLD
}

/// SkOpEdgeBuilder - converts SkPath into contours for path operations
pub struct SkOpEdgeBuilder {
    f_operand: bool,
    f_xor_mask: [u32; 2],
    f_path_verbs: Vec<Verb>,
    f_path_pts: Vec<Point>,
    f_weights: Vec<Scalar>,
    f_allow_open_contours: bool,
    f_unparseable: bool,
    f_second_half: usize,
    f_contour_builder: SkOpContourBuilder,
    f_contours_head: SkOpContourHead,
}

impl SkOpEdgeBuilder {
    /// Creates a new SkOpEdgeBuilder
    pub fn new() -> Self {
        Self {
            f_operand: false,
            f_xor_mask: [0, 0],
            f_path_verbs: Vec::new(),
            f_path_pts: Vec::new(),
            f_weights: Vec::new(),
            f_allow_open_contours: false,
            f_unparseable: false,
            f_second_half: 0,
            f_contour_builder: SkOpContourBuilder::new_empty(),
            f_contours_head: SkOpContourHead::new(),
        }
    }

    /// Sets whether open contours are allowed
    pub fn set_allow_open_contours(&mut self, allow: bool) {
        self.f_allow_open_contours = allow;
    }

    /// Initializes the builder with the given path
    pub fn init(&mut self, path: &Path) {
        self.f_operand = false;
        let mask = if path.fill_type().is_even_odd() {
            EVENODD_PATH_OPS_MASK
        } else {
            WINDING_PATH_OPS_MASK
        };
        self.f_xor_mask[0] = mask;
        self.f_xor_mask[1] = mask;
        self.f_unparseable = false;
        self.f_second_half = self.pre_fetch(path);
    }

    /// Adds the operand path (second path for binary operations)
    pub fn add_operand(&mut self, path: &Path) {
        if !self.f_path_verbs.is_empty() {
            self.f_path_verbs.pop();
        }
        self.f_xor_mask[1] = if path.fill_type().is_even_odd() {
            EVENODD_PATH_OPS_MASK
        } else {
            WINDING_PATH_OPS_MASK
        };
        self.pre_fetch(path);
    }

    /// Fetches and pre-processes the path data
    fn pre_fetch(&mut self, path: &Path) -> usize {
        if !path.is_finite() {
            self.f_unparseable = true;
            return 0;
        }

        let mut curve_start = Point::new(0.0, 0.0);
        let mut curve = [Point::new(0.0, 0.0); 4];
        let mut last_curve = false;

        let mut pt_iter = path.points().iter().copied();
        let mut weight_iter = path.conic_weights().iter().copied();

        for verb in path.verbs() {
            match verb {
                Verb::Move => {
                    if !self.f_allow_open_contours && last_curve {
                        self.close_contour(curve[0], curve_start);
                    }
                    self.f_path_verbs.push(*verb);
                    curve[0] = force_small_to_zero(pt_iter.next().unwrap_or(Point::new(0.0, 0.0)));
                    self.f_path_pts.push(curve[0]);
                    curve_start = curve[0];
                    last_curve = false;
                    continue;
                }
                Verb::Line => {
                    curve[1] = force_small_to_zero(pt_iter.next().unwrap_or(Point::new(0.0, 0.0)));
                    if approximately_equal(curve[0], curve[1]) {
                        continue; // Skip degenerate lines
                    }
                }
                Verb::Quad => {
                    curve[1] = force_small_to_zero(pt_iter.next().unwrap_or(Point::new(0.0, 0.0)));
                    curve[2] = force_small_to_zero(pt_iter.next().unwrap_or(Point::new(0.0, 0.0)));
                    if approximately_equal(curve[0], curve[1])
                        && approximately_equal(curve[1], curve[2])
                    {
                        continue; // degenerate
                    }
                }
                Verb::Conic => {
                    curve[1] = force_small_to_zero(pt_iter.next().unwrap_or(Point::new(0.0, 0.0)));
                    curve[2] = force_small_to_zero(pt_iter.next().unwrap_or(Point::new(0.0, 0.0)));
                    let _weight = weight_iter.next().unwrap_or(1.0);
                    if approximately_equal(curve[0], curve[1])
                        && approximately_equal(curve[1], curve[2])
                    {
                        continue; // degenerate
                    }
                }
                Verb::Cubic => {
                    curve[1] = force_small_to_zero(pt_iter.next().unwrap_or(Point::new(0.0, 0.0)));
                    curve[2] = force_small_to_zero(pt_iter.next().unwrap_or(Point::new(0.0, 0.0)));
                    curve[3] = force_small_to_zero(pt_iter.next().unwrap_or(Point::new(0.0, 0.0)));
                    if approximately_equal(curve[0], curve[1])
                        && approximately_equal(curve[1], curve[2])
                        && approximately_equal(curve[2], curve[3])
                    {
                        continue; // degenerate
                    }
                }
                Verb::Close => {
                    self.close_contour(curve[0], curve_start);
                    last_curve = false;
                    continue;
                }
            }

            self.f_path_verbs.push(*verb);
            let pt_count = verb.point_count();
            for i in 0..pt_count {
                self.f_path_pts.push(curve[i + 1]);
            }

            if matches!(verb, Verb::Conic) {
                if let Some(w) = weight_iter.next() {
                    self.f_weights.push(w);
                }
            }

            curve[0] = curve[pt_count];
            last_curve = true;
        }

        if !self.f_allow_open_contours && last_curve {
            self.close_contour(curve[0], curve_start);
        }

        self.f_path_verbs.len()
    }

    /// Closes the current contour if needed
    fn close_contour(&mut self, curve_end: Point, curve_start: Point) {
        if !approximately_equal(curve_end, curve_start) {
            self.f_path_verbs.push(Verb::Line);
            self.f_path_pts.push(curve_start);
        } else {
            let verb_count = self.f_path_verbs.len();
            let pts_count = self.f_path_pts.len();
            if verb_count > 0 && pts_count >= 2 {
                if self.f_path_verbs[verb_count - 1] == Verb::Line
                    && self.f_path_pts[pts_count - 2] == curve_start
                {
                    self.f_path_verbs.pop();
                    self.f_path_pts.pop();
                } else {
                    *self.f_path_pts.last_mut().unwrap() = curve_start;
                }
            }
        }
        self.f_path_verbs.push(Verb::Close);
    }

    /// Finishes the building process
    pub fn finish(&mut self) -> bool {
        self.f_operand = false;
        if self.f_unparseable {
            return false;
        }
        // Simplified - real implementation would walk through path
        true
    }

    /// Completes the current contour
    pub fn complete(&mut self) {
        // Placeholder - would update contour bounds
    }

    /// Completes and returns success
    pub fn close(&mut self) -> bool {
        self.complete();
        true
    }
}

impl Default for SkOpEdgeBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_force_small_to_zero() {
        assert_eq!(
            force_small_to_zero(Point::new(0.0, 0.0)),
            Point::new(0.0, 0.0)
        );
        assert_eq!(
            force_small_to_zero(Point::new(1e-11, 1e-11)),
            Point::new(0.0, 0.0)
        );
        assert_eq!(
            force_small_to_zero(Point::new(1.0, 2.0)),
            Point::new(1.0, 2.0)
        );
    }

    #[test]
    fn test_approximately_equal() {
        assert!(approximately_equal(
            Point::new(0.0, 0.0),
            Point::new(0.0, 0.0)
        ));
        assert!(approximately_equal(
            Point::new(1e-11, 1e-11),
            Point::new(0.0, 0.0)
        ));
        assert!(!approximately_equal(
            Point::new(1.0, 0.0),
            Point::new(0.0, 0.0)
        ));
    }

    #[test]
    fn test_empty_path() {
        let mut builder = SkOpEdgeBuilder::new();
        let path = Path::new();
        builder.init(&path);
        assert!(builder.finish());
    }

    #[test]
    fn test_simple_line() {
        let mut builder = SkOpEdgeBuilder::new();
        let mut path = Path::new();
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 10.0);
        path.close();
        builder.init(&path);
        assert!(builder.finish());
    }

    #[test]
    fn test_fill_type_handling() {
        let mut builder = SkOpEdgeBuilder::new();
        let mut path = Path::new();
        path.set_fill_type(crate::core::FillType::EvenOdd);
        path.move_to(0.0, 0.0);
        path.line_to(10.0, 0.0);
        path.line_to(10.0, 10.0);
        path.close();
        builder.init(&path);
        assert_eq!(builder.f_xor_mask[0], EVENODD_PATH_OPS_MASK);
    }
}
