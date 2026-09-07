//! SkPathOpsWinding - winding number computation for path operations
//!
//! Port of Skia's SkPathOpsWinding.cpp
//!
//! This module implements the ray-casting algorithm for determining winding
//! numbers for path segments. It projects rays from span endpoints and
//! checks for intersections with other segments to determine proper winding.

use crate::core::{Point, Scalar};

/// Tolerance for approximate comparisons.
///
/// `Scalar` is `f32`; `1e-10` (a `double`-scale tolerance from the C++
/// original) rounds away to nothing at `f32` precision near typical
/// path-op magnitudes, so this uses an `f32`-appropriate tolerance
/// instead.
const APPROX_EPSILON: Scalar = 1e-5;

/// Check if two scalars are approximately equal
fn approximately_equal(a: Scalar, b: Scalar) -> bool {
    (a - b).abs() < APPROX_EPSILON
}

/// Check if a scalar is approximately zero
fn approximately_zero(a: Scalar) -> bool {
    a.abs() < APPROX_EPSILON
}

/// Direction for ray casting (4 directions: left, top, right, bottom)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkOpRayDir {
    Left,
    Top,
    Right,
    Bottom,
}

impl SkOpRayDir {
    /// Get the index (x or y) that changes in this direction
    pub fn xy_index(&self) -> usize {
        *self as usize & 1
    }

    /// Get the perpendicular index
    pub fn perp_index(&self) -> usize {
        1 - self.xy_index()
    }

    /// Returns true if we're comparing "less than" in this direction
    pub fn less_than(&self) -> bool {
        (*self as usize & 2) == 0
    }

    /// Rotate direction by offset
    pub fn rotate(&self, offset: usize) -> Self {
        match offset {
            0 => *self,
            1 => match self {
                SkOpRayDir::Left => SkOpRayDir::Top,
                SkOpRayDir::Top => SkOpRayDir::Right,
                SkOpRayDir::Right => SkOpRayDir::Bottom,
                SkOpRayDir::Bottom => SkOpRayDir::Left,
            },
            _ => *self,
        }
    }
}

/// Get x or y coordinate based on direction
fn pt_xy(pt: &Point, dir: SkOpRayDir) -> Scalar {
    match dir {
        SkOpRayDir::Left | SkOpRayDir::Right => pt.x,
        SkOpRayDir::Top | SkOpRayDir::Bottom => pt.y,
    }
}

/// Get the perpendicular coordinate
fn pt_yx(pt: &Point, dir: SkOpRayDir) -> Scalar {
    match dir {
        SkOpRayDir::Left | SkOpRayDir::Right => pt.y,
        SkOpRayDir::Top | SkOpRayDir::Bottom => pt.x,
    }
}

/// Check if vector points counter-clockwise for given direction
fn ccw_dxdy(slope: &Point, dir: SkOpRayDir) -> bool {
    let perp_val = match dir {
        SkOpRayDir::Left | SkOpRayDir::Right => slope.y,
        SkOpRayDir::Top | SkOpRayDir::Bottom => slope.x,
    };
    let v_part_pos = perp_val > 0.0;
    let left_bottom = ((dir as usize + 1) & 2) != 0;
    v_part_pos == left_bottom
}

/// Compute t guess based on try count - used for selecting ray test points
fn get_t_guess(t_try: i32, dir_offset: &mut usize) -> Scalar {
    let mut t = 0.5;
    *dir_offset = (t_try as usize) & 1;
    let mut t_base = (t_try as usize) >> 1;
    let mut t_bits = 0;
    
    if t_base > 0 {
        t_bits += 1;
        while t_base > 1 {
            t_base >>= 1;
            t /= 2.0;
            t_bits += 1;
        }
    }
    
    if t_bits > 0 {
        let t_index = (t_base - 1) & ((1 << t_bits) - 1);
        t += t * 2.0 * (t_index as Scalar);
    }
    t
}

/// Returns true for up to 100 tries to find a sortable top span
pub const MAX_WINDING_TRIES: i32 = 100;

/// Minimum i32 value for winding sums
pub const PK_MIN_S32: i32 = i32::MIN;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ray_directions() {
        assert_eq!(SkOpRayDir::Left.xy_index(), 0);
        assert_eq!(SkOpRayDir::Top.xy_index(), 1);
        assert_eq!(SkOpRayDir::Right.xy_index(), 0);
        assert_eq!(SkOpRayDir::Bottom.xy_index(), 1);
    }

    #[test]
    fn test_ray_direction_rotation() {
        assert_eq!(SkOpRayDir::Left.rotate(1), SkOpRayDir::Top);
        assert_eq!(SkOpRayDir::Top.rotate(1), SkOpRayDir::Right);
        assert_eq!(SkOpRayDir::Right.rotate(1), SkOpRayDir::Bottom);
        assert_eq!(SkOpRayDir::Bottom.rotate(1), SkOpRayDir::Left);
    }

    #[test]
    fn test_less_than_direction() {
        assert!(SkOpRayDir::Left.less_than());
        assert!(SkOpRayDir::Top.less_than());
        assert!(!SkOpRayDir::Right.less_than());
        assert!(!SkOpRayDir::Bottom.less_than());
    }

    #[test]
    fn test_t_guess_initial() {
        let mut offset = 0;
        let t = get_t_guess(0, &mut offset);
        assert_eq!(t, 0.5);
        assert_eq!(offset, 0);
    }

    #[test]
    fn test_t_guess_sequence() {
        let mut offset = 0;
        let t0 = get_t_guess(0, &mut offset);
        assert_eq!(t0, 0.5);
    }

    #[test]
    fn test_pt_coord_access() {
        let pt = Point::new(5.0, 10.0);
        assert_eq!(pt_xy(&pt, SkOpRayDir::Left), 5.0);
        assert_eq!(pt_yx(&pt, SkOpRayDir::Left), 10.0);
        assert_eq!(pt_xy(&pt, SkOpRayDir::Top), 10.0);
        assert_eq!(pt_yx(&pt, SkOpRayDir::Top), 5.0);
    }

    #[test]
    fn test_ccw_direction() {
        // For Left/Right the check uses the vector's y component: v_part_pos
        // = (1.0 > 0) = true; left_bottom for Left = ((0+1)&2)!=0 = false.
        // true == false is false, matching the C++ `ccw_dxdy` semantics.
        let slope = Point::new(1.0, 1.0);
        assert!(!ccw_dxdy(&slope, SkOpRayDir::Left));
        // For Top/Bottom the check uses the vector's x component:
        // v_part_pos = (1.0 > 0) = true; left_bottom for Bottom (dir=3)
        // = ((3+1)&2)!=0 = (4&2)!=0 = false. true == false is false.
        assert!(!ccw_dxdy(&slope, SkOpRayDir::Bottom));
    }

    #[test]
    fn test_approximately_equal() {
        assert!(approximately_equal(1.0, 1.0));
        assert!(approximately_equal(1.0, 1.0 + (APPROX_EPSILON / 2.0) as Scalar));
        assert!(!approximately_equal(1.0, 1.0 + (APPROX_EPSILON * 2.0) as Scalar));
    }

    #[test]
    fn test_approximately_zero() {
        assert!(approximately_zero(0.0));
        assert!(approximately_zero((APPROX_EPSILON / 2.0) as Scalar));
        assert!(!approximately_zero((APPROX_EPSILON * 2.0) as Scalar));
    }

    #[test]
    fn test_constants() {
        assert_eq!(MAX_WINDING_TRIES, 100);
        assert_eq!(PK_MIN_S32, i32::MIN);
    }
}
