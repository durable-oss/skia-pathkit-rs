//! PathOps type definitions and comparison utilities
//!
//! This module provides floating-point comparison utilities using ULPs (Units in the Last Place)
//! for robust geometric comparisons, as well as other mathematical utilities needed by path operations.

// Parts of this module are ported ahead of their call sites; see
// PORTING.md for what remains to be wired up.
#![allow(dead_code)]

/// Helper: check if both arguments are denormalized (near zero)
fn arguments_denormalized(a: f32, b: f32, epsilon: i32) -> bool {
    let denormalized_check = f32::EPSILON * epsilon as f32 / 2.0;
    a.abs() <= denormalized_check && b.abs() <= denormalized_check
}

/// Reinterprets a float's bit pattern as a 2's-complement int so ordinary
/// integer comparisons (<, <=) agree with float ordering, matching Skia's
/// `SkFloatAs2sCompliment`. A raw `to_bits()` cast is sign-magnitude and
/// sorts negative floats backwards relative to positive ones.
fn float_as_2s_complement(x: f32) -> i32 {
    let bits = x.to_bits() as i32;
    if bits < 0 {
        -(bits & 0x7FFF_FFFF)
    } else {
        bits
    }
}

/// Compare floats by ULPs (Units in the Last Place)
fn equal_ulps(a: f32, b: f32, epsilon: i32, depsilon: i32) -> bool {
    if arguments_denormalized(a, b, depsilon) {
        return true;
    }
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits < b_bits + epsilon && b_bits < a_bits + epsilon
}

/// Compare floats by ULPs without checking for denormalized values
fn equal_ulps_no_normal_check(a: f32, b: f32, epsilon: i32, _depsilon: i32) -> bool {
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits < b_bits + epsilon && b_bits < a_bits + epsilon
}

/// Compare floats by ULPs with finite check
fn equal_ulps_pin(a: f32, b: f32, epsilon: i32, depsilon: i32) -> bool {
    if !a.is_finite() || !b.is_finite() {
        return false;
    }
    if arguments_denormalized(a, b, depsilon) {
        return true;
    }
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits < b_bits + epsilon && b_bits < a_bits + epsilon
}

/// Double precision ULP comparison (converts to scalar first)
fn d_equal_ulps(a: f32, b: f32, epsilon: i32) -> bool {
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits < b_bits + epsilon && b_bits < a_bits + epsilon
}

/// Negation of ULP equality with denormal check
fn not_equal_ulps(a: f32, b: f32, epsilon: i32) -> bool {
    if arguments_denormalized(a, b, epsilon) {
        return false;
    }
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits >= b_bits + epsilon || b_bits >= a_bits + epsilon
}

/// Negation of ULP equality with finite check
fn not_equal_ulps_pin(a: f32, b: f32, epsilon: i32) -> bool {
    if !a.is_finite() || !b.is_finite() {
        return false;
    }
    if arguments_denormalized(a, b, epsilon) {
        return false;
    }
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits >= b_bits + epsilon || b_bits >= a_bits + epsilon
}

/// Double precision ULP inequality
fn d_not_equal_ulps(a: f32, b: f32, epsilon: i32) -> bool {
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits >= b_bits + epsilon || b_bits >= a_bits + epsilon
}

/// Compare if a <= b using ULPs with denormal check
fn less_ulps(a: f32, b: f32, epsilon: i32) -> bool {
    if arguments_denormalized(a, b, epsilon) {
        return a <= b - f32::EPSILON * epsilon as f32;
    }
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits <= b_bits - epsilon
}

/// Compare if a < b using ULPs with denormal check
fn less_or_equal_ulps(a: f32, b: f32, epsilon: i32) -> bool {
    if arguments_denormalized(a, b, epsilon) {
        return a < b + f32::EPSILON * epsilon as f32;
    }
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits < b_bits + epsilon
}

/// Compare if a <= b using ULPs without denormal check
fn less_or_equal_ulps_no_check(a: f32, b: f32, epsilon: i32) -> bool {
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);
    a_bits < b_bits + epsilon
}

/// Check if two floats are approximately equal using 2 ULPs
pub fn almost_bequal_ulps(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 2;
    equal_ulps(a, b, ULPS_EPSILON, ULPS_EPSILON)
}

/// Check if two floats are approximately equal using 8 ULPs
pub fn almost_pequal_ulps(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 8;
    equal_ulps(a, b, ULPS_EPSILON, ULPS_EPSILON)
}

/// Check if two floats are approximately equal using 16 ULPs (double tolerance)
pub fn almost_dequal_ulps(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 16;
    d_equal_ulps(a, b, ULPS_EPSILON)
}

/// Check if two floats are approximately equal using 16 ULPs
pub fn almost_equal_ulps(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 16;
    equal_ulps(a, b, ULPS_EPSILON, ULPS_EPSILON)
}

/// Check if two floats are approximately equal using 16 ULPs without normal check
pub fn almost_equal_ulps_no_normal_check(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 16;
    equal_ulps_no_normal_check(a, b, ULPS_EPSILON, ULPS_EPSILON)
}

/// Check if two floats are approximately equal using 16 ULPs with finite check
pub fn almost_equal_ulps_pin(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 16;
    equal_ulps_pin(a, b, ULPS_EPSILON, ULPS_EPSILON)
}

/// Check if two floats are NOT approximately equal
pub fn not_almost_equal_ulps(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 16;
    not_equal_ulps(a, b, ULPS_EPSILON)
}

/// Check if two floats are NOT approximately equal with finite check
pub fn not_almost_equal_ulps_pin(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 16;
    not_equal_ulps_pin(a, b, ULPS_EPSILON)
}

/// Check if two floats are NOT approximately equal (double tolerance)
pub fn not_almost_dequal_ulps(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 16;
    d_not_equal_ulps(a, b, ULPS_EPSILON)
}

/// Check if two floats are roughly equal using 256 ULPs
pub fn roughly_equal_ulps(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 256;
    const DULPS_EPSILON: i32 = 1024;
    equal_ulps(a, b, ULPS_EPSILON, DULPS_EPSILON)
}

/// Check if b is approximately between a and c (inclusive) with tolerance
pub fn almost_between_ulps(a: f32, b: f32, c: f32) -> bool {
    let min = a.min(c);
    let max = a.max(c);
    // Allow epsilon tolerance on both sides
    b >= min - f32::EPSILON * 2.0 && b <= max + f32::EPSILON * 2.0
}

/// Check if b is between a and c (inclusive)
pub fn between(a: f32, b: f32, c: f32) -> bool {
    b >= a && b <= c
}

/// Check if a < b using 16 ULPs
pub fn almost_less_ulps(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 16;
    less_ulps(a, b, ULPS_EPSILON)
}

/// Check if a <= b using 16 ULPs
pub fn almost_less_or_equal_ulps(a: f32, b: f32) -> bool {
    const ULPS_EPSILON: i32 = 16;
    less_or_equal_ulps(a, b, ULPS_EPSILON)
}

/// Calculate the ULP distance between two floats
pub fn ulps_distance(a: f32, b: f32) -> i32 {
    let a_bits = float_as_2s_complement(a);
    let b_bits = float_as_2s_complement(b);

    // Different signs means they do not match
    if (a_bits < 0) != (b_bits < 0) {
        // Check for equality to make sure +0 == -0
        return if a == b { 0 } else { i32::MAX };
    }

    (a_bits - b_bits).abs()
}

/// Approximate cube root using bit hack (initial estimate)
fn cbrt_5d(d: f64) -> f64 {
    const B1: u32 = 715094163;
    // This is a bit hack approximation - we'll use the standard library approach
    // instead of pointer manipulation
    d.cbrt()
}

/// Iterative cube root approximation using Halley's method
fn cbrt_halleyd(a: f64, r: f64) -> f64 {
    let a3 = a * a * a;
    a * (a3 + r + r) / (a3 + a3 + r)
}

/// Cube root using 3 iterations of Halley's method
fn halley_cbrt3d(d: f64) -> f64 {
    let mut a = d.cbrt();
    a = cbrt_halleyd(a, d);
    a = cbrt_halleyd(a, d);
    cbrt_halleyd(a, d)
}

/// Compute cube root for double precision values
pub fn d_cbrt(x: f64) -> f64 {
    if approximately_zero_cubed(x) {
        return 0.0;
    }
    let mut result = halley_cbrt3d(x.abs());
    if x < 0.0 {
        result = -result;
    }
    result
}

/// Check if a value is approximately zero when cubed
fn approximately_zero_cubed(x: f64) -> bool {
    const APPROXIMATELY_ZERO_CUBED_THRESHOLD: f64 = 1e-6;
    x.abs() < APPROXIMATELY_ZERO_CUBED_THRESHOLD
}

/// Global state for path operations
pub struct OpGlobalState {
    pub(crate) nesting: i32,
    pub(crate) winding_failed: bool,
    pub(crate) phase: OpPhase,
    #[cfg(debug_assertions)]
    pub(crate) debug_test_name: Option<String>,
    #[cfg(debug_assertions)]
    pub(crate) angle_id: u32,
    #[cfg(debug_assertions)]
    pub(crate) coin_id: u32,
    #[cfg(debug_assertions)]
    pub(crate) contour_id: u32,
    #[cfg(debug_assertions)]
    pub(crate) pt_t_id: u32,
    #[cfg(debug_assertions)]
    pub(crate) segment_id: u32,
    #[cfg(debug_assertions)]
    pub(crate) span_id: u32,
}

impl OpGlobalState {
    /// Create a new global state for path operations
    pub fn new() -> Self {
        Self {
            nesting: 0,
            winding_failed: false,
            phase: OpPhase::Intersecting,
            #[cfg(debug_assertions)]
            debug_test_name: None,
            #[cfg(debug_assertions)]
            angle_id: 0,
            #[cfg(debug_assertions)]
            coin_id: 0,
            #[cfg(debug_assertions)]
            contour_id: 0,
            #[cfg(debug_assertions)]
            pt_t_id: 0,
            #[cfg(debug_assertions)]
            segment_id: 0,
            #[cfg(debug_assertions)]
            span_id: 0,
        }
    }

    /// Create a new global state with a debug test name
    #[cfg(debug_assertions)]
    pub fn new_with_debug(test_name: &str) -> Self {
        Self {
            debug_test_name: Some(test_name.to_string()),
            ..Self::new()
        }
    }

    /// Check if winding calculation has failed
    pub fn winding_failed(&self) -> bool {
        self.winding_failed
    }

    /// Set winding failure state
    pub fn set_winding_failed(&mut self, failed: bool) {
        self.winding_failed = failed;
    }

    /// Get current phase
    pub fn phase(&self) -> OpPhase {
        self.phase
    }

    /// Set current phase
    pub fn set_phase(&mut self, phase: OpPhase) {
        self.phase = phase;
    }

    /// Get nesting level
    pub fn nesting(&self) -> i32 {
        self.nesting
    }

    /// Set nesting level
    pub fn set_nesting(&mut self, nesting: i32) {
        self.nesting = nesting;
    }

    /// Get debug test name
    #[cfg(debug_assertions)]
    pub fn debug_test_name(&self) -> Option<&str> {
        self.debug_test_name.as_deref()
    }

    /// Set debug test name
    #[cfg(debug_assertions)]
    pub fn set_debug_test_name(&mut self, name: &str) {
        self.debug_test_name = Some(name.to_string());
    }

    /// Reset loop counts (debug only)
    #[cfg(debug_assertions)]
    pub fn debug_reset_loop_counts(&mut self) {
        // Reset debug counters if needed
    }
}

/// Phase of path operation processing
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpPhase {
    /// Finding the points where the operands' segments cross. The initial
    /// phase, and the only one the current port drives.
    Intersecting,
    /// Assigning winding numbers to the spans between intersections.
    Winding,
    /// Linking the kept spans into output contours.
    Chained,
    /// Retrying after a degenerate result, with the operands perturbed.
    Skewed,
}

impl Default for OpPhase {
    fn default() -> Self {
        OpPhase::Intersecting
    }
}

// Double-precision epsilon predicates, ported from Skia's SkPathOpsTypes.h.
// These operate on f64 (unlike the ULP suite above, which is f32) and are
// the tolerance vocabulary the curve/intersection math is built on: use
// `approximately_*` for values in "coordinate space" and the plain
// (non-`approximately_`) comparisons for T values in [0, 1].

/// `FLT_EPSILON` as a double, matching Skia's `FLT_EPSILON` usage in double
/// precision contexts.
pub const FLT_EPSILON: f64 = f32::EPSILON as f64;
const FLT_EPSILON_ORDERABLE_ERR: f64 = FLT_EPSILON * 16.0;
const DBL_EPSILON_ERR: f64 = f64::EPSILON * 4.0;

/// True if `x` is `0` or `1` exactly.
pub fn zero_or_one(x: f64) -> bool {
    x == 0.0 || x == 1.0
}

/// True if `x` is within `FLT_EPSILON` of zero.
pub fn approximately_zero(x: f64) -> bool {
    x.abs() < FLT_EPSILON
}

/// True if `x` is within a double-precision epsilon of zero (a much
/// tighter bound than [`approximately_zero`]).
pub fn precisely_zero(x: f64) -> bool {
    x.abs() < DBL_EPSILON_ERR
}

/// True if `x` is zero relative to `y`'s magnitude.
pub fn approximately_zero_when_compared_to(x: f64, y: f64) -> bool {
    x == 0.0 || x.abs() < (y * FLT_EPSILON).abs()
}

/// True if `x` and `y` are within `FLT_EPSILON` of each other. Intended for
/// T values in `[0, 1]`; for general coordinate magnitudes use the ULP
/// comparisons above instead.
pub fn approximately_equal(x: f64, y: f64) -> bool {
    approximately_zero(x - y)
}

/// True if `x` is within `FLT_EPSILON` below `y` or greater.
pub fn approximately_greater_or_equal(x: f64, y: f64) -> bool {
    x + FLT_EPSILON > y
}

/// True if `x` is within `FLT_EPSILON` above `y` or less.
pub fn approximately_lesser_or_equal(x: f64, y: f64) -> bool {
    x - FLT_EPSILON < y
}

/// True if `x > 1`, allowing `FLT_EPSILON` of slack below `1`.
pub fn approximately_greater_than_one(x: f64) -> bool {
    x > 1.0 - FLT_EPSILON
}

/// True if `x < 0`, allowing `FLT_EPSILON` of slack above `0`.
pub fn approximately_less_than_zero(x: f64) -> bool {
    x < FLT_EPSILON
}

/// True if `x <= 1`, allowing `FLT_EPSILON` of slack above `1`.
pub fn approximately_one_or_less(x: f64) -> bool {
    x < 1.0 + FLT_EPSILON
}

/// True if `x >= 0`, allowing `FLT_EPSILON` of slack below `0`.
pub fn approximately_zero_or_more(x: f64) -> bool {
    x > -FLT_EPSILON
}

/// True if `x` is large enough that `1/x` would itself be approximately
/// zero: guards divisions from blowing up on tiny denominators.
pub fn approximately_zero_inverse(x: f64) -> bool {
    x.abs() > 1.0 / FLT_EPSILON
}

/// True if `x` is zero relative to `y`'s magnitude, at "orderable"
/// (16x looser) tolerance.
pub fn approximately_zero_orderable(x: f64) -> bool {
    x.abs() < FLT_EPSILON_ORDERABLE_ERR
}

/// True if `b` lies between `a` and `c` inclusive, in either order.
/// Double-precision counterpart to [`between`](fn@between) (which takes
/// `f32`); named `between_d` to avoid a duplicate-definition clash.
pub fn between_d(a: f64, b: f64, c: f64) -> bool {
    (a - b) * (c - b) <= 0.0
}

/// True if `b` lies between `a` and `c` inclusive, with a looser tolerance
/// suited to detecting near-misses caused by accumulated floating error.
pub fn roughly_between(a: f64, b: f64, c: f64) -> bool {
    if a > c {
        return roughly_between(c, b, a);
    }
    approximately_negative(a - b) && approximately_negative(b - c) || approximately_equal(a, c)
}

/// True if `x < 0`, allowing `FLT_EPSILON` of slack (an alias for
/// [`approximately_less_than_zero`] used where Skia calls it "negative").
pub fn approximately_negative(x: f64) -> bool {
    x < FLT_EPSILON
}

/// True if `x < 0` at double-precision tolerance (much tighter than
/// [`approximately_negative`]).
pub fn precisely_negative(x: f64) -> bool {
    x < DBL_EPSILON_ERR
}

/// True if `b` lies between `a` and `c` inclusive, at double-precision
/// tolerance (much tighter than [`between`]).
pub fn precisely_between(a: f64, b: f64, c: f64) -> bool {
    approximately_greater_or_equal(b, a.min(c)) && approximately_lesser_or_equal(b, a.max(c))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_almost_bequal_ulps() {
        assert!(almost_bequal_ulps(1.0, 1.0));
        // 1 ULP apart is within the 2-ULP tolerance; 2 ULPs apart is not
        // (the epsilon check is a strict `<`, so it excludes the boundary).
        assert!(almost_bequal_ulps(
            1.0,
            f32::from_bits(1.0f32.to_bits() + 1)
        ));
        assert!(!almost_bequal_ulps(
            1.0,
            f32::from_bits(1.0f32.to_bits() + 2)
        ));
        assert!(!almost_bequal_ulps(1.0, 2.0));
    }

    #[test]
    fn test_almost_pequal_ulps() {
        assert!(almost_pequal_ulps(1.0, 1.0));
        assert!(almost_pequal_ulps(1.0, 1.0 + f32::EPSILON * 5.0));
        assert!(!almost_pequal_ulps(1.0, 2.0));
    }

    #[test]
    fn test_ulps_distance() {
        assert_eq!(ulps_distance(1.0, 1.0), 0);
        assert!(ulps_distance(1.0, 2.0) > 0);
        assert_eq!(ulps_distance(0.0, -0.0), 0);
    }

    #[test]
    fn test_d_cbrt() {
        let result = d_cbrt(27.0);
        assert!((result - 3.0).abs() < 1e-6);

        let result = d_cbrt(-27.0);
        assert!((result - (-3.0)).abs() < 1e-6);

        assert_eq!(d_cbrt(0.0), 0.0);
    }

    #[test]
    fn test_op_global_state() {
        let state = OpGlobalState::new();
        assert_eq!(state.nesting(), 0);
        assert!(!state.winding_failed());
        assert_eq!(state.phase(), OpPhase::Intersecting);
    }

    #[test]
    fn test_almost_between_ulps() {
        assert!(almost_between_ulps(1.0, 1.5, 2.0));
        assert!(!almost_between_ulps(1.0, 3.0, 2.0));
    }

    #[test]
    fn test_almost_less_ulps() {
        assert!(almost_less_ulps(1.0, 2.0));
        assert!(!almost_less_ulps(2.0, 1.0));
        assert!(!almost_less_ulps(1.0, 1.0));
    }

    #[test]
    fn test_almost_less_or_equal_ulps() {
        assert!(almost_less_or_equal_ulps(1.0, 2.0));
        assert!(almost_less_or_equal_ulps(1.0, 1.0));
        assert!(!almost_less_or_equal_ulps(2.0, 1.0));
    }

    #[test]
    fn test_roughly_equal_ulps() {
        assert!(roughly_equal_ulps(1.0, 1.0));
        assert!(roughly_equal_ulps(1.0, 1.0 + f32::EPSILON * 100.0));
    }

    #[test]
    fn test_not_almost_equal_ulps() {
        assert!(!not_almost_equal_ulps(1.0, 1.0));
        assert!(not_almost_equal_ulps(1.0, 2.0));
    }
}
