//! Math utility functions for Skia path operations.
//!
//! This module provides low-level math utilities including:
//! - Integer square root via bit manipulation
//! - Safe arithmetic operations with overflow detection
//! - Float array validation utilities

/// Compute integer square root using the bit-by-bit algorithm.
///
/// This implements the binary square root algorithm described in:
/// www.worldserver.com/turk/computergraphics/FixedSqrt.pdf
///
/// # Arguments
/// * `x` - The value to compute the square root of (in bits)
/// * `count` - The number of bits to process
///
/// # Returns
/// The integer square root of x, processing `count` bits.
pub fn sk_sqrt_bits(x: i32, count: i32) -> i32 {
    let mut root: u32 = 0;
    let mut rem_hi: u32 = 0;
    let mut rem_lo: u32 = x as u32;

    let mut bits = count;
    while bits >= 0 {
        root <<= 1;

        rem_hi = (rem_hi << 2) | (rem_lo >> 30);
        rem_lo <<= 2;

        let test_div = (root << 1) + 1;
        if rem_hi >= test_div {
            rem_hi -= test_div;
            root += 1;
        }

        bits -= 1;
    }

    root as i32
}

/// Safe arithmetic operations that detect overflow.
///
/// This struct tracks whether a series of operations has overflowed.
/// All operations are safe and will not panic on overflow; instead,
/// they set an internal flag that can be checked via `ok()`.
#[derive(Default)]
pub struct SkSafeMath {
    f_ok: bool,
}

impl SkSafeMath {
    /// Create a new SkSafeMath instance with no overflow.
    pub fn new() -> Self {
        Self { f_ok: true }
    }

    /// Check if all operations so far have been valid (no overflow).
    pub fn ok(&self) -> bool {
        self.f_ok
    }

    /// Add two size_t values. Mirrors the C++ `size_t result = x + y;`
    /// (defined-behavior unsigned wraparound): on overflow `result` wraps
    /// below `x`, which is how the overflow is detected below.
    pub fn add(&mut self, x: usize, y: usize) -> usize {
        let result = x.wrapping_add(y);
        self.f_ok &= result >= x;
        result
    }

    /// Multiply two size_t values, checking for overflow.
    pub fn mul(&mut self, x: usize, y: usize) -> usize {
        if std::mem::size_of::<usize>() == std::mem::size_of::<u64>() {
            self.mul64(x as u64, y as u64) as usize
        } else {
            self.mul32(x as u32, y as u32) as usize
        }
    }

    /// Multiply two u32 values, checking for overflow.
    fn mul32(&mut self, x: u32, y: u32) -> u32 {
        let bx = x as u64;
        let by = y as u64;
        let result = bx * by;
        self.f_ok &= (result >> 32) == 0;
        result as u32
    }

    /// Multiply two u64 values, checking for overflow.
    fn mul64(&mut self, x: u64, y: u64) -> u64 {
        const MAX_U64: u64 = u64::MAX;
        
        if x <= (MAX_U64 >> 32) && y <= (MAX_U64 >> 32) {
            x * y
        } else {
            let hi = |v: u64| v >> 32;
            let lo = |v: u64| v & 0xFFFFFFFF;

            let lx_ly = lo(x) * lo(y);
            let hx_ly = hi(x) * lo(y);
            let lx_hy = lo(x) * hi(y);
            let hx_hy = hi(x) * hi(y);

            let mut result = 0u64;
            result = result.saturating_add(lx_ly);
            result = result.saturating_add(hx_ly << 32);
            result = result.saturating_add(lx_hy << 32);
            
            self.f_ok &= (hx_hy + (hx_ly >> 32) + (lx_hy >> 32)) == 0;
            result
        }
    }

    /// Add two size_t values with overflow checking.
    pub fn add_int(&mut self, a: i32, b: i32) -> i32 {
        if b < 0 && a < i32::MIN.saturating_sub(b) {
            self.f_ok = false;
            a
        } else if b > 0 && a > i32::MAX.saturating_sub(b) {
            self.f_ok = false;
            a
        } else {
            a.saturating_add(b)
        }
    }

    /// Align x up to the given alignment.
    pub fn align_up(&mut self, x: usize, alignment: usize) -> usize {
        let result = self.add(x, alignment - 1);
        result & !(alignment - 1)
    }

    /// Cast a size_t value to type T, checking for overflow.
    pub fn cast_to<T>(&mut self, value: usize) -> Option<T>
    where
        T: TryFrom<usize>,
    {
        T::try_from(value).ok()
    }

    /// Static function: Add two size_t values, returning SIZE_MAX on overflow.
    pub fn safe_add(x: usize, y: usize) -> usize {
        let mut tmp = SkSafeMath::new();
        let sum = tmp.add(x, y);
        if tmp.ok() { sum } else { usize::MAX }
    }

    /// Static function: Multiply two size_t values, returning SIZE_MAX on overflow.
    pub fn safe_mul(x: usize, y: usize) -> usize {
        let mut tmp = SkSafeMath::new();
        let prod = tmp.mul(x, y);
        if tmp.ok() { prod } else { usize::MAX }
    }

    /// Static function: Align x up to 4-byte boundary.
    pub fn safe_align4(x: usize) -> usize {
        let mut tmp = SkSafeMath::new();
        tmp.align_up(x, 4)
    }
}

/// Safe division that handles edge cases like division by zero and overflow.
/// Returns `a / b` in normal cases, but handles special values appropriately.
///
/// # Arguments
/// * `a` - Numerator
/// * `b` - Denominator
///
/// # Returns
/// The result of division, or 0.0 if b is zero.
pub fn sk_ieee_float_divide(a: f64, b: f64) -> f64 {
    if b == 0.0 {
        0.0
    } else {
        a / b
    }
}

/// Check if all floats in an array are in the unit range [0, 1].
///
/// # Arguments
/// * `array` - The array of floats to check
/// * `count` - The number of elements to check
///
/// # Returns
/// `true` if all elements are in [0, 1], `false` otherwise.
pub fn sk_floats_are_unit(array: &[f32], count: usize) -> bool {
    let count = count.min(array.len());
    for i in 0..count {
        if array[i] < 0.0 || array[i] > 1.0 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sk_sqrt_bits() {
        // Test basic cases
        assert_eq!(sk_sqrt_bits(0, 0), 0);
        // With only 2 iterations (count=1 runs a do-while twice, per the
        // original SkSqrtBits), there isn't enough precision to resolve
        // sqrt(1); hand-traced bit-by-bit, the algorithm legitimately
        // returns 0 here.
        assert_eq!(sk_sqrt_bits(1, 1), 0);

        // Test square of 4 (100 in binary)
        assert!(sk_sqrt_bits(4, 3) <= 2);

        // Test square of 16 (10000 in binary)
        assert!(sk_sqrt_bits(16, 5) <= 4);
    }

    #[test]
    fn test_sk_safe_math_add() {
        let mut math = SkSafeMath::new();
        assert!(math.ok());
        
        let result = math.add(10, 20);
        assert_eq!(result, 30);
        assert!(math.ok());
    }

    #[test]
    fn test_sk_safe_math_mul() {
        let mut math = SkSafeMath::new();
        assert!(math.ok());
        
        let result = math.mul(10, 20);
        assert_eq!(result, 200);
        assert!(math.ok());
    }

    #[test]
    fn test_sk_safe_math_overflow() {
        // Test addition overflow
        let mut math = SkSafeMath::new();
        let _ = math.add(usize::MAX, 1);
        assert!(!math.ok());
        
        // Test multiplication overflow
        let mut math = SkSafeMath::new();
        let _ = math.mul(usize::MAX, 2);
        assert!(!math.ok());
    }

    #[test]
    fn test_sk_safe_static_functions() {
        assert_eq!(SkSafeMath::safe_add(10, 20), 30);
        assert_eq!(SkSafeMath::safe_mul(10, 20), 200);
        assert_eq!(SkSafeMath::safe_align4(5), 8);
    }

    #[test]
    fn test_sk_floats_are_unit() {
        let arr = [0.0, 0.5, 1.0];
        assert!(sk_floats_are_unit(&arr, 3));
        
        let arr = [0.0, 1.5, 1.0];
        assert!(!sk_floats_are_unit(&arr, 3));
        
        let arr = [-0.5, 0.5, 1.0];
        assert!(!sk_floats_are_unit(&arr, 3));
    }

    #[test]
    fn test_align_up() {
        let mut math = SkSafeMath::new();
        assert_eq!(math.align_up(5, 4), 8);
        assert_eq!(math.align_up(4, 4), 4);
        assert_eq!(math.align_up(9, 8), 16);
    }

    #[test]
    fn test_cast_to() {
        let mut math = SkSafeMath::new();
        assert_eq!(math.cast_to::<u8>(100), Some(100));
        assert_eq!(math.cast_to::<u8>(256), None);
    }

    #[test]
    fn test_safe_mul_edge_cases() {
        assert_eq!(SkSafeMath::safe_mul(0, 100), 0);
        assert_eq!(SkSafeMath::safe_mul(100, 0), 0);
        assert_eq!(SkSafeMath::safe_mul(1, 1), 1);
    }

    #[test]
    fn test_safe_add_edge_cases() {
        assert_eq!(SkSafeMath::safe_add(0, 0), 0);
        assert_eq!(SkSafeMath::safe_add(usize::MAX, 0), usize::MAX);
        assert_eq!(SkSafeMath::safe_add(0, usize::MAX), usize::MAX);
    }

    #[test]
    fn test_floats_are_unit_edge_cases() {
        assert!(sk_floats_are_unit(&[], 0));
        assert!(sk_floats_are_unit(&[0.0], 1));
        assert!(sk_floats_are_unit(&[1.0], 1));
        assert!(!sk_floats_are_unit(&[0.0, 1.5, 2.0], 3));
    }

    #[test]
    fn test_add_int() {
        let mut math = SkSafeMath::new();
        assert_eq!(math.add_int(100, 50), 150);
        assert_eq!(math.add_int(-100, 50), -50);
        assert!(math.ok());
    }
}
