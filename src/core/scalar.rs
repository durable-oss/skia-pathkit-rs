//! Scalar (`f32`) constants and helpers.
//!
//! Ported from `include/core/SkScalar.h` and the scalar-relevant parts of
//! `include/private/SkFloatingPoint.h`. Skia represents path coordinates as
//! `float`; this crate keeps that representation as a plain [`Scalar`] alias
//! rather than a newtype, matching the original API's ergonomics.

/// A path coordinate or measurement. Always `f32`, matching Skia's
/// `SkScalar`.
pub type Scalar = f32;

/// `1.0` as a [`Scalar`].
pub const SCALAR_1: Scalar = 1.0;
/// `0.5` as a [`Scalar`].
pub const SCALAR_HALF: Scalar = 0.5;
/// `sqrt(2)`.
pub const SCALAR_SQRT_2: Scalar = std::f32::consts::SQRT_2;
/// Pi.
pub const SCALAR_PI: Scalar = std::f32::consts::PI;
/// `sqrt(2) / 2`, i.e. `cos(45°)`.
pub const SCALAR_ROOT_2_OVER_2: Scalar = 0.707_106_77;
/// Largest finite [`Scalar`] value Skia treats as "in range".
pub const SCALAR_MAX: Scalar = 3.402_823_5e+38;
/// Smallest finite [`Scalar`] value Skia treats as "in range".
pub const SCALAR_MIN: Scalar = -SCALAR_MAX;
/// Positive infinity.
pub const SCALAR_INFINITY: Scalar = f32::INFINITY;
/// Negative infinity.
pub const SCALAR_NEGATIVE_INFINITY: Scalar = f32::NEG_INFINITY;
/// A quiet NaN.
pub const SCALAR_NAN: Scalar = f32::NAN;
/// Threshold below which a scalar is treated as zero by
/// [`nearly_zero`]. Equal to `1 / 4096`.
pub const NEARLY_ZERO: Scalar = SCALAR_1 / 4096.0;

/// Returns `true` if `x` is NaN.
///
/// # Examples
///
/// ```
/// use pathkit::core::scalar::is_nan;
/// assert!(is_nan(f32::NAN));
/// assert!(!is_nan(1.0));
/// ```
#[must_use]
pub fn is_nan(x: Scalar) -> bool {
    x.is_nan()
}

/// Returns `true` if `x` is neither NaN nor infinite.
///
/// # Examples
///
/// ```
/// use pathkit::core::scalar::is_finite;
/// assert!(is_finite(1.0));
/// assert!(!is_finite(f32::INFINITY));
/// assert!(!is_finite(f32::NAN));
/// ```
#[must_use]
pub fn is_finite(x: Scalar) -> bool {
    x.is_finite()
}

/// Returns `true` if both `a` and `b` are finite.
#[must_use]
pub fn are_finite(a: Scalar, b: Scalar) -> bool {
    is_finite(a) && is_finite(b)
}

/// Returns `true` if every value in `values` is finite.
///
/// Mirrors Skia's overflow-via-NaN-propagation trick
/// (`sk_floats_are_finite(const float[], int)`), but does so directly with
/// `f32::is_finite` since Rust makes that no less efficient.
#[must_use]
pub fn slice_is_finite(values: &[Scalar]) -> bool {
    values.iter().all(|v| v.is_finite())
}

/// Returns the fractional part of `x`.
///
/// # Examples
///
/// ```
/// use pathkit::core::scalar::fraction;
/// assert!((fraction(3.25) - 0.25).abs() < 1e-6);
/// ```
#[must_use]
pub fn fraction(x: Scalar) -> Scalar {
    x - x.trunc()
}

/// Returns `x * x`.
#[must_use]
pub fn square(x: Scalar) -> Scalar {
    x * x
}

/// Returns the square root of `x`.
#[must_use]
pub fn sqrt(x: Scalar) -> Scalar {
    x.sqrt()
}

/// Returns the cube root of `x`.
#[must_use]
pub fn cbrt(x: Scalar) -> Scalar {
    x.cbrt()
}

/// Returns `true` if `x` has no fractional part.
#[must_use]
pub fn is_int(x: Scalar) -> bool {
    x == x.floor()
}

/// Returns -1, 0, or 1 according to the sign of `x`.
///
/// # Examples
///
/// ```
/// use pathkit::core::scalar::sign_as_int;
/// assert_eq!(sign_as_int(-5.0), -1);
/// assert_eq!(sign_as_int(0.0), 0);
/// assert_eq!(sign_as_int(5.0), 1);
/// ```
#[must_use]
pub fn sign_as_int(x: Scalar) -> i32 {
    if x < 0.0 {
        -1
    } else {
        i32::from(x > 0.0)
    }
}

/// Returns -1.0, 0.0, or 1.0 according to the sign of `x`.
#[must_use]
pub fn sign_as_scalar(x: Scalar) -> Scalar {
    if x < 0.0 {
        -SCALAR_1
    } else if x > 0.0 {
        SCALAR_1
    } else {
        0.0
    }
}

/// Returns `true` if `x` is within `tolerance` of zero.
///
/// # Examples
///
/// ```
/// use pathkit::core::scalar::nearly_zero;
/// assert!(nearly_zero(0.0001, None));
/// assert!(!nearly_zero(1.0, None));
/// ```
#[must_use]
pub fn nearly_zero(x: Scalar, tolerance: Option<Scalar>) -> bool {
    x.abs() <= tolerance.unwrap_or(NEARLY_ZERO)
}

/// Returns `true` if `x` and `y` are within `tolerance` of each other.
#[must_use]
pub fn nearly_equal(x: Scalar, y: Scalar, tolerance: Option<Scalar>) -> bool {
    (x - y).abs() <= tolerance.unwrap_or(NEARLY_ZERO)
}

/// `sin(radians)`, snapped to exactly `0.0` when nearly zero.
#[must_use]
pub fn sin_snap_to_zero(radians: Scalar) -> Scalar {
    let v = radians.sin();
    if nearly_zero(v, None) {
        0.0
    } else {
        v
    }
}

/// `cos(radians)`, snapped to exactly `0.0` when nearly zero.
#[must_use]
pub fn cos_snap_to_zero(radians: Scalar) -> Scalar {
    let v = radians.cos();
    if nearly_zero(v, None) {
        0.0
    } else {
        v
    }
}

/// Linearly interpolates between `a` and `b` by `t`.
///
/// `t == 0.0` returns `a`; `t == 1.0` returns `b`.
///
/// # Examples
///
/// ```
/// use pathkit::core::scalar::interp;
/// assert_eq!(interp(0.0, 10.0, 0.5), 5.0);
/// ```
#[must_use]
pub fn interp(a: Scalar, b: Scalar, t: Scalar) -> Scalar {
    a + (b - a) * t
}

/// Interpolates along the piecewise-linear function described by
/// `(keys[i], values[i])`. `search_key` values outside `keys`' range clamp
/// to the nearest endpoint. `keys` must be sorted ascending (repeated keys
/// are allowed; the first match wins).
///
/// # Panics
///
/// Panics if `keys.len() != values.len()` or if `keys` is empty.
#[must_use]
pub fn interp_func(search_key: Scalar, keys: &[Scalar], values: &[Scalar]) -> Scalar {
    assert_eq!(
        keys.len(),
        values.len(),
        "keys and values must be the same length"
    );
    assert!(!keys.is_empty(), "keys must not be empty");

    let mut i = 0usize;
    while i < keys.len() && search_key > keys[i] {
        i += 1;
    }
    if i == keys.len() {
        return values[keys.len() - 1];
    }
    if i == 0 || keys[i] == search_key {
        return values[i];
    }
    let t = (search_key - keys[i - 1]) / (keys[i] - keys[i - 1]);
    interp(values[i - 1], values[i], t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nan_and_finite() {
        assert!(is_nan(SCALAR_NAN));
        assert!(!is_nan(1.0));
        assert!(is_finite(1.0));
        assert!(!is_finite(SCALAR_INFINITY));
        assert!(!is_finite(SCALAR_NAN));
    }

    #[test]
    fn slice_finiteness() {
        assert!(slice_is_finite(&[1.0, 2.0, 3.0]));
        assert!(!slice_is_finite(&[1.0, SCALAR_NAN, 3.0]));
        assert!(!slice_is_finite(&[1.0, SCALAR_INFINITY]));
    }

    #[test]
    fn fraction_and_square() {
        assert!((fraction(3.75) - 0.75).abs() < 1e-6);
        assert_eq!(square(3.0), 9.0);
    }

    #[test]
    fn sign_helpers() {
        assert_eq!(sign_as_int(-2.0), -1);
        assert_eq!(sign_as_int(0.0), 0);
        assert_eq!(sign_as_int(2.0), 1);
        assert_eq!(sign_as_scalar(-2.0), -1.0);
        assert_eq!(sign_as_scalar(0.0), 0.0);
        assert_eq!(sign_as_scalar(2.0), 1.0);
    }

    #[test]
    fn nearly_helpers() {
        assert!(nearly_zero(0.0, None));
        assert!(nearly_zero(NEARLY_ZERO / 2.0, None));
        assert!(!nearly_zero(1.0, None));
        assert!(nearly_equal(1.0, 1.0 + NEARLY_ZERO / 2.0, None));
    }

    #[test]
    fn interp_basic() {
        assert_eq!(interp(0.0, 10.0, 0.0), 0.0);
        assert_eq!(interp(0.0, 10.0, 1.0), 10.0);
        assert_eq!(interp(0.0, 10.0, 0.5), 5.0);
    }

    #[test]
    fn interp_func_clamps_and_interpolates() {
        let keys = [0.0, 1.0, 2.0];
        let values = [0.0, 10.0, 20.0];
        assert_eq!(interp_func(-1.0, &keys, &values), 0.0);
        assert_eq!(interp_func(3.0, &keys, &values), 20.0);
        assert_eq!(interp_func(0.5, &keys, &values), 5.0);
        assert_eq!(interp_func(1.0, &keys, &values), 10.0);
    }
}
