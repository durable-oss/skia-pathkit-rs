//! Path fill rules, contour winding direction, and path verbs.
//!
//! Ported from `include/core/SkPathTypes.h`.

/// Determines the winding rule used to fill a path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum FillType {
    /// "Inside" is computed by a non-zero sum of signed edge crossings.
    #[default]
    Winding,
    /// "Inside" is computed by an odd number of edge crossings.
    EvenOdd,
    /// Same as `Winding`, but fills outside the path rather than inside.
    InverseWinding,
    /// Same as `EvenOdd`, but fills outside the path rather than inside.
    InverseEvenOdd,
}

impl FillType {
    /// Returns `true` for the even-odd fill rules (inverse or not).
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::core::FillType;
    /// assert!(FillType::EvenOdd.is_even_odd());
    /// assert!(!FillType::Winding.is_even_odd());
    /// ```
    #[must_use]
    pub const fn is_even_odd(self) -> bool {
        matches!(self, FillType::EvenOdd | FillType::InverseEvenOdd)
    }

    /// Returns `true` for the inverse fill rules.
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::core::FillType;
    /// assert!(FillType::InverseWinding.is_inverse());
    /// assert!(!FillType::Winding.is_inverse());
    /// ```
    #[must_use]
    pub const fn is_inverse(self) -> bool {
        matches!(self, FillType::InverseWinding | FillType::InverseEvenOdd)
    }

    /// Returns the non-inverse fill rule with the same even-odd parity.
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::core::FillType;
    /// assert_eq!(FillType::InverseWinding.to_non_inverse(), FillType::Winding);
    /// assert_eq!(FillType::InverseEvenOdd.to_non_inverse(), FillType::EvenOdd);
    /// ```
    #[must_use]
    pub const fn to_non_inverse(self) -> FillType {
        if self.is_even_odd() {
            FillType::EvenOdd
        } else {
            FillType::Winding
        }
    }
}

/// Winding direction used when adding a closed contour to a path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Clockwise direction for adding closed contours.
    Cw,
    /// Counter-clockwise direction for adding closed contours.
    Ccw,
}

/// A single verb (segment type) making up a path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Verb {
    /// Starts a new contour; carries 1 point.
    Move,
    /// A straight line; carries 2 points.
    Line,
    /// A quadratic Bezier; carries 3 points.
    Quad,
    /// A conic (rational quadratic); carries 3 points plus a weight.
    Conic,
    /// A cubic Bezier; carries 4 points.
    Cubic,
    /// Closes the current contour; carries 0 points.
    Close,
}

impl Verb {
    /// Number of points this verb's iterator step returns, per Skia's
    /// `RawIter` convention (does not include the conic weight).
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::core::Verb;
    /// assert_eq!(Verb::Move.point_count(), 1);
    /// assert_eq!(Verb::Cubic.point_count(), 4);
    /// assert_eq!(Verb::Close.point_count(), 0);
    /// ```
    #[must_use]
    pub const fn point_count(self) -> usize {
        match self {
            Verb::Move => 1,
            Verb::Line => 2,
            Verb::Quad | Verb::Conic => 3,
            Verb::Cubic => 4,
            Verb::Close => 0,
        }
    }
}

/// Path contains at least one line segment. Part of the segment-mask
/// bitfield corresponding to `SkPathSegmentMask`.
pub const SEGMENT_MASK_LINE: u8 = 1 << 0;
/// Path contains at least one quadratic Bezier segment.
pub const SEGMENT_MASK_QUAD: u8 = 1 << 1;
/// Path contains at least one conic segment.
pub const SEGMENT_MASK_CONIC: u8 = 1 << 2;
/// Path contains at least one cubic Bezier segment.
pub const SEGMENT_MASK_CUBIC: u8 = 1 << 3;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fill_type_parity() {
        assert!(FillType::EvenOdd.is_even_odd());
        assert!(FillType::InverseEvenOdd.is_even_odd());
        assert!(!FillType::Winding.is_even_odd());
        assert!(!FillType::InverseWinding.is_even_odd());
    }

    #[test]
    fn fill_type_inverse() {
        assert!(FillType::InverseWinding.is_inverse());
        assert!(FillType::InverseEvenOdd.is_inverse());
        assert!(!FillType::Winding.is_inverse());
        assert!(!FillType::EvenOdd.is_inverse());
    }

    #[test]
    fn verb_point_counts() {
        assert_eq!(Verb::Move.point_count(), 1);
        assert_eq!(Verb::Line.point_count(), 2);
        assert_eq!(Verb::Quad.point_count(), 3);
        assert_eq!(Verb::Conic.point_count(), 3);
        assert_eq!(Verb::Cubic.point_count(), 4);
        assert_eq!(Verb::Close.point_count(), 0);
    }
}
