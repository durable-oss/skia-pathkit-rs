//! Axis-aligned rectangles.
//!
//! Ported from `old/pathkit/include/core/SkRect.h`.

use super::point::Point;
use super::scalar::{is_nan, Scalar};

/// An axis-aligned rectangle described by its four edge coordinates.
///
/// A rectangle is considered empty if `right <= left` or `bottom <= top`.
/// Unlike Skia's `SkRect`, edges are not required to be sorted by
/// construction; call [`Rect::sort`] or [`Rect::sorted`] to normalize.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Rect {
    /// Smaller x-axis bound.
    pub left: Scalar,
    /// Smaller y-axis bound.
    pub top: Scalar,
    /// Larger x-axis bound.
    pub right: Scalar,
    /// Larger y-axis bound.
    pub bottom: Scalar,
}

impl Rect {
    /// Returns the rectangle `(0, 0, 0, 0)`.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            left: 0.0,
            top: 0.0,
            right: 0.0,
            bottom: 0.0,
        }
    }

    /// Returns the rectangle `(0, 0, w, h)`. Does not validate that `w`/`h`
    /// are non-negative.
    #[must_use]
    pub const fn from_wh(w: Scalar, h: Scalar) -> Self {
        Self {
            left: 0.0,
            top: 0.0,
            right: w,
            bottom: h,
        }
    }

    /// Returns the rectangle `(l, t, r, b)`. Does not sort the inputs.
    #[must_use]
    pub const fn from_ltrb(l: Scalar, t: Scalar, r: Scalar, b: Scalar) -> Self {
        Self {
            left: l,
            top: t,
            right: r,
            bottom: b,
        }
    }

    /// Returns the rectangle `(x, y, x + w, y + h)`. Does not validate that
    /// `w`/`h` are non-negative.
    ///
    /// # Examples
    ///
    /// ```
    /// use pathkit::core::Rect;
    /// let r = Rect::from_xywh(1.0, 2.0, 3.0, 4.0);
    /// assert_eq!(r, Rect::from_ltrb(1.0, 2.0, 4.0, 6.0));
    /// ```
    #[must_use]
    pub const fn from_xywh(x: Scalar, y: Scalar, w: Scalar, h: Scalar) -> Self {
        Self {
            left: x,
            top: y,
            right: x + w,
            bottom: y + h,
        }
    }

    /// Returns the smallest rectangle enclosing all of `points`. Returns
    /// [`Rect::empty`] if `points` is empty or contains a non-finite value.
    #[must_use]
    pub fn from_points(points: &[Point]) -> Self {
        let mut rect = Self::empty();
        rect.set_bounds_check(points);
        rect
    }

    /// Returns `true` if the rectangle has zero or negative width or
    /// height (including when any edge is `NaN`).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        !(self.left < self.right && self.top < self.bottom)
    }

    /// Returns `true` if `left <= right` and `top <= bottom`.
    #[must_use]
    pub fn is_sorted(&self) -> bool {
        self.left <= self.right && self.top <= self.bottom
    }

    /// Returns `true` if no edge is infinite or `NaN`.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        !is_nan(self.left * self.top * self.right * self.bottom)
    }

    /// Returns the left edge (equivalent to [`Rect::left`] field access).
    #[must_use]
    pub fn x(&self) -> Scalar {
        self.left
    }

    /// Returns the top edge (equivalent to [`Rect::top`] field access).
    #[must_use]
    pub fn y(&self) -> Scalar {
        self.top
    }

    /// Returns `right - left`. Not clamped to zero for unsorted rectangles.
    #[must_use]
    pub fn width(&self) -> Scalar {
        self.right - self.left
    }

    /// Returns `bottom - top`. Not clamped to zero for unsorted rectangles.
    #[must_use]
    pub fn height(&self) -> Scalar {
        self.bottom - self.top
    }

    /// Returns the midpoint of the left and right edges.
    #[must_use]
    pub fn center_x(&self) -> Scalar {
        self.left * 0.5 + self.right * 0.5
    }

    /// Returns the midpoint of the top and bottom edges.
    #[must_use]
    pub fn center_y(&self) -> Scalar {
        self.top * 0.5 + self.bottom * 0.5
    }

    /// Returns the four corners of the rectangle in order: top-left,
    /// top-right, bottom-right, bottom-left.
    #[must_use]
    pub fn to_quad(&self) -> [Point; 4] {
        [
            Point::new(self.left, self.top),
            Point::new(self.right, self.top),
            Point::new(self.right, self.bottom),
            Point::new(self.left, self.bottom),
        ]
    }

    /// Sets the rectangle to the bounds of `points`, without requiring that
    /// the points be finite. If any point is non-finite, all bounds are set
    /// to `NaN`.
    pub fn set_bounds_no_check(&mut self, points: &[Point]) {
        if points.is_empty() {
            self.set_empty();
            return;
        }
        let (mut min_x, mut max_x) = (points[0].x, points[0].x);
        let (mut min_y, mut max_y) = (points[0].y, points[0].y);
        for p in points {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
        if points.iter().any(|p| !p.is_finite()) {
            self.set_ltrb(f32::NAN, f32::NAN, f32::NAN, f32::NAN);
        } else {
            self.set_ltrb(min_x, min_y, max_x, max_y);
        }
    }

    /// Sets the rectangle to `(0, 0, 0, 0)`.
    pub fn set_empty(&mut self) {
        *self = Self::empty();
    }

    /// Sets all four edges directly, without sorting.
    pub fn set_ltrb(&mut self, left: Scalar, top: Scalar, right: Scalar, bottom: Scalar) {
        self.left = left;
        self.top = top;
        self.right = right;
        self.bottom = bottom;
    }

    /// Sets the rectangle to the bounds of `points`. Leaves the rectangle
    /// at `(0, 0, 0, 0)` if `points` is empty or contains a non-finite
    /// value; returns whether all points were finite.
    pub fn set_bounds_check(&mut self, points: &[Point]) -> bool {
        if points.is_empty() {
            self.set_empty();
            return false;
        }
        let (mut min_x, mut max_x) = (points[0].x, points[0].x);
        let (mut min_y, mut max_y) = (points[0].y, points[0].y);
        let mut accum = 0.0f32;
        for p in points {
            accum *= p.x;
            accum *= p.y;
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
        if is_nan(accum) {
            self.set_empty();
            return false;
        }
        self.set_ltrb(min_x, min_y, max_x, max_y);
        true
    }

    /// Sets the rectangle to `(x, y, x + width, y + height)`.
    pub fn set_xywh(&mut self, x: Scalar, y: Scalar, width: Scalar, height: Scalar) {
        self.left = x;
        self.top = y;
        self.right = x + width;
        self.bottom = y + height;
    }

    /// Returns a copy offset by `(dx, dy)`.
    #[must_use]
    pub fn make_offset(&self, dx: Scalar, dy: Scalar) -> Self {
        Self::from_ltrb(
            self.left + dx,
            self.top + dy,
            self.right + dx,
            self.bottom + dy,
        )
    }

    /// Returns a copy inset by `(dx, dy)`: `dx`/`dy` positive shrinks the
    /// rectangle, negative grows it.
    #[must_use]
    pub fn make_inset(&self, dx: Scalar, dy: Scalar) -> Self {
        Self::from_ltrb(
            self.left + dx,
            self.top + dy,
            self.right - dx,
            self.bottom - dy,
        )
    }

    /// Returns a copy outset by `(dx, dy)`: `dx`/`dy` positive grows the
    /// rectangle, negative shrinks it.
    #[must_use]
    pub fn make_outset(&self, dx: Scalar, dy: Scalar) -> Self {
        self.make_inset(-dx, -dy)
    }

    /// Offsets the rectangle in place by `(dx, dy)`.
    pub fn offset(&mut self, dx: Scalar, dy: Scalar) {
        self.left += dx;
        self.top += dy;
        self.right += dx;
        self.bottom += dy;
    }

    /// Moves the rectangle so its top-left corner is `(new_x, new_y)`,
    /// preserving width and height.
    pub fn offset_to(&mut self, new_x: Scalar, new_y: Scalar) {
        self.right += new_x - self.left;
        self.bottom += new_y - self.top;
        self.left = new_x;
        self.top = new_y;
    }

    /// Insets the rectangle in place by `(dx, dy)`.
    pub fn inset(&mut self, dx: Scalar, dy: Scalar) {
        self.left += dx;
        self.top += dy;
        self.right -= dx;
        self.bottom -= dy;
    }

    /// Outsets the rectangle in place by `(dx, dy)`.
    pub fn outset(&mut self, dx: Scalar, dy: Scalar) {
        self.inset(-dx, -dy);
    }

    /// Intersects this rectangle with `other` in place. Returns `false`
    /// (leaving `self` unchanged) if either rectangle is empty or they do
    /// not overlap.
    pub fn intersect(&mut self, other: &Self) -> bool {
        if let Some(result) = Self::intersection(self, other) {
            *self = result;
            true
        } else {
            false
        }
    }

    /// Returns the intersection of `a` and `b`, or `None` if either is
    /// empty or they do not overlap.
    #[must_use]
    pub fn intersection(a: &Self, b: &Self) -> Option<Self> {
        let l = a.left.max(b.left);
        let t = a.top.max(b.top);
        let r = a.right.min(b.right);
        let bo = a.bottom.min(b.bottom);
        if l < r && t < bo {
            Some(Self::from_ltrb(l, t, r, bo))
        } else {
            None
        }
    }

    /// Returns `true` if this rectangle and `other` overlap. Returns
    /// `false` if either is empty.
    #[must_use]
    pub fn intersects(&self, other: &Self) -> bool {
        Self::rects_intersect(self, other)
    }

    /// Returns `true` if `a` and `b` overlap. Returns `false` if either is
    /// empty.
    #[must_use]
    pub fn rects_intersect(a: &Self, b: &Self) -> bool {
        let l = a.left.max(b.left);
        let t = a.top.max(b.top);
        let r = a.right.min(b.right);
        let bo = a.bottom.min(b.bottom);
        l < r && t < bo
    }

    /// Expands this rectangle in place to the union of itself and `other`.
    /// Leaves `self` unchanged if `other` is empty; if `self` is empty,
    /// sets `self` to `other`.
    pub fn join(&mut self, other: &Self) {
        if other.is_empty() {
            return;
        }
        if self.is_empty() {
            *self = *other;
        } else {
            self.join_possibly_empty(other);
        }
    }

    /// Expands this rectangle in place to the union of itself and `other`,
    /// without checking either for emptiness first.
    pub fn join_possibly_empty(&mut self, other: &Self) {
        self.left = self.left.min(other.left);
        self.top = self.top.min(other.top);
        self.right = self.right.max(other.right);
        self.bottom = self.bottom.max(other.bottom);
    }

    /// Returns `true` if `(x, y)` lies within the rectangle: `left <= x <
    /// right && top <= y < bottom`.
    #[must_use]
    pub fn contains_point(&self, x: Scalar, y: Scalar) -> bool {
        x >= self.left && x < self.right && y >= self.top && y < self.bottom
    }

    /// Returns `true` if this rectangle fully contains `other`. Returns
    /// `false` if either is empty.
    #[must_use]
    pub fn contains(&self, other: &Self) -> bool {
        !other.is_empty()
            && !self.is_empty()
            && self.left <= other.left
            && self.top <= other.top
            && self.right >= other.right
            && self.bottom >= other.bottom
    }

    /// Returns a copy with `left`/`top` floored and `right`/`bottom`
    /// ceilinged, expanding to the nearest enclosing integer bounds.
    #[must_use]
    pub fn round_out(&self) -> Self {
        Self::from_ltrb(
            self.left.floor(),
            self.top.floor(),
            self.right.ceil(),
            self.bottom.ceil(),
        )
    }

    /// Swaps edges in place so that `left <= right` and `top <= bottom`.
    pub fn sort(&mut self) {
        if self.left > self.right {
            std::mem::swap(&mut self.left, &mut self.right);
        }
        if self.top > self.bottom {
            std::mem::swap(&mut self.top, &mut self.bottom);
        }
    }

    /// Returns a sorted copy of this rectangle.
    #[must_use]
    pub fn sorted(&self) -> Self {
        Self::from_ltrb(
            self.left.min(self.right),
            self.top.min(self.bottom),
            self.left.max(self.right),
            self.top.max(self.bottom),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_xywh_matches_ltrb() {
        assert_eq!(
            Rect::from_xywh(1.0, 2.0, 3.0, 4.0),
            Rect::from_ltrb(1.0, 2.0, 4.0, 6.0)
        );
    }

    #[test]
    fn empty_detection() {
        assert!(Rect::empty().is_empty());
        assert!(!Rect::from_wh(1.0, 1.0).is_empty());
        assert!(Rect::from_ltrb(0.0, 0.0, 0.0, 1.0).is_empty());
    }

    #[test]
    fn intersect_overlapping() {
        let mut a = Rect::from_ltrb(0.0, 0.0, 10.0, 10.0);
        let b = Rect::from_ltrb(5.0, 5.0, 15.0, 15.0);
        assert!(a.intersect(&b));
        assert_eq!(a, Rect::from_ltrb(5.0, 5.0, 10.0, 10.0));
    }

    #[test]
    fn intersect_disjoint_leaves_unchanged() {
        let mut a = Rect::from_ltrb(0.0, 0.0, 1.0, 1.0);
        let original = a;
        let b = Rect::from_ltrb(5.0, 5.0, 6.0, 6.0);
        assert!(!a.intersect(&b));
        assert_eq!(a, original);
    }

    #[test]
    fn join_expands_bounds() {
        let mut a = Rect::from_ltrb(0.0, 0.0, 1.0, 1.0);
        a.join(&Rect::from_ltrb(2.0, 2.0, 3.0, 3.0));
        assert_eq!(a, Rect::from_ltrb(0.0, 0.0, 3.0, 3.0));
    }

    #[test]
    fn join_with_empty_self_takes_other() {
        let mut a = Rect::empty();
        let b = Rect::from_ltrb(1.0, 1.0, 2.0, 2.0);
        a.join(&b);
        assert_eq!(a, b);
    }

    #[test]
    fn contains_point_excludes_right_and_bottom_edges() {
        let r = Rect::from_ltrb(0.0, 0.0, 10.0, 10.0);
        assert!(r.contains_point(0.0, 0.0));
        assert!(!r.contains_point(10.0, 5.0));
        assert!(!r.contains_point(5.0, 10.0));
    }

    #[test]
    fn sort_normalizes_edges() {
        let mut r = Rect::from_ltrb(10.0, 10.0, 0.0, 0.0);
        r.sort();
        assert_eq!(r, Rect::from_ltrb(0.0, 0.0, 10.0, 10.0));
    }

    #[test]
    fn from_points_bounds() {
        let pts = [
            Point::new(1.0, 5.0),
            Point::new(-2.0, 3.0),
            Point::new(4.0, -1.0),
        ];
        let r = Rect::from_points(&pts);
        assert_eq!(r, Rect::from_ltrb(-2.0, -1.0, 4.0, 5.0));
    }

    #[test]
    fn from_points_empty_slice() {
        let r = Rect::from_points(&[]);
        assert_eq!(r, Rect::empty());
    }

    #[test]
    fn set_bounds_no_check_with_finite_points() {
        let mut r = Rect::empty();
        let pts = [
            Point::new(1.0, 2.0),
            Point::new(3.0, 4.0),
        ];
        r.set_bounds_no_check(&pts);
        assert_eq!(r, Rect::from_ltrb(1.0, 2.0, 3.0, 4.0));
    }

    #[test]
    fn set_bounds_no_check_with_nan_points() {
        let mut r = Rect::from_ltrb(0.0, 0.0, 10.0, 10.0);
        let pts = [
            Point::new(1.0, 2.0),
            Point::new(f32::NAN, 4.0),
        ];
        r.set_bounds_no_check(&pts);
        assert!(r.left.is_nan());
        assert!(r.top.is_nan());
        assert!(r.right.is_nan());
        assert!(r.bottom.is_nan());
    }

    #[test]
    fn set_bounds_no_check_with_infinity() {
        let mut r = Rect::empty();
        let pts = [Point::new(f32::INFINITY, 0.0)];
        r.set_bounds_no_check(&pts);
        assert!(r.left.is_nan());
    }

    #[test]
    fn set_bounds_no_check_empty_slice() {
        let mut r = Rect::from_ltrb(1.0, 1.0, 2.0, 2.0);
        r.set_bounds_no_check(&[]);
        assert!(r.is_empty());
    }

    #[test]
    fn intersection_returns_none_for_empty() {
        let a = Rect::empty();
        let b = Rect::from_wh(1.0, 1.0);
        assert!(Rect::intersection(&a, &b).is_none());
    }

    #[test]
    fn rects_intersect_returns_false_for_empty() {
        let a = Rect::empty();
        let b = Rect::from_wh(1.0, 1.0);
        assert!(!Rect::rects_intersect(&a, &b));
    }
}
