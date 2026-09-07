//! Shared reference-counted path storage.
//!
//! Ported from `src/core/SkPathRef.cpp` and `include/private/SkPathRef.h`.
//!
//! SkPathRef provides copy-on-write semantics for paths. Multiple SkPath
//! instances can share the same SkPathRef, which is reference-counted.

use super::{
    point::Point,
    rect::Rect,
    scalar::Scalar,
    types::Verb,
    types::{SEGMENT_MASK_CONIC, SEGMENT_MASK_CUBIC, SEGMENT_MASK_LINE, SEGMENT_MASK_QUAD},
};
use std::sync::Arc;

/// A reference-counted path storage.
///
/// SkPathRef provides copy-on-write semantics for paths. Multiple SkPath
/// instances can share the same SkPathRef, which is reference-counted.
#[derive(Debug)]
pub struct SkPathRef {
    pub(crate) verbs: Vec<u8>,
    pub(crate) points: Vec<Point>,
    pub(crate) conic_weights: Vec<Scalar>,
    pub(crate) bounds: Rect,
    pub(crate) bounds_is_dirty: bool,
    pub(crate) is_finite: bool,
    pub(crate) segment_mask: u8,
    pub(crate) is_oval: bool,
    pub(crate) is_rrect: bool,
    pub(crate) rrect_or_oval_start_idx: u32,
    pub(crate) rrect_or_oval_is_ccw: bool,
    generation_id: u32,
}

/// Generator ID for empty path
const EMPTY_GEN_ID: u32 = 1;

/// Mask for generation ID bits
const GENERATION_ID_MASK: u32 = 0x3FFFFFFF; // 30 bits

impl SkPathRef {
    /// Create a new empty SkPathRef
    pub fn new() -> Self {
        Self {
            verbs: Vec::new(),
            points: Vec::new(),
            conic_weights: Vec::new(),
            bounds: Rect::empty(),
            bounds_is_dirty: true,
            is_finite: false,
            segment_mask: 0,
            is_oval: false,
            is_rrect: false,
            rrect_or_oval_start_idx: 0,
            rrect_or_oval_is_ccw: false,
            generation_id: 0,
        }
    }

    /// Returns an Arc to an empty path
    pub fn create_empty() -> Arc<SkPathRef> {
        static EMPTY: once_cell::sync::Lazy<Arc<SkPathRef>> = once_cell::sync::Lazy::new(|| {
            let mut empty = SkPathRef::new();
            empty.compute_bounds();
            Arc::new(empty)
        });
        EMPTY.clone()
    }

    /// Computes the bounds of the path
    fn compute_bounds(&mut self) {
        self.bounds_is_dirty = false;
        if self.points.is_empty() {
            self.bounds = Rect::empty();
            self.is_finite = false;
        } else {
            self.bounds = Rect::from_ltrb(
                self.points[0].x,
                self.points[0].y,
                self.points[0].x,
                self.points[0].y,
            );
            for p in &self.points[1..] {
                self.bounds.left = self.bounds.left.min(p.x);
                self.bounds.top = self.bounds.top.min(p.y);
                self.bounds.right = self.bounds.right.max(p.x);
                self.bounds.bottom = self.bounds.bottom.max(p.y);
            }
            self.is_finite = self.bounds.is_finite();
        }
    }

    /// Get the generation ID, computing it if necessary
    pub fn gen_id(&mut self) -> u32 {
        if self.generation_id == 0 {
            if self.points.is_empty() && self.verbs.is_empty() {
                self.generation_id = EMPTY_GEN_ID;
            } else {
                use std::sync::atomic::{AtomicU32, Ordering};
                static NEXT_ID: AtomicU32 = AtomicU32::new(EMPTY_GEN_ID + 1);
                loop {
                    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed) & GENERATION_ID_MASK;
                    if id != 0 && id != EMPTY_GEN_ID {
                        self.generation_id = id;
                        break;
                    }
                }
            }
        }
        self.generation_id
    }

    /// Get a reference to the points
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    /// Get a mutable reference to the points
    pub fn writable_points(&mut self) -> &mut [Point] {
        &mut self.points
    }

    /// Get a reference to the verbs
    pub fn verbs(&self) -> &[u8] {
        &self.verbs
    }

    /// Get a mutable reference to the verbs
    pub fn verbs_mut(&mut self) -> &mut [u8] {
        &mut self.verbs
    }

    /// Get a reference to the conic weights
    pub fn conic_weights(&self) -> &[Scalar] {
        &self.conic_weights
    }

    /// Get a mutable reference to the conic weights
    pub fn conic_weights_mut(&mut self) -> &mut [Scalar] {
        &mut self.conic_weights
    }

    /// Get the bounds
    pub fn get_bounds(&self) -> Rect {
        if self.bounds_is_dirty {
            let mut bounds = Rect::empty();
            if !self.points.is_empty() {
                bounds = Rect::from_ltrb(
                    self.points[0].x,
                    self.points[0].y,
                    self.points[0].x,
                    self.points[0].y,
                );
                for p in &self.points[1..] {
                    bounds.left = bounds.left.min(p.x);
                    bounds.top = bounds.top.min(p.y);
                    bounds.right = bounds.right.max(p.x);
                    bounds.bottom = bounds.bottom.max(p.y);
                }
            }
            bounds
        } else {
            self.bounds
        }
    }

    /// Get whether the path has finite points
    pub fn is_finite(&self) -> bool {
        self.is_finite
    }

    /// Set whether the path has finite points
    pub fn set_is_finite(&mut self, finite: bool) {
        self.is_finite = finite;
    }

    /// Get whether bounds are dirty
    pub fn bounds_is_dirty(&self) -> bool {
        self.bounds_is_dirty
    }

    /// Mark bounds as dirty
    pub fn mark_bounds_dirty(&mut self) {
        self.bounds_is_dirty = true;
        self.is_finite = false;
    }

    /// Get the segment mask
    pub fn segment_mask(&self) -> u8 {
        self.segment_mask
    }

    /// Count points
    pub fn count_points(&self) -> usize {
        self.points.len()
    }

    /// Count verbs
    pub fn count_verbs(&self) -> usize {
        self.verbs.len()
    }

    /// Count weights (conic weights)
    pub fn count_weights(&self) -> usize {
        self.conic_weights.len()
    }

    /// Reset the path to a specific size
    pub fn reset_to_size(
        &mut self,
        _verb_cap: usize,
        _point_cap: usize,
        weight_cap: usize,
        verb_reserve: usize,
        point_reserve: usize,
    ) {
        self.verbs.clear();
        self.points.clear();
        self.conic_weights.clear();
        self.verbs.reserve(verb_reserve);
        self.points.reserve(point_reserve);
        self.conic_weights.reserve(weight_cap);
        self.bounds_is_dirty = true;
        self.is_finite = false;
    }

    /// Copy another SkPathRef
    pub fn copy(
        &mut self,
        src: &SkPathRef,
        additional_verb_reserve: usize,
        additional_point_reserve: usize,
    ) {
        self.reset_to_size(
            src.verbs.len(),
            src.points.len(),
            src.conic_weights.len(),
            additional_verb_reserve,
            additional_point_reserve,
        );
        self.verbs = src.verbs.clone();
        self.points = src.points.clone();
        self.conic_weights = src.conic_weights.clone();
        self.bounds_is_dirty = src.bounds_is_dirty;
        if !src.bounds_is_dirty {
            self.bounds = src.bounds;
            self.is_finite = src.is_finite;
        }
        self.segment_mask = src.segment_mask;
        self.is_oval = src.is_oval;
        self.is_rrect = src.is_rrect;
        self.rrect_or_oval_is_ccw = src.rrect_or_oval_is_ccw;
        self.rrect_or_oval_start_idx = src.rrect_or_oval_start_idx;
    }

    /// Grow to add verbs from another path
    pub fn grow_for_verbs_in_path(&mut self, path: &SkPathRef) -> Option<Point> {
        self.segment_mask |= path.segment_mask;
        self.bounds_is_dirty = true;
        self.is_finite = false;
        self.is_oval = false;
        self.is_rrect = false;

        if path.verbs.is_empty() {
            None
        } else {
            self.verbs.extend_from_slice(&path.verbs);
            Some(Point::new(
                self.points.len() as Scalar,
                self.points.len() as Scalar,
            ))
        }
    }

    /// Grow for a repeated verb (multiple same verb types)
    pub fn grow_for_repeated_verb(
        &mut self,
        verb: u8,
        num_verbs: usize,
    ) -> (Option<&mut Point>, Option<&mut Scalar>) {
        let point_count;
        let mask = match verb {
            verb if verb == Verb::Move as u8 => {
                point_count = num_verbs;
                0
            }
            verb if verb == Verb::Line as u8 => {
                point_count = num_verbs;
                SEGMENT_MASK_LINE
            }
            verb if verb == Verb::Quad as u8 => {
                point_count = 2 * num_verbs;
                SEGMENT_MASK_QUAD
            }
            verb if verb == Verb::Conic as u8 => {
                point_count = 2 * num_verbs;
                SEGMENT_MASK_CONIC
            }
            verb if verb == Verb::Cubic as u8 => {
                point_count = 3 * num_verbs;
                SEGMENT_MASK_CUBIC
            }
            _ => {
                point_count = 0;
                0
            }
        };

        self.segment_mask |= mask;
        self.bounds_is_dirty = true;
        self.is_finite = false;
        self.is_oval = false;
        self.is_rrect = false;

        let verbs_start = self.verbs.len();
        self.verbs.resize(verbs_start + num_verbs, verb);

        let points_start = self.points.len();
        self.points
            .resize(points_start + point_count, Point::new(0.0, 0.0));

        let weights = if verb == Verb::Conic as u8 {
            let weights_start = self.conic_weights.len();
            self.conic_weights.resize(weights_start + num_verbs, 1.0);
            self.conic_weights.get_mut(weights_start)
        } else {
            None
        };

        let points = if point_count > 0 {
            self.points.get_mut(points_start)
        } else {
            None
        };

        (points, weights)
    }

    /// Grow for a single verb
    pub fn grow_for_verb(&mut self, verb: u8, weight: Scalar) -> Option<Point> {
        let point_count;
        let mask = match verb {
            verb if verb == Verb::Move as u8 => {
                point_count = 1;
                0
            }
            verb if verb == Verb::Line as u8 => {
                point_count = 1;
                SEGMENT_MASK_LINE
            }
            verb if verb == Verb::Quad as u8 => {
                point_count = 2;
                SEGMENT_MASK_QUAD
            }
            verb if verb == Verb::Conic as u8 => {
                point_count = 2;
                SEGMENT_MASK_CONIC
            }
            verb if verb == Verb::Cubic as u8 => {
                point_count = 3;
                SEGMENT_MASK_CUBIC
            }
            _ => {
                point_count = 0;
                0
            }
        };

        self.segment_mask |= mask;
        self.bounds_is_dirty = true;
        self.is_finite = false;
        self.is_oval = false;
        self.is_rrect = false;

        self.verbs.push(verb);
        if verb == Verb::Conic as u8 {
            self.conic_weights.push(weight);
        }
        let points_start = self.points.len();
        self.points
            .resize(points_start + point_count, Point::new(0.0, 0.0));

        self.points.get_mut(points_start).cloned()
    }

    /// Interpolate between two paths
    pub fn interpolate(&self, ending: &SkPathRef, weight: Scalar, out: &mut SkPathRef) {
        if out.count_points() != self.count_points() {
            return;
        }

        for i in 0..out.count_points() {
            out.points[i] = Point::new(
                out.points[i].x * weight + ending.points[i].x * (1.0 - weight),
                out.points[i].y * weight + ending.points[i].y * (1.0 - weight),
            );
        }

        out.bounds_is_dirty = true;
        out.is_finite = false;
        out.is_oval = false;
        out.is_rrect = false;
    }

    /// Create a transformed copy of this path
    pub fn create_transformed_copy(
        src: &SkPathRef,
        matrix: &super::matrix::Matrix,
    ) -> Arc<SkPathRef> {
        if matrix.is_identity() {
            return Arc::new(src.clone());
        }

        let mut dst = SkPathRef::new();
        dst.verbs = src.verbs.clone();
        dst.conic_weights = src.conic_weights.clone();
        dst.points = src.points.clone();

        // Transform points
        for pt in dst.points.iter_mut() {
            *pt = matrix.map_point(*pt);
        }

        // Compute new bounds
        if !src.bounds_is_dirty && src.count_points() > 1 {
            dst.bounds = matrix.map_rect(&src.bounds);
            dst.bounds_is_dirty = false;
            dst.is_finite = src.is_finite && dst.bounds.is_finite();
            if !dst.is_finite {
                dst.bounds = Rect::empty();
            }
        } else {
            dst.bounds_is_dirty = true;
            dst.is_finite = false;
        }

        dst.segment_mask = src.segment_mask;
        dst.is_oval = src.is_oval;
        dst.is_rrect = src.is_rrect;
        dst.rrect_or_oval_start_idx = src.rrect_or_oval_start_idx;
        dst.rrect_or_oval_is_ccw = src.rrect_or_oval_is_ccw;

        Arc::new(dst)
    }

    /// Rewind the path
    pub fn rewind(path: &mut Option<Arc<SkPathRef>>) {
        let mut new_path = SkPathRef::new();
        new_path.bounds_is_dirty = true;
        new_path.is_finite = false;
        *path = Some(Arc::new(new_path));
    }
}

impl Clone for SkPathRef {
    fn clone(&self) -> Self {
        Self {
            verbs: self.verbs.clone(),
            points: self.points.clone(),
            conic_weights: self.conic_weights.clone(),
            bounds: self.bounds,
            bounds_is_dirty: self.bounds_is_dirty,
            is_finite: self.is_finite,
            segment_mask: self.segment_mask,
            is_oval: self.is_oval,
            is_rrect: self.is_rrect,
            rrect_or_oval_start_idx: self.rrect_or_oval_start_idx,
            rrect_or_oval_is_ccw: self.rrect_or_oval_is_ccw,
            generation_id: self.generation_id,
        }
    }
}

impl PartialEq for SkPathRef {
    fn eq(&self, other: &Self) -> bool {
        self.segment_mask == other.segment_mask
            && (self.generation_id != 0 && self.generation_id == other.generation_id
                || self.points == other.points
                    && self.conic_weights == other.conic_weights
                    && self.verbs == other.verbs)
    }
}

impl Default for SkPathRef {
    fn default() -> Self {
        Self::new()
    }
}

/// Iterator over path segments
pub struct PathIter<'a> {
    verbs: std::slice::Iter<'a, u8>,
    points: std::slice::Iter<'a, Point>,
    conic_weights: std::slice::Iter<'a, Scalar>,
}

impl<'a> PathIter<'a> {
    /// Create a new path iterator
    pub fn new(path: &'a SkPathRef) -> Self {
        Self {
            verbs: path.verbs.iter(),
            points: path.points.iter(),
            conic_weights: path.conic_weights.iter(),
        }
    }

    /// Get the next verb and its points
    pub fn next(&mut self, pts: &mut [Point; 4]) -> Option<Verb> {
        let verb = self.verbs.next().copied()?;

        match verb {
            verb if verb == Verb::Move as u8 => {
                pts[0] = *self.points.next().unwrap();
                Some(Verb::Move)
            }
            verb if verb == Verb::Line as u8 => {
                pts[0] = *self.points.next().unwrap();
                Some(Verb::Line)
            }
            verb if verb == Verb::Quad as u8 || verb == Verb::Conic as u8 => {
                pts[0] = *self.points.next().unwrap();
                pts[1] = *self.points.next().unwrap();
                if verb == Verb::Conic as u8 {
                    self.conic_weights.next();
                }
                if verb == Verb::Quad as u8 {
                    Some(Verb::Quad)
                } else {
                    Some(Verb::Conic)
                }
            }
            verb if verb == Verb::Cubic as u8 => {
                pts[0] = *self.points.next().unwrap();
                pts[1] = *self.points.next().unwrap();
                pts[2] = *self.points.next().unwrap();
                Some(Verb::Cubic)
            }
            verb if verb == Verb::Close as u8 => Some(Verb::Close),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_empty_path_ref() {
        let path_ref = SkPathRef::new();
        assert_eq!(path_ref.count_points(), 0);
        assert_eq!(path_ref.count_verbs(), 0);
        assert!(path_ref.bounds_is_dirty());
        assert!(!path_ref.is_finite());
    }

    #[test]
    fn test_create_empty() {
        let empty = SkPathRef::create_empty();
        assert_eq!(empty.count_verbs(), 0);
        assert!(!empty.bounds_is_dirty());
    }

    #[test]
    fn test_grow_for_verb() {
        let mut path_ref = SkPathRef::new();
        path_ref.grow_for_verb(Verb::Move as u8, 0.0);
        assert_eq!(path_ref.count_verbs(), 1);
        assert_eq!(path_ref.count_points(), 1);
    }

    #[test]
    fn test_grow_for_repeated_verb() {
        let mut path_ref = SkPathRef::new();
        let (points, weights) = path_ref.grow_for_repeated_verb(Verb::Line as u8, 3);
        assert!(points.is_some());
        assert!(weights.is_none());
        assert_eq!(path_ref.count_points(), 3);
    }

    #[test]
    fn test_gen_id() {
        let mut path_ref = SkPathRef::new();
        path_ref.points.push(Point::new(1.0, 2.0));
        path_ref.verbs.push(Verb::Move as u8);

        let id = path_ref.gen_id();
        assert!(id != 0);
        assert!(id != EMPTY_GEN_ID);

        // Second call should return same ID
        assert_eq!(path_ref.gen_id(), id);
    }

    #[test]
    fn test_interpolate() {
        let mut start = SkPathRef::new();
        start.points.push(Point::new(0.0, 0.0));
        start.points.push(Point::new(10.0, 10.0));
        start.verbs.push(Verb::Move as u8);
        start.verbs.push(Verb::Line as u8);
        start.bounds_is_dirty = false;

        let mut end = SkPathRef::new();
        end.points.push(Point::new(0.0, 0.0));
        end.points.push(Point::new(20.0, 20.0));
        end.verbs.push(Verb::Move as u8);
        end.verbs.push(Verb::Line as u8);
        end.bounds_is_dirty = false;

        let mut result = SkPathRef::new();
        result.points.push(Point::new(0.0, 0.0));
        result.points.push(Point::new(10.0, 10.0));
        result.verbs = start.verbs.clone();
        result.bounds_is_dirty = false;

        start.interpolate(&end, 0.5, &mut result);

        assert!(result.points[1].x > 10.0);
        assert!(result.points[1].x < 20.0);
    }

    #[test]
    fn test_clone() {
        let mut path_ref = SkPathRef::new();
        path_ref.points.push(Point::new(1.0, 2.0));
        path_ref.verbs.push(Verb::Move as u8);

        let cloned = path_ref.clone();
        assert_eq!(cloned.count_points(), 1);
        assert_eq!(cloned.points[0].x, 1.0);
    }

    #[test]
    fn test_partial_eq() {
        let path_ref1 = SkPathRef::new();
        let path_ref2 = SkPathRef::new();

        assert_eq!(path_ref1, path_ref2);
    }

    #[test]
    fn test_segment_mask() {
        let mut path_ref = SkPathRef::new();
        path_ref.grow_for_verb(Verb::Line as u8, 0.0);
        assert_eq!(path_ref.segment_mask(), SEGMENT_MASK_LINE);

        path_ref.grow_for_verb(Verb::Cubic as u8, 0.0);
        assert_eq!(
            path_ref.segment_mask(),
            SEGMENT_MASK_LINE | SEGMENT_MASK_CUBIC
        );
    }

    #[test]
    fn test_grow_for_conic_verb() {
        let mut path_ref = SkPathRef::new();
        let (points, weights) = path_ref.grow_for_repeated_verb(Verb::Conic as u8, 2);
        assert!(points.is_some());
        assert!(weights.is_some());
        assert_eq!(path_ref.count_points(), 4);
        assert_eq!(path_ref.conic_weights.len(), 2);
    }

    #[test]
    fn test_iter() {
        let mut path_ref = SkPathRef::new();
        path_ref.grow_for_verb(Verb::Move as u8, 0.0);
        path_ref.grow_for_verb(Verb::Line as u8, 0.0);

        let mut iter = PathIter::new(&path_ref);
        let mut pts = [Point::new(0.0, 0.0); 4];

        assert_eq!(iter.next(&mut pts), Some(Verb::Move));
        assert_eq!(iter.next(&mut pts), Some(Verb::Line));
        assert_eq!(iter.next(&mut pts), None);
    }

    #[test]
    fn test_copy() {
        let mut src = SkPathRef::new();
        src.points.push(Point::new(1.0, 2.0));
        src.verbs.push(Verb::Move as u8);
        src.bounds_is_dirty = false;
        src.bounds = Rect::from_ltrb(1.0, 2.0, 1.0, 2.0);

        let mut dst = SkPathRef::new();
        dst.copy(&src, 0, 0);

        assert_eq!(dst.count_points(), 1);
        assert_eq!(dst.points[0].x, 1.0);
        assert!(!dst.bounds_is_dirty());
    }

    #[test]
    fn test_grow_for_verbs_in_path() {
        let mut src = SkPathRef::new();
        src.verbs.push(Verb::Move as u8);
        src.points.push(Point::new(1.0, 2.0));

        let mut dst = SkPathRef::new();
        let result = dst.grow_for_verbs_in_path(&src);
        assert!(result.is_some());
        assert_eq!(dst.count_verbs(), 1);
    }
}
