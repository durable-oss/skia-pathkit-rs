//! Coincidence tracking: runs where two segments follow the same path.
//!
//! Port of Skia's `SkOpCoincidence.{h,cpp}`.
//!
//! Only the list plumbing is ported. The detection and repair passes
//! (`addMissing`, `expand`, `mark_collapsed`, `fix_up`) are still stubs; see
//! `TODO/06-op-coincidence.md`.

use super::sk_op_arena::{CoinId, OpArena, PtTId, SkCoincidentSpans};
use super::sk_op_segment::SkOpSegment;
use super::sk_op_span::{SkOpPtT, SkOpSpan};



/// Tracks runs where two segments follow the same path.
///
/// Port of `SkOpCoincidence`. The records themselves live in the arena as
/// [`SkCoincidentSpans`](super::sk_op_arena::SkCoincidentSpans); this holds
/// only the head of the list, matching how C++ keeps `fHead` and reaches
/// everything else through the global state.
#[derive(Debug, Default)]
pub struct SkOpCoincidence {
    /// First record in the list.
    pub f_head: Option<CoinId>,
    /// Records set aside during a pass and re-added afterwards.
    pub f_top: Option<CoinId>,
}

impl SkOpCoincidence {
    /// Returns an empty tracker.
    ///
    /// The arena is passed in at each call rather than held, since it also
    /// owns the segments and spans these records point at.
    #[must_use]
    pub fn new() -> Self {
        Self {
            f_head: None,
            f_top: None,
        }
    }

    /// Returns true when no coincident runs have been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.f_head.is_none()
    }

    /// Returns the number of records in the list.
    pub fn count(&self, arena: &OpArena) -> usize {
        let mut n = 0;
        let mut cur = self.f_head;
        while let Some(id) = cur {
            n += 1;
            cur = arena.coin(id).f_next;
        }
        n
    }

    /// Records that the run `coin_start`..`coin_end` matches
    /// `opp_start`..`opp_end` on another segment.
    ///
    /// Port of `SkOpCoincidence::add`. The record is pushed on the front of
    /// the list, as in C++.
    pub fn add_run(
        &mut self,
        arena: &mut OpArena,
        coin_start: PtTId,
        coin_end: PtTId,
        opp_start: PtTId,
        opp_end: PtTId,
        flipped: bool,
    ) -> CoinId {
        let id = arena.alloc_coin(SkCoincidentSpans {
            f_next: self.f_head,
            f_coin_ptt_start: Some(coin_start),
            f_coin_ptt_end: Some(coin_end),
            f_opp_ptt_start: Some(opp_start),
            f_opp_ptt_end: Some(opp_end),
            f_flipped: flipped,
            f_id: 0,
        });
        self.f_head = Some(id);
        arena.set_coincidence(self.f_head);
        id
    }

    /// Returns every record in the list, head first.
    pub fn records(&self, arena: &OpArena) -> Vec<CoinId> {
        let mut out = Vec::new();
        let mut cur = self.f_head;
        while let Some(id) = cur {
            out.push(id);
            cur = arena.coin(id).f_next;
        }
        out
    }

    /// Not ported. See `TODO/06-op-coincidence.md`.
    ///
    /// Use [`Self::add_run`], which records a run in the arena. This
    /// signature is kept because the orphaned `sk_add_intersections` calls it.
    pub fn add(&mut self, _start: &mut SkOpPtT, _end: &mut SkOpPtT) -> bool {
        true
    }

    /// Not ported: fills in runs implied by other coincidences.
    pub fn add_missing(
        &mut self,
        _seg: &mut SkOpSegment,
        _start: &SkOpSpan,
        _end: &SkOpSpan,
    ) -> bool {
        true
    }

    /// Not ported: grows runs to their full extent.
    pub fn expand(&mut self) -> bool {
        true
    }

    /// Not ported: marks a span whose segment collapsed to a point.
    pub fn mark_collapsed(&mut self, _span: &mut SkOpSpan) -> bool {
        true
    }

    /// Not ported: repairs runs whose endpoints moved.
    pub fn fix_up(&mut self) -> bool {
        true
    }

    /// Nothing to do: the arena drops wholesale rather than freeing nodes.
    pub fn release_deleted(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Point;

    fn ptt(arena: &mut OpArena, t: f32) -> PtTId {
        arena.alloc_ptt(SkOpPtT::new(t, Point::new(t, 0.0), None))
    }

    #[test]
    fn a_new_tracker_is_empty() {
        let arena = OpArena::new();
        let coin = SkOpCoincidence::new();
        assert!(coin.is_empty());
        assert_eq!(coin.count(&arena), 0);
        assert!(coin.records(&arena).is_empty());
    }

    #[test]
    fn adding_a_run_records_it_in_the_arena() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let (a, b, c, d) = (
            ptt(&mut arena, 0.0),
            ptt(&mut arena, 0.5),
            ptt(&mut arena, 0.25),
            ptt(&mut arena, 0.75),
        );
        let id = coin.add_run(&mut arena, a, b, c, d, false);

        assert!(!coin.is_empty());
        assert_eq!(coin.count(&arena), 1);
        let rec = arena.coin(id);
        assert_eq!(rec.f_coin_ptt_start, Some(a));
        assert_eq!(rec.f_coin_ptt_end, Some(b));
        assert_eq!(rec.f_opp_ptt_start, Some(c));
        assert_eq!(rec.f_opp_ptt_end, Some(d));
        assert!(!rec.f_flipped);
        // The arena's own head points at it too.
        assert_eq!(arena.coincidence(), Some(id));
    }

    #[test]
    fn runs_are_pushed_on_the_front() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let a = ptt(&mut arena, 0.0);
        let b = ptt(&mut arena, 1.0);
        let first = coin.add_run(&mut arena, a, b, a, b, false);
        let second = coin.add_run(&mut arena, a, b, a, b, true);
        assert_eq!(coin.count(&arena), 2);
        // Most recent first, as in C++.
        assert_eq!(coin.records(&arena), vec![second, first]);
        assert!(arena.coin(second).f_flipped);
        assert!(!arena.coin(first).f_flipped);
    }
}
