//! Coincidence tracking: runs where two segments follow the same path.
//!
//! Port of Skia's `SkOpCoincidence.{h,cpp}`.
//!
//! # What coincidence is for
//!
//! Two contours that share an edge would each count that edge's winding, so
//! the shared run reads as interior to both and the result keeps a seam that
//! should not be there. [`SkOpCoincidence::apply`] is the fix: it folds the
//! pair's winding onto one side and zeroes the other, so the edge is counted
//! once.

use super::sk_op_arena::{CoinId, OpArena, PtTId, SegmentId, SkCoincidentSpans};
use crate::core::Verb;



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

    /// Returns true when the two runs are traced in opposite directions.
    fn flipped(arena: &OpArena, coin: CoinId) -> bool {
        arena.coin(coin).f_flipped
    }

    /// Returns the segment a PtT node belongs to.
    fn ptt_seg(arena: &OpArena, id: Option<PtTId>) -> Option<SegmentId> {
        arena.ptt_segment(id?)
    }

    /// Returns the t of a PtT node.
    fn ptt_t(arena: &OpArena, id: Option<PtTId>) -> f32 {
        id.map_or(0.0, |i| arena.ptt(i).f_t)
    }

    /// Returns true when `coin_seg` sorts before `opp_seg`.
    ///
    /// Port of `SkOpCoincidence::Ordered`. The ordering is by verb first,
    /// then lexicographically through the raw coordinates. It exists only to
    /// give a coincident pair a canonical orientation, so that A-B and B-A
    /// are stored the same way and compare equal.
    #[must_use]
    pub fn ordered_segments(arena: &OpArena, coin_seg: SegmentId, opp_seg: SegmentId) -> bool {
        let (c_pts, c_verb, _) = arena.segment_curve(coin_seg);
        let (o_pts, o_verb, _) = arena.segment_curve(opp_seg);
        let c_rank = verb_rank(c_verb);
        let o_rank = verb_rank(o_verb);
        if c_rank != o_rank {
            return c_rank < o_rank;
        }
        for (c, o) in c_pts.iter().zip(o_pts.iter()) {
            for (cv, ov) in [(c.x, o.x), (c.y, o.y)] {
                if cv < ov {
                    return true;
                }
                if cv > ov {
                    return false;
                }
            }
        }
        true
    }

    /// Returns true when the two t ranges overlap.
    ///
    /// Port of `SkOpCoincidence::Overlaps`. The ranges may be given in
    /// either order; both are normalized before comparing.
    #[must_use]
    pub fn overlaps(s1: f32, e1: f32, s2: f32, e2: f32) -> Option<(f32, f32)> {
        let (lo1, hi1) = if s1 < e1 { (s1, e1) } else { (e1, s1) };
        let (lo2, hi2) = if s2 < e2 { (s2, e2) } else { (e2, s2) };
        let lo = lo1.max(lo2);
        let hi = hi1.min(hi2);
        if lo < hi {
            Some((lo, hi))
        } else {
            None
        }
    }

    /// Returns true when the record covers the run `s`..`e`.
    ///
    /// Port of `SkCoincidentSpans::contains`. Which pair of endpoints to
    /// compare against is decided by which segment `s` sits on.
    #[must_use]
    pub fn record_contains(arena: &OpArena, coin: CoinId, s: PtTId, e: PtTId) -> bool {
        let (s, e) = if arena.ptt(s).f_t > arena.ptt(e).f_t {
            (e, s)
        } else {
            (s, e)
        };
        let (s_t, e_t) = (arena.ptt(s).f_t, arena.ptt(e).f_t);
        let rec = arena.coin(coin);
        let seg = arena.ptt_segment(s);
        if seg.is_some() && seg == Self::ptt_seg(arena, rec.f_coin_ptt_start) {
            return Self::ptt_t(arena, rec.f_coin_ptt_start) <= s_t
                && e_t <= Self::ptt_t(arena, rec.f_coin_ptt_end);
        }
        let (mut o_s, mut o_e) = (
            Self::ptt_t(arena, rec.f_opp_ptt_start),
            Self::ptt_t(arena, rec.f_opp_ptt_end),
        );
        if o_s > o_e {
            std::mem::swap(&mut o_s, &mut o_e);
        }
        o_s <= s_t && e_t <= o_e
    }

    /// Widens a record to cover a run that reaches past its current ends.
    ///
    /// Port of `SkCoincidentSpans::extend`. Returns true when either end
    /// moved. Note the flipped cases compare the opposite ends the other way
    /// round: on a flipped pair, the opposite run's t *decreases* as the
    /// coincident run's increases.
    pub fn extend_record(
        arena: &mut OpArena,
        coin: CoinId,
        coin_start: PtTId,
        coin_end: PtTId,
        opp_start: PtTId,
        opp_end: PtTId,
    ) -> bool {
        let flipped = Self::flipped(arena, coin);
        let mut result = false;
        let rec = arena.coin(coin).clone();
        let start_moves = Self::ptt_t(arena, rec.f_coin_ptt_start) > arena.ptt(coin_start).f_t
            || if flipped {
                Self::ptt_t(arena, rec.f_opp_ptt_start) < arena.ptt(opp_start).f_t
            } else {
                Self::ptt_t(arena, rec.f_opp_ptt_start) > arena.ptt(opp_start).f_t
            };
        if start_moves {
            let r = arena.coin_mut(coin);
            r.f_coin_ptt_start = Some(coin_start);
            r.f_opp_ptt_start = Some(opp_start);
            result = true;
        }
        let end_moves = Self::ptt_t(arena, rec.f_coin_ptt_end) < arena.ptt(coin_end).f_t
            || if flipped {
                Self::ptt_t(arena, rec.f_opp_ptt_end) > arena.ptt(opp_end).f_t
            } else {
                Self::ptt_t(arena, rec.f_opp_ptt_end) < arena.ptt(opp_end).f_t
            };
        if end_moves {
            let r = arena.coin_mut(coin);
            r.f_coin_ptt_end = Some(coin_end);
            r.f_opp_ptt_end = Some(opp_end);
            result = true;
        }
        result
    }

    /// Adds a run, widening an existing record instead when one overlaps.
    ///
    /// Port of `SkOpCoincidence::extend` followed by `add`. The pair is put
    /// in canonical order first, so the same coincidence discovered from
    /// either side lands on one record.
    pub fn add_or_extend(
        &mut self,
        arena: &mut OpArena,
        coin_start: PtTId,
        coin_end: PtTId,
        opp_start: PtTId,
        opp_end: PtTId,
    ) -> Option<CoinId> {
        let (mut cs, mut ce, mut os, mut oe) = (coin_start, coin_end, opp_start, opp_end);
        let coin_seg = arena.ptt_segment(cs)?;
        let opp_seg = arena.ptt_segment(os)?;
        if !Self::ordered_segments(arena, coin_seg, opp_seg) {
            std::mem::swap(&mut cs, &mut os);
            std::mem::swap(&mut ce, &mut oe);
            if arena.ptt(cs).f_t > arena.ptt(ce).f_t {
                std::mem::swap(&mut cs, &mut ce);
                std::mem::swap(&mut os, &mut oe);
            }
        }
        let (coin_seg, opp_seg) = (arena.ptt_segment(cs)?, arena.ptt_segment(os)?);

        for test in self.records(arena) {
            let rec = arena.coin(test).clone();
            if Self::ptt_seg(arena, rec.f_coin_ptt_start) != Some(coin_seg)
                || Self::ptt_seg(arena, rec.f_opp_ptt_start) != Some(opp_seg)
            {
                continue;
            }
            let coin_overlap = Self::overlaps(
                Self::ptt_t(arena, rec.f_coin_ptt_start),
                Self::ptt_t(arena, rec.f_coin_ptt_end),
                arena.ptt(cs).f_t,
                arena.ptt(ce).f_t,
            );
            let opp_overlap = Self::overlaps(
                Self::ptt_t(arena, rec.f_opp_ptt_start),
                Self::ptt_t(arena, rec.f_opp_ptt_end),
                arena.ptt(os).f_t,
                arena.ptt(oe).f_t,
            );
            if coin_overlap.is_some() || opp_overlap.is_some() {
                Self::extend_record(arena, test, cs, ce, os, oe);
                return Some(test);
            }
        }
        // A pair whose opposite run descends as the coincident one ascends is
        // traced the other way round.
        let flipped = arena.ptt(os).f_t > arena.ptt(oe).f_t;
        Some(self.add_run(arena, cs, ce, os, oe, flipped))
    }

    /// Writes coincidence back onto the spans each record covers.
    ///
    /// Port of `SkOpCoincidence::mark`. The two runs need not have the same
    /// number of spans, so the ends are linked explicitly and the interiors
    /// are marked by walking each side independently.
    ///
    /// Returns false when a record names a span that cannot carry the mark.
    pub fn mark(&mut self, arena: &mut OpArena) -> bool {
        for coin in self.records(arena) {
            let rec = arena.coin(coin).clone();
            let (Some(start), Some(end), Some(o_start), Some(o_end)) = (
                rec.f_coin_ptt_start.and_then(|p| arena.ptt_span(p)),
                rec.f_coin_ptt_end.and_then(|p| arena.ptt_span(p)),
                rec.f_opp_ptt_start.and_then(|p| arena.ptt_span(p)),
                rec.f_opp_ptt_end.and_then(|p| arena.ptt_span(p)),
            ) else {
                return false;
            };
            // On a flipped pair the opposite run is walked from its far end.
            let (o_start, o_end) = if rec.f_flipped {
                (o_end, o_start)
            } else {
                (o_start, o_end)
            };
            arena.span_insert_coincidence(start, o_start);
            arena.span_insert_coin_end(end, o_end);

            // Interiors: each side is walked to its own end, since the two
            // runs may be split at different t values.
            let mut next = start;
            let mut guard = MARK_SAFETY;
            while let Some(n) = arena.span_next(next) {
                if n == end {
                    break;
                }
                guard -= 1;
                if guard == 0 {
                    return false;
                }
                arena.span_insert_coincidence(n, o_start);
                next = n;
            }
            let mut o_next = o_start;
            let mut guard = MARK_SAFETY;
            while let Some(n) = arena.span_next(o_next) {
                if n == o_end {
                    break;
                }
                guard -= 1;
                if guard == 0 {
                    return false;
                }
                arena.span_insert_coincidence(n, start);
                o_next = n;
            }
        }
        true
    }

    /// Folds each coincident pair's winding onto one side of the pair.
    ///
    /// Port of `SkOpCoincidence::apply`. This is where coincidence actually
    /// changes the answer: two contours sharing an edge have that edge
    /// counted once, on whichever side carries the larger winding, and the
    /// other side is zeroed and marked done so the walker never emits it.
    ///
    /// Returns false when a winding would go negative, which means the
    /// records disagree with the spans and the result cannot be trusted.
    pub fn apply(&mut self, arena: &mut OpArena) -> bool {
        for coin in self.records(arena) {
            if !Self::apply_one(arena, coin) {
                return false;
            }
        }
        true
    }

    /// Applies one record. Split out so the loop above stays readable.
    fn apply_one(arena: &mut OpArena, coin: CoinId) -> bool {
        let rec = arena.coin(coin).clone();
        let flipped = rec.f_flipped;
        let (Some(mut start), Some(end)) = (
            rec.f_coin_ptt_start.and_then(|p| arena.ptt_span(p)),
            rec.f_coin_ptt_end.and_then(|p| arena.ptt_span(p)),
        ) else {
            return true;
        };
        let (o_start_ptt, o_end_ptt) = if flipped {
            (rec.f_opp_ptt_end, rec.f_opp_ptt_start)
        } else {
            (rec.f_opp_ptt_start, rec.f_opp_ptt_end)
        };
        let (Some(mut o_start), Some(o_end)) = (
            o_start_ptt.and_then(|p| arena.ptt_span(p)),
            o_end_ptt.and_then(|p| arena.ptt_span(p)),
        ) else {
            return true;
        };
        let (Some(segment), Some(o_segment)) =
            (arena.span_segment(start), arena.span_segment(o_start))
        else {
            return true;
        };
        let operand_swap = arena.segment_operand(segment) != arena.segment_operand(o_segment);

        if flipped {
            // Walk the opposite run to its far end first, so both sides then
            // advance towards each other.
            let mut guard = MARK_SAFETY;
            while let Some(o_next) = arena.span_next(o_start) {
                if o_next == o_end {
                    break;
                }
                guard -= 1;
                if guard == 0 {
                    return false;
                }
                o_start = o_next;
            }
        }

        let mut guard = MARK_SAFETY;
        loop {
            guard -= 1;
            if guard == 0 {
                return false;
            }
            let mut wind_value = arena.span(start).wind_value();
            let mut opp_value = arena.span(start).opp_value();
            let mut o_wind_value = arena.span(o_start).wind_value();
            let mut o_opp_value = arena.span(o_start).opp_value();

            // Which operand each side contributes to depends on whether the
            // two segments come from the same input path.
            let mut wind_diff = if operand_swap { o_opp_value } else { o_wind_value };
            let mut o_wind_diff = if operand_swap { opp_value } else { wind_value };
            if !flipped {
                wind_diff = -wind_diff;
                o_wind_diff = -o_wind_diff;
            }
            // The side with more winding absorbs the other; a tie goes to the
            // coincident side unless it is already walked.
            let mut add_to_start = wind_value != 0
                && (wind_value > wind_diff
                    || (wind_value == wind_diff && o_wind_value <= o_wind_diff));
            let done = if add_to_start {
                arena.span(start).done()
            } else {
                arena.span(o_start).done()
            };
            if done {
                add_to_start = !add_to_start;
            }

            if add_to_start {
                if operand_swap {
                    std::mem::swap(&mut o_wind_value, &mut o_opp_value);
                }
                if flipped {
                    wind_value -= o_wind_value;
                    opp_value -= o_opp_value;
                } else {
                    wind_value += o_wind_value;
                    opp_value += o_opp_value;
                }
                if arena.segment_is_xor(segment) {
                    wind_value &= 1;
                }
                if arena.segment_opp_xor(segment) {
                    opp_value &= 1;
                }
                o_wind_value = 0;
                o_opp_value = 0;
            } else {
                if operand_swap {
                    std::mem::swap(&mut wind_value, &mut opp_value);
                }
                if flipped {
                    o_wind_value -= wind_value;
                    o_opp_value -= opp_value;
                } else {
                    o_wind_value += wind_value;
                    o_opp_value += opp_value;
                }
                if arena.segment_is_xor(o_segment) {
                    o_wind_value &= 1;
                }
                if arena.segment_opp_xor(o_segment) {
                    o_opp_value &= 1;
                }
                wind_value = 0;
                opp_value = 0;
            }

            if wind_value < 0 || o_wind_value < 0 {
                return false;
            }
            arena.span_mut(start).set_wind_value(wind_value);
            arena.span_mut(start).set_opp_value(opp_value);
            arena.span_mut(o_start).set_wind_value(o_wind_value);
            arena.span_mut(o_start).set_opp_value(o_opp_value);
            // A span contributing nothing to either operand is finished.
            if wind_value == 0 && opp_value == 0 {
                arena.mark_done(start);
            }
            if o_wind_value == 0 && o_opp_value == 0 {
                arena.mark_done(o_start);
            }

            let next = arena.span_next(start);
            let o_next = if flipped {
                arena.span_prev(o_start)
            } else {
                arena.span_next(o_start)
            };
            match next {
                Some(n) if n != end => start = n,
                _ => break,
            }
            // If the opposite run ran out first, keep reusing its last span:
            // the remaining coincident spans still have to be zeroed.
            if let Some(o_n) = o_next {
                o_start = o_n;
            }
        }
        true
    }

    /// Drops records whose endpoints were deleted.
    ///
    /// Port of `SkOpCoincidence::releaseDeleted`. Nothing is freed - the
    /// arena drops wholesale - but the record is unlinked so later passes do
    /// not walk through a retired PtT node.
    pub fn release_deleted(&mut self, arena: &mut OpArena) {
        let mut kept = Vec::new();
        for coin in self.records(arena) {
            let start_deleted = arena
                .coin(coin)
                .f_coin_ptt_start
                .is_some_and(|p| arena.ptt(p).f_deleted);
            if !start_deleted {
                kept.push(coin);
            }
        }
        self.relink(arena, &kept);
    }

    /// Replaces the list with `kept`, in the order given.
    fn relink(&mut self, arena: &mut OpArena, kept: &[CoinId]) {
        self.f_head = kept.first().copied();
        for pair in kept.windows(2) {
            arena.coin_mut(pair[0]).f_next = Some(pair[1]);
        }
        if let Some(&last) = kept.last() {
            arena.coin_mut(last).f_next = None;
        }
        arena.set_coincidence(self.f_head);
    }

    /// Repoints every reference to `deleted` at `kept`, dropping a record
    /// that would collapse to a point.
    ///
    /// Port of `SkOpCoincidence::fixUp`. A record whose two ends land on the
    /// same span no longer describes a run, so it is released rather than
    /// repaired.
    pub fn fix_up(&mut self, arena: &mut OpArena, deleted: PtTId, kept: PtTId) {
        debug_assert_ne!(deleted, kept);
        let kept_span = arena.ptt_span(kept);
        let mut survivors = Vec::new();
        for coin in self.records(arena) {
            let rec = arena.coin(coin).clone();
            // Each end is repointed unless doing so would put both ends of
            // its own pair on one span, which is a record of nothing.
            let repoint = |slot: Option<PtTId>, other: Option<PtTId>| {
                if slot != Some(deleted) {
                    return (slot, false);
                }
                if other.and_then(|o| arena.ptt_span(o)) == kept_span {
                    (slot, true)
                } else {
                    (Some(kept), false)
                }
            };
            let (coin_start, c1) = repoint(rec.f_coin_ptt_start, rec.f_coin_ptt_end);
            let (coin_end, c2) = repoint(rec.f_coin_ptt_end, rec.f_coin_ptt_start);
            let (opp_start, c3) = repoint(rec.f_opp_ptt_start, rec.f_opp_ptt_end);
            let (opp_end, c4) = repoint(rec.f_opp_ptt_end, rec.f_opp_ptt_start);
            if c1 || c2 || c3 || c4 {
                continue;
            }
            let r = arena.coin_mut(coin);
            r.f_coin_ptt_start = coin_start;
            r.f_coin_ptt_end = coin_end;
            r.f_opp_ptt_start = opp_start;
            r.f_opp_ptt_end = opp_end;
            survivors.push(coin);
        }
        self.relink(arena, &survivors);
    }

    /// Fills `overlaps` with the pairs implied by records that share a
    /// segment.
    ///
    /// Port of `SkOpCoincidence::findOverlaps`. When A is coincident with B
    /// and B with C over ranges that overlap, A and C are coincident over the
    /// shared part, and that pair is not in the list until this puts it
    /// there. `HandleCoincidence` drains the result and re-runs `apply`.
    ///
    /// Two records whose *first* segments are the same add nothing: they
    /// already agree on which side wins.
    pub fn find_overlaps(&self, arena: &mut OpArena, overlaps: &mut SkOpCoincidence) -> bool {
        overlaps.f_head = None;
        overlaps.f_top = None;
        let records = self.records(arena);
        for (i, &outer) in records.iter().enumerate() {
            let o = arena.coin(outer).clone();
            let (Some(outer_coin), Some(outer_opp)) = (
                Self::ptt_seg(arena, o.f_coin_ptt_start),
                Self::ptt_seg(arena, o.f_opp_ptt_start),
            ) else {
                continue;
            };
            for &inner in &records[i + 1..] {
                let n = arena.coin(inner).clone();
                let (Some(inner_coin), Some(inner_opp)) = (
                    Self::ptt_seg(arena, n.f_coin_ptt_start),
                    Self::ptt_seg(arena, n.f_opp_ptt_start),
                ) else {
                    continue;
                };
                if outer_coin == inner_coin {
                    // Same receiver, so there is no further overlap to find.
                    continue;
                }
                // The three ways the two records can meet on one segment.
                let shared = if outer_opp == inner_coin {
                    Self::shared_range(
                        arena,
                        (o.f_opp_ptt_start, o.f_opp_ptt_end),
                        (n.f_coin_ptt_start, n.f_coin_ptt_end),
                    )
                } else if outer_coin == inner_opp {
                    Self::shared_range(
                        arena,
                        (o.f_coin_ptt_start, o.f_coin_ptt_end),
                        (n.f_opp_ptt_start, n.f_opp_ptt_end),
                    )
                } else if outer_opp == inner_opp {
                    Self::shared_range(
                        arena,
                        (o.f_opp_ptt_start, o.f_opp_ptt_end),
                        (n.f_opp_ptt_start, n.f_opp_ptt_end),
                    )
                } else {
                    None
                };
                let Some((over_s, over_e)) = shared else {
                    continue;
                };
                if !overlaps.add_overlap(
                    arena,
                    (outer_coin, outer_opp),
                    (inner_coin, inner_opp),
                    over_s,
                    over_e,
                ) {
                    return false;
                }
            }
        }
        true
    }

    /// Records the pair implied by two records overlapping over `over_s`
    /// to `over_e`.
    ///
    /// Port of `SkOpCoincidence::addOverlap`. The overlap range is given as
    /// PtT nodes on some segment; each side of the new pair is found by
    /// looking those nodes up on its own segment through the PtT ring. A side
    /// whose spans carry no winding is not a real edge, so the first segment
    /// is tried and then its partner, and if neither contributes the overlap
    /// is dropped.
    fn add_overlap(
        &mut self,
        arena: &mut OpArena,
        first: (SegmentId, SegmentId),
        second: (SegmentId, SegmentId),
        over_s: PtTId,
        over_e: PtTId,
    ) -> bool {
        let Some((mut s1, mut e1)) = Self::find_contributing(arena, over_s, over_e, first) else {
            return true;
        };
        let Some((mut s2, mut e2)) = Self::find_contributing(arena, over_s, over_e, second) else {
            return true;
        };
        if arena.ptt_segment(s1) == arena.ptt_segment(s2) {
            // Both sides landed on one segment, which is not a pair.
            return true;
        }
        if arena.ptt(s1).f_t > arena.ptt(e1).f_t {
            std::mem::swap(&mut s1, &mut e1);
            std::mem::swap(&mut s2, &mut e2);
        }
        self.add_or_extend(arena, s1, e1, s2, e2).is_some()
    }

    /// Finds the overlap's endpoints on whichever of the two segments has
    /// winding to contribute.
    fn find_contributing(
        arena: &OpArena,
        over_s: PtTId,
        over_e: PtTId,
        pair: (SegmentId, SegmentId),
    ) -> Option<(PtTId, PtTId)> {
        for seg in [pair.0, pair.1] {
            let (Some(s), Some(e)) = (
                arena.ptt_find(over_s, seg),
                arena.ptt_find(over_e, seg),
            ) else {
                continue;
            };
            let has_winding = arena
                .ptt_span(s)
                .and_then(|ss| arena.ptt_span(e).map(|ee| (ss, ee)))
                .and_then(|(ss, ee)| arena.span_starter(ss, ee))
                .is_some_and(|starter| arena.span(starter).wind_value() != 0);
            if has_winding {
                return Some((s, e));
            }
        }
        None
    }

    /// Returns the PtT pair bounding the range two runs share, if any.
    ///
    /// Port of `SkOpPtT::Overlaps`, reduced to the endpoints the caller
    /// needs: the later of the two starts and the earlier of the two ends.
    fn shared_range(
        arena: &OpArena,
        a: (Option<PtTId>, Option<PtTId>),
        b: (Option<PtTId>, Option<PtTId>),
    ) -> Option<(PtTId, PtTId)> {
        let (a_s, a_e, b_s, b_e) = (a.0?, a.1?, b.0?, b.1?);
        let (a_s, a_e) = if arena.ptt(a_s).f_t <= arena.ptt(a_e).f_t {
            (a_s, a_e)
        } else {
            (a_e, a_s)
        };
        let (b_s, b_e) = if arena.ptt(b_s).f_t <= arena.ptt(b_e).f_t {
            (b_s, b_e)
        } else {
            (b_e, b_s)
        };
        let start = if arena.ptt(a_s).f_t >= arena.ptt(b_s).f_t {
            a_s
        } else {
            b_s
        };
        let end = if arena.ptt(a_e).f_t <= arena.ptt(b_e).f_t {
            a_e
        } else {
            b_e
        };
        if arena.ptt(start).f_t < arena.ptt(end).f_t {
            Some((start, end))
        } else {
            None
        }
    }

    /// Widens every record to the largest run its endpoints support, and
    /// drops any that became duplicates.
    ///
    /// Port of `SkOpCoincidence::expand`. Two records that grow into the same
    /// run are one record; leaving both would apply the same winding fold
    /// twice. Returns true when anything moved, which is how
    /// `HandleCoincidence` knows to re-run the passes that feed on it.
    pub fn expand(&mut self, arena: &mut OpArena) -> bool {
        let records = self.records(arena);
        let mut expanded = false;
        let mut duplicates = Vec::new();
        for (i, &coin) in records.iter().enumerate() {
            if duplicates.contains(&coin) {
                continue;
            }
            let c = arena.coin(coin).clone();
            let (Some(cs), Some(ce), Some(os), Some(oe)) = (
                c.f_coin_ptt_start,
                c.f_coin_ptt_end,
                c.f_opp_ptt_start,
                c.f_opp_ptt_end,
            ) else {
                continue;
            };
            // Reach out along each end's PtT ring for a node further along
            // the same segment than the record currently claims.
            let _ = (os, oe);
            let widened = Self::reach_out(arena, coin, cs, ce);
            if !widened {
                continue;
            }
            expanded = true;
            for &test in &records[i + 1..] {
                let t = arena.coin(test).clone();
                let c = arena.coin(coin).clone();
                // Both runs, both ends. Matching starts alone is not enough:
                // `reach_out` widens the coincident side without touching the
                // opposite one, so two records can briefly agree on where
                // they begin while describing different runs. Dropping one
                // then loses a real coincidence, and the segment it covered
                // keeps a winding it should have given up.
                if t.f_coin_ptt_start == c.f_coin_ptt_start
                    && t.f_coin_ptt_end == c.f_coin_ptt_end
                    && t.f_opp_ptt_start == c.f_opp_ptt_start
                    && t.f_opp_ptt_end == c.f_opp_ptt_end
                {
                    duplicates.push(test);
                }
            }
        }
        if !duplicates.is_empty() {
            let kept: Vec<CoinId> = self
                .records(arena)
                .into_iter()
                .filter(|c| !duplicates.contains(c))
                .collect();
            self.relink(arena, &kept);
        }
        expanded
    }

    /// Tries to push one record's ends outward onto aliased PtT nodes.
    ///
    /// Returns true when either end moved.
    fn reach_out(arena: &mut OpArena, coin: CoinId, cs: PtTId, ce: PtTId) -> bool {
        let Some(coin_seg) = arena.ptt_segment(cs) else {
            return false;
        };
        let mut moved = false;
        // The start may move earlier, the end later, but only onto a node
        // already in the same ring: this loosens the record, it does not
        // invent coincidence that was not found.
        let mut best_start = cs;
        for node in arena.ptt_ring(cs) {
            if arena.ptt_segment(node) == Some(coin_seg)
                && arena.ptt(node).f_t < arena.ptt(best_start).f_t
            {
                best_start = node;
            }
        }
        let mut best_end = ce;
        for node in arena.ptt_ring(ce) {
            if arena.ptt_segment(node) == Some(coin_seg)
                && arena.ptt(node).f_t > arena.ptt(best_end).f_t
            {
                best_end = node;
            }
        }
        if best_start != cs {
            arena.coin_mut(coin).f_coin_ptt_start = Some(best_start);
            moved = true;
        }
        if best_end != ce {
            arena.coin_mut(coin).f_coin_ptt_end = Some(best_end);
            moved = true;
        }
        moved
    }

    /// Adds a span to whichever side of each run is missing one.
    ///
    /// Port of `SkOpCoincidence::addExpanded`. The two runs of a coincident
    /// pair are split at whatever t values intersection happened to find, and
    /// those need not line up. Where one side has a span the other does not,
    /// this inserts the matching one, interpolating its t from how far along
    /// the run the existing span sits.
    ///
    /// Without it, `mark` and `apply` walk two runs of different lengths and
    /// pair spans that are not at the same place.
    pub fn add_expanded(&mut self, arena: &mut OpArena) -> bool {
        for coin in self.records(arena) {
            if !Self::add_expanded_one(arena, coin) {
                return false;
            }
        }
        true
    }

    /// Expands one record. Split out to keep the loop readable.
    fn add_expanded_one(arena: &mut OpArena, coin: CoinId) -> bool {
        let rec = arena.coin(coin).clone();
        let (Some(start_ptt), Some(end_ptt), Some(o_start_ptt), Some(o_end_ptt)) = (
            rec.f_coin_ptt_start,
            rec.f_coin_ptt_end,
            rec.f_opp_ptt_start,
            rec.f_opp_ptt_end,
        ) else {
            return true;
        };
        let (Some(seg), Some(o_seg)) = (
            arena.ptt_segment(start_ptt),
            arena.ptt_segment(o_start_ptt),
        ) else {
            return true;
        };
        let (t0, t1) = (arena.ptt(start_ptt).f_t, arena.ptt(end_ptt).f_t);
        let (o_t0, o_t1) = (arena.ptt(o_start_ptt).f_t, arena.ptt(o_end_ptt).f_t);
        let (range, o_range) = (t1 - t0, o_t1 - o_t0);
        if range == 0.0 || o_range == 0.0 {
            return true;
        }

        // Every interior t on each side, mapped to the other side's range.
        let coin_ts: Vec<f32> = interior_ts(arena, seg, t0, t1);
        let opp_ts: Vec<f32> = interior_ts(arena, o_seg, o_t0, o_t1);

        for t in &coin_ts {
            // Where along the run this span sits, as a fraction.
            let part = (t - t0) / range;
            let want = o_t0 + o_range * part;
            if !opp_ts.iter().any(|o| (o - want).abs() < T_MATCH) {
                let pt = arena.segment_pt_at_t(o_seg, want);
                arena.segment_add_t_coincident(o_seg, want, pt);
            }
        }
        for o_t in &opp_ts {
            let part = (o_t - o_t0) / o_range;
            let want = t0 + range * part;
            if !coin_ts.iter().any(|t| (t - want).abs() < T_MATCH) {
                let pt = arena.segment_pt_at_t(seg, want);
                arena.segment_add_t_coincident(seg, want, pt);
            }
        }
        true
    }
}

/// Returns the t values of `segment`'s spans strictly inside `t0`..`t1`.
fn interior_ts(arena: &OpArena, segment: SegmentId, t0: f32, t1: f32) -> Vec<f32> {
    let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
    arena
        .segment_spans(segment)
        .into_iter()
        .map(|s| arena.span(s).f_t)
        .filter(|t| *t > lo && *t < hi)
        .collect()
}

/// How close two t values must be to count as the same split.
///
/// Larger than the arithmetic tolerance elsewhere on purpose: the whole point
/// of `addExpanded` is to avoid inserting a span a hair away from one that
/// already exists, which would leave the two runs mismatched anyway.
const T_MATCH: f32 = 1e-4;

/// Bounds a walk over a span run, so a malformed record reports failure
/// rather than spinning. The C++ carries the same kind of guard.
const MARK_SAFETY: i32 = 1_000_000;

/// Returns the sort rank of a verb, matching the C++ `SkPath::Verb` order.
fn verb_rank(verb: Verb) -> u8 {
    match verb {
        Verb::Move => 0,
        Verb::Line => 1,
        Verb::Quad => 2,
        Verb::Conic => 3,
        Verb::Cubic => 4,
        Verb::Close => 5,
    }
}

#[cfg(test)]
mod tests {
    use super::super::sk_op_span::SkOpPtT;
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

    /// Builds a segment and returns it with the PtT nodes at both ends.
    fn seg_with_ends(
        arena: &mut OpArena,
        a: Point,
        b: Point,
    ) -> (SegmentId, PtTId, PtTId) {
        let seg = arena.alloc_segment_with_curve(&[a, b], Verb::Line, 1.0);
        let head = arena.segment(seg).f_head.expect("head");
        let tail = arena.segment(seg).f_tail.expect("tail");
        let hp = arena.span_ptt(head).expect("head ptt");
        let tp = arena.span_ptt(tail).expect("tail ptt");
        (seg, hp, tp)
    }

    #[test]
    fn overlaps_finds_the_shared_range_in_either_order() {
        assert_eq!(SkOpCoincidence::overlaps(0.0, 0.6, 0.4, 1.0), Some((0.4, 0.6)));
        // The same two ranges written backwards give the same answer.
        assert_eq!(SkOpCoincidence::overlaps(0.6, 0.0, 1.0, 0.4), Some((0.4, 0.6)));
        // Touching at a point is not an overlap: a run needs width.
        assert_eq!(SkOpCoincidence::overlaps(0.0, 0.5, 0.5, 1.0), None);
        assert_eq!(SkOpCoincidence::overlaps(0.0, 0.2, 0.8, 1.0), None);
    }

    #[test]
    fn ordered_segments_is_a_consistent_orientation() {
        let mut arena = OpArena::new();
        let a = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let b = arena.alloc_segment_with_curve(
            &[Point::new(5.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        // Whatever the answer is, asking the other way must give the opposite:
        // the point of the ordering is that A-B and B-A agree on one form.
        assert_ne!(
            SkOpCoincidence::ordered_segments(&arena, a, b),
            SkOpCoincidence::ordered_segments(&arena, b, a)
        );
        // Lower coordinates sort first.
        assert!(SkOpCoincidence::ordered_segments(&arena, a, b));
    }

    #[test]
    fn a_line_sorts_before_a_curve() {
        let mut arena = OpArena::new();
        let line = arena.alloc_segment_with_curve(
            &[Point::new(9.0, 9.0), Point::new(10.0, 9.0)],
            Verb::Line,
            1.0,
        );
        let quad = arena.alloc_segment_with_curve(
            &[
                Point::new(0.0, 0.0),
                Point::new(1.0, 1.0),
                Point::new(2.0, 0.0),
            ],
            Verb::Quad,
            1.0,
        );
        // Verb wins over coordinates: the line is at (9,9) and still first.
        assert!(SkOpCoincidence::ordered_segments(&arena, line, quad));
        assert!(!SkOpCoincidence::ordered_segments(&arena, quad, line));
    }

    #[test]
    fn extend_widens_a_record_at_both_ends() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let (_, a0, a1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        let (_, b0, b1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        // Start from an interior run, then extend to the full one.
        let mid_a = arena.alloc_ptt(SkOpPtT::new(0.25, Point::new(2.5, 0.0), None));
        let mid_a2 = arena.alloc_ptt(SkOpPtT::new(0.75, Point::new(7.5, 0.0), None));
        let mid_b = arena.alloc_ptt(SkOpPtT::new(0.25, Point::new(2.5, 0.0), None));
        let mid_b2 = arena.alloc_ptt(SkOpPtT::new(0.75, Point::new(7.5, 0.0), None));
        let rec = coin.add_run(&mut arena, mid_a, mid_a2, mid_b, mid_b2, false);

        assert!(SkOpCoincidence::extend_record(
            &mut arena, rec, a0, a1, b0, b1
        ));
        assert_eq!(arena.coin(rec).f_coin_ptt_start, Some(a0));
        assert_eq!(arena.coin(rec).f_coin_ptt_end, Some(a1));
        // Extending to the same range again moves nothing.
        assert!(!SkOpCoincidence::extend_record(
            &mut arena, rec, a0, a1, b0, b1
        ));
    }

    #[test]
    fn add_or_extend_folds_an_overlapping_run_into_one_record() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let seg_a = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let seg_b = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 1.0), Point::new(10.0, 1.0)],
            Verb::Line,
            1.0,
        );
        // Two runs on the same pair of segments that overlap in the middle.
        let a0 = arena.segment_add_t(seg_a, 0.0, Point::new(0.0, 0.0)).expect("a0");
        let a6 = arena.segment_add_t(seg_a, 0.6, Point::new(6.0, 0.0)).expect("a6");
        let a4 = arena.segment_add_t(seg_a, 0.4, Point::new(4.0, 0.0)).expect("a4");
        let a1 = arena.segment_add_t(seg_a, 1.0, Point::new(10.0, 0.0)).expect("a1");
        let b0 = arena.segment_add_t(seg_b, 0.0, Point::new(0.0, 1.0)).expect("b0");
        let b6 = arena.segment_add_t(seg_b, 0.6, Point::new(6.0, 1.0)).expect("b6");
        let b4 = arena.segment_add_t(seg_b, 0.4, Point::new(4.0, 1.0)).expect("b4");
        let b1 = arena.segment_add_t(seg_b, 1.0, Point::new(10.0, 1.0)).expect("b1");

        coin.add_or_extend(&mut arena, a0, a6, b0, b6).expect("first");
        coin.add_or_extend(&mut arena, a4, a1, b4, b1).expect("second");
        assert_eq!(
            coin.count(&arena),
            1,
            "0..0.6 and 0.4..1 overlap, so they are one run"
        );
        let rec = coin.records(&arena)[0];
        assert_eq!(
            SkOpCoincidence::ptt_t(&arena, arena.coin(rec).f_coin_ptt_end),
            1.0,
            "and the record now reaches the far end"
        );
    }

    #[test]
    fn add_or_extend_keeps_disjoint_runs_apart() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let seg_a = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let seg_b = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 1.0), Point::new(10.0, 1.0)],
            Verb::Line,
            1.0,
        );
        let a0 = arena.segment_add_t(seg_a, 0.0, Point::new(0.0, 0.0)).expect("a0");
        let a2 = arena.segment_add_t(seg_a, 0.2, Point::new(2.0, 0.0)).expect("a2");
        let a8 = arena.segment_add_t(seg_a, 0.8, Point::new(8.0, 0.0)).expect("a8");
        let a1 = arena.segment_add_t(seg_a, 1.0, Point::new(10.0, 0.0)).expect("a1");
        let b0 = arena.segment_add_t(seg_b, 0.0, Point::new(0.0, 1.0)).expect("b0");
        let b2 = arena.segment_add_t(seg_b, 0.2, Point::new(2.0, 1.0)).expect("b2");
        let b8 = arena.segment_add_t(seg_b, 0.8, Point::new(8.0, 1.0)).expect("b8");
        let b1 = arena.segment_add_t(seg_b, 1.0, Point::new(10.0, 1.0)).expect("b1");

        coin.add_or_extend(&mut arena, a0, a2, b0, b2).expect("first");
        coin.add_or_extend(&mut arena, a8, a1, b8, b1).expect("second");
        assert_eq!(
            coin.count(&arena),
            2,
            "0..0.2 and 0.8..1 share nothing, so they stay separate"
        );
    }

    #[test]
    fn apply_folds_a_shared_edge_onto_one_side() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        // Two segments running the same way over the same line: the case
        // dedup_coincident hacks around in sk_path_ops_simplify.
        let (seg_a, a0, a1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        let (seg_b, b0, b1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        for seg in [seg_a, seg_b] {
            for span in arena.segment_spans(seg) {
                arena.span_mut(span).set_wind_value(1);
            }
        }
        coin.add_run(&mut arena, a0, a1, b0, b1, false);
        assert!(coin.apply(&mut arena));

        let a_head = arena.segment(seg_a).f_head.expect("head");
        let b_head = arena.segment(seg_b).f_head.expect("head");
        let (wa, wb) = (
            arena.span(a_head).wind_value(),
            arena.span(b_head).wind_value(),
        );
        assert_eq!(
            wa + wb,
            2,
            "the pair's total winding is conserved, got {wa} and {wb}"
        );
        assert!(
            wa == 0 || wb == 0,
            "one side must be zeroed so the edge is counted once, got {wa} and {wb}"
        );
        // And the zeroed side is finished, so the walker never emits it.
        let zeroed = if wa == 0 { a_head } else { b_head };
        assert!(arena.span(zeroed).done(), "the absorbed side is marked done");
    }

    #[test]
    fn apply_on_a_flipped_pair_subtracts_instead_of_adding() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        // The same edge traced in opposite directions: the two cancel.
        let (seg_a, a0, a1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        let (seg_b, b0, b1) = seg_with_ends(
            &mut arena,
            Point::new(10.0, 0.0),
            Point::new(0.0, 0.0),
        );
        for seg in [seg_a, seg_b] {
            for span in arena.segment_spans(seg) {
                arena.span_mut(span).set_wind_value(1);
            }
        }
        coin.add_run(&mut arena, a0, a1, b0, b1, true);
        assert!(coin.apply(&mut arena));

        let a_head = arena.segment(seg_a).f_head.expect("head");
        let b_tail = arena.segment(seg_b).f_tail.expect("tail");
        // A flipped pair walks the opposite run from its far end, so a's head
        // is paired with b's tail, not b's head. Those two cancel.
        assert_eq!(
            arena.span(a_head).wind_value(),
            0,
            "the coincident side is subtracted away"
        );
        assert_eq!(
            arena.span(b_tail).wind_value(),
            0,
            "and so is the span it was paired with"
        );
    }

    #[test]
    fn apply_with_nothing_recorded_succeeds() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        assert!(coin.apply(&mut arena));
        assert!(coin.mark(&mut arena));
    }

    #[test]
    fn mark_links_the_two_runs_ends_together() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let (seg_a, a0, a1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        let (seg_b, b0, b1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        coin.add_run(&mut arena, a0, a1, b0, b1, false);
        assert!(coin.mark(&mut arena));

        let a_head = arena.segment(seg_a).f_head.expect("head");
        let b_head = arena.segment(seg_b).f_head.expect("head");
        assert!(
            arena.span_is_coincident(a_head),
            "the run's start carries its partner"
        );
        assert!(arena.span_contains_coincidence(a_head, b_head));
    }

    #[test]
    fn record_contains_answers_for_whichever_segment_is_asked() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let seg_a = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let seg_b = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 1.0), Point::new(10.0, 1.0)],
            Verb::Line,
            1.0,
        );
        let a0 = arena.segment_add_t(seg_a, 0.0, Point::new(0.0, 0.0)).expect("a0");
        let a8 = arena.segment_add_t(seg_a, 0.8, Point::new(8.0, 0.0)).expect("a8");
        let a4 = arena.segment_add_t(seg_a, 0.4, Point::new(4.0, 0.0)).expect("a4");
        let a9 = arena.segment_add_t(seg_a, 0.9, Point::new(9.0, 0.0)).expect("a9");
        let b0 = arena.segment_add_t(seg_b, 0.0, Point::new(0.0, 1.0)).expect("b0");
        let b8 = arena.segment_add_t(seg_b, 0.8, Point::new(8.0, 1.0)).expect("b8");
        let b4 = arena.segment_add_t(seg_b, 0.4, Point::new(4.0, 1.0)).expect("b4");

        let rec = coin.add_run(&mut arena, a0, a8, b0, b8, false);
        // Inside, on the coincident side.
        assert!(SkOpCoincidence::record_contains(&arena, rec, a0, a4));
        // Inside, on the opposite side: the other pair of endpoints is used.
        assert!(SkOpCoincidence::record_contains(&arena, rec, b0, b4));
        // Past the end.
        assert!(!SkOpCoincidence::record_contains(&arena, rec, a4, a9));
    }

    #[test]
    fn fix_up_repoints_a_moved_endpoint() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let (_, a0, a1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        let (_, b0, b1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        let rec = coin.add_run(&mut arena, a0, a1, b0, b1, false);
        // b0 is retired in favour of a0, which sits on a different span.
        coin.fix_up(&mut arena, b0, a0);
        assert_eq!(coin.count(&arena), 1, "the record survives");
        assert_eq!(arena.coin(rec).f_opp_ptt_start, Some(a0));
    }

    #[test]
    fn fix_up_drops_a_record_that_would_collapse() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let (_, a0, a1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        let (_, b0, b1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        coin.add_run(&mut arena, a0, a1, b0, b1, false);
        // Retiring a0 in favour of a1 would put both coincident ends on one
        // span, which describes no run at all.
        coin.fix_up(&mut arena, a0, a1);
        assert_eq!(coin.count(&arena), 0, "a zero-length run is released");
    }

    #[test]
    fn release_deleted_unlinks_a_retired_record() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let (_, a0, a1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
        );
        let (_, b0, b1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 1.0),
            Point::new(10.0, 1.0),
        );
        let (_, c0, c1) = seg_with_ends(
            &mut arena,
            Point::new(0.0, 2.0),
            Point::new(10.0, 2.0),
        );
        coin.add_run(&mut arena, a0, a1, b0, b1, false);
        let kept = coin.add_run(&mut arena, c0, c1, b0, b1, false);
        assert_eq!(coin.count(&arena), 2);

        arena.ptt_mut(a0).set_deleted(true);
        coin.release_deleted(&mut arena);
        assert_eq!(coin.count(&arena), 1);
        assert_eq!(coin.records(&arena), vec![kept]);
    }

    #[test]
    fn expand_keeps_two_records_that_only_share_a_start() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        // Two runs beginning at the same place on the same segment but
        // ending elsewhere. They are not the same coincidence, and dropping
        // one loses a real run - the segment it covered then keeps a winding
        // it should have given up.
        let seg_a = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let seg_b = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 1.0), Point::new(10.0, 1.0)],
            Verb::Line,
            1.0,
        );
        let seg_c = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 2.0), Point::new(10.0, 2.0)],
            Verb::Line,
            1.0,
        );
        let a0 = arena.segment_add_t(seg_a, 0.0, Point::new(0.0, 0.0)).expect("a0");
        let a3 = arena.segment_add_t(seg_a, 0.3, Point::new(3.0, 0.0)).expect("a3");
        let a7 = arena.segment_add_t(seg_a, 0.7, Point::new(7.0, 0.0)).expect("a7");
        let b0 = arena.segment_add_t(seg_b, 0.0, Point::new(0.0, 1.0)).expect("b0");
        let b3 = arena.segment_add_t(seg_b, 0.3, Point::new(3.0, 1.0)).expect("b3");
        let c0 = arena.segment_add_t(seg_c, 0.0, Point::new(0.0, 2.0)).expect("c0");
        let c7 = arena.segment_add_t(seg_c, 0.7, Point::new(7.0, 2.0)).expect("c7");

        coin.add_run(&mut arena, a0, a3, b0, b3, false);
        coin.add_run(&mut arena, a0, a7, c0, c7, false);
        assert_eq!(coin.count(&arena), 2);
        coin.expand(&mut arena);
        assert_eq!(
            coin.count(&arena),
            2,
            "same start, different runs: both are real"
        );
    }

    #[test]
    fn expand_scans_for_duplicates_only_where_something_moved() {
        let mut arena = OpArena::new();
        let mut coin = SkOpCoincidence::new();
        let seg_a = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 0.0), Point::new(10.0, 0.0)],
            Verb::Line,
            1.0,
        );
        let seg_b = arena.alloc_segment_with_curve(
            &[Point::new(0.0, 1.0), Point::new(10.0, 1.0)],
            Verb::Line,
            1.0,
        );
        let a0 = arena.segment_add_t(seg_a, 0.0, Point::new(0.0, 0.0)).expect("a0");
        let a5 = arena.segment_add_t(seg_a, 0.5, Point::new(5.0, 0.0)).expect("a5");
        let b0 = arena.segment_add_t(seg_b, 0.0, Point::new(0.0, 1.0)).expect("b0");
        let b5 = arena.segment_add_t(seg_b, 0.5, Point::new(5.0, 1.0)).expect("b5");
        // Two records describing exactly the same run, neither of which can
        // be widened - their ends are already the extremes.
        coin.add_run(&mut arena, a0, a5, b0, b5, false);
        coin.add_run(&mut arena, a0, a5, b0, b5, false);
        assert_eq!(coin.count(&arena), 2);
        assert!(
            !coin.expand(&mut arena),
            "nothing moved, so expand reports no change"
        );
        // And the duplicate scan does not run. C++ guards it the same way
        // (`if (coin->expand())`): duplicates are what *widening* creates,
        // and a pair that arrived identical is the caller's business.
        assert_eq!(coin.count(&arena), 2);
    }
}
