//! Debug utilities for path operations
//!
//! Port of Skia's SkPathOpsDebug.h/cpp

use crate::core::Point;

/// Debug flags for controlling verbose output
pub struct DebugFlags {
    /// Keep running the op after an internal consistency failure instead of
    /// bailing out, so the resulting damage can be inspected.
    pub g_run_fail: bool,
    /// Emit the per-span and per-angle trace output, not just the summary.
    pub g_very_verbose: bool,
}

impl DebugFlags {
    /// Create a flag set with all debug output disabled.
    pub fn new() -> Self {
        Self {
            g_run_fail: false,
            g_very_verbose: false,
        }
    }
}

impl Default for DebugFlags {
    fn default() -> Self {
        Self::new()
    }
}

// Global debug state (thread-local).
thread_local! {
    static DEBUG_FLAGS: DebugFlags = DebugFlags::new();
}

/// Glitch types for debugging coincidence operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GlitchType {
    /// No glitch recorded yet; the default state of a freshly added dictionary
    /// entry, which a later add may still fill in.
    Uninitialized,
    /// A coincident run was added whose endpoints do not agree with the spans
    /// they claim to join.
    AddCorruptCoin,
    /// An existing coincident run had to be widened to take in a neighbouring
    /// span.
    AddExpandedCoin,
    /// Widening a coincident run failed, usually because the extended range no
    /// longer matches on the opposite segment.
    AddExpandedFail,
    /// A coincident run was added over a range that has collapsed to a single
    /// point.
    AddIfCollapsed,
    /// A coincident run was inferred and added because the intersection pass
    /// had not produced one.
    AddIfMissingCoin,
    /// A coincidence that should already have been recorded was added after the
    /// fact.
    AddMissingCoin,
    /// A missing coincidence was repaired by extending an adjacent run rather
    /// than creating a new one.
    AddMissingExtend,
    /// A new coincident run either had to be added or merged into an existing
    /// overlapping one.
    AddOrOverlap,
    /// A coincident run whose t range shrank to nothing.
    CollapsedCoin,
    /// A span was marked done as a result of collapsing to zero length.
    CollapsedDone,
    /// The opposite-path winding value of a collapsed span had to be corrected.
    CollapsedOppValue,
    /// A span collapsed so that its start and end t values coincide.
    CollapsedSpan,
    /// The winding value of a collapsed span had to be corrected.
    CollapsedWindValue,
    /// The end of a span was moved to agree with the point its neighbour
    /// reports.
    CorrectEnd,
    /// A coincident run was removed, typically after collapsing or after being
    /// absorbed by another run.
    DeletedCoin,
    /// A coincident run was grown to cover spans adjacent to its current range.
    ExpandCoin,
    /// A generic unrecoverable inconsistency; the op gives up at this point.
    Fail,
    /// The end span of a coincident run was marked as coincident.
    MarkCoinEnd,
    /// A span had to be inserted at a t value so a coincident run could be
    /// marked there.
    MarkCoinInsert,
    /// A span that should have been marked coincident was not.
    MarkCoinMissing,
    /// The start span of a coincident run was marked as coincident.
    MarkCoinStart,
    /// Two span lists that describe the same point were merged.
    MergeMatches,
    /// A coincidence between two segments was detected that the coincidence
    /// list does not contain.
    MissingCoin,
    /// A span that should have been marked done was still left open.
    MissingDone,
    /// Two segments touch at a point for which no intersection was recorded.
    MissingIntersection,
    /// A span carrying several coincident references had to be moved as a
    /// group.
    MoveMultiple,
    /// The span's coincidence bits were cleared while moving it onto a nearby
    /// point.
    MoveNearbyClearAll,
    /// Second clear-all pass of the move-nearby walk, covering the spans found
    /// after the first pass.
    MoveNearbyClearAll2,
    /// Two spans closer together than the point tolerance were merged into one.
    MoveNearbyMerge,
    /// The last merge of the move-nearby pass, closing the loop back onto the
    /// head span.
    MoveNearbyMergeFinal,
    /// A span was released during move-nearby because another span already owns
    /// its point.
    MoveNearbyRelease,
    /// The final release of the move-nearby pass, on the span at the end of the
    /// list.
    MoveNearbyReleaseFinal,
    /// A span was released (removed from its segment) while checking health.
    ReleasedSpan,
    /// A routine reported failure to its caller; recorded so the return can be
    /// traced back to its origin.
    ReturnFalse,
    /// A span's stored point does not match the point its t value evaluates to.
    Unaligned,
    /// The head (start) span of a run is the unaligned one.
    UnalignedHead,
    /// The tail (end) span of a run is the unaligned one.
    UnalignedTail,
}

impl GlitchType {
    /// Convert to string representation
    pub fn as_str(&self) -> &'static str {
        match self {
            GlitchType::Uninitialized => "",
            GlitchType::AddCorruptCoin => "AddCorruptCoin",
            GlitchType::AddExpandedCoin => "AddExpandedCoin",
            GlitchType::AddExpandedFail => "AddExpandedFail",
            GlitchType::AddIfCollapsed => "AddIfCollapsed",
            GlitchType::AddIfMissingCoin => "AddIfMissingCoin",
            GlitchType::AddMissingCoin => "AddMissingCoin",
            GlitchType::AddMissingExtend => "AddMissingExtend",
            GlitchType::AddOrOverlap => "AddOrOverlap",
            GlitchType::CollapsedCoin => "CollapsedCoin",
            GlitchType::CollapsedDone => "CollapsedDone",
            GlitchType::CollapsedOppValue => "CollapsedOppValue",
            GlitchType::CollapsedSpan => "CollapsedSpan",
            GlitchType::CollapsedWindValue => "CollapsedWindValue",
            GlitchType::CorrectEnd => "CorrectEnd",
            GlitchType::DeletedCoin => "DeletedCoin",
            GlitchType::ExpandCoin => "ExpandCoin",
            GlitchType::Fail => "Fail",
            GlitchType::MarkCoinEnd => "MarkCoinEnd",
            GlitchType::MarkCoinInsert => "MarkCoinInsert",
            GlitchType::MarkCoinMissing => "MarkCoinMissing",
            GlitchType::MarkCoinStart => "MarkCoinStart",
            GlitchType::MergeMatches => "MergeMatches",
            GlitchType::MissingCoin => "MissingCoin",
            GlitchType::MissingDone => "MissingDone",
            GlitchType::MissingIntersection => "MissingIntersection",
            GlitchType::MoveMultiple => "MoveMultiple",
            GlitchType::MoveNearbyClearAll => "MoveNearbyClearAll",
            GlitchType::MoveNearbyClearAll2 => "MoveNearbyClearAll2",
            GlitchType::MoveNearbyMerge => "MoveNearbyMerge",
            GlitchType::MoveNearbyMergeFinal => "MoveNearbyMergeFinal",
            GlitchType::MoveNearbyRelease => "MoveNearbyRelease",
            GlitchType::MoveNearbyReleaseFinal => "MoveNearbyReleaseFinal",
            GlitchType::ReleasedSpan => "ReleasedSpan",
            GlitchType::ReturnFalse => "ReturnFalse",
            GlitchType::Unaligned => "Unaligned",
            GlitchType::UnalignedHead => "UnalignedHead",
            GlitchType::UnalignedTail => "UnalignedTail",
        }
    }
}

/// Entry in the coincidence dictionary
#[derive(Debug, Clone)]
pub struct CoinDictEntry {
    /// Which pass of the coincidence fix-up loop produced this entry; entries
    /// are keyed on this together with `line_number`.
    pub iteration: i32,
    /// Source line the entry was recorded from, standing in for the call site.
    pub line_number: i32,
    /// What went wrong at that call site, or `Uninitialized` if the site was
    /// merely visited.
    pub glitch_type: GlitchType,
    /// Name of the function the entry was recorded from.
    pub function_name: String,
}

impl CoinDictEntry {
    /// Record a visit to a call site with no glitch attached yet.
    pub fn new(line_no: i32, func_name: &str) -> Self {
        Self {
            iteration: 0,
            line_number: line_no,
            glitch_type: GlitchType::Uninitialized,
            function_name: func_name.to_string(),
        }
    }
}

/// Dictionary for tracking coincidence operations
#[derive(Debug, Default)]
pub struct CoinDict {
    entries: Vec<CoinDictEntry>,
}

impl CoinDict {
    /// Create an empty dictionary.
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    /// Record an entry, keyed on its iteration and line number. If that call
    /// site was already recorded this pass, the stored glitch type is only
    /// filled in when it is still uninitialized, so the first glitch seen at a
    /// site wins.
    pub fn add(&mut self, entry: CoinDictEntry) {
        // Check if entry with same iteration and line already exists
        if let Some(existing) = self
            .entries
            .iter_mut()
            .find(|e| e.iteration == entry.iteration && e.line_number == entry.line_number)
        {
            // Only set glitch type if uninitialized
            if existing.glitch_type == GlitchType::Uninitialized {
                existing.glitch_type = entry.glitch_type;
            }
        } else {
            self.entries.push(entry);
        }
    }

    /// Fold every entry of `other` into this dictionary under the same
    /// deduplication rule as [`CoinDict::add`].
    pub fn add_dict(&mut self, other: &CoinDict) {
        for entry in &other.entries {
            self.add(entry.clone());
        }
    }

    /// The recorded entries, in the order their call sites were first hit.
    pub fn entries(&self) -> &[CoinDictEntry] {
        &self.entries
    }

    /// Drop all entries, for instance between runs.
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

/// Global coin dictionaries
#[derive(Debug, Default)]
pub struct GlobalCoinDicts {
    /// Call sites that actually altered the coincidence data this run.
    pub changed: CoinDict,
    /// Every call site reached this run, whether or not it changed anything;
    /// comparing it against `changed` shows which paths are untested.
    pub visited: CoinDict,
}

/// Glitch record for logging debug issues
#[derive(Debug, Clone)]
pub struct SpanGlitch {
    /// Span the check started from, the one assumed to be correct.
    pub base_id: Option<i32>,
    /// Span that disagreed with the base and triggered the report.
    pub suspect_id: Option<i32>,
    /// Segment the spans belong to.
    pub segment_id: Option<i32>,
    /// Segment on the other side of the coincidence or intersection.
    pub opp_segment_id: Option<i32>,
    /// Span starting the coincident run under inspection.
    pub coin_span_id: Option<i32>,
    /// Span ending that run.
    pub end_span_id: Option<i32>,
    /// Span on the opposite segment matching `coin_span_id`.
    pub opp_span_id: Option<i32>,
    /// Span on the opposite segment matching `end_span_id`.
    pub opp_end_span_id: Option<i32>,
    /// Curve parameter where the run or span begins.
    pub start_t: Option<f64>,
    /// Curve parameter where it ends.
    pub end_t: Option<f64>,
    /// Corresponding start parameter on the opposite segment; it may run
    /// backwards relative to `start_t` when the two are reversed.
    pub opp_start_t: Option<f64>,
    /// Corresponding end parameter on the opposite segment.
    pub opp_end_t: Option<f64>,
    /// Point in question, for glitches about a position rather than a range.
    pub pt: Option<Point>,
    /// What kind of inconsistency this record describes.
    pub glitch_type: GlitchType,
}

impl SpanGlitch {
    /// Create a record of the given kind with no context fields filled in; the
    /// recording helpers on [`GlitchLog`] set the fields that apply.
    pub fn new(glitch_type: GlitchType) -> Self {
        Self {
            base_id: None,
            suspect_id: None,
            segment_id: None,
            opp_segment_id: None,
            coin_span_id: None,
            end_span_id: None,
            opp_span_id: None,
            opp_end_span_id: None,
            start_t: None,
            end_t: None,
            opp_start_t: None,
            opp_end_t: None,
            pt: None,
            glitch_type,
        }
    }
}

/// Log for recording glitches during debugging
#[derive(Debug, Default)]
pub struct GlitchLog {
    glitches: Vec<SpanGlitch>,
}

impl GlitchLog {
    /// Create an empty log.
    pub fn new() -> Self {
        Self {
            glitches: Vec::new(),
        }
    }

    /// Append a bare glitch and hand back a mutable reference so the caller can
    /// fill in whichever context fields it has.
    pub fn record(&mut self, glitch_type: GlitchType) -> &mut SpanGlitch {
        self.glitches.push(SpanGlitch::new(glitch_type));
        self.glitches.last_mut().unwrap()
    }

    /// Record a glitch against the span the check started from.
    pub fn record_with_base(&mut self, glitch_type: GlitchType, base_id: i32) -> &mut SpanGlitch {
        let glitch = self.record(glitch_type);
        glitch.base_id = Some(base_id);
        glitch
    }

    /// Record a glitch for a pair of spans that should have agreed: the base
    /// span and the suspect one that did not match it.
    pub fn record_with_span(
        &mut self,
        glitch_type: GlitchType,
        base_id: i32,
        suspect_id: i32,
    ) -> &mut SpanGlitch {
        let glitch = self.record(glitch_type);
        glitch.base_id = Some(base_id);
        glitch.suspect_id = Some(suspect_id);
        glitch
    }

    /// Record a glitch that concerns a whole segment rather than a particular
    /// span.
    pub fn record_with_segment(
        &mut self,
        glitch_type: GlitchType,
        segment_id: i32,
    ) -> &mut SpanGlitch {
        let glitch = self.record(glitch_type);
        glitch.segment_id = Some(segment_id);
        glitch
    }

    /// Record a glitch at a single location, given as the curve parameter and
    /// the point it was expected to land on.
    pub fn record_with_t_and_point(
        &mut self,
        glitch_type: GlitchType,
        t: f64,
        pt: Point,
    ) -> &mut SpanGlitch {
        let glitch = self.record(glitch_type);
        glitch.start_t = Some(t);
        glitch.pt = Some(pt);
        glitch
    }

    /// Record a glitch spanning a coincident run, identified by the spans at
    /// its two ends.
    pub fn record_with_coin_span(
        &mut self,
        glitch_type: GlitchType,
        coin_span_id: i32,
        end_span_id: i32,
    ) -> &mut SpanGlitch {
        let glitch = self.record(glitch_type);
        glitch.coin_span_id = Some(coin_span_id);
        glitch.end_span_id = Some(end_span_id);
        glitch
    }

    /// Record a glitch about a span and its counterpart on the other segment.
    ///
    /// Note that `span_id` lands in `end_span_id`, not in a same-named field.
    /// This helper has no single counterpart among the C++ `record` overloads
    /// and currently has no callers, so which field the first span belongs in
    /// is unsettled; it is left as-is rather than guessed at.
    pub fn record_with_opposing(
        &mut self,
        glitch_type: GlitchType,
        span_id: i32,
        opp_span_id: i32,
    ) -> &mut SpanGlitch {
        let glitch = self.record(glitch_type);
        glitch.end_span_id = Some(span_id);
        glitch.opp_span_id = Some(opp_span_id);
        glitch
    }

    /// Record a glitch describing both sides of a coincidence: the two segments
    /// and the t ranges on each. Every field is optional so callers can supply
    /// only what they know. The span id fields are left unset.
    pub fn record_full(
        &mut self,
        glitch_type: GlitchType,
        base_id: Option<i32>,
        segment_id: Option<i32>,
        opp_segment_id: Option<i32>,
        start_t: Option<f64>,
        end_t: Option<f64>,
        opp_start_t: Option<f64>,
        opp_end_t: Option<f64>,
        pt: Option<Point>,
    ) -> &mut SpanGlitch {
        let glitch = self.record(glitch_type);
        glitch.base_id = base_id;
        glitch.segment_id = segment_id;
        glitch.opp_segment_id = opp_segment_id;
        glitch.start_t = start_t;
        glitch.end_t = end_t;
        glitch.opp_start_t = opp_start_t;
        glitch.opp_end_t = opp_end_t;
        glitch.pt = pt;
        glitch
    }

    /// How many glitches have been recorded; zero means the run was clean.
    pub fn count(&self) -> usize {
        self.glitches.len()
    }

    /// Fetch one glitch by its position in the log, or `None` if out of range.
    pub fn get(&self, index: usize) -> Option<&SpanGlitch> {
        self.glitches.get(index)
    }

    /// Walk the glitches in the order they were recorded.
    pub fn iter(&self) -> impl Iterator<Item = &SpanGlitch> {
        self.glitches.iter()
    }

    /// Discard everything recorded so far, for instance before a new pass.
    pub fn clear(&mut self) {
        self.glitches.clear();
    }
}

/// Debug utilities for path operations
pub struct SkPathOpsDebug;

impl SkPathOpsDebug {
    /// Return string representation of path operation
    pub fn op_str(op: super::PathOp) -> &'static str {
        match op {
            super::PathOp::Difference => "diff",
            super::PathOp::Intersect => "sect",
            super::PathOp::Union => "union",
            super::PathOp::Xor => "xor",
            super::PathOp::ReverseDifference => "rdiff",
        }
    }

    /// Convert scientific notation to Mathematica format (e.g., 1e5 -> 1*^5)
    pub fn mathematicaize(s: &mut String) {
        let mut chars: Vec<char> = s.chars().collect();
        let mut num = false;

        let mut idx = 0;
        while idx < chars.len() {
            if num && chars[idx] == 'e' {
                // Replace the exponent marker `e` with Mathematica's `*^`.
                chars[idx] = '*';
                chars.insert(idx + 1, '^');
                idx += 2;
                num = false;
                continue;
            }
            num = chars[idx].is_ascii_digit();
            idx += 1;
        }

        *s = chars.into_iter().collect();
    }

    /// Check if winding value is valid (not at extreme limits)
    pub fn valid_wind(winding: i32) -> bool {
        winding > i32::MIN + 0xFFFF && winding < i32::MAX - 0xFFFF
    }

    /// Print winding value, showing '?' if invalid
    pub fn winding_printf(winding: i32) -> String {
        if winding == i32::MIN {
            "?".to_string()
        } else {
            winding.to_string()
        }
    }

    /// Check if any span in array contains the target span (debug utility)
    #[cfg(debug_assertions)]
    pub fn chase_contains<T: PartialEq>(chase_array: &[&T], span: &T) -> bool {
        chase_array.iter().any(|entry| **entry == *span)
    }

    /// Run comprehensive health check on contours (debug only)
    #[cfg(debug_assertions)]
    pub fn check_health() {
        // This would call contour health checks in a full implementation
        // For now, just a placeholder
    }

    /// Show active spans for debugging (debug only)
    #[cfg(debug_assertions)]
    pub fn show_active_spans() {
        // Placeholder - would traverse contours and print span states
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pathops::PathOp;

    #[test]
    fn test_op_str() {
        assert_eq!(SkPathOpsDebug::op_str(PathOp::Union), "union");
        assert_eq!(SkPathOpsDebug::op_str(PathOp::Difference), "diff");
        assert_eq!(SkPathOpsDebug::op_str(PathOp::Intersect), "sect");
        assert_eq!(SkPathOpsDebug::op_str(PathOp::Xor), "xor");
        assert_eq!(SkPathOpsDebug::op_str(PathOp::ReverseDifference), "rdiff");
    }

    #[test]
    fn test_mathematicaize() {
        let mut s = String::from("1e5");
        SkPathOpsDebug::mathematicaize(&mut s);
        assert_eq!(s, "1*^5");

        let mut s = String::from("1.5e-3");
        SkPathOpsDebug::mathematicaize(&mut s);
        assert_eq!(s, "1.5*^-3");
    }

    #[test]
    fn test_valid_wind() {
        assert!(SkPathOpsDebug::valid_wind(1000));
        assert!(SkPathOpsDebug::valid_wind(0));
        assert!(!SkPathOpsDebug::valid_wind(i32::MIN + 100));
        assert!(!SkPathOpsDebug::valid_wind(i32::MAX - 100));
    }

    #[test]
    fn test_winding_printf() {
        assert_eq!(SkPathOpsDebug::winding_printf(1000), "1000");
        assert_eq!(SkPathOpsDebug::winding_printf(i32::MIN), "?");
    }

    #[test]
    fn test_glitch_type_as_str() {
        assert_eq!(GlitchType::Fail.as_str(), "Fail");
        assert_eq!(GlitchType::MissingCoin.as_str(), "MissingCoin");
        assert_eq!(GlitchType::Uninitialized.as_str(), "");
    }

    #[test]
    fn test_coin_dict() {
        // Entries dedup on the (iteration, line_number) pair, matching
        // the original SkPathOpsDebug::CoinDict::add — both fields must
        // match for the second add() to merge into the first.
        let mut dict = CoinDict::new();
        let entry1 = CoinDictEntry::new(100, "test_func");
        let entry2 = CoinDictEntry {
            iteration: 0,
            line_number: 100,
            glitch_type: GlitchType::Fail,
            function_name: "test_func".to_string(),
        };

        dict.add(entry1);
        dict.add(entry2);

        // Should only have one entry with updated glitch type
        assert_eq!(dict.entries().len(), 1);
        assert_eq!(dict.entries()[0].glitch_type, GlitchType::Fail);
    }

    #[test]
    fn test_glitch_log() {
        let mut log = GlitchLog::new();
        let glitch = log.record(GlitchType::Fail);
        assert_eq!(glitch.glitch_type, GlitchType::Fail);
        assert_eq!(log.count(), 1);
    }

    #[test]
    fn test_span_glitch_record() {
        let mut log = GlitchLog::new();
        let glitch = log.record_with_base(GlitchType::MissingCoin, 42);
        assert_eq!(glitch.base_id, Some(42));
        assert_eq!(log.count(), 1);
    }

    #[test]
    fn test_chase_contains() {
        let span1 = 1;
        let span2 = 2;

        let array = vec![&span1, &span2];
        assert!(SkPathOpsDebug::chase_contains(&array, &span1));
        assert!(SkPathOpsDebug::chase_contains(&array, &span2));
        assert!(!SkPathOpsDebug::chase_contains(&array, &3));
    }
}
