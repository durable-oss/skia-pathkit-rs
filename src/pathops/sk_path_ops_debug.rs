//! Debug utilities for path operations
//!
//! Port of Skia's SkPathOpsDebug.h/cpp

use crate::core::{Point};

/// Debug flags for controlling verbose output
pub struct DebugFlags {
    pub g_run_fail: bool,
    pub g_very_verbose: bool,
}

impl DebugFlags {
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

/// Global debug state (thread-local)
thread_local! {
    static DEBUG_FLAGS: DebugFlags = DebugFlags::new();
}

/// Glitch types for debugging coincidence operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GlitchType {
    Uninitialized,
    AddCorruptCoin,
    AddExpandedCoin,
    AddExpandedFail,
    AddIfCollapsed,
    AddIfMissingCoin,
    AddMissingCoin,
    AddMissingExtend,
    AddOrOverlap,
    CollapsedCoin,
    CollapsedDone,
    CollapsedOppValue,
    CollapsedSpan,
    CollapsedWindValue,
    CorrectEnd,
    DeletedCoin,
    ExpandCoin,
    Fail,
    MarkCoinEnd,
    MarkCoinInsert,
    MarkCoinMissing,
    MarkCoinStart,
    MergeMatches,
    MissingCoin,
    MissingDone,
    MissingIntersection,
    MoveMultiple,
    MoveNearbyClearAll,
    MoveNearbyClearAll2,
    MoveNearbyMerge,
    MoveNearbyMergeFinal,
    MoveNearbyRelease,
    MoveNearbyReleaseFinal,
    ReleasedSpan,
    ReturnFalse,
    Unaligned,
    UnalignedHead,
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
    pub iteration: i32,
    pub line_number: i32,
    pub glitch_type: GlitchType,
    pub function_name: String,
}

impl CoinDictEntry {
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
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    pub fn add(&mut self, entry: CoinDictEntry) {
        // Check if entry with same iteration and line already exists
        if let Some(existing) = self.entries.iter_mut().find(|e| {
            e.iteration == entry.iteration && e.line_number == entry.line_number
        }) {
            // Only set glitch type if uninitialized
            if existing.glitch_type == GlitchType::Uninitialized {
                existing.glitch_type = entry.glitch_type;
            }
        } else {
            self.entries.push(entry);
        }
    }

    pub fn add_dict(&mut self, other: &CoinDict) {
        for entry in &other.entries {
            self.add(entry.clone());
        }
    }

    pub fn entries(&self) -> &[CoinDictEntry] {
        &self.entries
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

/// Global coin dictionaries
#[derive(Debug, Default)]
pub struct GlobalCoinDicts {
    pub changed: CoinDict,
    pub visited: CoinDict,
}

/// Glitch record for logging debug issues
#[derive(Debug, Clone)]
pub struct SpanGlitch {
    pub base_id: Option<i32>,
    pub suspect_id: Option<i32>,
    pub segment_id: Option<i32>,
    pub opp_segment_id: Option<i32>,
    pub coin_span_id: Option<i32>,
    pub end_span_id: Option<i32>,
    pub opp_span_id: Option<i32>,
    pub opp_end_span_id: Option<i32>,
    pub start_t: Option<f64>,
    pub end_t: Option<f64>,
    pub opp_start_t: Option<f64>,
    pub opp_end_t: Option<f64>,
    pub pt: Option<Point>,
    pub glitch_type: GlitchType,
}

impl SpanGlitch {
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
    pub fn new() -> Self {
        Self {
            glitches: Vec::new(),
        }
    }

    pub fn record(&mut self, glitch_type: GlitchType) -> &mut SpanGlitch {
        self.glitches.push(SpanGlitch::new(glitch_type));
        self.glitches.last_mut().unwrap()
    }

    pub fn record_with_base(
        &mut self,
        glitch_type: GlitchType,
        base_id: i32,
    ) -> &mut SpanGlitch {
        let glitch = self.record(glitch_type);
        glitch.base_id = Some(base_id);
        glitch
    }

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

    pub fn record_with_segment(
        &mut self,
        glitch_type: GlitchType,
        segment_id: i32,
    ) -> &mut SpanGlitch {
        let glitch = self.record(glitch_type);
        glitch.segment_id = Some(segment_id);
        glitch
    }

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

    pub fn count(&self) -> usize {
        self.glitches.len()
    }

    pub fn get(&self, index: usize) -> Option<&SpanGlitch> {
        self.glitches.get(index)
    }

    pub fn iter(&self) -> impl Iterator<Item = &SpanGlitch> {
        self.glitches.iter()
    }

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
        assert_eq!(
            SkPathOpsDebug::op_str(PathOp::ReverseDifference),
            "rdiff"
        );
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
