//! Bounded planner decision records. Times are relative to the current plan;
//! progress uses the supplied active route, including any rule-stop prefix.
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlanningFailure {
    InvalidInput,
    InvalidLocalGeometry,
    NoSelectableCandidate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CandidateDisposition {
    NoSideRoom,
    OutsideTargetCorridor,
    OppositeSideDuringPass,
    GeometryOutsideCorridor,
    UnreachableSpeedProfile,
    RefinementForecastConflict,
    NoCheckedStop,
    CompletePath,
    CheckedStop,
}

/// First conservative swept conflict on the nominal path, before stop retiming.
/// A chord/radius bound can reject a path before its actual centers touch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanningForecastConflict {
    pub prediction_id: u64,
    pub relative_time_s: f64,
    pub forecast_radius_m: f64,
    pub forecast_horizon_s: f64,
    /// Zero retains the original time/motion reserve; a positive floor never
    /// replaces a larger original reserve.
    pub minimum_clearance_floor_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanningProfileSummary {
    pub point_count: usize,
    pub end_progress_m: f64,
    pub end_time_s: f64,
    pub terminal_speed_m_s: f64,
    pub searched_stop: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanningCandidateDiagnostic {
    pub index: u8,
    pub lateral_target_m: f64,
    pub disposition: CandidateDisposition,
    pub nominal_forecast_conflict: Option<PlanningForecastConflict>,
    pub profile: Option<PlanningProfileSummary>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RefinementConfirmationDiagnostic {
    pub lateral_target_m: f64,
    pub acquisition_count: u8,
    pub first_acquisition_s: f64,
    pub last_acquisition_s: f64,
    pub confirmed: bool,
}

/// Describes the planner proposal; pipeline faults/rules can override its mode
/// or command. It does not prove feasibility beyond the enumerated candidates.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanningDiagnostics {
    pub failure: Option<PlanningFailure>,
    pub scene_acquisition_s: Option<f64>,
    pub route_progress_m: Option<f64>,
    /// Proposed span before stop retiming; optional local fitting can use a
    /// different internal arc from the supplied route's published progress.
    pub planning_span_m: Option<f64>,
    pub selected_candidate: Option<u8>,
    /// At most nine records, in candidate evaluation order.
    pub candidates: Vec<PlanningCandidateDiagnostic>,
    pub refinement_confirmation: Option<RefinementConfirmationDiagnostic>,
    pub obstruction_stop_requested: bool,
    pub obstruction_stop_active: bool,
}
