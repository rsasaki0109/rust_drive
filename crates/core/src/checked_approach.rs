//! Acquisition-owned confirmation of a newly checked stopping approach.
use crate::Vec2;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CheckedApproachDiagnostics {
    pub supporting_acquisitions: u8,
    pub first_acquisition_s: Option<f64>,
    pub last_acquisition_s: Option<f64>,
    pub proposal_end_progress_m: Option<f64>,
    pub proposal_endpoint: Option<Vec2>,
    pub proposal_lateral_target_m: Option<f64>,
    pub blocking_prediction_ids: Vec<u64>,
    /// True only on the fresh acquisition that replaces the retained stop.
    pub adopted: bool,
}
