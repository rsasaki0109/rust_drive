use serde::{Deserialize, Serialize};

/// Recent supported velocity acquisitions in the continuous tracking frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VelocityHypothesisTrack {
    pub id: u64,
    pub sample_count: usize,
    pub first_acquisition_s: Option<f64>,
    pub last_acquisition_s: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VelocityHypothesisDiagnostics {
    pub window_s: f64,
    pub maximum_samples_per_identity: usize,
    pub tracks: Vec<VelocityHypothesisTrack>,
}
