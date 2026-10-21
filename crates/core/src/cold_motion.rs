use serde::{Deserialize, Serialize};

/// Acquisition evidence in the continuous tracking frame; SI units.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ColdMotionTrack {
    pub id: u64,
    pub confirmed: bool,
    pub first_acquisition_s: Option<f64>,
    pub last_acquisition_s: Option<f64>,
    pub acquisition_count: usize,
    pub last_observed_acquisition_s: Option<f64>,
    pub confirmation_acquisition_s: Option<f64>,
    pub velocity_residual_m_s: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ColdMotionDiagnostics {
    pub maximum_speed_m_s: f64,
    pub tracks: Vec<ColdMotionTrack>,
}
