//! Measured-proxy forecast checks in continuous wheel/gyro coordinates.
use crate::Vec2;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ForecastConsistencyState {
    #[default]
    Observing,
    Holding,
    Confirming,
    ResetRequired,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ForecastConsistencyFault {
    MeasuredMiss,
    InvalidInput,
    Capacity,
    MotionDiscontinuity,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ForecastConsistencyMiss {
    pub id: u64,
    pub reference_origin_s: f64,
    pub reference_acquisition_s: f64,
    pub measurement_acquisition_s: f64,
    pub predicted_center: Vec2,
    pub predicted_radius_m: f64,
    pub measured_center: Vec2,
    pub measured_radius_m: f64,
    pub shortfall_m: f64,
}

/// Counts and witnesses describe the latest evaluated acquisition, retained
/// through cached control calls. Scan, evaluation and recovery clocks differ.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ForecastConsistencyDiagnostics {
    pub state: ForecastConsistencyState,
    pub fault: Option<ForecastConsistencyFault>,
    pub last_scan_acquisition_s: Option<f64>,
    pub evaluated_acquisition_s: Option<f64>,
    pub evaluated_at_control_s: Option<f64>,
    pub fault_control_time_s: Option<f64>,
    pub recovery_after_control_s: Option<f64>,
    pub checked_measurements: usize,
    pub unmonitored_measurements: usize,
    pub missed_measurements: usize,
    pub complete_scene_check: bool,
    pub recovery_samples: u8,
    pub recovery_first_acquisition_s: Option<f64>,
    pub held_identities: Vec<u64>,
    pub misses: Vec<ForecastConsistencyMiss>,
    pub retained_references: usize,
}
