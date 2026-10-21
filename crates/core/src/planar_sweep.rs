//! Acquired planar ray outcomes, distinct from an unannotated empty point cloud.
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanarLidarCalibration {
    pub ray_count: usize,
    pub first_angle_rad: f64,
    pub angular_step_rad: f64,
    pub min_range_m: f64,
    pub max_range_m: f64,
    pub range_error_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum PlanarRayOutcome {
    /// The commanded ray completed without a surface return within range.
    Clear,
    /// Index into the acquisition's original body-frame measured points.
    Return(usize),
    /// Missing/invalid query, dropped return or incomplete acquisition.
    Unavailable,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanarLidarSweep {
    pub calibration: PlanarLidarCalibration,
    pub rays: Vec<PlanarRayOutcome>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClearSweepRecoveryConfig {
    pub lidar: PlanarLidarCalibration,
    pub checked_center_radius_m: f64,
    pub minimum_resolved_disc_radius_m: f64,
}

/// Conditional resolution evidence from a validated acquired empty sweep.
/// It does not calibrate physical shape, detection probability or future motion.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClearSweepEvidence {
    pub acquisition_s: f64,
    pub checked_center_radius_m: f64,
    pub minimum_resolved_disc_radius_m: f64,
    pub maximum_angular_gap_rad: f64,
    pub maximum_transverse_gap_m: f64,
    pub ray_count: usize,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClearSweepRelease {
    pub identities: Vec<u64>,
    pub control_time_s: f64,
    pub first_acquisition_s: f64,
    pub last_acquisition_s: f64,
    pub acquisition_count: u8,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClearSweepRecoveryDiagnostics {
    pub last_evidence: Option<ClearSweepEvidence>,
    pub confirmation_samples: u8,
    pub first_acquisition_s: Option<f64>,
    pub last_acquisition_s: Option<f64>,
    pub release_count: u64,
    pub last_release: Option<ClearSweepRelease>,
}
