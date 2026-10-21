//! Shared sensor-to-command stack. No simulator, truth objects or physical world types.
mod gnss_recovery;
mod ground;
mod heading;
use gnss_recovery::Recovery;
pub use gnss_recovery::{GnssRecoveryConfig, GnssRecoveryDiagnostics, RecoveryPhase};
pub mod lane_change;
use lane_change::{LaneChangeConfig, LaneChangeGate, LaneChangeStatus};
pub mod intersections;
pub use ground::{GroundConfig, GroundDiagnostics, GroundPlane};
mod lidar3d;
pub use lidar3d::Lidar3dConfig;
mod multi_height;
pub use multi_height::MultiHeightLidarConfig;
mod map_localization;
use map_localization::MapLocalization;
pub use map_localization::{
    MapLocalizationConfig, MapLocalizationDecision, MapLocalizationDiagnostics,
};
mod perception_3d;
pub use perception_3d::{
    AdaptiveTerrainSupportDiagnostics, MeasuredObject3d, Perception3dConfig,
    Perception3dDiagnostics, TerrainSupportDiagnostics,
};
pub mod navigation;
mod planning_progress;
pub use planning_progress::{PlanningProgress, PlanningProgressState};
mod relocalization;
pub mod replay;
mod scan_pose;
use relocalization::Relocalizer;
pub use relocalization::{
    MapOnlySupportDiagnostics, MapOnlySupportStatus, RegionalSearchDiagnostics,
    RelocalizationCandidate, RelocalizationConfig, RelocalizationDiagnostics, RelocalizationPhase,
    RelocalizationSeed,
};
pub mod stop_signs;
mod tracking_frame;
pub mod traffic_controls;
use intersections::{IntersectionStatus, YieldIntersection, YieldIntersections};
use navigation::{
    NavigationConfig, NavigationPhase, NavigationStatus, NavigationUpdate, Navigator,
};
use rustdriving_control::{PurePursuit, guard};
use rustdriving_core::*;
use rustdriving_localization::Ekf;
use rustdriving_localization::latency::{GnssHistory, HistoryError};
use rustdriving_mapping::OccupancyGrid;
use rustdriving_perception::{LidarClusters, Tracker};
use rustdriving_planning::LatticePlanner;
use rustdriving_prediction::{
    ForecastConsistencyAdmission, ForecastConsistencyGuard, ObservedBraking,
};
use serde::{Deserialize, Serialize};
use stop_signs::{StopSignStatus, StopSigns};
use traffic_controls::{
    SignalObservation, StopLine, TrafficControlStatus, TrafficControls, validate_stop_lines,
};

/// Conservative, externally calibrated planning limits, independent of simulator truth.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotionLimits {
    #[serde(default = "default_acceleration")]
    pub max_acceleration_m_s2: f64,
    pub max_deceleration_m_s2: f64,
    pub max_lateral_acceleration_m_s2: f64,
}
fn is_false(value: &bool) -> bool {
    !*value
}
fn default_acceleration() -> f64 {
    2.0
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PipelineConfig {
    pub route: Route,
    pub initial_pose: Pose,
    /// Externally supplied initial ENU x/y/yaw calibration covariance.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub initial_pose_covariance: Option<[[f64; 3]; 3]>,
    /// Optional dimensionless ENU GNSS covariance shape (symmetric SPD, trace 2).
    /// Each fix retains its scalar mean variance; R = variance * shape.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_covariance_shape: Option<[[f64; 2]; 2]>,
    /// GNSS antenna displacement from estimated pose origin in body forward/left meters.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_antenna_offset_m: Option<Vec2>,
    /// Explicit bounded-proposal confirmation after an innovation brake hold.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_recovery: Option<GnssRecoveryConfig>,
    /// Require externally measured ENU body heading (synchronous by default).
    #[serde(default, skip_serializing_if = "is_false")]
    pub absolute_heading_fusion: bool,
    /// Permit at most 0.2 s heading delivery delay with measured-motion history.
    #[serde(default, skip_serializing_if = "is_false")]
    pub heading_latency_compensation: bool,
    pub vehicle: VehicleConfig,
    pub nominal_dt: f64,
    pub cruise_speed: f64,
    /// Surveyed intervals on the configured fixed route; not observed actor data.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub speed_zones: Vec<RouteSpeedZone>,
    /// Experimental history-confirmed slow-object forecasts; default off.
    #[serde(default, skip_serializing_if = "is_false")]
    pub confirm_slow_motion: bool,
    /// Confirm cold-track motion from stable acquired detections; default off.
    #[serde(default, skip_serializing_if = "is_false")]
    pub confirm_track_initialization: bool,
    /// Refit current track motion from a bounded recent acquisition window.
    #[serde(default, skip_serializing_if = "is_false")]
    pub fit_recent_track_motion: bool,
    /// Experimental circular hypotheses from three/four planar returns.
    #[serde(default, skip_serializing_if = "is_false")]
    pub fit_sparse_circles: bool,
    /// Preserve each newest measured disc at the corrected track center.
    #[serde(default, skip_serializing_if = "is_false")]
    pub enclose_track_measurements: bool,
    /// Optional declared motion-error budgets; does not estimate uncertainty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub forecast_envelope: Option<ForecastEnvelopeConfig>,
    /// Brake after a prior forecast misses a newly acquired measured disc.
    #[serde(default, skip_serializing_if = "is_false")]
    pub forecast_consistency_guard: bool,
    /// Require completed empty rays and synchronous stopped motion for absence recovery.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clear_sweep_recovery: Option<ClearSweepRecoveryConfig>,
    #[serde(default, skip_serializing_if = "is_false")]
    pub settle_obstruction_stops: bool,
    /// Search earlier collision-safe stopping positions and retain their hold.
    #[serde(default, skip_serializing_if = "is_false")]
    pub search_obstruction_stops: bool,
    /// Optional extra road-edge reserve for width-fitted lateral candidates.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lateral_candidate_margin_m: Option<f64>,
    /// Add six complete width-fitted intermediate paths with a one-meter reserve.
    #[serde(default, skip_serializing_if = "is_false")]
    pub refine_lateral_candidates: bool,
    /// Include bounded planner decision summaries in sensor-only outputs.
    #[serde(default, skip_serializing_if = "is_false")]
    pub record_planning_diagnostics: bool,
    /// Observe sustained stationary forecast blocking without changing commands.
    #[serde(default, skip_serializing_if = "is_false")]
    pub monitor_planning_progress: bool,
    /// Opt-in bounded acquisition-time GNSS correction and odometry replay.
    #[serde(default, skip_serializing_if = "is_false")]
    pub gnss_latency_compensation: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub local_route_geometry: bool,
    /// Optional chassis-to-rear-axle distance for low-speed no-slip motion.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rear_axle_offset_m: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion_limits: Option<MotionLimits>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub multi_height_lidar: Option<MultiHeightLidarConfig>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lidar3d: Option<Lidar3dConfig>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization_map: Option<MapLocalizationConfig>,
    /// Opt-in stopped regional map relocalization after motion continuity loss.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub relocalization: Option<RelocalizationConfig>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub perception3d: Option<Perception3dConfig>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation: Option<NavigationConfig>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stop_lines: Vec<StopLine>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stop_signs: Vec<StopLine>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub yield_intersections: Vec<YieldIntersection>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lane_change: Option<LaneChangeConfig>,
}
impl PipelineConfig {
    fn new_tracker(&self) -> Tracker {
        let tracker = if self.fit_recent_track_motion {
            Tracker::with_recent_motion_fit()
        } else if self.confirm_track_initialization {
            Tracker::with_confirmed_initialization()
        } else {
            Tracker::default()
        };
        if self.enclose_track_measurements {
            tracker.with_measurement_enclosure()
        } else {
            tracker
        }
    }

    pub fn new(route: Route, initial_pose: Pose, vehicle: VehicleConfig) -> Self {
        Self {
            route,
            initial_pose,
            initial_pose_covariance: None,
            gnss_covariance_shape: None,
            gnss_antenna_offset_m: None,
            gnss_recovery: None,
            absolute_heading_fusion: false,
            heading_latency_compensation: false,
            vehicle,
            nominal_dt: 0.05,
            cruise_speed: 8.0,
            speed_zones: vec![],
            confirm_slow_motion: false,
            confirm_track_initialization: false,
            fit_recent_track_motion: false,
            fit_sparse_circles: false,
            enclose_track_measurements: false,
            forecast_envelope: None,
            forecast_consistency_guard: false,
            clear_sweep_recovery: None,
            settle_obstruction_stops: false,
            search_obstruction_stops: false,
            lateral_candidate_margin_m: None,
            refine_lateral_candidates: false,
            record_planning_diagnostics: false,
            monitor_planning_progress: false,
            gnss_latency_compensation: false,
            local_route_geometry: false,
            rear_axle_offset_m: None,
            motion_limits: None,
            multi_height_lidar: None,
            lidar3d: None,
            localization_map: None,
            relocalization: None,
            perception3d: None,
            navigation: None,
            stop_lines: vec![],
            stop_signs: vec![],
            yield_intersections: vec![],
            lane_change: None,
        }
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.fit_recent_track_motion
            && (!self.enclose_track_measurements || self.confirm_track_initialization)
        {
            return Err(
                "recent track motion requires measured-disc enclosure and ordinary initialization"
                    .into(),
            );
        }
        if self.fit_sparse_circles && (self.multi_height_lidar.is_some() || self.lidar3d.is_some())
        {
            return Err("sparse circle fitting requires planar lidar".into());
        }
        if let Some(policy) = &self.clear_sweep_recovery {
            rustdriving_perception::planar_sweep::validate_recovery(policy)?;
            if !self.forecast_consistency_guard
                || self.multi_height_lidar.is_some()
                || self.lidar3d.is_some()
            {
                return Err(
                    "empty-sweep recovery requires the planar forecast consistency guard".into(),
                );
            }
        }
        if self.forecast_consistency_guard
            && (self.forecast_envelope.is_none() || !self.enclose_track_measurements)
        {
            return Err(
                "forecast consistency requires declared envelopes and acquired-disc preservation"
                    .into(),
            );
        }
        if self.refine_lateral_candidates && self.lateral_candidate_margin_m.is_none() {
            return Err("intermediate lateral candidates require road-width fitting".into());
        }
        if self.heading_latency_compensation
            && (!self.absolute_heading_fusion || !self.gnss_latency_compensation)
        {
            return Err(
                "heading latency compensation requires heading fusion and GNSS history".into(),
            );
        }
        if self.search_obstruction_stops && !self.settle_obstruction_stops {
            return Err("obstruction stop search requires obstruction stop retention".into());
        }
        if self.absolute_heading_fusion && self.relocalization.is_some() {
            return Err("absolute heading currently excludes relocalization".into());
        }
        if let Some(bounds) = self.forecast_envelope {
            bounds.validate()?;
        }
        if let Some(policy) = self.relocalization {
            policy.validate()?;
            if policy.preserve_navigation && self.navigation.is_none() {
                return Err(
                    "navigation-preserving relocalization requires a navigation mission".into(),
                );
            }
            if policy.preserve_stop_signs && self.stop_signs.is_empty() {
                return Err("stop-preserving relocalization requires mapped stop signs".into());
            }
            if policy.preserve_traffic_signals && self.stop_lines.is_empty() {
                return Err(
                    "signal-preserving relocalization requires mapped traffic signals".into(),
                );
            }
            if policy.preserve_yield_intersections && self.yield_intersections.is_empty() {
                return Err("yield-preserving relocalization requires mapped intersections".into());
            }
            if policy.confirm_gnss_handoff
                && (policy.map_only_grace_s.is_none()
                    || self.gnss_recovery.is_none()
                    || !self.gnss_latency_compensation)
            {
                return Err("confirmed GNSS handoff requires map-only grace, a GNSS recovery policy and acquisition-time GNSS fusion".into());
            }
            if policy.map_only_grace_s.is_some()
                && self
                    .localization_map
                    .as_ref()
                    .is_some_and(|map| map.allow_delayed_scans)
            {
                return Err("post-reset map-only support requires synchronous map scans".into());
            }
            if self.localization_map.is_none()
                || self
                    .localization_map
                    .as_ref()
                    .is_some_and(|map| map.min_pairs > 128)
                || (self.navigation.is_some() && !policy.preserve_navigation)
                || self.lane_change.is_some()
                || (!self.stop_lines.is_empty() && !policy.preserve_traffic_signals)
                || (!self.stop_signs.is_empty() && !policy.preserve_stop_signs)
                || (!self.yield_intersections.is_empty() && !policy.preserve_yield_intersections)
                || self.lidar3d.is_some()
                || self.multi_height_lidar.is_some()
            {
                return Err("regional relocalization requires a surveyed map and ordinary XY LiDAR, explicit preservation for navigation or fixed-route controls, and no lane changes".into());
            }
        }
        if self
            .lateral_candidate_margin_m
            .is_some_and(|margin| !margin.is_finite() || !(0.1..=1.0).contains(&margin))
        {
            return Err("lateral candidate margin requires 0.1 to 1.0 meters".into());
        }
        if let Some(offset) = self.gnss_antenna_offset_m {
            rustdriving_localization::validate_gnss_antenna_offset(offset)?;
        }
        if let Some(recovery) = self.gnss_recovery {
            recovery.validate()?;
        }
        if let Some(shape) = self.gnss_covariance_shape {
            rustdriving_localization::validate_gnss_covariance_shape(shape)?;
        }
        if let Some(covariance) = self.initial_pose_covariance {
            Ekf::with_pose_covariance(self.initial_pose, covariance)?;
        }
        if let Some(map) = &self.localization_map {
            map.validate()?;
            if map.allow_delayed_scans && !self.gnss_latency_compensation {
                return Err("delayed map scans require GNSS correction-event history".into());
            }
        }
        if let Some(perception) = &self.perception3d {
            perception.validate(
                self.lidar3d
                    .as_ref()
                    .ok_or("XYZ perception requires calibrated 3D LiDAR")?,
            )?;
        }
        if self.multi_height_lidar.is_some() && self.lidar3d.is_some() {
            return Err("LiDAR input modes must be exclusive".into());
        }
        if let Some(calibration) = &self.lidar3d {
            calibration.validate()?;
        }
        if let Some(calibration) = &self.multi_height_lidar {
            calibration.validate()?;
        }
        if self.route.points.len() > 5000 {
            return Err("route exceeds 5000 points".into());
        }
        let canonical = Route::new(self.route.points.clone(), self.route.half_width)?;
        if let Some(nav) = &self.navigation {
            let plan = nav.initial_plan()?;
            if plan.route.points != self.route.points
                || plan.route.half_width != self.route.half_width
            {
                return Err("navigation map does not match configured initial route".into());
            }
        }
        if canonical.lengths != self.route.lengths || canonical.length() > 1500.0 {
            return Err("invalid route arc-length metadata or excessive route length".into());
        }
        if self.speed_zones.len() > 128
            || self
                .speed_zones
                .iter()
                .any(|zone| !zone.valid() || zone.end_s_m > canonical.length())
            || (!self.speed_zones.is_empty()
                && (self.navigation.is_some()
                    || self
                        .lane_change
                        .as_ref()
                        .and_then(|change| change.return_on_conflict.as_ref())
                        .is_some_and(|config| config.speed_limit_m_s.is_none())))
        {
            return Err(
                "invalid speed zones or unsupported live/alternate-route speed zones".into(),
            );
        }
        if let Some(change) = &self.lane_change {
            change.validate(&self.route, self.vehicle)?;
            if !self.local_route_geometry
                || self.navigation.is_some()
                || !self.stop_lines.is_empty()
                || !self.stop_signs.is_empty()
                || !self.yield_intersections.is_empty()
                || self.route.project(self.initial_pose.position).0
                    > change.entry_s_m - self.vehicle.radius - 3.0
                || self.cruise_speed > 4.0
                || (change.return_on_conflict.is_some() && self.rear_axle_offset_m.is_some())
            {
                return Err("lane-change gate requires a fixed local route, pre-entry spawn and cruise <=4 m/s without other mapped controls".into());
            }
        }
        if self.motion_limits.is_some_and(|limits| {
            !limits.max_acceleration_m_s2.is_finite()
                || !(0.1..=2.0).contains(&limits.max_acceleration_m_s2)
                || !limits.max_deceleration_m_s2.is_finite()
                || !(0.1..=6.0).contains(&limits.max_deceleration_m_s2)
                || !limits.max_lateral_acceleration_m_s2.is_finite()
                || !(0.1..=6.0).contains(&limits.max_lateral_acceleration_m_s2)
        }) {
            return Err("invalid calibrated motion limits".into());
        }
        if self.rear_axle_offset_m.is_some_and(|offset| {
            !offset.is_finite()
                || offset <= 0.0
                || offset >= self.vehicle.wheelbase
                || !self.local_route_geometry
        }) {
            return Err(
                "chassis reference requires a bounded rear-axle offset and local route geometry"
                    .into(),
            );
        }
        if self.local_route_geometry {
            if self.motion_limits.is_none() {
                return Err(
                    "local route geometry requires explicit lateral motion calibration".into(),
                );
            }
            rustdriving_planning::validate_local_route_geometry(&self.route, self.vehicle)?;
            // Fixed controls truncate the supplied route at the original
            // surveyed arc before planner-local fillets are fitted. The fit
            // preserves that endpoint, and stop admission still projects onto
            // the original route. Yielding is restricted to a straight route
            // by intersections::validate, so its original arc is unchanged.
        }
        let controls: Vec<_> = self
            .stop_lines
            .iter()
            .chain(&self.stop_signs)
            .chain(self.yield_intersections.iter().map(|z| &z.stop_line))
            .cloned()
            .collect();
        validate_stop_lines(&controls, &self.route, self.vehicle.radius)?;
        intersections::validate(&self.yield_intersections, &self.route, self.vehicle.radius)?;
        if self.stop_signs.iter().enumerate().any(|(i, line)| {
            self.stop_signs[..i]
                .iter()
                .any(|old| (old.route_s_m - line.route_s_m).abs() < 8.0)
        }) {
            return Err("mapped stop signs must be at least 8 m apart".into());
        }
        if !controls.is_empty() && self.navigation.is_some() {
            return Err(
                "mapped traffic controls currently require a fixed route without live handover"
                    .into(),
            );
        }
        let v = self.vehicle;
        if !self.initial_pose.position.finite()
            || !self.initial_pose.yaw.is_finite()
            || ![
                v.wheelbase,
                v.radius,
                v.max_steer,
                self.nominal_dt,
                self.cruise_speed,
            ]
            .iter()
            .all(|x| x.is_finite())
            || !(0.5..=8.0).contains(&v.wheelbase)
            || !(0.2..=3.0).contains(&v.radius)
            || !(0.05..=0.55).contains(&v.max_steer)
            || v.radius >= self.route.half_width
            || !(0.01..=0.1).contains(&self.nominal_dt)
            || !(0.1..=8.0).contains(&self.cruise_speed)
        {
            return Err("invalid pipeline calibration or limits".into());
        }
        Ok(())
    }
}
/// New samples only; None means no new observation, not an empty healthy scan.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorFrame {
    pub time: f64,
    pub odometry: Option<Odometry>,
    pub gnss: Option<Gnss>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub absolute_heading: Option<AbsoluteHeading>,
    pub lidar: Option<LidarScan>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub multi_height_lidar: Option<MultiHeightLidarScan>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lidar3d: Option<Lidar3dScan>,
    /// Explicit adapter acquisition failure; recovery requires a valid newer acquisition.
    /// Empty successful returns are otherwise valid in every input mode.
    #[serde(default)]
    pub lidar_failed: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation_update: Option<NavigationUpdate>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub traffic_signal: Option<SignalObservation>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum HealthIssue {
    StaleAbsoluteHeading,
    InvalidAbsoluteHeading,
    HeadingInnovationHold,
    MissingOdometry,
    StaleOdometry,
    StaleLidar,
    StaleGnss,
    GnssInnovationHold,
    GnssHandoffExpired,
    InvalidOdometry,
    InvalidLidar,
    InvalidGnss,
    LocalizationUncertain,
    AcquisitionFailed,
    ClockGap,
    MotionDiscontinuity,
    InvalidNavigation,
    InvalidStopSignRecovery,
    InvalidTrafficSignalRecovery,
    InvalidIntersectionRecovery,
    InvalidTrafficSignal,
    InconsistentForecast,
}

/// The last valid motion epoch ended; only external pose reinitialization can
/// create a new operational pipeline. Later estimates retain the last state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotionDiscontinuity {
    pub last_covered_time: f64,
    pub detected_at: f64,
    pub last_accepted_odometry_stamp: Option<f64>,
    pub cause: HealthIssue,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GnssLatencyDiagnostics {
    pub compensated_fixes: u64,
    pub history_unavailable_fixes: u64,
    pub history_predictions: usize,
    pub last_accepted_delay_s: Option<f64>,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub history_conflict_fixes: u64,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub history_map_corrections: u64,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub replayed_map_corrections: u64,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub history_gnss_corrections: u64,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub replayed_gnss_corrections: u64,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub history_heading_corrections: u64,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub replayed_heading_corrections: u64,
}
fn is_zero(value: &u64) -> bool {
    *value == 0
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PipelineOutput {
    pub time: f64,
    pub estimate: EgoState,
    /// Continuous wheel/gyro frame expressed in the current world frame.
    /// Present in new logs; tracks and forecasts remain world-coordinate values.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tracking_frame: Option<Pose>,
    pub tracks: Vec<Track>,
    /// Last successfully acquired raw detections, in the continuous tracking frame.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub track_measurements: Option<Vec<TrackMeasurement>>,
    pub predictions: Vec<Prediction>,
    pub trajectory: Trajectory,
    pub command: ControlCommand,
    pub emergency: bool,
    pub health: Vec<HealthIssue>,
    pub position_variance: f64,
    /// Full ENU posterior for explicit initialization or GNSS calibration/policy.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pose_covariance: Option<[[f64; 3]; 3]>,
    /// Measured plane fit for this acquisition, absent on frames without one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ground: Option<GroundDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization: Option<LocalizationDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub absolute_heading: Option<HeadingDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_latency: Option<GnssLatencyDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_recovery: Option<GnssRecoveryDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion_discontinuity: Option<MotionDiscontinuity>,
    /// Track identities are scoped to this localization epoch; zero is legacy.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub localization_epoch: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub relocalization: Option<RelocalizationDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub map_localization: Option<MapLocalizationDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub perception3d: Option<Perception3dDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation: Option<NavigationStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub traffic_controls: Option<TrafficControlStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stop_signs: Option<StopSignStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intersections: Option<IntersectionStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lane_change: Option<LaneChangeStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub forecast_consistency: Option<ForecastConsistencyDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub planning_progress: Option<PlanningProgress>,
    /// Planner proposal only; final faults and mapped duties can override it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub planning_diagnostics: Option<rustdriving_core::PlanningDiagnostics>,
}
const MAX_ODOMETRY_AGE_S: f64 = 0.15;
const MAX_GNSS_AGE_S: f64 = 0.75;

/// Owns algorithm state; adapters supply observations and apply resulting commands.
pub struct DrivingPipeline {
    config: PipelineConfig,
    ekf: Ekf,
    gnss_history: Option<GnssHistory>,
    gnss_latency: GnssLatencyDiagnostics,
    gnss_recovery: Option<Recovery>,
    perception: LidarClusters,
    tracker: Tracker,
    predictor: ObservedBraking,
    forecast_consistency: Option<ForecastConsistencyGuard>,
    planner: LatticePlanner,
    planning_progress: planning_progress::ProgressMonitor,
    controller: PurePursuit,
    grid: OccupancyGrid,
    previous_time: Option<f64>,
    last_odom: Option<Odometry>,
    /// Latest malformed odometry delivery epoch; recovery needs a fresh later acquisition.
    odometry_fault: Option<f64>,
    motion_discontinuity: Option<MotionDiscontinuity>,
    relocalizer: Option<Relocalizer>,
    localization_epoch: u64,
    last_lidar: Option<f64>,
    /// Latest failure delivery epoch, shared by ordinary, layered and XYZ inputs.
    lidar_fault: Option<f64>,
    /// Latest malformed GNSS delivery epoch; only a fresh accepted fix clears it.
    gnss_fault: Option<f64>,
    heading_monitor: heading::HeadingMonitor,
    /// Acquisition cutoff for the opt-in measured epoch, separate from faults.
    gnss_reset_cutoff: Option<f64>,
    rejected_gnss_streak: u8,
    map_localization: Option<MapLocalization>,
    scan_poses: scan_pose::ScanPoseHistory,
    tracking_frame: tracking_frame::TrackingFrame,
    tracking_tracks: Vec<Track>,
    tracks: Vec<Track>,
    navigator: Option<Navigator>,
    traffic_controls: Option<TrafficControls>,
    stop_signs: Option<StopSigns>,
    intersections: Option<YieldIntersections>,
    lane_change: Option<LaneChangeGate>,
}
impl DrivingPipeline {
    pub fn new(config: PipelineConfig) -> Result<Self, String> {
        config.validate()?;
        let traffic_controls = (!config.stop_lines.is_empty())
            .then(|| TrafficControls::new(config.stop_lines.clone()));
        let stop_signs =
            (!config.stop_signs.is_empty()).then(|| StopSigns::new(config.stop_signs.clone()));
        let intersections = (!config.yield_intersections.is_empty()).then(|| {
            YieldIntersections::new(config.yield_intersections.clone(), config.route.clone())
        });
        let navigator = config.navigation.clone().map(Navigator::new).transpose()?;
        let mut planner = LatticePlanner::default();
        planner.cruise_speed = config.cruise_speed;
        planner.speed_zones = navigator.as_ref().map_or_else(
            || config.speed_zones.clone(),
            |nav| nav.speed_zones().to_vec(),
        );
        planner.local_route_geometry = config.local_route_geometry;
        planner.point_reference_recovery = config.rear_axle_offset_m.is_none();
        planner.settle_obstruction_stops = config.settle_obstruction_stops;
        planner.search_obstruction_stops = config.search_obstruction_stops;
        planner.lateral_candidate_margin_m = config.lateral_candidate_margin_m;
        planner.refine_lateral_candidates = config.refine_lateral_candidates;
        planner.set_diagnostics_enabled(
            config.record_planning_diagnostics || config.monitor_planning_progress,
        );
        planner.vehicle = config.vehicle;
        if let Some(limits) = config.motion_limits {
            planner.max_acceleration_m_s2 = limits.max_acceleration_m_s2;
            planner.max_deceleration_m_s2 = limits.max_deceleration_m_s2;
            planner.max_lateral_acceleration_m_s2 = Some(limits.max_lateral_acceleration_m_s2);
        }
        let mut controller = PurePursuit::with_vehicle(config.vehicle);
        controller.local_route_geometry = config.local_route_geometry;
        controller.rear_axle_offset_m = config.rear_axle_offset_m.unwrap_or(0.0);
        let mut map_points = config.route.points.clone();
        if let Some(nav) = &config.navigation {
            map_points.extend(
                nav.network
                    .edges
                    .iter()
                    .flat_map(|e| e.points.iter())
                    .copied(),
            );
        }
        let min_x = map_points.iter().map(|p| p.x).fold(f64::INFINITY, f64::min) - 10.0;
        let min_y = map_points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min) - 20.0;
        let max_x = map_points
            .iter()
            .map(|p| p.x)
            .fold(f64::NEG_INFINITY, f64::max)
            + 20.0;
        let max_y = map_points
            .iter()
            .map(|p| p.y)
            .fold(f64::NEG_INFINITY, f64::max)
            + 20.0;
        let width = ((max_x - min_x) / 0.5).ceil() as usize;
        let height = ((max_y - min_y) / 0.5).ceil() as usize;
        if width
            .checked_mul(height)
            .is_none_or(|cells| cells > 4_000_000)
        {
            return Err("map exceeds the occupancy allocation bound".into());
        }
        let grid = OccupancyGrid::new(Vec2::new(min_x, min_y), width, height, 0.5);
        let mut ekf = Ekf::with_calibration(
            config.initial_pose,
            config.initial_pose_covariance,
            config.gnss_covariance_shape,
        )?;
        if let Some(offset) = config.gnss_antenna_offset_m {
            ekf = ekf.with_gnss_antenna_offset(offset)?;
        }
        let gnss_history = config.gnss_latency_compensation.then(|| {
            if config.absolute_heading_fusion
                || config
                    .localization_map
                    .as_ref()
                    .is_some_and(|map| map.allow_delayed_scans)
            {
                GnssHistory::with_retained_corrections()
            } else {
                GnssHistory::default()
            }
        });
        let predictor = if config.confirm_slow_motion {
            ObservedBraking::with_slow_motion()
        } else {
            ObservedBraking::default()
        };
        let predictor = if let Some(bounds) = config.forecast_envelope {
            predictor.with_envelope(bounds)?
        } else {
            predictor
        };
        let tracking_frame = tracking_frame::TrackingFrame::new(config.initial_pose);
        let map_localization = config.localization_map.clone().map(MapLocalization::new);
        let lane_change = config
            .lane_change
            .clone()
            .map(|c| LaneChangeGate::new(c, config.vehicle, config.cruise_speed));
        let gnss_recovery = config.gnss_recovery.map(Recovery::new);
        let relocalizer = config.relocalization.map(Relocalizer::new);
        let tracker = config.new_tracker();
        let mut forecast_consistency = config
            .forecast_consistency_guard
            .then(ForecastConsistencyGuard::default);
        if config.clear_sweep_recovery.is_some() {
            forecast_consistency
                .as_mut()
                .unwrap()
                .enable_clear_sweep_recovery();
        }
        Ok(Self {
            config,
            ekf,
            gnss_history,
            gnss_latency: GnssLatencyDiagnostics::default(),
            gnss_recovery,
            perception: LidarClusters,
            tracker,
            predictor,
            forecast_consistency,
            planner,
            planning_progress: planning_progress::ProgressMonitor::default(),
            controller,
            grid,
            previous_time: None,
            last_odom: None,
            odometry_fault: None,
            motion_discontinuity: None,
            relocalizer,
            localization_epoch: 0,
            last_lidar: None,
            lidar_fault: None,
            gnss_fault: None,
            heading_monitor: heading::HeadingMonitor::default(),
            gnss_reset_cutoff: None,
            rejected_gnss_streak: 0,
            map_localization,
            scan_poses: scan_pose::ScanPoseHistory::default(),
            tracking_frame,
            tracking_tracks: vec![],
            tracks: vec![],
            navigator,
            traffic_controls,
            stop_signs,
            intersections,
            lane_change,
        })
    }
    /// Clock errors return Err before mutation; callers must apply emergency braking on Err.
    /// Invalid or missing sensor data produces a finite emergency command with diagnostics.
    pub fn step(&mut self, input: &SensorFrame) -> Result<PipelineOutput, String> {
        if !input.time.is_finite()
            || input.time < 0.0
            || self.previous_time.is_some_and(|t| input.time <= t)
        {
            return Err("frame time must be finite, nonnegative and strictly increasing".into());
        }
        let dt = self
            .previous_time
            .map_or(self.config.nominal_dt, |t| input.time - t);
        let first = self.previous_time.is_none();
        if let Some(observer) = &mut self.relocalizer {
            observer.begin_frame();
            if let Some(gap) = &self.motion_discontinuity {
                let proposal = observer.observe(
                    input,
                    gap,
                    self.ekf.state().pose,
                    self.config.localization_map.as_ref().unwrap(),
                );
                if let Some((pose, covariance)) = proposal {
                    return self.start_localization_epoch(input, pose, covariance);
                }
            }
        }
        let mut health = Vec::new();
        if dt > 0.25 {
            health.push(HealthIssue::ClockGap);
        }
        if let Some(odom) = input.odometry {
            if !stamp_valid(odom.stamp, input.time)
                || !odom.speed.is_finite()
                || !odom.yaw_rate.is_finite()
                || !(-0.1..=20.0).contains(&odom.speed)
                || odom.yaw_rate.abs() > 3.0
            {
                self.odometry_fault = Some(input.time);
            } else if self.last_odom.is_none_or(|last| odom.stamp > last.stamp)
                && self.odometry_fault.is_none_or(|fault| {
                    odom.stamp > fault && input.time - odom.stamp <= MAX_ODOMETRY_AGE_S + 1e-9
                })
            {
                self.last_odom = Some(odom);
                self.odometry_fault = None;
            }
        }
        if self.odometry_fault.is_some() {
            health.push(HealthIssue::InvalidOdometry);
        }
        let motion_allowed = self.motion_discontinuity.is_none();
        let mut motion_covered = first;
        let mut recorded_motion = false;
        match self.last_odom {
            None => health.push(HealthIssue::MissingOdometry),
            Some(odom) if input.time - odom.stamp > MAX_ODOMETRY_AGE_S + 1e-9 => {
                health.push(HealthIssue::StaleOdometry)
            }
            Some(odom) if !first && dt <= 0.25 && motion_allowed => {
                let before = self.ekf.state().pose;
                if let Some(history) = &mut self.gnss_history {
                    recorded_motion = history
                        .predict(
                            &mut self.ekf,
                            odom,
                            self.previous_time.unwrap(),
                            input.time,
                            self.config.rear_axle_offset_m.unwrap_or(0.),
                        )
                        .is_ok();
                    motion_covered = recorded_motion;
                    if !recorded_motion {
                        self.odometry_fault = Some(input.time);
                        health.push(HealthIssue::InvalidOdometry);
                    }
                } else if let Some(offset) = self.config.rear_axle_offset_m {
                    self.ekf.predict_chassis(odom, dt, offset);
                    motion_covered = true;
                } else {
                    self.ekf.predict(odom, dt);
                    motion_covered = true;
                }
                self.tracking_frame.advance(before, self.ekf.state().pose);
            }
            _ => {}
        }
        if !recorded_motion && let Some(history) = &mut self.gnss_history {
            history.reset(input.time);
        }
        if !first && motion_allowed && !motion_covered {
            let cause = health
                .iter()
                .find(|issue| {
                    matches!(
                        issue,
                        HealthIssue::ClockGap
                            | HealthIssue::MissingOdometry
                            | HealthIssue::StaleOdometry
                            | HealthIssue::InvalidOdometry
                    )
                })
                .cloned()
                .unwrap_or(HealthIssue::InvalidOdometry);
            self.motion_discontinuity = Some(MotionDiscontinuity {
                last_covered_time: self.previous_time.unwrap(),
                detected_at: input.time,
                last_accepted_odometry_stamp: self.last_odom.map(|odom| odom.stamp),
                cause,
            });
            // The missing rigid motion cannot be inferred from later scalar
            // readings. Discard scene history and never bridge this epoch.
            self.scan_poses = scan_pose::ScanPoseHistory::default();
            self.tracker = self.config.new_tracker();
            self.tracking_tracks.clear();
            self.tracks.clear();
            if let Some(guard) = &mut self.forecast_consistency {
                guard.invalidate_motion_epoch(input.time);
            }
            self.predictor = if self.config.confirm_slow_motion {
                ObservedBraking::with_slow_motion()
            } else {
                ObservedBraking::default()
            };
            self.lidar_fault = Some(input.time);
        }
        if self.motion_discontinuity.is_some() {
            health.push(HealthIssue::MotionDiscontinuity);
        }
        let uncovered_recovery_motion = health.iter().any(|issue| {
            matches!(
                issue,
                HealthIssue::MissingOdometry
                    | HealthIssue::StaleOdometry
                    | HealthIssue::InvalidOdometry
                    | HealthIssue::ClockGap
            )
        });
        if let Some(recovery) = &mut self.gnss_recovery {
            recovery.advance(input.time);
            if uncovered_recovery_motion {
                recovery.expire();
            }
        }
        if let Some(fix) = input.gnss
            && self.motion_discontinuity.is_none()
        {
            if !stamp_valid(fix.stamp, input.time)
                || !fix.position.finite()
                || !fix.variance.is_finite()
                || fix.variance <= 0.0
            {
                self.gnss_fault = Some(input.time);
                if let Some(recovery) = &mut self.gnss_recovery {
                    recovery.interrupt();
                }
            } else if self
                .gnss_reset_cutoff
                .is_none_or(|cutoff| fix.stamp > cutoff)
                && self.gnss_fault.is_none_or(|fault| {
                    fix.stamp > fault && input.time - fix.stamp <= MAX_GNSS_AGE_S + 1e-9
                })
            {
                // A held proposal must not change pose/covariance, event snapshots
                // or replay counters. Normal/default updates use the existing path.
                let saved = self
                    .gnss_recovery
                    .as_ref()
                    .filter(|r| r.active())
                    .map(|_| (self.ekf.clone(), self.gnss_history.clone()));
                let mut decision = if let Some(history) = &mut self.gnss_history {
                    match history.correct(&mut self.ekf, fix, input.time) {
                        Ok(decision) => decision,
                        Err(
                            HistoryError::MapCorrectionConflict
                            | HistoryError::GnssCorrectionConflict
                            | HistoryError::HeadingCorrectionConflict,
                        ) => {
                            self.gnss_latency.history_conflict_fixes =
                                self.gnss_latency.history_conflict_fixes.saturating_add(1);
                            GnssDecision::Invalid
                        }
                        Err(_) => {
                            self.gnss_latency.history_unavailable_fixes = self
                                .gnss_latency
                                .history_unavailable_fixes
                                .saturating_add(1);
                            GnssDecision::Invalid
                        }
                    }
                } else {
                    self.ekf.correct(fix)
                };
                if decision == GnssDecision::Accepted
                    && let Some((original, history)) = saved
                    && !self.gnss_recovery.as_mut().unwrap().consider(
                        input.time,
                        fix.stamp,
                        original.state().pose,
                        self.ekf.state().pose,
                    )
                {
                    let nis = self.ekf.diagnostics().last_nis.unwrap();
                    self.ekf = original;
                    self.gnss_history = history;
                    decision = self.ekf.hold_recovery_fix(fix, nis);
                }
                match decision {
                    GnssDecision::RejectedInnovation | GnssDecision::RecoveryHeld => {
                        self.rejected_gnss_streak = self.rejected_gnss_streak.saturating_add(1);
                        if decision == GnssDecision::RejectedInnovation
                            && let Some(recovery) = &mut self.gnss_recovery
                        {
                            recovery.rejected(input.time, self.rejected_gnss_streak);
                        }
                    }
                    GnssDecision::Accepted => {
                        if self.gnss_history.is_some() {
                            let delay = (input.time - fix.stamp).max(0.);
                            self.gnss_latency.last_accepted_delay_s = Some(delay);
                            if delay > 1e-9 {
                                self.gnss_latency.compensated_fixes =
                                    self.gnss_latency.compensated_fixes.saturating_add(1);
                            }
                        }
                        self.rejected_gnss_streak = 0;
                        self.gnss_fault = None;
                    }
                    GnssDecision::Invalid => {
                        self.gnss_fault = Some(input.time);
                        if let Some(recovery) = &mut self.gnss_recovery {
                            recovery.interrupt();
                        }
                    }
                    GnssDecision::IgnoredTimestamp => {}
                }
            }
        }
        self.heading_monitor.observe(
            &mut self.ekf,
            self.gnss_history.as_mut(),
            input,
            &self.config,
            self.motion_discontinuity.is_none(),
            &mut health,
        );
        if uncovered_recovery_motion && let Some(recovery) = &mut self.gnss_recovery {
            recovery.expire();
        }
        if self.gnss_fault.is_some() {
            health.push(HealthIssue::InvalidGnss);
        }
        if self.rejected_gnss_streak >= 2 {
            health.push(HealthIssue::GnssInnovationHold);
        }
        let gnss_handoff_expired = self.localization_epoch > 0
            && self
                .config
                .relocalization
                .is_some_and(|policy| policy.confirm_gnss_handoff)
            && self
                .gnss_recovery
                .as_ref()
                .is_some_and(|r| r.diagnostics.phase == RecoveryPhase::Expired);
        if gnss_handoff_expired {
            health.push(HealthIssue::GnssHandoffExpired);
        }
        if input.lidar_failed {
            health.push(HealthIssue::AcquisitionFailed);
            self.lidar_fault = Some(input.time);
        }
        let fused;
        let mut ground = None;
        let mut perception3d = None;
        let mut measured_acquisition = None;
        let scan = if self.motion_discontinuity.is_some() {
            None
        } else {
            match (
                &self.config.multi_height_lidar,
                &self.config.lidar3d,
                &input.multi_height_lidar,
                &input.lidar3d,
            ) {
                (Some(calibration), None, Some(layered), None) if input.lidar.is_none() => {
                    match calibration.fuse(layered) {
                        Ok(scan) => {
                            fused = scan;
                            Some(&fused)
                        }
                        Err(()) => {
                            health.push(HealthIssue::InvalidLidar);
                            self.lidar_fault = Some(input.time);
                            None
                        }
                    }
                }
                (None, Some(calibration), None, Some(cloud)) if input.lidar.is_none() => {
                    let projected = if (calibration.ground.is_some()
                        || self.config.perception3d.is_some())
                        && (input.lidar_failed || !stamp_valid(cloud.stamp, input.time))
                    {
                        Err(None)
                    } else if let Some(config) = &self.config.perception3d {
                        match perception_3d::process(calibration, config, cloud) {
                            Ok(acquisition) => {
                                let scan = acquisition.scan.clone();
                                perception3d = Some(acquisition.diagnostics.clone());
                                measured_acquisition = Some(acquisition);
                                Ok((scan, None))
                            }
                            Err(diagnostic) => {
                                perception3d = Some(*diagnostic);
                                Err(None)
                            }
                        }
                    } else {
                        calibration.project(cloud)
                    };
                    match projected {
                        Ok((scan, diagnostics)) => {
                            ground = diagnostics;
                            fused = scan;
                            Some(&fused)
                        }
                        Err(diagnostics) => {
                            ground = diagnostics.map(|diagnostic| *diagnostic);
                            health.push(HealthIssue::InvalidLidar);
                            self.lidar_fault = Some(input.time);
                            None
                        }
                    }
                }
                (Some(_), None, None, None) | (None, Some(_), None, None)
                    if input.lidar.is_none() =>
                {
                    None
                }
                (None, None, None, None) => input.lidar.as_ref(),
                _ => {
                    health.push(HealthIssue::InvalidLidar);
                    self.lidar_fault = Some(input.time);
                    None
                }
            }
        };
        // Ray outcomes are acquired sensing, including explicit unavailable rays.
        // Validate before either map correction or tracker acceptance can use them.
        let scan = scan.filter(|scan| {
            let valid = if let Some(policy) = &self.config.clear_sweep_recovery {
                rustdriving_perception::planar_sweep::certify_empty_sweep(scan, policy).is_ok()
            } else {
                scan.sweep.is_none()
                    || rustdriving_perception::planar_sweep::validate_sweep(scan).is_ok()
            };
            if !valid {
                health.push(HealthIssue::InvalidLidar);
                self.lidar_fault = Some(input.time);
            }
            valid
        });
        if let Some(map) = &mut self.map_localization {
            let eligible = self.motion_discontinuity.is_none()
                && !input.lidar_failed
                && !health.contains(&HealthIssue::InvalidLidar)
                && scan.is_none_or(|scan| {
                    stamp_valid(scan.stamp, input.time)
                        && scan.points.len() <= 20_000
                        && scan
                            .points
                            .iter()
                            .all(|p| p.finite() && p.x.hypot(p.y) <= 200.0)
                        && self.last_lidar.is_none_or(|last| scan.stamp > last)
                        && self
                            .lidar_fault
                            .is_none_or(|fault_time| scan.stamp > fault_time)
                });
            map.update(
                scan,
                input.time,
                eligible,
                &mut self.ekf,
                self.gnss_history.as_mut(),
            );
        }
        let estimate = self.ekf.state();
        let tracking_to_world = self.tracking_frame.to_world(estimate.pose);
        if self.motion_discontinuity.is_none() {
            self.scan_poses.record(input.time, self.tracking_frame.pose);
        }
        if let Some(scan) = scan {
            if input.lidar_failed
                || !stamp_valid(scan.stamp, input.time)
                || scan.points.len() > 20_000
                || scan
                    .points
                    .iter()
                    .any(|p| !p.finite() || p.x.hypot(p.y) > 200.0)
            {
                health.push(HealthIssue::InvalidLidar);
                self.lidar_fault = Some(input.time);
            } else if self.last_lidar.is_none_or(|last| scan.stamp > last)
                && self
                    .lidar_fault
                    .is_none_or(|fault_time| scan.stamp > fault_time)
            {
                if let Some(acquisition_pose) = self.scan_poses.at(scan.stamp, input.time) {
                    let detections = if let Some(acquisition) = &measured_acquisition {
                        acquisition.detections_at(acquisition_pose)
                    } else if self.config.fit_sparse_circles {
                        self.perception
                            .detect_with_sparse_circles(scan, acquisition_pose)
                    } else {
                        self.perception.detect(scan, acquisition_pose)
                    };
                    match self.tracker.try_update(&detections, scan.stamp) {
                        Ok(tracks) => {
                            self.tracking_tracks = tracks;
                            self.grid.update(
                                scan,
                                tracking_frame::pose_to_world(tracking_to_world, acquisition_pose),
                            );
                            self.last_lidar = Some(scan.stamp);
                            self.lidar_fault = None;
                        }
                        Err(_) => {
                            health.push(HealthIssue::InvalidLidar);
                            self.lidar_fault = Some(input.time);
                        }
                    }
                } else {
                    health.push(HealthIssue::InvalidLidar);
                    self.lidar_fault = Some(input.time);
                }
            }
        }
        if self.lidar_fault.is_some() && !health.contains(&HealthIssue::InvalidLidar) {
            health.push(HealthIssue::InvalidLidar);
        }
        if self.last_lidar.is_none_or(|t| input.time - t > 0.35 + 1e-9) {
            health.push(HealthIssue::StaleLidar);
        }
        let effective_fix_stamp = self
            .map_localization
            .as_ref()
            .map_or(self.ekf.last_gnss, |map| {
                map.effective_fix_stamp(self.ekf.last_gnss)
            });
        let map_diagnostic = self
            .map_localization
            .as_ref()
            .map(MapLocalization::diagnostics);
        let map_only_support = self.relocalizer.as_mut().is_some_and(|observer| {
            observer.map_only_support(
                input.time,
                map_diagnostic.as_ref(),
                self.ekf.last_gnss.is_finite(),
                self.gnss_fault.is_some()
                    || self.rejected_gnss_streak > 0
                    || gnss_handoff_expired
                    || self.motion_discontinuity.is_some()
                    || uncovered_recovery_motion,
                health.is_empty(),
            )
        });
        // This is control freshness from an admitted localization observation,
        // separate from actual GNSS clocks/counters. The same qualified basis
        // must reach the final independent guard as well as the health check.
        let control_localization_stamp = map_diagnostic
            .as_ref()
            .filter(|_| map_only_support)
            .and_then(|map| map.last_accepted_stamp)
            .unwrap_or(effective_fix_stamp);
        if !map_only_support
            && (!effective_fix_stamp.is_finite()
                || input.time - effective_fix_stamp > MAX_GNSS_AGE_S + 1e-9)
        {
            health.push(HealthIssue::StaleGnss);
        }
        let variance = self.ekf.position_variance();
        if !variance.is_finite() || !(0.0..=4.0).contains(&variance) {
            health.push(HealthIssue::LocalizationUncertain);
        }
        // Keep the tracker and its braking-history samples in one continuous
        // frame. Reproject cached observations too, without refreshing stamps.
        self.tracks = self
            .tracking_tracks
            .iter()
            .cloned()
            .map(|track| tracking_frame::track_to_world(tracking_to_world, track))
            .collect();
        let continuous_predictions = self.predictor.predict(&self.tracking_tracks, input.time);
        let clear_sweep = self
            .config
            .clear_sweep_recovery
            .as_ref()
            .and_then(|policy| {
                let acquired = scan.filter(|scan| {
                    scan.stamp == input.time && self.last_lidar == Some(scan.stamp)
                })?;
                self.last_odom
                    .filter(|motion| motion.stamp == input.time && motion.speed.abs() <= 0.2)?;
                rustdriving_perception::planar_sweep::certify_empty_sweep(acquired, policy)
                    .ok()
                    .flatten()
            });
        if let Some(guard) = &mut self.forecast_consistency {
            guard.observe_with_clear_sweep(
                input.time,
                self.last_lidar,
                self.tracker.measurements(),
                &self.tracking_tracks,
                &continuous_predictions,
                ForecastConsistencyAdmission {
                    upstream_healthy: health.is_empty(),
                    clear_sweep,
                },
            );
            if guard.blocking() {
                health.push(HealthIssue::InconsistentForecast);
            }
        }
        let predictions: Vec<_> = continuous_predictions
            .into_iter()
            .map(|prediction| tracking_frame::prediction_to_world(tracking_to_world, prediction))
            .collect();
        let unexpected_signal = self.traffic_controls.is_none() && input.traffic_signal.is_some();
        let mut switched = false;
        if let Some(nav) = &mut self.navigator {
            switched = nav.step(
                input.navigation_update.as_ref(),
                input.time,
                estimate,
                self.last_odom
                    .filter(|_| health.is_empty() && !unexpected_signal),
                self.config.vehicle.radius,
                self.planner.max_deceleration_m_s2,
            );
            if nav.status().phase == NavigationPhase::Fault {
                health.push(HealthIssue::InvalidNavigation);
            }
        } else if input.navigation_update.is_some() {
            health.push(HealthIssue::InvalidNavigation);
        }
        if switched {
            self.config.route = self.navigator.as_ref().unwrap().plan().route.clone();
            let speed_zones = self.navigator.as_ref().unwrap().speed_zones().to_vec();
            // Keep EKF, sensor ages, tracks and occupancy. Reset only route-dependent actuation state.
            self.planner.reset_route_state();
            self.planning_progress.reset();
            self.planner.speed_zones = speed_zones;
            self.controller.reset_route_state();
        }
        // Another latched mapped-duty fault must also prevent semantic passage,
        // even though its health issue is appended below before final actuation.
        let controls_healthy = health.is_empty()
            && self
                .stop_signs
                .as_ref()
                .is_none_or(|stops| !stops.recovery_fault())
            && self
                .intersections
                .as_ref()
                .is_none_or(|zones| !zones.recovery_fault());
        if let Some(controls) = &mut self.traffic_controls {
            controls.step(
                input.traffic_signal.as_ref(),
                input.time,
                self.config.route.project(estimate.pose.position).0,
                self.config.vehicle.radius,
                controls_healthy,
            );
            if controls.status().fault {
                health.push(HealthIssue::InvalidTrafficSignal);
            }
            if controls.recovery_fault() {
                health.push(HealthIssue::InvalidTrafficSignalRecovery);
            }
        } else if unexpected_signal {
            health.push(HealthIssue::InvalidTrafficSignal);
        }
        if let Some(stops) = &mut self.stop_signs {
            let measured_motion = self
                .last_odom
                .filter(|o| input.time - o.stamp <= stop_signs::MAX_STOP_MOTION_AGE_S + 1e-9);
            stops.step(
                &self.config.route,
                input.time,
                estimate,
                self.config.vehicle.radius,
                measured_motion,
                health.is_empty()
                    && self
                        .intersections
                        .as_ref()
                        .is_none_or(|zones| !zones.recovery_fault()),
            );
            if stops.recovery_fault() {
                health.push(HealthIssue::InvalidStopSignRecovery);
            }
        }
        if let Some(zones) = &mut self.intersections {
            zones.step(
                input.time,
                estimate,
                self.config.vehicle.radius,
                &predictions,
                self.last_lidar,
                health.is_empty(),
            );
            if zones.recovery_fault() {
                health.push(HealthIssue::InvalidIntersectionRecovery);
            }
        }
        if let Some(change) = &mut self.lane_change {
            change.step(
                &self.config.route,
                estimate,
                &predictions,
                input.time,
                self.last_lidar,
                health.is_empty(),
            );
            let changed_route = change.observe_return(lane_change::ReturnObservation {
                route: &self.config.route,
                speed_zones: &self.config.speed_zones,
                ego: estimate,
                predictions: &predictions,
                now: input.time,
                scan: self.last_lidar,
                healthy: health.is_empty(),
                motion: self
                    .last_odom
                    .filter(|m| input.time - m.stamp <= 0.05 + 1e-9),
                limits: self
                    .config
                    .motion_limits
                    .expect("validated local-route calibration"),
            });
            if changed_route {
                // Reset only route-dependent planning state. Preserve controller
                // steering history so the normal steering-rate bound still applies.
                self.planner.reset_route_state();
                self.planning_progress.reset();
                self.planner.speed_zones = change.alternate_speed_zones();
            }
        }
        let stop_route = self
            .navigator
            .as_ref()
            .and_then(Navigator::planning_route)
            .or_else(|| {
                let signal_route = self
                    .traffic_controls
                    .as_ref()
                    .and_then(|c| c.planning_route(&self.config.route));
                let sign_route = self
                    .stop_signs
                    .as_ref()
                    .and_then(|c| c.planning_route(&self.config.route));
                let intersection_route = self
                    .intersections
                    .as_ref()
                    .and_then(|c| c.planning_route(&self.config.route));
                let lane_change_route = self
                    .lane_change
                    .as_ref()
                    .and_then(|c| c.planning_route(&self.config.route));
                [
                    signal_route,
                    sign_route,
                    intersection_route,
                    lane_change_route,
                ]
                .into_iter()
                .flatten()
                .min_by(|a, b| a.length().total_cmp(&b.length()))
            });
        let configured_cruise = self.planner.cruise_speed;
        if let Some(change) = &self.lane_change {
            self.planner.cruise_speed = configured_cruise.min(change.approach_speed_limit(
                &self.config.route,
                estimate,
                self.config.vehicle.radius,
                self.planner.max_deceleration_m_s2,
            ));
        }
        if let Some(zones) = &self.intersections {
            self.planner.cruise_speed = configured_cruise.min(zones.approach_speed_limit(
                estimate,
                self.config.vehicle.radius,
                self.planner.max_deceleration_m_s2,
            ));
        }
        // The recorded pose remains body heading for sensing. Only local path
        // joining and pursuit use no-slip velocity course at the calibrated chassis.
        let mut motion_estimate = estimate;
        if let (Some(offset), Some(odom)) = (self.config.rear_axle_offset_m, self.last_odom)
            && odom.speed > 0.1
        {
            motion_estimate.pose.yaw =
                wrap_angle(estimate.pose.yaw + (offset * odom.yaw_rate).atan2(odom.speed));
        }
        // Temporary stopping positions retain their exact approach profile;
        // terminal position tolerance applies only to the active destination.
        self.planner.terminal_settling = stop_route.is_none();
        self.planner.observe_refinement_scene(if health.is_empty() {
            self.last_lidar
        } else {
            None
        });
        let mut trajectory = self.planner.plan(
            motion_estimate,
            stop_route.as_ref().unwrap_or(&self.config.route),
            &predictions,
        );
        self.planner.cruise_speed = configured_cruise;
        if self.navigator.as_ref().is_some_and(|nav| {
            matches!(
                nav.status().phase,
                NavigationPhase::Braking | NavigationPhase::Blocked
            )
        }) && trajectory.mode != DrivingMode::Emergency
        {
            trajectory.mode = DrivingMode::Yield;
        }
        if self
            .traffic_controls
            .as_ref()
            .is_some_and(|c| c.status().stop_s_m.is_some())
            && trajectory.mode != DrivingMode::Emergency
        {
            trajectory.mode = DrivingMode::Yield;
        }
        if self
            .stop_signs
            .as_ref()
            .is_some_and(|s| s.endpoint().is_some())
            && trajectory.mode != DrivingMode::Emergency
        {
            trajectory.mode = DrivingMode::Yield;
        }
        if self
            .intersections
            .as_ref()
            .is_some_and(|s| s.endpoint().is_some())
            && trajectory.mode != DrivingMode::Emergency
        {
            trajectory.mode = DrivingMode::Yield;
        }
        if self.lane_change.as_ref().is_some_and(|c| {
            c.status().stop_s_m.is_some() || c.returning() || c.launch_braking_limit().is_some()
        }) && trajectory.mode != DrivingMode::Emergency
        {
            trajectory.mode = DrivingMode::Yield;
        }
        let command = if health.is_empty() {
            let mut requested = self.controller.control(motion_estimate, &trajectory, dt);
            if self.planner.braking_obstruction_stop() {
                requested.acceleration = requested.acceleration.min(0.0);
            }
            if self
                .stop_signs
                .as_ref()
                .is_some_and(|s| s.holding_brake(&self.config.route, estimate))
                || self.planner.holding_obstruction_stop()
                || self
                    .intersections
                    .as_ref()
                    .is_some_and(|s| s.holding_brake(&self.config.route, estimate))
                || self
                    .lane_change
                    .as_ref()
                    .is_some_and(|c| c.holding_brake(&self.config.route, estimate))
                || self
                    .navigator
                    .as_ref()
                    .is_some_and(Navigator::holding_brake)
            {
                // Keep requesting braking while waiting. A noisy
                // low-speed feedback correction must not creep through the hold.
                requested.acceleration = requested.acceleration.min(-0.5);
            }
            if let Some(limit) = self
                .lane_change
                .as_ref()
                .and_then(LaneChangeGate::launch_braking_limit)
            {
                requested.acceleration = requested.acceleration.min(limit);
            }
            guard(
                requested,
                input.time,
                self.last_lidar.unwrap(),
                control_localization_stamp,
                variance,
            )
        } else {
            ControlCommand::emergency()
        };
        let emergency = !health.is_empty()
            || trajectory.mode == DrivingMode::Emergency
            || command.acceleration <= -5.99;
        if emergency {
            self.controller.reset_emergency_state();
            trajectory.mode = DrivingMode::Emergency;
        }
        let planning_progress = self.config.monitor_planning_progress.then(|| {
            self.planning_progress
                .observe(planning_progress::Observation {
                    now: input.time,
                    acquisition: self.last_lidar,
                    position: estimate.pose.position,
                    speed: estimate.speed,
                    route_remaining: self.config.route.length()
                        - self.config.route.project(estimate.pose.position).0,
                    healthy_unrestricted: health.is_empty() && !emergency && stop_route.is_none(),
                    braking: command.acceleration <= 0.0,
                    trajectory: &trajectory,
                    decisions: self.planner.diagnostics(),
                })
        });
        self.previous_time = Some(input.time);
        Ok(PipelineOutput {
            time: input.time,
            estimate,
            tracking_frame: Some(tracking_to_world),
            tracks: self.tracks.clone(),
            track_measurements: (self.config.confirm_track_initialization
                || self.config.enclose_track_measurements)
                .then(|| self.tracker.measurements().to_vec()),
            predictions,
            trajectory,
            command,
            emergency,
            health,
            position_variance: variance,
            pose_covariance: (self.config.initial_pose_covariance.is_some()
                || self.config.gnss_covariance_shape.is_some()
                || self.config.gnss_antenna_offset_m.is_some()
                || self.config.gnss_recovery.is_some()
                || self.config.absolute_heading_fusion
                || self.config.relocalization.is_some())
            .then(|| self.ekf.pose_covariance()),
            ground,
            localization: Some(self.ekf.diagnostics()),
            absolute_heading: self
                .config
                .absolute_heading_fusion
                .then(|| self.ekf.heading_diagnostics()),
            gnss_recovery: self.gnss_recovery.as_ref().map(|r| r.diagnostics),
            motion_discontinuity: self.motion_discontinuity.clone(),
            localization_epoch: self.localization_epoch,
            relocalization: self.relocalizer.as_ref().map(Relocalizer::diagnostics),
            gnss_latency: self
                .gnss_history
                .as_ref()
                .map(|history| GnssLatencyDiagnostics {
                    history_predictions: history.len(),
                    history_map_corrections: history.map_corrections() as u64,
                    replayed_map_corrections: history.replayed_map_corrections(),
                    history_gnss_corrections: history.gnss_corrections() as u64,
                    replayed_gnss_corrections: history.replayed_gnss_corrections(),
                    history_heading_corrections: history.heading_corrections() as u64,
                    replayed_heading_corrections: history.replayed_heading_corrections(),
                    ..self.gnss_latency
                }),
            map_localization: self
                .map_localization
                .as_ref()
                .map(MapLocalization::diagnostics),
            perception3d,
            navigation: self.navigator.as_ref().map(Navigator::status),
            traffic_controls: self.traffic_controls.as_ref().map(TrafficControls::status),
            stop_signs: self.stop_signs.as_ref().map(StopSigns::status),
            intersections: self.intersections.as_ref().map(YieldIntersections::status),
            lane_change: self.lane_change.as_ref().map(LaneChangeGate::status),
            forecast_consistency: self
                .forecast_consistency
                .as_ref()
                .map(|guard| guard.diagnostics().clone()),
            planning_progress,
            planning_diagnostics: self
                .config
                .record_planning_diagnostics
                .then(|| self.planner.diagnostics().cloned())
                .flatten(),
        })
    }
    fn start_localization_epoch(
        &mut self,
        input: &SensorFrame,
        pose: Pose,
        covariance: [[f64; 3]; 3],
    ) -> Result<PipelineOutput, String> {
        let epoch = self
            .localization_epoch
            .checked_add(1)
            .ok_or("localization epoch exhausted")?;
        let mut config = self.config.clone();
        config.initial_pose = pose;
        config.initial_pose_covariance = Some(covariance);
        // Construction validates the original mission. Its active plan may have
        // changed since startup, and is restored with its live semantic state below.
        if let Some(navigation) = &config.navigation {
            config.route = navigation.initial_plan()?.route;
        }
        // This constructs fresh histories, grid, tracks, planner and controller.
        let mut fresh = Self::new(config)?;
        fresh.localization_epoch = epoch;
        if let Some(mut navigation) = self.navigator.take() {
            navigation.relocalize(
                input.time,
                fresh.ekf.state(),
                fresh.config.vehicle.radius,
                fresh.planner.max_deceleration_m_s2,
            );
            fresh.config.route = navigation.plan().route.clone();
            fresh.planner.speed_zones = navigation.speed_zones().to_vec();
            fresh.navigator = Some(navigation);
        }
        if self.config.relocalization.unwrap().preserve_stop_signs {
            let mut stops = self.stop_signs.take().unwrap();
            stops.relocalize(
                &fresh.config.route,
                input.time,
                fresh.ekf.state(),
                fresh.config.vehicle.radius,
            );
            fresh.stop_signs = Some(stops);
        }
        if self.config.relocalization.unwrap().preserve_traffic_signals {
            let mut controls = self.traffic_controls.take().unwrap();
            controls.relocalize(
                &fresh.config.route,
                input.time,
                fresh.ekf.state(),
                fresh.config.vehicle.radius,
            );
            fresh.traffic_controls = Some(controls);
        }
        if self
            .config
            .relocalization
            .unwrap()
            .preserve_yield_intersections
        {
            let mut zones = self.intersections.take().unwrap();
            zones.relocalize(input.time, fresh.ekf.state(), fresh.config.vehicle.radius);
            fresh.intersections = Some(zones);
        }
        if self.config.relocalization.unwrap().confirm_gnss_handoff {
            fresh
                .gnss_recovery
                .as_mut()
                .unwrap()
                .arm_handoff(input.time);
        }
        if self
            .config
            .relocalization
            .unwrap()
            .map_only_grace_s
            .is_some()
        {
            fresh.gnss_reset_cutoff = Some(input.time);
        } else {
            fresh.gnss_fault = Some(input.time);
        }
        fresh.lidar_fault = Some(input.time);
        // The confirming scan supplies initialization once. Old or same-stamp
        // fixes/scans must not fuse again or renew the new epoch's freshness.
        let mut output = fresh.step(input)?;
        let mut observer = self.relocalizer.take().unwrap();
        observer.reinitialized(input.time);
        output.relocalization = Some(observer.diagnostics());
        fresh.relocalizer = Some(observer);
        *self = fresh;
        Ok(output)
    }
    pub fn occupied_cells(&self) -> Vec<Vec2> {
        self.grid.occupied_cells()
    }
    pub fn active_route(&self) -> &Route {
        &self.config.route
    }
    pub fn navigation_plan(&self) -> Option<&rustdriving_routing::RoutePlan> {
        self.navigator.as_ref().map(Navigator::plan)
    }
}
fn stamp_valid(stamp: f64, now: f64) -> bool {
    stamp.is_finite() && stamp >= 0.0 && stamp <= now + 1e-9
}
#[cfg(test)]
mod tests {
    use super::*;
    fn pipeline() -> DrivingPipeline {
        DrivingPipeline::new(PipelineConfig::new(
            Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 5.5).unwrap(),
            Pose::default(),
            VehicleConfig::default(),
        ))
        .unwrap()
    }
    fn healthy(time: f64) -> SensorFrame {
        SensorFrame {
            absolute_heading: None,
            time,
            odometry: Some(Odometry {
                stamp: time,
                speed: 0.0,
                yaw_rate: 0.0,
            }),
            gnss: Some(Gnss {
                stamp: time,
                position: Vec2::default(),
                variance: 0.02,
            }),
            lidar: Some(LidarScan {
                sweep: None,
                stamp: time,
                points: vec![],
            }),
            multi_height_lidar: None,
            lidar3d: None,
            lidar_failed: false,
            navigation_update: None,
            traffic_signal: None,
        }
    }
    #[test]
    fn tracker_rejection_brakes_without_refreshing_tracks_grid_or_lidar_age() {
        let mut p = pipeline();
        let with_circle = |time, x| {
            let mut frame = healthy(time);
            frame.lidar.as_mut().unwrap().points = (0..32)
                .map(|i| {
                    let a = i as f64 * std::f64::consts::TAU / 32.;
                    Vec2::new(x + a.cos(), 3. + a.sin())
                })
                .collect();
            frame
        };
        let initial = p.step(&with_circle(0., 20.)).unwrap();
        assert!(initial.health.is_empty());
        assert_eq!(initial.tracks.len(), 1);
        let tracks = serde_json::to_value(initial.tracks).unwrap();
        let grid = p.occupied_cells();
        assert!(!grid.is_empty());
        // Inject a dependency clock disagreement through its public API to
        // exercise failure propagation after ordinary scan preflight succeeds.
        p.tracker.try_update(&[], 0.1).unwrap();
        for time in [0.05, 0.1] {
            let output = p.step(&with_circle(time, 30.)).unwrap();
            assert!(output.health.contains(&HealthIssue::InvalidLidar));
            assert!(output.emergency);
            assert_eq!(output.command.acceleration, -6.);
            assert!(output.command.finite());
            assert_eq!(serde_json::to_value(output.tracks).unwrap(), tracks);
            assert_eq!(p.occupied_cells(), grid);
            assert_eq!(p.last_lidar, Some(0.));
            assert_eq!(p.lidar_fault, Some(time));
        }
        let recovered = p.step(&with_circle(0.15, 20.1)).unwrap();
        assert!(recovered.health.is_empty());
        assert!(!recovered.emergency);
        assert_eq!(p.last_lidar, Some(0.15));
        assert!(p.lidar_fault.is_none());
        assert_eq!(recovered.tracks[0].id, 1);
        assert_eq!(recovered.tracks[0].last_seen, 0.15);
    }

    #[test]
    fn chassis_reference_is_explicit_bounded_and_replayable() {
        let route = Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 3.0).unwrap();
        let mut config = PipelineConfig::new(route, Pose::default(), VehicleConfig::default());
        assert!(
            serde_json::to_value(&config)
                .unwrap()
                .get("rear_axle_offset_m")
                .is_none()
        );
        config.rear_axle_offset_m = Some(1.5);
        assert!(config.validate().unwrap_err().contains("chassis reference"));
        config.local_route_geometry = true;
        config.motion_limits = Some(MotionLimits {
            max_acceleration_m_s2: 1.0,
            max_deceleration_m_s2: 2.5,
            max_lateral_acceleration_m_s2: 1.0,
        });
        assert!(config.validate().is_ok());
        let decoded: PipelineConfig =
            serde_json::from_value(serde_json::to_value(&config).unwrap()).unwrap();
        assert_eq!(decoded.rear_axle_offset_m, Some(1.5));
        for offset in [0.0, -1.0, 2.7, f64::NAN, f64::INFINITY] {
            config.rear_axle_offset_m = Some(offset);
            assert!(config.validate().unwrap_err().contains("chassis reference"));
        }
    }
    #[test]
    fn local_route_geometry_is_opt_in_and_rejects_incompatible_coordinates() {
        let route = Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 3.0).unwrap();
        let mut config = PipelineConfig::new(route, Pose::default(), VehicleConfig::default());
        assert!(
            serde_json::to_value(&config)
                .unwrap()
                .get("local_route_geometry")
                .is_none()
        );
        config.local_route_geometry = true;
        assert!(
            config
                .validate()
                .unwrap_err()
                .contains("lateral motion calibration")
        );
        config.motion_limits = Some(MotionLimits {
            max_acceleration_m_s2: 1.0,
            max_deceleration_m_s2: 2.5,
            max_lateral_acceleration_m_s2: 1.0,
        });
        assert!(config.validate().is_ok());
        let encoded = serde_json::to_value(&config).unwrap();
        assert_eq!(encoded["local_route_geometry"], true);
        config.stop_lines.push(StopLine {
            id: "signal".into(),
            route_s_m: 30.0,
        });
        assert!(config.validate().is_ok());
        config.stop_lines.clear();
        config.route = Route::new(
            vec![Vec2::default(), Vec2::new(3.0, 0.0), Vec2::new(3.0, 3.0)],
            3.0,
        )
        .unwrap();
        assert!(config.validate().unwrap_err().contains("fillets overlap"));
    }
    #[test]
    fn empty_scan_is_healthy_but_failed_acquisition_brakes() {
        let mut p = pipeline();
        assert!(!p.step(&healthy(0.0)).unwrap().emergency);
        let mut f = healthy(0.05);
        f.lidar_failed = true;
        let out = p.step(&f).unwrap();
        assert!(out.health.contains(&HealthIssue::AcquisitionFailed));
        assert_eq!(out.command.acceleration, -6.0);
    }
    #[test]
    fn received_outliers_do_not_refresh_health_and_a_new_good_fix_recovers() {
        let route = Route::new(
            vec![
                Vec2::default(),
                Vec2::new(20.0, 10.0),
                Vec2::new(100.0, 10.0),
            ],
            5.5,
        )
        .unwrap();
        let mut p = DrivingPipeline::new(PipelineConfig::new(
            route,
            Pose::default(),
            VehicleConfig::default(),
        ))
        .unwrap();
        let mut previous_steering = 0.0_f64;
        for i in 0..=10 {
            let out = p.step(&healthy(i as f64 * 0.05)).unwrap();
            previous_steering = out.command.steering;
        }
        assert!(previous_steering.abs() > 0.035);
        for i in 11..=40 {
            let time = i as f64 * 0.05;
            let mut f = healthy(time);
            f.gnss.as_mut().unwrap().position = Vec2::new(30.0, -25.0);
            let out = p.step(&f).unwrap();
            let diagnostic = out.localization.unwrap();
            assert_eq!(diagnostic.last_accepted_stamp, Some(0.5));
            assert_eq!(diagnostic.last_observed_stamp, Some(time));
            assert_eq!(
                diagnostic.last_decision,
                Some(GnssDecision::RejectedInnovation)
            );
            assert_eq!(out.estimate.pose.position, Vec2::default());
            if time > 1.25 + 1e-9 {
                assert!(out.health.contains(&HealthIssue::StaleGnss));
                assert_eq!(out.command.acceleration, -6.0);
                assert_eq!(out.command.steering, 0.0);
            }
        }
        let out = p.step(&healthy(2.05)).unwrap();
        assert!(out.health.is_empty());
        assert_eq!(out.localization.unwrap().last_accepted_stamp, Some(2.05));
        assert!(out.command.steering.abs() <= 0.7 * 0.05 + 1e-12);
    }
    #[test]
    fn repeated_new_gnss_outliers_hold_until_a_new_accepted_fix() {
        let mut p = pipeline();
        assert!(p.step(&healthy(0.0)).unwrap().health.is_empty());
        let mut first = healthy(0.05);
        first.gnss.as_mut().unwrap().position = Vec2::new(30.0, -25.0);
        let first_fix = first.gnss;
        let out = p.step(&first).unwrap();
        assert_eq!(
            out.localization.unwrap().last_decision,
            Some(GnssDecision::RejectedInnovation)
        );
        assert!(!out.emergency);
        let mut duplicate = healthy(0.1);
        duplicate.gnss = first_fix;
        let out = p.step(&duplicate).unwrap();
        assert!(out.health.is_empty());
        assert_eq!(p.rejected_gnss_streak, 1);
        let mut second = healthy(0.15);
        second.gnss.as_mut().unwrap().position = Vec2::new(30.0, -25.0);
        let out = p.step(&second).unwrap();
        assert_eq!(out.health, vec![HealthIssue::GnssInnovationHold]);
        assert!(out.emergency);
        assert_eq!(out.command.acceleration, -6.0);
        let mut absent = healthy(0.2);
        absent.gnss = None;
        assert!(
            p.step(&absent)
                .unwrap()
                .health
                .contains(&HealthIssue::GnssInnovationHold)
        );
        // An old fix that would have been good cannot clear the hold.
        let mut old_good = healthy(0.25);
        old_good.gnss.as_mut().unwrap().stamp = 0.0;
        assert!(
            p.step(&old_good)
                .unwrap()
                .health
                .contains(&HealthIssue::GnssInnovationHold)
        );
        let mut invalid = healthy(0.3);
        invalid.gnss.as_mut().unwrap().variance = -1.0;
        let out = p.step(&invalid).unwrap();
        assert!(out.health.contains(&HealthIssue::InvalidGnss));
        assert!(out.health.contains(&HealthIssue::GnssInnovationHold));
        assert_eq!(p.rejected_gnss_streak, 2);
        let recovered = p.step(&healthy(0.35)).unwrap();
        assert!(recovered.health.is_empty());
        assert!(!recovered.emergency);
        assert_eq!(p.rejected_gnss_streak, 0);
        let mut isolated = healthy(0.4);
        isolated.gnss.as_mut().unwrap().position = Vec2::new(30.0, -25.0);
        assert!(p.step(&isolated).unwrap().health.is_empty());
        assert_eq!(p.rejected_gnss_streak, 1);
    }
    #[test]
    fn invalid_navigation_latches_braking_and_does_not_reset_localization() {
        let value: serde_json::Value =
            serde_json::from_str(include_str!("../../../scenarios/route-direct.json")).unwrap();
        let nav: NavigationConfig = serde_json::from_value(value["navigation"].clone()).unwrap();
        let mut config = PipelineConfig::new(
            nav.initial_plan().unwrap().route,
            Pose::default(),
            VehicleConfig::default(),
        );
        config.navigation = Some(nav);
        let mut p = DrivingPipeline::new(config).unwrap();
        for i in 0..=200 {
            let time = i as f64 * 0.05;
            let mut f = healthy(time);
            f.odometry.as_mut().unwrap().speed = 1.0;
            f.gnss.as_mut().unwrap().position = Vec2::new(time, 0.0);
            p.step(&f).unwrap();
        }
        let mut f = healthy(10.05);
        f.gnss = None;
        f.navigation_update = Some(NavigationUpdate {
            stamp: 10.05,
            revision: 1,
            closed_edges: vec!["typo".into()],
        });
        let out = p.step(&f).unwrap();
        assert!(out.health.contains(&HealthIssue::InvalidNavigation));
        assert_eq!(out.command.acceleration, -6.0);
        assert!(out.estimate.pose.position.x > 9.0);
        let mut f = healthy(10.1);
        f.gnss = None;
        assert!(
            p.step(&f)
                .unwrap()
                .health
                .contains(&HealthIssue::InvalidNavigation)
        );
        f.time = 10.15;
        f.odometry.as_mut().unwrap().stamp = 10.15;
        f.lidar.as_mut().unwrap().stamp = 10.15;
        f.navigation_update = Some(NavigationUpdate {
            stamp: 10.15,
            revision: 1,
            closed_edges: vec![],
        });
        let out = p.step(&f).unwrap();
        assert!(!out.emergency);
        assert!(out.estimate.pose.position.x > 9.0);
    }
    #[test]
    fn map_route_mismatch_and_excessive_grid_extent_are_rejected() {
        let value: serde_json::Value =
            serde_json::from_str(include_str!("../../../scenarios/route-direct.json")).unwrap();
        let nav: NavigationConfig = serde_json::from_value(value["navigation"].clone()).unwrap();
        let mut c = PipelineConfig::new(
            nav.initial_plan().unwrap().route,
            Pose::default(),
            VehicleConfig::default(),
        );
        c.navigation = Some(nav);
        let mut mismatch = c.clone();
        mismatch.route.half_width = 4.0;
        assert!(DrivingPipeline::new(mismatch).is_err());
        // An unused remote branch must not turn grid allocation into an overflow or OOM.
        let map = &mut c.navigation.as_mut().unwrap().network;
        map.nodes.push(rustdriving_routing::RoadNode {
            id: "remote".into(),
            position: Vec2::new(100000.0, 0.0),
        });
        map.edges.push(rustdriving_routing::RoadEdge {
            id: "remote-edge".into(),
            from: "east".into(),
            to: "remote".into(),
            points: vec![Vec2::new(200.0, 0.0), Vec2::new(100000.0, 0.0)],
            half_width: 5.5,
            speed_limit_m_s: None,
        });
        assert!(DrivingPipeline::new(c).is_err());
    }
    #[test]
    fn duplicate_and_delayed_scans_cannot_refresh_health() {
        let mut p = pipeline();
        p.step(&healthy(0.0)).unwrap();
        for i in 1..=8 {
            let mut f = healthy(i as f64 * 0.05);
            f.lidar.as_mut().unwrap().stamp = 0.0;
            let out = p.step(&f).unwrap();
            if i == 8 {
                assert!(out.health.contains(&HealthIssue::StaleLidar));
                assert!(out.emergency);
            }
        }
    }
    #[test]
    fn invalid_and_future_sensor_samples_brake() {
        for sensor in [0, 1, 2] {
            let mut p = pipeline();
            let mut f = healthy(0.0);
            match sensor {
                0 => f.odometry.as_mut().unwrap().speed = f64::NAN,
                1 => f.gnss.as_mut().unwrap().stamp = 1.0,
                _ => f
                    .lidar
                    .as_mut()
                    .unwrap()
                    .points
                    .push(Vec2::new(f64::INFINITY, 0.0)),
            };
            let out = p.step(&f).unwrap();
            assert!(out.emergency);
            assert!(out.command.finite());
        }
    }
    #[test]
    fn clock_rejection_does_not_mutate_and_gap_brakes() {
        let mut p = pipeline();
        p.step(&healthy(0.0)).unwrap();
        assert!(p.step(&healthy(0.0)).is_err());
        assert!(p.step(&healthy(f64::NAN)).is_err());
        assert!(!p.step(&healthy(0.05)).unwrap().emergency);
        assert!(
            p.step(&healthy(0.5))
                .unwrap()
                .health
                .contains(&HealthIssue::ClockGap)
        );
    }
    #[test]
    fn missing_odometry_stops_even_with_other_sensors() {
        let mut p = pipeline();
        p.step(&healthy(0.0)).unwrap();
        for i in 1..=4 {
            let mut f = healthy(i as f64 * 0.05);
            f.odometry = None;
            let out = p.step(&f).unwrap();
            if i == 4 {
                assert!(out.health.contains(&HealthIssue::StaleOdometry));
            }
        }
    }
    #[test]
    fn old_motion_limit_calibration_defaults_only_the_new_acceleration_field() {
        let limits: MotionLimits = serde_json::from_str(
            r#"{"max_deceleration_m_s2":1.2,"max_lateral_acceleration_m_s2":1.0}"#,
        )
        .unwrap();
        assert_eq!(limits.max_acceleration_m_s2, 2.0);
        assert_eq!(limits.max_deceleration_m_s2, 1.2);
        assert_eq!(limits.max_lateral_acceleration_m_s2, 1.0);
    }
    #[test]
    fn invalid_calibration_is_rejected_before_execution() {
        let mut config = PipelineConfig::new(
            Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 5.5).unwrap(),
            Pose::default(),
            VehicleConfig::default(),
        );
        config.motion_limits = Some(MotionLimits {
            max_acceleration_m_s2: 2.0,
            max_deceleration_m_s2: 0.0,
            max_lateral_acceleration_m_s2: 1.0,
        });
        assert!(DrivingPipeline::new(config.clone()).is_err());
        config.motion_limits.as_mut().unwrap().max_deceleration_m_s2 = 2.5;
        for acceleration in [0.0, f64::NAN, 3.0] {
            config.motion_limits.as_mut().unwrap().max_acceleration_m_s2 = acceleration;
            assert!(DrivingPipeline::new(config.clone()).is_err());
        }
    }
}
