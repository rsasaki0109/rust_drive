//! Deterministic closed-loop simulator. Only sensor observations enter the stack.
mod gnss_noise;
pub mod heading;
pub mod intersections;
mod lane_change_surface;
pub mod sensor_timing;
pub mod signals;
pub mod speed_zones;
pub mod stop_signs;
pub mod traffic;
use intersections::IntersectionRuleEvaluator;
use rustdriving_core::*;
use rustdriving_pipeline::intersections::{IntersectionStatus, YieldIntersection};
use rustdriving_pipeline::lane_change::{LaneChangeConfig, LaneChangeStatus};
use rustdriving_pipeline::navigation::{NavigationConfig, NavigationStatus, NavigationUpdate};
use rustdriving_pipeline::replay::SensorLog;
use rustdriving_pipeline::stop_signs::StopSignStatus;
use rustdriving_pipeline::traffic_controls::StopLine;
use rustdriving_pipeline::traffic_controls::TrafficControlStatus;
use rustdriving_pipeline::{DrivingPipeline, MapLocalizationConfig, PipelineConfig, SensorFrame};
use rustdriving_routing::{RoadNetwork, RoadNetworkSpec, RoutePlan};
use sensor_timing::{SensorDelivery, SensorTiming};
use serde::{Deserialize, Serialize};
use signals::{RuleEvaluator, SignalDropout, SignalSpec};
use std::f64::consts::PI;
use stop_signs::StopRuleEvaluator;
use traffic::{FollowingSpec, TrafficTelemetry, TrafficWorld};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorldMotion {
    pub position: Vec2,
    pub velocity: Vec2,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObjectSpec {
    pub s: f64,
    pub lateral: f64,
    pub radius: f64,
    #[serde(default)]
    pub speed: f64,
    #[serde(default)]
    pub lateral_speed: f64,
    #[serde(default)]
    pub active_from: f64,
    /// Motion starts at this time; the object already exists at active_from.
    #[serde(default)]
    pub moving_from: f64,
    /// Prescribed non-following motion stops at this acquisition-time boundary.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub moving_until: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub following: Option<FollowingSpec>,
    /// Prescribed world ENU motion, independent of the ego route; simulator-only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub world_motion: Option<WorldMotion>,
}
/// Plant calibration, supported only by the native dynamic RNE adapter.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DynamicsSpec {
    pub friction_coefficient: f64,
    pub steering_lag_s: f64,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Expected {
    Goal,
    Stop,
    Fault,
}
/// Known map and pre-departure closure information, not sensed obstacle labels.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NavigationSpec {
    pub network: RoadNetworkSpec,
    pub start: rustdriving_routing::RouteEndpoint,
    pub goal: rustdriving_routing::RouteEndpoint,
    #[serde(default)]
    pub closed_edges: Vec<String>,
    #[serde(default, skip_serializing_if = "is_false")]
    pub hold_for_handover: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub avoid_observed_obstructions: bool,
}
/// Simulator-only GNSS position fault; never supplied as pipeline calibration.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GnssBiasWindow {
    pub from: f64,
    pub until: f64,
    pub offset: Vec2,
}
/// Simulator-only acquisition outage, never supplied to the driving pipeline.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GnssDropoutWindow {
    pub from: f64,
    pub until: f64,
}
fn is_false(value: &bool) -> bool {
    !*value
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    pub name: String,
    /// Explicit externally supplied initial pose calibration, copied into the pipeline.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub initial_pose_covariance: Option<[[f64; 3]; 3]>,
    /// Simulator-only physical spawn misalignment; never an estimator input.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub initial_heading_offset_rad: Option<f64>,
    /// Externally supplied constant world ENU covariance shape, copied to the pipeline.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_covariance_shape: Option<[[f64; 2]; 2]>,
    /// Simulator-only noise shape; never inferred as operational calibration.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_noise_shape: Option<[[f64; 2]; 2]>,
    /// External GNSS antenna calibration, copied independently of physical mounting.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_antenna_offset_m: Option<Vec2>,
    /// Simulator-only physical antenna location in body forward/left meters.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_antenna_mount_m: Option<Vec2>,
    /// External recovery policy calibration; distinct from simulated faults.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gnss_recovery: Option<rustdriving_pipeline::GnssRecoveryConfig>,
    #[serde(default, skip_serializing_if = "is_false")]
    pub absolute_heading_fusion: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub heading_latency_compensation: bool,
    /// Simulator-only acquisition/noise/fault settings; never a driver input.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_sensor: Option<heading::HeadingSensor>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub relocalization: Option<rustdriving_pipeline::RelocalizationConfig>,
    /// Explicit experimental forecast policy, independent of actor identities.
    #[serde(default, skip_serializing_if = "is_false")]
    pub confirm_slow_motion: bool,
    /// Cold-track acquisition fit; independent of simulator object identities.
    #[serde(default, skip_serializing_if = "is_false")]
    pub confirm_track_initialization: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub fit_recent_track_motion: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub fit_sparse_circles: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub enclose_track_measurements: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub forecast_envelope: Option<rustdriving_core::ForecastEnvelopeConfig>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub forecast_pose_sigma: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub forecast_cold_max_speed_m_s: Option<f64>,
    #[serde(default, skip_serializing_if = "is_false")]
    pub forecast_recent_velocity_hypotheses: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub forecast_consistency_guard: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clear_sweep_recovery: Option<rustdriving_core::ClearSweepRecoveryConfig>,
    /// Preserve each completed ray outcome from the actual planar acquisition.
    #[serde(default, skip_serializing_if = "is_false")]
    pub record_planar_sweep: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub settle_obstruction_stops: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub search_obstruction_stops: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub replan_checked_approaches: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub search_later_obstruction_stops: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lateral_candidate_margin_m: Option<f64>,
    #[serde(default, skip_serializing_if = "is_false")]
    pub refine_lateral_candidates: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub record_planning_diagnostics: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub monitor_planning_progress: bool,
    /// Evaluation recording only; omitted from the operational replay header.
    #[serde(default, skip_serializing_if = "is_false")]
    pub full_rate_recording: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub gnss_latency_compensation: bool,
    #[serde(default, skip_serializing_if = "is_false")]
    pub local_route_geometry: bool,
    pub duration: f64,
    pub road_length: f64,
    pub half_width: f64,
    #[serde(default)]
    pub curve_amplitude: f64,
    pub expected: Expected,
    #[serde(default)]
    pub lidar_dropout: Option<f64>,
    #[serde(default)]
    pub gnss_dropout: Option<f64>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub gnss_bias_windows: Vec<GnssBiasWindow>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub gnss_dropout_windows: Vec<GnssDropoutWindow>,
    /// Explicit offline surveyed point map; never constructed from runtime truth.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization_map: Option<MapLocalizationConfig>,
    /// Simulator-only acquisition/delivery schedule; never supplied as pipeline calibration.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sensor_timing: Option<SensorTiming>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dynamics: Option<DynamicsSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation: Option<NavigationSpec>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub navigation_updates: Vec<NavigationUpdate>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cruise_speed: Option<f64>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub speed_zones: Vec<rustdriving_core::RouteSpeedZone>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion_limits: Option<rustdriving_pipeline::MotionLimits>,
    /// Independent swept-circle acceptance floor in meters; never a planner input.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_clearance_m: Option<f64>,
    /// Continuous physical residence at the goal before ending evaluation.
    /// This acceptance setting is never supplied to the driving pipeline.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub goal_hold_seconds: Option<f64>,
    pub objects: Vec<ObjectSpec>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub traffic_signals: Vec<SignalSpec>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub signal_dropout_windows: Vec<SignalDropout>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stop_signs: Vec<StopLine>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub yield_intersections: Vec<YieldIntersection>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lane_change: Option<LaneChangeConfig>,
}
impl Scenario {
    pub fn validate(&self) -> Result<(), String> {
        if self
            .initial_heading_offset_rad
            .is_some_and(|offset| !offset.is_finite() || offset.abs() > 0.2)
        {
            return Err(
                "simulated initial heading offset must be finite and within +/- 0.2 rad".into(),
            );
        }
        if self.name.is_empty()
            || !self.duration.is_finite()
            || !(1.0..=300.0).contains(&self.duration)
            || !self.road_length.is_finite()
            || !(20.0..=1000.0).contains(&self.road_length)
            || !self.half_width.is_finite()
            || !(1.5..=10.0).contains(&self.half_width)
            || !self.curve_amplitude.is_finite()
            || self.curve_amplitude.abs() > 8.0
        {
            return Err("invalid scenario geometry or duration".into());
        }
        if self.dynamics.is_some_and(|d| {
            !d.friction_coefficient.is_finite()
                || !(0.1..=1.2).contains(&d.friction_coefficient)
                || !d.steering_lag_s.is_finite()
                || !(0.0..=1.0).contains(&d.steering_lag_s)
        }) {
            return Err("invalid dynamic plant calibration".into());
        }
        if let Some(timing) = &self.sensor_timing {
            timing.validate(self.duration)?;
        }
        if self
            .min_clearance_m
            .is_some_and(|d| !d.is_finite() || d < 0.0)
        {
            return Err("minimum clearance must be finite and nonnegative".into());
        }
        if self
            .cruise_speed
            .is_some_and(|s| !s.is_finite() || !(0.1..=8.0).contains(&s))
        {
            return Err("cruise speed must be within 0.1..=8.0 m/s".into());
        }
        if self
            .goal_hold_seconds
            .is_some_and(|hold| !hold.is_finite() || hold < 0.0 || hold > self.duration)
        {
            return Err("goal hold must be finite, nonnegative and within duration".into());
        }
        let selected = self.navigation_plan()?;
        signals::validate(self)?;
        let c = pipeline_config(self);
        c.validate()?;
        if let Some(sensor) = &self.heading_sensor {
            sensor.validate(self.duration)?;
        }
        if let Some(mount) = self.gnss_antenna_mount_m {
            let mut physical = c.clone();
            physical.gnss_antenna_offset_m = Some(mount);
            physical.validate()?;
        }
        if let Some(shape) = self.gnss_noise_shape {
            let mut noise_config = c;
            noise_config.gnss_covariance_shape = Some(shape);
            noise_config.validate()?;
        }
        let mut previous_end = 0.0;
        for window in &self.gnss_bias_windows {
            if !window.from.is_finite()
                || !window.until.is_finite()
                || window.from < previous_end
                || window.until <= window.from
                || window.from >= self.duration
                || window.until > 300.0
                || !window.offset.finite()
                || window.offset.x.hypot(window.offset.y) > 1000.0
            {
                return Err("invalid or overlapping GNSS bias window".into());
            }
            previous_end = window.until;
        }
        if self.gnss_dropout_windows.len() > 64 {
            return Err("at most 64 GNSS acquisition dropout windows are supported".into());
        }
        let mut previous_end = 0.0;
        for window in &self.gnss_dropout_windows {
            if !window.from.is_finite()
                || !window.until.is_finite()
                || window.from < previous_end
                || window.from < 0.0
                || window.until <= window.from
                || window.from >= self.duration
                || window.until > self.duration
            {
                return Err("invalid or overlapping GNSS acquisition dropout window".into());
            }
            previous_end = window.until;
        }
        let mut last_stamp = -1.0;
        let mut last_revision = 0;
        for update in &self.navigation_updates {
            let nav = self
                .navigation
                .as_ref()
                .ok_or("map updates require navigation configuration")?;
            if !update.stamp.is_finite()
                || update.stamp <= last_stamp
                || update.stamp < 0.0
                || update.stamp >= self.duration
                || update.revision <= last_revision
                || update
                    .closed_edges
                    .iter()
                    .any(|id| !nav.network.edges.iter().any(|e| &e.id == id))
            {
                return Err("invalid scheduled map snapshot".into());
            }
            last_stamp = update.stamp;
            last_revision = update.revision;
        }
        let object_limit = if let Some(plan) = &selected {
            if !(20.0..=1000.0).contains(&plan.route.length())
                || !(1.5..=10.0).contains(&plan.route.half_width)
            {
                return Err("selected map route is outside simulation geometry bounds".into());
            }
            plan.route.length()
        } else {
            self.road_length
        };
        for time in [self.lidar_dropout, self.gnss_dropout]
            .into_iter()
            .flatten()
        {
            if !time.is_finite() || time < 0.0 || time >= self.duration {
                return Err("dropout must occur within scenario duration".into());
            }
        }
        for o in &self.objects {
            if let Some(motion) = &o.world_motion
                && (!motion.position.finite()
                    || motion.position.x.abs() > 1e6
                    || motion.position.y.abs() > 1e6
                    || !motion.velocity.finite()
                    || motion.velocity.x.hypot(motion.velocity.y) > 15.0
                    || o.s != 0.0
                    || o.lateral != 0.0
                    || o.speed != 0.0
                    || o.lateral_speed != 0.0
                    || o.following.is_some())
            {
                return Err("world-motion actors require bounded ENU position/velocity, zero route motion and no following policy".into());
            }
            if let Some(following) = &o.following {
                following.validate(o)?;
            }
            if ![
                o.s,
                o.lateral,
                o.radius,
                o.speed,
                o.lateral_speed,
                o.active_from,
                o.moving_from,
            ]
            .iter()
            .all(|x| x.is_finite())
                || o.s < 0.0
                || o.s > object_limit
                || !(0.2..=3.0).contains(&o.radius)
                || o.speed.abs() > 12.0
                || o.lateral_speed.abs() > 4.0
                || o.active_from < 0.0
                || o.moving_from < 0.0
                || o.moving_until.is_some_and(|until| {
                    !until.is_finite()
                        || until < o.moving_from.max(o.active_from)
                        || until > 300.0
                        || o.following.is_some()
                })
            {
                return Err("invalid object parameters".into());
            }
        }
        pipeline_config(self).validate()?;
        Ok(())
    }
    pub fn route(&self) -> Route {
        if self.navigation.is_some() {
            return self
                .navigation_plan()
                .expect("validate scenario before building its route")
                .unwrap()
                .route;
        }
        let n = self.road_length.ceil() as usize;
        Route::new(
            (0..=n)
                .map(|i| {
                    let x = self.road_length * i as f64 / n as f64;
                    Vec2::new(x, self.curve_amplitude * (x / 28.0).sin())
                })
                .collect(),
            self.half_width,
        )
        .unwrap()
    }
    /// Physics initialization only. The operational pose retains its supplied
    /// calibration; this perturbation is not exposed through PipelineConfig.
    pub fn spawn_pose(&self, mut calibrated: Pose) -> Pose {
        if let Some(offset) = self.initial_heading_offset_rad {
            calibrated.yaw = wrap_angle(calibrated.yaw + offset);
        }
        calibrated
    }
    /// Simulation-only antenna position at acquisition, shared by both plants.
    /// Operational calibration is deliberately independent of physical mounting.
    pub fn gnss_antenna_position(&self, pose: Pose) -> Vec2 {
        self.gnss_antenna_mount_m
            .map_or(pose.position, |mount| pose.to_world(mount))
    }
    /// Acquisition-time availability shared by the reference and native plants.
    pub fn gnss_available(&self, time: f64) -> bool {
        self.gnss_dropout.is_none_or(|from| time < from)
            && !self
                .gnss_dropout_windows
                .iter()
                .any(|window| time + 1e-9 >= window.from && time < window.until - 1e-9)
    }
    pub fn navigation_plan(&self) -> Result<Option<RoutePlan>, String> {
        self.navigation
            .as_ref()
            .map(|nav| {
                RoadNetwork::new(nav.network.clone())?.shortest_route_between(
                    &nav.start,
                    &nav.goal,
                    &nav.closed_edges,
                )
            })
            .transpose()
    }
    pub fn scheduled_objects(&self, route: &Route, t: f64) -> Vec<WorldObject> {
        self.objects
            .iter()
            .enumerate()
            .filter(|(_, o)| o.following.is_none() && t >= o.active_from)
            .map(|(id, o)| {
                let elapsed = (o.moving_until.map_or(t, |until| t.min(until))
                    - o.active_from.max(o.moving_from))
                .max(0.0);
                WorldObject {
                    id: id as u64,
                    position: route
                        .sample(
                            o.s + elapsed * o.speed,
                            o.lateral + elapsed * o.lateral_speed,
                        )
                        .0,
                    radius: o.radius,
                }
            })
            .collect()
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WorldObject {
    /// Opaque body identity, stable across frames; not a scenario array index.
    pub id: u64,
    pub position: Vec2,
    pub radius: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Frame {
    pub time: f64,
    pub truth: EgoState,
    pub estimate: EgoState,
    pub objects: Vec<WorldObject>,
    pub lidar: Vec<Vec2>,
    pub tracks: Vec<Track>,
    pub predictions: Vec<Prediction>,
    pub trajectory: Trajectory,
    pub command: ControlCommand,
    pub emergency: bool,
    pub progress: f64,
    pub clearance: f64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub traffic: Vec<TrafficTelemetry>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation: Option<NavigationStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization: Option<LocalizationDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub traffic_controls: Option<TrafficControlStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stop_signs: Option<StopSignStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intersections: Option<IntersectionStatus>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lane_change: Option<LaneChangeStatus>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Summary {
    pub scenario: String,
    pub seed: u64,
    pub simulated_seconds: f64,
    pub steps: usize,
    pub reached_goal: bool,
    pub collisions: usize,
    pub road_violations: usize,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub traffic_collisions: usize,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub traffic_road_violations: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub traffic_min_clearance: Option<f64>,
    pub min_clearance: f64,
    pub localization_rmse: f64,
    pub localization_max_error: f64,
    pub final_speed: f64,
    pub progress: f64,
    pub emergency_steps: usize,
    pub avoidance_steps: usize,
    pub max_tracks: usize,
    pub passed: bool,
    pub failures: Vec<String>,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub navigation_switches: usize,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub closure_violations: usize,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub signal_violations: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub signal_min_stopline_margin_m: Option<f64>,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub stop_sign_violations: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stop_sign_min_margin_m: Option<f64>,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub intersection_violations: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intersection_min_gap_s: Option<f64>,
}
fn is_zero(n: &usize) -> bool {
    *n == 0
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RouteTransition {
    pub time: f64,
    pub plan: RoutePlan,
    pub estimated_speed: f64,
    pub true_speed: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Run {
    #[serde(default)]
    pub backend: String,
    #[serde(skip)]
    pub sensor_log: Option<SensorLog>,
    pub schema_version: u32,
    pub scenario: Scenario,
    pub route: Route,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation: Option<RoutePlan>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub route_history: Vec<RouteTransition>,
    pub vehicle: VehicleConfig,
    pub frames: Vec<Frame>,
    pub occupied_cells: Vec<Vec2>,
    pub summary: Summary,
}
struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9e3779b97f4a7c15 } else { seed })
    }
    fn noise(&mut self, amplitude: f64) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64 / ((1_u64 << 53) as f64) * 2.0 - 1.0) * amplitude
    }
}
/// First-return planar ray/circle sensor; no object identifiers are emitted.
fn lidar(
    ego: Pose,
    objects: &[WorldObject],
    stamp: f64,
    rng: &mut Rng,
    record_sweep: bool,
) -> LidarScan {
    let mut points = Vec::new();
    let mut rays = record_sweep.then(|| Vec::with_capacity(720));
    for i in 0..720 {
        let angle = -PI + 2.0 * PI * i as f64 / 720.0;
        let direction = Vec2::new(angle.cos(), angle.sin()).rotated(ego.yaw);
        let mut closest = 45.0;
        for object in objects {
            let relative = object.position.minus(ego.position);
            let projection = relative.x * direction.x + relative.y * direction.y;
            let discriminant = object.radius.powi(2)
                - (relative.x.powi(2) + relative.y.powi(2) - projection.powi(2));
            if discriminant >= 0.0 {
                let range = projection - discriminant.sqrt();
                if range > 0.2 && range < closest {
                    closest = range;
                }
            }
        }
        if closest < 45.0 {
            let range = closest + rng.noise(0.015);
            if let Some(rays) = &mut rays {
                rays.push(rustdriving_core::PlanarRayOutcome::Return(points.len()));
            }
            points.push(Vec2::new(angle.cos() * range, angle.sin() * range));
        } else if let Some(rays) = &mut rays {
            rays.push(rustdriving_core::PlanarRayOutcome::Clear);
        }
    }
    LidarScan {
        sweep: rays.map(|rays| rustdriving_core::PlanarLidarSweep {
            calibration: rustdriving_core::PlanarLidarCalibration {
                ray_count: 720,
                first_angle_rad: -PI,
                angular_step_rad: std::f64::consts::TAU / 720.,
                min_range_m: 0.2,
                max_range_m: 45.,
                range_error_m: 0.05,
            },
            rays,
        }),
        stamp,
        points,
    }
}
/// Kinematic bicycle, semi-implicit integration; physical command limits enforced here too.
fn step_vehicle(ego: &mut EgoState, command: ControlCommand, config: VehicleConfig, dt: f64) {
    ego.speed = (ego.speed + command.acceleration.clamp(-6.0, 2.0) * dt).clamp(0.0, 12.0);
    let yaw_rate = ego.speed / config.wheelbase
        * command
            .steering
            .clamp(-config.max_steer, config.max_steer)
            .tan();
    let mid_yaw = ego.pose.yaw + yaw_rate * dt / 2.0;
    ego.pose.position = ego
        .pose
        .position
        .plus(Vec2::new(mid_yaw.cos(), mid_yaw.sin()).scaled(ego.speed * dt));
    ego.pose.yaw = wrap_angle(ego.pose.yaw + yaw_rate * dt);
}
fn swept_distance(a: Vec2, b: Vec2) -> f64 {
    let d = b.minus(a);
    let length = d.x * d.x + d.y * d.y;
    if length < 1e-12 {
        return a.x.hypot(a.y);
    }
    let t = (-(a.x * d.x + a.y * d.y) / length).clamp(0.0, 1.0);
    let nearest = a.plus(d.scaled(t));
    nearest.x.hypot(nearest.y)
}
/// Physics/sensor adapter boundary. The evaluator observes truth; the pipeline cannot.
pub trait SimulationBackend {
    fn state(&self) -> EgoState;
    fn objects(&self, time: f64) -> Vec<WorldObject>;
    fn traffic(&self) -> Vec<TrafficTelemetry> {
        vec![]
    }
    fn observe(&mut self, time: f64, tick: usize) -> Result<SensorFrame, String>;
    fn advance(&mut self, command: ControlCommand, dt: f64) -> Result<(), String>;
}
struct ReferenceBackend {
    scenario: Scenario,
    traffic: TrafficWorld,
    truth: EgoState,
    vehicle: VehicleConfig,
    rng: Rng,
    seed: u64,
    command: ControlCommand,
}
impl SimulationBackend for ReferenceBackend {
    fn state(&self) -> EgoState {
        self.truth
    }
    fn objects(&self, time: f64) -> Vec<WorldObject> {
        self.traffic.objects(time)
    }
    fn traffic(&self) -> Vec<TrafficTelemetry> {
        self.traffic.telemetry()
    }
    fn observe(&mut self, time: f64, tick: usize) -> Result<SensorFrame, String> {
        let odometry = Some(if tick == 0 {
            Odometry {
                stamp: time,
                speed: 0.0,
                yaw_rate: 0.0,
            }
        } else {
            Odometry {
                stamp: time,
                speed: self.truth.speed + self.rng.noise(0.015),
                yaw_rate: self.truth.speed / self.vehicle.wheelbase * self.command.steering.tan()
                    + self.rng.noise(0.001),
            }
        });
        let gnss = if tick.is_multiple_of(4) && self.scenario.gnss_available(time) {
            Some(Gnss {
                stamp: time,
                position: self.scenario.gnss_antenna_position(self.truth.pose).plus(
                    self.scenario
                        .gnss_noise(self.seed, tick)
                        .unwrap_or_else(|| Vec2::new(self.rng.noise(0.14), self.rng.noise(0.14))),
                ),
                variance: 0.02,
            })
        } else {
            None
        };
        let lidar = if tick.is_multiple_of(2)
            && self.scenario.lidar_dropout.is_none_or(|t| time < t)
        {
            Some(lidar(
                self.truth.pose,
                &self.objects(time),
                time,
                &mut self.rng,
                self.scenario.record_planar_sweep || self.scenario.clear_sweep_recovery.is_some(),
            ))
        } else {
            None
        };
        Ok(SensorFrame {
            absolute_heading: self.scenario.heading_observation(
                self.seed,
                tick,
                time,
                self.truth.pose.yaw,
            ),
            time,
            odometry,
            gnss,
            lidar,
            multi_height_lidar: None,
            lidar3d: None,
            lidar_failed: false,
            navigation_update: None,
            traffic_signal: None,
        })
    }
    fn advance(&mut self, command: ControlCommand, dt: f64) -> Result<(), String> {
        self.traffic.advance(self.truth, self.vehicle, dt)?;
        self.command = command;
        step_vehicle(&mut self.truth, command, self.vehicle, dt);
        Ok(())
    }
}
pub fn pipeline_config(scenario: &Scenario) -> PipelineConfig {
    let route = scenario.route();
    let (position, yaw) = route.sample(0.0, 0.0);
    let mut config = PipelineConfig::new(route, Pose { position, yaw }, VehicleConfig::default());
    config.initial_pose_covariance = scenario.initial_pose_covariance;
    config.gnss_covariance_shape = scenario.gnss_covariance_shape;
    config.gnss_antenna_offset_m = scenario.gnss_antenna_offset_m;
    config.gnss_recovery = scenario.gnss_recovery;
    config.absolute_heading_fusion = scenario.absolute_heading_fusion;
    config.heading_latency_compensation = scenario.heading_latency_compensation;
    config.relocalization = scenario.relocalization;
    config.confirm_slow_motion = scenario.confirm_slow_motion;
    config.confirm_track_initialization = scenario.confirm_track_initialization;
    config.fit_recent_track_motion = scenario.fit_recent_track_motion;
    config.fit_sparse_circles = scenario.fit_sparse_circles;
    config.forecast_pose_sigma = scenario.forecast_pose_sigma;
    config.forecast_cold_max_speed_m_s = scenario.forecast_cold_max_speed_m_s;
    config.forecast_recent_velocity_hypotheses = scenario.forecast_recent_velocity_hypotheses;
    config.enclose_track_measurements = scenario.enclose_track_measurements;
    config.forecast_envelope = scenario.forecast_envelope;
    config.forecast_consistency_guard = scenario.forecast_consistency_guard;
    config.clear_sweep_recovery = scenario.clear_sweep_recovery;
    config.settle_obstruction_stops = scenario.settle_obstruction_stops;
    config.search_obstruction_stops = scenario.search_obstruction_stops;
    config.replan_checked_approaches = scenario.replan_checked_approaches;
    config.search_later_obstruction_stops = scenario.search_later_obstruction_stops;
    config.lateral_candidate_margin_m = scenario.lateral_candidate_margin_m;
    config.refine_lateral_candidates = scenario.refine_lateral_candidates;
    config.record_planning_diagnostics = scenario.record_planning_diagnostics;
    config.monitor_planning_progress = scenario.monitor_planning_progress;
    config.gnss_latency_compensation = scenario.gnss_latency_compensation;
    config.speed_zones = scenario.speed_zones.clone();
    if let Some(speed) = scenario.cruise_speed {
        config.cruise_speed = speed;
    }
    config.motion_limits = scenario.motion_limits;
    config.local_route_geometry = scenario.local_route_geometry;
    config.localization_map = scenario.localization_map.clone();
    config.stop_signs = scenario.stop_signs.clone();
    config.yield_intersections = scenario.yield_intersections.clone();
    config.lane_change = scenario.lane_change.clone();
    config.stop_lines = scenario
        .traffic_signals
        .iter()
        .map(|s| s.stop_line.clone())
        .collect();
    // Keep legacy static logs unchanged. Restricted maps also record navigation
    // so replay recomputes the legal route instead of trusting resolved geometry.
    if !scenario.navigation_updates.is_empty()
        || scenario.navigation.as_ref().is_some_and(|nav| {
            nav.hold_for_handover
                || nav.avoid_observed_obstructions
                || matches!(nav.start, rustdriving_routing::RouteEndpoint::Edge(_))
                || matches!(nav.goal, rustdriving_routing::RouteEndpoint::Edge(_))
                || !nav.network.turn_restrictions.is_empty()
                || nav
                    .network
                    .edges
                    .iter()
                    .any(|edge| edge.speed_limit_m_s.is_some())
        })
    {
        config.navigation = scenario.navigation.as_ref().map(|nav| NavigationConfig {
            network: nav.network.clone(),
            start: nav.start.clone(),
            goal: nav.goal.clone(),
            closed_edges: nav.closed_edges.clone(),
            hold_for_handover: nav.hold_for_handover,
            avoid_observed_obstructions: nav.avoid_observed_obstructions,
        });
    }
    config
}
pub fn simulate(scenario: Scenario, seed: u64) -> Result<Run, String> {
    scenario.validate()?;
    if scenario.dynamics.is_some() {
        return Err("dynamic plant calibration requires RNE --plant dynamic".into());
    }
    let config = pipeline_config(&scenario);
    let backend = ReferenceBackend {
        scenario: scenario.clone(),
        traffic: TrafficWorld::new(scenario.clone(), config.route.clone()),
        truth: EgoState {
            pose: scenario.spawn_pose(config.initial_pose),
            speed: 0.0,
        },
        vehicle: config.vehicle,
        rng: Rng::new(seed),
        seed,
        command: ControlCommand::default(),
    };
    simulate_with_backend(scenario, seed, backend, config, "reference-2d")
}
/// Score any backend through the exact same sensor-only stack and acceptance evaluator.
pub fn simulate_with_backend(
    scenario: Scenario,
    seed: u64,
    mut backend: impl SimulationBackend,
    config: PipelineConfig,
    source: &str,
) -> Result<Run, String> {
    scenario.validate()?;
    let dt = config.nominal_dt;
    if scenario.sensor_timing.is_some() && (dt - 0.05).abs() > 1e-9 {
        return Err("sensor timing requires the 20 Hz simulation clock".into());
    }
    let mut route = config.route.clone();
    let mut navigation = scenario.navigation_plan()?;
    let mapped_speed_limits = scenario.navigation.as_ref().is_some_and(|nav| {
        nav.network
            .edges
            .iter()
            .any(|edge| edge.speed_limit_m_s.is_some())
    });
    let mut route_speed_zones = if mapped_speed_limits {
        speed_zones::road_zones(
            &scenario.navigation.as_ref().unwrap().network,
            navigation.as_ref().unwrap(),
        )
    } else {
        scenario.speed_zones.clone()
    };
    if let Some(plan) = &navigation
        && (plan.route.points != route.points || plan.route.half_width != route.half_width)
    {
        return Err("backend route differs from the selected navigation route".into());
    }
    let vehicle = config.vehicle;
    let mut pipeline = DrivingPipeline::new(config.clone())?;
    let mut sensor_log = SensorLog::new(source, config);
    let mut scan = LidarScan {
        sweep: None,
        stamp: 0.0,
        points: vec![],
    };
    let mut frames = Vec::new();
    let mut collisions = 0;
    let mut road_violations = 0;
    let mut traffic_collisions = 0;
    let mut traffic_road_violations = 0;
    let traffic_route = scenario.route();
    let mut traffic_minimum: Option<f64> = None;
    let mut minimum = 1000.0_f64;
    let mut error_sum = 0.0;
    let mut error_max = 0.0_f64;
    let mut emergency_steps = 0;
    let mut avoidance_steps = 0;
    let mut max_tracks = 0;
    let mut steps = 0;
    let mut reached_goal = false;
    let mut goal_since = None;
    let mut update_index = 0;
    let mut route_history = Vec::new();
    let mut navigation_switches = 0;
    let mut closure_violations = 0;
    let mut signal_rules = RuleEvaluator::default();
    let mut stop_rules = StopRuleEvaluator::new(scenario.stop_signs.len());
    let mut intersection_rules = IntersectionRuleEvaluator::new(scenario.yield_intersections.len());
    let mut speed_rules = speed_zones::SpeedZoneEvaluator::default();
    let mut speed_policy = speed_zones::AlternateSpeedPolicy::new(
        scenario
            .lane_change
            .as_ref()
            .and_then(|gate| gate.return_on_conflict.as_ref())
            .and_then(|config| config.speed_limit_m_s),
    );
    let mut sensor_delivery = SensorDelivery::default();
    let mut heading_delivery = heading::HeadingDelivery::default();
    let mut evaluation_closures = scenario
        .navigation
        .as_ref()
        .map_or_else(Vec::new, |nav| nav.closed_edges.clone());
    if !scenario.navigation_updates.is_empty() {
        route_history.push(RouteTransition {
            time: 0.0,
            plan: navigation.clone().unwrap(),
            estimated_speed: 0.0,
            true_speed: backend.state().speed,
        });
    }
    for i in 0..=(scenario.duration / dt).round() as usize {
        let time = i as f64 * dt;
        let truth = backend.state();
        let objects = backend.objects(time);
        let traffic = backend.traffic();
        let reactive_ids: Vec<_> = traffic.iter().map(|actor| actor.id).collect();
        let mut traffic_collided = false;
        for (i, a) in objects.iter().enumerate() {
            for b in &objects[i + 1..] {
                if !reactive_ids.contains(&a.id) && !reactive_ids.contains(&b.id) {
                    continue;
                }
                let separation = a.position.distance(b.position) - a.radius - b.radius;
                traffic_minimum = Some(traffic_minimum.map_or(separation, |v| v.min(separation)));
                traffic_collided |= separation < 0.0;
            }
        }
        speed_policy.observe(
            &mut speed_rules,
            &route,
            truth,
            vehicle.radius,
            &route_speed_zones,
        );
        // Evaluate true physical crossings against the external phase schedule.
        // Neither this evaluator nor its phase schedule enters the driver.
        signal_rules.observe(
            &scenario.traffic_signals,
            time,
            traffic_route.project(truth.pose.position).0 + vehicle.radius,
        );
        stop_rules.observe(
            &scenario.stop_signs,
            time,
            traffic_route.project(truth.pose.position).0 + vehicle.radius,
            truth.speed,
        );
        intersection_rules.observe(
            &scenario.yield_intersections,
            time,
            truth.pose.position,
            vehicle.radius,
            &objects,
        );
        let mut input = backend.observe(time, i)?;
        if let Some(sensor) = &scenario.heading_sensor {
            heading_delivery.apply(sensor.delay_ticks, i, &mut input.absolute_heading);
        }
        if let Some(timing) = &scenario.sensor_timing {
            sensor_delivery.apply(timing, i, &mut input);
        }
        input.traffic_signal = signals::observe(&scenario, time, i);
        // Fault injection changes observations only, using the acquisition clock.
        // The pipeline/replay header does not contain the scheduled fault labels.
        if let Some(fix) = &mut input.gnss {
            for window in &scenario.gnss_bias_windows {
                if fix.stamp + 1e-9 >= window.from && fix.stamp < window.until - 1e-9 {
                    fix.position = fix.position.plus(window.offset);
                }
            }
        }
        if let Some(update) = scenario.navigation_updates.get(update_index)
            && update.stamp <= time + 1e-9
        {
            input.navigation_update = Some(update.clone());
            evaluation_closures = update.closed_edges.clone();
            update_index += 1;
        }
        if let Some(new_scan) = &input.lidar {
            scan = new_scan.clone();
        }
        let result = pipeline.step(&input)?;
        let estimate = result.estimate;
        let tracks = result.tracks.clone();
        let predictions = result.predictions.clone();
        let trajectory = result.trajectory.clone();
        let command = result.command;
        let emergency = result.emergency;
        let navigation_status = result.navigation.clone();
        let localization_status = result.localization;
        let traffic_control_status = result.traffic_controls.clone();
        let stop_sign_status = result.stop_signs.clone();
        let intersection_status = result.intersections.clone();
        let lane_change_status = result.lane_change.clone();
        speed_policy.update(
            &mut speed_rules,
            &route,
            truth,
            vehicle,
            &scenario.speed_zones,
            lane_change_status.as_ref(),
        )?;
        if let Some(status) = &navigation_status
            && status.switches > navigation_switches
        {
            route = pipeline.active_route().clone();
            navigation = pipeline.navigation_plan().cloned();
            route_history.push(RouteTransition {
                time,
                plan: navigation.clone().unwrap(),
                estimated_speed: estimate.speed,
                true_speed: truth.speed,
            });
            navigation_switches = status.switches;
            if mapped_speed_limits {
                route_speed_zones = speed_zones::road_zones(
                    &scenario.navigation.as_ref().unwrap().network,
                    navigation.as_ref().unwrap(),
                );
                speed_rules.switch_route(&route, truth, vehicle.radius, &route_speed_zones);
            }
        }
        // Score external closures independently of the navigator's reported closure state.
        if !scenario.navigation_updates.is_empty()
            && let (Some(plan), Some(nav)) = (&navigation, &scenario.navigation)
        {
            let progress = route.project(truth.pose.position).0;
            let mut start_s = 0.0;
            for (edge_index, id) in plan.edge_ids.iter().enumerate() {
                let edge = nav.network.edges.iter().find(|e| &e.id == id).unwrap();
                if evaluation_closures.contains(id) && progress + vehicle.radius >= start_s {
                    closure_violations += 1;
                    break;
                }
                start_s += plan.traversals.get(edge_index).map_or_else(
                    || {
                        edge.points
                            .windows(2)
                            .map(|p| p[0].distance(p[1]))
                            .sum::<f64>()
                    },
                    |span| span.end_s_m - span.start_s_m,
                );
            }
        }
        sensor_log.record(input, result);
        if emergency {
            emergency_steps += 1;
        }
        if trajectory.mode == DrivingMode::Avoid {
            avoidance_steps += 1;
        }
        max_tracks = max_tracks.max(tracks.len());
        let (progress, lateral) = route.project(truth.pose.position);
        let off_road = if let Some(gate) = scenario
            .lane_change
            .as_ref()
            .filter(|c| c.return_on_conflict.is_some())
        {
            lane_change_surface::circle_margin(&route, gate, truth.pose.position, vehicle.radius)
                < 0.0
        } else {
            lateral.abs() + vehicle.radius > route.half_width
        };
        if off_road {
            road_violations += 1;
        }
        let error = truth.pose.position.distance(estimate.pose.position);
        error_sum += error.powi(2);
        error_max = error_max.max(error);
        steps += 1;
        let clearance = objects
            .iter()
            .map(|o| truth.pose.position.distance(o.position) - vehicle.radius - o.radius)
            .fold(1000.0, f64::min);
        minimum = minimum.min(clearance);
        let at_map_goal = navigation.as_ref().is_none_or(|plan| {
            truth
                .pose
                .position
                .distance(*plan.route.points.last().unwrap())
                <= 2.0
        });
        if progress >= route.length() - 2.0 && truth.speed < 0.2 && at_map_goal {
            let since = *goal_since.get_or_insert(time);
            reached_goal = time - since + 1e-9 >= scenario.goal_hold_seconds.unwrap_or(0.0);
        } else {
            goal_since = None;
        }
        let finished = reached_goal || time >= scenario.duration;
        for actor in &traffic {
            let object = objects
                .iter()
                .find(|o| o.id == actor.id)
                .ok_or("traffic telemetry has no corresponding physical body")?;
            if traffic_route.project(object.position).1.abs() + object.radius
                > traffic_route.half_width
                || actor.route_s_m + object.radius > traffic_route.length() + 1e-8
            {
                traffic_road_violations += 1;
            }
        }
        if scenario.full_rate_recording
            || !scenario.speed_zones.is_empty()
            || scenario.gnss_latency_compensation
            || scenario.sensor_timing.as_ref().is_some_and(|timing| {
                timing.gnss_delay_ticks > 0
                    || timing.odometry_delay_ticks > 0
                    || !timing.odometry_dropout_windows.is_empty()
            })
            || mapped_speed_limits
            || scenario.navigation.as_ref().is_some_and(|nav| {
                matches!(nav.start, rustdriving_routing::RouteEndpoint::Edge(_))
                    || matches!(nav.goal, rustdriving_routing::RouteEndpoint::Edge(_))
            })
            || scenario.lane_change.is_some()
            || !traffic.is_empty()
            || !scenario.traffic_signals.is_empty()
            || !scenario.stop_signs.is_empty()
            || !scenario.yield_intersections.is_empty()
            || scenario.localization_map.is_some()
            || !scenario.gnss_dropout_windows.is_empty()
            || i.is_multiple_of(2)
            || finished
            || goal_since == Some(time)
        {
            frames.push(Frame {
                time,
                truth,
                estimate,
                objects: objects.clone(),
                lidar: scan.points.clone(),
                tracks: tracks.clone(),
                predictions,
                trajectory,
                command,
                emergency,
                progress,
                clearance,
                traffic,
                navigation: navigation_status,
                localization: localization_status,
                traffic_controls: traffic_control_status,
                stop_signs: stop_sign_status,
                intersections: intersection_status,
                lane_change: lane_change_status,
            });
        }
        if finished {
            traffic_collisions += usize::from(traffic_collided);
            collisions += usize::from(clearance < 0.0);
            break;
        }
        let previous = truth.pose.position;
        backend.advance(command, dt)?;
        let next_truth = backend.state();
        let next_objects = backend.objects(time + dt);
        let next_reactive_ids: Vec<_> = backend.traffic().iter().map(|actor| actor.id).collect();
        for (i, a) in next_objects.iter().enumerate() {
            for b in &next_objects[i + 1..] {
                if !next_reactive_ids.contains(&a.id) && !next_reactive_ids.contains(&b.id) {
                    continue;
                }
                let separation = if let (Some(old_a), Some(old_b)) = (
                    objects.iter().find(|o| o.id == a.id),
                    objects.iter().find(|o| o.id == b.id),
                ) {
                    swept_distance(
                        old_a.position.minus(old_b.position),
                        a.position.minus(b.position),
                    )
                } else {
                    a.position.distance(b.position)
                } - a.radius
                    - b.radius;
                traffic_minimum = Some(traffic_minimum.map_or(separation, |v| v.min(separation)));
                traffic_collided |= separation < 0.0;
            }
        }
        traffic_collisions += usize::from(traffic_collided);
        let mut collided = clearance < 0.0;
        for next in &next_objects {
            let separation = if let Some(object) = objects.iter().find(|o| o.id == next.id) {
                swept_distance(
                    previous.minus(object.position),
                    next_truth.pose.position.minus(next.position),
                )
            } else {
                // Newly active actor: never interpolate it backward before existence.
                next_truth.pose.position.distance(next.position)
            } - vehicle.radius
                - next.radius;
            minimum = minimum.min(separation);
            collided |= separation < 0.0;
        }
        collisions += usize::from(collided);
    }
    let truth = backend.state();
    let progress = route.project(truth.pose.position).0;
    let mut failures = Vec::new();
    if speed_rules.violations != 0 {
        failures.push(format!(
            "{} physical speed-zone violation intervals (maximum excess {:.6} m/s, tolerance {:.3} m/s)",
            speed_rules.violations, speed_rules.maximum_excess_m_s, speed_zones::SPEED_TOLERANCE_M_S,
        ));
    }
    if collisions > 0 {
        failures.push(format!("{collisions} colliding integration steps"));
    }
    if traffic_road_violations > 0 {
        failures.push(format!(
            "{traffic_road_violations} traffic road boundary violations"
        ));
    }
    if traffic_collisions > 0 {
        failures.push(format!(
            "{traffic_collisions} colliding traffic integration steps"
        ));
    }
    if road_violations > 0 {
        failures.push(format!("{road_violations} road boundary violations"));
    }
    if closure_violations > 0 {
        failures.push(format!("{closure_violations} closed-edge entry violations"));
    }
    if let Some(required) = scenario.min_clearance_m
        && minimum < required
    {
        failures.push(format!(
            "minimum swept clearance {minimum:.3} m is below {required:.3} m"
        ));
    }
    if let (Some(required), Some(measured)) = (scenario.min_clearance_m, traffic_minimum)
        && measured < required
    {
        failures.push(format!(
            "traffic swept clearance {measured:.3} m is below {required:.3} m"
        ));
    }
    if error_max > 1.0 {
        failures.push(format!(
            "localization max error {error_max:.3} m exceeds 1 m"
        ));
    }
    intersection_rules.finish((steps - 1) as f64 * dt);
    if intersection_rules.violations != 0 {
        failures.push(format!(
            "{} physical intersection priority violations (required temporal gap 2.0 s)",
            intersection_rules.violations
        ));
    }
    if signal_rules.violations != 0 {
        failures.push(format!(
            "{} nonpermissive physical stop-line crossings",
            signal_rules.violations
        ));
    }
    if stop_rules.violations != 0 {
        failures.push(format!(
            "{} physical stop-sign crossings without a complete stop",
            stop_rules.violations
        ));
    }
    match scenario.expected {
        Expected::Goal if !reached_goal => failures.push("goal not reached within duration".into()),
        Expected::Stop if truth.speed > 0.2 || reached_goal || progress < 10.0 => {
            failures.push("blocked-road stop did not meet acceptance criteria".into())
        }
        Expected::Fault if emergency_steps == 0 || truth.speed > 0.2 || reached_goal => {
            failures.push("sensor-fault braking did not meet acceptance criteria".into())
        }
        _ => {}
    }
    let summary = Summary {
        scenario: scenario.name.clone(),
        seed,
        simulated_seconds: (steps - 1) as f64 * dt,
        steps,
        reached_goal,
        collisions,
        road_violations,
        traffic_collisions,
        traffic_road_violations,
        traffic_min_clearance: traffic_minimum,
        min_clearance: minimum,
        localization_rmse: (error_sum / steps as f64).sqrt(),
        localization_max_error: error_max,
        final_speed: truth.speed,
        progress,
        emergency_steps,
        avoidance_steps,
        max_tracks,
        passed: failures.is_empty(),
        failures,
        navigation_switches,
        closure_violations,
        signal_violations: signal_rules.violations,
        signal_min_stopline_margin_m: signal_rules.minimum_nonpermissive_margin_m,
        stop_sign_violations: stop_rules.violations,
        stop_sign_min_margin_m: stop_rules.minimum_unreleased_margin_m,
        intersection_violations: intersection_rules.violations,
        intersection_min_gap_s: intersection_rules.minimum_gap_s,
    };
    Ok(Run {
        backend: source.to_string(),
        sensor_log: Some(sensor_log),
        schema_version: 1,
        scenario,
        route,
        navigation,
        route_history,
        vehicle,
        frames,
        occupied_cells: pipeline.occupied_cells(),
        summary,
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reference_ray_capture_preserves_acquisition_noise_and_driving_bytes() {
        for seed in [7, 42] {
            let mut s: Scenario =
                serde_json::from_str(include_str!("../../../scenarios/blocked.json")).unwrap();
            s.duration = 1.;
            s.full_rate_recording = true;
            let old = simulate(s.clone(), seed).unwrap();
            s.record_planar_sweep = true;
            let captured = simulate(s, seed).unwrap();
            assert_eq!(
                serde_json::to_value(&old.frames).unwrap(),
                serde_json::to_value(&captured.frames).unwrap()
            );
            for (a, b) in old
                .sensor_log
                .as_ref()
                .unwrap()
                .ticks
                .iter()
                .zip(&captured.sensor_log.as_ref().unwrap().ticks)
            {
                assert_eq!(
                    serde_json::to_value(&a.expected).unwrap(),
                    serde_json::to_value(&b.expected).unwrap()
                );
                if let Some(scan) = &b.input.lidar {
                    assert_eq!(a.input.lidar.as_ref().unwrap().points, scan.points);
                    assert!(scan.sweep.as_ref().unwrap().calibration.angular_step_rad > 0.);
                    assert_eq!(scan.sweep.as_ref().unwrap().rays.len(), 720);
                    assert!(
                        !b.expected
                            .health
                            .contains(&rustdriving_pipeline::HealthIssue::InvalidLidar)
                    );
                }
            }
            let mut bytes = vec![];
            captured.sensor_log.unwrap().write(&mut bytes).unwrap();
            assert!(
                rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), vec![])
                    .unwrap()
                    .verified
            );
        }
    }
    #[test]
    fn static_mission_records_an_explicit_handover_policy_for_replay() {
        let mut scenario: Scenario =
            serde_json::from_str(include_str!("../../../scenarios/route-direct.json")).unwrap();
        assert!(scenario.navigation_updates.is_empty());
        assert!(pipeline_config(&scenario).navigation.is_none());
        scenario.navigation.as_mut().unwrap().hold_for_handover = true;
        let config = pipeline_config(&scenario);
        assert!(config.navigation.as_ref().unwrap().hold_for_handover);
        assert!(config.validate().is_ok());
        let encoded = serde_json::to_value(config).unwrap();
        assert_eq!(encoded["navigation"]["hold_for_handover"], true);
        assert!(encoded.get("navigation_updates").is_none());
    }
    #[test]
    fn gnss_acquisition_windows_are_bounded_and_recover_at_the_end_boundary() {
        let mut s: Scenario =
            serde_json::from_str(include_str!("../../../scenarios/mission.json")).unwrap();
        let original = serde_json::to_value(&s).unwrap();
        assert!(original.get("gnss_dropout_windows").is_none());
        assert!(original.get("localization_map").is_none());
        s.gnss_dropout_windows = vec![GnssDropoutWindow {
            from: 3.0,
            until: 8.0,
        }];
        assert!(s.validate().is_ok());
        assert!(s.gnss_available(2.8));
        assert!(!s.gnss_available(3.0));
        assert!(!s.gnss_available(7.8));
        assert!(s.gnss_available(8.0));
        for window in [
            GnssDropoutWindow {
                from: -1.0,
                until: 1.0,
            },
            GnssDropoutWindow {
                from: 3.0,
                until: 3.0,
            },
            GnssDropoutWindow {
                from: f64::NAN,
                until: 8.0,
            },
            GnssDropoutWindow {
                from: 3.0,
                until: s.duration + 1.0,
            },
        ] {
            s.gnss_dropout_windows = vec![window];
            assert!(s.validate().is_err());
        }
        s.gnss_dropout_windows = vec![
            GnssDropoutWindow {
                from: 3.0,
                until: 8.0,
            },
            GnssDropoutWindow {
                from: 7.0,
                until: 9.0,
            },
        ];
        assert!(s.validate().is_err());
    }
    #[test]
    fn opaque_adapter_body_ids_preserve_traffic_acceptance() {
        struct Renamed(ReferenceBackend);
        impl SimulationBackend for Renamed {
            fn state(&self) -> EgoState {
                self.0.state()
            }
            fn objects(&self, time: f64) -> Vec<WorldObject> {
                self.0
                    .objects(time)
                    .into_iter()
                    .map(|mut o| {
                        o.id += 100;
                        o
                    })
                    .collect()
            }
            fn traffic(&self) -> Vec<TrafficTelemetry> {
                self.0
                    .traffic()
                    .into_iter()
                    .map(|mut a| {
                        a.id += 100;
                        a
                    })
                    .collect()
            }
            fn observe(&mut self, time: f64, tick: usize) -> Result<SensorFrame, String> {
                self.0.observe(time, tick)
            }
            fn advance(&mut self, command: ControlCommand, dt: f64) -> Result<(), String> {
                self.0.advance(command, dt)
            }
        }
        let scenario: Scenario =
            serde_json::from_str(include_str!("../../../scenarios/traffic-queue.json")).unwrap();
        let original = simulate(scenario.clone(), 7).unwrap();
        let config = pipeline_config(&scenario);
        let backend = ReferenceBackend {
            scenario: scenario.clone(),
            traffic: TrafficWorld::new(scenario.clone(), config.route.clone()),
            truth: EgoState {
                pose: config.initial_pose,
                speed: 0.0,
            },
            vehicle: config.vehicle,
            rng: Rng::new(7),
            seed: 7,
            command: ControlCommand::default(),
        };
        let renamed =
            simulate_with_backend(scenario, 7, Renamed(backend), config, "renamed-reference")
                .unwrap();
        assert!(renamed.summary.passed);
        assert_eq!(
            serde_json::to_value(original.summary).unwrap(),
            serde_json::to_value(renamed.summary).unwrap()
        );
    }
    #[test]
    fn sensor_has_occlusion_and_no_labels() {
        let objects = vec![
            WorldObject {
                id: 99,
                position: Vec2::new(10.0, 0.0),
                radius: 1.0,
            },
            WorldObject {
                id: 100,
                position: Vec2::new(20.0, 0.0),
                radius: 1.0,
            },
        ];
        let scan = lidar(Pose::default(), &objects, 0.0, &mut Rng::new(1), false);
        let center = scan.points.iter().find(|p| p.y.abs() < 1e-6).unwrap();
        assert!((center.x - 9.0).abs() < 0.02);
    }
    #[test]
    fn bicycle_turns_and_brakes() {
        let mut e = EgoState {
            speed: 5.0,
            ..EgoState::default()
        };
        step_vehicle(
            &mut e,
            ControlCommand {
                acceleration: 0.0,
                steering: 0.2,
            },
            VehicleConfig::default(),
            0.1,
        );
        assert!(e.pose.yaw > 0.0);
        for _ in 0..30 {
            step_vehicle(
                &mut e,
                ControlCommand::emergency(),
                VehicleConfig::default(),
                0.1,
            );
        }
        assert_eq!(e.speed, 0.0);
    }
}
