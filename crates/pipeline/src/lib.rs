//! Shared sensor-to-command stack. No simulator, truth objects or physical world types.
pub mod navigation;
pub mod replay;
use navigation::{
    NavigationConfig, NavigationPhase, NavigationStatus, NavigationUpdate, Navigator,
};
use rustdrive_control::{PurePursuit, guard};
use rustdrive_core::*;
use rustdrive_localization::Ekf;
use rustdrive_mapping::OccupancyGrid;
use rustdrive_perception::{LidarClusters, Tracker};
use rustdrive_planning::LatticePlanner;
use rustdrive_prediction::ConstantVelocity;
use serde::{Deserialize, Serialize};

/// Conservative, externally calibrated planning limits, independent of simulator truth.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotionLimits {
    #[serde(default = "default_acceleration")]
    pub max_acceleration_m_s2: f64,
    pub max_deceleration_m_s2: f64,
    pub max_lateral_acceleration_m_s2: f64,
}
fn default_acceleration() -> f64 {
    2.0
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PipelineConfig {
    pub route: Route,
    pub initial_pose: Pose,
    pub vehicle: VehicleConfig,
    pub nominal_dt: f64,
    pub cruise_speed: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion_limits: Option<MotionLimits>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation: Option<NavigationConfig>,
}
impl PipelineConfig {
    pub fn new(route: Route, initial_pose: Pose, vehicle: VehicleConfig) -> Self {
        Self {
            route,
            initial_pose,
            vehicle,
            nominal_dt: 0.05,
            cruise_speed: 8.0,
            motion_limits: None,
            navigation: None,
        }
    }
    pub fn validate(&self) -> Result<(), String> {
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
    pub lidar: Option<LidarScan>,
    /// Explicit adapter acquisition failure. Empty returns are otherwise valid.
    #[serde(default)]
    pub lidar_failed: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation_update: Option<NavigationUpdate>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum HealthIssue {
    MissingOdometry,
    StaleOdometry,
    StaleLidar,
    StaleGnss,
    InvalidOdometry,
    InvalidLidar,
    InvalidGnss,
    LocalizationUncertain,
    AcquisitionFailed,
    ClockGap,
    InvalidNavigation,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PipelineOutput {
    pub time: f64,
    pub estimate: EgoState,
    pub tracks: Vec<Track>,
    pub predictions: Vec<Prediction>,
    pub trajectory: Trajectory,
    pub command: ControlCommand,
    pub emergency: bool,
    pub health: Vec<HealthIssue>,
    pub position_variance: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization: Option<LocalizationDiagnostics>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation: Option<NavigationStatus>,
}
/// Owns algorithm state; adapters supply observations and apply resulting commands.
pub struct DrivingPipeline {
    config: PipelineConfig,
    ekf: Ekf,
    perception: LidarClusters,
    tracker: Tracker,
    predictor: ConstantVelocity,
    planner: LatticePlanner,
    controller: PurePursuit,
    grid: OccupancyGrid,
    previous_time: Option<f64>,
    last_odom: Option<Odometry>,
    last_lidar: Option<f64>,
    tracks: Vec<Track>,
    navigator: Option<Navigator>,
}
impl DrivingPipeline {
    pub fn new(config: PipelineConfig) -> Result<Self, String> {
        config.validate()?;
        let mut planner = LatticePlanner::default();
        planner.cruise_speed = config.cruise_speed;
        planner.vehicle = config.vehicle;
        if let Some(limits) = config.motion_limits {
            planner.max_acceleration_m_s2 = limits.max_acceleration_m_s2;
            planner.max_deceleration_m_s2 = limits.max_deceleration_m_s2;
            planner.max_lateral_acceleration_m_s2 = Some(limits.max_lateral_acceleration_m_s2);
        }
        let controller = PurePursuit::with_vehicle(config.vehicle);
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
        let ekf = Ekf::new(config.initial_pose);
        let navigator = config.navigation.clone().map(Navigator::new).transpose()?;
        Ok(Self {
            config,
            ekf,
            perception: LidarClusters,
            tracker: Tracker::default(),
            predictor: ConstantVelocity::default(),
            planner,
            controller,
            grid,
            previous_time: None,
            last_odom: None,
            last_lidar: None,
            tracks: vec![],
            navigator,
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
                health.push(HealthIssue::InvalidOdometry);
            } else if self.last_odom.is_none_or(|last| odom.stamp > last.stamp) {
                self.last_odom = Some(odom);
            }
        }
        match self.last_odom {
            None => health.push(HealthIssue::MissingOdometry),
            Some(odom) if input.time - odom.stamp > 0.15 + 1e-9 => {
                health.push(HealthIssue::StaleOdometry)
            }
            Some(odom) if !first && dt <= 0.25 => self.ekf.predict(odom, dt),
            _ => {}
        }
        if let Some(fix) = input.gnss {
            if !stamp_valid(fix.stamp, input.time)
                || !fix.position.finite()
                || !fix.variance.is_finite()
                || fix.variance <= 0.0
            {
                health.push(HealthIssue::InvalidGnss);
            } else {
                self.ekf.update(fix);
            }
        }
        let estimate = self.ekf.state();
        if input.lidar_failed {
            health.push(HealthIssue::AcquisitionFailed);
        }
        if let Some(scan) = &input.lidar {
            if input.lidar_failed
                || !stamp_valid(scan.stamp, input.time)
                || scan.points.len() > 20_000
                || scan
                    .points
                    .iter()
                    .any(|p| !p.finite() || p.x.hypot(p.y) > 200.0)
            {
                health.push(HealthIssue::InvalidLidar);
            } else if self.last_lidar.is_none_or(|last| scan.stamp > last) {
                let detections = self.perception.detect(scan, estimate.pose);
                self.tracks = self.tracker.update(&detections, scan.stamp);
                self.grid.update(scan, estimate.pose);
                self.last_lidar = Some(scan.stamp);
            }
        }
        if self.last_lidar.is_none_or(|t| input.time - t > 0.35 + 1e-9) {
            health.push(HealthIssue::StaleLidar);
        }
        if !self.ekf.last_gnss.is_finite() || input.time - self.ekf.last_gnss > 0.75 + 1e-9 {
            health.push(HealthIssue::StaleGnss);
        }
        let variance = self.ekf.position_variance();
        if !variance.is_finite() || !(0.0..=4.0).contains(&variance) {
            health.push(HealthIssue::LocalizationUncertain);
        }
        let predictions = self.predictor.predict(&self.tracks);
        let mut switched = false;
        if let Some(nav) = &mut self.navigator {
            switched = nav.step(
                input.navigation_update.as_ref(),
                input.time,
                estimate,
                health.is_empty(),
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
            // Keep EKF, sensor ages, tracks and occupancy. Reset only route-dependent actuation state.
            let mut planner = LatticePlanner::default();
            planner.cruise_speed = self.planner.cruise_speed;
            planner.vehicle = self.planner.vehicle;
            planner.max_acceleration_m_s2 = self.planner.max_acceleration_m_s2;
            planner.max_deceleration_m_s2 = self.planner.max_deceleration_m_s2;
            planner.max_lateral_acceleration_m_s2 = self.planner.max_lateral_acceleration_m_s2;
            self.planner = planner;
            self.controller.reset_route_state();
        }
        let stop_route = self.navigator.as_ref().and_then(Navigator::planning_route);
        let mut trajectory = self.planner.plan(
            estimate,
            stop_route.as_ref().unwrap_or(&self.config.route),
            &predictions,
        );
        if self.navigator.as_ref().is_some_and(|nav| {
            matches!(
                nav.status().phase,
                NavigationPhase::Braking | NavigationPhase::Blocked
            )
        }) && trajectory.mode != DrivingMode::Emergency
        {
            trajectory.mode = DrivingMode::Yield;
        }
        let command = if health.is_empty() {
            let requested = self.controller.control(estimate, &trajectory, dt);
            guard(
                requested,
                input.time,
                self.last_lidar.unwrap(),
                self.ekf.last_gnss,
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
        self.previous_time = Some(input.time);
        Ok(PipelineOutput {
            time: input.time,
            estimate,
            tracks: self.tracks.clone(),
            predictions,
            trajectory,
            command,
            emergency,
            health,
            position_variance: variance,
            localization: Some(self.ekf.diagnostics()),
            navigation: self.navigator.as_ref().map(Navigator::status),
        })
    }
    pub fn occupied_cells(&self) -> Vec<Vec2> {
        self.grid.occupied_cells()
    }
    pub fn active_route(&self) -> &Route {
        &self.config.route
    }
    pub fn navigation_plan(&self) -> Option<&rustdrive_routing::RoutePlan> {
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
                stamp: time,
                points: vec![],
            }),
            lidar_failed: false,
            navigation_update: None,
        }
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
        map.nodes.push(rustdrive_routing::RoadNode {
            id: "remote".into(),
            position: Vec2::new(100000.0, 0.0),
        });
        map.edges.push(rustdrive_routing::RoadEdge {
            id: "remote-edge".into(),
            from: "east".into(),
            to: "remote".into(),
            points: vec![Vec2::new(200.0, 0.0), Vec2::new(100000.0, 0.0)],
            half_width: 5.5,
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
