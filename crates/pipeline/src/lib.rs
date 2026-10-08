//! Shared sensor-to-command stack. No simulator, truth objects or physical world types.
pub mod replay;
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
    pub max_deceleration_m_s2: f64,
    pub max_lateral_acceleration_m_s2: f64,
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
        }
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.route.points.len() > 5000 {
            return Err("route exceeds 5000 points".into());
        }
        let canonical = Route::new(self.route.points.clone(), self.route.half_width)?;
        if canonical.lengths != self.route.lengths || canonical.length() > 1500.0 {
            return Err("invalid route arc-length metadata or excessive route length".into());
        }
        if self.motion_limits.is_some_and(|limits| {
            !limits.max_deceleration_m_s2.is_finite()
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
}
impl DrivingPipeline {
    pub fn new(config: PipelineConfig) -> Result<Self, String> {
        config.validate()?;
        let mut planner = LatticePlanner::default();
        planner.cruise_speed = config.cruise_speed;
        planner.vehicle = config.vehicle;
        if let Some(limits) = config.motion_limits {
            planner.max_deceleration_m_s2 = limits.max_deceleration_m_s2;
            planner.max_lateral_acceleration_m_s2 = Some(limits.max_lateral_acceleration_m_s2);
        }
        let controller = PurePursuit::with_vehicle(config.vehicle);
        let min_x = config
            .route
            .points
            .iter()
            .map(|p| p.x)
            .fold(f64::INFINITY, f64::min)
            - 10.0;
        let min_y = config
            .route
            .points
            .iter()
            .map(|p| p.y)
            .fold(f64::INFINITY, f64::min)
            - 20.0;
        let max_x = config
            .route
            .points
            .iter()
            .map(|p| p.x)
            .fold(f64::NEG_INFINITY, f64::max)
            + 20.0;
        let max_y = config
            .route
            .points
            .iter()
            .map(|p| p.y)
            .fold(f64::NEG_INFINITY, f64::max)
            + 20.0;
        let grid = OccupancyGrid::new(
            Vec2::new(min_x, min_y),
            ((max_x - min_x) / 0.5).ceil() as usize,
            ((max_y - min_y) / 0.5).ceil() as usize,
            0.5,
        );
        let ekf = Ekf::new(config.initial_pose);
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
            } else if fix.stamp > self.ekf.last_gnss {
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
        let mut trajectory = self
            .planner
            .plan(estimate, &self.config.route, &predictions);
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
        })
    }
    pub fn occupied_cells(&self) -> Vec<Vec2> {
        self.grid.occupied_cells()
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
    fn invalid_calibration_is_rejected_before_execution() {
        let mut config = PipelineConfig::new(
            Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 5.5).unwrap(),
            Pose::default(),
            VehicleConfig::default(),
        );
        config.motion_limits = Some(MotionLimits {
            max_deceleration_m_s2: 0.0,
            max_lateral_acceleration_m_s2: 1.0,
        });
        assert!(DrivingPipeline::new(config).is_err());
    }
}
