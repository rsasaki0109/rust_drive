//! Deterministic closed-loop simulator. Only sensor observations enter the stack.
use rustdrive_core::*;
use rustdrive_pipeline::replay::SensorLog;
use rustdrive_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};
use rustdrive_routing::{RoadNetwork, RoadNetworkSpec, RoutePlan};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

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
    pub start: String,
    pub goal: String,
    #[serde(default)]
    pub closed_edges: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    pub name: String,
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dynamics: Option<DynamicsSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation: Option<NavigationSpec>,
    /// Independent swept-circle acceptance floor in meters; never a planner input.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_clearance_m: Option<f64>,
    pub objects: Vec<ObjectSpec>,
}
impl Scenario {
    pub fn validate(&self) -> Result<(), String> {
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
        if self
            .min_clearance_m
            .is_some_and(|d| !d.is_finite() || d < 0.0)
        {
            return Err("minimum clearance must be finite and nonnegative".into());
        }
        let selected = self.navigation_plan()?;
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
            {
                return Err("invalid object parameters".into());
            }
        }
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
    pub fn navigation_plan(&self) -> Result<Option<RoutePlan>, String> {
        self.navigation
            .as_ref()
            .map(|nav| {
                RoadNetwork::new(nav.network.clone())?.shortest_route(
                    &nav.start,
                    &nav.goal,
                    &nav.closed_edges,
                )
            })
            .transpose()
    }
    pub fn world_objects(&self, route: &Route, t: f64) -> Vec<WorldObject> {
        self.objects
            .iter()
            .enumerate()
            .filter(|(_, o)| t >= o.active_from)
            .map(|(id, o)| {
                let elapsed = (t - o.active_from.max(o.moving_from)).max(0.0);
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
fn lidar(ego: Pose, objects: &[WorldObject], stamp: f64, rng: &mut Rng) -> LidarScan {
    let mut points = Vec::new();
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
            points.push(Vec2::new(angle.cos() * range, angle.sin() * range));
        }
    }
    LidarScan { stamp, points }
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
    fn observe(&mut self, time: f64, tick: usize) -> Result<SensorFrame, String>;
    fn advance(&mut self, command: ControlCommand, dt: f64) -> Result<(), String>;
}
struct ReferenceBackend {
    scenario: Scenario,
    route: Route,
    truth: EgoState,
    vehicle: VehicleConfig,
    rng: Rng,
    command: ControlCommand,
}
impl SimulationBackend for ReferenceBackend {
    fn state(&self) -> EgoState {
        self.truth
    }
    fn objects(&self, time: f64) -> Vec<WorldObject> {
        self.scenario.world_objects(&self.route, time)
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
        let gnss = if tick.is_multiple_of(4) && self.scenario.gnss_dropout.is_none_or(|t| time < t)
        {
            Some(Gnss {
                stamp: time,
                position: self
                    .truth
                    .pose
                    .position
                    .plus(Vec2::new(self.rng.noise(0.14), self.rng.noise(0.14))),
                variance: 0.02,
            })
        } else {
            None
        };
        let lidar =
            if tick.is_multiple_of(2) && self.scenario.lidar_dropout.is_none_or(|t| time < t) {
                Some(lidar(
                    self.truth.pose,
                    &self.objects(time),
                    time,
                    &mut self.rng,
                ))
            } else {
                None
            };
        Ok(SensorFrame {
            time,
            odometry,
            gnss,
            lidar,
            lidar_failed: false,
        })
    }
    fn advance(&mut self, command: ControlCommand, dt: f64) -> Result<(), String> {
        self.command = command;
        step_vehicle(&mut self.truth, command, self.vehicle, dt);
        Ok(())
    }
}
pub fn pipeline_config(scenario: &Scenario) -> PipelineConfig {
    let route = scenario.route();
    let (position, yaw) = route.sample(0.0, 0.0);
    PipelineConfig::new(route, Pose { position, yaw }, VehicleConfig::default())
}
pub fn simulate(scenario: Scenario, seed: u64) -> Result<Run, String> {
    scenario.validate()?;
    if scenario.dynamics.is_some() {
        return Err("dynamic plant calibration requires RNE --plant dynamic".into());
    }
    let config = pipeline_config(&scenario);
    let backend = ReferenceBackend {
        scenario: scenario.clone(),
        route: config.route.clone(),
        truth: EgoState {
            pose: config.initial_pose,
            speed: 0.0,
        },
        vehicle: config.vehicle,
        rng: Rng::new(seed),
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
    let route = config.route.clone();
    let navigation = scenario.navigation_plan()?;
    if let Some(plan) = &navigation
        && (plan.route.points != route.points || plan.route.half_width != route.half_width)
    {
        return Err("backend route differs from the selected navigation route".into());
    }
    let vehicle = config.vehicle;
    let mut pipeline = DrivingPipeline::new(config.clone())?;
    let mut sensor_log = SensorLog::new(source, config);
    let mut scan = LidarScan {
        stamp: 0.0,
        points: vec![],
    };
    let mut frames = Vec::new();
    let mut collisions = 0;
    let mut road_violations = 0;
    let mut minimum = 1000.0_f64;
    let mut error_sum = 0.0;
    let mut error_max = 0.0_f64;
    let mut emergency_steps = 0;
    let mut avoidance_steps = 0;
    let mut max_tracks = 0;
    let mut steps = 0;
    let mut reached_goal = false;
    for i in 0..=(scenario.duration / dt).round() as usize {
        let time = i as f64 * dt;
        let truth = backend.state();
        let objects = backend.objects(time);
        let input = backend.observe(time, i)?;
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
        sensor_log.record(input, result);
        if emergency {
            emergency_steps += 1;
        }
        if trajectory.mode == DrivingMode::Avoid {
            avoidance_steps += 1;
        }
        max_tracks = max_tracks.max(tracks.len());
        let (progress, lateral) = route.project(truth.pose.position);
        if lateral.abs() + vehicle.radius > route.half_width {
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
        if i.is_multiple_of(2) {
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
            });
        }
        let at_map_goal = navigation.as_ref().is_none_or(|plan| {
            truth
                .pose
                .position
                .distance(*plan.route.points.last().unwrap())
                <= 2.0
        });
        if progress >= route.length() - 2.0 && truth.speed < 0.2 && at_map_goal {
            collisions += usize::from(clearance < 0.0);
            reached_goal = true;
            break;
        }
        if time >= scenario.duration {
            collisions += usize::from(clearance < 0.0);
            break;
        }
        let previous = truth.pose.position;
        backend.advance(command, dt)?;
        let next_truth = backend.state();
        let next_objects = backend.objects(time + dt);
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
    if collisions > 0 {
        failures.push(format!("{collisions} colliding integration steps"));
    }
    if road_violations > 0 {
        failures.push(format!("{road_violations} road boundary violations"));
    }
    if let Some(required) = scenario.min_clearance_m
        && minimum < required
    {
        failures.push(format!(
            "minimum swept clearance {minimum:.3} m is below {required:.3} m"
        ));
    }
    if error_max > 1.0 {
        failures.push(format!(
            "localization max error {error_max:.3} m exceeds 1 m"
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
    };
    Ok(Run {
        backend: source.to_string(),
        sensor_log: Some(sensor_log),
        schema_version: 1,
        scenario,
        route,
        navigation,
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
        let scan = lidar(Pose::default(), &objects, 0.0, &mut Rng::new(1));
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
