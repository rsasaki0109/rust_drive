//! Deterministic closed-loop simulator. Only sensor observations enter the stack.
use rustdrive_control::{PurePursuit, guard};
use rustdrive_core::*;
use rustdrive_localization::Ekf;
use rustdrive_mapping::OccupancyGrid;
use rustdrive_perception::{LidarClusters, Tracker};
use rustdrive_planning::LatticePlanner;
use rustdrive_prediction::ConstantVelocity;
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
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Expected {
    Goal,
    Stop,
    Fault,
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
            ]
            .iter()
            .all(|x| x.is_finite())
                || o.s < 0.0
                || o.s > self.road_length
                || !(0.2..=3.0).contains(&o.radius)
                || o.speed.abs() > 12.0
                || o.lateral_speed.abs() > 4.0
                || o.active_from < 0.0
            {
                return Err("invalid object parameters".into());
            }
        }
        Ok(())
    }
    pub fn route(&self) -> Route {
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
    fn world_objects(&self, route: &Route, t: f64) -> Vec<WorldObject> {
        self.objects
            .iter()
            .enumerate()
            .filter(|(_, o)| t >= o.active_from)
            .map(|(id, o)| {
                let elapsed = t - o.active_from;
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
    pub schema_version: u32,
    pub scenario: Scenario,
    pub route: Route,
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
pub fn simulate(scenario: Scenario, seed: u64) -> Result<Run, String> {
    scenario.validate()?;
    let dt = 0.05;
    let route = scenario.route();
    let vehicle = VehicleConfig::default();
    let (spawn, yaw) = route.sample(0.0, 0.0);
    let mut truth = EgoState {
        pose: Pose {
            position: spawn,
            yaw,
        },
        speed: 0.0,
    };
    let mut ekf = Ekf::new(truth.pose);
    let mut rng = Rng::new(seed);
    let mut perception = LidarClusters;
    let mut tracker = Tracker::default();
    let predictor = ConstantVelocity::default();
    let mut planner = LatticePlanner::default();
    let mut controller = PurePursuit::default();
    let mut grid = OccupancyGrid::new(
        Vec2::new(-10.0, -20.0),
        ((scenario.road_length + 30.0) / 0.5) as usize,
        80,
        0.5,
    );
    let mut command = ControlCommand::default();
    let mut scan = LidarScan {
        stamp: f64::NEG_INFINITY,
        points: vec![],
    };
    let mut tracks = Vec::new();
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
        if i > 0 {
            ekf.predict(
                Odometry {
                    stamp: time,
                    speed: truth.speed + rng.noise(0.015),
                    yaw_rate: truth.speed / vehicle.wheelbase * command.steering.tan()
                        + rng.noise(0.001),
                },
                dt,
            );
        }
        let objects = scenario.world_objects(&route, time);
        if i % 4 == 0 && scenario.gnss_dropout.is_none_or(|t| time < t) {
            ekf.update(Gnss {
                stamp: time,
                position: truth
                    .pose
                    .position
                    .plus(Vec2::new(rng.noise(0.14), rng.noise(0.14))),
                variance: 0.02,
            });
        }
        let estimate = ekf.state();
        if i % 2 == 0 && scenario.lidar_dropout.is_none_or(|t| time < t) {
            scan = lidar(truth.pose, &objects, time, &mut rng);
            let detections = perception.detect(&scan, estimate.pose);
            tracks = tracker.update(&detections, time);
            grid.update(&scan, estimate.pose);
        }
        let predictions = predictor.predict(&tracks);
        let mut trajectory = planner.plan(estimate, &route, &predictions);
        let requested = controller.control(estimate, &trajectory, dt);
        command = guard(
            requested,
            time,
            scan.stamp,
            ekf.last_gnss,
            ekf.position_variance(),
        );
        let emergency = command.acceleration <= -5.99;
        if emergency {
            trajectory.mode = DrivingMode::Emergency;
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
        if i % 2 == 0 {
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
        if progress >= route.length() - 2.0 && truth.speed < 0.2 {
            reached_goal = true;
            break;
        }
        if time >= scenario.duration {
            break;
        }
        let previous = truth.pose.position;
        step_vehicle(&mut truth, command, vehicle, dt);
        let next_objects = scenario.world_objects(&route, time + dt);
        for object in &objects {
            if let Some(next) = next_objects.iter().find(|n| n.id == object.id) {
                let clearance = swept_distance(
                    previous.minus(object.position),
                    truth.pose.position.minus(next.position),
                ) - vehicle.radius
                    - object.radius;
                minimum = minimum.min(clearance);
                if clearance < 0.0 {
                    collisions += 1;
                }
            }
        }
    }
    let progress = route.project(truth.pose.position).0;
    let mut failures = Vec::new();
    if collisions > 0 {
        failures.push(format!("{collisions} colliding integration steps"));
    }
    if road_violations > 0 {
        failures.push(format!("{road_violations} road boundary violations"));
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
        schema_version: 1,
        scenario,
        route,
        vehicle,
        frames,
        occupied_cells: grid.occupied_cells(),
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
