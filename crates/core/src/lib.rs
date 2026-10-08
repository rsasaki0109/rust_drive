//! Transport-independent contracts. SI units; world ENU, body x forward / y left.
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}
impl Vec2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    pub fn distance(self, other: Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
    pub fn rotated(self, yaw: f64) -> Self {
        Self::new(
            self.x * yaw.cos() - self.y * yaw.sin(),
            self.x * yaw.sin() + self.y * yaw.cos(),
        )
    }
    pub fn plus(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y)
    }
    pub fn minus(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y)
    }
    pub fn scaled(self, scale: f64) -> Self {
        Self::new(self.x * scale, self.y * scale)
    }
    pub fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}
pub fn wrap_angle(a: f64) -> f64 {
    (a + PI).rem_euclid(2.0 * PI) - PI
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Pose {
    pub position: Vec2,
    pub yaw: f64,
}
impl Pose {
    pub fn to_world(self, local: Vec2) -> Vec2 {
        local.rotated(self.yaw).plus(self.position)
    }
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct EgoState {
    pub pose: Pose,
    pub speed: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LidarScan {
    pub stamp: f64,
    pub points: Vec<Vec2>,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Odometry {
    pub stamp: f64,
    pub speed: f64,
    pub yaw_rate: f64,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Gnss {
    pub stamp: f64,
    pub position: Vec2,
    pub variance: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Detection {
    pub center: Vec2,
    pub radius: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Track {
    pub id: u64,
    pub position: Vec2,
    pub velocity: Vec2,
    pub radius: f64,
    pub last_seen: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Prediction {
    pub id: u64,
    pub positions: Vec<Vec2>,
    pub radius: f64,
    pub dt: f64,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    pub position: Vec2,
    /// Planned speed in m/s at this position and relative time.
    pub speed: f64,
    /// Seconds from the current sensor frame; segment speed varies linearly in time.
    pub time: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Trajectory {
    pub points: Vec<TrajectoryPoint>,
    pub mode: DrivingMode,
    pub lateral_target: f64,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub enum DrivingMode {
    Cruise,
    Avoid,
    Yield,
    Goal,
    Emergency,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct ControlCommand {
    pub acceleration: f64,
    pub steering: f64,
}
impl ControlCommand {
    pub fn emergency() -> Self {
        Self {
            acceleration: -6.0,
            steering: 0.0,
        }
    }
    pub fn finite(self) -> bool {
        self.acceleration.is_finite() && self.steering.is_finite()
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct VehicleConfig {
    pub wheelbase: f64,
    pub radius: f64,
    pub max_steer: f64,
}
impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            wheelbase: 2.7,
            radius: 1.25,
            max_steer: 0.55,
        }
    }
}
/// Arc-length parameterized polyline, supplied by a map adapter rather than perception.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Route {
    pub points: Vec<Vec2>,
    pub lengths: Vec<f64>,
    pub half_width: f64,
}
impl Route {
    pub fn new(points: Vec<Vec2>, half_width: f64) -> Result<Self, String> {
        if points.len() < 2
            || !half_width.is_finite()
            || half_width <= 0.0
            || points.iter().any(|p| !p.finite())
        {
            return Err("route requires finite points and a positive road half-width".into());
        }
        let mut lengths = vec![0.0];
        for pair in points.windows(2) {
            let d = pair[0].distance(pair[1]);
            if d < 1e-6 {
                return Err("duplicate consecutive route points".into());
            }
            lengths.push(lengths.last().unwrap() + d);
        }
        Ok(Self {
            points,
            lengths,
            half_width,
        })
    }
    pub fn length(&self) -> f64 {
        *self.lengths.last().unwrap()
    }
    pub fn sample(&self, s: f64, lateral: f64) -> (Vec2, f64) {
        let s = s.clamp(0.0, self.length());
        let i = self
            .lengths
            .partition_point(|x| *x < s)
            .saturating_sub(1)
            .min(self.points.len() - 2);
        let d = self.points[i + 1].minus(self.points[i]);
        let yaw = d.y.atan2(d.x);
        let ratio = (s - self.lengths[i]) / (self.lengths[i + 1] - self.lengths[i]);
        (
            self.points[i]
                .plus(d.scaled(ratio))
                .plus(Vec2::new(-yaw.sin(), yaw.cos()).scaled(lateral)),
            yaw,
        )
    }
    pub fn project(&self, point: Vec2) -> (f64, f64) {
        let mut best = (f64::INFINITY, 0.0, 0.0);
        for (i, pair) in self.points.windows(2).enumerate() {
            let d = pair[1].minus(pair[0]);
            let q = point.minus(pair[0]);
            let len = d.x.hypot(d.y);
            let t = ((q.x * d.x + q.y * d.y) / (len * len)).clamp(0.0, 1.0);
            let p = pair[0].plus(d.scaled(t));
            let dist = point.distance(p);
            if dist < best.0 {
                best = (
                    dist,
                    self.lengths[i] + t * len,
                    (d.x * q.y - d.y * q.x) / len,
                );
            }
        }
        (best.1, best.2)
    }
}
/// Implementations can use classical algorithms or inference libraries with the same contract.
pub trait Perception {
    fn detect(&mut self, scan: &LidarScan, pose: Pose) -> Vec<Detection>;
}
pub trait Predictor {
    fn predict(&self, tracks: &[Track]) -> Vec<Prediction>;
}
pub trait Planner {
    fn plan(&mut self, ego: EgoState, route: &Route, objects: &[Prediction]) -> Trajectory;
}
pub trait Controller {
    fn control(&mut self, ego: EgoState, trajectory: &Trajectory, dt: f64) -> ControlCommand;
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn route_round_trip() {
        let r = Route::new(
            vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
            ],
            5.0,
        )
        .unwrap();
        let (p, _) = r.sample(15.0, 1.0);
        let (s, d) = r.project(p);
        assert!((s - 15.0).abs() < 1e-9);
        assert!((d - 1.0).abs() < 1e-9);
    }
    #[test]
    fn rejects_bad_routes() {
        assert!(Route::new(vec![Vec2::default(); 2], 5.0).is_err());
    }
    #[test]
    fn angle_wrap() {
        assert!(wrap_angle(3.0 * PI).abs() <= PI);
    }
}
