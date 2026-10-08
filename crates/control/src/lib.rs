//! Pure-pursuit steering and acceleration feedforward with bounded PI speed feedback.
use rustdrive_core::{
    ControlCommand, Controller, DrivingMode, EgoState, Trajectory, VehicleConfig, wrap_angle,
};
#[derive(Default)]
pub struct PurePursuit {
    pub vehicle: VehicleConfig,
    integral: f64,
    steering: f64,
}
impl PurePursuit {
    pub fn with_vehicle(vehicle: VehicleConfig) -> Self {
        Self {
            vehicle,
            ..Self::default()
        }
    }
    /// Clear route-specific longitudinal feedback while retaining the last emitted
    /// steering command, so a stopped route handover respects the steering-rate limit.
    pub fn reset_route_state(&mut self) {
        self.integral = 0.0;
    }
}
impl Controller for PurePursuit {
    fn control(&mut self, ego: EgoState, path: &Trajectory, dt: f64) -> ControlCommand {
        if path.mode == DrivingMode::Emergency
            || path.points.is_empty()
            || !ego.pose.position.finite()
            || !ego.pose.yaw.is_finite()
            || !ego.speed.is_finite()
            || !dt.is_finite()
            || dt <= 0.0
            || path
                .points
                .iter()
                .any(|p| !p.position.finite() || !p.speed.is_finite() || !p.time.is_finite())
        {
            self.integral = 0.0;
            self.steering = 0.0;
            return ControlCommand::emergency();
        }
        // Shorter preview follows the smooth lateral maneuver more closely.
        // Interpolate its circle intersection to avoid sample-index steering jumps.
        let lookahead = (3.0 + ego.speed * 0.45).clamp(3.0, 8.0);
        let target = pursuit_target(path, ego.pose.position, lookahead);
        if !target.finite() {
            return ControlCommand::emergency();
        }
        let delta = target.minus(ego.pose.position);
        let distance = delta.x.hypot(delta.y).max(0.1);
        let alpha = wrap_angle(delta.y.atan2(delta.x) - ego.pose.yaw);
        let desired = (2.0 * self.vehicle.wheelbase * alpha.sin() / distance)
            .atan()
            .clamp(-self.vehicle.max_steer, self.vehicle.max_steer);
        self.steering += (desired - self.steering).clamp(-0.7 * dt, 0.7 * dt);
        let initial = &path.points[0];
        let acceleration = if let Some(next) = path.points.get(1) {
            let duration = next.time - initial.time;
            if !duration.is_finite()
                || duration <= 0.0
                || !next.speed.is_finite()
                || !initial.speed.is_finite()
            {
                return ControlCommand::emergency();
            }
            let feedforward = (next.speed - initial.speed) / duration;
            let error = initial.speed - ego.speed;
            self.integral = (self.integral + error * dt).clamp(-2.0, 2.0);
            (feedforward + 1.5 * error + 0.15 * self.integral).clamp(-4.0, 2.0)
        } else if initial.speed < 0.1 {
            self.integral = 0.0;
            -3.5
        } else {
            return ControlCommand::emergency();
        };
        ControlCommand {
            acceleration,
            steering: self.steering,
        }
    }
}
fn pursuit_target(
    path: &Trajectory,
    origin: rustdrive_core::Vec2,
    lookahead: f64,
) -> rustdrive_core::Vec2 {
    let mut previous = path.points[0].position;
    if previous.distance(origin) >= lookahead {
        return previous;
    }
    for point in path.points.iter().skip(1) {
        let next = point.position;
        if next.distance(origin) >= lookahead {
            let relative = previous.minus(origin);
            let delta = next.minus(previous);
            let a = delta.x * delta.x + delta.y * delta.y;
            let b = relative.x * delta.x + relative.y * delta.y;
            let c = relative.x * relative.x + relative.y * relative.y - lookahead * lookahead;
            let u = ((-b + (b * b - a * c).max(0.0).sqrt()) / a).clamp(0.0, 1.0);
            return previous.plus(delta.scaled(u));
        }
        previous = next;
    }
    previous
}
/// Independent freshness / numeric guard; a simulation gate, not a certified safety system.
pub fn guard(
    command: ControlCommand,
    now: f64,
    last_lidar: f64,
    last_gnss: f64,
    position_variance: f64,
) -> ControlCommand {
    if !command.finite()
        || !now.is_finite()
        || !last_lidar.is_finite()
        || !last_gnss.is_finite()
        || !position_variance.is_finite()
        || now - last_lidar > 0.35
        || now - last_gnss > 0.75
        || last_lidar > now + 0.01
        || last_gnss > now + 0.01
        || position_variance > 4.0
    {
        ControlCommand::emergency()
    } else {
        ControlCommand {
            acceleration: command.acceleration.clamp(-6.0, 2.0),
            steering: command.steering.clamp(-0.55, 0.55),
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn profile(initial: f64, next: f64, duration: f64) -> Trajectory {
        Trajectory {
            points: vec![
                rustdrive_core::TrajectoryPoint {
                    position: rustdrive_core::Vec2::default(),
                    speed: initial,
                    time: 0.0,
                },
                rustdrive_core::TrajectoryPoint {
                    position: rustdrive_core::Vec2::new(3.0, 0.0),
                    speed: next,
                    time: duration,
                },
            ],
            mode: DrivingMode::Cruise,
            lateral_target: 0.0,
        }
    }
    #[test]
    fn feedforward_starts_from_rest_and_tracks_planned_braking() {
        let mut controller = PurePursuit::default();
        assert_eq!(
            controller
                .control(EgoState::default(), &profile(0.0, 2.0, 1.0), 0.05)
                .acceleration,
            2.0
        );
        let ego = EgoState {
            speed: 4.0,
            ..EgoState::default()
        };
        assert_eq!(
            controller
                .control(ego, &profile(4.0, 2.0, 1.0), 0.05)
                .acceleration,
            -2.0
        );
    }
    #[test]
    fn stationary_hold_does_not_command_acceleration() {
        let mut controller = PurePursuit::default();
        assert_eq!(
            controller
                .control(EgoState::default(), &profile(0.0, 0.0, 8.0), 0.05)
                .acceleration,
            0.0
        );
    }
    #[test]
    fn route_handover_retains_steering_rate_continuity() {
        let mut turning = profile(0.0, 0.0, 8.0);
        turning.points[1].position.y = 3.0;
        let mut controller = PurePursuit::default();
        let mut before = ControlCommand::default();
        for _ in 0..20 {
            before = controller.control(EgoState::default(), &turning, 0.05);
        }
        assert!(before.steering.abs() > 0.3);
        controller.reset_route_state();
        let after = controller.control(EgoState::default(), &profile(0.0, 0.0, 8.0), 0.05);
        assert!((after.steering - before.steering).abs() <= 0.7 * 0.05 + 1e-10);
        assert_eq!(after.acceleration, 0.0);
    }
    #[test]
    fn invalid_profile_time_brakes() {
        let mut controller = PurePursuit::default();
        for duration in [0.0, -1.0, f64::NAN] {
            assert_eq!(
                controller
                    .control(EgoState::default(), &profile(0.0, 2.0, duration), 0.05)
                    .acceleration,
                -6.0
            );
        }
    }
    #[test]
    fn interpolated_target_and_steering_match_a_known_circle() {
        let radius = 20.0;
        let mut path = profile(4.0, 4.0, 1.0);
        path.points = (0..=80)
            .map(|i| {
                let angle = i as f64 * 0.01;
                rustdrive_core::TrajectoryPoint {
                    position: rustdrive_core::Vec2::new(
                        radius * angle.sin(),
                        radius * (1.0 - angle.cos()),
                    ),
                    speed: 4.0,
                    time: radius * angle / 4.0,
                }
            })
            .collect();
        let target = pursuit_target(&path, rustdrive_core::Vec2::default(), 4.8);
        assert!((target.x.hypot(target.y) - 4.8).abs() < 1e-10);
        let mut controller = PurePursuit::default();
        let ego = EgoState {
            speed: 4.0,
            ..EgoState::default()
        };
        let mut command = ControlCommand::default();
        for _ in 0..20 {
            command = controller.control(ego, &path, 0.05);
        }
        assert!((command.steering - (2.7_f64 / radius).atan()).abs() < 1e-4);
    }
    #[test]
    fn emergency_recovery_starts_from_the_last_emitted_steering_command() {
        let mut path = profile(4.0, 4.0, 1.0);
        path.points[1].position.y = 3.0;
        let mut controller = PurePursuit::default();
        for _ in 0..10 {
            controller.control(EgoState::default(), &path, 0.05);
        }
        let mut emergency = path.clone();
        emergency.mode = DrivingMode::Emergency;
        assert_eq!(
            controller
                .control(EgoState::default(), &emergency, 0.05)
                .steering,
            0.0
        );
        assert!(
            controller
                .control(EgoState::default(), &path, 0.05)
                .steering
                .abs()
                <= 0.035 + 1e-10
        );
    }
    #[test]
    fn stale_data_and_nan_brake() {
        let c = ControlCommand {
            acceleration: 1.0,
            steering: 0.2,
        };
        assert_eq!(guard(c, 1.0, 0.5, 1.0, 1.0).acceleration, -6.0);
        assert_eq!(
            guard(
                ControlCommand {
                    acceleration: f64::NAN,
                    ..c
                },
                1.0,
                1.0,
                1.0,
                1.0
            )
            .acceleration,
            -6.0
        );
    }
    #[test]
    fn bounds_actuation() {
        let c = guard(
            ControlCommand {
                acceleration: 100.0,
                steering: -2.0,
            },
            0.0,
            0.0,
            0.0,
            0.1,
        );
        assert_eq!(c.acceleration, 2.0);
        assert_eq!(c.steering, -0.55);
    }
}
