//! Pure-pursuit lateral control and bounded PI longitudinal control.
use rustdrive_core::{
    ControlCommand, Controller, DrivingMode, EgoState, Trajectory, VehicleConfig, wrap_angle,
};
#[derive(Default)]
pub struct PurePursuit {
    pub vehicle: VehicleConfig,
    integral: f64,
    steering: f64,
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
        {
            self.integral = 0.0;
            return ControlCommand::emergency();
        }
        let lookahead = (3.5 + ego.speed * 0.65).clamp(3.5, 10.0);
        let target = path
            .points
            .iter()
            .find(|p| p.position.distance(ego.pose.position) >= lookahead)
            .unwrap_or_else(|| path.points.last().unwrap());
        if !target.position.finite() || !target.speed.is_finite() {
            return ControlCommand::emergency();
        }
        let delta = target.position.minus(ego.pose.position);
        let distance = delta.x.hypot(delta.y).max(0.1);
        let alpha = wrap_angle(delta.y.atan2(delta.x) - ego.pose.yaw);
        let desired = (2.0 * self.vehicle.wheelbase * alpha.sin() / distance)
            .atan()
            .clamp(-self.vehicle.max_steer, self.vehicle.max_steer);
        self.steering += (desired - self.steering).clamp(-0.7 * dt, 0.7 * dt);
        let error = path.points[0].speed - ego.speed;
        self.integral = (self.integral + error * dt).clamp(-2.0, 2.0);
        let acceleration = if path.points[0].speed < 0.1 {
            self.integral = 0.0;
            -3.5
        } else {
            (1.5 * error + 0.15 * self.integral).clamp(-4.0, 2.0)
        };
        ControlCommand {
            acceleration,
            steering: self.steering,
        }
    }
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
