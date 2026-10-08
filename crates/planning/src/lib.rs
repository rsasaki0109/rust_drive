//! Receding-horizon lateral lattice with time-indexed collision envelopes.
use rustdrive_core::{
    DrivingMode, EgoState, Planner, Prediction, Route, Trajectory, TrajectoryPoint, VehicleConfig,
};
pub struct LatticePlanner {
    pub cruise_speed: f64,
    pub vehicle: VehicleConfig,
    previous_lateral: f64,
    maneuver_start: Option<(f64, f64)>,
}
impl Default for LatticePlanner {
    fn default() -> Self {
        Self {
            cruise_speed: 8.0,
            vehicle: VehicleConfig::default(),
            previous_lateral: 0.0,
            maneuver_start: None,
        }
    }
}
impl Planner for LatticePlanner {
    fn plan(&mut self, ego: EgoState, route: &Route, objects: &[Prediction]) -> Trajectory {
        let (s, lateral) = route.project(ego.pose.position);
        let remaining = (route.length() - s).max(0.0);
        let cruise = self
            .cruise_speed
            .min((2.0 * 1.6 * (remaining - 1.0).max(0.0)).sqrt());
        let horizon = 40.0_f64.min(remaining);
        let mut best: Option<(f64, Trajectory)> = None;
        for target in [0.0_f64, 3.5, -3.5] {
            if target.abs() + self.vehicle.radius > route.half_width
                || (target * lateral < 0.0 && lateral.abs() > 1.0)
            {
                continue;
            }
            let transition = (ego.speed * 1.7).clamp(10.0, 16.0);
            let mut points = Vec::new();
            let mut first_blocked = f64::INFINITY;
            for i in 0..=80 {
                let ds = horizon * i as f64 / 80.0;
                let (start_s, start_lateral) = if (target - self.previous_lateral).abs() < 0.01 {
                    self.maneuver_start.unwrap_or((s, lateral))
                } else {
                    (s, lateral)
                };
                let u = ((s + ds - start_s) / transition).clamp(0.0, 1.0);
                let blend = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
                let offset = start_lateral + (target - start_lateral) * blend;
                let (position, _) = route.sample(s + ds, offset);
                let time = ds / ego.speed.max(3.0);
                for obj in objects {
                    if obj.positions.is_empty() || obj.dt <= 0.0 {
                        continue;
                    }
                    let index = ((time / obj.dt).round() as usize).min(obj.positions.len() - 1);
                    let margin = 0.30 + 0.06 * time.min(5.0);
                    if position.distance(obj.positions[index])
                        < self.vehicle.radius + obj.radius + margin
                    {
                        first_blocked = first_blocked.min(ds);
                    }
                }
                points.push(TrajectoryPoint {
                    position,
                    speed: cruise,
                    time,
                });
            }
            // Braking distance, including one control cycle + conservative planning buffer.
            let permitted = (2.0 * 2.5 * (first_blocked - 4.0).max(0.0))
                .sqrt()
                .min(cruise);
            for point in &mut points {
                point.speed = permitted;
            }
            let switching_penalty = if target * self.previous_lateral < 0.0 {
                30.0
            } else {
                0.0
            };
            let score = switching_penalty
                + if first_blocked.is_finite() { 15.0 } else { 0.0 }
                + (cruise - permitted) * 12.0
                + target.abs() * 0.35
                + (target - self.previous_lateral).abs()
                    * if target.abs() < 0.01 { 0.2 } else { 0.8 };
            let mode = if remaining < 2.0 {
                DrivingMode::Goal
            } else if permitted < cruise - 0.5 {
                DrivingMode::Yield
            } else if target.abs() > 0.1 {
                DrivingMode::Avoid
            } else {
                DrivingMode::Cruise
            };
            let traj = Trajectory {
                points,
                mode,
                lateral_target: target,
            };
            if best.as_ref().is_none_or(|(cost, _)| score < *cost) {
                best = Some((score, traj));
            }
        }
        let trajectory = best.map(|(_, t)| t).unwrap_or(Trajectory {
            points: vec![],
            mode: DrivingMode::Emergency,
            lateral_target: 0.0,
        });
        if (trajectory.lateral_target - self.previous_lateral).abs() > 0.01 {
            self.maneuver_start = Some((s, lateral));
        }
        self.previous_lateral = trajectory.lateral_target;
        trajectory
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use rustdrive_core::Vec2;
    fn road(width: f64) -> Route {
        Route::new(vec![Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0)], width).unwrap()
    }
    fn obstacle() -> Prediction {
        Prediction {
            id: 1,
            positions: vec![Vec2::new(20.0, 0.0); 50],
            radius: 1.0,
            dt: 0.2,
        }
    }
    #[test]
    fn selects_avoidance_and_respects_road() {
        let t = LatticePlanner::default().plan(EgoState::default(), &road(5.5), &[obstacle()]);
        assert_eq!(t.mode, DrivingMode::Avoid);
        assert!(t.points.iter().all(|p| p.position.y.abs() + 1.25 <= 5.5));
    }
    #[test]
    fn replan_keeps_maneuver_progress() {
        let mut planner = LatticePlanner::default();
        let first = planner.plan(EgoState::default(), &road(5.5), &[obstacle()]);
        assert_eq!(first.mode, DrivingMode::Avoid);
        let mut ego = EgoState::default();
        ego.pose.position.x = 6.0;
        let next = planner.plan(ego, &road(5.5), &[obstacle()]);
        assert_eq!(next.lateral_target, first.lateral_target);
        assert!(
            next.points[0].position.y.abs() > 1.0,
            "quintic shift must not restart at every replan"
        );
    }
    #[test]
    fn returns_to_center_when_obstacle_has_passed() {
        let mut planner = LatticePlanner::default();
        planner.plan(EgoState::default(), &road(5.5), &[obstacle()]);
        let mut ego = EgoState::default();
        ego.pose.position = Vec2::new(30.0, 3.5);
        let next = planner.plan(ego, &road(5.5), &[obstacle()]);
        assert_eq!(next.lateral_target, 0.0);
        assert_eq!(next.mode, DrivingMode::Cruise);
    }
    #[test]
    fn blocked_narrow_road_slows() {
        let mut ego = EgoState::default();
        ego.pose.position.x = 8.0;
        let t = LatticePlanner::default().plan(ego, &road(2.0), &[obstacle()]);
        assert_eq!(t.mode, DrivingMode::Yield);
        assert!(t.points[0].speed < 8.0);
    }
}
