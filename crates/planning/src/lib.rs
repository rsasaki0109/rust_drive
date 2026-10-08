//! Receding-horizon lateral lattice with time-indexed collision envelopes.
mod collision;
mod speed;
use rustdrive_core::{
    DrivingMode, EgoState, Planner, Prediction, Route, Trajectory, VehicleConfig,
};
pub struct LatticePlanner {
    pub cruise_speed: f64,
    pub vehicle: VehicleConfig,
    /// Forward acceleration authority used for reachable speed profiles.
    pub max_acceleration_m_s2: f64,
    /// Braking authority used for obstacle and goal speed envelopes.
    pub max_deceleration_m_s2: f64,
    /// Optional curvature-derived speed limit; None retains the baseline behavior.
    pub max_lateral_acceleration_m_s2: Option<f64>,
    previous_lateral: f64,
    maneuver_start: Option<(f64, f64, f64)>,
    yield_stop_s: Option<f64>,
}
impl Default for LatticePlanner {
    fn default() -> Self {
        Self {
            cruise_speed: 8.0,
            vehicle: VehicleConfig::default(),
            max_acceleration_m_s2: 2.0,
            max_deceleration_m_s2: 2.5,
            max_lateral_acceleration_m_s2: None,
            previous_lateral: 0.0,
            maneuver_start: None,
            yield_stop_s: None,
        }
    }
}
impl Planner for LatticePlanner {
    fn plan(&mut self, ego: EgoState, route: &Route, objects: &[Prediction]) -> Trajectory {
        if !ego.speed.is_finite()
            || !ego.pose.position.finite()
            || !self.cruise_speed.is_finite()
            || self.cruise_speed <= 0.0
            || !self.max_acceleration_m_s2.is_finite()
            || self.max_acceleration_m_s2 <= 0.0
            || !self.max_deceleration_m_s2.is_finite()
            || self.max_deceleration_m_s2 <= 0.0
            || self
                .max_lateral_acceleration_m_s2
                .is_some_and(|a| !a.is_finite() || a <= 0.0)
            || objects.iter().any(|object| {
                object.positions.is_empty()
                    || !object.dt.is_finite()
                    || object.dt <= 0.0
                    || !object.radius.is_finite()
                    || object.radius < 0.0
                    || !(object.dt * (object.positions.len() - 1) as f64).is_finite()
                    || object.positions.iter().any(|position| !position.finite())
            })
        {
            return Trajectory {
                points: vec![],
                mode: DrivingMode::Emergency,
                lateral_target: 0.0,
            };
        }
        let (s, lateral) = route.project(ego.pose.position);
        let remaining = (route.length() - s).max(0.0);
        let cruise = self.cruise_speed;
        let horizon = 40.0_f64.min((remaining - 1.0).max(0.0));
        let limits = speed::Limits {
            acceleration: self.max_acceleration_m_s2,
            deceleration: self.max_deceleration_m_s2,
            lateral: self.max_lateral_acceleration_m_s2,
            cruise,
        };
        let mut best: Option<(f64, Trajectory)> = None;
        for target in [0.0_f64, 3.5, -3.5] {
            if target.abs() + self.vehicle.radius > route.half_width
                || (target * lateral < 0.0 && lateral.abs() > 1.0)
            {
                continue;
            }
            let transition = transition_distance(
                ego.speed,
                target - lateral,
                self.max_lateral_acceleration_m_s2,
            );
            let (start_s, start_lateral, transition) =
                if (target - self.previous_lateral).abs() < 0.01 {
                    self.maneuver_start.unwrap_or((s, lateral, transition))
                } else {
                    (s, lateral, transition)
                };
            let anchored_offset = |distance| {
                start_lateral
                    + (target - start_lateral) * quintic((s + distance - start_s) / transition)
            };
            let tracking_error = lateral - anchored_offset(0.0);
            let mut geometry = Vec::new();
            let mut contained = true;
            for i in 0..=80 {
                let ds = horizon * i as f64 / 80.0;
                // Preserve maneuver progress, but join it from the measured lateral
                // position. The controller may lag the previous maneuver, especially
                // after a stop; a sweep must not start from an imaginary shifted ego.
                let offset =
                    anchored_offset(ds) + tracking_error * (1.0 - quintic(ds / transition));
                if offset.abs() + self.vehicle.radius > route.half_width {
                    contained = false;
                    break;
                }
                let position = if i == 0 {
                    ego.pose.position
                } else {
                    route.sample(s + ds, offset).0
                };
                geometry.push(position);
            }
            if !contained {
                continue;
            }
            let goal_stop = (remaining <= 41.0).then_some(horizon);
            let Some(mut points) =
                speed::profile(&geometry, ego.speed.max(0.0), &limits, goal_stop)
            else {
                continue;
            };
            let contact = |path: &[rustdrive_core::TrajectoryPoint]| {
                objects
                    .iter()
                    .filter_map(|object| {
                        collision::first_contact_time(path, object, self.vehicle.radius)
                    })
                    .fold(f64::INFINITY, f64::min)
            };
            let first_contact = contact(&points);
            let blocked = first_contact.is_finite();
            if blocked {
                let stop = if ego.speed < 0.15
                    && self.yield_stop_s.is_some_and(|stop_s| s >= stop_s - 0.2)
                {
                    ego.speed.max(0.0).powi(2) / (2.0 * self.max_deceleration_m_s2)
                } else {
                    (speed::distance_at(&points, first_contact) - 4.0).max(0.0)
                };
                let Some(stopped) =
                    speed::profile(&geometry, ego.speed.max(0.0), &limits, Some(stop))
                else {
                    continue;
                };
                points = stopped;
                // Slowing changes arrival times. Revalidate the complete braking path,
                // including an eight-second stationary hold; reject unsafe retiming.
                if contact(&points).is_finite() {
                    continue;
                }
            }
            let permitted = points.iter().map(|p| p.speed).fold(0.0_f64, f64::max);
            let switching_penalty = if target * self.previous_lateral < 0.0 {
                30.0
            } else {
                0.0
            };
            let score = switching_penalty
                + if blocked {
                    if self.max_lateral_acceleration_m_s2.is_some() {
                        100.0
                    } else {
                        15.0
                    }
                } else {
                    0.0
                }
                + (cruise - permitted) * 12.0
                + target.abs() * 0.35
                + (target - self.previous_lateral).abs()
                    * if target.abs() < 0.01 { 0.2 } else { 0.8 };
            let mode = if remaining < 2.0 {
                DrivingMode::Goal
            } else if blocked || permitted < cruise - 0.5 {
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
        if trajectory.mode != DrivingMode::Emergency {
            if (trajectory.lateral_target - self.previous_lateral).abs() > 0.01 {
                self.maneuver_start = Some((
                    s,
                    lateral,
                    transition_distance(
                        ego.speed,
                        trajectory.lateral_target - lateral,
                        self.max_lateral_acceleration_m_s2,
                    ),
                ));
            }
            self.previous_lateral = trajectory.lateral_target;
        }
        self.yield_stop_s = if trajectory.mode == DrivingMode::Yield {
            trajectory
                .points
                .last()
                .filter(|p| p.speed < 1e-8)
                .map(|p| route.project(p.position).0)
        } else {
            None
        };
        trajectory
    }
}
fn transition_distance(speed: f64, shift: f64, lateral_limit: Option<f64>) -> f64 {
    let baseline = (speed * 1.7).clamp(10.0, 16.0);
    // The normalized quintic's maximum second derivative is below six. This
    // length proposal allows gentle high-speed shifts on low-friction roads.
    // Actual sampled curvature and reachable speed bounds still decide feasibility.
    lateral_limit.map_or(baseline, |limit| {
        baseline.max(speed * (6.0 * shift.abs() / limit).sqrt())
    })
}
fn quintic(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)
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
        ego.pose.position = first.points[12].position;
        let next = planner.plan(ego, &road(5.5), &[obstacle()]);
        assert_eq!(next.lateral_target, first.lateral_target);
        assert!(
            next.points[0].position.y.abs() > 1.0,
            "quintic shift must not restart at every replan"
        );
        assert!((next.points[8].position.y - first.points[20].position.y).abs() < 1e-9);
    }

    #[test]
    fn a_lagging_vehicle_is_not_assumed_to_have_completed_the_lateral_maneuver() {
        let mut planner = LatticePlanner::default();
        planner.plan(EgoState::default(), &road(5.5), &[obstacle()]);
        let mut ego = EgoState::default();
        ego.pose.position.x = 6.0;
        let next = planner.plan(ego, &road(5.5), &[obstacle()]);
        assert_eq!(next.points[0].position, ego.pose.position);
        assert!(next.points[1].position.y.abs() < 0.5);
        assert!(next.points[20].position.y.abs() > 3.0);
    }

    #[test]
    fn tracking_correction_cannot_push_a_candidate_outside_the_road() {
        let mut planner = LatticePlanner::default();
        planner.plan(EgoState::default(), &road(5.5), &[obstacle()]);
        let mut ego = EgoState::default();
        ego.pose.position = Vec2::new(6.0, 3.5);
        let next = planner.plan(ego, &road(5.5), &[obstacle()]);
        assert!(!next.points.is_empty());
        assert!(
            next.points
                .iter()
                .all(|point| point.position.y.abs() + 1.25 <= 5.5)
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
    #[test]
    fn reduced_braking_authority_reduces_reachable_obstacle_and_goal_speeds() {
        let ego = EgoState {
            speed: 2.0,
            ..EgoState::default()
        };
        let nominal = LatticePlanner::default().plan(ego, &road(2.1), &[obstacle()]);
        let mut limited = LatticePlanner {
            max_deceleration_m_s2: 0.8,
            ..LatticePlanner::default()
        };
        let low = limited.plan(ego, &road(2.1), &[obstacle()]);
        let peak = |path: &Trajectory| path.points.iter().map(|p| p.speed).fold(0.0_f64, f64::max);
        assert!(peak(&low) + 1.0 < peak(&nominal));
        assert_eq!(low.points[0].speed, ego.speed);
        let near_goal = EgoState {
            pose: rustdrive_core::Pose {
                position: Vec2::new(94.0, 0.0),
                yaw: 0.0,
            },
            ..ego
        };
        let nominal = LatticePlanner::default().plan(near_goal, &road(2.1), &[]);
        let low = limited.plan(near_goal, &road(2.1), &[]);
        assert!(peak(&low) < peak(&nominal));
        let infeasible = limited.plan(
            EgoState {
                speed: 6.0,
                ..near_goal
            },
            &road(2.1),
            &[],
        );
        assert_eq!(infeasible.mode, DrivingMode::Emergency);
        assert!(infeasible.points.is_empty());
    }
    #[test]
    fn avoidance_speed_obeys_calibrated_curvature_bound() {
        let mut planner = LatticePlanner {
            max_lateral_acceleration_m_s2: Some(0.6),
            ..LatticePlanner::default()
        };
        let path = planner.plan(
            EgoState {
                speed: 0.0,
                ..EgoState::default()
            },
            &road(5.5),
            &[obstacle()],
        );
        assert!(path.lateral_target.abs() > 3.0);
        for p in path.points.windows(3) {
            let a = p[1].position.minus(p[0].position);
            let b = p[2].position.minus(p[1].position);
            let c = p[2].position.minus(p[0].position);
            let curvature = 2.0 * (a.x * b.y - a.y * b.x).abs()
                / (a.x.hypot(a.y) * b.x.hypot(b.y) * c.x.hypot(c.y));
            assert!(p[1].speed.powi(2) * curvature <= 0.6 + 1e-9);
        }
    }
    #[test]
    fn low_friction_high_speed_avoidance_uses_a_reachable_gentle_shift() {
        let mut planner = LatticePlanner {
            max_lateral_acceleration_m_s2: Some(1.1772),
            ..LatticePlanner::default()
        };
        let mut object = obstacle();
        object.positions.fill(Vec2::new(35.0, 0.0));
        let path = planner.plan(
            EgoState {
                speed: 6.0,
                ..EgoState::default()
            },
            &road(5.5),
            &[object],
        );
        assert!(path.lateral_target.abs() > 3.0);
        assert_eq!(path.points[0].speed, 6.0);
        assert!(
            path.points[20].position.y.abs() < 1.5,
            "high-speed shift must extend beyond the previous 10 m transition"
        );
        assert!(path.points.last().unwrap().position.y.abs() > 3.4);
    }
    #[test]
    fn invalid_motion_limits_stop_planning() {
        let mut planner = LatticePlanner {
            max_deceleration_m_s2: f64::NAN,
            ..LatticePlanner::default()
        };
        assert_eq!(
            planner.plan(EgoState::default(), &road(5.5), &[]).mode,
            DrivingMode::Emergency
        );
    }

    #[test]
    fn yields_inside_braking_distance_and_waits_until_candidate_clears() {
        let mut planner = LatticePlanner::default();
        let mut object = obstacle();
        object.positions.fill(rustdrive_core::Vec2::new(24.0, 0.0));
        let approaching = EgoState {
            speed: 8.0,
            ..EgoState::default()
        };
        let stop = planner.plan(approaching, &road(2.1), &[object.clone()]);
        assert_eq!(stop.mode, DrivingMode::Yield);
        assert_eq!(stop.points[0].speed, approaching.speed);
        assert_eq!(stop.points.last().unwrap().speed, 0.0);
        let waiting_ego = EgoState {
            pose: rustdrive_core::Pose {
                position: stop.points.last().unwrap().position,
                yaw: 0.0,
            },
            ..EgoState::default()
        };
        let waiting = planner.plan(waiting_ego, &road(2.1), &[object]);
        assert_eq!(waiting.mode, DrivingMode::Yield);
        assert_eq!(
            waiting.points[0].speed, 0.0,
            "do not accelerate back into a blocked candidate"
        );
        let clear = planner.plan(waiting_ego, &road(2.1), &[]);
        assert_eq!(clear.mode, DrivingMode::Cruise);
        assert_eq!(clear.points[0].speed, 0.0);
        assert!(clear.points[1].speed > 0.0);
    }
}
