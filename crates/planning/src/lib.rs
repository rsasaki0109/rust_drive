//! Receding-horizon lateral lattice with time-indexed collision envelopes.
mod collision;
mod geometry;
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
    terminal_side_reserved: bool,
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
            terminal_side_reserved: false,
        }
    }
}
impl Planner for LatticePlanner {
    fn plan(&mut self, ego: EgoState, route: &Route, objects: &[Prediction]) -> Trajectory {
        if !ego.speed.is_finite()
            || !ego.pose.position.finite()
            || !ego.pose.yaw.is_finite()
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
        let goal_distance = (remaining - 1.0).max(0.0);
        // One meter before the endpoint is a desired goal, not an obstacle.
        // If noise/tracking carries the estimate past it while still moving,
        // use the remaining corridor for a reachable bounded stop.
        let recovering_goal = goal_distance < 0.05 && ego.speed > 0.0;
        let goal_distance = if recovering_goal {
            (ego.speed.powi(2) / (2.0 * self.max_deceleration_m_s2) + 0.05).min(remaining)
        } else {
            goal_distance
        };
        let horizon = 40.0_f64.min(goal_distance);
        let limits = speed::Limits {
            acceleration: self.max_acceleration_m_s2,
            deceleration: self.max_deceleration_m_s2,
            lateral: self.max_lateral_acceleration_m_s2,
            cruise: if recovering_goal { 0.0 } else { cruise },
        };
        // A finite constant-velocity forecast can end before traffic reaches
        // the goal. Reserve a lateral stopping place while there is still room
        // to finish the shift, using only observed forecast motion and the route.
        let terminal_traffic = remaining <= 41.0
            && objects.iter().any(|object| {
                let (object_s, object_lateral) = route.project(object.positions[0]);
                object_s >= s - 40.0
                    && object_lateral.abs() < object.radius + 0.3
                    && object
                        .positions
                        .get(1)
                        .is_some_and(|next| route.project(*next).0 - object_s > 0.7 * object.dt)
            });
        let reserve_terminal_side = terminal_traffic || self.terminal_side_reserved;
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
            let base = |distance| geometry::sample(route, s + distance, anchored_offset(distance));
            let base_initial = base(0.0);
            let base_tangent = base(0.01).minus(base_initial).scaled(100.0);
            let position_error = ego.pose.position.minus(base_initial);
            let desired_tangent = rustdrive_core::Vec2::new(ego.pose.yaw.cos(), ego.pose.yaw.sin())
                .scaled(base_tangent.x.hypot(base_tangent.y));
            let tangent_error = desired_tangent.minus(base_tangent);
            let mut geometry = Vec::new();
            let mut contained = true;
            for i in 0..=80 {
                let ds = horizon * i as f64 / 80.0;
                let position = if i == 0 {
                    ego.pose.position
                } else {
                    base(ds)
                        .plus(position_error.scaled(1.0 - quintic(ds / transition)))
                        .plus(tangent_error.scaled(geometry::heading_weight(ds, transition)))
                };
                // Interpolation and heading correction may leave the supplied corridor;
                // check the actual candidate against the unchanged route evaluator.
                let corridor_radius = route.half_width - self.vehicle.radius;
                // Distance to any point on the centerline is an upper bound on
                // nearest-centerline distance. Most samples can be accepted with
                // this bound; ambiguous ones still use the full corridor projection.
                let reference = route.sample(s + ds, 0.0).0;
                if position.distance(reference) > corridor_radius
                    && route.project(position).1.abs() > corridor_radius
                {
                    contained = false;
                    break;
                }
                geometry.push(position);
            }
            if !contained {
                continue;
            }
            // Route progress is not arc length when the candidate shifts laterally.
            // Stop at the generated terminal position, rather than truncating a
            // longer candidate using the route's longitudinal distance.
            let goal_stop =
                (remaining <= 41.0).then(|| geometry.windows(2).map(|p| p[0].distance(p[1])).sum());
            let Some(mut points) =
                speed::profile(&geometry, ego.speed.max(0.0), &limits, goal_stop)
            else {
                continue;
            };
            let contact = |path: &[rustdrive_core::TrajectoryPoint]| {
                objects
                    .iter()
                    .filter_map(|object| {
                        collision::first_contact_time(
                            path,
                            object,
                            self.vehicle.radius,
                            rustdrive_core::Vec2::new(ego.pose.yaw.cos(), ego.pose.yaw.sin()),
                        )
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
            // Forecast jitter should not abort an unfinished lateral shift while
            // the sensed object still spans the centerline ahead. This is a preference,
            // not a feasibility override: every candidate remains swept/retimed.
            let abandoning_pass = target.abs() < 0.01
                && self.previous_lateral.abs() > 0.1
                && self
                    .maneuver_start
                    .is_some_and(|(start_s, _, transition)| s < start_s + transition)
                && objects.iter().any(|object| {
                    let (object_s, object_lateral) = route.project(object.positions[0]);
                    object_s + object.radius + self.vehicle.radius >= s
                        && object_s <= s + horizon
                        && object_lateral.abs() < object.radius + 0.3
                });
            let terminal_center_cost = if reserve_terminal_side && target.abs() < 0.01 {
                5.0
            } else {
                0.0
            };
            let score = switching_penalty
                + terminal_center_cost
                + if abandoning_pass { 1.0 } else { 0.0 }
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
            if terminal_traffic && trajectory.lateral_target.abs() > 0.1 {
                // Keep the stopping-place preference through forecast jitter,
                // missed tracks, and the actor becoming stationary. Feasibility
                // (including the complete hold sweep) always takes precedence.
                self.terminal_side_reserved = true;
            }
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
    fn goal_stop_uses_candidate_arc_length_after_a_lateral_return() {
        let ego = EgoState {
            pose: rustdrive_core::Pose {
                position: Vec2::new(70.0, 3.5),
                yaw: 0.0,
            },
            speed: 2.0,
        };
        let path = LatticePlanner::default().plan(ego, &road(5.5), &[]);
        assert_eq!(path.lateral_target, 0.0);
        let end = path.points.last().unwrap();
        assert!(end.position.distance(Vec2::new(99.0, 0.0)) < 1e-8);
        assert_eq!(end.speed, 0.0);
        assert!((end.time - path.points[path.points.len() - 2].time - 8.0).abs() < 1e-8);
    }

    #[test]
    fn approaching_traffic_reserves_a_terminal_side_before_the_forecast_reaches_the_goal() {
        let route = road(5.5);
        let mut planner = LatticePlanner {
            cruise_speed: 6.0,
            ..LatticePlanner::default()
        };
        let mut ego = EgoState {
            speed: 6.0,
            ..EgoState::default()
        };
        ego.pose.position.x = 70.0;
        let traffic = Prediction {
            id: 1,
            radius: 1.0,
            dt: 0.2,
            positions: (0..=40)
                .map(|i| Vec2::new(62.0 + 0.5 * i as f64, 0.0))
                .collect(),
        };
        assert!(traffic.positions.last().unwrap().x < 99.0);
        let path = planner.plan(ego, &route, std::slice::from_ref(&traffic));
        assert!(path.lateral_target.abs() > 3.0);
        let end = path.points.last().unwrap();
        assert!((end.position.x - 99.0).abs() < 1e-8);
        assert!((end.position.y.abs() - 3.5).abs() < 1e-8);
        assert!(
            collision::first_contact_time(
                &path.points,
                &traffic,
                planner.vehicle.radius,
                Vec2::new(ego.pose.yaw.cos(), ego.pose.yaw.sin()),
            )
            .is_none()
        );
        // A missed track cannot induce a late center return to the reserved stop.
        ego.pose.position = Vec2::new(85.0, path.lateral_target);
        let held = planner.plan(ego, &route, &[]);
        assert_eq!(held.lateral_target, path.lateral_target);
        // The reservation remains a preference: a new obstacle on that side
        // must still reject its path, rather than bypass the stationary sweep.
        let blocked = Prediction {
            positions: vec![Vec2::new(99.0, path.lateral_target); 41],
            radius: 2.0,
            ..traffic
        };
        let rejected = planner.plan(ego, &route, std::slice::from_ref(&blocked));
        assert!(
            rejected.mode == DrivingMode::Emergency
                || collision::first_contact_time(
                    &rejected.points,
                    &blocked,
                    planner.vehicle.radius,
                    Vec2::new(ego.pose.yaw.cos(), ego.pose.yaw.sin()),
                )
                .is_none()
        );
        assert!(rejected.points.last().is_none_or(|p| p.position.x < 98.0));
    }

    #[test]
    fn terminal_reservation_ignores_stationary_departing_and_off_center_tracks() {
        let mut ego = EgoState {
            speed: 6.0,
            ..EgoState::default()
        };
        ego.pose.position.x = 70.0;
        for (velocity, y) in [(0.0, 0.0), (-2.5, 0.0), (2.5, 3.0)] {
            let object = Prediction {
                id: 1,
                radius: 1.0,
                dt: 0.2,
                positions: (0..=40)
                    .map(|i| Vec2::new(62.0 + velocity * 0.2 * i as f64, y))
                    .collect(),
            };
            let path = LatticePlanner::default().plan(ego, &road(5.5), &[object]);
            assert_eq!(path.lateral_target, 0.0);
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
        assert!((next.points[20].position.y - first.points[32].position.y).abs() < 1e-9);
    }

    #[test]
    fn forecast_changes_do_not_abort_an_unfinished_pass() {
        let route = road(5.5);
        let mut planner = LatticePlanner {
            cruise_speed: 6.0,
            ..LatticePlanner::default()
        };
        let mut ego = EgoState {
            speed: 6.0,
            ..EgoState::default()
        };
        let first = planner.plan(ego, &route, &[obstacle()]);
        assert_eq!(first.mode, DrivingMode::Avoid);
        let side = first.lateral_target;
        ego.pose.position = Vec2::new(6.0, side.signum() * 0.2);
        // An apparent rapidly departing object makes the center candidate clear.
        // A fresh planner confirms that candidate is feasible; the established
        // pass must remain preferred while its observed position is still ahead.
        let mut departing = obstacle();
        for (i, p) in departing.positions.iter_mut().enumerate() {
            p.x += 10.0 * departing.dt * i as f64;
        }
        let fresh = LatticePlanner {
            cruise_speed: 6.0,
            ..LatticePlanner::default()
        }
        .plan(ego, &route, &[departing.clone()]);
        assert_eq!(fresh.lateral_target, 0.0);
        let continued = planner.plan(ego, &route, &[departing.clone()]);
        assert_eq!(continued.lateral_target, side);
        assert_eq!(continued.mode, DrivingMode::Avoid);
        // Completion releases the preference even before passing the object;
        // it must not hold an offset indefinitely or mask a safer center return.
        ego.pose.position = Vec2::new(16.0, side);
        let completed = planner.plan(ego, &route, &[departing]);
        assert_eq!(completed.lateral_target, 0.0);
        ego.pose.position = Vec2::new(25.0, side);
        let passed = planner.plan(ego, &route, &[obstacle()]);
        assert_eq!(passed.lateral_target, 0.0);
        assert_eq!(passed.mode, DrivingMode::Cruise);
    }

    #[test]
    fn pass_preference_never_overrides_collision_feasibility() {
        let mut planner = LatticePlanner::default();
        let route = road(5.5);
        planner.plan(EgoState::default(), &route, &[obstacle()]);
        let mut blocking = obstacle();
        blocking.radius = 8.0;
        let stopped = planner.plan(EgoState::default(), &route, &[blocking.clone()]);
        assert!(matches!(
            stopped.mode,
            DrivingMode::Yield | DrivingMode::Emergency
        ));
        if !stopped.points.is_empty() {
            assert_eq!(stopped.points.last().unwrap().speed, 0.0);
            assert!(
                collision::first_contact_time(
                    &stopped.points,
                    &blocking,
                    planner.vehicle.radius,
                    Vec2::new(1.0, 0.0),
                )
                .is_none()
            );
        }
    }

    #[test]
    fn an_object_clear_of_the_centerline_does_not_keep_the_vehicle_offset() {
        let route = road(5.5);
        let mut planner = LatticePlanner::default();
        let first = planner.plan(EgoState::default(), &route, &[obstacle()]);
        let mut ego = EgoState::default();
        ego.pose.position = Vec2::new(6.0, first.lateral_target.signum() * 0.2);
        let mut off_center = obstacle();
        off_center.positions.fill(Vec2::new(20.0, 3.0));
        let clear = planner.plan(ego, &route, &[off_center]);
        assert_eq!(clear.lateral_target, 0.0);
        assert_eq!(clear.mode, DrivingMode::Cruise);
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
    fn path_joins_the_measured_heading_and_recovers_to_the_route() {
        let mut ego = EgoState {
            speed: 4.0,
            ..EgoState::default()
        };
        ego.pose.yaw = 0.15;
        let path = LatticePlanner::default().plan(ego, &road(5.5), &[]);
        let delta = path.points[1].position.minus(path.points[0].position);
        assert!((delta.y.atan2(delta.x) - ego.pose.yaw).abs() < 0.005);
        assert!(path.points[20].position.y.abs() < 1e-10);
    }
    #[test]
    fn goal_recovery_brakes_monotonically_within_the_remaining_corridor() {
        let mut ego = EgoState {
            speed: 0.6,
            ..EgoState::default()
        };
        ego.pose.position.x = 99.05;
        let path = LatticePlanner::default().plan(ego, &road(5.5), &[]);
        assert_eq!(path.mode, DrivingMode::Goal);
        assert_eq!(path.points[0].speed, ego.speed);
        assert!(path.points.windows(2).all(|p| p[1].speed <= p[0].speed));
        assert_eq!(path.points.last().unwrap().speed, 0.0);
        assert!(path.points.last().unwrap().position.x < 100.0);
        ego.pose.position.x = 99.98;
        ego.speed = 1.0;
        assert_eq!(
            LatticePlanner::default().plan(ego, &road(5.5), &[]).mode,
            DrivingMode::Emergency
        );
    }
    #[test]
    fn invalid_heading_rejects_the_candidate() {
        let mut ego = EgoState::default();
        ego.pose.yaw = f64::NAN;
        assert_eq!(
            LatticePlanner::default().plan(ego, &road(5.5), &[]).mode,
            DrivingMode::Emergency
        );
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
