//! Receding-horizon lateral lattice with time-indexed collision envelopes.
mod checked_approach;
mod collision;
mod corners;
mod diagnostics;
mod geometry;
mod speed;
mod speed_zones;
mod stopping;
use rustdriving_core::{
    CandidateDisposition, DrivingMode, EgoState, Planner, PlanningDiagnostics, PlanningFailure,
    PlanningForecastConflict, Prediction, RefinementConfirmationDiagnostic, Route, Trajectory,
    VehicleConfig,
};
/// Refined proposals retain at least one meter beyond acquired forecast discs.
pub const REFINED_CANDIDATE_CLEARANCE_M: f64 = 1.0;
#[derive(Clone)]
pub struct LatticePlanner {
    /// Known fixed-route arc intervals; overlaps use the lowest cap.
    pub speed_zones: Vec<rustdriving_core::RouteSpeedZone>,
    pub cruise_speed: f64,
    /// Opt-in locally bounded fillets for sparse imported road polylines.
    pub local_route_geometry: bool,
    /// Opt-in anchored recovery for a point/rear-axle pose reference. Chassis
    /// course tracking retains the original join except for narrowed width-fitted
    /// candidates, which also recover by their anchored completion arc.
    pub point_reference_recovery: bool,
    /// Finish a near-stationary goal within the terminal position tolerance.
    /// Disable for temporary regulatory or route-handover stopping prefixes.
    pub terminal_settling: bool,
    /// After collision-induced emergency braking, retain a feasible immediate
    /// stop until a complete unblocked candidate clears the ordinary sweep.
    pub settle_obstruction_stops: bool,
    /// Search earlier reachable stops after the ordinary retimed stop fails its
    /// complete collision sweep. Requires obstruction-stop retention.
    pub search_obstruction_stops: bool,
    /// Confirm a newly checked stopping approach using fresh scene acquisitions.
    pub replan_checked_approaches: bool,
    /// Optional additional road-edge reserve for width-fitted side candidates.
    /// None preserves the original fixed +/-3.5 m offsets.
    pub lateral_candidate_margin_m: Option<f64>,
    /// Add bounded intermediate offsets inside the original width-fitted sides.
    pub refine_lateral_candidates: bool,
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
    obstruction_stop_requested: bool,
    obstruction_stop_active: bool,
    searched_stop_s: Option<f64>,
    refinement_stamp: Option<f64>,
    refinement_support: Option<RefinementSupport>,
    diagnostics_enabled: bool,
    diagnostics: Option<PlanningDiagnostics>,
    approach_clock: Option<(f64, Option<f64>)>,
    approach_confirmation: checked_approach::Confirmation,
    prefer_stop_progress: bool,
}
#[derive(Clone)]
struct RefinementSupport {
    target: f64,
    first: f64,
    last: f64,
    count: u8,
}
struct Selection {
    index: u8,
    cost: (bool, bool, bool, f64, f64),
    trajectory: Trajectory,
    blocked: bool,
    searched: bool,
    stop_s: Option<f64>,
}
impl Default for LatticePlanner {
    fn default() -> Self {
        Self {
            speed_zones: vec![],
            cruise_speed: 8.0,
            local_route_geometry: false,
            point_reference_recovery: false,
            terminal_settling: true,
            settle_obstruction_stops: false,
            search_obstruction_stops: false,
            replan_checked_approaches: false,
            lateral_candidate_margin_m: None,
            refine_lateral_candidates: false,
            vehicle: VehicleConfig::default(),
            max_acceleration_m_s2: 2.0,
            max_deceleration_m_s2: 2.5,
            max_lateral_acceleration_m_s2: None,
            previous_lateral: 0.0,
            maneuver_start: None,
            yield_stop_s: None,
            terminal_side_reserved: false,
            obstruction_stop_requested: false,
            obstruction_stop_active: false,
            searched_stop_s: None,
            refinement_stamp: None,
            refinement_support: None,
            diagnostics_enabled: false,
            diagnostics: None,
            approach_clock: None,
            approach_confirmation: checked_approach::Confirmation::default(),
            prefer_stop_progress: false,
        }
    }
}
impl LatticePlanner {
    /// Record bounded decision summaries without changing candidate selection.
    pub fn set_diagnostics_enabled(&mut self, enabled: bool) {
        self.diagnostics_enabled = enabled;
        self.diagnostics = None;
    }

    pub fn diagnostics(&self) -> Option<&PlanningDiagnostics> {
        self.diagnostics.as_ref()
    }

    /// Supply current control time and a healthy, supported scene acquisition.
    /// None consumes a fault watermark; a cached scene cannot authorize restart.
    pub fn observe_checked_approach_scene(&mut self, now: f64, scene: Option<f64>) {
        self.approach_clock = Some((now, scene));
    }

    pub fn checked_approach_diagnostics(&self) -> &rustdriving_core::CheckedApproachDiagnostics {
        &self.approach_confirmation.diagnostics
    }

    /// Clear route-dependent maneuver history without withdrawing an unfinished
    /// collision brake. The new route must still offer a completely clear path.
    pub fn reset_route_state(&mut self) {
        self.previous_lateral = 0.0;
        self.maneuver_start = None;
        self.yield_stop_s = None;
        self.terminal_side_reserved = false;
        self.searched_stop_s = None;
        self.refinement_stamp = None;
        self.refinement_support = None;
        self.diagnostics = None;
        self.approach_confirmation.reset();
    }

    /// Supply the last accepted scene acquisition, never the control clock.
    /// Missing, invalid or regressing context removes only refinement support.
    pub fn observe_refinement_scene(&mut self, stamp: Option<f64>) {
        if stamp.is_none_or(|t| {
            !t.is_finite() || t < 0. || self.refinement_stamp.is_some_and(|old| t < old)
        }) {
            self.refinement_stamp = None;
            self.refinement_support = None;
        } else {
            self.refinement_stamp = stamp;
        }
    }

    fn refinement_confirmed(&mut self, target: f64) -> bool {
        let Some(stamp) = self.refinement_stamp else {
            self.refinement_support = None;
            return false;
        };
        let restart = self.refinement_support.as_ref().is_none_or(|s| {
            (s.target - target).abs() > 0.01 || stamp - s.last > 0.35 + 1e-9 || stamp < s.last
        });
        if restart {
            self.refinement_support = Some(RefinementSupport {
                target,
                first: stamp,
                last: stamp,
                count: 1,
            });
        }
        let support = self
            .refinement_support
            .as_mut()
            .expect("initialized refinement support");
        if stamp > support.last {
            support.count = support.count.saturating_add(1).min(3);
            support.last = stamp;
        }
        support.count >= 3 && support.last - support.first >= 0.2 - 1e-9
    }

    pub fn holding_obstruction_stop(&self) -> bool {
        self.settle_obstruction_stops && self.obstruction_stop_active
    }
    pub fn braking_obstruction_stop(&self) -> bool {
        self.search_obstruction_stops && self.obstruction_stop_requested
    }
}
impl Planner for LatticePlanner {
    fn plan(&mut self, ego: EgoState, route: &Route, objects: &[Prediction]) -> Trajectory {
        if !self.replan_checked_approaches {
            return self.plan_inner(ego, route, objects);
        }
        let capture = self.diagnostics_enabled;
        self.diagnostics_enabled = true;
        let mut result = self.plan_inner(ego, route, objects);
        let stopped = ego.speed.abs() <= 0.2
            && result.mode == DrivingMode::Yield
            && result.points.len() >= 2
            && result.points.iter().all(|p| {
                (0.0..=0.2).contains(&p.speed) && p.position.distance(ego.pose.position) <= 0.2
            });
        let mut alternative = None;
        let mut proposal = None;
        if stopped {
            let mut probe = self.clone();
            probe.obstruction_stop_requested = false;
            probe.obstruction_stop_active = false;
            probe.searched_stop_s = None;
            probe.yield_stop_s = None;
            probe.prefer_stop_progress = true;
            let path = probe.plan_inner(ego, route, objects);
            if let Some(record) = probe.diagnostics.as_ref()
                && let Some(chosen) = record
                    .selected_candidate
                    .and_then(|i| record.candidates.get(i as usize))
                && chosen.disposition == CandidateDisposition::CheckedStop
                && let Some(endpoint) = path.points.last()
                && endpoint.speed == 0.
                && path.points.len() >= 2
                && endpoint.position == path.points[path.points.len() - 2].position
                && endpoint.time - path.points[path.points.len() - 2].time >= 8. - 1e-9
            {
                let mut ids: Vec<_> = record
                    .candidates
                    .iter()
                    .filter_map(|c| {
                        c.nominal_forecast_conflict
                            .as_ref()
                            .map(|f| f.prediction_id)
                    })
                    .collect();
                ids.sort_unstable();
                ids.dedup();
                proposal = Some(checked_approach::Proposal {
                    endpoint: endpoint.position,
                    end_progress: route.project(endpoint.position).0,
                    target: path.lateral_target,
                    ids,
                });
                probe.prefer_stop_progress = false;
                alternative = Some((probe, path));
            }
        }
        if self.approach_confirmation.observe(
            self.approach_clock,
            ego.pose.position,
            route.project(ego.pose.position).0,
            proposal,
        ) {
            let confirmation = self.approach_confirmation.clone();
            let (probe, path) = alternative.expect("confirmed checked proposal");
            *self = probe;
            self.approach_confirmation = confirmation;
            result = path;
        }
        self.diagnostics_enabled = capture;
        if !capture {
            self.diagnostics = None;
        }
        result
    }
}
impl LatticePlanner {
    fn plan_inner(&mut self, ego: EgoState, route: &Route, objects: &[Prediction]) -> Trajectory {
        self.diagnostics = None;
        let mut diagnostics = self.diagnostics_enabled.then(|| PlanningDiagnostics {
            scene_acquisition_s: self.refinement_stamp,
            obstruction_stop_requested: self.obstruction_stop_requested,
            obstruction_stop_active: self.obstruction_stop_active,
            ..PlanningDiagnostics::default()
        });
        if !ego.speed.is_finite()
            || !ego.pose.position.finite()
            || !ego.pose.yaw.is_finite()
            || !self.cruise_speed.is_finite()
            || self.cruise_speed <= 0.0
            || (self.search_obstruction_stops && !self.settle_obstruction_stops)
            || (self.replan_checked_approaches
                && !(self.search_obstruction_stops
                    && self.settle_obstruction_stops
                    && self.lateral_candidate_margin_m.is_some()))
            || (self.refine_lateral_candidates && self.lateral_candidate_margin_m.is_none())
            || !self.max_acceleration_m_s2.is_finite()
            || self.max_acceleration_m_s2 <= 0.0
            || !self.max_deceleration_m_s2.is_finite()
            || self.max_deceleration_m_s2 <= 0.0
            || self.speed_zones.len() > 128
            || self.speed_zones.iter().any(|zone| !zone.valid())
            || self
                .lateral_candidate_margin_m
                .is_some_and(|margin| !margin.is_finite() || !(0.1..=1.0).contains(&margin))
            || self
                .max_lateral_acceleration_m_s2
                .is_some_and(|a| !a.is_finite() || a <= 0.0)
            || objects.iter().any(|object| {
                object.positions.is_empty()
                    || !object.dt.is_finite()
                    || object.dt <= 0.0
                    || !object.valid_radii()
                    || !(object.dt * (object.positions.len() - 1) as f64).is_finite()
                    || object.positions.iter().any(|position| !position.finite())
            })
        {
            if let Some(record) = &mut diagnostics {
                record.failure = Some(PlanningFailure::InvalidInput);
            }
            self.diagnostics = diagnostics;
            return Trajectory {
                points: vec![],
                mode: DrivingMode::Emergency,
                lateral_target: 0.0,
            };
        }
        let supplied_route = route;
        let local_route;
        let route = if self.local_route_geometry {
            match corners::fit(route, self.vehicle) {
                Ok(fitted) => {
                    local_route = fitted;
                    &local_route
                }
                Err(_) => {
                    if let Some(record) = &mut diagnostics {
                        record.failure = Some(PlanningFailure::InvalidLocalGeometry);
                    }
                    self.diagnostics = diagnostics;
                    return Trajectory {
                        points: vec![],
                        mode: DrivingMode::Emergency,
                        lateral_target: 0.0,
                    };
                }
            }
        } else {
            route
        };
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
        if let Some(record) = &mut diagnostics {
            record.scene_acquisition_s = self.refinement_stamp;
            record.route_progress_m = Some(supplied_route.project(ego.pose.position).0);
            record.planning_span_m = Some(horizon);
        }
        let limits = speed::Limits {
            zones: (!self.speed_zones.is_empty()).then_some((
                supplied_route,
                self.speed_zones.as_slice(),
                self.vehicle.radius,
            )),
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
                    && object_lateral.abs() < object.radius_at(0.0) + 0.3
                    && object
                        .positions
                        .get(1)
                        .is_some_and(|next| route.project(*next).0 - object_s > 0.7 * object.dt)
            });
        let reserve_terminal_side = terminal_traffic || self.terminal_side_reserved;
        let hold_requested = self.settle_obstruction_stops
            && (self.obstruction_stop_active
                || (self.obstruction_stop_requested && ego.speed <= 0.2));
        // A searched alternative is a commitment to brake, including before
        // reaching standstill. Ordinary blocked approaches retain their prior
        // behavior until a search or collision emergency actually requests it.
        let mut collision_seen = false;
        let mut best: Option<Selection> = None;
        let mut best_refined: Option<Selection> = None;
        let mut legacy_needs_refinement = false;
        let side = self.lateral_candidate_margin_m.map_or(3.5, |margin| {
            (route.half_width - self.vehicle.radius - margin).clamp(
                0.0,
                if self.replan_checked_approaches {
                    4.0
                } else {
                    3.5
                },
            )
        });
        let fitted_chassis_recovery =
            self.local_route_geometry && !self.point_reference_recovery && side < 3.5;
        for (index, fraction) in [0.0, 1.0, -1.0, 0.875, -0.875, 0.75, -0.75, 0.5, -0.5]
            .into_iter()
            .enumerate()
        {
            let refined = index >= 3;
            if index == 3 && self.refine_lateral_candidates {
                legacy_needs_refinement = best.as_ref().is_none_or(|choice| {
                    choice.blocked
                        || objects.iter().any(|object| {
                            collision::with_minimum_clearance(
                                &choice.trajectory.points,
                                object,
                                self.vehicle.radius,
                                rustdriving_core::Vec2::new(ego.pose.yaw.cos(), ego.pose.yaw.sin()),
                                REFINED_CANDIDATE_CLEARANCE_M,
                            )
                            .is_some()
                        })
                });
            }
            if refined && !self.refine_lateral_candidates {
                continue;
            }
            let target = side * fraction;
            let mut trace = diagnostics::CandidateTrace::new(diagnostics.as_mut(), index, target);
            // Avoid duplicate centerline paths when no useful side corridor fits.
            if index > 0 && target.abs() <= 0.1 {
                trace.reject(CandidateDisposition::NoSideRoom);
                continue;
            }
            if target.abs() + self.vehicle.radius > route.half_width {
                trace.reject(CandidateDisposition::OutsideTargetCorridor);
                continue;
            }
            if target * lateral < 0.0 && lateral.abs() > 1.0 {
                trace.reject(CandidateDisposition::OppositeSideDuringPass);
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
            let base = |distance| {
                if self.local_route_geometry {
                    route.sample(s + distance, anchored_offset(distance)).0
                } else {
                    geometry::sample(route, s + distance, anchored_offset(distance))
                }
            };
            let base_initial = base(0.0);
            let base_tangent = base(0.01).minus(base_initial).scaled(100.0);
            let position_error = ego.pose.position.minus(base_initial);
            let desired_tangent =
                rustdriving_core::Vec2::new(ego.pose.yaw.cos(), ego.pose.yaw.sin())
                    .scaled(base_tangent.x.hypot(base_tangent.y));
            let tangent_error = desired_tangent.minus(base_tangent);
            // Retain the maneuver's original completion arc. Reapplying its
            // whole transition at every local side-path tick would continually
            // defer correction of the remaining tracking error. A wheelbase
            // floor leaves a finite recovery span after the anchor has passed.
            let correction_transition = if self.local_route_geometry
                && (self.point_reference_recovery
                    || ((fitted_chassis_recovery || refined) && s >= start_s + transition))
                && target.abs() > 0.1
            {
                // Reuse the quintic's existing second-derivative bound for
                // the residual shift as well. A short recovery span must not
                // ignore the configured plant's lateral acceleration budget.
                let recovery_floor =
                    self.max_lateral_acceleration_m_s2
                        .map_or(self.vehicle.wheelbase, |limit| {
                            self.vehicle.wheelbase.max(
                                ego.speed
                                    * (6.0 * position_error.x.hypot(position_error.y) / limit)
                                        .sqrt(),
                            )
                        });
                let recovery = (start_s + transition - s).max(recovery_floor);
                if self.point_reference_recovery {
                    recovery.min(transition)
                } else {
                    // A temporary center brake can increase the residual while
                    // preserving the passing intent. Its recovery may need more
                    // room than the original shift, under the same speed limits.
                    recovery
                }
            } else {
                transition
            };
            let mut geometry = Vec::new();
            let mut contained = true;
            for i in 0..=80 {
                let ds = horizon * i as f64 / 80.0;
                let position = if i == 0 {
                    ego.pose.position
                } else {
                    base(ds)
                        .plus(position_error.scaled(1.0 - quintic(ds / correction_transition)))
                        .plus(
                            tangent_error
                                .scaled(geometry::heading_weight(ds, correction_transition)),
                        )
                };
                // Interpolation and heading correction may leave the supplied corridor;
                // check the actual candidate against the unchanged route evaluator.
                let corridor_radius = route.half_width - self.vehicle.radius;
                // Distance to any point on the centerline is an upper bound on
                // nearest-centerline distance. Most samples can be accepted with
                // this bound; ambiguous ones still use the full corridor projection.
                let reference = supplied_route.sample(s + ds, 0.0).0;
                if position.distance(reference) > corridor_radius
                    && supplied_route.project(position).1.abs() > corridor_radius
                {
                    contained = false;
                    break;
                }
                geometry.push(position);
            }
            if !contained {
                trace.reject(CandidateDisposition::GeometryOutsideCorridor);
                continue;
            }
            // Route progress is not arc length when the candidate shifts laterally.
            // Stop at the generated terminal position, rather than truncating a
            // longer candidate using the route's longitudinal distance.
            let goal_stop =
                (remaining <= 41.0).then(|| geometry.windows(2).map(|p| p[0].distance(p[1])).sum());
            // Once almost stopped and aligned within 0.5 m of the desired goal,
            // finish braking instead of accelerating to correct tiny position
            // errors. Other lateral candidates can still escape a forecast
            // conflict. A stop/hold that fails the sweep remains infeasible.
            let settling_goal = self.terminal_settling
                && (remaining - 1.0).abs() <= 0.5
                && ego.speed <= 0.2
                && (target - lateral).abs() <= 0.2
                && rustdriving_core::wrap_angle(
                    ego.pose.yaw - base_tangent.y.atan2(base_tangent.x),
                )
                .abs()
                    <= 0.1;
            let candidate_limits = speed::Limits {
                zones: limits.zones,
                cruise: if settling_goal { 0.0 } else { limits.cruise },
                acceleration: limits.acceleration,
                deceleration: limits.deceleration,
                lateral: limits.lateral,
            };
            let Some(mut points) =
                speed::profile(&geometry, ego.speed.max(0.0), &candidate_limits, goal_stop)
            else {
                trace.reject(CandidateDisposition::UnreachableSpeedProfile);
                continue;
            };
            let contact = |path: &[rustdriving_core::TrajectoryPoint]| {
                objects
                    .iter()
                    .filter_map(|object| {
                        let heading =
                            rustdriving_core::Vec2::new(ego.pose.yaw.cos(), ego.pose.yaw.sin());
                        if refined {
                            collision::with_minimum_clearance(
                                path,
                                object,
                                self.vehicle.radius,
                                heading,
                                REFINED_CANDIDATE_CLEARANCE_M,
                            )
                        } else {
                            collision::first_contact_time(
                                path,
                                object,
                                self.vehicle.radius,
                                heading,
                            )
                        }
                    })
                    .fold(f64::INFINITY, f64::min)
            };
            let first_contact = contact(&points);
            let blocked = first_contact.is_finite();
            if trace.enabled() && blocked {
                // Reuse the sweep solely for recording the responsible identity;
                // these read-only calls cannot influence the selected profile.
                let floor = if refined {
                    REFINED_CANDIDATE_CLEARANCE_M
                } else {
                    0.0
                };
                let heading = rustdriving_core::Vec2::new(ego.pose.yaw.cos(), ego.pose.yaw.sin());
                let conflict = objects
                    .iter()
                    .filter_map(|object| {
                        collision::with_minimum_clearance(
                            &points,
                            object,
                            self.vehicle.radius,
                            heading,
                            floor,
                        )
                        .map(|time| PlanningForecastConflict {
                            prediction_id: object.id,
                            relative_time_s: time,
                            forecast_radius_m: object.radius_at(time),
                            forecast_horizon_s: object.dt * (object.positions.len() - 1) as f64,
                            minimum_clearance_floor_m: floor,
                        })
                    })
                    .min_by(|a, b| a.relative_time_s.total_cmp(&b.relative_time_s));
                trace.conflict(conflict);
            }
            // Refinement offers a complete checked escape. Rejected added
            // proposals cannot replace or arm the established stopping duty.
            if refined && blocked {
                trace.reject(CandidateDisposition::RefinementForecastConflict);
                continue;
            }
            let mut searched = false;
            let mut used_stop_search = false;
            if blocked {
                collision_seen = true;
                let mut stop = if hold_requested {
                    let speed = ego.speed.max(0.0);
                    if speed == 0.0 {
                        0.0
                    } else {
                        // Keep a representable braking segment at tiny positive
                        // speeds, followed by the complete stationary hold.
                        (speed.powi(2) / (2.0 * self.max_deceleration_m_s2)).max(1e-7)
                    }
                } else if ego.speed < 0.15
                    && self.yield_stop_s.is_some_and(|stop_s| s >= stop_s - 0.2)
                {
                    ego.speed.max(0.0).powi(2) / (2.0 * self.max_deceleration_m_s2)
                } else {
                    (speed::distance_at(&points, first_contact) - 4.0).max(0.0)
                };
                if self.braking_obstruction_stop() && !hold_requested {
                    stop = self.searched_stop_s.map_or_else(
                        || ego.speed.max(0.0).powi(2) / (2.0 * self.max_deceleration_m_s2),
                        |anchor| {
                            stop.min(stopping::distance_at_progress(
                                &geometry,
                                horizon,
                                anchor - s,
                            ))
                        },
                    );
                }
                let stopping_limits = speed::Limits {
                    cruise: if hold_requested {
                        0.0
                    } else if self.braking_obstruction_stop() {
                        candidate_limits.cruise.min(ego.speed.max(0.0))
                    } else {
                        candidate_limits.cruise
                    },
                    ..candidate_limits
                };
                let Some((stopped, used_search)) = stopping::profile(
                    &geometry,
                    ego.speed.max(0.0),
                    &stopping_limits,
                    stop,
                    self.search_obstruction_stops && !hold_requested,
                    |path| !contact(path).is_finite(),
                ) else {
                    trace.reject(CandidateDisposition::NoCheckedStop);
                    continue;
                };
                points = stopped;
                used_stop_search = used_search;
                searched = used_search && ego.speed > 0.2;
            }
            trace.accept(&points, supplied_route, blocked, used_stop_search);
            let permitted = points.iter().map(|p| p.speed).fold(0.0_f64, f64::max);
            let switching_penalty = if target * self.previous_lateral < 0.0 {
                30.0
            } else {
                0.0
            };
            // Keep the passing side through an unfinished shift and until a
            // slower centerline object is behind the vehicle. Completing the
            // lateral shift alone must not trigger an early return alongside it.
            // This remains a preference; every candidate is swept and retimed.
            let unfinished_pass = self
                .maneuver_start
                .is_some_and(|(start_s, _, transition)| s < start_s + transition);
            let abandoning_pass = target.abs() < 0.01
                && self.previous_lateral.abs() > 0.1
                && objects.iter().any(|object| {
                    let (object_s, object_lateral) = route.project(object.positions[0]);
                    object_s + object.radius + self.vehicle.radius >= s
                        && object_s <= s + horizon
                        && object_lateral.abs() < object.radius + 0.3
                        && (unfinished_pass
                            || object.positions.get(1).is_none_or(|next| {
                                route.project(*next).0 - object_s <= ego.speed * object.dt
                            }))
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
            let stop_s = (self.search_obstruction_stops && blocked).then(|| {
                let distance = traj
                    .points
                    .windows(2)
                    .map(|p| p[0].position.distance(p[1].position))
                    .sum();
                s + stopping::progress_at_distance(&geometry, horizon, distance)
            });
            // Prefer an already settled stopping place when its complete hold
            // is feasible. Speed rewards must not restart a completed maneuver.
            // A genuinely unblocked route can release the stop, including a
            // safe lateral escape. A retimed approach to the same obstruction
            // cannot restart a completed emergency stop for a speed reward.
            let cost = (
                (hold_requested || self.prefer_stop_progress) && blocked,
                !settling_goal,
                !(refined && legacy_needs_refinement),
                if self.prefer_stop_progress && blocked {
                    -supplied_route
                        .project(traj.points.last().expect("profile endpoint").position)
                        .0
                } else {
                    0.
                },
                score,
            );
            let selection = if refined {
                &mut best_refined
            } else {
                &mut best
            };
            if selection
                .as_ref()
                .is_none_or(|previous| cost < previous.cost)
            {
                *selection = Some(Selection {
                    index: index as u8,
                    cost,
                    trajectory: traj,
                    blocked,
                    searched,
                    stop_s,
                });
            }
        }
        if let Some(choice) = best_refined
            .filter(|candidate| best.as_ref().is_none_or(|old| candidate.cost < old.cost))
        {
            if self.refinement_confirmed(choice.trajectory.lateral_target) {
                best = Some(choice);
            }
        } else {
            self.refinement_support = None;
        }
        if let Some(record) = &mut diagnostics {
            record.selected_candidate = best.as_ref().map(|choice| choice.index);
            if best.is_none() {
                record.failure = Some(PlanningFailure::NoSelectableCandidate);
            }
            record.refinement_confirmation =
                self.refinement_support
                    .as_ref()
                    .map(|support| RefinementConfirmationDiagnostic {
                        lateral_target_m: support.target,
                        acquisition_count: support.count,
                        first_acquisition_s: support.first,
                        last_acquisition_s: support.last,
                        confirmed: support.count >= 3 && support.last - support.first >= 0.2 - 1e-9,
                    });
        }
        let (blocked, searched, stop_s, trajectory) = best
            .map(|choice| {
                (
                    choice.blocked,
                    choice.searched,
                    choice.stop_s,
                    choice.trajectory,
                )
            })
            .unwrap_or((
                false,
                false,
                None,
                Trajectory {
                    points: vec![],
                    mode: DrivingMode::Emergency,
                    lateral_target: 0.0,
                },
            ));
        self.obstruction_stop_requested = self.settle_obstruction_stops
            && if trajectory.mode == DrivingMode::Emergency {
                self.obstruction_stop_requested || collision_seen
            } else {
                blocked && (searched || self.obstruction_stop_requested)
            };
        self.obstruction_stop_active = self.obstruction_stop_requested && hold_requested;
        if self.search_obstruction_stops {
            self.searched_stop_s = if !self.obstruction_stop_requested {
                None
            } else {
                match (self.searched_stop_s, stop_s) {
                    (Some(old), Some(new)) => Some(old.min(new)),
                    (old, new) => old.or(new),
                }
            };
        }
        if trajectory.mode != DrivingMode::Emergency {
            if terminal_traffic && trajectory.lateral_target.abs() > 0.1 {
                // Keep the stopping-place preference through forecast jitter,
                // missed tracks, and the actor becoming stationary. Feasibility
                // (including the complete hold sweep) always takes precedence.
                self.terminal_side_reserved = true;
            }
            // A retimed centerline stopping prefix is not a completed return.
            // Keep the narrowed chassis pass's completion arc through it. Each
            // subsequent candidate still joins the measured course and must
            // independently pass containment, speed and complete hold sweeps.
            let temporary_center_brake = (self.replan_checked_approaches
                || fitted_chassis_recovery
                || (self.refine_lateral_candidates
                    && self.local_route_geometry
                    && !self.point_reference_recovery
                    && self.previous_lateral.abs() < side - 0.01))
                && blocked
                && trajectory.lateral_target.abs() < 0.01
                && self.previous_lateral.abs() > 0.1;
            if !temporary_center_brake {
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
        if let Some(record) = &mut diagnostics {
            record.obstruction_stop_requested = self.obstruction_stop_requested;
            record.obstruction_stop_active = self.obstruction_stop_active;
        }
        self.diagnostics = diagnostics;
        trajectory
    }
}
/// Validate the optional planner-local geometry without changing map/evaluation coordinates.
pub fn validate_local_route_geometry(route: &Route, vehicle: VehicleConfig) -> Result<(), String> {
    corners::fit(route, vehicle).map(|_| ())
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
    use rustdriving_core::Vec2;
    fn road(width: f64) -> Route {
        Route::new(vec![Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0)], width).unwrap()
    }
    fn obstacle() -> Prediction {
        Prediction {
            additional_radius_m: vec![],
            id: 1,
            positions: vec![Vec2::new(20.0, 0.0); 50],
            radius: 1.0,
            dt: 0.2,
        }
    }
    #[test]
    fn local_side_path_reanchoring_does_not_defer_the_original_completion_arc() {
        for direction in [-1., 1.] {
            for s in [9., 12.] {
                let mut planner = LatticePlanner {
                    local_route_geometry: true,
                    point_reference_recovery: true,
                    previous_lateral: direction * 3.5,
                    maneuver_start: Some((0., 0., 10.)),
                    ..LatticePlanner::default()
                };
                let ego = EgoState {
                    pose: rustdriving_core::Pose {
                        position: Vec2::new(s, direction * 2.),
                        yaw: 0.,
                    },
                    speed: 2.,
                };
                let path = planner.plan(ego, &road(5.5), &[obstacle()]);
                assert_eq!(path.lateral_target, direction * 3.5);
                assert_eq!(path.points[0].position, ego.pose.position);
                // The original anchor is finished at x=10. Reanchoring must
                // disappear within one wheelbase here, rather than ten meters
                // later at every successive planning tick.
                for point in &path.points {
                    if point.position.x >= s + planner.vehicle.wheelbase {
                        assert!((point.position.y - direction * 3.5).abs() < 1e-12);
                    }
                }
            }
        }
    }
    #[test]
    fn narrowed_chassis_side_path_recovers_by_the_anchored_completion_arc() {
        for direction in [-1., 1.] {
            for s in [10., 12.] {
                let mut planner = LatticePlanner {
                    cruise_speed: 4.,
                    local_route_geometry: true,
                    point_reference_recovery: false,
                    lateral_candidate_margin_m: Some(0.35),
                    max_lateral_acceleration_m_s2: Some(2.5),
                    previous_lateral: direction * 2.9,
                    maneuver_start: Some((0., 0., 10.)),
                    ..LatticePlanner::default()
                };
                let ego = EgoState {
                    pose: rustdriving_core::Pose {
                        position: Vec2::new(s, direction * 2.5),
                        yaw: 0., // Measured course, as supplied by chassis mode.
                    },
                    speed: 2.,
                };
                let object = Prediction {
                    positions: vec![Vec2::new(25., 0.); 41],
                    radius: 0.5,
                    ..obstacle()
                };
                let path = planner.plan(ego, &road(4.5), &[object]);
                assert_ne!(path.mode, DrivingMode::Emergency);
                assert_eq!(path.lateral_target, direction * 2.9);
                assert_eq!(path.points[0].position, ego.pose.position);
                // The original shift finishes at x=10. Its 0.4 m residual at
                // 2 m/s admits recovery within four meters at the declared limit.
                // A fresh ten-meter join at each tick hides that error again.
                let recovered = path.points.iter().find(|p| p.position.x >= s + 4.).unwrap();
                assert!((recovered.position.y - direction * 2.9).abs() < 1e-12);
                assert!(path.points.iter().all(|p| p.position.y.abs() + 1.25 <= 4.5));
            }
        }
    }
    #[test]
    fn a_temporary_center_brake_preserves_the_narrow_chassis_passing_anchor() {
        for direction in [-1., 1.] {
            let anchor = Some((0., 0., 10.));
            let mut planner = LatticePlanner {
                cruise_speed: 4.,
                local_route_geometry: true,
                lateral_candidate_margin_m: Some(0.35),
                max_lateral_acceleration_m_s2: Some(2.5),
                previous_lateral: direction * 2.9,
                maneuver_start: anchor,
                ..LatticePlanner::default()
            };
            let objects = [0., -direction * 2.9].map(|y| Prediction {
                positions: vec![Vec2::new(25., y); 41],
                radius: 0.5,
                ..obstacle()
            });
            let mut ego = EgoState {
                pose: rustdriving_core::Pose {
                    position: Vec2::new(6., -direction * 1.1),
                    yaw: 0.,
                },
                speed: 2.,
            };
            let braking = planner.plan(ego, &road(4.5), &objects);
            assert_eq!(braking.mode, DrivingMode::Yield);
            assert_eq!(braking.lateral_target, 0.);
            assert_eq!(braking.points.last().unwrap().speed, 0.);
            assert_eq!(planner.previous_lateral, direction * 2.9);
            assert_eq!(planner.maneuver_start, anchor);
            ego.pose.position = Vec2::new(8., -direction * 0.5);
            let resumed = planner.plan(ego, &road(4.5), &objects);
            assert_ne!(resumed.mode, DrivingMode::Emergency);
            assert_eq!(resumed.lateral_target, direction * 2.9);
            assert_eq!(planner.maneuver_start, anchor);
            // A genuinely clear return withdraws the passing intent normally.
            let returned = planner.plan(ego, &road(4.5), &[]);
            assert_eq!(returned.lateral_target, 0.);
            assert_eq!(planner.previous_lateral, 0.);
            assert_ne!(planner.maneuver_start, anchor);
        }
    }
    #[test]
    fn goal_stop_uses_candidate_arc_length_after_a_lateral_return() {
        let ego = EgoState {
            pose: rustdriving_core::Pose {
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
            additional_radius_m: vec![],
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
                additional_radius_m: vec![],
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
    fn a_completed_lateral_shift_keeps_its_side_until_the_slower_object_is_passed() {
        let route = road(5.5);
        let mut planner = LatticePlanner {
            cruise_speed: 4.,
            ..LatticePlanner::default()
        };
        let mut ego = EgoState {
            speed: 4.,
            ..EgoState::default()
        };
        let object = obstacle();
        let first = planner.plan(ego, &route, std::slice::from_ref(&object));
        assert_ne!(first.lateral_target, 0.);
        // The original ten-meter lateral transition has finished. A return is
        // collision-free here, yet would cut back before the object is passed.
        ego.pose.position = Vec2::new(18., first.lateral_target);
        let fresh = LatticePlanner {
            cruise_speed: 4.,
            ..LatticePlanner::default()
        }
        .plan(ego, &route, std::slice::from_ref(&object));
        assert_eq!(fresh.lateral_target, 0.);
        let continued = planner.plan(ego, &route, std::slice::from_ref(&object));
        assert_eq!(continued.lateral_target, first.lateral_target);
        ego.pose.position.x = 25.;
        assert_eq!(planner.plan(ego, &route, &[object]).lateral_target, 0.);
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
            pose: rustdriving_core::Pose {
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
        object
            .positions
            .fill(rustdriving_core::Vec2::new(24.0, 0.0));
        let approaching = EgoState {
            speed: 8.0,
            ..EgoState::default()
        };
        let stop = planner.plan(approaching, &road(2.1), &[object.clone()]);
        assert_eq!(stop.mode, DrivingMode::Yield);
        assert_eq!(stop.points[0].speed, approaching.speed);
        assert_eq!(stop.points.last().unwrap().speed, 0.0);
        let waiting_ego = EgoState {
            pose: rustdriving_core::Pose {
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
