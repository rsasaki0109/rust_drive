//! Simulation scoring only; measured driver commands never read these truth states.
use rustdriving_core::{EgoState, Route, RouteSpeedZone};
use rustdriving_pipeline::lane_change::{LaneChangePhase, LaneChangeStatus};
pub const SPEED_TOLERANCE_M_S: f64 = 0.15;

#[derive(Default)]
pub(crate) struct SpeedZoneEvaluator {
    previous: Option<(f64, f64)>,
    pub violations: usize,
    pub maximum_excess_m_s: f64,
}
impl SpeedZoneEvaluator {
    pub fn switch_route(
        &mut self,
        route: &Route,
        ego: EgoState,
        radius: f64,
        zones: &[RouteSpeedZone],
    ) {
        self.previous = None;
        self.observe(route, ego, radius, zones);
    }
    pub fn observe(&mut self, route: &Route, ego: EgoState, radius: f64, zones: &[RouteSpeedZone]) {
        let end = (route.project(ego.pose.position).0, ego.speed);
        let start = self.previous.replace(end).unwrap_or(end);
        let mut violated = false;
        for zone in zones {
            let lower = zone.start_s_m - radius;
            let upper = zone.end_s_m + radius;
            let delta = end.0 - start.0;
            let interval = if delta.abs() <= 1e-12 {
                (lower..=upper).contains(&start.0).then_some((0., 1.))
            } else {
                let a = (lower - start.0) / delta;
                let b = (upper - start.0) / delta;
                let low = a.min(b).max(0.);
                let high = a.max(b).min(1.);
                (low <= high).then_some((low, high))
            };
            if let Some((low, high)) = interval {
                let speed = |f| start.1 + (end.1 - start.1) * f;
                let excess = speed(low).max(speed(high)) - zone.speed_m_s;
                self.maximum_excess_m_s = self.maximum_excess_m_s.max(excess);
                violated |= excess > SPEED_TOLERANCE_M_S + 1e-9;
            }
        }
        self.violations += usize::from(violated);
    }
}

/// Evaluation-only compilation from the original map and selected edge IDs.
/// Per-edge distances are summed independently of the planner's knot indexing.
pub(crate) fn road_zones(
    network: &rustdriving_routing::RoadNetworkSpec,
    plan: &rustdriving_routing::RoutePlan,
) -> Vec<RouteSpeedZone> {
    let mut zones = vec![];
    let mut distance = 0.;
    for (i, id) in plan.edge_ids.iter().enumerate() {
        let edge = network.edges.iter().find(|edge| &edge.id == id).unwrap();
        let mut points = edge.points.clone();
        points[0] = network
            .nodes
            .iter()
            .find(|node| node.id == edge.from)
            .unwrap()
            .position;
        *points.last_mut().unwrap() = network
            .nodes
            .iter()
            .find(|node| node.id == edge.to)
            .unwrap()
            .position;
        let start = distance;
        distance += plan.traversals.get(i).map_or_else(
            || {
                points
                    .windows(2)
                    .map(|pair| pair[0].distance(pair[1]))
                    .sum::<f64>()
            },
            |span| span.end_s_m - span.start_s_m,
        );
        if let Some(speed) = edge.speed_limit_m_s {
            zones.push(RouteSpeedZone {
                start_s_m: start,
                end_s_m: distance,
                speed_m_s: speed.min(8.),
            });
        }
    }
    zones
}

/// Evaluation-only policy selection. No physical state or score enters driving.
pub(crate) struct AlternateSpeedPolicy {
    cap: Option<f64>,
    resumed: bool,
    current: Option<(Route, Vec<RouteSpeedZone>)>,
}
impl AlternateSpeedPolicy {
    pub fn new(cap: Option<f64>) -> Self {
        Self {
            cap,
            resumed: false,
            current: None,
        }
    }
    pub fn observe(
        &self,
        score: &mut SpeedZoneEvaluator,
        original: &Route,
        ego: EgoState,
        radius: f64,
        zones: &[RouteSpeedZone],
    ) {
        if let Some((route, zones)) = &self.current {
            score.observe(route, ego, radius, zones);
        } else if !zones.is_empty() {
            score.observe(original, ego, radius, zones);
        }
    }
    pub fn update(
        &mut self,
        score: &mut SpeedZoneEvaluator,
        original: &Route,
        ego: EgoState,
        vehicle: rustdriving_core::VehicleConfig,
        zones: &[RouteSpeedZone],
        status: Option<&LaneChangeStatus>,
    ) -> Result<(), String> {
        let (Some(cap), Some(status)) = (self.cap, status) else {
            return Ok(());
        };
        let request = status
            .return_state
            .as_ref()
            .and_then(|r| r.resume_state.as_ref())
            .and_then(|r| r.request);
        if let Some(request) = request.filter(|_| !self.resumed) {
            let plan = rustdriving_routing::lane_resume::plan(request, original, vehicle)?;
            let zones = plan.speed_zones(original, zones, cap)?;
            self.current = Some((plan.route, zones));
            self.resumed = true;
        } else if self.current.is_none()
            && matches!(
                status.phase,
                LaneChangePhase::Aborting | LaneChangePhase::Aborted
            )
        {
            self.current = Some((
                original.clone(),
                vec![RouteSpeedZone {
                    start_s_m: 0.,
                    end_s_m: original.length(),
                    speed_m_s: cap,
                }],
            ));
        } else {
            return Ok(());
        }
        // Score the switching instant under the new policy. Its preceding
        // interval was already scored above; different route origins must not
        // be interpolated together.
        score.previous = None;
        self.observe(score, original, ego, vehicle.radius, zones);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::{Pose, Vec2};
    fn ego(s: f64, speed: f64) -> EgoState {
        EgoState {
            pose: Pose {
                position: Vec2::new(s, 0.),
                yaw: 0.,
            },
            speed,
        }
    }
    #[test]
    fn resume_switch_does_not_interpolate_progress_from_different_route_origins() {
        use rustdriving_core::VehicleConfig;
        use rustdriving_routing::lane_resume::{self, LaneResumeRequest};
        let original = Route::new(
            vec![
                Vec2::default(),
                Vec2::new(25., 0.),
                Vec2::new(55., 3.5),
                Vec2::new(80., 3.5),
                Vec2::new(160., 3.5),
            ],
            1.75,
        )
        .unwrap();
        let request = LaneResumeRequest {
            start: ego(42., 0.).pose,
            source_start: Vec2::default(),
            source_end: Vec2::new(80., 0.),
            source_width_m: 3.5,
            target_start: Vec2::new(0., 3.5),
            target_end: Vec2::new(80., 3.5),
            target_width_m: 3.5,
            original_exit_s_m: original.project(Vec2::new(55., 3.5)).0,
            distance_m: 30.,
        };
        let zone_start = original.project(Vec2::new(80., 3.5)).0;
        let zones = [RouteSpeedZone {
            start_s_m: zone_start,
            end_s_m: zone_start + 0.01,
            speed_m_s: 1.,
        }];
        let status: LaneChangeStatus = serde_json::from_value(serde_json::json!({"phase":"Resuming","blocking_tracks":[],
            "return_state":{"decision":"Stopped","source_blocking_tracks":[],"resume_state":{
                "decision":"Resuming","source_blocking_tracks":[],"target_blocking_tracks":[],"request":request}}})).unwrap();
        let mut score = SpeedZoneEvaluator::default();
        let mut policy = AlternateSpeedPolicy::new(Some(4.));
        policy.observe(&mut score, &original, ego(42., 3.), 1.25, &zones);
        policy
            .update(
                &mut score,
                &original,
                ego(42., 3.),
                VehicleConfig::default(),
                &zones,
                Some(&status),
            )
            .unwrap();
        assert_eq!(score.violations, 0);
        let plan = lane_resume::plan(request, &original, VehicleConfig::default()).unwrap();
        let mut speeding = ego(80., 3.);
        speeding.pose.position.y = 3.5;
        policy.observe(&mut score, &original, speeding, 1.25, &zones);
        assert_eq!(score.violations, 1);
        assert!(plan.connector_length_m > 30.);
    }
    #[test]
    fn a_zone_crossed_completely_between_frames_is_still_scored() {
        let route = Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 2.1).unwrap();
        let zones = [RouteSpeedZone {
            start_s_m: 10.4,
            end_s_m: 10.41,
            speed_m_s: 2.,
        }];
        let mut score = SpeedZoneEvaluator::default();
        score.observe(&route, ego(10., 6.), 0., &zones);
        score.observe(&route, ego(11., 6.), 0., &zones);
        assert_eq!(score.violations, 1);
        assert!((score.maximum_excess_m_s - 4.).abs() < 1e-12);
    }
    #[test]
    fn physical_front_entry_and_rear_release_use_the_full_radius() {
        let route = Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 2.1).unwrap();
        let zones = [RouteSpeedZone {
            start_s_m: 20.,
            end_s_m: 30.,
            speed_m_s: 2.,
        }];
        let mut score = SpeedZoneEvaluator::default();
        score.observe(&route, ego(18.8, 3.), 1.25, &zones);
        score.observe(&route, ego(31.2, 3.), 1.25, &zones);
        assert_eq!(score.violations, 2);
        let mut clear = SpeedZoneEvaluator::default();
        clear.observe(&route, ego(31.3, 5.), 1.25, &zones);
        assert_eq!(clear.violations, 0);
    }
    #[test]
    fn ignored_braking_commands_fail_physical_speed_acceptance() {
        use crate::{
            ReferenceBackend, SimulationBackend, WorldObject, pipeline_config,
            simulate_with_backend,
        };
        use rustdriving_core::ControlCommand;
        use rustdriving_pipeline::SensorFrame;
        struct IgnoreControl(ReferenceBackend);
        impl SimulationBackend for IgnoreControl {
            fn state(&self) -> EgoState {
                self.0.state()
            }
            fn objects(&self, time: f64) -> Vec<WorldObject> {
                self.0.objects(time)
            }
            fn observe(&mut self, time: f64, tick: usize) -> Result<SensorFrame, String> {
                self.0.observe(time, tick)
            }
            fn advance(&mut self, _command: ControlCommand, dt: f64) -> Result<(), String> {
                self.0.advance(
                    ControlCommand {
                        acceleration: 2.,
                        steering: 0.,
                    },
                    dt,
                )
            }
        }
        let scenario: crate::Scenario =
            serde_json::from_str(include_str!("../../../scenarios/speed-zone-single.json"))
                .unwrap();
        let config = pipeline_config(&scenario);
        let backend = IgnoreControl(ReferenceBackend {
            scenario: scenario.clone(),
            traffic: crate::traffic::TrafficWorld::new(scenario.clone(), config.route.clone()),
            truth: EgoState {
                pose: config.initial_pose,
                speed: 0.,
            },
            vehicle: config.vehicle,
            rng: crate::Rng::new(7),
            seed: 7,
            command: ControlCommand::default(),
        });
        let run =
            simulate_with_backend(scenario, 7, backend, config, "negative-speed-backend").unwrap();
        assert!(!run.summary.passed);
        assert!(
            run.summary
                .failures
                .iter()
                .any(|failure| failure.contains("physical speed-zone violation intervals"))
        );
        assert_eq!(run.frames.len(), run.summary.steps);
    }
}
