//! Simulator-side route-following traffic. Actor policy sees only scalar proximity observations.
use crate::{ObjectSpec, Scenario, WorldObject};
use rustdriving_core::{EgoState, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::traffic_controls::{SignalColor, SignalObservation};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Opt-in forward-route infrastructure signal controller for simulated actors.
/// Front extent is a declared physical/display envelope, independent of the
/// circular proximity collider. This policy does not enter the ego pipeline.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrafficSignalPolicy {
    pub front_extent_m: f64,
    pub stop_margin_m: f64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StopWindow {
    pub from: f64,
    pub until: f64,
}
/// IDM-style longitudinal actor calibration, not an ego planner configuration.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FollowingSpec {
    pub initial_speed_m_s: f64,
    pub minimum_gap_m: f64,
    pub time_headway_s: f64,
    pub max_acceleration_m_s2: f64,
    pub comfortable_deceleration_m_s2: f64,
    pub max_deceleration_m_s2: f64,
    pub sensor_range_m: f64,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub stop_windows: Vec<StopWindow>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signal_control: Option<TrafficSignalPolicy>,
}
impl Default for FollowingSpec {
    fn default() -> Self {
        Self {
            initial_speed_m_s: 0.0,
            minimum_gap_m: 3.0,
            time_headway_s: 1.5,
            max_acceleration_m_s2: 2.0,
            comfortable_deceleration_m_s2: 2.0,
            max_deceleration_m_s2: 4.0,
            sensor_range_m: 45.0,
            stop_windows: vec![],
            signal_control: None,
        }
    }
}
impl FollowingSpec {
    pub(crate) fn validate(&self, object: &ObjectSpec) -> Result<(), String> {
        for (value, low, high) in [
            (self.initial_speed_m_s, 0.0, 12.0),
            (self.minimum_gap_m, 0.5, 10.0),
            (self.time_headway_s, 0.5, 4.0),
            (self.max_acceleration_m_s2, 0.1, 4.0),
            (self.comfortable_deceleration_m_s2, 0.1, 6.0),
            (self.max_deceleration_m_s2, 0.1, 6.0),
            (self.sensor_range_m, 5.0, 100.0),
        ] {
            if !value.is_finite() || !(low..=high).contains(&value) {
                return Err("invalid following calibration".into());
            }
        }
        if !(0.1..=12.0).contains(&object.speed)
            || object.lateral_speed != 0.0
            || object.moving_from > object.active_from
            || self.comfortable_deceleration_m_s2 > self.max_deceleration_m_s2
        {
            return Err("following requires bounded positive desired speed, fixed lateral offset, no delayed motion and valid braking bounds".into());
        }
        if let Some(policy) = &self.signal_control
            && (!policy.front_extent_m.is_finite()
                || !(object.radius..=12.0).contains(&policy.front_extent_m)
                || !policy.stop_margin_m.is_finite()
                || !(0.5..=5.0).contains(&policy.stop_margin_m)
                || object.lateral != 0.0)
        {
            return Err("signal-controlled actors require a finite front extent, 0.5..=5 m stop margin and the forward route lane".into());
        }
        let mut end = 0.0;
        for window in &self.stop_windows {
            if !window.from.is_finite()
                || !window.until.is_finite()
                || window.from < end
                || window.until <= window.from
                || window.until > 300.0
            {
                return Err("invalid or overlapping traffic stop window".into());
            }
            end = window.until;
        }
        Ok(())
    }
    fn acceleration(
        &self,
        speed: f64,
        desired: f64,
        stopping: bool,
        observation: ProximityObservation,
    ) -> f64 {
        let free = if stopping {
            if speed > 0.0 {
                -self.comfortable_deceleration_m_s2
            } else {
                0.0
            }
        } else {
            self.max_acceleration_m_s2 * (1.0 - (speed / desired).powi(4))
        };
        let interaction = observation.gap_m.map_or(0.0, |gap| {
            let closing = observation.closing_speed_m_s.unwrap_or(speed);
            let desired_gap = self.minimum_gap_m
                + (speed * self.time_headway_s
                    + speed * closing
                        / (2.0
                            * (self.max_acceleration_m_s2 * self.comfortable_deceleration_m_s2)
                                .sqrt()))
                .max(0.0);
            self.max_acceleration_m_s2 * (desired_gap / gap.max(0.01)).powi(2)
        });
        (free - interaction).clamp(-self.max_deceleration_m_s2, self.max_acceleration_m_s2)
    }
}
/// Ideal finite-range route-aligned proximity measurement; no object identities or commands.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct ProximityObservation {
    pub stamp: f64,
    pub gap_m: Option<f64>,
    pub closing_speed_m_s: Option<f64>,
}
/// Evaluator truth and last applied actor command; never part of the ego sensor log.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrafficTelemetry {
    pub id: u64,
    pub route_s_m: f64,
    pub speed_m_s: f64,
    pub acceleration_m_s2: f64,
    pub observation: ProximityObservation,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub signal_control: Option<TrafficSignalTelemetry>,
}
/// Last actually applied actor signal decision, simulator-side evidence only.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrafficSignalTelemetry {
    pub id: String,
    /// Simulator time at which the bounded command was chosen, before motion.
    pub decision_stamp: f64,
    /// Time of the integrated physical front position in this evidence record.
    pub physical_front_stamp: f64,
    pub observation_stamp: Option<f64>,
    pub color: SignalColor,
    pub fresh: bool,
    pub permissive: bool,
    pub stop_line_s_m: f64,
    pub front_extent_m: f64,
    pub stop_margin_m: f64,
    pub physical_front_s_m: f64,
    pub remaining_stop_distance_m: f64,
    pub crossed_nonpermissive: bool,
}
struct Actor {
    s: f64,
    speed: f64,
    acceleration: f64,
    observation: ProximityObservation,
    signal_control: Option<TrafficSignalTelemetry>,
    passed_signals: BTreeSet<String>,
}
/// Stateful world shared by reference and RNE. Scheduled actors retain their analytic motion.
pub struct TrafficWorld {
    scenario: Scenario,
    route: Route,
    actors: Vec<Option<Actor>>,
    time: f64,
    tick: usize,
    signal_observation: Option<SignalObservation>,
}
impl TrafficWorld {
    pub fn new(scenario: Scenario, route: Route) -> Self {
        let actors = scenario
            .objects
            .iter()
            .map(|o| {
                o.following.as_ref().map(|f| Actor {
                    s: o.s,
                    speed: f.initial_speed_m_s,
                    acceleration: 0.0,
                    observation: ProximityObservation::default(),
                    signal_control: None,
                    passed_signals: scenario
                        .traffic_signals
                        .iter()
                        .filter(|signal| {
                            f.signal_control.as_ref().is_some_and(|p| {
                                signal.stop_line.route_s_m <= o.s + p.front_extent_m
                            })
                        })
                        .map(|signal| signal.stop_line.id.clone())
                        .collect(),
                })
            })
            .collect();
        Self {
            scenario,
            route,
            actors,
            time: 0.0,
            tick: 0,
            signal_observation: None,
        }
    }
    pub fn objects(&self, time: f64) -> Vec<WorldObject> {
        self.scenario
            .objects
            .iter()
            .enumerate()
            .filter(|(_, o)| time >= o.active_from)
            .map(|(id, o)| {
                let elapsed = (o.moving_until.map_or(time, |until| time.min(until))
                    - o.active_from.max(o.moving_from))
                .max(0.0);
                let (s, lateral) = self.actors[id].as_ref().map_or(
                    (
                        o.s + elapsed * o.speed,
                        o.lateral + elapsed * o.lateral_speed,
                    ),
                    |actor| (actor.s, o.lateral),
                );
                let (position, yaw) = self.route.sample(s, lateral);
                let position = if self.actors[id].is_some() && s > self.route.length() {
                    position.plus(Vec2::new(yaw.cos(), yaw.sin()).scaled(s - self.route.length()))
                } else {
                    position
                };
                WorldObject {
                    id: id as u64,
                    position,
                    radius: o.radius,
                }
            })
            .collect()
    }
    pub fn telemetry(&self) -> Vec<TrafficTelemetry> {
        self.actors
            .iter()
            .enumerate()
            .filter_map(|(id, state)| {
                let actor = state.as_ref()?;
                (self.time + 1e-9 >= self.scenario.objects[id].active_from).then_some(
                    TrafficTelemetry {
                        id: id as u64,
                        route_s_m: actor.s,
                        speed_m_s: actor.speed,
                        acceleration_m_s2: actor.acceleration,
                        observation: actor.observation,
                        signal_control: actor.signal_control.clone(),
                    },
                )
            })
            .collect()
    }
    /// Read a single pre-step scene for every actor, then integrate without a gap/position clamp.
    pub fn advance(
        &mut self,
        ego: EgoState,
        vehicle: VehicleConfig,
        dt: f64,
    ) -> Result<(), String> {
        if !dt.is_finite() || dt <= 0.0 || !ego.speed.is_finite() || !ego.pose.position.finite() {
            return Err("invalid traffic integration input".into());
        }
        if self.actors.iter().all(Option::is_none) {
            self.time += dt;
            return Ok(());
        }
        if self.scenario.objects.iter().any(|object| {
            object
                .following
                .as_ref()
                .is_some_and(|f| f.signal_control.is_some())
        }) && let Some(observation) =
            crate::signals::observe(&self.scenario, self.time, self.tick)
        {
            self.signal_observation = Some(observation);
        }
        let mut scene = self.objects(self.time);
        scene.push(WorldObject {
            id: u64::MAX,
            position: ego.pose.position,
            radius: vehicle.radius,
        });
        let projections: Vec<_> = scene
            .iter()
            .map(|o| (o, self.route.project(o.position)))
            .collect();
        for (id, state) in self.actors.iter_mut().enumerate() {
            let Some(actor) = state else {
                continue;
            };
            let spec = &self.scenario.objects[id];
            let calibration = spec.following.as_ref().unwrap();
            if self.time + 1e-9 < spec.active_from.max(spec.moving_from) {
                continue;
            }
            // The route endpoint is a stationary boundary observed by the actor.
            let endpoint_gap = (self.route.length() - spec.radius - actor.s).max(0.0);
            let mut gap = (endpoint_gap <= calibration.sensor_range_m).then_some(endpoint_gap);
            for (object, (s, lateral)) in &projections {
                if object.id == id as u64 || *s < actor.s {
                    continue;
                }
                let radius = spec.radius + object.radius;
                let side = lateral - spec.lateral;
                if side.abs() >= radius {
                    continue;
                }
                let measured = *s - actor.s - (radius.powi(2) - side.powi(2)).sqrt();
                if measured <= calibration.sensor_range_m && gap.is_none_or(|g| measured < g) {
                    gap = Some(measured);
                }
            }
            let closing = gap.map(|g| {
                actor
                    .observation
                    .gap_m
                    .map_or(actor.speed, |old| ((old - g) / dt).clamp(-12.0, 12.0))
            });
            let observation = ProximityObservation {
                stamp: self.time,
                gap_m: gap,
                closing_speed_m_s: closing,
            };
            let stopping = calibration
                .stop_windows
                .iter()
                .any(|w| self.time + 1e-9 >= w.from && self.time + 1e-9 < w.until);
            let mut acceleration =
                calibration.acceleration(actor.speed, spec.speed, stopping, observation);
            let mut signal_decision = None;
            if let Some(policy) = &calibration.signal_control {
                let front = actor.s + policy.front_extent_m;
                let signal = self
                    .scenario
                    .traffic_signals
                    .iter()
                    .filter(|s| {
                        !actor.passed_signals.contains(&s.stop_line.id)
                            && s.stop_line.route_s_m - front <= calibration.sensor_range_m
                    })
                    .min_by(|a, b| a.stop_line.route_s_m.total_cmp(&b.stop_line.route_s_m));
                if let Some(signal) = signal {
                    let received = self.signal_observation.as_ref();
                    let fresh = received.is_some_and(|o| {
                        self.time - o.stamp >= -1e-9 && self.time - o.stamp <= 0.75
                    });
                    let color = received
                        .and_then(|o| o.states.iter().find(|s| s.id == signal.stop_line.id))
                        .map_or(SignalColor::Unknown, |s| s.color);
                    let permissive = fresh && color == SignalColor::Green;
                    let clearance = signal.stop_line.route_s_m - front - policy.stop_margin_m;
                    if !permissive {
                        // Trapezoidal travel plus a remaining discrete braking
                        // envelope: (old_v+next_v)*dt/2 + next_v²/(2*b)
                        // + next_v*dt/2 <= gap. A 1 µm controller reserve avoids
                        // crossing the declared margin through roundoff.
                        // The applied acceleration remains physically bounded;
                        // infeasible late acquisition cannot teleport the actor.
                        let b = calibration.comfortable_deceleration_m_s2;
                        let available = (clearance - 0.5 * actor.speed * dt - 1e-6).max(0.0);
                        let next_cap = ((b * dt).powi(2) + 2.0 * b * available).sqrt() - b * dt;
                        acceleration = acceleration
                            .min((next_cap.max(0.0) - actor.speed) / dt)
                            .clamp(
                                -calibration.max_deceleration_m_s2,
                                calibration.max_acceleration_m_s2,
                            );
                    }
                    signal_decision = Some(TrafficSignalTelemetry {
                        id: signal.stop_line.id.clone(),
                        decision_stamp: self.time,
                        physical_front_stamp: self.time + dt,
                        observation_stamp: received.map(|o| o.stamp),
                        color,
                        fresh,
                        permissive,
                        stop_line_s_m: signal.stop_line.route_s_m,
                        front_extent_m: policy.front_extent_m,
                        stop_margin_m: policy.stop_margin_m,
                        physical_front_s_m: front,
                        remaining_stop_distance_m: clearance,
                        crossed_nonpermissive: false,
                    });
                } else if self.scenario.traffic_signals.is_empty() {
                    // A configured policy without a usable map fails closed.
                    acceleration = acceleration
                        .min(-calibration.comfortable_deceleration_m_s2)
                        .clamp(
                            -calibration.max_deceleration_m_s2,
                            calibration.max_acceleration_m_s2,
                        );
                }
            }
            let next_speed = (actor.speed + acceleration * dt).max(0.0);
            actor.s += (actor.speed + next_speed) * 0.5 * dt;
            actor.speed = next_speed;
            actor.acceleration = acceleration;
            actor.observation = observation;
            if let Some(decision) = signal_decision.as_mut() {
                let next_front = actor.s + decision.front_extent_m;
                if next_front >= decision.stop_line_s_m {
                    decision.crossed_nonpermissive = !decision.permissive;
                    actor.passed_signals.insert(decision.id.clone());
                }
                decision.physical_front_s_m = next_front;
                decision.remaining_stop_distance_m =
                    decision.stop_line_s_m - next_front - decision.stop_margin_m;
            }
            actor.signal_control = signal_decision;
        }
        self.time += dt;
        self.tick += 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::{Pose, Vec2};
    fn scenario(objects: serde_json::Value) -> Scenario {
        serde_json::from_value(serde_json::json!({"name":"traffic test","duration":30,
            "road_length":200,"half_width":2,"expected":"stop","objects":objects}))
        .unwrap()
    }
    fn ego(x: f64) -> EgoState {
        EgoState {
            pose: Pose {
                position: Vec2::new(x, 0.0),
                yaw: 0.0,
            },
            speed: 0.0,
        }
    }
    #[test]
    fn actor_stops_for_a_sensed_vehicle_without_position_clamping() {
        let s = scenario(
            serde_json::json!([{"s":0,"lateral":0,"radius":1,"speed":6,"following":{"initial_speed_m_s":6}}]),
        );
        let mut world = TrafficWorld::new(s.clone(), s.route());
        for _ in 0..400 {
            world
                .advance(ego(35.0), VehicleConfig::default(), 0.05)
                .unwrap();
        }
        let actor = &world.telemetry()[0];
        assert!(actor.speed_m_s < 0.05);
        assert!(actor.route_s_m < 35.0 - 2.25 - 2.9);
        assert!(actor.route_s_m > 20.0);
    }
    #[test]
    fn finite_sensor_range_does_not_grant_collision_avoidance_oracle() {
        let s = scenario(
            serde_json::json!([{"s":0,"lateral":0,"radius":1,"speed":10,"following":{"initial_speed_m_s":10,"sensor_range_m":5}}]),
        );
        let mut world = TrafficWorld::new(s.clone(), s.route());
        let mut overlapped = false;
        for _ in 0..100 {
            world
                .advance(ego(20.0), VehicleConfig::default(), 0.05)
                .unwrap();
            overlapped |= world.objects(world.time)[0]
                .position
                .distance(ego(20.0).pose.position)
                < 2.25;
        }
        assert!(
            overlapped,
            "a late detection must not teleport/clamp an infeasible actor clear"
        );
    }
    #[test]
    fn inactive_actor_does_not_move_and_a_departing_lead_releases_the_follower() {
        let s = scenario(
            serde_json::json!([{"s":0,"lateral":0,"radius":1,"speed":4,"active_from":2,"following":{}}]),
        );
        let mut world = TrafficWorld::new(s.clone(), s.route());
        for _ in 0..40 {
            world
                .advance(ego(12.0), VehicleConfig::default(), 0.05)
                .unwrap();
        }
        assert!(world.actors[0].as_ref().unwrap().s == 0.0);
        for _ in 0..200 {
            world
                .advance(ego(12.0), VehicleConfig::default(), 0.05)
                .unwrap();
        }
        let stopped = world.telemetry()[0].route_s_m;
        assert!(world.telemetry()[0].speed_m_s < 0.1);
        for _ in 0..100 {
            world
                .advance(ego(150.0), VehicleConfig::default(), 0.05)
                .unwrap();
        }
        assert!(world.telemetry()[0].route_s_m > stopped + 10.0);
        assert!(world.telemetry()[0].speed_m_s > 3.5);
    }
    #[test]
    fn policy_has_analytic_free_acceleration_and_bounded_emergency_braking() {
        let spec = FollowingSpec::default();
        assert_eq!(
            spec.acceleration(0.0, 6.0, false, ProximityObservation::default()),
            2.0
        );
        assert_eq!(
            spec.acceleration(6.0, 6.0, false, ProximityObservation::default()),
            0.0
        );
        assert_eq!(
            spec.acceleration(
                6.0,
                6.0,
                false,
                ProximityObservation {
                    gap_m: Some(0.1),
                    closing_speed_m_s: Some(6.0),
                    ..ProximityObservation::default()
                }
            ),
            -4.0
        );
    }
    fn signal_scenario() -> Scenario {
        serde_json::from_value(serde_json::json!({"name":"NPC signal test","duration":30,"road_length":120,"half_width":3,"expected":"stop",
            "objects":[{"s":24,"lateral":0,"radius":1,"speed":2,"following":{"initial_speed_m_s":2,"signal_control":{"front_extent_m":2.35,"stop_margin_m":0.5}}}],
            "traffic_signals":[{"stop_line":{"id":"already-behind","route_s_m":12},"phases":[{"from":0,"color":"Red"}]},
                {"stop_line":{"id":"crossing","route_s_m":34},"phases":[{"from":0,"color":"Red"},{"from":24,"color":"Green"}]}]})).unwrap()
    }
    #[test]
    fn optional_actor_signal_control_stops_physical_front_on_red_and_resumes_observed_green() {
        let scenario = signal_scenario();
        scenario.validate().unwrap();
        let mut traffic = TrafficWorld::new(scenario.clone(), scenario.route());
        let mut stopped = false;
        let mut released = false;
        for i in 0..580 {
            let before = traffic.telemetry()[0].clone();
            traffic
                .advance(ego(-100.0), VehicleConfig::default(), 0.05)
                .unwrap();
            let after = traffic.telemetry()[0].clone();
            assert!(
                (after.route_s_m - before.route_s_m - (before.speed_m_s + after.speed_m_s) * 0.025)
                    .abs()
                    < 1e-12,
                "no position clamp or teleport"
            );
            assert!((-4.0..=2.0).contains(&after.acceleration_m_s2));
            if i < 479 {
                assert!(
                    after.route_s_m + 2.35 <= 33.5 + 1e-9,
                    "physical front respects mapped stop margin"
                );
                let observed = after.signal_control.as_ref().unwrap();
                assert_eq!(observed.id, "crossing");
                assert!(!observed.permissive && !observed.crossed_nonpermissive);
                stopped |= i > 200 && after.speed_m_s < 0.01;
            }
            if i > 530 {
                released |= after.route_s_m + 2.35 > 34.0 && after.speed_m_s > 1.0;
            }
        }
        assert!(stopped && released);
    }
    #[test]
    fn missing_signal_feed_stops_but_bounded_controller_cannot_repair_infeasible_initial_state() {
        let mut scenario = signal_scenario();
        scenario.signal_dropout_windows = vec![crate::signals::SignalDropout {
            from: 0.0,
            until: 25.0,
        }];
        let mut traffic = TrafficWorld::new(scenario.clone(), scenario.route());
        for _ in 0..500 {
            traffic
                .advance(ego(-100.0), VehicleConfig::default(), 0.05)
                .unwrap();
        }
        let row = &traffic.telemetry()[0];
        assert!(row.speed_m_s < 0.01 && row.route_s_m + 2.35 <= 33.5);
        let state = row.signal_control.as_ref().unwrap();
        assert!(
            !state.fresh
                && !state.permissive
                && state.color == SignalColor::Unknown
                && state.observation_stamp.is_none()
        );
        scenario.objects[0].s = 31.64;
        scenario.objects[0]
            .following
            .as_mut()
            .unwrap()
            .initial_speed_m_s = 6.0;
        let mut infeasible = TrafficWorld::new(scenario.clone(), scenario.route());
        infeasible
            .advance(ego(-100.0), VehicleConfig::default(), 0.05)
            .unwrap();
        let row = &infeasible.telemetry()[0];
        assert!(row.route_s_m + 2.35 > 34.0 && row.speed_m_s >= 5.8);
        assert!(row.signal_control.as_ref().unwrap().crossed_nonpermissive);
    }
    #[test]
    fn prescribed_motion_until_stops_on_footpath_and_defaults_do_not_add_serialized_fields() {
        let bounded = scenario(
            serde_json::json!([{"s":18,"lateral":-5,"radius":0.4,"lateral_speed":1.2,"moving_from":2,"moving_until":10.666666666666666}]),
        );
        bounded.validate().unwrap();
        let traffic = TrafficWorld::new(bounded.clone(), bounded.route());
        let finish = traffic.objects(10.666666666666666)[0].position;
        assert!((finish.y - 5.4).abs() < 1e-12);
        assert_eq!(traffic.objects(30.0)[0].position, finish);
        assert_eq!(
            bounded.scheduled_objects(&bounded.route(), 30.0)[0].position,
            finish
        );
        let ordinary =
            scenario(serde_json::json!([{"s":0,"lateral":0,"radius":1,"speed":2,"following":{}}]));
        let encoded = serde_json::to_value(&ordinary).unwrap();
        assert!(encoded["objects"][0].get("moving_until").is_none());
        assert!(
            encoded["objects"][0]["following"]
                .get("signal_control")
                .is_none()
        );
        let traffic = TrafficWorld::new(ordinary.clone(), ordinary.route());
        assert!(
            serde_json::to_value(traffic.telemetry()).unwrap()[0]
                .get("signal_control")
                .is_none()
        );
    }
    #[test]
    fn signal_calibration_and_motion_stop_validation_reject_incompatible_actors() {
        let mut s = signal_scenario();
        s.objects[0]
            .following
            .as_mut()
            .unwrap()
            .signal_control
            .as_mut()
            .unwrap()
            .stop_margin_m = 0.49;
        assert!(s.validate().is_err());
        let mut s = signal_scenario();
        s.objects[0].lateral = -8.0;
        assert!(s.validate().is_err());
        let mut s = signal_scenario();
        s.objects[0].moving_until = Some(10.0);
        assert!(s.validate().is_err());
        let s = scenario(
            serde_json::json!([{"s":18,"lateral":-5,"radius":0.4,"lateral_speed":1.2,"moving_from":2,"moving_until":1.0}]),
        );
        assert!(s.validate().is_err());
    }
    #[test]
    fn stale_cached_green_yellow_and_unknown_cannot_authorize_forward_crossing() {
        for color in [
            SignalColor::Green,
            SignalColor::Yellow,
            SignalColor::Unknown,
        ] {
            let mut scenario = signal_scenario();
            scenario.traffic_signals[1].phases = vec![
                crate::signals::SignalPhase { from: 0.0, color },
                crate::signals::SignalPhase {
                    from: 0.5,
                    color: SignalColor::Red,
                },
            ];
            scenario.signal_dropout_windows = vec![crate::signals::SignalDropout {
                from: 0.2,
                until: 25.0,
            }];
            let mut traffic = TrafficWorld::new(scenario.clone(), scenario.route());
            let mut evaluator = crate::signals::RuleEvaluator::default();
            for _ in 0..450 {
                traffic
                    .advance(ego(-100.0), VehicleConfig::default(), 0.05)
                    .unwrap();
                let actor = &traffic.telemetry()[0];
                evaluator.observe(
                    &scenario.traffic_signals[1..],
                    traffic.time,
                    actor.route_s_m + 2.35,
                );
            }
            let actor = &traffic.telemetry()[0];
            let decision = actor.signal_control.as_ref().unwrap();
            assert!(!decision.fresh && !decision.permissive && decision.color == color);
            assert!(actor.speed_m_s < 0.01 && actor.route_s_m + 2.35 <= 33.5 + 1e-9);
            assert_eq!(evaluator.violations, 0);
        }
        let mut missing_map = signal_scenario();
        missing_map.traffic_signals.clear();
        let mut traffic = TrafficWorld::new(missing_map.clone(), missing_map.route());
        for _ in 0..60 {
            traffic
                .advance(ego(-100.0), VehicleConfig::default(), 0.05)
                .unwrap();
        }
        assert_eq!(traffic.telemetry()[0].speed_m_s, 0.0);
        assert!((-4.0..=2.0).contains(&traffic.telemetry()[0].acceleration_m_s2));
    }
}
