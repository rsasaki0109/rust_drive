//! Simulator-side route-following traffic. Actor policy sees only scalar proximity observations.
use crate::{ObjectSpec, Scenario, WorldObject};
use rustdrive_core::{EgoState, Route, Vec2, VehicleConfig};
use serde::{Deserialize, Serialize};

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
}
struct Actor {
    s: f64,
    speed: f64,
    acceleration: f64,
    observation: ProximityObservation,
}
/// Stateful world shared by reference and RNE. Scheduled actors retain their analytic motion.
pub struct TrafficWorld {
    scenario: Scenario,
    route: Route,
    actors: Vec<Option<Actor>>,
    time: f64,
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
                })
            })
            .collect();
        Self {
            scenario,
            route,
            actors,
            time: 0.0,
        }
    }
    pub fn objects(&self, time: f64) -> Vec<WorldObject> {
        self.scenario
            .objects
            .iter()
            .enumerate()
            .filter(|(_, o)| time >= o.active_from)
            .map(|(id, o)| {
                let elapsed = (time - o.active_from.max(o.moving_from)).max(0.0);
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
            let acceleration =
                calibration.acceleration(actor.speed, spec.speed, stopping, observation);
            let next_speed = (actor.speed + acceleration * dt).max(0.0);
            actor.s += (actor.speed + next_speed) * 0.5 * dt;
            actor.speed = next_speed;
            actor.acceleration = acceleration;
            actor.observation = observation;
        }
        self.time += dt;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdrive_core::{Pose, Vec2};
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
}
