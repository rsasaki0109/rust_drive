//! Simulation scoring only; measured driver commands never read these truth states.
use rustdriving_core::{EgoState, Route, RouteSpeedZone};
pub const SPEED_TOLERANCE_M_S: f64 = 0.15;

#[derive(Default)]
pub(crate) struct SpeedZoneEvaluator {
    previous: Option<(f64, f64)>,
    pub violations: usize,
    pub maximum_excess_m_s: f64,
}
impl SpeedZoneEvaluator {
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
