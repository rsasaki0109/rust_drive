//! Independent physical stop-sign rule scoring; driver diagnostics are never read.
use rustdrive_pipeline::traffic_controls::StopLine;

#[derive(Default)]
struct PhysicalStop {
    since: Option<f64>,
    satisfied: bool,
    crossed: bool,
}
pub struct StopRuleEvaluator {
    states: Vec<PhysicalStop>,
    pub violations: usize,
    pub minimum_unreleased_margin_m: Option<f64>,
}
impl StopRuleEvaluator {
    pub fn new(count: usize) -> Self {
        Self {
            states: (0..count).map(|_| PhysicalStop::default()).collect(),
            violations: 0,
            minimum_unreleased_margin_m: None,
        }
    }
    pub fn observe(&mut self, lines: &[StopLine], now: f64, front_s: f64, speed: f64) {
        for (line, state) in lines.iter().zip(&mut self.states) {
            if state.crossed {
                continue;
            }
            let margin = line.route_s_m - front_s;
            if !state.satisfied {
                self.minimum_unreleased_margin_m = Some(
                    self.minimum_unreleased_margin_m
                        .map_or(margin, |old| old.min(margin)),
                );
                if (0.0..=3.5).contains(&margin) && speed.abs() <= 0.1 {
                    let start = *state.since.get_or_insert(now);
                    state.satisfied = now - start + 1e-9 >= 2.0;
                } else {
                    state.since = None;
                }
            }
            if margin <= 0.0 {
                state.crossed = true;
                if !state.satisfied {
                    self.violations += 1;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Scenario;
    fn line() -> Vec<StopLine> {
        vec![StopLine {
            id: "stop".into(),
            route_s_m: 35.0,
        }]
    }
    #[test]
    fn rolling_far_or_interrupted_stops_cannot_satisfy_a_physical_crossing() {
        for mode in 0..3 {
            let mut e = StopRuleEvaluator::new(1);
            let lines = line();
            for i in 0..60 {
                let front = if mode == 1 { 10.0 } else { 33.0 };
                let speed = if mode == 0 || (mode == 2 && i == 30) {
                    0.2
                } else {
                    0.0
                };
                e.observe(&lines, i as f64 * 0.05, front, speed);
            }
            e.observe(&lines, 3.0, 36.0, 2.0);
            assert_eq!(e.violations, 1);
        }
    }
    #[test]
    fn complete_physical_hold_allows_crossing_without_any_planner_status() {
        let mut e = StopRuleEvaluator::new(1);
        let lines = line();
        for i in 0..41 {
            e.observe(&lines, i as f64 * 0.05, 33.0, 0.0);
        }
        e.observe(&lines, 2.05, 36.0, 1.0);
        assert_eq!(e.violations, 0);
        assert_eq!(e.minimum_unreleased_margin_m, Some(2.0));
    }
    #[test]
    fn controller_ignoring_backend_cannot_hide_a_missed_physical_stop() {
        use crate::{
            ReferenceBackend, SimulationBackend, WorldObject, pipeline_config,
            simulate_with_backend,
        };
        use rustdrive_core::{ControlCommand, EgoState};
        use rustdrive_pipeline::SensorFrame;
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
                        acceleration: 2.0,
                        steering: 0.0,
                    },
                    dt,
                )
            }
        }
        let scenario: Scenario =
            serde_json::from_str(include_str!("../../../scenarios/stop-sign-single.json")).unwrap();
        let config = pipeline_config(&scenario);
        let backend = IgnoreControl(ReferenceBackend {
            scenario: scenario.clone(),
            traffic: crate::traffic::TrafficWorld::new(scenario.clone(), config.route.clone()),
            truth: EgoState {
                pose: config.initial_pose,
                speed: 0.0,
            },
            vehicle: config.vehicle,
            rng: crate::Rng::new(7),
            command: ControlCommand::default(),
        });
        let run =
            simulate_with_backend(scenario, 7, backend, config, "negative-rule-backend").unwrap();
        assert!(!run.summary.passed);
        assert_eq!(run.summary.stop_sign_violations, 1);
        assert!(
            run.summary
                .failures
                .iter()
                .any(|s| s.contains("physical stop-sign crossings"))
        );
    }
}
