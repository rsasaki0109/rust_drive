//! Simulator-only infrastructure feed and independent stop-line rule evaluation.
use crate::Scenario;
use rustdriving_pipeline::traffic_controls::{
    SignalColor, SignalObservation, SignalState, StopLine, validate_stop_lines,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalPhase {
    pub from: f64,
    pub color: SignalColor,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalSpec {
    pub stop_line: StopLine,
    pub phases: Vec<SignalPhase>,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalDropout {
    pub from: f64,
    pub until: f64,
}
impl SignalSpec {
    pub fn color_at(&self, time: f64) -> SignalColor {
        self.phases
            .iter()
            .rev()
            .find(|p| p.from <= time + 1e-9)
            .unwrap()
            .color
    }
}
pub fn validate(scenario: &Scenario) -> Result<(), String> {
    let lines: Vec<_> = scenario
        .traffic_signals
        .iter()
        .map(|s| s.stop_line.clone())
        .collect();
    validate_stop_lines(
        &lines,
        &scenario.route(),
        rustdriving_core::VehicleConfig::default().radius,
    )?;
    if !lines.is_empty() && !scenario.navigation_updates.is_empty() {
        return Err("signal fixtures currently require a fixed route".into());
    }
    for signal in &scenario.traffic_signals {
        if signal.phases.is_empty() || signal.phases.len() > 128 || signal.phases[0].from != 0.0 {
            return Err("signal phases must start at zero and contain 1..=128 entries".into());
        }
        let mut last = -1.0;
        for phase in &signal.phases {
            if !phase.from.is_finite() || phase.from <= last || phase.from >= scenario.duration {
                return Err("signal phases must be finite, ordered and within duration".into());
            }
            last = phase.from;
        }
    }
    let mut end = 0.0;
    for window in &scenario.signal_dropout_windows {
        if lines.is_empty()
            || !window.from.is_finite()
            || !window.until.is_finite()
            || window.from < end
            || window.until <= window.from
            || window.until > scenario.duration
        {
            return Err("invalid or overlapping signal observation dropout".into());
        }
        end = window.until;
    }
    Ok(())
}
pub fn observe(scenario: &Scenario, time: f64, tick: usize) -> Option<SignalObservation> {
    if scenario.traffic_signals.is_empty()
        || !tick.is_multiple_of(4)
        || scenario
            .signal_dropout_windows
            .iter()
            .any(|w| time >= w.from - 1e-9 && time < w.until - 1e-9)
    {
        return None;
    }
    Some(SignalObservation {
        stamp: time,
        states: scenario
            .traffic_signals
            .iter()
            .map(|s| SignalState {
                id: s.stop_line.id.clone(),
                color: s.color_at(time),
            })
            .collect(),
    })
}

/// Scores actual physical front crossings, independent of planner mode or diagnostics.
#[derive(Default)]
pub struct RuleEvaluator {
    previous_front: Option<f64>,
    pub violations: usize,
    pub minimum_nonpermissive_margin_m: Option<f64>,
}
impl RuleEvaluator {
    pub fn observe(&mut self, specs: &[SignalSpec], time: f64, physical_front_s: f64) {
        for spec in specs {
            let line = spec.stop_line.route_s_m;
            if spec.color_at(time) != SignalColor::Green {
                if self.previous_front.is_none_or(|s| s < line) {
                    let margin = line - physical_front_s;
                    self.minimum_nonpermissive_margin_m = Some(
                        self.minimum_nonpermissive_margin_m
                            .map_or(margin, |m| m.min(margin)),
                    );
                }
                if self.previous_front.is_none_or(|s| s < line) && physical_front_s >= line {
                    self.violations += 1;
                }
            }
        }
        self.previous_front = Some(physical_front_s);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn signal(color: SignalColor) -> SignalSpec {
        SignalSpec {
            stop_line: StopLine {
                id: "main".into(),
                route_s_m: 35.0,
            },
            phases: vec![SignalPhase { from: 0.0, color }],
        }
    }
    #[test]
    fn physical_red_yellow_unknown_crossings_fail_even_without_driver_diagnostics() {
        for color in [SignalColor::Red, SignalColor::Yellow, SignalColor::Unknown] {
            let mut e = RuleEvaluator::default();
            let s = vec![signal(color)];
            e.observe(&s, 0.0, 34.0);
            e.observe(&s, 0.05, 35.25);
            e.observe(&s, 0.1, 36.0);
            assert_eq!(e.violations, 1);
            assert!(e.minimum_nonpermissive_margin_m.unwrap() < 0.0);
        }
    }
    #[test]
    fn green_crossings_and_later_red_behind_the_vehicle_are_permitted() {
        let mut s = signal(SignalColor::Green);
        s.phases.push(SignalPhase {
            from: 1.0,
            color: SignalColor::Red,
        });
        let mut e = RuleEvaluator::default();
        e.observe(&[s.clone()], 0.0, 34.0);
        e.observe(&[s.clone()], 0.05, 36.0);
        e.observe(&[s], 1.0, 40.0);
        assert_eq!(e.violations, 0);
    }
    #[test]
    fn controller_ignoring_backend_cannot_hide_a_physical_red_crossing() {
        use crate::{
            ReferenceBackend, SimulationBackend, WorldObject, pipeline_config,
            simulate_with_backend,
        };
        use rustdriving_core::{ControlCommand, EgoState};
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
                        acceleration: 2.0,
                        steering: 0.0,
                    },
                    dt,
                )
            }
        }
        let scenario: Scenario =
            serde_json::from_str(include_str!("../../../scenarios/signal-red-stop.json")).unwrap();
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
        assert_eq!(run.summary.signal_violations, 1);
        assert!(
            run.summary
                .failures
                .iter()
                .any(|s| s.contains("nonpermissive physical stop-line"))
        );
    }
}
