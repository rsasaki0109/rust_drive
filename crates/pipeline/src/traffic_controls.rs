//! Mapped stop lines and timestamped infrastructure signal observations.
//! Phase schedules and simulator truth never enter this module.
use rustdriving_core::Route;
use serde::{Deserialize, Serialize};

pub const MAX_SIGNAL_AGE_S: f64 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignalColor {
    Red,
    Yellow,
    Green,
    Unknown,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StopLine {
    pub id: String,
    /// Arc length in meters on the fixed configured route.
    pub route_s_m: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalState {
    pub id: String,
    pub color: SignalColor,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalObservation {
    pub stamp: f64,
    /// Complete snapshot of the configured infrastructure signals.
    pub states: Vec<SignalState>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlledSignal {
    pub id: String,
    pub color: SignalColor,
    pub passed: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrafficControlStatus {
    pub last_accepted_stamp: Option<f64>,
    pub fault: bool,
    pub signals: Vec<ControlledSignal>,
    /// Temporary planning endpoint, before the line and ego's circular front.
    pub stop_s_m: Option<f64>,
}

pub fn validate_stop_lines(lines: &[StopLine], route: &Route, radius: f64) -> Result<(), String> {
    if lines.len() > 64 {
        return Err("at most 64 mapped signals are supported".into());
    }
    for (i, line) in lines.iter().enumerate() {
        if line.id.is_empty()
            || line.id.len() > 128
            || !line.route_s_m.is_finite()
            || line.route_s_m < radius + 5.0
            || line.route_s_m > route.length() - radius - 3.0
            || lines[..i]
                .iter()
                .any(|old| old.id == line.id || (old.route_s_m - line.route_s_m).abs() < 1e-6)
        {
            return Err("invalid, duplicate or out-of-route mapped stop line".into());
        }
    }
    Ok(())
}

pub(crate) struct TrafficControls {
    lines: Vec<StopLine>,
    status: TrafficControlStatus,
    accepted: Vec<SignalState>,
}
impl TrafficControls {
    pub fn new(lines: Vec<StopLine>) -> Self {
        let signals = lines
            .iter()
            .map(|line| ControlledSignal {
                id: line.id.clone(),
                color: SignalColor::Unknown,
                passed: false,
            })
            .collect();
        Self {
            lines,
            accepted: vec![],
            status: TrafficControlStatus {
                last_accepted_stamp: None,
                fault: false,
                signals,
                stop_s_m: None,
            },
        }
    }
    pub fn step(
        &mut self,
        observation: Option<&SignalObservation>,
        now: f64,
        progress: f64,
        radius: f64,
        healthy: bool,
    ) {
        if let Some(snapshot) = observation {
            if !snapshot.stamp.is_finite() || snapshot.stamp < 0.0 || snapshot.stamp > now + 1e-9 {
                self.status.fault = true;
            } else if self
                .status
                .last_accepted_stamp
                .is_none_or(|last| snapshot.stamp > last)
            {
                if snapshot.states.len() != self.lines.len()
                    || self
                        .lines
                        .iter()
                        .any(|line| snapshot.states.iter().filter(|s| s.id == line.id).count() != 1)
                {
                    self.status.fault = true;
                } else {
                    self.accepted = snapshot.states.clone();
                    self.status.last_accepted_stamp = Some(snapshot.stamp);
                    self.status.fault = false;
                }
            }
        }
        let fresh = !self.status.fault
            && self
                .status
                .last_accepted_stamp
                .is_some_and(|stamp| now - stamp <= MAX_SIGNAL_AGE_S + 1e-9);
        self.status.stop_s_m = None;
        for (line, signal) in self.lines.iter().zip(&mut self.status.signals) {
            signal.color = if fresh {
                self.accepted
                    .iter()
                    .find(|s| s.id == line.id)
                    .unwrap()
                    .color
            } else {
                SignalColor::Unknown
            };
            // Commit only on a fresh permissive observation; red/unknown cannot
            // release a line merely because localization moved past it.
            if healthy && signal.color == SignalColor::Green && progress + radius >= line.route_s_m
            {
                signal.passed = true;
            }
            if !signal.passed && signal.color != SignalColor::Green {
                // Planner additionally aims one meter before this endpoint.
                let end = line.route_s_m - radius - 1.0;
                self.status.stop_s_m = Some(self.status.stop_s_m.map_or(end, |s| s.min(end)));
            }
        }
    }
    pub fn status(&self) -> TrafficControlStatus {
        self.status.clone()
    }
    pub fn planning_route(&self, route: &Route) -> Option<Route> {
        planning_prefix(route, self.status.stop_s_m?)
    }
}

/// Reuse the existing planner on a temporary route prefix without replacing the map.
pub(crate) fn planning_prefix(route: &Route, endpoint: f64) -> Option<Route> {
    let end = endpoint.min(route.length());
    if end >= route.length() {
        return None;
    }
    let mut points: Vec<_> = route
        .points
        .iter()
        .zip(&route.lengths)
        .filter(|(_, s)| **s < end - 1e-6)
        .map(|(p, _)| *p)
        .collect();
    points.push(route.sample(end, 0.0).0);
    Some(Route::new(points, route.half_width).expect("validated stop-line prefix"))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn line(id: &str, s: f64) -> StopLine {
        StopLine {
            id: id.into(),
            route_s_m: s,
        }
    }
    fn sample(stamp: f64, states: &[(&str, SignalColor)]) -> SignalObservation {
        SignalObservation {
            stamp,
            states: states
                .iter()
                .map(|(id, color)| SignalState {
                    id: (*id).into(),
                    color: *color,
                })
                .collect(),
        }
    }
    #[test]
    fn duplicates_reordered_samples_and_missing_updates_cannot_refresh_green() {
        let mut c = TrafficControls::new(vec![line("main", 35.0)]);
        let green = sample(0.0, &[("main", SignalColor::Green)]);
        c.step(Some(&green), 0.0, 0.0, 1.25, true);
        assert!(c.status().stop_s_m.is_none());
        c.step(Some(&green), 0.55, 10.0, 1.25, true);
        assert_eq!(c.status().last_accepted_stamp, Some(0.0));
        assert_eq!(c.status().signals[0].color, SignalColor::Unknown);
        assert!(c.status().stop_s_m.is_some());
        c.step(
            Some(&sample(1.0, &[("main", SignalColor::Red)])),
            1.0,
            10.0,
            1.25,
            true,
        );
        c.step(
            Some(&sample(0.8, &[("main", SignalColor::Green)])),
            1.05,
            10.0,
            1.25,
            true,
        );
        assert_eq!(c.status().signals[0].color, SignalColor::Red);
        assert_eq!(c.status().last_accepted_stamp, Some(1.0));
    }
    #[test]
    fn malformed_and_future_snapshots_latch_until_a_new_complete_snapshot() {
        let mut c = TrafficControls::new(vec![line("a", 35.0), line("b", 70.0)]);
        let green = sample(0.0, &[("a", SignalColor::Green), ("b", SignalColor::Green)]);
        c.step(Some(&green), 0.0, 0.0, 1.25, true);
        for bad in [
            sample(0.05, &[("a", SignalColor::Green)]),
            sample(
                0.05,
                &[("a", SignalColor::Green), ("a", SignalColor::Green)],
            ),
            sample(
                0.05,
                &[("a", SignalColor::Green), ("typo", SignalColor::Green)],
            ),
            sample(5.0, &[("a", SignalColor::Green), ("b", SignalColor::Green)]),
        ] {
            c.step(Some(&bad), 0.1, 10.0, 1.25, true);
            c.step(None, 0.15, 10.0, 1.25, true);
            assert!(c.status().fault);
            assert_eq!(c.status().signals[0].color, SignalColor::Unknown);
            c.step(Some(&green), 0.2, 10.0, 1.25, true);
            assert!(c.status().fault);
            let valid = sample(
                0.25,
                &[("a", SignalColor::Green), ("b", SignalColor::Green)],
            );
            c.step(Some(&valid), 0.25, 10.0, 1.25, true);
            assert!(!c.status().fault);
            // Restart each mutation at the same accepted acquisition stamp.
            c = TrafficControls::new(vec![line("a", 35.0), line("b", 70.0)]);
            c.step(Some(&green), 0.0, 0.0, 1.25, true);
        }
    }
    #[test]
    fn only_fresh_green_commits_a_crossing_and_nearest_blocked_line_wins() {
        let mut c = TrafficControls::new(vec![line("far", 70.0), line("near", 35.0)]);
        c.step(None, 0.0, 0.0, 1.25, true);
        assert_eq!(c.status().stop_s_m, Some(32.75));
        c.step(None, 0.05, 36.0, 1.25, true);
        assert!(!c.status().signals[1].passed);
        c.step(
            Some(&sample(
                0.1,
                &[("far", SignalColor::Yellow), ("near", SignalColor::Green)],
            )),
            0.1,
            36.0,
            1.25,
            true,
        );
        assert!(c.status().signals[1].passed);
        assert_eq!(c.status().stop_s_m, Some(67.75));
        c.step(
            Some(&sample(
                0.2,
                &[("far", SignalColor::Red), ("near", SignalColor::Red)],
            )),
            0.2,
            40.0,
            1.25,
            true,
        );
        assert!(c.status().signals[1].passed);
        assert_eq!(c.status().stop_s_m, Some(67.75));
    }
    #[test]
    fn invalid_maps_are_rejected_and_stop_route_preserves_original_polyline() {
        let route = Route::new(
            vec![
                rustdriving_core::Vec2::default(),
                rustdriving_core::Vec2::new(100.0, 0.0),
            ],
            2.1,
        )
        .unwrap();
        for lines in [
            vec![line("", 35.0)],
            vec![line("a", 3.0)],
            vec![line("a", 99.0)],
            vec![line("a", 35.0), line("a", 70.0)],
            vec![line("a", 35.0), line("b", 35.0)],
        ] {
            assert!(validate_stop_lines(&lines, &route, 1.25).is_err());
        }
        let mut c = TrafficControls::new(vec![line("a", 35.0)]);
        c.step(None, 0.0, 0.0, 1.25, true);
        let prefix = c.planning_route(&route).unwrap();
        assert_eq!(prefix.length(), 32.75);
        assert_eq!(prefix.half_width, route.half_width);
        assert_eq!(route.length(), 100.0);
    }
    #[test]
    fn unhealthy_localization_cannot_commit_an_estimated_crossing() {
        let mut c = TrafficControls::new(vec![line("main", 35.0)]);
        c.step(
            Some(&sample(0.0, &[("main", SignalColor::Green)])),
            0.0,
            34.0,
            1.25,
            false,
        );
        assert!(!c.status().signals[0].passed);
        c.step(None, 0.55, 34.0, 1.25, true);
        assert_eq!(c.status().signals[0].color, SignalColor::Unknown);
        assert!(c.status().stop_s_m.is_some());
    }
}
