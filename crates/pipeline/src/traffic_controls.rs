//! Mapped stop lines and timestamped infrastructure signal observations.
//! Phase schedules and simulator truth never enter this module.
use rustdriving_core::{EgoState, Route};
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization_reset: Option<SignalResetDiagnostics>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignalResetDiagnostics {
    pub stamp: f64,
    pub progress_s_m: f64,
    pub distance_to_route_m: f64,
    pub yaw_error_rad: f64,
    pub admitted: bool,
    pub retained_ids: Vec<String>,
    pub rearmed_ids: Vec<String>,
    pub unserved_too_close_ids: Vec<String>,
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
    /// Latest invalid snapshot delivery time; recovery needs a fresh later acquisition.
    fault_epoch: Option<f64>,
    /// Pre-reset permissions cannot establish passage in a new localization epoch.
    reset_cutoff: Option<f64>,
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
            fault_epoch: None,
            reset_cutoff: None,
            status: TrafficControlStatus {
                last_accepted_stamp: None,
                fault: false,
                signals,
                stop_s_m: None,
                localization_reset: None,
            },
        }
    }
    pub fn relocalize(&mut self, route: &Route, now: f64, ego: EgoState, radius: f64) {
        let (progress, _) = route.project(ego.pose.position);
        let (center, heading) = route.sample(progress, 0.);
        let distance = center.distance(ego.pose.position);
        let yaw_error = rustdriving_core::wrap_angle(ego.pose.yaw - heading).abs();
        let mut reset = SignalResetDiagnostics {
            stamp: now,
            progress_s_m: progress,
            distance_to_route_m: distance,
            yaw_error_rad: yaw_error,
            admitted: ego.speed.abs() <= 0.05
                && distance + radius + 0.2 < route.half_width
                && yaw_error < 0.2,
            retained_ids: vec![],
            rearmed_ids: vec![],
            unserved_too_close_ids: vec![],
        };
        self.reset_cutoff = Some(self.reset_cutoff.map_or(now, |s| s.max(now)));
        self.accepted.clear();
        self.status.last_accepted_stamp = None;
        self.fault_epoch = None;
        self.status.fault = false;
        self.status.stop_s_m = None;
        for (line, signal) in self.lines.iter().zip(&mut self.status.signals) {
            signal.color = SignalColor::Unknown;
            let margin = line.route_s_m - progress - radius;
            if signal.passed {
                if margin <= 0. {
                    reset.retained_ids.push(line.id.clone());
                    continue;
                }
                reset.rearmed_ids.push(line.id.clone());
            }
            signal.passed = false;
            if margin < 0.5 {
                reset.admitted = false;
                reset.unserved_too_close_ids.push(line.id.clone());
            }
            let end = line.route_s_m - radius - 1.;
            self.status.stop_s_m = Some(self.status.stop_s_m.map_or(end, |s| s.min(end)));
        }
        self.status.localization_reset = Some(reset);
    }
    pub fn recovery_fault(&self) -> bool {
        self.status
            .localization_reset
            .as_ref()
            .is_some_and(|r| !r.admitted)
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
            if !snapshot.stamp.is_finite()
                || snapshot.stamp < 0.0
                || snapshot.stamp > now + 1e-9
                || snapshot.states.len() != self.lines.len()
                || self
                    .lines
                    .iter()
                    .any(|line| snapshot.states.iter().filter(|s| s.id == line.id).count() != 1)
            {
                self.fault_epoch = Some(now);
            } else if self
                .status
                .last_accepted_stamp
                .is_none_or(|last| snapshot.stamp > last)
                && self.reset_cutoff.is_none_or(|reset| snapshot.stamp > reset)
                && self.fault_epoch.is_none_or(|fault| {
                    snapshot.stamp > fault && now - snapshot.stamp <= MAX_SIGNAL_AGE_S + 1e-9
                })
            {
                self.accepted = snapshot.states.clone();
                self.status.last_accepted_stamp = Some(snapshot.stamp);
                self.fault_epoch = None;
            }
        }
        self.status.fault = self.fault_epoch.is_some();
        let fresh = !self.status.fault
            && self
                .status
                .last_accepted_stamp
                .is_some_and(|stamp| now - stamp <= MAX_SIGNAL_AGE_S + 1e-9);
        self.status.stop_s_m = None;
        let recovery_fault = self.recovery_fault();
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
            if healthy
                && !recovery_fault
                && signal.color == SignalColor::Green
                && progress + radius >= line.route_s_m
            {
                signal.passed = true;
            }
            if !signal.passed && (signal.color != SignalColor::Green || recovery_fault) {
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
    fn route() -> Route {
        Route::new(
            vec![
                rustdriving_core::Vec2::default(),
                rustdriving_core::Vec2::new(100., 0.),
            ],
            3.5,
        )
        .unwrap()
    }
    fn ego(x: f64) -> EgoState {
        EgoState {
            pose: rustdriving_core::Pose {
                position: rustdriving_core::Vec2::new(x, 0.),
                yaw: 0.,
            },
            speed: 0.,
        }
    }
    #[test]
    fn reset_discards_green_and_requires_a_strictly_newer_acquisition() {
        let mut c = TrafficControls::new(vec![line("main", 35.)]);
        c.step(
            Some(&sample(4.9, &[("main", SignalColor::Green)])),
            4.9,
            30.,
            1.25,
            true,
        );
        c.relocalize(&route(), 5., ego(30.), 1.25);
        assert!(!c.recovery_fault());
        for stamp in [4.9, 5.] {
            c.step(
                Some(&sample(stamp, &[("main", SignalColor::Green)])),
                5.1,
                30.,
                1.25,
                true,
            );
            assert_eq!(c.status().last_accepted_stamp, None);
            assert_eq!(c.status().signals[0].color, SignalColor::Unknown);
            assert_eq!(c.status().stop_s_m, Some(32.75));
        }
        c.step(
            Some(&sample(5.1, &[("main", SignalColor::Red)])),
            5.1,
            30.,
            1.25,
            true,
        );
        c.step(
            Some(&sample(5.05, &[("main", SignalColor::Green)])),
            5.15,
            30.,
            1.25,
            true,
        );
        assert_eq!(c.status().signals[0].color, SignalColor::Red);
        c.step(
            Some(&sample(5.2, &[("main", SignalColor::Green)])),
            5.2,
            30.,
            1.25,
            true,
        );
        assert!(c.status().stop_s_m.is_none());
        assert!(!c.status().signals[0].passed);
        c.step(None, 5.75, 30., 1.25, true);
        assert_eq!(c.status().signals[0].color, SignalColor::Unknown);
        assert!(c.status().stop_s_m.is_some());
    }
    #[test]
    fn reset_preserves_passage_behind_and_rearms_permission_ahead() {
        let mut c = TrafficControls::new(vec![line("near", 35.), line("far", 70.)]);
        let greens = |stamp| {
            sample(
                stamp,
                &[("near", SignalColor::Green), ("far", SignalColor::Green)],
            )
        };
        c.step(Some(&greens(1.)), 1., 40., 1.25, true);
        c.relocalize(&route(), 5., ego(40.), 1.25);
        assert_eq!(
            c.status().localization_reset.as_ref().unwrap().retained_ids,
            ["near"]
        );
        c.step(None, 5., 40., 1.25, true);
        assert!(c.status().signals[0].passed);
        assert_eq!(c.status().stop_s_m, Some(67.75));
        c.relocalize(&route(), 6., ego(30.), 1.25);
        assert_eq!(
            c.status().localization_reset.as_ref().unwrap().rearmed_ids,
            ["near"]
        );
        assert!(!c.status().signals[0].passed);
        c.step(Some(&greens(6.)), 6., 30., 1.25, true);
        assert_eq!(c.status().stop_s_m, Some(32.75));
        c.step(Some(&greens(6.1)), 6.1, 30., 1.25, true);
        assert!(c.status().stop_s_m.is_none());
        c.step(None, 6.15, 34., 1.25, true);
        assert!(c.status().signals[0].passed);
    }
    #[test]
    fn invalid_reset_cannot_be_pardoned_by_fresh_green() {
        let mut poses = vec![ego(34.), ego(33.5)];
        let mut outside = ego(30.);
        outside.pose.position.y = 2.1;
        poses.push(outside);
        let mut yaw = ego(30.);
        yaw.pose.yaw = 0.21;
        poses.push(yaw);
        let mut moving = ego(30.);
        moving.speed = 0.06;
        poses.push(moving);
        for pose in poses {
            let mut c = TrafficControls::new(vec![line("main", 35.)]);
            c.relocalize(&route(), 5., pose, 1.25);
            for now in [5.1, 6., 10.] {
                c.step(
                    Some(&sample(now, &[("main", SignalColor::Green)])),
                    now,
                    40.,
                    1.25,
                    true,
                );
                assert!(c.recovery_fault());
                assert!(!c.status().signals[0].passed);
                assert_eq!(c.status().stop_s_m, Some(32.75));
            }
        }
    }
    #[test]
    fn malformed_post_reset_snapshot_still_requires_later_complete_evidence() {
        let mut c = TrafficControls::new(vec![line("main", 35.)]);
        c.relocalize(&route(), 5., ego(30.), 1.25);
        c.step(Some(&sample(5.1, &[])), 5.1, 30., 1.25, true);
        c.step(
            Some(&sample(5.1, &[("main", SignalColor::Green)])),
            5.15,
            30.,
            1.25,
            true,
        );
        assert!(c.status().fault);
        assert!(c.status().stop_s_m.is_some());
        c.step(
            Some(&sample(5.2, &[("main", SignalColor::Green)])),
            5.2,
            30.,
            1.25,
            true,
        );
        assert!(!c.status().fault);
        assert!(!c.recovery_fault());
        assert!(c.status().stop_s_m.is_none());
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
