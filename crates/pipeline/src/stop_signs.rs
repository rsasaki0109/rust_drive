//! Fixed-route stop signs. Only map geometry and healthy measured motion enter here.
use crate::traffic_controls::{StopLine, planning_prefix};
use rustdriving_core::{EgoState, Odometry, Route};
use serde::{Deserialize, Serialize};

pub const STOP_HOLD_S: f64 = 2.0;
pub const STOP_SPEED_M_S: f64 = 0.05;
pub(crate) const MAX_STOP_MOTION_AGE_S: f64 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum StopPhase {
    Approaching,
    Holding,
    Released,
    Passed,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlledStop {
    pub id: String,
    pub phase: StopPhase,
    pub held_s: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StopSignStatus {
    pub stops: Vec<ControlledStop>,
    pub stop_s_m: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization_reset: Option<StopSignResetDiagnostics>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StopSignResetDiagnostics {
    pub stamp: f64,
    pub progress_s_m: f64,
    pub distance_to_route_m: f64,
    pub yaw_error_rad: f64,
    pub admitted: bool,
    pub retained_ids: Vec<String>,
    pub rearmed_ids: Vec<String>,
    pub unserved_too_close_ids: Vec<String>,
}
#[derive(Clone, Copy)]
struct StopHold {
    /// First qualifying delivery; earlier acquisition time is never credited.
    started_at: f64,
    last_stamp: f64,
}
pub(crate) struct StopSigns {
    lines: Vec<StopLine>,
    holds: Vec<Option<StopHold>>,
    motion_watermark: Option<f64>,
    status: StopSignStatus,
}
impl StopSigns {
    pub fn new(lines: Vec<StopLine>) -> Self {
        Self {
            holds: vec![None; lines.len()],
            motion_watermark: None,
            status: StopSignStatus {
                stops: lines
                    .iter()
                    .map(|line| ControlledStop {
                        id: line.id.clone(),
                        phase: StopPhase::Approaching,
                        held_s: 0.0,
                    })
                    .collect(),
                stop_s_m: None,
                localization_reset: None,
            },
            lines,
        }
    }
    /// Retain semantic completion behind the measured front, while restarting
    /// all duties ahead. No observation from the old epoch can extend a dwell.
    pub fn relocalize(&mut self, route: &Route, now: f64, ego: EgoState, radius: f64) {
        let (progress, _) = route.project(ego.pose.position);
        let (center, heading) = route.sample(progress, 0.);
        let distance = center.distance(ego.pose.position);
        let yaw_error = rustdriving_core::wrap_angle(ego.pose.yaw - heading).abs();
        let mut reset = StopSignResetDiagnostics {
            stamp: now,
            progress_s_m: progress,
            distance_to_route_m: distance,
            yaw_error_rad: yaw_error,
            admitted: ego.speed.abs() <= STOP_SPEED_M_S
                && distance + radius + 0.2 < route.half_width
                && yaw_error < 0.2,
            retained_ids: vec![],
            rearmed_ids: vec![],
            unserved_too_close_ids: vec![],
        };
        self.motion_watermark = Some(self.motion_watermark.map_or(now, |s| s.max(now)));
        self.status.stop_s_m = None;
        for ((line, state), hold) in self
            .lines
            .iter()
            .zip(&mut self.status.stops)
            .zip(&mut self.holds)
        {
            *hold = None;
            let margin = line.route_s_m - progress - radius;
            if matches!(state.phase, StopPhase::Released | StopPhase::Passed) {
                if margin <= 0. {
                    reset.retained_ids.push(line.id.clone());
                    continue;
                }
                reset.rearmed_ids.push(line.id.clone());
            }
            state.phase = StopPhase::Approaching;
            state.held_s = 0.;
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
        route: &Route,
        now: f64,
        ego: EgoState,
        radius: f64,
        measured_motion: Option<Odometry>,
        healthy: bool,
    ) {
        let new_motion = measured_motion.filter(|motion| {
            healthy
                && self
                    .motion_watermark
                    .is_none_or(|stamp| motion.stamp > stamp)
        });
        let observed_stamp = if healthy {
            measured_motion.map_or(now, |motion| motion.stamp)
        } else {
            now
        };
        self.motion_watermark = Some(
            self.motion_watermark
                .map_or(observed_stamp, |stamp| stamp.max(observed_stamp)),
        );
        if self.recovery_fault() {
            return;
        }
        let (progress, lateral) = route.project(ego.pose.position);
        let heading = route.sample(progress, 0.0).1;
        let aligned = rustdriving_core::wrap_angle(ego.pose.yaw - heading).abs() < 0.2
            && lateral.abs() + radius <= route.half_width;
        self.status.stop_s_m = None;
        for ((line, state), hold) in self
            .lines
            .iter()
            .zip(&mut self.status.stops)
            .zip(&mut self.holds)
        {
            let margin = line.route_s_m - progress - radius;
            if state.phase == StopPhase::Released && healthy && aligned && margin <= 0.0 {
                state.phase = StopPhase::Passed;
            }
            if matches!(state.phase, StopPhase::Released | StopPhase::Passed) {
                continue;
            }
            let stopped = healthy
                && aligned
                && (0.5..=3.5).contains(&margin)
                && ego.speed.abs() <= STOP_SPEED_M_S
                && measured_motion.is_some_and(|motion| motion.speed.abs() <= STOP_SPEED_M_S);
            if stopped {
                if let Some(motion) = new_motion {
                    if hold.is_some_and(|held| {
                        motion.stamp - held.last_stamp > MAX_STOP_MOTION_AGE_S + 1e-9
                    }) {
                        *hold = None;
                    }
                    let held = hold.get_or_insert(StopHold {
                        started_at: now,
                        last_stamp: motion.stamp,
                    });
                    held.last_stamp = motion.stamp;
                    state.held_s = (held.last_stamp - held.started_at).max(0.0);
                    state.phase = if state.held_s + 1e-9 >= STOP_HOLD_S {
                        StopPhase::Released
                    } else {
                        StopPhase::Holding
                    };
                }
            } else {
                *hold = None;
                state.held_s = 0.0;
                state.phase = StopPhase::Approaching;
            }
            if state.phase != StopPhase::Released {
                let end = line.route_s_m - radius - 1.0;
                self.status.stop_s_m = Some(self.status.stop_s_m.map_or(end, |s| s.min(end)));
            }
        }
    }
    pub fn status(&self) -> StopSignStatus {
        self.status.clone()
    }
    pub fn endpoint(&self) -> Option<f64> {
        self.status.stop_s_m
    }
    pub fn holding_brake(&self, route: &Route, ego: EgoState) -> bool {
        self.status.stop_s_m.is_some_and(|end| {
            end + 1.0 - route.project(ego.pose.position).0 <= 3.5 && ego.speed.abs() <= 0.2
        })
    }
    pub fn planning_route(&self, route: &Route) -> Option<Route> {
        planning_prefix(route, self.status.stop_s_m?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::{Pose, Vec2};
    fn route() -> Route {
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 2.1).unwrap()
    }
    fn stops() -> StopSigns {
        StopSigns::new(vec![StopLine {
            id: "main".into(),
            route_s_m: 35.0,
        }])
    }
    fn ego(front: f64, speed: f64) -> EgoState {
        EgoState {
            pose: Pose {
                position: Vec2::new(front - 1.25, 0.0),
                yaw: 0.0,
            },
            speed,
        }
    }
    fn step(
        stops: &mut StopSigns,
        route: &Route,
        now: f64,
        ego: EgoState,
        measured_speed: Option<f64>,
        healthy: bool,
    ) {
        stops.step(
            route,
            now,
            ego,
            1.25,
            measured_speed.map(|speed| Odometry {
                stamp: now,
                speed,
                yaw_rate: 0.0,
            }),
            healthy,
        );
    }
    #[test]
    fn reset_discards_old_dwell_and_requires_two_seconds_of_new_acquisitions() {
        let mut s = stops();
        let r = route();
        for i in 0..30 {
            step(&mut s, &r, i as f64 * 0.05, ego(33., 0.), Some(0.), true);
        }
        assert_eq!(s.status.stops[0].phase, StopPhase::Holding);
        s.relocalize(&r, 5., ego(33., 0.), 1.25);
        assert_eq!(s.status.stops[0].held_s, 0.);
        assert!(!s.recovery_fault());
        step(&mut s, &r, 5., ego(33., 0.), Some(0.), true);
        assert_eq!(s.status.stops[0].phase, StopPhase::Approaching);
        for i in 1..=40 {
            step(
                &mut s,
                &r,
                5. + i as f64 * 0.05,
                ego(33., 0.),
                Some(0.),
                true,
            );
            assert_eq!(s.status.stops[0].phase, StopPhase::Holding);
        }
        step(&mut s, &r, 7.05, ego(33., 0.), Some(0.), true);
        assert_eq!(s.status.stops[0].phase, StopPhase::Released);
    }
    #[test]
    fn a_completed_stop_behind_is_retained_and_a_rewound_stop_is_rearmed() {
        for passed in [false, true] {
            let mut s = stops();
            let r = route();
            for i in 0..=40 {
                step(&mut s, &r, i as f64 * 0.05, ego(33., 0.), Some(0.), true);
            }
            if passed {
                step(&mut s, &r, 2.05, ego(36., 0.), Some(0.), true);
            }
            let original = s.status.stops[0].phase;
            s.relocalize(&r, 5., ego(40., 0.), 1.25);
            assert_eq!(s.status.stops[0].phase, original);
            assert_eq!(
                s.status.localization_reset.as_ref().unwrap().retained_ids,
                ["main"]
            );
            assert!(s.endpoint().is_none());
            s.relocalize(&r, 6., ego(33., 0.), 1.25);
            assert_eq!(s.status.stops[0].phase, StopPhase::Approaching);
            assert_eq!(s.status.stops[0].held_s, 0.);
            assert_eq!(
                s.status.localization_reset.as_ref().unwrap().rearmed_ids,
                ["main"]
            );
            assert!(s.endpoint().is_some());
        }
    }
    #[test]
    fn an_uncrossed_release_must_confirm_again_after_reset() {
        let mut s = stops();
        let r = route();
        for i in 0..=40 {
            step(&mut s, &r, i as f64 * 0.05, ego(33., 0.), Some(0.), true);
        }
        assert_eq!(s.status.stops[0].phase, StopPhase::Released);
        s.relocalize(&r, 3., ego(33., 0.), 1.25);
        assert_eq!(s.status.stops[0].phase, StopPhase::Approaching);
        assert_eq!(
            s.status.localization_reset.as_ref().unwrap().rearmed_ids,
            ["main"]
        );
        assert!(s.holding_brake(&r, ego(33., 0.)));
    }
    #[test]
    fn invalid_route_admission_or_unserved_crossing_latches_until_another_reset() {
        let r = route();
        let mut sideways = ego(33., 0.);
        sideways.pose.position.y = 3.;
        let mut wrong_heading = ego(33., 0.);
        wrong_heading.pose.yaw = 0.3;
        for bad in [
            ego(35.1, 0.),
            ego(34.75, 0.),
            sideways,
            wrong_heading,
            ego(33., 0.1),
        ] {
            let mut s = stops();
            s.relocalize(&r, 1., bad, 1.25);
            assert!(s.recovery_fault());
            for i in 21..=80 {
                step(&mut s, &r, i as f64 * 0.05, ego(33., 0.), Some(0.), true);
            }
            assert!(s.recovery_fault());
            assert_eq!(s.status.stops[0].phase, StopPhase::Approaching);
            assert_eq!(s.status.stops[0].held_s, 0.);
            s.relocalize(&r, 5., ego(33., 0.), 1.25);
            assert!(!s.recovery_fault());
        }
    }
    #[test]
    fn only_a_continuous_healthy_near_line_stop_releases() {
        let mut s = stops();
        let r = route();
        for i in 0..60 {
            step(&mut s, &r, i as f64 * 0.05, ego(10.0, 0.0), Some(0.0), true);
        }
        assert_eq!(s.status().stops[0].phase, StopPhase::Approaching);
        for i in 60..100 {
            step(&mut s, &r, i as f64 * 0.05, ego(33.0, 0.0), Some(0.0), true);
        }
        assert_eq!(s.status().stops[0].phase, StopPhase::Holding);
        step(&mut s, &r, 5.0, ego(33.0, 0.0), None, false);
        assert_eq!(s.status().stops[0].held_s, 0.0);
        for i in 101..142 {
            step(&mut s, &r, i as f64 * 0.05, ego(33.0, 0.0), Some(0.0), true);
        }
        assert_eq!(s.status().stops[0].phase, StopPhase::Released);
        assert!(s.endpoint().is_none());
        step(&mut s, &r, 7.15, ego(36.0, 1.0), Some(1.0), false);
        assert_eq!(s.status().stops[0].phase, StopPhase::Released);
        step(&mut s, &r, 7.2, ego(36.0, 1.0), Some(1.0), true);
        assert_eq!(s.status().stops[0].phase, StopPhase::Passed);
    }
    #[test]
    fn rolling_missing_motion_wrong_heading_and_early_crossing_never_release() {
        let r = route();
        for mode in 0..4 {
            let mut s = stops();
            for i in 0..80 {
                let mut e = ego(
                    if mode == 3 { 36.0 } else { 33.0 },
                    if mode == 0 { 0.2 } else { 0.0 },
                );
                if mode == 2 {
                    e.pose.yaw = 1.0;
                }
                step(
                    &mut s,
                    &r,
                    i as f64 * 0.05,
                    e,
                    if mode == 1 { None } else { Some(e.speed) },
                    true,
                );
            }
            assert_eq!(s.status().stops[0].phase, StopPhase::Approaching);
            assert!(s.endpoint().is_some());
        }
    }
}
