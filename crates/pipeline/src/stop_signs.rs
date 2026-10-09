//! Fixed-route stop signs. Only map geometry and healthy measured motion enter here.
use crate::traffic_controls::{StopLine, planning_prefix};
use rustdrive_core::{EgoState, Route};
use serde::{Deserialize, Serialize};

pub const STOP_HOLD_S: f64 = 2.0;
pub const STOP_SPEED_M_S: f64 = 0.05;

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
}
pub(crate) struct StopSigns {
    lines: Vec<StopLine>,
    since: Vec<Option<f64>>,
    status: StopSignStatus,
}
impl StopSigns {
    pub fn new(lines: Vec<StopLine>) -> Self {
        Self {
            since: vec![None; lines.len()],
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
            },
            lines,
        }
    }
    pub fn step(
        &mut self,
        route: &Route,
        now: f64,
        ego: EgoState,
        radius: f64,
        measured_speed: Option<f64>,
        healthy: bool,
    ) {
        let (progress, lateral) = route.project(ego.pose.position);
        let heading = route.sample(progress, 0.0).1;
        let aligned = rustdrive_core::wrap_angle(ego.pose.yaw - heading).abs() < 0.2
            && lateral.abs() + radius <= route.half_width;
        self.status.stop_s_m = None;
        for ((line, state), since) in self
            .lines
            .iter()
            .zip(&mut self.status.stops)
            .zip(&mut self.since)
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
                && measured_speed.is_some_and(|speed| speed.abs() <= STOP_SPEED_M_S);
            if stopped {
                let start = *since.get_or_insert(now);
                state.held_s = now - start;
                state.phase = if state.held_s + 1e-9 >= STOP_HOLD_S {
                    StopPhase::Released
                } else {
                    StopPhase::Holding
                };
            } else {
                *since = None;
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
    use rustdrive_core::{Pose, Vec2};
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
    #[test]
    fn only_a_continuous_healthy_near_line_stop_releases() {
        let mut s = stops();
        let r = route();
        for i in 0..60 {
            s.step(&r, i as f64 * 0.05, ego(10.0, 0.0), 1.25, Some(0.0), true);
        }
        assert_eq!(s.status().stops[0].phase, StopPhase::Approaching);
        for i in 60..100 {
            s.step(&r, i as f64 * 0.05, ego(33.0, 0.0), 1.25, Some(0.0), true);
        }
        assert_eq!(s.status().stops[0].phase, StopPhase::Holding);
        s.step(&r, 5.0, ego(33.0, 0.0), 1.25, None, false);
        assert_eq!(s.status().stops[0].held_s, 0.0);
        for i in 101..142 {
            s.step(&r, i as f64 * 0.05, ego(33.0, 0.0), 1.25, Some(0.0), true);
        }
        assert_eq!(s.status().stops[0].phase, StopPhase::Released);
        assert!(s.endpoint().is_none());
        s.step(&r, 7.15, ego(36.0, 1.0), 1.25, Some(1.0), false);
        assert_eq!(s.status().stops[0].phase, StopPhase::Released);
        s.step(&r, 7.2, ego(36.0, 1.0), 1.25, Some(1.0), true);
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
                s.step(
                    &r,
                    i as f64 * 0.05,
                    e,
                    1.25,
                    if mode == 1 { None } else { Some(e.speed) },
                    true,
                );
            }
            assert_eq!(s.status().stops[0].phase, StopPhase::Approaching);
            assert!(s.endpoint().is_some());
        }
    }
}
