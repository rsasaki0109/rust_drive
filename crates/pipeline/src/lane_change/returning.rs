//! Optional early return. Monitoring uses accepted scans and measured motion only.
use super::resuming::{LaneChangeResumeConfig, LaneChangeResumeStatus, ResumeMonitor};
use super::{
    LaneChangeConfig, LaneChangePhase, LaneChangeStatus, blocks, blocks_with_lateral_extra,
};
use crate::MotionLimits;
use rustdriving_core::{
    DrivingMode, EgoState, Odometry, Planner, Prediction, Route, Vec2, VehicleConfig,
};
use rustdriving_planning::LatticePlanner;
use rustdriving_routing::lane_return::{self, LaneReturnRequest};
use serde::{Deserialize, Serialize};
pub(super) const RETURN_CLEARANCE_M: f64 = 1.0;
pub(super) const RETURN_TRACKING_RESERVE_M: f64 = 0.3;

fn point_segment(point: Vec2, a: Vec2, b: Vec2) -> f64 {
    let d = b.minus(a);
    let q = point.minus(a);
    let squared = d.x * d.x + d.y * d.y;
    let t = if squared > 0.0 {
        ((q.x * d.x + q.y * d.y) / squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    point.distance(a.plus(d.scaled(t)))
}
fn segment_distance(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> f64 {
    let r = b.minus(a);
    let s = d.minus(c);
    let q = c.minus(a);
    let cross = |u: Vec2, v: Vec2| u.x * v.y - u.y * v.x;
    let denominator = cross(r, s);
    if denominator != 0.0 {
        let t = cross(q, s) / denominator;
        let u = cross(q, r) / denominator;
        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
            return 0.0;
        }
    }
    point_segment(a, c, d)
        .min(point_segment(b, c, d))
        .min(point_segment(c, a, b))
        .min(point_segment(d, a, b))
}
pub(super) fn observed_clearance(
    path: &[rustdriving_core::TrajectoryPoint],
    predictions: &[Prediction],
    radius: f64,
) -> f64 {
    // Conservative spatial separation across the entire bounded forecast, without
    // assuming that a traffic sample and an ego sample occur at different times.
    predictions
        .iter()
        .map(|p| {
            if !p.valid_radii() || p.positions.len() < 2 || !p.dt.is_finite() || p.dt <= 0.0 {
                return f64::NEG_INFINITY;
            }
            path.windows(2)
                .flat_map(|a| {
                    p.positions.windows(2).enumerate().map(move |(i, b)| {
                        segment_distance(a[0].position, a[1].position, b[0], b[1])
                            - radius
                            - p.radius_at(i as f64 * p.dt)
                                .max(p.radius_at((i + 1) as f64 * p.dt))
                    })
                })
                .fold(1000.0_f64, f64::min)
        })
        .fold(1000.0_f64, f64::min)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneChangeReturnConfig {
    pub source_start: Vec2,
    pub source_end: Vec2,
    pub source_width_m: f64,
    pub max_return_distance_m: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub resume_after_return: Option<LaneChangeResumeConfig>,
}
impl LaneChangeReturnConfig {
    pub fn validate(
        &self,
        gate: &LaneChangeConfig,
        route: &Route,
        vehicle: VehicleConfig,
    ) -> Result<(), String> {
        if let Some(config) = &self.resume_after_return {
            config.validate()?;
        }
        let a = self.source_end.minus(self.source_start);
        let b = gate.target_end.minus(gate.target_start);
        let length = a.x.hypot(a.y);
        let axis = b.scaled(1.0 / b.x.hypot(b.y));
        let offset = gate.target_start.minus(self.source_start);
        if !self.source_start.finite()
            || !self.source_end.finite()
            || [
                self.source_start.x,
                self.source_start.y,
                self.source_end.x,
                self.source_end.y,
            ]
            .iter()
            .any(|v| v.abs() > 1e6)
            || !self.source_width_m.is_finite()
            || !(2.6..=10.0).contains(&self.source_width_m)
            || self.source_width_m / 2.0 < vehicle.radius + 0.05
            || route.half_width > self.source_width_m / 2.0
            || !self.max_return_distance_m.is_finite()
            || !(10.0..=60.0).contains(&self.max_return_distance_m)
            || gate.entry_s_m + self.max_return_distance_m + vehicle.radius + 1.0 > length
            || self.source_start.distance(route.points[0]) > 1e-6
            || a.minus(b).x.hypot(a.minus(b).y) > 1e-6
            || (offset.x * axis.x + offset.y * axis.y).abs() > 1e-6
            || ((axis.x * offset.y - axis.y * offset.x).abs()
                - (self.source_width_m + gate.target_width_m) / 2.0)
                .abs()
                > 1e-6
        {
            return Err(
                "return requires explicit parallel adjacent source geometry and stopping room"
                    .into(),
            );
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReturnDecision {
    Monitoring,
    InvalidEvidence,
    SourceBlocked,
    AwaitingClear,
    OutsideSource,
    InfeasibleGeometry,
    InfeasibleTrajectory,
    Returning,
    Stopped,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneChangeReturnStatus {
    pub decision: ReturnDecision,
    pub source_blocking_tracks: Vec<u64>,
    pub source_clear_since: Option<f64>,
    pub started_at: Option<f64>,
    pub request: Option<LaneReturnRequest>,
    pub curvature_bound_m_inv: Option<f64>,
    pub observed_clearance_m: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub resume_state: Option<LaneChangeResumeStatus>,
}

pub(crate) struct ReturnObservation<'a> {
    pub route: &'a Route,
    pub ego: EgoState,
    pub predictions: &'a [Prediction],
    pub now: f64,
    pub scan: Option<f64>,
    pub healthy: bool,
    pub motion: Option<Odometry>,
    pub limits: MotionLimits,
}

pub(crate) struct ReturnMonitor {
    config: LaneChangeReturnConfig,
    source: LaneChangeConfig,
    target: LaneChangeConfig,
    vehicle: VehicleConfig,
    cruise: f64,
    route: Option<Route>,
    status: LaneChangeReturnStatus,
    last_scan: Option<f64>,
    after_fault: Option<f64>,
    clear_samples: usize,
    last_motion: Option<f64>,
    stopped_samples: usize,
    resume: Option<ResumeMonitor>,
}
impl ReturnMonitor {
    pub fn new(target: &LaneChangeConfig, vehicle: VehicleConfig, cruise: f64) -> Option<Self> {
        let config = target.return_on_conflict.clone()?;
        let mut source = target.clone();
        source.target_start = config.source_start;
        source.target_end = config.source_end;
        source.target_width_m = config.source_width_m;
        source.return_on_conflict = None;
        let resume = ResumeMonitor::new(target, &source, vehicle, cruise);
        Some(Self {
            config,
            source,
            target: target.clone(),
            vehicle,
            cruise,
            route: None,
            status: LaneChangeReturnStatus {
                decision: ReturnDecision::Monitoring,
                source_blocking_tracks: vec![],
                source_clear_since: None,
                started_at: None,
                request: None,
                curvature_bound_m_inv: None,
                observed_clearance_m: None,
                resume_state: None,
            },
            last_scan: None,
            after_fault: None,
            clear_samples: 0,
            last_motion: None,
            stopped_samples: 0,
            resume,
        })
    }
    pub fn status(&self) -> LaneChangeReturnStatus {
        let mut status = self.status.clone();
        status.resume_state = self.resume.as_ref().map(ResumeMonitor::status);
        status
    }
    pub fn route(&self) -> Option<&Route> {
        self.resume
            .as_ref()
            .and_then(ResumeMonitor::route)
            .or(self.route.as_ref())
    }
    pub fn braking_limit(&self) -> Option<f64> {
        self.resume.as_ref().and_then(ResumeMonitor::braking_limit)
    }
    pub fn step(&mut self, state: &mut LaneChangeStatus, o: ReturnObservation<'_>) -> bool {
        if state.phase == LaneChangePhase::Aborted
            || self.resume.as_ref().is_some_and(|m| m.route().is_some())
        {
            if let Some(resume) = &mut self.resume {
                return resume.step(state, &o, self.route.as_ref().expect("returned route"));
            }
            return false;
        }
        let fresh = o.healthy
            && o.scan
                .is_some_and(|s| s <= o.now && o.now - s <= 0.15 + 1e-9);
        let new_scan = o
            .scan
            .is_some_and(|s| self.last_scan.is_none_or(|old| s > old));
        let gap = new_scan
            && self
                .last_scan
                .zip(o.scan)
                .is_some_and(|(old, s)| s - old > 0.2 + 1e-9);
        if new_scan {
            self.last_scan = o.scan;
        }
        if !fresh {
            self.after_fault = Some(o.now);
        }
        let speed = self.cruise.max(o.ego.speed) + 0.5;
        self.status.source_blocking_tracks = o
            .predictions
            .iter()
            .filter(|p| {
                // Source occupancy uses its physical band. The complete return
                // preview separately requires 1 m clearance plus 0.3 m reserve.
                blocks_with_lateral_extra(
                    &self.source,
                    p,
                    self.source.coordinates(o.ego.pose.position).0,
                    speed,
                    self.vehicle.radius,
                    0.0,
                )
            })
            .map(|p| p.id)
            .collect();
        if !fresh || gap || !self.status.source_blocking_tracks.is_empty() {
            self.status.source_clear_since = None;
            self.clear_samples = 0;
        }
        if fresh
            && new_scan
            && self.status.source_blocking_tracks.is_empty()
            && self.after_fault.is_none_or(|epoch| o.scan.unwrap() > epoch)
        {
            self.status
                .source_clear_since
                .get_or_insert(o.scan.unwrap());
            self.clear_samples += 1;
        }
        if state.phase == LaneChangePhase::Aborting {
            let measured = o
                .motion
                .filter(|m| m.stamp <= o.now && o.now - m.stamp <= 0.05 + 1e-9);
            let stopped = o.healthy
                && measured.is_some_and(|m| {
                    m.speed.abs() <= 0.05 && self.after_fault.is_none_or(|epoch| m.stamp > epoch)
                })
                && o.ego.speed.abs() <= 0.1
                && self.source.coordinates(o.ego.pose.position).1.abs() + self.vehicle.radius
                    <= self.config.source_width_m / 2.0
                && self
                    .route
                    .as_ref()
                    .is_some_and(|r| r.length() - r.project(o.ego.pose.position).0 <= 2.0);
            if !stopped {
                self.stopped_samples = 0;
            }
            if let Some(motion) = measured {
                if self.last_motion.is_none_or(|old| motion.stamp > old) && stopped {
                    self.stopped_samples += 1;
                }
                self.last_motion = Some(motion.stamp);
            }
            if self.stopped_samples >= 3 {
                state.phase = LaneChangePhase::Aborted;
                self.status.decision = ReturnDecision::Stopped;
                if let Some(resume) = &mut self.resume {
                    resume.arm(o.now);
                }
            }
            return false;
        }
        if state.phase != LaneChangePhase::Changing {
            return false;
        }
        state.blocking_tracks = o
            .predictions
            .iter()
            .filter(|p| {
                blocks(
                    &self.target,
                    p,
                    self.target.coordinates(o.ego.pose.position).0,
                    speed,
                    self.vehicle.radius,
                )
            })
            .map(|p| p.id)
            .collect();
        self.status.decision = ReturnDecision::Monitoring;
        if state.blocking_tracks.is_empty() {
            return false;
        }
        let Some(motion) = o
            .motion
            .filter(|m| m.stamp <= o.now && o.now - m.stamp <= 0.05 + 1e-9 && m.speed >= 0.3)
        else {
            self.status.decision = ReturnDecision::InvalidEvidence;
            return false;
        };
        if !fresh || !new_scan {
            self.status.decision = ReturnDecision::InvalidEvidence;
            return false;
        }
        if !self.status.source_blocking_tracks.is_empty() {
            self.status.decision = ReturnDecision::SourceBlocked;
            return false;
        }
        if self.clear_samples < 3
            || self
                .status
                .source_clear_since
                .is_none_or(|s| o.scan.unwrap() - s < 1.0 - 1e-9)
        {
            self.status.decision = ReturnDecision::AwaitingClear;
            return false;
        }
        if self.source.coordinates(o.ego.pose.position).1.abs() + self.vehicle.radius + 0.05
            > self.config.source_width_m / 2.0
        {
            self.status.decision = ReturnDecision::OutsideSource;
            return false;
        }
        let maximum = self.config.max_return_distance_m;
        let mut distances: Vec<f64> = (10..=maximum.floor() as usize).map(|n| n as f64).collect();
        if maximum.fract() > 0.0 {
            distances.push(maximum);
        }
        let mut selected = None;
        for distance in distances {
            match self.candidate(&o, motion, distance) {
                Ok(candidate) => {
                    selected = Some(candidate);
                    break;
                }
                Err(decision) => self.status.decision = decision,
            }
        }
        let Some((request, plan, clearance)) = selected else {
            return false;
        };
        self.status.observed_clearance_m = Some(clearance);
        self.status.request = Some(request);
        self.status.started_at = Some(o.now);
        self.status.curvature_bound_m_inv = Some(plan.curvature_bound_m_inv);
        self.status.decision = ReturnDecision::Returning;
        self.route = Some(plan.route);
        state.phase = LaneChangePhase::Aborting;
        state.stop_s_m = None;
        true
    }
    fn candidate(
        &self,
        o: &ReturnObservation<'_>,
        motion: Odometry,
        distance: f64,
    ) -> Result<(LaneReturnRequest, lane_return::LaneReturnPlan, f64), ReturnDecision> {
        let request = LaneReturnRequest {
            start: o.ego.pose,
            curvature_m_inv: motion.yaw_rate / motion.speed,
            lane_start: self.config.source_start,
            lane_end: self.config.source_end,
            half_width_m: o.route.half_width.min(self.config.source_width_m / 2.0),
            distance_m: distance,
        };
        let plan = lane_return::plan(request, self.vehicle)
            .map_err(|_| ReturnDecision::InfeasibleGeometry)?;
        let source_offset = self.target.coordinates(self.config.source_start).1;
        let lower = (-self.target.target_width_m / 2.0)
            .min(source_offset - self.config.source_width_m / 2.0);
        let upper = (self.target.target_width_m / 2.0)
            .max(source_offset + self.config.source_width_m / 2.0);
        if plan.lateral_controls_m.iter().any(|y| {
            source_offset + y - self.vehicle.radius - 0.05 < lower
                || source_offset + y + self.vehicle.radius + 0.05 > upper
        }) {
            return Err(ReturnDecision::InfeasibleGeometry);
        }
        let mut preview = LatticePlanner::default();
        preview.vehicle = self.vehicle;
        preview.local_route_geometry = true;
        preview.cruise_speed = self.cruise;
        preview.max_acceleration_m_s2 = o.limits.max_acceleration_m_s2;
        preview.max_deceleration_m_s2 = o.limits.max_deceleration_m_s2;
        preview.max_lateral_acceleration_m_s2 = Some(o.limits.max_lateral_acceleration_m_s2);
        let trajectory = preview.plan(o.ego, &plan.route, o.predictions);
        if trajectory.mode == DrivingMode::Emergency
            || trajectory.points.last().is_none_or(|p| {
                p.position.distance(*plan.route.points.last().unwrap()) > 2.0 || p.speed > 0.05
            })
        {
            return Err(ReturnDecision::InfeasibleTrajectory);
        }
        let clearance = observed_clearance(&trajectory.points, o.predictions, self.vehicle.radius);
        if clearance < RETURN_CLEARANCE_M + RETURN_TRACKING_RESERVE_M {
            return Err(ReturnDecision::InfeasibleTrajectory);
        }
        Ok((request, plan, clearance))
    }
}

#[cfg(test)]
mod tests;
