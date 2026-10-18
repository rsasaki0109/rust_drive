//! A single opt-in mission retry after a measured early-return stop.
use super::returning::{
    RETURN_CLEARANCE_M, RETURN_TRACKING_RESERVE_M, ReturnObservation, observed_clearance,
};
use super::{
    LaneChangeConfig, LaneChangePhase, LaneChangeStatus, blocks, blocks_with_lateral_extra,
};
use crate::MotionLimits;
use rustdriving_core::{DrivingMode, Planner, Route, VehicleConfig, wrap_angle};
use rustdriving_planning::LatticePlanner;
use rustdriving_routing::lane_resume::{self, LaneResumeRequest};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneChangeResumeConfig {
    pub max_change_distance_m: f64,
}
impl LaneChangeResumeConfig {
    pub fn validate(&self) -> Result<(), String> {
        if !self.max_change_distance_m.is_finite()
            || !(10.0..=60.0).contains(&self.max_change_distance_m)
        {
            return Err("resume distance must be finite and in 10..60 m".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResumeDecision {
    AwaitingStop,
    InvalidEvidence,
    TrafficBlocked,
    AwaitingClear,
    InfeasibleGeometry,
    InfeasibleTrajectory,
    Resuming,
    Completed,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneChangeResumeStatus {
    pub decision: ResumeDecision,
    pub source_blocking_tracks: Vec<u64>,
    pub target_blocking_tracks: Vec<u64>,
    pub stopped_at: Option<f64>,
    pub clear_since: Option<f64>,
    pub started_at: Option<f64>,
    pub request: Option<LaneResumeRequest>,
    pub curvature_bound_m_inv: Option<f64>,
    pub observed_clearance_m: Option<f64>,
    pub connector_length_m: Option<f64>,
}

pub(super) struct ResumeMonitor {
    config: LaneChangeResumeConfig,
    target: LaneChangeConfig,
    source: LaneChangeConfig,
    vehicle: VehicleConfig,
    cruise: f64,
    route: Option<Route>,
    status: LaneChangeResumeStatus,
    last_scan: Option<f64>,
    last_motion: Option<f64>,
    after_fault: Option<f64>,
    clear_samples: usize,
}
impl ResumeMonitor {
    pub fn new(
        target: &LaneChangeConfig,
        source: &LaneChangeConfig,
        vehicle: VehicleConfig,
        cruise: f64,
    ) -> Option<Self> {
        let config = target
            .return_on_conflict
            .as_ref()?
            .resume_after_return
            .clone()?;
        Some(Self {
            config,
            target: target.clone(),
            source: source.clone(),
            vehicle,
            cruise,
            route: None,
            last_scan: None,
            last_motion: None,
            after_fault: None,
            clear_samples: 0,
            status: LaneChangeResumeStatus {
                decision: ResumeDecision::AwaitingStop,
                source_blocking_tracks: vec![],
                target_blocking_tracks: vec![],
                stopped_at: None,
                clear_since: None,
                started_at: None,
                request: None,
                curvature_bound_m_inv: None,
                observed_clearance_m: None,
                connector_length_m: None,
            },
        })
    }
    pub fn arm(&mut self, now: f64) {
        self.status.stopped_at = Some(now);
        self.after_fault = Some(now);
        self.status.clear_since = None;
        self.clear_samples = 0;
    }
    pub fn status(&self) -> LaneChangeResumeStatus {
        self.status.clone()
    }
    pub fn route(&self) -> Option<&Route> {
        self.route.as_ref()
    }
    pub fn step(
        &mut self,
        state: &mut LaneChangeStatus,
        o: &ReturnObservation<'_>,
        stopped_route: &Route,
    ) -> bool {
        if let Some(route) = &self.route {
            if state.phase == LaneChangePhase::Resuming
                && o.healthy
                && route.project(o.ego.pose.position).0 - self.vehicle.radius
                    >= self.status.connector_length_m.unwrap()
                && self.target.coordinates(o.ego.pose.position).1.abs() + self.vehicle.radius
                    <= self.target.target_width_m / 2.0
            {
                state.phase = LaneChangePhase::Completed;
                self.status.decision = ResumeDecision::Completed;
            }
            return false;
        }
        if state.phase != LaneChangePhase::Aborted {
            return false;
        }
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
        let measured = o
            .motion
            .filter(|m| m.stamp <= o.now && o.now - m.stamp <= 0.05 + 1e-9);
        let new_motion = measured.is_some_and(|m| self.last_motion.is_none_or(|old| m.stamp > old));
        if let Some(m) = measured {
            self.last_motion = Some(m.stamp);
        }
        let axis = self.source.target_end.minus(self.source.target_start);
        let fresh = o.healthy
            && o.scan
                .is_some_and(|s| s <= o.now && o.now - s <= 0.15 + 1e-9)
            && measured.is_some_and(|m| m.speed.abs() <= 0.05 && m.yaw_rate.abs() <= 0.01)
            && o.ego.speed.abs() <= 0.1
            && self.source.coordinates(o.ego.pose.position).1.abs() + self.vehicle.radius + 0.05
                <= self.source.target_width_m / 2.0
            && wrap_angle(o.ego.pose.yaw - axis.y.atan2(axis.x)).abs() <= 0.15
            && stopped_route.length() - stopped_route.project(o.ego.pose.position).0 <= 2.0;
        let speed = self.cruise.max(o.ego.speed) + 0.5;
        self.status.target_blocking_tracks = o
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
        self.status.source_blocking_tracks = o
            .predictions
            .iter()
            .filter(|p| {
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
        let blocked = !self.status.source_blocking_tracks.is_empty()
            || !self.status.target_blocking_tracks.is_empty();
        if !fresh || gap || blocked {
            self.status.clear_since = None;
            self.clear_samples = 0;
        }
        if !fresh {
            self.after_fault = Some(o.now);
            self.status.decision = ResumeDecision::InvalidEvidence;
            return false;
        }
        if blocked {
            self.status.decision = ResumeDecision::TrafficBlocked;
            return false;
        }
        self.status.decision = ResumeDecision::AwaitingClear;
        if !new_scan
            || !new_motion
            || self
                .after_fault
                .is_some_and(|t| o.scan.unwrap() <= t || measured.unwrap().stamp <= t)
        {
            return false;
        }
        let first = *self.status.clear_since.get_or_insert(o.scan.unwrap());
        self.clear_samples += 1;
        if self.clear_samples < 3 || o.scan.unwrap() - first < 1.0 - 1e-9 {
            return false;
        }
        let maximum = self.config.max_change_distance_m;
        let mut distances: Vec<f64> = (10..=maximum.floor() as usize).map(|n| n as f64).collect();
        if maximum.fract() > 0.0 {
            distances.push(maximum);
        }
        for distance in distances {
            match self.candidate(o, distance) {
                Ok((request, plan, clearance)) => {
                    self.status.decision = ResumeDecision::Resuming;
                    self.status.started_at = Some(o.now);
                    self.status.request = Some(request);
                    self.status.curvature_bound_m_inv = Some(plan.curvature_bound_m_inv);
                    self.status.observed_clearance_m = Some(clearance);
                    self.status.connector_length_m = Some(plan.connector_length_m);
                    self.route = Some(plan.route);
                    state.phase = LaneChangePhase::Resuming;
                    state.blocking_tracks.clear();
                    state.stop_s_m = None;
                    return true;
                }
                Err(reason) => self.status.decision = reason,
            }
        }
        false
    }
    fn candidate(
        &self,
        o: &ReturnObservation<'_>,
        distance: f64,
    ) -> Result<(LaneResumeRequest, lane_resume::LaneResumePlan, f64), ResumeDecision> {
        let r = self.target.return_on_conflict.as_ref().unwrap();
        let request = LaneResumeRequest {
            start: o.ego.pose,
            source_start: r.source_start,
            source_end: r.source_end,
            source_width_m: r.source_width_m,
            target_start: self.target.target_start,
            target_end: self.target.target_end,
            target_width_m: self.target.target_width_m,
            original_exit_s_m: self.target.exit_s_m,
            distance_m: distance,
        };
        let plan = lane_resume::plan(request, o.route, self.vehicle)
            .map_err(|_| ResumeDecision::InfeasibleGeometry)?;
        let mut preview = calibrated_planner(self.vehicle, self.cruise, o.limits);
        let trajectory = preview.plan(o.ego, &plan.route, o.predictions);
        if trajectory.mode == DrivingMode::Emergency
            || trajectory.points.last().is_none_or(|p| {
                plan.route.project(p.position).0
                    < plan.connector_length_m + self.vehicle.radius + 0.5
                    || p.speed <= 0.05
            })
        {
            return Err(ResumeDecision::InfeasibleTrajectory);
        }
        let clearance = observed_clearance(&trajectory.points, o.predictions, self.vehicle.radius);
        if clearance < RETURN_CLEARANCE_M + RETURN_TRACKING_RESERVE_M {
            return Err(ResumeDecision::InfeasibleTrajectory);
        }
        Ok((request, plan, clearance))
    }
}

fn calibrated_planner(vehicle: VehicleConfig, cruise: f64, limits: MotionLimits) -> LatticePlanner {
    let mut p = LatticePlanner::default();
    p.vehicle = vehicle;
    p.local_route_geometry = true;
    p.cruise_speed = cruise;
    p.max_acceleration_m_s2 = limits.max_acceleration_m_s2;
    p.max_deceleration_m_s2 = limits.max_deceleration_m_s2;
    p.max_lateral_acceleration_m_s2 = Some(limits.max_lateral_acceleration_m_s2);
    p
}

#[cfg(test)]
mod tests;
