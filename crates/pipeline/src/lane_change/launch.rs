//! Revoke an early resume while a calibrated source-lane stop still fits.
use super::resuming::calibrated_planner;
use super::returning::{
    RETURN_CLEARANCE_M, RETURN_TRACKING_RESERVE_M, ReturnObservation, observed_clearance,
};
use super::{LaneChangeConfig, blocks, blocks_with_lateral_extra};
use crate::traffic_controls::planning_prefix;
use rustdriving_core::{DrivingMode, Planner, Route, VehicleConfig};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LaunchHoldPhase {
    Braking,
    Holding,
    Confirming,
    Released,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneChangeLaunchHoldStatus {
    pub phase: LaunchHoldPhase,
    pub initiated_at: f64,
    /// Arc lengths on the fixed resume route, not the original itinerary.
    pub planning_endpoint_s_m: f64,
    pub source_limit_s_m: f64,
    pub reaction_speed_m_s: f64,
    pub deceleration_m_s2: f64,
    pub stopped_at: Option<f64>,
    pub after_fault_at: f64,
    pub clear_since: Option<f64>,
    pub released_at: Option<f64>,
    pub source_blocking_tracks: Vec<u64>,
    pub target_blocking_tracks: Vec<u64>,
    pub crossing_preview_clearance_m: Option<f64>,
}

pub(super) struct LaunchGuard {
    source: LaneChangeConfig,
    target: LaneChangeConfig,
    vehicle: VehicleConfig,
    cruise: f64,
    connector_length_m: f64,
    source_limit_s_m: f64,
    committed: bool,
    held_route: Option<Route>,
    status: Option<LaneChangeLaunchHoldStatus>,
    last_scan: Option<f64>,
    last_motion: Option<f64>,
    stopped_samples: usize,
    clear_samples: usize,
}
impl LaunchGuard {
    pub fn new(
        source: &LaneChangeConfig,
        target: &LaneChangeConfig,
        vehicle: VehicleConfig,
        cruise: f64,
        route: &Route,
        connector_length_m: f64,
        o: &ReturnObservation<'_>,
    ) -> Self {
        Self {
            source: source.clone(),
            target: target.clone(),
            vehicle,
            cruise,
            connector_length_m,
            source_limit_s_m: source_limit(route, source, vehicle),
            committed: false,
            held_route: None,
            status: None,
            last_scan: o.scan,
            last_motion: o.motion.map(|m| m.stamp),
            stopped_samples: 0,
            clear_samples: 0,
        }
    }
    pub fn status(&self) -> Option<LaneChangeLaunchHoldStatus> {
        self.status.clone()
    }
    pub fn route(&self) -> Option<&Route> {
        self.held_route.as_ref()
    }
    pub fn braking_limit(&self) -> Option<f64> {
        self.held_route.as_ref()?;
        Some(-self.status.as_ref()?.deceleration_m_s2)
    }
    pub fn step(&mut self, route: &Route, o: &ReturnObservation<'_>) -> bool {
        if self.committed {
            return false;
        }
        let progress = route.project(o.ego.pose.position).0;
        let measured = o
            .motion
            .filter(|m| m.stamp <= o.now && o.now - m.stamp <= 0.05 + 1e-9);
        let new_motion = measured.is_some_and(|m| self.last_motion.is_none_or(|old| m.stamp > old));
        if let Some(m) = measured {
            self.last_motion = Some(m.stamp);
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
        let fresh = o.healthy
            && measured.is_some()
            && o.scan
                .is_some_and(|s| s <= o.now && o.now - s <= 0.15 + 1e-9);
        let source_fits =
            self.source.coordinates(o.ego.pose.position).1.abs() + self.vehicle.radius + 0.05
                <= self.source.target_width_m / 2.0;
        let speed = self.cruise.max(o.ego.speed) + 0.5;
        let target_blockers: Vec<_> = o
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
        let source_blockers: Vec<_> = o
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
        let blocked = !target_blockers.is_empty() || !source_blockers.is_empty();
        if self.held_route.is_none() {
            // Reserve a 0.3 s response and stop using half the configured braking
            // capability. The planner stops 1 m before its supplied endpoint.
            let velocity = o
                .ego
                .speed
                .abs()
                .max(measured.map_or(0., |m| m.speed.abs()))
                + 0.1;
            let deceleration = 0.5 * o.limits.max_deceleration_m_s2;
            let distance = velocity * 0.3 + velocity * velocity / (2. * deceleration) + 0.1;
            let endpoint = progress + distance + 1.;
            if !source_fits || endpoint > self.source_limit_s_m {
                self.committed = true;
                return false;
            }
            if fresh && !gap && !blocked {
                return false;
            }
            self.held_route = planning_prefix(route, endpoint);
            if self.held_route.is_none() {
                self.committed = true;
                return false;
            }
            self.status = Some(LaneChangeLaunchHoldStatus {
                phase: LaunchHoldPhase::Braking,
                initiated_at: o.now,
                planning_endpoint_s_m: endpoint,
                source_limit_s_m: self.source_limit_s_m,
                reaction_speed_m_s: velocity,
                deceleration_m_s2: deceleration,
                stopped_at: None,
                after_fault_at: o.now,
                clear_since: None,
                released_at: None,
                source_blocking_tracks: source_blockers,
                target_blocking_tracks: target_blockers,
                crossing_preview_clearance_m: None,
            });
            return true;
        }
        let status = self.status.as_mut().unwrap();
        status.target_blocking_tracks = target_blockers;
        status.source_blocking_tracks = source_blockers;
        let stopped = fresh
            && source_fits
            && o.ego.speed.abs() <= 0.1
            && measured.is_some_and(|m| m.speed.abs() <= 0.05 && m.yaw_rate.abs() <= 0.01);
        if !stopped {
            status.phase = LaunchHoldPhase::Braking;
            status.after_fault_at = o.now;
            status.stopped_at = None;
            status.clear_since = None;
            self.clear_samples = 0;
            self.stopped_samples = 0;
            return false;
        }
        if new_motion && measured.unwrap().stamp > status.after_fault_at {
            self.stopped_samples += 1;
        }
        if self.stopped_samples < 3 {
            return false;
        }
        if status.stopped_at.is_none() {
            status.stopped_at = Some(o.now);
            status.after_fault_at = o.now;
            status.clear_since = None;
            self.clear_samples = 0;
        }
        status.phase = LaunchHoldPhase::Holding;
        if gap || blocked {
            status.clear_since = None;
            self.clear_samples = 0;
        }
        if !new_scan
            || !new_motion
            || blocked
            || o.scan.unwrap() <= status.after_fault_at
            || measured.unwrap().stamp <= status.after_fault_at
        {
            return false;
        }
        let first = *status.clear_since.get_or_insert(o.scan.unwrap());
        self.clear_samples += 1;
        status.phase = LaunchHoldPhase::Confirming;
        if self.clear_samples < 3 || o.scan.unwrap() - first < 1. - 1e-9 {
            return false;
        }
        let mut preview = calibrated_planner(self.vehicle, self.cruise, o.limits);
        let trajectory = preview.plan(o.ego, route, o.predictions);
        let crossing_end = self.connector_length_m + self.vehicle.radius + 0.5;
        let Some(last) = trajectory
            .points
            .iter()
            .position(|p| route.project(p.position).0 >= crossing_end)
        else {
            return false;
        };
        if trajectory.mode == DrivingMode::Emergency || trajectory.points[last].speed <= 0.05 {
            return false;
        }
        // Reserve the complete remaining crossing, including rear clearance.
        // The full planner still checks time-aligned collisions beyond that point.
        let clearance = observed_clearance(
            &trajectory.points[..=last],
            o.predictions,
            self.vehicle.radius,
        );
        status.crossing_preview_clearance_m = Some(clearance);
        if clearance < RETURN_CLEARANCE_M + RETURN_TRACKING_RESERVE_M {
            return false;
        }
        status.phase = LaunchHoldPhase::Released;
        status.released_at = Some(o.now);
        self.held_route = None;
        // Only this early stop/release is supported; later traffic retains the
        // ordinary planner and health braking, without another route or retry.
        self.committed = true;
        true
    }
}

fn source_limit(route: &Route, source: &LaneChangeConfig, vehicle: VehicleConfig) -> f64 {
    let margin = vehicle.radius + 0.25;
    let length = source.target_start.distance(source.target_end);
    let bounds = |p| {
        let (x, y) = source.coordinates(p);
        [
            x - margin,
            length - margin - x,
            source.target_width_m / 2. - margin - y,
            source.target_width_m / 2. - margin + y,
        ]
    };
    let mut previous = bounds(route.points[0]);
    if previous.iter().any(|v| *v < 0.) {
        return 0.;
    }
    for i in 1..route.points.len() {
        let next = bounds(route.points[i]);
        let mut fraction = 1.0_f64;
        for (a, b) in previous.iter().zip(next) {
            if b < 0. {
                fraction = fraction.min(a / (a - b));
            }
        }
        if fraction < 1. {
            return route.lengths[i - 1] + fraction * (route.lengths[i] - route.lengths[i - 1]);
        }
        previous = next;
    }
    route.length()
}

#[cfg(test)]
mod tests;
