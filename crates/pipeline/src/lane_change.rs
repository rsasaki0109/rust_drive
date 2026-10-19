//! Observed-gap admission for one authored lane change; no truth or actor intentions.
use crate::traffic_controls::planning_prefix;
use rustdriving_core::{EgoState, Prediction, Route, Vec2, VehicleConfig, wrap_angle};
use serde::{Deserialize, Serialize};
mod launch;
mod resuming;
mod returning;
pub use launch::{
    LaneChangeLaunchHoldRecord, LaneChangeLaunchHoldStatus, LaneChangeLaunchLimitEvent,
    LaunchHoldPhase,
};
pub use resuming::{LaneChangeResumeConfig, LaneChangeResumeStatus, ResumeDecision};
use returning::ReturnMonitor;
pub(crate) use returning::ReturnObservation;
pub use returning::{LaneChangeReturnConfig, LaneChangeReturnStatus, ReturnDecision};

const HORIZON_S: f64 = 6.0;
const MIN_GAP_M: f64 = 6.0;
const HEADWAY_S: f64 = 2.0;
const CLEAR_S: f64 = 1.0;
const MAX_SCAN_AGE_S: f64 = 0.15;
const MAX_SCAN_GAP_S: f64 = 0.2;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneChangeConfig {
    /// Arc lengths on the resolved ego route, in meters.
    pub entry_s_m: f64,
    pub exit_s_m: f64,
    /// Straight target lane centerline, world ENU meters, ordered in travel direction.
    pub target_start: Vec2,
    pub target_end: Vec2,
    pub target_width_m: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub return_on_conflict: Option<LaneChangeReturnConfig>,
}
impl LaneChangeConfig {
    fn coordinates(&self, point: Vec2) -> (f64, f64) {
        let d = self.target_end.minus(self.target_start);
        let axis = d.scaled(1.0 / d.x.hypot(d.y));
        let p = point.minus(self.target_start);
        (p.x * axis.x + p.y * axis.y, axis.x * p.y - axis.y * p.x)
    }
    pub fn validate(&self, route: &Route, vehicle: VehicleConfig) -> Result<(), String> {
        if route.points.len() > 5000 || !vehicle.radius.is_finite() || vehicle.radius <= 0.0 {
            return Err("invalid lane-change route or vehicle radius".into());
        }
        let canonical = Route::new(route.points.clone(), route.half_width)?;
        if canonical.lengths != route.lengths
            || !canonical.length().is_finite()
            || canonical.length() > 1500.0
        {
            return Err("invalid lane-change route arc metadata".into());
        }
        let target_length = self.target_start.distance(self.target_end);
        if !self.target_start.finite()
            || !self.target_end.finite()
            || [
                self.target_start.x,
                self.target_start.y,
                self.target_end.x,
                self.target_end.y,
            ]
            .iter()
            .any(|x| x.abs() > 1e6)
            || !target_length.is_finite()
            || !(20.0..=1000.0).contains(&target_length)
            || !self.target_width_m.is_finite()
            || !(2.6..=10.0).contains(&self.target_width_m)
            || self.target_width_m / 2.0 < vehicle.radius + 0.05
            || route.half_width > self.target_width_m / 2.0
            || !self.entry_s_m.is_finite()
            || !self.exit_s_m.is_finite()
            || self.entry_s_m < vehicle.radius + 5.0
            || self.exit_s_m < self.entry_s_m + 10.0
            || self.exit_s_m > route.length() - vehicle.radius - 3.0
        {
            return Err("invalid lane-change gate geometry or approach/exit margin".into());
        }
        let (start_x, source_y) = self.coordinates(route.points[0]);
        let (entry_x, entry_y) = self.coordinates(route.sample(self.entry_s_m, 0.0).0);
        let (exit_x, exit_y) = self.coordinates(route.sample(self.exit_s_m, 0.0).0);
        if start_x.abs() > 1e-6
            || (entry_x - self.entry_s_m).abs() > 1e-6
            || (entry_y - source_y).abs() > 1e-6
            || exit_y.abs() > 1e-6
            || source_y.abs() + 1e-6 < route.half_width + self.target_width_m / 2.0
            || source_y.abs() > 12.0
            || exit_x <= entry_x
            || exit_x > target_length
        {
            return Err("lane-change gate does not match its source prefix and target lane".into());
        }
        let sign = source_y.signum();
        let mut previous_x = 0.0;
        let mut previous_y = source_y.abs();
        for (&point, &s) in route
            .points
            .iter()
            .zip(&route.lengths)
            .take_while(|(_, s)| **s <= self.exit_s_m + 1e-6)
        {
            let (x, y) = self.coordinates(point);
            let y = sign * y;
            if x < previous_x - 1e-6
                || y < -1e-6
                || y > previous_y + 1e-6
                || (s <= self.entry_s_m && (y - source_y.abs()).abs() > 1e-6)
            {
                return Err(
                    "lane-change gate requires a straight prefix and monotonic connector".into(),
                );
            }
            previous_x = x;
            previous_y = y;
        }
        if let Some(config) = &self.return_on_conflict {
            config.validate(self, route, vehicle)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LaneChangePhase {
    Waiting,
    Ready,
    Changing,
    Completed,
    Aborting,
    Aborted,
    Resuming,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LaneChangeStatus {
    pub phase: LaneChangePhase,
    pub blocking_tracks: Vec<u64>,
    pub front_gap_m: Option<f64>,
    pub rear_gap_m: Option<f64>,
    pub clear_since: Option<f64>,
    pub stop_s_m: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub return_state: Option<LaneChangeReturnStatus>,
}

// Clip a segment's parameter against a linear inequality value >= 0.
fn clip(a: f64, b: f64, low: &mut f64, high: &mut f64) -> bool {
    let delta = b - a;
    if delta > 0.0 {
        *low = low.max(-a / delta);
    } else if delta < 0.0 {
        *high = high.min(-a / delta);
    } else if a < 0.0 {
        return false;
    }
    *low <= *high
}

fn blocks(
    config: &LaneChangeConfig,
    prediction: &Prediction,
    ego_x: f64,
    speed: f64,
    radius: f64,
) -> bool {
    blocks_with_lateral_extra(config, prediction, ego_x, speed, radius, 0.5)
}

fn blocks_with_lateral_extra(
    config: &LaneChangeConfig,
    prediction: &Prediction,
    ego_x: f64,
    speed: f64,
    radius: f64,
    lateral_extra: f64,
) -> bool {
    if !prediction.dt.is_finite()
        || prediction.dt <= 0.0
        || prediction.positions.len() < 2
        || prediction.positions.len() > 1024
        || prediction.dt * ((prediction.positions.len() - 1) as f64) < HORIZON_S
        || !prediction.radius.is_finite()
        || !prediction.valid_radii()
        || !(0.0..=200.0).contains(&prediction.radius)
        || prediction
            .positions
            .iter()
            .any(|p| !p.finite() || p.x.abs() > 2e6 || p.y.abs() > 2e6)
    {
        return true;
    }
    let positions: Vec<_> = prediction
        .positions
        .iter()
        .map(|p| config.coordinates(*p))
        .collect();
    if positions
        .iter()
        .any(|(x, y)| !x.is_finite() || !y.is_finite())
    {
        return true;
    }
    let rear_speed = positions
        .windows(2)
        .enumerate()
        .map(|(i, p)| {
            // An uncertain forecast's center can move more slowly than its
            // forward occupied edge. Include radius growth in closing headway;
            // shrinking uncertainty must not discount measured center motion.
            let growth = if prediction.additional_radius_m.is_empty() {
                0.0
            } else {
                (prediction.additional_radius_m[i + 1] - prediction.additional_radius_m[i]).max(0.0)
                    / prediction.dt
            };
            (p[1].0 - p[0].0) / prediction.dt + growth
        })
        .fold(0.0_f64, f64::max);
    if !rear_speed.is_finite() || rear_speed > 30.0 {
        return true;
    }
    // Ego's assumed forward travel spans [0, speed*t]. Segment clipping covers
    // continuous traffic motion, including a lateral crossing between samples.
    for (i, pair) in positions.windows(2).enumerate() {
        let t0 = i as f64 * prediction.dt;
        if t0 >= HORIZON_S {
            break;
        }
        let t1 = ((i + 1) as f64 * prediction.dt).min(HORIZON_S);
        let occupied_radius = prediction.radius_at(t0).max(prediction.radius_at(t1));
        let body = radius + occupied_radius;
        let behind = body + MIN_GAP_M + rear_speed * HEADWAY_S;
        let ahead = body + MIN_GAP_M + speed * HEADWAY_S;
        let lateral = config.target_width_m / 2.0 + occupied_radius + lateral_extra;
        let fraction = (t1 - t0) / prediction.dt;
        let a = pair[0];
        let b = (
            a.0 + fraction * (pair[1].0 - a.0),
            a.1 + fraction * (pair[1].1 - a.1),
        );
        let mut low = 0.0_f64;
        let mut high = 1.0_f64;
        if clip(
            a.0 - ego_x + behind,
            b.0 - ego_x + behind,
            &mut low,
            &mut high,
        ) && clip(
            ego_x + speed * t0 + ahead - a.0,
            ego_x + speed * t1 + ahead - b.0,
            &mut low,
            &mut high,
        ) && clip(a.1 + lateral, b.1 + lateral, &mut low, &mut high)
            && clip(lateral - a.1, lateral - b.1, &mut low, &mut high)
        {
            return true;
        }
    }
    false
}

pub(crate) struct LaneChangeGate {
    config: LaneChangeConfig,
    state: LaneChangeStatus,
    last_scan: Option<f64>,
    after_fault: Option<f64>,
    clear_samples: usize,
    vehicle: VehicleConfig,
    cruise: f64,
    return_monitor: Option<ReturnMonitor>,
}
impl LaneChangeGate {
    pub fn new(config: LaneChangeConfig, vehicle: VehicleConfig, cruise: f64) -> Self {
        let return_monitor = ReturnMonitor::new(&config, vehicle, cruise);
        Self {
            config,
            state: LaneChangeStatus {
                phase: LaneChangePhase::Waiting,
                blocking_tracks: vec![],
                front_gap_m: None,
                rear_gap_m: None,
                clear_since: None,
                stop_s_m: None,
                return_state: None,
            },
            last_scan: None,
            after_fault: None,
            clear_samples: 0,
            vehicle,
            cruise,
            return_monitor,
        }
    }
    pub fn step(
        &mut self,
        route: &Route,
        ego: EgoState,
        predictions: &[Prediction],
        now: f64,
        scan: Option<f64>,
        healthy: bool,
    ) {
        let vehicle = self.vehicle;
        let (progress, lateral) = route.project(ego.pose.position);
        if matches!(
            self.state.phase,
            LaneChangePhase::Aborting | LaneChangePhase::Aborted | LaneChangePhase::Resuming
        ) {
            return;
        }
        if matches!(
            self.state.phase,
            LaneChangePhase::Changing | LaneChangePhase::Completed
        ) {
            if healthy
                && progress - vehicle.radius >= self.config.exit_s_m
                && self.config.coordinates(ego.pose.position).1.abs() + vehicle.radius
                    <= self.config.target_width_m / 2.0
            {
                self.state.phase = LaneChangePhase::Completed;
            }
            return;
        }
        let aligned = wrap_angle(ego.pose.yaw - route.sample(progress, 0.0).1).abs() <= 0.25
            && lateral.abs() + vehicle.radius <= route.half_width;
        let fresh = healthy
            && aligned
            && scan.is_some_and(|s| s <= now && now - s <= MAX_SCAN_AGE_S + 1e-9);
        let new_scan = scan.is_some_and(|s| self.last_scan.is_none_or(|old| s > old));
        let gap = new_scan
            && self
                .last_scan
                .zip(scan)
                .is_some_and(|(old, s)| s - old > MAX_SCAN_GAP_S + 1e-9);
        if new_scan {
            self.last_scan = scan;
        }
        if !fresh {
            self.after_fault = Some(now);
        }
        self.state.blocking_tracks.clear();
        self.state.front_gap_m = None;
        self.state.rear_gap_m = None;
        let ego_x = self.config.coordinates(ego.pose.position).0;
        let speed = self.cruise.max(ego.speed) + 0.5;
        for p in predictions {
            if blocks(&self.config, p, ego_x, speed, vehicle.radius) {
                self.state.blocking_tracks.push(p.id);
            }
            if let Some(first) = p.positions.first().filter(|p| p.finite()) {
                let (x, y) = self.config.coordinates(*first);
                let occupied_radius = p.radius_at(0.0);
                if occupied_radius.is_finite()
                    && y.abs() <= self.config.target_width_m / 2.0 + occupied_radius + 0.5
                {
                    let distance = (x - ego_x).abs() - vehicle.radius - occupied_radius;
                    let field = if x >= ego_x {
                        &mut self.state.front_gap_m
                    } else {
                        &mut self.state.rear_gap_m
                    };
                    *field = Some(field.map_or(distance, |old| old.min(distance)));
                }
            }
        }
        if !fresh || gap || !self.state.blocking_tracks.is_empty() {
            self.state.phase = LaneChangePhase::Waiting;
            self.state.clear_since = None;
            self.clear_samples = 0;
        }
        if fresh
            && self.state.blocking_tracks.is_empty()
            && new_scan
            && self.after_fault.is_none_or(|epoch| scan.unwrap() > epoch)
        {
            let stamp = scan.unwrap();
            let first = *self.state.clear_since.get_or_insert(stamp);
            self.clear_samples += 1;
            if self.clear_samples >= 3 && stamp - first + 1e-9 >= CLEAR_S {
                self.state.phase = LaneChangePhase::Ready;
            }
        }
        if self.state.phase == LaneChangePhase::Ready && fresh && progress >= self.config.entry_s_m
        {
            self.state.phase = LaneChangePhase::Changing;
        }
        self.state.stop_s_m = (self.state.phase == LaneChangePhase::Waiting)
            .then_some(self.config.entry_s_m - vehicle.radius - 1.0);
    }
    pub fn status(&self) -> LaneChangeStatus {
        let mut status = self.state.clone();
        status.return_state = self.return_monitor.as_ref().map(ReturnMonitor::status);
        status
    }
    pub fn observe_return(&mut self, observation: ReturnObservation<'_>) -> bool {
        self.return_monitor
            .as_mut()
            .is_some_and(|m| m.step(&mut self.state, observation))
    }
    pub fn returning(&self) -> bool {
        matches!(
            self.state.phase,
            LaneChangePhase::Aborting | LaneChangePhase::Aborted
        )
    }
    pub fn launch_braking_limit(&self) -> Option<f64> {
        self.return_monitor
            .as_ref()
            .and_then(ReturnMonitor::braking_limit)
    }
    pub fn alternate_speed_zones(&self) -> Vec<rustdriving_core::RouteSpeedZone> {
        self.return_monitor
            .as_ref()
            .map_or_else(Vec::new, ReturnMonitor::speed_zones)
    }
    pub fn planning_route(&self, route: &Route) -> Option<Route> {
        if let Some(route) = self.return_monitor.as_ref().and_then(ReturnMonitor::route) {
            return Some(route.clone());
        }
        planning_prefix(route, self.state.stop_s_m?)
    }
    pub fn holding_brake(&self, route: &Route, ego: EgoState) -> bool {
        if self.returning() {
            return self
                .return_monitor
                .as_ref()
                .and_then(ReturnMonitor::route)
                .is_some_and(|r| {
                    r.length() - r.project(ego.pose.position).0 <= 3.5 && ego.speed.abs() <= 0.2
                });
        }
        self.state.stop_s_m.is_some_and(|end| {
            end + 1.0 - route.project(ego.pose.position).0 <= 3.5 && ego.speed.abs() <= 0.2
        })
    }
    pub fn approach_speed_limit(
        &self,
        route: &Route,
        ego: EgoState,
        radius: f64,
        deceleration: f64,
    ) -> f64 {
        if self.state.phase != LaneChangePhase::Ready {
            return f64::INFINITY;
        }
        let available =
            (self.config.entry_s_m - route.project(ego.pose.position).0 - radius - 1.0).max(0.0);
        let braking = deceleration * 0.5;
        let response = braking * 0.3;
        ((response * response + 2.0 * braking * available).sqrt() - response).max(0.6)
    }
}
