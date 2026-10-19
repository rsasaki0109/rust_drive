//! Known map updates and a conservative stop-before-divergence route handover.
use rustdriving_core::{EgoState, Odometry, Route, wrap_angle};
use rustdriving_routing::{RoadNetwork, RoadNetworkSpec, RoutePlan, SpeedRoutePlan};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NavigationConfig {
    pub network: RoadNetworkSpec,
    pub start: String,
    pub goal: String,
    #[serde(default)]
    pub closed_edges: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::{Pose, Vec2};
    fn config() -> NavigationConfig {
        let value: serde_json::Value =
            serde_json::from_str(include_str!("../../../scenarios/route-direct.json")).unwrap();
        serde_json::from_value(value["navigation"].clone()).unwrap()
    }
    fn ego(x: f64, speed: f64) -> EgoState {
        EgoState {
            pose: Pose {
                position: Vec2::new(x, 0.0),
                yaw: 0.0,
            },
            speed,
        }
    }
    fn update(stamp: f64, revision: u64, closed: &[&str]) -> NavigationUpdate {
        NavigationUpdate {
            stamp,
            revision,
            closed_edges: closed.iter().map(|s| (*s).into()).collect(),
        }
    }
    fn step(
        n: &mut Navigator,
        u: Option<&NavigationUpdate>,
        t: f64,
        e: EgoState,
        healthy: bool,
    ) -> bool {
        n.step(
            u,
            t,
            e,
            healthy.then_some(Odometry {
                stamp: t,
                speed: e.speed,
                yaw_rate: 0.0,
            }),
            1.25,
            2.5,
        )
    }
    #[test]
    fn handover_requires_three_healthy_stopped_samples_before_the_divergence() {
        let mut n = Navigator::new(config()).unwrap();
        assert!(!step(
            &mut n,
            Some(&update(1.0, 1, &["main"])),
            1.0,
            ego(10.0, 4.0),
            true
        ));
        assert_eq!(n.status.phase, NavigationPhase::Braking);
        assert_eq!(n.status.stop_s, Some(36.75));
        assert!(n.status.active_edges.contains(&"main".into()));
        assert!(n.planning_route().unwrap().length() < 40.0);
        for i in 1..=3 {
            assert!(!step(
                &mut n,
                None,
                1.0 + i as f64 * 0.05,
                ego(20.0, 4.0),
                true
            ));
        }
        for i in 1..=2 {
            assert!(!step(
                &mut n,
                None,
                2.0 + i as f64 * 0.05,
                ego(36.0, 0.0),
                true
            ));
        }
        assert!(!step(&mut n, None, 2.15, ego(36.0, 0.0), false));
        for i in 1..=2 {
            assert!(!step(
                &mut n,
                None,
                3.0 + i as f64 * 0.05,
                ego(36.0, 0.0),
                true
            ));
        }
        assert!(step(&mut n, None, 3.15, ego(36.0, 0.0), true));
        assert_eq!(n.status.phase, NavigationPhase::Following);
        assert_eq!(n.status.switches, 1);
        assert_eq!(n.plan().edge_ids, vec!["approach", "detour", "east-exit"]);
    }
    #[test]
    fn duplicate_and_reordered_updates_cannot_undo_pending_closures() {
        let mut n = Navigator::new(config()).unwrap();
        step(
            &mut n,
            Some(&update(1.0, 1, &["main"])),
            1.0,
            ego(10.0, 4.0),
            true,
        );
        step(
            &mut n,
            Some(&update(2.0, 1, &[])),
            2.0,
            ego(10.0, 4.0),
            true,
        );
        step(
            &mut n,
            Some(&update(0.5, 2, &[])),
            2.05,
            ego(10.0, 4.0),
            true,
        );
        assert_eq!(n.status.revision, 1);
        assert_eq!(n.status.phase, NavigationPhase::Braking);
        step(
            &mut n,
            Some(&update(2.1, 2, &[])),
            2.1,
            ego(10.0, 4.0),
            true,
        );
        assert_eq!(n.status.phase, NavigationPhase::Following);
        assert!(n.pending.is_none());
        assert_eq!(n.status.switches, 0);
    }
    #[test]
    fn malformed_new_snapshots_latch_fault_until_a_valid_snapshot() {
        for bad in [
            update(1.0, 1, &["typo"]),
            update(2.0, 1, &["main"]),
            update(f64::NAN, 1, &[]),
            update(0.0, 1, &[]),
        ] {
            let mut n = Navigator::new(config()).unwrap();
            step(&mut n, Some(&bad), 1.5, ego(10.0, 4.0), true);
            assert_eq!(n.status.phase, NavigationPhase::Fault);
            step(&mut n, None, 1.55, ego(10.0, 0.0), true);
            assert_eq!(n.status.phase, NavigationPhase::Fault);
            step(
                &mut n,
                Some(&update(1.6, 1, &[])),
                1.6,
                ego(10.0, 0.0),
                true,
            );
            assert_eq!(n.status.phase, NavigationPhase::Following);
        }
    }
    #[test]
    fn unreachable_late_or_too_narrow_routes_never_handover() {
        for (closed, x) in [(&["main", "detour"][..], 10.0), (&["main"][..], 60.0)] {
            let mut n = Navigator::new(config()).unwrap();
            step(
                &mut n,
                Some(&update(1.0, 1, closed)),
                1.0,
                ego(x, 4.0),
                true,
            );
            assert_eq!(n.status.phase, NavigationPhase::Blocked);
            for i in 1..=5 {
                assert!(!step(
                    &mut n,
                    None,
                    1.0 + i as f64 * 0.05,
                    ego(x, 0.0),
                    true
                ));
            }
            assert_eq!(n.status.switches, 0);
        }
        let mut c = config();
        c.network
            .edges
            .iter_mut()
            .find(|e| e.id == "detour")
            .unwrap()
            .half_width = 0.5;
        let mut n = Navigator::new(c).unwrap();
        step(
            &mut n,
            Some(&update(1.0, 1, &["main"])),
            1.0,
            ego(10.0, 4.0),
            true,
        );
        assert_eq!(n.status.phase, NavigationPhase::Blocked);
    }
    #[test]
    fn stopped_but_off_corridor_or_misaligned_estimates_cannot_switch() {
        let mut n = Navigator::new(config()).unwrap();
        step(
            &mut n,
            Some(&update(1.0, 1, &["main"])),
            1.0,
            ego(10.0, 4.0),
            true,
        );
        for (y, yaw) in [(5.0, 0.0), (0.0, 0.7)] {
            let e = EgoState {
                pose: Pose {
                    position: Vec2::new(36.0, y),
                    yaw,
                },
                speed: 0.0,
            };
            for i in 1..=3 {
                assert!(!step(&mut n, None, 2.0 + i as f64 * 0.05, e, true));
            }
        }
        assert_eq!(n.status.switches, 0);
    }

    #[test]
    fn speed_policy_changes_atomically_with_the_healthy_stopped_route_handover() {
        let mut c = config();
        for edge in &mut c.network.edges {
            edge.speed_limit_m_s = Some(if edge.id == "detour" { 2. } else { 4. });
        }
        let mut n = Navigator::new(c).unwrap();
        assert_eq!(n.speed_zones().len(), 1);
        step(
            &mut n,
            Some(&update(1., 1, &["main"])),
            1.,
            ego(10., 4.),
            true,
        );
        assert!(
            n.pending
                .as_ref()
                .unwrap()
                .speed_zones
                .iter()
                .any(|z| z.speed_m_s == 2.)
        );
        assert!(n.speed_zones().iter().all(|z| z.speed_m_s == 4.));
        for i in 1..=3 {
            let switched = step(&mut n, None, 2. + i as f64 * 0.05, ego(36., 0.), true);
            assert_eq!(switched, i == 3);
            if i < 3 {
                assert!(n.speed_zones().iter().all(|z| z.speed_m_s == 4.));
            }
        }
        assert_eq!(n.speed_zones().len(), 3);
        assert_eq!(n.speed_zones()[1].start_s_m, 40.);
        assert!(n.speed_zones()[1].end_s_m > 160.);
        assert_eq!(n.speed_zones()[1].speed_m_s, 2.);
    }

    #[test]
    fn no_route_hold_latches_after_measured_standstill_until_the_route_is_accepted() {
        let mut n = Navigator::new(config()).unwrap();
        step(
            &mut n,
            Some(&update(1., 1, &["main", "detour"])),
            1.,
            ego(10., 4.),
            true,
        );
        assert!(!n.holding_brake());
        step(&mut n, None, 1.025, ego(36., 0.1), true);
        assert!(!n.holding_brake());
        step(&mut n, None, 1.05, ego(36., 0.), false);
        assert!(!n.holding_brake());
        step(&mut n, None, 1.1, ego(36., 0.), true);
        assert!(n.holding_brake());
        step(&mut n, None, 1.15, ego(36., 0.1), true);
        assert!(n.holding_brake());
        // A reopening snapshot is still pending until all three fresh stopped
        // samples arrive. Releasing on that snapshot would restart too early.
        assert!(!step(
            &mut n,
            Some(&update(2., 2, &["main"])),
            2.,
            ego(36., 0.),
            true
        ));
        assert!(n.holding_brake());
        assert!(!step(&mut n, None, 2.05, ego(36., 0.), true));
        assert!(n.holding_brake());
        assert!(step(&mut n, None, 2.1, ego(36., 0.), true));
        assert!(!n.holding_brake());
        step(
            &mut n,
            Some(&update(2.2, 3, &["detour"])),
            2.2,
            ego(36., 0.),
            true,
        );
        assert_eq!(n.status.phase, NavigationPhase::Braking);
        assert!(!n.holding_brake());
    }
}
impl NavigationConfig {
    pub fn initial_plan(&self) -> Result<RoutePlan, String> {
        RoadNetwork::new(self.network.clone())?
            .shortest_route_with_speed_zones(&self.start, &self.goal, &self.closed_edges)
            .map(|compiled| compiled.plan)
    }
}

/// Complete external closure snapshot; not simulator obstacle data.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NavigationUpdate {
    pub stamp: f64,
    pub revision: u64,
    pub closed_edges: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum NavigationPhase {
    Following,
    Braking,
    Blocked,
    Fault,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NavigationStatus {
    pub revision: u64,
    pub phase: NavigationPhase,
    pub active_edges: Vec<String>,
    pub pending_edges: Vec<String>,
    pub closed_edges: Vec<String>,
    pub stop_s: Option<f64>,
    pub switches: usize,
}

pub(crate) struct Navigator {
    config: NavigationConfig,
    network: RoadNetwork,
    active: SpeedRoutePlan,
    pending: Option<SpeedRoutePlan>,
    status: NavigationStatus,
    last_stamp: Option<f64>,
    /// Latest malformed newer-update delivery time; recovery needs a later acquisition.
    fault_epoch: Option<f64>,
    /// Last examined motion acquisition, advanced to the control time on unhealthy frames.
    motion_watermark: Option<f64>,
    stopped_samples: usize,
    blocked_hold: bool,
}
impl Navigator {
    pub fn new(config: NavigationConfig) -> Result<Self, String> {
        let network = RoadNetwork::new(config.network.clone())?;
        let active = network.shortest_route_with_speed_zones(
            &config.start,
            &config.goal,
            &config.closed_edges,
        )?;
        let status = NavigationStatus {
            revision: 0,
            phase: NavigationPhase::Following,
            active_edges: active.plan.edge_ids.clone(),
            pending_edges: vec![],
            closed_edges: config.closed_edges.clone(),
            stop_s: None,
            switches: 0,
        };
        Ok(Self {
            config,
            network,
            active,
            pending: None,
            status,
            last_stamp: None,
            fault_epoch: None,
            motion_watermark: None,
            stopped_samples: 0,
            blocked_hold: false,
        })
    }
    pub fn plan(&self) -> &RoutePlan {
        &self.active.plan
    }
    pub fn speed_zones(&self) -> &[rustdriving_core::RouteSpeedZone] {
        &self.active.speed_zones
    }
    pub fn status(&self) -> NavigationStatus {
        self.status.clone()
    }
    pub fn fault(&mut self, now: f64) {
        self.status.phase = NavigationPhase::Fault;
        self.fault_epoch = Some(now);
        self.pending = None;
        self.blocked_hold = false;
        self.status.pending_edges.clear();
        self.invalidate_stop_confirmation(now);
    }
    fn invalidate_stop_confirmation(&mut self, now: f64) {
        self.stopped_samples = 0;
        self.motion_watermark = Some(self.motion_watermark.map_or(now, |stamp| stamp.max(now)));
    }
    /// Returns true only when the active route changes at a healthy stopped state.
    /// `healthy_motion` is accepted, unexpired odometry after the pipeline's sensor checks.
    pub fn step(
        &mut self,
        update: Option<&NavigationUpdate>,
        now: f64,
        ego: EgoState,
        healthy_motion: Option<Odometry>,
        radius: f64,
        deceleration: f64,
    ) -> bool {
        let new_motion = healthy_motion.filter(|motion| {
            self.motion_watermark
                .is_none_or(|stamp| motion.stamp > stamp)
        });
        let observed_stamp = healthy_motion.map_or(now, |motion| motion.stamp);
        self.motion_watermark = Some(
            self.motion_watermark
                .map_or(observed_stamp, |stamp| stamp.max(observed_stamp)),
        );
        if let Some(update) = update {
            if update.revision > self.status.revision && !update.stamp.is_finite() {
                self.fault(now);
                return false;
            }
            // Repeated or reordered snapshots never roll back a valid decision.
            if update.revision > self.status.revision
                && self.last_stamp.is_none_or(|s| update.stamp > s)
            {
                let unknown = update
                    .closed_edges
                    .iter()
                    .any(|id| !self.config.network.edges.iter().any(|e| &e.id == id));
                if !update.stamp.is_finite()
                    || update.stamp < 0.0
                    || update.stamp > now + 1e-9
                    || now - update.stamp > 1.0
                    || unknown
                {
                    self.fault(now);
                } else if self.fault_epoch.is_none_or(|fault| update.stamp > fault) {
                    self.status.revision = update.revision;
                    self.status.closed_edges = update.closed_edges.clone();
                    self.last_stamp = Some(update.stamp);
                    self.fault_epoch = None;
                    self.stopped_samples = 0;
                    self.pending = None;
                    self.status.pending_edges.clear();
                    match self.network.shortest_route_with_speed_zones(
                        &self.config.start,
                        &self.config.goal,
                        &update.closed_edges,
                    ) {
                        Ok(plan) if plan.plan.edge_ids == self.active.plan.edge_ids => {
                            self.status.phase = NavigationPhase::Following;
                            self.status.stop_s = None;
                        }
                        candidate => {
                            let mut boundary = self.active.plan.route.length();
                            let mut length = 0.0;
                            let mut prefix = 0.0;
                            for (i, id) in self.active.plan.edge_ids.iter().enumerate() {
                                if update.closed_edges.contains(id) {
                                    boundary = boundary.min(length);
                                }
                                let edge = self
                                    .config
                                    .network
                                    .edges
                                    .iter()
                                    .find(|e| &e.id == id)
                                    .unwrap();
                                let edge_length: f64 =
                                    edge.points.windows(2).map(|p| p[0].distance(p[1])).sum();
                                if candidate
                                    .as_ref()
                                    .is_ok_and(|p| p.plan.edge_ids.get(i) == Some(id))
                                    && (prefix - length).abs() < 1e-6
                                {
                                    prefix += edge_length;
                                }
                                length += edge_length;
                            }
                            if candidate.is_ok() {
                                boundary = boundary.min(prefix);
                            }
                            let stop = (boundary - radius - 2.0).max(0.0);
                            self.status.stop_s = Some(stop);
                            let progress = self.active.plan.route.project(ego.pose.position).0;
                            // Do not redirect onto a branch already passed or an unreachable stopping point.
                            let reachable = stop - progress
                                >= ego.speed.max(0.0).powi(2) / (1.6 * deceleration) + 1.0
                                || (ego.speed.abs() <= 0.05 && progress <= stop + 0.5);
                            if let Ok(plan) = candidate
                                && reachable
                                && plan.plan.route.half_width > radius
                                && plan.plan.route.points.len() <= 5000
                                && plan.plan.route.length() <= 1500.0
                            {
                                self.status.phase = NavigationPhase::Braking;
                                self.status.pending_edges = plan.plan.edge_ids.clone();
                                self.pending = Some(plan);
                            } else {
                                self.status.phase = NavigationPhase::Blocked;
                            }
                        }
                    }
                }
            }
        }
        if self.status.phase == NavigationPhase::Fault {
            self.invalidate_stop_confirmation(now);
            return false;
        }
        if self.status.phase == NavigationPhase::Following {
            self.blocked_hold = false;
        }
        // Preserve the approach and latch only after a fresh healthy measured
        // stop. Low-speed feedback noise must not restart an unreachable route.
        if self.status.phase == NavigationPhase::Blocked
            && new_motion.is_some_and(|motion| motion.speed.abs() <= 0.05)
            && ego.speed.abs() <= 0.05
            && self.status.stop_s.is_some_and(|stop| {
                stop - self.active.plan.route.project(ego.pose.position).0 <= 3.5
            })
        {
            self.blocked_hold = true;
        }
        if self.status.phase == NavigationPhase::Braking
            && healthy_motion.is_some_and(|motion| motion.speed.abs() <= 0.05)
            && ego.speed.abs() <= 0.05
        {
            let (s, lateral) = self.active.plan.route.project(ego.pose.position);
            let stop = self.status.stop_s.unwrap();
            let yaw = self.active.plan.route.sample(s, 0.0).1;
            if s <= stop + 0.5
                && lateral.abs() + radius + 0.2 < self.active.plan.route.half_width
                && wrap_angle(ego.pose.yaw - yaw).abs() < 0.35
            {
                // Cached motion can maintain a healthy estimate, but is not another sample.
                if new_motion.is_none() {
                    return false;
                }
                self.stopped_samples += 1;
                if self.stopped_samples >= 3 {
                    self.active = self.pending.take().unwrap();
                    self.status.active_edges = self.active.plan.edge_ids.clone();
                    self.status.pending_edges.clear();
                    self.status.stop_s = None;
                    self.status.switches += 1;
                    self.status.phase = NavigationPhase::Following;
                    self.blocked_hold = false;
                    self.stopped_samples = 0;
                    return true;
                }
            } else {
                self.stopped_samples = 0;
            }
        } else {
            self.stopped_samples = 0;
        }
        false
    }
    pub fn planning_route(&self) -> Option<Route> {
        if self.status.phase == NavigationPhase::Following {
            return None;
        }
        let stop = self.status.stop_s?;
        // The local planner stops one meter before its endpoint.
        let end = (stop + 1.0).min(self.active.plan.route.length());
        let mut points: Vec<_> = self
            .active
            .plan
            .route
            .points
            .iter()
            .zip(&self.active.plan.route.lengths)
            .filter(|(_, s)| **s < end - 1e-6)
            .map(|(p, _)| *p)
            .collect();
        points.push(self.active.plan.route.sample(end, 0.0).0);
        Route::new(points, self.active.plan.route.half_width).ok()
    }
    pub fn holding_brake(&self) -> bool {
        matches!(
            self.status.phase,
            NavigationPhase::Blocked | NavigationPhase::Braking
        ) && self.blocked_hold
    }
}
