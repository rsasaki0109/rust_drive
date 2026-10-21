//! Known map updates and a conservative stop-before-divergence route handover.
use rustdriving_core::{EgoState, Odometry, Route, wrap_angle};
use rustdriving_routing::{RoadNetwork, RoadNetworkSpec, RouteEndpoint, RoutePlan, SpeedRoutePlan};
use serde::{Deserialize, Serialize};
mod observed;
#[cfg(test)]
mod observed_tests;
pub(crate) use observed::ObservedObstructionInput;
pub use observed::{
    ObservedAvoidanceDecision, ObservedEdgeAvoidance, ObservedEdgeEvidence,
    ObservedObstructionStatus,
};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NavigationConfig {
    pub network: RoadNetworkSpec,
    pub start: RouteEndpoint,
    pub goal: RouteEndpoint,
    #[serde(default)]
    pub closed_edges: Vec<String>,
    /// Retain braking once a healthy stopped handover confirmation begins.
    #[serde(default, skip_serializing_if = "is_false")]
    pub hold_for_handover: bool,
    /// Retain bounded mission-local edge preferences from acquired obstructions.
    #[serde(default, skip_serializing_if = "is_false")]
    pub avoid_observed_obstructions: bool,
}

fn is_false(value: &bool) -> bool {
    !*value
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
    fn held_config() -> NavigationConfig {
        NavigationConfig {
            hold_for_handover: true,
            ..config()
        }
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
    fn stopped_handover_holds_braking_without_counting_cached_motion() {
        let mut n = Navigator::new(held_config()).unwrap();
        assert!(!step(
            &mut n,
            Some(&update(0., 1, &["main"])),
            0.,
            ego(10., 0.),
            true
        ));
        assert!(n.holding_brake());
        assert!(n.status.handover_hold);
        assert!(!n.step(
            None,
            0.025,
            ego(10., 0.),
            Some(Odometry {
                stamp: 0.,
                speed: 0.,
                yaw_rate: 0.
            }),
            1.25,
            2.5
        ));
        assert_eq!(n.stopped_samples, 1);
        assert!(n.holding_brake());
        assert!(!step(&mut n, None, 0.05, ego(10., 0.), true));
        assert!(n.holding_brake());
        assert!(step(&mut n, None, 0.1, ego(10., 0.), true));
        assert!(!n.holding_brake());
        assert!(!n.status.handover_hold);
        assert_eq!(n.status.switches, 1);
    }
    #[test]
    fn interrupted_confirmation_retains_the_hold_but_discards_votes() {
        let mut n = Navigator::new(held_config()).unwrap();
        step(
            &mut n,
            Some(&update(0., 1, &["main"])),
            0.,
            ego(10., 0.),
            true,
        );
        for (t, speed, healthy) in [(0.05, 0.1, true), (0.1, 0., false)] {
            assert!(!step(&mut n, None, t, ego(10., speed), healthy));
            assert_eq!(n.stopped_samples, 0);
            assert!(n.holding_brake());
        }
        let misaligned = EgoState {
            pose: rustdriving_core::Pose {
                yaw: 0.5,
                ..ego(10., 0.).pose
            },
            speed: 0.,
        };
        assert!(!step(&mut n, None, 0.15, misaligned, true));
        assert_eq!(n.stopped_samples, 0);
        assert!(n.holding_brake());
        for (i, t) in [0.2, 0.25, 0.3].into_iter().enumerate() {
            assert_eq!(step(&mut n, None, t, ego(10., 0.), true), i == 2);
        }
    }
    #[test]
    fn closure_replacement_keeps_a_hold_and_cancellation_releases_it() {
        let mut n = Navigator::new(held_config()).unwrap();
        step(
            &mut n,
            Some(&update(0., 1, &["main"])),
            0.,
            ego(10., 0.),
            true,
        );
        step(
            &mut n,
            Some(&update(0.05, 2, &["main", "detour"])),
            0.05,
            ego(10., 0.),
            true,
        );
        assert_eq!(n.status.phase, NavigationPhase::Blocked);
        assert!(n.holding_brake());
        step(&mut n, Some(&update(0.1, 3, &[])), 0.1, ego(10., 0.), true);
        assert_eq!(n.status.phase, NavigationPhase::Following);
        assert!(!n.holding_brake());
        assert!(!n.status.handover_hold);
        assert_eq!(n.status.switches, 0);
        step(
            &mut n,
            Some(&update(0.15, 4, &["main"])),
            0.15,
            ego(10., 0.),
            true,
        );
        assert!(n.status.handover_hold);
        step(
            &mut n,
            Some(&update(0.2, 5, &["unknown"])),
            0.2,
            ego(10., 0.),
            true,
        );
        assert_eq!(n.status.phase, NavigationPhase::Fault);
        assert!(n.status.handover_hold);
    }
    #[test]
    fn measured_reset_discards_a_hold_and_requires_new_acquisitions() {
        let mut n = Navigator::new(held_config()).unwrap();
        step(
            &mut n,
            Some(&update(0., 1, &["main"])),
            0.,
            ego(10., 0.),
            true,
        );
        step(&mut n, None, 0.05, ego(10., 0.), true);
        n.relocalize(0.1, ego(10., 0.), 1.25, 2.5);
        assert!(!n.status.handover_hold);
        assert_eq!(n.stopped_samples, 0);
        assert!(!step(&mut n, None, 0.1, ego(10., 0.), true));
        assert!(!n.holding_brake());
        for (i, t) in [0.15, 0.2, 0.25].into_iter().enumerate() {
            assert_eq!(step(&mut n, None, t, ego(10., 0.), true), i == 2);
            assert_eq!(n.holding_brake(), i != 2);
        }
    }
    #[test]
    fn absent_policy_retains_legacy_serialization_and_no_early_brake_hold() {
        let c = config();
        assert!(!c.hold_for_handover);
        assert!(
            serde_json::to_value(&c)
                .unwrap()
                .get("hold_for_handover")
                .is_none()
        );
        let mut n = Navigator::new(c).unwrap();
        step(
            &mut n,
            Some(&update(0., 1, &["main"])),
            0.,
            ego(10., 0.),
            true,
        );
        assert!(!n.holding_brake());
        assert!(
            serde_json::to_value(n.status())
                .unwrap()
                .get("handover_hold")
                .is_none()
        );
    }
    fn partial_config() -> NavigationConfig {
        let mut c = config();
        c.start = rustdriving_routing::RouteEndpoint::Edge(rustdriving_routing::EdgePosition {
            edge: "approach".into(),
            s_m: 12.5,
        });
        c.goal = rustdriving_routing::RouteEndpoint::Edge(rustdriving_routing::EdgePosition {
            edge: "east-exit".into(),
            s_m: 25.25,
        });
        c
    }
    #[test]
    fn partial_origin_rebases_closure_stop_and_handover_geometry() {
        let mut n = Navigator::new(partial_config()).unwrap();
        assert!(!step(
            &mut n,
            Some(&update(0., 1, &["main"])),
            0.,
            ego(12.5, 4.),
            true
        ));
        assert_eq!(n.status.stop_s, Some(24.25));
        assert_eq!(n.status.phase, NavigationPhase::Braking);
        for i in 1..=3 {
            let switched = step(&mut n, None, i as f64 * 0.05, ego(30., 0.), true);
            assert_eq!(switched, i == 3);
        }
        assert_eq!(n.plan().edge_ids, ["approach", "detour", "east-exit"]);
        assert_eq!(n.plan().route.points[0], Vec2::new(12.5, 0.));
        assert_eq!(
            *n.plan().route.points.last().unwrap(),
            Vec2::new(185.25, 0.)
        );
        assert_eq!(n.status.switches, 1);
    }
    #[test]
    fn partial_destination_closure_holds_before_its_actual_entry() {
        let mut n = Navigator::new(partial_config()).unwrap();
        step(
            &mut n,
            Some(&update(0., 1, &["east-exit"])),
            0.,
            ego(12.5, 2.),
            true,
        );
        assert_eq!(n.status.phase, NavigationPhase::Blocked);
        assert_eq!(n.status.stop_s, Some(144.25));
        assert!(n.status.pending_edges.is_empty());
        assert_eq!(n.plan().edge_ids, ["approach", "main", "east-exit"]);
    }
    #[test]
    fn endpoint_json_retains_node_strings_and_rejects_ambiguous_objects() {
        let c = config();
        let value = serde_json::to_value(&c).unwrap();
        assert_eq!(value["start"], "start");
        assert_eq!(value["goal"], "east");
        assert!(
            serde_json::from_str::<RouteEndpoint>(r#"{"edge":"approach","s_m":1,"node":"start"}"#)
                .is_err()
        );
        assert!(serde_json::from_str::<RouteEndpoint>(r#"{"edge":"approach"}"#).is_err());
        let p = serde_json::to_value(c.initial_plan().unwrap()).unwrap();
        assert!(p.get("traversals").is_none());
        let partial = partial_config();
        let copy: NavigationConfig =
            serde_json::from_value(serde_json::to_value(&partial).unwrap()).unwrap();
        assert_eq!(copy.start, partial.start);
        assert_eq!(copy.goal, partial.goal);
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
    #[test]
    fn measured_reset_clears_handover_votes_and_preserves_closure_ownership() {
        let mut n = Navigator::new(partial_config()).unwrap();
        step(
            &mut n,
            Some(&update(0., 7, &["main"])),
            0.,
            ego(20., 0.),
            true,
        );
        step(&mut n, None, 0.05, ego(20., 0.), true);
        assert_eq!(n.stopped_samples, 2);
        n.relocalize(0.1, ego(22., 0.), 1.25, 2.5);
        assert_eq!(n.stopped_samples, 0);
        assert_eq!(n.status.revision, 7);
        assert_eq!(n.status.closed_edges, ["main"]);
        assert!(n.status.localization_reset.as_ref().unwrap().admitted);
        assert!(!n.step(
            None,
            0.15,
            ego(22., 0.),
            Some(Odometry {
                stamp: 0.1,
                speed: 0.,
                yaw_rate: 0.
            }),
            1.25,
            2.5
        ));
        for i in 0..3 {
            assert_eq!(
                step(&mut n, None, 0.2 + i as f64 * 0.05, ego(22., 0.), true),
                i == 2
            );
        }
        assert_eq!(n.status.switches, 1);
        assert_eq!(n.plan().route.points[0], Vec2::new(12.5, 0.));
        assert_eq!(
            *n.plan().route.points.last().unwrap(),
            Vec2::new(185.25, 0.)
        );
        assert!(n.plan().edge_ids.contains(&"detour".into()));
    }
    #[test]
    fn measured_reset_keeps_the_changed_active_plan_and_its_speed_limits() {
        let mut c = config();
        c.network
            .edges
            .iter_mut()
            .find(|e| e.id == "detour")
            .unwrap()
            .speed_limit_m_s = Some(2.);
        let mut n = Navigator::new(c).unwrap();
        step(
            &mut n,
            Some(&update(0., 11, &["main"])),
            0.,
            ego(10., 0.),
            true,
        );
        step(&mut n, None, 0.05, ego(10., 0.), true);
        assert!(step(&mut n, None, 0.1, ego(10., 0.), true));
        let before = serde_json::to_value(n.speed_zones()).unwrap();
        let plan = serde_json::to_value(n.plan()).unwrap();
        n.relocalize(0.2, ego(12., 0.), 1.25, 2.5);
        assert_eq!(serde_json::to_value(n.plan()).unwrap(), plan);
        assert_eq!(serde_json::to_value(n.speed_zones()).unwrap(), before);
        assert!(n.speed_zones().iter().any(|z| z.speed_m_s == 2.));
        assert_eq!(n.status.phase, NavigationPhase::Following);
        assert_eq!(n.status.revision, 11);
        assert_eq!(n.status.switches, 1);
    }
    #[test]
    fn measured_reset_rechecks_a_pending_branch_against_the_new_pose() {
        let mut n = Navigator::new(config()).unwrap();
        step(
            &mut n,
            Some(&update(0., 1, &["main"])),
            0.,
            ego(10., 4.),
            true,
        );
        assert_eq!(n.status.phase, NavigationPhase::Braking);
        n.relocalize(0.1, ego(60., 0.), 1.25, 2.5);
        assert_eq!(n.status.phase, NavigationPhase::Blocked);
        assert!(n.pending.is_none());
        assert!(n.status.pending_edges.is_empty());
        assert_eq!(n.status.closed_edges, ["main"]);
        for i in 2..=6 {
            assert!(!step(&mut n, None, i as f64 * 0.1, ego(60., 0.), true));
        }
        assert_eq!(n.status.switches, 0);
    }
    #[test]
    fn localization_reset_does_not_clear_an_existing_navigation_fault() {
        let mut n = Navigator::new(config()).unwrap();
        step(
            &mut n,
            Some(&update(0.1, 3, &["unknown"])),
            0.1,
            ego(10., 0.),
            true,
        );
        assert_eq!(n.status.phase, NavigationPhase::Fault);
        n.relocalize(0.2, ego(10., 0.), 1.25, 2.5);
        step(&mut n, Some(&update(0.1, 4, &[])), 0.25, ego(10., 0.), true);
        assert_eq!(n.status.phase, NavigationPhase::Fault);
        step(&mut n, Some(&update(0.3, 4, &[])), 0.3, ego(10., 0.), true);
        assert_eq!(n.status.phase, NavigationPhase::Following);
        assert_eq!(n.status.revision, 4);
    }
    #[test]
    fn failed_route_admission_latches_through_updates_until_measured_initialization() {
        for pose in [
            Pose {
                position: Vec2::new(10., 5.),
                yaw: 0.,
            },
            Pose {
                position: Vec2::new(10., 0.),
                yaw: 0.36,
            },
            Pose {
                position: Vec2::new(250., 0.),
                yaw: 0.,
            },
        ] {
            let mut n = Navigator::new(config()).unwrap();
            n.relocalize(0.1, EgoState { pose, speed: 0. }, 1.25, 2.5);
            assert!(!n.status.localization_reset.as_ref().unwrap().admitted);
            for i in 2..=4 {
                let now = i as f64 * 0.1;
                step(&mut n, Some(&update(now, i, &[])), now, ego(10., 0.), true);
                assert_eq!(n.status.phase, NavigationPhase::Fault);
                assert_eq!(n.status.revision, 0);
            }
            n.relocalize(0.5, ego(10., 0.), 1.25, 2.5);
            step(&mut n, Some(&update(0.6, 5, &[])), 0.6, ego(10., 0.), true);
            assert_eq!(n.status.phase, NavigationPhase::Following);
        }
    }
    #[test]
    fn default_navigation_status_omits_localization_reset_evidence() {
        let n = Navigator::new(config()).unwrap();
        assert!(
            serde_json::to_value(n.status())
                .unwrap()
                .get("localization_reset")
                .is_none()
        );
    }
}
impl NavigationConfig {
    pub fn initial_plan(&self) -> Result<RoutePlan, String> {
        RoadNetwork::new(self.network.clone())?
            .shortest_route_between_with_speed_zones(&self.start, &self.goal, &self.closed_edges)
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
pub struct NavigationResetDiagnostics {
    pub stamp: f64,
    pub progress_s_m: f64,
    pub lateral_m: f64,
    pub distance_to_route_m: f64,
    pub yaw_error_rad: f64,
    pub admitted: bool,
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub localization_reset: Option<NavigationResetDiagnostics>,
    #[serde(default, skip_serializing_if = "is_false")]
    pub handover_hold: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub observed_obstructions: Option<ObservedObstructionStatus>,
}

#[derive(Clone)]
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
    observed: Option<observed::ObservedObstructions>,
}
impl Navigator {
    pub fn new(config: NavigationConfig) -> Result<Self, String> {
        let network = RoadNetwork::new(config.network.clone())?;
        let active = network.shortest_route_between_with_speed_zones(
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
            localization_reset: None,
            handover_hold: false,
            observed_obstructions: None,
        };
        let observed = config
            .avoid_observed_obstructions
            .then(observed::ObservedObstructions::default);
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
            observed,
        })
    }
    pub fn plan(&self) -> &RoutePlan {
        &self.active.plan
    }
    pub fn speed_zones(&self) -> &[rustdriving_core::RouteSpeedZone] {
        &self.active.speed_zones
    }
    pub fn status(&self) -> NavigationStatus {
        let mut status = self.status.clone();
        status.observed_obstructions = self.observed.as_ref().map(|o| o.report.clone());
        status
    }
    fn effective_closed_edges(&self) -> Vec<String> {
        let mut closed = self.status.closed_edges.clone();
        if let Some(observed) = &self.observed {
            for record in &observed.report.retained_edges {
                if !closed.contains(&record.evidence.edge) {
                    closed.push(record.evidence.edge.clone());
                }
            }
        }
        closed
    }
    /// Propose a new route preference before the ordinary stopped handover step.
    /// External revisions/closures remain separately owned and cannot be forged here.
    pub fn observe_obstructions(&mut self, input: ObservedObstructionInput<'_>) {
        let ObservedObstructionInput {
            now,
            acquisition,
            healthy,
            ego,
            tracks,
            decisions,
            update,
            vehicle_radius: radius,
            deceleration,
        } = input;
        let Some(observer) = &mut self.observed else {
            return;
        };
        let evidence = observer.observe(observed::Observation {
            now,
            acquisition,
            healthy,
            phase: self.status.phase,
            newer_external_update: update.is_some_and(|u| u.revision > self.status.revision),
            ego,
            vehicle_radius: radius,
            tracks,
            decisions,
            active: &self.active.plan,
            network: &self.config.network,
        });
        let Some(evidence) = evidence else {
            return;
        };
        if observer.report.retained_edges.len() >= observed::MAX_RETAINED_EDGES {
            observer.decision(now, ObservedAvoidanceDecision::CapacityReached);
            return;
        }
        // Preview transactionally. Rejected/late preferences leave the active
        // plan, external knowledge, handover votes and stop ownership untouched.
        let mut proposed = self.clone();
        proposed
            .observed
            .as_mut()
            .unwrap()
            .report
            .retained_edges
            .push(ObservedEdgeAvoidance {
                evidence,
                requested_at_control_s: now,
            });
        let excluded = proposed.effective_closed_edges();
        if proposed
            .network
            .shortest_route_between_with_speed_zones(
                &proposed.config.start,
                &proposed.config.goal,
                &excluded,
            )
            .is_err()
        {
            self.observed
                .as_mut()
                .unwrap()
                .decision(now, ObservedAvoidanceDecision::NoAlternative);
            return;
        }
        proposed.replan(ego, radius, deceleration);
        if proposed.status.phase != NavigationPhase::Braking || proposed.pending.is_none() {
            self.observed
                .as_mut()
                .unwrap()
                .decision(now, ObservedAvoidanceDecision::UnreachableDivergence);
            return;
        }
        proposed
            .observed
            .as_mut()
            .unwrap()
            .decision(now, ObservedAvoidanceDecision::DetourRequested);
        *self = proposed;
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
    /// Keep accepted mission knowledge, but discard pose-dependent handover votes.
    /// Route admission is latched: an unrelated closure snapshot cannot repair
    /// a measured pose outside the active corridor.
    pub fn relocalize(&mut self, now: f64, ego: EgoState, radius: f64, deceleration: f64) {
        if let Some(observer) = &mut self.observed {
            observer.reset_probe(now);
        }
        let route = &self.active.plan.route;
        let (progress, lateral) = route.project(ego.pose.position);
        let (center, heading) = route.sample(progress, 0.);
        let distance = center.distance(ego.pose.position);
        let yaw_error = wrap_angle(ego.pose.yaw - heading).abs();
        let admitted = ego.speed.abs() <= 0.05
            && distance + radius + 0.2 < route.half_width
            && yaw_error < 0.35;
        self.status.localization_reset = Some(NavigationResetDiagnostics {
            stamp: now,
            progress_s_m: progress,
            lateral_m: lateral,
            distance_to_route_m: distance,
            yaw_error_rad: yaw_error,
            admitted,
        });
        self.invalidate_stop_confirmation(now);
        self.blocked_hold = false;
        self.status.handover_hold = false;
        if !admitted {
            self.fault(now);
        } else if matches!(
            self.status.phase,
            NavigationPhase::Braking | NavigationPhase::Blocked
        ) {
            self.pending = None;
            self.status.pending_edges.clear();
            self.replan(ego, radius, deceleration);
        }
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
        if self
            .status
            .localization_reset
            .as_ref()
            .is_some_and(|reset| !reset.admitted)
        {
            self.invalidate_stop_confirmation(now);
            return false;
        }
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
                    // A newly reopened unrelated edge cannot withdraw an
                    // already reachable observed detour while it is braking.
                    // Still reset confirmation votes and recheck map topology.
                    let keep_observed_pending = self.observed.is_some()
                        && self.status.phase == NavigationPhase::Braking
                        && self.pending.as_ref().is_some_and(|pending| {
                            self.network
                                .shortest_route_between_with_speed_zones(
                                    &self.config.start,
                                    &self.config.goal,
                                    &self.effective_closed_edges(),
                                )
                                .is_ok_and(|plan| plan.plan.edge_ids == pending.plan.edge_ids)
                        });
                    if !keep_observed_pending {
                        self.pending = None;
                        self.status.pending_edges.clear();
                        self.replan(ego, radius, deceleration);
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
            self.status.handover_hold = false;
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
                // Hold through confirmation interruptions. Only a successful
                // switch or an accepted cancellation may authorize acceleration.
                self.status.handover_hold |= self.config.hold_for_handover;
                self.stopped_samples += 1;
                if self.stopped_samples >= 3 {
                    self.active = self.pending.take().unwrap();
                    self.status.active_edges = self.active.plan.edge_ids.clone();
                    self.status.pending_edges.clear();
                    self.status.stop_s = None;
                    self.status.switches += 1;
                    self.status.phase = NavigationPhase::Following;
                    self.blocked_hold = false;
                    self.status.handover_hold = false;
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
    fn replan(&mut self, ego: EgoState, radius: f64, deceleration: f64) {
        let closed = self.effective_closed_edges();
        match self.network.shortest_route_between_with_speed_zones(
            &self.config.start,
            &self.config.goal,
            &closed,
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
                    if closed.contains(id) {
                        boundary = boundary.min(length);
                    }
                    let edge = self
                        .config
                        .network
                        .edges
                        .iter()
                        .find(|e| &e.id == id)
                        .unwrap();
                    let edge_length: f64 = self.active.plan.traversals.get(i).map_or_else(
                        || edge.points.windows(2).map(|p| p[0].distance(p[1])).sum(),
                        |span| span.end_s_m - span.start_s_m,
                    );
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
        ) && (self.blocked_hold || self.status.handover_hold)
    }
}
