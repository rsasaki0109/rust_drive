use super::*;
use rustdriving_core::{
    CandidateDisposition, PlanningCandidateDiagnostic, PlanningDiagnostics,
    PlanningForecastConflict, Pose, Track, Vec2,
};

fn config() -> NavigationConfig {
    let value: serde_json::Value =
        serde_json::from_str(include_str!("../../../../scenarios/route-direct.json")).unwrap();
    let mut config: NavigationConfig = serde_json::from_value(value["navigation"].clone()).unwrap();
    config.hold_for_handover = true;
    config.avoid_observed_obstructions = true;
    config
}
fn ego(x: f64, speed: f64) -> EgoState {
    EgoState {
        pose: Pose {
            position: Vec2::new(x, 0.),
            yaw: 0.,
        },
        speed,
    }
}
fn track(stamp: f64) -> Track {
    Track {
        id: 7,
        position: Vec2::new(65., 0.),
        velocity: Vec2::default(),
        radius: 1.,
        last_seen: stamp,
    }
}
fn decisions(stamp: f64) -> PlanningDiagnostics {
    PlanningDiagnostics {
        scene_acquisition_s: Some((stamp - 0.05).max(0.)),
        candidates: [0., 3.5, -3.5]
            .into_iter()
            .enumerate()
            .map(|(i, target)| PlanningCandidateDiagnostic {
                index: i as u8,
                lateral_target_m: target,
                disposition: CandidateDisposition::CheckedStop,
                nominal_forecast_conflict: Some(PlanningForecastConflict {
                    prediction_id: 7,
                    relative_time_s: 3.,
                    forecast_radius_m: 3.,
                    forecast_horizon_s: 8.,
                    minimum_clearance_floor_m: 0.,
                }),
                profile: None,
            })
            .collect(),
        ..PlanningDiagnostics::default()
    }
}
fn observe(
    n: &mut Navigator,
    now: f64,
    acquisition: f64,
    e: EgoState,
    t: Track,
    d: &PlanningDiagnostics,
    healthy: bool,
) {
    n.observe_obstructions(ObservedObstructionInput {
        now,
        acquisition: Some(acquisition),
        healthy,
        ego: e,
        tracks: &[t],
        decisions: Some(d),
        update: None,
        vehicle_radius: 1.25,
        deceleration: 2.5,
    });
}
fn mature(n: &mut Navigator, e: EgoState) {
    for stamp in [0., 0.1, 0.2] {
        observe(n, stamp, stamp, e, track(stamp), &decisions(stamp), true);
    }
}
fn wheel(stamp: f64) -> Option<Odometry> {
    Some(Odometry {
        stamp,
        speed: 0.,
        yaw_rate: 0.,
    })
}

#[test]
fn acquired_detour_keeps_external_knowledge_separate_and_requires_stopped_handover() {
    let mut n = Navigator::new(config()).unwrap();
    mature(&mut n, ego(20., 4.));
    let status = n.status();
    assert_eq!(status.phase, NavigationPhase::Braking);
    assert_eq!(status.stop_s, Some(36.75));
    assert_eq!(status.revision, 0);
    assert!(status.closed_edges.is_empty());
    let observed = status.observed_obstructions.unwrap();
    assert_eq!(
        observed.last_decision,
        Some(ObservedAvoidanceDecision::DetourRequested)
    );
    assert_eq!(observed.retained_edges.len(), 1);
    assert_eq!(observed.retained_edges[0].evidence.edge, "main");
    assert_eq!(observed.retained_edges[0].evidence.acquisition_count, 3);
    assert!(!n.step(
        None,
        0.2,
        ego(20., 4.),
        Some(Odometry {
            stamp: 0.2,
            speed: 4.,
            yaw_rate: 0.
        }),
        1.25,
        2.5
    ));
    for (i, stamp) in [0.3, 0.35, 0.4].into_iter().enumerate() {
        assert_eq!(
            n.step(None, stamp, ego(20., 0.), wheel(stamp), 1.25, 2.5),
            i == 2
        );
        assert_eq!(n.holding_brake(), i < 2);
    }
    assert_eq!(n.status().switches, 1);
    assert!(n.plan().edge_ids.contains(&"detour".into()));
    assert!(!n.plan().edge_ids.contains(&"main".into()));
}

#[test]
fn no_alternative_or_unreachable_divergence_cannot_change_a_route_or_create_a_navigation_fault() {
    for no_route in [false, true] {
        let mut c = config();
        if no_route {
            c.network.edges.retain(|e| e.id != "detour");
        }
        let mut n = Navigator::new(c).unwrap();
        let before = serde_json::to_value(n.plan()).unwrap();
        mature(&mut n, ego(if no_route { 20. } else { 50. }, 0.));
        assert_eq!(n.status().phase, NavigationPhase::Following);
        assert_eq!(n.status().stop_s, None);
        assert!(n.pending.is_none());
        assert_eq!(serde_json::to_value(n.plan()).unwrap(), before);
        let observed = n.status().observed_obstructions.unwrap();
        assert!(observed.retained_edges.is_empty());
        assert_eq!(
            observed.last_decision,
            Some(if no_route {
                ObservedAvoidanceDecision::NoAlternative
            } else {
                ObservedAvoidanceDecision::UnreachableDivergence
            })
        );
    }
}

#[test]
fn cached_scenes_cannot_vote_and_fault_recovery_requires_new_support() {
    let mut n = Navigator::new(config()).unwrap();
    for now in [0., 0.05, 0.1] {
        observe(
            &mut n,
            now,
            0.,
            ego(20., 4.),
            track(0.),
            &decisions(0.),
            true,
        );
    }
    assert_eq!(
        n.status()
            .observed_obstructions
            .unwrap()
            .candidate
            .unwrap()
            .acquisition_count,
        1
    );
    observe(
        &mut n,
        0.2,
        0.2,
        ego(20., 4.),
        track(0.2),
        &decisions(0.2),
        false,
    );
    observe(
        &mut n,
        0.25,
        0.2,
        ego(20., 4.),
        track(0.2),
        &decisions(0.2),
        true,
    );
    assert!(
        n.status()
            .observed_obstructions
            .unwrap()
            .candidate
            .is_none()
    );
    for stamp in [0.3, 0.4, 0.5] {
        observe(
            &mut n,
            stamp,
            stamp,
            ego(20., 4.),
            track(stamp),
            &decisions(stamp),
            true,
        );
    }
    assert_eq!(n.status().phase, NavigationPhase::Braking);
}

#[test]
fn motion_an_available_path_or_different_blockers_prevent_a_detour() {
    for variant in 0..4 {
        let mut n = Navigator::new(config()).unwrap();
        for stamp in [0., 0.1, 0.2, 0.3] {
            let mut t = track(stamp);
            let mut d = decisions(stamp);
            match variant {
                0 => t.velocity.x = 0.21,
                1 => t.position.x += 3. * stamp,
                2 => {
                    d.candidates[1].disposition = CandidateDisposition::CompletePath;
                    d.candidates[1].nominal_forecast_conflict = None;
                }
                _ => {
                    d.candidates[1]
                        .nominal_forecast_conflict
                        .as_mut()
                        .unwrap()
                        .prediction_id = 8
                }
            }
            observe(&mut n, stamp, stamp, ego(20., 4.), t, &d, true);
        }
        assert_eq!(n.status().phase, NavigationPhase::Following);
        assert!(
            n.status()
                .observed_obstructions
                .unwrap()
                .retained_edges
                .is_empty()
        );
    }
}

#[test]
fn external_revisions_do_not_erase_observed_preferences_or_own_their_generation() {
    let mut n = Navigator::new(config()).unwrap();
    let update = NavigationUpdate {
        stamp: 0.,
        revision: 5,
        closed_edges: vec!["south-branch".into()],
    };
    n.step(
        Some(&update),
        0.,
        ego(20., 4.),
        Some(Odometry {
            stamp: 0.,
            speed: 4.,
            yaw_rate: 0.,
        }),
        1.25,
        2.5,
    );
    mature(&mut n, ego(20., 4.));
    assert_eq!(n.status().closed_edges, ["south-branch"]);
    assert_eq!(n.status().revision, 5);
    let cancel = NavigationUpdate {
        stamp: 0.3,
        revision: 6,
        closed_edges: vec![],
    };
    n.step(
        Some(&cancel),
        0.3,
        ego(36.6, 0.2),
        Some(Odometry {
            stamp: 0.3,
            speed: 4.,
            yaw_rate: 0.,
        }),
        1.25,
        2.5,
    );
    assert_eq!(n.status().phase, NavigationPhase::Braking);
    assert_eq!(n.status().revision, 6);
    assert!(n.status().closed_edges.is_empty());
    assert_eq!(
        n.status()
            .observed_obstructions
            .unwrap()
            .retained_edges
            .len(),
        1
    );
    assert!(n.status().pending_edges.contains(&"detour".into()));
    let invalid = NavigationUpdate {
        stamp: 0.4,
        revision: 7,
        closed_edges: vec!["unknown".into()],
    };
    n.step(Some(&invalid), 0.4, ego(20., 0.), wheel(0.4), 1.25, 2.5);
    observe(
        &mut n,
        0.5,
        0.5,
        ego(20., 0.),
        track(0.5),
        &decisions(0.5),
        true,
    );
    assert_eq!(n.status().phase, NavigationPhase::Fault);
    assert_eq!(
        n.status()
            .observed_obstructions
            .unwrap()
            .retained_edges
            .len(),
        1
    );
}

#[test]
fn off_corridor_or_behind_measurements_cannot_exclude_a_mapped_edge() {
    for off_corridor in [false, true] {
        let mut n = Navigator::new(config()).unwrap();
        for stamp in [0., 0.1, 0.2] {
            let mut t = track(stamp);
            if off_corridor {
                t.position.y = 5.;
            } else {
                t.position.x = 15.;
            }
            observe(
                &mut n,
                stamp,
                stamp,
                ego(20., 4.),
                t,
                &decisions(stamp),
                true,
            );
        }
        assert_eq!(n.status().phase, NavigationPhase::Following);
        assert!(
            n.status()
                .observed_obstructions
                .unwrap()
                .candidate
                .is_none()
        );
    }
}

#[test]
fn partial_route_origins_use_traversed_lengths_and_disabled_status_stays_absent() {
    let mut c = config();
    c.start = serde_json::from_value(serde_json::json!({"edge":"approach","s_m":12.5})).unwrap();
    let mut n = Navigator::new(c).unwrap();
    mature(&mut n, ego(20., 4.));
    assert_eq!(n.status().stop_s, Some(24.25));
    let mut c = config();
    c.avoid_observed_obstructions = false;
    let mut n = Navigator::new(c.clone()).unwrap();
    mature(&mut n, ego(20., 4.));
    assert_eq!(n.status().phase, NavigationPhase::Following);
    assert!(
        !serde_json::to_string(&c)
            .unwrap()
            .contains("avoid_observed_obstructions")
    );
    assert!(
        !serde_json::to_string(&n.status())
            .unwrap()
            .contains("observed_obstructions")
    );
}
