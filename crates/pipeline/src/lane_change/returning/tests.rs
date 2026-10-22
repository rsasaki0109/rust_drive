use super::*;
use crate::lane_change::LaneChangeGate;
use rustdriving_core::Pose;

fn gate() -> LaneChangeGate {
    LaneChangeGate::new(
        LaneChangeConfig {
            entry_s_m: 25.0,
            exit_s_m: 56.0,
            target_start: Vec2::new(0.0, 3.5),
            target_end: Vec2::new(80.0, 3.5),
            target_width_m: 3.5,
            return_on_conflict: Some(LaneChangeReturnConfig {
                source_start: Vec2::default(),
                source_end: Vec2::new(80.0, 0.0),
                source_width_m: 3.5,
                max_return_distance_m: 30.0,
                speed_limit_m_s: None,
                resume_after_return: None,
            }),
        },
        VehicleConfig::default(),
        2.0,
    )
}
fn route() -> Route {
    Route::new(
        vec![
            Vec2::default(),
            Vec2::new(25.0, 0.0),
            Vec2::new(55.0, 3.5),
            Vec2::new(160.0, 3.5),
        ],
        1.75,
    )
    .unwrap()
}
fn ego() -> EgoState {
    EgoState {
        pose: Pose {
            position: Vec2::new(28.0, 0.1),
            yaw: 0.08,
        },
        speed: 2.0,
    }
}
fn obstacle(x: f64, y: f64, id: u64) -> Prediction {
    Prediction {
        additional_radius_m: vec![],
        id,
        positions: vec![Vec2::new(x, y); 41],
        radius: 1.0,
        dt: 0.2,
    }
}
fn observation<'a>(
    route: &'a Route,
    predictions: &'a [Prediction],
    now: f64,
) -> ReturnObservation<'a> {
    ReturnObservation {
        speed_zones: &[],
        route,
        ego: ego(),
        predictions,
        now,
        scan: Some(now),
        healthy: true,
        motion: Some(Odometry {
            stamp: now,
            speed: 2.0,
            yaw_rate: 0.02,
        }),
        limits: MotionLimits {
            max_acceleration_m_s2: 1.0,
            max_deceleration_m_s2: 2.5,
            max_lateral_acceleration_m_s2: 1.0,
        },
    }
}
fn mature(g: &mut LaneChangeGate, r: &Route) {
    for i in 0..=10 {
        assert!(!g.observe_return(observation(r, &[], i as f64 * 0.1)));
    }
    g.state.phase = LaneChangePhase::Changing;
}

#[test]
fn early_return_requires_source_evidence_and_a_track_free_preview_then_holds_until_measured_stop() {
    let r = route();
    let mut g = gate();
    mature(&mut g, &r);
    let traffic = [obstacle(52.0, 3.5, 1)];
    assert!(g.observe_return(observation(&r, &traffic, 1.1)));
    assert_eq!(g.status().phase, LaneChangePhase::Aborting);
    let planned = g.planning_route(&r).unwrap();
    assert_eq!(planned.points[0], ego().pose.position);
    let stop_x = planned.points.last().unwrap().x - 1.0;
    assert!(planned.length() < 30.0);
    assert!(
        g.status()
            .return_state
            .unwrap()
            .curvature_bound_m_inv
            .is_some()
    );
    // A cached zero-speed observation earns one measured stop only.
    for i in 0..=2 {
        let now = 1.2 + i as f64 * 0.02;
        let mut o = observation(&r, &[], now);
        o.ego = EgoState {
            pose: Pose {
                position: Vec2::new(stop_x, 0.0),
                yaw: 0.0,
            },
            speed: 0.0,
        };
        o.motion = Some(Odometry {
            stamp: 1.2,
            speed: 0.0,
            yaw_rate: 0.0,
        });
        assert!(!g.observe_return(o));
        assert_eq!(g.status().phase, LaneChangePhase::Aborting);
    }
    for (index, now) in [1.25, 1.3].into_iter().enumerate() {
        let mut o = observation(&r, &[], now);
        o.ego = EgoState {
            pose: Pose {
                position: Vec2::new(stop_x, 0.0),
                yaw: 0.0,
            },
            speed: 0.0,
        };
        o.motion = Some(Odometry {
            stamp: now,
            speed: 0.0,
            yaw_rate: 0.0,
        });
        assert!(!g.observe_return(o));
        assert_eq!(
            g.status().phase,
            if index == 0 {
                LaneChangePhase::Aborting
            } else {
                LaneChangePhase::Aborted
            }
        );
    }
    assert!(g.holding_brake(
        &r,
        EgoState {
            pose: Pose {
                position: Vec2::new(stop_x, 0.0),
                yaw: 0.0
            },
            speed: 0.0
        }
    ));
}

#[test]
fn source_conflict_late_pose_invalid_motion_or_infeasible_curvature_cannot_start_return() {
    let r = route();
    for case in 0..5 {
        let mut g = gate();
        mature(&mut g, &r);
        let mut traffic = vec![obstacle(52.0, 3.5, 1)];
        if case == 0 {
            traffic.push(obstacle(40.0, 0.0, 2));
        }
        let mut o = observation(&r, &traffic, 1.1);
        match case {
            1 => o.ego.pose.position.y = 0.6,
            2 => o.motion = None,
            3 => o.motion.as_mut().unwrap().yaw_rate = 1.0,
            4 => o.healthy = false,
            _ => (),
        }
        assert!(!g.observe_return(o), "case {case}");
        assert_eq!(g.status().phase, LaneChangePhase::Changing);
        assert!(g.planning_route(&r).is_none());
        assert_eq!(
            g.status().return_state.unwrap().decision,
            match case {
                0 => ReturnDecision::SourceBlocked,
                1 => ReturnDecision::OutsideSource,
                3 => ReturnDecision::InfeasibleGeometry,
                _ => ReturnDecision::InvalidEvidence,
            }
        );
    }
}

#[test]
fn fault_epoch_delayed_scans_and_missing_acquisitions_restart_source_confirmation() {
    let r = route();
    let mut g = gate();
    mature(&mut g, &r);
    let traffic = [obstacle(52.0, 3.5, 1)];
    let mut fault = observation(&r, &traffic, 1.1);
    fault.healthy = false;
    fault.scan = None;
    assert!(!g.observe_return(fault));
    let mut delayed = observation(&r, &traffic, 1.2);
    delayed.scan = Some(1.05);
    assert!(!g.observe_return(delayed));
    for i in 0..10 {
        let now = 1.3 + i as f64 * 0.1;
        assert!(!g.observe_return(observation(&r, &traffic, now)));
        assert_eq!(g.status().phase, LaneChangePhase::Changing);
    }
    assert!(g.observe_return(observation(&r, &traffic, 2.3)));
}

#[test]
fn long_return_clearance_regression_selects_a_shorter_steerable_path_without_relaxing_the_margin() {
    let r = route();
    let mut g = gate();
    mature(&mut g, &r);
    let mut traffic = [obstacle(52.0, 3.5, 1)];
    traffic[0].radius = 1.22;
    let mut o = observation(&r, &traffic, 1.1);
    o.ego = EgoState {
        pose: Pose {
            position: Vec2::new(31.60237278618318, 0.3449022519984197),
            yaw: 0.11033525665344968,
        },
        speed: 2.0,
    };
    o.motion = Some(Odometry {
        stamp: 1.1,
        speed: 2.0,
        yaw_rate: 2.0 * 0.017511504868354028,
    });
    let monitor = g.return_monitor.as_ref().unwrap();
    assert!(matches!(
        monitor.candidate(&o, o.motion.unwrap(), 30.0),
        Err(ReturnDecision::InfeasibleTrajectory)
    ));
    assert!(g.observe_return(o));
    let status = g.status().return_state.unwrap();
    assert!(status.request.unwrap().distance_m < 30.0);
    assert!(status.observed_clearance_m.unwrap() >= RETURN_CLEARANCE_M + RETURN_TRACKING_RESERVE_M);
}

#[test]
fn preview_clearance_covers_crossings_between_both_sets_of_sampled_endpoints() {
    let p = |x, y| Vec2::new(x, y);
    assert_eq!(
        segment_distance(p(0., 0.), p(10., 0.), p(5., -10.), p(5., 10.)),
        0.0
    );
    assert_eq!(
        segment_distance(p(0., 0.), p(10., 0.), p(2., 0.), p(5., 0.)),
        0.0
    );
    assert_eq!(
        segment_distance(p(0., 0.), p(10., 0.), p(0., 2.), p(10., 2.)),
        2.0
    );
    assert_eq!(
        segment_distance(p(0., 0.), p(0., 0.), p(3., 4.), p(3., 4.)),
        5.0
    );
    let path = [
        rustdriving_core::TrajectoryPoint {
            position: p(0., 0.),
            speed: 1.,
            time: 0.,
        },
        rustdriving_core::TrajectoryPoint {
            position: p(10., 0.),
            speed: 1.,
            time: 10.,
        },
    ];
    let crossing = Prediction {
        additional_radius_m: vec![],
        id: 1,
        positions: vec![p(5., -10.), p(5., 10.)],
        radius: 1.,
        dt: 8.,
    };
    assert_eq!(observed_clearance(&path, &[crossing], 1.25), -2.25);
}

#[test]
fn neighboring_track_outside_the_source_band_does_not_erase_source_evidence() {
    let r = route();
    let mut g = gate();
    mature(&mut g, &r);
    let mut p = obstacle(52.00630378022505, 3.3303360098384203, 1);
    p.radius = 1.2203906507411388;
    assert!(g.observe_return(observation(&r, &[p], 1.1)));
    let s = g.status().return_state.unwrap();
    assert!(s.source_blocking_tracks.is_empty());
    assert_eq!(s.source_clear_since, Some(0.0));
    assert!(s.observed_clearance_m.unwrap() >= 1.3);
}

#[test]
fn return_preview_subtracts_the_forecast_occupancy_envelope() {
    let path = [0., 10.].map(|x| rustdriving_core::TrajectoryPoint {
        position: Vec2::new(x, 0.),
        speed: 0.,
        time: x,
    });
    let mut p = obstacle(5., 5., 9);
    let base = observed_clearance(&path, std::slice::from_ref(&p), 1.25);
    p.additional_radius_m = vec![2.; p.positions.len()];
    let expanded = observed_clearance(&path, std::slice::from_ref(&p), 1.25);
    assert!((base - expanded - 2.).abs() < 1e-12);
    p.additional_radius_m.pop();
    assert_eq!(observed_clearance(&path, &[p], 1.25), f64::NEG_INFINITY);
}

#[test]
fn return_preview_and_active_policy_use_the_new_route_origin_and_its_entire_length() {
    let r = route();
    let mut g = gate();
    g.return_monitor.as_mut().unwrap().config.speed_limit_m_s = Some(1.2);
    mature(&mut g, &r);
    assert!(g.alternate_speed_zones().is_empty());
    let traffic = [obstacle(52., 3.5, 1)];
    let mut o = observation(&r, &traffic, 1.1);
    o.ego.speed = 1.;
    o.motion.as_mut().unwrap().speed = 1.;
    assert!(g.observe_return(o));
    let returned = g.planning_route(&r).unwrap();
    let zones = g.alternate_speed_zones();
    assert_eq!(zones.len(), 1);
    assert_eq!(zones[0].start_s_m, 0.);
    assert_eq!(zones[0].end_s_m, returned.length());
    let mut planner = super::super::resuming::calibrated_planner(
        VehicleConfig::default(),
        4.,
        observation(&r, &traffic, 1.1).limits,
    );
    planner.speed_zones = zones;
    let mut e = ego();
    e.speed = 1.;
    let trajectory = planner.plan(e, &returned, &traffic);
    assert_ne!(trajectory.mode, DrivingMode::Emergency);
    assert!(trajectory.points.iter().all(|p| p.speed <= 1.2 + 1e-9));
    assert!(trajectory.points.last().unwrap().speed <= 0.05);
}
