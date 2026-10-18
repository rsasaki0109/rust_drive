use super::*;
use crate::MotionLimits;
use rustdriving_core::{EgoState, Odometry, Pose, Prediction, Vec2};
use rustdriving_routing::lane_resume::{self, LaneResumeRequest};

fn fixture() -> (LaunchGuard, Route, Route) {
    let original = Route::new(
        vec![
            Vec2::default(),
            Vec2::new(25., 0.),
            Vec2::new(55., 3.5),
            Vec2::new(80., 3.5),
            Vec2::new(160., 3.5),
        ],
        1.75,
    )
    .unwrap();
    let source = LaneChangeConfig {
        entry_s_m: 25.,
        exit_s_m: original.project(Vec2::new(55., 3.5)).0,
        target_start: Vec2::default(),
        target_end: Vec2::new(80., 0.),
        target_width_m: 3.5,
        return_on_conflict: None,
    };
    let target = LaneChangeConfig {
        target_start: Vec2::new(0., 3.5),
        target_end: Vec2::new(80., 3.5),
        ..source.clone()
    };
    let plan = lane_resume::plan(
        LaneResumeRequest {
            start: Pose {
                position: Vec2::new(42., 0.),
                yaw: 0.,
            },
            source_start: source.target_start,
            source_end: source.target_end,
            source_width_m: 3.5,
            target_start: target.target_start,
            target_end: target.target_end,
            target_width_m: 3.5,
            original_exit_s_m: target.exit_s_m,
            distance_m: 19.,
        },
        &original,
        VehicleConfig::default(),
    )
    .unwrap();
    let o = observation(&original, &plan.route, 11.1, 0., 0., &[]);
    let guard = LaunchGuard::new(
        &source,
        &target,
        VehicleConfig::default(),
        2.,
        &plan.route,
        plan.connector_length_m,
        &o,
    );
    (guard, plan.route, original)
}
fn observation<'a>(
    original: &'a Route,
    route: &Route,
    now: f64,
    progress: f64,
    speed: f64,
    predictions: &'a [Prediction],
) -> ReturnObservation<'a> {
    let (position, yaw) = route.sample(progress, 0.);
    ReturnObservation {
        route: original,
        ego: EgoState {
            pose: Pose { position, yaw },
            speed,
        },
        predictions,
        now,
        scan: Some(now),
        healthy: true,
        motion: Some(Odometry {
            stamp: now,
            speed,
            yaw_rate: 0.,
        }),
        limits: MotionLimits {
            max_acceleration_m_s2: 1.,
            max_deceleration_m_s2: 2.5,
            max_lateral_acceleration_m_s2: 1.,
        },
    }
}
fn rear() -> Prediction {
    Prediction {
        id: 9,
        radius: 1.1,
        dt: 0.2,
        positions: (0..=40)
            .map(|i| Vec2::new(30. + 6. * i as f64 * 0.2, 3.5))
            .collect(),
    }
}
fn pause(g: &mut LaunchGuard, route: &Route, original: &Route) {
    assert!(g.step(
        route,
        &observation(original, route, 11.2, 0.1, 1., &[rear()])
    ));
    assert_eq!(g.status().unwrap().phase, LaunchHoldPhase::Braking);
    assert_eq!(g.braking_limit(), Some(-1.25));
    let state = g.status().unwrap();
    assert!(state.planning_endpoint_s_m < state.source_limit_s_m);
    assert!(state.target_blocking_tracks.contains(&9));
    assert!(g.route().unwrap().length() < route.length());
}
fn stop(g: &mut LaunchGuard, route: &Route, original: &Route, start: f64) {
    for i in 0..3 {
        assert!(!g.step(
            route,
            &observation(original, route, start + i as f64 * 0.05, 0.5, 0., &[rear()])
        ));
    }
    assert!(g.status().unwrap().stopped_at.is_some());
}

#[test]
fn a_fresh_rear_conflict_brakes_then_requires_distinct_stops_and_post_stop_clear_scans() {
    let (mut g, route, original) = fixture();
    pause(&mut g, &route, &original);
    for i in 0..3 {
        let now = 11.3 + i as f64 * 0.01;
        let mut o = observation(&original, &route, now, 0.5, 0., &[]);
        o.motion.as_mut().unwrap().stamp = 11.3;
        assert!(!g.step(&route, &o));
    }
    assert!(g.status().unwrap().stopped_at.is_none());
    for now in [11.35, 11.4] {
        assert!(!g.step(&route, &observation(&original, &route, now, 0.5, 0., &[])));
    }
    assert_eq!(g.status().unwrap().stopped_at, Some(11.4));
    for i in 0..10 {
        assert!(!g.step(
            &route,
            &observation(&original, &route, 11.5 + i as f64 * 0.1, 0.5, 0., &[])
        ));
    }
    assert!(g.step(&route, &observation(&original, &route, 12.5, 0.5, 0., &[])));
    let status = g.status().unwrap();
    assert_eq!(status.phase, LaunchHoldPhase::Released);
    assert_eq!(status.clear_since, Some(11.5));
    assert_eq!(status.released_at, Some(12.5));
    assert!(status.crossing_preview_clearance_m.unwrap() >= 1.3);
    assert!(g.route().is_none() && g.braking_limit().is_none());
    // One fixed connector, with no second pause/retry after release.
    assert!(!g.step(
        &route,
        &observation(&original, &route, 12.6, 0.5, 0., &[rear()])
    ));
    assert_eq!(g.status().unwrap().released_at, Some(12.5));
}

#[test]
fn sensing_fault_delayed_scans_and_repeated_motion_cannot_reuse_clearance() {
    let (mut g, route, original) = fixture();
    pause(&mut g, &route, &original);
    stop(&mut g, &route, &original, 11.3);
    for i in 0..10 {
        assert!(!g.step(
            &route,
            &observation(&original, &route, 11.5 + i as f64 * 0.1, 0.5, 0., &[])
        ));
    }
    let mut fault = observation(&original, &route, 12.5, 0.5, 0., &[]);
    fault.healthy = false;
    fault.scan = None;
    assert!(!g.step(&route, &fault));
    let mut delayed = observation(&original, &route, 12.6, 0.5, 0., &[]);
    delayed.scan = Some(12.5);
    assert!(!g.step(&route, &delayed));
    assert!(g.status().unwrap().clear_since.is_none());
    for now in [12.65, 12.7] {
        assert!(!g.step(&route, &observation(&original, &route, now, 0.5, 0., &[])));
    }
    assert_eq!(g.status().unwrap().stopped_at, Some(12.7));
    for i in 0..10 {
        assert!(!g.step(
            &route,
            &observation(&original, &route, 12.8 + i as f64 * 0.1, 0.5, 0., &[])
        ));
    }
    assert!(g.step(&route, &observation(&original, &route, 13.8, 0.5, 0., &[])));
}

#[test]
fn a_gap_in_acquisitions_or_persistent_traffic_keeps_the_source_hold() {
    let (mut g, route, original) = fixture();
    pause(&mut g, &route, &original);
    stop(&mut g, &route, &original, 11.3);
    for i in 0..30 {
        assert!(!g.step(
            &route,
            &observation(&original, &route, 11.5 + i as f64 * 0.1, 0.5, 0., &[rear()])
        ));
    }
    assert!(g.status().unwrap().clear_since.is_none());
    for i in 0..10 {
        assert!(!g.step(
            &route,
            &observation(&original, &route, 14.5 + i as f64 * 0.1, 0.5, 0., &[])
        ));
    }
    assert!(!g.step(&route, &observation(&original, &route, 16., 0.5, 0., &[])));
    for i in 1..10 {
        assert!(!g.step(
            &route,
            &observation(&original, &route, 16. + i as f64 * 0.1, 0.5, 0., &[])
        ));
    }
    assert!(g.step(&route, &observation(&original, &route, 17., 0.5, 0., &[])));
}

#[test]
fn insufficient_braking_room_cannot_select_a_source_prefix_after_commitment() {
    let (mut g, route, original) = fixture();
    let progress = g.source_limit_s_m - 0.5;
    assert!(!g.step(
        &route,
        &observation(&original, &route, 11.2, progress, 2., &[rear()])
    ));
    assert!(g.committed && g.status().is_none() && g.route().is_none());
    // A later apparent backward estimate does not undo the committed crossing.
    assert!(!g.step(
        &route,
        &observation(&original, &route, 11.3, 0., 0., &[rear()])
    ));
    assert!(g.status().is_none());
}

#[test]
fn source_prefix_boundary_matches_an_independent_intersection_in_rotated_frames() {
    for angle in [0.0_f64, 0.37, std::f64::consts::FRAC_PI_2] {
        for side in [-1., 1.] {
            let turn = |x: f64, y: f64| {
                Vec2::new(
                    10. + x * angle.cos() - side * y * angle.sin(),
                    -20. + x * angle.sin() + side * y * angle.cos(),
                )
            };
            let source = LaneChangeConfig {
                entry_s_m: 25.,
                exit_s_m: 55.,
                target_start: turn(0., 0.),
                target_end: turn(80., 0.),
                target_width_m: 3.5,
                return_on_conflict: None,
            };
            let route = Route::new(
                vec![
                    turn(40., 0.),
                    turn(50., 0.),
                    turn(60., 3.5),
                    turn(160., 3.5),
                ],
                1.75,
            )
            .unwrap();
            let expected = 10. + 0.25 / 3.5 * 10.0_f64.hypot(3.5);
            let limit = source_limit(&route, &source, VehicleConfig::default());
            assert!((limit - expected).abs() < 1e-10);
            let p = route.sample(limit, 0.).0;
            let (_, y) = source.coordinates(p);
            assert!((y.abs() - 0.25).abs() < 1e-10);
            let beyond = source.coordinates(route.sample(limit + 0.01, 0.).0).1.abs();
            assert!(beyond > 0.25);
        }
    }
}
