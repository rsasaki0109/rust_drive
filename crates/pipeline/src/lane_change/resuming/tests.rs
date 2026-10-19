use super::*;
use crate::lane_change::LaneChangeReturnConfig;
use rustdriving_core::{EgoState, Odometry, Pose, Prediction, Vec2};

fn fixture() -> (ResumeMonitor, LaneChangeStatus, Route, Route) {
    let route = Route::new(
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
    let target = LaneChangeConfig {
        entry_s_m: 25.,
        exit_s_m: route.project(Vec2::new(55., 3.5)).0,
        target_start: Vec2::new(0., 3.5),
        target_end: Vec2::new(80., 3.5),
        target_width_m: 3.5,
        return_on_conflict: Some(LaneChangeReturnConfig {
            source_start: Vec2::default(),
            source_end: Vec2::new(80., 0.),
            source_width_m: 3.5,
            max_return_distance_m: 30.,
            speed_limit_m_s: None,
            resume_after_return: Some(LaneChangeResumeConfig {
                max_change_distance_m: 30.,
            }),
        }),
    };
    let source = LaneChangeConfig {
        target_start: Vec2::default(),
        target_end: Vec2::new(80., 0.),
        ..target.clone()
    };
    let mut monitor = ResumeMonitor::new(&target, &source, VehicleConfig::default(), 2.).unwrap();
    monitor.arm(10.);
    let state = LaneChangeStatus {
        phase: LaneChangePhase::Aborted,
        blocking_tracks: vec![],
        front_gap_m: None,
        rear_gap_m: None,
        clear_since: None,
        stop_s_m: None,
        return_state: None,
    };
    (
        monitor,
        state,
        route,
        Route::new(vec![Vec2::new(30., 0.), Vec2::new(43., 0.)], 1.75).unwrap(),
    )
}
fn observation<'a>(
    route: &'a Route,
    now: f64,
    predictions: &'a [Prediction],
) -> ReturnObservation<'a> {
    ReturnObservation {
        speed_zones: &[],
        route,
        now,
        predictions,
        ego: EgoState {
            pose: Pose {
                position: Vec2::new(42., 0.),
                yaw: 0.,
            },
            speed: 0.,
        },
        scan: Some(now),
        healthy: true,
        motion: Some(Odometry {
            stamp: now,
            speed: 0.,
            yaw_rate: 0.,
        }),
        limits: MotionLimits {
            max_acceleration_m_s2: 1.,
            max_deceleration_m_s2: 2.5,
            max_lateral_acceleration_m_s2: 1.,
        },
    }
}
fn obstacle(x: f64, y: f64) -> Prediction {
    Prediction {
        additional_radius_m: vec![],
        id: 1,
        positions: vec![Vec2::new(x, y); 41],
        dt: 0.2,
        radius: 1.1,
    }
}

#[test]
fn resume_needs_post_stop_acquisitions_then_reaches_target_once_and_keeps_the_new_route() {
    let (mut m, mut state, r, stop) = fixture();
    for i in 0..=10 {
        assert!(!m.step(
            &mut state,
            &observation(&r, 10. + i as f64 * 0.1, &[]),
            &stop
        ));
    }
    assert!(m.step(&mut state, &observation(&r, 11.1, &[]), &stop));
    assert_eq!(state.phase, LaneChangePhase::Resuming);
    let request = m.status.request.unwrap();
    assert_eq!(request.start.position, Vec2::new(42., 0.));
    let selected = m.route.clone().unwrap();
    assert_eq!(*selected.points.last().unwrap(), Vec2::new(160., 3.5));
    let mut moving = observation(&r, 12., &[]);
    moving.ego.pose.position = Vec2::new(78., 3.5);
    moving.ego.speed = 2.;
    assert!(!m.step(&mut state, &moving, &stop));
    assert_eq!(state.phase, LaneChangePhase::Completed);
    assert_eq!(m.route().unwrap().points, selected.points);
    assert_eq!(m.status.started_at, Some(11.1));
}

#[test]
fn retry_and_launch_previews_keep_the_rebased_tail_policy() {
    let (mut monitor, mut state, route, stop) = fixture();
    monitor.cruise = 4.;
    monitor
        .target
        .return_on_conflict
        .as_mut()
        .unwrap()
        .speed_limit_m_s = Some(2.);
    let zones = [rustdriving_core::RouteSpeedZone {
        start_s_m: route.project(Vec2::new(80., 3.5)).0,
        end_s_m: route.project(Vec2::new(80., 3.5)).0 + 0.01,
        speed_m_s: 1.,
    }];
    for i in 0..=11 {
        let mut o = observation(&route, 10. + i as f64 * 0.1, &[]);
        o.speed_zones = &zones;
        monitor.step(&mut state, &o, &stop);
    }
    assert_eq!(state.phase, LaneChangePhase::Resuming);
    let active = monitor.speed_zones().unwrap();
    assert_eq!(active.len(), 2);
    assert_eq!(active[0].start_s_m, 0.);
    assert_eq!(
        active[0].end_s_m,
        monitor.status.connector_length_m.unwrap()
    );
    assert_eq!(active[0].speed_m_s, 2.);
    assert!((active[1].end_s_m - active[1].start_s_m - 0.01).abs() < 1e-12);
    assert_eq!(
        serde_json::to_value(active).unwrap(),
        serde_json::to_value(&monitor.launch.as_ref().unwrap().speed_zones).unwrap()
    );
}

#[test]
fn target_and_fast_source_rear_tracks_prevent_retry_even_after_the_hold_timer() {
    for case in 0..3 {
        let (mut m, mut state, r, stop) = fixture();
        let mut p = obstacle(52., 3.5);
        if case == 1 {
            for (i, q) in p.positions.iter_mut().enumerate() {
                *q = Vec2::new(20. + 3. * i as f64 * 0.2, 0.);
            }
        }
        if case == 2 {
            p.positions[20].x = f64::NAN;
        }
        for i in 1..=30 {
            assert!(!m.step(
                &mut state,
                &observation(&r, 10. + i as f64 * 0.1, std::slice::from_ref(&p)),
                &stop
            ));
        }
        assert_eq!(state.phase, LaneChangePhase::Aborted);
        assert_eq!(m.status.decision, ResumeDecision::TrafficBlocked);
        assert!(m.status.clear_since.is_none());
        assert!(m.route().is_none());
    }
}

#[test]
fn faults_delayed_cached_and_gapped_scans_cannot_reuse_a_mature_resume_confirmation() {
    for case in 0..6 {
        let (mut m, mut state, r, stop) = fixture();
        for i in 1..=10 {
            assert!(!m.step(
                &mut state,
                &observation(&r, 10. + i as f64 * 0.1, &[]),
                &stop
            ));
        }
        let mut fault = observation(&r, 11.1, &[]);
        match case {
            0 => fault.healthy = false,
            1 => fault.motion = None,
            2 => fault.motion.as_mut().unwrap().speed = 0.2,
            3 => fault.motion.as_mut().unwrap().yaw_rate = 0.1,
            4 => fault.scan = None,
            _ => fault.ego.pose.position.y = 0.6,
        }
        assert!(!m.step(&mut state, &fault, &stop));
        let mut delayed = observation(&r, 11.2, &[]);
        delayed.scan = Some(11.1);
        assert!(!m.step(&mut state, &delayed, &stop));
        assert!(m.status.clear_since.is_none());
        for i in 0..10 {
            let mut cached = observation(&r, 11.25 + i as f64 * 0.005, &[]);
            cached.scan = Some(11.21);
            cached.motion.as_mut().unwrap().stamp = 11.21;
            assert!(!m.step(&mut state, &cached, &stop));
        }
        assert!(m.clear_samples <= 1);
        // A missing acquisition interval restarts the full one-second window.
        assert!(!m.step(&mut state, &observation(&r, 12., &[]), &stop));
        for i in 1..10 {
            assert!(!m.step(
                &mut state,
                &observation(&r, 12. + i as f64 * 0.1, &[]),
                &stop
            ));
        }
        assert!(
            m.step(&mut state, &observation(&r, 13., &[]), &stop),
            "case {case}"
        );
    }
}

#[test]
fn insufficient_remaining_section_or_bad_heading_retains_the_return_stop() {
    for case in 0..2 {
        let (mut m, mut state, r, _) = fixture();
        let stop = Route::new(vec![Vec2::new(30., 0.), Vec2::new(72., 0.)], 1.75).unwrap();
        for i in 1..=20 {
            let mut o = observation(&r, 10. + i as f64 * 0.1, &[]);
            o.ego.pose.position.x = 71.;
            if case == 1 {
                o.ego.pose.yaw = 0.2;
            }
            assert!(!m.step(&mut state, &o, &stop));
        }
        assert_eq!(state.phase, LaneChangePhase::Aborted);
        assert!(m.route().is_none());
        assert_eq!(
            m.status.decision,
            if case == 0 {
                ResumeDecision::InfeasibleGeometry
            } else {
                ResumeDecision::InvalidEvidence
            }
        );
    }
}
