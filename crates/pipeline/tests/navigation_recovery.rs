use rustdriving_core::*;
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, PipelineConfig, PipelineOutput, SensorFrame,
    navigation::{NavigationConfig, NavigationPhase, NavigationUpdate},
    replay::{SensorLog, verify},
};
use std::io::Cursor;

fn config() -> PipelineConfig {
    let scenario: serde_json::Value =
        serde_json::from_str(include_str!("../../../scenarios/route-direct.json")).unwrap();
    let navigation: NavigationConfig =
        serde_json::from_value(scenario["navigation"].clone()).unwrap();
    let mut config = PipelineConfig::new(
        navigation.initial_plan().unwrap().route,
        Pose {
            position: Vec2::new(10.0, 0.0),
            yaw: 0.0,
        },
        VehicleConfig::default(),
    );
    config.navigation = Some(navigation);
    config
}
fn update(stamp: f64, revision: u64, closed: &[&str]) -> NavigationUpdate {
    NavigationUpdate {
        stamp,
        revision,
        closed_edges: closed.iter().map(|edge| (*edge).into()).collect(),
    }
}
fn frame(time: f64, navigation_update: Option<NavigationUpdate>) -> SensorFrame {
    SensorFrame {
        time,
        odometry: Some(Odometry {
            stamp: time,
            speed: 0.0,
            yaw_rate: 0.0,
        }),
        gnss: Some(Gnss {
            stamp: time,
            position: Vec2::new(10.0, 0.0),
            variance: 0.02,
        }),
        lidar: Some(LidarScan {
            stamp: time,
            points: vec![],
        }),
        multi_height_lidar: None,
        lidar3d: None,
        lidar_failed: false,
        navigation_update,
        traffic_signal: None,
    }
}
fn held(output: &PipelineOutput, revision: u64) {
    assert!(output.health.contains(&HealthIssue::InvalidNavigation));
    assert!(output.emergency);
    assert_eq!(output.command.acceleration, -6.0);
    assert_eq!(output.command.steering, 0.0);
    let navigation = output.navigation.as_ref().unwrap();
    assert_eq!(navigation.phase, NavigationPhase::Fault);
    assert_eq!(navigation.revision, revision);
    assert_eq!(navigation.switches, 0);
    assert!(navigation.pending_edges.is_empty());
}

#[test]
fn malformed_updates_hold_until_a_post_fault_snapshot_arrives() {
    for case in 0..6 {
        let mut driver = DrivingPipeline::new(config()).unwrap();
        for i in 0..=20 {
            assert!(
                driver
                    .step(&frame(i as f64 * 0.05, None))
                    .unwrap()
                    .health
                    .is_empty()
            );
        }
        let mut invalid = update(1.05, 1, &[]);
        match case {
            0 => invalid.closed_edges.push("unknown".into()),
            1 => invalid.stamp = 5.0,
            2 => invalid.stamp = f64::NAN,
            3 => invalid.stamp = f64::INFINITY,
            4 => invalid.stamp = -1.0,
            _ => invalid.stamp = 0.0,
        }
        held(&driver.step(&frame(1.05, Some(invalid))).unwrap(), 0);
        held(
            &driver
                .step(&frame(1.1, Some(update(1.025, 1, &[]))))
                .unwrap(),
            0,
        );
        held(
            &driver
                .step(&frame(1.15, Some(update(1.05, 1, &[]))))
                .unwrap(),
            0,
        );
        let recovered = driver.step(&frame(1.2, Some(update(1.1, 1, &[])))).unwrap();
        assert!(recovered.health.is_empty());
        assert!(!recovered.emergency);
        let status = recovered.navigation.unwrap();
        assert_eq!(status.phase, NavigationPhase::Following);
        assert_eq!(status.revision, 1);
        assert_eq!(status.switches, 0);
    }
}

#[test]
fn repeated_invalid_updates_advance_the_cutoff_and_valid_no_path_remains_blocked() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, None)).unwrap();
    held(
        &driver
            .step(&frame(0.05, Some(update(0.05, 1, &["unknown"]))))
            .unwrap(),
        0,
    );
    held(
        &driver.step(&frame(0.1, Some(update(9.0, 1, &[])))).unwrap(),
        0,
    );
    held(
        &driver
            .step(&frame(0.15, Some(update(0.075, 1, &[]))))
            .unwrap(),
        0,
    );
    held(
        &driver.step(&frame(0.2, Some(update(0.1, 1, &[])))).unwrap(),
        0,
    );
    let blocked = driver
        .step(&frame(0.25, Some(update(0.15, 1, &["main", "detour"]))))
        .unwrap();
    assert!(blocked.health.is_empty());
    let status = blocked.navigation.unwrap();
    assert_eq!(status.phase, NavigationPhase::Blocked);
    assert_eq!(status.revision, 1);
    assert_eq!(status.switches, 0);
    assert!(status.pending_edges.is_empty());
    assert!(status.stop_s.is_some());
    for (index, time) in [0.3, 0.35, 0.4].into_iter().enumerate() {
        let output = driver
            .step(&frame(
                time,
                (index == 0).then(|| update(time, 2, &["main"])),
            ))
            .unwrap();
        assert!(output.health.is_empty());
        let status = output.navigation.unwrap();
        assert_eq!(
            status.phase,
            if index < 2 {
                NavigationPhase::Braking
            } else {
                NavigationPhase::Following
            }
        );
        assert_eq!(status.switches, usize::from(index == 2));
    }
}

#[test]
fn faults_cancel_pending_handover_and_recovery_needs_three_new_stopped_samples() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    let original_edges = driver.navigation_plan().unwrap().edge_ids.clone();
    driver.step(&frame(0.0, None)).unwrap();
    for (time, snapshot) in [(0.05, Some(update(0.05, 1, &["main"]))), (0.1, None)] {
        let output = driver.step(&frame(time, snapshot)).unwrap();
        assert_eq!(output.navigation.unwrap().phase, NavigationPhase::Braking);
    }
    held(
        &driver
            .step(&frame(0.15, Some(update(0.15, 2, &["unknown"]))))
            .unwrap(),
        1,
    );
    held(
        &driver
            .step(&frame(0.2, Some(update(0.1, 2, &["main"]))))
            .unwrap(),
        1,
    );
    held(&driver.step(&frame(0.25, None)).unwrap(), 1);
    for (index, time) in [0.3, 0.35, 0.4].into_iter().enumerate() {
        let output = driver
            .step(&frame(
                time,
                (index == 0).then(|| update(0.25, 2, &["main"])),
            ))
            .unwrap();
        assert!(output.health.is_empty());
        let status = output.navigation.unwrap();
        assert_eq!(status.revision, 2);
        if index < 2 {
            assert_eq!(status.phase, NavigationPhase::Braking);
            assert_eq!(status.active_edges, original_edges);
            assert_eq!(status.switches, 0);
        } else {
            assert_eq!(status.phase, NavigationPhase::Following);
            assert_eq!(status.switches, 1);
            assert!(status.active_edges.contains(&"detour".into()));
            assert!(!status.active_edges.contains(&"main".into()));
        }
        assert_eq!(output.estimate.pose.position, Vec2::new(10.0, 0.0));
    }
}

#[test]
fn ordinary_duplicate_and_reordered_snapshots_keep_the_accepted_closure() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, None)).unwrap();
    for (index, (time, snapshot)) in [
        (0.05, update(0.05, 2, &["main"])),
        (0.1, update(0.1, 1, &[])),
        (0.15, update(0.025, 3, &[])),
    ]
    .into_iter()
    .enumerate()
    {
        let output = driver.step(&frame(time, Some(snapshot))).unwrap();
        assert!(output.health.is_empty());
        let status = output.navigation.unwrap();
        assert_eq!(status.revision, 2);
        assert_eq!(status.closed_edges, vec!["main"]);
        assert_eq!(status.switches, usize::from(index == 2));
        assert_eq!(
            status.phase,
            if index < 2 {
                NavigationPhase::Braking
            } else {
                NavigationPhase::Following
            }
        );
    }
}

#[test]
fn replay_reconstructs_navigation_fault_recovery_and_rejects_pre_fault_release() {
    let config = config();
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("navigation-fault-recovery", config);
    for (index, (time, snapshot)) in [
        (0.0, None),
        (0.05, Some(update(0.05, 1, &["unknown"]))),
        (0.1, Some(update(0.025, 1, &[]))),
        (0.15, Some(update(0.05, 1, &[]))),
        (0.2, Some(update(0.1, 1, &[]))),
    ]
    .into_iter()
    .enumerate()
    {
        let input = frame(time, snapshot);
        let output = driver.step(&input).unwrap();
        assert_eq!(output.emergency, (1..=3).contains(&index));
        log.record(input, output);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert_eq!(
        verify(Cursor::new(bytes), std::io::sink()).unwrap().ticks,
        5
    );
    log.ticks[4].input.navigation_update.as_mut().unwrap().stamp = 0.05;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}

#[test]
fn repeated_or_missing_odometry_cannot_accumulate_stopped_handover_samples() {
    for case in 0..3 {
        let mut driver = DrivingPipeline::new(config()).unwrap();
        driver.step(&frame(0.0, None)).unwrap();
        let pending = driver
            .step(&frame(0.05, Some(update(0.05, 1, &["main"]))))
            .unwrap();
        let original_edges = pending.navigation.unwrap().active_edges;
        for time in [0.075, 0.1, 0.125] {
            let mut input = frame(time, None);
            match case {
                0 => input.odometry = None,
                1 => input.odometry.as_mut().unwrap().stamp = 0.05,
                _ => input.odometry.as_mut().unwrap().stamp = 0.025,
            }
            let output = driver.step(&input).unwrap();
            assert!(output.health.is_empty());
            let status = output.navigation.unwrap();
            assert_eq!(status.phase, NavigationPhase::Braking);
            assert_eq!(status.switches, 0);
            assert_eq!(status.active_edges, original_edges);
        }
        let second = driver.step(&frame(0.15, None)).unwrap();
        assert_eq!(second.navigation.unwrap().phase, NavigationPhase::Braking);
        let third = driver.step(&frame(0.175, None)).unwrap();
        assert!(third.health.is_empty());
        let status = third.navigation.unwrap();
        assert_eq!(status.phase, NavigationPhase::Following);
        assert_eq!(status.switches, 1);
        assert!(status.active_edges.contains(&"detour".into()));
    }
}

#[test]
fn a_cached_pre_request_stop_is_not_a_new_handover_sample() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, None)).unwrap();
    for time in [0.025, 0.05, 0.075] {
        let mut input = frame(time, (time == 0.025).then(|| update(time, 1, &["main"])));
        input.odometry = None;
        let output = driver.step(&input).unwrap();
        assert!(output.health.is_empty());
        let status = output.navigation.unwrap();
        assert_eq!(status.phase, NavigationPhase::Braking);
        assert_eq!(status.switches, 0);
    }
    for (index, time) in [0.1, 0.125, 0.15].into_iter().enumerate() {
        let output = driver.step(&frame(time, None)).unwrap();
        assert!(output.health.is_empty());
        let status = output.navigation.unwrap();
        assert_eq!(status.switches, usize::from(index == 2));
        assert_eq!(
            status.phase,
            if index < 2 {
                NavigationPhase::Braking
            } else {
                NavigationPhase::Following
            }
        );
    }
}

#[test]
fn startup_requires_measured_standstill_even_before_the_first_prediction() {
    for speed in [-0.1, 0.1, 2.0] {
        let mut driver = DrivingPipeline::new(config()).unwrap();
        let mut input = frame(0.0, Some(update(0.0, 1, &["main"])));
        input.odometry.as_mut().unwrap().speed = speed;
        let output = driver.step(&input).unwrap();
        assert!(output.health.is_empty());
        assert_eq!(output.estimate.speed, 0.0);
        assert_eq!(output.navigation.unwrap().phase, NavigationPhase::Braking);
        for (index, time) in [0.05, 0.1, 0.15].into_iter().enumerate() {
            let output = driver.step(&frame(time, None)).unwrap();
            assert!(output.health.is_empty());
            assert_eq!(output.navigation.unwrap().switches, usize::from(index == 2));
        }
    }
}

#[test]
fn a_health_interruption_requires_new_motion_before_confirmation_resumes() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, None)).unwrap();
    for time in [0.025, 0.05] {
        let output = driver
            .step(&frame(
                time,
                (time == 0.025).then(|| update(time, 1, &["main"])),
            ))
            .unwrap();
        assert_eq!(output.navigation.unwrap().phase, NavigationPhase::Braking);
    }
    let mut failed = frame(0.075, None);
    failed.lidar_failed = true;
    let output = driver.step(&failed).unwrap();
    assert!(output.health.contains(&HealthIssue::InvalidLidar));
    assert!(output.emergency);
    assert_eq!(output.navigation.unwrap().phase, NavigationPhase::Braking);
    for time in [0.1, 0.125, 0.15] {
        let mut input = frame(time, None);
        input.odometry.as_mut().unwrap().stamp = 0.075;
        let output = driver.step(&input).unwrap();
        assert!(output.health.is_empty());
        let status = output.navigation.unwrap();
        assert_eq!(status.phase, NavigationPhase::Braking);
        assert_eq!(status.switches, 0);
    }
    for (index, time) in [0.175, 0.2, 0.225].into_iter().enumerate() {
        let output = driver.step(&frame(time, None)).unwrap();
        assert!(output.health.is_empty());
        let status = output.navigation.unwrap();
        assert_eq!(status.switches, usize::from(index == 2));
        assert_eq!(
            status.phase,
            if index < 2 {
                NavigationPhase::Braking
            } else {
                NavigationPhase::Following
            }
        );
    }
}

#[test]
fn expired_motion_restarts_confirmation_from_three_new_acquisitions() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, None)).unwrap();
    for time in [0.05, 0.1] {
        let output = driver
            .step(&frame(
                time,
                (time == 0.05).then(|| update(time, 1, &["main"])),
            ))
            .unwrap();
        assert_eq!(output.navigation.unwrap().phase, NavigationPhase::Braking);
    }
    let mut input = frame(0.275, None);
    input.odometry = None;
    let output = driver.step(&input).unwrap();
    assert!(output.health.contains(&HealthIssue::StaleOdometry));
    assert!(output.emergency);
    assert_eq!(output.navigation.unwrap().phase, NavigationPhase::Braking);
    for (index, time) in [0.3, 0.325, 0.35].into_iter().enumerate() {
        let output = driver.step(&frame(time, None)).unwrap();
        assert!(output.health.is_empty());
        let status = output.navigation.unwrap();
        assert_eq!(status.switches, usize::from(index == 2));
        assert_eq!(
            status.phase,
            if index < 2 {
                NavigationPhase::Braking
            } else {
                NavigationPhase::Following
            }
        );
    }
}

#[test]
fn an_unsolicited_signal_cannot_allow_handover_before_its_health_fault_is_reported() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, None)).unwrap();
    for time in [0.05, 0.1] {
        let output = driver
            .step(&frame(
                time,
                (time == 0.05).then(|| update(time, 1, &["main"])),
            ))
            .unwrap();
        assert_eq!(output.navigation.unwrap().phase, NavigationPhase::Braking);
    }
    let mut input = frame(0.15, None);
    input.traffic_signal = Some(rustdriving_pipeline::traffic_controls::SignalObservation {
        stamp: 0.15,
        states: vec![],
    });
    let output = driver.step(&input).unwrap();
    assert!(output.health.contains(&HealthIssue::InvalidTrafficSignal));
    assert!(output.emergency);
    let status = output.navigation.unwrap();
    assert_eq!(status.phase, NavigationPhase::Braking);
    assert_eq!(status.switches, 0);
    for (index, time) in [0.175, 0.2, 0.225].into_iter().enumerate() {
        let output = driver.step(&frame(time, None)).unwrap();
        assert!(output.health.is_empty());
        assert_eq!(output.navigation.unwrap().switches, usize::from(index == 2));
    }
}

#[test]
fn navigation_fault_hold_excludes_motion_acquired_before_its_last_unhealthy_frame() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, None)).unwrap();
    for time in [0.025, 0.05] {
        driver
            .step(&frame(
                time,
                (time == 0.025).then(|| update(time, 1, &["main"])),
            ))
            .unwrap();
    }
    for time in [0.075, 0.1] {
        let mut input = frame(time, (time == 0.075).then(|| update(time, 2, &["unknown"])));
        input.odometry = None;
        held(&driver.step(&input).unwrap(), 1);
    }
    let mut input = frame(0.125, Some(update(0.11, 2, &["main"])));
    input.odometry.as_mut().unwrap().stamp = 0.09;
    let output = driver.step(&input).unwrap();
    assert!(output.health.is_empty());
    assert_eq!(output.navigation.unwrap().phase, NavigationPhase::Braking);
    for (index, time) in [0.175, 0.2, 0.225].into_iter().enumerate() {
        let output = driver.step(&frame(time, None)).unwrap();
        assert!(output.health.is_empty());
        assert_eq!(output.navigation.unwrap().switches, usize::from(index == 2));
    }
}

#[test]
fn replay_requires_distinct_motion_samples_for_a_route_handover() {
    let config = config();
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("navigation-stop-evidence", config);
    for (index, time) in [0.0, 0.05, 0.075, 0.1, 0.125, 0.15, 0.175]
        .into_iter()
        .enumerate()
    {
        let mut input = frame(time, (index == 1).then(|| update(time, 1, &["main"])));
        match index {
            2 => input.odometry = None,
            3 => input.odometry.as_mut().unwrap().stamp = 0.05,
            4 => input.odometry.as_mut().unwrap().stamp = 0.025,
            _ => {}
        }
        let output = driver.step(&input).unwrap();
        assert!(output.health.is_empty());
        assert_eq!(
            output.navigation.as_ref().unwrap().switches,
            usize::from(index == 6)
        );
        log.record(input, output);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert_eq!(
        verify(Cursor::new(bytes), std::io::sink()).unwrap().ticks,
        7
    );
    log.ticks[5].input.odometry.as_mut().unwrap().stamp = 0.05;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
