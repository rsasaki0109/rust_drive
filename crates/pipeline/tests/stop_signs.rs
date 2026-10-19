use rustdriving_core::*;
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::stop_signs::StopPhase;
use rustdriving_pipeline::traffic_controls::{
    SignalColor, SignalObservation, SignalState, StopLine,
};
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, MotionLimits, PipelineConfig, SensorFrame,
};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 2.1).unwrap(),
        Pose {
            position: Vec2::new(31.75, 0.0),
            yaw: 0.0,
        },
        VehicleConfig::default(),
    );
    c.stop_signs = vec![StopLine {
        id: "stop".into(),
        route_s_m: 35.0,
    }];
    c
}
fn frame(time: f64) -> SensorFrame {
    SensorFrame {
        time,
        odometry: Some(Odometry {
            stamp: time,
            speed: 0.0,
            yaw_rate: 0.0,
        }),
        gnss: Some(Gnss {
            stamp: time,
            position: Vec2::new(31.75, 0.0),
            variance: 0.02,
        }),
        lidar: Some(LidarScan {
            stamp: time,
            points: vec![],
        }),
        multi_height_lidar: None,
        lidar3d: None,
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}
#[test]
fn local_geometry_keeps_surveyed_stop_arcs_before_and_after_a_turn() {
    for rotated in [false, true] {
        let rotate = |p: Vec2| {
            if rotated { Vec2::new(-p.y, p.x) } else { p }
        };
        let route = Route::new(
            [Vec2::default(), Vec2::new(40., 0.), Vec2::new(40., 40.)]
                .into_iter()
                .map(rotate)
                .collect(),
            5.5,
        )
        .unwrap();
        for line_s in [30., 60.] {
            for chassis in [None, Some(1.5)] {
                let (position, yaw) = route.sample(line_s - 3.25, 0.);
                let mut c = config();
                c.route = route.clone();
                c.initial_pose = Pose { position, yaw };
                c.local_route_geometry = true;
                c.rear_axle_offset_m = chassis;
                c.gnss_latency_compensation = true;
                c.motion_limits = Some(MotionLimits {
                    max_acceleration_m_s2: 2.,
                    max_deceleration_m_s2: 2.5,
                    max_lateral_acceleration_m_s2: 2.5,
                });
                c.stop_signs[0].route_s_m = line_s;
                let mut p = DrivingPipeline::new(c.clone()).unwrap();
                let mut log = SensorLog::new("surveyed-stop-with-local-corners", c);
                for i in 0..43 {
                    let mut input = frame(i as f64 * 0.05);
                    input.gnss.as_mut().unwrap().position = position;
                    let out = p.step(&input).unwrap();
                    assert!(out.health.is_empty());
                    assert_eq!(p.active_route().points, route.points);
                    let stops = out.stop_signs.as_ref().unwrap();
                    if i < 40 {
                        assert_eq!(stops.stops[0].phase, StopPhase::Holding);
                        assert_eq!(stops.stop_s_m, Some(line_s - 2.25));
                        assert!(out.command.acceleration <= -0.5);
                        assert!(!out.trajectory.points.is_empty());
                        assert!(
                            out.trajectory
                                .points
                                .iter()
                                .all(|point| { route.project(point.position).0 + 1.25 < line_s })
                        );
                    } else {
                        assert_eq!(stops.stops[0].phase, StopPhase::Released);
                        assert!(
                            out.trajectory
                                .points
                                .iter()
                                .any(|point| { route.project(point.position).0 > line_s })
                        );
                    }
                    log.record(input, out);
                }
                let mut bytes = vec![];
                log.write(&mut bytes).unwrap();
                assert!(
                    verify(std::io::Cursor::new(bytes), Vec::new())
                        .unwrap()
                        .verified
                );
            }
        }
    }
}
#[test]
fn held_stop_releases_from_measured_motion_and_recomputes_every_output() {
    let c = config();
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("stop-sign-test", c);
    for i in 0..43 {
        let input = frame(i as f64 * 0.05);
        let out = p.step(&input).unwrap();
        assert!(!out.emergency);
        let phase = out.stop_signs.as_ref().unwrap().stops[0].phase;
        if i < 40 {
            assert_eq!(phase, StopPhase::Holding);
            assert!(out.command.acceleration <= -0.5);
            assert_eq!(out.trajectory.mode, DrivingMode::Yield);
            assert!(
                out.trajectory
                    .points
                    .iter()
                    .all(|p| p.position.x + 1.25 < 35.0)
            );
        } else {
            assert_eq!(phase, StopPhase::Released);
            assert!(out.trajectory.points.iter().any(|p| p.position.x > 35.0));
        }
        log.record(input, out);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), Vec::new()).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 43);
}
#[test]
fn acquisition_failure_resets_elapsed_stop_time_before_recovery() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    for i in 0..20 {
        p.step(&frame(i as f64 * 0.05)).unwrap();
    }
    let mut bad = frame(1.0);
    bad.lidar_failed = true;
    let out = p.step(&bad).unwrap();
    assert!(out.emergency);
    assert_eq!(out.stop_signs.unwrap().stops[0].held_s, 0.0);
    for i in 21..61 {
        let out = p.step(&frame(i as f64 * 0.05)).unwrap();
        assert_eq!(out.stop_signs.unwrap().stops[0].phase, StopPhase::Holding);
    }
    assert_eq!(
        p.step(&frame(3.05)).unwrap().stop_signs.unwrap().stops[0].phase,
        StopPhase::Released
    );
}
#[test]
fn releasing_a_stop_sign_cannot_release_a_red_signal_or_duplicate_map_line() {
    let mut c = config();
    c.stop_lines = vec![StopLine {
        id: "light".into(),
        route_s_m: 50.0,
    }];
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    for i in 0..43 {
        let mut f = frame(i as f64 * 0.05);
        f.traffic_signal = Some(SignalObservation {
            stamp: f.time,
            states: vec![SignalState {
                id: "light".into(),
                color: SignalColor::Red,
            }],
        });
        let out = p.step(&f).unwrap();
        if i >= 40 {
            assert_eq!(out.stop_signs.unwrap().stops[0].phase, StopPhase::Released);
            assert_eq!(out.trajectory.mode, DrivingMode::Yield);
            assert!(
                out.trajectory
                    .points
                    .iter()
                    .all(|p| p.position.x + 1.25 < 50.0)
            );
        }
    }
    c.stop_lines[0].route_s_m = 35.0;
    assert!(c.validate().is_err());
    c.stop_lines[0].route_s_m = 50.0;
    c.stop_lines[0].id = "stop".into();
    assert!(c.validate().is_err());
}

#[test]
fn cached_or_missing_motion_cannot_complete_the_stop_hold() {
    for case in 0..3 {
        let mut driver = DrivingPipeline::new(config()).unwrap();
        for i in 0..40 {
            driver.step(&frame(i as f64 * 0.05)).unwrap();
        }
        let mut input = frame(2.0);
        match case {
            0 => input.odometry = None,
            1 => input.odometry.as_mut().unwrap().stamp = 1.95,
            _ => input.odometry.as_mut().unwrap().stamp = 1.9,
        }
        let output = driver.step(&input).unwrap();
        assert!(output.health.is_empty());
        assert_eq!(output.trajectory.mode, DrivingMode::Yield);
        let stop = &output.stop_signs.unwrap().stops[0];
        assert_eq!(stop.phase, StopPhase::Holding);
        assert!((stop.held_s - 1.95).abs() < 1e-9);
        let mut input = frame(2.025);
        input.odometry.as_mut().unwrap().stamp = 2.0;
        let output = driver.step(&input).unwrap();
        assert!(output.health.is_empty());
        let stop = &output.stop_signs.unwrap().stops[0];
        assert_eq!(stop.phase, StopPhase::Released);
        assert!((stop.held_s - 2.0).abs() < 1e-9);
    }
}

#[test]
fn stop_confirmation_cannot_bridge_missing_acquisitions_between_control_calls() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for i in 0..20 {
        driver.step(&frame(i as f64 * 0.05)).unwrap();
    }
    // The enclosing clock gap is valid, but the required motion sample at 1.0 is absent.
    let output = driver.step(&frame(1.05)).unwrap();
    assert!(output.health.is_empty());
    let stop = &output.stop_signs.unwrap().stops[0];
    assert_eq!(stop.phase, StopPhase::Holding);
    assert_eq!(stop.held_s, 0.0);
    for i in 1..=40 {
        let output = driver.step(&frame(1.05 + i as f64 * 0.05)).unwrap();
        assert!(output.health.is_empty());
        assert_eq!(
            output.stop_signs.unwrap().stops[0].phase,
            if i < 40 {
                StopPhase::Holding
            } else {
                StopPhase::Released
            }
        );
    }
}

#[test]
fn cached_motion_after_a_fault_cannot_start_the_new_stop_hold() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for i in 0..20 {
        driver.step(&frame(i as f64 * 0.05)).unwrap();
    }
    let mut failed = frame(1.0);
    failed.lidar_failed = true;
    let output = driver.step(&failed).unwrap();
    assert!(output.health.contains(&HealthIssue::InvalidLidar));
    assert!(output.emergency);
    let mut input = frame(1.025);
    input.odometry = None;
    let output = driver.step(&input).unwrap();
    assert!(output.health.is_empty());
    let stop = &output.stop_signs.unwrap().stops[0];
    assert_eq!(stop.phase, StopPhase::Approaching);
    assert_eq!(stop.held_s, 0.0);
    for i in 0..=40 {
        let output = driver.step(&frame(1.05 + i as f64 * 0.05)).unwrap();
        assert!(output.health.is_empty());
        assert_eq!(
            output.stop_signs.unwrap().stops[0].phase,
            if i < 40 {
                StopPhase::Holding
            } else {
                StopPhase::Released
            }
        );
    }
}

#[test]
fn delayed_motion_does_not_credit_time_before_the_first_near_line_observation() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for i in 0..=41 {
        let mut input = frame(0.025 + i as f64 * 0.05);
        input.odometry.as_mut().unwrap().stamp = i as f64 * 0.05;
        let output = driver.step(&input).unwrap();
        assert!(output.health.is_empty());
        let stop = &output.stop_signs.unwrap().stops[0];
        assert_eq!(
            stop.phase,
            if i < 41 {
                StopPhase::Holding
            } else {
                StopPhase::Released
            }
        );
        if i == 40 {
            assert!((stop.held_s - 1.975).abs() < 1e-9);
        }
    }
}

#[test]
fn a_served_stop_remains_released_through_later_sensor_failures() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for i in 0..=40 {
        driver.step(&frame(i as f64 * 0.05)).unwrap();
    }
    let mut failed = frame(2.05);
    failed.lidar_failed = true;
    failed.odometry = None;
    let output = driver.step(&failed).unwrap();
    assert!(output.emergency);
    let status = output.stop_signs.unwrap();
    assert_eq!(status.stops[0].phase, StopPhase::Released);
    assert!(status.stop_s_m.is_none());
    let output = driver.step(&frame(2.1)).unwrap();
    assert!(output.health.is_empty());
    assert_eq!(
        output.stop_signs.unwrap().stops[0].phase,
        StopPhase::Released
    );
}

#[test]
fn replay_rejects_a_stop_release_without_the_final_new_motion_sample() {
    let config = config();
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("stop-sign-motion-evidence", config);
    for i in 0..=40 {
        let mut input = frame(i as f64 * 0.05);
        if i == 40 {
            input.odometry.as_mut().unwrap().stamp = 1.95;
        }
        let output = driver.step(&input).unwrap();
        assert_eq!(
            output.stop_signs.as_ref().unwrap().stops[0].phase,
            StopPhase::Holding
        );
        log.record(input, output);
    }
    let mut input = frame(2.025);
    input.odometry.as_mut().unwrap().stamp = 2.0;
    let output = driver.step(&input).unwrap();
    assert_eq!(
        output.stop_signs.as_ref().unwrap().stops[0].phase,
        StopPhase::Released
    );
    log.record(input, output);
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert_eq!(
        verify(std::io::Cursor::new(bytes), std::io::sink())
            .unwrap()
            .ticks,
        42
    );
    log.ticks[41].input.odometry.as_mut().unwrap().stamp = 1.95;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
