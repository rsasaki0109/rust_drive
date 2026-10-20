use rustdriving_core::*;
use rustdriving_pipeline::intersections::{ConflictBounds, IntersectionPhase, YieldIntersection};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::traffic_controls::{
    SignalColor, SignalObservation, SignalState, StopLine,
};
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, MotionLimits, PipelineConfig, PipelineOutput, SensorFrame,
};
fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 2.1).unwrap(),
        Pose {
            position: Vec2::new(20.0, 0.0),
            yaw: 0.0,
        },
        VehicleConfig::default(),
    );
    c.stop_lines = vec![StopLine {
        id: "main".into(),
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
            position: Vec2::new(20.0, 0.0),
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
fn observation(stamp: f64, color: SignalColor) -> SignalObservation {
    SignalObservation {
        stamp,
        states: vec![SignalState {
            id: "main".into(),
            color,
        }],
    }
}
fn held_signal_fault(output: &PipelineOutput) {
    assert!(output.health.contains(&HealthIssue::InvalidTrafficSignal));
    assert!(output.emergency);
    assert_eq!(output.command.acceleration, -6.0);
    assert_eq!(output.command.steering, 0.0);
    let signal = output.traffic_controls.as_ref().unwrap();
    assert!(signal.fault);
    assert_eq!(signal.last_accepted_stamp, Some(0.0));
    assert_eq!(signal.signals[0].color, SignalColor::Unknown);
    assert!(signal.stop_s_m.is_some());
}

#[test]
fn local_corner_geometry_keeps_original_signal_coordinates_and_green_release() {
    for rotated in [false, true] {
        let rotate = |p: Vec2| if rotated { Vec2::new(-p.y, p.x) } else { p };
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
                c.stop_lines[0].route_s_m = line_s;
                let mut p = DrivingPipeline::new(c.clone()).unwrap();
                let mut log = SensorLog::new("surveyed-signal-with-local-corners", c);
                for i in 0..14 {
                    let mut input = frame(i as f64 * 0.05);
                    input.gnss.as_mut().unwrap().position = position;
                    input.traffic_signal = Some(observation(
                        input.time,
                        if i < 12 {
                            SignalColor::Red
                        } else {
                            SignalColor::Green
                        },
                    ));
                    let out = p.step(&input).unwrap();
                    assert!(out.health.is_empty());
                    assert_eq!(p.active_route().points, route.points);
                    let status = out.traffic_controls.as_ref().unwrap();
                    if i < 12 {
                        assert_eq!(status.stop_s_m, Some(line_s - 2.25));
                        assert!(!out.trajectory.points.is_empty());
                        assert!(
                            out.trajectory.points.iter().all(|point| route
                                .project(point.position)
                                .0
                                + 1.25
                                < line_s)
                        );
                    } else {
                        assert!(status.stop_s_m.is_none());
                        assert!(
                            out.trajectory
                                .points
                                .iter()
                                .any(|point| route.project(point.position).0 > line_s)
                        );
                    }
                    log.record(input, out);
                }
                let mut bytes = vec![];
                log.write(&mut bytes).unwrap();
                assert!(
                    verify(std::io::Cursor::new(bytes), std::io::sink())
                        .unwrap()
                        .verified
                );
            }
        }
    }
}

#[test]
fn malformed_signals_wait_for_a_fresh_post_fault_complete_snapshot() {
    for case in 0..8 {
        let mut pipeline = DrivingPipeline::new(config()).unwrap();
        let mut first = frame(0.0);
        first.traffic_signal = Some(observation(0.0, SignalColor::Green));
        assert!(!pipeline.step(&first).unwrap().emergency);
        let mut invalid = frame(0.05);
        let mut snapshot = observation(0.05, SignalColor::Green);
        match case {
            0 => snapshot.states.clear(),
            1 => snapshot.states.push(snapshot.states[0].clone()),
            2 => snapshot.states[0].id = "unknown".into(),
            3 => snapshot.stamp = -1.0,
            4 => snapshot.stamp = f64::NAN,
            5 => snapshot.stamp = f64::INFINITY,
            6 => snapshot.stamp = 2.0,
            _ => {
                snapshot.stamp = 0.0;
                snapshot.states.clear();
            }
        }
        invalid.traffic_signal = Some(snapshot);
        held_signal_fault(&pipeline.step(&invalid).unwrap());
        for i in 2..=12 {
            let mut input = frame(i as f64 * 0.05);
            input.traffic_signal = match i {
                2 => Some(observation(0.025, SignalColor::Green)),
                3 => Some(observation(0.05, SignalColor::Green)),
                _ => None,
            };
            held_signal_fault(&pipeline.step(&input).unwrap());
        }
        let mut expired = frame(0.65);
        expired.traffic_signal = Some(observation(0.1, SignalColor::Green));
        held_signal_fault(&pipeline.step(&expired).unwrap());
        let mut valid = frame(0.7);
        valid.traffic_signal = Some(observation(0.6, SignalColor::Green));
        let recovered = pipeline.step(&valid).unwrap();
        assert!(recovered.health.is_empty());
        assert!(!recovered.emergency);
        let status = recovered.traffic_controls.unwrap();
        assert!(!status.fault);
        assert_eq!(status.last_accepted_stamp, Some(0.6));
        assert_eq!(status.signals[0].color, SignalColor::Green);
        assert!(status.stop_s_m.is_none());
    }
}

#[test]
fn repeated_signal_failures_advance_recovery_and_valid_red_still_stops() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let mut first = frame(0.0);
    first.traffic_signal = Some(observation(0.0, SignalColor::Green));
    pipeline.step(&first).unwrap();
    let mut invalid = frame(0.05);
    invalid.traffic_signal = Some(SignalObservation {
        stamp: 0.05,
        states: vec![],
    });
    held_signal_fault(&pipeline.step(&invalid).unwrap());
    let mut again = frame(0.1);
    again.traffic_signal = Some(observation(9.0, SignalColor::Green));
    held_signal_fault(&pipeline.step(&again).unwrap());
    for (time, stamp) in [(0.15, 0.075), (0.2, 0.1)] {
        let mut input = frame(time);
        input.traffic_signal = Some(observation(stamp, SignalColor::Green));
        held_signal_fault(&pipeline.step(&input).unwrap());
    }
    for (time, stamp, color) in [
        (0.25, 0.15, SignalColor::Red),
        (0.3, 0.3, SignalColor::Unknown),
        (0.35, 0.35, SignalColor::Green),
    ] {
        let mut input = frame(time);
        input.traffic_signal = Some(observation(stamp, color));
        let output = pipeline.step(&input).unwrap();
        assert!(output.health.is_empty());
        assert!(!output.emergency);
        let status = output.traffic_controls.unwrap();
        assert!(!status.fault);
        assert_eq!(status.signals[0].color, color);
        assert_eq!(status.stop_s_m.is_some(), color != SignalColor::Green);
        if color != SignalColor::Green {
            assert_eq!(output.trajectory.mode, DrivingMode::Yield);
        }
    }
}

#[test]
fn signal_fault_keeps_intersection_dwell_reset_until_post_fault_recovery() {
    let mut config = config();
    config.stop_lines[0].route_s_m = 30.0;
    config.yield_intersections = vec![YieldIntersection {
        stop_line: StopLine {
            id: "junction".into(),
            route_s_m: 35.0,
        },
        conflict_bounds: ConflictBounds {
            min: Vec2::new(42.0, -2.1),
            max: Vec2::new(48.0, 2.1),
        },
        exit_s_m: 48.0,
    }];
    let mut pipeline = DrivingPipeline::new(config).unwrap();
    let recovery_time = 14.0 * 0.05;
    for i in 0..=34 {
        let time = i as f64 * 0.05;
        let mut input = frame(time);
        input.traffic_signal = Some(if i == 10 {
            SignalObservation {
                stamp: time,
                states: vec![],
            }
        } else if (11..14).contains(&i) {
            observation(0.475, SignalColor::Green)
        } else {
            observation(time, SignalColor::Green)
        });
        let output = pipeline.step(&input).unwrap();
        let zone = &output.intersections.as_ref().unwrap().zones[0];
        if (10..14).contains(&i) {
            assert!(output.health.contains(&HealthIssue::InvalidTrafficSignal));
            assert!(output.emergency);
            assert!(zone.clear_since.is_none());
        } else {
            assert!(output.health.is_empty());
        }
        assert_eq!(
            zone.phase,
            if i < 34 {
                IntersectionPhase::Waiting
            } else {
                IntersectionPhase::Proceeding
            }
        );
        assert!(!zone.committed);
        if i >= 14 {
            assert_eq!(zone.clear_since, Some(recovery_time));
        }
    }
}

#[test]
fn signal_recovery_replay_reconstructs_the_fault_epoch() {
    let config = config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("signal-fault-recovery", config);
    for (index, (time, stamp)) in [
        (0.0, 0.0),
        (0.05, 0.05),
        (0.1, 0.025),
        (0.15, 0.05),
        (0.2, 0.1),
    ]
    .into_iter()
    .enumerate()
    {
        let mut input = frame(time);
        let mut snapshot = observation(stamp, SignalColor::Green);
        if index == 1 {
            snapshot.states.clear();
        }
        input.traffic_signal = Some(snapshot);
        let output = pipeline.step(&input).unwrap();
        assert_eq!(output.emergency, (1..=3).contains(&index));
        log.record(input, output);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert_eq!(
        verify(std::io::Cursor::new(bytes), std::io::sink())
            .unwrap()
            .ticks,
        5
    );
    log.ticks[4].input.traffic_signal.as_mut().unwrap().stamp = 0.05;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
#[test]
fn constrained_stop_and_green_release_replay_from_observations_only() {
    let c = config();
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("signal-test", c);
    for i in 0..8 {
        let t = i as f64 * 0.05;
        let mut input = frame(t);
        input.traffic_signal = Some(observation(
            t,
            if i < 4 {
                SignalColor::Red
            } else {
                SignalColor::Green
            },
        ));
        let out = p.step(&input).unwrap();
        assert!(!out.emergency);
        if i < 4 {
            assert_eq!(out.trajectory.mode, DrivingMode::Yield);
            assert!(
                out.trajectory
                    .points
                    .iter()
                    .all(|x| x.position.x + 1.25 < 35.0)
            );
        } else {
            assert!(out.trajectory.points.iter().any(|x| x.position.x > 35.0));
        }
        log.record(input, out);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), Vec::new()).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 8);
}
#[test]
fn future_signal_fault_brakes_until_a_new_valid_snapshot() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    let mut input = frame(0.0);
    input.traffic_signal = Some(observation(9.0, SignalColor::Green));
    let out = p.step(&input).unwrap();
    assert!(out.health.contains(&HealthIssue::InvalidTrafficSignal));
    assert_eq!(out.command.acceleration, -6.0);
    assert!(p.step(&frame(0.05)).unwrap().emergency);
    input = frame(0.1);
    input.traffic_signal = Some(observation(0.1, SignalColor::Green));
    assert!(!p.step(&input).unwrap().emergency);
}
#[test]
fn repeated_green_samples_expire_without_refreshing_the_accepted_clock() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    for i in 0..=11 {
        let mut f = frame(i as f64 * 0.05);
        f.traffic_signal = Some(observation(0.0, SignalColor::Green));
        let out = p.step(&f).unwrap();
        assert!(!out.emergency);
        let state = out.traffic_controls.unwrap();
        assert_eq!(state.last_accepted_stamp, Some(0.0));
        if i == 11 {
            assert_eq!(state.signals[0].color, SignalColor::Unknown);
            assert_eq!(out.trajectory.mode, DrivingMode::Yield);
        }
    }
}
