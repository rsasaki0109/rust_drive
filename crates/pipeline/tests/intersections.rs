use rustdriving_core::*;
use rustdriving_pipeline::intersections::{ConflictBounds, IntersectionPhase, YieldIntersection};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::traffic_controls::{SignalObservation, StopLine};
use rustdriving_pipeline::{DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 2.1).unwrap(),
        Pose {
            position: Vec2::new(20.0, 0.0),
            yaw: 0.0,
        },
        VehicleConfig::default(),
    );
    c.yield_intersections = vec![YieldIntersection {
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
    c
}
fn frame(time: f64) -> SensorFrame {
    SensorFrame {
        absolute_heading: None,
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
            sweep: None,
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
fn replay_recomputes_clear_permission_and_a_new_sensed_conflict_revokes_it() {
    let c = config();
    let mut pipeline = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("observed-intersection", c);
    for i in 0..=24 {
        let now = i as f64 * 0.05;
        let mut input = frame(now);
        if i == 24 {
            // Body-frame returns describe a real circular cluster in the mapped conflict zone.
            input.lidar.as_mut().unwrap().points = (0..24)
                .map(|j| {
                    let angle = j as f64 * std::f64::consts::TAU / 24.0;
                    Vec2::new(25.0 + angle.cos(), angle.sin())
                })
                .collect();
        }
        let output = pipeline.step(&input).unwrap();
        let state = &output.intersections.as_ref().unwrap().zones[0];
        assert!(!output.emergency);
        if i < 20 || i == 24 {
            assert_eq!(state.phase, IntersectionPhase::Waiting);
            assert!(output.trajectory.points.iter().all(|p| p.position.x < 35.0));
        } else {
            assert_eq!(state.phase, IntersectionPhase::Proceeding);
        }
        if i == 24 {
            assert!(!state.blocking_tracks.is_empty());
            assert!(state.clear_since.is_none());
        }
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), vec![]).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 25);
}

#[test]
fn acquisition_fault_interrupts_clear_time_and_recovery_requires_a_new_full_second() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    for i in 0..=31 {
        let now = i as f64 * 0.05;
        let mut input = frame(now);
        if i == 10 {
            input.lidar_failed = true;
            input.lidar = None;
        }
        let output = pipeline.step(&input).unwrap();
        let state = &output.intersections.unwrap().zones[0];
        if i == 10 {
            assert!(output.emergency);
            assert_eq!(state.phase, IntersectionPhase::Waiting);
            assert!(state.clear_since.is_none());
        } else if i < 31 {
            assert_eq!(state.phase, IntersectionPhase::Waiting);
        } else {
            assert_eq!(state.phase, IntersectionPhase::Proceeding);
            assert_eq!(state.clear_since, Some(0.55));
        }
    }
}

#[test]
fn repeated_empty_scan_does_not_establish_fresh_clear_permission() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    for i in 0..=30 {
        let mut input = frame(i as f64 * 0.05);
        input.lidar.as_mut().unwrap().stamp = 0.0;
        let output = pipeline.step(&input).unwrap();
        let state = &output.intersections.unwrap().zones[0];
        assert_eq!(state.phase, IntersectionPhase::Waiting);
        assert!(!state.committed);
        if i >= 4 {
            assert!(state.clear_since.is_none());
        }
    }
}

fn interrupted_clear_frames(case: usize) -> Vec<SensorFrame> {
    let mut frames: Vec<_> = (0..10).map(|i| frame(i as f64 * 0.05)).collect();
    for (index, time) in [0.5, 0.55].into_iter().enumerate() {
        let mut input = frame(time);
        input.lidar = None;
        match case {
            0 if index == 0 => input.gnss.as_mut().unwrap().variance = 0.0,
            0 => input.gnss = None,
            1 if index == 0 => input.odometry.as_mut().unwrap().speed = 21.0,
            1 => input.odometry = None,
            _ => {
                input.traffic_signal = Some(SignalObservation {
                    stamp: time,
                    states: vec![],
                })
            }
        }
        frames.push(input);
    }
    for (time, stamp) in [(0.6, 0.525), (0.65, 0.55)] {
        let mut input = frame(time);
        input.lidar.as_mut().unwrap().stamp = stamp;
        frames.push(input);
    }
    for i in 0..=20 {
        let mut input = frame(0.7 + i as f64 * 0.05);
        input.lidar.as_mut().unwrap().stamp = 0.65 + i as f64 * 0.05;
        frames.push(input);
    }
    frames
}

#[test]
fn post_fault_clear_scans_must_follow_the_last_unhealthy_frame() {
    for (case, issue) in [
        (0, HealthIssue::InvalidGnss),
        (1, HealthIssue::InvalidOdometry),
        (2, HealthIssue::InvalidTrafficSignal),
    ] {
        let mut pipeline = DrivingPipeline::new(config()).unwrap();
        for (index, input) in interrupted_clear_frames(case).into_iter().enumerate() {
            let output = pipeline.step(&input).unwrap();
            if (10..=11).contains(&index) {
                assert!(output.health.contains(&issue));
                assert!(output.emergency);
            } else {
                assert!(output.health.is_empty());
            }
            let state = &output.intersections.unwrap().zones[0];
            assert!(!state.committed);
            assert!(state.blocking_tracks.is_empty());
            assert_eq!(
                state.phase,
                if index == 34 {
                    IntersectionPhase::Proceeding
                } else {
                    IntersectionPhase::Waiting
                }
            );
            if (10..=13).contains(&index) {
                assert!(state.clear_since.is_none());
            } else if index >= 14 {
                assert_eq!(state.clear_since, Some(0.65));
            }
        }
    }
}

#[test]
fn unobserved_scan_gaps_cannot_accumulate_a_clear_second() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    for i in 0..=4 {
        let time = i as f64 * 0.25;
        let output = pipeline.step(&frame(time)).unwrap();
        assert!(output.health.is_empty());
        let state = &output.intersections.unwrap().zones[0];
        assert_eq!(state.phase, IntersectionPhase::Waiting);
        assert_eq!(state.clear_since, Some(time));
    }
    for i in 1..=20 {
        let output = pipeline.step(&frame(1.0 + i as f64 * 0.05)).unwrap();
        assert!(output.health.is_empty());
        let state = &output.intersections.unwrap().zones[0];
        assert_eq!(state.clear_since, Some(1.0));
        assert_eq!(
            state.phase,
            if i == 20 {
                IntersectionPhase::Proceeding
            } else {
                IntersectionPhase::Waiting
            }
        );
    }
}

#[test]
fn a_long_acquisition_gap_revokes_uncommitted_permission() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    for i in 0..=20 {
        pipeline.step(&frame(i as f64 * 0.05)).unwrap();
    }
    let output = pipeline.step(&frame(1.25)).unwrap();
    assert!(output.health.is_empty());
    let state = &output.intersections.unwrap().zones[0];
    assert!(!state.committed);
    assert_eq!(state.phase, IntersectionPhase::Waiting);
    assert_eq!(state.clear_since, Some(1.25));
    for i in 1..=20 {
        let output = pipeline.step(&frame(1.25 + i as f64 * 0.05)).unwrap();
        let state = &output.intersections.unwrap().zones[0];
        assert_eq!(
            state.phase,
            if i == 20 {
                IntersectionPhase::Proceeding
            } else {
                IntersectionPhase::Waiting
            }
        );
    }
}

#[test]
fn regular_five_hz_and_delayed_ten_hz_scans_support_clear_permission() {
    for (period, delay, permission_tick) in [(4, 0.0, 20), (2, 0.1, 22)] {
        let mut pipeline = DrivingPipeline::new(config()).unwrap();
        pipeline.step(&frame(0.0)).unwrap();
        for i in 1..=24 {
            let mut input = frame(i as f64 * 0.05);
            if i % period == 0 && input.time > delay {
                input.lidar.as_mut().unwrap().stamp = input.time - delay;
            } else {
                input.lidar = None;
            }
            let output = pipeline.step(&input).unwrap();
            assert!(output.health.is_empty());
            let state = &output.intersections.unwrap().zones[0];
            assert_eq!(state.clear_since, Some(0.0));
            assert_eq!(
                state.phase,
                if i >= permission_tick {
                    IntersectionPhase::Proceeding
                } else {
                    IntersectionPhase::Waiting
                }
            );
        }
    }
}

#[test]
fn replay_recomputes_the_post_fault_clear_epoch() {
    let config = config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("intersection-post-fault-evidence", config);
    for (index, input) in interrupted_clear_frames(0).into_iter().enumerate() {
        let output = pipeline.step(&input).unwrap();
        if (10..=13).contains(&index) {
            assert!(
                output.intersections.as_ref().unwrap().zones[0]
                    .clear_since
                    .is_none()
            );
        }
        log.record(input, output);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert_eq!(
        verify(std::io::Cursor::new(bytes), std::io::sink())
            .unwrap()
            .ticks,
        35
    );
    assert_eq!(
        log.ticks[34].expected.intersections.as_ref().unwrap().zones[0].phase,
        IntersectionPhase::Proceeding
    );
    log.ticks[34].input.lidar.as_mut().unwrap().stamp = 1.6;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}

#[test]
fn replay_recomputes_clear_confirmation_across_acquisition_gaps() {
    let config = config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("intersection-acquisition-gap", config);
    let times = (0..=4)
        .map(|i| i as f64 * 0.25)
        .chain((1..=20).map(|i| 1.0 + i as f64 * 0.05));
    for (index, time) in times.enumerate() {
        let input = frame(time);
        let output = pipeline.step(&input).unwrap();
        assert_eq!(
            output.intersections.as_ref().unwrap().zones[0].phase,
            if index == 24 {
                IntersectionPhase::Proceeding
            } else {
                IntersectionPhase::Waiting
            }
        );
        log.record(input, output);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert_eq!(
        verify(std::io::Cursor::new(bytes), std::io::sink())
            .unwrap()
            .ticks,
        25
    );
    log.ticks[24].input.lidar.as_mut().unwrap().stamp = 1.95;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
