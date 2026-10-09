use rustdrive_core::*;
use rustdrive_pipeline::intersections::{ConflictBounds, IntersectionPhase, YieldIntersection};
use rustdrive_pipeline::replay::{SensorLog, verify};
use rustdrive_pipeline::traffic_controls::StopLine;
use rustdrive_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

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
