use rustdriving_core::*;
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::stop_signs::StopPhase;
use rustdriving_pipeline::traffic_controls::{
    SignalColor, SignalObservation, SignalState, StopLine,
};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

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
