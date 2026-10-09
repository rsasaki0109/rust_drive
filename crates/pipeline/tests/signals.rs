use rustdriving_core::*;
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::traffic_controls::{
    SignalColor, SignalObservation, SignalState, StopLine,
};
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
