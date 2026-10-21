use rustdriving_core::{Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    let mut config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    config.confirm_track_initialization = true;
    config
}

fn frame(time: f64, biased: bool) -> SensorFrame {
    serde_json::from_value(serde_json::json!({
        "time":time,"odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":time,"position":{"x":0.,"y":if biased {0.1} else {0.}},"variance":0.02},
        "lidar":{"stamp":time,"points":(0..32).map(|i|{
            let angle=i as f64*std::f64::consts::TAU/32.;
            Vec2::new(20.+2.*time+angle.cos(),4.+angle.sin())
        }).collect::<Vec<_>>()}
    }))
    .unwrap()
}

#[test]
fn raw_acquisitions_and_confirmed_velocity_survive_absolute_pose_corrections() {
    let mut a = DrivingPipeline::new(config()).unwrap();
    let mut b = DrivingPipeline::new(config()).unwrap();
    for i in 0..=10 {
        let time = i as f64 * 0.1;
        let clean = a.step(&frame(time, false)).unwrap();
        let corrected = b.step(&frame(time, true)).unwrap();
        assert_eq!(clean.track_measurements, corrected.track_measurements);
        let t = &corrected.tracks[0];
        let transform = corrected.tracking_frame.unwrap();
        if i < 2 {
            assert_eq!(t.velocity, Vec2::default());
        } else {
            assert!(
                t.velocity
                    .rotated(-transform.yaw)
                    .distance(Vec2::new(2., 0.))
                    < 1e-8
            );
        }
        assert_eq!(
            corrected.track_measurements.as_ref().unwrap()[0].stamp,
            time
        );
    }
}

#[test]
fn omitted_and_rejected_scans_preserve_raw_stamp_empty_scan_clears_it() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let before = pipeline.step(&frame(0., false)).unwrap().track_measurements;
    let mut missing = frame(0.05, true);
    missing.lidar = None;
    assert_eq!(pipeline.step(&missing).unwrap().track_measurements, before);
    let mut invalid = frame(0.1, true);
    invalid.lidar.as_mut().unwrap().points[0].x = f64::NAN;
    let rejected = pipeline.step(&invalid).unwrap();
    assert!(rejected.emergency);
    assert_eq!(rejected.track_measurements, before);
    let mut empty = frame(0.15, false);
    empty.lidar.as_mut().unwrap().points.clear();
    let accepted = pipeline.step(&empty).unwrap();
    assert!(accepted.track_measurements.unwrap().is_empty());
    assert_eq!(accepted.tracks[0].last_seen, 0.);
    let lost = pipeline.step(&frame(0.5, false)).unwrap();
    assert!(lost.motion_discontinuity.is_some());
    assert!(lost.track_measurements.unwrap().is_empty());
    assert!(lost.tracks.is_empty());
}

#[test]
fn raw_sensor_replay_requires_the_flag_and_complete_acquisition_evidence() {
    let config = config();
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("confirmed-track-motion", config);
    for time in [0., 0.1, 0.2, 0.25] {
        let input = frame(time, false);
        let output = driver.step(&input).unwrap();
        log.record(input, output);
    }
    let check = |log: &SensorLog| {
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        verify(std::io::Cursor::new(bytes), vec![]).is_ok_and(|r| r.verified)
    };
    assert!(check(&log));
    log.header.config.confirm_track_initialization = false;
    assert!(!check(&log));
    log.header.config.confirm_track_initialization = true;
    log.ticks[2].expected.track_measurements.as_mut().unwrap()[0]
        .center
        .x += 0.1;
    assert!(!check(&log));
}
