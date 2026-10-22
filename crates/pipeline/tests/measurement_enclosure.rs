use rustdriving_core::{Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    let mut config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    config.enclose_track_measurements = true;
    config
}

fn frame(time: f64, corrected: bool) -> SensorFrame {
    let radius = if time == 0. { 0.5 } else { 1. };
    serde_json::from_value(serde_json::json!({
        "time":time,"odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":time,"position":{"x":0.,"y":if corrected {0.1} else {0.}},"variance":0.02},
        "lidar":{"stamp":time,"points":(0..32).map(|i|{
            let a=i as f64*std::f64::consts::TAU/32.;
            Vec2::new(20.+4.*time+radius*a.cos(),4.+radius*a.sin())
        }).collect::<Vec<_>>()}
    }))
    .unwrap()
}

#[test]
fn the_world_track_contains_the_full_raw_disc_after_absolute_pose_correction() {
    let mut clean = DrivingPipeline::new(config()).unwrap();
    let mut corrected = DrivingPipeline::new(config()).unwrap();
    for time in [0., 0.1, 0.2] {
        let a = clean.step(&frame(time, false)).unwrap();
        let b = corrected.step(&frame(time, true)).unwrap();
        assert_eq!(a.track_measurements, b.track_measurements);
        let raw = &b.track_measurements.as_ref().unwrap()[0];
        let center = b.tracking_frame.unwrap().to_world(raw.center);
        for i in 0..64 {
            let angle = i as f64 * std::f64::consts::TAU / 64.;
            let boundary = center.plus(Vec2::new(
                raw.radius * angle.cos(),
                raw.radius * angle.sin(),
            ));
            assert!(boundary.distance(b.tracks[0].position) <= b.tracks[0].radius + 1e-9);
        }
        assert_eq!(b.tracks[0].last_seen, time);
    }
}

#[test]
fn cached_rejected_and_uncovered_observations_cannot_renew_the_extent() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0., false)).unwrap();
    let accepted = driver.step(&frame(0.1, false)).unwrap();
    let mut absent = frame(0.15, true);
    absent.lidar = None;
    let cached = driver.step(&absent).unwrap();
    assert_eq!(cached.track_measurements, accepted.track_measurements);
    assert_eq!(cached.tracks[0].radius, accepted.tracks[0].radius);
    let mut bad = frame(0.2, true);
    bad.lidar.as_mut().unwrap().points[0].y = f64::NAN;
    let rejected = driver.step(&bad).unwrap();
    assert!(rejected.emergency);
    assert_eq!(rejected.track_measurements, accepted.track_measurements);
    assert_eq!(rejected.tracks[0].radius, accepted.tracks[0].radius);
    let lost = driver.step(&frame(0.6, false)).unwrap();
    assert!(lost.motion_discontinuity.is_some());
    assert!(lost.track_measurements.unwrap().is_empty());
    assert!(lost.tracks.is_empty());
}

#[test]
fn sensor_replay_refuses_lost_enclosure_policy_and_shrunk_output_geometry() {
    let config = config();
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("measured-disc-enclosure", config);
    for time in [0., 0.1, 0.2] {
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
    log.header.config.enclose_track_measurements = false;
    assert!(!check(&log));
    log.header.config.enclose_track_measurements = true;
    log.ticks[1].expected.tracks[0].radius -= 0.1;
    assert!(!check(&log));
}
