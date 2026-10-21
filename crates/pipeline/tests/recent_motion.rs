use rustdriving_core::{Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.fit_recent_track_motion = true;
    c.enclose_track_measurements = true;
    c
}

fn frame(time: f64, corrected: bool) -> SensorFrame {
    serde_json::from_value(serde_json::json!({
        "time":time,"odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":time,"position":{"x":0.,"y":if corrected {0.1} else {0.}},"variance":0.02},
        "lidar":{"stamp":time,"points":(0..8).map(|i| {
            let a=std::f64::consts::PI-0.5+i as f64/7.;
            Vec2::new(30.+4.*time+a.cos(),4.+a.sin())
        }).collect::<Vec<_>>()}
    }))
    .unwrap()
}

#[test]
fn recent_raw_motion_does_not_absorb_absolute_localization_corrections() {
    let mut plain = DrivingPipeline::new(config()).unwrap();
    let mut corrected = DrivingPipeline::new(config()).unwrap();
    for time in [0., 0.1, 0.2, 0.3, 0.4] {
        let a = plain.step(&frame(time, false)).unwrap();
        let b = corrected.step(&frame(time, true)).unwrap();
        assert_eq!(a.track_measurements, b.track_measurements);
        assert!(a.tracks[0].velocity.distance(b.tracks[0].velocity) < 1e-9);
        if time >= 0.2 {
            assert!(a.tracks[0].velocity.distance(Vec2::new(4., 0.)) < 1e-9);
        }
    }
}

#[test]
fn replay_binds_the_motion_policy_and_every_reported_track() {
    let c = config();
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("recent-acquired-track-motion", c);
    for time in [0., 0.1, 0.2, 0.3] {
        let input = frame(time, false);
        let out = driver.step(&input).unwrap();
        log.record(input, out);
    }
    let check = |log: &SensorLog| {
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        verify(std::io::Cursor::new(bytes), vec![]).is_ok_and(|r| r.verified)
    };
    assert!(check(&log));
    log.header.config.fit_recent_track_motion = false;
    assert!(!check(&log));
    log.header.config.fit_recent_track_motion = true;
    log.ticks[2].expected.tracks[0].velocity.x += 0.1;
    assert!(!check(&log));
}

#[test]
fn invalid_policy_compositions_reject_and_an_omitted_policy_remains_disabled() {
    let mut c = config();
    c.enclose_track_measurements = false;
    assert!(c.validate().is_err());
    c.enclose_track_measurements = true;
    c.confirm_track_initialization = true;
    assert!(c.validate().is_err());
    c.confirm_track_initialization = false;
    c.fit_recent_track_motion = false;
    let value = serde_json::to_value(c).unwrap();
    assert!(value.get("fit_recent_track_motion").is_none());
    let decoded: PipelineConfig = serde_json::from_value(value).unwrap();
    assert!(!decoded.fit_recent_track_motion);
}

#[test]
fn delayed_turning_observations_fit_in_the_acquisition_frame() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    let center = Vec2::new(40., 3.);
    for i in 0..=6 {
        let mut input = frame(i as f64 * 0.05, false);
        input.lidar = None;
        let odometry = input.odometry.as_mut().unwrap();
        odometry.speed = 4.;
        odometry.yaw_rate = 0.8;
        if i > 0 {
            input.gnss = None;
        }
        if i >= 2 && i % 2 == 0 {
            let acquired = i - 2;
            let mut position = Vec2::default();
            for j in 0..acquired {
                let yaw = j as f64 * 0.04;
                position = position.plus(Vec2::new(yaw.cos(), yaw.sin()).scaled(0.2));
            }
            input.lidar = Some(rustdriving_core::LidarScan {
                stamp: acquired as f64 * 0.05,
                sweep: None,
                points: (0..8)
                    .map(|j| {
                        let a = std::f64::consts::PI - 0.5 + j as f64 / 7.;
                        center
                            .plus(Vec2::new(a.cos(), a.sin()))
                            .minus(position)
                            .rotated(-(acquired as f64 * 0.04))
                    })
                    .collect(),
            });
        }
        let out = driver.step(&input).unwrap();
        if i >= 2 {
            assert!(out.health.is_empty());
            assert!(out.tracks[0].position.distance(center) < 1e-8);
            assert!(out.tracks[0].velocity.x.hypot(out.tracks[0].velocity.y) < 1e-8);
            assert!(out.tracks[0].last_seen <= input.time - 0.1 + 1e-9);
        }
    }
}
