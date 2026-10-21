use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

#[test]
fn raw_delayed_slow_returns_predict_motion_using_original_acquisition_times() {
    let mut config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 2.1).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    config.confirm_slow_motion = true;
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("measured-delayed-slow-crossing", config);
    for i in 0_usize..=40 {
        let time = i as f64 * 0.05;
        let lidar = i.is_multiple_of(2).then(|| {
            let stamp = if i == 0 { 0. } else { time - 0.1 };
            LidarScan {
                sweep: None,
                stamp,
                points: (0..32)
                    .map(|j| {
                        let angle = j as f64 * std::f64::consts::TAU / 32.;
                        Vec2::new(
                            30. + 0.7 * angle.cos(),
                            5. - 0.5 * stamp + 0.7 * angle.sin(),
                        )
                    })
                    .collect(),
            }
        });
        let input = SensorFrame {
            absolute_heading: None,
            time,
            odometry: Some(Odometry {
                stamp: time,
                speed: 0.,
                yaw_rate: 0.,
            }),
            gnss: Some(Gnss {
                stamp: time,
                position: Vec2::default(),
                variance: 0.02,
            }),
            lidar,
            multi_height_lidar: None,
            lidar3d: None,
            lidar_failed: false,
            navigation_update: None,
            traffic_signal: None,
        };
        let output = driver.step(&input).unwrap();
        log.record(input, output);
    }
    let last = &log.ticks.last().unwrap().expected;
    assert_eq!(last.tracks.len(), 1);
    let track = &last.tracks[0];
    assert!((track.last_seen - 1.9).abs() < 1e-12);
    assert!((track.velocity.y + 0.5).abs() < 0.03);
    assert!(last.predictions[0].positions[40].distance(last.predictions[0].positions[0]) > 3.);
    for (i, p) in last.predictions[0].positions.iter().enumerate() {
        let expected = track
            .position
            .plus(track.velocity.scaled(0.1 + i as f64 * 0.2));
        assert!(p.distance(expected) < 1e-8);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), vec![]).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 41);
    // Replay must honor the explicit policy and reject expectations obtained
    // with the experimental mode when the recorded mode is switched off.
    let mut changed = log.clone();
    changed.header.config.confirm_slow_motion = false;
    let mut bytes = vec![];
    changed.write(&mut bytes).unwrap();
    assert!(verify(std::io::Cursor::new(bytes), vec![]).is_err());
}
