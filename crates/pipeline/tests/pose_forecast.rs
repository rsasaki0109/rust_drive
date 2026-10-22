use rustdriving_core::{
    AbsoluteHeading, ForecastEnvelopeConfig, Gnss, LidarScan, Odometry, Pose, Route, Vec2,
    VehicleConfig,
};
use rustdriving_pipeline::{
    DrivingPipeline, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.absolute_heading_fusion = true;
    c.enclose_track_measurements = true;
    c.forecast_consistency_guard = true;
    c.forecast_envelope = Some(ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    });
    c
}
fn frame(i: usize) -> SensorFrame {
    let time = i as f64 * 0.05;
    SensorFrame {
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
        absolute_heading: i.is_multiple_of(4).then_some(AbsoluteHeading {
            stamp: time,
            yaw_rad: 0.,
            variance_rad2: 0.0001,
        }),
        lidar: i.is_multiple_of(2).then(|| LidarScan {
            stamp: time,
            sweep: None,
            points: (0..48)
                .map(|k| {
                    let a = std::f64::consts::TAU * k as f64 / 48.;
                    Vec2::new(15. + a.cos(), a.sin())
                })
                .collect(),
        }),
        multi_height_lidar: None,
        lidar3d: None,
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}

#[test]
fn world_pose_occupancy_preserves_tracking_and_the_acquired_continuous_guard() {
    let c = config();
    let mut old = DrivingPipeline::new(c.clone()).unwrap();
    let mut added = c;
    added.forecast_pose_sigma = Some(3.);
    let mut p = DrivingPipeline::new(added.clone()).unwrap();
    let mut log = SensorLog::new("pose-forecast", added);
    let mut cached_radius = None;
    for i in 0..16 {
        let input = frame(i);
        let before = old.step(&input).unwrap();
        let output = p.step(&input).unwrap();
        assert_eq!(
            serde_json::to_value(&before.tracks).unwrap(),
            serde_json::to_value(&output.tracks).unwrap()
        );
        assert_eq!(
            serde_json::to_value(before.estimate).unwrap(),
            serde_json::to_value(output.estimate).unwrap()
        );
        assert_eq!(
            serde_json::to_value(&before.forecast_consistency).unwrap(),
            serde_json::to_value(&output.forecast_consistency).unwrap()
        );
        for (a, b) in before.predictions.iter().zip(&output.predictions) {
            assert_eq!(a.positions, b.positions);
            assert_eq!(a.radius, b.radius);
            assert!(
                a.additional_radius_m
                    .iter()
                    .zip(&b.additional_radius_m)
                    .all(|(x, y)| y > x)
            );
        }
        if i == 0 {
            cached_radius = Some(output.predictions[0].additional_radius_m[0]);
        }
        if i == 1 {
            assert_eq!(output.tracks[0].last_seen, 0.);
            assert_ne!(
                Some(output.predictions[0].additional_radius_m[0]),
                cached_radius
            );
        }
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(&bytes), vec![])
            .unwrap()
            .verified
    );
    let original = String::from_utf8(bytes).unwrap();
    let changed = original.replacen(
        "\"forecast_pose_sigma\":3.0",
        "\"forecast_pose_sigma\":1.0",
        1,
    );
    assert_ne!(changed, original);
    assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
}

#[test]
fn pose_forecast_requires_declared_heading_and_motion_support() {
    for sigma in [0., 0.9, 5.1, f64::INFINITY, f64::NAN] {
        let mut c = config();
        c.forecast_pose_sigma = Some(sigma);
        assert!(c.validate().is_err());
    }
    let mut c = config();
    c.forecast_pose_sigma = Some(3.);
    c.absolute_heading_fusion = false;
    assert!(c.validate().is_err());
    c.absolute_heading_fusion = true;
    c.forecast_envelope = None;
    assert!(c.validate().is_err());
    assert!(
        !serde_json::to_value(config())
            .unwrap()
            .as_object()
            .unwrap()
            .contains_key("forecast_pose_sigma")
    );
}
