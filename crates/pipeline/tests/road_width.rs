use rustdriving_core::{DrivingMode, Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 4.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.cruise_speed = 4.;
    c
}

#[test]
fn raw_body_lidar_selects_a_fitted_pass_and_the_header_reproduces_it_exactly() {
    let mut c = config();
    c.lateral_candidate_margin_m = Some(0.35);
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("road-width-measured-fixture", c);
    for i in 0..12 {
        let time = i as f64 * 0.05;
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
            lidar: Some(LidarScan {
                sweep: None,
                stamp: time,
                points: (0..32)
                    .map(|j| {
                        let a = j as f64 * std::f64::consts::TAU / 32.;
                        Vec2::new(35. + 0.5 * a.cos(), 0.5 * a.sin())
                    })
                    .collect(),
            }),
            lidar_failed: false,
            multi_height_lidar: None,
            lidar3d: None,
            navigation_update: None,
            traffic_signal: None,
        };
        let out = driver.step(&input).unwrap();
        assert_eq!(out.trajectory.mode, DrivingMode::Avoid);
        assert!((out.trajectory.lateral_target.abs() - 2.9).abs() < 1e-12);
        log.record(input, out);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), vec![])
            .unwrap()
            .verified
    );
    log.header.config.lateral_candidate_margin_m = None;
    let mut changed = vec![];
    log.write(&mut changed).unwrap();
    assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
}

#[test]
fn absent_and_null_reserves_preserve_old_configuration_and_bad_values_are_rejected() {
    let mut value = serde_json::to_value(config()).unwrap();
    assert!(value.get("lateral_candidate_margin_m").is_none());
    value["lateral_candidate_margin_m"] = serde_json::Value::Null;
    assert!(
        serde_json::from_value::<PipelineConfig>(value.clone())
            .unwrap()
            .lateral_candidate_margin_m
            .is_none()
    );
    value["lateral_candidate_margin_m"] = serde_json::json!(true);
    assert!(serde_json::from_value::<PipelineConfig>(value).is_err());
    for margin in [f64::NAN, f64::INFINITY, -0.1, 0., 0.09, 1.01] {
        let mut c = config();
        c.lateral_candidate_margin_m = Some(margin);
        assert!(DrivingPipeline::new(c).is_err());
    }
}
