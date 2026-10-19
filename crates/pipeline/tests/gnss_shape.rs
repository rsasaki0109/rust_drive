use rustdriving_core::{Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};
use std::io::Cursor;

fn config() -> PipelineConfig {
    PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(60., 0.)], 3.).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    )
}
fn frame(t: f64) -> SensorFrame {
    serde_json::from_value(serde_json::json!({"time":t,"odometry":{"stamp":t,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":t,"position":{"x":0.1,"y":-0.03},"variance":0.02},"lidar":{"stamp":t,"points":[]},"lidar_failed":false})).unwrap()
}

#[test]
fn gnss_shape_alone_publishes_full_covariance_and_replay_requires_its_calibration() {
    let mut c = config();
    c.gnss_covariance_shape = Some([[1., 0.9], [0.9, 1.]]);
    let mut pipeline = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("gnss-shape-contract", c);
    for i in 0..8 {
        let input = frame(i as f64 * 0.05);
        let output = pipeline.step(&input).unwrap();
        let p = output.pose_covariance.unwrap();
        assert!((p[0][0] + p[1][1] - output.position_variance).abs() < 1e-14);
        assert!(p[0][1] > 0.);
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(verify(Cursor::new(&bytes), vec![]).unwrap().verified);
    log.header.config.gnss_covariance_shape = None;
    let mut changed = vec![];
    log.write(&mut changed).unwrap();
    assert!(verify(Cursor::new(changed), vec![]).is_err());
    log.header.config.gnss_covariance_shape = Some([[1., 0.9], [0.9, 1.]]);
    log.ticks[3].expected.pose_covariance.as_mut().unwrap()[0][1] += 0.001;
    let mut corrupted = vec![];
    log.write(&mut corrupted).unwrap();
    assert!(verify(Cursor::new(corrupted), vec![]).is_err());
}

#[test]
fn invalid_shapes_fail_before_pipeline_creation_and_default_serialization_is_unchanged() {
    for shape in [
        [[1., 1.], [1., 1.]],
        [[1., 0.1], [0., 1.]],
        [[2., 0.], [0., 1.]],
        [[f64::INFINITY, 0.], [0., 1.]],
    ] {
        let mut c = config();
        c.gnss_covariance_shape = Some(shape);
        assert!(c.validate().is_err());
        assert!(DrivingPipeline::new(c).is_err());
    }
    let c = config();
    assert!(
        serde_json::to_value(&c)
            .unwrap()
            .get("gnss_covariance_shape")
            .is_none()
    );
    assert!(
        DrivingPipeline::new(c)
            .unwrap()
            .step(&frame(0.))
            .unwrap()
            .pose_covariance
            .is_none()
    );
}

#[test]
fn finite_scalar_whose_shaped_noise_overflows_holds_braking_until_new_accepted_fix() {
    let mut c = config();
    c.gnss_covariance_shape = Some([[1.9, 0.], [0., 0.1]]);
    let mut driver = DrivingPipeline::new(c).unwrap();
    driver.step(&frame(0.)).unwrap();
    let mut bad = frame(0.05);
    bad.gnss.as_mut().unwrap().variance = f64::MAX;
    let out = driver.step(&bad).unwrap();
    assert!(
        out.health
            .contains(&rustdriving_pipeline::HealthIssue::InvalidGnss)
    );
    assert_eq!(out.command.acceleration, -6.);
    assert_eq!(out.localization.unwrap().last_accepted_stamp, Some(0.));
    let mut missing = frame(0.1);
    missing.gnss = None;
    assert_eq!(driver.step(&missing).unwrap().command.acceleration, -6.);
    let recovered = driver.step(&frame(0.15)).unwrap();
    assert!(recovered.health.is_empty());
    assert_eq!(
        recovered.localization.unwrap().last_accepted_stamp,
        Some(0.15)
    );
}
