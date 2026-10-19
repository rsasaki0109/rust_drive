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
        "gnss":{"stamp":t,"position":{"x":0.05,"y":-0.03},"variance":0.02},"lidar":{"stamp":t,"points":[]},"lidar_failed":false})).unwrap()
}
#[test]
fn supplied_prior_is_recorded_published_and_checked_on_every_replay_tick() {
    let mut c = config();
    c.initial_pose_covariance = Some([
        [0.25, 0.05, 0.005],
        [0.05, 0.17, -0.005],
        [0.005, -0.005, 0.000725],
    ]);
    let mut pipeline = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("prior-contract", c);
    for i in 0..8 {
        let input = frame(i as f64 * 0.05);
        let output = pipeline.step(&input).unwrap();
        let p = output.pose_covariance.unwrap();
        assert!((p[0][0] + p[1][1] - output.position_variance).abs() < 1e-14);
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(verify(Cursor::new(&bytes), vec![]).unwrap().verified);
    log.header.config.initial_pose_covariance = None;
    let mut changed = vec![];
    log.write(&mut changed).unwrap();
    assert!(verify(Cursor::new(changed), vec![]).is_err());
    log.header.config.initial_pose_covariance = Some([
        [0.25, 0.05, 0.005],
        [0.05, 0.17, -0.005],
        [0.005, -0.005, 0.000725],
    ]);
    log.ticks[3].expected.pose_covariance.as_mut().unwrap()[0][2] += 0.001;
    let mut corrupted = vec![];
    log.write(&mut corrupted).unwrap();
    assert!(verify(Cursor::new(corrupted), vec![]).is_err());
    log.ticks[3].expected.pose_covariance.as_mut().unwrap()[1][1] = f64::NAN;
    assert!(log.write(vec![]).is_err());
}
#[test]
fn malformed_priors_fail_at_configuration_and_absent_priors_preserve_legacy_json() {
    for covariance in [
        [[1., 0., 0.], [0., 1., 0.], [0., 0., 0.]],
        [[1., 2., 0.], [2., 1., 0.], [0., 0., 0.03]],
        [[1., 0.1, 0.], [0., 1., 0.], [0., 0., 0.03]],
        [[f64::NAN, 0., 0.], [0., 1., 0.], [0., 0., 0.03]],
    ] {
        let mut c = config();
        c.initial_pose_covariance = Some(covariance);
        assert!(c.validate().is_err());
        assert!(DrivingPipeline::new(c).is_err());
    }
    let c = config();
    assert!(
        serde_json::to_value(&c)
            .unwrap()
            .get("initial_pose_covariance")
            .is_none()
    );
    let out = DrivingPipeline::new(c).unwrap().step(&frame(0.)).unwrap();
    assert!(
        serde_json::to_value(out)
            .unwrap()
            .get("pose_covariance")
            .is_none()
    );
}
