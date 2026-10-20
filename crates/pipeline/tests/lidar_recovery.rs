use rustdriving_core::*;
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, PipelineConfig, PipelineOutput, SensorFrame,
    replay::{SensorLog, verify},
};
use std::io::Cursor;

fn config() -> PipelineConfig {
    PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    )
}
fn frame(time: f64, acquisition: Option<f64>) -> SensorFrame {
    SensorFrame {
        absolute_heading: None,
        time,
        odometry: Some(Odometry {
            stamp: time,
            speed: 0.0,
            yaw_rate: 0.0,
        }),
        gnss: Some(Gnss {
            stamp: time,
            position: Vec2::default(),
            variance: 0.02,
        }),
        lidar: acquisition.map(|stamp| LidarScan {
            stamp,
            points: vec![],
        }),
        multi_height_lidar: None,
        lidar3d: None,
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}
fn held(output: &PipelineOutput) {
    assert!(output.health.contains(&HealthIssue::InvalidLidar));
    assert!(output.emergency);
    assert_eq!(output.command.acceleration, -6.0);
    assert!(output.command.finite());
}
#[test]
fn acquisition_failure_waits_for_a_valid_post_fault_acquisition() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    assert!(!driver.step(&frame(0.0, Some(0.0))).unwrap().emergency);
    let mut failed = frame(0.05, None);
    failed.lidar_failed = true;
    assert!(driver.step(&failed).unwrap().emergency);
    // The pre-fault observation is still within its normal age limit.
    for (time, stamp) in [
        (0.1, None),
        (0.15, Some(0.0)),
        (0.2, Some(0.025)),
        (0.25, Some(0.05)),
    ] {
        let output = driver.step(&frame(time, stamp)).unwrap();
        held(&output);
        assert!(!output.health.contains(&HealthIssue::StaleLidar));
    }
    // A delayed but post-fault empty successful scan is enough to recover.
    let output = driver.step(&frame(0.3, Some(0.1))).unwrap();
    assert!(output.health.is_empty());
    assert!(!output.emergency);
}
#[test]
fn malformed_ordinary_returns_latch_without_changing_accepted_observations() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, Some(0.0))).unwrap();
    let before = driver.occupied_cells();
    let mut invalid = frame(0.05, Some(0.05));
    invalid
        .lidar
        .as_mut()
        .unwrap()
        .points
        .push(Vec2::new(201.0, 0.0));
    held(&driver.step(&invalid).unwrap());
    held(&driver.step(&frame(0.1, None)).unwrap());
    held(&driver.step(&frame(0.15, Some(0.05))).unwrap());
    assert_eq!(driver.occupied_cells(), before);
    assert!(!driver.step(&frame(0.2, Some(0.2))).unwrap().emergency);
}
#[test]
fn simultaneous_success_cannot_clear_failure_and_new_failures_move_the_cutoff() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, Some(0.0))).unwrap();
    let mut failed = frame(0.05, Some(0.05));
    failed.lidar_failed = true;
    assert!(driver.step(&failed).unwrap().emergency);
    held(&driver.step(&frame(0.1, Some(0.05))).unwrap());
    let mut failed = frame(0.15, None);
    failed.lidar_failed = true;
    assert!(driver.step(&failed).unwrap().emergency);
    held(&driver.step(&frame(0.2, Some(0.1))).unwrap());
    held(&driver.step(&frame(0.25, Some(0.15))).unwrap());
    assert!(!driver.step(&frame(0.3, Some(0.25))).unwrap().emergency);
}
#[test]
fn unconfigured_or_mixed_representations_require_a_new_ordinary_scan() {
    for mixed in [false, true] {
        let mut driver = DrivingPipeline::new(config()).unwrap();
        driver.step(&frame(0.0, Some(0.0))).unwrap();
        let mut invalid = frame(0.05, mixed.then_some(0.05));
        invalid.lidar3d = Some(Lidar3dScan {
            stamp: 0.05,
            returns: vec![],
        });
        held(&driver.step(&invalid).unwrap());
        held(&driver.step(&frame(0.1, None)).unwrap());
        assert!(!driver.step(&frame(0.15, Some(0.15))).unwrap().emergency);
    }
}
#[test]
fn missing_acquisition_pose_latches_until_a_transformable_post_fault_scan() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    held(&driver.step(&frame(1.0, Some(0.9))).unwrap());
    held(&driver.step(&frame(1.05, None)).unwrap());
    let recovered = driver.step(&frame(1.1, Some(1.05))).unwrap();
    assert!(recovered.health.is_empty());
    assert!(!recovered.emergency);
}
#[test]
fn sensor_only_replay_reconstructs_the_hold_and_rejects_pre_fault_recovery() {
    let config = config();
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("ordinary-lidar-fault-recovery", config);
    let mut failure = frame(0.05, None);
    failure.lidar_failed = true;
    let inputs = [
        frame(0.0, Some(0.0)),
        failure,
        frame(0.1, None),
        frame(0.15, Some(0.05)),
        frame(0.2, Some(0.1)),
    ];
    for (index, input) in inputs.into_iter().enumerate() {
        let output = driver.step(&input).unwrap();
        assert_eq!(output.emergency, (1..=3).contains(&index));
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert_eq!(
        verify(Cursor::new(bytes), std::io::sink()).unwrap().ticks,
        5
    );
    log.ticks[4].input.lidar.as_mut().unwrap().stamp = 0.05;
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
