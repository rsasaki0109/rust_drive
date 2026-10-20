use rustdriving_core::*;
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, PipelineConfig, PipelineOutput, SensorFrame,
    replay::{SensorLog, verify},
};
use std::io::Cursor;

fn config() -> PipelineConfig {
    PipelineConfig::new(
        Route::new(
            vec![
                Vec2::default(),
                Vec2::new(20.0, 10.0),
                Vec2::new(100.0, 10.0),
            ],
            5.5,
        )
        .unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    )
}
fn frame(time: f64, acquisition: Option<f64>) -> SensorFrame {
    SensorFrame {
        absolute_heading: None,
        time,
        odometry: acquisition.map(|stamp| Odometry {
            stamp,
            speed: 0.0,
            yaw_rate: 0.0,
        }),
        gnss: Some(Gnss {
            stamp: time,
            position: Vec2::default(),
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
fn failed(time: f64) -> SensorFrame {
    let mut input = frame(time, Some(time));
    input.odometry.as_mut().unwrap().speed = 21.0;
    input
}
fn held(output: &PipelineOutput) {
    assert!(output.health.contains(&HealthIssue::InvalidOdometry));
    assert!(output.emergency);
    assert_eq!(output.trajectory.mode, DrivingMode::Emergency);
    assert_eq!(output.command.acceleration, -6.0);
    assert_eq!(output.command.steering, 0.0);
    assert!(output.command.finite());
}

#[test]
fn malformed_odometry_holds_until_a_valid_post_fault_acquisition() {
    for case in 0..8 {
        let mut driver = DrivingPipeline::new(config()).unwrap();
        let mut last_steering = 0.0;
        for i in 0..=10 {
            let time = i as f64 * 0.05;
            let output = driver.step(&frame(time, Some(time))).unwrap();
            assert!(output.health.is_empty());
            last_steering = output.command.steering;
        }
        assert!(last_steering.abs() > 0.035);
        let mut invalid = frame(0.55, Some(0.55));
        let odom = invalid.odometry.as_mut().unwrap();
        match case {
            0 => odom.speed = 21.0,
            1 => odom.speed = -1.0,
            2 => odom.speed = f64::NAN,
            3 => odom.yaw_rate = 4.0,
            4 => odom.yaw_rate = f64::NAN,
            5 => odom.stamp = -1.0,
            6 => odom.stamp = 1.0,
            _ => odom.stamp = f64::INFINITY,
        }
        held(&driver.step(&invalid).unwrap());
        let missing = driver.step(&frame(0.6, None)).unwrap();
        held(&missing);
        assert!(!missing.health.contains(&HealthIssue::StaleOdometry));
        let recovered = driver.step(&frame(0.65, Some(0.6))).unwrap();
        assert!(recovered.health.is_empty());
        assert!(!recovered.emergency);
        assert!(recovered.command.steering.abs() <= 0.7 * 0.05 + 1e-12);
    }
}

#[test]
fn pre_fault_duplicate_and_expired_samples_cannot_replace_accepted_motion() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    let mut reference = DrivingPipeline::new(config()).unwrap();
    for pipeline in [&mut driver, &mut reference] {
        pipeline.step(&frame(0.0, Some(0.0))).unwrap();
        held(&pipeline.step(&failed(0.05)).unwrap());
    }
    for (time, stamp) in [(0.1, 0.0), (0.125, 0.025), (0.15, 0.05), (0.3, 0.1)] {
        let mut input = frame(time, Some(stamp));
        let odom = input.odometry.as_mut().unwrap();
        odom.speed = 10.0;
        odom.yaw_rate = 1.0;
        let actual = driver.step(&input).unwrap();
        let expected = reference.step(&frame(time, None)).unwrap();
        held(&actual);
        assert!(
            serde_json::to_vec(&actual).unwrap() == serde_json::to_vec(&expected).unwrap(),
            "rejected observation changed state at {time}"
        );
        assert_eq!(actual.estimate.pose.position, Vec2::default());
        assert_eq!(actual.estimate.pose.yaw, 0.0);
        assert_eq!(actual.estimate.speed, 0.0);
    }
    held(&driver.step(&frame(0.35, None)).unwrap());
    let recovered = driver.step(&frame(0.4, Some(0.3))).unwrap();
    assert!(recovered.health.contains(&HealthIssue::MotionDiscontinuity));
    assert!(recovered.emergency);
}

#[test]
fn each_new_invalid_observation_advances_the_recovery_cutoff() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, Some(0.0))).unwrap();
    held(&driver.step(&failed(0.05)).unwrap());
    let mut second = frame(0.1, Some(0.0));
    second.odometry.as_mut().unwrap().yaw_rate = 4.0;
    held(&driver.step(&second).unwrap());
    held(&driver.step(&frame(0.125, Some(0.1))).unwrap());
    held(&driver.step(&frame(0.15, Some(0.075))).unwrap());
    let recovered = driver.step(&frame(0.175, Some(0.15))).unwrap();
    assert!(recovered.health.is_empty());
    assert!(!recovered.emergency);
}

#[test]
fn ordinary_missing_samples_keep_grace_but_an_uncovered_interval_latches() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, Some(0.0))).unwrap();
    for time in [0.05, 0.1, 0.15] {
        let output = driver.step(&frame(time, None)).unwrap();
        assert!(output.health.is_empty());
        assert!(!output.emergency);
    }
    let stale = driver.step(&frame(0.2, None)).unwrap();
    assert!(stale.health.contains(&HealthIssue::StaleOdometry));
    assert!(!stale.health.contains(&HealthIssue::InvalidOdometry));
    assert!(stale.emergency);
    // A new sample cannot retroactively cover the missing motion interval.
    let recovered = driver.step(&frame(0.25, Some(0.15))).unwrap();
    assert!(recovered.health.contains(&HealthIssue::MotionDiscontinuity));
    assert!(recovered.emergency);
}

#[test]
fn replay_reconstructs_the_odometry_hold_and_rejects_a_pre_fault_recovery() {
    let config = config();
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("odometry-fault-recovery", config);
    let inputs = [
        frame(0.0, Some(0.0)),
        failed(0.05),
        frame(0.1, None),
        frame(0.15, Some(0.05)),
        frame(0.2, Some(0.1)),
    ];
    for (index, input) in inputs.into_iter().enumerate() {
        let output = driver.step(&input).unwrap();
        assert_eq!(output.emergency, (1..=3).contains(&index));
        log.record(input, output);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert_eq!(
        verify(Cursor::new(bytes), std::io::sink()).unwrap().ticks,
        5
    );
    log.ticks[4].input.odometry.as_mut().unwrap().stamp = 0.05;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
