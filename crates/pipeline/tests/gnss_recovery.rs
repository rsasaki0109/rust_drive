use rustdriving_core::*;
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, MapLocalizationConfig, MapLocalizationDecision, PipelineConfig,
    PipelineOutput, SensorFrame,
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
        odometry: Some(Odometry {
            stamp: time,
            speed: 0.0,
            yaw_rate: 0.0,
        }),
        gnss: acquisition.map(|stamp| Gnss {
            stamp,
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
    input.gnss.as_mut().unwrap().variance = 0.0;
    input
}
fn held(output: &PipelineOutput) {
    assert!(output.health.contains(&HealthIssue::InvalidGnss));
    assert!(output.emergency);
    assert_eq!(output.trajectory.mode, DrivingMode::Emergency);
    assert_eq!(output.command.acceleration, -6.0);
    assert_eq!(output.command.steering, 0.0);
    assert!(output.command.finite());
}

#[test]
fn malformed_gnss_holds_until_a_valid_post_fault_fix_is_accepted() {
    for case in 0..9 {
        let mut driver = DrivingPipeline::new(config()).unwrap();
        let mut steering = 0.0;
        for i in 0..=10 {
            let time = i as f64 * 0.05;
            let output = driver.step(&frame(time, Some(time))).unwrap();
            assert!(output.health.is_empty());
            steering = output.command.steering;
        }
        assert!(steering.abs() > 0.035);
        let mut invalid = frame(0.55, Some(0.55));
        let fix = invalid.gnss.as_mut().unwrap();
        match case {
            0 => fix.variance = 0.0,
            1 => fix.variance = -0.01,
            2 => fix.variance = f64::NAN,
            3 => fix.variance = f64::INFINITY,
            4 => fix.position.x = f64::NAN,
            5 => fix.position.y = f64::INFINITY,
            6 => fix.stamp = -1.0,
            7 => fix.stamp = 2.0,
            _ => fix.stamp = f64::NAN,
        }
        held(&driver.step(&invalid).unwrap());
        let missing = driver.step(&frame(0.6, None)).unwrap();
        held(&missing);
        assert!(!missing.health.contains(&HealthIssue::StaleGnss));
        assert_eq!(missing.localization.unwrap().last_accepted_stamp, Some(0.5));
        let recovered = driver.step(&frame(0.65, Some(0.6))).unwrap();
        assert!(recovered.health.is_empty());
        assert!(!recovered.emergency);
        assert_eq!(
            recovered.localization.unwrap().last_accepted_stamp,
            Some(0.6)
        );
        assert!(recovered.command.steering.abs() <= 0.7 * 0.05 + 1e-12);
    }
}

#[test]
fn pre_fault_duplicate_and_expired_fixes_cannot_update_estimates_or_diagnostics() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    let mut reference = DrivingPipeline::new(config()).unwrap();
    for pipeline in [&mut driver, &mut reference] {
        pipeline.step(&frame(0.0, Some(0.0))).unwrap();
        held(&pipeline.step(&failed(0.05)).unwrap());
    }
    for i in 2..=20 {
        let time = i as f64 * 0.05;
        let stamp = match i {
            2 => Some(0.0),
            3 => Some(0.025),
            4 => Some(0.05),
            18 => Some(0.1),
            _ => None,
        };
        let mut input = frame(time, stamp);
        if let Some(fix) = &mut input.gnss {
            fix.position = Vec2::new(0.05, 0.02);
        }
        let actual = driver.step(&input).unwrap();
        let expected = reference.step(&frame(time, None)).unwrap();
        held(&actual);
        assert!(
            serde_json::to_vec(&actual).unwrap() == serde_json::to_vec(&expected).unwrap(),
            "rejected fix changed state at {time}"
        );
        let diagnostic = actual.localization.unwrap();
        assert_eq!(diagnostic.accepted_fixes, 1);
        assert_eq!(diagnostic.last_observed_stamp, Some(0.0));
        assert_eq!(actual.estimate.pose.position, Vec2::default());
    }
    let recovered = driver.step(&frame(1.05, Some(1.0))).unwrap();
    assert!(recovered.health.is_empty());
    assert!(!recovered.emergency);
}

#[test]
fn new_failures_advance_the_cutoff_and_innovation_rejection_cannot_clear_it() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, Some(0.0))).unwrap();
    held(&driver.step(&failed(0.05)).unwrap());
    let mut second = failed(0.1);
    second.gnss.as_mut().unwrap().stamp = 0.0;
    held(&driver.step(&second).unwrap());
    held(&driver.step(&frame(0.15, Some(0.075))).unwrap());
    held(&driver.step(&frame(0.2, Some(0.1))).unwrap());
    for (time, stamp, innovation_hold) in [(0.25, 0.2, false), (0.3, 0.3, true)] {
        let mut outlier = frame(time, Some(stamp));
        outlier.gnss.as_mut().unwrap().position = Vec2::new(30.0, -25.0);
        let output = driver.step(&outlier).unwrap();
        held(&output);
        assert_eq!(
            output.localization.unwrap().last_decision,
            Some(GnssDecision::RejectedInnovation)
        );
        assert_eq!(
            output.health.contains(&HealthIssue::GnssInnovationHold),
            innovation_hold
        );
    }
    held(&driver.step(&frame(0.35, None)).unwrap());
    let recovered = driver.step(&frame(0.4, Some(0.35))).unwrap();
    assert!(recovered.health.is_empty());
    assert!(!recovered.emergency);
    assert_eq!(
        recovered.localization.unwrap().last_decision,
        Some(GnssDecision::Accepted)
    );
}

#[test]
fn ordinary_missing_fixes_and_one_outlier_keep_the_existing_grace() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0.0, Some(0.0))).unwrap();
    for i in 1..=15 {
        let time = i as f64 * 0.05;
        let mut input = frame(time, None);
        if i == 4 {
            input.gnss = Some(Gnss {
                stamp: time,
                position: Vec2::new(30.0, -25.0),
                variance: 0.02,
            });
        }
        let output = driver.step(&input).unwrap();
        assert!(output.health.is_empty());
        assert!(!output.emergency);
    }
    let stale = driver.step(&frame(0.8, None)).unwrap();
    assert!(stale.health.contains(&HealthIssue::StaleGnss));
    assert!(!stale.health.contains(&HealthIssue::InvalidGnss));
    assert!(stale.emergency);
    // Staleness alone does not impose a malformed-fix recovery cutoff.
    let recovered = driver.step(&frame(0.85, Some(0.7))).unwrap();
    assert!(recovered.health.is_empty());
    assert!(!recovered.emergency);
}

#[test]
fn accepted_map_localization_cannot_clear_a_malformed_gnss_hold() {
    let points: Vec<_> = (0..36)
        .map(|i| {
            Vec2::new(
                4.0 + 0.7 * i as f64,
                4.0 * (1.7 * i as f64).sin() + 0.03 * i as f64,
            )
        })
        .collect();
    let mut config = config();
    config.localization_map = Some(MapLocalizationConfig {
        points: points.clone(),
        ..MapLocalizationConfig::default()
    });
    let mut driver = DrivingPipeline::new(config).unwrap();
    for i in 0..=20 {
        let time = i as f64 * 0.05;
        let mut input = match i {
            0 => frame(time, Some(time)),
            1 => failed(time),
            _ => frame(time, None),
        };
        input.lidar.as_mut().unwrap().points = points.clone();
        let output = driver.step(&input).unwrap();
        let map = output.map_localization.as_ref().unwrap();
        assert_eq!(map.decision, MapLocalizationDecision::Accepted);
        if i > 0 {
            held(&output);
        }
        if i == 20 {
            assert!(map.gnss_outage_covered);
            assert!(!output.health.contains(&HealthIssue::StaleGnss));
            assert_eq!(output.localization.unwrap().last_accepted_stamp, Some(0.0));
        }
    }
    let mut input = frame(1.05, Some(1.05));
    input.lidar.as_mut().unwrap().points = points;
    let recovered = driver.step(&input).unwrap();
    assert!(recovered.health.is_empty());
    assert_eq!(
        recovered.localization.unwrap().last_accepted_stamp,
        Some(1.05)
    );
}

#[test]
fn replay_reconstructs_the_gnss_hold_and_rejects_a_pre_fault_recovery() {
    let config = config();
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("gnss-fault-recovery", config);
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
    log.ticks[4].input.gnss.as_mut().unwrap().stamp = 0.05;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
