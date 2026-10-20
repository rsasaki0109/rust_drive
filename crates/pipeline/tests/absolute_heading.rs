use rustdriving_core::{
    AbsoluteHeading, Gnss, HeadingDecision, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig,
};
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.absolute_heading_fusion = true;
    c
}
fn sample(t: f64, yaw: f64) -> AbsoluteHeading {
    AbsoluteHeading {
        stamp: t,
        yaw_rad: yaw,
        variance_rad2: 1e-4,
    }
}
fn frame(t: f64, heading: Option<AbsoluteHeading>) -> SensorFrame {
    SensorFrame {
        time: t,
        absolute_heading: heading,
        odometry: Some(Odometry {
            stamp: t,
            speed: 0.,
            yaw_rate: 0.,
        }),
        gnss: Some(Gnss {
            stamp: t,
            position: Vec2::default(),
            variance: 0.02,
        }),
        lidar: Some(LidarScan {
            stamp: t,
            points: vec![],
        }),
        multi_height_lidar: None,
        lidar3d: None,
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}

#[test]
fn a_required_heading_stream_that_never_starts_cannot_launch_the_vehicle() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for i in 0..10 {
        let out = driver.step(&frame(i as f64 * 0.05, None)).unwrap();
        assert!(out.health.contains(&HealthIssue::StaleAbsoluteHeading));
        assert!(out.emergency);
        assert_eq!(out.command.acceleration, -6.);
        assert_eq!(out.command.steering, 0.);
        assert_eq!(out.absolute_heading.unwrap().last_accepted_stamp, None);
    }
}

#[test]
fn fresh_heading_is_required_without_extending_gnss_freshness() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for i in 0usize..=20 {
        let t = i as f64 * 0.05;
        let mut input = frame(t, i.is_multiple_of(4).then(|| sample(t, 0.)));
        if i > 0 {
            input.gnss = None;
        }
        let out = driver.step(&input).unwrap();
        assert_eq!(out.localization.unwrap().last_accepted_stamp, Some(0.));
        if t >= 0.8 {
            assert!(out.health.contains(&HealthIssue::StaleGnss));
            assert!(!out.health.contains(&HealthIssue::StaleAbsoluteHeading));
            assert!(out.command.acceleration <= -5.99);
        }
    }
}

#[test]
fn lost_heading_expires_even_with_fresh_gnss_and_old_duplicates() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for i in 0..=8 {
        let t = i as f64 * 0.05;
        let out = driver.step(&frame(t, Some(sample(0., 0.)))).unwrap();
        assert_eq!(out.absolute_heading.unwrap().accepted_samples, 1);
        assert_eq!(out.absolute_heading.unwrap().last_accepted_stamp, Some(0.));
        assert!(!out.health.contains(&HealthIssue::InvalidAbsoluteHeading));
        assert_eq!(
            out.health.contains(&HealthIssue::StaleAbsoluteHeading),
            i == 8
        );
        if i == 8 {
            assert!(out.command.acceleration <= -5.99);
        }
    }
}

#[test]
fn an_innovation_fault_holds_through_old_samples_until_a_new_accepted_acquisition() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0., Some(sample(0., 0.)))).unwrap();
    let rejected = driver.step(&frame(0.05, Some(sample(0.05, 0.8)))).unwrap();
    assert_eq!(
        rejected.absolute_heading.unwrap().last_decision,
        Some(HeadingDecision::RejectedInnovation)
    );
    assert!(
        rejected
            .health
            .contains(&HealthIssue::HeadingInnovationHold)
    );
    assert!(rejected.command.acceleration <= -5.99);
    let old = driver.step(&frame(0.1, Some(sample(0.05, 0.)))).unwrap();
    assert!(old.health.contains(&HealthIssue::HeadingInnovationHold));
    assert_eq!(old.absolute_heading.unwrap().last_accepted_stamp, Some(0.));
    let restored = driver.step(&frame(0.15, Some(sample(0.15, 0.)))).unwrap();
    assert!(restored.health.is_empty());
    assert_eq!(restored.absolute_heading.unwrap().accepted_samples, 2);
}

#[test]
fn malformed_delayed_and_future_samples_cannot_change_the_accepted_clock() {
    for bad in [
        sample(0.025, 0.),
        sample(0.1, 0.),
        AbsoluteHeading {
            variance_rad2: f64::NAN,
            ..sample(0.05, 0.)
        },
    ] {
        let mut driver = DrivingPipeline::new(config()).unwrap();
        driver.step(&frame(0., Some(sample(0., 0.)))).unwrap();
        let out = driver.step(&frame(0.05, Some(bad))).unwrap();
        assert!(out.health.contains(&HealthIssue::InvalidAbsoluteHeading));
        assert_eq!(out.absolute_heading.unwrap().last_accepted_stamp, Some(0.));
        let old = driver.step(&frame(0.1, Some(sample(0., 0.)))).unwrap();
        assert!(old.health.contains(&HealthIssue::InvalidAbsoluteHeading));
        assert!(
            driver
                .step(&frame(0.15, Some(sample(0.15, 0.))))
                .unwrap()
                .health
                .is_empty()
        );
    }
}

#[test]
fn unsupported_history_and_unconfigured_measurements_fail_closed() {
    let mut c = config();
    c.gnss_latency_compensation = true;
    assert!(DrivingPipeline::new(c).is_err());
    let mut c = config();
    c.absolute_heading_fusion = false;
    assert!(
        serde_json::to_value(&c)
            .unwrap()
            .get("absolute_heading_fusion")
            .is_none()
    );
    let mut driver = DrivingPipeline::new(c).unwrap();
    let out = driver.step(&frame(0., Some(sample(0., 0.)))).unwrap();
    assert!(out.health.contains(&HealthIssue::InvalidAbsoluteHeading));
    assert!(out.absolute_heading.is_none());
    assert!(out.pose_covariance.is_none());
}

#[test]
fn heading_cannot_cover_a_missing_rigid_motion_interval() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    driver.step(&frame(0., Some(sample(0., 0.)))).unwrap();
    let mut gap = frame(0.2, Some(sample(0.2, 0.)));
    gap.odometry = None;
    let out = driver.step(&gap).unwrap();
    assert!(out.health.contains(&HealthIssue::MotionDiscontinuity));
    assert_eq!(out.absolute_heading.unwrap().accepted_samples, 1);
    assert!(out.command.acceleration <= -5.99);
}

#[test]
fn sensor_only_replay_reconstructs_heading_and_rejects_changed_data_or_policy() {
    let c = config();
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("absolute-heading-fixture", c);
    for i in 0..10 {
        let t = i as f64 * 0.05;
        let input = frame(t, Some(sample(t, 0.02)));
        let output = driver.step(&input).unwrap();
        assert!(output.pose_covariance.is_some());
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes.clone()), vec![])
            .unwrap()
            .verified
    );
    let text = String::from_utf8(bytes).unwrap();
    for changed in [
        text.replacen("\"yaw_rad\":0.02", "\"yaw_rad\":0.2", 1),
        text.replacen(
            "\"absolute_heading_fusion\":true",
            "\"absolute_heading_fusion\":false",
            1,
        ),
    ] {
        assert_ne!(changed, text);
        assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
    }
}
