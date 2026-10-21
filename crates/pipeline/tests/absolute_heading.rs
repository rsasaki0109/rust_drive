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

fn delayed_config() -> PipelineConfig {
    let mut c = config();
    c.gnss_latency_compensation = true;
    c.heading_latency_compensation = true;
    c
}

#[test]
fn delayed_heading_requires_explicit_sensor_and_measured_history_policies() {
    for (heading, history) in [(false, false), (false, true), (true, false)] {
        let mut c = delayed_config();
        c.absolute_heading_fusion = heading;
        c.gnss_latency_compensation = history;
        assert!(DrivingPipeline::new(c).is_err());
    }
    assert!(
        serde_json::to_value(config())
            .unwrap()
            .get("heading_latency_compensation")
            .is_none()
    );
}

#[test]
fn delayed_heading_owns_acquisition_clocks_and_replays_without_policy_or_data_changes() {
    let c = delayed_config();
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("delayed-heading", c);
    for i in 0usize..=24 {
        let now = i as f64 * 0.05;
        let h = (i >= 4 && (i - 4).is_multiple_of(4)).then(|| sample((i - 4) as f64 * 0.05, 0.01));
        let mut input = frame(now, h);
        input.gnss = (i >= 1 && (i - 1).is_multiple_of(4)).then(|| Gnss {
            stamp: (i - 1) as f64 * 0.05,
            position: Vec2::default(),
            variance: 0.02,
        });
        let out = driver.step(&input).unwrap();
        let diagnostics = out.absolute_heading.unwrap();
        assert_eq!(diagnostics.accepted_samples, (i / 4) as u64);
        assert_eq!(
            diagnostics.last_accepted_stamp,
            (i >= 4).then(|| ((i / 4 - 1) * 4) as f64 * 0.05)
        );
        if i >= 4 {
            assert!(out.health.is_empty(), "{:?}", out.health);
        }
        log.record(input, out);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes.clone()), vec![])
            .unwrap()
            .verified
    );
    let original = String::from_utf8(bytes).unwrap();
    for changed in [
        original.replacen(
            "\"heading_latency_compensation\":true",
            "\"heading_latency_compensation\":false",
            1,
        ),
        original.replacen("\"yaw_rad\":0.01", "\"yaw_rad\":0.1", 1),
    ] {
        assert_ne!(changed, original);
        assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
    }
}

#[test]
fn delayed_heading_cannot_clear_a_fault_with_a_pre_fault_acquisition_or_hide_staleness() {
    let mut driver = DrivingPipeline::new(delayed_config()).unwrap();
    driver.step(&frame(0., Some(sample(0., 0.)))).unwrap();
    let bad = driver.step(&frame(0.05, Some(sample(0.025, 0.8)))).unwrap();
    assert!(bad.health.contains(&HealthIssue::HeadingInnovationHold));
    let old = driver.step(&frame(0.1, Some(sample(0.05, 0.)))).unwrap();
    assert_eq!(
        old.absolute_heading.unwrap().last_accepted_stamp,
        Some(0.05)
    );
    assert!(old.health.contains(&HealthIssue::HeadingInnovationHold));
    assert!(old.command.acceleration <= -5.99);
    let fresh = driver.step(&frame(0.15, Some(sample(0.1, 0.)))).unwrap();
    assert!(fresh.health.is_empty());
    for i in 4..=10 {
        let out = driver.step(&frame(i as f64 * 0.05, None)).unwrap();
        if i == 10 {
            assert!(out.health.contains(&HealthIssue::StaleAbsoluteHeading));
            assert!(out.command.acceleration <= -5.99);
            assert_eq!(out.absolute_heading.unwrap().last_accepted_stamp, Some(0.1));
        }
    }
}

#[test]
fn unsupported_heading_delay_and_motion_loss_brake_without_accepting_the_sample() {
    for gap in [false, true] {
        let mut driver = DrivingPipeline::new(delayed_config()).unwrap();
        driver.step(&frame(0., Some(sample(0., 0.)))).unwrap();
        let mut input = frame(0.25, Some(sample(0.01, 0.)));
        if gap {
            input.odometry = None;
        }
        let out = driver.step(&input).unwrap();
        assert!(out.command.acceleration <= -5.99);
        assert_eq!(out.absolute_heading.unwrap().accepted_samples, 1);
        assert!(out.health.contains(&if gap {
            HealthIssue::MotionDiscontinuity
        } else {
            HealthIssue::InvalidAbsoluteHeading
        }));
    }
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
fn heading_history_is_supported_and_unconfigured_measurements_fail_closed() {
    let mut c = config();
    c.gnss_latency_compensation = true;
    assert!(DrivingPipeline::new(c).is_ok());
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

#[test]
fn delayed_gnss_retains_heading_clocks_covariance_and_complete_sensor_replay() {
    let mut c = config();
    c.gnss_latency_compensation = true;
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("heading-gnss-history-fixture", c);
    for i in 0usize..=24 {
        let now = i as f64 * 0.05;
        let mut input = frame(now, i.is_multiple_of(4).then(|| sample(now, 0.02)));
        input.gnss = (i >= 3 && (i - 3).is_multiple_of(4)).then(|| Gnss {
            stamp: (i - 3) as f64 * 0.05,
            position: Vec2::default(),
            variance: 0.02,
        });
        let out = driver.step(&input).unwrap();
        let heading = out.absolute_heading.unwrap();
        assert_eq!(heading.accepted_samples, (i / 4 + 1) as u64);
        assert_eq!(heading.last_accepted_stamp, Some((i / 4 * 4) as f64 * 0.05));
        assert!(out.pose_covariance.is_some());
        if i >= 3 {
            assert!(out.health.is_empty(), "{:?}", out.health);
        }
        if i == 24 {
            assert!(out.gnss_latency.unwrap().replayed_heading_corrections > 0);
            assert_eq!(out.localization.unwrap().accepted_fixes, 6);
            assert_eq!(out.localization.unwrap().last_accepted_stamp, Some(1.));
        }
        log.record(input, out);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes.clone()), vec![])
            .unwrap()
            .verified
    );
    let original = String::from_utf8(bytes).unwrap();
    let changed = original.replacen(
        "\"gnss_latency_compensation\":true",
        "\"gnss_latency_compensation\":false",
        1,
    );
    assert_ne!(original, changed);
    assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
}

#[test]
fn gnss_rewinds_cannot_renew_a_lost_required_heading() {
    let mut c = config();
    c.gnss_latency_compensation = true;
    let mut driver = DrivingPipeline::new(c).unwrap();
    for i in 0usize..=12 {
        let now = i as f64 * 0.05;
        let mut input = frame(now, (i == 0).then(|| sample(0., 0.)));
        input.gnss = (i >= 2 && i.is_multiple_of(4)).then(|| Gnss {
            stamp: (i - 2) as f64 * 0.05,
            position: Vec2::default(),
            variance: 0.02,
        });
        let out = driver.step(&input).unwrap();
        assert_eq!(out.absolute_heading.unwrap().accepted_samples, 1);
        assert_eq!(out.absolute_heading.unwrap().last_accepted_stamp, Some(0.));
        if i >= 8 {
            assert!(out.health.contains(&HealthIssue::StaleAbsoluteHeading));
            assert!(!out.health.contains(&HealthIssue::StaleGnss));
            assert!(out.command.acceleration <= -5.99);
        }
    }
}

#[test]
fn delayed_gnss_cannot_withdraw_a_heading_innovation_hold() {
    let mut c = config();
    c.gnss_latency_compensation = true;
    let mut driver = DrivingPipeline::new(c).unwrap();
    driver.step(&frame(0., Some(sample(0., 0.)))).unwrap();
    let bad = driver.step(&frame(0.05, Some(sample(0.05, 0.8)))).unwrap();
    assert!(bad.health.contains(&HealthIssue::HeadingInnovationHold));
    let mut input = frame(0.1, Some(sample(0., 0.)));
    input.gnss.as_mut().unwrap().stamp = 0.025;
    let held = driver.step(&input).unwrap();
    assert!(held.health.contains(&HealthIssue::HeadingInnovationHold));
    assert_eq!(held.absolute_heading.unwrap().rejected_samples, 1);
    assert_eq!(held.absolute_heading.unwrap().last_accepted_stamp, Some(0.));
    assert!(held.command.acceleration <= -5.99);
    let resumed = driver.step(&frame(0.15, Some(sample(0.15, 0.)))).unwrap();
    assert!(resumed.health.is_empty());
}

#[test]
fn conflicting_gnss_rewind_brakes_without_changing_observation_ownership() {
    let mut c = config();
    c.gnss_latency_compensation = true;
    c.initial_pose_covariance = Some([[0.1, 0., 0.], [0., 0.1, 0.04], [0., 0.04, 0.03]]);
    let mut driver = DrivingPipeline::new(c).unwrap();
    let mut start = frame(0., None);
    start.gnss = None;
    driver.step(&start).unwrap();
    let mut acquired = frame(0.05, Some(sample(0.05, 0.)));
    acquired.gnss = None;
    driver.step(&acquired).unwrap();
    let mut input = frame(0.1, None);
    input.gnss = Some(Gnss {
        stamp: 0.,
        position: Vec2::new(0., 1.2),
        variance: 0.001,
    });
    let held = driver.step(&input).unwrap();
    assert!(held.health.contains(&HealthIssue::InvalidGnss));
    assert!(held.command.acceleration <= -5.99);
    assert_eq!(held.gnss_latency.unwrap().history_conflict_fixes, 1);
    assert_eq!(held.localization.unwrap().accepted_fixes, 0);
    assert_eq!(held.localization.unwrap().last_observed_stamp, None);
    assert_eq!(held.absolute_heading.unwrap().accepted_samples, 1);
    assert_eq!(
        held.absolute_heading.unwrap().last_accepted_stamp,
        Some(0.05)
    );
    let recovered = driver.step(&frame(0.15, Some(sample(0.15, 0.)))).unwrap();
    assert!(recovered.health.is_empty());
}
