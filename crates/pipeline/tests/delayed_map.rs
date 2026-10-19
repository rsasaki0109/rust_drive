use rustdriving_core::{Gnss, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, MapLocalizationConfig, MapLocalizationDecision, PipelineConfig,
    SensorFrame,
    replay::{SensorLog, verify},
};

fn config() -> PipelineConfig {
    let mut config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    config.gnss_latency_compensation = true;
    config.localization_map = Some(MapLocalizationConfig {
        points: (0..36)
            .map(|i| {
                Vec2::new(
                    4. + 0.7 * i as f64,
                    4. * (1.7 * i as f64).sin() + 0.03 * i as f64,
                )
            })
            .collect(),
        allow_delayed_scans: true,
        ..Default::default()
    });
    config
}
fn frame(config: &PipelineConfig, now: f64, scan: Option<f64>, fix: Option<f64>) -> SensorFrame {
    serde_json::from_value(serde_json::json!({
        "time":now,"odometry":Odometry{stamp:now,speed:3.,yaw_rate:0.},
        "gnss":fix.map(|stamp|Gnss{stamp,position:Vec2::new(3.*stamp,0.),variance:0.02}),
        "lidar":scan.map(|stamp|serde_json::json!({"stamp":stamp,"points":config.localization_map.as_ref().unwrap().points.iter().map(|p|Vec2::new(p.x-3.*stamp,p.y)).collect::<Vec<_>>()})),
        "lidar_failed":false
    })).unwrap()
}

#[test]
fn delayed_scan_registration_uses_acquisition_pose_and_replays_both_gnss_arrival_orders() {
    for lag in [0, 8] {
        let config = config();
        let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
        let mut log = SensorLog::new("asynchronous-map-and-gnss", config.clone());
        for tick in 0..=28 {
            let now = tick as f64 * 0.05;
            let fix = (tick >= lag && (tick - lag) % 4 == 0).then(|| (tick - lag) as f64 * 0.05);
            let scan = (tick >= 2 && tick % 2 == 0).then(|| (tick - 2) as f64 * 0.05);
            let input = frame(&config, now, scan, fix);
            let output = pipeline.step(&input).unwrap();
            assert!(
                output
                    .estimate
                    .pose
                    .position
                    .distance(Vec2::new(3. * now, 0.))
                    < 1e-8
            );
            if let Some(stamp) = scan {
                let map = output.map_localization.as_ref().unwrap();
                assert_eq!(map.decision, MapLocalizationDecision::Accepted, "{map:?}");
                assert_eq!(map.last_accepted_stamp, Some(stamp));
                assert!((map.acquisition_delay_s.unwrap() - 0.1).abs() < 1e-12);
                assert!(
                    map.acquisition_prior
                        .unwrap()
                        .position
                        .distance(Vec2::new(3. * stamp, 0.))
                        < 1e-8
                );
            }
            log.record(input, output);
        }
        let latency = log.ticks.last().unwrap().expected.gnss_latency.unwrap();
        if lag == 0 {
            assert!(latency.replayed_gnss_corrections > 0);
        } else {
            assert!(latency.replayed_map_corrections > 0);
        }
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        assert!(
            verify(std::io::Cursor::new(bytes), std::io::sink())
                .unwrap()
                .verified
        );
    }
}

#[test]
fn queued_pre_fault_map_cannot_recover_lidar_and_a_new_acquisition_can() {
    let config = config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    for tick in 0..=8 {
        let now = tick as f64 * 0.05;
        pipeline
            .step(&frame(&config, now, Some(now), Some(now)))
            .unwrap();
    }
    let mut failure = frame(&config, 0.5, None, Some(0.5));
    failure.lidar_failed = true;
    let output = pipeline.step(&failure).unwrap();
    assert!(output.emergency);
    assert_eq!(output.command.acceleration, -6.);
    let old = pipeline
        .step(&frame(&config, 0.55, Some(0.4), Some(0.55)))
        .unwrap();
    assert!(old.health.contains(&HealthIssue::InvalidLidar));
    assert_eq!(old.map_localization.unwrap().last_accepted_stamp, Some(0.4));
    assert_eq!(old.command.acceleration, -6.);
    let fresh = pipeline
        .step(&frame(&config, 0.6, Some(0.55), Some(0.6)))
        .unwrap();
    assert!(fresh.health.is_empty(), "{:?}", fresh.health);
    assert_eq!(
        fresh.map_localization.unwrap().last_accepted_stamp,
        Some(0.55)
    );
}

#[test]
fn a_clock_gap_cannot_seed_registration_from_uncovered_history() {
    let config = config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let stationary = |now, scan, fix| {
        let mut input = frame(&config, now, scan, fix);
        input.odometry.as_mut().unwrap().speed = 0.;
        if let Some(gnss) = &mut input.gnss {
            gnss.position = Vec2::default();
        }
        if let Some(scan) = &mut input.lidar {
            scan.points = config.localization_map.as_ref().unwrap().points.clone();
        }
        input
    };
    pipeline.step(&stationary(0., Some(0.), Some(0.))).unwrap();
    let gap = pipeline.step(&stationary(1., Some(0.9), Some(1.))).unwrap();
    assert!(gap.health.contains(&HealthIssue::ClockGap));
    assert!(gap.emergency);
    let map = gap.map_localization.unwrap();
    assert_eq!(map.decision, MapLocalizationDecision::UnsupportedTiming);
    assert_eq!(map.last_accepted_stamp, Some(0.));
    let old = pipeline
        .step(&stationary(1.05, Some(1.), Some(1.05)))
        .unwrap();
    assert!(old.health.contains(&HealthIssue::InvalidLidar));
    let fresh = pipeline
        .step(&stationary(1.1, Some(1.05), Some(1.1)))
        .unwrap();
    assert!(fresh.health.is_empty(), "{:?}", fresh.health);
    assert_eq!(
        fresh.map_localization.unwrap().last_accepted_stamp,
        Some(1.05)
    );
}

#[test]
fn lidar_age_valid_for_perception_cannot_extend_the_stricter_map_freshness_limit() {
    let config = config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    for tick in 0..=16 {
        let now = tick as f64 * 0.05;
        let scan = (tick <= 8).then_some(now);
        pipeline
            .step(&frame(&config, now, scan, (tick == 0).then_some(0.)))
            .unwrap();
    }
    let out = pipeline
        .step(&frame(&config, 0.85, Some(0.6), None))
        .unwrap();
    assert!(out.health.contains(&HealthIssue::StaleGnss));
    assert!(!out.health.contains(&HealthIssue::StaleLidar));
    assert!(!out.health.contains(&HealthIssue::InvalidLidar));
    assert_eq!(out.command.acceleration, -6.);
    let map = out.map_localization.unwrap();
    assert_eq!(map.decision, MapLocalizationDecision::UnsupportedTiming);
    assert_eq!(map.last_accepted_stamp, Some(0.4));
    assert!(!map.gnss_outage_covered);
}

#[test]
fn delayed_map_mode_requires_explicit_event_history_and_default_serialization_is_unchanged() {
    let mut config = config();
    config.gnss_latency_compensation = false;
    assert!(
        config
            .validate()
            .unwrap_err()
            .contains("require GNSS correction-event history")
    );
    config
        .localization_map
        .as_mut()
        .unwrap()
        .allow_delayed_scans = false;
    assert!(config.validate().is_ok());
    assert!(
        serde_json::to_value(config.localization_map.unwrap())
            .unwrap()
            .get("allow_delayed_scans")
            .is_none()
    );
}
