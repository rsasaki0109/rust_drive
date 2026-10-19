use rustdriving_core::{Gnss, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    let mut config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(120., 0.)], 3.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    config.gnss_latency_compensation = true;
    config
}

fn frame(time: f64, fix: Option<f64>) -> SensorFrame {
    serde_json::from_value(serde_json::json!({
        "time":time,
        "odometry":Odometry{stamp:time,speed:0.,yaw_rate:0.},
        "gnss":fix.map(|stamp| Gnss{stamp,position:Vec2::default(),variance:0.02}),
        "lidar":{"stamp":time,"points":[]},
        "lidar_failed":false
    }))
    .unwrap()
}

#[test]
fn unavailable_history_latches_fault_until_a_covered_post_fault_acquisition() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    assert!(
        pipeline
            .step(&frame(0., Some(0.)))
            .unwrap()
            .health
            .is_empty()
    );
    for tick in 1..=12 {
        let now = tick as f64 * 0.05;
        let output = pipeline
            .step(&frame(now, (tick == 12).then_some(0.05)))
            .unwrap();
        if tick == 12 {
            assert!(output.health.contains(&HealthIssue::InvalidGnss));
            assert!(output.command.acceleration <= -6.);
            assert_eq!(output.gnss_latency.unwrap().history_unavailable_fixes, 1);
            assert_eq!(output.localization.unwrap().last_accepted_stamp, Some(0.));
        }
    }
    let quarantined = pipeline.step(&frame(0.65, Some(0.4))).unwrap();
    assert!(quarantined.health.contains(&HealthIssue::InvalidGnss));
    for now in [0.7, 0.75] {
        pipeline.step(&frame(now, None)).unwrap();
    }
    let recovered = pipeline.step(&frame(0.8, Some(0.7))).unwrap();
    assert!(recovered.health.is_empty(), "{:?}", recovered.health);
    assert_eq!(
        recovered.localization.unwrap().last_accepted_stamp,
        Some(0.7)
    );
    assert_eq!(recovered.gnss_latency.unwrap().compensated_fixes, 1);
}

#[test]
fn uncovered_clock_gap_is_not_replayed_and_a_fresh_covered_fix_recovers() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    pipeline.step(&frame(0., Some(0.))).unwrap();
    let gap = pipeline.step(&frame(1., Some(0.9))).unwrap();
    assert!(gap.health.contains(&HealthIssue::ClockGap));
    assert!(gap.health.contains(&HealthIssue::InvalidGnss));
    assert_eq!(gap.estimate.pose.position, Vec2::default());
    assert!(
        pipeline
            .step(&frame(1.05, Some(1.02)))
            .unwrap()
            .health
            .is_empty()
    );
}

#[test]
fn duplicate_and_future_fixes_cannot_refresh_accepted_acquisition_time() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    pipeline.step(&frame(0., Some(0.))).unwrap();
    let duplicate = pipeline.step(&frame(0.05, Some(0.))).unwrap();
    assert_eq!(duplicate.localization.unwrap().accepted_fixes, 1);
    let future = pipeline.step(&frame(0.1, Some(0.2))).unwrap();
    assert!(future.health.contains(&HealthIssue::InvalidGnss));
    let old = pipeline.step(&frame(0.15, Some(0.05))).unwrap();
    assert!(old.health.contains(&HealthIssue::InvalidGnss));
    assert_eq!(old.localization.unwrap().last_accepted_stamp, Some(0.));
    assert!(
        pipeline
            .step(&frame(0.2, Some(0.2)))
            .unwrap()
            .health
            .is_empty()
    );
}

#[test]
fn zero_delay_preserves_every_existing_output_bit_except_added_diagnostics() {
    let mut enabled = DrivingPipeline::new(config()).unwrap();
    let mut ordinary = config();
    ordinary.gnss_latency_compensation = false;
    let mut ordinary = DrivingPipeline::new(ordinary).unwrap();
    for tick in 0..100 {
        let now = tick as f64 * 0.05;
        let mut input = frame(now, (tick % 4 == 0).then_some(now));
        input.odometry = Some(Odometry {
            stamp: now,
            speed: 2.,
            yaw_rate: 0.1,
        });
        if let Some(fix) = &mut input.gnss {
            fix.position = Vec2::new(2. * now, 0.1 * now * now);
        }
        let mut actual = serde_json::to_value(enabled.step(&input).unwrap()).unwrap();
        actual.as_object_mut().unwrap().remove("gnss_latency");
        assert_eq!(
            actual,
            serde_json::to_value(ordinary.step(&input).unwrap()).unwrap()
        );
    }
}

fn map_config() -> PipelineConfig {
    let mut combined = config();
    combined.localization_map = Some(rustdriving_pipeline::MapLocalizationConfig {
        points: (0..36)
            .map(|i| {
                Vec2::new(
                    4. + 0.7 * i as f64,
                    4. * (1.7 * i as f64).sin() + 0.03 * i as f64,
                )
            })
            .collect(),
        max_gnss_outage_s: 1.,
        ..Default::default()
    });
    combined
}

fn map_frame(config: &PipelineConfig, now: f64, fix: Option<f64>) -> SensorFrame {
    let mut input = frame(now, fix);
    input.lidar.as_mut().unwrap().points = config.localization_map.as_ref().unwrap().points.clone();
    input
}

#[test]
fn delayed_fixes_and_map_outage_permission_keep_original_acquisition_clocks_in_replay() {
    use rustdriving_pipeline::replay::{SensorLog, verify};
    let config = map_config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("delayed-gnss-with-map", config.clone());
    for tick in 0..=29 {
        let now = tick as f64 * 0.05;
        let fix = match tick {
            0 => Some(0.),
            7 => Some(0.1),
            9 => Some(0.2),
            29 => Some(1.3),
            _ => None,
        };
        let input = map_frame(&config, now, fix);
        let output = pipeline.step(&input).unwrap();
        let map = output.map_localization.as_ref().unwrap();
        assert_eq!(map.last_accepted_stamp, Some(now));
        if (10..=28).contains(&tick) {
            assert_eq!(
                output.localization.as_ref().unwrap().last_accepted_stamp,
                Some(0.2)
            );
            assert_eq!(map.gnss_outage_covered, (20..=24).contains(&tick));
        }
        if (25..=28).contains(&tick) {
            assert!(output.health.contains(&HealthIssue::StaleGnss));
            assert_eq!(output.command.acceleration, -6.);
        } else {
            assert!(output.health.is_empty(), "{:?}", output.health);
        }
        log.record(input, output);
    }
    assert!(
        log.ticks
            .last()
            .unwrap()
            .expected
            .gnss_latency
            .unwrap()
            .replayed_map_corrections
            > 0
    );
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), std::io::sink())
            .unwrap()
            .verified
    );
    // Suppressing a new map acquisition does not let historical replay renew it.
    let mut missing = map_frame(&config, 1.5, Some(1.4));
    missing.lidar = None;
    let output = pipeline.step(&missing).unwrap();
    assert!(
        (output
            .map_localization
            .unwrap()
            .last_accepted_stamp
            .unwrap()
            - 1.45)
            .abs()
            < 1e-12
    );
}

#[test]
fn map_conflict_latches_gnss_fault_despite_fresh_map_and_requires_post_fault_acquisition() {
    let config = map_config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    pipeline.step(&map_frame(&config, 0., None)).unwrap();
    let mut conflict = map_frame(&config, 0.1, Some(0.));
    conflict.gnss.as_mut().unwrap().position.x = 5.;
    conflict.gnss.as_mut().unwrap().variance = 0.001;
    let output = pipeline.step(&conflict).unwrap();
    assert!(output.health.contains(&HealthIssue::InvalidGnss));
    assert_eq!(output.command.acceleration, -6.);
    assert_eq!(output.gnss_latency.unwrap().history_conflict_fixes, 1);
    assert_eq!(output.localization.unwrap().accepted_fixes, 0);
    assert_eq!(
        output.map_localization.unwrap().last_accepted_stamp,
        Some(0.1)
    );
    let quarantined = pipeline
        .step(&map_frame(&config, 0.15, Some(0.05)))
        .unwrap();
    assert!(quarantined.health.contains(&HealthIssue::InvalidGnss));
    assert_eq!(quarantined.localization.unwrap().accepted_fixes, 0);
    let recovered = pipeline.step(&map_frame(&config, 0.2, Some(0.15))).unwrap();
    assert!(recovered.health.is_empty());
    assert_eq!(
        recovered.localization.unwrap().last_accepted_stamp,
        Some(0.15)
    );
    assert_eq!(recovered.gnss_latency.unwrap().history_conflict_fixes, 1);
}

#[test]
fn zero_delay_map_fusion_preserves_every_existing_output_value() {
    let config = map_config();
    let mut ordinary_config = config.clone();
    ordinary_config.gnss_latency_compensation = false;
    let mut enabled = DrivingPipeline::new(config.clone()).unwrap();
    let mut ordinary = DrivingPipeline::new(ordinary_config).unwrap();
    for tick in 0..=40 {
        let input = map_frame(
            &config,
            tick as f64 * 0.05,
            (tick % 4 == 0).then_some(tick as f64 * 0.05),
        );
        let mut actual = serde_json::to_value(enabled.step(&input).unwrap()).unwrap();
        actual.as_object_mut().unwrap().remove("gnss_latency");
        assert_eq!(
            actual,
            serde_json::to_value(ordinary.step(&input).unwrap()).unwrap()
        );
    }
}

#[test]
fn fresh_map_replay_cannot_clear_repeated_gnss_innovation_rejection() {
    let config = map_config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    pipeline.step(&map_frame(&config, 0., Some(0.))).unwrap();
    for tick in 1..=14 {
        let now = tick as f64 * 0.05;
        let fix = match tick {
            6 => Some(0.1),
            8 => Some(0.2),
            14 => Some(0.5),
            _ => None,
        };
        let mut input = map_frame(&config, now, fix);
        if [6, 8].contains(&tick) {
            input.gnss.as_mut().unwrap().position = Vec2::new(30., -25.);
        }
        let out = pipeline.step(&input).unwrap();
        assert_eq!(out.map_localization.unwrap().last_accepted_stamp, Some(now));
        if (8..14).contains(&tick) {
            assert!(out.health.contains(&HealthIssue::GnssInnovationHold));
            assert_eq!(out.command.acceleration, -6.);
            assert_eq!(out.localization.unwrap().last_accepted_stamp, Some(0.));
        } else {
            assert!(!out.health.contains(&HealthIssue::GnssInnovationHold));
        }
    }
}
