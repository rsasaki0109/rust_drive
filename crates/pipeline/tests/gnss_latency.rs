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

#[test]
fn external_map_corrections_require_a_separate_history_contract() {
    let mut combined = config();
    combined.localization_map = Some(rustdriving_pipeline::MapLocalizationConfig {
        points: (0..25)
            .map(|i| Vec2::new((i % 5) as f64, (i / 5) as f64))
            .collect(),
        ..Default::default()
    });
    assert!(
        combined
            .validate()
            .unwrap_err()
            .contains("cannot replay external map")
    );
    combined.gnss_latency_compensation = false;
    assert!(combined.validate().is_ok());
}
