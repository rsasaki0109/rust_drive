use rustdriving_core::*;
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    )
}
fn frame(time: f64) -> SensorFrame {
    serde_json::from_value(serde_json::json!({
        "time":time,"odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":time,"position":{"x":0.,"y":0.},"variance":0.02},
        "lidar":{"stamp":time,"points":(0..32).map(|i|{
            let angle=i as f64*std::f64::consts::TAU/32.;
            Vec2::new(20.+0.6*angle.cos(),4.+0.6*angle.sin())
        }).collect::<Vec<_>>()}
    }))
    .unwrap()
}

#[test]
fn uncovered_clock_gap_clears_scene_and_fresh_observations_cannot_resume_driving() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let first = pipeline.step(&frame(0.)).unwrap();
    assert!(first.health.is_empty());
    assert_eq!(first.tracks.len(), 1);
    let gap = pipeline.step(&frame(0.4)).unwrap();
    assert!(
        gap.tracks.is_empty(),
        "uncovered motion must invalidate the old scene"
    );
    assert!(gap.predictions.is_empty());
    for time in [0.45, 0.5, 0.6] {
        let output = pipeline.step(&frame(time)).unwrap();
        assert!(
            output.emergency,
            "fresh scalar sensors cannot establish the missing body motion"
        );
        assert_eq!(output.command.acceleration, -6.);
        assert_eq!(output.command.steering, 0.);
        assert!(output.tracks.is_empty());
        assert!(output.predictions.is_empty());
        assert_eq!(
            output.localization.unwrap().accepted_fixes,
            first.localization.unwrap().accepted_fixes
        );
        assert_eq!(output.position_variance, first.position_variance);
        assert!(
            serde_json::to_value(output)
                .unwrap()
                .get("motion_discontinuity")
                .is_some()
        );
    }
    // A new instance starts from an explicit externally supplied calibration.
    let mut initialized = DrivingPipeline::new(config()).unwrap();
    assert!(initialized.step(&frame(0.65)).unwrap().health.is_empty());
}

#[test]
fn cached_motion_grace_and_a_short_malformed_delivery_keep_the_existing_recovery() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    let first = p.step(&frame(0.)).unwrap();
    for time in [0.05, 0.1, 0.15] {
        let mut input = frame(time);
        input.odometry = None;
        let out = p.step(&input).unwrap();
        assert!(out.health.is_empty());
        assert_eq!(out.tracks[0].id, first.tracks[0].id);
        assert!(
            serde_json::to_value(out)
                .unwrap()
                .get("motion_discontinuity")
                .is_none()
        );
    }
    let mut p = DrivingPipeline::new(config()).unwrap();
    p.step(&frame(0.)).unwrap();
    let mut bad = frame(0.05);
    bad.odometry.as_mut().unwrap().speed = 21.;
    assert!(p.step(&bad).unwrap().emergency);
    assert!(p.step(&frame(0.1)).unwrap().health.is_empty());
}

#[test]
fn an_expired_motion_cache_requires_reinitialization_even_after_new_odometry() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    p.step(&frame(0.)).unwrap();
    for time in [0.05, 0.1, 0.15, 0.2] {
        let mut f = frame(time);
        f.odometry = None;
        p.step(&f).unwrap();
    }
    let out = p.step(&frame(0.25)).unwrap();
    assert!(
        out.emergency,
        "a new wheel sample cannot reconstruct the uncovered interval"
    );
    assert!(out.tracks.is_empty());
}

#[test]
fn replay_checks_the_latch_epoch_and_preserves_the_complete_pre_gap_posterior() {
    use rustdriving_pipeline::replay::{SensorLog, verify};
    use std::io::Cursor;
    let mut calibration = config();
    calibration.gnss_latency_compensation = true;
    calibration.initial_pose_covariance = Some([[0.25, 0., 0.], [0., 0.36, 0.], [0., 0., 0.04]]);
    let mut pipeline = DrivingPipeline::new(calibration.clone()).unwrap();
    let mut log = SensorLog::new("motion-discontinuity", calibration);
    for time in [0., 0.05, 0.4, 0.45, 0.5] {
        let input = frame(time);
        let output = pipeline.step(&input).unwrap();
        log.record(input, output);
    }
    for tick in &log.ticks[2..] {
        assert_eq!(
            tick.expected.pose_covariance,
            log.ticks[1].expected.pose_covariance
        );
        let gap = tick.expected.motion_discontinuity.as_ref().unwrap();
        assert_eq!(gap.last_covered_time, 0.05);
        assert_eq!(gap.detected_at, 0.4);
        assert_eq!(gap.cause, rustdriving_pipeline::HealthIssue::ClockGap);
        assert_eq!(tick.expected.gnss_latency.unwrap().history_predictions, 0);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap()
            .verified
    );
    log.ticks[4]
        .expected
        .motion_discontinuity
        .as_mut()
        .unwrap()
        .detected_at = 0.45;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
