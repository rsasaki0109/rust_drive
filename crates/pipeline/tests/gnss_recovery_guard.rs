use rustdriving_core::{GnssDecision, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, GnssRecoveryConfig, HealthIssue, PipelineConfig, RecoveryPhase, SensorFrame,
    replay::{SensorLog, verify},
};
use std::io::Cursor;

fn config(latency: bool, retained: bool) -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(120., 0.)], 3.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.gnss_latency_compensation = latency;
    c.gnss_recovery = Some(GnssRecoveryConfig {
        max_position_correction_m: 0.4,
        max_yaw_correction_rad: 0.1,
        confirmation_fixes: 3,
        max_hold_s: 5.,
    });
    if retained {
        c.localization_map=Some(serde_json::from_value(serde_json::json!({"points":(0..36).map(|i|Vec2::new(i as f64,10.)).collect::<Vec<_>>(),"allow_delayed_scans":true})).unwrap());
    }
    c
}
fn frame(now: f64, fix: Option<(f64, f64)>) -> SensorFrame {
    serde_json::from_value(serde_json::json!({"time":now,"odometry":{"stamp":now,"speed":0.,"yaw_rate":0.},"gnss":fix.map(|(stamp,x)|serde_json::json!({"stamp":stamp,"position":{"x":x,"y":0.},"variance":0.02})),"lidar":{"stamp":now,"points":[]},"lidar_failed":false})).unwrap()
}
fn warm(p: &mut DrivingPipeline, log: &mut SensorLog) {
    for i in 0..=8 {
        let t = i as f64 * 0.05;
        let fix = match i {
            0 => Some((t, 0.)),
            4 | 8 => Some((t, 10.)),
            _ => None,
        };
        let input = frame(t, fix);
        let out = p.step(&input).unwrap();
        log.record(input, out);
    }
}
#[test]
fn held_proposals_preserve_pose_covariance_and_both_history_modes_until_three_new_fixes() {
    for (latency, retained) in [(false, false), (true, false), (true, true)] {
        let c = config(latency, retained);
        let mut p = DrivingPipeline::new(c.clone()).unwrap();
        let mut reference = DrivingPipeline::new(c.clone()).unwrap();
        let mut log = SensorLog::new("recovery-transactions", c.clone());
        warm(&mut p, &mut log);
        warm(&mut reference, &mut SensorLog::new("reference", c));
        for i in 9..=20 {
            let t = i as f64 * 0.05;
            let stamp = t - if latency { 0.1 } else { 0. };
            let fix = match i {
                12 | 16 | 20 => Some((stamp, 0.12)),
                13 => Some((0.6 - if latency { 0.1 } else { 0. }, 0.12)),
                _ => None,
            };
            let input = frame(t, fix);
            let out = p.step(&input).unwrap();
            if i < 20 {
                let old = reference.step(&frame(t, None)).unwrap();
                assert_eq!(
                    serde_json::to_value(out.estimate).unwrap(),
                    serde_json::to_value(old.estimate).unwrap()
                );
                assert_eq!(out.pose_covariance, old.pose_covariance);
                assert_eq!(
                    serde_json::to_value(out.gnss_latency).unwrap(),
                    serde_json::to_value(old.gnss_latency).unwrap()
                );
                assert!(out.health.contains(&HealthIssue::GnssInnovationHold));
                assert_eq!(out.command.acceleration, -6.);
                assert_eq!(out.localization.unwrap().last_accepted_stamp, Some(0.));
                if i == 12 || i == 16 {
                    assert_eq!(
                        out.localization.unwrap().last_decision,
                        Some(GnssDecision::RecoveryHeld)
                    );
                }
                if i == 13 {
                    assert_eq!(out.gnss_recovery.unwrap().consecutive_candidates, 1);
                }
            } else {
                assert!(out.health.is_empty(), "{:?}", out.health);
                let d = out.gnss_recovery.unwrap();
                assert_eq!(d.phase, RecoveryPhase::Ready);
                assert_eq!(d.held_fixes, 2);
                assert_eq!(d.completed_recoveries, 1);
                assert_eq!(out.localization.unwrap().accepted_fixes, 2);
                if latency {
                    assert_eq!(out.gnss_latency.as_ref().unwrap().compensated_fixes, 1);
                }
            }
            log.record(input, out);
        }
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        assert!(verify(Cursor::new(&bytes), vec![]).unwrap().verified);
        log.ticks[12]
            .expected
            .gnss_recovery
            .as_mut()
            .unwrap()
            .held_fixes += 1;
        let mut corrupt = vec![];
        log.write(&mut corrupt).unwrap();
        assert!(verify(Cursor::new(corrupt), vec![]).is_err());
    }
}
#[test]
fn expired_hold_never_accepts_healthy_proposals_without_external_reinitialization() {
    let mut c = config(false, false);
    c.gnss_recovery.as_mut().unwrap().max_hold_s = 0.5;
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    warm(&mut p, &mut SensorLog::new("expiry", c));
    for i in 9..=36 {
        let t = i as f64 * 0.05;
        let out = p.step(&frame(t, (i % 4 == 0).then_some((t, 0.)))).unwrap();
        if t > 0.9 + 1e-9 {
            assert_eq!(out.gnss_recovery.unwrap().phase, RecoveryPhase::Expired);
            assert_eq!(out.localization.unwrap().last_accepted_stamp, Some(0.));
            assert_eq!(out.command.acceleration, -6.);
        }
    }
}
#[test]
fn invalid_measured_motion_expires_an_active_hold_even_after_fresh_motion_returns() {
    let c = config(false, false);
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    warm(&mut p, &mut SensorLog::new("uncovered", c));
    let mut bad = frame(0.45, None);
    bad.odometry.as_mut().unwrap().speed = f64::NAN;
    let out = p.step(&bad).unwrap();
    assert_eq!(out.gnss_recovery.unwrap().phase, RecoveryPhase::Expired);
    for i in 10..=24 {
        let t = i as f64 * 0.05;
        let out = p.step(&frame(t, (i % 4 == 0).then_some((t, 0.)))).unwrap();
        assert_eq!(out.gnss_recovery.unwrap().phase, RecoveryPhase::Expired);
        assert_eq!(out.command.acceleration, -6.);
    }
}
#[test]
fn malformed_recovery_calibration_is_rejected_and_none_preserves_legacy_fields() {
    for case in 0..5 {
        let mut c = config(false, false);
        let g = c.gnss_recovery.as_mut().unwrap();
        match case {
            0 => g.max_position_correction_m = f64::NAN,
            1 => g.max_yaw_correction_rad = 0.,
            2 => g.confirmation_fixes = 1,
            3 => g.confirmation_fixes = 9,
            _ => g.max_hold_s = 10.1,
        }
        assert!(DrivingPipeline::new(c).is_err());
    }
    let mut c = config(false, false);
    c.gnss_recovery = None;
    assert!(
        serde_json::to_value(&c)
            .unwrap()
            .get("gnss_recovery")
            .is_none()
    );
    assert!(
        serde_json::to_value(
            DrivingPipeline::new(c)
                .unwrap()
                .step(&frame(0., Some((0., 0.))))
                .unwrap()
        )
        .unwrap()
        .get("gnss_recovery")
        .is_none()
    );
}
