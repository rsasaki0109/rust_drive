use rustdriving_core::*;
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, MapLocalizationConfig, MapOnlySupportStatus, PipelineConfig,
    PipelineOutput, RelocalizationConfig, RelocalizationPhase, SensorFrame,
    replay::{SensorLog, verify},
    traffic_controls::StopLine,
};
use std::io::Cursor;

fn map() -> Vec<Vec2> {
    (0..80)
        .map(|i| {
            let t = i as f64;
            Vec2::new(
                20. + 4. * (t * 0.73).sin() + (t * 0.13).cos(),
                6. + 2. * (t * 0.29).cos(),
            )
        })
        .collect()
}
fn config() -> PipelineConfig {
    let mut config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    config.gnss_latency_compensation = true;
    config.gnss_covariance_shape = Some([[1., 0.4], [0.4, 1.]]);
    config.gnss_antenna_offset_m = Some(Vec2::new(1., 0.2));
    config.localization_map = Some(MapLocalizationConfig {
        points: map(),
        ..MapLocalizationConfig::default()
    });
    config.relocalization = Some(RelocalizationConfig::default());
    config
}
fn measured_pose() -> Pose {
    Pose {
        position: Vec2::new(3.2, -0.4),
        yaw: 0.12,
    }
}
fn frame(time: f64, pose: Pose) -> SensorFrame {
    serde_json::from_value(serde_json::json!({
        "time":time,"odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":time,"position":pose.to_world(Vec2::new(1.,0.2)),"variance":0.02},
        "lidar":{"stamp":time,"points":map().iter().map(|p|p.minus(pose.position).rotated(-pose.yaw)).collect::<Vec<_>>()}
    })).unwrap()
}
fn lost(p: &mut DrivingPipeline) {
    p.step(&frame(0., Pose::default())).unwrap();
    let out = p.step(&frame(0.4, measured_pose())).unwrap();
    assert!(out.motion_discontinuity.is_some());
    assert!(out.emergency);
}

#[test]
fn consistent_measured_scans_create_a_new_epoch_and_pre_reset_fixes_cannot_resume_it() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    lost(&mut p);
    let mut recovered = None;
    for index in 9..=60 {
        let now = index as f64 * 0.05;
        let out = p.step(&frame(now, measured_pose())).unwrap();
        if out.localization_epoch == 1 {
            recovered = Some(out);
            break;
        }
        assert!(out.motion_discontinuity.is_some());
        assert!(out.emergency);
    }
    let recovered = recovered.expect("measured map should repair the missing pose");
    assert!(recovered.motion_discontinuity.is_none());
    assert!(
        recovered
            .estimate
            .pose
            .position
            .distance(measured_pose().position)
            < 1e-6
    );
    assert!(wrap_angle(recovered.estimate.pose.yaw - measured_pose().yaw).abs() < 1e-6);
    assert_eq!(recovered.command.acceleration, -6.);
    assert!(recovered.tracks.is_empty());
    assert!(recovered.predictions.is_empty());
    assert_eq!(recovered.localization.unwrap().accepted_fixes, 0);
    assert_eq!(recovered.localization.unwrap().last_observed_stamp, None);
    let diagnostic = recovered.relocalization.as_ref().unwrap();
    assert_eq!(diagnostic.phase, RelocalizationPhase::Reinitialized);
    assert_eq!(diagnostic.confirmed_scans, 3);
    assert_eq!(diagnostic.attempts, 3);
    assert_eq!(diagnostic.search.as_ref().unwrap().seeds_tested, 125);
    assert_eq!(recovered.gnss_latency.unwrap().history_predictions, 0);
    assert!(recovered.pose_covariance.unwrap()[0][0] >= 0.01);
    let mut stale = frame(recovered.time + 0.05, measured_pose());
    stale.gnss.as_mut().unwrap().stamp = recovered.time - 0.1;
    let out = p.step(&stale).unwrap();
    assert!(out.emergency);
    assert_eq!(out.localization.unwrap().accepted_fixes, 0);
    let out = p
        .step(&frame(recovered.time + 0.1, measured_pose()))
        .unwrap();
    assert!(out.health.is_empty(), "{:?}", out.health);
    assert_eq!(out.localization.unwrap().accepted_fixes, 1);
    assert_eq!(out.localization_epoch, 1);
}

#[test]
fn moving_repeated_missing_or_delayed_samples_cannot_confirm_a_reset() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    lost(&mut p);
    for index in 9..=30 {
        let now = index as f64 * 0.05;
        let mut input = frame(now, measured_pose());
        match index {
            9..=12 => input.odometry.as_mut().unwrap().speed = 0.1,
            13..=16 => input.odometry.as_mut().unwrap().stamp = 0.6,
            17..=20 => input.odometry = None,
            _ => input.lidar.as_mut().unwrap().stamp = now - 0.1,
        }
        let out = p.step(&input).unwrap();
        assert!(out.emergency);
        assert_eq!(out.localization_epoch, 0);
        assert!(out.motion_discontinuity.is_some());
        assert_eq!(out.relocalization.unwrap().confirmed_scans, 0);
    }
}

#[test]
fn exhausted_search_stays_stopped_and_unsupported_missions_fail_before_driving() {
    let mut cfg = config();
    cfg.relocalization = Some(RelocalizationConfig {
        max_neighbor_checks: 1,
        max_attempts: 2,
        confirmations: 2,
        ..RelocalizationConfig::default()
    });
    let mut p = DrivingPipeline::new(cfg).unwrap();
    lost(&mut p);
    let mut last = None;
    for index in 9..=60 {
        last = Some(
            p.step(&frame(index as f64 * 0.05, measured_pose()))
                .unwrap(),
        );
    }
    let last = last.unwrap();
    assert!(last.emergency);
    assert_eq!(last.localization_epoch, 0);
    let diagnostic = last.relocalization.unwrap();
    assert_eq!(diagnostic.phase, RelocalizationPhase::Exhausted);
    assert_eq!(diagnostic.attempts, 2);
    let mut cfg = config();
    cfg.localization_map = None;
    assert!(DrivingPipeline::new(cfg).is_err());
    let mut cfg = config();
    cfg.stop_lines = vec![StopLine {
        id: "signal".into(),
        route_s_m: 30.,
    }];
    assert!(DrivingPipeline::new(cfg).is_err());
    let mut cfg = config();
    cfg.relocalization.as_mut().unwrap().confirmations = 1;
    assert!(DrivingPipeline::new(cfg).is_err());
}

#[test]
fn replay_reconstructs_the_measured_reset_and_rejects_a_forged_epoch() {
    let cfg = config();
    let mut p = DrivingPipeline::new(cfg.clone()).unwrap();
    let mut log = SensorLog::new("regional-relocalization", cfg);
    for time in [0., 0.4]
        .into_iter()
        .chain((9..=36).map(|i| i as f64 * 0.05))
    {
        let input = frame(
            time,
            if time == 0. {
                Pose::default()
            } else {
                measured_pose()
            },
        );
        let output = p.step(&input).unwrap();
        log.record(input, output);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap()
            .verified
    );
    let tick = log
        .ticks
        .iter_mut()
        .find(|t| t.expected.localization_epoch == 1)
        .unwrap();
    tick.expected.localization_epoch = 0;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}

fn without_gnss(time: f64) -> SensorFrame {
    let mut input = frame(time, measured_pose());
    input.gnss = None;
    input
}
fn map_only_config(grace: f64) -> PipelineConfig {
    let mut cfg = config();
    cfg.relocalization.as_mut().unwrap().map_only_grace_s = Some(grace);
    cfg
}
fn map_only_reset(grace: f64) -> (DrivingPipeline, PipelineOutput) {
    let mut pipeline = DrivingPipeline::new(map_only_config(grace)).unwrap();
    lost(&mut pipeline);
    for i in 9..=60 {
        let output = pipeline.step(&without_gnss(i as f64 * 0.05)).unwrap();
        if output.localization_epoch == 1 {
            assert!(output.emergency);
            assert_eq!(output.localization.unwrap().accepted_fixes, 0);
            assert_eq!(
                output
                    .relocalization
                    .as_ref()
                    .unwrap()
                    .map_only_support
                    .as_ref()
                    .unwrap()
                    .status,
                MapOnlySupportStatus::WaitingForMap
            );
            return (pipeline, output);
        }
    }
    panic!("measured reset required");
}

#[test]
fn map_only_support_uses_new_scans_without_minting_gnss_and_hands_off_to_a_real_fix() {
    let (mut pipeline, reset) = map_only_reset(5.);
    for i in 1..=4 {
        let mut input = without_gnss(reset.time + i as f64 * 0.05);
        // A late pre-reset fix cannot enter the new history or close the window.
        if i == 2 {
            input.gnss = frame(reset.time - 0.1, measured_pose()).gnss;
        }
        let output = pipeline.step(&input).unwrap();
        assert!(output.health.is_empty(), "{:?}", output.health);
        assert!(!output.emergency);
        assert!(output.command.acceleration > 0.);
        let gnss = output.localization.unwrap();
        assert_eq!(gnss.accepted_fixes, 0);
        assert_eq!(gnss.last_observed_stamp, None);
        assert_eq!(gnss.last_accepted_stamp, None);
        assert!(!output.map_localization.unwrap().gnss_outage_covered);
        let support = output.relocalization.unwrap().map_only_support.unwrap();
        assert_eq!(support.status, MapOnlySupportStatus::Active);
        assert_eq!(support.expires_at, reset.time + 5.);
    }
    let output = pipeline
        .step(&frame(reset.time + 0.25, measured_pose()))
        .unwrap();
    assert!(output.health.is_empty());
    assert_eq!(output.localization.unwrap().accepted_fixes, 1);
    assert_eq!(
        output
            .relocalization
            .unwrap()
            .map_only_support
            .unwrap()
            .status,
        MapOnlySupportStatus::GnssAccepted
    );
}

#[test]
fn fresh_maps_cannot_renew_the_absolute_deadline_and_later_gnss_can_recover() {
    let (mut pipeline, reset) = map_only_reset(0.5);
    for i in 1..=14 {
        let output = pipeline
            .step(&without_gnss(reset.time + i as f64 * 0.05))
            .unwrap();
        let support = output.relocalization.unwrap().map_only_support.unwrap();
        assert_eq!(support.expires_at, reset.time + 0.5);
        assert_eq!(output.localization.unwrap().accepted_fixes, 0);
        if i <= 10 {
            assert!(output.health.is_empty(), "{:?}", output.health);
            assert_eq!(support.status, MapOnlySupportStatus::Active);
        } else {
            assert!(output.emergency);
            assert!(output.health.contains(&HealthIssue::StaleGnss));
            assert_eq!(support.status, MapOnlySupportStatus::Expired);
        }
    }
    let output = pipeline
        .step(&frame(reset.time + 0.75, measured_pose()))
        .unwrap();
    assert!(output.health.is_empty());
    assert_eq!(output.localization.unwrap().accepted_fixes, 1);
}

#[test]
fn failed_repeated_delayed_or_stale_maps_cannot_support_the_new_epoch() {
    for mode in 0..5 {
        let (mut pipeline, reset) = map_only_reset(5.);
        let active = pipeline.step(&without_gnss(reset.time + 0.05)).unwrap();
        assert!(active.health.is_empty());
        assert_eq!(
            active
                .relocalization
                .unwrap()
                .map_only_support
                .unwrap()
                .status,
            MapOnlySupportStatus::Active
        );
        let mut last = None;
        for i in 2..=7 {
            let mut input = without_gnss(reset.time + i as f64 * 0.05);
            match mode {
                0 => input.lidar_failed = true,
                1 => input.lidar.as_mut().unwrap().stamp = reset.time + 0.05,
                2 => input.lidar.as_mut().unwrap().stamp -= 0.05,
                3 => input.lidar = None,
                _ => input.lidar.as_mut().unwrap().points.clear(),
            }
            last = Some(pipeline.step(&input).unwrap());
        }
        let output = last.unwrap();
        assert!(output.emergency);
        assert!(output.health.contains(&HealthIssue::StaleGnss));
        assert_ne!(
            output
                .relocalization
                .unwrap()
                .map_only_support
                .unwrap()
                .status,
            MapOnlySupportStatus::Active
        );
    }
}

#[test]
fn malformed_rejected_gnss_and_missing_motion_interrupt_support() {
    for mode in 0..3 {
        let (mut pipeline, reset) = map_only_reset(5.);
        for i in 1..=8 {
            let mut input = without_gnss(reset.time + i as f64 * 0.05);
            if mode == 2 {
                input.odometry = None;
            } else if i == 1 {
                input.gnss = frame(input.time, measured_pose()).gnss;
                if mode == 0 {
                    input.gnss.as_mut().unwrap().variance = -1.;
                } else {
                    input.gnss.as_mut().unwrap().position.x += 100.;
                }
            }
            let output = pipeline.step(&input).unwrap();
            if mode != 2 || i >= 4 {
                assert!(output.emergency);
                assert_eq!(
                    output
                        .relocalization
                        .unwrap()
                        .map_only_support
                        .unwrap()
                        .status,
                    MapOnlySupportStatus::Interrupted
                );
            }
        }
    }
}

#[test]
fn map_only_bounds_and_synchronous_scan_contract_are_validated() {
    for grace in [0., 0.49, 5.01, f64::INFINITY, f64::NAN] {
        assert!(DrivingPipeline::new(map_only_config(grace)).is_err());
    }
    let mut cfg = map_only_config(5.);
    cfg.localization_map.as_mut().unwrap().allow_delayed_scans = true;
    assert!(DrivingPipeline::new(cfg).is_err());
}

#[test]
fn replay_checks_map_only_evidence_and_rejects_a_forged_deadline() {
    let cfg = map_only_config(5.);
    let mut pipeline = DrivingPipeline::new(cfg.clone()).unwrap();
    let mut log = SensorLog::new("post-reset-map-only", cfg);
    for time in [0., 0.4]
        .into_iter()
        .chain((9..=36).map(|i| i as f64 * 0.05))
    {
        let input = if time == 0. {
            frame(time, Pose::default())
        } else {
            without_gnss(time)
        };
        let output = pipeline.step(&input).unwrap();
        log.record(input, output);
    }
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap()
            .verified
    );
    let support = log
        .ticks
        .iter_mut()
        .find_map(|tick| {
            tick.expected
                .relocalization
                .as_mut()?
                .map_only_support
                .as_mut()
        })
        .unwrap();
    support.expires_at += 1.;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}
