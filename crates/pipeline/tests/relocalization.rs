use rustdriving_core::*;
use rustdriving_pipeline::{
    DrivingPipeline, GnssRecoveryConfig, HealthIssue, MapLocalizationConfig, MapOnlySupportStatus,
    PipelineConfig, PipelineOutput, RecoveryPhase, RelocalizationConfig, RelocalizationPhase,
    SensorFrame,
    navigation::{NavigationConfig, NavigationPhase, NavigationUpdate},
    replay::{SensorLog, verify},
    traffic_controls::{SignalColor, SignalObservation, SignalState, StopLine},
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

fn stop_recovery_config() -> PipelineConfig {
    let mut c = config();
    c.stop_signs = vec![StopLine {
        id: "near".into(),
        route_s_m: 7.,
    }];
    c.relocalization.as_mut().unwrap().preserve_stop_signs = true;
    c
}

fn signal_recovery_config() -> PipelineConfig {
    let mut c = config();
    c.stop_lines = vec![StopLine {
        id: "signal".into(),
        route_s_m: 7.,
    }];
    c.relocalization.as_mut().unwrap().preserve_traffic_signals = true;
    c
}
fn signal(stamp: f64, color: SignalColor) -> SignalObservation {
    SignalObservation {
        stamp,
        states: vec![SignalState {
            id: "signal".into(),
            color,
        }],
    }
}

#[test]
fn measured_signal_reset_discards_green_then_accepts_new_permission_and_replays() {
    let c = signal_recovery_config();
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("signal-preserving-relocalization", c);
    let mut reset = None;
    let mut new_green = false;
    for i in (0..=70).filter(|i| !(1..8).contains(i)) {
        let now = i as f64 * 0.05;
        let mut input = frame(
            now,
            if i == 0 {
                Pose::default()
            } else {
                measured_pose()
            },
        );
        input.traffic_signal = Some(signal(if now < 2. { now } else { 2. }, SignalColor::Green));
        // The reset occurs before 2 s. Its own green must be withdrawn; only
        // a strictly later acquisition can establish fresh permission.
        let out = p.step(&input).unwrap();
        if out.localization_epoch == 1 {
            let status = out.traffic_controls.as_ref().unwrap();
            if reset.is_none() {
                reset = Some(now);
                assert!(status.localization_reset.as_ref().unwrap().admitted);
                assert_eq!(status.last_accepted_stamp, None);
                assert_eq!(status.signals[0].color, SignalColor::Unknown);
                assert!(status.stop_s_m.is_some());
                assert_eq!(out.command.acceleration, -6.);
            } else if now <= 2. {
                new_green = true;
                assert_eq!(status.signals[0].color, SignalColor::Green);
                assert!(status.stop_s_m.is_none());
            } else if now > 2.5 + 1e-9 {
                assert_eq!(status.signals[0].color, SignalColor::Unknown);
                assert!(status.stop_s_m.is_some());
            }
        }
        log.record(input, out);
    }
    assert!(reset.is_some() && new_green);
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap()
            .verified
    );
    log.ticks
        .iter_mut()
        .find(|t| t.expected.localization_epoch == 1)
        .unwrap()
        .expected
        .traffic_controls
        .as_mut()
        .unwrap()
        .localization_reset
        .as_mut()
        .unwrap()
        .admitted = false;
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(verify(Cursor::new(bytes), std::io::sink()).is_err());
}

#[test]
fn unserved_signal_behind_measured_reset_stays_faulted_through_green_and_gnss() {
    let mut c = signal_recovery_config();
    c.initial_pose = measured_pose();
    let mut p = DrivingPipeline::new(c).unwrap();
    p.step(&frame(0., measured_pose())).unwrap();
    let mut rejected = false;
    for i in 8..=80 {
        let now = i as f64 * 0.05;
        let mut pose = measured_pose();
        pose.position.x = 6.5;
        let mut input = frame(now, pose);
        input.traffic_signal = Some(signal(now, SignalColor::Green));
        let out = p.step(&input).unwrap();
        if out.localization_epoch == 1 {
            rejected = true;
            assert!(
                out.health
                    .contains(&HealthIssue::InvalidTrafficSignalRecovery)
            );
            assert_eq!(out.command.acceleration, -6.);
            let status = out.traffic_controls.unwrap();
            assert!(!status.localization_reset.unwrap().admitted);
            assert!(!status.signals[0].passed);
            assert!(status.stop_s_m.is_some());
        }
    }
    assert!(rejected);
}

#[test]
fn signal_recovery_is_explicit_and_composes_with_explicit_fixed_stops() {
    let mut c = signal_recovery_config();
    c.relocalization.as_mut().unwrap().preserve_traffic_signals = false;
    assert!(
        serde_json::to_value(&c).unwrap()["relocalization"]
            .get("preserve_traffic_signals")
            .is_none()
    );
    assert!(c.validate().is_err());
    let mut c = config();
    c.relocalization.as_mut().unwrap().preserve_traffic_signals = true;
    assert!(c.validate().is_err());
    let mut c = stop_recovery_config();
    c.stop_lines = signal_recovery_config().stop_lines;
    c.stop_lines[0].route_s_m = 30.;
    // The map identifiers must remain distinct across independent rule types.
    c.relocalization.as_mut().unwrap().preserve_traffic_signals = true;
    assert!(c.validate().is_ok());
    c.relocalization.as_mut().unwrap().preserve_stop_signs = false;
    assert!(c.validate().is_err());
}

#[test]
fn failed_stop_recovery_prevents_signal_commitment_even_with_fresh_moving_observations() {
    let mut c = stop_recovery_config();
    c.initial_pose = measured_pose();
    c.stop_lines = vec![StopLine {
        id: "signal".into(),
        route_s_m: 12.,
    }];
    c.relocalization.as_mut().unwrap().preserve_traffic_signals = true;
    let mut p = DrivingPipeline::new(c).unwrap();
    p.step(&frame(0., measured_pose())).unwrap();
    let mut crossed_measured_front = false;
    for i in 8..=120 {
        let now = i as f64 * 0.05;
        let mut pose = measured_pose();
        pose.position.x = 6.5;
        let moving = now >= 2.05 - 1e-9;
        if moving {
            pose.position = pose
                .position
                .plus(Vec2::new(2. * (now - 2.), 0.).rotated(pose.yaw));
        }
        let mut input = frame(now, pose);
        input.odometry.as_mut().unwrap().speed = if moving { 2. } else { 0. };
        input.traffic_signal = Some(signal(now, SignalColor::Green));
        let out = p.step(&input).unwrap();
        if out.localization_epoch == 1 {
            assert!(out.health.contains(&HealthIssue::InvalidStopSignRecovery));
            assert_eq!(out.command.acceleration, -6.);
            let controls = out.traffic_controls.as_ref().unwrap();
            assert!(controls.localization_reset.as_ref().unwrap().admitted);
            assert!(!controls.signals[0].passed);
            crossed_measured_front |= out.estimate.pose.position.x + 1.25 >= 12.;
        }
    }
    assert!(crossed_measured_front);
}

#[test]
fn measured_stop_recovery_restarts_dwell_and_replays_complete_outputs() {
    use rustdriving_pipeline::stop_signs::StopPhase;
    let c = stop_recovery_config();
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("stop-preserving-relocalization", c);
    let mut reset = None;
    let mut release = None;
    for i in 0..=100 {
        if (1..8).contains(&i) {
            continue;
        }
        let now = i as f64 * 0.05;
        let input = frame(
            now,
            if i == 0 {
                Pose::default()
            } else {
                measured_pose()
            },
        );
        let out = p.step(&input).unwrap();
        if out.localization_epoch == 1 {
            let stops = out.stop_signs.as_ref().unwrap();
            if reset.is_none() {
                reset = Some(now);
                assert!(stops.localization_reset.as_ref().unwrap().admitted);
                assert_eq!(stops.stops[0].held_s, 0.);
                assert_eq!(out.command.acceleration, -6.);
            }
            if stops.stops[0].phase == StopPhase::Released {
                release.get_or_insert(now);
            } else {
                assert!(out.command.acceleration <= -0.5);
            }
        }
        log.record(input, out);
    }
    assert!(release.unwrap() >= reset.unwrap() + 2.05 - 1e-9);
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap()
            .verified
    );
    log.ticks
        .iter_mut()
        .find(|t| t.expected.localization_epoch == 1)
        .unwrap()
        .expected
        .stop_signs
        .as_mut()
        .unwrap()
        .stops[0]
        .held_s = 2.;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(verify(Cursor::new(bytes), std::io::sink()).is_err());
}

#[test]
fn measured_unserved_stop_crossing_keeps_braking_through_fresh_gnss() {
    let mut c = stop_recovery_config();
    c.initial_pose = measured_pose();
    let mut p = DrivingPipeline::new(c).unwrap();
    p.step(&frame(0., measured_pose())).unwrap();
    let mut rejected = false;
    for i in 8..=100 {
        let now = i as f64 * 0.05;
        let mut pose = measured_pose();
        pose.position.x = 6.5;
        let out = p.step(&frame(now, pose)).unwrap();
        if out.localization_epoch == 1 {
            rejected = true;
            assert!(out.health.contains(&HealthIssue::InvalidStopSignRecovery));
            assert_eq!(out.command.acceleration, -6.);
            assert!(
                !out.stop_signs
                    .as_ref()
                    .unwrap()
                    .localization_reset
                    .as_ref()
                    .unwrap()
                    .admitted
            );
            assert_eq!(out.stop_signs.as_ref().unwrap().stops[0].held_s, 0.);
        }
    }
    assert!(rejected);
}

#[test]
fn mapped_stops_require_explicit_recovery_and_other_control_types_stay_excluded() {
    let mut c = stop_recovery_config();
    c.relocalization.as_mut().unwrap().preserve_stop_signs = false;
    assert!(
        serde_json::to_value(&c).unwrap()["relocalization"]
            .get("preserve_stop_signs")
            .is_none()
    );
    assert!(DrivingPipeline::new(c).is_err());
    let mut c = config();
    c.relocalization.as_mut().unwrap().preserve_stop_signs = true;
    assert!(DrivingPipeline::new(c).is_err());
    let mut c = stop_recovery_config();
    c.stop_lines.push(StopLine {
        id: "signal".into(),
        route_s_m: 30.,
    });
    assert!(DrivingPipeline::new(c).is_err());
    assert!(DrivingPipeline::new(stop_recovery_config()).is_ok());
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
    reset_with_config(map_only_config(grace))
}
fn reset_with_config(cfg: PipelineConfig) -> (DrivingPipeline, PipelineOutput) {
    let mut pipeline = DrivingPipeline::new(cfg).unwrap();
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

fn confirmed_handoff_config() -> PipelineConfig {
    let mut cfg = map_only_config(5.);
    cfg.relocalization.as_mut().unwrap().confirm_gnss_handoff = true;
    cfg.gnss_recovery = Some(GnssRecoveryConfig {
        max_position_correction_m: 0.4,
        max_yaw_correction_rad: 0.1,
        confirmation_fixes: 3,
        max_hold_s: 5.,
    });
    cfg
}

#[test]
fn handoff_is_armed_without_fake_rejections_and_commits_only_the_last_delayed_candidate() {
    let cfg = confirmed_handoff_config();
    let (mut p, reset) = reset_with_config(cfg.clone());
    let (mut no_fix, _) = reset_with_config(cfg);
    let armed = reset.gnss_recovery.unwrap();
    assert_eq!(armed.phase, RecoveryPhase::Holding);
    assert_eq!(armed.hold_started_at, Some(reset.time));
    assert_eq!(armed.last_candidate_stamp, None);
    assert_eq!(armed.held_fixes, 0);
    assert_eq!(reset.localization.unwrap().rejected_fixes, 0);
    let active = p.step(&without_gnss(reset.time + 0.05)).unwrap();
    no_fix.step(&without_gnss(reset.time + 0.05)).unwrap();
    assert!(!active.emergency);
    for i in 2..=4 {
        let now = reset.time + i as f64 * 0.05;
        let reference = no_fix.step(&without_gnss(now)).unwrap();
        let mut input = frame(now, measured_pose());
        input.gnss.as_mut().unwrap().stamp = now - 0.05;
        if i == 2 {
            input.gnss.as_mut().unwrap().position.y += 0.3;
        }
        let output = p.step(&input).unwrap();
        let diagnostic = output.gnss_recovery.unwrap();
        let actual = output.localization.unwrap();
        if i < 4 {
            assert!(output.emergency);
            assert_eq!(actual.last_decision, Some(GnssDecision::RecoveryHeld));
            assert_eq!(actual.accepted_fixes, 0);
            assert_eq!(actual.last_accepted_stamp, None);
            assert_eq!(diagnostic.consecutive_candidates, (i - 1) as u8);
            assert_eq!(
                output.estimate.pose.position.x,
                reference.estimate.pose.position.x
            );
            assert_eq!(
                output.estimate.pose.position.y,
                reference.estimate.pose.position.y
            );
            assert_eq!(output.estimate.pose.yaw, reference.estimate.pose.yaw);
            assert_eq!(output.pose_covariance, reference.pose_covariance);
            assert_eq!(output.gnss_latency.unwrap().history_gnss_corrections, 0);
            assert_eq!(output.gnss_latency.unwrap().replayed_map_corrections, 0);
        } else {
            assert!(!output.emergency, "{:?}", output.health);
            assert_eq!(actual.accepted_fixes, 1);
            assert_eq!(actual.rejected_fixes, 2);
            assert_eq!(actual.last_accepted_stamp, Some(now - 0.05));
            assert_eq!(diagnostic.phase, RecoveryPhase::Ready);
            assert_eq!(diagnostic.completed_recoveries, 1);
            // Synchronous-map history folds the accepted fix into its anchor;
            // separate GNSS events are retained only for delayed map mode.
            assert_eq!(output.gnss_latency.unwrap().history_gnss_corrections, 0);
            assert_eq!(output.gnss_latency.unwrap().compensated_fixes, 1);
            assert!(output.gnss_latency.unwrap().replayed_map_corrections > 0);
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
    }
}

#[test]
fn handoff_deadline_expires_before_any_fix_and_fresh_scans_or_fixes_cannot_renew_it() {
    let mut cfg = confirmed_handoff_config();
    cfg.gnss_recovery.as_mut().unwrap().max_hold_s = 0.2;
    let (mut p, reset) = reset_with_config(cfg);
    for i in 1..=14 {
        let now = reset.time + i as f64 * 0.05;
        let input = if i < 7 {
            without_gnss(now)
        } else {
            frame(now, measured_pose())
        };
        let output = p.step(&input).unwrap();
        let recovery = output.gnss_recovery.unwrap();
        assert_eq!(recovery.hold_started_at, Some(reset.time));
        assert_eq!(output.localization.unwrap().accepted_fixes, 0);
        if i <= 4 {
            assert!(!output.emergency);
        } else {
            assert!(output.emergency);
            assert!(output.health.contains(&HealthIssue::GnssHandoffExpired));
            assert_eq!(recovery.phase, RecoveryPhase::Expired);
            assert_eq!(recovery.consecutive_candidates, 0);
        }
    }
}

#[test]
fn late_repeated_or_malformed_fixes_cannot_supply_the_handoff_confirmations() {
    let (mut p, reset) = reset_with_config(confirmed_handoff_config());
    let mut old = frame(reset.time + 0.05, measured_pose());
    old.gnss.as_mut().unwrap().stamp = reset.time;
    let output = p.step(&old).unwrap();
    assert_eq!(output.localization.unwrap().last_observed_stamp, None);
    assert_eq!(output.gnss_recovery.unwrap().consecutive_candidates, 0);
    let first = frame(reset.time + 0.1, measured_pose());
    assert_eq!(
        p.step(&first)
            .unwrap()
            .gnss_recovery
            .unwrap()
            .consecutive_candidates,
        1
    );
    let mut repeat = frame(reset.time + 0.15, measured_pose());
    repeat.gnss = first.gnss;
    assert_eq!(
        p.step(&repeat)
            .unwrap()
            .gnss_recovery
            .unwrap()
            .consecutive_candidates,
        1
    );
    let mut malformed = frame(reset.time + 0.2, measured_pose());
    malformed.gnss.as_mut().unwrap().variance = -1.;
    let output = p.step(&malformed).unwrap();
    assert!(output.emergency);
    assert_eq!(output.gnss_recovery.unwrap().consecutive_candidates, 0);
    assert_eq!(output.gnss_recovery.unwrap().phase, RecoveryPhase::Holding);
}

#[test]
fn handoff_dependencies_are_checked_and_existing_recovery_without_the_flag_stays_ready() {
    for mode in 0..3 {
        let mut cfg = confirmed_handoff_config();
        match mode {
            0 => cfg.gnss_recovery = None,
            1 => cfg.gnss_latency_compensation = false,
            _ => cfg.relocalization.as_mut().unwrap().map_only_grace_s = None,
        }
        assert!(DrivingPipeline::new(cfg).is_err());
    }
    let mut cfg = confirmed_handoff_config();
    cfg.relocalization.as_mut().unwrap().confirm_gnss_handoff = false;
    assert!(
        serde_json::to_value(&cfg).unwrap()["relocalization"]
            .get("confirm_gnss_handoff")
            .is_none()
    );
    let (mut p, reset) = reset_with_config(cfg);
    assert_eq!(reset.gnss_recovery.unwrap().phase, RecoveryPhase::Ready);
    let output = p.step(&frame(reset.time + 0.05, measured_pose())).unwrap();
    assert_eq!(output.localization.unwrap().accepted_fixes, 1);
    assert_eq!(output.gnss_recovery.unwrap().completed_recoveries, 0);
}

#[test]
fn replay_rejects_a_forged_handoff_confirmation_counter() {
    let cfg = confirmed_handoff_config();
    let mut p = DrivingPipeline::new(cfg.clone()).unwrap();
    let mut log = SensorLog::new("confirmed-gnss-handoff", cfg);
    for time in [0., 0.4]
        .into_iter()
        .chain((9..=40).map(|i| i as f64 * 0.05))
    {
        let input = if time == 0. {
            frame(time, Pose::default())
        } else if time < 1.6 {
            without_gnss(time)
        } else {
            frame(time, measured_pose())
        };
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
        .find(|t| t.expected.gnss_recovery.unwrap().phase == RecoveryPhase::Confirming)
        .unwrap();
    tick.expected
        .gnss_recovery
        .as_mut()
        .unwrap()
        .consecutive_candidates += 1;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(
        verify(Cursor::new(bytes), std::io::sink())
            .unwrap_err()
            .contains("mismatch")
    );
}

fn navigation_recovery_config() -> PipelineConfig {
    let mut cfg = config();
    let value: serde_json::Value =
        serde_json::from_str(include_str!("../../../scenarios/route-direct.json")).unwrap();
    let mut navigation: NavigationConfig =
        serde_json::from_value(value["navigation"].clone()).unwrap();
    navigation
        .network
        .edges
        .iter_mut()
        .find(|edge| edge.id == "detour")
        .unwrap()
        .speed_limit_m_s = Some(2.5);
    cfg.route = navigation.initial_plan().unwrap().route;
    cfg.navigation = Some(navigation);
    cfg.relocalization.as_mut().unwrap().preserve_navigation = true;
    cfg.relocalization.as_mut().unwrap().map_only_grace_s = Some(5.);
    cfg
}
fn closure(time: f64, revision: u64, closed: &[&str]) -> NavigationUpdate {
    NavigationUpdate {
        stamp: time,
        revision,
        closed_edges: closed.iter().map(|id| (*id).into()).collect(),
    }
}

#[test]
fn measured_reset_preserves_an_already_changed_mission_and_newer_closure_revision() {
    let cfg = navigation_recovery_config();
    let mut p = DrivingPipeline::new(cfg.clone()).unwrap();
    let mut log = SensorLog::new("measured-navigation-mission-recovery", cfg);
    for i in 0..=2 {
        let mut input = frame(i as f64 * 0.05, Pose::default());
        if i == 0 {
            input.navigation_update = Some(closure(input.time, 17, &["main"]));
        }
        let output = p.step(&input).unwrap();
        log.record(input, output);
    }
    assert_eq!(
        p.navigation_plan().unwrap().edge_ids,
        ["approach", "detour", "east-exit"]
    );
    let input = frame(0.4, measured_pose());
    let output = p.step(&input).unwrap();
    assert!(output.motion_discontinuity.is_some());
    log.record(input, output);
    let reset = (9..=50)
        .find_map(|i| {
            let input = frame(i as f64 * 0.05, measured_pose());
            let out = p.step(&input).unwrap();
            let stamp = (out.localization_epoch == 1).then_some(out.time);
            if stamp.is_some() {
                let nav = out.navigation.as_ref().unwrap();
                assert_eq!(nav.revision, 17);
                assert_eq!(nav.switches, 1);
                assert_eq!(nav.closed_edges, ["main"]);
                assert_eq!(nav.active_edges, ["approach", "detour", "east-exit"]);
                let evidence = nav.localization_reset.as_ref().unwrap();
                assert_eq!(evidence.stamp, out.time);
                assert!(evidence.admitted);
                assert_eq!(out.localization.unwrap().accepted_fixes, 0);
                assert!(out.emergency);
            }
            log.record(input, out);
            stamp
        })
        .unwrap();
    let mut input = frame(reset + 0.05, measured_pose());
    input.navigation_update = Some(closure(input.time, 1, &[]));
    let out = p.step(&input).unwrap();
    assert!(out.health.is_empty(), "{:?}", out.health);
    assert_eq!(out.navigation.as_ref().unwrap().revision, 17);
    assert_eq!(
        out.navigation.as_ref().unwrap().active_edges,
        ["approach", "detour", "east-exit"]
    );
    log.record(input, out);
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(verify(Cursor::new(bytes), std::io::sink()).is_ok());
    let tick = log
        .ticks
        .iter_mut()
        .find(|t| {
            t.expected
                .navigation
                .as_ref()
                .is_some_and(|n| n.localization_reset.is_some())
        })
        .unwrap();
    tick.expected.navigation.as_mut().unwrap().revision = 0;
    let mut bytes = Vec::new();
    log.write(&mut bytes).unwrap();
    assert!(verify(Cursor::new(bytes), std::io::sink()).is_err());
}

#[test]
fn navigation_fault_survives_measured_recovery_and_fresh_localization() {
    let mut p = DrivingPipeline::new(navigation_recovery_config()).unwrap();
    p.step(&frame(0., Pose::default())).unwrap();
    let mut input = frame(0.4, measured_pose());
    input.navigation_update = Some(closure(0.4, 1, &["unknown"]));
    assert_eq!(
        p.step(&input).unwrap().navigation.unwrap().phase,
        NavigationPhase::Fault
    );
    let reset = (9..=50)
        .find_map(|i| {
            let out = p.step(&frame(i as f64 * 0.05, measured_pose())).unwrap();
            (out.localization_epoch == 1).then_some(out.time)
        })
        .unwrap();
    for i in 1..=5 {
        let out = p
            .step(&frame(reset + i as f64 * 0.05, measured_pose()))
            .unwrap();
        assert!(out.emergency);
        assert!(out.health.contains(&HealthIssue::InvalidNavigation));
        assert_eq!(out.navigation.unwrap().phase, NavigationPhase::Fault);
        assert!(out.localization.unwrap().accepted_fixes > 0);
    }
}

#[test]
fn navigation_preservation_requires_the_explicit_mission_opt_in() {
    let mut cfg = navigation_recovery_config();
    cfg.relocalization.as_mut().unwrap().preserve_navigation = false;
    assert!(
        serde_json::to_value(&cfg).unwrap()["relocalization"]
            .get("preserve_navigation")
            .is_none()
    );
    assert!(DrivingPipeline::new(cfg).is_err());
    let mut cfg = config();
    cfg.relocalization.as_mut().unwrap().preserve_navigation = true;
    assert!(DrivingPipeline::new(cfg).is_err());
    assert!(DrivingPipeline::new(navigation_recovery_config()).is_ok());
}
