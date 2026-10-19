use rustdriving_core::*;
use rustdriving_pipeline::{
    DrivingPipeline, MapLocalizationConfig, PipelineConfig, RelocalizationConfig,
    RelocalizationPhase, SensorFrame,
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
