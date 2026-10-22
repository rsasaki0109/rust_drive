use rustdriving_core::*;
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};
use std::f64::consts::{PI, TAU};

fn policy() -> ClearSweepRecoveryConfig {
    ClearSweepRecoveryConfig {
        lidar: PlanarLidarCalibration {
            ray_count: 720,
            first_angle_rad: -PI,
            angular_step_rad: TAU / 720.,
            min_range_m: 0.2,
            max_range_m: 45.,
            range_error_m: 0.05,
        },
        checked_center_radius_m: 40.,
        minimum_resolved_disc_radius_m: 0.4,
    }
}
fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.forecast_envelope = Some(ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    });
    c.enclose_track_measurements = true;
    c.forecast_consistency_guard = true;
    c.clear_sweep_recovery = Some(policy());
    c
}
fn frame(i: usize) -> SensorFrame {
    let time = i as f64 * 0.05;
    let mut points = vec![];
    let mut rays = vec![];
    for beam in 0..720 {
        let angle = -PI + TAU * beam as f64 / 720.;
        let d = Vec2::new(angle.cos(), angle.sin());
        let x = if i < 6 { 25. } else { 25.8 };
        let projection = x * d.x;
        let discriminant = 0.5_f64.powi(2) - (x * x - projection * projection);
        let hit = if i < 8 && discriminant >= 0. {
            Some(projection - discriminant.sqrt()).filter(|r| *r > 0.2 && *r < 45.)
        } else {
            None
        };
        if let Some(range) = hit {
            rays.push(PlanarRayOutcome::Return(points.len()));
            points.push(d.scaled(range));
        } else {
            rays.push(PlanarRayOutcome::Clear);
        }
    }
    serde_json::from_value(serde_json::json!({"time":time,
        "odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":time,"position":{"x":0.,"y":0.},"variance":0.02},
        "lidar":LidarScan {stamp:time,points,sweep:Some(PlanarLidarSweep{calibration:policy().lidar,rays})}})).unwrap()
}

#[test]
fn stopped_complete_fresh_empty_sweeps_can_release_a_lost_identity_with_auditable_support() {
    let c = config();
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("clear-sweep", c);
    for i in 0..24 {
        let input = frame(i);
        let out = p.step(&input).unwrap();
        let d = out.forecast_consistency.as_ref().unwrap();
        if (6..12).contains(&i) {
            assert!(out.health.contains(&HealthIssue::InconsistentForecast));
            assert!(out.command.acceleration <= -5.99);
        }
        if i == 10 {
            assert_eq!(d.state, ForecastConsistencyState::Confirming);
            assert_eq!(d.clear_sweep.as_ref().unwrap().confirmation_samples, 3);
        }
        if i == 12 {
            assert_eq!(d.state, ForecastConsistencyState::Observing);
            let clear = d.clear_sweep.as_ref().unwrap();
            assert_eq!(clear.release_count, 1);
            let r = clear.last_release.as_ref().unwrap();
            assert_eq!(r.identities, vec![1]);
            assert_eq!(r.first_acquisition_s, 0.4);
            assert!((r.last_acquisition_s - 0.6).abs() < 1e-12);
            assert!(
                !d.complete_scene_check,
                "negative evidence is a separate basis, not a fabricated measured comparison"
            );
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
    let text = String::from_utf8(bytes).unwrap();
    let changed = text.replacen("\"release_count\":1", "\"release_count\":0", 1);
    assert_ne!(text, changed);
    assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
}

#[test]
fn missing_partial_or_an_unowned_return_cannot_be_accepted_as_clear_sensing() {
    for mode in 0..3 {
        let mut p = DrivingPipeline::new(config()).unwrap();
        for i in 0..24 {
            let mut f = frame(i);
            if i >= 8 {
                let scan = f.lidar.as_mut().unwrap();
                match mode {
                    0 => scan.sweep = None,
                    1 => scan.sweep.as_mut().unwrap().rays[7] = PlanarRayOutcome::Unavailable,
                    _ => scan.points.push(Vec2::new(5., 0.)),
                }
            }
            let out = p.step(&f).unwrap();
            if i >= 8 {
                assert!(out.health.contains(&HealthIssue::InvalidLidar));
                assert!(out.health.contains(&HealthIssue::InconsistentForecast));
                assert!(out.command.acceleration <= -5.99);
            }
        }
    }
}

#[test]
fn moving_cached_motion_and_delayed_scans_do_not_supply_stopped_clear_evidence() {
    for mode in 0..3 {
        let mut p = DrivingPipeline::new(config()).unwrap();
        for i in 0..14 {
            let mut f = frame(i);
            if i >= 8 {
                match mode {
                    0 => f.odometry.as_mut().unwrap().speed = 0.21,
                    1 if i < 10 => f.odometry = None,
                    2 => f.lidar.as_mut().unwrap().stamp -= 0.05,
                    _ => {}
                }
            }
            let out = p.step(&f).unwrap();
            if i >= 8 {
                assert!(out.health.contains(&HealthIssue::InconsistentForecast));
                assert!(out.command.acceleration <= -5.99);
            }
        }
    }
}

#[test]
fn a_sensor_fault_discards_clear_votes_and_requires_new_post_fault_stopped_sweeps() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    let mut released = None;
    for i in 0..26 {
        let mut f = frame(i);
        if i == 10 {
            f.lidar_failed = true;
            f.lidar = None;
        }
        let out = p.step(&f).unwrap();
        let c = out
            .forecast_consistency
            .as_ref()
            .unwrap()
            .clear_sweep
            .as_ref()
            .unwrap();
        if i == 10 {
            assert_eq!(c.confirmation_samples, 0);
        }
        if c.release_count > 0 && released.is_none() {
            released = Some(i);
        }
    }
    assert_eq!(released, Some(15));
}

#[test]
fn independent_navigation_braking_survives_a_successful_empty_sweep_release() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    let mut released = false;
    for i in 0..24 {
        let mut f = frame(i);
        if i >= 6 {
            f.navigation_update = Some(rustdriving_pipeline::navigation::NavigationUpdate {
                stamp: f.time,
                revision: i as u64,
                closed_edges: vec![],
            });
        }
        let out = p.step(&f).unwrap();
        if i >= 6 {
            assert!(out.health.contains(&HealthIssue::InvalidNavigation));
            assert!(out.command.acceleration <= -5.99);
        }
        released |= out
            .forecast_consistency
            .as_ref()
            .unwrap()
            .clear_sweep
            .as_ref()
            .unwrap()
            .release_count
            > 0;
    }
    assert!(released);
}

#[test]
fn recovery_calibration_requires_the_guard_and_resolved_planar_acquisition() {
    let mut c = config();
    c.forecast_consistency_guard = false;
    assert!(DrivingPipeline::new(c).is_err());
    let mut c = config();
    c.clear_sweep_recovery
        .as_mut()
        .unwrap()
        .lidar
        .angular_step_rad = PI / 720.;
    assert!(DrivingPipeline::new(c).is_err());
    let mut c = config();
    c.clear_sweep_recovery = None;
    assert!(
        serde_json::to_value(c)
            .unwrap()
            .get("clear_sweep_recovery")
            .is_none()
    );
}
