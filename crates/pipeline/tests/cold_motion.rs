use rustdriving_core::{ForecastEnvelopeConfig, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.fit_recent_track_motion = true;
    c.enclose_track_measurements = true;
    c.forecast_consistency_guard = true;
    c.forecast_cold_max_speed_m_s = Some(15.);
    c.forecast_envelope = Some(ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    });
    c
}

fn frame(time: f64) -> SensorFrame {
    serde_json::from_value(serde_json::json!({
        "time":time,"odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":time,"position":{"x":0.,"y":0.},"variance":0.02},
        "lidar":{"stamp":time,"points":(0..8).map(|i| {
            let a=std::f64::consts::PI-0.5+i as f64/7.;
            Vec2::new(30.+2.5*time+a.cos(),4.+a.sin())
        }).collect::<Vec<_>>()}
    }))
    .unwrap()
}

#[test]
fn measured_speed_exceeding_the_declared_bound_clears_forecasts_and_brakes() {
    let mut c = config();
    c.forecast_cold_max_speed_m_s = Some(0.7);
    let mut driver = DrivingPipeline::new(c).unwrap();
    for time in [0., 0.1] {
        driver.step(&frame(time)).unwrap();
    }
    let out = driver.step(&frame(0.2)).unwrap();
    assert!(out.emergency);
    assert!(
        out.health
            .contains(&rustdriving_pipeline::HealthIssue::InconsistentForecast)
    );
    assert!(!out.predictions.is_empty());
    assert!(
        out.predictions
            .iter()
            .all(|p| p.positions.is_empty() && p.additional_radius_m.is_empty())
    );
    assert!(!out.cold_motion.as_ref().unwrap().tracks[0].confirmed);
}

#[test]
fn first_forecasts_remain_scored_and_replay_binds_the_acquisition_support() {
    let c = config();
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("cold-acquired-motion", c.clone());
    let mut old_config = c;
    old_config.forecast_cold_max_speed_m_s = None;
    let mut old = DrivingPipeline::new(old_config).unwrap();
    for i in 0_usize..9 {
        let mut input = frame(i as f64 * 0.05);
        if !i.is_multiple_of(2) {
            input.lidar = None;
        }
        let out = driver.step(&input).unwrap();
        let previous = old.step(&input).unwrap();
        assert_eq!(
            serde_json::to_value(&out.tracks).unwrap(),
            serde_json::to_value(&previous.tracks).unwrap()
        );
        assert_eq!(out.predictions.len(), 1);
        let support = &out.cold_motion.as_ref().unwrap().tracks[0];
        assert_eq!(support.confirmed, i >= 4);
        assert_eq!(support.acquisition_count, (i / 2 + 1).min(3));
        let p = &out.predictions[0];
        assert_eq!(p.positions, previous.predictions[0].positions);
        if i < 4 {
            let age = input.time - out.tracks[0].last_seen;
            for (j, center) in p.positions.iter().enumerate() {
                let reach =
                    center.distance(out.tracks[0].position) + 0.2 + 15. * (age + j as f64 * p.dt);
                assert!(p.additional_radius_m[j] + 1e-8 >= reach);
            }
        } else {
            assert_eq!(
                p.additional_radius_m,
                previous.predictions[0].additional_radius_m
            );
            assert!(out.tracks[0].velocity.distance(Vec2::new(2.5, 0.)) < 1e-8);
        }
        assert!(out.forecast_consistency.as_ref().unwrap().fault.is_none());
        log.record(input, out);
    }
    let check = |log: &SensorLog| {
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        verify(std::io::Cursor::new(bytes), vec![]).is_ok_and(|r| r.verified)
    };
    assert!(check(&log));
    log.header.config.forecast_cold_max_speed_m_s = Some(14.);
    assert!(!check(&log));
    log.header.config.forecast_cold_max_speed_m_s = Some(15.);
    log.ticks[1].expected.cold_motion.as_mut().unwrap().tracks[0].confirmed = true;
    assert!(!check(&log));
}

#[test]
fn lidar_failure_rearms_motion_support_and_cached_scenes_cannot_confirm_it() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for time in [0., 0.1, 0.2] {
        driver.step(&frame(time)).unwrap();
    }
    let mut failed = frame(0.3);
    failed.lidar = None;
    failed.lidar_failed = true;
    let out = driver.step(&failed).unwrap();
    assert!(out.emergency);
    assert_eq!(out.cold_motion.unwrap().tracks[0].acquisition_count, 0);
    let mut cached = frame(0.35);
    cached.lidar = None;
    let out = driver.step(&cached).unwrap();
    assert_eq!(out.cold_motion.unwrap().tracks[0].acquisition_count, 0);
    for (i, time) in [0.4, 0.5, 0.6].into_iter().enumerate() {
        let out = driver.step(&frame(time)).unwrap();
        let status = &out.cold_motion.as_ref().unwrap().tracks[0];
        assert_eq!(status.acquisition_count, i + 1);
        assert_eq!(status.confirmed, i == 2);
        assert_eq!(status.first_acquisition_s, Some(0.4));
    }
}

#[test]
fn cold_motion_requires_declared_bounded_support_and_is_omitted_when_disabled() {
    for speed in [0., 0.69, 15.1, f64::INFINITY, f64::NAN] {
        let mut c = config();
        c.forecast_cold_max_speed_m_s = Some(speed);
        assert!(c.validate().is_err());
    }
    for missing in 0..4 {
        let mut c = config();
        match missing {
            0 => c.fit_recent_track_motion = false,
            1 => c.enclose_track_measurements = false,
            2 => c.forecast_consistency_guard = false,
            _ => c.forecast_envelope = None,
        }
        assert!(c.validate().is_err());
    }
    let mut c = config();
    c.forecast_cold_max_speed_m_s = None;
    assert!(
        serde_json::to_value(&c)
            .unwrap()
            .get("forecast_cold_max_speed_m_s")
            .is_none()
    );
    let mut driver = DrivingPipeline::new(c).unwrap();
    assert!(
        serde_json::to_value(driver.step(&frame(0.)).unwrap())
            .unwrap()
            .get("cold_motion")
            .is_none()
    );
}
