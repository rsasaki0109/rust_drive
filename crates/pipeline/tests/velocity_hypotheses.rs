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
    c.forecast_recent_velocity_hypotheses = true;
    c.forecast_envelope = Some(ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    });
    c
}
fn frame(time: f64) -> SensorFrame {
    serde_json::from_value(serde_json::json!({"time":time,
        "odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
        "gnss":{"stamp":time,"position":{"x":0.,"y":0.},"variance":0.02},
        "lidar":{"stamp":time,"points":(0..8).map(|i| {
            let a=std::f64::consts::PI-0.5+i as f64/7.;Vec2::new(30.+2.5*time+a.cos(),4.+a.sin())
        }).collect::<Vec<_>>()}}))
    .unwrap()
}

#[test]
fn only_confirmed_acquisitions_enter_history_without_changing_enclosed_predictions() {
    let c = config();
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("recent-velocity-points", c.clone());
    let mut old_c = c;
    old_c.forecast_recent_velocity_hypotheses = false;
    let mut old = DrivingPipeline::new(old_c).unwrap();
    for i in 0_usize..13 {
        let mut input = frame(i as f64 * 0.05);
        if !i.is_multiple_of(2) {
            input.lidar = None;
        }
        let out = driver.step(&input).unwrap();
        let previous = old.step(&input).unwrap();
        let h = &out.recent_velocity_hypotheses.as_ref().unwrap().tracks[0];
        assert_eq!(h.sample_count, if i < 4 { 0 } else { i / 2 - 1 });
        assert_eq!(h.first_acquisition_s, if i < 4 { None } else { Some(0.2) });
        let mut projected = serde_json::to_value(&out).unwrap();
        projected
            .as_object_mut()
            .unwrap()
            .remove("recent_velocity_hypotheses");
        assert_eq!(projected, serde_json::to_value(previous).unwrap());
        log.record(input, out);
    }
    let check = |log: &SensorLog| {
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        verify(std::io::Cursor::new(bytes), vec![]).is_ok_and(|r| r.verified)
    };
    assert!(check(&log));
    log.header.config.forecast_recent_velocity_hypotheses = false;
    assert!(!check(&log));
    log.header.config.forecast_recent_velocity_hypotheses = true;
    log.ticks[4]
        .expected
        .recent_velocity_hypotheses
        .as_mut()
        .unwrap()
        .tracks[0]
        .sample_count += 1;
    assert!(!check(&log));
}

#[test]
fn failed_scans_clear_history_and_old_cached_motion_cannot_reseed_it() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for time in [0., 0.1, 0.2] {
        driver.step(&frame(time)).unwrap();
    }
    let mut fault = frame(0.3);
    fault.lidar = None;
    fault.lidar_failed = true;
    let out = driver.step(&fault).unwrap();
    assert!(out.emergency);
    assert_eq!(
        out.recent_velocity_hypotheses.unwrap().tracks[0].sample_count,
        0
    );
    for time in [0.35, 0.4, 0.5, 0.6] {
        let mut input = frame(time);
        if time == 0.35 {
            input.lidar = None;
        }
        let out = driver.step(&input).unwrap();
        let h = &out.recent_velocity_hypotheses.as_ref().unwrap().tracks[0];
        assert_eq!(h.sample_count, usize::from(time == 0.6));
        assert_eq!(
            h.first_acquisition_s,
            if time == 0.6 { Some(0.6) } else { None }
        );
    }
}

#[test]
fn recent_velocity_requires_cold_support_and_disabled_serialization_stays_omitted() {
    let mut c = config();
    c.forecast_cold_max_speed_m_s = None;
    assert!(c.validate().is_err());
    c.forecast_recent_velocity_hypotheses = false;
    assert!(c.validate().is_ok());
    assert!(
        serde_json::to_value(&c)
            .unwrap()
            .get("forecast_recent_velocity_hypotheses")
            .is_none()
    );
    let mut driver = DrivingPipeline::new(c).unwrap();
    assert!(
        serde_json::to_value(driver.step(&frame(0.)).unwrap())
            .unwrap()
            .get("recent_velocity_hypotheses")
            .is_none()
    );
}
