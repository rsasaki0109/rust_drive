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
    c.settle_obstruction_stops = true;
    c.search_obstruction_stops = true;
    c.lateral_candidate_margin_m = Some(0.35);
    c.replan_checked_approaches = true;
    c
}

#[test]
fn the_policy_requires_retained_search_stops_declared_width_and_supported_velocity_occupancy() {
    assert!(DrivingPipeline::new(config()).is_ok());
    for which in 0..5 {
        let mut c = config();
        match which {
            0 => c.settle_obstruction_stops = false,
            1 => c.search_obstruction_stops = false,
            2 => c.lateral_candidate_margin_m = None,
            3 => c.forecast_recent_velocity_hypotheses = false,
            _ => c.forecast_cold_max_speed_m_s = None,
        }
        assert!(DrivingPipeline::new(c).is_err());
    }
    let mut c = config();
    c.replan_checked_approaches = false;
    assert!(
        serde_json::to_value(c)
            .unwrap()
            .get("replan_checked_approaches")
            .is_none()
    );
    let mut value = serde_json::to_value(config()).unwrap();
    value["replan_checked_approaches"] = serde_json::json!(1);
    assert!(serde_json::from_value::<PipelineConfig>(value).is_err());
}

#[test]
fn sensor_replay_binds_the_new_confirmation_record_and_its_header_policy() {
    let c = config();
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("checked-approach", c);
    for i in 0..7 {
        let time = i as f64 * 0.05;
        let frame: SensorFrame = serde_json::from_value(serde_json::json!({"time":time,
            "odometry":{"stamp":time,"speed":0.,"yaw_rate":0.},
            "gnss":{"stamp":time,"position":{"x":0.,"y":0.},"variance":0.02},
            "lidar":{"stamp":time,"points":[]}}))
        .unwrap();
        let out = driver.step(&frame).unwrap();
        assert_eq!(
            out.checked_approach
                .as_ref()
                .unwrap()
                .supporting_acquisitions,
            0
        );
        assert!(!out.checked_approach.as_ref().unwrap().adopted);
        log.record(frame, out);
    }
    let check = |log: &SensorLog| {
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        verify(std::io::Cursor::new(bytes), vec![]).is_ok_and(|r| r.verified)
    };
    assert!(check(&log));
    log.header.config.replan_checked_approaches = false;
    assert!(!check(&log));
    log.header.config.replan_checked_approaches = true;
    log.ticks[4]
        .expected
        .checked_approach
        .as_mut()
        .unwrap()
        .supporting_acquisitions = 1;
    assert!(!check(&log));
}
