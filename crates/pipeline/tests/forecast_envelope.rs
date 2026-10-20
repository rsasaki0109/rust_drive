use rustdriving_core::{
    ForecastEnvelopeConfig, Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig,
};
use rustdriving_pipeline::{
    DrivingPipeline, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};

fn config() -> PipelineConfig {
    PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 2.1).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    )
}
fn bounds() -> ForecastEnvelopeConfig {
    ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    }
}
fn frame(time: f64, stamp: f64) -> SensorFrame {
    SensorFrame {
        time,
        odometry: Some(Odometry {
            stamp: time,
            speed: 0.,
            yaw_rate: 0.,
        }),
        gnss: Some(Gnss {
            stamp: time,
            position: Vec2::default(),
            variance: 0.02,
        }),
        lidar: Some(LidarScan {
            stamp,
            points: (0..32)
                .map(|j| {
                    let angle = j as f64 * std::f64::consts::TAU / 32.;
                    Vec2::new(
                        30. + 0.7 * angle.cos(),
                        5. - 0.5 * stamp + 0.7 * angle.sin(),
                    )
                })
                .collect(),
        }),
        multi_height_lidar: None,
        lidar3d: None,
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}

#[test]
fn delayed_acquisition_envelope_is_operational_and_replay_rejects_missing_or_changed_policy() {
    let mut c = config();
    c.forecast_envelope = Some(bounds());
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("delayed-envelope", c);
    for i in 0..=40 {
        let time = i as f64 * 0.05;
        let input = frame(time, (time - 0.1).max(0.));
        let out = driver.step(&input).unwrap();
        log.record(input, out);
    }
    let out = &log.ticks.last().unwrap().expected;
    let t = &out.tracks[0];
    let p = &out.predictions[0];
    assert!(t.velocity.y.abs() > 0.4 && t.velocity.y.abs() < 0.7);
    let cv = t.position.plus(t.velocity.scaled(8.1));
    assert!(
        cv.distance(p.positions[40]) + 0.2 + 0.3 * 8.1 + 0.05 * 8.1_f64.powi(2)
            <= p.additional_radius_m[40] + 1e-8
    );
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes), vec![])
            .unwrap()
            .verified
    );
    for modified in [
        None,
        Some(ForecastEnvelopeConfig {
            velocity_error_m_s: 0.1,
            ..bounds()
        }),
    ] {
        let mut changed = log.clone();
        changed.header.config.forecast_envelope = modified;
        let mut bytes = vec![];
        changed.write(&mut bytes).unwrap();
        assert!(verify(std::io::Cursor::new(bytes), vec![]).is_err());
    }
}

#[test]
fn an_uncertain_remote_object_withdraws_the_otherwise_clear_driving_candidate() {
    let mut plain = DrivingPipeline::new(config()).unwrap();
    let mut c = config();
    c.forecast_envelope = Some(ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 2.,
        acceleration_error_m_s2: 0.,
    });
    let mut cautious = DrivingPipeline::new(c).unwrap();
    let input = frame(0., 0.);
    let clear = plain.step(&input).unwrap();
    let covered = cautious.step(&input).unwrap();
    assert!(clear.command.acceleration > 0.);
    assert!(covered.command.acceleration <= 0.);
    assert!(covered.predictions[0].radius_at(8.) > 16.);
}

#[test]
fn malformed_configuration_is_refused_and_absent_policy_keeps_the_legacy_header() {
    assert!(
        serde_json::to_value(config())
            .unwrap()
            .get("forecast_envelope")
            .is_none()
    );
    let mut c = config();
    c.forecast_envelope = Some(ForecastEnvelopeConfig {
        acceleration_error_m_s2: -0.1,
        ..bounds()
    });
    assert!(DrivingPipeline::new(c).is_err());
    let mut value = serde_json::to_value(config()).unwrap();
    value["forecast_envelope"] = serde_json::json!({"position_error_m": 0.2, "velocity_error_m_s": 0.3, "acceleration_error_m_s2": 0.1, "confidence": 0.99});
    assert!(serde_json::from_value::<PipelineConfig>(value).is_err());
}
