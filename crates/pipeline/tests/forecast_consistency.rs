use rustdriving_core::{
    ForecastConsistencyState as State, ForecastEnvelopeConfig, Gnss, LidarScan, Odometry, Pose,
    Route, Vec2, VehicleConfig,
};
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame,
    navigation::NavigationUpdate,
    replay::{SensorLog, verify},
};

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
    c
}
fn frame(i: usize) -> SensorFrame {
    let time = i as f64 * 0.05;
    let x = if i < 6 { 25. } else { 25.8 };
    SensorFrame {
        time,
        absolute_heading: None,
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
            sweep: None,
            stamp: time,
            points: (0..48)
                .map(|i| {
                    let a = std::f64::consts::TAU * i as f64 / 48.;
                    Vec2::new(x + 0.5 * a.cos(), 0.5 * a.sin())
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
fn required_envelope_and_measured_geometry_are_explicit_and_disabled_schema_stays_omitted() {
    let mut c = config();
    c.forecast_envelope = None;
    assert!(DrivingPipeline::new(c).is_err());
    let mut c = config();
    c.enclose_track_measurements = false;
    assert!(DrivingPipeline::new(c).is_err());
    let mut c = config();
    c.forecast_consistency_guard = false;
    assert!(
        serde_json::to_value(c)
            .unwrap()
            .get("forecast_consistency_guard")
            .is_none()
    );
}

#[test]
fn an_acquired_forecast_breach_brakes_immediately_and_recovers_from_fresh_consistent_evidence() {
    let c = config();
    let mut pipeline = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("forecast-consistency", c);
    let mut saw_hold = false;
    let mut saw_recovery = false;
    for i in 0..60 {
        let input = frame(i);
        let out = pipeline.step(&input).unwrap();
        let diagnostic = out.forecast_consistency.as_ref().unwrap();
        if i == 6 {
            assert_eq!(diagnostic.state, State::Holding);
            assert!(!diagnostic.misses.is_empty());
            assert_eq!(diagnostic.misses[0].measurement_acquisition_s, input.time);
        }
        if matches!(diagnostic.state, State::Holding | State::Confirming) {
            saw_hold = true;
            assert!(out.health.contains(&HealthIssue::InconsistentForecast));
            assert!(out.emergency);
            assert!(out.command.acceleration <= -5.99);
            assert_eq!(out.command.steering, 0.);
        } else if saw_hold && diagnostic.state == State::Observing {
            saw_recovery = true;
            assert!(!out.health.contains(&HealthIssue::InconsistentForecast));
        }
        log.record(input, out);
    }
    assert!(saw_hold && saw_recovery);
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(bytes.clone()), vec![])
            .unwrap()
            .verified
    );
    let original = String::from_utf8(bytes).unwrap();
    for (from, to) in [
        (
            "\"forecast_consistency_guard\":true",
            "\"forecast_consistency_guard\":false",
        ),
        ("\"MeasuredMiss\"", "\"InvalidInput\""),
    ] {
        let changed = original.replacen(from, to, 1);
        assert_ne!(original, changed);
        assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
    }
}

#[test]
fn an_independent_navigation_fault_retains_command_braking_after_monitor_recovery() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let mut recovered = false;
    for i in 0..60 {
        let mut input = frame(i);
        if i >= 6 {
            input.navigation_update = Some(NavigationUpdate {
                stamp: input.time,
                revision: i as u64,
                closed_edges: vec![],
            });
        }
        let out = pipeline.step(&input).unwrap();
        if i >= 6 {
            assert!(out.health.contains(&HealthIssue::InvalidNavigation));
            assert!(out.command.acceleration <= -5.99);
            if i > 20 && out.forecast_consistency.as_ref().unwrap().state == State::Observing {
                recovered = true;
            }
        }
    }
    assert!(recovered);
}

#[test]
fn uncovered_motion_discards_forecast_references_and_cannot_auto_resume_the_old_epoch() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    for i in 0..30 {
        let mut input = frame(i);
        if (8..=11).contains(&i) {
            input.odometry = None;
        }
        let out = pipeline.step(&input).unwrap();
        if i >= 11 {
            assert!(out.health.contains(&HealthIssue::MotionDiscontinuity));
            assert!(out.command.acceleration <= -5.99);
            let report = out.forecast_consistency.unwrap();
            assert_eq!(report.state, State::ResetRequired);
            assert_eq!(report.retained_references, 0);
        }
    }
}
