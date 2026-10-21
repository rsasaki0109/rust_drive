use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, PipelineConfig, PlanningProgressState, SensorFrame,
    replay::{SensorLog, verify},
};

fn config(enabled: bool) -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 1.6).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.monitor_planning_progress = enabled;
    c.settle_obstruction_stops = true;
    c.search_obstruction_stops = true;
    c.lateral_candidate_margin_m = Some(0.35);
    c.forecast_envelope = Some(rustdriving_core::ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    });
    c
}

fn frame(i: usize) -> SensorFrame {
    let time = i as f64 * 0.05;
    SensorFrame {
        time,
        absolute_heading: None,
        odometry: Some(Odometry {
            stamp: time,
            speed: if i <= 1 { 8. } else { 0. },
            yaw_rate: 0.,
        }),
        gnss: Some(Gnss {
            stamp: time,
            position: Vec2::default(),
            variance: 0.0001,
        }),
        lidar: Some(LidarScan {
            sweep: None,
            stamp: (i / 2) as f64 * 0.1,
            points: (0..48)
                .map(|k| {
                    let angle = std::f64::consts::TAU * k as f64 / 48.;
                    Vec2::new(18. + angle.cos(), angle.sin())
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
fn monitor_only_mode_preserves_operational_bytes_without_exposing_candidate_capture() {
    let mut plain = DrivingPipeline::new(config(false)).unwrap();
    let mut observed = DrivingPipeline::new(config(true)).unwrap();
    let mut detected = false;
    for i in 0..70 {
        let mut input = frame(i);
        input.lidar_failed = i == 55;
        let expected = plain.step(&input).unwrap();
        let mut actual = observed.step(&input).unwrap();
        assert!(actual.planning_diagnostics.is_none());
        let report = actual.planning_progress.take().unwrap();
        if i == 55 {
            assert!(actual.emergency);
            assert_eq!(report.supporting_acquisitions, 0);
        }
        detected |= report.state == PlanningProgressState::StationaryForecastBlocked;
        assert_eq!(
            serde_json::to_vec(&actual).unwrap(),
            serde_json::to_vec(&expected).unwrap()
        );
    }
    assert!(detected, "the stationary checked stop must be observable");
    assert!(
        !serde_json::to_string(&config(false))
            .unwrap()
            .contains("monitor_planning_progress")
    );
}

#[test]
fn replay_recomputes_monitor_evidence_and_rejects_changed_state_or_policy() {
    let c = config(true);
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("planning-progress", c);
    for i in 0..60 {
        let input = frame(i);
        let output = p.step(&input).unwrap();
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(&bytes), vec![])
            .unwrap()
            .verified
    );
    let original = String::from_utf8(bytes).unwrap();
    for (from, to) in [
        ("StationaryForecastBlocked", "Observing"),
        (
            "\"supporting_acquisitions\":1,",
            "\"supporting_acquisitions\":2,",
        ),
        (
            "\"monitor_planning_progress\":true",
            "\"monitor_planning_progress\":false",
        ),
    ] {
        let changed = original.replacen(from, to, 1);
        assert!(original != changed, "missing corruption target {from}");
        assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
    }
}

#[test]
fn config_rejects_a_mistyped_monitor_policy() {
    let mut value = serde_json::to_value(config(false)).unwrap();
    value["monitor_planning_progress"] = serde_json::json!(1);
    assert!(serde_json::from_value::<PipelineConfig>(value).is_err());
}
