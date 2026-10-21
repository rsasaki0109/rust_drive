use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, PipelineConfig, SensorFrame,
    navigation::{NavigationConfig, ObservedAvoidanceDecision},
    replay::{SensorLog, verify},
};

fn config() -> PipelineConfig {
    let scene: serde_json::Value =
        serde_json::from_str(include_str!("../../../scenarios/route-direct.json")).unwrap();
    let mut nav: NavigationConfig = serde_json::from_value(scene["navigation"].clone()).unwrap();
    nav.hold_for_handover = true;
    nav.avoid_observed_obstructions = true;
    let mut config = PipelineConfig::new(
        nav.initial_plan().unwrap().route,
        Pose {
            position: Vec2::new(20., 0.),
            yaw: 0.,
        },
        VehicleConfig::default(),
    );
    config.navigation = Some(nav);
    config.cruise_speed = 4.;
    config.enclose_track_measurements = true;
    config.lateral_candidate_margin_m = Some(0.35);
    config.forecast_envelope = Some(rustdriving_core::ForecastEnvelopeConfig {
        position_error_m: 0.2,
        velocity_error_m_s: 0.3,
        acceleration_error_m_s2: 0.1,
    });
    config
}
fn frame(i: usize) -> SensorFrame {
    let time = i as f64 * 0.05;
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
            position: Vec2::new(20., 0.),
            variance: 0.02,
        }),
        lidar: i.is_multiple_of(2).then(|| LidarScan {
            stamp: time,
            sweep: None,
            points: (0..48)
                .map(|k| {
                    let a = std::f64::consts::TAU * k as f64 / 48.;
                    Vec2::new(45. + a.cos(), a.sin())
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
fn sensor_only_detour_uses_internal_decisions_and_replays_without_capture_outputs() {
    let c = config();
    let mut p = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("observed-detour", c);
    for i in 0..14 {
        let input = frame(i);
        let output = p.step(&input).unwrap();
        assert!(output.planning_diagnostics.is_none());
        let nav = output.navigation.as_ref().unwrap();
        assert!(nav.closed_edges.is_empty());
        assert_eq!(nav.revision, 0);
        if (6..8).contains(&i) {
            assert!(nav.handover_hold);
            assert!(output.command.acceleration <= -0.5);
        }
        if i >= 8 {
            assert_eq!(nav.switches, 1);
            assert!(nav.active_edges.contains(&"detour".into()));
        }
        log.record(input, output);
    }
    assert_eq!(
        log.ticks[6]
            .expected
            .navigation
            .as_ref()
            .unwrap()
            .observed_obstructions
            .as_ref()
            .unwrap()
            .last_decision,
        Some(ObservedAvoidanceDecision::DetourRequested)
    );
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(&bytes), vec![])
            .unwrap()
            .verified
    );
    let original = String::from_utf8(bytes).unwrap();
    for (a, b) in [
        (
            "\"avoid_observed_obstructions\":true",
            "\"avoid_observed_obstructions\":false",
        ),
        ("\"prediction_id\":1", "\"prediction_id\":99"),
        ("DetourRequested", "NoAlternative"),
    ] {
        let changed = original.replacen(a, b, 1);
        assert!(changed != original, "missing corruption target {a}");
        assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
    }
}

#[test]
fn a_lidar_fault_clears_votes_and_recovery_requires_a_new_complete_acquired_prefix() {
    let mut p = DrivingPipeline::new(config()).unwrap();
    for i in 0..12 {
        let mut input = frame(i);
        input.lidar_failed = i == 3;
        let output = p.step(&input).unwrap();
        let nav = output.navigation.as_ref().unwrap();
        let observed = nav.observed_obstructions.as_ref().unwrap();
        if i == 3 {
            assert!(output.emergency);
            assert!(observed.candidate.is_none());
        }
        if i < 10 {
            assert!(observed.retained_edges.is_empty());
        }
        if i == 10 {
            assert_eq!(observed.retained_edges.len(), 1);
            assert_eq!(
                observed.retained_edges[0].evidence.first_acquisition_s,
                frame(6).time
            );
        }
    }
}

#[test]
fn automatic_observation_requires_braked_handover() {
    let mut c = config();
    c.navigation.as_mut().unwrap().hold_for_handover = false;
    assert!(c.validate().unwrap_err().contains("braked handover"));
}
