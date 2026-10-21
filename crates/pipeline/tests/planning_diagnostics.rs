use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};

fn config(enabled: bool) -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.record_planning_diagnostics = enabled;
    c.lateral_candidate_margin_m = Some(0.35);
    c.refine_lateral_candidates = true;
    c
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
            position: Vec2::default(),
            variance: 0.02,
        }),
        lidar: Some(LidarScan {
            stamp: time,
            points: [(0., 0.5), (7.1, 1.5), (-7.1, 1.5)]
                .into_iter()
                .flat_map(|(y, r)| {
                    (0..48).map(move |i| {
                        let angle = std::f64::consts::TAU * i as f64 / 48.;
                        Vec2::new(25. + r * angle.cos(), y + r * angle.sin())
                    })
                })
                .collect(),
        }),
        multi_height_lidar: None,
        lidar3d: None,
        lidar_failed: i == 8,
        navigation_update: None,
        traffic_signal: None,
    }
}

#[test]
fn capturing_decisions_preserves_every_operational_output_through_a_sensor_fault() {
    let mut plain = DrivingPipeline::new(config(false)).unwrap();
    let mut observed = DrivingPipeline::new(config(true)).unwrap();
    for i in 0..18 {
        let input = frame(i);
        let expected = plain.step(&input).unwrap();
        let actual = observed.step(&input).unwrap();
        let mut projected = serde_json::to_value(&actual).unwrap();
        let report = projected
            .as_object_mut()
            .unwrap()
            .remove("planning_diagnostics")
            .unwrap();
        assert_eq!(projected, serde_json::to_value(expected).unwrap());
        assert_eq!(report["candidates"].as_array().unwrap().len(), 9);
        if i == 8 {
            assert!(actual.emergency);
            assert!(actual.command.acceleration <= -5.99);
            assert!(report["scene_acquisition_s"].is_null());
        }
        if let Some(index) = actual
            .planning_diagnostics
            .as_ref()
            .unwrap()
            .selected_candidate
        {
            let selected = actual
                .planning_diagnostics
                .as_ref()
                .unwrap()
                .candidates
                .iter()
                .find(|c| c.index == index)
                .unwrap();
            assert_eq!(selected.lateral_target_m, actual.trajectory.lateral_target);
        }
    }
    assert!(
        serde_json::to_value(config(false))
            .unwrap()
            .get("record_planning_diagnostics")
            .is_none()
    );
}

#[test]
fn complete_replay_reconstructs_diagnostics_and_rejects_changed_reasons_or_capture_policy() {
    let c = config(true);
    let mut pipeline = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("planning-decisions", c);
    for i in 0..18 {
        let input = frame(i);
        let out = pipeline.step(&input).unwrap();
        log.record(input, out);
    }
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
            "\"record_planning_diagnostics\":true",
            "\"record_planning_diagnostics\":false",
        ),
        ("\"prediction_id\":1", "\"prediction_id\":99"),
    ] {
        let changed = original.replacen(from, to, 1);
        assert_ne!(original, changed);
        assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
    }
}
