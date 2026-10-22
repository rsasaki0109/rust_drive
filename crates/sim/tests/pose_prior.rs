use rustdriving_sim::{Scenario, pipeline_config, simulate};
#[test]
fn physical_spawn_misalignment_is_excluded_from_replay_configuration() {
    let mut scenario: Scenario =
        serde_json::from_str(include_str!("../../../scenarios/mid-edge-same-edge.json")).unwrap();
    scenario.initial_pose_covariance = Some([
        [1., 0., 0.],
        [0., 1., 0.],
        [0., 0., (3_f64.to_radians()).powi(2) / 3.],
    ]);
    scenario.initial_heading_offset_rad = Some(2_f64.to_radians());
    let config = pipeline_config(&scenario);
    let calibrated = config.initial_pose;
    let run = simulate(scenario, 7).unwrap();
    assert!(run.summary.passed, "{:?}", run.summary);
    assert!((run.frames[0].truth.pose.yaw - calibrated.yaw - 2_f64.to_radians()).abs() < 1e-14);
    let log = run.sensor_log.as_ref().unwrap();
    let value = serde_json::to_value(&log.header.config).unwrap();
    assert!(value.get("initial_heading_offset_rad").is_none());
    assert_eq!(log.header.config.initial_pose.yaw, calibrated.yaw);
    assert!(run.frames.iter().all(|f| {
        rustdriving_core::wrap_angle(f.truth.pose.yaw - f.estimate.pose.yaw).abs() <= 0.1
    }));
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        rustdriving_pipeline::replay::verify(std::io::Cursor::new(bytes), vec![])
            .unwrap()
            .verified
    );
}
#[test]
fn nonfinite_or_excessive_truth_offsets_fail_before_physics_initialization() {
    let mut scenario: Scenario =
        serde_json::from_str(include_str!("../../../scenarios/mid-edge-same-edge.json")).unwrap();
    for value in [f64::NAN, f64::INFINITY, -0.21, 0.21] {
        scenario.initial_heading_offset_rad = Some(value);
        assert!(scenario.validate().is_err());
    }
}

#[test]
fn local_reference_side_paths_finish_both_new_offset_obstacle_regressions() {
    for seed in [505, 606] {
        let mut scenario: Scenario =
            serde_json::from_str(include_str!("../../../scenarios/mid-edge-obstacle.json"))
                .unwrap();
        scenario.initial_pose_covariance = Some([
            [1., 0., 0.],
            [0., 1., 0.],
            [0., 0., (3_f64.to_radians()).powi(2) / 3.],
        ]);
        scenario.initial_heading_offset_rad = Some(-2_f64.to_radians());
        let r = simulate(scenario, seed).unwrap();
        assert!(r.summary.passed, "seed {seed}: {:?}", r.summary);
        assert!(
            r.summary.reached_goal
                && r.summary.min_clearance >= 1.
                && r.summary.simulated_seconds <= 65.
        );
    }
}
