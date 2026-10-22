use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::{
    DrivingPipeline, PipelineConfig, SensorFrame,
    replay::{SensorLog, verify},
};

fn config() -> PipelineConfig {
    let mut c = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    c.lateral_candidate_margin_m = Some(0.35);
    c.refine_lateral_candidates = true;
    c
}

#[test]
fn refinement_requires_explicit_width_fitting_and_omits_the_disabled_schema_field() {
    let mut c = config();
    c.lateral_candidate_margin_m = None;
    assert!(DrivingPipeline::new(c).is_err());
    let mut c = config();
    c.refine_lateral_candidates = false;
    assert!(
        serde_json::to_value(c)
            .unwrap()
            .get("refine_lateral_candidates")
            .is_none()
    );
}

#[test]
fn sensor_only_replay_binds_the_new_candidate_policy_and_required_fault_braking() {
    let c = config();
    let mut pipeline = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("refined-lateral", c);
    for i in 0..=16 {
        let t = i as f64 * 0.05;
        let input = SensorFrame {
            time: t,
            absolute_heading: None,
            odometry: Some(Odometry {
                stamp: t,
                speed: 0.,
                yaw_rate: 0.,
            }),
            gnss: Some(Gnss {
                stamp: t,
                position: Vec2::default(),
                variance: 0.02,
            }),
            lidar: Some(LidarScan {
                sweep: None,
                stamp: t,
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
        };
        let out = pipeline.step(&input).unwrap();
        if i == 8 {
            assert!(out.emergency);
            assert!(out.command.acceleration <= -5.99);
        } else if (4..8).contains(&i) || i >= 13 {
            assert_eq!(out.trajectory.lateral_target, 3.0625);
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
    let original = String::from_utf8(bytes).unwrap();
    let changed = original.replacen(
        "\"refine_lateral_candidates\":true",
        "\"refine_lateral_candidates\":false",
        1,
    );
    assert_ne!(original, changed);
    assert!(verify(std::io::Cursor::new(changed), vec![]).is_err());
}
