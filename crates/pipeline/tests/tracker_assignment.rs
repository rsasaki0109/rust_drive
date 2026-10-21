use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, PipelineOutput, SensorFrame};

fn circles(centers: &[f64]) -> Vec<Vec2> {
    centers
        .iter()
        .flat_map(|x| {
            (0..32).map(move |i| {
                let a = i as f64 * std::f64::consts::TAU / 32.;
                Vec2::new(x + 0.5 * a.cos(), 3. + 0.5 * a.sin())
            })
        })
        .collect()
}

fn run(reversed: bool) -> PipelineOutput {
    let config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("global-track-assignment-circles", config);
    for i in 0..=10 {
        let time = i as f64 * 0.05;
        let points = match i {
            0 => Some(circles(&[10., 12.])),
            10 => Some(circles(if reversed { &[7.9, 10.9] } else { &[10.9, 7.9] })),
            _ => None,
        };
        let input = SensorFrame {
            absolute_heading: None,
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
            lidar: points.map(|points| LidarScan {
                sweep: None,
                stamp: time,
                points,
            }),
            multi_height_lidar: None,
            lidar3d: None,
            lidar_failed: false,
            navigation_update: None,
            traffic_signal: None,
        };
        let output = pipeline.step(&input).unwrap();
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), vec![]).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 11);
    log.ticks.pop().unwrap().expected
}

#[test]
fn raw_cluster_order_cannot_steal_an_existing_identity_on_reacquisition() {
    let a = run(false);
    let b = run(true);
    for output in [&a, &b] {
        assert!(output.health.is_empty());
        assert_eq!(output.tracks.len(), 2, "{:?}", output.tracks);
        for (t, x, v) in [
            (&output.tracks[0], 8.425, -0.504),
            (&output.tracks[1], 11.175, -0.264),
        ] {
            assert_eq!(t.last_seen, 0.5);
            assert!(t.position.distance(Vec2::new(x, 3.)) < 1e-8);
            assert!(t.velocity.distance(Vec2::new(v, 0.)) < 1e-8);
        }
        assert_eq!((output.tracks[0].id, output.tracks[1].id), (1, 2));
    }
    assert_eq!(
        serde_json::to_value(a.tracks).unwrap(),
        serde_json::to_value(b.tracks).unwrap()
    );
}

#[test]
fn crowded_measured_components_brake_without_partial_updates_and_recover_on_new_acquisitions() {
    let config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("crowded-measured-association", config);
    let points = |shift| {
        (0..72)
            .flat_map(|i| {
                let p = Vec2::new(5. + (i % 9) as f64 * 0.6 + shift, 4. + (i / 9) as f64 * 0.6);
                [p, p.plus(Vec2::new(0.01, 0.)), p.plus(Vec2::new(0., 0.01))]
            })
            .collect::<Vec<_>>()
    };
    let mut initial_tracks = None;
    let mut initial_grid = None;
    for (time, stamp) in [(0., 0.), (0.1, 0.1), (0.15, 0.1), (0.2, 0.2)] {
        let input = SensorFrame {
            absolute_heading: None,
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
                sweep: None,
                stamp,
                points: if time == 0.2 {
                    vec![]
                } else {
                    points(if time == 0. { 0. } else { 0.1 })
                },
            }),
            multi_height_lidar: None,
            lidar3d: None,
            lidar_failed: false,
            navigation_update: None,
            traffic_signal: None,
        };
        let output = pipeline.step(&input).unwrap();
        if time == 0. {
            assert!(output.health.is_empty());
            assert_eq!(output.tracks.len(), 72);
            initial_tracks = Some(serde_json::to_value(&output.tracks).unwrap());
            initial_grid = Some(pipeline.occupied_cells());
        } else if time < 0.2 {
            assert!(
                output
                    .health
                    .contains(&rustdriving_pipeline::HealthIssue::InvalidLidar)
            );
            assert!(output.emergency);
            assert_eq!(output.command.acceleration, -6.);
            assert_eq!(
                Some(serde_json::to_value(&output.tracks).unwrap()),
                initial_tracks
            );
            assert_eq!(Some(pipeline.occupied_cells()), initial_grid);
        } else {
            assert!(output.health.is_empty());
            assert_eq!(output.tracks.len(), 72);
            assert!(output.tracks.iter().all(|t| t.last_seen == 0.));
        }
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), vec![]).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 4);
}
