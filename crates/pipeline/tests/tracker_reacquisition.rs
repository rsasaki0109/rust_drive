use rustdriving_core::{Gnss, LidarScan, Odometry, Pose, Route, Vec2, VehicleConfig};
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, PipelineConfig, PipelineOutput, SensorFrame};

fn run(intervening_empty_scans: bool) -> PipelineOutput {
    let config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    let mut driver = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("per-track-acquisition-interval", config);
    for i in 0..=13 {
        let time = i as f64 * 0.05;
        let observation = match i {
            0 => Some((0., 10.)),
            2 => Some((0.1, 11.)),
            // Delayed delivery: the last measurement was at 0.1 s, so
            // the tracked displacement spans 0.5 s, not the 0.1 s scan gap.
            13 => Some((0.6, 12.5)),
            _ => None,
        };
        let lidar = observation
            .map(|(stamp, x)| LidarScan {
                stamp,
                points: (0..32)
                    .map(|j| {
                        let a = j as f64 * std::f64::consts::TAU / 32.;
                        Vec2::new(x + a.cos(), 3. + a.sin())
                    })
                    .collect(),
            })
            .or_else(|| {
                (intervening_empty_scans && matches!(i, 4 | 6 | 8 | 10)).then_some(LidarScan {
                    stamp: time,
                    points: vec![],
                })
            });
        let input = SensorFrame {
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
            lidar,
            multi_height_lidar: None,
            lidar3d: None,
            lidar_failed: false,
            navigation_update: None,
            traffic_signal: None,
        };
        let output = driver.step(&input).unwrap();
        log.record(input, output);
    }
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), vec![]).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 14);
    log.ticks.pop().unwrap().expected
}

#[test]
fn raw_delayed_reacquisition_uses_the_tracks_observation_interval_and_forecast_age() {
    let sparse = run(false);
    let busy = run(true);
    for output in [&sparse, &busy] {
        assert!(output.health.is_empty());
        assert_eq!(output.tracks.len(), 1);
        let t = &output.tracks[0];
        assert_eq!(t.id, 1);
        assert_eq!(t.last_seen, 0.6);
        assert!(t.position.distance(Vec2::new(12.2125, 3.)) < 1e-9);
        assert!(t.velocity.distance(Vec2::new(1.476, 0.)) < 1e-9);
        let p = &output.predictions[0];
        assert_eq!(p.id, 1);
        assert_eq!(p.positions.len(), 41);
        // Acquisition age is 0.05 s at delivery; forecasts start at that
        // control clock and continue with the same measured velocity.
        for (i, point) in p.positions.iter().enumerate() {
            let expected = Vec2::new(12.2125 + 1.476 * (0.05 + i as f64 * 0.2), 3.);
            assert!(point.distance(expected) < 1e-8);
        }
    }
    assert!(sparse.tracks[0].velocity.distance(busy.tracks[0].velocity) < 1e-12);
}
