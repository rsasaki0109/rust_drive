use rustdriving_core::*;
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame};

fn config() -> PipelineConfig {
    PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 5.5).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    )
}
fn frame(time: f64) -> SensorFrame {
    SensorFrame {
        time,
        odometry: Some(Odometry {
            stamp: time,
            speed: 4.0,
            yaw_rate: 0.8,
        }),
        gnss: (time == 0.0).then_some(Gnss {
            stamp: 0.0,
            position: Vec2::default(),
            variance: 0.02,
        }),
        lidar: None,
        multi_height_lidar: None,
        lidar3d: None,
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}
#[test]
fn delayed_body_scan_keeps_a_stationary_world_track_while_ego_translates_and_turns() {
    let c = config();
    let mut driver = DrivingPipeline::new(c.clone()).unwrap();
    let mut log = SensorLog::new("delayed-turning-observations", c);
    let center = Vec2::new(20.0, 3.0);
    for i in 0..=6 {
        let mut input = frame(i as f64 * 0.05);
        if i >= 2 && i % 2 == 0 {
            let acquisition_tick = i - 2;
            // Independent integration of the declared exact wheel/gyro observations.
            let mut acquired_position = Vec2::default();
            for j in 0..acquisition_tick {
                let yaw = j as f64 * 0.04;
                acquired_position =
                    acquired_position.plus(Vec2::new(yaw.cos(), yaw.sin()).scaled(0.2));
            }
            let acquired_yaw = acquisition_tick as f64 * 0.04;
            input.lidar = Some(LidarScan {
                stamp: acquisition_tick as f64 * 0.05,
                points: (0..32)
                    .map(|j| {
                        let angle = j as f64 * std::f64::consts::TAU / 32.0;
                        center
                            .plus(Vec2::new(angle.cos(), angle.sin()))
                            .minus(acquired_position)
                            .rotated(-acquired_yaw)
                    })
                    .collect(),
            });
        }
        let output = driver.step(&input).unwrap();
        if i >= 2 {
            assert!(output.health.is_empty());
            assert_eq!(output.tracks.len(), 1);
            let track = &output.tracks[0];
            assert!(track.position.distance(center) < 1e-8);
            assert!(track.velocity.x.hypot(track.velocity.y) < 1e-8);
            assert!(track.last_seen <= input.time - 0.1 + 1e-9);
        }
        log.record(input, output);
    }
    assert!(
        driver
            .occupied_cells()
            .iter()
            .any(|p| p.distance(center) < 2.0)
    );
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    let report = verify(std::io::Cursor::new(bytes), vec![]).unwrap();
    assert!(report.verified);
    assert_eq!(report.ticks, 7);
}
#[test]
fn a_scan_before_history_start_is_rejected_and_a_current_acquisition_recovers() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    let mut input = frame(1.0);
    input.gnss = Some(Gnss {
        stamp: 1.0,
        position: Vec2::default(),
        variance: 0.02,
    });
    input.lidar = Some(LidarScan {
        stamp: 0.9,
        points: vec![],
    });
    let bad = driver.step(&input).unwrap();
    assert!(bad.health.contains(&HealthIssue::InvalidLidar));
    assert!(bad.health.contains(&HealthIssue::StaleLidar));
    assert!(bad.emergency);
    input.time = 1.05;
    input.odometry.as_mut().unwrap().stamp = 1.05;
    input.lidar.as_mut().unwrap().stamp = 1.05;
    let good = driver.step(&input).unwrap();
    assert!(good.health.is_empty());
    assert!(!good.emergency);
}
#[test]
fn an_expired_new_scan_cannot_refresh_health_or_update_tracks() {
    let mut driver = DrivingPipeline::new(config()).unwrap();
    for i in 0..=8 {
        let mut input = frame(i as f64 * 0.05);
        if i == 0 {
            input.lidar = Some(LidarScan {
                stamp: 0.0,
                points: vec![],
            });
        }
        if i == 8 {
            input.lidar = Some(LidarScan {
                stamp: 0.01,
                points: vec![],
            });
        }
        let out = driver.step(&input).unwrap();
        if i == 8 {
            assert!(out.health.contains(&HealthIssue::InvalidLidar));
            assert!(out.health.contains(&HealthIssue::StaleLidar));
            assert!(out.emergency);
            assert!(out.tracks.is_empty());
        }
    }
}
