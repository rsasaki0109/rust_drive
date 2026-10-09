use rustdriving_core::*;
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{
    DrivingPipeline, HealthIssue, Lidar3dConfig, MultiHeightLidarConfig, PipelineConfig,
    SensorFrame,
};
use std::f64::consts::{PI, TAU};

fn calibration() -> Lidar3dConfig {
    Lidar3dConfig {
        azimuth_columns: 720,
        elevation_rings: 16,
        min_elevation_rad: -PI / 12.0,
        max_elevation_rad: PI / 12.0,
        mount_height_m: 0.6,
        min_range_m: 0.2,
        max_range_m: 45.0,
        collision_bottom_m: -1.15,
        collision_top_m: 2.35,
        ground: None,
    }
}
fn config() -> PipelineConfig {
    let mut cfg = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 2.1).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    cfg.lidar3d = Some(calibration());
    cfg
}
fn measured(column: usize, ring: usize, range: f64) -> Lidar3dReturn {
    let azimuth = -PI + TAU * column as f64 / 719.0;
    let elevation = -PI / 12.0 + PI / 6.0 * ring as f64 / 15.0;
    Lidar3dReturn {
        ray_index: column * 16 + ring,
        point: Vec3::new(
            range * elevation.cos() * azimuth.cos(),
            -range * elevation.cos() * azimuth.sin(),
            0.6 + range * elevation.sin(),
        ),
    }
}
fn frame(time: f64) -> SensorFrame {
    SensorFrame {
        time,
        odometry: Some(Odometry {
            stamp: time,
            speed: 0.0,
            yaw_rate: 0.0,
        }),
        gnss: Some(Gnss {
            stamp: time,
            position: Vec2::default(),
            variance: 0.02,
        }),
        lidar: None,
        multi_height_lidar: None,
        lidar3d: Some(Lidar3dScan {
            stamp: time,
            returns: vec![],
        }),
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}
fn obstacle(ring: usize, range: f64) -> Vec<Lidar3dReturn> {
    (355..365).map(|c| measured(c, ring, range)).collect()
}
#[test]
fn actual_mid_height_returns_change_plan_and_map_but_overhead_returns_do_not() {
    let mut input = frame(0.0);
    input.lidar3d.as_mut().unwrap().returns = obstacle(9, 12.0);
    assert!(
        input
            .lidar3d
            .as_ref()
            .unwrap()
            .returns
            .iter()
            .all(|r| r.point.z > 1.2 && r.point.z < 1.3)
    );
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let stopped = pipeline.step(&input).unwrap();
    assert!(stopped.health.is_empty());
    assert_eq!(stopped.tracks.len(), 1);
    assert!(
        stopped
            .trajectory
            .points
            .iter()
            .all(|p| p.position.x < 12.0)
    );
    assert!(!pipeline.occupied_cells().is_empty());
    input.lidar3d.as_mut().unwrap().returns = obstacle(13, 15.0);
    assert!(
        input
            .lidar3d
            .as_ref()
            .unwrap()
            .returns
            .iter()
            .all(|r| r.point.z > 3.0)
    );
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let clear = pipeline.step(&input).unwrap();
    assert!(clear.health.is_empty());
    assert!(clear.tracks.is_empty());
    assert!(clear.trajectory.points.iter().any(|p| p.position.x > 20.0));
    assert!(pipeline.occupied_cells().is_empty());
}
#[test]
fn same_tilted_ray_has_different_measured_height_at_different_ranges() {
    let near = measured(360, 9, 10.0);
    let far = measured(360, 9, 40.0);
    assert_eq!(near.ray_index, far.ray_index);
    assert!(near.point.z < 2.35 && far.point.z > 2.35);
    for (point, expected_tracks) in [(near, 1), (far, 0)] {
        let mut input = frame(0.0);
        let range = point
            .point
            .x
            .hypot(point.point.y)
            .hypot(point.point.z - 0.6);
        input.lidar3d.as_mut().unwrap().returns = obstacle(9, range);
        let output = DrivingPipeline::new(config())
            .unwrap()
            .step(&input)
            .unwrap();
        assert!(output.health.is_empty());
        assert_eq!(output.tracks.len(), expected_tracks);
    }
}
#[test]
fn ordinal_order_is_deterministic_and_overlapping_ring_projections_are_deduplicated() {
    let mut input = frame(0.0);
    input.lidar3d.as_mut().unwrap().returns = obstacle(7, 12.0);
    let original = DrivingPipeline::new(config())
        .unwrap()
        .step(&input)
        .unwrap();
    input
        .lidar3d
        .as_mut()
        .unwrap()
        .returns
        .extend(obstacle(8, 12.0));
    input.lidar3d.as_mut().unwrap().returns.reverse();
    let merged = DrivingPipeline::new(config())
        .unwrap()
        .step(&input)
        .unwrap();
    assert_eq!(
        serde_json::to_value(original).unwrap(),
        serde_json::to_value(merged).unwrap()
    );
}
#[test]
fn malformed_ordinals_xyz_directions_ranges_and_future_stamps_fail_closed() {
    for variant in 0..10 {
        let mut pipeline = DrivingPipeline::new(config()).unwrap();
        pipeline.step(&frame(0.0)).unwrap();
        let mut input = frame(0.05);
        let scan = input.lidar3d.as_mut().unwrap();
        scan.returns = obstacle(9, 12.0);
        match variant {
            0 => scan.returns[0].ray_index = 11_520,
            1 => scan.returns[1].ray_index = scan.returns[0].ray_index,
            2 => scan.returns[0].point.z = f64::NAN,
            3 => scan.returns[0].point.x = f64::INFINITY,
            4 => scan.returns[0].point.y += 0.2,
            5 => scan.returns[0].point.z -= 0.6,
            6 => scan.returns[0] = measured(355, 9, 46.0),
            7 => scan.returns[0] = measured(355, 9, 0.1),
            8 => scan.returns = vec![measured(360, 9, 12.0); 11_521],
            _ => scan.stamp = 0.1,
        }
        let output = pipeline.step(&input).unwrap();
        assert!(
            output.health.contains(&HealthIssue::InvalidLidar),
            "variant {variant}"
        );
        assert_eq!(output.command.acceleration, -6.0);
        assert!(output.tracks.is_empty());
        assert!(pipeline.occupied_cells().is_empty());
        let mut absent = frame(0.1);
        absent.lidar3d = None;
        assert!(pipeline.step(&absent).unwrap().emergency);
        assert!(pipeline.step(&frame(0.15)).unwrap().health.is_empty());
    }
}
#[test]
fn mixed_input_modes_and_conflicting_calibration_are_rejected() {
    for variant in 0..4 {
        let mut cfg = config();
        let mut input = frame(0.0);
        match variant {
            0 => {
                input.lidar = Some(LidarScan {
                    stamp: 0.0,
                    points: vec![],
                })
            }
            1 => {
                input.multi_height_lidar = Some(MultiHeightLidarScan {
                    stamp: 0.0,
                    planes: vec![],
                })
            }
            2 => cfg.lidar3d = None,
            _ => {
                cfg.lidar3d = None;
                cfg.multi_height_lidar = Some(MultiHeightLidarConfig {
                    heights_m: vec![0.15, 0.6],
                    collision_bottom_m: -1.15,
                    collision_top_m: 2.35,
                });
            }
        }
        let output = DrivingPipeline::new(cfg).unwrap().step(&input).unwrap();
        assert!(output.health.contains(&HealthIssue::InvalidLidar));
        assert!(output.emergency);
    }
    let mut cfg = config();
    cfg.multi_height_lidar = Some(MultiHeightLidarConfig {
        heights_m: vec![0.15, 0.6],
        collision_bottom_m: -1.15,
        collision_top_m: 2.35,
    });
    assert!(DrivingPipeline::new(cfg).is_err());
}
#[test]
fn calibration_rejects_nonfinite_overflow_counts_and_invalid_physical_limits() {
    for variant in 0..13 {
        let mut cfg = config();
        let c = cfg.lidar3d.as_mut().unwrap();
        match variant {
            0 => c.azimuth_columns = 1,
            1 => c.elevation_rings = 1,
            2 => c.azimuth_columns = usize::MAX,
            3 => {
                c.azimuth_columns = 2048;
                c.elevation_rings = 64;
            }
            4 => c.min_elevation_rad = f64::NAN,
            5 => c.min_elevation_rad = PI,
            6 => c.max_elevation_rad = c.min_elevation_rad,
            7 => c.mount_height_m = f64::INFINITY,
            8 => c.min_range_m = 0.0,
            9 => c.max_range_m = 0.1,
            10 => c.max_range_m = 201.0,
            11 => c.collision_bottom_m = c.collision_top_m,
            _ => c.collision_top_m = 11.0,
        }
        assert!(DrivingPipeline::new(cfg).is_err(), "variant {variant}");
    }
}
#[test]
fn no_new_scan_duplicates_and_old_acquisitions_never_refresh_health_or_fault_latch() {
    for failed in [false, true] {
        let mut pipeline = DrivingPipeline::new(config()).unwrap();
        pipeline.step(&frame(0.0)).unwrap();
        if failed {
            let mut input = frame(0.05);
            input.lidar_failed = true;
            assert!(pipeline.step(&input).unwrap().emergency);
        }
        for i in if failed { 2 } else { 1 }..=8 {
            let mut input = frame(i as f64 * 0.05);
            if i % 2 == 0 {
                input.lidar3d.as_mut().unwrap().stamp = 0.0;
            } else {
                input.lidar3d = None;
            }
            let output = pipeline.step(&input).unwrap();
            if failed {
                assert!(output.health.contains(&HealthIssue::InvalidLidar));
            }
            if i == 8 {
                assert!(output.health.contains(&HealthIssue::StaleLidar));
            }
        }
        assert!(pipeline.step(&frame(0.45)).unwrap().health.is_empty());
    }
}
#[test]
fn acquisition_before_history_and_expired_new_stamp_are_rejected() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let mut input = frame(1.0);
    input.lidar3d.as_mut().unwrap().stamp = 0.9;
    assert!(
        pipeline
            .step(&input)
            .unwrap()
            .health
            .contains(&HealthIssue::InvalidLidar)
    );
    assert!(pipeline.step(&frame(1.05)).unwrap().health.is_empty());
    for i in 22..=28 {
        let mut input = frame(i as f64 * 0.05);
        input.lidar3d = None;
        pipeline.step(&input).unwrap();
    }
    let mut input = frame(1.45);
    input.lidar3d.as_mut().unwrap().stamp = 1.06;
    let output = pipeline.step(&input).unwrap();
    assert!(output.health.contains(&HealthIssue::InvalidLidar));
    assert!(output.health.contains(&HealthIssue::StaleLidar));
}
#[test]
fn tilted_scan_uses_acquisition_pose_and_replay_detects_actual_point_corruption() {
    let cfg = config();
    let mut pipeline = DrivingPipeline::new(cfg.clone()).unwrap();
    let mut log = SensorLog::new("tilted-delayed-turn", cfg);
    for i in 0..=6 {
        let mut input = frame(i as f64 * 0.05);
        input.odometry.as_mut().unwrap().speed = 4.0;
        input.odometry.as_mut().unwrap().yaw_rate = 0.8;
        if i > 0 {
            input.gnss = None;
        }
        input.lidar3d = None;
        if i >= 2 && i % 2 == 0 {
            let acquired = i - 2;
            let yaw = acquired as f64 * 0.04;
            let mut position = Vec2::default();
            for j in 0..acquired {
                let yaw = j as f64 * 0.04;
                position = position.plus(Vec2::new(yaw.cos(), yaw.sin()).scaled(0.2));
            }
            let returns = (340..380)
                .filter_map(|column| {
                    let unit = measured(column, 9, 1.0).point;
                    let world_direction = Vec2::new(unit.x, unit.y).rotated(yaw);
                    let range = (20.0 - position.x) / world_direction.x;
                    let hit_y = position.y + range * world_direction.y;
                    (hit_y.abs() <= 3.0).then(|| measured(column, 9, range))
                })
                .collect();
            input.lidar3d = Some(Lidar3dScan {
                stamp: acquired as f64 * 0.05,
                returns,
            });
        }
        let output = pipeline.step(&input).unwrap();
        if i >= 2 {
            assert!(output.health.is_empty());
            assert_eq!(output.tracks.len(), 1);
            assert!((output.tracks[0].position.x - 20.0).abs() < 1e-8);
            assert!(output.tracks[0].velocity.x.abs() < 1e-8);
            assert!(output.tracks[0].last_seen <= input.time - 0.1 + 1e-9);
        }
        log.record(input, output);
    }
    assert!(
        pipeline
            .occupied_cells()
            .iter()
            .all(|p| (p.x - 20.0).abs() <= 0.25 + 1e-9)
    );
    let mut bytes = vec![];
    log.write(&mut bytes).unwrap();
    assert!(
        verify(std::io::Cursor::new(&bytes), std::io::sink())
            .unwrap()
            .verified
    );
    let mut records: Vec<serde_json::Value> = String::from_utf8(bytes)
        .unwrap()
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect();
    records[3]["tick"]["input"]["lidar3d"]["returns"][0]["point"]["z"] = serde_json::json!(0.6);
    let edited = records
        .iter()
        .map(|r| serde_json::to_string(r).unwrap() + "\n")
        .collect::<String>();
    assert!(verify(std::io::Cursor::new(edited), std::io::sink()).is_err());
}
#[test]
fn optional_3d_fields_are_absent_in_previous_frame_and_header_modes() {
    let mut cfg = config();
    cfg.lidar3d = None;
    let mut input = frame(0.0);
    input.lidar3d = None;
    input.lidar = Some(LidarScan {
        stamp: 0.0,
        points: vec![],
    });
    assert!(!serde_json::to_string(&cfg).unwrap().contains("lidar3d"));
    assert!(!serde_json::to_string(&input).unwrap().contains("lidar3d"));
    let cfg: PipelineConfig = serde_json::from_value(serde_json::to_value(cfg).unwrap()).unwrap();
    let input: SensorFrame = serde_json::from_value(serde_json::to_value(input).unwrap()).unwrap();
    assert!(
        DrivingPipeline::new(cfg)
            .unwrap()
            .step(&input)
            .unwrap()
            .health
            .is_empty()
    );
}
