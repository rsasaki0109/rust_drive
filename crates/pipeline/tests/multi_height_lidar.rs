use rustdrive_core::*;
use rustdrive_pipeline::replay::{SensorLog, verify};
use rustdrive_pipeline::{
    DrivingPipeline, HealthIssue, MultiHeightLidarConfig, PipelineConfig, SensorFrame,
};

fn config() -> PipelineConfig {
    let mut config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(100.0, 0.0)], 2.1).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    config.multi_height_lidar = Some(MultiHeightLidarConfig {
        heights_m: vec![0.6, 0.15, 3.7],
        collision_bottom_m: -1.15,
        collision_top_m: 2.35,
    });
    config
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
        multi_height_lidar: Some(MultiHeightLidarScan {
            stamp: time,
            planes: vec![
                LidarPlane {
                    height_m: 0.6,
                    points: vec![],
                },
                LidarPlane {
                    height_m: 0.15,
                    points: vec![],
                },
                LidarPlane {
                    height_m: 3.7,
                    points: vec![],
                },
            ],
        }),
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}
fn barrier() -> Vec<Vec2> {
    (-10..=10).map(|i| Vec2::new(6.0, i as f64 * 0.2)).collect()
}
#[test]
fn low_returns_alone_change_planning_and_map_but_overhead_returns_do_not() {
    let mut low = DrivingPipeline::new(config()).unwrap();
    let mut input = frame(0.0);
    input.multi_height_lidar.as_mut().unwrap().planes[1].points = barrier();
    let stopped = low.step(&input).unwrap();
    assert!(stopped.health.is_empty());
    assert_eq!(stopped.tracks.len(), 1);
    assert_eq!(stopped.predictions.len(), 1);
    assert!(
        stopped
            .trajectory
            .points
            .iter()
            .map(|p| p.position.x)
            .fold(0.0, f64::max)
            < 6.0
    );
    assert!(!low.occupied_cells().is_empty());
    let mut overhead = DrivingPipeline::new(config()).unwrap();
    let mut input = frame(0.0);
    input.multi_height_lidar.as_mut().unwrap().planes[2].points = barrier();
    let clear = overhead.step(&input).unwrap();
    assert!(clear.health.is_empty());
    assert!(clear.tracks.is_empty());
    assert!(clear.trajectory.points.iter().any(|p| p.position.x > 20.0));
    assert!(overhead.occupied_cells().is_empty());
}
#[test]
fn repeated_plane_returns_do_not_change_detections_and_plane_order_is_immaterial() {
    let mut input = frame(0.0);
    input.multi_height_lidar.as_mut().unwrap().planes[1].points = barrier();
    let one = DrivingPipeline::new(config())
        .unwrap()
        .step(&input)
        .unwrap();
    input.multi_height_lidar.as_mut().unwrap().planes[0].points = barrier();
    input.multi_height_lidar.as_mut().unwrap().planes.reverse();
    let two = DrivingPipeline::new(config())
        .unwrap()
        .step(&input)
        .unwrap();
    assert_eq!(
        serde_json::to_value(one).unwrap(),
        serde_json::to_value(two).unwrap()
    );
}
#[test]
fn malformed_missing_duplicate_nonfinite_and_oversized_planes_brake_without_updates() {
    for variant in 0..8 {
        let mut pipeline = DrivingPipeline::new(config()).unwrap();
        pipeline.step(&frame(0.0)).unwrap();
        let mut input = frame(0.05);
        let scan = input.multi_height_lidar.as_mut().unwrap();
        match variant {
            0 => {
                scan.planes.pop();
            }
            1 => scan.planes[0].height_m = 0.15,
            2 => scan.planes[2].height_m = 4.0,
            3 => scan.planes[2].height_m = f64::NAN,
            4 => scan.planes[2].points.push(Vec2::new(f64::NAN, 0.0)),
            5 => scan.planes[2].points.push(Vec2::new(201.0, 0.0)),
            6 => scan.planes[2].points = vec![Vec2::default(); 20_001],
            _ => {
                scan.stamp = 0.1;
            }
        }
        let out = pipeline.step(&input).unwrap();
        assert!(
            out.health.contains(&HealthIssue::InvalidLidar),
            "variant {variant}"
        );
        assert_eq!(out.command.acceleration, -6.0);
        assert!(out.tracks.is_empty());
        assert!(pipeline.occupied_cells().is_empty());
        assert!(pipeline.step(&frame(0.1)).unwrap().health.is_empty());
    }
}
#[test]
fn acquisition_failure_and_ambiguous_or_out_of_mode_inputs_brake() {
    for variant in 0..4 {
        let mut cfg = config();
        let mut input = frame(0.0);
        match variant {
            0 => input.lidar_failed = true,
            1 => {
                input.lidar = Some(LidarScan {
                    stamp: 0.0,
                    points: vec![],
                })
            }
            2 => {
                input.multi_height_lidar = None;
                input.lidar = Some(LidarScan {
                    stamp: 0.0,
                    points: vec![],
                });
            }
            _ => cfg.multi_height_lidar = None,
        }
        let out = DrivingPipeline::new(cfg).unwrap().step(&input).unwrap();
        assert!(out.emergency);
        assert_eq!(out.command.acceleration, -6.0);
        assert!(out.health.contains(&if variant == 0 {
            HealthIssue::AcquisitionFailed
        } else {
            HealthIssue::InvalidLidar
        }));
    }
}
#[test]
fn missing_new_scan_and_duplicate_stamps_cannot_extend_health() {
    for duplicate in [false, true] {
        let mut pipeline = DrivingPipeline::new(config()).unwrap();
        pipeline.step(&frame(0.0)).unwrap();
        for i in 1..=8 {
            let mut input = frame(i as f64 * 0.05);
            if duplicate {
                input.multi_height_lidar.as_mut().unwrap().stamp = 0.0;
            } else {
                input.multi_height_lidar = None;
            }
            let out = pipeline.step(&input).unwrap();
            if i <= 7 {
                assert!(out.health.is_empty());
            } else {
                assert!(out.health.contains(&HealthIssue::StaleLidar));
                assert!(out.emergency);
            }
        }
    }
}
#[test]
fn invalid_calibration_is_rejected_before_pipeline_creation() {
    for variant in 0..9 {
        let mut cfg = config();
        let calibration = cfg.multi_height_lidar.as_mut().unwrap();
        match variant {
            0 => calibration.heights_m.clear(),
            1 => calibration.heights_m = vec![0.6],
            2 => calibration.heights_m = (0..17).map(|i| i as f64 * 0.1).collect(),
            3 => calibration.heights_m[0] = 0.15,
            4 => calibration.heights_m[0] = f64::INFINITY,
            5 => calibration.collision_bottom_m = 2.35,
            6 => calibration.collision_top_m = 20.0,
            7 => calibration.heights_m = vec![4.0, 5.0],
            _ => calibration.heights_m[0] = 0.151,
        }
        assert!(DrivingPipeline::new(cfg).is_err(), "variant {variant}");
    }
}
#[test]
fn delayed_bundle_uses_acquisition_pose_for_stationary_world_tracks_and_replays() {
    let cfg = config();
    let mut pipeline = DrivingPipeline::new(cfg.clone()).unwrap();
    let mut log = SensorLog::new("multi-height-delayed-turn", cfg);
    let center = Vec2::new(20.0, 3.0);
    for i in 0..=6 {
        let mut input = frame(i as f64 * 0.05);
        input.odometry.as_mut().unwrap().speed = 4.0;
        input.odometry.as_mut().unwrap().yaw_rate = 0.8;
        if i > 0 {
            input.gnss = None;
        }
        input.multi_height_lidar = None;
        if i >= 2 && i % 2 == 0 {
            let acquisition_tick = i - 2;
            let mut position = Vec2::default();
            for j in 0..acquisition_tick {
                let yaw = j as f64 * 0.04;
                position = position.plus(Vec2::new(yaw.cos(), yaw.sin()).scaled(0.2));
            }
            let yaw = acquisition_tick as f64 * 0.04;
            let mut scan = frame(acquisition_tick as f64 * 0.05)
                .multi_height_lidar
                .unwrap();
            scan.planes[1].points = (0..32)
                .map(|j| {
                    let angle = j as f64 * std::f64::consts::TAU / 32.0;
                    center
                        .plus(Vec2::new(angle.cos(), angle.sin()))
                        .minus(position)
                        .rotated(-yaw)
                })
                .collect();
            input.multi_height_lidar = Some(scan);
        }
        let output = pipeline.step(&input).unwrap();
        if i >= 2 {
            assert!(output.health.is_empty());
            assert_eq!(output.tracks.len(), 1);
            assert!(output.tracks[0].position.distance(center) < 1e-8);
            assert!(
                output.tracks[0]
                    .velocity
                    .x
                    .hypot(output.tracks[0].velocity.y)
                    < 1e-8
            );
            assert!(output.tracks[0].last_seen <= input.time - 0.1 + 1e-9);
        }
        log.record(input, output);
    }
    assert!(
        pipeline
            .occupied_cells()
            .iter()
            .any(|p| p.distance(center) < 2.0)
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
    records[3]["tick"]["input"]["multi_height_lidar"]["planes"][1]["points"] =
        serde_json::json!([]);
    let changed = records
        .iter()
        .map(|r| serde_json::to_string(r).unwrap() + "\n")
        .collect::<String>();
    assert!(verify(std::io::Cursor::new(changed), std::io::sink()).is_err());
}
#[test]
fn expired_new_scan_or_acquisition_before_history_does_not_refresh() {
    for before_history in [false, true] {
        let mut pipeline = DrivingPipeline::new(config()).unwrap();
        if before_history {
            let mut input = frame(1.0);
            input.multi_height_lidar.as_mut().unwrap().stamp = 0.9;
            let out = pipeline.step(&input).unwrap();
            assert!(out.health.contains(&HealthIssue::InvalidLidar));
            assert!(out.health.contains(&HealthIssue::StaleLidar));
        } else {
            pipeline.step(&frame(0.0)).unwrap();
            for i in 1..=8 {
                let mut input = frame(i as f64 * 0.05);
                input.multi_height_lidar = None;
                if i == 8 {
                    input.multi_height_lidar = frame(0.01).multi_height_lidar;
                }
                let out = pipeline.step(&input).unwrap();
                if i == 8 {
                    assert!(out.health.contains(&HealthIssue::InvalidLidar));
                    assert!(out.health.contains(&HealthIssue::StaleLidar));
                }
            }
        }
    }
}
#[test]
fn optional_contract_does_not_change_ordinary_serialized_frames_or_configs() {
    let mut cfg = config();
    cfg.multi_height_lidar = None;
    let mut input = frame(0.0);
    input.multi_height_lidar = None;
    input.lidar = Some(LidarScan {
        stamp: 0.0,
        points: vec![],
    });
    let old_cfg = serde_json::to_string(&cfg).unwrap();
    let old_frame = serde_json::to_string(&input).unwrap();
    assert!(!old_cfg.contains("multi_height_lidar"));
    assert!(!old_frame.contains("multi_height_lidar"));
    assert!(
        DrivingPipeline::new(serde_json::from_str(&old_cfg).unwrap())
            .unwrap()
            .step(&serde_json::from_str(&old_frame).unwrap())
            .unwrap()
            .health
            .is_empty()
    );
}

#[test]
fn malformed_or_failed_bundle_holds_through_absence_duplicates_and_prefault_delays() {
    for failed in [false, true] {
        let mut pipeline = DrivingPipeline::new(config()).unwrap();
        assert!(pipeline.step(&frame(0.0)).unwrap().health.is_empty());
        let mut malformed = frame(0.05);
        if failed {
            malformed.lidar_failed = true;
        } else {
            malformed.multi_height_lidar.as_mut().unwrap().planes.pop();
        }
        assert!(pipeline.step(&malformed).unwrap().emergency);
        for (time, stamp) in [(0.1, None), (0.15, Some(0.0)), (0.2, Some(0.025))] {
            let mut input = frame(time);
            input.multi_height_lidar = stamp.map(|stamp| {
                let mut scan = frame(stamp).multi_height_lidar.unwrap();
                scan.planes[1].points = barrier();
                scan
            });
            let out = pipeline.step(&input).unwrap();
            assert!(out.health.contains(&HealthIssue::InvalidLidar));
            assert_eq!(out.command.acceleration, -6.0);
            assert!(out.tracks.is_empty());
            assert!(pipeline.occupied_cells().is_empty());
        }
        let mut fresh = frame(0.25);
        fresh.multi_height_lidar.as_mut().unwrap().planes[1].points = barrier();
        let resumed = pipeline.step(&fresh).unwrap();
        assert!(resumed.health.is_empty());
        assert_eq!(resumed.tracks.len(), 1);
        assert!(!pipeline.occupied_cells().is_empty());
    }
}

#[test]
fn accepted_height_label_tolerance_cannot_remove_a_boundary_plane_from_fusion() {
    for (bottom, top) in [(0.15, 2.35), (-1.15, 0.15)] {
        let mut cfg = config();
        cfg.multi_height_lidar.as_mut().unwrap().collision_bottom_m = bottom;
        cfg.multi_height_lidar.as_mut().unwrap().collision_top_m = top;
        let mut input = frame(0.0);
        let plane = &mut input.multi_height_lidar.as_mut().unwrap().planes[1];
        plane.height_m += if bottom == 0.15 { -5e-10 } else { 5e-10 };
        plane.points = barrier();
        let out = DrivingPipeline::new(cfg).unwrap().step(&input).unwrap();
        assert!(out.health.is_empty());
        assert_eq!(out.tracks.len(), 1);
    }
}
