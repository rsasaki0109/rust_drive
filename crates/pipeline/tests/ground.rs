use rustdriving_core::*;
use rustdriving_pipeline::replay::{SensorLog, verify};
use rustdriving_pipeline::{
    DrivingPipeline, GroundConfig, HealthIssue, Lidar3dConfig, PipelineConfig, SensorFrame,
};
use std::f64::consts::{PI, TAU};

fn ground() -> GroundConfig {
    GroundConfig {
        reference_height_m: 0.0,
        max_slope: 0.05,
        max_height_offset_m: 0.03,
        residual_threshold_m: 0.02,
        fit_radius_m: 8.0,
        min_inliers: 200,
        min_sector_inliers: 20,
        min_cell_inliers: 6,
    }
}
fn config() -> PipelineConfig {
    let mut config = PipelineConfig::new(
        Route::new(vec![Vec2::default(), Vec2::new(80.0, 0.0)], 2.1).unwrap(),
        Pose::default(),
        VehicleConfig::default(),
    );
    config.lidar3d = Some(Lidar3dConfig {
        azimuth_columns: 180,
        elevation_rings: 16,
        min_elevation_rad: -PI / 12.0,
        max_elevation_rad: PI / 12.0,
        mount_height_m: 0.6,
        min_range_m: 0.2,
        max_range_m: 45.0,
        collision_bottom_m: -1.15,
        collision_top_m: 2.35,
        ground: Some(ground()),
    });
    config
}
fn direction(column: usize, ring: usize) -> Vec3 {
    let a = -PI + TAU * column as f64 / 179.0;
    let e = -PI / 12.0 + PI / 6.0 * ring as f64 / 15.0;
    Vec3::new(e.cos() * a.cos(), -e.cos() * a.sin(), e.sin())
}
fn box_intersection(d: Vec3) -> Option<f64> {
    let mut entry = 0.0_f64;
    let mut exit = f64::INFINITY;
    for (start, direction, lo, hi) in [
        (0.0, d.x, 6.0, 6.5),
        (0.0, d.y, -2.0, 2.0),
        (0.6, d.z, 0.0, 0.1),
    ] {
        if direction.abs() < 1e-12 {
            if start < lo || start > hi {
                return None;
            }
        } else {
            let a = (lo - start) / direction;
            let b = (hi - start) / direction;
            entry = entry.max(a.min(b));
            exit = exit.min(a.max(b));
        }
    }
    (entry <= exit).then_some(entry)
}
fn cloud(time: f64, a: f64, b: f64, c: f64, curb: bool) -> Lidar3dScan {
    let returns = (0..180)
        .flat_map(|column| {
            (0..16).filter_map(move |ring| {
                let d = direction(column, ring);
                let plane_range = (c - 0.6) / (d.z - a * d.x - b * d.y);
                let mut range = if (0.2..=45.0).contains(&plane_range) {
                    plane_range
                } else {
                    f64::INFINITY
                };
                if curb && let Some(distance) = box_intersection(d) {
                    range = range.min(distance);
                }
                if !range.is_finite() {
                    return None;
                }
                Some(Lidar3dReturn {
                    ray_index: column * 16 + ring,
                    point: Vec3::new(range * d.x, range * d.y, 0.6 + range * d.z),
                })
            })
        })
        .collect();
    Lidar3dScan {
        stamp: time,
        returns,
    }
}
fn frame(time: f64, scan: Lidar3dScan) -> SensorFrame {
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
        lidar3d: Some(scan),
        lidar_failed: false,
        navigation_update: None,
        traffic_signal: None,
    }
}
#[test]
fn actual_flat_ground_is_removed_from_mapping_and_driving_without_a_global_height_cut() {
    let input = frame(0.0, cloud(0.0, 0.0, 0.0, 0.0, false));
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let output = pipeline.step(&input).unwrap();
    let diagnostic = output.ground.unwrap();
    assert!(diagnostic.confidence);
    assert_eq!(diagnostic.inliers, 1080);
    assert!(diagnostic.sector_inliers.iter().all(|n| *n >= 20));
    assert_eq!(
        diagnostic.removed_points,
        input.lidar3d.as_ref().unwrap().returns.len()
    );
    assert_eq!(diagnostic.preserved_points, 0);
    assert!(output.health.is_empty());
    assert!(output.tracks.is_empty());
    assert!(pipeline.occupied_cells().is_empty());
    assert!(output.trajectory.points.iter().any(|p| p.position.x > 20.0));
    let mut config = config();
    config.lidar3d.as_mut().unwrap().ground = None;
    let mut unsegmented = DrivingPipeline::new(config).unwrap();
    let out = unsegmented.step(&input).unwrap();
    assert!(!out.tracks.is_empty());
    assert!(!unsegmented.occupied_cells().is_empty());
    assert!(out.ground.is_none());
}
#[test]
fn measured_small_grade_recovers_plane_and_rejects_a_low_curb_as_ground() {
    let input = frame(0.0, cloud(0.0, 0.015, 0.02, 0.005, false));
    let output = DrivingPipeline::new(config())
        .unwrap()
        .step(&input)
        .unwrap();
    let diagnostic = output.ground.unwrap();
    let plane = diagnostic.plane.unwrap();
    assert!(diagnostic.confidence);
    assert!((plane.a - 0.015).abs() < 1e-10);
    assert!((plane.b - 0.02).abs() < 1e-10);
    assert!((plane.c - 0.005).abs() < 1e-10);
    assert!(diagnostic.removed_points as f64 / input.lidar3d.unwrap().returns.len() as f64 > 0.95);
    let input = frame(0.0, cloud(0.0, 0.0, 0.0, 0.0, true));
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let out = pipeline.step(&input).unwrap();
    let diagnostic = out.ground.unwrap();
    assert!(diagnostic.confidence);
    assert!(diagnostic.preserved_points >= 3);
    assert!(
        input
            .lidar3d
            .unwrap()
            .returns
            .iter()
            .filter(|r| r.point.z > 0.03)
            .count()
            >= diagnostic.preserved_points
    );
    assert!(out.health.is_empty());
    assert_eq!(out.tracks.len(), 1);
    assert!(out.trajectory.points.iter().all(|p| p.position.x < 6.0));
}
#[test]
fn broad_elevated_plane_or_incomplete_spatial_support_cannot_be_assumed_ground() {
    for elevated in [false, true] {
        let mut scan = cloud(0.0, 0.0, 0.0, if elevated { 0.1 } else { 0.0 }, false);
        if !elevated {
            scan.returns
                .retain(|r| (45..135).contains(&(r.ray_index / 16)));
        }
        let output = DrivingPipeline::new(config())
            .unwrap()
            .step(&frame(0.0, scan))
            .unwrap();
        assert!(output.health.contains(&HealthIssue::InvalidLidar));
        assert_eq!(output.command.acceleration, -6.0);
        let diagnostics = output.ground.unwrap();
        assert!(!diagnostics.confidence);
        assert_eq!(diagnostics.removed_points, 0);
        assert!(output.tracks.is_empty());
    }
}
#[test]
fn an_isolated_far_plane_point_is_preserved_without_local_measured_support() {
    let mut scan = cloud(0.0, 0.0, 0.0, 0.0, false);
    scan.returns
        .retain(|r| r.point.x.hypot(r.point.y) <= 8.0 || r.ray_index == 90 * 16 + 7);
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    let out = pipeline.step(&frame(0.0, scan)).unwrap();
    let diagnostics = out.ground.unwrap();
    assert!(diagnostics.confidence);
    assert_eq!(diagnostics.preserved_points, 1);
    let mut scan = cloud(0.05, 0.0, 0.0, 0.0, false);
    scan.returns
        .retain(|r| r.point.x.hypot(r.point.y) <= 8.0 || r.ray_index == 90 * 16 + 7);
    pipeline.step(&frame(0.05, scan)).unwrap();
    assert_eq!(pipeline.occupied_cells().len(), 1);
}
#[test]
fn fit_failure_latches_through_missing_or_duplicate_scans_until_a_new_measured_plane() {
    let mut pipeline = DrivingPipeline::new(config()).unwrap();
    pipeline
        .step(&frame(0.0, cloud(0.0, 0.0, 0.0, 0.0, false)))
        .unwrap();
    let bad = frame(
        0.05,
        Lidar3dScan {
            stamp: 0.05,
            returns: vec![],
        },
    );
    let out = pipeline.step(&bad).unwrap();
    assert_eq!(out.ground.unwrap().candidate_points, 0);
    assert!(out.emergency);
    for (time, stamp) in [(0.1, None), (0.15, Some(0.0)), (0.2, Some(0.025))] {
        let mut input = frame(time, cloud(stamp.unwrap_or(time), 0.0, 0.0, 0.0, false));
        if stamp.is_none() {
            input.lidar3d = None;
        }
        assert_eq!(pipeline.step(&input).unwrap().command.acceleration, -6.0);
    }
    assert!(
        pipeline
            .step(&frame(0.25, cloud(0.25, 0.0, 0.0, 0.0, false)))
            .unwrap()
            .health
            .is_empty()
    );
}
#[test]
fn every_xyz_return_is_validated_before_any_ground_is_removed() {
    let mut scan = cloud(0.0, 0.0, 0.0, 0.0, false);
    scan.returns[0].point.z = f64::NAN;
    let out = DrivingPipeline::new(config())
        .unwrap()
        .step(&frame(0.0, scan))
        .unwrap();
    assert!(out.ground.is_none());
    assert!(out.health.contains(&HealthIssue::InvalidLidar));
    assert!(out.emergency);
}
#[test]
fn invalid_ground_calibration_is_rejected_before_a_step() {
    for variant in 0..9 {
        let mut config = config();
        let ground = config.lidar3d.as_mut().unwrap().ground.as_mut().unwrap();
        match variant {
            0 => ground.residual_threshold_m = 0.031,
            1 => ground.max_height_offset_m = 0.1,
            2 => ground.max_slope = f64::NAN,
            3 => ground.min_inliers = 0,
            4 => ground.min_sector_inliers = 200,
            5 => ground.min_cell_inliers = 1,
            6 => ground.fit_radius_m = 0.1,
            7 => ground.reference_height_m = 0.6,
            _ => ground.fit_radius_m = f64::INFINITY,
        }
        assert!(DrivingPipeline::new(config).is_err(), "variant {variant}");
    }
}
#[test]
fn ground_diagnostics_and_raw_cloud_replay_deterministically_and_old_mode_omits_fields() {
    let config = config();
    let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
    let mut log = SensorLog::new("measured-ground", config);
    for i in 0..3 {
        let time = i as f64 * 0.05;
        let input = frame(time, cloud(time, 0.0, 0.0, 0.0, false));
        let output = pipeline.step(&input).unwrap();
        log.record(input, output);
    }
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
    records[1]["tick"]["expected"]["ground"]["plane"]["c"] = serde_json::json!(0.1);
    let edited = records
        .iter()
        .map(|r| serde_json::to_string(r).unwrap() + "\n")
        .collect::<String>();
    assert!(verify(std::io::Cursor::new(edited), std::io::sink()).is_err());
    let mut config = crate::config();
    config.lidar3d.as_mut().unwrap().ground = None;
    assert!(!serde_json::to_string(&config).unwrap().contains("ground"));
}
#[test]
fn plane_estimation_does_not_depend_on_return_order() {
    let mut scan = cloud(0.0, 0.015, 0.02, 0.005, false);
    let first = DrivingPipeline::new(config())
        .unwrap()
        .step(&frame(0.0, scan.clone()))
        .unwrap();
    scan.returns.reverse();
    let second = DrivingPipeline::new(config())
        .unwrap()
        .step(&frame(0.0, scan))
        .unwrap();
    assert_eq!(
        serde_json::to_value(first).unwrap(),
        serde_json::to_value(second).unwrap()
    );
}

#[test]
fn radial_measurement_noise_and_a_foreground_curb_do_not_bias_ground_into_obstacles() {
    let mut scan = cloud(0.0, 0.0, 0.0, 0.0, true);
    for measured in &mut scan.returns {
        let p = measured.point;
        let range = p.x.hypot(p.y).hypot(p.z - 0.6);
        let error = if measured.ray_index % 2 == 0 {
            0.008
        } else {
            -0.008
        };
        let ratio = (range + error) / range;
        measured.point = Vec3::new(p.x * ratio, p.y * ratio, 0.6 + (p.z - 0.6) * ratio);
    }
    let output = DrivingPipeline::new(config())
        .unwrap()
        .step(&frame(0.0, scan))
        .unwrap();
    let diagnostics = output.ground.unwrap();
    let plane = diagnostics.plane.unwrap();
    assert!(diagnostics.confidence);
    assert!(plane.a.abs() < 0.003 && plane.b.abs() < 0.003 && plane.c.abs() < 0.003);
    assert!(diagnostics.max_inlier_residual_m <= 0.02 && diagnostics.rms_residual_m < 0.003);
    assert!(diagnostics.preserved_points >= 3);
    assert_eq!(output.tracks.len(), 1);
}

#[test]
fn threshold_adjacent_native_returns_converge_to_the_reported_inliers_fit() {
    // Actual native kinematic ground-low-slab seed 7 acquisition at 8.8 s,
    // recorded before the convergence fix (8c1459). The former two-refinement
    // fit differed from least squares of its reported mask by 2.06e-5 m.
    // Snapshot SHA256: 57fd0a2b695f713a3eacaa02fc2189a5e5930d9f7c5c0a567b8c89e346758515.
    let scan: Lidar3dScan =
        serde_json::from_str(include_str!("fixtures/ground-threshold-scan.json")).unwrap();
    let out = DrivingPipeline::new(config())
        .unwrap()
        .step(&frame(scan.stamp, scan.clone()))
        .unwrap();
    let diagnostics = out.ground.unwrap();
    assert!(diagnostics.confidence);
    let plane = diagnostics.plane.unwrap();
    let points: Vec<_> = scan
        .returns
        .iter()
        .map(|r| r.point)
        .filter(|p| {
            let radius = p.x.hypot(p.y);
            (0.5..=8.0).contains(&radius)
                && p.z.abs() <= 0.03 + 0.05 * radius + 0.02
                && (p.z - plane.a * p.x - plane.b * p.y - plane.c).abs() <= 0.02
        })
        .collect();
    assert_eq!(points.len(), diagnostics.inliers);
    // Independent centered covariance solution rather than the driver's 3x3
    // normal-equation elimination or its candidate/refinement implementation.
    let n = points.len() as f64;
    let (mx, my, mz) = points.iter().fold((0.0, 0.0, 0.0), |(x, y, z), p| {
        (x + p.x / n, y + p.y / n, z + p.z / n)
    });
    let (xx, yy, xy, xz, yz) =
        points
            .iter()
            .fold((0.0, 0.0, 0.0, 0.0, 0.0), |(xx, yy, xy, xz, yz), p| {
                let (x, y, z) = (p.x - mx, p.y - my, p.z - mz);
                (xx + x * x, yy + y * y, xy + x * y, xz + x * z, yz + y * z)
            });
    let determinant = xx * yy - xy * xy;
    let a = (xz * yy - yz * xy) / determinant;
    let b = (yz * xx - xz * xy) / determinant;
    let c = mz - a * mx - b * my;
    assert!(
        (plane.a - a).abs() < 1e-10 && (plane.b - b).abs() < 1e-10 && (plane.c - c).abs() < 1e-10
    );
}
