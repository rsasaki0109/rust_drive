//! Optional terrain and measured XYZ object perception on validated raw returns.
//! Uses acquisition-body XYZ only; no point labels, scene roles or world truth.
use crate::Lidar3dConfig;
use rustdrive_core::{Detection, Lidar3dScan, LidarScan, Pose, Vec2, Vec3};
use rustdrive_perception::{ObjectClusterConfig, TerrainConfig, classify_ground, cluster_objects};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Perception3dConfig {
    #[serde(with = "TerrainConfigSerde")]
    pub terrain: TerrainConfig,
    #[serde(with = "ObjectConfigSerde")]
    pub objects: ObjectClusterConfig,
}
impl Default for Perception3dConfig {
    fn default() -> Self {
        Self {
            terrain: TerrainConfig {
                max_points: 20_000,
                max_cells: 20_000,
                max_candidate_work: 8_000_000,
                ..TerrainConfig::default()
            },
            objects: ObjectClusterConfig {
                max_points: 20_000,
                max_cluster_points: 20_000,
                max_clusters: 1_000,
                max_candidate_work: 1_000_000,
                ..ObjectClusterConfig::default()
            },
        }
    }
}
impl Perception3dConfig {
    pub(crate) fn validate(&self, calibration: &Lidar3dConfig) -> Result<(), String> {
        calibration.validate()?;
        if calibration.ground.is_some() {
            return Err("3D terrain perception excludes the separate near-flat ground mode".into());
        }
        self.terrain.validate()?;
        self.objects.validate()?;
        if self.terrain.max_points > 20_000
            || self.terrain.max_cells > 20_000
            || self.objects.max_points > 20_000
        {
            return Err(
                "pipeline XYZ perception resource counts must not exceed the 20000-return contract"
                    .into(),
            );
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize)]
#[serde(remote = "TerrainConfig", deny_unknown_fields)]
struct TerrainConfigSerde {
    cell_size_m: f64,
    initial_height_m: f64,
    max_height_m: f64,
    max_slope: f64,
    window_radii_cells: Vec<usize>,
    min_support_neighbors: usize,
    min_supported_cells: usize,
    min_supported_fraction: f64,
    max_points: usize,
    max_cells: usize,
    max_candidate_work: usize,
    coordinate_bound_m: f64,
}

#[derive(Serialize, Deserialize)]
#[serde(remote = "ObjectClusterConfig", deny_unknown_fields)]
struct ObjectConfigSerde {
    tolerance_m: f64,
    voxel_size_m: f64,
    min_points: usize,
    max_points: usize,
    max_cluster_points: usize,
    max_clusters: usize,
    max_candidate_work: usize,
    coordinate_bound_m: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TerrainSupportDiagnostics {
    pub occupied_cells: usize,
    pub candidate_ground_cells: usize,
    pub supported_cells: usize,
    pub rejected_cells: usize,
    pub candidate_work: usize,
    pub supported_fraction: f64,
    pub confident: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeasuredObject3d {
    /// Acquisition-body XYZ; an AABB of observed points, not the hidden object.
    pub center: Vec3,
    pub min: Vec3,
    pub max: Vec3,
    pub point_count: usize,
    pub return_indices: Vec<usize>,
    pub collision_relevant: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Perception3dDiagnostics {
    pub stamp: f64,
    pub measured_points: usize,
    pub ground_points: usize,
    pub non_ground_points: usize,
    pub ground_return_indices: Vec<usize>,
    pub terrain: Option<TerrainSupportDiagnostics>,
    pub objects: Vec<MeasuredObject3d>,
    pub noise_points: usize,
    pub object_candidate_work: usize,
    pub projected_points: usize,
    /// Measured local support only, not semantic certainty or a probability.
    pub confident: bool,
    pub reason: Option<String>,
}
pub(crate) struct Perception3dAcquisition {
    pub(crate) scan: LidarScan,
    pub(crate) diagnostics: Perception3dDiagnostics,
}
impl Perception3dAcquisition {
    pub(crate) fn detections_at(&self, pose: Pose) -> Vec<Detection> {
        self.diagnostics
            .objects
            .iter()
            .filter(|object| object.collision_relevant)
            .map(|object| Detection {
                center: pose.to_world(Vec2::new(object.center.x, object.center.y)),
                radius: (0.5 * (object.max.x - object.min.x))
                    .hypot(0.5 * (object.max.y - object.min.y)),
            })
            .collect()
    }
}
fn failure(
    diagnostics: &Perception3dDiagnostics,
    reason: impl Into<String>,
) -> Box<Perception3dDiagnostics> {
    let mut failure = diagnostics.clone();
    failure.confident = false;
    failure.reason = Some(reason.into());
    Box::new(failure)
}
/// Validate ordinal/range/beam geometry before classifying any genuine XYZ.
/// Unsupported terrain produces a diagnostic error; callers must retain sensor
/// fault braking and may not clear unknown terrain or silently use planar fallback.
pub(crate) fn process(
    calibration: &Lidar3dConfig,
    config: &Perception3dConfig,
    scan: &Lidar3dScan,
) -> Result<Perception3dAcquisition, Box<Perception3dDiagnostics>> {
    let mut diagnostics = Perception3dDiagnostics {
        stamp: scan.stamp,
        measured_points: scan.returns.len(),
        ground_points: 0,
        non_ground_points: 0,
        ground_return_indices: vec![],
        terrain: None,
        objects: vec![],
        noise_points: 0,
        object_candidate_work: 0,
        projected_points: 0,
        confident: false,
        reason: None,
    };
    config
        .validate(calibration)
        .map_err(|error| failure(&diagnostics, error))?;
    if !scan.stamp.is_finite() || scan.stamp < 0.0 || calibration.validate_returns(scan).is_err() {
        return Err(failure(
            &diagnostics,
            "invalid measured XYZ timestamp/ordinal/range/direction",
        ));
    }
    let points: Vec<_> = scan.returns.iter().map(|measured| measured.point).collect();
    let ground =
        classify_ground(&points, &config.terrain).map_err(|error| failure(&diagnostics, error))?;
    let support = &ground.diagnostics;
    diagnostics.terrain = Some(TerrainSupportDiagnostics {
        occupied_cells: support.occupied_cells,
        candidate_ground_cells: support.candidate_ground_cells,
        supported_cells: support.supported_cells,
        rejected_cells: support.rejected_cells,
        candidate_work: support.candidate_work,
        supported_fraction: support.supported_fraction,
        confident: support.confident,
    });
    diagnostics.ground_points = ground.ground_indices.len();
    diagnostics.non_ground_points = ground.non_ground_indices.len();
    diagnostics.ground_return_indices = ground.ground_indices;
    if !support.confident {
        return Err(failure(
            &diagnostics,
            "insufficient measured terrain support",
        ));
    }
    let objects = cluster_objects(&points, &ground.non_ground_indices, &config.objects)
        .map_err(|error| failure(&diagnostics, error))?;
    diagnostics.noise_points = objects.noise_indices.len();
    diagnostics.object_candidate_work = objects.candidate_work;
    let mut projected = vec![];
    let mut measured_objects = vec![];
    for object in objects.objects {
        let relevant = object.max.z >= calibration.collision_bottom_m
            && object.min.z <= calibration.collision_top_m;
        if relevant {
            for &index in &object.indices {
                let point = points[index];
                if point.z >= calibration.collision_bottom_m
                    && point.z <= calibration.collision_top_m
                {
                    projected.push((scan.returns[index].ray_index, Vec2::new(point.x, point.y)));
                }
            }
        }
        measured_objects.push(MeasuredObject3d {
            center: object.center,
            min: object.min,
            max: object.max,
            point_count: object.point_count,
            return_indices: object.indices,
            collision_relevant: relevant,
        });
    }
    projected.sort_by_key(|(ordinal, _)| *ordinal);
    diagnostics.projected_points = projected.len();
    diagnostics.objects = measured_objects;
    diagnostics.confident = true;
    Ok(Perception3dAcquisition {
        scan: LidarScan {
            stamp: scan.stamp,
            points: projected.into_iter().map(|(_, point)| point).collect(),
        },
        diagnostics,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdrive_core::Lidar3dReturn;
    use std::f64::consts::{PI, TAU};
    fn measured_patch() -> (Lidar3dConfig, Lidar3dScan) {
        let cfg = Lidar3dConfig {
            azimuth_columns: 256,
            elevation_rings: 32,
            min_elevation_rad: -0.55,
            max_elevation_rad: 0.5,
            mount_height_m: 2.,
            min_range_m: 0.1,
            max_range_m: 30.,
            collision_bottom_m: 0.1,
            collision_top_m: 1.5,
            ground: None,
        };
        let mut returns = vec![];
        for column in 0..cfg.azimuth_columns {
            for ring in 0..cfg.elevation_rings {
                let a = -PI + TAU * column as f64 / (cfg.azimuth_columns - 1) as f64;
                let e = cfg.min_elevation_rad
                    + (cfg.max_elevation_rad - cfg.min_elevation_rad) * ring as f64
                        / (cfg.elevation_rings - 1) as f64;
                let direction = Vec3::new(e.cos() * a.cos(), -e.cos() * a.sin(), e.sin());
                // Synthetic acquisition fixture: inclined ground and two walls.
                // Only the resulting genuine XYZ/ray ordinals enter the algorithm.
                let denom = direction.z - 0.04 * direction.x;
                let mut closest = if denom < 0. {
                    -cfg.mount_height_m / denom
                } else {
                    f64::INFINITY
                };
                for (x, bottom, top) in [(8., 0.4, 2.), (-8., 4., 6.)] {
                    let range = x / direction.x;
                    let y = range * direction.y;
                    let z = cfg.mount_height_m + range * direction.z;
                    if range > 0. && y.abs() <= 3. && z >= bottom && z <= top {
                        closest = closest.min(range);
                    }
                }
                if (cfg.min_range_m..=cfg.max_range_m).contains(&closest) {
                    returns.push(Lidar3dReturn {
                        ray_index: column * cfg.elevation_rings + ring,
                        point: Vec3::new(
                            closest * direction.x,
                            closest * direction.y,
                            cfg.mount_height_m + closest * direction.z,
                        ),
                    });
                }
            }
        }
        (cfg, Lidar3dScan { stamp: 0., returns })
    }
    #[test]
    fn raw_sloped_xyz_yields_measured_aabbs_and_height_filtered_conservative_circles() {
        let (cfg, scan) = measured_patch();
        let acquisition = process(&cfg, &Perception3dConfig::default(), &scan).unwrap();
        assert!(acquisition.diagnostics.confident);
        assert!(acquisition.diagnostics.ground_points > 1_000);
        assert!(
            acquisition
                .diagnostics
                .objects
                .iter()
                .any(|object| object.collision_relevant)
        );
        assert!(
            acquisition
                .diagnostics
                .objects
                .iter()
                .any(|object| !object.collision_relevant && object.min.z > 1.5)
        );
        let detections = acquisition.detections_at(Pose::default());
        assert!(!detections.is_empty());
        for (object, detection) in acquisition
            .diagnostics
            .objects
            .iter()
            .filter(|object| object.collision_relevant)
            .zip(detections)
        {
            for &index in &object.return_indices {
                let point = scan.returns[index].point;
                assert!(
                    Vec2::new(point.x, point.y).distance(detection.center)
                        <= detection.radius + 1e-8
                );
                assert!(
                    point.x >= object.min.x
                        && point.x <= object.max.x
                        && point.z >= object.min.z
                        && point.z <= object.max.z
                );
            }
        }
        assert_eq!(acquisition.scan.stamp, scan.stamp);
    }
    #[test]
    fn ordinal_fault_and_sparse_ground_are_errors_without_planar_fallback() {
        let (cfg, mut scan) = measured_patch();
        let config = Perception3dConfig::default();
        scan.returns.truncate(1);
        let error = process(&cfg, &config, &scan).err().unwrap();
        assert!(!error.confident);
        assert!(error.reason.unwrap().contains("support"));
        scan.returns.push(scan.returns[0].clone());
        let error = process(&cfg, &config, &scan).err().unwrap();
        assert!(error.reason.unwrap().contains("ordinal"));
        assert!(error.objects.is_empty());
    }
    #[test]
    fn calibration_roundtrips_strictly_and_excludes_double_ground_removal() {
        let (mut cfg, _) = measured_patch();
        let config = Perception3dConfig::default();
        let value = serde_json::to_value(&config).unwrap();
        let decoded: Perception3dConfig = serde_json::from_value(value.clone()).unwrap();
        assert!(decoded.validate(&cfg).is_ok());
        let mut wrong = value;
        wrong["terrain"]["label"] = serde_json::json!("ground");
        assert!(serde_json::from_value::<Perception3dConfig>(wrong).is_err());
        cfg.ground = Some(crate::GroundConfig {
            reference_height_m: 0.0,
            max_slope: 0.05,
            max_height_offset_m: 0.03,
            residual_threshold_m: 0.02,
            fit_radius_m: 8.0,
            min_inliers: 200,
            min_sector_inliers: 20,
            min_cell_inliers: 6,
        });
        assert!(
            config
                .validate(&cfg)
                .unwrap_err()
                .contains("separate near-flat")
        );
        cfg.ground = None;
        let mut too_large = config;
        too_large.terrain.max_points = 100_000;
        assert!(too_large.validate(&cfg).is_err());
    }
    #[test]
    fn pipeline_measured_xyz_fault_latch_recovery_and_raw_replay() {
        use crate::replay::{SensorLog, verify};
        use crate::{DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame};
        use rustdrive_core::{Route, VehicleConfig};
        let (calibration, cloud) = measured_patch();
        let mut config = PipelineConfig::new(
            Route::new(vec![Vec2::default(), Vec2::new(80.0, 0.0)], 5.5).unwrap(),
            Pose::default(),
            VehicleConfig::default(),
        );
        assert!(
            serde_json::to_value(&config)
                .unwrap()
                .get("perception3d")
                .is_none()
        );
        config.lidar3d = Some(calibration);
        config.perception3d = Some(Perception3dConfig::default());
        let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
        let mut log = SensorLog::new("raw-terrain-XYZ", config);
        for index in 0..4 {
            let time = index as f64 * 0.05;
            let mut measured = cloud.clone();
            measured.stamp = time;
            if index == 1 {
                measured.returns.truncate(1);
            }
            let input: SensorFrame = serde_json::from_value(serde_json::json!({"time":time,
                "odometry":{"stamp":time,"speed":0.0,"yaw_rate":0.0},
                "gnss":{"stamp":time,"position":{"x":0.0,"y":0.0},"variance":0.1},
                "lidar":null,"lidar3d":if index==2 {None} else {Some(measured)},"lidar_failed":false
            }))
            .unwrap();
            let output = pipeline.step(&input).unwrap();
            if index == 0 || index == 3 {
                assert!(output.perception3d.as_ref().unwrap().confident);
                assert!(!output.health.contains(&HealthIssue::InvalidLidar));
                assert!(!output.tracks.is_empty());
            } else {
                assert!(output.health.contains(&HealthIssue::InvalidLidar));
                assert!(output.emergency);
                assert_eq!(output.command.acceleration, -6.0);
            }
            log.record(input, output);
        }
        let mut bytes = vec![];
        log.write(&mut bytes).unwrap();
        assert!(
            verify(std::io::Cursor::new(bytes), std::io::sink())
                .unwrap()
                .verified
        );
    }
}
