//! Optional terrain and measured XYZ object perception on validated raw returns.
//! Uses acquisition-body XYZ only; no point labels, scene roles or world truth.
use crate::Lidar3dConfig;
use rustdriving_core::{Detection, Lidar3dScan, LidarScan, Pose, Vec2, Vec3};
use rustdriving_perception::{
    AdaptiveTerrainConfig, ObjectClusterConfig, TerrainConfig, classify_ground,
    classify_ground_adaptive, cluster_objects,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Perception3dConfig {
    #[serde(with = "TerrainConfigSerde")]
    pub terrain: TerrainConfig,
    #[serde(with = "ObjectConfigSerde")]
    pub objects: ObjectClusterConfig,
    /// Explicit opt-in; the frozen PMF profile remains the default.
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        with = "adaptive_option"
    )]
    pub adaptive_terrain: Option<AdaptiveTerrainConfig>,
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
            adaptive_terrain: None,
        }
    }
}
impl Perception3dConfig {
    /// Frozen adaptive algorithm with explicit per-acquisition resource caps.
    /// The smaller caps are limits, not a guarantee that any scan is supported.
    pub fn adaptive() -> Self {
        Self {
            adaptive_terrain: Some(AdaptiveTerrainConfig {
                max_points: 20_000,
                max_cells: 20_000,
                max_candidate_work: 8_000_000,
                ..AdaptiveTerrainConfig::default()
            }),
            ..Self::default()
        }
    }
    pub(crate) fn validate(&self, calibration: &Lidar3dConfig) -> Result<(), String> {
        calibration.validate()?;
        if calibration.ground.is_some() {
            return Err("3D terrain perception excludes the separate near-flat ground mode".into());
        }
        self.terrain.validate()?;
        self.objects.validate()?;
        if let Some(adaptive) = &self.adaptive_terrain {
            adaptive.validate()?;
            if adaptive.max_points > 20_000
                || adaptive.max_cells > 20_000
                || adaptive.max_candidate_work > 8_000_000
            {
                return Err(
                    "adaptive XYZ terrain exceeds 20000-return/8000000-work contract".into(),
                );
            }
        }
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
#[serde(remote = "AdaptiveTerrainConfig", deny_unknown_fields)]
struct AdaptiveTerrainConfigSerde {
    cell_size_m: f64,
    support_radius_m: f64,
    max_slope: f64,
    max_residual_m: f64,
    min_support_neighbors: usize,
    max_support_neighbors: usize,
    min_supported_cells: usize,
    min_supported_fraction: f64,
    max_points: usize,
    max_cells: usize,
    max_candidate_work: usize,
    coordinate_bound_m: f64,
}
mod adaptive_option {
    use super::*;
    #[derive(Serialize)]
    struct Borrowed<'a>(#[serde(with = "AdaptiveTerrainConfigSerde")] &'a AdaptiveTerrainConfig);
    #[derive(Deserialize)]
    struct Owned(#[serde(with = "AdaptiveTerrainConfigSerde")] AdaptiveTerrainConfig);
    pub fn serialize<S: serde::Serializer>(
        value: &Option<AdaptiveTerrainConfig>,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        value.as_ref().map(Borrowed).serialize(serializer)
    }
    pub fn deserialize<'de, D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> Result<Option<AdaptiveTerrainConfig>, D::Error> {
        Option::<Owned>::deserialize(deserializer).map(|value| value.map(|wrapped| wrapped.0))
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
pub struct AdaptiveTerrainSupportDiagnostics {
    pub occupied_cells: usize,
    pub supported_cells: usize,
    pub rejected_low_cells: usize,
    pub rejected_elevated_cells: usize,
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub adaptive_terrain: Option<AdaptiveTerrainSupportDiagnostics>,
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
        adaptive_terrain: None,
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
    let (ground_indices, non_ground_indices, support) =
        if let Some(adaptive) = &config.adaptive_terrain {
            let ground = classify_ground_adaptive(&points, adaptive)
                .map_err(|error| failure(&diagnostics, error))?;
            let support = ground.diagnostics;
            diagnostics.adaptive_terrain = Some(AdaptiveTerrainSupportDiagnostics {
                occupied_cells: support.occupied_cells,
                supported_cells: support.supported_cells,
                rejected_low_cells: support.rejected_low_cells,
                rejected_elevated_cells: support.rejected_elevated_cells,
                candidate_work: support.candidate_work,
                supported_fraction: support.supported_fraction,
                confident: support.confident,
            });
            // Adaptive candidates are the cells with a supported fitted terrain
            // surface. Original points still must pass their own XYZ residual test;
            // an elevated point is not ground merely because a surface was fitted.
            let summary = TerrainSupportDiagnostics {
                occupied_cells: support.occupied_cells,
                candidate_ground_cells: support.supported_cells,
                supported_cells: support.supported_cells,
                rejected_cells: support.occupied_cells - support.supported_cells,
                candidate_work: support.candidate_work,
                supported_fraction: support.supported_fraction,
                confident: support.confident,
            };
            (ground.ground_indices, ground.non_ground_indices, summary)
        } else {
            let ground = classify_ground(&points, &config.terrain)
                .map_err(|error| failure(&diagnostics, error))?;
            let support = ground.diagnostics;
            let summary = TerrainSupportDiagnostics {
                occupied_cells: support.occupied_cells,
                candidate_ground_cells: support.candidate_ground_cells,
                supported_cells: support.supported_cells,
                rejected_cells: support.rejected_cells,
                candidate_work: support.candidate_work,
                supported_fraction: support.supported_fraction,
                confident: support.confident,
            };
            (ground.ground_indices, ground.non_ground_indices, summary)
        };
    let confident = support.confident;
    diagnostics.terrain = Some(support);
    diagnostics.ground_points = ground_indices.len();
    diagnostics.non_ground_points = non_ground_indices.len();
    diagnostics.ground_return_indices = ground_indices;
    if !confident {
        return Err(failure(
            &diagnostics,
            "insufficient measured terrain support",
        ));
    }
    let objects = cluster_objects(&points, &non_ground_indices, &config.objects)
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
    use rustdriving_core::Lidar3dReturn;
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
    fn adaptive_acquisition(columns: usize, rings: usize) -> (Lidar3dConfig, Lidar3dScan) {
        let (mut cfg, _) = measured_patch();
        cfg.azimuth_columns = columns;
        cfg.elevation_rings = rings;
        let mut returns = vec![];
        for column in 0..columns {
            for ring in 0..rings {
                let a = -PI + TAU * column as f64 / (columns - 1) as f64;
                let e = cfg.min_elevation_rad
                    + (cfg.max_elevation_rad - cfg.min_elevation_rad) * ring as f64
                        / (rings - 1) as f64;
                let direction = Vec3::new(e.cos() * a.cos(), -e.cos() * a.sin(), e.sin());
                let denom = direction.z - 0.12 * direction.x - 0.03 * direction.y;
                let mut closest = if denom < 0.0 {
                    -cfg.mount_height_m / denom
                } else {
                    f64::INFINITY
                };
                // A low solid obstacle above the local terrain and an overhead
                // roof. Ray intersections generate points; the pipeline receives
                // neither these boxes nor their authored roles.
                for (min, max) in [
                    (Vec3::new(5.0, -2.0, 1.1), Vec3::new(7.0, 2.0, 1.4)),
                    (Vec3::new(-8.0, -3.0, 3.0), Vec3::new(-5.0, 3.0, 4.0)),
                ] {
                    let mut near: f64 = 0.0;
                    let mut far = f64::INFINITY;
                    for (origin, ray, bottom, top) in [
                        (0.0, direction.x, min.x, max.x),
                        (0.0, direction.y, min.y, max.y),
                        (cfg.mount_height_m, direction.z, min.z, max.z),
                    ] {
                        if ray.abs() < 1e-12 {
                            if origin < bottom || origin > top {
                                far = -1.0;
                            }
                        } else {
                            let t0 = (bottom - origin) / ray;
                            let t1 = (top - origin) / ray;
                            near = near.max(t0.min(t1));
                            far = far.min(t0.max(t1));
                        }
                    }
                    if far >= near && near > 0.0 {
                        closest = closest.min(near);
                    }
                }
                if (cfg.min_range_m..=cfg.max_range_m).contains(&closest) {
                    returns.push(Lidar3dReturn {
                        ray_index: column * rings + ring,
                        point: Vec3::new(
                            closest * direction.x,
                            closest * direction.y,
                            cfg.mount_height_m + closest * direction.z,
                        ),
                    });
                }
            }
        }
        (
            cfg,
            Lidar3dScan {
                stamp: 0.0,
                returns,
            },
        )
    }
    #[test]
    fn adaptive_raw_sparse_and_dense_slopes_preserve_low_objects_and_roofs() {
        for (columns, rings) in [(128, 16), (256, 32)] {
            let (calibration, scan) = adaptive_acquisition(columns, rings);
            assert!(calibration.validate_returns(&scan).is_ok());
            let output = process(&calibration, &Perception3dConfig::adaptive(), &scan).unwrap();
            let adaptive = output.diagnostics.adaptive_terrain.as_ref().unwrap();
            assert!(adaptive.confident);
            assert!(adaptive.candidate_work <= 8_000_000);
            assert!(output.diagnostics.ground_points > scan.returns.len() / 2);
            let relevant: Vec<_> = output
                .diagnostics
                .objects
                .iter()
                .filter(|o| o.collision_relevant)
                .collect();
            assert!(
                relevant
                    .iter()
                    .any(|o| o.min.x >= 4.9 && o.max.x <= 7.1 && o.min.z >= 1.09)
            );
            assert!(
                output
                    .diagnostics
                    .objects
                    .iter()
                    .any(|o| !o.collision_relevant && o.min.z >= 2.99)
            );
            assert!(!output.scan.points.is_empty());
            for object in relevant {
                assert!(
                    object
                        .return_indices
                        .iter()
                        .all(|i| !output.diagnostics.ground_return_indices.contains(i))
                );
            }
        }
    }
    #[test]
    fn adaptive_configuration_is_strict_bounded_and_omitted_from_legacy_json() {
        let (calibration, scan) = measured_patch();
        let legacy = Perception3dConfig::default();
        let old_json = serde_json::to_value(&legacy).unwrap();
        assert!(old_json.get("adaptive_terrain").is_none());
        let decoded: Perception3dConfig = serde_json::from_value(old_json.clone()).unwrap();
        let a = process(&calibration, &legacy, &scan).unwrap();
        let b = process(&calibration, &decoded, &scan).unwrap();
        assert_eq!(
            serde_json::to_vec(&a.diagnostics).unwrap(),
            serde_json::to_vec(&b.diagnostics).unwrap()
        );
        assert!(
            serde_json::to_value(&a.diagnostics)
                .unwrap()
                .get("adaptive_terrain")
                .is_none()
        );
        let config = Perception3dConfig::adaptive();
        let value = serde_json::to_value(&config).unwrap();
        assert_eq!(value["adaptive_terrain"]["max_candidate_work"], 8_000_000);
        let decoded: Perception3dConfig = serde_json::from_value(value.clone()).unwrap();
        assert!(decoded.validate(&calibration).is_ok());
        let mut unknown = value;
        unknown["adaptive_terrain"]["simulator_ground_labels"] = serde_json::json!(true);
        assert!(serde_json::from_value::<Perception3dConfig>(unknown).is_err());
        for (points, cells, work) in [
            (20_001, 20_000, 8_000_000),
            (20_000, 20_001, 8_000_000),
            (20_000, 20_000, 8_000_001),
        ] {
            let mut excessive = config.clone();
            let adaptive = excessive.adaptive_terrain.as_mut().unwrap();
            adaptive.max_points = points;
            adaptive.max_cells = cells;
            adaptive.max_candidate_work = work;
            assert!(excessive.validate(&calibration).is_err());
        }
    }
    #[test]
    fn adaptive_unsupported_geometry_and_work_exhaustion_fail_without_fallback() {
        let (cfg, mut scan) = adaptive_acquisition(128, 16);
        let mut config = Perception3dConfig::adaptive();
        config.adaptive_terrain.as_mut().unwrap().max_candidate_work = 1;
        let error = process(&cfg, &config, &scan).err().unwrap();
        assert!(error.reason.unwrap().contains("work"));
        assert!(error.objects.is_empty());
        assert!(!error.confident);
        config = Perception3dConfig::adaptive();
        scan.returns.truncate(1);
        let error = process(&cfg, &config, &scan).err().unwrap();
        assert!(error.reason.unwrap().contains("support"));
        assert!(error.adaptive_terrain.is_some());
        assert!(error.objects.is_empty());
        assert!(!error.confident);
        scan.returns.push(scan.returns[0].clone());
        let error = process(&cfg, &config, &scan).err().unwrap();
        assert!(error.reason.unwrap().contains("ordinal"));
        assert!(error.adaptive_terrain.is_none());
    }
    #[test]
    fn adaptive_sensor_only_replay_preserves_fault_epoch_and_recovery() {
        use crate::replay::{SensorLog, verify};
        use crate::{DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame};
        use rustdriving_core::{Route, VehicleConfig};
        let (calibration, cloud) = adaptive_acquisition(128, 16);
        let mut config = PipelineConfig::new(
            Route::new(vec![Vec2::default(), Vec2::new(80.0, 0.0)], 5.5).unwrap(),
            Pose::default(),
            VehicleConfig::default(),
        );
        config.lidar3d = Some(calibration);
        config.perception3d = Some(Perception3dConfig::adaptive());
        let mut pipeline = DrivingPipeline::new(config.clone()).unwrap();
        let mut log = SensorLog::new("adaptive-measured-XYZ", config);
        for index in 0..4 {
            let time = index as f64 * 0.05;
            let mut measured = cloud.clone();
            measured.stamp = if index == 2 { 0.0 } else { time };
            if index == 1 {
                measured.returns.truncate(1);
            }
            let input: SensorFrame = serde_json::from_value(serde_json::json!({"time":time,
                "odometry":{"stamp":time,"speed":0.0,"yaw_rate":0.0},
                "gnss":{"stamp":time,"position":{"x":0.0,"y":0.0},"variance":0.1},
                "lidar":null,"lidar3d":measured,"lidar_failed":false
            }))
            .unwrap();
            let output = pipeline.step(&input).unwrap();
            if index == 0 || index == 3 {
                assert!(
                    output
                        .perception3d
                        .as_ref()
                        .unwrap()
                        .adaptive_terrain
                        .as_ref()
                        .unwrap()
                        .confident
                );
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
    #[test]
    fn pipeline_measured_xyz_fault_latch_recovery_and_raw_replay() {
        use crate::replay::{SensorLog, verify};
        use crate::{DrivingPipeline, HealthIssue, PipelineConfig, SensorFrame};
        use rustdriving_core::{Route, VehicleConfig};
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
