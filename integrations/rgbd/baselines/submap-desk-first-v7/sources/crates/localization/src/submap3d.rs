//! Bounded measured-cloud fusion in a fixed initial-camera coordinate frame.
//!
//! Accepted registrations supply local root poses. Only accepted clouds can
//! update the map; an atomic update rejected by a resource limit leaves the
//! previous map intact. This is neither global SLAM nor a confidence guarantee:
//! ICP covariance excludes correlated map error and association uncertainty.
use crate::registration3d::{Pose3, Registration3dConfig, Registration3dResult, match_scan};
use rustdriving_core::Vec3;
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct SubmapConfig3d {
    pub registration: Registration3dConfig,
    pub voxel_m: f64,
    pub map_update_interval_s: f64,
    pub max_unobserved_s: f64,
    /// Reject the whole proposed map update before a voxel count exceeds this.
    pub max_points_per_voxel: usize,
}
impl Default for SubmapConfig3d {
    fn default() -> Self {
        Self {
            registration: Registration3dConfig {
                max_map_points: 5000,
                max_correspondence_m: 0.15,
                ..Default::default()
            },
            voxel_m: 0.06,
            map_update_interval_s: 0.10,
            max_unobserved_s: 0.20,
            max_points_per_voxel: 1000,
        }
    }
}
impl SubmapConfig3d {
    pub fn validate(&self) -> Result<(), String> {
        self.registration.validate()?;
        if self.registration.max_map_points > 5000
            || !self.voxel_m.is_finite()
            || !(0.001..=1.0).contains(&self.voxel_m)
            || !self.map_update_interval_s.is_finite()
            || !self.max_unobserved_s.is_finite()
            || self.map_update_interval_s <= 0.0
            || self.map_update_interval_s > self.max_unobserved_s
            || self.max_unobserved_s > 10.0
            || self.max_points_per_voxel == 0
            || self.max_points_per_voxel > 1_000_000
        {
            return Err("invalid bounded measured-submap configuration".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct SubmapRegistration3d {
    pub root_origin_frame_index: usize,
    /// Fixed initial-camera frame from this measured sensor frame.
    pub root_from_scan: Pose3,
    pub initial_pose: Pose3,
    /// Registration is against the map BEFORE this acquisition is integrated.
    /// Initialization instead guards the first cloud against itself.
    pub registration: Registration3dResult,
    pub initialized: bool,
    pub map_generation_before: usize,
    pub map_generation_after: usize,
    pub map_point_count_before: usize,
    pub map_point_count_after: usize,
    pub map_updated: bool,
    /// A valid pose can survive a rejected map update; it still renews pose age.
    pub map_update_rejection: Option<String>,
}

/// Historical map statistics for bounded audit/export, never ground truth.
#[derive(Clone, Debug)]
pub struct SubmapVoxel3d {
    pub key: (i64, i64, i64),
    pub mean: Vec3,
    pub count: usize,
}

#[derive(Clone)]
struct Voxel {
    mean: Vec3,
    count: usize,
}
type Voxels = BTreeMap<(i64, i64, i64), Voxel>;

fn fused(
    original: &Voxels,
    scan: &[Vec3],
    root_from_scan: Pose3,
    config: &SubmapConfig3d,
) -> Result<Voxels, String> {
    // At most two bounded maps and one input scan exist during an update.
    let mut proposed = original.clone();
    for &point in scan {
        let p = root_from_scan.transform(point);
        let cells = [p.x, p.y, p.z].map(|v| (v / config.voxel_m).floor());
        if !p.finite() || cells.iter().any(|v| !v.is_finite() || v.abs() > 1e14) {
            return Err("submap transformed coordinate exceeds voxel bound".into());
        }
        let key = (cells[0] as i64, cells[1] as i64, cells[2] as i64);
        if let Some(voxel) = proposed.get_mut(&key) {
            if voxel.count >= config.max_points_per_voxel {
                return Err("submap voxel observation-count limit reached".into());
            }
            let count = voxel.count + 1;
            // Stable online mean; never sum large coordinates or reweight an
            // existing representative as if it were one raw measurement.
            let n = count as f64;
            let mean = Vec3::new(
                voxel.mean.x + (p.x - voxel.mean.x) / n,
                voxel.mean.y + (p.y - voxel.mean.y) / n,
                voxel.mean.z + (p.z - voxel.mean.z) / n,
            );
            if !mean.finite() {
                return Err("nonfinite submap voxel representative".into());
            }
            *voxel = Voxel { mean, count };
        } else {
            if proposed.len() >= config.registration.max_map_points {
                return Err("submap point limit reached".into());
            }
            proposed.insert(key, Voxel { mean: p, count: 1 });
        }
    }
    if proposed.len() < config.registration.min_pairs {
        return Err("too few fused submap representatives".into());
    }
    Ok(proposed)
}

pub struct SubmapLocalizer3d {
    config: SubmapConfig3d,
    map: Voxels,
    root_origin_frame_index: Option<usize>,
    last_observed: Option<(usize, f64)>,
    last_accepted_stamp: Option<f64>,
    last_map_update_stamp: Option<f64>,
    last_pose: Pose3,
    map_generation: usize,
    lost: bool,
}
impl SubmapLocalizer3d {
    pub fn new(config: SubmapConfig3d) -> Result<Self, String> {
        config.validate()?;
        Ok(Self {
            config,
            map: BTreeMap::new(),
            root_origin_frame_index: None,
            last_observed: None,
            last_accepted_stamp: None,
            last_map_update_stamp: None,
            last_pose: Pose3::identity(),
            map_generation: 0,
            lost: false,
        })
    }
    /// Reset starts a separate origin; it cannot reconnect a lost trajectory.
    pub fn reset(&mut self) {
        self.map.clear();
        self.root_origin_frame_index = None;
        self.last_observed = None;
        self.last_accepted_stamp = None;
        self.last_map_update_stamp = None;
        self.last_pose = Pose3::identity();
        self.map_generation = 0;
        self.lost = false;
    }
    pub fn is_lost(&self) -> bool {
        self.lost
    }
    pub fn last_accepted_stamp(&self) -> Option<f64> {
        self.last_accepted_stamp
    }
    pub fn map_generation(&self) -> usize {
        self.map_generation
    }
    pub fn root_origin_frame_index(&self) -> Option<usize> {
        self.root_origin_frame_index
    }
    pub fn last_map_update_stamp(&self) -> Option<f64> {
        self.last_map_update_stamp
    }
    /// Last accepted historical pose, including after loss. Callers must check
    /// acquisition validity; this does not grant permission to reuse a pose.
    pub fn last_pose(&self) -> Option<Pose3> {
        self.root_origin_frame_index.map(|_| self.last_pose)
    }
    pub fn map_voxels(&self) -> Vec<SubmapVoxel3d> {
        self.map
            .iter()
            .map(|(&key, value)| SubmapVoxel3d {
                key,
                mean: value.mean,
                count: value.count,
            })
            .collect()
    }
    /// Deterministic lexicographic voxel-key order, in the fixed local root.
    pub fn map_points(&self) -> Vec<Vec3> {
        self.map.values().map(|v| v.mean).collect()
    }
    pub fn observe(
        &mut self,
        frame_index: usize,
        stamp: f64,
        scan: &[Vec3],
    ) -> Result<SubmapRegistration3d, String> {
        if self.lost {
            return Err("submap localization lost; explicit new origin required".into());
        }
        if !stamp.is_finite()
            || !(0.0..=1e12).contains(&stamp)
            || self
                .last_observed
                .is_some_and(|(index, previous)| frame_index <= index || stamp <= previous)
        {
            return Err("invalid or non-increasing submap observation".into());
        }
        // Acquisition chronology advances even for a malformed cloud. Check
        // pose expiry first so invalid data cannot conceal an elapsed timeout.
        self.last_observed = Some((frame_index, stamp));
        if self
            .last_accepted_stamp
            .is_some_and(|accepted| stamp - accepted > self.config.max_unobserved_s + 1e-9)
        {
            self.lost = true;
            return Err("submap accepted-pose age exceeded; localization lost".into());
        }
        if scan.len() < self.config.registration.min_pairs
            || scan.len() > self.config.registration.max_scan_points
            || scan.iter().any(|p| !p.finite())
        {
            if self.root_origin_frame_index.is_none() {
                self.lost = true;
            }
            return Err("invalid or oversized submap cloud".into());
        }
        if self.root_origin_frame_index.is_none() {
            // Both self-match and fusion must succeed before publishing state.
            let initialized = (|| {
                let registration =
                    match_scan(scan, scan, Pose3::identity(), &self.config.registration)?;
                let map = fused(&self.map, scan, Pose3::identity(), &self.config)?;
                Ok::<_, String>((registration, map))
            })();
            let (registration, map) = match initialized {
                Ok(value) => value,
                Err(reason) => {
                    self.lost = true;
                    return Err(format!("submap initialization rejected: {reason}"));
                }
            };
            let point_count = map.len();
            self.map = map;
            self.root_origin_frame_index = Some(frame_index);
            self.last_accepted_stamp = Some(stamp);
            self.last_map_update_stamp = Some(stamp);
            self.last_pose = Pose3::identity();
            self.map_generation = 1;
            return Ok(SubmapRegistration3d {
                root_origin_frame_index: frame_index,
                root_from_scan: Pose3::identity(),
                initial_pose: Pose3::identity(),
                registration,
                initialized: true,
                map_generation_before: 0,
                map_generation_after: 1,
                map_point_count_before: 0,
                map_point_count_after: point_count,
                map_updated: true,
                map_update_rejection: None,
            });
        }
        let initial_pose = self.last_pose;
        let points = self.map_points();
        let registration = match_scan(scan, &points, initial_pose, &self.config.registration)?;
        let root_from_scan = registration.pose;
        let generation_before = self.map_generation;
        let count_before = self.map.len();
        let mut updated = false;
        let mut update_rejection = None;
        if self
            .last_map_update_stamp
            .is_some_and(|last| stamp - last + 1e-9 >= self.config.map_update_interval_s)
        {
            match fused(&self.map, scan, root_from_scan, &self.config) {
                Ok(map) => match self.map_generation.checked_add(1) {
                    Some(generation) => {
                        self.map = map;
                        self.map_generation = generation;
                        self.last_map_update_stamp = Some(stamp);
                        updated = true;
                    }
                    None => update_rejection = Some("submap generation limit reached".into()),
                },
                Err(reason) => update_rejection = Some(reason),
            }
        }
        self.last_pose = root_from_scan;
        self.last_accepted_stamp = Some(stamp);
        Ok(SubmapRegistration3d {
            root_origin_frame_index: self.root_origin_frame_index.unwrap(),
            root_from_scan,
            initial_pose,
            registration,
            initialized: false,
            map_generation_before: generation_before,
            map_generation_after: self.map_generation,
            map_point_count_before: count_before,
            map_point_count_after: self.map.len(),
            map_updated: updated,
            map_update_rejection: update_rejection,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registration3d::Quaternion;

    fn geometry() -> Vec<Vec3> {
        (0..180)
            .map(|i| {
                let t = i as f64;
                Vec3::new(
                    (t * 1.73).sin() * 1.3 + (t * 0.21).cos() * 0.2,
                    (t * 0.77).cos() * 0.9,
                    (t * 0.49).sin() * 0.7 + (t * 0.11).cos() * 0.1,
                )
            })
            .collect()
    }
    fn assert_pose(actual: Pose3, expected: Pose3) {
        assert!((actual.translation.x - expected.translation.x).abs() < 0.005);
        assert!((actual.translation.y - expected.translation.y).abs() < 0.005);
        assert!((actual.translation.z - expected.translation.z).abs() < 0.005);
        assert!(actual.rotation.angular_distance(expected.rotation) < 0.005);
    }
    #[test]
    fn fused_clouds_keep_poses_in_the_original_measured_origin() {
        let world = geometry();
        let mut local = SubmapLocalizer3d::new(SubmapConfig3d::default()).unwrap();
        let first = local.observe(7, 0.0, &world).unwrap();
        assert!(first.initialized);
        assert_eq!(first.map_generation_before, 0);
        assert_eq!(first.map_generation_after, 1);
        for index in 1..=5 {
            let pose = Pose3 {
                translation: Vec3::new(0.02 * index as f64, -0.01, 0.01),
                rotation: Quaternion::from_axis_angle(
                    Vec3::new(0.3, 0.7, 0.2),
                    0.01 * index as f64,
                )
                .unwrap(),
            };
            let scan: Vec<_> = world.iter().map(|p| pose.inverse().transform(*p)).collect();
            let result = local.observe(7 + index, 0.1 * index as f64, &scan).unwrap();
            assert_eq!(result.root_origin_frame_index, 7);
            assert_eq!(result.registration.conditioning.ambiguity_probes, 12);
            assert!(result.map_updated);
            assert_eq!(result.map_generation_before, index);
            assert_eq!(result.map_generation_after, index + 1);
            assert!(result.map_point_count_after <= 5000);
            assert_pose(result.root_from_scan, pose);
        }
    }
    #[test]
    fn rejected_fit_does_not_fuse_points_or_renew_pose_age() {
        let cloud = geometry();
        let mut local = SubmapLocalizer3d::new(SubmapConfig3d::default()).unwrap();
        local.observe(0, 0.0, &cloud).unwrap();
        let map = local.map_points();
        let unrelated: Vec<_> = cloud
            .iter()
            .map(|p| Vec3::new(p.x + 20.0, p.y, p.z))
            .collect();
        assert!(local.observe(1, 0.1, &unrelated).is_err());
        assert_eq!(local.map_points(), map);
        assert_eq!(local.map_generation(), 1);
        assert_eq!(local.last_accepted_stamp(), Some(0.0));
        assert!(local.observe(2, 0.15, &cloud).unwrap().map_updated);
        assert_eq!(local.map_generation(), 2);
    }
    #[test]
    fn malformed_late_acquisition_latches_loss_until_a_new_origin_is_declared() {
        let cloud = geometry();
        let mut local = SubmapLocalizer3d::new(SubmapConfig3d::default()).unwrap();
        local.observe(0, 0.0, &cloud).unwrap();
        assert!(local.observe(1, 0.201, &[]).is_err());
        assert!(local.is_lost());
        assert_eq!(local.last_accepted_stamp(), Some(0.0));
        assert!(local.observe(2, 0.15, &cloud).is_err());
        local.reset();
        let result = local.observe(2, 0.3, &cloud).unwrap();
        assert!(result.initialized);
        assert_eq!(result.root_origin_frame_index, 2);
        assert_eq!(result.map_generation_after, 1);
    }
    #[test]
    fn malformed_and_retrograde_acquisitions_do_not_refresh_permission() {
        let cloud = geometry();
        let mut local = SubmapLocalizer3d::new(SubmapConfig3d::default()).unwrap();
        local.observe(0, 0.0, &cloud).unwrap();
        assert!(local.observe(0, 0.01, &cloud).is_err());
        assert!(local.observe(1, f64::NAN, &cloud).is_err());
        let mut bad = cloud.clone();
        bad[0].x = f64::INFINITY;
        assert!(local.observe(1, 0.02, &bad).is_err());
        assert!(local.observe(1, 0.03, &cloud).is_err());
        assert!(local.observe(2, 0.01, &cloud).is_err());
        assert_eq!(local.last_accepted_stamp(), Some(0.0));
        assert!(local.observe(2, 0.05, &cloud).is_ok());
    }
    #[test]
    fn full_map_update_is_atomic_when_new_voxels_exceed_the_point_limit() {
        let cloud = geometry();
        let mut config = SubmapConfig3d::default();
        let original = fused(&BTreeMap::new(), &cloud, Pose3::identity(), &config).unwrap();
        config.registration.max_map_points = original.len();
        let mut local = SubmapLocalizer3d::new(config).unwrap();
        local.observe(0, 0.0, &cloud).unwrap();
        let before = local.map_points();
        let mut additional = cloud.clone();
        additional.push(Vec3::new(20.0, 20.0, 20.0));
        let result = local.observe(1, 0.1, &additional).unwrap();
        assert!(!result.map_updated);
        assert!(result.map_update_rejection.unwrap().contains("point limit"));
        assert_eq!(result.map_generation_after, 1);
        assert_eq!(local.map_points(), before);
        assert_eq!(local.last_accepted_stamp(), Some(0.1));
        let result = local.observe(2, 0.2, &cloud).unwrap();
        assert!(result.map_updated);
        assert_eq!(result.map_generation_after, 2);
    }
    #[test]
    fn voxel_count_limit_preserves_existing_means_and_pose_can_remain_valid() {
        let cloud = geometry();
        let config = SubmapConfig3d {
            max_points_per_voxel: 2,
            ..Default::default()
        };
        let mut local = SubmapLocalizer3d::new(config).unwrap();
        local.observe(0, 0.0, &cloud).unwrap();
        local.observe(1, 0.1, &cloud).unwrap();
        let before = local.map_points();
        let generation = local.map_generation();
        let result = local.observe(2, 0.2, &cloud).unwrap();
        assert!(!result.map_updated);
        assert!(result.map_update_rejection.unwrap().contains("count limit"));
        assert_eq!(local.map_generation(), generation);
        assert_eq!(local.map_points(), before);
        assert_eq!(local.last_accepted_stamp(), Some(0.2));
    }
    #[test]
    fn representatives_average_raw_observations_in_deterministic_voxel_order() {
        let config = SubmapConfig3d {
            registration: Registration3dConfig {
                min_pairs: 6,
                ..Default::default()
            },
            ..Default::default()
        };
        let first: Vec<_> = (0..6)
            .map(|i| Vec3::new(i as f64 * 0.12 + 0.01, 0.01, 0.01))
            .collect();
        let second: Vec<_> = first
            .iter()
            .map(|p| Vec3::new(p.x + 0.01, p.y, p.z))
            .collect();
        let mut map = fused(&BTreeMap::new(), &first, Pose3::identity(), &config).unwrap();
        map = fused(&map, &second, Pose3::identity(), &config).unwrap();
        for (index, voxel) in map.values().enumerate() {
            assert_eq!(voxel.count, 2);
            assert!((voxel.mean.x - (index as f64 * 0.12 + 0.015)).abs() < 1e-12);
        }
        let before = map.clone();
        assert!(
            fused(
                &map,
                &[Vec3::new(1e20, 0.0, 0.0)],
                Pose3::identity(),
                &config
            )
            .is_err()
        );
        assert_eq!(map.len(), before.len());
        for (a, b) in map.values().zip(before.values()) {
            assert_eq!(a.mean, b.mean);
            assert_eq!(a.count, b.count);
        }
    }
    #[test]
    fn unobservable_initialization_and_invalid_resource_settings_are_rejected() {
        let plane: Vec<_> = (0..10)
            .flat_map(|x| (0..10).map(move |y| Vec3::new(x as f64 * 0.06, y as f64 * 0.06, 2.0)))
            .collect();
        let mut local = SubmapLocalizer3d::new(SubmapConfig3d::default()).unwrap();
        assert!(local.observe(0, 0.0, &plane).is_err());
        assert!(local.is_lost());
        assert!(local.map_points().is_empty());
        let mut invalid = SubmapConfig3d::default();
        invalid.registration.max_map_points = 5001;
        assert!(SubmapLocalizer3d::new(invalid).is_err());
        let invalid = SubmapConfig3d {
            voxel_m: f64::NAN,
            ..Default::default()
        };
        assert!(SubmapLocalizer3d::new(invalid).is_err());
    }
}
