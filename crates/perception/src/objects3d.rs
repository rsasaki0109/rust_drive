//! Bounded voxel-neighbor Euclidean components with measured XYZ AABBs.
//! Outputs describe observed surfaces; they do not infer hidden shape or semantics.
use rustdrive_core::Vec3;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug)]
pub struct ObjectClusterConfig {
    pub tolerance_m: f64,
    pub voxel_size_m: f64,
    pub min_points: usize,
    pub max_points: usize,
    pub max_cluster_points: usize,
    pub max_clusters: usize,
    pub max_candidate_work: usize,
    pub coordinate_bound_m: f64,
}
impl Default for ObjectClusterConfig {
    fn default() -> Self {
        Self {
            tolerance_m: 0.6,
            voxel_size_m: 0.6,
            min_points: 3,
            max_points: 500_000,
            max_cluster_points: 200_000,
            max_clusters: 10_000,
            max_candidate_work: 20_000_000,
            coordinate_bound_m: 10_000_000.0,
        }
    }
}
impl ObjectClusterConfig {
    pub fn validate(&self) -> Result<(), String> {
        if !self.tolerance_m.is_finite()
            || !(0.01..=20.0).contains(&self.tolerance_m)
            || !self.voxel_size_m.is_finite()
            || !(0.01..=20.0).contains(&self.voxel_size_m)
            || self.tolerance_m / self.voxel_size_m > 8.0
            || self.min_points == 0
            || self.min_points > self.max_cluster_points
            || !(1..=2_000_000).contains(&self.max_points)
            || self.max_cluster_points == 0
            || self.max_cluster_points > self.max_points
            || !(1..=100_000).contains(&self.max_clusters)
            || !(1..=500_000_000).contains(&self.max_candidate_work)
            || !self.coordinate_bound_m.is_finite()
            || !(1.0..=10_000_000.0).contains(&self.coordinate_bound_m)
        {
            return Err("invalid or unsupported XYZ clustering calibration/resource bounds".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug)]
pub struct ObjectAabb {
    pub center: Vec3,
    pub min: Vec3,
    pub max: Vec3,
    pub point_count: usize,
    pub indices: Vec<usize>,
}
#[derive(Clone, Debug, Default)]
pub struct ObjectClassification {
    pub objects: Vec<ObjectAabb>,
    /// Small components, retained explicitly rather than fabricated objects.
    pub noise_indices: Vec<usize>,
    pub candidate_work: usize,
    pub occupied_voxels: usize,
}
fn voxel(point: Vec3, size: f64) -> (i64, i64, i64) {
    (
        (point.x / size).floor() as i64,
        (point.y / size).floor() as i64,
        (point.z / size).floor() as i64,
    )
}
fn spend(work: &mut usize, maximum: usize) -> Result<(), String> {
    *work += 1;
    if *work > maximum {
        Err("XYZ clustering candidate-work limit exceeded".into())
    } else {
        Ok(())
    }
}

/// Cluster exactly the supplied original indices, typically non-ground output.
/// XYZ distance determines connectivity, not voxel identity or horizontal distance.
/// Resource failures never return partial objects. Order is deterministic by the
/// smallest original component index; points within each component are sorted.
pub fn cluster_objects(
    points: &[Vec3],
    indices: &[usize],
    config: &ObjectClusterConfig,
) -> Result<ObjectClassification, String> {
    config.validate()?;
    if points.len() > config.max_points || indices.len() > config.max_points {
        return Err("XYZ clustering point limit exceeded".into());
    }
    let mut selected = BTreeSet::new();
    let mut grid = BTreeMap::<(i64, i64, i64), Vec<usize>>::new();
    for &index in indices {
        let point = *points
            .get(index)
            .ok_or("XYZ clustering index outside input")?;
        if !selected.insert(index) {
            return Err("duplicate XYZ clustering index".into());
        }
        if !point.finite()
            || point.x.abs() > config.coordinate_bound_m
            || point.y.abs() > config.coordinate_bound_m
            || point.z.abs() > config.coordinate_bound_m
        {
            return Err("XYZ clustering point is non-finite or outside coordinate bound".into());
        }
        grid.entry(voxel(point, config.voxel_size_m))
            .or_default()
            .push(index);
    }
    let mut result = ObjectClassification {
        occupied_voxels: grid.len(),
        ..ObjectClassification::default()
    };
    let mut visited = vec![false; points.len()];
    let radius = (config.tolerance_m / config.voxel_size_m).ceil() as i64;
    let tolerance2 = config.tolerance_m * config.tolerance_m;
    for &start in &selected {
        if visited[start] {
            continue;
        }
        visited[start] = true;
        // Remove each accepted point from its bucket immediately, so dense
        // identical points do not entail a quadratic scan of visited entries.
        let bucket = grid
            .get_mut(&voxel(points[start], config.voxel_size_m))
            .unwrap();
        let slot = bucket.iter().position(|&index| index == start).unwrap();
        bucket.swap_remove(slot);
        let mut queue = vec![start];
        let mut cursor = 0;
        while cursor < queue.len() {
            let point = points[queue[cursor]];
            cursor += 1;
            let (x, y, z) = voxel(point, config.voxel_size_m);
            for dx in -radius..=radius {
                for dy in -radius..=radius {
                    for dz in -radius..=radius {
                        spend(&mut result.candidate_work, config.max_candidate_work)?;
                        if let Some(bucket) = grid.get_mut(&(x + dx, y + dy, z + dz)) {
                            let mut slot = 0;
                            while slot < bucket.len() {
                                spend(&mut result.candidate_work, config.max_candidate_work)?;
                                let index = bucket[slot];
                                let other = points[index];
                                let distance2 = (point.x - other.x).powi(2)
                                    + (point.y - other.y).powi(2)
                                    + (point.z - other.z).powi(2);
                                if distance2 <= tolerance2 {
                                    visited[index] = true;
                                    queue.push(index);
                                    bucket.swap_remove(slot);
                                    if queue.len() > config.max_cluster_points {
                                        return Err(
                                            "XYZ clustering component point limit exceeded".into(),
                                        );
                                    }
                                } else {
                                    slot += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        queue.sort_unstable();
        if queue.len() < config.min_points {
            result.noise_indices.extend(queue);
            continue;
        }
        if result.objects.len() >= config.max_clusters {
            return Err("XYZ clustering object limit exceeded".into());
        }
        let mut min = points[start];
        let mut max = min;
        for &index in &queue {
            let p = points[index];
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }
        result.objects.push(ObjectAabb {
            center: Vec3::new(
                (min.x + max.x) * 0.5,
                (min.y + max.y) * 0.5,
                (min.z + max.z) * 0.5,
            ),
            min,
            max,
            point_count: queue.len(),
            indices: queue,
        });
    }
    result.noise_indices.sort_unstable();
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn measured_building_and_tree_components_preserve_xyz_bounds_and_noise() {
        let points = vec![
            Vec3::new(0., 0., 1.),
            Vec3::new(0.4, 0., 1.),
            Vec3::new(0.8, 0., 1.4),
            Vec3::new(3., 1., 2.),
            Vec3::new(3., 1., 2.5),
            Vec3::new(3., 1., 3.),
            Vec3::new(20., 20., 4.),
        ];
        let result = cluster_objects(
            &points,
            &(0..points.len()).collect::<Vec<_>>(),
            &ObjectClusterConfig::default(),
        )
        .unwrap();
        assert_eq!(result.objects.len(), 2);
        assert_eq!(result.noise_indices, vec![6]);
        assert_eq!(result.objects[0].indices, vec![0, 1, 2]);
        assert_eq!(result.objects[0].min, Vec3::new(0., 0., 1.));
        assert_eq!(result.objects[0].max, Vec3::new(0.8, 0., 1.4));
        assert_eq!(result.objects[1].center, Vec3::new(3., 1., 2.5));
    }
    #[test]
    fn euclidean_connectivity_crosses_voxels_and_never_merges_vertical_gaps() {
        let p = vec![
            Vec3::new(-0.1, 0., 0.),
            Vec3::new(0.1, 0., 0.),
            Vec3::new(0.6, 0., 0.),
            Vec3::new(0., 0., 3.),
            Vec3::new(0.1, 0., 3.),
            Vec3::new(0.2, 0., 3.),
        ];
        let a = cluster_objects(&p, &[5, 1, 4, 0, 3, 2], &ObjectClusterConfig::default()).unwrap();
        let b = cluster_objects(&p, &[0, 1, 2, 3, 4, 5], &ObjectClusterConfig::default()).unwrap();
        assert_eq!(
            a.objects.iter().map(|o| &o.indices).collect::<Vec<_>>(),
            b.objects.iter().map(|o| &o.indices).collect::<Vec<_>>()
        );
        assert_eq!(a.objects.len(), 2);
        assert_eq!(a.objects[0].point_count, 3);
    }
    #[test]
    fn identical_dense_returns_use_linear_bounded_work() {
        let p = vec![Vec3::new(0., 0., 1.); 1_000];
        let result = cluster_objects(
            &p,
            &(0..p.len()).collect::<Vec<_>>(),
            &ObjectClusterConfig::default(),
        )
        .unwrap();
        assert_eq!(result.objects[0].point_count, 1_000);
        assert!(result.candidate_work < 30_000);
    }
    #[test]
    fn malformed_or_exhausted_input_cannot_return_partial_objects() {
        let p = vec![Vec3::default(); 4];
        let mut cfg = ObjectClusterConfig {
            max_candidate_work: 1,
            ..ObjectClusterConfig::default()
        };
        assert!(
            cluster_objects(&p, &[0, 1, 2, 3], &cfg)
                .unwrap_err()
                .contains("work")
        );
        cfg = ObjectClusterConfig::default();
        cfg.max_cluster_points = 3;
        assert!(
            cluster_objects(&p, &[0, 1, 2, 3], &cfg)
                .unwrap_err()
                .contains("component")
        );
        assert!(cluster_objects(&p, &[0, 0], &ObjectClusterConfig::default()).is_err());
        assert!(cluster_objects(&p, &[10], &ObjectClusterConfig::default()).is_err());
        assert!(
            cluster_objects(
                &[Vec3::new(0., 0., f64::INFINITY)],
                &[0],
                &ObjectClusterConfig::default()
            )
            .is_err()
        );
        cfg = ObjectClusterConfig::default();
        cfg.voxel_size_m = 0.01;
        assert!(cluster_objects(&p, &[0], &cfg).is_err());
    }
}
