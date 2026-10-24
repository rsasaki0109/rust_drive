//! Bounded selection of measured components; withheld hypotheses do not erase returns.
//! Support counts rank candidates, without semantic classes or calibrated confidence.
use crate::ObjectAabb;
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Debug)]
pub struct ComponentSelectionConfig {
    pub min_points_by_range: [usize; 3],
    pub range_breaks_m: [f64; 2],
    pub overlap_threshold: Option<f64>,
    pub max_center_height_gap_m: f64,
    pub radius_padding_m: f64,
    pub min_proxy_radius_m: f64,
    pub max_proxy_radius_m: f64,
    pub max_components: usize,
    pub max_return_indices: usize,
    pub max_candidate_work: usize,
    pub coordinate_bound_m: f64,
}
impl Default for ComponentSelectionConfig {
    fn default() -> Self {
        Self {
            min_points_by_range: [8, 5, 3],
            range_breaks_m: [15.0, 30.0],
            overlap_threshold: Some(0.5),
            max_center_height_gap_m: 0.5,
            radius_padding_m: 0.65,
            min_proxy_radius_m: 0.85,
            max_proxy_radius_m: 3.0,
            max_components: 10_000,
            max_return_indices: 500_000,
            max_candidate_work: 2_000_000,
            coordinate_bound_m: 10_000_000.0,
        }
    }
}
impl ComponentSelectionConfig {
    pub fn validate(&self) -> Result<(), String> {
        let numbers = [
            self.range_breaks_m[0],
            self.range_breaks_m[1],
            self.max_center_height_gap_m,
            self.radius_padding_m,
            self.min_proxy_radius_m,
            self.max_proxy_radius_m,
            self.coordinate_bound_m,
        ];
        if !numbers.into_iter().all(f64::is_finite)
            || self.range_breaks_m[0] <= 0.0
            || self.range_breaks_m[0] >= self.range_breaks_m[1]
            || self.range_breaks_m[1] > 200.0
            || self
                .min_points_by_range
                .iter()
                .any(|&n| n == 0 || n > self.max_return_indices)
            || self.min_points_by_range.windows(2).any(|w| w[0] < w[1])
            || !(0.0..=10.0).contains(&self.max_center_height_gap_m)
            || !(0.0..=5.0).contains(&self.radius_padding_m)
            || !(0.05..=20.0).contains(&self.min_proxy_radius_m)
            || self.max_proxy_radius_m < self.min_proxy_radius_m
            || self.max_proxy_radius_m > 20.0
            || self
                .overlap_threshold
                .is_some_and(|v| !v.is_finite() || v <= 0.0 || v > 1.0)
            || !(1..=100_000).contains(&self.max_components)
            || !(1..=2_000_000).contains(&self.max_return_indices)
            || !(1..=100_000_000).contains(&self.max_candidate_work)
            || !(1.0..=10_000_000.0).contains(&self.coordinate_bound_m)
        {
            return Err("invalid component-selection calibration or resource bounds".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ComponentSuppression {
    pub candidate: usize,
    pub representative: usize,
}
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ComponentSelection {
    /// Input ordinals, ascending. Original measured components remain unchanged.
    pub kept_indices: Vec<usize>,
    pub low_support_indices: Vec<usize>,
    pub suppressed: Vec<ComponentSuppression>,
    /// Tested retained competitors, including height-gate rejections.
    pub candidate_work: usize,
}
#[derive(Clone, Copy)]
struct Proxy {
    x: f64,
    y: f64,
    z: f64,
    r: f64,
}
fn proxy(object: &ObjectAabb, cfg: &ComponentSelectionConfig) -> Proxy {
    Proxy {
        x: (object.min.x + object.max.x) * 0.5,
        y: (object.min.y + object.max.y) * 0.5,
        z: (object.min.z + object.max.z) * 0.5,
        r: ((object.max.x - object.min.x).hypot(object.max.y - object.min.y) * 0.5
            + cfg.radius_padding_m)
            .clamp(cfg.min_proxy_radius_m, cfg.max_proxy_radius_m),
    }
}
fn circle_iou(a: Proxy, b: Proxy) -> f64 {
    let d = (a.x - b.x).hypot(a.y - b.y);
    if d >= a.r + b.r {
        return 0.0;
    }
    if d <= (a.r - b.r).abs() {
        return (a.r.min(b.r) / a.r.max(b.r)).powi(2);
    }
    let alpha = ((d * d + a.r * a.r - b.r * b.r) / (2.0 * d * a.r))
        .clamp(-1.0, 1.0)
        .acos();
    let beta = ((d * d + b.r * b.r - a.r * a.r) / (2.0 * d * b.r))
        .clamp(-1.0, 1.0)
        .acos();
    let product = (-d + a.r + b.r) * (d + a.r - b.r) * (d - a.r + b.r) * (d + a.r + b.r);
    let intersection = a.r * a.r * alpha + b.r * b.r * beta - 0.5 * product.max(0.0).sqrt();
    (intersection / (std::f64::consts::PI * (a.r * a.r + b.r * b.r) - intersection)).clamp(0.0, 1.0)
}
/// Select disjoint measured components, never modifying their bounds or point membership.
/// Indices must be sorted, globally disjoint and below max_return_indices. More supported
/// components rank first; the smallest source-return ordinal breaks support ties. NMS
/// additionally requires nearby measured height centres, to preserve vertical separation.
/// Resource/validation failure returns no partial selection. Withheld returns remain
/// evidence; absence from kept_indices is not free-space or semantic-noise proof.
pub fn select_components(
    objects: &[ObjectAabb],
    cfg: &ComponentSelectionConfig,
) -> Result<ComponentSelection, String> {
    cfg.validate()?;
    if objects.len() > cfg.max_components {
        return Err("component-selection input limit exceeded".into());
    }
    let mut used = BTreeSet::new();
    let mut proxies = Vec::with_capacity(objects.len());
    for object in objects {
        if object.point_count == 0
            || object.point_count != object.indices.len()
            || object.indices.windows(2).any(|w| w[0] >= w[1])
        {
            return Err(
                "component support must be nonempty sorted unique measured return ordinals".into(),
            );
        }
        for p in [object.min, object.max, object.center] {
            if !p.finite()
                || p.x.abs() > cfg.coordinate_bound_m
                || p.y.abs() > cfg.coordinate_bound_m
                || p.z.abs() > cfg.coordinate_bound_m
            {
                return Err(
                    "component bounds/centre are nonfinite or exceed coordinate limits".into(),
                );
            }
        }
        if object.min.x > object.max.x || object.min.y > object.max.y || object.min.z > object.max.z
        {
            return Err("component bounds are inverted".into());
        }
        let p = proxy(object, cfg);
        if (p.x - object.center.x).abs() > 1e-8
            || (p.y - object.center.y).abs() > 1e-8
            || (p.z - object.center.z).abs() > 1e-8
        {
            return Err("component centre differs from measured AABB midpoint".into());
        }
        for &index in &object.indices {
            if index >= cfg.max_return_indices || !used.insert(index) {
                return Err("component return ordinals overlap or exceed limits".into());
            }
        }
        proxies.push(p);
    }
    let mut order: Vec<_> = (0..objects.len()).collect();
    order.sort_unstable_by(|&a, &b| {
        objects[b]
            .point_count
            .cmp(&objects[a].point_count)
            .then(objects[a].indices[0].cmp(&objects[b].indices[0]))
            .then(a.cmp(&b))
    });
    let mut priority = vec![0; objects.len()];
    for (rank, &i) in order.iter().enumerate() {
        priority[i] = rank;
    }
    let cell_size = 2.0 * cfg.max_proxy_radius_m;
    let cell = |p: Proxy| {
        (
            (p.x / cell_size).floor() as i64,
            (p.y / cell_size).floor() as i64,
        )
    };
    let mut grid = BTreeMap::<(i64, i64), Vec<usize>>::new();
    let mut result = ComponentSelection::default();
    for i in order {
        let p = proxies[i];
        let range = p.x.hypot(p.y);
        let band = if range < cfg.range_breaks_m[0] {
            0
        } else if range < cfg.range_breaks_m[1] {
            1
        } else {
            2
        };
        if objects[i].point_count < cfg.min_points_by_range[band] {
            result.low_support_indices.push(i);
            continue;
        }
        let (x, y) = cell(p);
        let mut representative = None;
        if let Some(threshold) = cfg.overlap_threshold {
            let mut competitors = Vec::new();
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if let Some(bucket) = grid.get(&(x + dx, y + dy)) {
                        competitors.extend_from_slice(bucket);
                    }
                }
            }
            competitors.sort_unstable_by_key(|&j| priority[j]);
            for j in competitors {
                result.candidate_work += 1;
                if result.candidate_work > cfg.max_candidate_work {
                    return Err("component-selection comparison budget exceeded".into());
                }
                if (p.z - proxies[j].z).abs() <= cfg.max_center_height_gap_m
                    && circle_iou(p, proxies[j]) >= threshold
                {
                    representative = Some(j);
                    break;
                }
            }
        }
        if let Some(j) = representative {
            result.suppressed.push(ComponentSuppression {
                candidate: i,
                representative: j,
            });
        } else {
            result.kept_indices.push(i);
            grid.entry((x, y)).or_default().push(i);
        }
    }
    result.kept_indices.sort_unstable();
    result.low_support_indices.sort_unstable();
    result.suppressed.sort_unstable_by_key(|s| s.candidate);
    Ok(result)
}
#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_core::Vec3;
    fn object(x: f64, z: f64, n: usize, first: usize, half: f64) -> ObjectAabb {
        ObjectAabb {
            center: Vec3::new(x, 0., z),
            min: Vec3::new(x - half, -half, z - 0.1),
            max: Vec3::new(x + half, half, z + 0.1),
            point_count: n,
            indices: (first..first + n).collect(),
        }
    }
    #[test]
    fn range_support_keeps_sparse_far_components_and_retains_raw_evidence() {
        let raw = vec![
            object(5., 0., 6, 0, 0.1),
            object(20., 0., 5, 6, 0.1),
            object(35., 0., 3, 11, 0.1),
        ];
        let before: Vec<_> = raw.iter().map(|o| o.indices.clone()).collect();
        let result = select_components(&raw, &ComponentSelectionConfig::default()).unwrap();
        assert_eq!(result.kept_indices, vec![1, 2]);
        assert_eq!(result.low_support_indices, vec![0]);
        assert_eq!(
            before,
            raw.iter().map(|o| o.indices.clone()).collect::<Vec<_>>()
        );
    }
    #[test]
    fn higher_support_suppresses_overlapping_proxy_across_grid_boundary() {
        let raw = vec![object(5.99, 0., 8, 0, 0.1), object(6.01, 0., 12, 8, 0.1)];
        let r = select_components(&raw, &ComponentSelectionConfig::default()).unwrap();
        assert_eq!(r.kept_indices, vec![1]);
        assert_eq!(
            r.suppressed,
            vec![ComponentSuppression {
                candidate: 0,
                representative: 1
            }]
        );
    }
    #[test]
    fn vertical_separation_and_different_proxy_scales_remain_distinct() {
        let raw = vec![
            object(10., 0., 10, 0, 0.1),
            object(10., 3., 10, 10, 0.1),
            object(10., 0., 30, 20, 4.),
        ];
        let r = select_components(&raw, &ComponentSelectionConfig::default()).unwrap();
        assert_eq!(r.kept_indices, vec![0, 1, 2]);
        assert!(r.suppressed.is_empty());
    }
    #[test]
    fn support_tie_uses_source_ordinals_independent_of_input_permutation() {
        let raw = vec![object(-0.01, 0., 10, 30, 0.1), object(0.01, 0., 10, 0, 0.1)];
        let a = select_components(&raw, &ComponentSelectionConfig::default()).unwrap();
        let swapped = vec![raw[1].clone(), raw[0].clone()];
        let b = select_components(&swapped, &ComponentSelectionConfig::default()).unwrap();
        assert_eq!(
            raw[a.kept_indices[0]].indices,
            swapped[b.kept_indices[0]].indices
        );
    }
    #[test]
    fn disabled_suppression_with_minimum_three_keeps_every_raw_component() {
        let raw = vec![object(5., 0., 3, 0, 0.1), object(5., 0., 3, 3, 0.1)];
        let cfg = ComponentSelectionConfig {
            min_points_by_range: [3; 3],
            overlap_threshold: None,
            ..Default::default()
        };
        let r = select_components(&raw, &cfg).unwrap();
        assert_eq!(r.kept_indices, vec![0, 1]);
        assert_eq!(r.candidate_work, 0);
    }
    #[test]
    fn malformed_support_geometry_and_exhausted_work_fail_without_partial_result() {
        let a = object(5., 0., 10, 0, 0.1);
        let mut bad = a.clone();
        bad.point_count = 9;
        assert!(select_components(&[bad], &Default::default()).is_err());
        assert!(select_components(&[a.clone(), a.clone()], &Default::default()).is_err());
        let mut bad = a.clone();
        bad.min.z = f64::NAN;
        assert!(select_components(&[bad], &Default::default()).is_err());
        let raw = vec![a, object(7., 0., 10, 10, 0.1), object(9., 0., 10, 20, 0.1)];
        let cfg = ComponentSelectionConfig {
            max_candidate_work: 1,
            ..Default::default()
        };
        assert!(select_components(&raw, &cfg).is_err());
        let cfg = ComponentSelectionConfig {
            overlap_threshold: Some(f64::NAN),
            ..Default::default()
        };
        assert!(select_components(&[], &cfg).is_err());
    }
    #[test]
    fn spatially_separated_components_avoid_all_pairs_comparisons() {
        let raw: Vec<_> = (0..500)
            .map(|i| object(i as f64 * 10., 0., 8, i * 8, 0.1))
            .collect();
        let r = select_components(&raw, &Default::default()).unwrap();
        assert_eq!(r.kept_indices.len(), raw.len());
        assert!(r.candidate_work < raw.len() * 3);
    }
}
