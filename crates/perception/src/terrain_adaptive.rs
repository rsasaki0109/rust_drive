//! Optional, bounded density-aware terrain estimation in an explicit Z-up frame.
//!
//! Sparse neighborhoods are supported by measured XY distance rather than grid
//! adjacency. Elevated returns are screened against a slope-limited lower
//! envelope, then deterministic robust planes classify each original XYZ point.
//! Unknown geometry remains non-ground. Broad isolated roofs and ground-like
//! shallow obstacles are intrinsically ambiguous; confidence is support, not a
//! calibrated probability. This does not replace the frozen PMF baseline.
use rustdriving_core::Vec3;
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct AdaptiveTerrainConfig {
    pub cell_size_m: f64,
    pub support_radius_m: f64,
    pub max_slope: f64,
    pub max_residual_m: f64,
    pub min_support_neighbors: usize,
    pub max_support_neighbors: usize,
    pub min_supported_cells: usize,
    pub min_supported_fraction: f64,
    pub max_points: usize,
    pub max_cells: usize,
    pub max_candidate_work: usize,
    pub coordinate_bound_m: f64,
}
impl Default for AdaptiveTerrainConfig {
    fn default() -> Self {
        Self {
            cell_size_m: 1.0,
            support_radius_m: 16.0,
            max_slope: 0.45,
            max_residual_m: 0.18,
            min_support_neighbors: 6,
            max_support_neighbors: 32,
            min_supported_cells: 12,
            min_supported_fraction: 0.5,
            max_points: 500_000,
            max_cells: 250_000,
            max_candidate_work: 100_000_000,
            coordinate_bound_m: 10_000_000.0,
        }
    }
}
impl AdaptiveTerrainConfig {
    pub fn validate(&self) -> Result<(), String> {
        if !self.cell_size_m.is_finite()
            || !(0.05..=20.0).contains(&self.cell_size_m)
            || !self.support_radius_m.is_finite()
            || !(self.cell_size_m..=64.0 * self.cell_size_m).contains(&self.support_radius_m)
            || !self.max_slope.is_finite()
            || !(0.0..=1.0).contains(&self.max_slope)
            || !self.max_residual_m.is_finite()
            || !(0.01..=0.5).contains(&self.max_residual_m)
            || !(3..=64).contains(&self.min_support_neighbors)
            || !(self.min_support_neighbors..=64).contains(&self.max_support_neighbors)
            || self.min_supported_cells == 0
            || self.min_supported_cells > self.max_cells
            || !self.min_supported_fraction.is_finite()
            || !(0.0..=1.0).contains(&self.min_supported_fraction)
            || !(1..=2_000_000).contains(&self.max_points)
            || !(1..=500_000).contains(&self.max_cells)
            || !(1..=500_000_000).contains(&self.max_candidate_work)
            || !self.coordinate_bound_m.is_finite()
            || !(1.0..=10_000_000.0).contains(&self.coordinate_bound_m)
        {
            return Err("invalid adaptive terrain calibration/resource bounds".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Default)]
pub struct AdaptiveTerrainDiagnostics {
    pub occupied_cells: usize,
    pub supported_cells: usize,
    pub rejected_low_cells: usize,
    pub rejected_elevated_cells: usize,
    pub candidate_work: usize,
    pub supported_fraction: f64,
    pub confident: bool,
}
#[derive(Clone, Debug, Default)]
pub struct AdaptiveGroundClassification {
    pub ground_indices: Vec<usize>,
    pub non_ground_indices: Vec<usize>,
    pub diagnostics: AdaptiveTerrainDiagnostics,
}
type Cell = (i64, i64);
fn cell(p: Vec3, size: f64) -> Cell {
    ((p.x / size).floor() as i64, (p.y / size).floor() as i64)
}
fn charge(work: &mut usize, amount: usize, cfg: &AdaptiveTerrainConfig) -> Result<(), String> {
    *work = work
        .checked_add(amount)
        .filter(|n| *n <= cfg.max_candidate_work)
        .ok_or("adaptive terrain candidate-work limit exceeded")?;
    Ok(())
}
fn neighbors(
    key: Cell,
    origin: Vec3,
    field: &BTreeMap<Cell, Vec3>,
    radius_m: f64,
    cfg: &AdaptiveTerrainConfig,
    work: &mut usize,
) -> Result<Vec<(f64, Cell, Vec3)>, String> {
    let radius = (radius_m / cfg.cell_size_m).ceil() as i64;
    let mut result = Vec::new();
    for x in key.0 - radius..=key.0 + radius {
        charge(work, 1, cfg)?;
        for (&other, &p) in field.range((x, key.1 - radius)..=(x, key.1 + radius)) {
            charge(work, 1, cfg)?;
            if other != key {
                let distance = (p.x - origin.x).hypot(p.y - origin.y);
                if distance <= radius_m {
                    result.push((distance, other, p));
                }
            }
        }
    }
    result.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    Ok(result)
}

fn local_neighbors(
    key: Cell,
    origin: Vec3,
    field: &BTreeMap<Cell, Vec3>,
    cfg: &AdaptiveTerrainConfig,
    work: &mut usize,
) -> Result<Vec<(f64, Cell, Vec3)>, String> {
    let mut radius = cfg.cell_size_m.min(cfg.support_radius_m);
    loop {
        let mut near = neighbors(key, origin, field, radius, cfg, work)?;
        if near.len() >= cfg.max_support_neighbors || radius >= cfg.support_radius_m {
            near.truncate(cfg.max_support_neighbors);
            return Ok(near);
        }
        radius = (radius * 2.0).min(cfg.support_radius_m);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn slope(spacing: f64) -> Vec<Vec3> {
        (0..13)
            .flat_map(|x| {
                (0..13).map(move |y| {
                    let (x, y) = (x as f64 * spacing, y as f64 * spacing);
                    Vec3::new(x, y, 0.18 * x - 0.07 * y)
                })
            })
            .collect()
    }
    #[test]
    fn measured_distance_support_handles_different_sampling_densities() {
        for spacing in [0.5, 1.0, 3.0] {
            let points = slope(spacing);
            let result =
                classify_ground_adaptive(&points, &AdaptiveTerrainConfig::default()).unwrap();
            assert!(result.diagnostics.confident, "spacing {spacing}");
            assert!(
                result.ground_indices.len() > points.len() * 95 / 100,
                "spacing {spacing}: {:?}",
                result.diagnostics
            );
        }
    }
    #[test]
    fn slope_roof_tree_low_outlier_and_low_obstacle_are_separated() {
        let mut points = slope(1.0);
        let mut roof = Vec::new();
        for (i, p) in points.iter_mut().enumerate() {
            if (4.0..=8.0).contains(&p.x) && (4.0..=8.0).contains(&p.y) {
                p.z += 4.0;
                roof.push(i);
            }
        }
        let start = points.len();
        points.extend([
            Vec3::new(2.1, 2.1, 0.18 * 2.1 - 0.07 * 2.1 + 0.3),
            Vec3::new(1.1, 1.1, 3.0),
            Vec3::new(1.2, 1.2, 5.0),
            Vec3::new(2.0, 2.0, -10.0),
            Vec3::new(40.0, 40.0, -10.0),
        ]);
        let result = classify_ground_adaptive(&points, &AdaptiveTerrainConfig::default()).unwrap();
        assert!(result.diagnostics.confident);
        assert!(roof.iter().all(|i| result.non_ground_indices.contains(i)));
        assert!((start..points.len()).all(|i| result.non_ground_indices.contains(&i)));
        assert!(result.ground_indices.len() >= 140);
        assert!(
            result.ground_indices.contains(&28),
            "good sample sharing a cell with a low outlier survives"
        );
        assert_eq!(
            result.ground_indices.len() + result.non_ground_indices.len(),
            points.len()
        );
    }
    #[test]
    fn unknown_collinear_sparse_and_steep_geometry_stays_non_ground() {
        for points in [
            vec![Vec3::new(0.0, 0.0, 0.0)],
            (0..20).map(|x| Vec3::new(x as f64, 0.0, 0.0)).collect(),
            slope(1.0)
                .iter()
                .map(|p| Vec3::new(p.x, p.y, p.x * 0.8))
                .collect(),
        ] {
            let result =
                classify_ground_adaptive(&points, &AdaptiveTerrainConfig::default()).unwrap();
            assert!(!result.diagnostics.confident);
            assert!(result.ground_indices.is_empty());
        }
    }
    #[test]
    fn ordering_and_large_integer_translation_preserve_labels() {
        let mut points = slope(1.0);
        points.push(Vec3::new(2.1, 2.1, 3.0));
        let cfg = AdaptiveTerrainConfig::default();
        let result = classify_ground_adaptive(&points, &cfg).unwrap();
        let transformed: Vec<_> = points
            .iter()
            .rev()
            .map(|p| Vec3::new(p.x + 500_000.0, p.y + 5_000_000.0, p.z + 100.0))
            .collect();
        let changed = classify_ground_adaptive(&transformed, &cfg).unwrap();
        let mut mapped: Vec<_> = changed
            .ground_indices
            .iter()
            .map(|i| points.len() - 1 - i)
            .collect();
        mapped.sort_unstable();
        assert_eq!(result.ground_indices, mapped);
    }
    #[test]
    fn noisy_ground_survives_but_empty_space_is_not_extrapolated() {
        let mut points = slope(1.0);
        for (i, p) in points.iter_mut().enumerate() {
            p.z += ((i * 37 % 101) as f64 / 100.0 - 0.5) * 0.06;
        }
        let known = points.len();
        // This point fits the terrain's infinite plane, but no local density
        // evidence supports the eight-meter gap beyond the measured patch.
        points.push(Vec3::new(20.0, 6.0, 0.18 * 20.0 - 0.07 * 6.0));
        let result = classify_ground_adaptive(&points, &AdaptiveTerrainConfig::default()).unwrap();
        assert!(result.diagnostics.confident);
        assert!(result.ground_indices.len() > known * 95 / 100);
        assert!(result.non_ground_indices.contains(&known));
    }
    #[test]
    fn resource_and_invalid_data_rejections_return_no_partial_output() {
        let points = slope(1.0);
        for cfg in [
            AdaptiveTerrainConfig {
                max_candidate_work: 1,
                ..AdaptiveTerrainConfig::default()
            },
            AdaptiveTerrainConfig {
                max_points: 10,
                ..AdaptiveTerrainConfig::default()
            },
            AdaptiveTerrainConfig {
                max_cells: 12,
                ..AdaptiveTerrainConfig::default()
            },
            AdaptiveTerrainConfig {
                support_radius_m: f64::INFINITY,
                ..AdaptiveTerrainConfig::default()
            },
        ] {
            assert!(classify_ground_adaptive(&points, &cfg).is_err());
        }
        assert!(
            classify_ground_adaptive(
                &[Vec3::new(f64::NAN, 0.0, 0.0)],
                &AdaptiveTerrainConfig::default()
            )
            .is_err()
        );
        assert!(
            classify_ground_adaptive(
                &[Vec3::new(1e8, 0.0, 0.0)],
                &AdaptiveTerrainConfig::default()
            )
            .is_err()
        );
        let empty = classify_ground_adaptive(&[], &AdaptiveTerrainConfig::default()).unwrap();
        assert!(!empty.diagnostics.confident);
    }
}
#[derive(Clone, Copy)]
struct Plane {
    a: f64,
    b: f64,
    c: f64,
}
impl Plane {
    fn height(self, p: Vec3, origin: Vec3) -> f64 {
        self.a * (p.x - origin.x) + self.b * (p.y - origin.y) + self.c
    }
}
fn triple_plane(p: Vec3, q: Vec3, r: Vec3, origin: Vec3) -> Option<Plane> {
    let (dx, dy, dz) = (q.x - p.x, q.y - p.y, q.z - p.z);
    let (ex, ey, ez) = (r.x - p.x, r.y - p.y, r.z - p.z);
    let determinant = dx * ey - dy * ex;
    // Reject a nearly collinear triangle relative to its own side lengths.
    if determinant.abs() < 0.01 * dx.hypot(dy) * ex.hypot(ey) {
        return None;
    }
    let a = (dz * ey - dy * ez) / determinant;
    let b = (dx * ez - dz * ex) / determinant;
    let c = p.z - a * (p.x - origin.x) - b * (p.y - origin.y);
    (a.is_finite() && b.is_finite() && c.is_finite()).then_some(Plane { a, b, c })
}
fn refit(points: &[Vec3], origin: Vec3) -> Option<Plane> {
    let n = points.len() as f64;
    let (mut mx, mut my, mut mz) = (0.0, 0.0, 0.0);
    for p in points {
        mx += (p.x - origin.x) / n;
        my += (p.y - origin.y) / n;
        mz += p.z / n;
    }
    let (mut xx, mut xy, mut yy, mut xz, mut yz) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for p in points {
        let (x, y, z) = (p.x - origin.x - mx, p.y - origin.y - my, p.z - mz);
        xx += x * x;
        xy += x * y;
        yy += y * y;
        xz += x * z;
        yz += y * z;
    }
    let determinant = xx * yy - xy * xy;
    if determinant <= 1e-4 * (xx + yy).powi(2) || determinant <= 1e-12 {
        return None;
    }
    let a = (xz * yy - yz * xy) / determinant;
    let b = (yz * xx - xz * xy) / determinant;
    Some(Plane {
        a,
        b,
        c: mz - a * mx - b * my,
    })
}
fn robust_plane(
    points: &[Vec3],
    origin: Vec3,
    cfg: &AdaptiveTerrainConfig,
    work: &mut usize,
) -> Result<Option<Plane>, String> {
    if points.len() < cfg.min_support_neighbors {
        return Ok(None);
    }
    // Six deterministic distance-ranked anchors bound the hypothesis budget.
    let anchors: Vec<_> = (0..points.len().min(6))
        .map(|i| i * (points.len() - 1) / (points.len().min(6) - 1))
        .collect();
    let mut best: Option<(usize, f64, Plane)> = None;
    for a in 0..anchors.len() {
        for b in a + 1..anchors.len() {
            for c in b + 1..anchors.len() {
                charge(work, 1, cfg)?;
                let Some(plane) = triple_plane(
                    points[anchors[a]],
                    points[anchors[b]],
                    points[anchors[c]],
                    origin,
                ) else {
                    continue;
                };
                if plane.a.hypot(plane.b) > cfg.max_slope {
                    continue;
                }
                charge(work, points.len(), cfg)?;
                let count = points
                    .iter()
                    .filter(|p| (p.z - plane.height(**p, origin)).abs() <= cfg.max_residual_m)
                    .count();
                if count >= cfg.min_support_neighbors
                    && best.is_none_or(|(n, z, _)| count > n || (count == n && plane.c < z))
                {
                    best = Some((count, plane.c, plane));
                }
            }
        }
    }
    let Some((_, _, plane)) = best else {
        return Ok(None);
    };
    let inliers: Vec<_> = points
        .iter()
        .copied()
        .filter(|p| (p.z - plane.height(*p, origin)).abs() <= cfg.max_residual_m)
        .collect();
    let refined = refit(&inliers, origin).filter(|p| p.a.hypot(p.b) <= cfg.max_slope);
    Ok(refined)
}

fn supported_gap(
    near: &[(f64, Cell, Vec3)],
    cfg: &AdaptiveTerrainConfig,
    work: &mut usize,
) -> Result<bool, String> {
    if near.len() < cfg.min_support_neighbors {
        return Ok(false);
    }
    // Compare the focal gap with measured neighbor-to-neighbor spacing. This
    // allows sparse sampling while refusing distant extrapolation across holes.
    let mut spacing = Vec::new();
    for (i, (_, _, p)) in near.iter().take(8).enumerate() {
        let mut closest = f64::INFINITY;
        for (j, (_, _, q)) in near.iter().enumerate() {
            charge(work, 1, cfg)?;
            if i != j {
                closest = closest.min((p.x - q.x).hypot(p.y - q.y));
            }
        }
        spacing.push(closest);
    }
    spacing.sort_unstable_by(f64::total_cmp);
    Ok(near[0].0 <= 2.5 * spacing[spacing.len() / 2])
}

/// Fit measured local terrain; returned indices partition the input in order.
/// An unconfident result must not authorize terrain removal in a driving system.
/// Resource exhaustion returns an error, never a partial classification.
pub fn classify_ground_adaptive(
    points: &[Vec3],
    cfg: &AdaptiveTerrainConfig,
) -> Result<AdaptiveGroundClassification, String> {
    cfg.validate()?;
    if points.len() > cfg.max_points {
        return Err("adaptive terrain point limit exceeded".into());
    }
    let mut buckets = BTreeMap::<Cell, Vec<usize>>::new();
    for (i, &p) in points.iter().enumerate() {
        if !p.finite() || p.x.abs().max(p.y.abs()).max(p.z.abs()) > cfg.coordinate_bound_m {
            return Err("adaptive terrain point outside finite coordinate bound".into());
        }
        buckets.entry(cell(p, cfg.cell_size_m)).or_default().push(i);
        if buckets.len() > cfg.max_cells {
            return Err("adaptive terrain occupied-cell limit exceeded".into());
        }
    }
    for indices in buckets.values_mut() {
        indices.sort_unstable_by(|&a, &b| {
            points[a]
                .z
                .total_cmp(&points[b].z)
                .then(points[a].x.total_cmp(&points[b].x))
                .then(points[a].y.total_cmp(&points[b].y))
        });
    }
    let original: BTreeMap<_, _> = buckets
        .iter()
        .map(|(&key, values)| (key, points[values[0]]))
        .collect();
    let mut field = original.clone();
    let mut work = 0;
    let mut rejected_low_cells = 0;
    // A single anomalously low return must not lower the whole terrain surface.
    for (&key, &p) in &original {
        let near = local_neighbors(key, p, &original, cfg, &mut work)?;
        if near.len() >= cfg.min_support_neighbors {
            let mut bounds: Vec<_> = near
                .iter()
                .map(|(d, _, q)| q.z - cfg.max_slope * d - cfg.max_residual_m)
                .collect();
            bounds.sort_unstable_by(f64::total_cmp);
            let lower = bounds[bounds.len() / 2];
            if p.z < lower {
                rejected_low_cells += 1;
                if let Some(&index) = buckets[&key].iter().find(|&&i| points[i].z >= lower) {
                    field.insert(key, points[index]);
                } else {
                    field.remove(&key);
                }
            }
        }
    }
    let screened = field.clone();
    let mut rejected_elevated_cells = 0;
    for (&key, &p) in &screened {
        let near = neighbors(key, p, &screened, cfg.support_radius_m, cfg, &mut work)?;
        let lower = near
            .iter()
            .map(|(d, _, q)| q.z + cfg.max_slope * d)
            .fold(p.z, f64::min);
        if p.z > lower + cfg.max_residual_m {
            field.remove(&key);
            rejected_elevated_cells += 1;
        }
    }
    let mut surfaces = BTreeMap::new();
    for (&key, &origin) in &original {
        let near = local_neighbors(key, origin, &field, cfg, &mut work)?;
        if !supported_gap(&near, cfg, &mut work)? {
            continue;
        }
        let support: Vec<_> = near
            .iter()
            .take(cfg.max_support_neighbors)
            .map(|(_, _, p)| *p)
            .collect();
        if let Some(plane) = robust_plane(&support, origin, cfg, &mut work)? {
            surfaces.insert(key, (origin, plane));
        }
    }
    let supported_cells = surfaces.len();
    let supported_fraction = if original.is_empty() {
        0.0
    } else {
        supported_cells as f64 / original.len() as f64
    };
    let mut result = AdaptiveGroundClassification {
        diagnostics: AdaptiveTerrainDiagnostics {
            occupied_cells: original.len(),
            supported_cells,
            rejected_low_cells,
            rejected_elevated_cells,
            candidate_work: work,
            supported_fraction,
            confident: supported_cells >= cfg.min_supported_cells
                && supported_fraction >= cfg.min_supported_fraction,
        },
        ..AdaptiveGroundClassification::default()
    };
    for (i, &p) in points.iter().enumerate() {
        if surfaces
            .get(&cell(p, cfg.cell_size_m))
            .is_some_and(|&(origin, plane)| {
                (p.z - plane.height(p, origin)).abs() <= cfg.max_residual_m
            })
        {
            result.ground_indices.push(i);
        } else {
            result.non_ground_indices.push(i);
        }
    }
    Ok(result)
}
