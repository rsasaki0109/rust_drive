//! Bounded progressive morphological terrain classification in an explicit XYZ frame.
//!
//! Z must point up. XY coordinates may be translated map coordinates; no vehicle
//! pose, semantic label or simulator ground role is consulted. Sparse cells with
//! inadequate neighboring terrain support are conservatively kept as non-ground.
//! As with height-based PMF generally, an isolated broad roof can be ambiguous.
use rustdriving_core::Vec3;
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct TerrainConfig {
    pub cell_size_m: f64,
    pub initial_height_m: f64,
    pub max_height_m: f64,
    /// Height threshold growth per meter of morphological window enlargement.
    pub max_slope: f64,
    pub window_radii_cells: Vec<usize>,
    pub min_support_neighbors: usize,
    pub min_supported_cells: usize,
    pub min_supported_fraction: f64,
    pub max_points: usize,
    pub max_cells: usize,
    pub max_candidate_work: usize,
    pub coordinate_bound_m: f64,
}
impl Default for TerrainConfig {
    fn default() -> Self {
        // Fixed PMF baseline parameters; dataset labels must not select these.
        Self {
            cell_size_m: 1.0,
            initial_height_m: 0.15,
            max_height_m: 2.5,
            max_slope: 0.3,
            window_radii_cells: vec![1, 2, 4, 8, 16],
            min_support_neighbors: 3,
            min_supported_cells: 12,
            min_supported_fraction: 0.5,
            max_points: 500_000,
            max_cells: 250_000,
            max_candidate_work: 100_000_000,
            coordinate_bound_m: 10_000_000.0,
        }
    }
}
impl TerrainConfig {
    pub fn validate(&self) -> Result<(), String> {
        if !self.cell_size_m.is_finite()
            || !(0.05..=20.0).contains(&self.cell_size_m)
            || !self.initial_height_m.is_finite()
            || !(0.0..=5.0).contains(&self.initial_height_m)
            || !self.max_height_m.is_finite()
            || self.max_height_m < self.initial_height_m
            || self.max_height_m > 10.0
            || !self.max_slope.is_finite()
            || !(0.0..=2.0).contains(&self.max_slope)
            || self.window_radii_cells.is_empty()
            || self.window_radii_cells.len() > 8
            || self
                .window_radii_cells
                .iter()
                .any(|r| !(1..=64).contains(r))
            || self.window_radii_cells.windows(2).any(|r| r[0] >= r[1])
            || !(1..=8).contains(&self.min_support_neighbors)
            || self.min_supported_cells == 0
            || !self.min_supported_fraction.is_finite()
            || !(0.0..=1.0).contains(&self.min_supported_fraction)
            || !(1..=2_000_000).contains(&self.max_points)
            || !(1..=500_000).contains(&self.max_cells)
            || !(1..=500_000_000).contains(&self.max_candidate_work)
            || !self.coordinate_bound_m.is_finite()
            || !(1.0..=10_000_000.0).contains(&self.coordinate_bound_m)
        {
            return Err("invalid or unsupported terrain calibration/resource bounds".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Default)]
pub struct TerrainDiagnostics {
    pub occupied_cells: usize,
    pub candidate_ground_cells: usize,
    pub supported_cells: usize,
    pub rejected_cells: usize,
    pub candidate_work: usize,
    pub supported_fraction: f64,
    /// Local geometric support, not a calibrated probability or semantic truth.
    pub confident: bool,
}
#[derive(Clone, Debug, Default)]
pub struct GroundClassification {
    /// Original input indices, in ascending order.
    pub ground_indices: Vec<usize>,
    pub non_ground_indices: Vec<usize>,
    pub diagnostics: TerrainDiagnostics,
}

type Cell = (i64, i64);
fn cell(point: Vec3, size: f64) -> Cell {
    (
        (point.x / size).floor() as i64,
        (point.y / size).floor() as i64,
    )
}
fn pass(
    field: &BTreeMap<Cell, f64>,
    radius: usize,
    axis: usize,
    minimum: bool,
    work: &mut usize,
) -> BTreeMap<Cell, f64> {
    field
        .keys()
        .map(|&(x, y)| {
            let mut height = if minimum {
                f64::INFINITY
            } else {
                f64::NEG_INFINITY
            };
            for offset in -(radius as i64)..=radius as i64 {
                *work += 1;
                let neighbor = if axis == 0 {
                    (x + offset, y)
                } else {
                    (x, y + offset)
                };
                if let Some(&value) = field.get(&neighbor) {
                    height = if minimum {
                        height.min(value)
                    } else {
                        height.max(value)
                    };
                }
            }
            ((x, y), height)
        })
        .collect()
}

/// Classify XYZ samples using a sparse minimum-height grid and progressive openings.
///
/// Cell minima exceeding a slope-aware opened surface are rejected permanently.
/// Point residuals are checked against accepted original cell minima. Neighbor
/// support rejects isolated low returns rather than treating every minimum as road.
/// Returned indices partition the input; an unconfident result must not authorize
/// removing terrain in a driving pipeline. Resource overflow fails without output.
pub fn classify_ground(
    points: &[Vec3],
    config: &TerrainConfig,
) -> Result<GroundClassification, String> {
    config.validate()?;
    if points.len() > config.max_points {
        return Err("terrain point limit exceeded".into());
    }
    let mut field = BTreeMap::<Cell, f64>::new();
    for &point in points {
        if !point.finite()
            || point.x.abs() > config.coordinate_bound_m
            || point.y.abs() > config.coordinate_bound_m
            || point.z.abs() > config.coordinate_bound_m
        {
            return Err("terrain point is non-finite or outside the coordinate bound".into());
        }
        let index = cell(point, config.cell_size_m);
        field
            .entry(index)
            .and_modify(|z| *z = z.min(point.z))
            .or_insert(point.z);
        if field.len() > config.max_cells {
            return Err("terrain occupied-cell limit exceeded".into());
        }
    }
    let work_per_cell = 16
        + config
            .window_radii_cells
            .iter()
            .map(|r| 4 * (2 * r + 1))
            .sum::<usize>();
    if field
        .len()
        .checked_mul(work_per_cell)
        .is_none_or(|work| work > config.max_candidate_work)
    {
        return Err("terrain candidate-work limit exceeded".into());
    }
    let original = field.clone();
    let mut candidates: BTreeMap<_, _> = field.keys().map(|&key| (key, true)).collect();
    let mut work = 0;
    // Isolated low minima must not erode a supported terrain patch across
    // larger windows. Use measured neighbors before the morphological passes.
    for (&(x, y), &height) in &original {
        let mut count = 0;
        for dx in -1_i64..=1 {
            for dy in -1_i64..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                work += 1;
                if original.get(&(x + dx, y + dy)).is_some_and(|neighbor| {
                    (height - neighbor).abs()
                        <= config.initial_height_m
                            + config.max_slope * config.cell_size_m * (dx as f64).hypot(dy as f64)
                }) {
                    count += 1;
                }
            }
        }
        if count < config.min_support_neighbors {
            *candidates.get_mut(&(x, y)).unwrap() = false;
            field.remove(&(x, y));
        }
    }
    let mut previous_radius = 0;
    for &radius in &config.window_radii_cells {
        let erosion = pass(
            &pass(&field, radius, 0, true, &mut work),
            radius,
            1,
            true,
            &mut work,
        );
        let opened = pass(
            &pass(&erosion, radius, 0, false, &mut work),
            radius,
            1,
            false,
            &mut work,
        );
        let threshold = (config.initial_height_m
            + config.max_slope * config.cell_size_m * 2.0 * (radius - previous_radius) as f64)
            .min(config.max_height_m);
        for (key, height) in &mut field {
            if *height - opened[key] > threshold {
                *candidates.get_mut(key).unwrap() = false;
            }
            *height = height.min(opened[key]);
        }
        previous_radius = radius;
    }
    let mut supported = BTreeMap::new();
    for (&(x, y), &is_candidate) in &candidates {
        let mut count = 0;
        for dx in -1_i64..=1 {
            for dy in -1_i64..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                work += 1;
                let key = (x + dx, y + dy);
                if candidates.get(&key) == Some(&true)
                    && (original[&(x, y)] - original[&key]).abs()
                        <= config.initial_height_m
                            + config.max_slope * config.cell_size_m * (dx as f64).hypot(dy as f64)
                {
                    count += 1;
                }
            }
        }
        supported.insert(
            (x, y),
            is_candidate && count >= config.min_support_neighbors,
        );
    }
    let supported_cells = supported.values().filter(|&&yes| yes).count();
    let candidate_ground_cells = candidates.values().filter(|&&yes| yes).count();
    let supported_fraction = if original.is_empty() {
        0.0
    } else {
        supported_cells as f64 / original.len() as f64
    };
    let point_threshold =
        config.initial_height_m + config.max_slope * config.cell_size_m * 2.0_f64.sqrt();
    let mut result = GroundClassification {
        diagnostics: TerrainDiagnostics {
            occupied_cells: original.len(),
            candidate_ground_cells,
            supported_cells,
            rejected_cells: original.len() - candidate_ground_cells,
            candidate_work: work,
            supported_fraction,
            confident: supported_cells >= config.min_supported_cells
                && supported_fraction >= config.min_supported_fraction,
        },
        ..GroundClassification::default()
    };
    for (index, &point) in points.iter().enumerate() {
        let key = cell(point, config.cell_size_m);
        // An opening lowers the outer boundary of a sloped finite patch. It
        // decides whether a cell minimum survives, but must not replace an
        // accepted measured minimum with an extrapolated lower boundary height.
        if supported[&key] && (point.z - original[&key]).abs() <= point_threshold {
            result.ground_indices.push(index);
        } else {
            result.non_ground_indices.push(index);
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn terrain() -> Vec<Vec3> {
        (0..25)
            .flat_map(|x| {
                (0..25)
                    .map(move |y| Vec3::new(x as f64, y as f64, 0.15 * x as f64 + 0.05 * y as f64))
            })
            .collect()
    }
    #[test]
    fn sloped_terrain_building_tree_and_low_outlier_are_separated() {
        let mut points = terrain();
        let terrain_len = points.len();
        // Elevated roof removes the ground sample in its occupied cells.
        for point in &mut points {
            if (9.0..=13.0).contains(&point.x) && (9.0..=13.0).contains(&point.y) {
                point.z += 5.0;
            }
        }
        let roof: Vec<_> = points
            .iter()
            .enumerate()
            .filter(|(_, p)| (9.0..=13.0).contains(&p.x) && (9.0..=13.0).contains(&p.y))
            .map(|(i, _)| i)
            .collect();
        points.extend([
            Vec3::new(6.0, 6.0, 4.0),
            Vec3::new(6.1, 6.1, 6.0),
            Vec3::new(100.0, 100.0, -20.0),
            Vec3::new(3.0, 3.0, -20.0),
        ]);
        let result = classify_ground(&points, &TerrainConfig::default()).unwrap();
        assert!(result.diagnostics.confident);
        assert!(roof.iter().all(|i| result.non_ground_indices.contains(i)));
        assert!((terrain_len..points.len()).all(|i| result.non_ground_indices.contains(&i)));
        assert!(result.ground_indices.len() > 500);
        assert_eq!(
            result.ground_indices.len() + result.non_ground_indices.len(),
            points.len()
        );
    }
    #[test]
    fn translation_and_input_order_preserve_geometric_labels() {
        let points = terrain();
        let cfg = TerrainConfig::default();
        let original = classify_ground(&points, &cfg).unwrap();
        let shifted: Vec<_> = points
            .iter()
            .rev()
            .map(|p| Vec3::new(p.x + 500_000.0, p.y + 5_000_000.0, p.z + 100.0))
            .collect();
        let changed = classify_ground(&shifted, &cfg).unwrap();
        let mut back: Vec<_> = changed
            .ground_indices
            .iter()
            .map(|i| points.len() - 1 - i)
            .collect();
        back.sort_unstable();
        assert_eq!(original.ground_indices, back);
    }
    #[test]
    fn sparse_and_steep_support_do_not_authorize_ground_removal() {
        let sparse = classify_ground(&[Vec3::new(0., 0., 0.)], &TerrainConfig::default()).unwrap();
        assert!(!sparse.diagnostics.confident);
        assert_eq!(sparse.non_ground_indices, vec![0]);
        let steep: Vec<_> = (0..10)
            .flat_map(|x| {
                (0..10).map(move |y| Vec3::new(x as f64, y as f64, x as f64 * 2. + y as f64 * 2.))
            })
            .collect();
        let result = classify_ground(&steep, &TerrainConfig::default()).unwrap();
        assert!(!result.diagnostics.confident);
    }
    #[test]
    fn invalid_and_resource_exhaustion_fail_without_partial_classification() {
        let points = terrain();
        let mut cfg = TerrainConfig {
            max_candidate_work: 1,
            ..TerrainConfig::default()
        };
        assert!(classify_ground(&points, &cfg).unwrap_err().contains("work"));
        cfg = TerrainConfig::default();
        cfg.max_cells = 2;
        assert!(classify_ground(&points, &cfg).unwrap_err().contains("cell"));
        cfg = TerrainConfig::default();
        cfg.max_points = 2;
        assert!(
            classify_ground(&points, &cfg)
                .unwrap_err()
                .contains("point")
        );
        assert!(
            classify_ground(&[Vec3::new(f64::NAN, 0., 0.)], &TerrainConfig::default()).is_err()
        );
        assert!(classify_ground(&[Vec3::new(1e8, 0., 0.)], &TerrainConfig::default()).is_err());
        cfg = TerrainConfig::default();
        cfg.window_radii_cells = vec![2, 1];
        assert!(classify_ground(&points, &cfg).is_err());
    }
}
