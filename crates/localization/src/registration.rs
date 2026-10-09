//! Bounded deterministic point-to-point registration against a supplied fixed 2D
//! map. Correspondences and pose come only from the supplied points; no simulator
//! labels or true poses are accepted. This is local ICP, not global localization.
use rustdrive_core::{Pose, Vec2, wrap_angle};
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct RegistrationConfig {
    pub max_scan_points: usize,
    pub max_map_points: usize,
    pub max_iterations: usize,
    /// Global point-distance comparison budget, including ambiguity probes.
    pub max_neighbor_checks: usize,
    pub max_correspondence_m: f64,
    /// Fraction of nearest-neighbor pairs removed by descending residual.
    pub trim_fraction: f64,
    pub min_pairs: usize,
    pub min_overlap: f64,
    pub max_translation_jump_m: f64,
    pub max_yaw_jump_rad: f64,
    pub max_rms_m: f64,
    pub min_geometry_ratio: f64,
    pub max_condition_number: f64,
    pub max_position_variance_m2: f64,
    pub max_yaw_variance_rad2: f64,
    pub translation_tolerance_m: f64,
    pub yaw_tolerance_rad: f64,
    pub ambiguity_translation_probe_m: f64,
    pub ambiguity_yaw_probe_rad: f64,
    pub ambiguity_rms_ratio: f64,
}
impl Default for RegistrationConfig {
    fn default() -> Self {
        Self {
            max_scan_points: 20_000,
            max_map_points: 100_000,
            max_iterations: 30,
            max_neighbor_checks: 20_000_000,
            max_correspondence_m: 1.5,
            trim_fraction: 0.2,
            min_pairs: 20,
            min_overlap: 0.5,
            max_translation_jump_m: 2.0,
            max_yaw_jump_rad: 0.35,
            max_rms_m: 0.2,
            min_geometry_ratio: 0.01,
            max_condition_number: 10_000.0,
            max_position_variance_m2: 0.25,
            max_yaw_variance_rad2: 0.01,
            translation_tolerance_m: 0.0001,
            yaw_tolerance_rad: 0.0001,
            ambiguity_translation_probe_m: 0.5,
            ambiguity_yaw_probe_rad: 0.1,
            ambiguity_rms_ratio: 1.05,
        }
    }
}
#[derive(Clone, Debug)]
pub struct RegistrationConditioning {
    /// Minor / major eigenvalue of retained map-point planar scatter.
    pub geometry_ratio: f64,
    /// Centered SE(2) information condition number, meters / radians units.
    pub condition_number: f64,
    /// Number of map-point distance comparisons used, including all probes.
    pub neighbor_checks: usize,
    pub ambiguity_probes: usize,
}
#[derive(Clone, Debug)]
pub struct RegistrationResult {
    pub pose: Pose,
    pub rms_m: f64,
    pub inlier_fraction: f64,
    pub inlier_count: usize,
    pub iterations: usize,
    pub converged: bool,
    /// Local independent-point least-squares estimate in x/y/yaw units. It does
    /// not account for association errors, correlated scans or map uncertainty.
    pub covariance: [[f64; 3]; 3],
    pub conditioning: RegistrationConditioning,
}

impl RegistrationConfig {
    pub fn validate(&self) -> Result<(), String> {
        let positive =
            |value: f64, maximum: f64| value.is_finite() && value > 0.0 && value <= maximum;
        if self.max_scan_points == 0
            || self.max_scan_points > 20_000
            || self.max_map_points == 0
            || self.max_map_points > 100_000
            || self.max_iterations == 0
            || self.max_iterations > 50
            || self.max_neighbor_checks == 0
            || self.max_neighbor_checks > 100_000_000
            || self.min_pairs < 3
            || self.min_pairs > self.max_scan_points
            || self.min_pairs > self.max_map_points
            || !positive(self.max_correspondence_m, 10.0)
            || !self.trim_fraction.is_finite()
            || !(0.0..=0.8).contains(&self.trim_fraction)
            || !positive(self.min_overlap, 1.0)
            || !positive(self.max_translation_jump_m, 50.0)
            || !positive(self.max_yaw_jump_rad, std::f64::consts::PI)
            || !positive(self.max_rms_m, self.max_correspondence_m)
            || !positive(self.min_geometry_ratio, 1.0)
            || !self.max_condition_number.is_finite()
            || !(1.0..=1e12).contains(&self.max_condition_number)
            || !positive(self.max_position_variance_m2, 1e6)
            || !positive(self.max_yaw_variance_rad2, std::f64::consts::PI.powi(2))
            || !positive(self.translation_tolerance_m, self.max_correspondence_m)
            || !positive(self.yaw_tolerance_rad, self.max_yaw_jump_rad)
            || !positive(
                self.ambiguity_translation_probe_m,
                self.max_translation_jump_m,
            )
            || !positive(self.ambiguity_yaw_probe_rad, self.max_yaw_jump_rad)
            || !self.ambiguity_rms_ratio.is_finite()
            || !(1.0..=2.0).contains(&self.ambiguity_rms_ratio)
        {
            return Err("invalid bounded registration configuration".into());
        }
        Ok(())
    }
}

struct Grid<'a> {
    map: &'a [Vec2],
    cells: BTreeMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
}
impl<'a> Grid<'a> {
    fn new(map: &'a [Vec2], cell_size: f64) -> Result<Self, String> {
        let mut grid = Self {
            map,
            cells: BTreeMap::new(),
            cell_size,
        };
        for (index, &point) in map.iter().enumerate() {
            grid.cells.entry(grid.key(point)?).or_default().push(index);
        }
        Ok(grid)
    }
    fn key(&self, point: Vec2) -> Result<(i64, i64), String> {
        let x = (point.x / self.cell_size).floor();
        let y = (point.y / self.cell_size).floor();
        // Leave room for the nine-cell query without integer overflow. A margin
        // of 2048 also avoids f64 rounding at i64's upper representable boundary.
        let bound = i64::MAX as f64 - 2048.0;
        if !point.finite()
            || !x.is_finite()
            || !y.is_finite()
            || x.abs() >= bound
            || y.abs() >= bound
        {
            return Err("registration coordinate exceeds spatial-grid numeric bounds".into());
        }
        Ok((x as i64, y as i64))
    }
    fn nearest(&self, point: Vec2, budget: &mut Budget) -> Result<Option<(usize, f64)>, String> {
        let (x, y) = self.key(point)?;
        let mut best: Option<(usize, f64)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(indices) = self.cells.get(&(x + dx, y + dy)) {
                    for &index in indices {
                        budget.charge()?;
                        let distance = point.distance(self.map[index]);
                        if distance <= self.cell_size
                            && best.is_none_or(|(old_index, old_distance)| {
                                distance < old_distance
                                    || (distance == old_distance && index < old_index)
                            })
                        {
                            best = Some((index, distance));
                        }
                    }
                }
            }
        }
        Ok(best)
    }
}
struct Budget {
    checks: usize,
    maximum: usize,
}
impl Budget {
    fn charge(&mut self) -> Result<(), String> {
        if self.checks >= self.maximum {
            return Err("registration comparison budget exhausted".into());
        }
        self.checks += 1;
        Ok(())
    }
}
#[derive(Clone, Copy)]
struct Pair {
    scan_index: usize,
    map_index: usize,
    distance: f64,
}
fn correspondences(
    scan: &[Vec2],
    grid: &Grid<'_>,
    pose: Pose,
    config: &RegistrationConfig,
    budget: &mut Budget,
) -> Result<Vec<Pair>, String> {
    // Keep only the closest scan point for each map point. Repeated matches to a
    // single corner cannot inflate overlap or the covariance's sample count.
    let mut unique: BTreeMap<usize, Pair> = BTreeMap::new();
    for (scan_index, &point) in scan.iter().enumerate() {
        if let Some((map_index, distance)) = grid.nearest(pose.to_world(point), budget)? {
            let candidate = Pair {
                scan_index,
                map_index,
                distance,
            };
            match unique.get_mut(&map_index) {
                Some(previous)
                    if distance < previous.distance
                        || (distance == previous.distance && scan_index < previous.scan_index) =>
                {
                    *previous = candidate
                }
                None => {
                    unique.insert(map_index, candidate);
                }
                _ => {}
            }
        }
    }
    let mut pairs: Vec<_> = unique.into_values().collect();
    pairs.sort_by(|a, b| {
        a.distance
            .total_cmp(&b.distance)
            .then(a.scan_index.cmp(&b.scan_index))
            .then(a.map_index.cmp(&b.map_index))
    });
    let retain = (pairs.len() as f64 * (1.0 - config.trim_fraction)).floor() as usize;
    pairs.truncate(retain);
    if pairs.len() < config.min_pairs
        || (pairs.len() as f64 / scan.len() as f64) < config.min_overlap
    {
        return Err("insufficient unique trimmed registration overlap".into());
    }
    Ok(pairs)
}
fn mean(points: impl Iterator<Item = Vec2> + Clone) -> Result<Vec2, String> {
    let anchor = points
        .clone()
        .next()
        .ok_or("empty registration point set")?;
    let (sum, count) = points.fold((Vec2::default(), 0usize), |(sum, count), point| {
        (sum.plus(point.minus(anchor)), count + 1)
    });
    let result = anchor.plus(sum.scaled(1.0 / count as f64));
    if !result.finite() {
        return Err("registration centroid arithmetic is nonfinite".into());
    }
    Ok(result)
}
fn procrustes(scan: &[Vec2], map: &[Vec2], pairs: &[Pair]) -> Result<Pose, String> {
    let source_mean = mean(pairs.iter().map(|pair| scan[pair.scan_index]))?;
    let target_mean = mean(pairs.iter().map(|pair| map[pair.map_index]))?;
    let mut dot = 0.0;
    let mut cross = 0.0;
    for pair in pairs {
        let a = scan[pair.scan_index].minus(source_mean);
        let b = map[pair.map_index].minus(target_mean);
        dot += a.x * b.x + a.y * b.y;
        cross += a.x * b.y - a.y * b.x;
    }
    if !dot.is_finite() || !cross.is_finite() || dot.hypot(cross) <= 1e-12 {
        return Err("degenerate registration rotation fit".into());
    }
    let yaw = cross.atan2(dot);
    let position = target_mean.minus(source_mean.rotated(yaw));
    if !position.finite() {
        return Err("nonfinite registration translation fit".into());
    }
    Ok(Pose { position, yaw })
}
struct Fit {
    result: RegistrationResult,
}
fn solve(
    scan: &[Vec2],
    grid: &Grid<'_>,
    initial: Pose,
    config: &RegistrationConfig,
    budget: &mut Budget,
) -> Result<Fit, String> {
    let mut pose = initial;
    let mut iterations = 0;
    let mut converged = false;
    for i in 0..config.max_iterations {
        let pairs = correspondences(scan, grid, pose, config, budget)?;
        let next = procrustes(scan, grid.map, &pairs)?;
        iterations = i + 1;
        converged = next.position.distance(pose.position) <= config.translation_tolerance_m
            && wrap_angle(next.yaw - pose.yaw).abs() <= config.yaw_tolerance_rad;
        pose = next;
        if converged {
            break;
        }
    }
    if !converged {
        return Err("registration did not converge within iteration bound".into());
    }
    let pairs = correspondences(scan, grid, pose, config, budget)?;
    let rms_m =
        (pairs.iter().map(|p| p.distance * p.distance).sum::<f64>() / pairs.len() as f64).sqrt();
    if !rms_m.is_finite() || rms_m > config.max_rms_m {
        return Err("registration residual exceeds accepted bound".into());
    }
    let (covariance, geometry_ratio, condition_number) =
        conditioning(scan, grid.map, &pairs, pose, rms_m, config)?;
    Ok(Fit {
        result: RegistrationResult {
            pose,
            rms_m,
            inlier_fraction: pairs.len() as f64 / scan.len() as f64,
            inlier_count: pairs.len(),
            iterations,
            converged,
            covariance,
            conditioning: RegistrationConditioning {
                geometry_ratio,
                condition_number,
                neighbor_checks: budget.checks,
                ambiguity_probes: 0,
            },
        },
    })
}

fn conditioning(
    scan: &[Vec2],
    map: &[Vec2],
    pairs: &[Pair],
    pose: Pose,
    rms: f64,
    config: &RegistrationConfig,
) -> Result<([[f64; 3]; 3], f64, f64), String> {
    let source_mean = mean(pairs.iter().map(|p| scan[p.scan_index]))?;
    let target_mean = mean(pairs.iter().map(|p| map[p.map_index]))?;
    let mut xx = 0.0;
    let mut xy = 0.0;
    let mut yy = 0.0;
    let mut source_scatter = 0.0;
    for pair in pairs {
        let point = map[pair.map_index].minus(target_mean);
        xx += point.x * point.x;
        xy += point.x * point.y;
        yy += point.y * point.y;
        let source = scan[pair.scan_index].minus(source_mean);
        source_scatter += source.x * source.x + source.y * source.y;
    }
    let major = ((xx + yy) + (xx - yy).hypot(2.0 * xy)) / 2.0;
    // det / major is more stable than trace-minus-discriminant for a thin cloud.
    let minor = (xx * yy - xy * xy).max(0.0) / major;
    let geometry_ratio = minor / major;
    let n = pairs.len() as f64;
    let condition_number = n.max(source_scatter) / n.min(source_scatter);
    if !geometry_ratio.is_finite()
        || geometry_ratio < config.min_geometry_ratio
        || !condition_number.is_finite()
        || condition_number > config.max_condition_number
    {
        return Err("collinear or poorly conditioned registration geometry".into());
    }
    let noise_variance = (rms * rms * n / (2.0 * n - 3.0)).max(1e-6);
    let yaw_variance = noise_variance / source_scatter;
    let translation_variance = noise_variance / n;
    let rotated_mean = source_mean.rotated(pose.yaw);
    let g = Vec2::new(-rotated_mean.y, rotated_mean.x);
    let covariance = [
        [
            translation_variance + g.x * g.x * yaw_variance,
            g.x * g.y * yaw_variance,
            -g.x * yaw_variance,
        ],
        [
            g.x * g.y * yaw_variance,
            translation_variance + g.y * g.y * yaw_variance,
            -g.y * yaw_variance,
        ],
        [-g.x * yaw_variance, -g.y * yaw_variance, yaw_variance],
    ];
    if covariance.iter().flatten().any(|v| !v.is_finite())
        || covariance[0][0].max(covariance[1][1]) > config.max_position_variance_m2
        || yaw_variance > config.max_yaw_variance_rad2
    {
        return Err("registration covariance exceeds accepted bound".into());
    }
    Ok((covariance, geometry_ratio, condition_number))
}
fn within_jump(pose: Pose, initial: Pose, config: &RegistrationConfig) -> bool {
    pose.position.distance(initial.position) <= config.max_translation_jump_m
        && wrap_angle(pose.yaw - initial.yaw).abs() <= config.max_yaw_jump_rad
}

/// Match body/sensor-frame scan points to a supplied world-frame map. Inputs and
/// all nearest-neighbor comparison work are bounded. A successful result passed
/// convergence, unique overlap, residual, geometry, jump, covariance and six local
/// competing-fit checks. Probes do not establish global uniqueness of a pose.
pub fn match_scan(
    scan: &[Vec2],
    map: &[Vec2],
    initial: Pose,
    config: &RegistrationConfig,
) -> Result<RegistrationResult, String> {
    config.validate()?;
    if scan.len() < config.min_pairs
        || scan.len() > config.max_scan_points
        || map.len() < config.min_pairs
        || map.len() > config.max_map_points
        || scan.iter().chain(map).any(|p| !p.finite())
        || !initial.position.finite()
        || !initial.yaw.is_finite()
    {
        return Err("invalid or oversized registration point input".into());
    }
    let grid = Grid::new(map, config.max_correspondence_m)?;
    let mut budget = Budget {
        checks: 0,
        maximum: config.max_neighbor_checks,
    };
    let mut fits = Vec::with_capacity(7);
    let (probe_center, main_error) = match solve(scan, &grid, initial, config, &mut budget) {
        Ok(fit) if within_jump(fit.result.pose, initial, config) => {
            let center = fit.result.pose;
            fits.push(fit.result);
            (center, "no accepted bounded registration fit".to_string())
        }
        Ok(_) => (
            initial,
            "registration displacement exceeds accepted jump bound".to_string(),
        ),
        Err(error) if error.contains("budget exhausted") => return Err(error),
        Err(error) => (initial, error),
    };
    let probes = [
        (Vec2::new(config.ambiguity_translation_probe_m, 0.0), 0.0),
        (Vec2::new(-config.ambiguity_translation_probe_m, 0.0), 0.0),
        (Vec2::new(0.0, config.ambiguity_translation_probe_m), 0.0),
        (Vec2::new(0.0, -config.ambiguity_translation_probe_m), 0.0),
        (Vec2::default(), config.ambiguity_yaw_probe_rad),
        (Vec2::default(), -config.ambiguity_yaw_probe_rad),
    ];
    for (offset, yaw_delta) in probes {
        let seed = Pose {
            position: probe_center.position.plus(offset),
            yaw: wrap_angle(probe_center.yaw + yaw_delta),
        };
        let candidate = match solve(scan, &grid, seed, config, &mut budget) {
            Ok(candidate) => candidate.result,
            Err(error) if error.contains("budget exhausted") => return Err(error),
            Err(_) => continue,
        };
        if within_jump(candidate.pose, initial, config) {
            fits.push(candidate);
        }
    }
    if fits.is_empty() {
        return Err(main_error);
    }
    // Stable sorting retains deterministic seed order for equal residuals.
    fits.sort_by(|a, b| a.rms_m.total_cmp(&b.rms_m));
    let mut result = fits.remove(0);
    for candidate in fits {
        let distinct = candidate.pose.position.distance(result.pose.position)
            > config.ambiguity_translation_probe_m / 2.0
            || wrap_angle(candidate.pose.yaw - result.pose.yaw).abs()
                > config.ambiguity_yaw_probe_rad / 2.0;
        // The floor reflects the covariance model's 1 mm noise floor; exact
        // floating-point matches must not hide an equally good periodic fit.
        if distinct && candidate.rms_m <= result.rms_m.max(0.001) * config.ambiguity_rms_ratio {
            return Err("ambiguous registration: distinct comparable local fit".into());
        }
    }
    result.conditioning.ambiguity_probes = 6;
    result.conditioning.neighbor_checks = budget.checks;
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn asymmetric() -> Vec<Vec2> {
        (0..120)
            .map(|i| {
                let t = i as f64;
                Vec2::new(
                    (1.73 * t).sin() * 3.0 + (0.21 * t).cos() * 0.5,
                    (0.77 * t).cos() * 2.0 + (0.13 * t).sin() * 0.3,
                )
            })
            .collect()
    }
    fn pose(x: f64, y: f64, yaw: f64) -> Pose {
        Pose {
            position: Vec2::new(x, y),
            yaw,
        }
    }
    #[test]
    fn asymmetric_cloud_recovers_known_pose_with_noise_and_far_outliers() {
        let clean = asymmetric();
        let truth = pose(0.4, -0.3, 0.09);
        let map: Vec<_> = clean.iter().map(|p| truth.to_world(*p)).collect();
        let mut scan: Vec<_> = clean
            .iter()
            .enumerate()
            .map(|(i, p)| {
                p.plus(Vec2::new(
                    (i as f64).sin() * 0.002,
                    (i as f64 * 0.7).cos() * 0.0015,
                ))
            })
            .collect();
        scan.extend((0..20).map(|i| Vec2::new(30.0 + i as f64, -40.0)));
        let result = match_scan(
            &scan,
            &map,
            pose(0.35, -0.27, 0.07),
            &RegistrationConfig::default(),
        )
        .unwrap();
        assert!(result.pose.position.distance(truth.position) < 0.002);
        assert!(wrap_angle(result.pose.yaw - truth.yaw).abs() < 0.001);
        assert!(result.rms_m < 0.004);
        assert!(result.inlier_fraction > 0.6 && result.inlier_fraction < 0.8);
        assert!(result.converged && result.iterations <= 30);
        assert_eq!(result.conditioning.ambiguity_probes, 6);
        assert!(result.conditioning.neighbor_checks <= 20_000_000);
        for i in 0..3 {
            assert!(result.covariance[i][i] > 0.0);
            for j in 0..3 {
                assert_eq!(result.covariance[i][j], result.covariance[j][i]);
            }
        }
    }
    #[test]
    fn centered_fit_is_stable_with_large_world_coordinates() {
        let scan = asymmetric();
        let truth = pose(1e9 + 0.4, -1e9 - 0.3, 0.09);
        let map: Vec<_> = scan.iter().map(|p| truth.to_world(*p)).collect();
        let initial = pose(1e9 + 0.35, -1e9 - 0.27, 0.07);
        let a = match_scan(&scan, &map, initial, &RegistrationConfig::default()).unwrap();
        let b = match_scan(&scan, &map, initial, &RegistrationConfig::default()).unwrap();
        assert!(a.pose.position.distance(truth.position) < 1e-5);
        assert!(wrap_angle(a.pose.yaw - truth.yaw).abs() < 1e-6);
        assert_eq!(a.pose.position, b.pose.position);
        assert_eq!(a.pose.yaw.to_bits(), b.pose.yaw.to_bits());
        assert_eq!(a.rms_m.to_bits(), b.rms_m.to_bits());
        assert_eq!(
            a.conditioning.neighbor_checks,
            b.conditioning.neighbor_checks
        );
    }
    #[test]
    fn bounded_multistart_recovers_a_better_fit_instead_of_calling_it_ambiguous() {
        let scan = asymmetric();
        let truth = pose(0.4, -0.25, 0.06);
        let map: Vec<_> = scan.iter().map(|p| truth.to_world(*p)).collect();
        let result =
            match_scan(&scan, &map, Pose::default(), &RegistrationConfig::default()).unwrap();
        assert!(result.pose.position.distance(truth.position) < 1e-8);
        assert!(wrap_angle(result.pose.yaw - truth.yaw).abs() < 1e-8);
        assert_eq!(result.conditioning.ambiguity_probes, 6);
    }
    #[test]
    fn collinear_and_repeated_map_geometry_cannot_establish_a_pose() {
        let line: Vec<_> = (0..80).map(|i| Vec2::new(i as f64 * 0.1, 0.0)).collect();
        assert!(
            match_scan(
                &line,
                &line,
                Pose::default(),
                &RegistrationConfig::default()
            )
            .unwrap_err()
            .contains("conditioned")
        );
        let identical = vec![Vec2::default(); 80];
        assert!(
            match_scan(
                &identical,
                &identical,
                Pose::default(),
                &RegistrationConfig::default()
            )
            .is_err()
        );
    }
    #[test]
    fn distinct_comparable_repeated_cloud_alignment_is_rejected_as_ambiguous() {
        let scan = asymmetric();
        let mut map = scan.clone();
        map.extend(scan.iter().map(|p| p.plus(Vec2::new(1.0, 0.0))));
        let cfg = RegistrationConfig {
            ambiguity_translation_probe_m: 1.0,
            ..RegistrationConfig::default()
        };
        assert!(
            match_scan(&scan, &map, Pose::default(), &cfg)
                .unwrap_err()
                .contains("ambiguous")
        );
    }
    #[test]
    fn invalid_oversized_wrong_initial_and_excessive_jump_inputs_are_rejected() {
        let scan = asymmetric();
        let map = scan.clone();
        let cfg = RegistrationConfig::default();
        assert!(match_scan(&scan, &map, pose(50.0, 50.0, 0.0), &cfg).is_err());
        assert!(match_scan(&[Vec2::new(f64::NAN, 0.0)], &map, Pose::default(), &cfg).is_err());
        assert!(match_scan(&vec![Vec2::default(); 20_001], &map, Pose::default(), &cfg).is_err());
        assert!(
            match_scan(
                &scan,
                &vec![Vec2::default(); 100_001],
                Pose::default(),
                &cfg
            )
            .is_err()
        );
        assert!(
            match_scan(
                &scan,
                &map,
                Pose::default(),
                &RegistrationConfig {
                    max_iterations: 51,
                    ..cfg.clone()
                }
            )
            .is_err()
        );
        let bound = RegistrationConfig {
            max_translation_jump_m: 0.01,
            ambiguity_translation_probe_m: 0.01,
            ..cfg.clone()
        };
        assert!(
            match_scan(&scan, &map, pose(0.04, 0.0, 0.0), &bound)
                .unwrap_err()
                .contains("jump")
        );
        let covariance = RegistrationConfig {
            max_position_variance_m2: 1e-20,
            ..cfg.clone()
        };
        assert!(
            match_scan(&scan, &map, Pose::default(), &covariance)
                .unwrap_err()
                .contains("covariance")
        );
        let budget = RegistrationConfig {
            max_neighbor_checks: 100,
            ..cfg
        };
        assert!(
            match_scan(&scan, &map, Pose::default(), &budget)
                .unwrap_err()
                .contains("budget")
        );
    }
    #[test]
    fn nearest_neighbor_ties_are_stable_and_do_not_escape_the_radius() {
        let map = [Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0)];
        let grid = Grid::new(&map, 1.5).unwrap();
        let mut budget = Budget {
            checks: 0,
            maximum: 100,
        };
        assert_eq!(
            grid.nearest(Vec2::default(), &mut budget).unwrap(),
            Some((0, 1.0))
        );
        assert_eq!(
            grid.nearest(Vec2::new(10.0, 10.0), &mut budget).unwrap(),
            None
        );
    }
}
