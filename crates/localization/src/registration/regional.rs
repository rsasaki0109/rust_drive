//! Finite regional hypotheses sharing one spatial index and comparison budget.
//! A sampled unique fit does not establish global map uniqueness.
use super::*;

#[derive(Clone, Copy, Debug)]
pub struct RegionalSearchConfig {
    pub position_half_extent_m: f64,
    pub yaw_half_extent_rad: f64,
    pub max_neighbor_checks: usize,
}
impl Default for RegionalSearchConfig {
    fn default() -> Self {
        Self {
            position_half_extent_m: 4.,
            yaw_half_extent_rad: 0.3,
            max_neighbor_checks: 20_000_000,
        }
    }
}
impl RegionalSearchConfig {
    pub fn validate(&self) -> Result<(), String> {
        if !self.position_half_extent_m.is_finite()
            || !(0.1..=10.).contains(&self.position_half_extent_m)
            || !self.yaw_half_extent_rad.is_finite()
            || !(0.01..=std::f64::consts::PI).contains(&self.yaw_half_extent_rad)
            || self.max_neighbor_checks == 0
            || self.max_neighbor_checks > 100_000_000
        {
            return Err("invalid bounded regional search configuration".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct RegionalCandidate {
    pub seed_index: usize,
    pub fit: RegistrationResult,
}
#[derive(Clone, Debug, Default)]
pub struct RegionalSearchReport {
    /// Nonempty on any incomplete, ambiguous or unsupported search. A best
    /// proposal in such a report is diagnostic evidence and cannot be admitted.
    pub error: Option<String>,
    pub best: Option<RegistrationResult>,
    pub candidates: Vec<RegionalCandidate>,
    pub seeds_tested: usize,
    pub neighbor_checks: usize,
    pub correlation_comparisons: usize,
    pub hypotheses: Vec<RegionalSeed>,
    pub scan_points_used: usize,
}

#[derive(Clone, Debug)]
pub struct RegionalSeed {
    pub pose: Pose,
    pub votes: usize,
}

pub fn search_region(
    scan: &[Vec2],
    map: &[Vec2],
    center: Pose,
    registration: &RegistrationConfig,
    region: &RegionalSearchConfig,
) -> RegionalSearchReport {
    let mut report = RegionalSearchReport::default();
    let validation = registration.validate().and_then(|()| region.validate());
    if let Err(error) = validation {
        report.error = Some(error);
        return report;
    }
    if scan.len() < registration.min_pairs
        || scan.len() > registration.max_scan_points
        || map.len() < registration.min_pairs
        || map.len() > registration.max_map_points
        || scan.iter().chain(map).any(|p| !p.finite())
        || !center.position.finite()
        || !center.yaw.is_finite()
    {
        report.error = Some("invalid or oversized regional point input".into());
        return report;
    }
    // Fix one evenly distributed raw subset before any hypothesis is scored.
    // This bounds dense-scan voting without selecting points by residual.
    let count = scan.len().min(128);
    let selected: Vec<_> = (0..count).map(|i| scan[i * scan.len() / count]).collect();
    let scan = selected.as_slice();
    report.scan_points_used = count;
    let mut grid = match Grid::new(map, registration.max_correspondence_m) {
        Ok(grid) => grid,
        Err(error) => {
            report.error = Some(error);
            return report;
        }
    };
    grid.regional_tree = Some(KdTree::new(map));
    let mut budget = Budget {
        checks: 0,
        maximum: region.max_neighbor_checks,
    };
    // Vote every scan/map pair at five yaw angles, then refine the best25
    // translation cells per angle. The complete coarse pass and at most125 ICP
    // hypotheses share one budget. A coarse prefix cannot authorize a pose.
    let cell = region.position_half_extent_m / 8.;
    for yaw_index in -2..=2 {
        let yaw = wrap_angle(center.yaw + yaw_index as f64 * region.yaw_half_extent_rad / 2.);
        let mut votes: BTreeMap<(i64, i64), usize> = BTreeMap::new();
        for point in scan {
            let rotated = point.rotated(yaw);
            for target in map {
                if let Err(error) = budget.charge() {
                    report.error = Some(error);
                    report.neighbor_checks = budget.checks;
                    report.correlation_comparisons = budget.checks;
                    return report;
                }
                let delta = target.minus(rotated).minus(center.position);
                if delta.x.abs() <= region.position_half_extent_m
                    && delta.y.abs() <= region.position_half_extent_m
                {
                    let key = (
                        (delta.x / cell).round() as i64,
                        (delta.y / cell).round() as i64,
                    );
                    *votes.entry(key).or_default() += 1;
                }
            }
        }
        let mut ordered: Vec<_> = votes.into_iter().collect();
        ordered.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        for ((x, y), count) in ordered.into_iter().take(25) {
            report.hypotheses.push(RegionalSeed {
                pose: Pose {
                    position: center
                        .position
                        .plus(Vec2::new(x as f64 * cell, y as f64 * cell)),
                    yaw,
                },
                votes: count,
            });
        }
    }
    report.correlation_comparisons = budget.checks;
    for (seed_index, seed) in report.hypotheses.iter().enumerate() {
        report.seeds_tested += 1;
        let fit = match solve(scan, &grid, seed.pose, registration, &mut budget) {
            Ok(fit) => fit.result,
            Err(error) if error.contains("budget exhausted") => {
                report.error = Some(error);
                report.neighbor_checks = budget.checks;
                return report;
            }
            Err(_) => continue,
        };
        if within_jump(fit.pose, seed.pose, registration)
            && (fit.pose.position.x - center.position.x).abs()
                <= region.position_half_extent_m + 1e-9
            && (fit.pose.position.y - center.position.y).abs()
                <= region.position_half_extent_m + 1e-9
            && wrap_angle(fit.pose.yaw - center.yaw).abs() <= region.yaw_half_extent_rad + 1e-9
        {
            report
                .candidates
                .push(RegionalCandidate { seed_index, fit });
        }
    }
    report.neighbor_checks = budget.checks;
    report
        .candidates
        .sort_by(|a, b| a.fit.rms_m.total_cmp(&b.fit.rms_m));
    let Some(first) = report.candidates.first() else {
        report.error = Some("no accepted bounded regional fit".into());
        return report;
    };
    let mut best = first.fit.clone();
    for candidate in &report.candidates[1..] {
        let distinct = candidate.fit.pose.position.distance(best.pose.position) > 0.25
            || wrap_angle(candidate.fit.pose.yaw - best.pose.yaw).abs() > 0.05;
        if distinct && candidate.fit.rms_m <= best.rms_m.max(0.02) * 1.1 {
            report.error = Some("ambiguous regional registration: comparable distinct fits".into());
            break;
        }
    }
    best.conditioning.neighbor_checks = budget.checks;
    best.conditioning.ambiguity_probes = report.seeds_tested - 1;
    report.best = Some(best);
    report
}

// This exact nearest-neighbor index is used only by regional search. Ordinary
// local registration retains its original grid traversal and diagnostic bytes.
pub(super) struct KdTree {
    nodes: Vec<KdNode>,
    root: Option<usize>,
}
struct KdNode {
    map_index: usize,
    axis: usize,
    left: Option<usize>,
    right: Option<usize>,
}
impl KdTree {
    fn coordinate(point: Vec2, axis: usize) -> f64 {
        if axis == 0 { point.x } else { point.y }
    }
    fn new(map: &[Vec2]) -> Self {
        fn build(
            map: &[Vec2],
            indices: &mut [usize],
            depth: usize,
            nodes: &mut Vec<KdNode>,
        ) -> Option<usize> {
            if indices.is_empty() {
                return None;
            }
            let middle = indices.len() / 2;
            let axis = depth % 2;
            indices.select_nth_unstable_by(middle, |a, b| {
                KdTree::coordinate(map[*a], axis)
                    .total_cmp(&KdTree::coordinate(map[*b], axis))
                    .then(a.cmp(b))
            });
            let index = nodes.len();
            nodes.push(KdNode {
                map_index: indices[middle],
                axis,
                left: None,
                right: None,
            });
            let (left, right) = indices.split_at_mut(middle);
            let a = build(map, left, depth + 1, nodes);
            let b = build(map, &mut right[1..], depth + 1, nodes);
            nodes[index].left = a;
            nodes[index].right = b;
            Some(index)
        }
        let mut nodes = Vec::with_capacity(map.len());
        let root = build(map, &mut (0..map.len()).collect::<Vec<_>>(), 0, &mut nodes);
        Self { nodes, root }
    }
    pub(super) fn nearest(
        &self,
        map: &[Vec2],
        point: Vec2,
        radius: f64,
        budget: &mut Budget,
    ) -> Result<Option<(usize, f64)>, String> {
        let mut best = None;
        self.visit(self.root, map, point, radius, budget, &mut best)?;
        Ok(best)
    }
    fn visit(
        &self,
        index: Option<usize>,
        map: &[Vec2],
        point: Vec2,
        radius: f64,
        budget: &mut Budget,
        best: &mut Option<(usize, f64)>,
    ) -> Result<(), String> {
        let Some(index) = index else {
            return Ok(());
        };
        let node = &self.nodes[index];
        budget.charge()?;
        let distance = point.distance(map[node.map_index]);
        if distance <= radius
            && best.is_none_or(|(old, d)| distance < d || (distance == d && node.map_index < old))
        {
            *best = Some((node.map_index, distance));
        }
        let delta =
            Self::coordinate(point, node.axis) - Self::coordinate(map[node.map_index], node.axis);
        let (near, far) = if delta < 0. {
            (node.left, node.right)
        } else {
            (node.right, node.left)
        };
        self.visit(near, map, point, radius, budget, best)?;
        // Only discard a half-space whose axial distance exceeds the best
        // Euclidean bound. Equality stays searchable for stable index ties.
        if delta.abs() <= best.map_or(radius, |(_, d)| d) {
            self.visit(far, map, point, radius, budget, best)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn cloud() -> Vec<Vec2> {
        (0..80)
            .map(|i| {
                let t = i as f64;
                Vec2::new(
                    4. * (t * 0.73).sin() + (t * 0.13).cos(),
                    2. * (t * 0.29).cos(),
                )
            })
            .collect()
    }
    #[test]
    fn a_displaced_scan_is_recovered_without_using_its_true_pose_as_a_seed() {
        let map = cloud();
        let actual = Pose {
            position: Vec2::new(3.2, -0.4),
            yaw: 0.12,
        };
        let scan = map
            .iter()
            .map(|p| p.minus(actual.position).rotated(-actual.yaw))
            .collect::<Vec<_>>();
        let result = search_region(
            &scan,
            &map,
            Pose::default(),
            &RegistrationConfig::default(),
            &RegionalSearchConfig::default(),
        );
        assert!(result.error.is_none(), "{:?}", result.error);
        assert_eq!(result.seeds_tested, 125);
        assert!(result.neighbor_checks <= 20_000_000);
        let best = result.best.unwrap();
        assert!(best.pose.position.distance(actual.position) < 1e-6);
        assert!(wrap_angle(best.pose.yaw - actual.yaw).abs() < 1e-6);
    }
    #[test]
    fn periodic_world_hypotheses_are_retained_and_rejected() {
        let scan = cloud();
        let map = (-3..=3)
            .flat_map(|i| {
                scan.iter()
                    .map(move |p| p.plus(Vec2::new(i as f64 * 2., 0.)))
            })
            .collect::<Vec<_>>();
        let result = search_region(
            &scan,
            &map,
            Pose::default(),
            &RegistrationConfig::default(),
            &RegionalSearchConfig::default(),
        );
        assert!(result.error.unwrap().contains("ambiguous"));
        assert!(result.candidates.len() > 1);
        assert_eq!(result.seeds_tested, 125);
    }
    #[test]
    fn budget_exhaustion_and_invalid_configuration_cannot_return_an_accepted_prefix() {
        let map = cloud();
        let mut config = RegionalSearchConfig {
            max_neighbor_checks: 1,
            ..RegionalSearchConfig::default()
        };
        let result = search_region(
            &map,
            &map,
            Pose::default(),
            &RegistrationConfig::default(),
            &config,
        );
        assert!(result.error.unwrap().contains("budget exhausted"));
        assert_eq!(result.neighbor_checks, 1);
        assert!(result.best.is_none());
        config.position_half_extent_m = f64::NAN;
        let result = search_region(
            &map,
            &map,
            Pose::default(),
            &RegistrationConfig::default(),
            &config,
        );
        assert!(result.error.is_some());
        assert_eq!(result.seeds_tested, 0);
    }
    #[test]
    fn regional_neighbors_match_exhaustive_search_including_ties_and_empty_radius() {
        let mut map: Vec<_> = (0..2000)
            .map(|i| {
                let t = i as f64;
                Vec2::new(
                    4. * (t * 0.73).sin() + (t * 0.13).cos(),
                    2. * (t * 0.29).cos(),
                )
            })
            .collect();
        map.extend([map[0], map[1]]);
        let tree = KdTree::new(&map);
        let mut budget = Budget {
            checks: 0,
            maximum: 10_000_000,
        };
        for i in 0..500 {
            let t = i as f64;
            let point = if i == 0 {
                map[0]
            } else {
                Vec2::new(6. * (t * 0.67).sin(), 4. * (t * 0.31).cos())
            };
            let expected = map
                .iter()
                .enumerate()
                .map(|(index, target)| (index, point.distance(*target)))
                .filter(|(_, d)| *d <= 1.5)
                .min_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
            assert_eq!(
                tree.nearest(&map, point, 1.5, &mut budget).unwrap(),
                expected
            );
        }
        assert!(budget.checks < 500 * map.len() / 5);
        let mut empty = Budget {
            checks: 0,
            maximum: 0,
        };
        assert!(
            tree.nearest(&map, map[0], 1.5, &mut empty)
                .unwrap_err()
                .contains("budget exhausted")
        );
    }
}
