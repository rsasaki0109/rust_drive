//! Robust rigid registration of measured, visually associated RGB-D points.
//!
//! Input correspondence identities must come from sensor observations. This
//! module does not infer pixel matches, use reference poses, or estimate a
//! calibrated covariance. All hypotheses and final consensus fits are bounded.
use crate::registration3d::{Pose3, Quaternion};
use rustdriving_core::Vec3;

const HARD_MATCHES: usize = 256;
const HARD_HYPOTHESES: usize = 128;
const HARD_REFITS: usize = 4;
const HARD_POINT_CHECKS: usize = (HARD_HYPOTHESES + HARD_REFITS) * HARD_MATCHES;

#[derive(Clone, Copy, Debug)]
pub struct Correspondence3d {
    pub previous: Vec3,
    pub current: Vec3,
}

#[derive(Clone, Debug)]
pub struct VisualOdometry3dConfig {
    pub max_matches: usize,
    pub max_hypotheses: usize,
    pub max_refits: usize,
    /// Individual measured-correspondence distance tests, excluding bounded
    /// small-matrix algebra and linear input/centroid/scatter calculations.
    pub max_point_checks: usize,
    pub inlier_distance_m: f64,
    pub min_inliers: usize,
    pub min_inlier_ratio: f64,
    /// Middle/largest scatter eigenvalue. Known associations require rank two;
    /// noncollinear planar points determine a proper rigid transform.
    pub min_geometry_ratio: f64,
    pub max_translation_m: f64,
    pub max_rotation_rad: f64,
    pub ambiguity_support_ratio: f64,
    pub ambiguity_rms_ratio: f64,
    pub ambiguity_translation_m: f64,
    pub ambiguity_rotation_rad: f64,
    pub ambiguity_noise_floor_m: f64,
}
impl Default for VisualOdometry3dConfig {
    fn default() -> Self {
        Self {
            max_matches: HARD_MATCHES,
            max_hypotheses: HARD_HYPOTHESES,
            max_refits: HARD_REFITS,
            max_point_checks: HARD_POINT_CHECKS,
            inlier_distance_m: 0.04,
            min_inliers: 12,
            min_inlier_ratio: 0.5,
            min_geometry_ratio: 0.005,
            max_translation_m: 0.5,
            max_rotation_rad: 0.35,
            ambiguity_support_ratio: 0.95,
            ambiguity_rms_ratio: 1.05,
            ambiguity_translation_m: 0.04,
            ambiguity_rotation_rad: 0.07,
            ambiguity_noise_floor_m: 0.001,
        }
    }
}
impl VisualOdometry3dConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.max_matches < 12
            || self.max_matches > HARD_MATCHES
            || self.max_hypotheses == 0
            || self.max_hypotheses > HARD_HYPOTHESES
            || self.max_refits == 0
            || self.max_refits > HARD_REFITS
            || self.max_point_checks == 0
            || self.max_point_checks > HARD_POINT_CHECKS
            || self.min_inliers < 12
            || self.min_inliers > self.max_matches
            || !self.inlier_distance_m.is_finite()
            || !(0.0..=0.04).contains(&self.inlier_distance_m)
            || self.inlier_distance_m == 0.0
            || !self.min_inlier_ratio.is_finite()
            || !(0.5..=1.0).contains(&self.min_inlier_ratio)
            || !self.min_geometry_ratio.is_finite()
            || !(0.005..=1.0).contains(&self.min_geometry_ratio)
            || !self.max_translation_m.is_finite()
            || !(0.0..=0.5).contains(&self.max_translation_m)
            || self.max_translation_m == 0.0
            || !self.max_rotation_rad.is_finite()
            || !(0.0..=0.35).contains(&self.max_rotation_rad)
            || self.max_rotation_rad == 0.0
            || !self.ambiguity_support_ratio.is_finite()
            || !(0.5..=0.95).contains(&self.ambiguity_support_ratio)
            || !self.ambiguity_rms_ratio.is_finite()
            || !(1.05..=2.0).contains(&self.ambiguity_rms_ratio)
            || !self.ambiguity_translation_m.is_finite()
            || !(0.0..=0.04).contains(&self.ambiguity_translation_m)
            || self.ambiguity_translation_m == 0.0
            || !self.ambiguity_rotation_rad.is_finite()
            || !(0.0..=0.07).contains(&self.ambiguity_rotation_rad)
            || self.ambiguity_rotation_rad == 0.0
            || !self.ambiguity_noise_floor_m.is_finite()
            || !(0.001..=0.04).contains(&self.ambiguity_noise_floor_m)
        {
            return Err("invalid bounded visual correspondence configuration".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct VisualOdometry3dResult {
    /// Previous optical camera frame from the current optical camera frame.
    pub pose: Pose3,
    pub rms_m: f64,
    /// Indices into the original input; strictly ascending and unique.
    pub inlier_indices: Vec<usize>,
    pub inlier_count: usize,
    pub inlier_ratio: f64,
    /// Triple attempts, including rejected degenerate or excessive-motion seeds.
    pub hypotheses_evaluated: usize,
    pub point_checks: usize,
    pub refits: usize,
    pub geometry_ratio_current: f64,
    pub geometry_ratio_previous: f64,
    pub candidate_models: usize,
    pub competing_models: usize,
}

fn subtract(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x - b.x, a.y - b.y, a.z - b.z)
}
fn norm(p: Vec3) -> f64 {
    p.x.hypot(p.y).hypot(p.z)
}
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}
fn array(p: Vec3) -> [f64; 3] {
    [p.x, p.y, p.z]
}
fn valid_point(p: Vec3) -> bool {
    p.finite() && norm(p) <= 1e6
}
fn mean(points: &[Vec3]) -> Vec3 {
    // Center around a measured point instead of summing large map origins.
    let origin = points[0];
    let sum = points
        .iter()
        .skip(1)
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, p| {
            Vec3::new(
                sum.x + (p.x - origin.x),
                sum.y + (p.y - origin.y),
                sum.z + (p.z - origin.z),
            )
        });
    let n = points.len() as f64;
    Vec3::new(
        origin.x + sum.x / n,
        origin.y + sum.y / n,
        origin.z + sum.z / n,
    )
}

// Small symmetric matrices only. Each sweep picks the largest off-diagonal
// entry deterministically; columns of the returned matrix are eigenvectors.
fn symmetric_eigen<const N: usize>(
    mut a: [[f64; N]; N],
) -> Result<([f64; N], [[f64; N]; N]), String> {
    let mut v = [[0.0; N]; N];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for _ in 0..100 * N * N {
        let (mut p, mut q, mut largest) = (0, 1, 0.0);
        for (i, row) in a.iter().enumerate() {
            for (j, value) in row.iter().enumerate().skip(i + 1) {
                if value.abs() > largest {
                    (p, q, largest) = (i, j, value.abs());
                }
            }
        }
        let magnitude = a
            .iter()
            .enumerate()
            .map(|(i, row)| row[i].abs())
            .fold(0.0, f64::max);
        if largest <= 1e-13 * magnitude.max(1e-30) {
            let values = std::array::from_fn(|i| a[i][i]);
            if values.iter().any(|x| !x.is_finite()) {
                return Err("nonfinite visual rigid-fit eigensystem".into());
            }
            return Ok((values, v));
        }
        let angle = 0.5 * (2.0 * a[p][q]).atan2(a[q][q] - a[p][p]);
        let c = angle.cos();
        let s = angle.sin();
        let (app, aqq, apq) = (a[p][p], a[q][q], a[p][q]);
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        for k in 0..N {
            if k != p && k != q {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[p][k] = a[k][p];
                a[k][q] = s * akp + c * akq;
                a[q][k] = a[k][q];
            }
        }
        for row in &mut v {
            let (vp, vq) = (row[p], row[q]);
            row[p] = c * vp - s * vq;
            row[q] = s * vp + c * vq;
        }
    }
    Err("visual rigid-fit eigensolver iteration bound exceeded".into())
}

fn geometry_ratio(points: &[Vec3]) -> Result<f64, String> {
    let center = mean(points);
    let mut scatter = [[0.0; 3]; 3];
    for &p in points {
        let a = array(subtract(p, center));
        for i in 0..3 {
            for j in 0..3 {
                scatter[i][j] += a[i] * a[j];
            }
        }
    }
    let (mut values, _) = symmetric_eigen(scatter)?;
    values.sort_by(f64::total_cmp);
    // A proper rotation and translation are fixed by three noncollinear known
    // correspondences, including when the observed points all share a plane.
    let ratio = values[1] / values[2];
    if !ratio.is_finite() {
        return Err("degenerate visual correspondence geometry".into());
    }
    Ok(ratio)
}

/// Validate a bounded measured feature cloud before initialization. This emits
/// geometry evidence only; it does not estimate a pose or reference covariance.
pub fn validate_geometry(points: &[Vec3], config: &VisualOdometry3dConfig) -> Result<f64, String> {
    config.validate()?;
    if points.len() < config.min_inliers
        || points.len() > config.max_matches
        || points.iter().any(|&p| !valid_point(p))
    {
        return Err("invalid or oversized visual initialization geometry".into());
    }
    let ratio = geometry_ratio(points)?;
    if ratio < config.min_geometry_ratio {
        return Err("collinear or poorly conditioned visual initialization geometry".into());
    }
    Ok(ratio)
}

fn fit(matches: &[Correspondence3d], indices: &[usize]) -> Result<Pose3, String> {
    let current: Vec<_> = indices.iter().map(|&i| matches[i].current).collect();
    let previous: Vec<_> = indices.iter().map(|&i| matches[i].previous).collect();
    let sm = mean(&current);
    let tm = mean(&previous);
    let mut covariance = [[0.0; 3]; 3];
    for (&s, &t) in current.iter().zip(&previous) {
        let a = array(subtract(s, sm));
        let b = array(subtract(t, tm));
        for i in 0..3 {
            for (j, &bj) in b.iter().enumerate() {
                covariance[i][j] += a[i] * bj;
            }
        }
    }
    let [xx, xy, xz] = covariance[0];
    let [yx, yy, yz] = covariance[1];
    let [zx, zy, zz] = covariance[2];
    let horn = [
        [xx + yy + zz, yz - zy, zx - xz, xy - yx],
        [yz - zy, xx - yy - zz, xy + yx, zx + xz],
        [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
        [xy - yx, zx + xz, yz + zy, -xx - yy + zz],
    ];
    let (values, vectors) = symmetric_eigen(horn)?;
    let index = (0..4)
        .max_by(|&a, &b| values[a].total_cmp(&values[b]))
        .unwrap();
    let rotation = Quaternion {
        w: vectors[0][index],
        x: vectors[1][index],
        y: vectors[2][index],
        z: vectors[3][index],
    }
    .normalized()?;
    let pose = Pose3 {
        translation: subtract(tm, rotation.rotate(sm)),
        rotation,
    };
    if !valid_point(pose.translation) {
        return Err("nonfinite visual rigid-fit pose".into());
    }
    Ok(pose)
}

fn noncollinear(points: [Vec3; 3]) -> bool {
    let a = subtract(points[1], points[0]);
    let b = subtract(points[2], points[0]);
    let scale = norm(a) * norm(b);
    scale.is_finite() && scale > 1e-10 && norm(cross(a, b)) / scale >= 0.001
}
fn motion_allowed(pose: Pose3, config: &VisualOdometry3dConfig) -> bool {
    norm(pose.translation) <= config.max_translation_m
        && pose.rotation.angular_distance(Quaternion::identity()) <= config.max_rotation_rad
}
fn score(
    matches: &[Correspondence3d],
    pose: Pose3,
    config: &VisualOdometry3dConfig,
    point_checks: &mut usize,
) -> Result<(Vec<usize>, f64), String> {
    let mut inliers = Vec::new();
    let mut sum_squares = 0.0;
    for (i, pair) in matches.iter().enumerate() {
        if *point_checks >= config.max_point_checks {
            return Err("visual correspondence point-check budget exhausted".into());
        }
        *point_checks += 1;
        let residual = norm(subtract(pose.transform(pair.current), pair.previous));
        if !residual.is_finite() {
            return Err("nonfinite visual correspondence residual".into());
        }
        if residual <= config.inlier_distance_m {
            inliers.push(i);
            sum_squares += residual * residual;
        }
    }
    Ok((inliers, sum_squares))
}
fn next_sample(state: &mut u64, n: usize) -> usize {
    *state = state.wrapping_add(0x9e3779b97f4a7c15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
    ((z ^ (z >> 31)) % n as u64) as usize
}

/// Register known measured correspondences as previous-from-current. A failed
/// robust fit emits no pose; callers own accepted-pose age and trajectory state.
pub fn register_correspondences(
    matches: &[Correspondence3d],
    config: &VisualOdometry3dConfig,
) -> Result<VisualOdometry3dResult, String> {
    config.validate()?;
    if matches.len() < config.min_inliers
        || matches.len() > config.max_matches
        || matches
            .iter()
            .any(|p| !valid_point(p.current) || !valid_point(p.previous))
    {
        return Err("invalid or oversized visual correspondences".into());
    }
    let mut state = 0x7275737464726976;
    let mut best: Option<(Vec<usize>, f64)> = None;
    let mut candidates = Vec::with_capacity(config.max_hypotheses);
    let mut point_checks = 0;
    for _ in 0..config.max_hypotheses {
        let a = next_sample(&mut state, matches.len());
        let mut b = next_sample(&mut state, matches.len() - 1);
        if b >= a {
            b += 1;
        }
        let mut c = next_sample(&mut state, matches.len() - 2);
        for skip in [a.min(b), a.max(b)] {
            if c >= skip {
                c += 1;
            }
        }
        let indices = [a, b, c];
        if !noncollinear(indices.map(|i| matches[i].current))
            || !noncollinear(indices.map(|i| matches[i].previous))
        {
            continue;
        }
        let pose = fit(matches, &indices)?;
        if !motion_allowed(pose, config) {
            continue;
        }
        let (inliers, error) = score(matches, pose, config, &mut point_checks)?;
        if inliers.len() < config.min_inliers
            || inliers.len() as f64 / (matches.len() as f64) < config.min_inlier_ratio
        {
            continue;
        }
        let current: Vec<_> = inliers.iter().map(|&i| matches[i].current).collect();
        let previous: Vec<_> = inliers.iter().map(|&i| matches[i].previous).collect();
        if geometry_ratio(&current)? < config.min_geometry_ratio
            || geometry_ratio(&previous)? < config.min_geometry_ratio
        {
            continue;
        }
        candidates.push((pose, inliers.len(), (error / inliers.len() as f64).sqrt()));
        if best.as_ref().is_none_or(|(old, old_error)| {
            inliers.len() > old.len() || (inliers.len() == old.len() && error < *old_error)
        }) {
            best = Some((inliers, error));
        }
    }
    let (mut consensus, _) = best.ok_or("no bounded visual correspondence consensus")?;
    for refits in 1..=config.max_refits {
        let pose = fit(matches, &consensus)?;
        if !motion_allowed(pose, config) {
            return Err("visual consensus refit exceeds motion bound".into());
        }
        let (next, error) = score(matches, pose, config, &mut point_checks)?;
        if next.len() < config.min_inliers
            || next.len() as f64 / (matches.len() as f64) < config.min_inlier_ratio
        {
            return Err("visual consensus refit has insufficient inliers".into());
        }
        if next == consensus {
            let current: Vec<_> = next.iter().map(|&i| matches[i].current).collect();
            let previous: Vec<_> = next.iter().map(|&i| matches[i].previous).collect();
            let current_ratio = geometry_ratio(&current)?;
            let previous_ratio = geometry_ratio(&previous)?;
            if current_ratio < config.min_geometry_ratio
                || previous_ratio < config.min_geometry_ratio
            {
                return Err("collinear or poorly conditioned visual consensus geometry".into());
            }
            let count = next.len();
            let rms = (error / count as f64).sqrt();
            let competing_models = candidates
                .iter()
                .filter(|(candidate, support, residual)| {
                    *support as f64 >= count as f64 * config.ambiguity_support_ratio
                        && *residual
                            <= rms.max(config.ambiguity_noise_floor_m) * config.ambiguity_rms_ratio
                        && (norm(subtract(candidate.translation, pose.translation))
                            > config.ambiguity_translation_m
                            || candidate.rotation.angular_distance(pose.rotation)
                                > config.ambiguity_rotation_rad)
                })
                .count();
            if competing_models > 0 {
                return Err(
                    "ambiguous visual correspondence registration: comparable distinct rigid model"
                        .into(),
                );
            }
            return Ok(VisualOdometry3dResult {
                pose,
                rms_m: rms,
                inlier_indices: next,
                inlier_count: count,
                inlier_ratio: count as f64 / matches.len() as f64,
                hypotheses_evaluated: config.max_hypotheses,
                point_checks,
                refits,
                geometry_ratio_current: current_ratio,
                geometry_ratio_previous: previous_ratio,
                candidate_models: candidates.len(),
                competing_models,
            });
        }
        consensus = next;
    }
    Err("visual consensus did not stabilize within refit bound".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn points() -> Vec<Vec3> {
        (0..96)
            .map(|i| {
                let t = i as f64;
                Vec3::new(
                    (t * 1.37).sin(),
                    (t * 0.77).cos() * 0.8,
                    2.0 + (t * 0.49).sin() * 0.6,
                )
            })
            .collect()
    }
    fn measurements(pose: Pose3) -> Vec<Correspondence3d> {
        points()
            .into_iter()
            .map(|current| Correspondence3d {
                current,
                previous: pose.transform(current),
            })
            .collect()
    }
    fn assert_pose(actual: Pose3, expected: Pose3, tolerance: f64) {
        assert!(norm(subtract(actual.translation, expected.translation)) < tolerance);
        assert!(actual.rotation.angular_distance(expected.rotation) < tolerance);
    }
    #[test]
    fn measured_associations_recover_all_three_rotation_axes_and_translation() {
        for axis in [
            Vec3::new(1., 0., 0.),
            Vec3::new(0., 1., 0.),
            Vec3::new(0., 0., 1.),
        ] {
            let pose = Pose3 {
                translation: Vec3::new(0.08, -0.03, 0.02),
                rotation: Quaternion::from_axis_angle(axis, 0.12).unwrap(),
            };
            let result =
                register_correspondences(&measurements(pose), &VisualOdometry3dConfig::default())
                    .unwrap();
            assert_pose(result.pose, pose, 1e-8);
            assert_eq!(result.inlier_count, 96);
            assert_eq!(result.inlier_indices, (0..96).collect::<Vec<_>>());
            assert!(result.rms_m < 1e-10);
            assert!(result.point_checks <= HARD_POINT_CHECKS);
        }
    }
    #[test]
    fn outliers_and_bounded_noise_are_removed_without_losing_motion() {
        let pose = Pose3 {
            translation: Vec3::new(0.06, -0.04, 0.03),
            rotation: Quaternion::from_axis_angle(Vec3::new(0.3, 0.7, 0.2), 0.08).unwrap(),
        };
        let mut pairs = measurements(pose);
        for (i, pair) in pairs.iter_mut().enumerate() {
            if i % 3 == 0 {
                pair.previous = Vec3::new(8.0 + i as f64, -4.0, 3.0);
            } else {
                let noise = (i as f64 * 2.1).sin() * 0.002;
                pair.previous.x += noise;
                pair.previous.y -= noise;
            }
        }
        let result = register_correspondences(&pairs, &VisualOdometry3dConfig::default()).unwrap();
        assert_pose(result.pose, pose, 0.003);
        assert_eq!(
            result.inlier_indices,
            (0..96).filter(|i| i % 3 != 0).collect::<Vec<_>>()
        );
        assert!(result.rms_m < 0.004);
        // Independently check first-order least-squares stationarity, rather
        // than recomputing the same Horn implementation. A noisy triple alone
        // generally fails these consensus-wide translation/rotation gradients.
        let count = result.inlier_count as f64;
        let mut center = Vec3::new(0.0, 0.0, 0.0);
        for &i in &result.inlier_indices {
            center.x += pairs[i].current.x / count;
            center.y += pairs[i].current.y / count;
            center.z += pairs[i].current.z / count;
        }
        let mut translation_gradient = Vec3::new(0.0, 0.0, 0.0);
        let mut rotation_gradient = Vec3::new(0.0, 0.0, 0.0);
        for &i in &result.inlier_indices {
            let pair = pairs[i];
            let residual = subtract(pair.previous, result.pose.transform(pair.current));
            translation_gradient.x += residual.x;
            translation_gradient.y += residual.y;
            translation_gradient.z += residual.z;
            let centered = result.pose.rotation.rotate(subtract(pair.current, center));
            let gradient = cross(centered, residual);
            rotation_gradient.x += gradient.x;
            rotation_gradient.y += gradient.y;
            rotation_gradient.z += gradient.z;
        }
        assert!(norm(translation_gradient) < 1e-8);
        assert!(norm(rotation_gradient) < 1e-8);
        let repeat = register_correspondences(&pairs, &VisualOdometry3dConfig::default()).unwrap();
        assert_pose(result.pose, repeat.pose, 1e-10);
        assert_eq!(result.point_checks, repeat.point_checks);
    }
    #[test]
    fn unrelated_correspondences_or_excessive_motion_cannot_issue_a_pose() {
        let cfg = VisualOdometry3dConfig::default();
        let mut mismatches = measurements(Pose3::identity());
        let original = mismatches.clone();
        for (i, p) in mismatches.iter_mut().enumerate() {
            p.previous = original[(i * 31 + 7) % original.len()].previous;
        }
        assert!(register_correspondences(&mismatches, &cfg).is_err());
        let excessive = measurements(Pose3 {
            translation: Vec3::new(0.6, 0., 0.),
            rotation: Quaternion::identity(),
        });
        assert!(register_correspondences(&excessive, &cfg).is_err());
        let excessive = measurements(Pose3 {
            translation: Vec3::new(0., 0., 0.),
            rotation: Quaternion::from_axis_angle(Vec3::new(0., 1., 0.), 0.4).unwrap(),
        });
        assert!(register_correspondences(&excessive, &cfg).is_err());
    }
    #[test]
    fn noncollinear_planar_known_associations_recover_a_proper_rigid_transform() {
        let cloud: Vec<_> = (0..36)
            .map(|i| Vec3::new((i % 6) as f64 * 0.1, (i / 6) as f64 * 0.1, 2.0))
            .collect();
        let config = VisualOdometry3dConfig::default();
        assert!((validate_geometry(&cloud, &config).unwrap() - 1.0).abs() < 1e-12);
        let pose = Pose3 {
            translation: Vec3::new(0.09, -0.04, 0.03),
            rotation: Quaternion::from_axis_angle(Vec3::new(0.3, 0.7, 0.2), 0.12).unwrap(),
        };
        let pairs: Vec<_> = cloud
            .into_iter()
            .map(|current| Correspondence3d {
                previous: pose.transform(current),
                current,
            })
            .collect();
        let result = register_correspondences(&pairs, &config).unwrap();
        assert_pose(result.pose, pose, 1e-8);
        assert_eq!(result.inlier_count, 36);
        assert!((result.geometry_ratio_current - 1.0).abs() < 1e-10);
        assert!((result.geometry_ratio_previous - 1.0).abs() < 1e-10);
        assert_eq!(result.competing_models, 0);
    }
    #[test]
    fn collinear_or_duplicate_observations_are_rejected() {
        for cloud in [
            (0..36)
                .map(|i| Vec3::new(i as f64 * 0.1, 0., 2.))
                .collect::<Vec<_>>(),
            vec![Vec3::new(0., 0., 2.); 36],
        ] {
            assert!(validate_geometry(&cloud, &VisualOdometry3dConfig::default()).is_err());
            let pairs: Vec<_> = cloud
                .into_iter()
                .map(|p| Correspondence3d {
                    previous: p,
                    current: p,
                })
                .collect();
            assert!(register_correspondences(&pairs, &VisualOdometry3dConfig::default()).is_err());
        }
    }
    #[test]
    fn finite_point_and_resource_bounds_apply_before_hypothesis_work() {
        let config = VisualOdometry3dConfig::default();
        let cloud = points();
        assert!(validate_geometry(&cloud, &config).is_ok());
        assert!(validate_geometry(&[], &config).is_err());
        let mut pairs = measurements(Pose3::identity());
        pairs[0].previous.z = f64::NAN;
        assert!(register_correspondences(&pairs, &config).is_err());
        pairs[0].previous.z = 1e308;
        assert!(register_correspondences(&pairs, &config).is_err());
        let pairs = vec![
            Correspondence3d {
                previous: cloud[0],
                current: cloud[0]
            };
            257
        ];
        assert!(register_correspondences(&pairs, &config).is_err());
        assert!(
            VisualOdometry3dConfig {
                min_inliers: 11,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            VisualOdometry3dConfig {
                inlier_distance_m: 0.05,
                ..Default::default()
            }
            .validate()
            .is_err()
        );
    }
    #[test]
    fn exhausted_work_budget_rejects_instead_of_returning_partial_consensus() {
        let pairs = measurements(Pose3::identity());
        let config = VisualOdometry3dConfig {
            max_point_checks: 95,
            ..Default::default()
        };
        assert!(
            register_correspondences(&pairs, &config)
                .unwrap_err()
                .contains("budget exhausted")
        );
        let config = VisualOdometry3dConfig {
            max_point_checks: HARD_POINT_CHECKS + 1,
            ..Default::default()
        };
        assert!(config.validate().is_err());
    }
    #[test]
    fn equal_support_for_distinct_moving_groups_is_rejected_as_ambiguous() {
        let mut pairs = measurements(Pose3::identity());
        let other = Pose3 {
            translation: Vec3::new(0.12, 0.0, 0.0),
            rotation: Quaternion::identity(),
        };
        for (i, pair) in pairs.iter_mut().enumerate() {
            if i % 2 == 0 {
                pair.previous = other.transform(pair.current);
            }
        }
        let error =
            register_correspondences(&pairs, &VisualOdometry3dConfig::default()).unwrap_err();
        assert!(error.contains("ambiguous visual"), "{error}");
    }
    #[test]
    fn seeded_triples_are_distinct_and_sampling_covers_the_input_range() {
        let mut state = 0x7275737464726976;
        let mut seen = [false; 256];
        for n in 12..=256 {
            let a = next_sample(&mut state, n);
            let mut b = next_sample(&mut state, n - 1);
            if b >= a {
                b += 1;
            }
            let mut c = next_sample(&mut state, n - 2);
            for skip in [a.min(b), a.max(b)] {
                if c >= skip {
                    c += 1;
                }
            }
            assert!(a < n && b < n && c < n && a != b && a != c && b != c);
            for i in [a, b, c] {
                seen[i] = true;
            }
        }
        assert!(seen[..128].iter().filter(|&&s| s).count() > 120);
    }
}
