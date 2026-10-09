//! Bounded local SE(3) point-to-point ICP. Inputs are measured sensor-frame
//! points and a fixed map in the target frame, never current ground-truth poses.
//! This is not SLAM or global relocalization. Covariance is conditional on the
//! retained correspondences and an independent isotropic point-noise model.
use rustdrive_core::Vec3;
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl Quaternion {
    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
    pub fn normalized(self) -> Result<Self, String> {
        let n = self.w.hypot(self.x).hypot(self.y).hypot(self.z);
        if !n.is_finite() || n < 1e-12 {
            return Err("invalid quaternion".into());
        }
        let s = if self.w < 0.0 { -1.0 / n } else { 1.0 / n };
        Ok(Self {
            w: self.w * s,
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
        })
    }
    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Result<Self, String> {
        let n = norm(axis);
        if !axis.finite() || n < 1e-12 || !angle.is_finite() {
            return Err("invalid axis angle".into());
        }
        let s = (angle * 0.5).sin() / n;
        Self {
            w: (angle * 0.5).cos(),
            x: axis.x * s,
            y: axis.y * s,
            z: axis.z * s,
        }
        .normalized()
    }
    pub fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
    pub fn multiply(self, b: Self) -> Self {
        Self {
            w: self.w * b.w - self.x * b.x - self.y * b.y - self.z * b.z,
            x: self.w * b.x + self.x * b.w + self.y * b.z - self.z * b.y,
            y: self.w * b.y - self.x * b.z + self.y * b.w + self.z * b.x,
            z: self.w * b.z + self.x * b.y - self.y * b.x + self.z * b.w,
        }
    }
    pub fn rotate(self, p: Vec3) -> Vec3 {
        let v = Vec3::new(self.x, self.y, self.z);
        let t = scale(cross(v, p), 2.0);
        add(p, add(scale(t, self.w), cross(v, t)))
    }
    pub fn angular_distance(self, b: Self) -> f64 {
        let d = self.multiply(b.conjugate());
        2.0 * norm(Vec3::new(d.x, d.y, d.z)).atan2(d.w.abs())
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Pose3 {
    pub translation: Vec3,
    pub rotation: Quaternion,
}
impl Pose3 {
    pub fn identity() -> Self {
        Self {
            translation: Vec3::new(0.0, 0.0, 0.0),
            rotation: Quaternion::identity(),
        }
    }
    pub fn transform(self, p: Vec3) -> Vec3 {
        add(self.rotation.rotate(p), self.translation)
    }
    pub fn inverse(self) -> Self {
        let rotation = self.rotation.conjugate();
        Self {
            translation: rotation.rotate(scale(self.translation, -1.0)),
            rotation,
        }
    }
    pub fn compose(self, b: Self) -> Self {
        Self {
            translation: self.transform(b.translation),
            rotation: self.rotation.multiply(b.rotation),
        }
    }
}
#[derive(Clone, Debug)]
pub struct Registration3dConfig {
    pub max_scan_points: usize,
    pub max_map_points: usize,
    pub max_iterations: usize,
    /// Global distance-comparison budget shared by the fit and all probes.
    pub max_neighbor_checks: usize,
    pub max_correspondence_m: f64,
    pub trim_fraction: f64,
    pub min_pairs: usize,
    pub min_overlap: f64,
    pub max_translation_jump_m: f64,
    pub max_rotation_jump_rad: f64,
    pub max_rms_m: f64,
    pub min_geometry_ratio: f64,
    pub max_condition_number: f64,
    pub max_position_variance_m2: f64,
    pub max_rotation_variance_rad2: f64,
    pub translation_tolerance_m: f64,
    pub rotation_tolerance_rad: f64,
    pub ambiguity_translation_probe_m: f64,
    pub ambiguity_rotation_probe_rad: f64,
    pub ambiguity_rms_ratio: f64,
}
impl Default for Registration3dConfig {
    fn default() -> Self {
        Self {
            max_scan_points: 5000,
            max_map_points: 20000,
            max_iterations: 30,
            max_neighbor_checks: 20_000_000,
            max_correspondence_m: 0.3,
            trim_fraction: 0.2,
            min_pairs: 30,
            min_overlap: 0.4,
            max_translation_jump_m: 0.5,
            max_rotation_jump_rad: 0.35,
            max_rms_m: 0.08,
            min_geometry_ratio: 0.005,
            max_condition_number: 10_000.0,
            max_position_variance_m2: 0.25,
            max_rotation_variance_rad2: 0.01,
            translation_tolerance_m: 0.0001,
            rotation_tolerance_rad: 0.0001,
            ambiguity_translation_probe_m: 0.15,
            ambiguity_rotation_probe_rad: 0.07,
            ambiguity_rms_ratio: 1.05,
        }
    }
}
impl Registration3dConfig {
    pub fn validate(&self) -> Result<(), String> {
        let pos = |v: f64, max: f64| v.is_finite() && v > 0.0 && v <= max;
        if self.max_scan_points == 0
            || self.max_scan_points > 5000
            || self.max_map_points == 0
            || self.max_map_points > 20000
            || self.max_iterations == 0
            || self.max_iterations > 50
            || self.max_neighbor_checks == 0
            || self.max_neighbor_checks > 20_000_000
            || self.min_pairs < 6
            || self.min_pairs > self.max_scan_points
            || self.min_pairs > self.max_map_points
            || !pos(self.max_correspondence_m, 10.0)
            || !self.trim_fraction.is_finite()
            || !(0.0..=0.8).contains(&self.trim_fraction)
            || !pos(self.min_overlap, 1.0)
            || !pos(self.max_translation_jump_m, 50.0)
            || !pos(self.max_rotation_jump_rad, std::f64::consts::PI)
            || !pos(self.max_rms_m, self.max_correspondence_m)
            || !pos(self.min_geometry_ratio, 1.0)
            || !self.max_condition_number.is_finite()
            || !(1.0..=1e12).contains(&self.max_condition_number)
            || !pos(self.max_position_variance_m2, 1e6)
            || !pos(self.max_rotation_variance_rad2, 10.0)
            || !pos(self.translation_tolerance_m, self.max_correspondence_m)
            || !pos(self.rotation_tolerance_rad, self.max_rotation_jump_rad)
            || !pos(
                self.ambiguity_translation_probe_m,
                self.max_translation_jump_m,
            )
            || !pos(
                self.ambiguity_rotation_probe_rad,
                self.max_rotation_jump_rad,
            )
            || !self.ambiguity_rms_ratio.is_finite()
            || !(1.0..=2.0).contains(&self.ambiguity_rms_ratio)
        {
            return Err("invalid bounded SE3 registration configuration".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug)]
pub struct Registration3dConditioning {
    /// Smallest / largest centered target scatter eigenvalue. Rejects planar
    /// scenes conservatively even though known point associations could observe
    /// a planar rigid transform; nearest-plane associations are not known.
    pub geometry_ratio: f64,
    /// Centered translation/rotation information, with meters/radians units.
    pub condition_number: f64,
    pub neighbor_checks: usize,
    pub ambiguity_probes: usize,
}
#[derive(Clone, Debug)]
pub struct Registration3dResult {
    pub pose: Pose3,
    pub rms_m: f64,
    pub inlier_fraction: f64,
    pub inlier_count: usize,
    pub iterations: usize,
    pub converged: bool,
    /// x/y/z translation, then small world-frame rotation vector. Local IID
    /// conditional covariance; excludes map error and association uncertainty.
    pub covariance: [[f64; 6]; 6],
    pub conditioning: Registration3dConditioning,
}
fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x + b.x, a.y + b.y, a.z + b.z)
}
fn sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x - b.x, a.y - b.y, a.z - b.z)
}
fn scale(a: Vec3, s: f64) -> Vec3 {
    Vec3::new(a.x * s, a.y * s, a.z * s)
}
fn norm(a: Vec3) -> f64 {
    a.x.hypot(a.y).hypot(a.z)
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
fn mean(points: impl Iterator<Item = Vec3>) -> Vec3 {
    // Offset summation avoids catastrophic cancellation for large map origins.
    let mut it = points;
    let origin = it.next().unwrap();
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    let mut n = 1usize;
    for p in it {
        sum = add(sum, sub(p, origin));
        n += 1;
    }
    add(origin, scale(sum, 1.0 / n as f64))
}
/// Deterministic symmetric Jacobi eigenvectors are columns. Relative stopping
/// is scale-aware; failure to diagonalize is rejected rather than hidden.
fn eigen<const N: usize>(mut a: [[f64; N]; N]) -> Result<([f64; N], [[f64; N]; N]), String> {
    let mut v = [[0.0; N]; N];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for _ in 0..(100 * N * N) {
        let mut p = 0;
        let mut q = 1;
        let mut largest = 0.0;
        for (i, row) in a.iter().enumerate() {
            for (j, &value) in row.iter().enumerate().skip(i + 1) {
                if value.abs() > largest {
                    largest = value.abs();
                    p = i;
                    q = j;
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
                return Err("nonfinite eigensystem".into());
            }
            return Ok((values, v));
        }
        let theta = 0.5 * (2.0 * a[p][q]).atan2(a[q][q] - a[p][p]);
        let c = theta.cos();
        let s = theta.sin();
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        for k in 0..N {
            if k != p && k != q {
                let kp = a[k][p];
                let kq = a[k][q];
                a[k][p] = c * kp - s * kq;
                a[p][k] = a[k][p];
                a[k][q] = s * kp + c * kq;
                a[q][k] = a[k][q];
            }
        }
        for row in &mut v {
            let vp = row[p];
            let vq = row[q];
            row[p] = c * vp - s * vq;
            row[q] = s * vp + c * vq;
        }
    }
    Err("eigensolver iteration bound exceeded".into())
}
struct Grid<'a> {
    map: &'a [Vec3],
    cells: BTreeMap<(i64, i64, i64), Vec<usize>>,
    size: f64,
}
impl<'a> Grid<'a> {
    fn key(&self, p: Vec3) -> Result<(i64, i64, i64), String> {
        let a = array(p).map(|v| (v / self.size).floor());
        if a.iter().any(|v| !v.is_finite() || v.abs() > 1e14) {
            return Err("registration coordinate exceeds grid bound".into());
        }
        Ok((a[0] as i64, a[1] as i64, a[2] as i64))
    }
    fn new(map: &'a [Vec3], size: f64) -> Result<Self, String> {
        let mut grid = Self {
            map,
            cells: BTreeMap::new(),
            size,
        };
        for (i, &p) in map.iter().enumerate() {
            grid.cells.entry(grid.key(p)?).or_default().push(i);
        }
        Ok(grid)
    }
}
struct Budget {
    checks: usize,
    limit: usize,
}
#[derive(Clone, Copy)]
struct Pair {
    source: usize,
    target: usize,
    distance: f64,
}
fn pairs(
    scan: &[Vec3],
    grid: &Grid<'_>,
    pose: Pose3,
    cfg: &Registration3dConfig,
    budget: &mut Budget,
) -> Result<Vec<Pair>, String> {
    let mut unique: BTreeMap<usize, Pair> = BTreeMap::new();
    for (i, &p) in scan.iter().enumerate() {
        let p = pose.transform(p);
        let (x, y, z) = grid.key(p)?;
        let mut best: Option<(usize, f64)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(indices) = grid.cells.get(&(x + dx, y + dy, z + dz)) {
                        for &j in indices {
                            if budget.checks == budget.limit {
                                return Err("registration distance budget exhausted".into());
                            }
                            budget.checks += 1;
                            let d = norm(sub(p, grid.map[j]));
                            if d <= cfg.max_correspondence_m
                                && best.is_none_or(|(k, e)| d < e || (d == e && j < k))
                            {
                                best = Some((j, d));
                            }
                        }
                    }
                }
            }
        }
        if let Some((target, distance)) = best {
            let candidate = Pair {
                source: i,
                target,
                distance,
            };
            let keep = unique.get(&target).is_none_or(|old| {
                distance < old.distance || (distance == old.distance && i < old.source)
            });
            if keep {
                unique.insert(target, candidate);
            }
        }
    }
    let mut result: Vec<_> = unique.into_values().collect();
    result.sort_by(|a, b| {
        a.distance
            .total_cmp(&b.distance)
            .then(a.source.cmp(&b.source))
    });
    result.truncate(((result.len() as f64) * (1.0 - cfg.trim_fraction)).floor() as usize);
    if result.len() < cfg.min_pairs || (result.len() as f64) / (scan.len() as f64) < cfg.min_overlap
    {
        return Err("insufficient unique registration overlap".into());
    }
    Ok(result)
}
fn rigid_fit(scan: &[Vec3], map: &[Vec3], pairs: &[Pair]) -> Result<Pose3, String> {
    let sm = mean(pairs.iter().map(|p| scan[p.source]));
    let tm = mean(pairs.iter().map(|p| map[p.target]));
    let mut s = [[0.0; 3]; 3];
    for p in pairs {
        let a = array(sub(scan[p.source], sm));
        let b = array(sub(map[p.target], tm));
        for i in 0..3 {
            for (j, bj) in b.iter().enumerate() {
                s[i][j] += a[i] * bj;
            }
        }
    }
    let [xx, xy, xz] = s[0];
    let [yx, yy, yz] = s[1];
    let [zx, zy, zz] = s[2];
    let horn = [
        [xx + yy + zz, yz - zy, zx - xz, xy - yx],
        [yz - zy, xx - yy - zz, xy + yx, zx + xz],
        [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
        [xy - yx, zx + xz, yz + zy, -xx - yy + zz],
    ];
    let (values, v) = eigen(horn)?;
    let index = (0..4)
        .max_by(|&a, &b| values[a].total_cmp(&values[b]))
        .unwrap();
    let rotation = Quaternion {
        w: v[0][index],
        x: v[1][index],
        y: v[2][index],
        z: v[3][index],
    }
    .normalized()?;
    Ok(Pose3 {
        translation: sub(tm, rotation.rotate(sm)),
        rotation,
    })
}
fn conditioning(
    scan: &[Vec3],
    map: &[Vec3],
    pairs: &[Pair],
    pose: Pose3,
    rms: f64,
    cfg: &Registration3dConfig,
) -> Result<([[f64; 6]; 6], f64, f64), String> {
    let sm = mean(pairs.iter().map(|p| scan[p.source]));
    let tm = mean(pairs.iter().map(|p| map[p.target]));
    let mut scatter = [[0.0; 3]; 3];
    let mut rotated = [[0.0; 3]; 3];
    for p in pairs {
        let a = array(sub(map[p.target], tm));
        let b = array(pose.rotation.rotate(sub(scan[p.source], sm)));
        for i in 0..3 {
            for j in 0..3 {
                scatter[i][j] += a[i] * a[j];
                rotated[i][j] += b[i] * b[j];
            }
        }
    }
    let (values, _) = eigen(scatter)?;
    let major = values.iter().copied().fold(0.0, f64::max);
    let minor = values.iter().copied().fold(f64::INFINITY, f64::min);
    let ratio = minor / major;
    let trace = (0..3).map(|i| rotated[i][i]).sum::<f64>();
    let info: [[f64; 3]; 3] = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            if i == j {
                trace - rotated[i][j]
            } else {
                -rotated[i][j]
            }
        })
    });
    let (values, v) = eigen(info)?;
    let n = pairs.len() as f64;
    let low = values.iter().copied().fold(n, f64::min);
    let high = values.iter().copied().fold(n, f64::max);
    let condition = high / low;
    if !ratio.is_finite()
        || ratio < cfg.min_geometry_ratio
        || !condition.is_finite()
        || low <= 0.0
        || condition > cfg.max_condition_number
    {
        return Err("planar or poorly conditioned SE3 registration geometry".into());
    }
    let noise = (rms * rms * n / (3.0 * n - 6.0)).max(1e-6);
    let mut centered = [[0.0; 6]; 6];
    for (i, row) in centered.iter_mut().enumerate().take(3) {
        row[i] = noise / n;
    }
    for i in 0..3 {
        for j in 0..3 {
            centered[i + 3][j + 3] = (0..3).map(|k| v[i][k] * v[j][k] * noise / values[k]).sum();
        }
    }
    let m = pose.rotation.rotate(sm);
    let skew = [[0.0, -m.z, m.y], [m.z, 0.0, -m.x], [-m.y, m.x, 0.0]];
    let mut t = [[0.0; 6]; 6];
    for (i, row) in t.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for i in 0..3 {
        for j in 0..3 {
            t[i][j + 3] = skew[i][j];
        }
    }
    let mut covariance = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in i..6 {
            let mut value = 0.0;
            for (k, centered_row) in centered.iter().enumerate() {
                for (l, tl) in t[j].iter().enumerate() {
                    value += t[i][k] * centered_row[l] * tl;
                }
            }
            covariance[i][j] = value;
            covariance[j][i] = value;
        }
    }
    if covariance.iter().flatten().any(|v| !v.is_finite())
        || (0..3).any(|i| covariance[i][i] > cfg.max_position_variance_m2)
        || (3..6).any(|i| covariance[i][i] > cfg.max_rotation_variance_rad2)
    {
        return Err("SE3 registration covariance exceeds accepted bound".into());
    }
    Ok((covariance, ratio, condition))
}
fn solve(
    scan: &[Vec3],
    grid: &Grid<'_>,
    initial: Pose3,
    cfg: &Registration3dConfig,
    budget: &mut Budget,
) -> Result<Registration3dResult, String> {
    let mut pose = initial;
    let mut iterations = 0;
    let mut converged = false;
    for _ in 0..cfg.max_iterations {
        iterations += 1;
        let p = pairs(scan, grid, pose, cfg, budget)?;
        let next = rigid_fit(scan, grid.map, &p)?;
        converged = norm(sub(next.translation, pose.translation)) <= cfg.translation_tolerance_m
            && next.rotation.angular_distance(pose.rotation) <= cfg.rotation_tolerance_rad;
        pose = next;
        if converged {
            break;
        }
    }
    if !converged {
        return Err("SE3 registration did not converge within iteration bound".into());
    }
    let p = pairs(scan, grid, pose, cfg, budget)?;
    let rms = (p.iter().map(|p| p.distance * p.distance).sum::<f64>() / p.len() as f64).sqrt();
    if !rms.is_finite() || rms > cfg.max_rms_m {
        return Err("SE3 registration residual exceeds accepted bound".into());
    }
    let (covariance, geometry_ratio, condition_number) =
        conditioning(scan, grid.map, &p, pose, rms, cfg)?;
    Ok(Registration3dResult {
        pose,
        rms_m: rms,
        inlier_fraction: p.len() as f64 / scan.len() as f64,
        inlier_count: p.len(),
        iterations,
        converged,
        covariance,
        conditioning: Registration3dConditioning {
            geometry_ratio,
            condition_number,
            neighbor_checks: budget.checks,
            ambiguity_probes: 0,
        },
    })
}
fn within_jump(pose: Pose3, initial: Pose3, cfg: &Registration3dConfig) -> bool {
    norm(sub(pose.translation, initial.translation)) <= cfg.max_translation_jump_m
        && pose.rotation.angular_distance(initial.rotation) <= cfg.max_rotation_jump_rad
}
/// Estimate the sensor-to-map rigid transform. Twelve bounded nearby seeds test
/// for competing fits. They do not prove global uniqueness or physical accuracy.
pub fn match_scan(
    scan: &[Vec3],
    map: &[Vec3],
    initial: Pose3,
    cfg: &Registration3dConfig,
) -> Result<Registration3dResult, String> {
    cfg.validate()?;
    if scan.len() < cfg.min_pairs
        || scan.len() > cfg.max_scan_points
        || map.len() < cfg.min_pairs
        || map.len() > cfg.max_map_points
        || scan.iter().chain(map).any(|p| !p.finite())
        || !initial.translation.finite()
    {
        return Err("invalid or oversized SE3 registration input".into());
    }
    let normalized = initial.rotation.normalized()?;
    if [
        initial.rotation.w - normalized.w,
        initial.rotation.x - normalized.x,
        initial.rotation.y - normalized.y,
        initial.rotation.z - normalized.z,
    ]
    .iter()
    .any(|x| x.abs() > 1e-6)
    {
        return Err("initial rotation must be a unit canonical quaternion".into());
    }
    let grid = Grid::new(map, cfg.max_correspondence_m)?;
    let mut budget = Budget {
        checks: 0,
        limit: cfg.max_neighbor_checks,
    };
    let mut fits = Vec::with_capacity(13);
    let (center, main_error) = match solve(scan, &grid, initial, cfg, &mut budget) {
        Ok(r) if within_jump(r.pose, initial, cfg) => {
            let pose = r.pose;
            fits.push(r);
            (pose, "no bounded accepted SE3 fit".to_string())
        }
        Ok(_) => (
            initial,
            "SE3 registration displacement exceeds jump bound".to_string(),
        ),
        Err(e) if e.contains("budget exhausted") => return Err(e),
        Err(e) => (initial, e),
    };
    for axis in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ] {
        for sign in [-1.0, 1.0] {
            for rotation_probe in [false, true] {
                let mut seed = center;
                if rotation_probe {
                    seed.rotation =
                        Quaternion::from_axis_angle(axis, sign * cfg.ambiguity_rotation_probe_rad)?
                            .multiply(seed.rotation);
                } else {
                    seed.translation = add(
                        seed.translation,
                        scale(axis, sign * cfg.ambiguity_translation_probe_m),
                    );
                }
                match solve(scan, &grid, seed, cfg, &mut budget) {
                    Ok(r) if within_jump(r.pose, initial, cfg) => fits.push(r),
                    Err(e) if e.contains("budget exhausted") => return Err(e),
                    _ => {}
                }
            }
        }
    }
    if fits.is_empty() {
        return Err(main_error);
    }
    fits.sort_by(|a, b| a.rms_m.total_cmp(&b.rms_m));
    let mut result = fits.remove(0);
    for r in fits {
        let distinct = norm(sub(r.pose.translation, result.pose.translation))
            > cfg.ambiguity_translation_probe_m * 0.5
            || r.pose.rotation.angular_distance(result.pose.rotation)
                > cfg.ambiguity_rotation_probe_rad * 0.5;
        if distinct && r.rms_m <= result.rms_m.max(0.001) * cfg.ambiguity_rms_ratio {
            return Err("ambiguous SE3 registration: distinct comparable local fit".into());
        }
    }
    result.conditioning.neighbor_checks = budget.checks;
    result.conditioning.ambiguity_probes = 12;
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn cloud() -> Vec<Vec3> {
        (0..180)
            .map(|i| {
                let t = i as f64;
                Vec3::new(
                    (1.73 * t).sin() * 1.3 + (t * 0.21).cos() * 0.2,
                    (t * 0.77).cos() * 0.9,
                    (t * 0.49).sin() * 0.7 + (t * 0.11).cos() * 0.1,
                )
            })
            .collect()
    }
    fn truth() -> Pose3 {
        Pose3 {
            translation: Vec3::new(0.06, -0.04, 0.03),
            rotation: Quaternion::from_axis_angle(Vec3::new(0.3, 0.7, 0.2), 0.05).unwrap(),
        }
    }
    #[test]
    fn rigid_noisy_cloud_recovers_six_dof_with_outliers() {
        let clean = cloud();
        let t = truth();
        let map: Vec<_> = clean.iter().map(|&p| t.transform(p)).collect();
        let mut scan: Vec<_> = clean
            .iter()
            .enumerate()
            .map(|(i, &p)| {
                add(
                    p,
                    Vec3::new(
                        (i as f64).sin() * 0.001,
                        (i as f64 * 0.7).cos() * 0.001,
                        (i as f64 * 0.4).sin() * 0.001,
                    ),
                )
            })
            .collect();
        scan.extend((0..20).map(|i| Vec3::new(30.0 + i as f64, -20.0, 15.0)));
        let r = match_scan(
            &scan,
            &map,
            Pose3::identity(),
            &Registration3dConfig::default(),
        )
        .unwrap();
        assert!(norm(sub(r.pose.translation, t.translation)) < 0.001);
        assert!(r.pose.rotation.angular_distance(t.rotation) < 0.001);
        assert!(r.rms_m < 0.003);
        assert_eq!(r.conditioning.ambiguity_probes, 12);
        assert!(r.conditioning.neighbor_checks <= 20_000_000);
        for i in 0..6 {
            assert!(r.covariance[i][i] > 0.0);
            for j in 0..6 {
                assert_eq!(r.covariance[i][j], r.covariance[j][i]);
            }
        }
    }
    #[test]
    fn inverse_compose_and_quaternion_sign_are_consistent() {
        let t = truth();
        let p = Vec3::new(2.0, -3.0, 4.0);
        assert!(norm(sub(t.inverse().transform(t.transform(p)), p)) < 1e-12);
        assert!(norm(sub(t.compose(t.inverse()).transform(p), p)) < 1e-12);
        let q = t.rotation;
        let neg = Quaternion {
            w: -q.w,
            x: -q.x,
            y: -q.y,
            z: -q.z,
        };
        assert!(q.angular_distance(neg) < 1e-12);
    }
    #[test]
    fn horn_fit_handles_half_turns_and_all_rotation_axes() {
        // Known correspondences isolate the closed-form rigid-fit solver from
        // ICP's intentionally local capture range. Half turns exercise negative
        // trace cases that a small-angle quaternion approximation would miss.
        let scan = cloud();
        let pairs: Vec<_> = (0..scan.len())
            .map(|i| Pair {
                source: i,
                target: i,
                distance: 0.0,
            })
            .collect();
        for axis in [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(-0.2, 0.7, 0.9),
        ] {
            for angle in [-2.7, -0.9, 0.0, 1.3, std::f64::consts::PI] {
                let target = Pose3 {
                    translation: Vec3::new(-1.2, 0.7, 2.0),
                    rotation: Quaternion::from_axis_angle(axis, angle).unwrap(),
                };
                let map: Vec<_> = scan.iter().map(|&p| target.transform(p)).collect();
                let fit = rigid_fit(&scan, &map, &pairs).unwrap();
                assert!(norm(sub(fit.translation, target.translation)) < 1e-12);
                assert!(fit.rotation.angular_distance(target.rotation) < 1e-12);
            }
        }
    }
    #[test]
    fn jacobi_eigenvectors_reconstruct_a_symmetric_information_matrix() {
        let matrix = [[6.0, 2.0, -1.0], [2.0, 4.0, 0.7], [-1.0, 0.7, 3.0]];
        let (values, vectors) = eigen(matrix).unwrap();
        for i in 0..3 {
            for j in 0..3 {
                let reconstruction: f64 = (0..3)
                    .map(|k| vectors[i][k] * values[k] * vectors[j][k])
                    .sum();
                assert!((reconstruction - matrix[i][j]).abs() < 1e-11);
                let orthogonality: f64 = (0..3).map(|k| vectors[k][i] * vectors[k][j]).sum();
                assert!((orthogonality - if i == j { 1.0 } else { 0.0 }).abs() < 1e-12);
            }
        }
    }
    #[test]
    fn centered_large_coordinate_fit_is_deterministic() {
        let scan = cloud();
        let mut t = truth();
        t.translation = add(t.translation, Vec3::new(1e8, -1e8, 1e8));
        let map: Vec<_> = scan.iter().map(|&p| t.transform(p)).collect();
        let initial = Pose3 {
            translation: Vec3::new(1e8, -1e8, 1e8),
            rotation: Quaternion::identity(),
        };
        let cfg = Registration3dConfig::default();
        let a = match_scan(&scan, &map, initial, &cfg).unwrap();
        let b = match_scan(&scan, &map, initial, &cfg).unwrap();
        assert!(norm(sub(a.pose.translation, t.translation)) < 1e-6);
        assert!(a.pose.rotation.angular_distance(t.rotation) < 1e-6);
        assert_eq!(a.rms_m.to_bits(), b.rms_m.to_bits());
        assert_eq!(a.pose.rotation, b.pose.rotation);
    }
    #[test]
    fn planes_lines_and_identical_points_do_not_establish_confidence() {
        for scan in [
            vec![Vec3::new(0.0, 0.0, 0.0); 100],
            (0..100)
                .map(|i| Vec3::new(i as f64 * 0.03, 0.0, 0.0))
                .collect(),
            (0..100)
                .map(|i| Vec3::new((i % 10) as f64 * 0.1, (i / 10) as f64 * 0.1, 0.0))
                .collect(),
        ] {
            assert!(
                match_scan(
                    &scan,
                    &scan,
                    Pose3::identity(),
                    &Registration3dConfig::default()
                )
                .is_err()
            );
        }
    }
    #[test]
    fn repeated_3d_geometry_has_competing_local_fits() {
        let scan = cloud();
        let mut map = scan.clone();
        map.extend(scan.iter().map(|&p| add(p, Vec3::new(0.4, 0.0, 0.0))));
        let cfg = Registration3dConfig {
            ambiguity_translation_probe_m: 0.4,
            ..Registration3dConfig::default()
        };
        assert!(
            match_scan(&scan, &map, Pose3::identity(), &cfg)
                .unwrap_err()
                .contains("ambiguous")
        );
    }
    #[test]
    fn invalid_inputs_work_exhaustion_and_wrong_prior_rejected() {
        let scan = cloud();
        let cfg = Registration3dConfig::default();
        assert!(
            match_scan(
                &scan,
                &scan,
                Pose3 {
                    translation: Vec3::new(30.0, 0.0, 0.0),
                    rotation: Quaternion::identity()
                },
                &cfg
            )
            .is_err()
        );
        assert!(
            match_scan(
                &vec![Vec3::new(0.0, 0.0, 0.0); 5001],
                &scan,
                Pose3::identity(),
                &cfg
            )
            .is_err()
        );
        assert!(
            match_scan(
                &scan,
                &scan,
                Pose3::identity(),
                &Registration3dConfig {
                    max_neighbor_checks: 10,
                    ..cfg.clone()
                }
            )
            .unwrap_err()
            .contains("budget exhausted")
        );
        let mut invalid = scan.clone();
        invalid[0].x = f64::NAN;
        assert!(match_scan(&invalid, &scan, Pose3::identity(), &cfg).is_err());
        assert!(
            match_scan(
                &scan,
                &scan,
                Pose3::identity(),
                &Registration3dConfig {
                    max_iterations: 51,
                    ..cfg
                }
            )
            .is_err()
        );
    }
}
