//! Optical x-right/y-down/z-forward depths in metres. Research odometry only.
//! Optional associated grayscale supports photometric fitting. No reference
//! poses, clocks supplied by wall time or driving controls.

use serde::{Deserialize, Serialize};

mod depth_weight;
pub mod fixed_map;
mod fusion_sampling;
pub use fusion_sampling::FusionSampling;
mod information;
mod motion;
pub use information::{InformationFailure, Spectrum};
pub mod multireference;
pub mod photometric;
pub use motion::{
    MotionModel, Prediction, Status as PredictionStatus, TimeWarp, Timing, WarpStatus,
};
mod step_control;
mod surface;
pub use step_control::StepPolicy;
pub mod supervision;
mod vertex_model;
mod visibility;
pub use multireference::ImageObservation;
use multireference::{Gray, gray_pyramid};
use photometric::{PhotoEquation, PhotoPair, PhotoState, Photometric};
pub use visibility::{Visibility, VisibilityStats};

type V = [f64; 3];
type M = [[f64; 3]; 3];
type H = [[f64; 6]; 6];
const MIN_Z: f64 = 0.3;
const MAX_Z: f64 = 5.0;
const MIN_SUPPORT: usize = 200;
const AGE: f64 = 0.2;

/// Point splats preserve the original model. Tangent planes compensate for
/// rounding a measured surface point onto a different camera ray.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Reprojection {
    #[default]
    Point,
    TangentPlane,
}

/// Reciprocal matching rejects a target that maps back to another source pixel.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Correspondence {
    #[default]
    Forward,
    Reciprocal,
    Bilinear,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Residual {
    #[default]
    TargetPlane,
    SymmetricPlanes,
    /// Average both signed plane distances before robust squaring. Its rotation
    /// derivative includes the moving source normal.
    SymmetricMidpoint,
    /// Sum two independently projected correspondence sets. Each direction
    /// must have its own complete geometric support.
    BidirectionalMidpoint,
    /// Relative axial-depth variance under constant disparity noise, bounded
    /// by a one-metre floor. This does not supply calibrated uncertainty.
    DepthVariancePlane,
    /// Use the same relative variance to standardize the robust residual as
    /// well as weighting its quadratic region. Raw RMS admission is unchanged.
    DepthWhitenedPlane,
}

/// An inverse-depth plane is planar in pixel coordinates, including on tilted
/// surfaces. Native-resolution planar patches can reduce noisy tangents without
/// discarding the original support on curved surfaces or at coarse scales.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum NormalEstimation {
    #[default]
    CentralDifference,
    InverseDepthPlane,
}

/// Depth re-creates XY on rounded pixel rays; vertices preserve measured XYZ
/// through model fusion, pyramid reduction and reference-view changes.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ModelGeometry {
    #[default]
    Depth,
    Vertices,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct TrackerConfig {
    pub visibility: Visibility,
    pub fusion_sampling: FusionSampling,
    pub reprojection: Reprojection,
    pub correspondence: Correspondence,
    pub residual: Residual,
    pub normals: NormalEstimation,
    pub geometry: ModelGeometry,
    pub photometric: Photometric,
    pub step_control: StepPolicy,
    pub motion_model: MotionModel,
    pub timing: Timing,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Camera {
    pub width: usize,
    pub height: usize,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}
impl Camera {
    pub fn validate(self) -> Result<(), String> {
        if !(32..=640).contains(&self.width)
            || !(32..=480).contains(&self.height)
            || !self.width.is_multiple_of(4)
            || !self.height.is_multiple_of(4)
            || ![self.fx, self.fy, self.cx, self.cy]
                .iter()
                .all(|v| v.is_finite())
            || !(20.0..=5000.0).contains(&self.fx)
            || !(20.0..=5000.0).contains(&self.fy)
            || !(0.0..self.width as f64).contains(&self.cx)
            || !(0.0..self.height as f64).contains(&self.cy)
        {
            return Err("invalid bounded pinhole camera".into());
        }
        Ok(())
    }
    fn scaled(self, scale: usize) -> Self {
        let s = scale as f64;
        Self {
            width: self.width / scale,
            height: self.height / scale,
            fx: self.fx / s,
            fy: self.fy / s,
            cx: (self.cx + 0.5) / s - 0.5,
            cy: (self.cy + 0.5) / s - 0.5,
        }
    }
    fn point(self, i: usize, z: f64) -> V {
        [
            ((i % self.width) as f64 - self.cx) * z / self.fx,
            ((i / self.width) as f64 - self.cy) * z / self.fy,
            z,
        ]
    }
    fn project(self, p: V, interior: bool) -> Option<usize> {
        if p[2] <= MIN_Z || !p.iter().all(|v| v.is_finite()) {
            return None;
        }
        let x = (self.fx * p[0] / p[2] + self.cx).round_ties_even();
        let y = (self.fy * p[1] / p[2] + self.cy).round_ties_even();
        let border = usize::from(interior);
        if x >= border as f64
            && x < (self.width - border) as f64
            && y >= border as f64
            && y < (self.height - border) as f64
        {
            Some(y as usize * self.width + x as usize)
        } else {
            None
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Pose {
    pub rotation: M,
    pub translation: V,
}
impl Pose {
    pub fn identity() -> Self {
        Self {
            rotation: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            translation: [0.; 3],
        }
    }
    pub fn rotate(self, v: V) -> V {
        mv(self.rotation, v)
    }
    pub fn transform(self, v: V) -> V {
        add(self.rotate(v), self.translation)
    }
    pub fn compose(self, b: Self) -> Self {
        Self {
            rotation: mm(self.rotation, b.rotation),
            translation: self.transform(b.translation),
        }
    }
    pub fn inverse(self) -> Self {
        let rotation = transpose(self.rotation);
        Self {
            rotation,
            translation: mul(mv(rotation, self.translation), -1.),
        }
    }
    fn increment(self, d: [f64; 6]) -> Self {
        let w = [d[3], d[4], d[5]];
        let angle = norm(w);
        let k = [[0., -w[2], w[1]], [w[2], 0., -w[0]], [-w[1], w[0], 0.]];
        let k2 = mm(k, k);
        let (a, b) = if angle < 1e-12 {
            (1., 0.)
        } else {
            (angle.sin() / angle, (1. - angle.cos()) / (angle * angle))
        };
        let mut rotation = Self::identity().rotation;
        for i in 0..3 {
            for j in 0..3 {
                rotation[i][j] += a * k[i][j] + b * k2[i][j];
            }
        }
        Self {
            rotation,
            translation: [d[0], d[1], d[2]],
        }
        .compose(self)
    }
    fn angle(self) -> f64 {
        ((self.rotation[0][0] + self.rotation[1][1] + self.rotation[2][2] - 1.) / 2.)
            .clamp(-1., 1.)
            .acos()
    }
    fn finite_rigid(self) -> bool {
        let a = mm(transpose(self.rotation), self.rotation);
        self.translation.iter().all(|v| v.is_finite())
            && a.iter().enumerate().all(|(i, row)| {
                row.iter()
                    .enumerate()
                    .all(|(j, v)| v.is_finite() && (v - f64::from(i == j)).abs() < 1e-8)
            })
            && (dot(self.rotation[0], cross(self.rotation[1], self.rotation[2])) - 1.).abs() < 1e-8
    }
}
fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn mul(a: V, b: f64) -> V {
    [a[0] * b, a[1] * b, a[2] * b]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: V) -> f64 {
    dot(a, a).sqrt()
}
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn transpose(a: M) -> M {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}
fn mv(a: M, b: V) -> V {
    a.map(|row| dot(row, b))
}
fn mm(a: M, b: M) -> M {
    let bt = transpose(b);
    std::array::from_fn(|i| std::array::from_fn(|j| dot(a[i], bt[j])))
}

#[derive(Clone)]
struct Level {
    camera: Camera,
    vertices: Vec<V>,
    normals: Vec<V>,
}
impl Level {
    fn bilinear(&self, query: V) -> Option<(V, V)> {
        if query[2] <= MIN_Z || !query.iter().all(|v| v.is_finite()) {
            return None;
        }
        let c = self.camera;
        let u = c.fx * query[0] / query[2] + c.cx;
        let v = c.fy * query[1] / query[2] + c.cy;
        let x = u.floor();
        let y = v.floor();
        if x < 1.
            || x >= (c.width - 2) as f64
            || y < 1.
            || y >= (c.height - 2) as f64
            || !u.is_finite()
            || !v.is_finite()
        {
            return None;
        }
        let i = y as usize * c.width + x as usize;
        let indices = [i, i + 1, i + c.width, i + c.width + 1];
        let mut low: f64 = MAX_Z;
        let mut high: f64 = MIN_Z;
        for j in indices {
            let z = self.vertices[j][2];
            if z <= MIN_Z || z >= MAX_Z || dot(self.normals[j], self.normals[j]) <= 0.25 {
                return None;
            }
            low = low.min(z);
            high = high.max(z);
        }
        if high - low > 0.05 + 0.01 * low {
            return None;
        }
        let a = u - x;
        let b = v - y;
        let weights = [(1. - a) * (1. - b), a * (1. - b), (1. - a) * b, a * b];
        let mut point = [0.; 3];
        let mut normal = [0.; 3];
        for (j, w) in indices.into_iter().zip(weights) {
            point = add(point, mul(self.vertices[j], w));
            normal = add(normal, mul(self.normals[j], w));
        }
        let length = norm(normal);
        (length > 0.5).then(|| (point, mul(normal, 1. / length)))
    }
}
fn pyramid(camera: Camera, raw: &[f64]) -> Vec<Level> {
    pyramid_with_normals(camera, raw, NormalEstimation::CentralDifference)
}

fn pyramid_with_normals(camera: Camera, raw: &[f64], policy: NormalEstimation) -> Vec<Level> {
    let mut depth = raw.to_vec();
    let mut levels = Vec::with_capacity(3);
    for s in [1, 2, 4] {
        let c = camera.scaled(s);
        if s != 1 {
            let old_width = c.width * 2;
            depth = (0..c.width * c.height)
                .map(|i| {
                    let k = i / c.width * 2 * old_width + i % c.width * 2;
                    let values = [
                        depth[k],
                        depth[k + 1],
                        depth[k + old_width],
                        depth[k + old_width + 1],
                    ];
                    let lo = values.iter().copied().fold(f64::INFINITY, f64::min);
                    let hi = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                    if lo < MIN_Z || hi > MAX_Z || hi - lo > 0.05 + 0.01 * lo {
                        0.
                    } else {
                        values.iter().sum::<f64>() / 4.
                    }
                })
                .collect();
        }
        let vertices: Vec<V> = depth
            .iter()
            .enumerate()
            .map(|(i, &z)| c.point(i, z))
            .collect();
        let mut normals = vec![[0.; 3]; depth.len()];
        for y in 1..c.height - 1 {
            for x in 1..c.width - 1 {
                let i = y * c.width + x;
                let z = depth[i];
                if z <= MIN_Z || z >= MAX_Z {
                    continue;
                }
                let neighbours = [i + 1, i - 1, i + c.width, i - c.width];
                if neighbours
                    .iter()
                    .any(|&j| depth[j] <= MIN_Z || (depth[j] - z).abs() >= 0.05 + 0.01 * z)
                {
                    continue;
                }
                let normal = cross(
                    sub(vertices[i + 1], vertices[i - 1]),
                    sub(vertices[i + c.width], vertices[i - c.width]),
                );
                let size = norm(normal);
                if size > 1e-10 {
                    normals[i] = mul(normal, 1. / size);
                }
            }
        }
        if policy == NormalEstimation::InverseDepthPlane && s == 1 {
            let planes = inverse_depth_normals(c, &depth);
            for (original, plane) in normals.iter_mut().zip(planes) {
                if dot(plane, plane) > 0.25 {
                    *original = plane;
                }
            }
        }
        levels.push(Level {
            camera: c,
            vertices,
            normals,
        });
    }
    levels.reverse();
    levels
}

fn inverse_depth_normals(camera: Camera, depth: &[f64]) -> Vec<V> {
    let mut normals = vec![[0.; 3]; depth.len()];
    for y in 2..camera.height - 2 {
        'pixel: for x in 2..camera.width - 2 {
            let center = depth[y * camera.width + x];
            let mut coefficients = [0.; 3];
            for dy in -2_isize..=2 {
                for dx in -2_isize..=2 {
                    let z = depth
                        [(y as isize + dy) as usize * camera.width + (x as isize + dx) as usize];
                    if z <= MIN_Z || z >= MAX_Z || (z - center).abs() >= 0.05 + 0.01 * center {
                        continue 'pixel;
                    }
                    coefficients[0] += dx as f64 / z;
                    coefficients[1] += dy as f64 / z;
                    coefficients[2] += 1. / z;
                }
            }
            // Orthogonal uniform 5x5 design: sum(du^2)=sum(dv^2)=50.
            let [a, b, c] = [
                coefficients[0] / 50.,
                coefficients[1] / 50.,
                coefficients[2] / 25.,
            ];
            // A plane is only used when it explains the observed patch within
            // 2 mm in axial depth. Curvature and unsupported patches keep their
            // original central-difference normal in pyramid_with_normals.
            let mut squared = 0.;
            for dy in -2_isize..=2 {
                for dx in -2_isize..=2 {
                    let z = depth
                        [(y as isize + dy) as usize * camera.width + (x as isize + dx) as usize];
                    let predicted = 1. / (a * dx as f64 + b * dy as f64 + c);
                    squared += (predicted - z).powi(2);
                }
            }
            if squared > 25. * 0.002_f64.powi(2) {
                continue;
            }
            let n = [
                camera.fx * a,
                camera.fy * b,
                c - (x as f64 - camera.cx) * a - (y as f64 - camera.cy) * b,
            ];
            let length = norm(n);
            if length > 1e-10 {
                normals[y * camera.width + x] = mul(n, 1. / length);
            }
        }
    }
    normals
}

#[derive(Clone, Debug, Serialize)]
pub struct Equation {
    pub count: usize,
    pub rms_m: f64,
    pub h: H,
    pub g: [f64; 6],
    #[serde(skip_serializing_if = "Option::is_none")]
    pub direction_counts: Option<[usize; 2]>,
}
impl Equation {
    fn supported(&self) -> bool {
        self.count >= MIN_SUPPORT
            && self.direction_counts.is_none_or(|counts| {
                counts.iter().all(|&n| n >= MIN_SUPPORT)
                    && counts.iter().sum::<usize>() == self.count
            })
    }
}
fn equation(
    target: &Level,
    source: &Level,
    pose: Pose,
    scale: usize,
    correspondence: Correspondence,
    residual: Residual,
) -> Equation {
    let mut e = Equation {
        count: 0,
        rms_m: 0.,
        h: [[0.; 6]; 6],
        g: [0.; 6],
        direction_counts: None,
    };
    let stride = if scale == 1 { 2 } else { 1 };
    let mut squared = 0.;
    let inverse = pose.inverse();
    for y in (0..source.camera.height).step_by(stride) {
        for x in (0..source.camera.width).step_by(stride) {
            let i = y * source.camera.width + x;
            if dot(source.normals[i], source.normals[i]) <= 0.25 {
                continue;
            }
            let q = pose.transform(source.vertices[i]);
            let matched = if correspondence == Correspondence::Bilinear {
                target.bilinear(q)
            } else {
                target
                    .camera
                    .project(q, true)
                    .map(|j| (target.vertices[j], target.normals[j]))
            };
            let Some((target_point, n)) = matched else {
                continue;
            };
            if dot(n, n) <= 0.25
                || norm(sub(q, target_point)) >= if scale == 4 { 0.12 } else { 0.08 }
                || dot(pose.rotate(source.normals[i]), n) <= 0.8
            {
                continue;
            }
            let r = dot(n, sub(q, target_point));
            if correspondence == Correspondence::Reciprocal
                && source
                    .camera
                    .project(inverse.transform(target_point), false)
                    != Some(i)
            {
                continue;
            }
            if matches!(
                residual,
                Residual::SymmetricMidpoint | Residual::BidirectionalMidpoint
            ) {
                let (r, j) = surface::midpoint(q, target_point, pose.rotate(source.normals[i]), n);
                add_residual(&mut e, j, r, 1.);
                squared += r * r;
            } else if residual == Residual::SymmetricPlanes {
                let source_normal = pose.rotate(source.normals[i]);
                let source_r = dot(source_normal, sub(q, target_point));
                add_plane(&mut e, n, q, r, 0.5);
                // Rotation of the source normal leaves the target lever arm.
                add_plane(&mut e, source_normal, target_point, source_r, 0.5);
                squared += 0.5 * (r * r + source_r * source_r);
            } else {
                let coefficient = if residual == Residual::DepthVariancePlane {
                    depth_weight::relative(source.vertices[i][2], target_point[2])
                } else if residual == Residual::DepthWhitenedPlane {
                    depth_weight::robust_coefficient(source.vertices[i][2], target_point[2], r)
                } else {
                    1.
                };
                add_plane(&mut e, n, q, r, coefficient);
                squared += r * r;
            }
            e.count += 1;
        }
    }
    if residual == Residual::BidirectionalMidpoint {
        let forward = e.count;
        squared += surface::reverse(&mut e, target, source, pose, scale);
        e.direction_counts = Some([forward, e.count - forward]);
    }
    e.rms_m = if e.count == 0 {
        0.
    } else {
        (squared / e.count as f64).sqrt()
    };
    e
}

fn add_plane(e: &mut Equation, normal: V, pivot: V, r: f64, coefficient: f64) {
    let rotation = cross(pivot, normal);
    let j = [
        normal[0],
        normal[1],
        normal[2],
        rotation[0],
        rotation[1],
        rotation[2],
    ];
    add_residual(e, j, r, coefficient);
}

fn add_residual(e: &mut Equation, j: [f64; 6], r: f64, coefficient: f64) {
    let weight = (0.01 / r.abs().max(1e-15)).min(1.) * coefficient;
    for a in 0..6 {
        e.g[a] += j[a] * r * weight;
        for b in 0..6 {
            e.h[a][b] += j[a] * j[b] * weight;
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct Iteration {
    pub scale: usize,
    pub before: Pose,
    pub equation: Equation,
    pub scaled_condition: f64,
    pub delta: [f64; 6],
    pub after: Pose,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub photometric: Option<PhotoEquation>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step_control: Option<step_control::StepControl>,
}
struct FitPolicy {
    correspondence: Correspondence,
    residual: Residual,
    safeguarded: bool,
}
fn fit(
    target: &[Level],
    source: &[Level],
    initial: Pose,
    trace: &mut Vec<Iteration>,
    information_failure: &mut Option<InformationFailure>,
    policy: FitPolicy,
    photo: Option<&PhotoPair<'_>>,
) -> Result<(Pose, Equation, Option<PhotoEquation>), String> {
    let mut pose = initial;
    for ((t, s), (scale, cap)) in target.iter().zip(source).zip([(4, 10), (2, 7), (1, 5)]) {
        for _ in 0..cap {
            let e = equation(t, s, pose, scale, policy.correspondence, policy.residual);
            if !e.supported() {
                return Err("insufficient geometric support".into());
            }
            let p = photo.map(|pair| pair.equation(t, s, pose, scale));
            information::check(false, scale, pose, &e, p.as_ref(), information_failure)?;
            let (h, g) = photometric::combine(&e, p.as_ref());
            let (d, condition) = solve_normal(h, g, 1e6)?;
            if norm([d[0], d[1], d[2]]) > 0.1 || norm([d[3], d[4], d[5]]) > 0.15 {
                return Err("oversized geometric increment".into());
            }
            let step_control = if policy.safeguarded {
                let witnesses = step_control::Witnesses::new(t, s, pose, scale, photo, p.as_ref());
                if witnesses.counts()
                    != (
                        e.count,
                        p.as_ref().filter(|p| p.used).map_or(0, |p| p.count),
                    )
                {
                    return Err("fixed-support witness mismatch".into());
                }
                Some(witnesses.select(d)?)
            } else {
                None
            };
            let applied_scale = step_control.as_ref().map_or(1., |s| s.applied_scale);
            let next = if applied_scale == 0. {
                pose
            } else {
                pose.increment(d.map(|v| v * applied_scale))
            };
            if !next.finite_rigid() {
                return Err("nonrigid geometric result".into());
            }
            trace.push(Iteration {
                scale,
                before: pose,
                equation: e,
                scaled_condition: condition,
                delta: d,
                after: next,
                photometric: p,
                step_control,
            });
            pose = next;
            if applied_scale == 0.
                || (norm([d[0], d[1], d[2]]) < 1e-5 && norm([d[3], d[4], d[5]]) < 1e-5)
            {
                break;
            }
        }
    }
    // Admission uses correspondences at the returned pose, not the previous iterate.
    let final_e = equation(
        &target[2],
        &source[2],
        pose,
        1,
        policy.correspondence,
        policy.residual,
    );
    if !final_e.supported() || final_e.rms_m > 0.03 || norm(pose.translation) > 0.5 {
        return Err("final geometric residual, support or displacement gate".into());
    }
    let final_photo = photo.map(|pair| pair.equation(&target[2], &source[2], pose, 1));
    information::check(
        true,
        1,
        pose,
        &final_e,
        final_photo.as_ref(),
        information_failure,
    )?;
    Ok((pose, final_e, final_photo))
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct RasterStats {
    pub source_valid: usize,
    pub projected: usize,
    pub selected: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub tangent_adjusted: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub surface_merged: usize,
}
fn is_zero(value: &usize) -> bool {
    *value == 0
}
#[cfg(test)]
fn raster(
    camera: Camera,
    depth: &[f64],
    weights: &[u8],
    pose: Pose,
) -> (Vec<f64>, Vec<u8>, RasterStats) {
    raster_with_tangents(camera, depth, weights, pose, None)
}
fn raster_with_tangents(
    camera: Camera,
    depth: &[f64],
    weights: &[u8],
    pose: Pose,
    normals: Option<&[V]>,
) -> (Vec<f64>, Vec<u8>, RasterStats) {
    let mut output = vec![0.; depth.len()];
    let mut support = vec![0; depth.len()];
    let mut stats = RasterStats::default();
    for (i, &z) in depth.iter().enumerate() {
        if z <= MIN_Z || z >= MAX_Z {
            continue;
        }
        stats.source_valid += 1;
        let p = pose.transform(camera.point(i, z));
        if p[2] >= MAX_Z {
            continue;
        }
        let Some(j) = camera.project(p, false) else {
            continue;
        };
        let mut projected_z = p[2];
        if let Some(normals) = normals
            && dot(normals[i], normals[i]) > 0.25
        {
            let normal = pose.rotate(normals[i]);
            let ray = camera.point(j, 1.);
            let denominator = dot(normal, ray);
            if denominator.abs() > 0.1 * norm(ray) {
                let corrected = dot(normal, p) / denominator;
                // A local tangent is only an approximation. Keep the point
                // splat if grazing, unsupported or outside the fusion band.
                if corrected.is_finite()
                    && corrected > MIN_Z
                    && corrected < MAX_Z
                    && (corrected - p[2]).abs() < 0.03 + 0.01 * p[2]
                {
                    projected_z = corrected;
                    stats.tangent_adjusted += 1;
                }
            }
        }
        stats.projected += 1;
        if output[j] == 0. || projected_z < output[j] {
            if output[j] == 0. {
                stats.selected += 1;
            }
            output[j] = projected_z;
            support[j] = weights[i];
        }
        // Strictly smaller Z only: row-major traversal keeps the first exact tie.
    }
    (output, support, stats)
}
#[derive(Clone, Debug, Default, Serialize)]
pub struct FusionStats {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visibility: Option<VisibilityStats>,
    pub warp: Option<RasterStats>,
    pub measurement: RasterStats,
    pub filled: usize,
    pub averaged: usize,
    pub replaced: usize,
    pub ignored: usize,
}
#[cfg(test)]
fn fuse(
    camera: Camera,
    model: &mut [f64],
    weights: &mut [u8],
    depth: &[f64],
    pose: Pose,
) -> FusionStats {
    fuse_with_tangents(camera, model, weights, depth, pose, None)
}
fn fuse_with_tangents(
    camera: Camera,
    model: &mut [f64],
    weights: &mut [u8],
    depth: &[f64],
    pose: Pose,
    normals: Option<&[V]>,
) -> FusionStats {
    let (new, present, measurement) =
        raster_with_tangents(camera, depth, &vec![1; depth.len()], pose, normals);
    fuse_raster(model, weights, &new, &present, measurement)
}
fn fuse_raster(
    model: &mut [f64],
    weights: &mut [u8],
    new: &[f64],
    present: &[u8],
    measurement: RasterStats,
) -> FusionStats {
    let mut stats = FusionStats {
        measurement,
        ..FusionStats::default()
    };
    for i in 0..model.len() {
        if present[i] == 0 {
            continue;
        }
        let old = model[i];
        if weights[i] == 0 {
            model[i] = new[i];
            weights[i] = 1;
            stats.filled += 1;
        } else if (new[i] - old).abs() < 0.03 + 0.01 * old {
            let w = f64::from(weights[i].min(16));
            model[i] = (w * old + new[i]) / (w + 1.);
            weights[i] = (weights[i] + 1).min(16);
            stats.averaged += 1;
        } else if new[i] < old - 0.03 - 0.01 * old {
            model[i] = new[i];
            weights[i] = 1;
            stats.replaced += 1;
        } else {
            stats.ignored += 1;
        }
    }
    stats
}

#[derive(Debug, Serialize)]
pub struct Update {
    pub timestamp: f64,
    pub accepted: bool,
    pub initialized: bool,
    pub lost: bool,
    pub rejection: Option<String>,
    pub first_from_current: Option<Pose>,
    pub last_accepted_timestamp: Option<f64>,
    pub keyframe_replaced: bool,
    pub iterations: Vec<Iteration>,
    pub final_equation: Option<Equation>,
    pub fusion: Option<FusionStats>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub photometric: Option<PhotoState>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_photometric: Option<PhotoEquation>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub prediction: Option<Prediction>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub information_failure: Option<InformationFailure>,
}
struct Model {
    depth: Vec<f64>,
    weights: Vec<u8>,
    pyramid: Vec<Level>,
    key_root: Pose,
    initial: Pose,
    gray: Option<Vec<Gray>>,
    rgb_timestamp: Option<f64>,
    phase: f64,
    depth_timestamp: f64,
    interval: Option<motion::Interval>,
}
pub struct Tracker {
    camera: Camera,
    visibility: Visibility,
    fusion_sampling: FusionSampling,
    reprojection: Reprojection,
    correspondence: Correspondence,
    residual: Residual,
    normals: NormalEstimation,
    geometry: ModelGeometry,
    photometric: Photometric,
    step_policy: StepPolicy,
    motion_model: MotionModel,
    timing: Timing,
    history: motion::History,
    last_rgb: Option<f64>,
    model: Option<Model>,
    last_observed: Option<f64>,
    last_accepted: Option<f64>,
    lost: bool,
}
impl Tracker {
    pub fn new(camera: Camera) -> Result<Self, String> {
        Self::with_reprojection(camera, Reprojection::default())
    }
    /// Select the model reprojection explicitly; all fitting and clock guards
    /// are shared by both policies.
    pub fn with_reprojection(camera: Camera, reprojection: Reprojection) -> Result<Self, String> {
        Self::with_options(camera, reprojection, Correspondence::default())
    }
    pub fn with_options(
        camera: Camera,
        reprojection: Reprojection,
        correspondence: Correspondence,
    ) -> Result<Self, String> {
        Self::with_config(
            camera,
            TrackerConfig {
                visibility: Visibility::default(),
                fusion_sampling: FusionSampling::default(),
                reprojection,
                correspondence,
                residual: Residual::default(),
                normals: NormalEstimation::default(),
                geometry: ModelGeometry::default(),
                photometric: Photometric::default(),
                step_control: StepPolicy::default(),
                motion_model: MotionModel::default(),
                timing: Timing::default(),
            },
        )
    }
    /// Configure measured residual/model policies with shared clock and fault guards.
    pub fn with_config(camera: Camera, config: TrackerConfig) -> Result<Self, String> {
        camera.validate()?;
        if config.visibility == Visibility::FreeSpaceDecay
            && (config.fusion_sampling != FusionSampling::Nearest
                || config.geometry != ModelGeometry::Depth
                || config.reprojection != Reprojection::Point
                || config.correspondence != Correspondence::Forward
                || config.residual != Residual::TargetPlane
                || config.normals != NormalEstimation::CentralDifference
                || config.photometric != Photometric::Disabled
                || config.motion_model != MotionModel::Stationary
                || config.step_control != StepPolicy::Disabled
                || config.timing != Timing::Disabled)
        {
            return Err("free_space_decay requires original stationary nearest-depth tracking without other experimental policies".into());
        }
        if config.fusion_sampling == FusionSampling::VisibleMean
            && (config.geometry != ModelGeometry::Depth
                || config.reprojection != Reprojection::Point
                || config.correspondence != Correspondence::Forward
                || config.residual != Residual::TargetPlane
                || config.normals != NormalEstimation::CentralDifference
                || config.photometric != Photometric::Disabled
                || config.motion_model != MotionModel::Stationary
                || config.step_control != StepPolicy::Disabled
                || config.timing != Timing::Disabled)
        {
            return Err("visible_mean requires original stationary depth tracking without other experimental policies".into());
        }
        if matches!(
            config.residual,
            Residual::SymmetricMidpoint
                | Residual::BidirectionalMidpoint
                | Residual::DepthVariancePlane
                | Residual::DepthWhitenedPlane
        ) && (config.geometry != ModelGeometry::Depth
            || config.reprojection != Reprojection::Point
            || config.correspondence != Correspondence::Forward
            || config.normals != NormalEstimation::CentralDifference
            || config.photometric != Photometric::Disabled)
        {
            return Err("experimental plane residuals require depth/point/forward/central_difference without photometric tracking".into());
        }
        if config.timing == Timing::PreviousMotion
            && (config.photometric != Photometric::Keyframe
                || config.motion_model != MotionModel::ConstantVelocity
                || config.step_control != StepPolicy::Disabled)
        {
            return Err("previous-motion timing requires keyframe photometric, constant_velocity and disabled step control".into());
        }
        if config.step_control == StepPolicy::FixedSupport
            && config.photometric != Photometric::Keyframe
        {
            return Err("fixed-support step control requires keyframe photometric tracking".into());
        }
        if config.photometric == Photometric::Keyframe
            && (config.geometry != ModelGeometry::Depth
                || config.reprojection != Reprojection::Point
                || config.correspondence != Correspondence::Forward
                || config.residual != Residual::TargetPlane
                || config.normals != NormalEstimation::CentralDifference)
        {
            return Err("photometric tracking requires the original depth/point/forward/target_plane/central_difference policies".into());
        }
        if config.geometry == ModelGeometry::Vertices
            && (config.reprojection != Reprojection::Point
                || config.correspondence != Correspondence::Forward
                || config.residual != Residual::TargetPlane
                || config.normals != NormalEstimation::CentralDifference)
        {
            return Err(
                "vertex geometry requires point/forward/target_plane/central_difference".into(),
            );
        }
        Ok(Self {
            camera,
            visibility: config.visibility,
            fusion_sampling: config.fusion_sampling,
            reprojection: config.reprojection,
            correspondence: config.correspondence,
            residual: config.residual,
            normals: config.normals,
            geometry: config.geometry,
            photometric: config.photometric,
            step_policy: config.step_control,
            motion_model: config.motion_model,
            timing: config.timing,
            history: motion::History::default(),
            last_rgb: None,
            model: None,
            last_observed: None,
            last_accepted: None,
            lost: false,
        })
    }
    /// Rejected fits do not mutate pose/model/accepted time. Valid acquisition
    /// clocks advance even for malformed depth, so expiry cannot be hidden.
    pub fn step(&mut self, timestamp: f64, depth: &[f64]) -> Update {
        self.step_with_image(timestamp, depth, None)
    }

    pub fn step_with_image(
        &mut self,
        timestamp: f64,
        depth: &[f64],
        image: Option<ImageObservation<'_>>,
    ) -> Update {
        let mut out = Update {
            timestamp,
            accepted: false,
            initialized: self.model.is_some(),
            lost: self.lost,
            rejection: None,
            first_from_current: None,
            last_accepted_timestamp: self.last_accepted,
            keyframe_replaced: false,
            iterations: Vec::new(),
            final_equation: None,
            fusion: None,
            photometric: None,
            final_photometric: None,
            prediction: None,
            information_failure: None,
        };
        let result = (|| -> Result<(), String> {
            if !timestamp.is_finite()
                || !(0.0..=1e12).contains(&timestamp)
                || self.last_observed.is_some_and(|last| timestamp <= last)
            {
                return Err("invalid, duplicate or regressing acquisition clock".into());
            }
            self.last_observed = Some(timestamp);
            if self
                .last_accepted
                .is_some_and(|last| timestamp - last > AGE)
            {
                self.lost = true;
            }
            if self.lost {
                return Err("accepted-pose expiry latched; no reset".into());
            }
            let (gray, rgb_timestamp, phase, fresh) = match (self.photometric, image) {
                (Photometric::Disabled, None) => (None, None, 0., false),
                (Photometric::Disabled, Some(_)) => {
                    return Err("depth mode expects no RGB image".into());
                }
                (Photometric::Keyframe, None) => {
                    return Err("photometric tracking requires an associated image".into());
                }
                (Photometric::Keyframe, Some(image)) => {
                    if !image.timestamp.is_finite()
                        || !(0.0..=1e12).contains(&image.timestamp)
                        || (image.timestamp - timestamp).abs() > 0.02
                        || self.last_rgb.is_some_and(|t| image.timestamp < t)
                    {
                        return Err("invalid, regressing or unassociated RGB clock".into());
                    }
                    let fresh = self.last_rgb.is_none_or(|t| image.timestamp > t);
                    self.last_rgb = Some(image.timestamp);
                    if image.gray.len() != self.camera.width * self.camera.height {
                        return Err("invalid grayscale image size".into());
                    }
                    out.photometric = Some(PhotoState {
                        source_timestamp: image.timestamp,
                        reference_timestamp: self.model.as_ref().and_then(|m| m.rgb_timestamp),
                        phase_qualified: self.model.as_ref().is_some_and(|m| {
                            m.gray.is_some()
                                && ((image.timestamp - timestamp) - m.phase).abs() <= 0.005
                        }),
                        fresh,
                        timing: None,
                    });
                    if self.timing == Timing::PreviousMotion {
                        let source =
                            motion::time_warp(timestamp, image.timestamp, self.history.interval());
                        let reference = self.model.as_ref().and_then(|m| {
                            m.rgb_timestamp
                                .map(|rgb| motion::time_warp(m.depth_timestamp, rgb, m.interval))
                        });
                        let state = out.photometric.as_mut().expect("image state");
                        state.phase_qualified = source.image_from_depth.is_some()
                            && reference
                                .as_ref()
                                .is_some_and(|r| r.image_from_depth.is_some())
                            && self.model.as_ref().is_some_and(|m| m.gray.is_some());
                        state.timing = Some(photometric::TimingState { source, reference });
                    }
                    (
                        Some(gray_pyramid(self.camera, image.gray)),
                        Some(image.timestamp),
                        image.timestamp - timestamp,
                        fresh,
                    )
                }
            };
            if depth.len() != self.camera.width * self.camera.height
                || depth
                    .iter()
                    .any(|v| !v.is_finite() || !(0.0..=100.).contains(v))
            {
                return Err("invalid bounded depth image".into());
            }
            let current = if self.geometry == ModelGeometry::Vertices {
                vertex_model::pyramid(
                    self.camera,
                    depth
                        .iter()
                        .enumerate()
                        .map(|(i, &z)| self.camera.point(i, z))
                        .collect(),
                )
            } else {
                pyramid_with_normals(self.camera, depth, self.normals)
            };
            if self.model.is_none() {
                if current[0]
                    .normals
                    .iter()
                    .filter(|n| dot(**n, **n) > 0.25)
                    .count()
                    < MIN_SUPPORT
                {
                    return Err("insufficient initialization surface".into());
                }
                self.model = Some(Model {
                    depth: depth.to_vec(),
                    weights: depth.iter().map(|&z| u8::from(z > MIN_Z)).collect(),
                    pyramid: current,
                    key_root: Pose::identity(),
                    initial: Pose::identity(),
                    gray,
                    rgb_timestamp,
                    phase,
                    depth_timestamp: timestamp,
                    interval: None,
                });
                out.first_from_current = Some(Pose::identity());
                out.initialized = true;
            } else {
                let old = self.model.as_ref().expect("model checked");
                let mut initial = old.initial;
                if self.motion_model == MotionModel::ConstantVelocity {
                    let prediction = self.history.predict(timestamp);
                    if let Some(increment) = prediction.latest_from_prediction {
                        // Stay in the maintained reference frame. Repeated
                        // key.inverse()*global_pose round trips amplify the
                        // tiny nonorthogonality of floating-point rotations.
                        initial = old.initial.compose(increment);
                    }
                    out.prediction = Some(prediction);
                }
                let photo = match (&old.gray, &gray) {
                    (Some(target), Some(source)) => Some(PhotoPair {
                        target,
                        source,
                        enabled: fresh
                            && out.photometric.as_ref().is_some_and(|s| s.phase_qualified),
                        warps: out
                            .photometric
                            .as_ref()
                            .and_then(|s| s.timing.as_ref())
                            .and_then(|t| {
                                Some((
                                    t.reference.as_ref()?.image_from_depth?,
                                    t.source.image_from_depth?,
                                ))
                            }),
                    }),
                    _ => None,
                };
                let (relative, final_e, final_photo) = fit(
                    &old.pyramid,
                    &current,
                    initial,
                    &mut out.iterations,
                    &mut out.information_failure,
                    FitPolicy {
                        correspondence: self.correspondence,
                        residual: self.residual,
                        safeguarded: self.step_policy == StepPolicy::FixedSupport,
                    },
                    photo.as_ref(),
                )?;
                let root = old.key_root.compose(relative);
                if !root.finite_rigid() {
                    return Err("nonrigid composed root".into());
                }
                let replace = norm(relative.translation) > 0.1 || relative.angle() > 0.1;
                let (map, weights, next_pyramid, fusion) = if self.geometry
                    == ModelGeometry::Vertices
                {
                    let (mut points, mut weights, warp) = if replace {
                        let (points, weights, stats) = vertex_model::raster(
                            self.camera,
                            &old.pyramid[2].vertices,
                            &old.weights,
                            relative.inverse(),
                        );
                        (points, weights, Some(stats))
                    } else {
                        (old.pyramid[2].vertices.clone(), old.weights.clone(), None)
                    };
                    let mut fusion = vertex_model::fuse(
                        self.camera,
                        &mut points,
                        &mut weights,
                        &current[2].vertices,
                        if replace { Pose::identity() } else { relative },
                    );
                    fusion.warp = warp;
                    let map = points.iter().map(|p| p[2]).collect();
                    (
                        map,
                        weights,
                        vertex_model::pyramid(self.camera, points),
                        fusion,
                    )
                } else {
                    let mut prior_depth = old.depth.clone();
                    let mut prior_weights = old.weights.clone();
                    let visibility = (self.visibility == Visibility::FreeSpaceDecay).then(|| {
                        visibility::decay(
                            self.camera,
                            &mut prior_depth,
                            &mut prior_weights,
                            depth,
                            &current[2].normals,
                            relative.inverse(),
                        )
                    });
                    let (mut map, mut weights, warp) = if replace {
                        let (d, w, stats) = if self.fusion_sampling == FusionSampling::VisibleMean {
                            fusion_sampling::raster(
                                self.camera,
                                &prior_depth,
                                &prior_weights,
                                relative.inverse(),
                            )
                        } else {
                            raster_with_tangents(
                                self.camera,
                                &prior_depth,
                                &prior_weights,
                                relative.inverse(),
                                (self.reprojection == Reprojection::TangentPlane)
                                    .then_some(old.pyramid[2].normals.as_slice()),
                            )
                        };
                        (d, w, Some(stats))
                    } else {
                        (prior_depth, prior_weights, None)
                    };
                    let transform = if replace { Pose::identity() } else { relative };
                    let mut fusion = if self.fusion_sampling == FusionSampling::VisibleMean {
                        let (new, present, stats) = fusion_sampling::raster(
                            self.camera,
                            depth,
                            &vec![1; depth.len()],
                            transform,
                        );
                        fuse_raster(&mut map, &mut weights, &new, &present, stats)
                    } else {
                        fuse_with_tangents(
                            self.camera,
                            &mut map,
                            &mut weights,
                            depth,
                            transform,
                            (self.reprojection == Reprojection::TangentPlane)
                                .then_some(current[2].normals.as_slice()),
                        )
                    };
                    fusion.warp = warp;
                    fusion.visibility = visibility;
                    let next_pyramid = pyramid_with_normals(self.camera, &map, self.normals);
                    (map, weights, next_pyramid, fusion)
                };
                let (next_gray, next_rgb_timestamp, next_phase) = if replace && fresh {
                    (gray, rgb_timestamp, phase)
                } else if replace {
                    // A repeated image does not establish a color anchor in a
                    // new depth reference view. Fresh depth remains usable.
                    (None, None, 0.)
                } else {
                    (old.gray.clone(), old.rgb_timestamp, old.phase)
                };
                self.model = Some(Model {
                    depth: map,
                    weights,
                    pyramid: next_pyramid,
                    key_root: if replace { root } else { old.key_root },
                    initial: if replace { Pose::identity() } else { relative },
                    gray: next_gray,
                    rgb_timestamp: next_rgb_timestamp,
                    phase: next_phase,
                    depth_timestamp: if replace {
                        timestamp
                    } else {
                        old.depth_timestamp
                    },
                    interval: if replace { None } else { old.interval },
                });
                out.first_from_current = Some(root);
                out.final_equation = Some(final_e);
                out.keyframe_replaced = replace;
                out.fusion = Some(fusion);
                out.final_photometric = final_photo;
            }
            if self.motion_model == MotionModel::ConstantVelocity {
                self.history
                    .observe(timestamp, out.first_from_current.expect("accepted pose"));
                if self.timing == Timing::PreviousMotion
                    && let Some(interval) = self.history.interval()
                    && let Some(model) = self.model.as_mut()
                    && (model.depth_timestamp == timestamp
                        || (model.interval.is_none() && model.depth_timestamp == interval.previous))
                {
                    model.interval = Some(interval);
                }
            }
            self.last_accepted = Some(timestamp);
            out.accepted = true;
            Ok(())
        })();
        out.rejection = result.err();
        out.lost = self.lost;
        out.last_accepted_timestamp = self.last_accepted;
        out
    }
}

// Scaled SPD solver and bounded Jacobi from this repository's reprojection implementation.
fn solve_normal(
    h: [[f64; 6]; 6],
    g: [f64; 6],
    max_condition: f64,
) -> Result<([f64; 6], f64), String> {
    let mut scale = [0.0; 6];
    let mut a = [[0.0; 6]; 6];
    for i in 0..6 {
        if !h[i][i].is_finite() || h[i][i] <= 0.0 || !g[i].is_finite() {
            return Err("ill-conditioned projective depth normal matrix".into());
        }
        scale[i] = h[i][i].sqrt();
    }
    for i in 0..6 {
        for j in 0..6 {
            a[i][j] = h[i][j] / (scale[i] * scale[j]);
            if !a[i][j].is_finite() {
                return Err("nonfinite projective depth normal matrix".into());
            }
        }
    }
    let eigen = eigenvalues(a);
    let smallest = eigen.iter().copied().fold(f64::INFINITY, f64::min);
    let largest = eigen.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let condition = largest / smallest;
    if smallest <= 0.0 || !condition.is_finite() || condition > max_condition {
        return Err("ill-conditioned projective depth normal matrix".into());
    }
    let mut l = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..=i {
            let mut value = a[i][j];
            for k in 0..j {
                value -= l[i][k] * l[j][k];
            }
            if i == j {
                if value <= 0.0 || !value.is_finite() {
                    return Err("singular projective depth normal solve".into());
                }
                l[i][j] = value.sqrt();
            } else {
                l[i][j] = value / l[j][j];
            }
        }
    }
    let mut y = [0.0; 6];
    for i in 0..6 {
        let mut value = -g[i] / scale[i];
        for (j, previous) in y.iter().enumerate().take(i) {
            value -= l[i][j] * previous;
        }
        y[i] = value / l[i][i];
    }
    let mut d = [0.0; 6];
    for i in (0..6).rev() {
        let mut value = y[i];
        for j in i + 1..6 {
            value -= l[j][i] * d[j];
        }
        d[i] = value / l[i][i];
    }
    for i in 0..6 {
        d[i] /= scale[i];
        if !d[i].is_finite() {
            return Err("nonfinite projective depth increment".into());
        }
    }
    Ok((d, condition))
}
/// Maximum-pivot symmetric Jacobi, bounded to 256 rotations on a 6x6 matrix.
fn eigenvalues(mut a: [[f64; 6]; 6]) -> [f64; 6] {
    for _ in 0..256 {
        let (mut p, mut q) = (0, 1);
        let mut maximum = a[p][q].abs();
        for (i, row) in a.iter().enumerate() {
            for (j, value) in row.iter().enumerate().skip(i + 1) {
                if value.abs() > maximum {
                    maximum = value.abs();
                    p = i;
                    q = j;
                }
            }
        }
        if maximum <= 1e-14 {
            break;
        }
        let angle = 0.5 * (2.0 * a[p][q]).atan2(a[q][q] - a[p][p]);
        let (s, c) = angle.sin_cos();
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];
        for k in 0..6 {
            if k != p && k != q {
                let akp = a[k][p];
                let akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[p][k] = a[k][p];
                a[k][q] = s * akp + c * akq;
                a[q][k] = a[k][q];
            }
        }
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
    }
    std::array::from_fn(|i| a[i][i])
}

#[cfg(test)]
mod tests {
    use super::*;
    fn camera() -> Camera {
        Camera {
            width: 128,
            height: 96,
            fx: 110.,
            fy: 110.,
            cx: 63.5,
            cy: 47.5,
        }
    }
    fn surface() -> Vec<f64> {
        (0..128 * 96)
            .map(|i| {
                1.5 + 0.12 * ((i % 128) as f64 / 9.).sin() + 0.08 * ((i / 128) as f64 / 11.).cos()
            })
            .collect()
    }
    #[test]
    fn duplicate_and_bad_depth_do_not_refresh_and_expiry_latches() {
        let mut tracker = Tracker::new(camera()).unwrap();
        let depth = surface();
        assert!(tracker.step(1., &depth).accepted);
        assert!(!tracker.step(1., &depth).accepted);
        let rejected = tracker.step(1.1, &[]);
        assert!(!rejected.accepted && !rejected.lost);
        assert_eq!(rejected.last_accepted_timestamp, Some(1.));
        let expired = tracker.step(1.201, &[]);
        assert!(expired.lost && !expired.accepted);
        assert!(!tracker.step(1.21, &depth).accepted);
        assert_eq!(tracker.last_accepted, Some(1.));
    }
    #[test]
    fn geometric_rejection_does_not_modify_model_or_accepted_pose() {
        for policy in [Reprojection::Point, Reprojection::TangentPlane] {
            let mut tracker = Tracker::with_reprojection(camera(), policy).unwrap();
            assert!(tracker.step(1., &surface()).accepted);
            let before = tracker.model.as_ref().unwrap().depth.clone();
            let weights = tracker.model.as_ref().unwrap().weights.clone();
            let rejected = tracker.step(1.03, &vec![0.; 128 * 96]);
            assert!(!rejected.accepted && rejected.first_from_current.is_none());
            assert_eq!(tracker.model.as_ref().unwrap().depth, before);
            assert_eq!(tracker.model.as_ref().unwrap().weights, weights);
            assert_eq!(tracker.last_accepted, Some(1.));
            assert!(tracker.step(1.06, &surface()).accepted);
        }
    }
    #[test]
    fn a_single_plane_is_rejected_as_unobservable() {
        let c = camera();
        let depth = vec![2.; c.width * c.height];
        let mut tracker = Tracker::new(c).unwrap();
        assert!(tracker.step(1., &depth).accepted);
        let result = tracker.step(1.03, &depth);
        assert!(!result.accepted);
        assert!(result.rejection.unwrap().contains("condition"));
    }
    #[test]
    fn malformed_initial_frame_does_not_establish_a_pose() {
        let mut tracker = Tracker::new(camera()).unwrap();
        for depth in [
            vec![0.; 128 * 96],
            vec![f64::NAN; 128 * 96],
            vec![-1.; 128 * 96],
            vec![1.; 20],
        ] {
            assert!(
                !tracker
                    .step(tracker.last_observed.unwrap_or(0.) + 0.01, &depth)
                    .accepted
            );
            assert!(tracker.model.is_none() && tracker.last_accepted.is_none());
        }
        assert!(tracker.step(1., &surface()).accepted);
    }
    #[test]
    fn raster_has_hand_calculated_nearest_depth_and_index_ties() {
        let c = Camera {
            width: 32,
            height: 32,
            fx: 32.,
            fy: 32.,
            cx: 15.5,
            cy: 15.5,
        };
        let mut depth = vec![0.; 1024];
        let mut weights = vec![0; 1024];
        depth[16 * 32 + 16] = 1.;
        weights[16 * 32 + 16] = 2;
        depth[16 * 32 + 17] = 1.;
        weights[16 * 32 + 17] = 7;
        let pose = Pose {
            translation: [0., 0., 1.],
            ..Pose::identity()
        };
        let (z, w, stats) = raster(c, &depth, &weights, pose);
        assert_eq!(
            (stats.source_valid, stats.projected, stats.selected),
            (2, 2, 1)
        );
        assert_eq!((z[16 * 32 + 16], w[16 * 32 + 16]), (2., 2));
        depth[17 * 32 + 16] = 0.5;
        weights[17 * 32 + 16] = 4;
        let (z, w, stats) = raster(c, &depth, &weights, pose);
        assert_eq!((stats.projected, stats.selected), (3, 1));
        assert_eq!((z[16 * 32 + 16], w[16 * 32 + 16]), (1.5, 4));
    }
    #[test]
    fn depth_fusion_weight_cap_keeps_updating_without_unbounded_weight() {
        let c = camera();
        let mut model = vec![2.; 128 * 96];
        let mut weights = vec![16; 128 * 96];
        let stats = fuse(
            c,
            &mut model,
            &mut weights,
            &vec![2.01; 128 * 96],
            Pose::identity(),
        );
        assert_eq!(stats.averaged, 128 * 96);
        assert!(weights.iter().all(|&w| w == 16));
        assert!((model[123] - (16. * 2. + 2.01) / 17.).abs() < 1e-15);
    }

    #[test]
    fn tangent_splats_land_on_the_transformed_plane_at_pixel_centers() {
        let c = camera();
        let normal = [-0.3, 0.2, 1.];
        let unit = mul(normal, 1. / norm(normal));
        let depth: Vec<_> = (0..c.width * c.height)
            .map(|i| 2. / dot(normal, c.point(i, 1.)))
            .collect();
        let normals = vec![unit; depth.len()];
        let weights = vec![1; depth.len()];
        let p = Pose::identity().increment([0.012, -0.008, 0.005, 0.007, 0.009, -0.003]);
        let (point, _, _) = raster(c, &depth, &weights, p);
        let (plane, _, stats) = raster_with_tangents(c, &depth, &weights, p, Some(&normals));
        let transformed = p.rotate(normal);
        let offset = 2. + dot(transformed, p.translation);
        let mut point_error: f64 = 0.;
        for (i, &z) in plane.iter().enumerate().filter(|(_, z)| **z > 0.) {
            let exact = offset / dot(transformed, c.point(i, 1.));
            assert!((z - exact).abs() < 2e-14);
            point_error = point_error.max((point[i] - exact).abs());
        }
        assert!(stats.tangent_adjusted > 10_000 && point_error > 0.001);
    }

    #[test]
    fn missing_or_grazing_normals_preserve_point_splats() {
        let c = camera();
        let mut depth = vec![0.; c.width * c.height];
        depth[48 * c.width + 64] = 2.;
        let weights = vec![1; depth.len()];
        let p = Pose::identity().increment([0.001, 0., 0., 0., 0., 0.]);
        let (expected, support, _) = raster(c, &depth, &weights, p);
        for normal in [[0.; 3], [1., 0., 0.]] {
            let normals = vec![normal; depth.len()];
            let (actual, w, stats) = raster_with_tangents(c, &depth, &weights, p, Some(&normals));
            assert_eq!(actual, expected);
            assert_eq!(w, support);
            assert_eq!(stats.tangent_adjusted, 0);
        }
    }

    #[test]
    fn reciprocal_matching_rejects_a_different_return_pixel() {
        let c = Camera {
            width: 32,
            height: 32,
            fx: 32.,
            fy: 32.,
            cx: 15.5,
            cy: 15.5,
        };
        let vertices: Vec<_> = (0..1024).map(|i| c.point(i, 2.)).collect();
        let mut source = Level {
            camera: c,
            vertices: vertices.clone(),
            normals: vec![[0.; 3]; 1024],
        };
        let i = 16 * 32 + 16;
        source.normals[i] = [0., 0., 1.];
        let mut target = Level {
            camera: c,
            vertices,
            normals: vec![[0., 0., 1.]; 1024],
        };
        let p = Pose::identity().increment([0.031875, 0., 0., 0., 0., 0.]);
        assert_eq!(
            equation(
                &target,
                &source,
                p,
                1,
                Correspondence::Reciprocal,
                Residual::TargetPlane
            )
            .count,
            1
        );
        target.vertices[i + 1] = c.point(i + 1, 2.06);
        assert_eq!(
            equation(
                &target,
                &source,
                p,
                1,
                Correspondence::Forward,
                Residual::TargetPlane
            )
            .count,
            1
        );
        assert_eq!(
            equation(
                &target,
                &source,
                p,
                1,
                Correspondence::Reciprocal,
                Residual::TargetPlane
            )
            .count,
            0
        );
    }
    #[test]
    fn rotating_source_plane_has_the_target_point_lever_arm() {
        let source = [0.13, -0.07, 1.8];
        let target = [0.15, -0.04, 1.83];
        let normal = mul([0.2, -0.1, 1.], 1. / norm([0.2, -0.1, 1.]));
        let p = Pose::identity().increment([0.01, -0.02, 0.005, 0.02, -0.01, 0.015]);
        let n = p.rotate(normal);
        let rotation = cross(target, n);
        let expected = [n[0], n[1], n[2], rotation[0], rotation[1], rotation[2]];
        for axis in 0..6 {
            let value = |sign: f64| {
                let mut d = [0.; 6];
                d[axis] = sign * 1e-7;
                let moved = p.increment(d);
                dot(moved.rotate(normal), sub(moved.transform(source), target))
            };
            assert!(((value(1.) - value(-1.)) / 2e-7 - expected[axis]).abs() < 3e-9);
        }
    }
    #[test]
    fn bilinear_targets_preserve_planes_and_reject_missing_corners() {
        let c = camera();
        let raw = vec![2.; c.width * c.height];
        let mut levels = pyramid(c, &raw);
        let layer = &mut levels[2];
        let query = [(64.25 - c.cx) * 2. / c.fx, (48.75 - c.cy) * 2. / c.fy, 2.];
        let (point, normal) = layer.bilinear(query).unwrap();
        assert!(norm(sub(point, query)) < 1e-14);
        assert_eq!(normal, [0., 0., 1.]);
        layer.vertices[49 * c.width + 65][2] = 0.;
        assert!(layer.bilinear(query).is_none());
        assert!(layer.bilinear([f64::NAN, 0., 2.]).is_none());
    }

    #[test]
    fn inverse_depth_fit_recovers_a_tilted_plane_in_camera_coordinates() {
        let c = camera();
        let expected = mul([0.2, -0.15, 1.], 1. / norm([0.2, -0.15, 1.]));
        let depth: Vec<_> = (0..c.width * c.height)
            .map(|i| 2. / dot(expected, c.point(i, 1.)))
            .collect();
        let normals = inverse_depth_normals(c, &depth);
        for y in 2..c.height - 2 {
            for x in 2..c.width - 2 {
                assert!(norm(sub(normals[y * c.width + x], expected)) < 1e-12);
            }
        }
    }

    #[test]
    fn plane_normals_reject_holes_discontinuities_and_incomplete_support() {
        let c = camera();
        let i = 48 * c.width + 64;
        for unsupported in [0., 0.3, 5., 2.1] {
            let mut depth = vec![2.; c.width * c.height];
            depth[i - 2] = unsupported;
            let normals = inverse_depth_normals(c, &depth);
            assert_eq!(normals[i], [0.; 3]);
            assert!(norm(sub(normals[i + 1], [0., 0., 1.])) < 1e-12);
            assert_eq!(normals[0], [0.; 3]);
            assert_eq!(normals[c.width + 1], [0.; 3]);
        }
    }

    #[test]
    fn plane_fit_reduces_a_known_single_sample_depth_perturbation() {
        let c = camera();
        let i = 48 * c.width + 64;
        let mut depth = vec![2.; c.width * c.height];
        depth[i + 1] += 0.002;
        let central = pyramid(c, &depth)[2].normals[i];
        let fitted = inverse_depth_normals(c, &depth)[i];
        assert!(norm(sub(fitted, [0., 0., 1.])) < norm(sub(central, [0., 0., 1.])) / 20.);
    }

    #[test]
    fn nonplanar_patches_and_coarse_levels_keep_original_normals() {
        let c = camera();
        let i = 48 * c.width + 64;
        let mut depth = vec![2.; c.width * c.height];
        depth[i] = 2.;
        depth[i + 1] = 2.02;
        depth[i - 1] = 1.98;
        depth[i + c.width] = 2.02;
        depth[i - c.width] = 1.98;
        assert_eq!(inverse_depth_normals(c, &depth)[i], [0.; 3]);
        let original = pyramid(c, &depth);
        let fitted = pyramid_with_normals(c, &depth, NormalEstimation::InverseDepthPlane);
        assert_eq!(original[2].normals[i], fitted[2].normals[i]);
        assert!(norm(fitted[2].normals[i]) > 0.5);
        for level in 0..2 {
            assert_eq!(original[level].normals, fitted[level].normals);
        }
    }
}
